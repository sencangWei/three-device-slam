"""Live binary transport for the pinned D435i ORB-SLAM3 engine.

This module does not own a camera.  The existing single RSUSB owner supplies
stereo pairs and combined IMU samples; a native child calls TrackStereo and
returns local camera poses over a dedicated pipe.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import struct
import subprocess
import threading
import time
from typing import Callable

import numpy as np

from three_device_slam.spatial.d435i_live_vins import CombinedImu


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RUNTIME = ROOT / "artifacts/runtime/orbslam3-4452a3c4-d435i-live-v5"
DEFAULT_SETTINGS = (
    ROOT
    / "artifacts/spatial_bench/d435i_orbslam3_dynamic_recheck_input_20260915_run1"
    / "orbslam3_d435i.yaml"
)
DEFAULT_SETTINGS_SHA256 = (
    "663ce323bcbc9954688709875e40872dfc3bba7dd2d2cfd370ba36130812b87b"
)
INPUT_MAGIC = b"ORBI"
POSE_MAGIC = b"ORBP"
FRAME_PACKET = 1
SHUTDOWN_PACKET = 2
TRACKING_OK = 2
TRACKING_OK_KLT = 5
_INPUT_HEADER = struct.Struct("<4sB3xqIIIII")
_IMU_WIRE = struct.Struct("<q6d")
_POSE_WIRE = struct.Struct("<4sqiI9d")
RUNTIME_SCHEMA = "three-device-slam.orbslam3-live-runtime.v2"
RUNTIME_PROTOCOL = "ORBI/ORBP packed little-endian v2"
EXPECTED_ORB_COMMIT = "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4"
REQUIRED_RUNTIME_FILES = (
    "bin/orbslam3_live_adapter",
    "lib/libORB_SLAM3.so",
    "share/ORBvoc.txt",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _settings_without_atlas_save(settings: Path) -> str:
    """Disable ORB Atlas serialization for the ephemeral live process.

    ORB-SLAM3's Atlas::PreSave mutates/iterates map associations and is not
    needed by this adapter, which exports trajectories separately.  Keeping
    it enabled makes shutdown nondeterministically crash in MapPoint::PreSave.
    """
    result = []
    replaced = False
    for line in settings.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith("System.SaveAtlasToFile:"):
            indentation = line[: len(line) - len(line.lstrip())]
            result.append(f'{indentation}System.SaveAtlasToFile: ""')
            replaced = True
        else:
            result.append(line)
    if not replaced:
        result.append('System.SaveAtlasToFile: ""')
    return "\n".join(result) + "\n"


def validate_runtime(runtime: Path) -> tuple[dict, Path, Path]:
    runtime = runtime.resolve()
    manifest_path = runtime / "runtime_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != RUNTIME_SCHEMA:
        raise ValueError("runtime is not a pinned ORB-SLAM3 live adapter")
    if manifest.get("protocol") != RUNTIME_PROTOCOL:
        raise ValueError("runtime ORB live protocol version mismatch")
    if manifest.get("pose_wire_bytes") != _POSE_WIRE.size:
        raise ValueError("runtime ORB pose packet size mismatch")
    if manifest.get("orbslam3_commit") != EXPECTED_ORB_COMMIT:
        raise ValueError("runtime ORB-SLAM3 commit mismatch")
    provenance = (
        ("patch", "patch_sha256"),
        ("adapter_source", "adapter_source_sha256"),
    )
    for path_field, hash_field in provenance:
        relative = manifest.get(path_field)
        expected = manifest.get(hash_field)
        if not isinstance(relative, str) or not isinstance(expected, str):
            raise ValueError(f"runtime provenance is incomplete: {path_field}")
        source = ROOT / relative
        if not source.is_file() or sha256(source) != expected:
            raise ValueError(f"runtime provenance hash mismatch: {relative}")
    files = manifest.get("files")
    if not isinstance(files, dict) or any(
        relative not in files for relative in REQUIRED_RUNTIME_FILES
    ):
        raise ValueError("runtime manifest omits a required hashed file")
    for relative, expected in files.items():
        path = runtime / relative
        if not path.is_file() or sha256(path) != expected:
            raise ValueError(f"runtime hash mismatch: {relative}")
    binary = runtime / "bin/orbslam3_live_adapter"
    vocabulary = runtime / "share/ORBvoc.txt"
    if not binary.is_file() or not vocabulary.is_file():
        raise FileNotFoundError("runtime binary or vocabulary is missing")
    return manifest, binary, vocabulary


@dataclass(frozen=True)
class OrbSlam3Pose:
    timestamp_ns: int
    tracking_state: int
    imu_samples: int
    local_from_camera: np.ndarray
    processing_ms: float
    inertial_ba2_ready: bool
    arrival_monotonic_ns: int

    @property
    def tracking_ok(self) -> bool:
        return self.tracking_state in (TRACKING_OK, TRACKING_OK_KLT)


@dataclass(frozen=True)
class _StereoItem:
    timestamp_ns: int
    left: bytes
    right: bytes
    enqueued_monotonic_ns: int


class OrbSlam3LiveProcess:
    """Feed ordered stereo/IMU packets to a hash-pinned native ORB child."""

    def __init__(
        self,
        output: Path,
        *,
        runtime: Path = DEFAULT_RUNTIME,
        settings: Path = DEFAULT_SETTINGS,
        image_shape: tuple[int, int] = (720, 1280),
        imu_lead_ns: int = 5_000_000,
        queue_depth: int = 64,
        max_write_backlog: int | None = None,
        backpressure_after_frames: int = 0,
        min_imu_samples_per_frame: int = 2,
        ready_timeout_s: float = 90.0,
        pose_callback: Callable[[OrbSlam3Pose], None] | None = None,
    ):
        if (
            queue_depth < 1
            or imu_lead_ns < 0
            or ready_timeout_s <= 0
            or (max_write_backlog is not None and max_write_backlog < 1)
            or backpressure_after_frames < 0
            or min_imu_samples_per_frame < 2
        ):
            raise ValueError("invalid ORB live process bounds")
        self.output = output.resolve()
        self.runtime = runtime.resolve()
        self.settings = settings.resolve()
        self.image_shape = tuple(image_shape)
        self.imu_lead_ns = int(imu_lead_ns)
        self.ready_timeout_s = float(ready_timeout_s)
        self.max_write_backlog = max_write_backlog
        self.backpressure_after_frames = int(backpressure_after_frames)
        self.min_imu_samples_per_frame = int(min_imu_samples_per_frame)
        self.pose_callback = pose_callback
        self._write_queue: queue.Queue = queue.Queue(maxsize=queue_depth)
        self._pending_stereo: deque[_StereoItem] = deque()
        self._imu: deque[CombinedImu] = deque()
        self._poses: queue.Queue[OrbSlam3Pose] = queue.Queue()
        self._lock = threading.RLock()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._failure: str | None = None
        self._child: subprocess.Popen | None = None
        self._writer: threading.Thread | None = None
        self._reader: threading.Thread | None = None
        self._log_file = None
        self._pose_read_fd: int | None = None
        self._last_imu_ns: int | None = None
        self._last_stereo_ns: int | None = None
        self._last_sent_stereo_ns: int | None = None
        self._frame_enqueue_times: dict[int, int] = {}
        self.stereo_received = 0
        self.stereo_dropped_backpressure = 0
        self.stereo_dropped_insufficient_imu = 0
        self.stereo_sent = 0
        self.imu_received = 0
        self.imu_sent = 0
        self.poses_received = 0
        self.tracking_ok_poses = 0
        self.inertial_ba2_ready_poses = 0
        self.max_write_queue = 0
        self.max_pending_stereo = 0
        self._maximum_pending_stereo = min(queue_depth, 8)
        self.end_to_end_ms: deque[float] = deque(maxlen=6000)
        self.processing_ms: deque[float] = deque(maxlen=6000)

    def start(self) -> None:
        if self._child is not None or self.output.exists():
            raise RuntimeError("ORB live process cannot reuse an output directory")
        _manifest, binary, vocabulary = validate_runtime(self.runtime)
        if not self.settings.is_file():
            raise FileNotFoundError(self.settings)
        if self.settings == DEFAULT_SETTINGS.resolve() and sha256(self.settings) != DEFAULT_SETTINGS_SHA256:
            raise RuntimeError("pinned D435i ORB settings hash mismatch")
        self.output.mkdir(parents=True)
        live_settings = self.output / "orbslam3_live_settings.yaml"
        live_settings.write_text(
            _settings_without_atlas_save(self.settings), encoding="utf-8"
        )
        pose_read_fd, pose_write_fd = os.pipe()
        self._pose_read_fd = pose_read_fd
        self._log_file = (self.output / "orbslam3.log").open("wb")
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = str(self.runtime / "lib") + (
            os.pathsep + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else ""
        )
        command = [
            str(binary),
            str(vocabulary),
            str(live_settings),
            str(self.output),
            str(pose_write_fd),
        ]
        self._child = subprocess.Popen(
            command,
            cwd=self.output,
            env=env,
            stdin=subprocess.PIPE,
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            pass_fds=(pose_write_fd,),
            start_new_session=True,
        )
        os.close(pose_write_fd)
        self._reader = threading.Thread(
            target=self._reader_loop, name="orbslam3-pose-reader", daemon=True
        )
        self._writer = threading.Thread(
            target=self._writer_loop, name="orbslam3-input-writer", daemon=True
        )
        self._reader.start()
        self._writer.start()
        if not self._ready.wait(timeout=self.ready_timeout_s):
            self.stop(force=True)
            raise RuntimeError(self._failure or "ORB-SLAM3 startup timed out")
        self.check_alive()

    def push_imu(self, sample: CombinedImu) -> None:
        with self._lock:
            self._raise_if_failed()
            timestamp = int(sample.timestamp_ns)
            if self._last_imu_ns is not None and timestamp <= self._last_imu_ns:
                raise ValueError("ORB live IMU timestamps must increase strictly")
            self._last_imu_ns = timestamp
            self._imu.append(sample)
            self.imu_received += 1
            self._drain_ready_frames()

    def push_stereo(self, timestamp_ns: int, left, right) -> None:
        timestamp = int(timestamp_ns)
        left_array = np.ascontiguousarray(left, dtype=np.uint8)
        right_array = np.ascontiguousarray(right, dtype=np.uint8)
        if left_array.shape != self.image_shape or right_array.shape != self.image_shape:
            raise ValueError("D435i ORB live stereo image shape mismatch")
        with self._lock:
            self._raise_if_failed()
            if self._last_stereo_ns is not None and timestamp <= self._last_stereo_ns:
                raise ValueError("ORB live stereo timestamps must increase strictly")
            self._last_stereo_ns = timestamp
            self.stereo_received += 1
            if (
                self.max_write_backlog is not None
                and self.stereo_received > self.backpressure_after_frames
                and self._write_queue.qsize() >= self.max_write_backlog
            ):
                # Drop before consuming IMU.  The next accepted image then
                # receives the complete integration interval since the last
                # image actually sent to ORB-SLAM3.
                self.stereo_dropped_backpressure += 1
                return False
            self._pending_stereo.append(
                _StereoItem(
                    timestamp,
                    left_array.tobytes(),
                    right_array.tobytes(),
                    time.monotonic_ns(),
                )
            )
            self.max_pending_stereo = max(
                self.max_pending_stereo, len(self._pending_stereo)
            )
            if len(self._pending_stereo) > self._maximum_pending_stereo:
                self._set_failure("orb_stereo_waiting_for_imu_overflow")
                raise RuntimeError("orb_stereo_waiting_for_imu_overflow")
            self._drain_ready_frames()
            return True

    publish_imu = push_imu
    publish_stereo = push_stereo

    def finish_input(self) -> None:
        """Release a final frame once its own timestamp is covered by IMU."""
        with self._lock:
            original_lead = self.imu_lead_ns
            self.imu_lead_ns = 0
            try:
                self._drain_ready_frames()
            finally:
                self.imu_lead_ns = original_lead

    def _drain_ready_frames(self) -> None:
        while (
            self._pending_stereo
            and self._last_imu_ns is not None
            and self._last_imu_ns
            >= self._pending_stereo[0].timestamp_ns + self.imu_lead_ns
        ):
            frame = self._pending_stereo.popleft()
            lower = self._last_sent_stereo_ns
            measurements = []
            if lower is None:
                # Match the official EuRoC runner: the first image carries no
                # IMU, while the last sample at/before it is retained as the
                # integration boundary for the second image.
                while (
                    len(self._imu) >= 2
                    and self._imu[1].timestamp_ns <= frame.timestamp_ns
                ):
                    self._imu.popleft()
            else:
                eligible = sum(
                    sample.timestamp_ns <= frame.timestamp_ns for sample in self._imu
                )
                if eligible < self.min_imu_samples_per_frame:
                    # Do not consume the integration interval.  The next image
                    # receives all accumulated IMU samples, preventing the
                    # upstream ORB one-sample preintegration null path.
                    self.stereo_dropped_insufficient_imu += 1
                    continue
                while self._imu and self._imu[0].timestamp_ns <= frame.timestamp_ns:
                    measurements.append(self._imu.popleft())
            self._enqueue_frame(frame, measurements)
            self._last_sent_stereo_ns = frame.timestamp_ns

    def _enqueue_frame(self, frame: _StereoItem, measurements) -> None:
        header = _INPUT_HEADER.pack(
            INPUT_MAGIC,
            FRAME_PACKET,
            frame.timestamp_ns,
            self.image_shape[1],
            self.image_shape[0],
            len(measurements),
            len(frame.left),
            len(frame.right),
        )
        imu_payload = b"".join(
            _IMU_WIRE.pack(
                int(sample.timestamp_ns),
                *map(float, sample.accel_m_s2),
                *map(float, sample.gyro_rad_s),
            )
            for sample in measurements
        )
        try:
            self._write_queue.put_nowait((header, imu_payload, frame.left, frame.right))
        except queue.Full as exc:
            self._set_failure("orb_input_queue_overflow")
            raise RuntimeError("orb_input_queue_overflow") from exc
        self._frame_enqueue_times[frame.timestamp_ns] = frame.enqueued_monotonic_ns
        self.stereo_sent += 1
        self.imu_sent += len(measurements)
        self.max_write_queue = max(self.max_write_queue, self._write_queue.qsize())

    def poll_pose(self) -> OrbSlam3Pose | None:
        self._raise_if_failed()
        try:
            return self._poses.get_nowait()
        except queue.Empty:
            return None

    def check_alive(self) -> None:
        self._raise_if_failed()
        if self._child is None or self._child.poll() is not None:
            code = None if self._child is None else self._child.returncode
            raise RuntimeError(f"ORB-SLAM3 live process exited: returncode={code}")

    def _writer_loop(self) -> None:
        try:
            while True:
                packet = self._write_queue.get()
                try:
                    if packet is None:
                        return
                    if self._child is None or self._child.stdin is None:
                        raise RuntimeError("ORB-SLAM3 stdin is unavailable")
                    for part in packet:
                        self._child.stdin.write(part)
                    self._child.stdin.flush()
                finally:
                    self._write_queue.task_done()
        except (BrokenPipeError, OSError, RuntimeError) as exc:
            if not self._stop.is_set():
                self._set_failure(f"orb_writer_failed:{type(exc).__name__}")

    def _reader_loop(self) -> None:
        assert self._pose_read_fd is not None
        try:
            with os.fdopen(self._pose_read_fd, "rb", buffering=0) as stream:
                self._pose_read_fd = None
                while True:
                    payload = self._read_exact(stream, _POSE_WIRE.size)
                    if payload is None:
                        if not self._stop.is_set():
                            self._set_failure("orb_pose_pipe_closed")
                            self._ready.set()
                        return
                    values = _POSE_WIRE.unpack(payload)
                    if values[0] != POSE_MAGIC:
                        raise RuntimeError("invalid ORB pose packet magic")
                    timestamp_ns, state, imu_count = values[1:4]
                    pose_values = values[4:]
                    if timestamp_ns == -1:
                        self._ready.set()
                        continue
                    transform = np.eye(4, dtype=np.float64)
                    transform[:3, :3] = self._quaternion_matrix(*pose_values[3:7])
                    transform[:3, 3] = pose_values[:3]
                    now_ns = time.monotonic_ns()
                    pose = OrbSlam3Pose(
                        timestamp_ns,
                        state,
                        imu_count,
                        transform,
                        pose_values[7],
                        bool(pose_values[8]),
                        now_ns,
                    )
                    enqueued = self._frame_enqueue_times.pop(timestamp_ns, None)
                    if enqueued is not None:
                        self.end_to_end_ms.append((now_ns - enqueued) / 1_000_000)
                    self.processing_ms.append(pose.processing_ms)
                    self.poses_received += 1
                    if pose.tracking_ok:
                        self.tracking_ok_poses += 1
                    if pose.inertial_ba2_ready:
                        self.inertial_ba2_ready_poses += 1
                    self._poses.put(pose)
                    if self.pose_callback is not None:
                        self.pose_callback(pose)
        except (OSError, RuntimeError, ValueError) as exc:
            if not self._stop.is_set():
                self._set_failure(f"orb_reader_failed:{type(exc).__name__}")
                self._ready.set()

    @staticmethod
    def _read_exact(stream, size: int) -> bytes | None:
        parts = []
        remaining = size
        while remaining:
            part = stream.read(remaining)
            if not part:
                return None if not parts else (_ for _ in ()).throw(
                    RuntimeError("truncated ORB pose packet")
                )
            parts.append(part)
            remaining -= len(part)
        return b"".join(parts)

    @staticmethod
    def _quaternion_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        quaternion = np.array((qw, qx, qy, qz), dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("invalid ORB pose quaternion")
        w, x, y, z = quaternion / norm
        return np.array(
            (
                (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
            ),
            dtype=np.float64,
        )

    @staticmethod
    def _percentile(values, fraction: float) -> float | None:
        if not values:
            return None
        return float(np.percentile(np.asarray(values), fraction * 100))

    def stats(self) -> dict:
        if self._failure:
            status = "FAILED"
        elif self._child is not None and self._child.poll() is not None:
            status = "STOPPED"
        else:
            status = "RUNNING"
        return {
            "status": status,
            "failure": self._failure,
            "pid": None if self._child is None else self._child.pid,
            "returncode": None if self._child is None else self._child.returncode,
            "stereo_received": self.stereo_received,
            "stereo_dropped_backpressure": self.stereo_dropped_backpressure,
            "stereo_dropped_insufficient_imu": self.stereo_dropped_insufficient_imu,
            "stereo_sent": self.stereo_sent,
            "imu_received": self.imu_received,
            "imu_sent": self.imu_sent,
            "poses_received": self.poses_received,
            "tracking_ok_poses": self.tracking_ok_poses,
            "inertial_ba2_ready_poses": self.inertial_ba2_ready_poses,
            "pending_stereo": len(self._pending_stereo),
            "pending_imu": len(self._imu),
            "write_queue": self._write_queue.qsize(),
            "max_write_queue": self.max_write_queue,
            "max_pending_stereo": self.max_pending_stereo,
            "processing_ms_p50": self._percentile(self.processing_ms, 0.50),
            "processing_ms_p95": self._percentile(self.processing_ms, 0.95),
            "processing_ms_max": max(self.processing_ms, default=None),
            "end_to_end_ms_p50": self._percentile(self.end_to_end_ms, 0.50),
            "end_to_end_ms_p95": self._percentile(self.end_to_end_ms, 0.95),
            "end_to_end_ms_max": max(self.end_to_end_ms, default=None),
        }

    def _set_failure(self, reason: str) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = reason

    def _raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError(self._failure)

    def stop(self, *, force: bool = False) -> None:
        if self._child is None:
            return
        self._stop.set()
        if not force and self._child.poll() is None:
            try:
                header = _INPUT_HEADER.pack(
                    INPUT_MAGIC, SHUTDOWN_PACKET, 0, 0, 0, 0, 0, 0
                )
                self._write_queue.put((header,), timeout=2.0)
                self._write_queue.join()
            except (queue.Full, BrokenPipeError, OSError):
                force = True
        try:
            self._write_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._child.stdin is not None:
            try:
                self._child.stdin.close()
            except BrokenPipeError:
                pass
        if self._writer is not None:
            self._writer.join(timeout=2.0)
        if self._child.poll() is None:
            if force:
                os.killpg(self._child.pid, signal.SIGTERM)
            try:
                self._child.wait(timeout=120.0 if not force else 5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self._child.pid, signal.SIGKILL)
                self._child.wait(timeout=5.0)
        if self._reader is not None:
            self._reader.join(timeout=2.0)
        if self._pose_read_fd is not None:
            os.close(self._pose_read_fd)
            self._pose_read_fd = None
        if self._log_file is not None:
            self._log_file.close()
