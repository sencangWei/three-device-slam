from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import multiprocessing
import os
import queue
import re
import threading
import time
from pathlib import Path

from three_device_slam.core import AppendOnlySessionWriter, SensorRecord

from .capture import (
    TwoUQ2Packet,
    _CtypesXuBridge,
    _load_gst,
    build_pipeline_description,
)


_SAFE_SESSION_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
_PREVIEW_WINDOW = "2UQ2 Ego calibration: left half | right half"
_PREVIEW_SLOT_CAPACITY = 4 * 1024 * 1024
_IMU_QUEUE_CAPACITY = 256
_IMU_BATCH_SIZE = 32
_VIDEO_PERIOD_NS = 1_000_000_000 / 60.0
_IMU_PERIOD_NS = 5_000_000
_CAPTURE_STAGES = ("strict", "stereo", "cam-imu-provisional")


def estimated_missing_samples(interval_ns: int, expected_period_ns: float) -> int:
    if interval_ns <= 0 or expected_period_ns <= 0:
        return 0
    return max(0, int(round(interval_ns / expected_period_ns)) - 1)


def period_multiple_jitter_ns(interval_ns: int, expected_period_ns: float) -> int:
    """Return phase jitter without counting an inferred drop twice."""
    if interval_ns <= 0 or expected_period_ns <= 0:
        return 0
    elapsed_periods = max(1, int(round(interval_ns / expected_period_ns)))
    return int(abs(interval_ns - elapsed_periods * expected_period_ns))


class LatestFrameSlot:
    """One-item, non-blocking handoff used only by the lossy preview path."""

    def __init__(self):
        self._queue = queue.Queue(maxsize=1)

    def put(self, item) -> int:
        try:
            self._queue.put_nowait(item)
            return 0
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._queue.put_nowait(item)
            return 1

    def get(self, *, timeout: float):
        return self._queue.get(timeout=timeout)


def split_side_by_side(image):
    if getattr(image, "ndim", 0) not in (2, 3):
        raise ValueError("side-by-side image must have two or three dimensions")
    width = int(image.shape[1])
    if width <= 0 or width % 2:
        raise ValueError("side-by-side image width must be positive and even")
    midpoint = width // 2
    return image[:, :midpoint], image[:, midpoint:]


def expected_aprilgrid_detections(detections):
    import numpy as np

    selected = []
    seen_ids = set()
    for detection in detections:
        try:
            tag_id = int(detection.tag_id)
            corners = np.asarray(detection.corners, dtype=np.float64)
        except (AttributeError, TypeError, ValueError):
            continue
        if (
            tag_id < 0
            or tag_id >= 36
            or tag_id in seen_ids
            or corners.size != 8
            or not np.all(np.isfinite(corners))
        ):
            continue
        selected.append(detection)
        seen_ids.add(tag_id)
    return selected


def aprilgrid_geometry_is_consistent(
    detections,
    *,
    columns: int = 6,
    spacing_ratio: float = 0.3,
    max_reprojection_rms_px: float = 8.0,
) -> bool:
    import cv2
    import numpy as np

    selected = expected_aprilgrid_detections(detections)
    if len(selected) < 4:
        return False
    pitch = 1.0 + spacing_ratio
    object_corners = []
    image_corners = []
    for detection in selected:
        row, column = divmod(int(detection.tag_id), columns)
        x0, y0 = column * pitch, row * pitch
        object_corners.extend(
            ([x0, y0], [x0 + 1.0, y0], [x0 + 1.0, y0 + 1.0], [x0, y0 + 1.0])
        )
        image_corners.extend(
            np.asarray(detection.corners, dtype=np.float64).reshape(4, 2)
        )
    object_array = np.asarray(object_corners, dtype=np.float64)
    image_array = np.asarray(image_corners, dtype=np.float64)
    homography, _ = cv2.findHomography(object_array, image_array, 0)
    if homography is None or not np.all(np.isfinite(homography)):
        return False
    projected = cv2.perspectiveTransform(
        object_array.reshape(-1, 1, 2), homography
    ).reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected - image_array) ** 2, axis=1))))
    return bool(np.isfinite(rms) and rms <= max_reprojection_rms_px)


def build_preview_report(stats: dict) -> dict:
    processed = int(stats.get("processed_frames", 0))
    left_good = int(stats.get("left_frames_with_four_tags", 0))
    right_good = int(stats.get("right_frames_with_four_tags", 0))
    both_good = int(stats.get("both_frames_with_four_tags", 0))
    left_rate = left_good / processed if processed else 0.0
    right_rate = right_good / processed if processed else 0.0
    both_rate = both_good / processed if processed else 0.0
    left_average = (
        float(stats.get("left_tag_total", 0)) / processed if processed else 0.0
    )
    right_average = (
        float(stats.get("right_tag_total", 0)) / processed if processed else 0.0
    )
    checks = {
        "preview_frames": {
            "status": "PASS" if processed >= 10 else "FAIL",
            "threshold": ">= 10",
            "measurement": processed,
        },
        "left_grid_detection_rate": {
            "status": "PASS" if left_rate >= 0.6 else "FAIL",
            "threshold": ">= 0.6 with at least 4 tags",
            "measurement": left_rate,
        },
        "right_grid_detection_rate": {
            "status": "PASS" if right_rate >= 0.6 else "FAIL",
            "threshold": ">= 0.6 with at least 4 tags",
            "measurement": right_rate,
        },
        "simultaneous_grid_detection_rate": {
            "status": "PASS" if both_rate >= 0.6 else "FAIL",
            "threshold": ">= 0.6 with at least 4 tags in each half",
            "measurement": both_rate,
        },
        "left_average_tags": {
            "status": "PASS" if left_average >= 6.0 else "FAIL",
            "threshold": ">= 6.0",
            "measurement": left_average,
        },
        "right_average_tags": {
            "status": "PASS" if right_average >= 6.0 else "FAIL",
            "threshold": ">= 6.0",
            "measurement": right_average,
        },
        "processing_errors": {
            "status": "PASS" if int(stats.get("processing_errors", 0)) == 0 else "FAIL",
            "threshold": "== 0",
            "measurement": int(stats.get("processing_errors", 0)),
        },
        "operator_aborted": {
            "status": "FAIL" if bool(stats.get("operator_aborted")) else "PASS",
            "threshold": "== false",
            "measurement": bool(stats.get("operator_aborted")),
        },
    }
    return {
        "status": (
            "PASS"
            if all(check["status"] == "PASS" for check in checks.values())
            else "FAIL"
        ),
        "checks": checks,
        "stats": stats,
        "note": (
            "This gate checks that both image halves repeatedly see the AprilGrid. "
            "Final calibration quality still requires Kalibr residuals and held-out validation."
        ),
    }


def capture_was_aborted(loop_aborted: bool, preview_stats: dict | None) -> bool:
    return bool(loop_aborted or (preview_stats and preview_stats.get("operator_aborted")))


def _put_latest_process_message(target, message) -> int:
    try:
        target.put_nowait(message)
        return 0
    except queue.Full:
        try:
            target.get_nowait()
        except queue.Empty:
            return 1
        try:
            target.put_nowait(message)
        except queue.Full:
            pass
        return 1


def _configure_preview_worker_environment() -> None:
    # The OpenBLAS linked by the preview's NumPy/OpenCV wheels otherwise creates
    # one worker per logical CPU. Those workers can starve the 100 Hz USB XU
    # polling loop even though the preview itself runs in a separate process.
    os.environ["OPENBLAS_NUM_THREADS"] = "1"


def _preview_process_main(
    shared_memory_name,
    slot_capacity,
    shared_metadata,
    new_frame_event,
    result_queue,
    stop_event,
    abort_event,
) -> None:
    _configure_preview_worker_environment()
    stats = {
        "processed_frames": 0,
        "left_frames_with_four_tags": 0,
        "right_frames_with_four_tags": 0,
        "both_frames_with_four_tags": 0,
        "left_tag_total": 0,
        "right_tag_total": 0,
        "processing_errors": 0,
        "operator_aborted": False,
    }
    cv2 = None
    shared_memory_handle = None
    try:
        import cv2
        from multiprocessing import shared_memory
        import numpy as np
        from aprilgrid import Detector

        shared_memory_handle = shared_memory.SharedMemory(
            name=shared_memory_name, create=False
        )
        cv2.setNumThreads(1)
        detector = Detector("t36h11")
        cv2.namedWindow(_PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_PREVIEW_WINDOW, 1600, 520)
        _put_latest_process_message(result_queue, {"kind": "ready"})
        rendered = None
        while not stop_event.is_set():
            item = None
            if new_frame_event.wait(timeout=0.02):
                with shared_metadata.get_lock():
                    slot = int(shared_metadata[0])
                    length = int(shared_metadata[1])
                    sequence = int(shared_metadata[2])
                    offset = slot * slot_capacity
                    jpeg = bytes(
                        shared_memory_handle.buf[offset : offset + length]
                    )
                    new_frame_event.clear()
                item = (sequence, jpeg)
            if item is not None:
                sequence, jpeg = item
                encoded = np.frombuffer(jpeg, dtype=np.uint8)
                image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if image is None or image.shape[:2] != (1080, 3840):
                    raise ValueError("preview_jpeg_is_not_3840x1080")
                left, right = split_side_by_side(image)
                left = cv2.resize(left, (768, 432), interpolation=cv2.INTER_AREA)
                right = cv2.resize(right, (768, 432), interpolation=cv2.INTER_AREA)

                summaries = []
                for half in (left, right):
                    gray = cv2.cvtColor(half, cv2.COLOR_BGR2GRAY)
                    detections = expected_aprilgrid_detections(detector.detect(gray))
                    grid_valid = aprilgrid_geometry_is_consistent(detections)
                    color = (0, 255, 0) if grid_valid else (0, 165, 255)
                    for detection in detections:
                        corners = np.asarray(detection.corners, dtype=np.int32)
                        if corners.size != 8:
                            continue
                        polygon = corners.reshape(4, 2)
                        cv2.polylines(half, [polygon], True, color, 2)
                        center = tuple(np.rint(polygon.mean(axis=0)).astype(int))
                        cv2.putText(
                            half,
                            str(int(detection.tag_id)),
                            center,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            color,
                            1,
                            cv2.LINE_AA,
                        )
                    summaries.append((len(detections), grid_valid))

                left_count, left_grid_valid = summaries[0]
                right_count, right_grid_valid = summaries[1]
                left_good = left_count >= 4 and left_grid_valid
                right_good = right_count >= 4 and right_grid_valid
                both_good = left_good and right_good
                rendered = cv2.hconcat((left, right))
                rendered = cv2.copyMakeBorder(
                    rendered,
                    72,
                    0,
                    0,
                    0,
                    cv2.BORDER_CONSTANT,
                    value=(0, 0, 0),
                )
                color = (0, 220, 0) if both_good else (0, 165, 255)
                status = (
                    "BOTH CAMERAS SEE APRILGRID"
                    if both_good
                    else "KEEP APRILGRID VISIBLE IN BOTH HALVES"
                )
                cv2.putText(
                    rendered,
                    f"LEFT HALF tags={left_count}/36",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.72,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    rendered,
                    f"RIGHT HALF tags={right_count}/36",
                    (790, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.72,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    rendered,
                    f"{status} | raw frame={sequence} | q/ESC aborts this run",
                    (20, 62),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.68,
                    color,
                    2,
                    cv2.LINE_AA,
                )
                stats["processed_frames"] += 1
                stats["left_frames_with_four_tags"] += int(left_good)
                stats["right_frames_with_four_tags"] += int(right_good)
                stats["both_frames_with_four_tags"] += int(both_good)
                stats["left_tag_total"] += left_count
                stats["right_tag_total"] += right_count
                _put_latest_process_message(
                    result_queue, {"kind": "stats", "stats": dict(stats)}
                )

            if rendered is not None:
                cv2.imshow(_PREVIEW_WINDOW, rendered)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                stats["operator_aborted"] = True
                abort_event.set()
                _put_latest_process_message(
                    result_queue, {"kind": "stats", "stats": dict(stats)}
                )
                return
    except BaseException as exc:
        stats["processing_errors"] += 1
        _put_latest_process_message(
            result_queue,
            {"kind": "error", "error": f"{type(exc).__name__}: {exc}", "stats": stats},
        )
        abort_event.set()
    finally:
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
        if shared_memory_handle is not None:
            shared_memory_handle.close()


class StereoAprilGridPreview:
    """Latest-only AprilGrid preview that never back-pressures raw recording."""

    def __init__(self, *, preview_hz: float = 5.0):
        if preview_hz <= 0:
            raise ValueError("preview_hz must be positive")
        self._period_ns = int(1_000_000_000 / preview_hz)
        self._context = multiprocessing.get_context("spawn")
        self._result_queue = self._context.Queue(maxsize=64)
        self._stop = self._context.Event()
        self._abort = self._context.Event()
        self._new_frame = self._context.Event()
        self._shared_metadata = self._context.Array("Q", 4, lock=True)
        self._shared_memory = None
        self._process = None
        self._process_started = False
        self._lock = threading.Lock()
        self._last_submitted_ns = None
        self._error = None
        self._stats = {
            "submitted_frames": 0,
            "queue_replacements": 0,
            "processed_frames": 0,
            "left_frames_with_four_tags": 0,
            "right_frames_with_four_tags": 0,
            "both_frames_with_four_tags": 0,
            "left_tag_total": 0,
            "right_tag_total": 0,
            "processing_errors": 0,
            "operator_aborted": False,
        }

    def start(self) -> None:
        if not os.environ.get("DISPLAY"):
            raise RuntimeError("preview_display_unavailable: DISPLAY is not set")
        from multiprocessing import shared_memory

        self._shared_memory = shared_memory.SharedMemory(
            create=True, size=2 * _PREVIEW_SLOT_CAPACITY
        )
        self._process = self._context.Process(
            target=_preview_process_main,
            args=(
                self._shared_memory.name,
                _PREVIEW_SLOT_CAPACITY,
                self._shared_metadata,
                self._new_frame,
                self._result_queue,
                self._stop,
                self._abort,
            ),
            name="2uq2-aprilgrid-preview-process",
            daemon=True,
        )
        try:
            self._process.start()
        except BaseException:
            self._process_started = False
            raise
        self._process_started = True
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            try:
                message = self._result_queue.get(timeout=0.1)
            except queue.Empty:
                if not self._process.is_alive():
                    raise RuntimeError("preview_process_exited_before_ready")
                continue
            self._handle_message(message)
            self.raise_if_failed(_drain=False)
            if message.get("kind") == "ready":
                return
        raise RuntimeError("preview_process_ready_timeout")

    def submit(self, *, sequence: int, acquisition_ns: int, jpeg: bytes) -> None:
        with self._lock:
            if (
                self._last_submitted_ns is not None
                and acquisition_ns - self._last_submitted_ns < self._period_ns
            ):
                return
            self._last_submitted_ns = acquisition_ns
            self._stats["submitted_frames"] += 1
        if len(jpeg) > _PREVIEW_SLOT_CAPACITY:
            raise RuntimeError("preview_jpeg_exceeds_shared_slot")
        lock = self._shared_metadata.get_lock()
        if not lock.acquire(False):
            with self._lock:
                self._stats["queue_replacements"] += 1
            return
        try:
            replaced = int(self._new_frame.is_set())
            slot = 1 - int(self._shared_metadata[0])
            offset = slot * _PREVIEW_SLOT_CAPACITY
            self._shared_memory.buf[offset : offset + len(jpeg)] = jpeg
            self._shared_metadata[0] = slot
            self._shared_metadata[1] = len(jpeg)
            self._shared_metadata[2] = sequence
            self._shared_metadata[3] += 1
            self._new_frame.set()
        finally:
            lock.release()
        if replaced:
            with self._lock:
                self._stats["queue_replacements"] += replaced

    def pump(self) -> bool:
        self.raise_if_failed()
        with self._lock:
            return bool(self._stats["operator_aborted"] or self._abort.is_set())

    def raise_if_failed(self, *, _drain: bool = True) -> None:
        if _drain:
            self._drain_results()
        self._check_process_health()
        with self._lock:
            error = self._error
        if error is not None:
            raise RuntimeError("preview_processing_failed") from error

    def _check_process_health(self) -> None:
        if (
            not self._process_started
            or self._process is None
            or self._stop.is_set()
            or self._abort.is_set()
            or self._process.is_alive()
        ):
            return
        with self._lock:
            if self._error is None:
                self._stats["processing_errors"] += 1
                self._error = RuntimeError(
                    f"preview_process_exited_unexpectedly:{self._process.exitcode}"
                )

    def _handle_message(self, message) -> None:
        if not isinstance(message, dict):
            return
        child_stats = message.get("stats")
        with self._lock:
            if isinstance(child_stats, dict):
                for key, value in child_stats.items():
                    if key in self._stats:
                        self._stats[key] = value
            if message.get("kind") == "error" and self._error is None:
                self._error = RuntimeError(str(message.get("error", "preview error")))

    def _drain_results(self) -> None:
        while True:
            try:
                message = self._result_queue.get_nowait()
            except (queue.Empty, ValueError, OSError):
                return
            self._handle_message(message)

    def stop(self) -> None:
        self._stop.set()
        if self._process_started and self._process is not None:
            self._process.join(timeout=5.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2.0)
                with self._lock:
                    self._stats["processing_errors"] += 1
                    if self._error is None:
                        self._error = RuntimeError("preview_process_failed_to_stop")
            elif self._process.exitcode not in (0, None) and not self._abort.is_set():
                with self._lock:
                    self._stats["processing_errors"] += 1
                    if self._error is None:
                        self._error = RuntimeError(
                            f"preview_process_exitcode:{self._process.exitcode}"
                        )
        self._drain_results()
        if self._shared_memory is not None:
            self._shared_memory.close()
            try:
                self._shared_memory.unlink()
            except FileNotFoundError:
                pass
            self._shared_memory = None

    def stats(self) -> dict:
        self._drain_results()
        with self._lock:
            self._stats["operator_aborted"] = bool(
                self._stats["operator_aborted"] or self._abort.is_set()
            )
            return dict(self._stats)


def _drain_imu_messages(messages, recorder) -> int:
    drained = 0
    while True:
        try:
            message = messages.get_nowait()
        except queue.Empty:
            return drained
        kind = message[0]
        if kind == "sample":
            _, raw, read_started_ns, read_ended_ns = message
            recorder.record_packet(
                TwoUQ2Packet.parse(raw),
                read_started_ns=read_started_ns,
                read_ended_ns=read_ended_ns,
            )
        elif kind == "xu_error":
            recorder.record_xu_failure()
        elif kind == "batch":
            _, samples, xu_failures = message
            for raw, read_started_ns, read_ended_ns in samples:
                recorder.record_packet(
                    TwoUQ2Packet.parse(raw),
                    read_started_ns=read_started_ns,
                    read_ended_ns=read_ended_ns,
                )
            for _ in range(xu_failures):
                recorder.record_xu_failure()
            drained += len(samples) + xu_failures - 1
        else:
            raise RuntimeError(f"unknown isolated IMU message: {kind}")
        drained += 1


def _isolated_imu_process_main(
    device: str,
    xu_library: str,
    messages,
    status,
    start_requested,
    stop_requested,
    deadline_ns,
    warmup_seconds: float,
) -> None:
    bridge = _CtypesXuBridge(xu_library)
    descriptor = None
    try:
        descriptor = bridge.open(device)
        warmup_deadline_ns = time.monotonic_ns() + int(
            warmup_seconds * 1_000_000_000
        )
        while (
            time.monotonic_ns() < warmup_deadline_ns
            and not stop_requested.is_set()
        ):
            bridge.read_imu27(descriptor)
        status.put_nowait({"kind": "ready"})
        while not start_requested.wait(timeout=0.05):
            if stop_requested.is_set():
                return
        batch = []
        xu_failures = 0
        while (
            time.monotonic_ns() < deadline_ns.value
            and not stop_requested.is_set()
        ):
            read_started_ns = time.monotonic_ns()
            try:
                raw = bridge.read_imu27(descriptor)
            except OSError:
                xu_failures += 1
            else:
                read_ended_ns = time.monotonic_ns()
                batch.append((raw, read_started_ns, read_ended_ns))
            if len(batch) + xu_failures >= _IMU_BATCH_SIZE:
                messages.put_nowait(("batch", tuple(batch), xu_failures))
                batch.clear()
                xu_failures = 0
        if batch or xu_failures:
            messages.put_nowait(("batch", tuple(batch), xu_failures))
        status.put_nowait({"kind": "complete"})
    except BaseException as error:
        try:
            status.put_nowait(
                {"kind": "error", "error": f"{type(error).__name__}:{error}"}
            )
        except queue.Full:
            pass
    finally:
        if descriptor is not None:
            try:
                bridge.close(descriptor)
            except OSError:
                pass


class IsolatedImuReader:
    """Poll the synchronous XU control in a process isolated from video Python."""

    def __init__(
        self,
        *,
        device: str,
        xu_library: str,
        warmup_seconds: float = 0.75,
    ):
        self._device = device
        self._xu_library = xu_library
        self._warmup_seconds = warmup_seconds
        self._context = multiprocessing.get_context("spawn")
        self._messages = None
        self._status = None
        self._start_requested = None
        self._stop_requested = None
        self._deadline_ns = None
        self._process = None
        self._ready = False
        self._closed = False

    @property
    def start_method(self) -> str:
        return self._context.get_start_method()

    def start(self, *, timeout_seconds: float = 5.0) -> None:
        if self._process is not None or self._closed:
            raise RuntimeError("isolated IMU reader cannot be restarted")
        self._messages = self._context.Queue(maxsize=_IMU_QUEUE_CAPACITY)
        self._status = self._context.Queue(maxsize=8)
        self._start_requested = self._context.Event()
        self._stop_requested = self._context.Event()
        self._deadline_ns = self._context.Value("q", 0)
        self._process = self._context.Process(
            target=_isolated_imu_process_main,
            args=(
                self._device,
                self._xu_library,
                self._messages,
                self._status,
                self._start_requested,
                self._stop_requested,
                self._deadline_ns,
                self._warmup_seconds,
            ),
            name="2uq2-isolated-imu-reader",
            daemon=False,
        )
        self._process.start()
        wait_deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < wait_deadline:
            try:
                message = self._status.get(timeout=0.05)
            except queue.Empty:
                if not self._process.is_alive():
                    break
                continue
            if message.get("kind") == "ready":
                self._ready = True
                return
            if message.get("kind") == "error":
                raise RuntimeError(
                    f"isolated_imu_start_failed:{message.get('error')}"
                )
        raise RuntimeError("isolated_imu_ready_timeout")

    def begin(self, *, deadline_ns: int) -> None:
        if not self._ready or self._closed:
            raise RuntimeError("isolated IMU reader is not ready")
        if deadline_ns <= time.monotonic_ns():
            raise ValueError("isolated IMU deadline must be in the future")
        self._deadline_ns.value = deadline_ns
        self._start_requested.set()

    def drain(self, recorder) -> int:
        if self._messages is None:
            return 0
        return _drain_imu_messages(self._messages, recorder)

    def finish(self, recorder, *, timeout_seconds: float = 5.0) -> None:
        if self._process is None:
            return
        wait_deadline = time.monotonic() + timeout_seconds
        while self._process.is_alive() and time.monotonic() < wait_deadline:
            self.drain(recorder)
            self._process.join(timeout=0.02)
        self.drain(recorder)
        if self._process.is_alive():
            raise RuntimeError("isolated_imu_process_failed_to_stop")
        errors = []
        while True:
            try:
                message = self._status.get_nowait()
            except queue.Empty:
                break
            if message.get("kind") == "error":
                errors.append(message.get("error"))
        if self._process.exitcode != 0:
            errors.append(f"exitcode:{self._process.exitcode}")
        if errors:
            raise RuntimeError("isolated_imu_process_failed:" + ",".join(errors))

    def request_stop(self) -> None:
        """Ask the child to stop without closing queues needed for final drain."""
        if self._stop_requested is not None:
            self._stop_requested.set()

    def stop(self) -> None:
        if self._closed:
            return
        if self._stop_requested is not None:
            self._stop_requested.set()
        if self._process is not None:
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2.0)
        for channel in (self._messages, self._status):
            if channel is not None:
                channel.close()
                channel.join_thread()
        self._closed = True


def has_complete_jpeg_boundaries(payload: bytes) -> bool:
    return len(payload) >= 4 and payload[:2] == b"\xff\xd8" and payload[-2:] == b"\xff\xd9"


def resolve_output_directory(requested: Path, artifact_root: Path) -> Path:
    root = Path(os.path.abspath(artifact_root))
    if not root.is_dir() or root.resolve(strict=True) != root:
        raise ValueError("artifact root must be an existing canonical directory")
    candidate = Path(os.path.abspath(requested))
    if candidate.parent != root:
        raise ValueError("output must be a direct child of the calibration artifact root")
    if not _SAFE_SESSION_NAME.fullmatch(candidate.name):
        raise ValueError("output session name is unsafe")
    if candidate.exists():
        raise FileExistsError(f"output already exists: {candidate}")
    return candidate


def build_capture_report(
    *,
    duration_s: float,
    video_frames: int,
    imu_packets: int,
    imu_samples: int,
    video_timestamp_regressions: int,
    imu_timestamp_regressions: int,
    bad_jpegs: int,
    xu_failures: int,
    operator_aborted: bool = False,
    video_estimated_drops: int = 0,
    imu_estimated_drops: int = 0,
    video_max_jitter_ns: int = 0,
    imu_max_jitter_ns: int = 0,
    imu_max_interval_ns: int = 0,
) -> dict:
    rates = {
        "video_hz": video_frames / duration_s,
        "imu_packet_hz": imu_packets / duration_s,
        "imu_sample_hz": imu_samples / duration_s,
    }
    measurements = {
        **rates,
        "one_main_sample_per_packet": imu_samples / imu_packets if imu_packets else None,
        "video_timestamp_regressions": video_timestamp_regressions,
        "imu_timestamp_regressions": imu_timestamp_regressions,
        "bad_jpegs": bad_jpegs,
        "xu_failures": xu_failures,
        "operator_aborted": operator_aborted,
        "video_drop_ratio": (
            video_estimated_drops / (video_frames + video_estimated_drops)
            if video_frames + video_estimated_drops
            else 1.0
        ),
        "imu_drop_ratio": (
            imu_estimated_drops / (imu_samples + imu_estimated_drops)
            if imu_samples + imu_estimated_drops
            else 1.0
        ),
        "video_max_jitter_ms": video_max_jitter_ns / 1_000_000,
        "imu_max_jitter_ms": imu_max_jitter_ns / 1_000_000,
        "imu_max_interval_ms": imu_max_interval_ns / 1_000_000,
    }
    accepted = {
        "video_hz": 57.0 <= rates["video_hz"] <= 63.0,
        "imu_packet_hz": 190.0 <= rates["imu_packet_hz"] <= 210.0,
        "imu_sample_hz": 190.0 <= rates["imu_sample_hz"] <= 210.0,
        "one_main_sample_per_packet": imu_samples == imu_packets,
        "video_timestamp_regressions": video_timestamp_regressions == 0,
        "imu_timestamp_regressions": imu_timestamp_regressions == 0,
        "bad_jpegs": bad_jpegs == 0,
        "xu_failures": xu_failures == 0,
        "operator_aborted": not operator_aborted,
        "video_drop_ratio": measurements["video_drop_ratio"] <= 0.01,
        "imu_drop_ratio": measurements["imu_drop_ratio"] <= 0.01,
        "video_max_jitter_ms": measurements["video_max_jitter_ms"] <= 8.334,
        "imu_max_jitter_ms": measurements["imu_max_jitter_ms"] <= 2.5,
    }
    thresholds = {
        "video_hz": "57.0 <= value <= 63.0",
        "imu_packet_hz": "190.0 <= value <= 210.0",
        "imu_sample_hz": "190.0 <= value <= 210.0",
        "one_main_sample_per_packet": "== 1.0",
        "video_timestamp_regressions": "== 0",
        "imu_timestamp_regressions": "== 0",
        "bad_jpegs": "== 0",
        "xu_failures": "== 0",
        "operator_aborted": "== false",
        "video_drop_ratio": "<= 0.01 inferred from acquisition intervals",
        "imu_drop_ratio": "<= 0.01 inferred from acquisition intervals",
        "video_max_jitter_ms": "<= 8.334",
        "imu_max_jitter_ms": "<= 2.5",
    }
    return {
        "schema": "ego.2uq2.calibration-capture.v1",
        "status": "PASS" if all(accepted.values()) else "FAIL",
        "duration_s": duration_s,
        "rates": rates,
        "counts": {
            "video_frames": video_frames,
            "imu_packets": imu_packets,
            "imu_samples": imu_samples,
            "video_estimated_drops": video_estimated_drops,
            "imu_estimated_drops": imu_estimated_drops,
        },
        "checks": {
            name: {
                "status": "PASS" if passed else "FAIL",
                "threshold": thresholds[name],
                "measurement": measurements[name],
            }
            for name, passed in accepted.items()
        },
        "timing": {
            "imu_max_interval_ms": measurements["imu_max_interval_ms"],
        },
    }


def apply_stage_acceptance(
    report: dict,
    *,
    stage: str,
    preview_report: dict | None,
) -> dict:
    if stage not in _CAPTURE_STAGES:
        raise ValueError(f"unsupported capture stage: {stage}")
    strict_transport_status = report["status"]
    report["stage"] = stage
    report["strict_transport_status"] = strict_transport_status
    report["transport_status"] = strict_transport_status

    if stage == "strict":
        checks = {
            "strict_transport": {
                "status": strict_transport_status,
                "threshold": "all strict transport checks PASS",
                "measurement": strict_transport_status,
            }
        }
        if preview_report is not None:
            checks["calibration_preview"] = {
                "status": preview_report["status"],
                "threshold": "== PASS when preview is enabled",
                "measurement": preview_report["status"],
            }
        status = "PASS" if all(
            check["status"] == "PASS" for check in checks.values()
        ) else "FAIL"
        report["stage_acceptance"] = {"status": status, "checks": checks}
        report["status"] = status
        return report

    strict_video_checks = (
        "video_hz",
        "video_timestamp_regressions",
        "bad_jpegs",
        "operator_aborted",
        "video_drop_ratio",
        "video_max_jitter_ms",
    )
    checks = {name: dict(report["checks"][name]) for name in strict_video_checks}

    if stage == "stereo":
        preview_status = preview_report["status"] if preview_report is not None else "FAIL"
        checks["calibration_preview"] = {
            "status": preview_status,
            "threshold": "== PASS with repeated AprilGrid visibility in both halves",
            "measurement": preview_status if preview_report is not None else "SKIPPED",
        }
        status = "PASS" if all(
            check["status"] == "PASS" for check in checks.values()
        ) else "FAIL"
        report["stage_acceptance"] = {
            "status": status,
            "checks": checks,
            "note": "Stereo calibration does not use IMU samples; strict IMU failures remain visible in checks.",
        }
        report["status"] = status
        return report

    measurements = {
        "imu_packet_hz": report["rates"]["imu_packet_hz"],
        "imu_sample_hz": report["rates"]["imu_sample_hz"],
        "imu_drop_ratio": report["checks"]["imu_drop_ratio"]["measurement"],
        "imu_max_interval_ms": report["timing"]["imu_max_interval_ms"],
    }
    provisional = {
        "imu_packet_hz": 170.0 <= measurements["imu_packet_hz"] <= 210.0,
        "imu_sample_hz": 170.0 <= measurements["imu_sample_hz"] <= 210.0,
        "imu_drop_ratio": measurements["imu_drop_ratio"] <= 0.15,
        "imu_max_interval_ms": measurements["imu_max_interval_ms"] <= 30.0,
    }
    thresholds = {
        "imu_packet_hz": "170.0 <= value <= 210.0 (provisional calibration only)",
        "imu_sample_hz": "170.0 <= value <= 210.0 (provisional calibration only)",
        "imu_drop_ratio": "<= 0.15 inferred from acquisition intervals (provisional)",
        "imu_max_interval_ms": "<= 30.0 (provisional)",
    }
    for name, passed in provisional.items():
        checks[name] = {
            "status": "PASS" if passed else "FAIL",
            "threshold": thresholds[name],
            "measurement": measurements[name],
        }
    for name in (
        "one_main_sample_per_packet",
        "imu_timestamp_regressions",
        "xu_failures",
        "imu_max_jitter_ms",
    ):
        checks[name] = dict(report["checks"][name])
    status = "PROVISIONAL_PASS" if all(
        check["status"] == "PASS" for check in checks.values()
    ) else "FAIL"
    report["stage_acceptance"] = {
        "status": status,
        "checks": checks,
        "note": (
            "Temporary calibration gate authorized by the operator. "
            "This is not final product transport acceptance."
        ),
    }
    report["status"] = status
    return report


class CalibrationRecorder:
    def __init__(
        self,
        writer: AppendOnlySessionWriter,
        gst,
        pipeline,
        *,
        queue_capacity: int = 1024,
        preview: StereoAprilGridPreview | None = None,
    ):
        self._writer = writer
        self._gst = gst
        self._pipeline = pipeline
        self._preview = preview
        self._lock = threading.Lock()
        self._active = False
        self._error = None
        self._video_sequence = 0
        self._imu_packet_index = 0
        self._imu_sample_index = 0
        self._video_frames = 0
        self._imu_packets = 0
        self._imu_samples = 0
        self._video_timestamp_regressions = 0
        self._imu_timestamp_regressions = 0
        self._bad_jpegs = 0
        self._xu_failures = 0
        self._last_video_ns = None
        self._last_imu_ns = None
        self._video_estimated_drops = 0
        self._imu_estimated_drops = 0
        self._video_max_jitter_ns = 0
        self._imu_max_jitter_ns = 0
        self._imu_max_interval_ns = 0
        self._write_queue = queue.Queue(maxsize=queue_capacity)
        self._write_sentinel = object()
        self._writer_finished = False
        self._writer_thread = threading.Thread(
            target=self._write_loop,
            name="2uq2-calibration-writer",
            daemon=True,
        )
        self._writer_thread.start()

    def begin(self) -> None:
        with self._lock:
            self._active = True

    def end(self) -> None:
        with self._lock:
            self._active = False

    def raise_if_failed(self) -> None:
        with self._lock:
            error = self._error
        if error is not None:
            raise RuntimeError("calibration_capture_callback_failed") from error

    def _write_loop(self) -> None:
        while True:
            item = self._write_queue.get()
            try:
                if item is self._write_sentinel:
                    return
                record, payload = item
                with self._lock:
                    failed = self._error is not None
                if not failed:
                    try:
                        self._writer.append(record, payload)
                    except Exception as exc:
                        with self._lock:
                            if self._error is None:
                                self._error = exc
            finally:
                self._write_queue.task_done()

    def _queue_write(self, record: SensorRecord, payload: bytes) -> None:
        self.raise_if_failed()
        try:
            self._write_queue.put_nowait((record, payload))
        except queue.Full as exc:
            with self._lock:
                if self._error is None:
                    self._error = RuntimeError("calibration_write_queue_full")
            raise RuntimeError("calibration_write_queue_full") from exc

    def finish_writes(self) -> None:
        if self._writer_finished:
            self.raise_if_failed()
            return
        self._write_queue.join()
        self._write_queue.put(self._write_sentinel)
        self._writer_thread.join()
        self._writer_finished = True
        self.raise_if_failed()

    def video_callback(self, sink):
        try:
            sample = sink.emit("pull-sample")
            if sample is None:
                raise RuntimeError("appsink returned no sample")
            buffer = sample.get_buffer()
            if buffer is None:
                raise RuntimeError("sample returned no buffer")
            mapped, map_info = buffer.map(self._gst.MapFlags.READ)
            if not mapped:
                raise RuntimeError("failed to map MJPEG buffer")
            try:
                jpeg = bytes(map_info.data)
            finally:
                buffer.unmap(map_info)
            caps = sample.get_caps()
            structure = caps.get_structure(0) if caps is not None else None
            width = int(structure.get_value("width")) if structure is not None else 0
            height = int(structure.get_value("height")) if structure is not None else 0
            base_time = int(self._pipeline.get_base_time())
            pts = int(buffer.pts)
            if base_time == self._gst.CLOCK_TIME_NONE or pts == self._gst.CLOCK_TIME_NONE:
                raise RuntimeError("video_timestamp_unavailable")
            acquisition_ns = base_time + pts
            arrival_ns = time.monotonic_ns()
            jpeg_valid = has_complete_jpeg_boundaries(jpeg)
            valid = (width, height) == (3840, 1080) and jpeg_valid
            with self._lock:
                if not self._active:
                    record = None
                    preview_sequence = self._video_sequence
                else:
                    self._video_sequence += 1
                    self._video_frames += 1
                    if self._last_video_ns is not None and acquisition_ns < self._last_video_ns:
                        self._video_timestamp_regressions += 1
                    elif self._last_video_ns is not None:
                        interval_ns = acquisition_ns - self._last_video_ns
                        self._video_estimated_drops += estimated_missing_samples(
                            interval_ns, _VIDEO_PERIOD_NS
                        )
                        self._video_max_jitter_ns = max(
                            self._video_max_jitter_ns,
                            period_multiple_jitter_ns(
                                interval_ns, _VIDEO_PERIOD_NS
                            ),
                        )
                    self._last_video_ns = acquisition_ns
                    self._bad_jpegs += int(not jpeg_valid)
                    record = SensorRecord(
                        stream_id="ego.video",
                        sequence=self._video_sequence,
                        acquisition_ns=acquisition_ns,
                        arrival_ns=arrival_ns,
                        clock_domain="host_monotonic",
                        warmup=False,
                        valid=valid,
                        metadata={
                            "encoding": "MJPG",
                            "width": width,
                            "height": height,
                            "layout": "side_by_side_left_then_right_unverified",
                        },
                    )
                    preview_sequence = record.sequence
            if record is None:
                if self._preview is not None:
                    self._preview.submit(
                        sequence=preview_sequence,
                        acquisition_ns=acquisition_ns,
                        jpeg=jpeg,
                    )
                return self._gst.FlowReturn.OK
            self._queue_write(record, jpeg)
            if self._preview is not None:
                self._preview.submit(
                    sequence=record.sequence,
                    acquisition_ns=acquisition_ns,
                    jpeg=jpeg,
                )
            return self._gst.FlowReturn.OK
        except Exception as exc:
            with self._lock:
                if self._error is None:
                    self._error = exc
            return self._gst.FlowReturn.ERROR

    def record_xu_failure(self) -> None:
        with self._lock:
            self._xu_failures += 1

    def record_packet(
        self,
        packet: TwoUQ2Packet,
        *,
        read_started_ns: int,
        read_ended_ns: int,
    ) -> None:
        acquisition_ns = (read_started_ns + read_ended_ns) // 2
        queued = []
        with self._lock:
            if not self._active:
                return
            self._imu_packet_index += 1
            self._imu_packets += 1
            queued.append((
                SensorRecord(
                    stream_id="ego.xu.packet",
                    sequence=self._imu_packet_index,
                    acquisition_ns=acquisition_ns,
                    arrival_ns=read_ended_ns,
                    clock_domain="host_monotonic",
                    warmup=False,
                    valid=True,
                    metadata={
                        "vendor_video_sequence": packet.sequence,
                        "size_expected": 27,
                        "published_samples_in_packet": 1,
                        "published_group": "main_bytes_3_14_only",
                        "backup_group_policy": "raw_packet_only_not_published",
                    },
                ),
                packet.raw,
            ))
            self._imu_sample_index += 1
            self._imu_samples += 1
            if self._last_imu_ns is not None and acquisition_ns < self._last_imu_ns:
                self._imu_timestamp_regressions += 1
            elif self._last_imu_ns is not None:
                interval_ns = acquisition_ns - self._last_imu_ns
                self._imu_max_interval_ns = max(
                    self._imu_max_interval_ns,
                    interval_ns,
                )
                self._imu_estimated_drops += estimated_missing_samples(
                    interval_ns, _IMU_PERIOD_NS
                )
                self._imu_max_jitter_ns = max(
                    self._imu_max_jitter_ns,
                    period_multiple_jitter_ns(interval_ns, _IMU_PERIOD_NS),
                )
            self._last_imu_ns = acquisition_ns
            queued.append((
                SensorRecord(
                    stream_id="ego.imu",
                    sequence=self._imu_sample_index,
                    acquisition_ns=acquisition_ns,
                    arrival_ns=read_ended_ns,
                    clock_domain="host_monotonic",
                    warmup=False,
                    valid=True,
                    metadata={
                        "vendor_video_sequence": packet.sequence,
                        "wire_group": "main_bytes_3_14",
                        "accel_m_s2": packet.main_accel_m_s2,
                        "gyro_rad_s": packet.main_gyro_rad_s,
                        "timestamp_model": "host_monotonic_control_transfer_midpoint",
                    },
                ),
                packet.main_raw,
            ))
        for record, payload in queued:
            self._queue_write(record, payload)

    def stats(self) -> dict:
        with self._lock:
            return {
                "video_frames": self._video_frames,
                "imu_packets": self._imu_packets,
                "imu_samples": self._imu_samples,
                "video_timestamp_regressions": self._video_timestamp_regressions,
                "imu_timestamp_regressions": self._imu_timestamp_regressions,
                "bad_jpegs": self._bad_jpegs,
                "xu_failures": self._xu_failures,
                "video_estimated_drops": self._video_estimated_drops,
                "imu_estimated_drops": self._imu_estimated_drops,
                "video_max_jitter_ns": self._video_max_jitter_ns,
                "imu_max_jitter_ns": self._imu_max_jitter_ns,
                "imu_max_interval_ns": self._imu_max_interval_ns,
            }


def format_capture_progress(*, elapsed_s: float, duration_s: float, stats: dict) -> str:
    remaining_s = max(0.0, duration_s - elapsed_s)
    return (
        f"PROGRESS: elapsed={elapsed_s:.1f}s remaining={remaining_s:.1f}s "
        f"video_frames={int(stats.get('video_frames', 0))} "
        f"imu_samples={int(stats.get('imu_samples', 0))}"
    )


def _capture_until_deadline(
    *,
    recorder,
    imu_reader,
    preview_monitor,
    window_start_ns: int,
    deadline_ns: int,
    progress_interval_s: float = 5.0,
    clock_ns=time.monotonic_ns,
    sleep=time.sleep,
    emit=lambda message: print(message, flush=True),
) -> tuple[int, bool]:
    """Run the observable capture loop and turn Ctrl-C into a clean abort."""
    operator_aborted = False
    duration_s = (deadline_ns - window_start_ns) / 1_000_000_000
    progress_interval_ns = max(1, int(progress_interval_s * 1_000_000_000))
    next_progress_ns = window_start_ns + progress_interval_ns
    try:
        while clock_ns() < deadline_ns:
            recorder.raise_if_failed()
            imu_reader.drain(recorder)
            if preview_monitor is not None and preview_monitor.pump():
                operator_aborted = True
                emit("ABORTED: q/ESC received from preview; finalizing data and report.")
                break
            now_ns = clock_ns()
            if now_ns >= next_progress_ns:
                elapsed_s = (now_ns - window_start_ns) / 1_000_000_000
                emit(
                    format_capture_progress(
                        elapsed_s=elapsed_s,
                        duration_s=duration_s,
                        stats=recorder.stats(),
                    )
                )
                while next_progress_ns <= now_ns:
                    next_progress_ns += progress_interval_ns
            sleep(0.001)
    except KeyboardInterrupt:
        operator_aborted = True
        emit("ABORTED: Ctrl-C received; finalizing data and report.")
    return clock_ns(), operator_aborted


def capture(
    *,
    device: str,
    xu_library: str,
    output: Path,
    duration_s: float,
    preview: bool = True,
    preview_hz: float = 5.0,
    stage: str = "strict",
) -> dict:
    print(
        f"SETUP: device={device} stage={stage} duration={duration_s:.1f}s "
        f"output={output}",
        flush=True,
    )
    gst = _load_gst()
    gst.init(None)
    print("SETUP: opening MJPG 3840x1080@60 video pipeline...", flush=True)
    pipeline = gst.parse_launch(build_pipeline_description(device))
    sink = pipeline.get_by_name("sink")
    if sink is None:
        raise RuntimeError("2UQ2 pipeline has no appsink")
    preview_monitor = StereoAprilGridPreview(preview_hz=preview_hz) if preview else None
    imu_reader = IsolatedImuReader(device=device, xu_library=xu_library)
    window_start_ns = None
    window_end_ns = None
    operator_aborted = False
    with ExitStack() as cleanup:
        if preview_monitor is not None:
            print(
                f"PREVIEW: starting live stereo AprilGrid view at {preview_hz:g} Hz...",
                flush=True,
            )
            cleanup.callback(preview_monitor.stop)
            preview_monitor.start()
            print("PREVIEW: ready; press q/ESC in the window to abort cleanly.", flush=True)
        else:
            print("PREVIEW: disabled; terminal progress remains enabled.", flush=True)
        writer = AppendOnlySessionWriter(output)
        print("STORAGE: append-only session opened.", flush=True)
        cleanup.callback(writer.close)
        recorder = CalibrationRecorder(writer, gst, pipeline, preview=preview_monitor)
        cleanup.callback(recorder.finish_writes)
        cleanup.callback(recorder.end)
        handler = sink.connect("new-sample", recorder.video_callback)
        cleanup.callback(sink.disconnect, handler)
        cleanup.callback(pipeline.set_state, gst.State.NULL)
        if pipeline.set_state(gst.State.PLAYING) == gst.StateChangeReturn.FAILURE:
            raise RuntimeError("gstreamer_start_failed")
        pipeline.get_state(5 * gst.SECOND)
        print("VIDEO: streaming.", flush=True)
        cleanup.callback(imu_reader.stop)
        print("IMU: opening XU reader and completing active warm-up...", flush=True)
        imu_reader.start()
        print("IMU: ready.", flush=True)
        recorder.raise_if_failed()
        if preview_monitor is not None:
            preview_monitor.raise_if_failed()
        window_start_ns = time.monotonic_ns()
        deadline_ns = window_start_ns + int(duration_s * 1_000_000_000)
        recorder.begin()
        imu_reader.begin(deadline_ns=deadline_ns)
        print(
            "RECORDING: started; move the Ego smoothly through all 6 axes. "
            "Progress prints every 5 seconds.",
            flush=True,
        )
        window_end_ns, operator_aborted = _capture_until_deadline(
            recorder=recorder,
            imu_reader=imu_reader,
            preview_monitor=preview_monitor,
            window_start_ns=window_start_ns,
            deadline_ns=deadline_ns,
        )
        print("FINALIZING: draining IMU and writing manifest/report...", flush=True)
        if operator_aborted:
            imu_reader.request_stop()
        imu_reader.finish(recorder)
        recorder.end()

    measured_duration_s = (window_end_ns - window_start_ns) / 1_000_000_000
    stats = recorder.stats()
    preview_stats = preview_monitor.stats() if preview_monitor is not None else None
    operator_aborted = capture_was_aborted(operator_aborted, preview_stats)
    manifest = writer.close()
    report = build_capture_report(
        duration_s=measured_duration_s,
        operator_aborted=operator_aborted,
        **stats,
    )
    report["output"] = str(output)
    report["device"] = {
        "path": device,
        "video": "MJPG 3840x1080 60 Hz",
        "imu": "one physical IMU, main bytes 3-14 published; backup retained raw only",
    }
    report["manifest"] = manifest
    if preview_monitor is None:
        report["calibration_preview"] = {
            "status": "SKIPPED",
            "reason": "explicit_no_preview",
        }
    else:
        preview_report = build_preview_report(preview_stats)
        report["calibration_preview"] = preview_report
    apply_stage_acceptance(
        report,
        stage=stage,
        preview_report=preview_report if preview_monitor is not None else None,
    )
    report_path = output / "capture_report.json"
    temporary = output / "capture_report.json.tmp"
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(report, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, report_path)
    print(f"REPORT: {report_path}", flush=True)
    return report


def main(argv=None) -> int:
    repository_root = Path(__file__).resolve().parents[3]
    artifact_root = repository_root / "artifacts" / "ego_calibration"
    parser = argparse.ArgumentParser(
        description="Record 2UQ2 60 Hz SBS plus 200 Hz IMU calibration data"
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="disable the default live left/right AprilGrid preview",
    )
    parser.add_argument(
        "--preview-hz",
        type=float,
        default=5.0,
        help="AprilGrid preview processing rate (default: 5 Hz)",
    )
    parser.add_argument(
        "--xu-library", default="/opt/three-device-slam/lib/libtwo_uq2_xu.so"
    )
    parser.add_argument(
        "--stage",
        choices=_CAPTURE_STAGES,
        default="strict",
        help="acceptance stage: strict, stereo, or provisional camera-IMU",
    )
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.preview_hz <= 0:
        parser.error("--preview-hz must be positive")
    try:
        output = resolve_output_directory(args.output, artifact_root)
        report = capture(
            device=args.device,
            xu_library=args.xu_library,
            output=output,
            duration_s=args.duration,
            preview=not args.no_preview,
            preview_hz=args.preview_hz,
            stage=args.stage,
        )
    except KeyboardInterrupt:
        print("ABORTED: Ctrl-C received during setup/finalization; inspect the output before reuse.")
        return 130
    except Exception as exc:
        print(f"BLOCKED: {type(exc).__name__}: {exc}")
        return 3
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    if report["status"] == "PASS":
        if args.stage == "stereo":
            print("NEXT: preserve this dataset and run the offline stereo calibration solver.")
        else:
            print("NEXT: strict capture accepted; preserve the dataset and continue the calibrated workflow.")
    elif report["status"] == "PROVISIONAL_PASS":
        print("NEXT: run the provisional camera-IMU solver; final transport acceptance remains pending vendor FIFO support.")
    else:
        print("STOP: this stage did not pass; do not advance to the solver.")
    return 0 if report["status"] in ("PASS", "PROVISIONAL_PASS") else 2


if __name__ == "__main__":
    raise SystemExit(main())
