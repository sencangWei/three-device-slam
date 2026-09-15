"""Capture one bounded D405 RGB/Y8I + STM32 session on an RK3576 host."""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import hashlib
import json
import mmap
import os
import queue
import re
import select
import shutil
import signal
import subprocess
import termios
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable

from three_device_slam.devices.d405_umi.imu.stream_protocol import StreamDecoder
from three_device_slam.edge_rk3576.preview import MjpegPreviewServer


SCHEMA = "three-device-slam.rk3576-umi-session.v1"
WIDTH = 1280
HEIGHT = 720
FPS = 30
RGB_FPS = 30
IR_FRAME_BYTES = WIDTH * HEIGHT * 2  # D405 Y8I: one byte per eye per pixel.
STM32_PACKET_BYTES = 63
STM32_BAUD = 921_600
STM32_NOMINAL_HZ = 400
RGB_BITRATE = 6_000_000
Y8I_FOURCC = "0x20493859"
MIN_FREE_RESERVE_BYTES = 2 * 1024**3
CAPTURE_QUEUE_FRAMES = 128
IR_V4L2_BUFFERS = 32
DIRECT_IO_CHUNK_BYTES = 256 * 1024
DIRECT_IO_RATE_BYTES_PER_SECOND = 64 * 1024 * 1024

_V4L2_FRAME = re.compile(
    r"cap dqbuf:\s+\d+\s+seq:\s*(\d+)\s+bytesused:\s*(\d+)\s+"
    r"ts:\s*(\d+)\.(\d{6})\s+(.*)$"
)
_GST_RGB_FRAME = re.compile(
    r"GstIdentity:rgbraw: last-message = chain\s+\*+\s+"
    r"\(rgbraw:sink\) \((\d+) bytes, .*?pts:\s*(\d+):(\d{2}):(\d{2})\.(\d{9}),"
)


class CaptureError(RuntimeError):
    """The requested acquisition cannot be completed safely."""


def _json_bytes(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        + "\n"
    ).encode("utf-8")


def _write_atomic(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("xb") as stream:
        stream.write(_json_bytes(value))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_v4l2_frame_line(line: str) -> dict[str, Any] | None:
    match = _V4L2_FRAME.search(line)
    if match is None:
        return None
    sequence, bytes_used, seconds, micros, flags = match.groups()
    return {
        "sequence": int(sequence),
        "bytes_used": int(bytes_used),
        "source_monotonic_ns": int(seconds) * 1_000_000_000 + int(micros) * 1000,
        "flags": flags.strip(),
    }


def parse_gstreamer_rgb_line(line: str) -> dict[str, int] | None:
    match = _GST_RGB_FRAME.search(line)
    if match is None:
        return None
    size, hours, minutes, seconds, nanos = (int(value) for value in match.groups())
    pts_ns = (
        ((hours * 60 + minutes) * 60 + seconds) * 1_000_000_000 + nanos
    )
    return {"bytes_used": size, "pipeline_pts_ns": pts_ns}


def _run(command: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=check,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def _udev_properties(node: str) -> dict[str, str]:
    result = _run(["udevadm", "info", "-q", "property", "-n", node])
    properties: dict[str, str] = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            properties[key] = value
    return properties


def _profile_available(text: str, fourcc: str) -> bool:
    section = None
    for block in re.split(r"\n(?=\s*\[\d+\]:)", text):
        if f"'{fourcc}'" in block:
            section = block
            break
    return bool(
        section
        and re.search(r"Size:\s+Discrete\s+1280x720", section)
        and re.search(r"30\.000 fps", section)
    )


def discover_d405_nodes(serial: str) -> tuple[str, str]:
    candidates: list[tuple[str, str]] = []
    for node in sorted(glob.glob("/dev/video[0-9]*")):
        try:
            properties = _udev_properties(node)
        except subprocess.CalledProcessError:
            continue
        if properties.get("ID_SERIAL_SHORT") != serial:
            continue
        formats = _run(
            ["v4l2-ctl", "-d", node, "--list-formats-ext"], check=False
        ).stdout
        if _profile_available(formats, "Y8I "):
            candidates.append(("ir", node))
        if _profile_available(formats, "YUYV"):
            candidates.append(("rgb", node))
    ir_nodes = [node for kind, node in candidates if kind == "ir"]
    rgb_nodes = [node for kind, node in candidates if kind == "rgb"]
    if len(ir_nodes) != 1 or len(rgb_nodes) != 1:
        raise CaptureError(
            f"cannot identify one D405 Y8I and one RGB node for serial {serial}: "
            f"ir={ir_nodes}, rgb={rgb_nodes}"
        )
    return ir_nodes[0], rgb_nodes[0]


def verify_d405_superspeed(serial: str) -> dict[str, str]:
    matches = []
    for device in Path("/sys/bus/usb/devices").glob("*"):
        try:
            if (device / "serial").read_text().strip() == serial:
                matches.append(device)
        except OSError:
            continue
    if len(matches) != 1:
        raise CaptureError(f"expected one USB device for D405 serial {serial}")
    device = matches[0]
    speed = (device / "speed").read_text().strip()
    vidpid = (
        f"{(device / 'idVendor').read_text().strip()}:"
        f"{(device / 'idProduct').read_text().strip()}"
    )
    if vidpid.lower() != "8086:0b5b" or float(speed) < 5000:
        raise CaptureError(f"D405 is not on SuperSpeed: vidpid={vidpid}, speed={speed}")
    return {"usb_device": device.name, "vidpid": vidpid, "speed_mbit_s": speed}


def discover_stm32_port(requested: str | None) -> str:
    if requested:
        path = Path(requested)
        if not path.exists():
            raise CaptureError(f"STM32 serial path does not exist: {requested}")
        return str(path)
    matches = sorted(glob.glob("/dev/serial/by-id/*CP2102N*"))
    if len(matches) != 1:
        raise CaptureError(f"expected one CP2102N serial adapter, found {matches}")
    return matches[0]


def _device_owners(nodes: list[str]) -> list[dict[str, Any]]:
    targets = {str(Path(node).resolve()) for node in nodes}
    owners = []
    for proc in Path("/proc").glob("[0-9]*"):
        if int(proc.name) == os.getpid():
            continue
        try:
            for descriptor in (proc / "fd").iterdir():
                try:
                    target = str(descriptor.resolve(strict=True))
                except OSError:
                    continue
                if target in targets:
                    command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                        "utf-8", "replace"
                    )
                    owners.append({"pid": int(proc.name), "device": target, "command": command})
        except (OSError, ValueError):
            continue
    return owners


class _LoggedProcess:
    def __init__(
        self,
        command: list[str],
        log_path: Path,
        callback: Callable[[str, int], None],
    ) -> None:
        self.command = command
        self.log_path = log_path
        self.callback = callback
        self.process: subprocess.Popen[str] | None = None
        self.returncode: int | None = None
        self.started_monotonic_ns: int | None = None
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _loop(self) -> None:
        with self.log_path.open("x", encoding="utf-8") as log:
            self.started_monotonic_ns = time.monotonic_ns()
            self.process = subprocess.Popen(
                self.command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
            )
            assert self.process.stdout is not None
            for line in self.process.stdout:
                log.write(line)
                self.callback(line, time.monotonic_ns())
            self.returncode = self.process.wait()
            log.flush()
            os.fsync(log.fileno())

    def terminate(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)

    def stop(self) -> None:
        self.terminate()
        if self.thread.ident is None:
            return
        self.thread.join(timeout=3.0)
        if self.thread.is_alive() and self.process is not None:
            self.process.kill()
            self.thread.join(timeout=2.0)


class _FrameSpooler:
    """Drain fixed-size FIFO frames into a bounded queue and write separately."""

    def __init__(
        self,
        fifo_path: Path,
        output_path: Path,
        *,
        frame_bytes: int,
        queue_frames: int = CAPTURE_QUEUE_FRAMES,
    ) -> None:
        if frame_bytes < 1 or queue_frames < 1:
            raise ValueError("frame_bytes and queue_frames must be positive")
        self.fifo_path = fifo_path
        self.output_path = output_path
        self.frame_bytes = frame_bytes
        self.queue_frames = queue_frames
        self._queue: queue.Queue[bytes | None] = queue.Queue(queue_frames)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer_done = threading.Event()
        self._failure: BaseException | None = None
        self._failure_lock = threading.Lock()
        self._started = False
        self.frames = 0
        self.bytes = 0
        self.maximum_queue_depth_frames = 0
        self.direct_io = bool(
            getattr(os, "O_DIRECT", 0) and self.frame_bytes % mmap.PAGESIZE == 0
        )

    def _set_failure(self, exc: BaseException) -> None:
        with self._failure_lock:
            if self._failure is None:
                self._failure = exc

    def _enqueue(self, frame: bytes | None) -> None:
        while not self._writer_done.is_set():
            try:
                self._queue.put(frame, timeout=0.1)
                if frame is not None:
                    self.maximum_queue_depth_frames = max(
                        self.maximum_queue_depth_frames, self._queue.qsize()
                    )
                return
            except queue.Full:
                continue
        if frame is not None:
            raise CaptureError("IR spool writer stopped before FIFO reader")

    def _read_loop(self) -> None:
        try:
            with self.fifo_path.open("rb", buffering=0) as source:
                while True:
                    frame = bytearray()
                    while len(frame) < self.frame_bytes:
                        block = source.read(self.frame_bytes - len(frame))
                        if not block:
                            break
                        frame.extend(block)
                    if not frame:
                        break
                    if len(frame) != self.frame_bytes:
                        raise CaptureError(
                            f"IR FIFO ended with a partial frame: "
                            f"bytes={len(frame)}, expected={self.frame_bytes}"
                        )
                    self._enqueue(bytes(frame))
        except BaseException as exc:
            self._set_failure(exc)
        finally:
            try:
                self._enqueue(None)
            except BaseException as exc:
                self._set_failure(exc)

    def _write_loop(self) -> None:
        descriptor: int | None = None
        aligned: mmap.mmap | None = None
        try:
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
            if self.direct_io:
                flags |= os.O_DIRECT
            descriptor = os.open(self.output_path, flags, 0o644)
            if self.direct_io:
                aligned = mmap.mmap(-1, self.frame_bytes)
            while True:
                frame = self._queue.get()
                try:
                    if frame is None:
                        break
                    if aligned is not None:
                        aligned[:] = frame
                        full_view = memoryview(aligned)
                        frame_write_started = time.monotonic()
                        offset = 0
                        try:
                            while offset < len(full_view):
                                end = min(offset + DIRECT_IO_CHUNK_BYTES, len(full_view))
                                view = full_view[offset:end]
                                try:
                                    while view:
                                        written = os.write(descriptor, view)
                                        if not written:
                                            raise OSError("IR spool write made no progress")
                                        offset += written
                                        view = view[written:]
                                finally:
                                    view.release()
                                target = frame_write_started + (
                                    offset / DIRECT_IO_RATE_BYTES_PER_SECOND
                                )
                                delay = target - time.monotonic()
                                if delay > 0:
                                    time.sleep(delay)
                        finally:
                            full_view.release()
                    else:
                        view = memoryview(frame)
                        try:
                            while view:
                                written = os.write(descriptor, view)
                                if not written:
                                    raise OSError("IR spool write made no progress")
                                view = view[written:]
                        finally:
                            view.release()
                    self.frames += 1
                    self.bytes += len(frame)
                finally:
                    self._queue.task_done()
            os.fsync(descriptor)
        except BaseException as exc:
            self._set_failure(exc)
        finally:
            if aligned is not None:
                aligned.close()
            if descriptor is not None:
                os.close(descriptor)
            self._writer_done.set()

    def start(self) -> None:
        if self._started:
            raise RuntimeError("IR frame spooler is already started")
        os.mkfifo(self.fifo_path, mode=0o600)
        self._started = True
        self._writer.start()
        self._reader.start()

    def finish(self) -> None:
        if not self._started:
            return
        self._reader.join(timeout=10.0)
        if self._reader.is_alive():
            try:
                descriptor = os.open(self.fifo_path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:
                pass
            else:
                os.close(descriptor)
            self._reader.join(timeout=2.0)
        self._writer.join(timeout=30.0)
        if self.fifo_path.exists():
            self.fifo_path.unlink()
        if self._reader.is_alive() or self._writer.is_alive():
            raise CaptureError("IR frame spooler did not stop")
        if self._failure is not None:
            raise CaptureError(f"IR frame spooler failed: {self._failure}")

    def metrics(self) -> dict[str, int]:
        return {
            "frames": self.frames,
            "bytes": self.bytes,
            "queue_capacity_frames": self.queue_frames,
            "maximum_queue_depth_frames": self.maximum_queue_depth_frames,
            "direct_io": self.direct_io,
            "direct_io_rate_bytes_per_second": (
                DIRECT_IO_RATE_BYTES_PER_SECOND if self.direct_io else None
            ),
        }


class _SerialCollector:
    def __init__(self, port: str, payload_path: Path, index_path: Path) -> None:
        self.port = port
        self.payload_path = payload_path
        self.index_path = index_path
        self.ready = threading.Event()
        self.recording = threading.Event()
        self.stop_requested = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.failure: BaseException | None = None
        self.decoder = StreamDecoder("stm32_combined_v1")
        self.count = 0
        self.first_rx_ns: int | None = None
        self.last_rx_ns: int | None = None
        self.last_sequence: int | None = None
        self.sequence_gaps = 0
        self.sequence_regressions = 0
        self.invalid_imu_flags = 0
        self.invalid_encoder_flags = 0

    @staticmethod
    def _configure(fd: int) -> None:
        attributes = termios.tcgetattr(fd)
        attributes[0] = termios.IGNPAR
        attributes[1] = 0
        attributes[2] = termios.B921600 | termios.CS8 | termios.CREAD | termios.CLOCAL
        attributes[3] = 0
        attributes[4] = termios.B921600
        attributes[5] = termios.B921600
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        termios.tcflush(fd, termios.TCIFLUSH)

    def start(self) -> None:
        self.thread.start()
        if not self.ready.wait(3.0):
            raise CaptureError("STM32 serial collector did not become ready")
        if self.failure is not None:
            raise CaptureError(f"STM32 serial setup failed: {self.failure}")

    def begin(self) -> None:
        self.recording.set()

    def stop(self) -> None:
        self.stop_requested.set()
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            raise CaptureError("STM32 serial collector did not stop")
        if self.failure is not None:
            raise CaptureError(f"STM32 serial collector failed: {self.failure}")

    def _loop(self) -> None:
        descriptor = None
        try:
            descriptor = os.open(
                self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK
            )
            self._configure(descriptor)
            warmup_deadline = time.monotonic() + 0.75
            while time.monotonic() < warmup_deadline:
                readable, _, _ = select.select([descriptor], [], [], 0.05)
                if readable:
                    try:
                        os.read(descriptor, 4096)
                    except BlockingIOError:
                        pass
            termios.tcflush(descriptor, termios.TCIFLUSH)
            self.decoder = StreamDecoder("stm32_combined_v1")
            self.ready.set()
            offset = 0
            with self.payload_path.open("xb") as payload, self.index_path.open(
                "x", encoding="utf-8"
            ) as index:
                while not self.stop_requested.is_set():
                    readable, _, _ = select.select([descriptor], [], [], 0.05)
                    if not readable:
                        continue
                    try:
                        data = os.read(descriptor, 4096)
                    except BlockingIOError:
                        continue
                    rx_ns = time.monotonic_ns()
                    for packet in self.decoder.feed(data):
                        if not self.recording.is_set():
                            continue
                        sequence = int(packet.sequence)
                        if self.last_sequence is not None:
                            delta = (sequence - self.last_sequence) & 0xFFFFFFFF
                            if delta == 0 or delta > 0x7FFFFFFF:
                                self.sequence_regressions += 1
                            elif delta != 1:
                                self.sequence_gaps += delta - 1
                        self.last_sequence = sequence
                        if not (packet.flags & 0x01):
                            self.invalid_imu_flags += 1
                        if not (packet.flags & 0x02):
                            self.invalid_encoder_flags += 1
                        payload.write(packet.raw_packet)
                        row = {
                            "record_index": self.count,
                            "offset": offset,
                            "size": STM32_PACKET_BYTES,
                            "host_read_complete_monotonic_ns": rx_ns,
                            "sequence": sequence,
                            "flags": packet.flags,
                            "imu_time_us": packet.imu_first_byte_rx_us,
                            "encoder_time_us": packet.encoder_read_us,
                            "imu_counter": packet.counter,
                            "encoder_response": packet.encoder_response,
                        }
                        index.write(_json_bytes(row).decode("utf-8"))
                        offset += STM32_PACKET_BYTES
                        self.count += 1
                        if self.first_rx_ns is None:
                            self.first_rx_ns = rx_ns
                        self.last_rx_ns = rx_ns
                payload.flush()
                index.flush()
                os.fsync(payload.fileno())
                os.fsync(index.fileno())
        except BaseException as exc:
            self.failure = exc
            self.ready.set()
        finally:
            if descriptor is not None:
                os.close(descriptor)

    def metrics(self) -> dict[str, Any]:
        span_ns = (
            self.last_rx_ns - self.first_rx_ns
            if self.first_rx_ns is not None
            and self.last_rx_ns is not None
            and self.last_rx_ns > self.first_rx_ns
            else 0
        )
        rate = (self.count - 1) * 1_000_000_000 / span_ns if span_ns else 0.0
        return {
            "packets": self.count,
            "observed_rate_hz": rate,
            "first_host_ns": self.first_rx_ns,
            "last_host_ns": self.last_rx_ns,
            "crc_errors": self.decoder.crc_or_checksum_errors,
            "discarded_bytes": self.decoder.discarded_bytes,
            "sequence_gaps": self.sequence_gaps,
            "sequence_regressions": self.sequence_regressions,
            "invalid_imu_flags": self.invalid_imu_flags,
            "invalid_encoder_flags": self.invalid_encoder_flags,
        }


def _kernel_usb_events(since: str) -> list[str]:
    result = _run(
        ["journalctl", "-k", "--since", since, "--no-pager", "-o", "short-iso-precise"],
        check=False,
    )
    return [
        line
        for line in result.stdout.splitlines()
        if re.search(r"usb|uvc|xhci", line, re.IGNORECASE)
    ]


def _thermal_snapshot() -> dict[str, Any]:
    zones = []
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            zones.append(
                {
                    "zone": path.name,
                    "type": (path / "type").read_text().strip(),
                    "temp_millidegree_c": int((path / "temp").read_text().strip()),
                }
            )
        except (OSError, ValueError):
            continue
    maximum = max((zone["temp_millidegree_c"] for zone in zones), default=None)
    return {"zones": zones, "maximum_millidegree_c": maximum}


def _file_manifest(root: Path) -> dict[str, dict[str, Any]]:
    files = {}
    for path in sorted(root.iterdir()):
        if not path.is_file() or path.name in {".recording", "manifest.json"}:
            continue
        files[path.name] = {"size": path.stat().st_size, "sha256": _sha256(path)}
    return files


def capture_session(
    *,
    output_root: Path,
    duration_s: int,
    d405_serial: str,
    stm32_port: str | None = None,
    preview_listen: str | None = None,
    preview_port: int = 8765,
) -> Path:
    if duration_s < 1 or duration_s > 3600:
        raise CaptureError("duration must be between 1 and 3600 seconds")
    for executable in ("udevadm", "v4l2-ctl", "gst-launch-1.0"):
        if shutil.which(executable) is None:
            raise CaptureError(f"required executable is unavailable: {executable}")

    usb = verify_d405_superspeed(d405_serial)
    ir_node, rgb_node = discover_d405_nodes(d405_serial)
    serial_path = discover_stm32_port(stm32_port)
    owners = _device_owners([ir_node, rgb_node, serial_path])
    if owners:
        raise CaptureError(f"a target device is already open: {owners}")

    frames = duration_s * FPS
    rgb_frames = duration_s * RGB_FPS
    estimated = IR_FRAME_BYTES * frames + (RGB_BITRATE // 8) * duration_s * 2
    output_root = output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(output_root).free
    if free < estimated + MIN_FREE_RESERVE_BYTES:
        raise CaptureError(
            f"insufficient free space: free={free}, estimated={estimated}, "
            f"reserve={MIN_FREE_RESERVE_BYTES}"
        )
    thermal_before = _thermal_snapshot()
    if (
        thermal_before["maximum_millidegree_c"] is not None
        and thermal_before["maximum_millidegree_c"] >= 85_000
    ):
        raise CaptureError(f"host is too hot to start capture: {thermal_before}")

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_id = f"rk3576-{stamp}-{uuid.uuid4().hex[:8]}"
    partial = output_root / f".{session_id}.partial"
    final = output_root / session_id
    partial.mkdir(mode=0o700)
    (partial / ".recording").write_text("unsealed\n", encoding="ascii")

    config = {
        "schema": SCHEMA,
        "session_id": session_id,
        "capture_host": {
            "hostname": os.uname().nodename,
            "machine": os.uname().machine,
            "thermal_before": thermal_before,
        },
        "device": {
            "d405_serial": d405_serial,
            "d405_usb": usb,
            "ir_node_at_capture": ir_node,
            "rgb_node_at_capture": rgb_node,
            "stm32_port": serial_path,
        },
        "profile": {
            "infrared": {
                "encoding": "y8i_raw",
                "layout": "byte_interleaved_left_right",
                "width": WIDTH,
                "height": HEIGHT,
                "fps": FPS,
                "frame_bytes": IR_FRAME_BYTES,
                "semantics": "lossless_stereo_sensor_sample",
            },
            "rgb": {
                "encoding": "h265_annex_b",
                "width": WIDTH,
                "height": HEIGHT,
                "fps": RGB_FPS,
                "target_bitrate": RGB_BITRATE,
                "semantics": "lossy_preview_and_training_image",
            },
            "stm32": {
                "encoding": "stm32_combined_v1_raw",
                "packet_bytes": STM32_PACKET_BYTES,
                "baud": STM32_BAUD,
                "nominal_rate_hz": STM32_NOMINAL_HZ,
                "semantics": "lossless_wire_packet",
            },
        },
        "requested": {"duration_s": duration_s, "frames": frames},
    }
    if preview_listen is not None:
        config["preview"] = {
            "transport": "http_mjpeg",
            "listen": preview_listen,
            "port": preview_port,
            "width": 640,
            "height": 360,
            "fps": 10,
        }
    _write_atomic(partial / "session_config.json", config)

    ir_rows: list[dict[str, Any]] = []
    rgb_rows: list[dict[str, Any]] = []
    ir_index = (partial / "ir_frames.jsonl").open("x", encoding="utf-8")
    rgb_index = (partial / "rgb_frames.jsonl").open("x", encoding="utf-8")

    def on_ir(line: str, observed_ns: int) -> None:
        row = parse_v4l2_frame_line(line)
        if row is None or row["bytes_used"] == 0 or "error" in row["flags"]:
            return
        row["record_index"] = len(ir_rows)
        row["observed_host_monotonic_ns"] = observed_ns
        ir_rows.append(row)
        ir_index.write(_json_bytes(row).decode("utf-8"))

    def on_rgb(line: str, observed_ns: int) -> None:
        row = parse_gstreamer_rgb_line(line)
        if row is None:
            return
        row["record_index"] = len(rgb_rows)
        row["observed_host_monotonic_ns"] = observed_ns
        rgb_rows.append(row)
        rgb_index.write(_json_bytes(row).decode("utf-8"))

    ir_partial = partial / "infrared-y8i.raw.partial"
    ir_fifo = partial / ".infrared-y8i.pipe"
    rgb_partial = partial / "rgb.h265.partial"
    preview_fifo = partial / ".preview.mjpeg.pipe"
    preview: MjpegPreviewServer | None = None
    preview_enabled = preview_listen is not None
    serial = _SerialCollector(
        serial_path,
        partial / "stm32.bin.partial",
        partial / "stm32_packets.jsonl",
    )
    ir_spooler = _FrameSpooler(
        ir_fifo,
        ir_partial,
        frame_bytes=IR_FRAME_BYTES,
    )
    ir_command = [
        "v4l2-ctl",
        "-d",
        ir_node,
        f"--set-fmt-video=width={WIDTH},height={HEIGHT},pixelformat={Y8I_FOURCC}",
        f"--set-parm={FPS}",
        f"--stream-mmap={IR_V4L2_BUFFERS}",
        f"--stream-count={frames}",
        f"--stream-to={ir_fifo}",
        "--stream-poll",
        "--verbose",
    ]
    rgb_command = [
        "gst-launch-1.0",
        "-v",
        "-e",
        "v4l2src",
        f"device={rgb_node}",
        # The D405/uvcvideo path emits two flagged startup buffers on this
        # RK3576 kernel. Allow a bounded reserve, drop buffers explicitly
        # marked CORRUPTED, then end after exactly ``frames`` good buffers.
        f"num-buffers={rgb_frames + 8}",
        "io-mode=mmap",
        "do-timestamp=false",
        "!",
        f"video/x-raw,format=YUY2,width={WIDTH},height={HEIGHT},framerate={RGB_FPS}/1",
        "!",
        "queue",
        f"max-size-buffers={CAPTURE_QUEUE_FRAMES}",
        "max-size-bytes=0",
        "max-size-time=0",
        "!",
        "identity",
        "name=dropper",
        "silent=true",
        "drop-buffer-flags=corrupted",
        "!",
        "identity",
        "name=limiter",
        "silent=true",
        f"eos-after={rgb_frames + 1}",
        "!",
        "tee",
        "name=rgbtee",
        "rgbtee.",
        "!",
        "queue",
        "max-size-buffers=6",
        "!",
        "identity",
        "name=rgbraw",
        "silent=false",
        "!",
        "videoconvert",
        "!",
        "video/x-raw,format=NV12",
        "!",
        "mpph265enc",
        f"bps={RGB_BITRATE}",
        f"gop={RGB_FPS}",
        "header-mode=each-idr",
        "!",
        "filesink",
        f"location={rgb_partial}",
    ]
    if preview_enabled:
        rgb_command.extend(
            [
                "rgbtee.",
                "!",
                "queue",
                "leaky=downstream",
                "max-size-buffers=1",
                "!",
                "videorate",
                "drop-only=true",
                "!",
                "video/x-raw,framerate=10/1",
                "!",
                "videoscale",
                "!",
                "video/x-raw,width=640,height=360",
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=I420",
                "!",
                "jpegenc",
                "quality=75",
                "!",
                "filesink",
                "sync=false",
                f"location={preview_fifo}",
            ]
        )
    ir_process = _LoggedProcess(ir_command, partial / "ir-v4l2.log", on_ir)
    rgb_process = _LoggedProcess(rgb_command, partial / "rgb-gstreamer.log", on_rgb)
    capture_since = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
    anchors = {
        "realtime_ns_before_start": time.time_ns(),
        "monotonic_ns_before_start": time.monotonic_ns(),
    }

    try:
        if preview_listen is not None:
            os.mkfifo(preview_fifo, mode=0o600)
            preview = MjpegPreviewServer(preview_fifo, preview_listen, preview_port)
            preview.start()
        serial.start()
        serial.begin()
        ir_spooler.start()
        ir_process.start()
        rgb_process.start()
        deadline = time.monotonic() + duration_s + 20.0
        for process in (ir_process, rgb_process):
            process.thread.join(max(0.0, deadline - time.monotonic()))
        if ir_process.thread.is_alive() or rgb_process.thread.is_alive():
            ir_process.terminate()
            rgb_process.terminate()
            raise CaptureError("camera capture exceeded its bounded deadline")
        ir_spooler.finish()
        serial.stop()
        if preview is not None:
            preview.stop()
            if preview_fifo.exists():
                preview_fifo.unlink()
        ir_index.flush()
        rgb_index.flush()
        os.fsync(ir_index.fileno())
        os.fsync(rgb_index.fileno())
        ir_index.close()
        rgb_index.close()

        if ir_process.returncode != 0 or rgb_process.returncode != 0:
            raise CaptureError(
                f"camera process failed: ir={ir_process.returncode}, rgb={rgb_process.returncode}"
            )
        ir_spool_metrics = ir_spooler.metrics()
        if (
            len(ir_rows) != frames
            or ir_spool_metrics["frames"] != frames
            or ir_partial.stat().st_size != frames * IR_FRAME_BYTES
        ):
            raise CaptureError(
                f"IR count/size mismatch: rows={len(ir_rows)}, "
                f"spooled={ir_spool_metrics['frames']}, "
                f"bytes={ir_partial.stat().st_size}, expected_frames={frames}"
            )
        ir_gaps = sum(
            max(0, current["sequence"] - previous["sequence"] - 1)
            for previous, current in zip(ir_rows, ir_rows[1:])
        )
        ir_regressions = sum(
            current["sequence"] <= previous["sequence"]
            for previous, current in zip(ir_rows, ir_rows[1:])
        )
        if ir_gaps or ir_regressions:
            raise CaptureError(
                f"IR sequence discontinuity: gaps={ir_gaps}, regressions={ir_regressions}"
            )
        if len(rgb_rows) != rgb_frames or rgb_partial.stat().st_size == 0:
            raise CaptureError(
                f"RGB count/payload mismatch: rows={len(rgb_rows)}, "
                f"bytes={rgb_partial.stat().st_size}, expected_frames={rgb_frames}"
            )
        rgb_pts_regressions = sum(
            current["pipeline_pts_ns"] <= previous["pipeline_pts_ns"]
            for previous, current in zip(rgb_rows, rgb_rows[1:])
        )
        if rgb_pts_regressions:
            raise CaptureError(f"RGB PTS regressions: {rgb_pts_regressions}")
        with rgb_partial.open("rb") as stream:
            prefix = stream.read(4)
        if prefix not in {b"\x00\x00\x00\x01", b"\x00\x00\x01\x40"} and not prefix.startswith(
            b"\x00\x00\x01"
        ):
            raise CaptureError("RGB output is not H.265 Annex-B")
        serial_metrics = serial.metrics()
        if not 395.0 <= serial_metrics["observed_rate_hz"] <= 405.0:
            raise CaptureError(f"STM32 rate is outside acceptance band: {serial_metrics}")
        for key in (
            "crc_errors",
            "discarded_bytes",
            "sequence_gaps",
            "sequence_regressions",
            "invalid_imu_flags",
            "invalid_encoder_flags",
        ):
            if serial_metrics[key]:
                raise CaptureError(f"STM32 integrity failure: {serial_metrics}")

        for source, target in (
            (ir_partial, partial / "infrared-y8i.raw"),
            (rgb_partial, partial / "rgb.h265"),
            (partial / "stm32.bin.partial", partial / "stm32.bin"),
        ):
            os.replace(source, target)
        kernel_events = _kernel_usb_events(capture_since)
        (partial / "kernel-usb.log").write_text(
            "\n".join(kernel_events) + ("\n" if kernel_events else ""), encoding="utf-8"
        )
        eproto = [line for line in kernel_events if "uvcvideo" in line and "(-71)" in line]
        fatal_usb_patterns = (
            "usb disconnect",
            "reset superspeed usb device",
            "device descriptor read",
            "unable to submit",
            "failed to submit",
        )
        fatal_usb_events = [
            line
            for line in kernel_events
            if any(pattern in line.lower() for pattern in fatal_usb_patterns)
        ]
        if fatal_usb_events:
            raise CaptureError(f"fatal USB kernel events: {fatal_usb_events}")
        thermal_after = _thermal_snapshot()
        if (
            thermal_after["maximum_millidegree_c"] is not None
            and thermal_after["maximum_millidegree_c"] >= 85_000
        ):
            raise CaptureError(f"host exceeded the thermal limit: {thermal_after}")
        warnings = []
        if eproto:
            warnings.append(f"uvc_startup_eproto_events={len(eproto)}")
        metrics = {
            "ir": {
                "frames": len(ir_rows),
                "sequence_gaps": ir_gaps,
                "sequence_regressions": ir_regressions,
                "first_source_monotonic_ns": ir_rows[0]["source_monotonic_ns"],
                "last_source_monotonic_ns": ir_rows[-1]["source_monotonic_ns"],
                "spooler": ir_spool_metrics,
            },
            "rgb": {
                "input_frames": len(rgb_rows),
                "encoded_bytes": (partial / "rgb.h265").stat().st_size,
                "pts_regressions": rgb_pts_regressions,
                "timestamp_semantics": "pipeline_relative_preview_only",
            },
            "stm32": serial_metrics,
            "anchors": anchors,
            "thermal_before": thermal_before,
            "thermal_after": thermal_after,
            "kernel_uvc_eproto_events": len(eproto),
        }
        if preview is not None:
            preview_metrics = preview.metrics()
            if preview_metrics["reader_failure"] is not None:
                raise CaptureError(f"preview reader failed: {preview_metrics}")
            if preview_metrics["published_frames"] == 0:
                raise CaptureError("preview produced no frames")
            metrics["preview"] = preview_metrics
        manifest = {
            "schema": SCHEMA,
            "status": "SEALED_WITH_WARNINGS" if warnings else "SEALED",
            "session_id": session_id,
            "warnings": warnings,
            "profile": config["profile"],
            "device": config["device"],
            "counts": {
                "ir_frames": len(ir_rows),
                "rgb_input_frames": len(rgb_rows),
                "stm32_packets": serial.count,
            },
            "metrics": metrics,
            "files": _file_manifest(partial),
        }
        _write_atomic(partial / "manifest.json", manifest)
        (partial / ".recording").unlink()
        _fsync_directory(partial)
        os.replace(partial, final)
        _fsync_directory(output_root)
        return final
    except BaseException as exc:
        ir_process.stop()
        rgb_process.stop()
        try:
            ir_spooler.finish()
        except BaseException:
            pass
        serial.stop_requested.set()
        serial.thread.join(timeout=2.0)
        if preview is not None:
            preview.stop()
            try:
                preview_fifo.unlink(missing_ok=True)
            except OSError:
                # A failed removable target can be remounted read-only while
                # unwinding. Preserve the original capture error in that case.
                pass
        for stream in (ir_index, rgb_index):
            if not stream.closed:
                stream.close()
        try:
            _write_atomic(
                partial / "failure.json",
                {
                    "schema": SCHEMA,
                    "status": "FAILED",
                    "reason": str(exc),
                    "diagnostics": {
                        "ir_rows": len(ir_rows),
                        "rgb_rows": len(rgb_rows),
                        "ir_spooler": ir_spooler.metrics(),
                    },
                },
            )
        except OSError:
            pass
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=Path("/home/pi/rk3576-umi-sessions"))
    parser.add_argument("--duration", type=int, default=10)
    parser.add_argument("--d405-serial", required=True)
    parser.add_argument("--stm32-port")
    parser.add_argument("--preview-listen")
    parser.add_argument("--preview-port", type=int, default=8765)
    args = parser.parse_args(argv)
    try:
        session = capture_session(
            output_root=args.output_root,
            duration_s=args.duration,
            d405_serial=args.d405_serial,
            stm32_port=args.stm32_port,
            preview_listen=args.preview_listen,
            preview_port=args.preview_port,
        )
    except CaptureError as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    status = json.loads((session / "manifest.json").read_text(encoding="utf-8"))[
        "status"
    ]
    print(json.dumps({"status": status, "session": str(session)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
