from __future__ import annotations

import ctypes
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional


class VendorXuLinkError(RuntimeError):
    pass


class DeviceUnavailableError(RuntimeError):
    pass


class CaptureRuntimeError(RuntimeError):
    pass


@dataclass(frozen=True)
class TwoUQ2Packet:
    sequence: int
    main_accel_g: tuple[float, float, float]
    main_gyro_dps: tuple[float, float, float]
    backup_accel_g: tuple[float, float, float]
    backup_gyro_dps: tuple[float, float, float]
    raw: bytes

    @classmethod
    def parse(cls, raw: bytes | bytearray | memoryview) -> "TwoUQ2Packet":
        if not isinstance(raw, (bytes, bytearray, memoryview)):
            raise TypeError("packet must be bytes-like")
        raw_bytes = bytes(raw)
        if len(raw_bytes) != 27:
            raise ValueError("2UQ2 packet must be exactly 27 bytes")

        sequence = (raw_bytes[0] << 16) | (raw_bytes[1] << 8) | raw_bytes[2]
        main = struct.unpack(">6h", raw_bytes[3:15])
        backup = struct.unpack(">6h", raw_bytes[15:27])
        return cls(
            sequence=sequence,
            main_accel_g=tuple(value / 4096.0 for value in main[:3]),
            main_gyro_dps=tuple(value / 16.4 for value in main[3:]),
            backup_accel_g=tuple(value / 4096.0 for value in backup[:3]),
            backup_gyro_dps=tuple(value / 16.4 for value in backup[3:]),
            raw=raw_bytes,
        )


@dataclass(frozen=True)
class TwoUQ2Frame:
    sequence: Optional[int]
    acquisition_ns: Optional[int]
    arrival_ns: int
    jpeg: bytes
    xu_raw: bytes
    packet: Optional[TwoUQ2Packet]
    valid: bool = True
    error_reason: str = ""
    width: Optional[int] = None
    height: Optional[int] = None


def build_pipeline_description(device: str) -> str:
    if not isinstance(device, str) or not device or any(
        character in device for character in ("\x00", "\n", "\r")
    ):
        raise ValueError("device must be a non-empty single-line path")
    escaped_device = device.replace("\\", "\\\\").replace('"', '\\"')
    return (
        f'v4l2src device="{escaped_device}" do-timestamp=true ! '
        "image/jpeg,width=3840,height=1080,framerate=60/1 ! "
        "appsink name=sink emit-signals=true sync=false max-buffers=8 drop=false"
    )


def is_structurally_valid_jpeg(jpeg: bytes, *, max_segments: int = 1024) -> bool:
    if len(jpeg) < 4 or jpeg[:2] != b"\xff\xd8":
        return False
    position = 2
    segments = 0
    saw_start_of_frame = False
    while position < len(jpeg):
        if jpeg[position] != 0xFF:
            return False
        while position < len(jpeg) and jpeg[position] == 0xFF:
            position += 1
        if position >= len(jpeg):
            return False
        marker = jpeg[position]
        position += 1
        if marker in (0x00, 0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            return False
        if position + 2 > len(jpeg):
            return False
        segment_length = int.from_bytes(jpeg[position : position + 2], "big")
        if segment_length < 2 or position + segment_length > len(jpeg):
            return False
        segments += 1
        if segments > max_segments:
            return False
        if marker in {
            0xC0,
            0xC1,
            0xC2,
            0xC3,
            0xC5,
            0xC6,
            0xC7,
            0xC9,
            0xCA,
            0xCB,
            0xCD,
            0xCE,
            0xCF,
        }:
            if segment_length < 8:
                return False
            saw_start_of_frame = True
        if marker != 0xDA:
            position += segment_length
            continue
        if not saw_start_of_frame or segment_length < 6:
            return False
        position += segment_length
        saw_scan_data = False
        while position < len(jpeg):
            if jpeg[position] != 0xFF:
                saw_scan_data = True
                position += 1
                continue
            if position + 1 >= len(jpeg):
                return False
            following = jpeg[position + 1]
            if following == 0x00:
                saw_scan_data = True
                position += 2
                continue
            if 0xD0 <= following <= 0xD7:
                position += 2
                continue
            return following == 0xD9 and saw_scan_data and position + 2 == len(jpeg)
        return False
    return False


class _CtypesXuBridge:
    def __init__(self, library_path: str):
        self._library_path = library_path
        self._library = None

    def _load(self):
        if self._library is None:
            try:
                library = ctypes.CDLL(self._library_path)
            except OSError as exc:
                raise VendorXuLinkError("vendor_xu_link") from exc
            try:
                library.ylx_open.argtypes = [ctypes.c_char_p]
                library.ylx_open.restype = ctypes.c_int
                library.ylx_read_imu27.argtypes = [
                    ctypes.c_int,
                    ctypes.POINTER(ctypes.c_uint8),
                ]
                library.ylx_read_imu27.restype = ctypes.c_int
                library.ylx_close.argtypes = [ctypes.c_int]
                library.ylx_close.restype = ctypes.c_int
            except (AttributeError, TypeError) as exc:
                raise VendorXuLinkError("vendor_xu_link") from exc
            self._library = library
        return self._library

    def open(self, device: str) -> int:
        descriptor = self._load().ylx_open(device.encode())
        if descriptor < 0:
            raise DeviceUnavailableError("2uq2_hardware_unavailable")
        return descriptor

    def read_imu27(self, descriptor: int) -> bytes:
        output = (ctypes.c_uint8 * 27)()
        rc = self._load().ylx_read_imu27(descriptor, output)
        if rc != 0:
            raise OSError("ylx_read_imu27_failed")
        return bytes(output)

    def close(self, descriptor: int) -> None:
        rc = self._load().ylx_close(descriptor)
        if rc != 0:
            raise OSError("ylx_close_failed")


def _load_gst():
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
    except (ImportError, ValueError) as exc:
        raise CaptureRuntimeError("gstreamer_unavailable") from exc
    return Gst


class TwoUQ2Capture:
    def __init__(
        self,
        device: str,
        xu_library: str,
        on_frame: Callable[[TwoUQ2Frame], None],
        *,
        _gst=None,
        _xu=None,
        _monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ):
        self.device = device
        self.xu_library = xu_library
        self.on_frame = on_frame
        self._gst = _gst
        self._xu = _xu
        self._monotonic_ns = _monotonic_ns
        self._pipeline = None
        self._bus = None
        self._sink = None
        self._sink_handler = None
        self._xu_fd = None
        self._lock = threading.Lock()
        self._last_sequence = None
        self._last_acquisition_ns = None
        self._first_arrival_ns = None
        self._last_arrival_ns = None
        self._terminal_error = None
        self._stats = {
            "frames": 0,
            "valid_sequences": 0,
            "xu_failures": 0,
            "duplicate_sequences": 0,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
            "timestamp_regressions": 0,
            "timestamp_unavailable": 0,
            "bad_jpegs": 0,
            "measured_width": None,
            "measured_height": None,
            "resolution_mismatches": 0,
            "callback_exceptions": 0,
            "capture_errors": 0,
        }

    def start(self) -> None:
        if self._pipeline is not None:
            raise RuntimeError("capture already started")

        gst = self._gst if self._gst is not None else _load_gst()
        xu = self._xu if self._xu is not None else _CtypesXuBridge(self.xu_library)
        gst.init(None)
        pipeline = gst.parse_launch(build_pipeline_description(self.device))
        sink = pipeline.get_by_name("sink")
        if sink is None:
            raise RuntimeError("2UQ2 pipeline has no appsink")

        descriptor = None
        handler = None
        try:
            descriptor = xu.open(self.device)
            self._gst = gst
            self._xu = xu
            self._pipeline = pipeline
            self._bus = pipeline.get_bus()
            self._sink = sink
            self._xu_fd = descriptor
            handler = sink.connect("new-sample", self._on_new_sample)
            self._sink_handler = handler
            result = pipeline.set_state(gst.State.PLAYING)
            if result == gst.StateChangeReturn.FAILURE:
                raise DeviceUnavailableError("2uq2_hardware_unavailable")
        except Exception:
            try:
                pipeline.set_state(gst.State.NULL)
            except Exception:
                pass
            try:
                if handler is not None:
                    sink.disconnect(handler)
            finally:
                try:
                    if descriptor is not None:
                        xu.close(descriptor)
                finally:
                    self._pipeline = None
                    self._bus = None
                    self._sink = None
                    self._sink_handler = None
                    self._xu_fd = None
            raise

    def stop(self) -> None:
        pipeline = self._pipeline
        if pipeline is None:
            return
        try:
            pipeline.set_state(self._gst.State.NULL)
        finally:
            try:
                if self._sink is not None and self._sink_handler is not None:
                    self._sink.disconnect(self._sink_handler)
            finally:
                try:
                    if self._xu is not None and self._xu_fd is not None:
                        self._xu.close(self._xu_fd)
                finally:
                    self._pipeline = None
                    self._bus = None
                    self._sink = None
                    self._sink_handler = None
                    self._xu_fd = None

    def poll_terminal(self) -> Optional[str]:
        with self._lock:
            if self._terminal_error is not None:
                return self._terminal_error
            bus = self._bus
        if bus is None:
            return None
        message = bus.timed_pop_filtered(
            0, self._gst.MessageType.ERROR | self._gst.MessageType.EOS
        )
        if message is None:
            return None
        reason = (
            "gstreamer_error"
            if message.type == self._gst.MessageType.ERROR
            else "gstreamer_eos"
        )
        with self._lock:
            if self._terminal_error is None:
                self._terminal_error = reason
                self._stats["capture_errors"] += 1
            reason = self._terminal_error
        self.stop()
        return reason

    def stats(self) -> dict:
        with self._lock:
            result = dict(self._stats)
            first_arrival = self._first_arrival_ns
            last_arrival = self._last_arrival_ns
        elapsed_ns = (
            last_arrival - first_arrival
            if first_arrival is not None and last_arrival is not None
            else 0
        )
        result["rate_hz"] = (
            (result["frames"] - 1) * 1_000_000_000 / elapsed_ns
            if result["frames"] > 1 and elapsed_ns > 0
            else 0.0
        )
        result["gap_ratio"] = (
            result["sequence_gaps"] / result["valid_sequences"]
            if result["valid_sequences"]
            else 0.0
        )
        return result

    def _on_new_sample(self, sink):
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
            width = int(structure.get_value("width")) if structure is not None else None
            height = int(structure.get_value("height")) if structure is not None else None

            arrival_ns = self._monotonic_ns()
            base_time = int(self._pipeline.get_base_time())
            pts = int(buffer.pts)
            errors = []
            if base_time == self._gst.CLOCK_TIME_NONE or pts == self._gst.CLOCK_TIME_NONE:
                acquisition_ns = None
                errors.append("timestamp_unavailable")
                with self._lock:
                    self._stats["timestamp_unavailable"] += 1
            else:
                acquisition_ns = base_time + pts
            packet = None
            xu_raw = b""
            try:
                candidate = self._xu.read_imu27(self._xu_fd)
                packet = TwoUQ2Packet.parse(candidate)
                xu_raw = packet.raw
            except Exception:
                errors.append("xu_read_failed")
                with self._lock:
                    self._stats["xu_failures"] += 1

            if (width, height) != (3840, 1080):
                errors.append("resolution_mismatch")
                with self._lock:
                    self._stats["resolution_mismatches"] += 1

            if not is_structurally_valid_jpeg(jpeg):
                errors.append("bad_jpeg")
                with self._lock:
                    self._stats["bad_jpegs"] += 1

            self._observe_frame(packet, acquisition_ns, arrival_ns)
            with self._lock:
                self._stats["measured_width"] = width
                self._stats["measured_height"] = height
            frame = TwoUQ2Frame(
                sequence=packet.sequence if packet is not None else None,
                acquisition_ns=acquisition_ns,
                arrival_ns=arrival_ns,
                jpeg=jpeg,
                xu_raw=xu_raw,
                packet=packet,
                valid=not errors,
                error_reason=",".join(errors),
                width=width,
                height=height,
            )
            try:
                self.on_frame(frame)
            except Exception:
                with self._lock:
                    self._stats["callback_exceptions"] += 1
            return self._gst.FlowReturn.OK
        except Exception:
            with self._lock:
                self._stats["capture_errors"] += 1
            return self._gst.FlowReturn.ERROR

    def _observe_frame(
        self,
        packet: Optional[TwoUQ2Packet],
        acquisition_ns: Optional[int],
        arrival_ns: int,
    ) -> None:
        with self._lock:
            self._stats["frames"] += 1
            if self._first_arrival_ns is None:
                self._first_arrival_ns = arrival_ns
            self._last_arrival_ns = arrival_ns
            if acquisition_ns is not None:
                if (
                    self._last_acquisition_ns is not None
                    and acquisition_ns < self._last_acquisition_ns
                ):
                    self._stats["timestamp_regressions"] += 1
                self._last_acquisition_ns = acquisition_ns

            if packet is None:
                return
            self._stats["valid_sequences"] += 1
            if self._last_sequence is not None:
                delta = (packet.sequence - self._last_sequence) & 0xFFFFFF
                if delta == 0:
                    self._stats["duplicate_sequences"] += 1
                elif delta < 0x800000:
                    if delta > 1:
                        self._stats["sequence_gaps"] += delta - 1
                else:
                    self._stats["sequence_regressions"] += 1
            self._last_sequence = packet.sequence
