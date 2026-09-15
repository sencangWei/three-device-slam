"""Crash-detectable storage contract for the customized Z2 collector.

Hardware capture is deliberately kept outside this module.  The same contract can
be driven by a Python prototype now and by the final C/C++ Z2 service later.
"""

from __future__ import annotations

import hashlib
import json
import os
import zlib
from pathlib import Path
from typing import Any

from three_device_slam.devices.d405_umi.imu.stream_protocol import parse_combined


SCHEMA = "three-device-slam.z2-edge-session.v1"
RGB_WIDTH = 1280
RGB_HEIGHT = 720
RGB_FPS = 30
IR_WIDTH = 1280
IR_HEIGHT = 720
IR_FPS = 30
IR_FRAME_BYTES = IR_WIDTH * IR_HEIGHT
STM32_PACKET_BYTES = 63
STM32_RATE_HZ = 400
DEFAULT_MAX_CHUNK_BYTES = 1 << 30


class EdgeSessionError(RuntimeError):
    """The edge session cannot accept or seal the requested data."""


def _require_int(name: str, value: int, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise EdgeSessionError(f"{name} must be an integer >= {minimum}")
    return value


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode(
        "utf-8"
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_atomic(path: Path, payload: bytes) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    _fsync_directory(path.parent)


class _ChunkedPayloadWriter:
    def __init__(self, root: Path, stem: str, max_chunk_bytes: int) -> None:
        self._root = root
        self._stem = stem
        self._max_chunk_bytes = max_chunk_bytes
        self._stream = None
        self._relative_path: str | None = None
        self._size = 0
        self._chunk_number = -1

    def _rotate(self) -> None:
        if self._stream is not None:
            self._stream.flush()
            os.fsync(self._stream.fileno())
            self._stream.close()
        self._chunk_number += 1
        self._relative_path = f"data/{self._stem}-{self._chunk_number:06d}.bin"
        self._stream = (self._root / self._relative_path).open("xb")
        self._size = 0

    def append(self, payload: bytes) -> dict[str, Any]:
        if not payload:
            raise EdgeSessionError(f"{self._stem} payload must not be empty")
        if len(payload) > self._max_chunk_bytes:
            raise EdgeSessionError(
                f"{self._stem} payload ({len(payload)} bytes) exceeds chunk limit "
                f"({self._max_chunk_bytes} bytes)"
            )
        if self._stream is None or self._size + len(payload) > self._max_chunk_bytes:
            self._rotate()
        assert self._stream is not None
        assert self._relative_path is not None
        offset = self._size
        self._stream.write(payload)
        self._size += len(payload)
        return {
            "file": self._relative_path,
            "offset": offset,
            "size": len(payload),
            "crc32": f"{zlib.crc32(payload) & 0xFFFFFFFF:08x}",
        }

    def close(self) -> None:
        if self._stream is None:
            return
        self._stream.flush()
        os.fsync(self._stream.fileno())
        self._stream.close()
        self._stream = None


class Z2EdgeSessionWriter:
    """Append RGB, synchronized dual-IR and STM32 records into one sealed session."""

    def __init__(
        self,
        root: str | Path,
        *,
        session_id: str,
        board_id: str,
        d405_serial: str,
        calibration_id: str,
        max_chunk_bytes: int = DEFAULT_MAX_CHUNK_BYTES,
    ) -> None:
        for name, value in (
            ("session_id", session_id),
            ("board_id", board_id),
            ("d405_serial", d405_serial),
            ("calibration_id", calibration_id),
        ):
            if not isinstance(value, str) or not value.strip():
                raise EdgeSessionError(f"{name} must be a non-empty string")
        max_chunk_bytes = _require_int(
            "max_chunk_bytes", max_chunk_bytes, minimum=IR_FRAME_BYTES
        )

        self.root = Path(root)
        if self.root.exists():
            raise EdgeSessionError(f"session directory already exists: {self.root}")
        self.root.mkdir(parents=True)
        (self.root / "data").mkdir()
        (self.root / ".recording").write_text("unsealed\n", encoding="ascii")

        self._state = "active"
        self._counts = {"ir_framesets": 0, "rgb_access_units": 0, "stm32_packets": 0}
        self._last: dict[str, int | None] = {
            "ir_sequence": None,
            "ir_host_ns": None,
            "rgb_sequence": None,
            "rgb_host_ns": None,
            "rgb_pts_ns": None,
            "stm32_host_ns": None,
        }
        self._config = {
            "schema": SCHEMA,
            "session_id": session_id,
            "device": {
                "board_id": board_id,
                "d405_serial": d405_serial,
                "calibration_id": calibration_id,
            },
            "profile": {
                "rgb": {
                    "width": RGB_WIDTH,
                    "height": RGB_HEIGHT,
                    "fps": RGB_FPS,
                    "encoding": "h265_annex_b",
                    "semantics": "lossy_preview_and_training_image",
                },
                "infrared_left": {
                    "width": IR_WIDTH,
                    "height": IR_HEIGHT,
                    "fps": IR_FPS,
                    "encoding": "y8",
                    "semantics": "lossless_sensor_sample",
                },
                "infrared_right": {
                    "width": IR_WIDTH,
                    "height": IR_HEIGHT,
                    "fps": IR_FPS,
                    "encoding": "y8",
                    "semantics": "lossless_sensor_sample",
                },
                "stm32": {
                    "packet_bytes": STM32_PACKET_BYTES,
                    "nominal_rate_hz": STM32_RATE_HZ,
                    "encoding": "umi_combined_v1_raw",
                    "semantics": "lossless_wire_packet",
                },
            },
            "max_chunk_bytes": max_chunk_bytes,
        }
        _write_atomic(self.root / "session_config.json", _json_bytes(self._config))

        self._ir_left = _ChunkedPayloadWriter(self.root, "infrared-left-y8", max_chunk_bytes)
        self._ir_right = _ChunkedPayloadWriter(self.root, "infrared-right-y8", max_chunk_bytes)
        self._rgb = _ChunkedPayloadWriter(self.root, "rgb-h265", max_chunk_bytes)
        self._stm32 = _ChunkedPayloadWriter(self.root, "stm32-combined", max_chunk_bytes)
        self._ir_index = (self.root / "infrared_framesets.jsonl").open("xb")
        self._rgb_index = (self.root / "rgb_access_units.jsonl").open("xb")
        self._stm32_index = (self.root / "stm32_packets.jsonl").open("xb")

    def _require_active(self) -> None:
        if self._state != "active":
            raise EdgeSessionError(f"session is not writable (state={self._state})")

    def _require_increasing(self, key: str, value: int) -> None:
        previous = self._last[key]
        if previous is not None and value <= previous:
            raise EdgeSessionError(f"{key} must increase: previous={previous}, new={value}")

    @staticmethod
    def _write_index(stream: Any, row: dict[str, Any]) -> None:
        stream.write(_json_bytes(row))

    def append_ir_pair(
        self,
        *,
        frameset_sequence: int,
        host_monotonic_ns: int,
        left_frame_number: int,
        left_device_timestamp_ns: int,
        left_y8: bytes,
        right_frame_number: int,
        right_device_timestamp_ns: int,
        right_y8: bytes,
    ) -> None:
        self._require_active()
        values = {
            "frameset_sequence": frameset_sequence,
            "host_monotonic_ns": host_monotonic_ns,
            "left_frame_number": left_frame_number,
            "left_device_timestamp_ns": left_device_timestamp_ns,
            "right_frame_number": right_frame_number,
            "right_device_timestamp_ns": right_device_timestamp_ns,
        }
        for name, value in values.items():
            _require_int(name, value)
        if len(left_y8) != IR_FRAME_BYTES or len(right_y8) != IR_FRAME_BYTES:
            raise EdgeSessionError(
                f"each Y8 frame must contain exactly {IR_FRAME_BYTES} bytes"
            )
        if abs(left_device_timestamp_ns - right_device_timestamp_ns) > 2_000_000:
            raise EdgeSessionError("left/right D405 timestamps differ by more than 2 ms")
        self._require_increasing("ir_sequence", frameset_sequence)
        self._require_increasing("ir_host_ns", host_monotonic_ns)

        try:
            left_ref = self._ir_left.append(left_y8)
            right_ref = self._ir_right.append(right_y8)
            self._write_index(
                self._ir_index,
                {
                    "frameset_sequence": frameset_sequence,
                    "host_monotonic_ns": host_monotonic_ns,
                    "left": {
                        "frame_number": left_frame_number,
                        "device_timestamp_ns": left_device_timestamp_ns,
                        "payload": left_ref,
                    },
                    "right": {
                        "frame_number": right_frame_number,
                        "device_timestamp_ns": right_device_timestamp_ns,
                        "payload": right_ref,
                    },
                },
            )
        except BaseException:
            self._state = "failed"
            raise
        self._last["ir_sequence"] = frameset_sequence
        self._last["ir_host_ns"] = host_monotonic_ns
        self._counts["ir_framesets"] += 1

    def append_rgb_access_unit(
        self,
        *,
        frameset_sequence: int,
        frame_number: int,
        device_timestamp_ns: int,
        host_monotonic_ns: int,
        encoder_pts_ns: int,
        keyframe: bool,
        h265_annex_b: bytes,
    ) -> None:
        self._require_active()
        for name, value in (
            ("frameset_sequence", frameset_sequence),
            ("frame_number", frame_number),
            ("device_timestamp_ns", device_timestamp_ns),
            ("host_monotonic_ns", host_monotonic_ns),
            ("encoder_pts_ns", encoder_pts_ns),
        ):
            _require_int(name, value)
        if not isinstance(keyframe, bool):
            raise EdgeSessionError("keyframe must be boolean")
        if not (h265_annex_b.startswith(b"\x00\x00\x01") or h265_annex_b.startswith(b"\x00\x00\x00\x01")):
            raise EdgeSessionError("RGB access unit must use H.265 Annex-B start codes")
        self._require_increasing("rgb_sequence", frameset_sequence)
        self._require_increasing("rgb_host_ns", host_monotonic_ns)
        self._require_increasing("rgb_pts_ns", encoder_pts_ns)

        try:
            payload_ref = self._rgb.append(h265_annex_b)
            self._write_index(
                self._rgb_index,
                {
                    "frameset_sequence": frameset_sequence,
                    "frame_number": frame_number,
                    "device_timestamp_ns": device_timestamp_ns,
                    "host_monotonic_ns": host_monotonic_ns,
                    "encoder_pts_ns": encoder_pts_ns,
                    "keyframe": keyframe,
                    "payload": payload_ref,
                },
            )
        except BaseException:
            self._state = "failed"
            raise
        self._last["rgb_sequence"] = frameset_sequence
        self._last["rgb_host_ns"] = host_monotonic_ns
        self._last["rgb_pts_ns"] = encoder_pts_ns
        self._counts["rgb_access_units"] += 1

    def append_stm32_packet(self, *, host_monotonic_ns: int, packet: bytes) -> None:
        self._require_active()
        _require_int("host_monotonic_ns", host_monotonic_ns)
        if len(packet) != STM32_PACKET_BYTES:
            raise EdgeSessionError(f"STM32 packet must contain {STM32_PACKET_BYTES} bytes")
        try:
            parsed = parse_combined(packet)
        except (TypeError, ValueError) as exc:
            raise EdgeSessionError(f"invalid STM32 combined packet: {exc}") from exc
        self._require_increasing("stm32_host_ns", host_monotonic_ns)

        try:
            payload_ref = self._stm32.append(packet)
            self._write_index(
                self._stm32_index,
                {
                    "host_monotonic_ns": host_monotonic_ns,
                    "sequence": parsed.sequence,
                    "imu_time_us": parsed.imu_first_byte_rx_us,
                    "encoder_time_us": parsed.encoder_read_us,
                    "encoder_counter": parsed.counter,
                    "flags": parsed.flags,
                    "payload": payload_ref,
                },
            )
        except BaseException:
            self._state = "failed"
            raise
        self._last["stm32_host_ns"] = host_monotonic_ns
        self._counts["stm32_packets"] += 1

    def _close_payloads_and_indexes(self) -> None:
        for payload in (self._ir_left, self._ir_right, self._rgb, self._stm32):
            payload.close()
        for index in (self._ir_index, self._rgb_index, self._stm32_index):
            if index.closed:
                continue
            index.flush()
            os.fsync(index.fileno())
            index.close()

    def abort(self) -> None:
        """Close files but intentionally leave the `.recording` marker behind."""
        if self._state == "sealed":
            return
        self._close_payloads_and_indexes()
        self._state = "aborted"

    def seal(self) -> Path:
        self._require_active()
        missing = [name for name, count in self._counts.items() if count == 0]
        if missing:
            raise EdgeSessionError(f"cannot seal an incomplete session; no records for {missing}")
        try:
            self._close_payloads_and_indexes()
            files: dict[str, dict[str, Any]] = {}
            candidates = [
                self.root / "session_config.json",
                self.root / "infrared_framesets.jsonl",
                self.root / "rgb_access_units.jsonl",
                self.root / "stm32_packets.jsonl",
                *sorted((self.root / "data").glob("*.bin")),
            ]
            for path in candidates:
                relative = path.relative_to(self.root).as_posix()
                files[relative] = {"size": path.stat().st_size, "sha256": _sha256(path)}
            manifest = {
                "schema": SCHEMA,
                "status": "SEALED",
                "session_id": self._config["session_id"],
                "profile": self._config["profile"],
                "device": self._config["device"],
                "counts": dict(self._counts),
                "files": files,
            }
            manifest_path = self.root / "manifest.json"
            _write_atomic(manifest_path, _json_bytes(manifest))
            (self.root / ".recording").unlink()
            _fsync_directory(self.root)
        except BaseException:
            self._state = "failed"
            raise
        self._state = "sealed"
        return manifest_path
