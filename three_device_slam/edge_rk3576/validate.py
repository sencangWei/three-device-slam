"""Validate a sealed RK3576 D405 + STM32 acquisition session."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
from pathlib import Path
from typing import Any, Iterable

from three_device_slam.devices.d405_umi.imu.stream_protocol import parse_combined

from .capture import (
    FPS,
    HEIGHT,
    IR_FRAME_BYTES,
    MIN_FREE_RESERVE_BYTES,
    RGB_FPS,
    SCHEMA,
    STM32_PACKET_BYTES,
    WIDTH,
)
from .rsusb_capture import (
    IR_H265_BITRATE_MAX_PER_EYE,
    IR_H265_BITRATE_MIN_PER_EYE,
    IR_H265_BITRATE_PER_EYE,
    IR_H265_GOP_FRAMES,
    STORAGE_STOP_HEADROOM_BYTES,
)


RSUSB_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v1"
RSUSB_ZSTD_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v2"
RSUSB_SPLIT_ZSTD_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v3"
RSUSB_SPLIT_H265_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v4"
SUPPORTED_SCHEMAS = {
    SCHEMA,
    RSUSB_SCHEMA,
    RSUSB_ZSTD_SCHEMA,
    RSUSB_SPLIT_ZSTD_SCHEMA,
    RSUSB_SPLIT_H265_SCHEMA,
}


class SessionValidationError(RuntimeError):
    """A session is incomplete, corrupt, or incompatible."""


def _fail(message: str) -> None:
    raise SessionValidationError(message)


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        _fail(f"{name} must be an object")
    return value


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        _fail(f"{name} must be a positive integer")
    return value


def _nonnegative_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        _fail(f"{name} must be a non-negative integer")
    return value


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        _fail(f"{name} must be a finite number")
    return result


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        _fail(f"cannot read {path.name}: {exc}")
    if not isinstance(value, dict):
        _fail(f"{path.name} must contain an object")
    return value


def _read_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            for number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    _fail(f"{path.name}:{number} is invalid JSON: {exc}")
                if not isinstance(row, dict):
                    _fail(f"{path.name}:{number} must contain an object")
                yield row
    except OSError as exc:
        _fail(f"cannot read {path.name}: {exc}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _zstd_decompressed_size(path: Path) -> int:
    if shutil.which("zstd") is None:
        _fail("zstd is required to validate compressed lossless IR")
    process = subprocess.Popen(
        ["zstd", "--decompress", "--quiet", "--stdout", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if process.stdout is None:
        _fail("zstd validator stdout is unavailable")
    total = 0
    for block in iter(lambda: process.stdout.read(1024 * 1024), b""):
        total += len(block)
    process.stdout.close()
    return_code = process.wait()
    stderr = (
        process.stderr.read().decode("utf-8", errors="replace")
        if process.stderr is not None
        else ""
    )
    if process.stderr is not None:
        process.stderr.close()
    if return_code != 0:
        _fail(f"IR zstd stream is invalid: {stderr.strip()}")
    return total


def _validate_files(root: Path, manifest: dict[str, Any]) -> None:
    claims = manifest.get("files")
    if not isinstance(claims, dict) or not claims:
        _fail("manifest.files must be a non-empty object")
    expected = set(claims) | {"manifest.json"}
    actual = {path.name for path in root.iterdir() if path.is_file() or path.is_symlink()}
    if actual != expected:
        _fail(
            f"file set mismatch: missing={sorted(expected-actual)}, "
            f"extra={sorted(actual-expected)}"
        )
    for name, claim in claims.items():
        path = root / name
        if path.is_symlink() or not path.is_file() or not isinstance(claim, dict):
            _fail(f"invalid session file: {name}")
        if path.stat().st_size != claim.get("size"):
            _fail(f"size mismatch: {name}")
        if _sha256(path) != claim.get("sha256"):
            _fail(f"SHA-256 mismatch: {name}")


def _decode_video_frames(path: Path, label: str) -> int:
    try:
        import cv2
    except ImportError as exc:
        _fail(f"OpenCV is required to decode {label}: {exc}")
    capture = cv2.VideoCapture(str(path))
    count = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame.shape[:2] != (HEIGHT, WIDTH):
            capture.release()
            _fail(f"decoded {label} shape mismatch: {frame.shape}")
        count += 1
    capture.release()
    return count


def validate_rk3576_session(
    root: str | Path,
    *,
    decode_rgb: bool = False,
    decode_ir: bool = False,
) -> dict[str, Any]:
    root = Path(root)
    if not root.is_dir():
        _fail(f"session directory does not exist: {root}")
    if (root / ".recording").exists() or root.name.endswith(".partial"):
        _fail("session is not sealed")
    manifest = _read_json(root / "manifest.json")
    if manifest.get("schema") not in SUPPORTED_SCHEMAS or manifest.get("status") not in {
        "SEALED",
        "SEALED_WITH_WARNINGS",
    }:
        _fail("manifest schema/status mismatch")
    _validate_files(root, manifest)
    config = _read_json(root / "session_config.json")
    if config.get("schema") != manifest.get("schema"):
        _fail("session_config schema mismatch")
    if config.get("profile") != manifest.get("profile") or config.get(
        "device"
    ) != manifest.get("device"):
        _fail("manifest/config identity or profile mismatch")
    profile = _object(manifest.get("profile"), "manifest.profile")
    ir_profile = _object(profile.get("infrared"), "manifest.profile.infrared")
    ir_encoding = ir_profile.get("encoding")
    if ir_encoding == "y8i_zstd" and manifest.get("schema") != RSUSB_ZSTD_SCHEMA:
        _fail("compressed Y8I requires the RSUSB v2 schema")
    if (
        ir_encoding == "y8_split_zstd"
        and manifest.get("schema") != RSUSB_SPLIT_ZSTD_SCHEMA
    ):
        _fail("split compressed Y8 requires the RSUSB v3 schema")
    if (
        ir_encoding == "y8_split_h265"
        and manifest.get("schema") != RSUSB_SPLIT_H265_SCHEMA
    ):
        _fail("split H.265 Y8 requires the RSUSB v4 schema")
    if manifest.get("schema") == RSUSB_ZSTD_SCHEMA and ir_encoding != "y8i_zstd":
        _fail("RSUSB v2 requires compressed Y8I")
    if (
        manifest.get("schema") == RSUSB_SPLIT_ZSTD_SCHEMA
        and ir_encoding != "y8_split_zstd"
    ):
        _fail("RSUSB v3 requires split compressed Y8")
    if (
        manifest.get("schema") == RSUSB_SPLIT_H265_SCHEMA
        and ir_encoding != "y8_split_h265"
    ):
        _fail("RSUSB v4 requires split H.265 Y8")
    if (
        ir_encoding,
        ir_profile.get("width"),
        ir_profile.get("height"),
        ir_profile.get("fps"),
        ir_profile.get("frame_bytes"),
    ) not in {
        ("y8i_raw", WIDTH, HEIGHT, FPS, IR_FRAME_BYTES),
        ("y8i_zstd", WIDTH, HEIGHT, FPS, IR_FRAME_BYTES),
        ("y8_split_zstd", WIDTH, HEIGHT, FPS, IR_FRAME_BYTES),
        ("y8_split_h265", WIDTH, HEIGHT, FPS, IR_FRAME_BYTES),
    }:
        _fail("locked Y8I profile mismatch")
    if ir_encoding == "y8_split_zstd" and (
        ir_profile.get("layout") != "separate_left_right_files"
        or ir_profile.get("frame_bytes_per_eye") != WIDTH * HEIGHT
        or ir_profile.get("semantics") != "lossless_stereo_sensor_sample"
        or ir_profile.get("compression")
        != {
            "codec": "zstd",
            "level": 1,
            "threads_per_stream": 1,
            "content": "concatenated_complete_y8_frames_per_eye",
        }
        or ir_profile.get("files")
        != {
            "left": "infrared-left-y8.raw.zst",
            "right": "infrared-right-y8.raw.zst",
        }
    ):
        _fail("locked split Y8 profile mismatch")
    if ir_encoding == "y8_split_h265" and (
        ir_profile.get("layout") != "separate_left_right_files"
        or ir_profile.get("frame_bytes_per_eye") != WIDTH * HEIGHT
        or ir_profile.get("semantics") != "lossy_stereo_slam_candidate"
        or ir_profile.get("compression")
        != {
            "codec": "h265",
            "container": "annex_b",
            "encoder": "rockchip_mpp",
            "rate_control": "cbr",
            "target_bitrate_per_eye": IR_H265_BITRATE_PER_EYE,
            "minimum_bitrate_per_eye": IR_H265_BITRATE_MIN_PER_EYE,
            "maximum_bitrate_per_eye": IR_H265_BITRATE_MAX_PER_EYE,
            "gop_frames": IR_H265_GOP_FRAMES,
            "pixel_format": "yuv420p_nv12",
            "source_encoding": "y8",
            "content": "one_access_unit_per_complete_y8_frame_per_eye",
        }
        or ir_profile.get("files")
        != {
            "left": "infrared-left-y8.h265",
            "right": "infrared-right-y8.h265",
        }
    ):
        _fail("locked split H.265 Y8 profile mismatch")
    rgb_profile = _object(profile.get("rgb"), "manifest.profile.rgb")
    if (
        rgb_profile.get("encoding"),
        rgb_profile.get("width"),
        rgb_profile.get("height"),
        rgb_profile.get("fps"),
    ) != ("h265_annex_b", WIDTH, HEIGHT, RGB_FPS):
        _fail("locked RGB profile mismatch")
    stm32_profile = _object(profile.get("stm32"), "manifest.profile.stm32")
    if (stm32_profile.get("encoding"), stm32_profile.get("packet_bytes")) != (
        "stm32_combined_v1_raw",
        STM32_PACKET_BYTES,
    ):
        _fail("locked STM32 profile mismatch")

    ir_rows = list(_read_jsonl(root / "ir_frames.jsonl"))
    previous_sequence: int | None = None
    previous_source_ns: int | None = None
    previous_left_ms: float | None = None
    previous_right_ms: float | None = None
    for expected, row in enumerate(ir_rows):
        if row.get("record_index") != expected or row.get("bytes_used") != IR_FRAME_BYTES:
            _fail(f"IR index mismatch at row {expected}")
        sequence = _nonnegative_int(
            row.get("sequence"), f"ir_frames.jsonl:{expected + 1}.sequence"
        )
        source_ns = _nonnegative_int(
            row.get("source_monotonic_ns"),
            f"ir_frames.jsonl:{expected + 1}.source_monotonic_ns",
        )
        if previous_sequence is not None and sequence != previous_sequence + 1:
            _fail(f"IR sequence gap at row {expected}")
        if previous_source_ns is not None and source_ns <= previous_source_ns:
            _fail(f"IR timestamp regression at row {expected}")
        if ir_encoding in {"y8_split_zstd", "y8_split_h265"}:
            if row.get("left_sequence") != sequence or row.get("right_sequence") != sequence:
                _fail(f"split IR eye sequence mismatch at row {expected}")
            left_ms = _finite_number(
                row.get("left_source_timestamp_ms"),
                f"ir_frames.jsonl:{expected + 1}.left_source_timestamp_ms",
            )
            right_ms = _finite_number(
                row.get("right_source_timestamp_ms"),
                f"ir_frames.jsonl:{expected + 1}.right_source_timestamp_ms",
            )
            if source_ns != round(left_ms * 1_000_000):
                _fail(f"split IR source timestamp mismatch at row {expected}")
            if (
                previous_left_ms is not None
                and previous_right_ms is not None
                and (left_ms <= previous_left_ms or right_ms <= previous_right_ms)
            ):
                _fail(f"split IR eye timestamp regression at row {expected}")
            previous_left_ms = left_ms
            previous_right_ms = right_ms
        previous_sequence = sequence
        previous_source_ns = source_ns
    if ir_encoding == "y8_split_zstd":
        left_bytes = _zstd_decompressed_size(root / "infrared-left-y8.raw.zst")
        right_bytes = _zstd_decompressed_size(root / "infrared-right-y8.raw.zst")
        expected_eye_bytes = len(ir_rows) * WIDTH * HEIGHT
        if left_bytes != expected_eye_bytes or right_bytes != expected_eye_bytes:
            _fail("split IR payload length does not match its index")
        ir_payload_bytes = left_bytes + right_bytes
    elif ir_encoding == "y8_split_h265":
        for side in ("left", "right"):
            path = root / f"infrared-{side}-y8.h265"
            if path.stat().st_size == 0:
                _fail(f"split IR {side} H.265 payload is empty")
            with path.open("rb") as stream:
                prefix = stream.read(4)
            if not (
                prefix.startswith(b"\x00\x00\x01")
                or prefix == b"\x00\x00\x00\x01"
            ):
                _fail(f"split IR {side} payload is not Annex-B")
        ir_payload_bytes = len(ir_rows) * IR_FRAME_BYTES
    else:
        ir_payload = root / (
            "infrared-y8i.raw.zst"
            if ir_encoding == "y8i_zstd"
            else "infrared-y8i.raw"
        )
        if ir_encoding == "y8i_zstd":
            ir_payload_bytes = _zstd_decompressed_size(ir_payload)
        else:
            ir_payload_bytes = ir_payload.stat().st_size
    if ir_payload_bytes != len(ir_rows) * IR_FRAME_BYTES:
        _fail("IR payload length does not match its index")

    rgb_rows = list(_read_jsonl(root / "rgb_frames.jsonl"))
    previous_pts: int | None = None
    for expected, row in enumerate(rgb_rows):
        if row.get("record_index") != expected or row.get("bytes_used") != WIDTH * HEIGHT * 2:
            _fail(f"RGB input index mismatch at row {expected}")
        pts = _nonnegative_int(
            row.get("pipeline_pts_ns"),
            f"rgb_frames.jsonl:{expected + 1}.pipeline_pts_ns",
        )
        if previous_pts is not None and pts <= previous_pts:
            _fail(f"RGB PTS regression at row {expected}")
        previous_pts = pts
    with (root / "rgb.h265").open("rb") as stream:
        prefix = stream.read(4)
    if not (prefix.startswith(b"\x00\x00\x01") or prefix == b"\x00\x00\x00\x01"):
        _fail("RGB payload is not Annex-B")

    stm32_rows = list(_read_jsonl(root / "stm32_packets.jsonl"))
    with (root / "stm32.bin").open("rb") as payload:
        previous_sequence = None
        for expected, row in enumerate(stm32_rows):
            packet = payload.read(STM32_PACKET_BYTES)
            if len(packet) != STM32_PACKET_BYTES:
                _fail(f"STM32 payload is truncated at row {expected}")
            try:
                parsed = parse_combined(packet)
            except ValueError as exc:
                _fail(f"STM32 packet validation failed at row {expected}: {exc}")
            if row.get("record_index") != expected or row.get(
                "offset"
            ) != expected * STM32_PACKET_BYTES:
                _fail(f"STM32 index mismatch at row {expected}")
            if row.get("sequence") != parsed.sequence or row.get("flags") != parsed.flags:
                _fail(f"STM32 parsed metadata mismatch at row {expected}")
            if previous_sequence is not None and (
                (parsed.sequence - previous_sequence) & 0xFFFFFFFF
            ) != 1:
                _fail(f"STM32 sequence discontinuity at row {expected}")
            previous_sequence = parsed.sequence
        if payload.read(1):
            _fail("STM32 payload contains unindexed trailing bytes")

    counts = {
        "ir_frames": len(ir_rows),
        "rgb_input_frames": len(rgb_rows),
        "stm32_packets": len(stm32_rows),
    }
    if counts != manifest.get("counts") or any(value == 0 for value in counts.values()):
        _fail(f"manifest count mismatch: {counts}")
    metrics = _object(manifest.get("metrics"), "manifest.metrics")
    ir_metrics = _object(metrics.get("ir"), "manifest.metrics.ir")
    rgb_metrics = _object(metrics.get("rgb"), "manifest.metrics.rgb")
    stm32_metrics = _object(metrics.get("stm32"), "manifest.metrics.stm32")
    if ir_metrics.get("frames") != counts["ir_frames"]:
        _fail("IR metric count mismatch")
    if rgb_metrics.get("input_frames") != counts["rgb_input_frames"]:
        _fail("RGB metric count mismatch")
    if rgb_metrics.get("pts_regressions") != 0:
        _fail("RGB metrics report a PTS regression")
    if stm32_metrics.get("packets") != counts["stm32_packets"]:
        _fail("STM32 metric count mismatch")
    stm32_rate = _finite_number(
        stm32_metrics.get("observed_rate_hz"),
        "manifest.metrics.stm32.observed_rate_hz",
    )
    if not 395.0 <= stm32_rate <= 405.0:
        _fail("STM32 metrics report an invalid rate")
    for key in (
        "crc_errors",
        "discarded_bytes",
        "sequence_gaps",
        "sequence_regressions",
        "invalid_imu_flags",
        "invalid_encoder_flags",
    ):
        if stm32_metrics.get(key) != 0:
            _fail(f"STM32 metrics report {key}")
    if manifest.get("schema") in {
        RSUSB_SPLIT_ZSTD_SCHEMA,
        RSUSB_SPLIT_H265_SCHEMA,
    }:
        requested = _object(config.get("requested"), "session_config.requested")
        request_mode = requested.get("mode", "fixed_frames")
        if request_mode == "fixed_frames":
            requested_frames = _positive_int(
                requested.get("frames"), "session_config.requested.frames"
            )
            requested_duration = _finite_number(
                requested.get("duration_s"), "session_config.requested.duration_s"
            )
            if requested_duration <= 0:
                _fail("session_config.requested.duration_s must be positive")
            if not math.isclose(
                requested_duration * FPS,
                requested_frames,
                rel_tol=0.0,
                abs_tol=1e-9,
            ):
                _fail("requested duration/frame relationship is invalid")
            if requested_frames != counts["ir_frames"] or requested_frames != counts[
                "rgb_input_frames"
            ]:
                _fail("requested frame count does not match captured RGB/IR frames")
        elif request_mode == "until_signal":
            maximum_duration = _positive_int(
                requested.get("maximum_duration_s"),
                "session_config.requested.maximum_duration_s",
            )
            if maximum_duration > 3600:
                _fail("session_config.requested.maximum_duration_s exceeds the limit")
            termination = _object(
                metrics.get("termination"), "manifest.metrics.termination"
            )
            reason = termination.get("reason")
            if (
                counts["rgb_input_frames"] != counts["ir_frames"]
                or
                termination.get("mode") != "until_signal"
                or reason
                not in {
                    "external_stop",
                    "maximum_duration_complete",
                    "low_storage_guard",
                }
                or termination.get("captured_frames") != counts["ir_frames"]
            ):
                _fail("signal-controlled termination evidence mismatch")
            formal_span = _finite_number(
                metrics.get("formal_host_span_s"),
                "manifest.metrics.formal_host_span_s",
            )
            if formal_span <= 0 or formal_span > maximum_duration + 5:
                _fail("signal-controlled capture span is outside its bound")
            formal_start_ns = _positive_int(
                metrics.get("formal_start_monotonic_ns"),
                "manifest.metrics.formal_start_monotonic_ns",
            )
            formal_stop_ns = _positive_int(
                metrics.get("formal_stop_monotonic_ns"),
                "manifest.metrics.formal_stop_monotonic_ns",
            )
            if formal_stop_ns <= formal_start_ns or not math.isclose(
                (formal_stop_ns - formal_start_ns) / 1e9,
                formal_span,
                rel_tol=0.0,
                abs_tol=1e-6,
            ):
                _fail("signal-controlled formal capture window is inconsistent")
            nominal_span = (counts["ir_frames"] - 1) / FPS
            if not math.isclose(
                formal_span,
                nominal_span,
                rel_tol=0.0,
                abs_tol=0.25,
            ):
                _fail("signal-controlled capture span does not match frame count")
            maximum_frames = maximum_duration * FPS
            if counts["ir_frames"] > maximum_frames:
                _fail("signal-controlled capture exceeds its frame bound")
            stop_observed = termination.get("stop_observed_monotonic_ns")
            storage_guard = termination.get("storage_guard")
            if reason == "maximum_duration_complete":
                if (
                    counts["ir_frames"] != maximum_frames
                    or stop_observed is not None
                    or storage_guard is not None
                ):
                    _fail("maximum-duration termination evidence mismatch")
            elif reason == "external_stop":
                stop_ns = _positive_int(
                    stop_observed,
                    "manifest.metrics.termination.stop_observed_monotonic_ns",
                )
                if not formal_start_ns <= stop_ns <= formal_stop_ns:
                    _fail("external-stop timestamp is outside the capture window")
                if storage_guard is not None:
                    _fail("external-stop termination has storage-guard evidence")
            else:
                guard = _object(
                    storage_guard,
                    "manifest.metrics.termination.storage_guard",
                )
                stop_ns = _positive_int(
                    stop_observed,
                    "manifest.metrics.termination.stop_observed_monotonic_ns",
                )
                observed_free = _nonnegative_int(
                    guard.get("observed_free_bytes"),
                    "manifest.metrics.termination.storage_guard.observed_free_bytes",
                )
                threshold = MIN_FREE_RESERVE_BYTES + STORAGE_STOP_HEADROOM_BYTES
                if (
                    guard.get("threshold_bytes") != threshold
                    or guard.get("observed_monotonic_ns") != stop_ns
                    or observed_free > threshold
                ):
                    _fail("low-storage termination evidence mismatch")
                if not formal_start_ns <= stop_ns <= formal_stop_ns:
                    _fail("low-storage stop timestamp is outside the capture window")
        else:
            _fail("session_config.requested.mode is unsupported")

        stream_reports = _object(
            metrics.get("rsusb_streams"), "manifest.metrics.rsusb_streams"
        )
        expected_stream_counts = {
            "color": counts["rgb_input_frames"],
            "infrared_left": counts["ir_frames"],
            "infrared_right": counts["ir_frames"],
        }
        if set(stream_reports) != set(expected_stream_counts):
            _fail("RSUSB stream metric set mismatch")
        for stream_name, expected_count in expected_stream_counts.items():
            report = _object(
                stream_reports.get(stream_name),
                f"manifest.metrics.rsusb_streams.{stream_name}",
            )
            if report.get("received") != expected_count:
                _fail(f"RSUSB {stream_name} received count mismatch")
            observed_rate = _finite_number(
                report.get("observed_rate_hz"),
                f"manifest.metrics.rsusb_streams.{stream_name}.observed_rate_hz",
            )
            timestamp_span = _finite_number(
                report.get("timestamp_span_s"),
                f"manifest.metrics.rsusb_streams.{stream_name}.timestamp_span_s",
            )
            maximum_interval = _finite_number(
                report.get("max_arrival_interval_ms"),
                f"manifest.metrics.rsusb_streams.{stream_name}.max_arrival_interval_ms",
            )
            if not 29.0 <= observed_rate <= 31.0:
                _fail(f"RSUSB {stream_name} observed rate is outside the 30 Hz band")
            if timestamp_span <= 0 or maximum_interval < 0 or maximum_interval > 250.0:
                _fail(f"RSUSB {stream_name} timing evidence is invalid")
            expected_timestamp_span = (expected_count - 1) / observed_rate
            if not math.isclose(
                timestamp_span,
                expected_timestamp_span,
                rel_tol=0.02,
                abs_tol=0.005,
            ):
                _fail(f"RSUSB {stream_name} timestamp span is inconsistent")
            for key in (
                "payload_size_errors",
                "repeated_sequences",
                "sequence_gaps",
                "sequence_regressions",
                "timestamp_regressions",
            ):
                if report.get(key) != 0:
                    _fail(f"RSUSB {stream_name} metrics report {key}")

        writer = _object(ir_metrics.get("writer"), "manifest.metrics.ir.writer")
        expected_eye_bytes = counts["ir_frames"] * WIDTH * HEIGHT
        if ir_encoding == "y8_split_zstd":
            left_path = root / "infrared-left-y8.raw.zst"
            right_path = root / "infrared-right-y8.raw.zst"
        else:
            left_path = root / "infrared-left-y8.h265"
            right_path = root / "infrared-right-y8.h265"
        stream_writer_metrics = _object(
            writer.get("streams"), "manifest.metrics.ir.writer.streams"
        )
        if set(stream_writer_metrics) != {"left", "right"}:
            _fail("split IR writer stream set mismatch")
        compressed_total = 0
        for side, path in (("left", left_path), ("right", right_path)):
            report = _object(
                stream_writer_metrics.get(side),
                f"manifest.metrics.ir.writer.streams.{side}",
            )
            compressed_size = path.stat().st_size
            compressed_total += compressed_size
            expected = (
                {
                    "encoding": "y8_zstd",
                    "frames": counts["ir_frames"],
                    "bytes": expected_eye_bytes,
                    "compressed_bytes": compressed_size,
                    "zstd_level": 1,
                    "zstd_threads": 1,
                    "queue_overflows": 0,
                }
                if ir_encoding == "y8_split_zstd"
                else {
                    "encoding": "y8_h265_annex_b",
                    "codec": "h265",
                    "encoder": "rockchip_mpp",
                    "lossless": False,
                    "target_bitrate": IR_H265_BITRATE_PER_EYE,
                    "gop_frames": IR_H265_GOP_FRAMES,
                    "frames": counts["ir_frames"],
                    "bytes": expected_eye_bytes,
                    "compressed_bytes": compressed_size,
                    "queue_overflows": 0,
                }
            )
            for key, value in expected.items():
                if report.get(key) != value:
                    _fail(f"split IR {side} writer metric mismatch: {key}")
        aggregate_expected = {
            "frames": counts["ir_frames"],
            "input_bytes": expected_eye_bytes * 2,
            "compressed_bytes": compressed_total,
            "queue_overflows": 0,
        }
        for key, value in aggregate_expected.items():
            if writer.get(key) != value:
                _fail(f"split IR aggregate writer metric mismatch: {key}")
        ratio = _finite_number(
            writer.get("compression_ratio"),
            "manifest.metrics.ir.writer.compression_ratio",
        )
        if not math.isclose(
            ratio,
            compressed_total / (expected_eye_bytes * 2),
            rel_tol=1e-12,
            abs_tol=0.0,
        ):
            _fail("split IR compression ratio mismatch")
    warnings = manifest.get("warnings")
    if not isinstance(warnings, list):
        _fail("manifest warnings must be a list")
    if bool(warnings) != (manifest.get("status") == "SEALED_WITH_WARNINGS"):
        _fail("manifest warning/status mismatch")
    decoded = (
        _decode_video_frames(root / "rgb.h265", "RGB") if decode_rgb else None
    )
    if decoded is not None and decoded != len(rgb_rows):
        _fail(f"RGB decode count mismatch: decoded={decoded}, indexed={len(rgb_rows)}")
    decoded_ir: dict[str, int] | None = None
    if decode_ir:
        if ir_encoding != "y8_split_h265":
            _fail("--decode-ir requires a split H.265 Y8 session")
        decoded_ir = {
            side: _decode_video_frames(
                root / f"infrared-{side}-y8.h265", f"IR {side}"
            )
            for side in ("left", "right")
        }
        if any(count != len(ir_rows) for count in decoded_ir.values()):
            _fail(
                "IR decode count mismatch: "
                f"decoded={decoded_ir}, indexed={len(ir_rows)}"
            )
    return {
        "status": "PASS_WITH_WARNINGS" if manifest.get("warnings") else "PASS",
        "session_id": manifest.get("session_id"),
        "counts": counts,
        "rgb_decoded_frames": decoded,
        "ir_decoded_frames": decoded_ir,
        "warnings": manifest.get("warnings", []),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--decode-rgb", action="store_true")
    parser.add_argument("--decode-ir", action="store_true")
    args = parser.parse_args(argv)
    try:
        report = validate_rk3576_session(
            args.session,
            decode_rgb=args.decode_rgb,
            decode_ir=args.decode_ir,
        )
    except (SessionValidationError, ValueError) as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
