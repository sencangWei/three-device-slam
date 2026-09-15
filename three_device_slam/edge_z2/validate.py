"""Strict offline validation for a sealed Z2 edge session."""

from __future__ import annotations

import argparse
import hashlib
import json
import zlib
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

from three_device_slam.devices.d405_umi.imu.stream_protocol import parse_combined

from .session import (
    IR_FRAME_BYTES,
    SCHEMA,
    STM32_PACKET_BYTES,
)


class EdgeSessionValidationError(RuntimeError):
    """A sealed edge session is incomplete, corrupt or semantically invalid."""


def _fail(message: str) -> None:
    raise EdgeSessionValidationError(message)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        _fail(f"cannot read JSON {path.name}: {exc}")
    if not isinstance(value, dict):
        _fail(f"{path.name} must contain a JSON object")
    return value


def _read_jsonl(path: Path) -> Iterable[tuple[int, dict[str, Any]]]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            for number, line in enumerate(stream, 1):
                if not line.strip():
                    _fail(f"{path.name}:{number} is blank")
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    _fail(f"{path.name}:{number} is invalid JSON: {exc}")
                if not isinstance(row, dict):
                    _fail(f"{path.name}:{number} must contain an object")
                yield number, row
    except OSError as exc:
        _fail(f"cannot read {path.name}: {exc}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _strict_int(row: dict[str, Any], key: str, context: str) -> int:
    value = row.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        _fail(f"{context}.{key} must be a non-negative integer")
    return value


def _check_increasing(previous: int | None, value: int, context: str) -> int:
    if previous is not None and value <= previous:
        _fail(f"{context} is not strictly increasing: {previous} -> {value}")
    return value


def _safe_file(root: Path, relative: str) -> Path:
    candidate = root / relative
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        _fail(f"missing file {relative}: {exc}")
    if root.resolve() not in resolved.parents:
        _fail(f"file escapes session directory: {relative}")
    if candidate.is_symlink() or not candidate.is_file():
        _fail(f"file must be a regular non-symlink: {relative}")
    return candidate


def validate_z2_edge_session(root: str | Path) -> dict[str, Any]:
    root = Path(root)
    if not root.is_dir():
        _fail(f"session directory does not exist: {root}")
    if (root / ".recording").exists():
        _fail("session is unsealed: .recording marker exists")

    manifest = _read_json(root / "manifest.json")
    if manifest.get("schema") != SCHEMA or manifest.get("status") != "SEALED":
        _fail("manifest schema/status does not describe a sealed Z2 v1 session")
    files = manifest.get("files")
    if not isinstance(files, dict) or not files:
        _fail("manifest.files must be a non-empty object")

    expected_files = set(files) | {"manifest.json"}
    actual_files = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() or path.is_symlink()
    }
    if actual_files != expected_files:
        _fail(
            f"file set differs from manifest; missing={sorted(expected_files - actual_files)}, "
            f"unexpected={sorted(actual_files - expected_files)}"
        )

    resolved_files: dict[str, Path] = {}
    for relative, claim in files.items():
        if not isinstance(relative, str) or not isinstance(claim, dict):
            _fail("manifest.files entries must map paths to objects")
        path = _safe_file(root, relative)
        size = claim.get("size")
        digest = claim.get("sha256")
        if path.stat().st_size != size:
            _fail(f"size mismatch for {relative}")
        if not isinstance(digest, str) or _sha256(path) != digest:
            _fail(f"SHA-256 mismatch for {relative}")
        resolved_files[relative] = path

    config = _read_json(resolved_files.get("session_config.json", root / "missing"))
    if config.get("schema") != SCHEMA:
        _fail("session_config schema mismatch")
    if config.get("profile") != manifest.get("profile"):
        _fail("profile differs between session_config and manifest")
    if config.get("device") != manifest.get("device"):
        _fail("device metadata differs between session_config and manifest")
    profile = config.get("profile", {})
    expected_profile = {
        "rgb": (1280, 720, 30, "h265_annex_b"),
        "infrared_left": (1280, 720, 30, "y8"),
        "infrared_right": (1280, 720, 30, "y8"),
    }
    for stream_name, expected in expected_profile.items():
        stream = profile.get(stream_name, {})
        actual = (stream.get("width"), stream.get("height"), stream.get("fps"), stream.get("encoding"))
        if actual != expected:
            _fail(f"locked profile mismatch for {stream_name}: {actual}")
    stm32_profile = profile.get("stm32", {})
    if (stm32_profile.get("packet_bytes"), stm32_profile.get("nominal_rate_hz")) != (63, 400):
        _fail("locked STM32 profile mismatch")

    coverage: dict[str, list[tuple[int, int]]] = defaultdict(list)

    def validate_payload(ref: Any, context: str, *, exact_size: int | None = None) -> bytes:
        if not isinstance(ref, dict):
            _fail(f"{context}.payload must be an object")
        relative = ref.get("file")
        if not isinstance(relative, str) or not relative.startswith("data/"):
            _fail(f"{context}.payload.file is invalid")
        path = resolved_files.get(relative)
        if path is None:
            _fail(f"{context} references a file absent from the manifest: {relative}")
        offset = _strict_int(ref, "offset", f"{context}.payload")
        size = _strict_int(ref, "size", f"{context}.payload")
        if size == 0 or (exact_size is not None and size != exact_size):
            _fail(f"{context}.payload has invalid size {size}")
        if offset + size > path.stat().st_size:
            _fail(f"{context}.payload extends past {relative}")
        with path.open("rb") as stream:
            stream.seek(offset)
            payload = stream.read(size)
        if len(payload) != size:
            _fail(f"{context}.payload is truncated")
        if ref.get("crc32") != f"{zlib.crc32(payload) & 0xFFFFFFFF:08x}":
            _fail(f"CRC-32 mismatch for {context}.payload")
        coverage[relative].append((offset, offset + size))
        return payload

    ir_count = 0
    previous_ir_sequence = None
    previous_ir_host = None
    for line, row in _read_jsonl(resolved_files["infrared_framesets.jsonl"]):
        context = f"infrared_framesets.jsonl:{line}"
        sequence = _strict_int(row, "frameset_sequence", context)
        host_ns = _strict_int(row, "host_monotonic_ns", context)
        previous_ir_sequence = _check_increasing(previous_ir_sequence, sequence, f"{context}.frameset_sequence")
        previous_ir_host = _check_increasing(previous_ir_host, host_ns, f"{context}.host_monotonic_ns")
        left = row.get("left")
        right = row.get("right")
        if not isinstance(left, dict) or not isinstance(right, dict):
            _fail(f"{context} must contain left/right objects")
        left_timestamp = _strict_int(left, "device_timestamp_ns", f"{context}.left")
        right_timestamp = _strict_int(right, "device_timestamp_ns", f"{context}.right")
        _strict_int(left, "frame_number", f"{context}.left")
        _strict_int(right, "frame_number", f"{context}.right")
        if abs(left_timestamp - right_timestamp) > 2_000_000:
            _fail(f"{context} left/right timestamps differ by more than 2 ms")
        validate_payload(left.get("payload"), f"{context}.left", exact_size=IR_FRAME_BYTES)
        validate_payload(right.get("payload"), f"{context}.right", exact_size=IR_FRAME_BYTES)
        ir_count += 1

    rgb_count = 0
    previous_rgb_sequence = None
    previous_rgb_host = None
    previous_rgb_pts = None
    for line, row in _read_jsonl(resolved_files["rgb_access_units.jsonl"]):
        context = f"rgb_access_units.jsonl:{line}"
        sequence = _strict_int(row, "frameset_sequence", context)
        host_ns = _strict_int(row, "host_monotonic_ns", context)
        pts_ns = _strict_int(row, "encoder_pts_ns", context)
        previous_rgb_sequence = _check_increasing(previous_rgb_sequence, sequence, f"{context}.frameset_sequence")
        previous_rgb_host = _check_increasing(previous_rgb_host, host_ns, f"{context}.host_monotonic_ns")
        previous_rgb_pts = _check_increasing(previous_rgb_pts, pts_ns, f"{context}.encoder_pts_ns")
        _strict_int(row, "frame_number", context)
        _strict_int(row, "device_timestamp_ns", context)
        if not isinstance(row.get("keyframe"), bool):
            _fail(f"{context}.keyframe must be boolean")
        payload = validate_payload(row.get("payload"), context)
        if not (payload.startswith(b"\x00\x00\x01") or payload.startswith(b"\x00\x00\x00\x01")):
            _fail(f"{context} is not H.265 Annex-B")
        rgb_count += 1

    stm32_count = 0
    previous_stm32_host = None
    for line, row in _read_jsonl(resolved_files["stm32_packets.jsonl"]):
        context = f"stm32_packets.jsonl:{line}"
        host_ns = _strict_int(row, "host_monotonic_ns", context)
        previous_stm32_host = _check_increasing(previous_stm32_host, host_ns, f"{context}.host_monotonic_ns")
        payload = validate_payload(row.get("payload"), context, exact_size=STM32_PACKET_BYTES)
        try:
            parsed = parse_combined(payload)
        except (TypeError, ValueError) as exc:
            _fail(f"{context} contains an invalid STM32 packet: {exc}")
        parsed_metadata = {
            "sequence": parsed.sequence,
            "imu_time_us": parsed.imu_first_byte_rx_us,
            "encoder_time_us": parsed.encoder_read_us,
            "encoder_counter": parsed.counter,
            "flags": parsed.flags,
        }
        for key, expected in parsed_metadata.items():
            if row.get(key) != expected:
                _fail(f"{context}.{key} does not match the raw packet")
        stm32_count += 1

    counts = {
        "ir_framesets": ir_count,
        "rgb_access_units": rgb_count,
        "stm32_packets": stm32_count,
    }
    if counts != manifest.get("counts") or any(count == 0 for count in counts.values()):
        _fail(f"record counts are missing or differ from manifest: {counts}")

    data_files = {name for name in resolved_files if name.startswith("data/")}
    if set(coverage) != data_files:
        _fail("one or more payload chunks are unreferenced")
    for relative, intervals in coverage.items():
        cursor = 0
        for start, end in sorted(intervals):
            if start != cursor:
                _fail(f"payload coverage in {relative} has a gap or overlap at {cursor}")
            cursor = end
        if cursor != resolved_files[relative].stat().st_size:
            _fail(f"payload coverage in {relative} does not reach end of file")

    return {
        "status": "PASS",
        "schema": SCHEMA,
        "session_id": manifest.get("session_id"),
        "counts": counts,
        "files": len(files),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args(argv)
    try:
        report = validate_z2_edge_session(args.session)
    except EdgeSessionValidationError as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
