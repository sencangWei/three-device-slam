from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

from three_device_slam.devices.d405_umi.imu.stream_protocol import crc16_ccitt_false
from three_device_slam.edge_z2.session import (
    IR_FRAME_BYTES,
    EdgeSessionError,
    Z2EdgeSessionWriter,
)
from three_device_slam.edge_z2.validate import (
    EdgeSessionValidationError,
    validate_z2_edge_session,
)


def _stm32_packet(sequence: int = 7) -> bytes:
    raw_imu = bytearray(37)
    raw_imu[0:3] = b"\xeb\x90\x22"
    struct.pack_into("<7f", raw_imu, 4, 0.1, 0.2, 9.8, 0.01, 0.02, 0.03, 25.0)
    struct.pack_into("<I", raw_imu, 32, 1234)
    raw_imu[36] = sum(raw_imu[:36]) & 0xFF

    packet = bytearray(63)
    packet[0:2] = b"\xa5\x5a"
    packet[2] = 1
    packet[3] = 63
    struct.pack_into("<H", packet, 4, 0x03)
    struct.pack_into("<IIII", packet, 6, 100_000 + sequence, 100_010 + sequence, 99, 1234)
    packet[22:24] = b"\x12\x34"
    packet[24:61] = raw_imu
    struct.pack_into("<H", packet, 61, crc16_ccitt_false(packet[:61]))
    return bytes(packet)


def _new_writer(path: Path, *, chunk_bytes: int = IR_FRAME_BYTES) -> Z2EdgeSessionWriter:
    return Z2EdgeSessionWriter(
        path,
        session_id="run-0001",
        board_id="YLX000114ELUZS",
        d405_serial="D405-TEST",
        calibration_id="calibration-sha256:test",
        max_chunk_bytes=chunk_bytes,
    )


def _write_valid_session(path: Path) -> Path:
    writer = _new_writer(path)
    for sequence in (1, 2):
        base = 1_000_000_000 + sequence * 33_333_333
        writer.append_ir_pair(
            frameset_sequence=sequence,
            host_monotonic_ns=base,
            left_frame_number=100 + sequence,
            left_device_timestamp_ns=base - 1_000_000,
            left_y8=bytes([sequence]) * IR_FRAME_BYTES,
            right_frame_number=200 + sequence,
            right_device_timestamp_ns=base - 999_500,
            right_y8=bytes([sequence + 10]) * IR_FRAME_BYTES,
        )
        writer.append_rgb_access_unit(
            frameset_sequence=sequence,
            frame_number=300 + sequence,
            device_timestamp_ns=base - 999_000,
            host_monotonic_ns=base + 100,
            encoder_pts_ns=sequence * 33_333_333,
            keyframe=sequence == 1,
            h265_annex_b=b"\x00\x00\x00\x01\x26" + bytes([sequence]) * 9,
        )
        writer.append_stm32_packet(
            host_monotonic_ns=base + 200,
            packet=_stm32_packet(sequence),
        )
    writer.seal()
    return path


def test_sealed_session_passes_and_freezes_locked_profile(tmp_path: Path) -> None:
    session = _write_valid_session(tmp_path / "session")

    report = validate_z2_edge_session(session)
    manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))

    assert report["status"] == "PASS"
    assert report["counts"] == {
        "ir_framesets": 2,
        "rgb_access_units": 2,
        "stm32_packets": 2,
    }
    assert manifest["profile"]["rgb"]["encoding"] == "h265_annex_b"
    assert manifest["profile"]["infrared_left"]["semantics"] == "lossless_sensor_sample"
    assert manifest["profile"]["stm32"]["packet_bytes"] == 63
    assert len(list((session / "data").glob("infrared-left-y8-*.bin"))) == 2
    assert not (session / ".recording").exists()


def test_bad_ir_size_is_rejected_before_payload_write(tmp_path: Path) -> None:
    writer = _new_writer(tmp_path / "session")

    with pytest.raises(EdgeSessionError, match="exactly"):
        writer.append_ir_pair(
            frameset_sequence=1,
            host_monotonic_ns=1,
            left_frame_number=1,
            left_device_timestamp_ns=1,
            left_y8=b"short",
            right_frame_number=1,
            right_device_timestamp_ns=1,
            right_y8=bytes(IR_FRAME_BYTES),
        )

    assert list((writer.root / "data").iterdir()) == []
    writer.abort()


def test_payload_corruption_is_rejected(tmp_path: Path) -> None:
    session = _write_valid_session(tmp_path / "session")
    payload = next((session / "data").glob("infrared-left-y8-*.bin"))
    with payload.open("r+b") as stream:
        stream.seek(10)
        original = stream.read(1)
        stream.seek(10)
        stream.write(bytes([original[0] ^ 0xFF]))

    with pytest.raises(EdgeSessionValidationError, match="SHA-256 mismatch"):
        validate_z2_edge_session(session)


def test_truncated_payload_is_rejected(tmp_path: Path) -> None:
    session = _write_valid_session(tmp_path / "session")
    payload = next((session / "data").glob("infrared-right-y8-*.bin"))
    with payload.open("r+b") as stream:
        stream.truncate(payload.stat().st_size - 1)

    with pytest.raises(EdgeSessionValidationError, match="size mismatch"):
        validate_z2_edge_session(session)


def test_unsealed_session_is_rejected(tmp_path: Path) -> None:
    writer = _new_writer(tmp_path / "session")
    writer.abort()

    with pytest.raises(EdgeSessionValidationError, match="unsealed"):
        validate_z2_edge_session(writer.root)


def test_non_increasing_rgb_pts_is_rejected(tmp_path: Path) -> None:
    writer = _new_writer(tmp_path / "session")
    common = dict(
        frame_number=1,
        device_timestamp_ns=1,
        keyframe=True,
        h265_annex_b=b"\x00\x00\x00\x01\x26\x01",
    )
    writer.append_rgb_access_unit(
        frameset_sequence=1,
        host_monotonic_ns=1,
        encoder_pts_ns=10,
        **common,
    )

    with pytest.raises(EdgeSessionError, match="rgb_pts_ns must increase"):
        writer.append_rgb_access_unit(
            frameset_sequence=2,
            host_monotonic_ns=2,
            encoder_pts_ns=10,
            **common,
        )
    writer.abort()
