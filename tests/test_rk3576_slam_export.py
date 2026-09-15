from __future__ import annotations

import csv
import json
import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from three_device_slam.devices.d405_umi.imu.stream_protocol import (
    crc16_ccitt_false,
)
from three_device_slam.edge_rk3576.export_slam_session import (
    IMU_RECORD,
    SlamExportError,
    _open_ir_reader,
    _write_frame_clock_csv,
    _write_imu_sidecar,
)
from three_device_slam.edge_rk3576.capture import HEIGHT, WIDTH


def _combined_packet(sequence: int) -> bytes:
    imu = bytearray(37)
    imu[:3] = b"\xeb\x90\x22"
    struct.pack_into("<7f", imu, 4, 1, 2, 3, 4, 5, 6, 25)
    struct.pack_into("<I", imu, 32, sequence + 1)
    imu[36] = sum(imu[:36]) & 0xFF
    packet = bytearray(63)
    packet[:4] = b"\xa5\x5a\x01\x3f"
    struct.pack_into("<H", packet, 4, 3)
    struct.pack_into("<IIII", packet, 6, sequence, 10, 20, sequence + 1)
    struct.pack_into("<H", packet, 22, 1234)
    packet[24:61] = imu
    struct.pack_into("<H", packet, 61, crc16_ccitt_false(packet[:61]))
    return bytes(packet)


def test_slam_export_preserves_frame_clock_mapping(tmp_path: Path) -> None:
    path = tmp_path / "d405_frames.csv"
    _write_frame_clock_csv(
        path,
        [
            {
                "left_sequence": 7,
                "right_sequence": 7,
                "left_source_timestamp_ms": 1_700_000_000_123.5,
                "right_source_timestamp_ms": 1_700_000_000_123.5,
                "observed_host_monotonic_ns": 12_345_678_900,
            }
        ],
    )

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows == [
        {
            "set_index": "0",
            "infrared_left_frame_number": "7",
            "infrared_left_device_ms": "1700000000123.5",
            "infrared_left_mono": "12.345678900",
            "infrared_left_domain": "global_time",
            "infrared_right_frame_number": "7",
            "infrared_right_device_ms": "1700000000123.5",
            "infrared_right_mono": "12.345678900",
            "infrared_right_domain": "global_time",
        }
    ]


def test_slam_export_converts_validated_stm32_to_legacy_imu(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    packet = _combined_packet(40)
    (source / "stm32.bin").write_bytes(packet)
    (source / "stm32_packets.jsonl").write_text(
        json.dumps(
            {
                "record_index": 0,
                "sequence": 40,
                "host_read_complete_monotonic_ns": 12_500_000_000,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    destination = tmp_path / "imu.bin"

    assert _write_imu_sidecar(source, destination) == 1
    assert destination.stat().st_size == IMU_RECORD.size
    assert IMU_RECORD.unpack(destination.read_bytes()) == pytest.approx(
        (12.5, 41, 1, 2, 3, 4, 5, 6, 25)
    )


def test_slam_export_rejects_stm32_index_disagreement(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "stm32.bin").write_bytes(_combined_packet(40))
    (source / "stm32_packets.jsonl").write_text(
        json.dumps(
            {
                "record_index": 0,
                "sequence": 99,
                "host_read_complete_monotonic_ns": 12_500_000_000,
            }
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(SlamExportError, match="index mismatch"):
        _write_imu_sidecar(source, tmp_path / "imu.bin")


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_slam_export_streams_split_zstd_frames_without_expanding_to_disk(
    tmp_path: Path,
) -> None:
    frame_bytes = WIDTH * HEIGHT
    raw = bytes(range(256)) * (frame_bytes * 2 // 256)
    compressed = tmp_path / "infrared-left-y8.raw.zst"
    subprocess.run(
        ["zstd", "--quiet", "-1", "--stdout"],
        input=raw,
        stdout=compressed.open("wb"),
        check=True,
    )

    reader = _open_ir_reader(compressed, "y8_split_zstd")
    try:
        assert reader.read_frame("left IR", 0) == raw[:frame_bytes]
        assert reader.read_frame("left IR", 1) == raw[frame_bytes:]
        reader.assert_exhausted("left IR")
    finally:
        reader.close()


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_slam_export_rejects_unindexed_zstd_trailing_frame(tmp_path: Path) -> None:
    frame_bytes = WIDTH * HEIGHT
    raw = b"\x2a" * (frame_bytes * 2)
    compressed = tmp_path / "infrared-left-y8.raw.zst"
    subprocess.run(
        ["zstd", "--quiet", "-1", "--stdout"],
        input=raw,
        stdout=compressed.open("wb"),
        check=True,
    )

    reader = _open_ir_reader(compressed, "y8_split_zstd")
    try:
        assert reader.read_frame("left IR", 0) == raw[:frame_bytes]
        with pytest.raises(SlamExportError, match="trailing bytes"):
            reader.assert_exhausted("left IR")
    finally:
        reader.close()
