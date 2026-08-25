import hashlib
import json
from zlib import crc32

import pytest

from three_device_slam.core.model import SensorRecord
from three_device_slam.core.session_writer import AppendOnlySessionWriter


def test_writer_records_all_fields_offsets_crc_and_hashes(tmp_path):
    writer = AppendOnlySessionWriter(tmp_path)
    record = SensorRecord(
        "ego.video",
        3,
        1000,
        1050,
        "host_monotonic",
        False,
        True,
        {"encoding": "MJPG", "width": 3840, "height": 1080},
    )
    ref = writer.append(record, b"jpeg")
    second_ref = writer.append(record, b"raw")
    manifest = writer.close()
    manifest_again = writer.close()
    payload_path = tmp_path / "ego.video.bin"
    index_path = tmp_path / "ego.video.jsonl"
    rows = [json.loads(line) for line in index_path.read_text().splitlines()]
    assert ref == "ego.video.bin:0:4"
    assert second_ref == "ego.video.bin:4:3"
    assert rows == [
        {
            "stream_id": "ego.video",
            "sequence": 3,
            "acquisition_ns": 1000,
            "arrival_ns": 1050,
            "clock_domain": "host_monotonic",
            "warmup": False,
            "valid": True,
            "metadata": {"encoding": "MJPG", "width": 3840, "height": 1080},
            "offset": 0,
            "size": 4,
            "crc32": crc32(b"jpeg") & 0xFFFFFFFF,
        },
        {
            "stream_id": "ego.video",
            "sequence": 3,
            "acquisition_ns": 1000,
            "arrival_ns": 1050,
            "clock_domain": "host_monotonic",
            "warmup": False,
            "valid": True,
            "metadata": {"encoding": "MJPG", "width": 3840, "height": 1080},
            "offset": 4,
            "size": 3,
            "crc32": crc32(b"raw") & 0xFFFFFFFF,
        },
    ]
    assert manifest["schema"] == "ego.three_device.raw_session.v1"
    assert manifest_again == manifest
    assert manifest["streams"]["ego.video"] == {
        "payload_sha256": hashlib.sha256(payload_path.read_bytes()).hexdigest(),
        "index_sha256": hashlib.sha256(index_path.read_bytes()).hexdigest(),
    }


def test_writer_rejects_append_after_close(tmp_path):
    writer = AppendOnlySessionWriter(tmp_path)
    writer.close()
    try:
        writer.append(
            SensorRecord("ego.xu", 1, 1, 1, "host_monotonic", True, True, {}),
            bytes(27),
        )
    except RuntimeError as exc:
        assert "closed" in str(exc)
    else:
        raise AssertionError("append after close was accepted")


@pytest.mark.parametrize("stream_id", ["../escape", "ego/video", "ego\\video", "ego:video", "CON"])
def test_writer_rejects_unsafe_stream_id_before_writing(tmp_path, stream_id):
    root = tmp_path / "session"
    writer = AppendOnlySessionWriter(root)

    with pytest.raises(ValueError, match="stream_id"):
        writer.append(
            SensorRecord(stream_id, 1, 1, 1, "host_monotonic", True, True, {}),
            b"payload",
        )

    assert not (tmp_path / "escape.bin").exists()
    assert list(root.iterdir()) == []


def test_invalid_metadata_leaves_existing_stream_files_unchanged(tmp_path):
    writer = AppendOnlySessionWriter(tmp_path)
    writer.append(
        SensorRecord(
            "ego.video", 1, 1, 1, "host_monotonic", False, True, {"value": "x" * 9000}
        ),
        b"valid" * 2000,
    )
    payload_path = tmp_path / "ego.video.bin"
    index_path = tmp_path / "ego.video.jsonl"
    before_payload = payload_path.read_bytes()
    before_index = index_path.read_bytes()

    with pytest.raises(TypeError):
        writer.append(
            SensorRecord(
                "ego.video", 2, 2, 2, "host_monotonic", False, True, {"bad": object()}
            ),
            b"orphan" * 2000,
        )

    assert payload_path.read_bytes() == before_payload
    assert index_path.read_bytes() == before_index


def test_close_fsyncs_session_directory_after_manifest_replace(tmp_path, monkeypatch):
    writer = AppendOnlySessionWriter(tmp_path)
    fsynced_after_replace = []
    original_fsync = __import__("os").fsync

    def observe_fsync(fd):
        if (tmp_path / "manifest.json").exists():
            fsynced_after_replace.append(fd)
        return original_fsync(fd)

    monkeypatch.setattr("three_device_slam.core.session_writer.os.fsync", observe_fsync)
    writer.close()

    assert fsynced_after_replace, "manifest replace must be followed by directory fsync"
