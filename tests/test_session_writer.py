import hashlib
import json
from zlib import crc32

import pytest

from three_device_slam.core.model import SensorRecord
from three_device_slam.core import session_lifecycle
from three_device_slam.core.session_lifecycle import offline_claim, producer_claim
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


def test_writer_rejects_reopening_sealed_session_without_mutation(tmp_path):
    root = tmp_path / "session"
    writer = AppendOnlySessionWriter(root)
    writer.append(
        SensorRecord("ego.video", 1, 1, 1, "host_monotonic", False, True, {}),
        b"raw",
    )
    writer.close()
    payload_path = root / "ego.video.bin"
    manifest_path = root / "manifest.json"
    payload_before = payload_path.read_bytes()
    manifest_before = manifest_path.read_bytes()

    with pytest.raises(RuntimeError, match="sealed"):
        AppendOnlySessionWriter(root)

    assert payload_path.read_bytes() == payload_before
    assert manifest_path.read_bytes() == manifest_before
    assert not (root / ".writer.lock").exists()


def test_only_one_writer_can_claim_an_unsealed_session(tmp_path):
    root = tmp_path / "session"
    first = AppendOnlySessionWriter(root)

    with pytest.raises(RuntimeError, match="active writer"):
        AppendOnlySessionWriter(root)

    first.append(
        SensorRecord("ego.video", 1, 1, 1, "host_monotonic", False, True, {}),
        b"a",
    )
    manifest = first.close()
    payload = (root / "ego.video.bin").read_bytes()

    assert payload == b"a"
    assert json.loads((root / "manifest.json").read_bytes()) == manifest
    assert manifest["streams"]["ego.video"]["payload_sha256"] == hashlib.sha256(
        payload
    ).hexdigest()
    assert not (root / ".writer.lock").exists()


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
    assert [path.name for path in root.iterdir()] == [".writer.lock"]


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


def test_close_failure_releases_writer_claim(tmp_path, monkeypatch):
    writer = AppendOnlySessionWriter(tmp_path)
    writer.append(
        SensorRecord("ego.video", 1, 1, 1, "host_monotonic", False, True, {}),
        b"raw",
    )
    stream_files = writer._streams["ego.video"]

    def fail_replace(*_args):
        raise OSError("simulated manifest publish failure")

    monkeypatch.setattr(
        "three_device_slam.core.session_writer.os.replace", fail_replace
    )

    with pytest.raises(OSError, match="simulated manifest publish failure"):
        writer.close()

    assert not (tmp_path / "manifest.json").exists()
    assert not (tmp_path / ".writer.lock").exists()
    assert all(file.closed for file in stream_files)
    with pytest.raises(RuntimeError, match="terminal"):
        writer.append(
            SensorRecord("ego.video", 2, 2, 2, "host_monotonic", False, True, {}),
            b"forbidden",
        )
    with pytest.raises(RuntimeError, match="terminal"):
        writer.close()


def test_early_close_failure_is_terminal_and_closes_streams(tmp_path, monkeypatch):
    writer = AppendOnlySessionWriter(tmp_path)
    writer.append(
        SensorRecord("ego.video", 1, 1, 1, "host_monotonic", False, True, {}),
        b"raw",
    )
    stream_files = writer._streams["ego.video"]
    original_fsync = __import__("os").fsync
    calls = 0

    def fail_first_fsync(fd):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise OSError("simulated raw fsync failure")
        return original_fsync(fd)

    monkeypatch.setattr(
        "three_device_slam.core.session_writer.os.fsync", fail_first_fsync
    )

    with pytest.raises(OSError, match="simulated raw fsync failure"):
        writer.close()

    payload_path = tmp_path / "ego.video.bin"
    payload_after_failure = payload_path.read_bytes()
    assert all(file.closed for file in stream_files)
    assert not (tmp_path / ".writer.lock").exists()
    with pytest.raises(RuntimeError, match="terminal"):
        writer.append(
            SensorRecord("ego.video", 2, 2, 2, "host_monotonic", False, True, {}),
            b"forbidden",
        )
    assert payload_path.read_bytes() == payload_after_failure
    with pytest.raises(RuntimeError, match="terminal"):
        writer.close()


def test_directory_fsync_failure_is_terminal_after_manifest_replace(
    tmp_path, monkeypatch
):
    writer = AppendOnlySessionWriter(tmp_path)
    writer.append(
        SensorRecord("ego.video", 1, 1, 1, "host_monotonic", False, True, {}),
        b"raw",
    )
    original_fsync = __import__("os").fsync

    def fail_directory_fsync(fd):
        if (tmp_path / "manifest.json").exists():
            raise OSError("simulated directory fsync failure")
        return original_fsync(fd)

    monkeypatch.setattr(
        "three_device_slam.core.session_writer.os.fsync", fail_directory_fsync
    )

    with pytest.raises(OSError, match="simulated directory fsync failure"):
        writer.close()

    assert (tmp_path / "manifest.json").exists()
    assert not (tmp_path / ".writer.lock").exists()
    with pytest.raises(RuntimeError, match="terminal"):
        writer.append(
            SensorRecord("ego.video", 2, 2, 2, "host_monotonic", False, True, {}),
            b"forbidden",
        )
    with pytest.raises(RuntimeError, match="terminal"):
        writer.close()
    with pytest.raises(RuntimeError, match="sealed"):
        AppendOnlySessionWriter(tmp_path)
    assert not (tmp_path / ".writer.lock").exists()


def test_close_cleanup_does_not_mask_original_error(tmp_path, monkeypatch):
    writer = AppendOnlySessionWriter(tmp_path)
    writer.append(
        SensorRecord("ego.video", 1, 1, 1, "host_monotonic", False, True, {}),
        b"raw",
    )

    def fail_primary(_fd):
        raise OSError("primary close failure")

    def fail_cleanup():
        raise OSError("secondary cleanup failure")

    monkeypatch.setattr("three_device_slam.core.session_writer.os.fsync", fail_primary)
    monkeypatch.setattr(writer, "_close_streams_best_effort", fail_cleanup)

    with pytest.raises(OSError, match="primary close failure"):
        writer.close()

    assert not (tmp_path / ".writer.lock").exists()
    with pytest.raises(RuntimeError, match="terminal"):
        writer.close()


def test_producer_and_offline_claims_are_mutually_exclusive_and_cleaned(tmp_path):
    session = tmp_path / "session"
    session.mkdir()

    with producer_claim(session, "ego"):
        assert (session / ".producer.ego.lock").is_file()
        with pytest.raises(RuntimeError, match="active producer"):
            with offline_claim(session):
                raise AssertionError("offline claim entered")
    assert not (session / ".producer.ego.lock").exists()

    with offline_claim(session):
        assert (session / ".offline.lock").is_file()
        with pytest.raises(RuntimeError, match="offline operation"):
            with producer_claim(session, "left"):
                raise AssertionError("producer claim entered")
    assert not (session / ".offline.lock").exists()


@pytest.mark.parametrize("stale", [".producer.ego.lock", ".offline.lock"])
def test_crash_residue_fails_safe_without_stale_recovery(tmp_path, stale):
    session = tmp_path / "session"
    session.mkdir()
    (session / stale).write_bytes(b"")

    claim = offline_claim(session) if stale.startswith(".producer") else producer_claim(
        session, "right"
    )
    with pytest.raises(RuntimeError, match="active producer|offline operation"):
        with claim:
            raise AssertionError("stale claim was recovered")

    assert (session / stale).is_file()


@pytest.mark.parametrize("marker", ["session.seal.json", ".offline.lock"])
def test_append_only_writer_rejects_session_lifecycle_marker_before_writing(
    tmp_path, marker
):
    session = tmp_path / "session"
    ego = session / "ego"
    ego.mkdir(parents=True)
    (session / marker).write_text("sealed", encoding="utf-8")

    with pytest.raises(RuntimeError, match="sealed|offline"):
        AppendOnlySessionWriter(ego)

    assert list(ego.iterdir()) == []


@pytest.mark.parametrize("winner", ["producer", "offline"])
def test_claim_creation_race_has_exactly_one_winner(tmp_path, monkeypatch, winner):
    session = tmp_path / "session"
    session.mkdir()
    loser_rejected = []

    def interleave(kind, root):
        if kind != winner:
            return
        contender = offline_claim(root) if winner == "producer" else producer_claim(
            root, "left"
        )
        with pytest.raises(RuntimeError):
            with contender:
                raise AssertionError("losing claim entered")
        loser_rejected.append(True)

    monkeypatch.setattr(session_lifecycle, "_claim_created_hook", interleave)
    winner_claim = (
        producer_claim(session, "ego") if winner == "producer" else offline_claim(session)
    )

    with winner_claim:
        assert loser_rejected == [True]
