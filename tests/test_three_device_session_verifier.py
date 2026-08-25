import csv
import hashlib
import json
import subprocess
import sys
from zlib import crc32
from pathlib import Path

import pytest


SECTIONS = {
    "provenance",
    "ego",
    "left",
    "right",
    "joint_gate",
    "clock_domains",
    "sync_index",
    "storage",
}
EGO_FORMAL_NS = (
    1_010_000_000,
    1_026_666_667,
    1_043_333_334,
    1_060_000_001,
    1_076_666_668,
)
LEFT_FORMAL_NS = (1_004_000_000, 1_037_333_333, 1_070_666_666, 1_103_999_999)


def _verifier():
    from three_device_slam.quality import verify_session

    return verify_session


def _builder():
    from three_device_slam.synchronization import build_index

    return build_index


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")


def _ego_row(sequence, acquisition_ns, *, warmup, valid=True, domain="host_monotonic"):
    return {
        "stream_id": "ego.video",
        "sequence": sequence,
        "acquisition_ns": acquisition_ns,
        "arrival_ns": acquisition_ns + 1_000,
        "clock_domain": domain,
        "warmup": warmup,
        "valid": valid,
        "metadata": {
            "vendor_sequence": sequence,
            "acquisition_available": domain == "host_monotonic",
        },
        "offset": sequence * 4,
        "size": 4,
        "crc32": 0,
    }


def _ego_acceptance(streams):
    zero_checks = {
        name: {"status": "PASS", "measurement": 0}
        for name in (
            "timestamp_regressions",
            "timestamp_unavailable",
            "xu_failures",
            "duplicate_sequences",
            "sequence_regressions",
            "bad_jpegs",
            "callback_exceptions",
            "capture_errors",
        )
    }
    zero_checks["rate_hz"] = {"status": "PASS", "measurement": 60.0}
    return {
        "schema": "ego.2uq2.acceptance.v1",
        "status": "PASS",
        "checks": zero_checks,
        "counts": {"warmup_frames": 1, "formal_frames": 5},
        "formal_window": {
            "requested_duration_s": 1 / 12,
            "measured_duration_s": 1 / 12,
            "formal_frames": 5,
            "start_ns": 1_000_000_000,
            "deadline_ns": 1_083_333_333,
        },
        "formal_evidence": {
            "observed_frames": 5,
            "total_frames": 5,
            "valid_frames": 5,
            "comparable_frames": 5,
            "first_acquisition_ns": EGO_FORMAL_NS[0],
            "last_acquisition_ns": EGO_FORMAL_NS[-1],
            "window_rate_hz": 60.0,
            "rate_hz": 59.9999988,
            "timestamp_regressions": 0,
            "timestamp_unavailable": 0,
            "xu_failures": 0,
            "bad_jpegs": 0,
            "duplicate_sequences": 0,
            "valid_sequences": 5,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
        },
        "capture_diagnostics": {"frames": 6, "callback_exceptions": 0, "capture_errors": 0},
        "rates": {"video_hz": 60.0, "observed_span_hz": 60.0, "scope": "formal_window"},
        "sequence_evidence": {
            "valid_sequences": 5,
            "duplicate_sequences": 0,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
        },
        "xu_failures": 0,
        "bad_jpegs": 0,
        "hashes": {"xu_library_sha256": "a" * 64, "streams": streams},
    }


def _stream_stats():
    return {
        "received": 4,
        "first_frame_number": 10,
        "last_frame_number": 13,
        "skipped_frames": 0,
        "gap_events": 0,
        "gap_ratio": 0.0,
        "rate_hz": 30.0,
        "repeated_frames": 0,
        "frame_number_resets": 0,
        "timestamp_regressions": 0,
    }


def _d405_acceptance(device):
    imu_zero = {
        key: 0
        for key in (
            "frames_bad",
            "resyncs",
            "dropped_frames",
            "counter_resets",
            "counter_stalls",
            "sequence_gaps",
            "invalid_imu_flags",
            "queue_overflow_flags",
            "serial_errors",
            "serial_reconnects",
        )
    }
    imu_zero["frames_ok"] = 40
    return {
        "result": "PASS",
        "capture_error": None,
        "duration_s": 0.1,
        "warmup_recorded": True,
        "formal_complete_framesets_for_csv": 4,
        "camera_storage": {
            "recorder_clean_shutdown": True,
            "recorder_shutdown_error": None,
            "authoritative_output_stable": True,
            "move_error": None,
        },
        "camera": {
            "result": "PASS",
            "authoritative_stream_stats": "db3_streams",
            "db3_analysis_error": None,
            "csv_rebuild_error": None,
            "db3_streams": {
                "color": _stream_stats(),
                "infrared_left": _stream_stats(),
                "infrared_right": _stream_stats(),
            },
        },
        "imu": {
            "result": "PASS",
            "protocol": "kt_ex9_37",
            "formal_window_stats": imu_zero,
            "formal_samples": 40,
            "samples_written": 45,
            "recorder_clean_shutdown": True,
            "rate_hz": 400.0,
            "recorder_drops": 0,
            "recorder_write_error": None,
            "persisted_formal_lower_bound_ok": True,
        },
        "camera_clock": {
            "configuration": {
                "supported": True,
                "requested": 1.0,
                "readback": 1.0,
                "verified": True,
            },
            "required_streams": ["color", "infrared_left", "infrared_right"],
            "observed_domains": {
                stream: {"global_time": 5}
                for stream in ("color", "infrared_left", "infrared_right")
            },
            "verified": True,
        },
        "joint_start": {
            "device_id": device,
            "formal_start_host_monotonic_ns": 1_000_000_000,
            "first_formal_camera_device_ms": 1000.0,
            "first_formal_imu_counter": 500,
            "clock_domain": "host_monotonic",
            "raw_recording_is_append_only": True,
            "raw_warmup_preserved": True,
            "classification_metadata": "d405_frames.csv:warmup",
        },
    }


def _write_d405_csv(
    path, offset_ns=0, formal_ns=None, timestamp_domain="global_time"
):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        fields = ["set_index", "arrival_mono", "arrival_wall", "warmup"]
        for stream_id in ("color", "infrared_left", "infrared_right"):
            fields.extend(
                (
                    f"{stream_id}_frame_number",
                    f"{stream_id}_device_ms",
                    f"{stream_id}_mono",
                    f"{stream_id}_domain",
                )
            )
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        formal_ns = formal_ns or tuple(value + offset_ns for value in LEFT_FORMAL_NS)
        rows = [(0, 900_000_000 + offset_ns, "1")]
        rows.extend((index, value, "0") for index, value in enumerate(formal_ns, start=1))
        for sequence, timestamp_ns, warmup in rows:
            row = {
                "set_index": sequence,
                "arrival_mono": f"{timestamp_ns / 1_000_000_000:.9f}",
                "arrival_wall": f"{timestamp_ns / 1_000_000_000:.6f}",
                "warmup": warmup,
            }
            for stream_id in ("color", "infrared_left", "infrared_right"):
                row.update(
                    {
                        f"{stream_id}_frame_number": sequence + 10,
                        f"{stream_id}_device_ms": f"{timestamp_ns / 1_000_000:.6f}",
                        f"{stream_id}_mono": f"{timestamp_ns / 1_000_000_000:.9f}",
                        f"{stream_id}_domain": timestamp_domain,
                    }
                )
            writer.writerow(row)
    _refresh_d405_raw_evidence(path)


def _refresh_d405_raw_evidence(path):
    acceptance_path = path.parent / "acceptance.json"
    if not acceptance_path.exists():
        return
    acceptance = json.loads(acceptance_path.read_text())
    raw_files = acceptance.get("raw_files")
    if not isinstance(raw_files, dict) or "d405_frames.csv" not in raw_files:
        return
    raw_files["d405_frames.csv"] = {
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }
    _write_json(acceptance_path, acceptance)
    coordinator_path = path.parents[1] / "coordinator.json"
    if coordinator_path.exists():
        coordinator = json.loads(coordinator_path.read_text())
        coordinator["worker_acceptance"][path.parent.name] = acceptance
        _write_json(coordinator_path, coordinator)


def _valid_session(tmp_path):
    session = tmp_path / "session"
    ego = session / "ego"
    ego.mkdir(parents=True)
    video_bin = ego / "ego.video.bin"
    xu_bin = ego / "ego.xu.bin"
    video_rows = [
        _ego_row(0, 900_000_000, warmup=True),
    ]
    video_rows.extend(
        _ego_row(sequence, timestamp, warmup=False)
        for sequence, timestamp in enumerate(EGO_FORMAL_NS, start=1)
    )
    video_payloads = [bytes((sequence,)) * 4 for sequence in range(len(video_rows))]
    video_bin.write_bytes(b"".join(video_payloads))
    for row, payload in zip(video_rows, video_payloads):
        row["crc32"] = crc32(payload) & 0xFFFFFFFF
    video_jsonl = ego / "ego.video.jsonl"
    video_jsonl.write_text("".join(json.dumps(row) + "\n" for row in video_rows), encoding="utf-8")
    xu_rows = []
    xu_payloads = []
    for offset_index, video_row in enumerate(video_rows):
        vendor_sequence = video_row["metadata"]["vendor_sequence"]
        payload = vendor_sequence.to_bytes(3, "big") + bytes(24)
        row = {
            **video_row,
            "stream_id": "ego.xu",
            "offset": offset_index * 27,
            "size": 27,
            "crc32": crc32(payload) & 0xFFFFFFFF,
        }
        xu_rows.append(row)
        xu_payloads.append(payload)
    xu_bin.write_bytes(b"".join(xu_payloads))
    xu_jsonl = ego / "ego.xu.jsonl"
    xu_jsonl.write_text(
        "".join(json.dumps(row) + "\n" for row in xu_rows), encoding="utf-8"
    )
    streams = {
        "ego.video": {"payload_sha256": _sha256(video_bin), "index_sha256": _sha256(video_jsonl)},
        "ego.xu": {"payload_sha256": _sha256(xu_bin), "index_sha256": _sha256(xu_jsonl)},
    }
    _write_json(ego / "manifest.json", {"schema": "ego.three_device.raw_session.v1", "streams": streams})

    ego_acceptance = _ego_acceptance(streams)
    left_acceptance = _d405_acceptance("left")
    right_acceptance = _d405_acceptance("right")
    _write_json(ego / "acceptance.json", ego_acceptance)
    _write_json(session / "left" / "acceptance.json", left_acceptance)
    _write_json(session / "right" / "acceptance.json", right_acceptance)
    _write_d405_csv(session / "left" / "d405_frames.csv")
    _write_d405_csv(session / "right" / "d405_frames.csv", 2_000_000)
    for device, acceptance in (("left", left_acceptance), ("right", right_acceptance)):
        device_root = session / device
        files = {
            "camera.db3": b"sqlite-camera",
            "external_imu/raw_imu_packets.bin": bytes(45 * 37),
            "external_imu/imu.bin": b"parsed-imu",
            "external_imu/imu_ts.csv": b"counter,ts_mono\n",
        }
        for relative, payload in files.items():
            path = device_root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
        acceptance["raw_files"] = {
            relative: {
                "size_bytes": (device_root / relative).stat().st_size,
                "sha256": _sha256(device_root / relative),
            }
            for relative in (*files, "d405_frames.csv")
        }
        _write_json(device_root / "acceptance.json", acceptance)

    workers = {
        device: {
            "exit_code": 0,
            "early_exit": False,
            "observed_running_at_or_after_formal_end": True,
        }
        for device in ("ego", "left", "right")
    }
    coordinator = {
        "schema": "ego.three_device.coordinator.v1",
        "status": "PASS",
        "reason": "acquisition_timing",
        "workers": workers,
        "scheduled_start_ns": 1_000_000_000,
        "task_start_ego_frame_ns": EGO_FORMAL_NS[0],
        "transitions": [
            {"state": "JOINT_READY", "at_ns": 900_000_000},
            {"state": "RECORDING", "at_ns": 1_000_000_000},
        ],
        "git": {"hash": "b" * 40, "dirty": False},
        "worker_acceptance": {
            "ego": ego_acceptance,
            "left": left_acceptance,
            "right": right_acceptance,
        },
        "barrier_failures": {},
        "coordinator_errors": [],
        "first_error": None,
    }
    _write_json(session / "coordinator.json", coordinator)
    _builder().build_index(session)
    return session


def _rebuild_fixture(session):
    """Re-seal a deliberately mutated test fixture before offline verification."""
    for name in ("sync", "quality"):
        directory = session / name
        if directory.exists():
            for path in directory.iterdir():
                path.unlink()
            directory.rmdir()
    (session / "session.seal.json").unlink(missing_ok=True)
    return _builder().build_index(session)


def _refresh_ego_hash_evidence(session):
    manifest_path = session / "ego" / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["streams"]["ego.video"]["index_sha256"] = _sha256(
        session / "ego" / "ego.video.jsonl"
    )
    manifest["streams"]["ego.xu"]["payload_sha256"] = _sha256(
        session / "ego" / "ego.xu.bin"
    )
    manifest["streams"]["ego.xu"]["index_sha256"] = _sha256(
        session / "ego" / "ego.xu.jsonl"
    )
    _write_json(manifest_path, manifest)
    acceptance_path = session / "ego" / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["hashes"]["streams"] = manifest["streams"]
    _write_json(acceptance_path, acceptance)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["worker_acceptance"]["ego"] = acceptance
    _write_json(coordinator_path, coordinator)


def _set_d405_clock_evidence(
    session,
    device,
    *,
    configuration,
    observed_domains,
    verified,
    joint_domain,
    csv_domain,
):
    acceptance_path = session / device / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["camera_clock"] = {
        "configuration": configuration,
        "required_streams": ["color", "infrared_left", "infrared_right"],
        "observed_domains": observed_domains,
        "verified": verified,
    }
    acceptance["joint_start"]["clock_domain"] = joint_domain
    _write_json(acceptance_path, acceptance)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["worker_acceptance"][device] = acceptance
    _write_json(coordinator_path, coordinator)

    csv_path = session / device / "d405_frames.csv"
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    fieldnames = list(rows[0])
    for row in rows:
        for stream in ("color", "infrared_left", "infrared_right"):
            row[f"{stream}_domain"] = csv_domain
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    _refresh_d405_raw_evidence(csv_path)
    try:
        _rebuild_fixture(session)
    except ValueError:
        pass


def _make_ego_clock_unavailable(
    session, *, domain="gstreamer_timestamp_unavailable", clear_task_start=True,
    unavailable_rows_are_warmup=False
):
    video_path = session / "ego" / "ego.video.jsonl"
    rows = [json.loads(line) for line in video_path.read_text().splitlines()]
    for row in rows:
        if not row["warmup"]:
            row["acquisition_ns"] = 0
            row["clock_domain"] = domain
            row["warmup"] = unavailable_rows_are_warmup
            row["metadata"].update(
                acquisition_available=False,
            )
    video_path.write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    xu_path = session / "ego" / "ego.xu.jsonl"
    xu_rows = [json.loads(line) for line in xu_path.read_text().splitlines()]
    for video_row, xu_row in zip(rows, xu_rows):
        for field in ("acquisition_ns", "clock_domain", "warmup"):
            xu_row[field] = video_row[field]
        xu_row["metadata"]["acquisition_available"] = video_row["metadata"][
            "acquisition_available"
        ]
    xu_path.write_text(
        "".join(json.dumps(row) + "\n" for row in xu_rows), encoding="utf-8"
    )
    acceptance_path = session / "ego" / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["status"] = "BLOCKED"
    acceptance["reason"] = "2uq2_acquisition_clock_unavailable"
    acceptance["formal_evidence"].update(
        comparable_frames=0,
        first_acquisition_ns=None,
        last_acquisition_ns=None,
        rate_hz=0.0,
        timestamp_unavailable=5,
        valid_sequences=5,
    )
    acceptance["sequence_evidence"].update(
        valid_sequences=5,
        duplicate_sequences=0,
        sequence_gaps=0,
        sequence_regressions=0,
    )
    if unavailable_rows_are_warmup:
        acceptance["counts"].update(warmup_frames=6, formal_frames=0)
        acceptance["formal_window"]["formal_frames"] = 0
    for check in acceptance["checks"].values():
        check["status"] = "BLOCKED"
    _write_json(acceptance_path, acceptance)
    _refresh_ego_hash_evidence(session)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["status"] = "BLOCKED"
    coordinator["reason"] = "2uq2_acquisition_clock_unavailable"
    coordinator["workers"]["ego"]["exit_code"] = 3
    coordinator["first_error"] = {
        "reason": "2uq2_acquisition_clock_unavailable",
        "device_id": "ego",
        "worker_status": "BLOCKED",
        "at_ns": 1_100_000_000,
    }
    if clear_task_start:
        coordinator["task_start_ego_frame_ns"] = None
    coordinator["worker_acceptance"]["ego"] = json.loads(acceptance_path.read_text())
    _write_json(coordinator_path, coordinator)
    try:
        _rebuild_fixture(session)
    except ValueError:
        pass


def _run(session):
    verifier = _verifier()
    exit_code = verifier.main(["--session", str(session)])
    report = json.loads(
        (session / "quality" / "acquisition_timing.json").read_text(encoding="utf-8")
    )
    return exit_code, report


def test_phase_one_pass_keeps_product_overall_blocked():
    assert _verifier().compose_product_status("PASS") == {
        "acquisition_timing": "PASS",
        "ego_slam": "BLOCKED/phase_not_delivered",
        "markerless_tracking": "BLOCKED/phase_not_delivered",
        "fusion": "BLOCKED/phase_not_delivered",
        "spatial_accuracy": "BLOCKED/phase_not_delivered",
        "overall": "BLOCKED",
    }


def test_product_status_rejects_invalid_acquisition_status():
    with pytest.raises(ValueError, match="invalid acquisition_timing status"):
        _verifier().compose_product_status("UNKNOWN")


def test_complete_valid_fixture_passes_with_all_sections(tmp_path):
    session = _valid_session(tmp_path)

    exit_code, report = _run(session)

    assert exit_code == 0
    assert report["schema"] == "ego.three_device.acceptance.v1"
    assert report["status"] == "PASS"
    assert set(report["sections"]) == SECTIONS
    assert all(report["sections"][name]["status"] == "PASS" for name in SECTIONS)
    assert report["sections"]["left"]["measurements"]["imu_rate_hz"] == 400.0
    assert report["sections"]["right"]["measurements"]["camera_rates_hz"]["color"] == 30.0
    product_status = json.loads(
        (session / "quality" / "product_status.json").read_text(encoding="utf-8")
    )
    assert product_status == _verifier().compose_product_status("PASS")
    assert not list(session.rglob("*trajectory*"))


def test_verify_session_api_publishes_both_quality_reports(tmp_path):
    session = _valid_session(tmp_path)

    report = _verifier().verify_session(session)

    assert report["status"] == "PASS"
    assert json.loads(
        (session / "quality" / "acquisition_timing.json").read_text()
    ) == report
    assert json.loads(
        (session / "quality" / "product_status.json").read_text()
    ) == _verifier().compose_product_status("PASS")


def test_quality_publication_failure_removes_both_final_reports(tmp_path, monkeypatch):
    session = _valid_session(tmp_path)
    verifier = _verifier()
    quality = session / "quality"
    acquisition_path = quality / "acquisition_timing.json"
    product_path = quality / "product_status.json"
    real_replace = verifier.os.replace

    def fail_product_replace(source, target):
        if Path(target) == quality:
            raise OSError("product status replace failed")
        return real_replace(source, target)

    monkeypatch.setattr(verifier.os, "replace", fail_product_replace)

    with pytest.raises(OSError, match="product status replace failed"):
        verifier.verify_session(session)

    assert not acquisition_path.exists()
    assert not product_path.exists()
    assert not quality.exists()
    assert not list(session.glob(".quality.*.tmp"))


def test_quality_symlink_to_raw_is_rejected_before_any_raw_write(tmp_path):
    session = _valid_session(tmp_path)
    raw_before = {
        path.relative_to(session): path.read_bytes()
        for directory in (session / "ego", session / "left", session / "right")
        for path in directory.rglob("*")
        if path.is_file()
    }
    (session / "quality").symlink_to(session / "ego", target_is_directory=True)

    with pytest.raises(ValueError, match="unsafe.*quality"):
        _verifier().verify_session(session)

    assert raw_before == {
        path.relative_to(session): path.read_bytes()
        for directory in (session / "ego", session / "left", session / "right")
        for path in directory.rglob("*")
        if path.is_file()
    }
    assert not (session / "ego" / "acquisition_timing.json").exists()


def test_second_quality_member_failure_leaves_no_final_or_temporary_directory(
    tmp_path, monkeypatch
):
    session = _valid_session(tmp_path)
    verifier = _verifier()
    original = getattr(verifier, "_write_quality_member", lambda *_args: None)

    def fail_product(directory, name, payload):
        if name == "product_status.json":
            raise OSError("product member write failed")
        return original(directory, name, payload)

    monkeypatch.setattr(verifier, "_write_quality_member", fail_product, raising=False)

    with pytest.raises(OSError, match="product member write failed"):
        verifier.verify_session(session)

    assert not (session / "quality").exists()
    assert not list(session.glob(".quality.*.tmp"))


def test_quality_directory_appears_only_after_both_members_are_durable(
    tmp_path, monkeypatch
):
    session = _valid_session(tmp_path)
    verifier = _verifier()
    real_replace = verifier.os.replace
    observed = []

    def observe_directory_publish(source, target):
        if Path(target) == session / "quality":
            assert not Path(target).exists()
            assert {path.name for path in Path(source).iterdir()} == {
                "acquisition_timing.json",
                "product_status.json",
            }
            observed.append(True)
        return real_replace(source, target)

    monkeypatch.setattr(verifier.os, "replace", observe_directory_publish)

    verifier.verify_session(session)

    assert observed == [True]
    assert (session / "quality" / "acquisition_timing.json").is_file()
    assert (session / "quality" / "product_status.json").is_file()


def test_existing_quality_is_strictly_idempotent_and_never_replaced(
    tmp_path, monkeypatch
):
    session = _valid_session(tmp_path)
    verifier = _verifier()
    expected = verifier.verify_session(session)

    def forbid_replace(_source, target):
        raise AssertionError(f"immutable quality directory was replaced: {target}")

    monkeypatch.setattr(verifier.os, "replace", forbid_replace)

    assert verifier.verify_session(session) == expected


def test_existing_incomplete_quality_is_rejected_without_overwrite(tmp_path):
    session = _valid_session(tmp_path)
    quality = session / "quality"
    quality.mkdir()
    stale = quality / "acquisition_timing.json"
    stale.write_text("partial", encoding="utf-8")

    with pytest.raises(RuntimeError, match="existing quality"):
        _verifier().verify_session(session)

    assert stale.read_text(encoding="utf-8") == "partial"
    assert not (quality / "product_status.json").exists()


def test_verifier_shares_offline_lease_with_builder(tmp_path):
    session = _valid_session(tmp_path)
    builder = _builder()

    with builder._offline_session_lease(session):
        with pytest.raises(RuntimeError, match="offline publication.*progress"):
            _verifier().verify_session(session)


def test_sealed_raw_tamper_cannot_publish_quality_pass(tmp_path):
    session = _valid_session(tmp_path)
    (session / "left" / "d405_frames.csv").write_bytes(
        (session / "left" / "d405_frames.csv").read_bytes() + b"\n"
    )

    assert _verifier().main(["--session", str(session)]) == 2
    product = session / "quality" / "product_status.json"
    assert not product.exists() or json.loads(product.read_text())["acquisition_timing"] != "PASS"


def test_missing_common_clock_evidence_is_blocked(tmp_path):
    session = _valid_session(tmp_path)
    _set_d405_clock_evidence(
        session,
        "right",
        configuration={
            "supported": False,
            "requested": 1.0,
            "readback": None,
            "verified": False,
        },
        observed_domains={
            stream: {} for stream in ("color", "infrared_left", "infrared_right")
        },
        verified=False,
        joint_domain="unverified",
        csv_domain="unverified",
    )

    exit_code, report = _run(session)

    assert exit_code == 3
    assert report["status"] == "BLOCKED"
    assert report["sections"]["clock_domains"]["status"] == "BLOCKED"


@pytest.mark.parametrize(
    ("configuration", "observed_domains"),
    (
        (
            {
                "supported": True,
                "requested": 1.0,
                "readback": 0.0,
                "verified": False,
            },
            {
                stream: {"global_time": 5}
                for stream in ("color", "infrared_left", "infrared_right")
            },
        ),
        (
            {
                "supported": True,
                "requested": 1.0,
                "readback": 1.0,
                "verified": True,
            },
            {
                "color": {"global_time": 5},
                "infrared_left": {"system_time": 5},
                "infrared_right": {"hardware_clock": 5},
            },
        ),
        (
            {
                "supported": True,
                "requested": 1.0,
                "readback": 1.0,
                "verified": True,
            },
            {
                "color": {"global_time": 5},
                "infrared_left": {"global_time": 5},
                "infrared_right": {},
            },
        ),
    ),
)
def test_unverified_d405_camera_clock_is_blocked_not_failed(
    tmp_path, configuration, observed_domains
):
    session = _valid_session(tmp_path)
    _set_d405_clock_evidence(
        session,
        "left",
        configuration=configuration,
        observed_domains=observed_domains,
        verified=False,
        joint_domain="unverified",
        csv_domain="unverified",
    )

    exit_code, report = _run(session)

    assert exit_code == 3
    assert report["status"] == "BLOCKED"
    assert report["sections"]["clock_domains"]["status"] == "BLOCKED"


def _make_real_d405_clock_blocked_session(session, *, exit_code=3):
    _set_d405_clock_evidence(
        session,
        "left",
        configuration={
            "supported": True,
            "requested": 1.0,
            "readback": 0.0,
            "verified": False,
        },
        observed_domains={
            stream: {"global_time": 5}
            for stream in ("color", "infrared_left", "infrared_right")
        },
        verified=False,
        joint_domain="unverified",
        csv_domain="unverified",
    )
    acceptance_path = session / "left" / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance.update(result="BLOCKED", reason="common_clock_unverified")
    _write_json(acceptance_path, acceptance)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator.update(status="BLOCKED", reason="common_clock_unverified")
    coordinator["workers"]["left"]["exit_code"] = exit_code
    coordinator["worker_acceptance"]["left"] = acceptance
    coordinator["first_error"] = {
        "reason": "common_clock_unverified",
        "device_id": "left",
        "worker_status": "BLOCKED",
        "at_ns": 1_100_000_000,
    }
    _write_json(coordinator_path, coordinator)
    try:
        _rebuild_fixture(session)
    except ValueError:
        pass


def test_real_worker_exit3_and_matching_clock_acceptance_remain_blocked(tmp_path):
    session = _valid_session(tmp_path)
    _make_real_d405_clock_blocked_session(session)

    exit_code, report = _run(session)

    assert exit_code == 3
    assert report["status"] == "BLOCKED"
    assert report["sections"]["joint_gate"]["status"] == "BLOCKED"


@pytest.mark.parametrize("error_kind", ["barrier_failure", "barrier_io", "storage_io"])
def test_blocked_clock_evidence_cannot_mask_other_coordinator_errors(
    tmp_path, error_kind
):
    session = _valid_session(tmp_path)
    _make_real_d405_clock_blocked_session(session)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    if error_kind == "barrier_failure":
        coordinator["barrier_failures"]["right"] = {
            "device_id": "right",
            "reason": "timestamp_regression",
            "at_ns": 1_200_000_000,
        }
    else:
        coordinator["coordinator_errors"].append(
            {"reason": error_kind, "at_ns": 1_200_000_000}
        )
    _write_json(coordinator_path, coordinator)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["joint_gate"]["status"] == "FAIL"


@pytest.mark.parametrize("worker_exit", [2, 7])
def test_non_blocked_nonzero_worker_exit_fails_even_with_blocked_acceptance(
    tmp_path, worker_exit
):
    session = _valid_session(tmp_path)
    _make_real_d405_clock_blocked_session(session, exit_code=worker_exit)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["joint_gate"]["status"] == "FAIL"


def test_worker_exit3_without_matching_blocked_acceptance_fails(tmp_path):
    session = _valid_session(tmp_path)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator.update(status="BLOCKED", reason="common_clock_unverified")
    coordinator["workers"]["left"]["exit_code"] = 3
    coordinator["first_error"] = {
        "reason": "common_clock_unverified",
        "device_id": "left",
        "worker_status": "BLOCKED",
        "at_ns": 1_100_000_000,
    }
    _write_json(coordinator_path, coordinator)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["joint_gate"]["status"] == "FAIL"


def test_worker_exit0_with_blocked_acceptance_fails_as_contradiction(tmp_path):
    session = _valid_session(tmp_path)
    _make_real_d405_clock_blocked_session(session, exit_code=0)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["joint_gate"]["status"] == "FAIL"


def test_d405_camera_clock_claim_contradiction_is_fail(tmp_path):
    session = _valid_session(tmp_path)
    _set_d405_clock_evidence(
        session,
        "left",
        configuration={
            "supported": True,
            "requested": 1.0,
            "readback": 0.0,
            "verified": False,
        },
        observed_domains={
            stream: {"global_time": 5}
            for stream in ("color", "infrared_left", "infrared_right")
        },
        verified=False,
        joint_domain="host_monotonic",
        csv_domain="global_time",
    )

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["status"] == "FAIL"
    assert report["sections"]["clock_domains"]["status"] == "FAIL"


def test_d405_csv_domain_contradicting_verified_camera_clock_is_fail(tmp_path):
    session = _valid_session(tmp_path)
    _set_d405_clock_evidence(
        session,
        "right",
        configuration={
            "supported": True,
            "requested": 1.0,
            "readback": 1.0,
            "verified": True,
        },
        observed_domains={
            stream: {"global_time": 5}
            for stream in ("color", "infrared_left", "infrared_right")
        },
        verified=True,
        joint_domain="host_monotonic",
        csv_domain="unverified",
    )

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["clock_domains"]["status"] == "FAIL"


def test_malformed_d405_camera_clock_count_is_fail(tmp_path):
    session = _valid_session(tmp_path)
    acceptance_path = session / "left" / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["camera_clock"]["observed_domains"]["color"]["global_time"] = "five"
    _write_json(acceptance_path, acceptance)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["worker_acceptance"]["left"] = acceptance
    _write_json(coordinator_path, coordinator)
    try:
        _rebuild_fixture(session)
    except ValueError:
        pass

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["clock_domains"]["status"] == "FAIL"


def test_current_d405_producer_shape_without_explicit_clock_domain_is_blocked(tmp_path):
    session = _valid_session(tmp_path)
    for device in ("left", "right"):
        acceptance_path = session / device / "acceptance.json"
        acceptance = json.loads(acceptance_path.read_text())
        acceptance["joint_start"].pop("clock_domain")
        _write_json(acceptance_path, acceptance)
        coordinator_path = session / "coordinator.json"
        coordinator = json.loads(coordinator_path.read_text())
        coordinator["worker_acceptance"][device] = acceptance
        _write_json(coordinator_path, coordinator)
    try:
        _rebuild_fixture(session)
    except ValueError:
        pass

    exit_code, report = _run(session)

    assert exit_code == 3
    assert report["status"] == "BLOCKED"
    assert report["sections"]["clock_domains"]["status"] == "BLOCKED"
    assert "common_clock_unverified" in json.dumps(
        report["sections"]["clock_domains"]["reasons"]
    )


@pytest.mark.parametrize(
    ("domain", "clear_task_start", "unavailable_rows_are_warmup"),
    (
        ("gstreamer_timestamp_unavailable", True, False),
        ("arrival_only", False, False),
        ("gstreamer_timestamp_unavailable", True, True),
    ),
)
def test_unavailable_formal_ego_clock_and_absent_derived_are_blocked(
    tmp_path, domain, clear_task_start, unavailable_rows_are_warmup
):
    session = _valid_session(tmp_path)
    _make_ego_clock_unavailable(
        session,
        domain=domain,
        clear_task_start=clear_task_start,
        unavailable_rows_are_warmup=unavailable_rows_are_warmup,
    )

    exit_code, report = _run(session)

    assert exit_code == 3
    assert report["status"] == "BLOCKED"
    assert report["sections"]["clock_domains"]["status"] == "BLOCKED"
    assert report["sections"]["sync_index"]["status"] == "BLOCKED"
    assert report["sections"]["storage"]["status"] != "FAIL"


def test_vendor_sequence_gap_cannot_be_hidden_by_zero_summary(tmp_path):
    session = _valid_session(tmp_path)
    video_path = session / "ego" / "ego.video.jsonl"
    rows = [json.loads(line) for line in video_path.read_text().splitlines()]
    rows[3]["metadata"]["vendor_sequence"] += 1
    video_path.write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    _refresh_ego_hash_evidence(session)
    _rebuild_fixture(session)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["ego"]["status"] == "FAIL"
    assert "sequence_gaps" in json.dumps(report["sections"]["ego"]["reasons"])


def test_d405_regression_fails_even_when_ego_clock_blocks_sync(tmp_path):
    session = _valid_session(tmp_path)
    _make_ego_clock_unavailable(session)
    path = session / "left" / "d405_frames.csv"
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    rows[-1]["arrival_mono"] = "0.500000000"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["status"] == "FAIL"
    assert report["sections"]["clock_domains"]["status"] == "FAIL"
    assert "left" in json.dumps(report["sections"]["clock_domains"]["reasons"])


def test_formal_timestamp_regression_is_fail(tmp_path):
    session = _valid_session(tmp_path)
    path = session / "ego" / "ego.video.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[-1]["acquisition_ns"] = rows[-2]["acquisition_ns"] - 1
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["status"] == "FAIL"
    assert report["sections"]["clock_domains"]["status"] == "FAIL"


def test_worker_disconnect_is_fail_and_precedes_blocked(tmp_path):
    session = _valid_session(tmp_path)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["status"] = "FAIL"
    coordinator["reason"] = "camera_disconnect"
    coordinator["workers"]["left"]["exit_code"] = 2
    coordinator["first_error"] = {"reason": "camera_disconnect", "device_id": "left", "at_ns": 2}
    _write_json(coordinator_path, coordinator)

    ego_path = session / "ego" / "acceptance.json"
    ego = json.loads(ego_path.read_text())
    ego["status"] = "BLOCKED"
    ego["reason"] = "2uq2_xu_frame_relation"
    _write_json(ego_path, ego)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["status"] == "FAIL"
    assert "camera_disconnect" in json.dumps(report["reasons"])


def test_retained_over_hard_limit_row_is_valid_only_when_nontrainable(tmp_path):
    session = _valid_session(tmp_path)
    _write_d405_csv(
        session / "right" / "d405_frames.csv",
        formal_ns=(1_020_000_000, 1_055_000_000, 1_072_700_000, 1_106_000_000),
    )
    _rebuild_fixture(session)

    exit_code, report = _run(session)

    assert exit_code == 0
    counts = report["sections"]["sync_index"]["measurements"]["reason_counts"]
    assert counts["span_over_hard_limit"] > 0

    csv_path = session / "sync" / "common_30hz.csv"
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    over = next(row for row in rows if row["reason"] == "span_over_hard_limit")
    over["trainable"] = "1"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    manifest_path = session / "sync" / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["output_csv_sha256"] = _sha256(csv_path)
    _write_json(manifest_path, manifest)
    for path in (session / "quality").iterdir():
        path.unlink()
    (session / "quality").rmdir()

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["sync_index"]["status"] == "FAIL"


def test_skipped_xu_frame_relation_is_blocked(tmp_path):
    session = _valid_session(tmp_path)
    path = session / "ego" / "acceptance.json"
    ego = json.loads(path.read_text())
    ego["status"] = "BLOCKED"
    ego["reason"] = "2uq2_xu_frame_relation"
    for check in ego["checks"].values():
        check["status"] = "BLOCKED"
    _write_json(path, ego)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["status"] = "BLOCKED"
    coordinator["reason"] = "2uq2_xu_frame_relation"
    coordinator["workers"]["ego"]["exit_code"] = 3
    coordinator["first_error"] = {
        "reason": "2uq2_xu_frame_relation",
        "device_id": "ego",
        "worker_status": "BLOCKED",
        "at_ns": 1_100_000_000,
    }
    coordinator["worker_acceptance"]["ego"] = ego
    _write_json(coordinator_path, coordinator)
    _rebuild_fixture(session)

    exit_code, report = _run(session)

    assert exit_code == 3
    assert report["status"] == "BLOCKED"
    assert report["sections"]["ego"]["status"] == "BLOCKED"


def test_nested_ego_blocked_check_blocks_even_when_top_status_says_pass(tmp_path):
    session = _valid_session(tmp_path)
    acceptance_path = session / "ego" / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["checks"]["xu_failures"]["status"] = "BLOCKED"
    _write_json(acceptance_path, acceptance)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["worker_acceptance"]["ego"] = acceptance
    _write_json(coordinator_path, coordinator)
    _rebuild_fixture(session)

    exit_code, report = _run(session)

    assert exit_code == 3
    assert report["sections"]["ego"]["status"] == "BLOCKED"


@pytest.mark.parametrize("device", ("ego", "left", "right"))
def test_raw_warmup_and_formal_counts_must_reconcile_with_acceptance(tmp_path, device):
    session = _valid_session(tmp_path)
    if device == "ego":
        path = session / "ego" / "ego.video.jsonl"
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        path.write_text(
            "".join(json.dumps(row) + "\n" for row in rows if not row["warmup"]),
            encoding="utf-8",
        )
        _refresh_ego_hash_evidence(session)
    else:
        path = session / device / "d405_frames.csv"
        rows = list(csv.DictReader(path.open(encoding="utf-8")))
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0])
            writer.writeheader()
            writer.writerows(row for row in rows if row["warmup"] == "0")
    _rebuild_fixture(session)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"][device]["status"] == "FAIL"


def test_one_formal_ego_row_cannot_establish_a_measured_rate(tmp_path):
    session = _valid_session(tmp_path)
    video_path = session / "ego" / "ego.video.jsonl"
    rows = [json.loads(line) for line in video_path.read_text().splitlines()]
    video_path.write_text(
        "".join(
            json.dumps(row) + "\n"
            for row in rows
            if row["warmup"] or row["sequence"] == 1
        ),
        encoding="utf-8",
    )
    acceptance_path = session / "ego" / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["counts"]["formal_frames"] = 1
    acceptance["formal_window"].update(
        requested_duration_s=1 / 60,
        measured_duration_s=1 / 60,
        formal_frames=1,
        deadline_ns=EGO_FORMAL_NS[0],
    )
    acceptance["formal_evidence"].update(
        observed_frames=1,
        total_frames=1,
        valid_frames=1,
        comparable_frames=1,
        first_acquisition_ns=EGO_FORMAL_NS[0],
        last_acquisition_ns=EGO_FORMAL_NS[0],
        window_rate_hz=60.0,
        rate_hz=0.0,
    )
    acceptance["sequence_evidence"]["valid_sequences"] = 1
    _write_json(acceptance_path, acceptance)
    _refresh_ego_hash_evidence(session)
    _rebuild_fixture(session)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["ego"]["status"] == "FAIL"


def test_malformed_unhashable_status_still_publishes_fail_acceptance(tmp_path):
    session = _valid_session(tmp_path)
    path = session / "ego" / "acceptance.json"
    acceptance = json.loads(path.read_text())
    acceptance["status"] = []
    _write_json(path, acceptance)

    verifier = _verifier()
    assert verifier.main(["--session", str(session)]) == 2
    report = json.loads(
        (session / "quality" / "acquisition_timing.json").read_text()
    )
    assert report["status"] == "FAIL"


@pytest.mark.parametrize("mutation", ["payload", "offset", "size", "sequence", "missing_row"])
def test_ego_xu_index_is_verified_against_exact_27_byte_payloads(tmp_path, mutation):
    session = _valid_session(tmp_path)
    xu_path = session / "ego" / "ego.xu.bin"
    index_path = session / "ego" / "ego.xu.jsonl"
    rows = [json.loads(line) for line in index_path.read_text().splitlines()]
    if mutation == "payload":
        payload = bytearray(xu_path.read_bytes())
        payload[0] ^= 1
        xu_path.write_bytes(payload)
    elif mutation == "offset":
        rows[0]["offset"] = 1
    elif mutation == "size":
        rows[0]["size"] = 26
    elif mutation == "sequence":
        rows[0]["metadata"]["vendor_sequence"] += 1
    else:
        rows.pop()
    if mutation != "payload":
        index_path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    _refresh_ego_hash_evidence(session)

    report = _verifier().verify_session(session)

    assert report["status"] == "FAIL"
    assert report["sections"]["storage"]["status"] == "FAIL"


@pytest.mark.parametrize(
    "mutation", ["video_offset", "video_size", "video_crc", "xu_imu_crc"]
)
def test_ego_video_and_xu_payload_crc_and_coverage_are_independently_verified(
    tmp_path, mutation
):
    session = _valid_session(tmp_path)
    video_index = session / "ego" / "ego.video.jsonl"
    xu_index = session / "ego" / "ego.xu.jsonl"
    video_rows = [json.loads(line) for line in video_index.read_text().splitlines()]
    if mutation == "video_offset":
        video_rows[1]["offset"] += 1
    elif mutation == "video_size":
        video_rows[1]["size"] -= 1
    elif mutation == "video_crc":
        video_rows[1]["crc32"] ^= 1
    else:
        xu_payload = session / "ego" / "ego.xu.bin"
        payload = bytearray(xu_payload.read_bytes())
        payload[10] ^= 1
        xu_payload.write_bytes(payload)
    if mutation != "xu_imu_crc":
        video_index.write_text(
            "".join(json.dumps(row) + "\n" for row in video_rows), encoding="utf-8"
        )
    _refresh_ego_hash_evidence(session)
    _rebuild_fixture(session)

    report = _verifier().verify_session(session)

    assert report["status"] == "FAIL"
    assert "payload" in json.dumps(report["sections"]["storage"]["reasons"])


def test_d405_raw_manifest_detects_truncated_file_even_if_acceptance_says_pass(tmp_path):
    session = _valid_session(tmp_path)
    path = session / "left" / "external_imu" / "raw_imu_packets.bin"
    path.write_bytes(path.read_bytes()[:-1])

    report = _verifier().verify_session(session)

    assert report["status"] == "FAIL"
    assert "d405_raw" in json.dumps(report["sections"]["storage"]["reasons"])


def test_d405_raw_packet_count_must_cover_declared_parsed_samples(tmp_path):
    session = _valid_session(tmp_path)
    device_root = session / "left"
    path = device_root / "external_imu" / "raw_imu_packets.bin"
    path.write_bytes(bytes(44 * 37))
    acceptance_path = device_root / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["raw_files"]["external_imu/raw_imu_packets.bin"] = {
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }
    _write_json(acceptance_path, acceptance)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["worker_acceptance"]["left"] = acceptance
    _write_json(coordinator_path, coordinator)
    _rebuild_fixture(session)

    report = _verifier().verify_session(session)

    assert report["status"] == "FAIL"
    assert "raw_packet_count" in json.dumps(report["sections"]["storage"]["reasons"])


def test_sync_index_without_any_trainable_row_cannot_pass(tmp_path):
    session = _valid_session(tmp_path)
    _write_d405_csv(
        session / "right" / "d405_frames.csv",
        formal_ns=(1_020_000_000, 1_055_000_000, 1_088_000_000, 1_121_000_000),
    )
    _rebuild_fixture(session)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["sections"]["sync_index"]["status"] == "FAIL"
    assert report["sections"]["sync_index"]["measurements"]["row_count"] > 0
    assert report["sections"]["sync_index"]["measurements"]["trainable_count"] == 0


@pytest.mark.parametrize(
    ("mutation", "section"),
    [
        ("hash_mismatch", "storage"),
        ("malformed_schema", "ego"),
        ("malformed_nested", "left"),
        ("path_escape", "storage"),
        ("missing_raw", "storage"),
        ("warmup_leakage", "sync_index"),
        ("recorder_drop", "left"),
        ("write_error", "right"),
        ("unsupported_imu_protocol", "left"),
        ("camera_empty", "left"),
        ("camera_count_mismatch", "left"),
        ("sync_input_clock", "sync_index"),
        ("sync_overlap", "sync_index"),
    ],
)
def test_corrupt_or_broken_evidence_fails_closed(tmp_path, mutation, section):
    session = _valid_session(tmp_path)
    if mutation == "hash_mismatch":
        (session / "ego" / "ego.video.bin").write_bytes(b"changed")
    elif mutation == "malformed_schema":
        path = session / "ego" / "acceptance.json"
        value = json.loads(path.read_text())
        value["schema"] = "wrong"
        _write_json(path, value)
    elif mutation == "malformed_nested":
        path = session / "left" / "acceptance.json"
        value = json.loads(path.read_text())
        value["imu"] = []
        _write_json(path, value)
    elif mutation == "path_escape":
        path = session / "ego" / "manifest.json"
        value = json.loads(path.read_text())
        value["streams"]["../escape"] = {
            "payload_sha256": "0" * 64,
            "index_sha256": "0" * 64,
        }
        _write_json(path, value)
    elif mutation == "missing_raw":
        (session / "ego" / "ego.xu.bin").unlink()
    elif mutation == "warmup_leakage":
        path = session / "sync" / "common_30hz.csv"
        rows = list(csv.DictReader(path.open(encoding="utf-8")))
        rows[0]["ego_sequence"] = "0"
        rows[0]["ego_ref"] = "ego/ego.video.bin:0:4"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0])
            writer.writeheader()
            writer.writerows(rows)
        manifest_path = path.with_name("manifest.json")
        manifest = json.loads(manifest_path.read_text())
        manifest["output_csv_sha256"] = _sha256(path)
        _write_json(manifest_path, manifest)
    elif mutation == "recorder_drop":
        path = session / "left" / "acceptance.json"
        value = json.loads(path.read_text())
        value["imu"]["recorder_drops"] = 1
        _write_json(path, value)
    elif mutation == "write_error":
        path = session / "right" / "acceptance.json"
        value = json.loads(path.read_text())
        value["imu"]["recorder_write_error"] = "disk full"
        _write_json(path, value)
    elif mutation == "unsupported_imu_protocol":
        path = session / "left" / "acceptance.json"
        value = json.loads(path.read_text())
        value["imu"]["protocol"] = "unknown"
        _write_json(path, value)
    elif mutation == "camera_empty":
        path = session / "left" / "acceptance.json"
        value = json.loads(path.read_text())
        value["camera"]["db3_streams"]["color"]["received"] = 0
        _write_json(path, value)
    elif mutation == "camera_count_mismatch":
        path = session / "left" / "acceptance.json"
        value = json.loads(path.read_text())
        value["camera"]["db3_streams"]["color"]["received"] = 3
        _write_json(path, value)
    elif mutation == "sync_input_clock":
        path = session / "sync" / "manifest.json"
        value = json.loads(path.read_text())
        value["input_config"]["right"]["clock_domain"] = "arrival_only"
        _write_json(path, value)
    else:
        path = session / "sync" / "manifest.json"
        value = json.loads(path.read_text())
        value["common_overlap"]["start_ns"] += 1
        _write_json(path, value)

    exit_code, report = _run(session)

    assert exit_code == 2
    assert report["status"] == "FAIL"
    assert report["sections"][section]["status"] == "FAIL"


def test_output_is_atomic_durable_and_exit_mapping_is_stable(tmp_path, monkeypatch):
    session = _valid_session(tmp_path)
    verifier = _verifier()
    replacements = []
    fsynced_directories = []
    real_replace = verifier.os.replace
    real_fsync_directory = verifier._fsync_directory

    def observe_replace(source, target):
        replacements.append((Path(source), Path(target)))
        assert Path(source).is_dir()
        return real_replace(source, target)

    def observe_fsync(path):
        fsynced_directories.append(Path(path))
        return real_fsync_directory(path)

    monkeypatch.setattr(verifier.os, "replace", observe_replace)
    monkeypatch.setattr(verifier, "_fsync_directory", observe_fsync)

    assert verifier.main(["--session", str(session)]) == 0
    assert [target for _, target in replacements] == [session / "quality"]
    assert session in fsynced_directories
    assert not list(session.glob(".quality.*.tmp"))
    assert verifier.exit_code_for_status("PASS") == 0
    assert verifier.exit_code_for_status("FAIL") == 2
    assert verifier.exit_code_for_status("BLOCKED") == 3
    with pytest.raises(ValueError):
        verifier.exit_code_for_status("UNKNOWN")


@pytest.mark.parametrize(
    "module",
    (
        "three_device_slam.synchronization.build_index",
        "three_device_slam.quality.verify_session",
    ),
)
def test_module_cli_help_has_no_preimport_warning(module):
    completed = subprocess.run(
        [sys.executable, "-m", module, "--help"],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "--session" in completed.stdout
    assert "RuntimeWarning" not in completed.stderr
