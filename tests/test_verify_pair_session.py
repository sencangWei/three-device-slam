"""Offline verification tests for D435i-Ego + single-UMI pair sessions."""

import hashlib
import json
from zlib import crc32

from three_device_slam.quality import verify_pair_session
from three_device_slam.synchronization import build_index as builder
from three_device_slam.synchronization.sync_index import GRID_NS


TASK_START_NS = 20_000_000_000
CLOCK_DOMAIN = "realsense_global_time_mapped_to_host_monotonic"


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _ego_acceptance(first_formal_ns):
    def stream(hz):
        return {
            "status": "PASS",
            "measured_hz": hz,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
            "timestamp_domains": ["global_time"],
        }

    return {
        "schema": "ego.d435i.acceptance.v1",
        "status": "PASS",
        "calibration_id": "ego-cal-v1",
        "streams": {
            "ir_left": stream(30.0),
            "ir_right": stream(30.0),
            "gyro": stream(200.0),
            "accel": stream(202.0),
        },
        "stereo_pairing": {"status": "PASS", "unmatched_frames": 0},
        "writer_queue_drops": 0,
        "formal_evidence": {"first_acquisition_ns": first_formal_ns},
    }


def _left_acceptance():
    def stream(hz):
        return {
            "status": "PASS",
            "measured_hz": hz,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
            "timestamp_domains": ["global_time"],
        }

    return {
        "schema": "umi.d405.append_only.acceptance.v1",
        "result": "PASS",
        "calibration_id": "left-cal-v1",
        "live_vins": {"imu_calibration": "left-cal-v1"},
        "capture_error": None,
        "writer_queue_drops": 0,
        "camera_clock": {"verified": True},
        "camera_streams": {
            "color": stream(30.0),
            "ir_left": stream(30.0),
            "ir_right": stream(30.0),
        },
        "imu": {
            "result": "PASS",
            "rate_hz": 400.0,
            "recorder_drops": 0,
            "formal_samples": 12000,
        },
        "joint_start": {"clock_domain": "host_monotonic"},
    }


def _write_stream(session, device_dir, stream_id, rows_spec):
    """rows_spec: list of (sequence, acquisition_ns, warmup)."""
    payload = bytearray()
    index_rows = []
    for sequence, acquisition_ns, warmup in rows_spec:
        frame = bytes([sequence & 0xFF]) * 16
        offset = len(payload)
        payload.extend(frame)
        index_rows.append(
            {
                "stream_id": stream_id,
                "sequence": sequence,
                "acquisition_ns": acquisition_ns,
                "arrival_ns": acquisition_ns + 4_000_000,
                "clock_domain": CLOCK_DOMAIN,
                "warmup": warmup,
                "valid": True,
                "offset": offset,
                "size": len(frame),
                "crc32": crc32(frame) & 0xFFFFFFFF,
                "metadata": {"timestamp_domain": "global_time"},
            }
        )
    device = session / device_dir
    device.mkdir(parents=True, exist_ok=True)
    index_payload = (
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in index_rows)
    ).encode("utf-8")
    (device / f"{stream_id}.jsonl").write_bytes(index_payload)
    (device / f"{stream_id}.bin").write_bytes(bytes(payload))
    return {
        "payload_sha256": _sha256(bytes(payload)),
        "index_sha256": _sha256(index_payload),
    }


def _write_manifest(session, device_dir, streams):
    (session / device_dir / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "ego.three_device.raw_session.v1",
                "streams": streams,
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )


def write_verified_pair_session(tmp_path, *, formal_frames=15):
    session = tmp_path / "session"
    session.mkdir()
    frame_count = formal_frames + 3
    ego_left = [
        (index, TASK_START_NS - 3_000_000 + index * 1_000_000, index < 3)
        if index < 3
        else (index, TASK_START_NS + 100_000 + (index - 3) * GRID_NS, False)
        for index in range(frame_count)
    ]
    ego_right = [
        (sequence, ns + 100, warmup) for sequence, ns, warmup in ego_left
    ]
    gyro = [
        (index, TASK_START_NS - 3_000_000 + index * 5_000_000, index < 3)
        if index < 3
        else (index, TASK_START_NS + 100_000 + (index - 3) * 5_000_000, False)
        for index in range(frame_count)
    ]
    accel = [(s, ns + 50, w) for s, ns, w in gyro]
    left_ir = [
        (index, TASK_START_NS - 2_000_000 + index * 1_000_000, index < 2)
        if index < 2
        else (index, TASK_START_NS + 120_000 + (index - 2) * GRID_NS, False)
        for index in range(formal_frames + 2)
    ]
    left_color = [(s, ns + 80, w) for s, ns, w in left_ir]
    left_right = [(s, ns + 160, w) for s, ns, w in left_ir]

    ego_streams = {
        "ego.ir_left": _write_stream(session, "ego", "ego.ir_left", ego_left),
        "ego.ir_right": _write_stream(session, "ego", "ego.ir_right", ego_right),
        "ego.gyro": _write_stream(session, "ego", "ego.gyro", gyro),
        "ego.accel": _write_stream(session, "ego", "ego.accel", accel),
    }
    left_streams = {
        "left.color": _write_stream(session, "left", "left.color", left_color),
        "left.ir_left": _write_stream(session, "left", "left.ir_left", left_ir),
        "left.ir_right": _write_stream(session, "left", "left.ir_right", left_right),
    }
    _write_manifest(session, "ego", ego_streams)
    _write_manifest(session, "left", left_streams)

    ego_acceptance = _ego_acceptance(TASK_START_NS + 100_000)
    left_acceptance = _left_acceptance()
    (session / "ego" / "acceptance.json").write_text(
        json.dumps(ego_acceptance, sort_keys=True), encoding="utf-8"
    )
    (session / "left" / "acceptance.json").write_text(
        json.dumps(left_acceptance, sort_keys=True), encoding="utf-8"
    )

    coordinator = {
        "schema": "ego.three_device.coordinator.v1",
        "status": "PASS",
        "reason": "acquisition_timing",
        "capture_topology": "single_umi",
        "owner_model": "single_process_single_thread_realsense_lifecycle",
        "scheduled_start_ns": TASK_START_NS - 500_000,
        "task_start_ego_frame_ns": TASK_START_NS + 100_000,
        "transitions": [
            {"state": "COLD", "at_ns": TASK_START_NS - 10_000_000},
            {"state": "JOINT_READY", "at_ns": TASK_START_NS - 1_000_000},
            {"state": "RECORDING", "at_ns": TASK_START_NS - 500_000},
            {"state": "COMPLETE", "at_ns": TASK_START_NS + 30_000_000_000},
        ],
        "workers": {
            device: {
                "exit_code": 0,
                "early_exit": False,
                "observed_running_at_or_after_formal_end": True,
            }
            for device in ("ego", "left")
        },
        "calibration_expectations": {
            "ego": {
                "expected": "ego-cal-v1",
                "observed": "ego-cal-v1",
                "status": "PASS",
            },
            "left": {
                "expected": "left-cal-v1",
                "observed": "left-cal-v1",
                "status": "PASS",
            },
        },
        "worker_acceptance": {
            "ego": ego_acceptance,
            "left": left_acceptance,
        },
        "git": {"hash": "pairabc123", "dirty": False},
    }
    (session / "coordinator.json").write_text(
        json.dumps(coordinator, sort_keys=True), encoding="utf-8"
    )
    return session


def convert_verified_pair_session_to_right(session):
    left = session / "left"
    right = session / "right"
    left.rename(right)
    for path in list(right.iterdir()):
        if path.name.startswith("left."):
            path.rename(right / path.name.replace("left.", "right.", 1))
    for path in right.glob("right.*.jsonl"):
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        for row in rows:
            row["stream_id"] = row["stream_id"].replace("left.", "right.", 1)
        path.write_text(
            "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
            encoding="utf-8",
        )
    manifest_path = right / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["streams"] = {
        key.replace("left.", "right.", 1): value
        for key, value in manifest["streams"].items()
    }
    for key, value in manifest["streams"].items():
        index_path = right / f"{key}.jsonl"
        value["index_sha256"] = _sha256(index_path.read_bytes())
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    acceptance_path = right / "acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["calibration_id"] = "right-cal-v1"
    acceptance["live_vins"]["imu_calibration"] = "right-cal-v1"
    acceptance_path.write_text(json.dumps(acceptance, sort_keys=True), encoding="utf-8")
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["workers"]["right"] = coordinator["workers"].pop("left")
    expectation = coordinator["calibration_expectations"].pop("left")
    expectation.update(expected="right-cal-v1", observed="right-cal-v1")
    coordinator["calibration_expectations"]["right"] = expectation
    coordinator["worker_acceptance"].pop("left")
    coordinator["worker_acceptance"]["right"] = acceptance
    coordinator_path.write_text(json.dumps(coordinator, sort_keys=True), encoding="utf-8")
    return session


def test_pair_session_verifies_pass_end_to_end(tmp_path):
    session = write_verified_pair_session(tmp_path)
    builder.build_index(session)

    report = verify_pair_session.verify_pair_session(session)

    assert report["schema"] == verify_pair_session.SCHEMA
    assert report["status"] == "PASS", report["reasons"]
    for name, section in report["sections"].items():
        assert section["status"] == "PASS", (name, section["reasons"])
    quality = session / "quality"
    assert (quality / "acquisition_timing.json").is_file()
    product_status = json.loads(
        (quality / "product_status.json").read_text(encoding="utf-8")
    )
    assert product_status["acquisition_timing"] == "PASS"
    assert product_status["overall"] == "BLOCKED"


def test_right_pair_session_verifies_role_end_to_end(tmp_path):
    session = convert_verified_pair_session_to_right(write_verified_pair_session(tmp_path))
    builder.build_index(session)

    report = verify_pair_session.verify_pair_session(session)

    assert report["schema"] == verify_pair_session.ROLE_SCHEMA
    assert report["umi_role"] == "right"
    assert report["status"] == "PASS", report["reasons"]
    assert set(report["sections"]) == {
        "provenance", "ego", "right", "joint_gate", "clock_domains",
        "sync_index", "storage",
    }


def test_pair_verification_is_repeatable(tmp_path):
    session = write_verified_pair_session(tmp_path)
    builder.build_index(session)
    first = verify_pair_session.verify_pair_session(session)
    second = verify_pair_session.verify_pair_session(session)
    assert second == first


def test_pair_verify_fails_on_raw_payload_tamper(tmp_path):
    session = write_verified_pair_session(tmp_path)
    builder.build_index(session)
    payload_path = session / "ego" / "ego.ir_left.bin"
    payload = bytearray(payload_path.read_bytes())
    payload[0] ^= 0xFF
    payload_path.write_bytes(bytes(payload))

    report = verify_pair_session.verify_pair_session(session)

    assert report["status"] == "FAIL"
    assert any("storage" in reason for reason in report["reasons"])


def test_pair_verify_fails_when_topology_is_not_single_umi(tmp_path):
    session = write_verified_pair_session(tmp_path)
    builder.build_index(session)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text(encoding="utf-8"))
    coordinator["capture_topology"] = "dual_umi"
    coordinator_path.write_text(
        json.dumps(coordinator, sort_keys=True), encoding="utf-8"
    )

    report = verify_pair_session.verify_pair_session(session)

    assert report["status"] == "FAIL"
    assert any("joint_gate" in reason for reason in report["reasons"])


def test_pair_verify_fails_closed_when_sync_index_missing(tmp_path):
    session = write_verified_pair_session(tmp_path)

    report = verify_pair_session.verify_pair_session(session)

    assert report["status"] == "FAIL"
    assert any("storage" in reason for reason in report["reasons"])
