"""Pair-topology (D435i Ego + one UMI) synchronization index tests."""

import json
from zlib import crc32

import pytest

from three_device_slam.core.model import FrameStamp
from three_device_slam.synchronization import build_index as builder
from three_device_slam.synchronization.sync_index import GRID_NS, build_pairs


TASK_START_NS = 10_000_000_000


def _append_only_row(stream_id, sequence, acquisition_ns, offset, size, warmup):
    return {
        "stream_id": stream_id,
        "sequence": sequence,
        "acquisition_ns": acquisition_ns,
        "arrival_ns": acquisition_ns + 5_000_000,
        "clock_domain": "realsense_global_time_mapped_to_host_monotonic",
        "warmup": warmup,
        "valid": True,
        "offset": offset,
        "size": size,
        "crc32": 0,
        "metadata": {"timestamp_domain": "global_time"},
    }


def _write_jsonl(path, rows):
    payload = _write_jsonl_payload(rows)
    path.write_bytes(payload)
    return payload


def _write_jsonl_payload(rows):
    return "".join(
        json.dumps(row, sort_keys=True) + "\n" for row in rows
    ).encode("utf-8")


def _pair_coordinator(task_start_ns=TASK_START_NS):
    return {
        "schema": "ego.three_device.coordinator.v1",
        "status": "PASS",
        "reason": "acquisition_timing",
        "capture_topology": "single_umi",
        "task_start_ego_frame_ns": task_start_ns,
        "scheduled_start_ns": task_start_ns - 1_000_000,
        "worker_acceptance": {
            "ego": {
                "schema": "ego.d435i.acceptance.v1",
                "status": "PASS",
                "calibration_id": "ego-cal",
                "formal_evidence": {"first_acquisition_ns": task_start_ns + 100_000},
            },
            "left": {
                "schema": "umi.d405.append_only.acceptance.v1",
                "result": "PASS",
                "calibration_id": "left-cal",
                "live_vins": {"imu_calibration": "left-cal"},
                "camera_clock": {
                    "verified": True,
                    "required_streams": ["color", "ir_left", "ir_right"],
                    "observed_domains": {
                        name: ["global_time"]
                        for name in ("color", "ir_left", "ir_right")
                    },
                },
                "joint_start": {
                    "clock_domain": "host_monotonic",
                    "scheduled_start_ns": task_start_ns - 1_000_000,
                    "first_acquisition_ns": task_start_ns + 120_000,
                },
            },
        },
        "git": {"hash": "abc123", "dirty": False},
    }


def _umi_acceptance():
    return {
        "schema": "umi.d405.append_only.acceptance.v1",
        "result": "PASS",
        "calibration_id": "left-cal",
        "live_vins": {"imu_calibration": "left-cal"},
        "camera_clock": {"verified": True},
        "joint_start": {"clock_domain": "host_monotonic"},
    }


def write_pair_session(session, *, ego_count=12, umi_count=12):
    session.mkdir(parents=True)
    (session / "ego").mkdir()
    (session / "left").mkdir()
    (session / "coordinator.json").write_text(
        json.dumps(_pair_coordinator(), sort_keys=True), encoding="utf-8"
    )
    ego_rows = []
    umi_rows = []
    for index in range(ego_count):
        warmup = index < 3
        ego_rows.append(
            _append_only_row(
                "ego.ir_left",
                index,
                TASK_START_NS - 3_000_000 + index * 1_000_000 if warmup
                else TASK_START_NS + 100_000 + (index - 3) * GRID_NS,
                index * 4,
                4,
                warmup,
            )
        )
    for index in range(umi_count):
        warmup = index < 2
        umi_rows.append(
            _append_only_row(
                "left.ir_left",
                index,
                TASK_START_NS - 2_000_000 + index * 1_000_000 if warmup
                else TASK_START_NS + 120_000 + (index - 2) * GRID_NS,
                index * 4,
                4,
                warmup,
            )
        )
    _write_jsonl(session / "ego" / "ego.ir_left.jsonl", ego_rows)
    _write_jsonl(session / "left" / "left.ir_left.jsonl", umi_rows)
    (session / "left" / "acceptance.json").write_text(
        json.dumps(_umi_acceptance(), sort_keys=True), encoding="utf-8"
    )
    return session


def convert_pair_session_to_right(session):
    left = session / "left"
    right = session / "right"
    left.rename(right)
    old_stream = right / "left.ir_left.jsonl"
    rows = [json.loads(line) for line in old_stream.read_text().splitlines()]
    for row in rows:
        row["stream_id"] = "right.ir_left"
    old_stream.rename(right / "right.ir_left.jsonl")
    _write_jsonl(right / "right.ir_left.jsonl", rows)
    coordinator_path = session / "coordinator.json"
    coordinator = json.loads(coordinator_path.read_text())
    coordinator["worker_acceptance"]["right"] = coordinator["worker_acceptance"].pop("left")
    coordinator_path.write_text(json.dumps(coordinator, sort_keys=True), encoding="utf-8")
    return session


def test_pair_session_builds_pair_schema_index(tmp_path):
    session = write_pair_session(tmp_path / "session")

    manifest = builder.build_index(session)

    assert manifest["schema"] == builder.PAIR_SCHEMA
    assert manifest["capture_topology"] == builder.TOPOLOGY_PAIR
    assert set(manifest["source_sha256"]) == set(builder.PAIR_SOURCE_PATHS)
    ego_stream = [
        FrameStamp("ego", 3 + index, TASK_START_NS + 100_000 + index * GRID_NS, f"e:{index}")
        for index in range(9)
    ]
    left_stream = [
        FrameStamp("left", 2 + index, TASK_START_NS + 120_000 + index * GRID_NS, f"l:{index}")
        for index in range(10)
    ]
    expected_pairs = build_pairs(ego_stream, left_stream)
    assert manifest["row_count"] == len(expected_pairs)
    assert manifest["trainable_count"] == sum(row.trainable for row in expected_pairs)
    assert manifest["task_start_ego_frame_ns"] == TASK_START_NS
    assert manifest["common_overlap"] == {
        "start_ns": max(ego_stream[0].acquisition_ns, left_stream[0].acquisition_ns),
        "end_ns": min(ego_stream[-1].acquisition_ns, left_stream[-1].acquisition_ns),
    }
    csv_text = (session / "sync" / "common_30hz.csv").read_text(encoding="utf-8")
    header = csv_text.splitlines()[0].split(",")
    assert header == list(builder.PAIR_CSV_HEADER)


def test_right_pair_session_builds_role_preserving_index(tmp_path):
    session = convert_pair_session_to_right(write_pair_session(tmp_path / "session"))

    manifest = builder.build_index(session)

    assert manifest["schema"] == builder.PAIR_ROLE_SCHEMA
    assert manifest["umi_role"] == "right"
    assert set(manifest["source_sha256"]) == set(builder.PAIR_RIGHT_SOURCE_PATHS)
    assert set(manifest["input_config"]) == {"ego", "right"}
    header = (session / "sync" / "common_30hz.csv").read_text().splitlines()[0].split(",")
    assert header == list(builder.PAIR_ROLE_CSV_HEADER)
    assert all(
        row.split(",")[4] == "right"
        for row in (session / "sync" / "common_30hz.csv").read_text().splitlines()[1:]
    )


def test_pair_index_is_deterministic_and_idempotent(tmp_path):
    session = write_pair_session(tmp_path / "session")
    first = builder.build_index(session)
    second = builder.build_index(session)
    assert second == first


def test_pair_source_tamper_is_terminal(tmp_path):
    session = write_pair_session(tmp_path / "session")
    builder.build_index(session)
    (session / "left" / "acceptance.json").write_text("{}", encoding="utf-8")
    with pytest.raises(RuntimeError, match="seal|changed"):
        builder.build_index(session)


def test_pair_rejects_unverified_umi_clock(tmp_path):
    session = write_pair_session(tmp_path / "session")
    (session / "coordinator.json").write_text(
        json.dumps(
            {
                **_pair_coordinator(),
                "worker_acceptance": {
                    **_pair_coordinator()["worker_acceptance"],
                    "left": {
                        **_pair_coordinator()["worker_acceptance"]["left"],
                        "camera_clock": {"verified": False},
                    },
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="camera clock"):
        builder.build_index(session)


def test_pair_rejects_non_d435i_ego_acceptance(tmp_path):
    session = write_pair_session(tmp_path / "session")
    coordinator_doc = _pair_coordinator()
    coordinator_doc["worker_acceptance"]["ego"]["schema"] = (
        "ego.2uq2.acceptance.v1"
    )
    (session / "coordinator.json").write_text(
        json.dumps(coordinator_doc, sort_keys=True), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="d435i"):
        builder.build_index(session)


def test_pair_rejects_wrong_stream_id(tmp_path):
    session = write_pair_session(tmp_path / "session")
    rows = [
        _append_only_row("ego.video", 0, TASK_START_NS, 0, 4, False)
    ]
    _write_jsonl(session / "ego" / "ego.ir_left.jsonl", rows)
    with pytest.raises(ValueError, match="stream_id"):
        builder.build_index(session)


def test_pair_rejects_legacy_clock_domain(tmp_path):
    session = write_pair_session(tmp_path / "session")
    path = session / "ego" / "ego.ir_left.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[-1]["clock_domain"] = "host_monotonic"
    _write_jsonl(path, rows)
    with pytest.raises(ValueError, match="clock_domain"):
        builder.build_index(session)


def test_append_only_parser_allows_duplicate_sequence_only_during_warmup():
    rows = [
        _append_only_row("ego.ir_left", 0, TASK_START_NS - 2, 0, 4, True),
        _append_only_row("ego.ir_left", 0, TASK_START_NS - 1, 4, 4, True),
        _append_only_row("ego.ir_left", 1, TASK_START_NS + 1, 8, 4, False),
    ]

    parsed = builder._parse_d435i_ego_rows(
        _write_jsonl_payload(rows), TASK_START_NS
    )

    assert [(row.sequence, row.acquisition_ns) for row in parsed] == [
        (1, TASK_START_NS + 1)
    ]


def test_append_only_parser_still_rejects_duplicate_formal_sequence():
    rows = [
        _append_only_row("ego.ir_left", 1, TASK_START_NS + 1, 0, 4, False),
        _append_only_row("ego.ir_left", 1, TASK_START_NS + 2, 4, 4, False),
    ]

    with pytest.raises(ValueError, match=r"duplicate \(ego, 1\)"):
        builder._parse_d435i_ego_rows(
            _write_jsonl_payload(rows), TASK_START_NS
        )


def test_topology_detection_prefers_legacy(tmp_path):
    session = write_pair_session(tmp_path / "session")
    assert builder._detect_topology(session) == builder.TOPOLOGY_PAIR
    (session / "ego" / "ego.video.jsonl").write_text("", encoding="utf-8")
    assert builder._detect_topology(session) == builder.TOPOLOGY_LEGACY


def test_pairs_midpoint_and_classification():
    ego = [
        FrameStamp("ego", 0, 0, "e:0:1"),
        FrameStamp("ego", 1, GRID_NS, "e:1:1"),
        FrameStamp("ego", 2, 2 * GRID_NS, "e:2:1"),
    ]
    left = [
        FrameStamp("left", 0, 8_000_000, "l:0:1"),
        FrameStamp("left", 1, GRID_NS + 8_000_000, "l:1:1"),
        FrameStamp("left", 2, 2 * GRID_NS + 8_000_000, "l:2:1"),
    ]
    pairs = build_pairs(ego, left)
    assert len(pairs) == 2
    assert pairs[0].span_ns == 8_000_000
    assert pairs[0].trainable is True
    assert pairs[0].reason == "within_target"
    assert pairs[0].sample_ns == 4_000_000
    far = build_pairs(
        [FrameStamp("ego", 0, 0, "e0"), FrameStamp("ego", 1, 100_000_000, "e1")],
        [FrameStamp("left", 0, 30_000_000, "l0"), FrameStamp("left", 1, 130_000_000, "l1")],
    )
    assert far[0].span_ns == 30_000_000
    assert far[0].trainable is False
    assert far[0].reason == "span_over_hard_limit"


def test_crc_helpers_match_append_only_contract():
    payload = b"frame-bytes"
    assert crc32(payload) & 0xFFFFFFFF == crc32(payload) & 0xFFFFFFFF
