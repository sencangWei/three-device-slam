import csv
import hashlib
import json
from pathlib import Path

import pytest

from three_device_slam.core.model import FrameStamp


GRID_NS = 33_333_333
CSV_HEADER = [
    "sample_ns",
    "ego_sequence",
    "ego_acquisition_ns",
    "ego_ref",
    "left_sequence",
    "left_acquisition_ns",
    "left_ref",
    "right_sequence",
    "right_acquisition_ns",
    "right_ref",
    "span_ns",
    "trainable",
    "reason",
]


def _api():
    from three_device_slam.synchronization.sync_index import build_triplets

    return build_triplets


def _builder():
    from three_device_slam.synchronization import build_index

    return build_index


def _frame(device, sequence, acquisition_ns, payload_ref=None):
    return FrameStamp(
        device,
        sequence,
        acquisition_ns,
        payload_ref if payload_ref is not None else f"{device}:{sequence}",
    )


def _classification_rows(span_ns):
    end_ns = 200_000_000
    return _api()(
        [_frame("ego", 1, 0), _frame("ego", 2, end_ns)],
        [_frame("left", 1, span_ns // 2), _frame("left", 2, end_ns)],
        [_frame("right", 1, span_ns), _frame("right", 2, end_ns)],
    )


@pytest.mark.parametrize(
    ("span_ns", "trainable", "reason"),
    [
        (8_000_000, True, "within_target"),
        (10_000_000, True, "within_target"),
        (10_000_001, True, "within_hard_limit"),
        (16_700_000, True, "within_hard_limit"),
        (16_700_001, False, "span_over_hard_limit"),
    ],
)
def test_span_classification_boundaries_are_inclusive(span_ns, trainable, reason):
    first = _classification_rows(span_ns)[0]

    assert first.span_ns == span_ns
    assert first.trainable is trainable
    assert first.reason == reason


def test_sample_and_span_use_selected_real_timestamps_and_refs():
    rows = _api()(
        [_frame("ego", 1, 100_000_000, "ego-real"), _frame("ego", 2, 200_000_000)],
        [_frame("left", 1, 106_000_000, "left-real"), _frame("left", 2, 200_000_000)],
        [_frame("right", 1, 110_000_000, "right-real"), _frame("right", 2, 200_000_000)],
    )

    first = rows[0]
    assert first.sample_ns == 106_000_000
    assert first.span_ns == 10_000_000
    assert (first.ego.payload_ref, first.left.payload_ref, first.right.payload_ref) == (
        "ego-real",
        "left-real",
        "right-real",
    )


def test_grid_is_anchored_at_inclusive_common_overlap():
    rows = _api()(
        [_frame("ego", 0, 0), _frame("ego", 1, 100_000_000)],
        [_frame("left", 0, 10_000_000), _frame("left", 1, 80_000_000)],
        [_frame("right", 0, 5_000_000), _frame("right", 1, 90_000_000)],
    )

    assert len(rows) == 3
    assert [row.left.sequence for row in rows] == [0, 0, 1]
    assert 10_000_000 + 2 * GRID_NS <= 80_000_000
    assert 10_000_000 + 3 * GRID_NS > 80_000_000


def test_empty_or_nonoverlapping_streams_return_no_triplets():
    build_triplets = _api()
    assert build_triplets([], [_frame("left", 1, 0)], [_frame("right", 1, 0)]) == []
    assert (
        build_triplets(
            [_frame("ego", 1, 0), _frame("ego", 2, 1)],
            [_frame("left", 1, 2), _frame("left", 2, 3)],
            [_frame("right", 1, 0), _frame("right", 2, 4)],
        )
        == []
    )


def test_nearest_match_tie_chooses_earlier_timestamp():
    rows = _api()(
        [_frame("ego", 10, 0), _frame("ego", 11, 10)],
        [_frame("left", 1, 5), _frame("left", 2, 10)],
        [_frame("right", 1, 5), _frame("right", 2, 10)],
    )

    assert rows[0].ego.sequence == 10


def test_equal_timestamp_tie_preserves_earlier_input_order():
    rows = _api()(
        [_frame("ego", 9, 0, "first"), _frame("ego", 1, 0, "second"), _frame("ego", 2, 10)],
        [_frame("left", 1, 5), _frame("left", 2, 10)],
        [_frame("right", 1, 5), _frame("right", 2, 10)],
    )

    assert rows[0].ego.payload_ref == "first"


def test_repeated_selected_triplets_are_retained_deterministically():
    streams = (
        [_frame("ego", 0, 0), _frame("ego", 1, 200_000_000)],
        [_frame("left", 0, 0), _frame("left", 1, 200_000_000)],
        [_frame("right", 0, 0), _frame("right", 1, 200_000_000)],
    )

    first = _api()(*streams)
    second = _api()(*streams)

    assert first == second
    assert len(first) == 7
    selected = [(row.ego.sequence, row.left.sequence, row.right.sequence) for row in first]
    assert selected.count((0, 0, 0)) == 4
    assert selected.count((1, 1, 1)) == 3


def test_timestamp_regression_and_duplicate_sequence_are_rejected_without_sorting():
    build_triplets = _api()
    with pytest.raises(ValueError, match="regression"):
        build_triplets(
            [_frame("ego", 1, 2), _frame("ego", 2, 1)],
            [_frame("left", 1, 0)],
            [_frame("right", 1, 0)],
        )
    with pytest.raises(ValueError, match="duplicate"):
        build_triplets(
            [_frame("ego", 1, 0), _frame("ego", 1, 1)],
            [_frame("left", 1, 0)],
            [_frame("right", 1, 0)],
        )


def test_equal_timestamps_with_distinct_sequences_are_allowed():
    rows = _api()(
        [_frame("ego", 1, 0), _frame("ego", 2, 0)],
        [_frame("left", 1, 0)],
        [_frame("right", 1, 0)],
    )

    assert rows[0].ego.sequence == 1


@pytest.mark.parametrize(
    ("stream_name", "frame"),
    [
        ("ego", _frame("wrong", 1, 0)),
        ("ego", _frame("ego", True, 0)),
        ("ego", _frame("ego", 1.5, 0)),
        ("ego", _frame("ego", -1, 0)),
        ("ego", _frame("ego", 1, True)),
        ("ego", _frame("ego", 1, 1.5)),
        ("ego", _frame("ego", 1, -1)),
        ("ego", _frame("ego", 1, 0, "")),
        ("left", _frame("wrong", 1, 0)),
        ("right", _frame("wrong", 1, 0)),
    ],
)
def test_malformed_frame_values_and_wrong_devices_are_rejected(stream_name, frame):
    streams = {
        "ego": [_frame("ego", 1, 0)],
        "left": [_frame("left", 1, 0)],
        "right": [_frame("right", 1, 0)],
    }
    streams[stream_name] = [frame]

    with pytest.raises(ValueError):
        _api()(streams["ego"], streams["left"], streams["right"])


def test_all_nonempty_inputs_are_validated_even_when_another_stream_is_empty():
    with pytest.raises(ValueError, match="device_id"):
        _api()([], [_frame("wrong", 1, 0)], [_frame("right", 1, 0)])


def _ego_row(sequence, acquisition_ns, *, warmup=False, valid=True, **updates):
    row = {
        "stream_id": "ego.video",
        "sequence": sequence,
        "acquisition_ns": acquisition_ns,
        "arrival_ns": acquisition_ns + 100,
        "clock_domain": "host_monotonic",
        "warmup": warmup,
        "valid": valid,
        "metadata": {},
        "offset": sequence * 1000,
        "size": 1000,
        "crc32": 0,
    }
    row.update(updates)
    return row


def _write_session(session, *, coordinator=None, ego_rows=None, left_rows=None, right_rows=None):
    session.mkdir()
    if coordinator is None:
        coordinator = {
            "schema": "ego.three_device.coordinator.v1",
            "task_start_ego_frame_ns": 10_000_000_000,
            "status": "PASS",
            "git": {"hash": "capture-hash", "dirty": False},
        }
    (session / "coordinator.json").write_text(
        json.dumps(coordinator, sort_keys=True), encoding="utf-8"
    )
    ego_rows = ego_rows or [
        _ego_row(0, 9_900_000_000, warmup=True),
        _ego_row(1, 10_000_000_000),
        _ego_row(2, 10_100_000_000),
    ]
    ego_dir = session / "ego"
    ego_dir.mkdir()
    (ego_dir / "ego.video.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in ego_rows),
        encoding="utf-8",
    )
    default_d405_rows = [
        {"set_index": "0", "arrival_mono": "9.900000000", "warmup": "1"},
        {"set_index": "1", "arrival_mono": "10.004000000", "warmup": "0"},
        {"set_index": "2", "arrival_mono": "10.104000000", "warmup": "0"},
    ]
    for device, rows in (
        ("left", left_rows or default_d405_rows),
        ("right", right_rows or default_d405_rows),
    ):
        device_dir = session / device
        device_dir.mkdir()
        (device_dir / "acceptance.json").write_text(
            json.dumps(
                {
                    "result": "PASS",
                    "camera_clock": {
                        "verified": True,
                        "required_streams": [
                            "color",
                            "infrared_left",
                            "infrared_right",
                        ],
                    },
                    "joint_start": {"clock_domain": "host_monotonic"},
                },
                sort_keys=True,
            ),
            encoding="utf-8",
        )
        with (device_dir / "d405_frames.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            domain_fields = [
                "color_domain",
                "infrared_left_domain",
                "infrared_right_domain",
            ]
            writer = csv.DictWriter(
                stream,
                fieldnames=["set_index", "arrival_mono", "warmup", *domain_fields],
            )
            writer.writeheader()
            writer.writerows(
                [
                    {**row, **{field: row.get(field, "global_time") for field in domain_fields}}
                    for row in rows
                ]
            )
    return session


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def test_session_builder_filters_formal_rows_and_writes_deterministic_hashed_outputs(tmp_path):
    session = _write_session(
        tmp_path / "session",
        ego_rows=[
            _ego_row(0, 9_900_000_000, warmup=True),
            _ego_row(1, 9_950_000_000, valid=False),
            _ego_row(2, 10_000_000_000),
            _ego_row(3, 10_100_000_000),
        ],
    )
    builder = _builder()

    manifest = builder.build_index(session)
    csv_path = session / "sync" / "common_30hz.csv"
    manifest_path = session / "sync" / "manifest.json"
    first_csv = csv_path.read_bytes()
    first_manifest = manifest_path.read_bytes()
    rows = list(csv.DictReader(first_csv.decode("utf-8").splitlines()))
    stored = json.loads(first_manifest)

    assert list(rows[0]) == CSV_HEADER
    assert rows[0]["ego_sequence"] == "2"
    assert rows[0]["ego_acquisition_ns"] == "10000000000"
    assert rows[0]["ego_ref"] == "ego/ego.video.bin:2000:1000"
    assert rows[0]["left_acquisition_ns"] == "10004000000"
    assert rows[0]["left_ref"] == "left/d405_frames.csv:3:1"
    assert rows[0]["right_ref"] == "right/d405_frames.csv:3:1"
    assert stored == manifest
    assert stored["schema"] == "ego.three_device.sync_index.v1"
    assert stored["grid_ns"] == 33_333_333
    assert stored["target_span_ns"] == 10_000_000
    assert stored["hard_span_ns"] == 16_700_000
    assert stored["task_start_ego_frame_ns"] == 10_000_000_000
    assert stored["common_overlap"] == {
        "start_ns": 10_004_000_000,
        "end_ns": 10_100_000_000,
    }
    assert stored["row_count"] == len(rows) == 3
    assert stored["trainable_count"] == 3
    assert stored["reason_counts"] == {
        "span_over_hard_limit": 0,
        "within_hard_limit": 0,
        "within_target": 3,
    }
    expected_sources = {
        relative: _sha256(session / relative)
        for relative in (
            "coordinator.json",
            "ego/ego.video.jsonl",
            "left/acceptance.json",
            "left/d405_frames.csv",
            "right/acceptance.json",
            "right/d405_frames.csv",
        )
    }
    assert stored["source_sha256"] == expected_sources
    assert stored["output_csv_sha256"] == _sha256(csv_path)
    assert set(stored["git"]) == {"hash", "dirty"}
    assert "host_monotonic" in json.dumps(stored["input_config"])
    assert not any(str(session) in key for key in stored["source_sha256"])
    builder.build_index(session)
    assert csv_path.read_bytes() == first_csv
    assert manifest_path.read_bytes() == first_manifest


def test_builder_refuses_unverified_d405_clock_and_leaves_no_manifest(tmp_path):
    session = _write_session(tmp_path / "session")
    for device in ("left", "right"):
        (session / device / "acceptance.json").write_text(
            json.dumps(
                {
                    "result": "BLOCKED" if device == "left" else "PASS",
                    "camera_clock": {"verified": device == "right"},
                    "joint_start": {
                        "clock_domain": "unverified" if device == "left" else "host_monotonic"
                    },
                }
            ),
            encoding="utf-8",
        )

    with pytest.raises(ValueError, match="clock"):
        _builder().build_index(session)

    assert not (session / "sync" / "manifest.json").exists()
    assert not list((session / "sync").glob("*.tmp"))



def test_decimal_arrival_conversion_is_exact_not_binary_float(tmp_path):
    session = _write_session(
        tmp_path / "session",
        left_rows=[
            {"set_index": "1", "arrival_mono": "10.000000001", "warmup": "0"},
            {"set_index": "2", "arrival_mono": "10.100000001", "warmup": "0"},
        ],
        right_rows=[
            {"set_index": "1", "arrival_mono": "10.000000002", "warmup": "0"},
            {"set_index": "2", "arrival_mono": "10.100000002", "warmup": "0"},
        ],
    )

    _builder().build_index(session)
    rows = list(
        csv.DictReader(
            (session / "sync" / "common_30hz.csv").open(
                encoding="utf-8"
            )
        )
    )

    assert rows[0]["left_acquisition_ns"] == "10000000001"
    assert rows[0]["right_acquisition_ns"] == "10000000002"


def test_unavailable_clock_on_invalid_warmup_ego_row_is_skipped(tmp_path):
    session = _write_session(
        tmp_path / "session",
        ego_rows=[
            _ego_row(
                0,
                0,
                warmup=True,
                valid=False,
                clock_domain="gstreamer_timestamp_unavailable",
            ),
            _ego_row(1, 10_000_000_000),
            _ego_row(2, 10_100_000_000),
        ],
    )

    _builder().build_index(session)
    rows = list(
        csv.DictReader(
            (session / "sync" / "common_30hz.csv").open(
                encoding="utf-8"
            )
        )
    )

    assert rows[0]["ego_sequence"] == "1"


def test_unavailable_clock_on_valid_formal_ego_row_is_terminal(tmp_path):
    session = _write_session(
        tmp_path / "session",
        ego_rows=[
            _ego_row(
                1,
                0,
                warmup=False,
                valid=True,
                clock_domain="gstreamer_timestamp_unavailable",
            )
        ],
    )

    with pytest.raises(ValueError, match="clock_domain"):
        _builder().build_index(session)


def test_unknown_clock_domain_on_skipped_ego_row_is_terminal(tmp_path):
    session = _write_session(
        tmp_path / "session",
        ego_rows=[
            _ego_row(0, 0, warmup=True, valid=False, clock_domain="unknown"),
            _ego_row(1, 10_000_000_000),
        ],
    )

    with pytest.raises(ValueError, match="clock_domain"):
        _builder().build_index(session)


def test_host_monotonic_warmup_row_participates_in_ego_regression_check(tmp_path):
    session = _write_session(
        tmp_path / "session",
        ego_rows=[
            _ego_row(0, 200, warmup=True, valid=False),
            _ego_row(1, 100, warmup=False, valid=True),
        ],
    )

    with pytest.raises(ValueError, match="regression"):
        _builder().build_index(session)


@pytest.mark.parametrize(
    "coordinator",
    [
        {},
        {"schema": "wrong", "task_start_ego_frame_ns": 10_000_000_000},
        {
            "schema": "ego.three_device.coordinator.v1",
            "task_start_ego_frame_ns": None,
        },
        {
            "schema": "ego.three_device.coordinator.v1",
            "task_start_ego_frame_ns": True,
        },
        {
            "schema": "ego.three_device.coordinator.v1",
            "task_start_ego_frame_ns": -1,
        },
    ],
)
def test_invalid_or_absent_task_anchor_is_terminal_and_leaves_no_manifest(
    tmp_path, coordinator
):
    session = _write_session(tmp_path / "session", coordinator=coordinator)
    manifest_path = session / "sync" / "manifest.json"
    manifest_path.parent.mkdir(parents=True)
    manifest_path.write_text("stale success", encoding="utf-8")

    with pytest.raises(ValueError):
        _builder().build_index(session)

    assert not manifest_path.exists()


@pytest.mark.parametrize(
    ("ego_rows", "message"),
    [
        ([_ego_row(1, 10_000_000_000, clock_domain="epoch")], "clock_domain"),
        ([_ego_row(True, 10_000_000_000)], "sequence"),
        ([_ego_row(1, 10_000_000_000), _ego_row(1, 10_100_000_000)], "duplicate"),
        ([_ego_row(1, 10_100_000_000), _ego_row(2, 10_000_000_000)], "regression"),
        ([_ego_row(1, 10_000_000_000, warmup=0)], "warmup"),
        ([_ego_row(1, 10_000_000_000, valid=1)], "valid"),
        ([_ego_row(1, 10_000_000_000, offset=True)], "offset"),
        ([_ego_row(1, 10_000_000_000, size=-1)], "size"),
    ],
)
def test_malformed_ego_rows_are_terminal(tmp_path, ego_rows, message):
    session = _write_session(tmp_path / "session", ego_rows=ego_rows)

    with pytest.raises(ValueError, match=message):
        _builder().build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


@pytest.mark.parametrize(
    ("left_rows", "message"),
    [
        ([{"set_index": "x", "arrival_mono": "10", "warmup": "0"}], "set_index"),
        ([{"set_index": "1", "arrival_mono": "NaN", "warmup": "0"}], "arrival_mono"),
        ([{"set_index": "1", "arrival_mono": "10.0000000001", "warmup": "0"}], "nanosecond"),
        ([{"set_index": "1", "arrival_mono": "10", "warmup": "false"}], "warmup"),
        (
            [
                {"set_index": "1", "arrival_mono": "10", "warmup": "0"},
                {"set_index": "1", "arrival_mono": "10.1", "warmup": "0"},
            ],
            "duplicate",
        ),
        (
            [
                {"set_index": "1", "arrival_mono": "10.1", "warmup": "0"},
                {"set_index": "2", "arrival_mono": "10.0", "warmup": "0"},
            ],
            "regression",
        ),
    ],
)
def test_malformed_d405_rows_are_terminal(tmp_path, left_rows, message):
    session = _write_session(tmp_path / "session", left_rows=left_rows)

    with pytest.raises(ValueError, match=message):
        _builder().build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


@pytest.mark.parametrize(
    "csv_text",
    [
        "set_index,arrival_mono,warmup,warmup\n1,10.0,0,0\n2,10.1,0,0\n",
        "set_index,arrival_mono,warmup\n1,10.0,0,surplus\n2,10.1,0,surplus\n",
        "set_index,arrival_mono,warmup,\n1,10.0,0,\n2,10.1,0,\n",
        "set_index,arrival_mono,warmup,named_extra\n1,10.0,0\n2,10.1,0\n",
    ],
    ids=("duplicate-header", "surplus-field", "empty-header", "short-row"),
)
def test_d405_csv_structure_must_be_strict(tmp_path, csv_text):
    session = _write_session(tmp_path / "session")
    (session / "left" / "d405_frames.csv").write_text(csv_text, encoding="utf-8")

    with pytest.raises(ValueError):
        _builder().build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


def test_producer_sixteen_column_d405_csv_is_accepted(tmp_path):
    session = _write_session(tmp_path / "session")
    headers = ["set_index", "arrival_mono", "arrival_wall", "warmup"]
    for stream_id in ("color", "infrared_left", "infrared_right"):
        headers.extend(
            [
                f"{stream_id}_frame_number",
                f"{stream_id}_device_ms",
                f"{stream_id}_mono",
                f"{stream_id}_domain",
            ]
        )
    assert len(headers) == 16
    with (session / "left" / "d405_frames.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=headers)
        writer.writeheader()
        for sequence, arrival_mono in ((1, "10.004000000"), (2, "10.104000000")):
            row = {field: "evidence" for field in headers}
            for field in headers:
                if field.endswith("_domain"):
                    row[field] = "global_time"
            row.update(
                set_index=str(sequence),
                arrival_mono=arrival_mono,
                arrival_wall=arrival_mono,
                warmup="0",
            )
            writer.writerow(row)

    manifest = _builder().build_index(session)

    assert manifest["row_count"] == 3


def test_missing_source_is_terminal_and_leaves_no_success_manifest(tmp_path):
    session = _write_session(tmp_path / "session")
    (session / "right" / "d405_frames.csv").unlink()
    manifest_path = session / "sync" / "manifest.json"
    manifest_path.parent.mkdir(parents=True)
    manifest_path.write_text("stale success", encoding="utf-8")

    with pytest.raises(OSError):
        _builder().build_index(session)

    assert not manifest_path.exists()


def test_source_change_before_commit_is_terminal(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    original = builder._sha256_file
    calls = 0

    def change_source(path):
        nonlocal calls
        calls += 1
        if calls == 1:
            (session / "left" / "d405_frames.csv").write_text(
                "set_index,arrival_mono,warmup\n9,10.0,0\n", encoding="utf-8"
            )
        return original(path)

    monkeypatch.setattr(builder, "_sha256_file", change_source)

    with pytest.raises(RuntimeError, match="changed"):
        builder.build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


def test_source_change_while_temporary_outputs_are_written_is_terminal(
    tmp_path, monkeypatch
):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    original = builder._write_temporary
    calls = 0

    def change_source_after_temp(path, payload):
        nonlocal calls
        temporary = original(path, payload)
        calls += 1
        if calls == 1:
            source = session / "left" / "d405_frames.csv"
            source.write_bytes(source.read_bytes() + b"\n")
        return temporary

    monkeypatch.setattr(builder, "_write_temporary", change_source_after_temp)

    with pytest.raises(RuntimeError, match="changed"):
        builder.build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


def test_source_change_during_csv_commit_never_gets_a_manifest(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    csv_path = session / "sync" / "common_30hz.csv"
    real_replace = builder.os.replace

    def change_source_after_csv_replace(source, target):
        result = real_replace(source, target)
        if Path(target) == csv_path:
            changed = session / "left" / "d405_frames.csv"
            changed.write_bytes(changed.read_bytes() + b"\n")
        return result

    monkeypatch.setattr(builder.os, "replace", change_source_after_csv_replace)

    with pytest.raises(RuntimeError, match="changed"):
        builder.build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


def test_source_change_during_manifest_replace_removes_manifest(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    manifest_path = session / "sync" / "manifest.json"
    real_replace = builder.os.replace

    def change_source_after_manifest_replace(source, target):
        result = real_replace(source, target)
        if Path(target) == manifest_path:
            changed = session / "left" / "d405_frames.csv"
            changed.write_bytes(changed.read_bytes() + b"\n")
        return result

    monkeypatch.setattr(builder.os, "replace", change_source_after_manifest_replace)

    with pytest.raises(RuntimeError, match="changed"):
        builder.build_index(session)

    assert not manifest_path.exists()


def test_committed_csv_change_after_replace_removes_manifest(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    csv_path = session / "sync" / "common_30hz.csv"
    manifest_path = session / "sync" / "manifest.json"
    real_replace = builder.os.replace

    def corrupt_csv_after_replace(source, target):
        result = real_replace(source, target)
        if Path(target) == csv_path:
            csv_path.write_bytes(csv_path.read_bytes() + b"corrupt")
        return result

    monkeypatch.setattr(builder.os, "replace", corrupt_csv_after_replace)

    with pytest.raises(RuntimeError, match="CSV changed"):
        builder.build_index(session)

    assert not manifest_path.exists()


def test_final_verification_detects_earlier_source_mutated_while_hashing_later_source(
    tmp_path, monkeypatch
):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    manifest_path = session / "sync" / "manifest.json"
    right_path = session / "right" / "d405_frames.csv"
    coordinator_path = session / "coordinator.json"
    original = getattr(builder, "_hash_open_file", None)
    mutated = False

    def mutate_after_hashing_right(path, stream):
        nonlocal mutated
        if original is None:
            digest = hashlib.sha256(stream.read()).hexdigest()
        else:
            digest = original(path, stream)
        if Path(path) == right_path and not mutated:
            mutated = True
            coordinator_path.write_bytes(coordinator_path.read_bytes() + b" ")
        return digest

    monkeypatch.setattr(
        builder, "_hash_open_file", mutate_after_hashing_right, raising=False
    )

    with pytest.raises(RuntimeError, match="changed"):
        builder.build_index(session)

    assert mutated
    assert not manifest_path.exists()


def test_publication_lock_is_exclusive(tmp_path):
    builder = _builder()
    output_directory = tmp_path / "sync"
    output_directory.mkdir(parents=True)

    with builder._exclusive_publication_lock(output_directory):
        with pytest.raises(RuntimeError, match="in progress"):
            with builder._exclusive_publication_lock(output_directory):
                raise AssertionError("second builder acquired the publication lock")


def test_manifest_replace_failure_never_leaves_success_manifest(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    manifest_path = session / "sync" / "manifest.json"
    real_replace = builder.os.replace

    def fail_manifest(source, target):
        if Path(target) == manifest_path:
            raise OSError("manifest replace failed")
        return real_replace(source, target)

    monkeypatch.setattr(builder.os, "replace", fail_manifest)

    with pytest.raises(OSError, match="manifest replace"):
        builder.build_index(session)

    assert not manifest_path.exists()


def test_git_provenance_commands_are_rooted_at_repository(monkeypatch):
    builder = _builder()
    calls = []

    class Result:
        def __init__(self, stdout):
            self.stdout = stdout

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return Result("abc123\n" if "rev-parse" in command else "")

    monkeypatch.setattr(builder.subprocess, "run", fake_run)

    assert builder._git_provenance() == {"hash": "abc123", "dirty": False}
    assert calls
    assert all(kwargs["cwd"] == builder.ROOT for _, kwargs in calls)


def test_current_repository_git_provenance_is_available():
    provenance = _builder()._git_provenance()

    assert isinstance(provenance["hash"], str) and provenance["hash"]
    assert isinstance(provenance["dirty"], bool)


def test_wsl_windows_gitdir_selects_git_exe(tmp_path, monkeypatch):
    builder = _builder()
    (tmp_path / ".git").write_text(
        "gitdir: D:/repo/.git/worktrees/three-device\n", encoding="utf-8"
    )
    monkeypatch.setattr(builder, "ROOT", tmp_path)
    monkeypatch.setattr(builder.os, "name", "posix")

    assert builder._git_executable() == "git.exe"


def test_git_provenance_subprocess_failure_is_terminal(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()

    def fail(command, **kwargs):
        raise builder.subprocess.CalledProcessError(1, command)

    monkeypatch.setattr(builder.subprocess, "run", fail)

    with pytest.raises(RuntimeError, match="Git provenance"):
        builder.build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


def test_empty_git_hash_is_terminal(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()

    class Result:
        def __init__(self, stdout):
            self.stdout = stdout

    def empty_hash(command, **kwargs):
        return Result(" \n" if "rev-parse" in command else "")

    monkeypatch.setattr(builder.subprocess, "run", empty_hash)

    with pytest.raises(RuntimeError, match="Git provenance"):
        builder.build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


def test_nonboolean_git_dirty_is_terminal(tmp_path, monkeypatch):
    session = _write_session(tmp_path / "session")
    builder = _builder()
    monkeypatch.setattr(
        builder, "_git_provenance", lambda: {"hash": "abc123", "dirty": 1}
    )

    with pytest.raises(RuntimeError, match="Git provenance"):
        builder.build_index(session)

    assert not (session / "sync" / "manifest.json").exists()


def test_main_returns_nonzero_without_manifest_for_invalid_input(tmp_path):
    session = _write_session(tmp_path / "session")
    (session / "coordinator.json").write_text("{}", encoding="utf-8")

    assert _builder().main(["--session", str(session)]) != 0
    assert not (session / "sync" / "manifest.json").exists()
