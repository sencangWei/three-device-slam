import pytest

from three_device_slam.core.model import FrameStamp


GRID_NS = 33_333_333


def _api():
    from three_device_slam.synchronization.sync_index import build_triplets

    return build_triplets


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
