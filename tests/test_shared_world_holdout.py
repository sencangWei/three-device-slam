import pytest

from three_device_slam.spatial.shared_world_holdout import (
    motion_phases,
    select_phase_stratified_pairs,
)


def test_holdout_uses_capture_tolerance_and_covers_all_motion_phases():
    second = 1_000_000_000
    ego = list(range(0, 60 * second, second))
    # Later camera phase exceeds the old, erroneous 10 ms private cutoff while
    # remaining inside the accepted 20 ms acquisition contract.
    right = [stamp + (6_000_000 if stamp < 10 * second else 15_000_000)
             for stamp in ego]

    selected, counts = select_phase_stratified_pairs(
        ego, right,
        formal_start_ns=0,
        formal_end_ns=60 * second,
        max_pair_delta_ns=20_000_000,
        keyframe_separation_ns=0,
        pairs_per_phase=3,
    )

    assert counts == {
        "initial_static": 10,
        "translation": 20,
        "rotation_return": 20,
        "final_static": 10,
    }
    assert {pair.phase for pair in selected} == {
        "initial_static", "translation", "rotation_return", "final_static"
    }
    assert all(abs(pair.right_stamp_ns - pair.ego_stamp_ns) <= 20_000_000
               for pair in selected)


def test_holdout_rejects_invalid_timestamp_contract():
    with pytest.raises(ValueError, match="strictly increasing"):
        select_phase_stratified_pairs(
            [1, 1], [1, 2], formal_start_ns=0,
            formal_end_ns=60_000_000_000, max_pair_delta_ns=20_000_000)


def test_motion_phase_boundaries_match_capture_ui():
    assert motion_phases(100, 60_000_000_100) == (
        ("initial_static", 100, 10_000_000_100),
        ("translation", 10_000_000_100, 30_000_000_100),
        ("rotation_return", 30_000_000_100, 50_000_000_100),
        ("final_static", 50_000_000_100, 60_000_000_100),
    )
