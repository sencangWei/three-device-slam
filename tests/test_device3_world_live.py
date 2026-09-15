import numpy as np
import pytest
from types import SimpleNamespace

from scripts.device3_world_live_pair import (
    _assert_ego_anchor_epoch_valid,
    _board_pose_is_live_usable,
    _nearest_timestamp_metrics,
    _orb_local_from_body,
    _rolling_position_metrics,
)
from scripts.rerun_world_pair_viewer import _message_image
from three_device_slam.spatial.device3_world_live import (
    AnchorCandidateWindow,
    TimedTransform,
    matrix_from_pose_components,
    nearest_transform,
    pose_components_from_matrix,
)
from three_device_slam.spatial.device3_world_runtime import AnchorCandidate
from three_device_slam.spatial.se3 import transform_from_xyz_rpy


def test_ros_pose_component_roundtrip_preserves_transform():
    source = matrix_from_pose_components(
        (0.2, -0.1, 0.4), (0.1, -0.2, 0.3, 0.9)
    )
    xyz, xyzw = pose_components_from_matrix(source)
    rebuilt = matrix_from_pose_components(xyz, xyzw)
    assert np.allclose(rebuilt, source, atol=1e-10)
    assert np.isclose(np.linalg.norm(xyzw), 1.0)


def test_pose_component_loader_rejects_zero_quaternion():
    with pytest.raises(ValueError, match="quaternion"):
        matrix_from_pose_components((0, 0, 0), (0, 0, 0, 0))


def test_orb_camera_pose_converts_to_body_and_reconstructs_camera():
    local_from_camera = transform_from_xyz_rpy((1.2, -0.4, 0.8), (0.1, -0.2, 0.3))
    body_from_camera = transform_from_xyz_rpy((0.03, 0.01, -0.02), (0.0, 0.0, 0.1))

    local_from_body = _orb_local_from_body(local_from_camera, body_from_camera)

    assert np.allclose(local_from_body @ body_from_camera, local_from_camera, atol=1e-10)


def test_world_anchor_fails_closed_when_orb_map_readiness_is_lost():
    _assert_ego_anchor_epoch_valid(False, False)
    _assert_ego_anchor_epoch_valid(True, True)
    with pytest.raises(RuntimeError, match="旧世界变换已作废"):
        _assert_ego_anchor_epoch_valid(True, False)


def test_nearest_timestamp_metrics_uses_only_overlap_and_both_neighbors():
    result = _nearest_timestamp_metrics(
        [0, 10_000_000, 20_000_000, 30_000_000],
        [8_000_000, 18_000_000, 28_000_000],
    )

    assert result["samples"] == 2
    assert result["p50_ms"] == pytest.approx(2.0)
    assert result["p95_ms"] == pytest.approx(2.0)
    assert result["max_ms"] == pytest.approx(2.0)


def test_nearest_timestamp_metrics_reports_empty_nonoverlap():
    result = _nearest_timestamp_metrics([0, 1], [10, 11])

    assert result == {
        "samples": 0,
        "p50_ms": None,
        "p95_ms": None,
        "max_ms": None,
    }


def test_rolling_position_metrics_reports_recent_motion_only():
    samples = [
        TimedTransform(0, transform_from_xyz_rpy((9.0, 0.0, 0.0))),
        TimedTransform(6_000_000_000, transform_from_xyz_rpy((0.0, 0.0, 0.0))),
        TimedTransform(10_000_000_000, transform_from_xyz_rpy((0.03, 0.04, 0.0))),
    ]

    result = _rolling_position_metrics(samples, 5.0)

    assert result["samples"] == 2
    assert result["endpoint_delta_m"] == pytest.approx(0.05)


def test_world_viewer_decodes_padded_mono8_ros_image():
    message = SimpleNamespace(
        data=bytes((1, 2, 99, 3, 4, 99)),
        encoding="mono8",
        height=2,
        width=2,
        step=3,
    )

    image = _message_image(message)

    assert image.tolist() == [[1, 2], [3, 4]]
    assert image.flags.c_contiguous


def test_nearest_transform_enforces_maximum_age():
    samples = (
        TimedTransform(1_000_000_000, transform_from_xyz_rpy((1, 0, 0))),
        TimedTransform(1_100_000_000, transform_from_xyz_rpy((2, 0, 0))),
    )
    result = nearest_transform(samples, 1_080_000_000, max_delta_ns=30_000_000)
    assert result.timestamp_ns == 1_100_000_000
    with pytest.raises(ValueError, match="stale"):
        nearest_transform(samples, 1_200_000_000, max_delta_ns=30_000_000)


def test_candidate_window_emits_nonoverlapping_multiframe_batches():
    window = AnchorCandidateWindow(window_ns=200_000_000, maximum_candidates=20)

    def item(stamp):
        return AnchorCandidate(
            timestamp_ns=stamp,
            gyro_norm_deg_s=0.2,
            world_from_odom=transform_from_xyz_rpy((stamp / 1e12, 0, 0)),
        )

    assert window.add(item(1_000_000_000)) is None
    assert window.add(item(1_100_000_000)) is None
    batch = window.add(item(1_200_000_000))
    assert tuple(value.timestamp_ns for value in batch) == (
        1_000_000_000,
        1_100_000_000,
        1_200_000_000,
    )
    assert window.pending_count == 0
    assert window.add(item(1_300_000_000)) is None


def test_candidate_window_rejects_timestamp_regression():
    window = AnchorCandidateWindow(window_ns=200_000_000)
    candidate = AnchorCandidate(
        timestamp_ns=10,
        gyro_norm_deg_s=0.0,
        world_from_odom=np.eye(4),
    )
    window.add(candidate)
    with pytest.raises(ValueError, match="strictly increasing"):
        window.add(candidate)


def test_live_ego_board_pose_requires_spatially_distributed_tags():
    accepted = {tag_id: None for tag_id in (0, 1, 2, 6, 12, 14)}
    one_row = {tag_id: None for tag_id in range(6)}

    assert _board_pose_is_live_usable(accepted, 0.8)
    assert not _board_pose_is_live_usable(one_row, 0.8)
    assert not _board_pose_is_live_usable(accepted, 3.01)
    assert not _board_pose_is_live_usable(accepted, float("nan"))
