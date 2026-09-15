import math

import numpy as np
import pytest

from three_device_slam.spatial.alignment import PoseSample
from three_device_slam.spatial.apriltag_alignment import (
    AnchorAcceptance,
    TagMount,
    TagObservationPolicy,
    TagPoseObservation,
    estimate_tag_anchor_window,
    reconcile_anchor,
    tag_observation_to_camera_gripper,
)
from three_device_slam.spatial.se3 import (
    compose,
    invert,
    pose_error,
    transform_from_xyz_rpy,
)


def test_tag_mount_composes_detector_pose_into_gripper_pose():
    mount = TagMount(
        device_id="left",
        family="tag36h11",
        tag_id=0,
        tag_from_gripper=transform_from_xyz_rpy(
            (0.012, -0.006, 0.031), (math.pi, 0.0, math.pi / 2.0)
        ),
        calibration_id="left-tag-mount-v1",
    )
    camera_from_tag = transform_from_xyz_rpy(
        (0.45, 0.18, 0.72), (0.08, -0.22, 0.15)
    )
    observation = TagPoseObservation(
        stamped_timestamp_ns=100,
        detector_completed_ns=190,
        family="tag36h11",
        tag_id=0,
        camera_from_tag=camera_from_tag,
        reprojection_error_px=0.4,
    )

    converted = tag_observation_to_camera_gripper(observation, mount)

    assert converted.timestamp_ns == 100
    assert np.allclose(
        converted.ego_from_umi,
        compose(camera_from_tag, mount.tag_from_gripper),
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("observation_kwargs", "reason"),
    (
        ({"tag_id": 1}, "tag_identity_mismatch"),
        ({"family": "tagStandard41h12"}, "tag_identity_mismatch"),
        ({"reprojection_error_px": 2.1}, "reprojection_error"),
        ({"hamming": 1}, "hamming"),
    ),
)
def test_tag_quality_rejections_are_auditable(observation_kwargs, reason):
    mount = _mount("left")
    observation = _observation(100, mount, **observation_kwargs)

    estimate = estimate_tag_anchor_window(
        _static_samples(0, 200, 10, np.eye(4)),
        _static_samples(0, 200, 10, np.eye(4)),
        mount,
        (observation,),
        policy=TagObservationPolicy(min_inliers=1),
        observation_delay_ns=0,
        max_interpolation_gap_ns=10,
    )

    assert estimate.verdict == "BLOCKED/no_consensus"
    assert estimate.candidate_count == 0
    assert [item.reason for item in estimate.rejections] == [reason]


def test_depth_out_of_range_is_rejected():
    mount = _mount("left")
    observation = _observation(
        100,
        mount,
        camera_from_tag=transform_from_xyz_rpy((0.4, 0.1, -0.2)),
    )

    estimate = estimate_tag_anchor_window(
        _static_samples(0, 200, 10, np.eye(4)),
        _static_samples(0, 200, 10, np.eye(4)),
        mount,
        (observation,),
        policy=TagObservationPolicy(min_inliers=1),
        observation_delay_ns=0,
        max_interpolation_gap_ns=10,
    )

    assert [item.reason for item in estimate.rejections] == ["depth_out_of_range"]


def test_anchor_window_rejects_one_planar_pose_flip_and_recovers_truth():
    mount = _mount("left")
    timestamps = tuple(index * 10_000_000 for index in range(9))
    world_from_camera = tuple(
        transform_from_xyz_rpy(
            (0.02 * index, -0.003 * index, 0.01),
            (0.0, 0.0, 0.015 * index),
        )
        for index in range(len(timestamps))
    )
    world_from_gripper = tuple(
        transform_from_xyz_rpy(
            (0.5 + 0.01 * index, 0.25, 0.14),
            (0.02, -0.01, -0.008 * index),
        )
        for index in range(len(timestamps))
    )
    odom_from_world = transform_from_xyz_rpy(
        (3.2, -1.4, 0.8), (0.12, -0.06, 0.72)
    )
    ego_samples = tuple(
        PoseSample(timestamp, pose)
        for timestamp, pose in zip(timestamps, world_from_camera)
    )
    odom_samples = tuple(
        PoseSample(timestamp, compose(odom_from_world, pose))
        for timestamp, pose in zip(timestamps, world_from_gripper)
    )
    observations = []
    for index in (2, 3, 4, 5, 6):
        camera_from_tag = compose(
            invert(world_from_camera[index]),
            world_from_gripper[index],
            invert(mount.tag_from_gripper),
        )
        if index == 4:
            camera_from_tag = compose(
                camera_from_tag,
                transform_from_xyz_rpy((0.18, 0.0, 0.0), (0.0, math.pi, 0.0)),
            )
        observations.append(
            _observation(
                timestamps[index],
                mount,
                camera_from_tag=camera_from_tag,
            )
        )

    estimate = estimate_tag_anchor_window(
        ego_samples,
        odom_samples,
        mount,
        observations,
        policy=TagObservationPolicy(
            min_inliers=3,
            anchor_translation_threshold_m=0.03,
            anchor_rotation_threshold_deg=3.0,
        ),
        observation_delay_ns=0,
        max_interpolation_gap_ns=10_000_000,
    )

    error = pose_error(estimate.world_from_odom, invert(odom_from_world))
    assert estimate.verdict == "PASS/replay"
    assert estimate.candidate_count == 5
    assert len(estimate.inlier_timestamps_ns) == 4
    assert [item.reason for item in estimate.rejections] == [
        "anchor_consensus_outlier"
    ]
    assert error.translation_m < 1e-10
    assert error.rotation_deg < 1e-8


def test_signed_observation_delay_is_applied_but_detector_latency_is_not():
    mount = _mount("right")
    timestamps = tuple(index * 10_000_000 for index in range(12))
    world_from_camera = tuple(
        transform_from_xyz_rpy((0.03 * index, 0.0, 0.0), (0.0, 0.0, 0.02 * index))
        for index in range(len(timestamps))
    )
    world_from_gripper = tuple(
        transform_from_xyz_rpy((0.55, -0.22 + 0.01 * index, 0.12), (0.0, 0.0, -0.01 * index))
        for index in range(len(timestamps))
    )
    odom_from_world = transform_from_xyz_rpy((-2.0, 4.0, 0.5), (0.1, 0.04, -0.8))
    ego_samples = tuple(PoseSample(t, p) for t, p in zip(timestamps, world_from_camera))
    odom_samples = tuple(
        PoseSample(t, compose(odom_from_world, p))
        for t, p in zip(timestamps, world_from_gripper)
    )
    delay_ns = 20_000_000
    observations = []
    for index in (3, 5, 7, 9):
        camera_from_tag = compose(
            invert(world_from_camera[index]),
            world_from_gripper[index],
            invert(mount.tag_from_gripper),
        )
        stamped = timestamps[index] + delay_ns
        observations.append(
            _observation(
                stamped,
                mount,
                detector_completed_ns=stamped + 750_000_000,
                camera_from_tag=camera_from_tag,
            )
        )

    estimate = estimate_tag_anchor_window(
        ego_samples,
        odom_samples,
        mount,
        observations,
        policy=TagObservationPolicy(min_inliers=3),
        observation_delay_ns=delay_ns,
        max_interpolation_gap_ns=10_000_000,
    )

    error = pose_error(estimate.world_from_odom, invert(odom_from_world))
    assert estimate.verdict == "PASS/replay"
    assert estimate.acquisition_timestamps_ns == tuple(
        timestamps[index] for index in (3, 5, 7, 9)
    )
    assert error.translation_m < 1e-10
    assert error.rotation_deg < 1e-8


def test_negative_observation_delay_moves_an_early_stamp_forward():
    mount = _mount("right")
    timestamps = tuple(index * 10_000_000 for index in range(9))
    world_from_camera = tuple(
        transform_from_xyz_rpy((0.02 * index, 0.0, 0.0))
        for index in range(len(timestamps))
    )
    world_from_gripper = tuple(
        transform_from_xyz_rpy((0.5, -0.2 + 0.01 * index, 0.15))
        for index in range(len(timestamps))
    )
    odom_from_world = transform_from_xyz_rpy((2.0, -1.0, 0.3), (0.0, 0.0, 0.4))
    ego_samples = tuple(PoseSample(t, p) for t, p in zip(timestamps, world_from_camera))
    odom_samples = tuple(
        PoseSample(t, compose(odom_from_world, p))
        for t, p in zip(timestamps, world_from_gripper)
    )
    delay_ns = -20_000_000
    observations = []
    for index in (3, 4, 5):
        stamped = timestamps[index] + delay_ns
        observations.append(
            _observation(
                stamped,
                mount,
                detector_completed_ns=stamped + 100_000_000,
                camera_from_tag=compose(
                    invert(world_from_camera[index]),
                    world_from_gripper[index],
                    invert(mount.tag_from_gripper),
                ),
            )
        )

    estimate = estimate_tag_anchor_window(
        ego_samples,
        odom_samples,
        mount,
        observations,
        policy=TagObservationPolicy(min_inliers=3),
        observation_delay_ns=delay_ns,
        max_interpolation_gap_ns=10_000_000,
    )

    assert estimate.verdict == "PASS/replay"
    assert estimate.acquisition_timestamps_ns == tuple(
        timestamps[index] for index in (3, 4, 5)
    )
    assert np.allclose(estimate.world_from_odom, invert(odom_from_world), atol=1e-10)


def test_one_shot_pose_iterables_are_materialized_for_the_whole_window():
    mount = _mount("left")
    world_from_camera = _static_samples(0, 40, 10, np.eye(4))
    world_from_odom = transform_from_xyz_rpy((0.6, -0.2, 0.1))
    odom_from_world = invert(world_from_odom)
    world_from_gripper = tuple(
        transform_from_xyz_rpy((0.5 + 0.01 * index, 0.2, 0.15))
        for index in range(5)
    )
    odom_samples = tuple(
        PoseSample(timestamp, compose(odom_from_world, gripper))
        for timestamp, gripper in zip(range(0, 41, 10), world_from_gripper)
    )
    observations = tuple(
        _observation(
            timestamp,
            mount,
            camera_from_tag=compose(
                world_from_gripper[index], invert(mount.tag_from_gripper)
            ),
        )
        for index, timestamp in enumerate((10, 20, 30), start=1)
    )

    estimate = estimate_tag_anchor_window(
        iter(world_from_camera),
        iter(odom_samples),
        mount,
        observations,
        policy=TagObservationPolicy(min_inliers=3),
        observation_delay_ns=0,
        max_interpolation_gap_ns=10,
    )

    assert estimate.verdict == "PASS/replay"
    assert estimate.candidate_count == 3
    assert np.allclose(estimate.world_from_odom, world_from_odom, atol=1e-10)


def test_duplicate_acquisition_time_cannot_satisfy_consensus():
    mount = _mount("left")
    observation = _observation(10, mount)

    estimate = estimate_tag_anchor_window(
        _static_samples(0, 20, 10, np.eye(4)),
        _static_samples(0, 20, 10, np.eye(4)),
        mount,
        (observation, observation, observation),
        policy=TagObservationPolicy(min_inliers=3),
        observation_delay_ns=0,
        max_interpolation_gap_ns=10,
    )

    assert estimate.verdict == "BLOCKED/no_consensus"
    assert estimate.candidate_count == 1
    assert [item.reason for item in estimate.rejections] == [
        "duplicate_acquisition_time",
        "duplicate_acquisition_time",
    ]


def test_blocked_consensus_still_reports_the_geometric_outlier():
    mount = _mount("left")
    ego_samples = _static_samples(0, 30, 10, np.eye(4))
    odom_samples = _static_samples(0, 30, 10, np.eye(4))
    good = transform_from_xyz_rpy((0.4, 0.1, 0.8))
    flipped = compose(
        good,
        transform_from_xyz_rpy((0.2, 0.0, 0.0), (0.0, math.pi, 0.0)),
    )

    estimate = estimate_tag_anchor_window(
        ego_samples,
        odom_samples,
        mount,
        (
            _observation(10, mount, camera_from_tag=good),
            _observation(20, mount, camera_from_tag=good),
            _observation(30, mount, camera_from_tag=flipped),
        ),
        policy=TagObservationPolicy(min_inliers=3),
        observation_delay_ns=0,
        max_interpolation_gap_ns=10,
    )

    assert estimate.verdict == "BLOCKED/no_consensus"
    assert len(estimate.inlier_timestamps_ns) == 2
    assert [item.reason for item in estimate.rejections] == [
        "anchor_consensus_outlier"
    ]


def test_left_and_right_anchors_remain_independent():
    world_from_camera = _static_samples(0, 40, 10, np.eye(4))
    left_mount = _mount("left")
    right_mount = _mount("right")
    left_world_from_odom = transform_from_xyz_rpy((1.0, 0.0, 0.0))
    right_world_from_odom = transform_from_xyz_rpy((-1.0, 0.4, 0.2), (0.0, 0.0, 0.2))

    left = _simple_anchor_estimate(
        left_mount, world_from_camera, left_world_from_odom, y=0.25
    )
    right = _simple_anchor_estimate(
        right_mount, world_from_camera, right_world_from_odom, y=-0.25
    )

    assert left.device_id == "left"
    assert right.device_id == "right"
    assert np.allclose(left.world_from_odom, left_world_from_odom, atol=1e-10)
    assert np.allclose(right.world_from_odom, right_world_from_odom, atol=1e-10)
    assert not np.allclose(left.world_from_odom, right.world_from_odom)


def test_relocalization_jump_is_rejected_and_previous_anchor_survives_occlusion():
    mount = _mount("left")
    current = AnchorAcceptance.initial(
        device_id="left",
        world_from_odom=transform_from_xyz_rpy((0.3, -0.1, 0.2)),
        accepted_at_ns=100,
    )
    discontinuous = AnchorAcceptance.initial(
        device_id="left",
        world_from_odom=transform_from_xyz_rpy((0.8, -0.1, 0.2), (0.0, 0.0, 0.4)),
        accepted_at_ns=200,
    )

    retained = reconcile_anchor(
        current,
        discontinuous,
        max_translation_update_m=0.05,
        max_rotation_update_deg=5.0,
    )

    assert retained.accepted is False
    assert retained.reason == "relocalization_jump"
    assert retained.state.accepted_at_ns == 100
    assert np.allclose(retained.state.world_from_odom, current.world_from_odom)


def _mount(device_id: str) -> TagMount:
    return TagMount(
        device_id=device_id,
        family="tag36h11",
        tag_id=0 if device_id == "left" else 1,
        tag_from_gripper=transform_from_xyz_rpy(
            (0.015, 0.0, 0.035), (math.pi, 0.0, 0.0)
        ),
        calibration_id=f"{device_id}-tag-mount-v1",
    )


def _observation(
    timestamp_ns: int,
    mount: TagMount,
    *,
    detector_completed_ns: int | None = None,
    family: str | None = None,
    tag_id: int | None = None,
    camera_from_tag: np.ndarray | None = None,
    reprojection_error_px: float = 0.2,
    hamming: int = 0,
) -> TagPoseObservation:
    return TagPoseObservation(
        stamped_timestamp_ns=timestamp_ns,
        detector_completed_ns=(
            timestamp_ns + 1 if detector_completed_ns is None else detector_completed_ns
        ),
        family=mount.family if family is None else family,
        tag_id=mount.tag_id if tag_id is None else tag_id,
        camera_from_tag=(
            transform_from_xyz_rpy((0.4, 0.1, 0.8))
            if camera_from_tag is None
            else camera_from_tag
        ),
        reprojection_error_px=reprojection_error_px,
        hamming=hamming,
    )


def _static_samples(
    start: int, end: int, step: int, transform: np.ndarray
) -> tuple[PoseSample, ...]:
    return tuple(PoseSample(timestamp, transform) for timestamp in range(start, end + 1, step))


def _simple_anchor_estimate(
    mount: TagMount,
    world_from_camera: tuple[PoseSample, ...],
    world_from_odom: np.ndarray,
    *,
    y: float,
):
    timestamps = tuple(sample.timestamp_ns for sample in world_from_camera)
    world_from_gripper = tuple(
        transform_from_xyz_rpy((0.5 + 0.001 * index, y, 0.15))
        for index in range(len(timestamps))
    )
    odom_from_world = invert(world_from_odom)
    odom_samples = tuple(
        PoseSample(timestamp, compose(odom_from_world, pose))
        for timestamp, pose in zip(timestamps, world_from_gripper)
    )
    observations = tuple(
        _observation(
            timestamp,
            mount,
            camera_from_tag=compose(
                invert(camera.transform),
                gripper,
                invert(mount.tag_from_gripper),
            ),
        )
        for timestamp, camera, gripper in zip(
            timestamps[1:4], world_from_camera[1:4], world_from_gripper[1:4]
        )
    )
    return estimate_tag_anchor_window(
        world_from_camera,
        odom_samples,
        mount,
        observations,
        policy=TagObservationPolicy(min_inliers=3),
        observation_delay_ns=0,
        max_interpolation_gap_ns=10,
    )
