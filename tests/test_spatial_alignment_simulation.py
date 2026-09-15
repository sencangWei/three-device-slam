import math

import numpy as np
import pytest

from three_device_slam.spatial.alignment import (
    PoseSample,
    RelativePoseObservation,
    body_pose_from_camera_pose,
    estimate_world_vio_anchor,
    initialize_ego_world,
    propagate_world_trajectory,
    sample_pose_at_time,
)
from three_device_slam.spatial.se3 import (
    compose,
    invert,
    pose_error,
    transform_from_xyz_rpy,
)
from three_device_slam.spatial.simulation import run_alignment_simulation


def test_d405_body_to_camera_extrinsic_recovers_body_pose():
    world_from_body = transform_from_xyz_rpy(
        (1.2, -0.4, 0.8), (0.12, -0.08, 0.35)
    )
    body_from_camera = transform_from_xyz_rpy(
        (0.035, -0.012, 0.018), (0.01, -0.02, 0.005)
    )
    world_from_camera = compose(world_from_body, body_from_camera)

    recovered = body_pose_from_camera_pose(
        world_from_camera, body_from_camera
    )

    assert np.allclose(recovered, world_from_body, atol=1e-12)


def test_ego_world_uses_first_healthy_post_recording_pose_and_gravity():
    source_from_world = transform_from_xyz_rpy(
        (3.0, -1.0, 0.4), (0.18, -0.12, 0.7)
    )
    gravity_world = np.array([0.0, 0.0, -9.81])
    gravity_source = source_from_world[:3, :3] @ gravity_world
    samples = (
        PoseSample(100, compose(source_from_world, transform_from_xyz_rpy((-1, 0, 0)))),
        PoseSample(200, source_from_world, healthy=False),
        PoseSample(300, source_from_world),
        PoseSample(400, compose(source_from_world, transform_from_xyz_rpy((0.1, 0, 0)))),
    )

    initialized = initialize_ego_world(
        samples,
        recording_start_ns=150,
        gravity_in_source=gravity_source,
        forward_axis_in_ego=np.array([1.0, 0.0, 0.0]),
    )

    assert initialized.anchor_timestamp_ns == 300
    anchor_world_from_ego = compose(
        initialized.world_from_source, samples[2].transform
    )
    assert np.allclose(anchor_world_from_ego[:3, 3], 0.0, atol=1e-12)
    assert np.allclose(
        initialized.world_from_source[:3, :3] @ gravity_source,
        gravity_world,
        atol=1e-10,
    )
    assert np.allclose(anchor_world_from_ego[:3, 0], [1.0, 0.0, 0.0], atol=1e-10)


def test_arbitrary_umi_vio_origin_is_anchored_into_ego_world():
    timestamps = (0, 10_000_000, 20_000_000)
    world_from_ego = tuple(
        transform_from_xyz_rpy((0.2 * i, -0.03 * i, 0.0), (0.0, 0.0, 0.08 * i))
        for i in range(3)
    )
    world_from_umi = tuple(
        transform_from_xyz_rpy((0.4 + 0.12 * i, 0.3, 0.15), (0.0, 0.03, -0.04 * i))
        for i in range(3)
    )
    vio_from_world = transform_from_xyz_rpy((7.0, -2.0, 1.5), (0.2, -0.1, 1.1))
    ego_samples = tuple(PoseSample(t, pose) for t, pose in zip(timestamps, world_from_ego))
    vio_samples = tuple(
        PoseSample(t, compose(vio_from_world, pose))
        for t, pose in zip(timestamps, world_from_umi)
    )
    anchor_index = 1
    ego_from_umi = compose(
        invert(world_from_ego[anchor_index]), world_from_umi[anchor_index]
    )
    observation = RelativePoseObservation(timestamps[anchor_index], ego_from_umi)

    world_from_vio = estimate_world_vio_anchor(
        ego_samples,
        vio_samples,
        observation,
        max_interpolation_gap_ns=10_000_000,
    )
    reconstructed = propagate_world_trajectory(world_from_vio, vio_samples)

    assert np.allclose(world_from_vio, invert(vio_from_world), atol=1e-10)
    for recovered, expected in zip(reconstructed, world_from_umi):
        assert np.allclose(recovered.transform, expected, atol=1e-10)


def test_timestamp_delay_creates_error_and_explicit_correction_recovers():
    timestamps = tuple(i * 10_000_000 for i in range(9))
    world_from_ego = tuple(
        transform_from_xyz_rpy((0.03 * i, 0.002 * i, 0.0), (0.0, 0.0, 0.025 * i))
        for i in range(9)
    )
    world_from_umi = tuple(
        transform_from_xyz_rpy((0.6 + 0.01 * i, 0.25, 0.12), (0.0, 0.0, -0.01 * i))
        for i in range(9)
    )
    vio_from_world = transform_from_xyz_rpy((-3.0, 4.0, 0.5), (0.1, 0.05, -0.8))
    ego_samples = tuple(PoseSample(t, p) for t, p in zip(timestamps, world_from_ego))
    vio_samples = tuple(
        PoseSample(t, compose(vio_from_world, p))
        for t, p in zip(timestamps, world_from_umi)
    )
    true_index = 4
    delay_ns = 30_000_000
    relative_at_acquisition = compose(
        invert(world_from_ego[true_index]), world_from_umi[true_index]
    )
    delayed_observation = RelativePoseObservation(
        timestamps[true_index] + delay_ns, relative_at_acquisition
    )

    uncorrected = estimate_world_vio_anchor(
        ego_samples,
        vio_samples,
        delayed_observation,
        max_interpolation_gap_ns=10_000_000,
    )
    corrected = estimate_world_vio_anchor(
        ego_samples,
        vio_samples,
        delayed_observation,
        observation_delay_ns=delay_ns,
        max_interpolation_gap_ns=10_000_000,
    )
    expected = invert(vio_from_world)
    uncorrected_error = pose_error(uncorrected, expected)
    corrected_error = pose_error(corrected, expected)

    assert uncorrected_error.translation_m > 0.05
    assert uncorrected_error.rotation_deg > 1.0
    assert corrected_error.translation_m < 1e-10
    assert corrected_error.rotation_deg < 1e-8


def test_off_grid_acquisition_time_interpolates_translation_and_rotation():
    samples = (
        PoseSample(10, transform_from_xyz_rpy((0.0, 0.0, 0.0))),
        PoseSample(
            110,
            transform_from_xyz_rpy((2.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2.0)),
        ),
    )

    interpolated = sample_pose_at_time(samples, 60, max_gap_ns=100)
    expected = transform_from_xyz_rpy(
        (1.0, 0.0, 0.0), (0.0, 0.0, math.pi / 4.0)
    )

    assert np.allclose(interpolated, expected, atol=1e-10)


def test_interpolation_rejects_large_or_unhealthy_pose_gaps():
    large_gap = (
        PoseSample(10, transform_from_xyz_rpy((0.0, 0.0, 0.0))),
        PoseSample(210, transform_from_xyz_rpy((2.0, 0.0, 0.0))),
    )
    unhealthy_gap = (
        PoseSample(10, transform_from_xyz_rpy((0.0, 0.0, 0.0))),
        PoseSample(110, transform_from_xyz_rpy((1.0, 0.0, 0.0)), healthy=False),
    )

    with pytest.raises(ValueError, match="gap"):
        sample_pose_at_time(large_gap, 60, max_gap_ns=100)
    with pytest.raises(ValueError, match="unhealthy"):
        sample_pose_at_time(unhealthy_gap, 60, max_gap_ns=100)


def test_ego_physical_forward_axis_is_an_explicit_frame_contract():
    identity = transform_from_xyz_rpy((0.0, 0.0, 0.0))
    initialized = initialize_ego_world(
        (PoseSample(10, identity),),
        recording_start_ns=0,
        gravity_in_source=np.array([0.0, 0.0, -9.81]),
        forward_axis_in_ego=np.array([0.0, 1.0, 0.0]),
    )

    world_from_ego = compose(initialized.world_from_source, identity)

    assert np.allclose(
        world_from_ego[:3, :3] @ np.array([0.0, 1.0, 0.0]),
        [1.0, 0.0, 0.0],
        atol=1e-10,
    )


def test_negative_observation_delay_is_applied_with_its_sign():
    timestamps = (10, 20, 30)
    ego_poses = tuple(
        transform_from_xyz_rpy((0.1 * index, 0.0, 0.0))
        for index in range(3)
    )
    umi_poses = tuple(
        transform_from_xyz_rpy((0.5, 0.1 * index, 0.0))
        for index in range(3)
    )
    vio_from_world = transform_from_xyz_rpy((2.0, -1.0, 0.4), (0.0, 0.0, 0.3))
    ego_samples = tuple(PoseSample(t, p) for t, p in zip(timestamps, ego_poses))
    vio_samples = tuple(
        PoseSample(t, compose(vio_from_world, p))
        for t, p in zip(timestamps, umi_poses)
    )
    relative_at_20 = compose(invert(ego_poses[1]), umi_poses[1])
    early_stamp = RelativePoseObservation(10, relative_at_20)

    recovered = estimate_world_vio_anchor(
        ego_samples,
        vio_samples,
        early_stamp,
        observation_delay_ns=-10,
        max_interpolation_gap_ns=10,
    )

    assert np.allclose(recovered, invert(vio_from_world), atol=1e-10)


def test_invalid_rotation_is_rejected_at_the_frame_boundary():
    invalid = np.eye(4)
    invalid[0, 0] = 2.0

    with pytest.raises(ValueError, match="rotation"):
        PoseSample(0, invalid)


def test_simulator_defaults_to_three_devices_and_keeps_chains_independent():
    left_only_diagnostic = run_alignment_simulation(("left",))
    three_device_product = run_alignment_simulation()

    assert three_device_product["verdict"] == "PASS/simulation"
    assert three_device_product["mode"] == "deterministic_simulation"
    assert three_device_product["hil_status"] == "NOT_RUN"
    assert tuple(left_only_diagnostic["devices"]) == ("left",)
    assert tuple(three_device_product["devices"]) == ("left", "right")
    assert left_only_diagnostic["chains"]["left"] == three_device_product["chains"]["left"]
    assert three_device_product["chains"]["left"]["anchor_id"] != three_device_product["chains"]["right"]["anchor_id"]
    for chain in three_device_product["chains"].values():
        assert chain["corrected_translation_rmse_m"] < 1e-9
        assert chain["uncorrected_translation_rmse_m"] > 0.01
        assert chain["timestamp_correction_ns"] != 0
