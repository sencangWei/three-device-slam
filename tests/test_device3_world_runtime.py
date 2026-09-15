import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from three_device_slam.spatial.device3_world_runtime import (
    AnchorCandidate,
    Device3WorldAnchorController,
    RuntimeProfileError,
    anchor_candidate_from_synced_observation,
    evaluate_device3_runtime_preflight,
    load_device3_runtime_profile,
    replay_retained_constraints,
)
from three_device_slam.spatial.se3 import compose, invert, transform_from_xyz_rpy


REPOSITORY = Path(__file__).resolve().parents[1]
PROFILE = REPOSITORY / "config/device3_right_world_runtime_pair_20260915.json"
CONSTRAINTS = (
    REPOSITORY
    / "artifacts/spatial_bench/device3_anchor_hil_run24/anchor_hil_v2/anchor_constraints.jsonl"
)
RIGHT_ODOMETRY = (
    REPOSITORY
    / "artifacts/spatial_bench/device3_anchor_hil_run24/right_vins_run/odometry_raw.csv"
)


def candidate(timestamp_ns, xyz=(0.0, 0.0, 0.0), yaw_deg=0.0, gyro=0.2):
    return AnchorCandidate(
        timestamp_ns=timestamp_ns,
        gyro_norm_deg_s=gyro,
        world_from_odom=transform_from_xyz_rpy(
            xyz, (0.0, 0.0, np.deg2rad(yaw_deg))
        ),
    )


def test_accepted_pair_profile_is_hash_bound_and_keeps_group_deferred():
    profile = load_device3_runtime_profile(PROFILE)

    assert profile.profile_id == "DEVICE3_RIGHT_WORLD_RUNTIME_PAIR_V1_20260915"
    assert profile.scope == "ego_plus_right_only"
    assert profile.ego_serial == "327122078613"
    assert profile.right_serial == "260422274454"
    assert profile.mount.device_id == "right"
    assert profile.mount.tag_id == 1
    assert profile.stationary_gyro_max_deg_s == 3.0
    assert profile.mount.calibration_id == "UMI_DEVICE3_RIGHT_TAG_MOUNT_V1_0_0_20260914"
    assert profile.three_device_group_ready is False


def test_profile_loader_rejects_a_changed_calibration_before_runtime(tmp_path):
    source = json.loads(PROFILE.read_text())
    calibration = Path(source["accepted_calibration"]["path"])
    changed = tmp_path / "changed.json"
    changed.write_bytes(calibration.read_bytes() + b"\n")
    source["accepted_calibration"]["path"] = str(changed)
    changed_profile = tmp_path / "profile.json"
    changed_profile.write_text(json.dumps(source))

    with pytest.raises(RuntimeProfileError, match="calibration SHA-256"):
        load_device3_runtime_profile(changed_profile)


def test_controller_requires_stationary_multiframe_consensus_and_composes_world_pose():
    profile = load_device3_runtime_profile(PROFILE)
    controller = Device3WorldAnchorController(profile)

    fast = controller.process_window(
        [candidate(1, gyro=8.0), candidate(2, gyro=9.0), candidate(3, gyro=7.0)]
    )
    assert (fast.status, fast.reason) == ("BLOCKED", "fast_motion")
    single = controller.process_window([candidate(4)])
    assert (single.status, single.reason) == ("BLOCKED", "insufficient_inliers")

    initialized = controller.process_window(
        [
            candidate(10, xyz=(0.100, 0.0, 0.0)),
            candidate(11, xyz=(0.101, 0.0, 0.0)),
            candidate(12, xyz=(0.099, 0.0, 0.0)),
            candidate(13, xyz=(0.4, 0.0, 0.0), yaw_deg=20.0),
        ]
    )
    assert initialized.status == "INITIALIZED"
    assert initialized.inlier_count == 3

    local_body = transform_from_xyz_rpy((0.2, -0.1, 0.05), (0.0, 0.0, 0.1))
    assert np.allclose(
        controller.world_from_body(local_body),
        compose(initialized.world_from_odom, local_body),
    )


def test_synced_observation_builds_the_frozen_runtime_world_chain():
    profile = load_device3_runtime_profile(PROFILE)
    world_from_ego_camera = transform_from_xyz_rpy(
        (0.3, -0.2, 0.1), (0.1, -0.2, 0.3)
    )
    ego_camera_from_tag = transform_from_xyz_rpy(
        (0.2, 0.1, 0.8), (-0.1, 0.05, -0.2)
    )
    odom_from_body = transform_from_xyz_rpy(
        (0.4, -0.1, 0.2), (0.2, 0.1, -0.1)
    )

    result = anchor_candidate_from_synced_observation(
        profile,
        timestamp_ns=123,
        gyro_norm_deg_s=0.5,
        world_from_ego_camera=world_from_ego_camera,
        ego_camera_from_mount_tag=ego_camera_from_tag,
        right_odom_from_body=odom_from_body,
    )
    expected = compose(
        world_from_ego_camera,
        ego_camera_from_tag,
        profile.mount.tag_from_gripper,
        invert(odom_from_body),
    )
    assert result.timestamp_ns == 123
    assert np.allclose(result.world_from_odom, expected)


def test_controller_holds_through_motion_loss_and_bad_relocalization_then_updates():
    controller = Device3WorldAnchorController(load_device3_runtime_profile(PROFILE))
    initial = controller.process_window(
        [candidate(10), candidate(11, xyz=(0.001, 0, 0)), candidate(12)]
    )
    original = initial.world_from_odom.copy()

    moving = controller.process_window(
        [candidate(20, xyz=(0.2, 0, 0), gyro=20.0)]
    )
    missing = controller.process_window([])
    one_frame = controller.process_window([candidate(21, xyz=(0.002, 0, 0))])
    jump = controller.process_window(
        [
            candidate(30, xyz=(0.2, 0, 0)),
            candidate(31, xyz=(0.201, 0, 0)),
            candidate(32, xyz=(0.199, 0, 0)),
        ]
    )
    for decision in (moving, missing, one_frame, jump):
        assert decision.status == "HELD"
        assert np.array_equal(decision.world_from_odom, original)
    assert moving.reason == "fast_motion"
    assert missing.reason == "tag_unavailable"
    assert one_frame.reason == "insufficient_inliers"
    assert jump.reason == "relocalization_jump"

    updated = controller.process_window(
        [
            candidate(40, xyz=(0.008, 0, 0), yaw_deg=0.8),
            candidate(41, xyz=(0.009, 0, 0), yaw_deg=0.9),
            candidate(42, xyz=(0.007, 0, 0), yaw_deg=0.7),
        ]
    )
    assert (updated.status, updated.reason) == ("UPDATED", "accepted")
    assert not np.array_equal(updated.world_from_odom, original)


def test_retained_run24_replay_writes_immutable_pair_only_evidence(tmp_path):
    output = tmp_path / "runtime-replay"
    report = replay_retained_constraints(
        PROFILE, CONSTRAINTS, output, right_odometry_path=RIGHT_ODOMETRY
    )

    assert report["status"] == "PASS_RETAINED_RUNTIME_REPLAY"
    assert report["scope"] == "ego_plus_right_only"
    assert report["three_device_group"] == "DEFERRED"
    assert report["decisions"]["initialization"]["status"] == "INITIALIZED"
    assert report["decisions"]["fast_motion"]["status"] == "HELD"
    assert report["decisions"]["occlusion"]["status"] == "HELD"
    assert report["decisions"]["reacquisition"]["status"] == "UPDATED"
    assert report["endpoint_update"]["translation_m"] <= 0.03
    assert report["endpoint_update"]["rotation_deg"] <= 3.0
    assert report["world_trajectory"]["rows"] == 1328
    assert report["world_trajectory"]["anchor_epochs"] == {
        "initial": 1325,
        "reacquired": 3,
    }
    assert report["world_trajectory"]["anchor_switch_count"] == 1
    assert report["world_trajectory"]["anchor_switch_position_step_m"] <= 0.03
    assert report["world_trajectory"]["max_position_step_m"] <= 0.1
    assert report["checks"]["world_trajectory_continuous"] is True
    trajectory = (output / "right_world_trajectory.csv").read_text().splitlines()
    assert len(trajectory) == 1329
    assert trajectory[0] == (
        "timestamp_ns,x,y,z,qw,qx,qy,qz,anchor_epoch"
    )

    manifest = json.loads((output / "manifest.json").read_text())
    payload = (output / "runtime_replay_report.json").read_bytes()
    assert manifest["files"]["runtime_replay_report.json"] == {
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }
    trajectory_payload = (output / "right_world_trajectory.csv").read_bytes()
    assert manifest["files"]["right_world_trajectory.csv"] == {
        "bytes": len(trajectory_payload),
        "sha256": hashlib.sha256(trajectory_payload).hexdigest(),
    }
    with pytest.raises(FileExistsError):
        replay_retained_constraints(
            PROFILE, CONSTRAINTS, output, right_odometry_path=RIGHT_ODOMETRY
        )


def test_preflight_blocks_usb2_ego_and_passes_the_same_inventory_at_usb3():
    profile = load_device3_runtime_profile(PROFILE)
    inventory = [
        {
            "serial": profile.ego_serial,
            "name": "Intel RealSense D435I",
            "usb_type": "2.1",
            "physical_port": "1-3",
        },
        {
            "serial": profile.right_serial,
            "name": "Intel RealSense D405",
            "usb_type": "3.2",
            "physical_port": "2-3",
        },
    ]
    blocked = evaluate_device3_runtime_preflight(
        profile,
        inventory=inventory,
        imu_available=True,
        free_bytes=20_000_000_000,
        required_free_bytes=15_000_000_000,
        active_holders=(),
    )
    assert blocked["status"] == "BLOCKED_HARDWARE_PREFLIGHT"
    assert blocked["checks"]["ego_usb3"] is False

    inventory[0]["usb_type"] = "3.2"
    ready = evaluate_device3_runtime_preflight(
        profile,
        inventory=inventory,
        imu_available=True,
        free_bytes=20_000_000_000,
        required_free_bytes=15_000_000_000,
        active_holders=(),
    )
    assert ready["status"] == "PASS_READY_FOR_BOUNDED_LIVE_CAPTURE"
    assert all(ready["checks"].values())
