"""Export a deterministic Ego-world + dual-UMI AprilTag DEMO session."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .alignment import PoseSample, propagate_world_trajectory
from .apriltag_alignment import (
    AnchorAcceptance,
    TagMount,
    TagObservationPolicy,
    TagPoseObservation,
    estimate_tag_anchor_window,
    reconcile_anchor,
)
from .se3 import compose, invert, pose_error, transform_from_xyz_rpy, validate_transform


_SCHEMA = "three-device-slam.apriltag-demo.v1"
_FAMILY = "tag36h11"
_TAG_SIZE_M = 0.04
_DEVICES = ("left", "right")
_BASE_CAMERA_ROTATION = np.array(
    ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    dtype=np.float64,
)


def export_apriltag_demo(
    output: Path | str,
    *,
    duration_s: float = 30.0,
    fps: int = 30,
    render_video: bool = True,
) -> dict:
    """Write one immutable simulation session and return its quality report."""
    output_path = Path(output)
    if output_path.exists():
        raise FileExistsError(f"DEMO output already exists: {output_path}")
    if (
        isinstance(duration_s, bool)
        or not isinstance(duration_s, (int, float))
        or not math.isfinite(float(duration_s))
        or float(duration_s) <= 0.0
    ):
        raise ValueError("duration_s must be positive and finite")
    if isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0:
        raise ValueError("fps must be a positive integer")
    sample_count = int(round(float(duration_s) * fps))
    if sample_count < 6:
        raise ValueError("DEMO timeline must contain at least six samples")

    (output_path / "calibration").mkdir(parents=True)
    (output_path / "derived").mkdir()
    (output_path / "quality").mkdir()
    (output_path / "exports").mkdir()

    timestamps_ns = tuple(round(index * 1_000_000_000 / fps) for index in range(sample_count))
    times_s = tuple(timestamp / 1e9 for timestamp in timestamps_ns)
    world_from_camera = tuple(_ego_camera_pose(time_s) for time_s in times_s)
    ego_samples = tuple(
        PoseSample(timestamp, pose)
        for timestamp, pose in zip(timestamps_ns, world_from_camera)
    )
    mounts = {device_id: _mount(device_id) for device_id in _DEVICES}
    chains = {
        device_id: _build_chain(
            device_id,
            timestamps_ns,
            times_s,
            ego_samples,
            mounts[device_id],
            fps=fps,
        )
        for device_id in _DEVICES
    }

    calibration_payload = {
        "schema": "three-device-slam.apriltag-mounts.v1",
        "scope": "simulation_only_not_installation_calibration",
        "frame_convention": "T_A_B maps coordinates from B into A",
        "tag_size_definition": "effective black square between detection corners",
        "mounts": [
            {
                "device_id": mount.device_id,
                "family": mount.family,
                "tag_id": mount.tag_id,
                "effective_tag_size_m": _TAG_SIZE_M,
                "calibration_id": mount.calibration_id,
                "T_tag_gripper": _pose_payload(mount.tag_from_gripper),
            }
            for mount in mounts.values()
        ],
    }
    _write_json(output_path / "calibration/tag_mounts.json", calibration_payload)

    tag_rows = []
    for chain in chains.values():
        tag_rows.extend(chain["tag_rows"])
    tag_rows.sort(key=lambda row: (row["timestamp_ns"], row["device_id"]))
    _write_jsonl(output_path / "derived/tag_observations.jsonl", tag_rows)

    trajectory_rows = []
    for index, timestamp_ns in enumerate(timestamps_ns):
        device_rows = {
            "ego": {
                "pose_source": "ego_slam_simulation",
                "T_world_device": _pose_payload(world_from_camera[index]),
                "quality": "VALID/simulation",
            }
        }
        for device_id in _DEVICES:
            chain = chains[device_id]
            visible = chain["visibility"][index]
            device_rows[device_id] = {
                "pose_source": "fused_tag_vio" if visible else "umi_vio_bridge",
                "T_world_device": _pose_payload(
                    chain["world_trajectory"][index].transform
                ),
                "quality": "VALID/simulation",
                "tag_visible": visible,
            }
        trajectory_rows.append(
            {
                "schema": "three-device-slam.ego-world-trajectory-row.v1",
                "timestamp_ns": timestamp_ns,
                "devices": ["ego", "left", "right"],
                "poses": device_rows,
            }
        )
    _write_jsonl(
        output_path / "derived/trajectories_ego_world.jsonl", trajectory_rows
    )

    video_path = output_path / "exports/demo_overlay.mp4"
    if render_video:
        _render_video(video_path, trajectory_rows, fps=fps)
        video_result = {
            "status": "PASS/simulation",
            "path": "exports/demo_overlay.mp4",
            "fps": fps,
            "frames": sample_count,
            "label": "SIMULATION DEMO",
        }
    else:
        video_result = {"status": "SKIPPED/test_or_operator_choice"}

    report = {
        "schema": "three-device-slam.apriltag-demo-acceptance.v1",
        "mode": "deterministic_simulation",
        "verdict": "PASS/simulation",
        "hil_status": "NOT_RUN",
        "world_frame": "Ego SLAM world W; tags do not define or reset W",
        "frame_convention": "T_A_B maps coordinates from B into A",
        "timeline": {
            "duration_s": float(duration_s),
            "fps": fps,
            "samples": sample_count,
            "first_timestamp_ns": timestamps_ns[0],
            "last_timestamp_ns": timestamps_ns[-1],
        },
        "chains": {
            device_id: chains[device_id]["report"] for device_id in _DEVICES
        },
        "video": video_result,
        "limitations": [
            "All images, poses, reprojection errors, and timing are synthetic.",
            "No camera, IMU, VIO, SLAM, detector, or HIL accuracy is claimed.",
            "T_tag_gripper values are simulation fixtures, not measured installation extrinsics.",
        ],
    }
    _write_json(output_path / "quality/alignment_report.json", report)

    evidence_paths = (
        "calibration/tag_mounts.json",
        "derived/tag_observations.jsonl",
        "derived/trajectories_ego_world.jsonl",
        "quality/alignment_report.json",
    ) + (("exports/demo_overlay.mp4",) if render_video else ())
    manifest = {
        "schema": _SCHEMA,
        "mode": "deterministic_simulation",
        "verdict": "PASS/simulation",
        "hil_status": "NOT_RUN",
        "generator_sources": _source_hashes(),
        "files": {
            relative_path: _file_evidence(output_path / relative_path)
            for relative_path in evidence_paths
        },
    }
    _write_json(output_path / "manifest.json", manifest)
    return report


def _build_chain(
    device_id: str,
    timestamps_ns: tuple[int, ...],
    times_s: tuple[float, ...],
    ego_samples: tuple[PoseSample, ...],
    mount: TagMount,
    *,
    fps: int,
) -> dict:
    world_truth = tuple(_gripper_pose(device_id, time_s) for time_s in times_s)
    odom_from_world = (
        transform_from_xyz_rpy((2.7, -1.1, 0.5), (0.08, -0.04, 0.65))
        if device_id == "left"
        else transform_from_xyz_rpy((-2.2, 3.4, 0.7), (-0.06, 0.03, -0.72))
    )
    world_from_odom_truth = invert(odom_from_world)
    odom_samples = tuple(
        PoseSample(timestamp, compose(odom_from_world, pose))
        for timestamp, pose in zip(timestamps_ns, world_truth)
    )

    maximum_index = min(len(timestamps_ns) - 1, max(5, 2 * fps - 1))
    initial_indices = tuple(
        max(1, round(value))
        for value in np.linspace(1, maximum_index, num=5)
    )
    initial_indices = tuple(dict.fromkeys(initial_indices))
    if len(initial_indices) < 5:
        initial_indices = (1, 2, 3, 4, 5)
    flip_index = initial_indices[2]
    initial_observations = []
    special_observations: dict[int, TagPoseObservation] = {}
    special_status: dict[int, str] = {}
    for index in initial_indices:
        camera_from_tag = _camera_from_tag(
            ego_samples[index].transform, world_truth[index], mount
        )
        if index == flip_index:
            camera_from_tag = compose(
                camera_from_tag,
                transform_from_xyz_rpy(
                    (0.18, 0.0, 0.0), (0.0, math.pi, 0.0)
                ),
            )
            special_status[index] = "REJECTED/anchor_consensus_outlier"
        else:
            special_status[index] = "ACCEPTED/anchor_consensus_inlier"
        observation = TagPoseObservation(
            stamped_timestamp_ns=timestamps_ns[index],
            detector_completed_ns=timestamps_ns[index] + 18_000_000,
            family=mount.family,
            tag_id=mount.tag_id,
            camera_from_tag=camera_from_tag,
            reprojection_error_px=0.35,
            hamming=0,
        )
        initial_observations.append(observation)
        special_observations[index] = observation

    estimate = estimate_tag_anchor_window(
        ego_samples,
        odom_samples,
        mount,
        initial_observations,
        policy=TagObservationPolicy(
            min_inliers=3,
            anchor_translation_threshold_m=0.03,
            anchor_rotation_threshold_deg=3.0,
        ),
        observation_delay_ns=0,
        max_interpolation_gap_ns=max(1, round(1_500_000_000 / fps)),
    )
    if estimate.verdict != "PASS/replay" or estimate.world_from_odom is None:
        raise RuntimeError(f"{device_id} synthetic anchor did not reach consensus")
    current_anchor = AnchorAcceptance.initial(
        device_id=device_id,
        world_from_odom=estimate.world_from_odom,
        accepted_at_ns=max(estimate.acquisition_timestamps_ns),
    )
    visibility = tuple(_tag_visible(device_id, time_s) for time_s in times_s)
    occlusion_end_s = 25.0 if device_id == "left" else 28.0
    reacquisition_estimate = None
    reconciliation = None
    reacquisition_indices: tuple[int, ...] = ()
    first_reacquired_index = next(
        (
            index
            for index, (visible, time_s) in enumerate(zip(visibility, times_s))
            if visible and time_s >= occlusion_end_s
        ),
        None,
    )
    if first_reacquired_index is not None:
        last_reacquisition_index = min(
            len(timestamps_ns) - 1,
            first_reacquired_index + max(4, round(0.5 * fps)),
        )
        reacquisition_indices = tuple(
            dict.fromkeys(
                max(first_reacquired_index, round(value))
                for value in np.linspace(
                    first_reacquired_index,
                    last_reacquisition_index,
                    num=5,
                )
            )
        )
        if len(reacquisition_indices) == 5:
            reacquisition_observations = []
            reacquisition_flip_index = reacquisition_indices[2]
            for index in reacquisition_indices:
                camera_from_tag = _camera_from_tag(
                    ego_samples[index].transform, world_truth[index], mount
                )
                if index == reacquisition_flip_index:
                    camera_from_tag = compose(
                        camera_from_tag,
                        transform_from_xyz_rpy(
                            (0.18, 0.0, 0.0), (0.0, math.pi, 0.0)
                        ),
                    )
                    special_status[index] = "REJECTED/anchor_consensus_outlier"
                else:
                    special_status[index] = (
                        "ACCEPTED/reacquisition_consensus_inlier"
                    )
                observation = TagPoseObservation(
                    stamped_timestamp_ns=timestamps_ns[index],
                    detector_completed_ns=timestamps_ns[index] + 18_000_000,
                    family=mount.family,
                    tag_id=mount.tag_id,
                    camera_from_tag=camera_from_tag,
                    reprojection_error_px=0.35,
                    hamming=0,
                )
                reacquisition_observations.append(observation)
                special_observations[index] = observation
            reacquisition_estimate = estimate_tag_anchor_window(
                ego_samples,
                odom_samples,
                mount,
                reacquisition_observations,
                policy=TagObservationPolicy(
                    min_inliers=3,
                    anchor_translation_threshold_m=0.03,
                    anchor_rotation_threshold_deg=3.0,
                ),
                observation_delay_ns=0,
                max_interpolation_gap_ns=max(
                    1, round(1_500_000_000 / fps)
                ),
            )
            if (
                reacquisition_estimate.verdict == "PASS/replay"
                and reacquisition_estimate.world_from_odom is not None
            ):
                candidate_anchor = AnchorAcceptance.initial(
                    device_id=device_id,
                    world_from_odom=reacquisition_estimate.world_from_odom,
                    accepted_at_ns=max(
                        reacquisition_estimate.acquisition_timestamps_ns
                    ),
                )
                reconciliation = reconcile_anchor(
                    current_anchor,
                    candidate_anchor,
                    max_translation_update_m=0.05,
                    max_rotation_update_deg=5.0,
                )

    initial_trajectory = propagate_world_trajectory(
        current_anchor.world_from_odom, odom_samples
    )
    if reconciliation is not None and reconciliation.accepted:
        reacquired_trajectory = propagate_world_trajectory(
            reconciliation.state.world_from_odom, odom_samples
        )
        switch_timestamp_ns = reconciliation.state.accepted_at_ns
        world_trajectory = tuple(
            after if after.timestamp_ns >= switch_timestamp_ns else before
            for before, after in zip(initial_trajectory, reacquired_trajectory)
        )
    else:
        world_trajectory = initial_trajectory
    anchor_error = pose_error(estimate.world_from_odom, world_from_odom_truth)

    detection_stride = max(1, fps // 10)
    tag_rows = []
    detection_indices = sorted(
        set(range(0, len(timestamps_ns), detection_stride))
        | set(special_observations)
    )
    for index in detection_indices:
        if not visibility[index]:
            continue
        observation = special_observations.get(index)
        if observation is None:
            observation = TagPoseObservation(
                stamped_timestamp_ns=timestamps_ns[index],
                detector_completed_ns=timestamps_ns[index] + 18_000_000,
                family=mount.family,
                tag_id=mount.tag_id,
                camera_from_tag=_camera_from_tag(
                    ego_samples[index].transform, world_truth[index], mount
                ),
                reprojection_error_px=0.35
                + 0.05 * abs(math.sin(times_s[index])),
                hamming=0,
            )
        tag_rows.append(
            {
                "schema": "three-device-slam.apriltag-observation-row.v1",
                "timestamp_ns": observation.stamped_timestamp_ns,
                "acquisition_timestamp_ns": observation.stamped_timestamp_ns,
                "detector_completed_ns": observation.detector_completed_ns,
                "detector_latency_ns": observation.detector_latency_ns,
                "device_id": device_id,
                "family": mount.family,
                "tag_id": mount.tag_id,
                "calibration_id": mount.calibration_id,
                "effective_tag_size_m": _TAG_SIZE_M,
                "T_camera_tag": _pose_payload(observation.camera_from_tag),
                "reprojection_error_px": observation.reprojection_error_px,
                "hamming": observation.hamming,
                "quality": special_status.get(index, "VISIBLE/simulation"),
            }
        )

    bridge_samples = sum(not item for item in visibility)
    reacquired = reconciliation is not None and reconciliation.accepted
    reacquisition_outliers = (
        sum(
            item.reason == "anchor_consensus_outlier"
            for item in reacquisition_estimate.rejections
        )
        if reacquisition_estimate is not None
        else 0
    )
    return {
        "world_trajectory": world_trajectory,
        "visibility": visibility,
        "tag_rows": tag_rows,
        "report": {
            "tag_family": mount.family,
            "tag_id": mount.tag_id,
            "mount_calibration_id": mount.calibration_id,
            "anchor_verdict": estimate.verdict,
            "anchor_candidates": estimate.candidate_count,
            "anchor_inliers": len(estimate.inlier_timestamps_ns),
            "initial_consensus_outliers": sum(
                item.reason == "anchor_consensus_outlier"
                for item in estimate.rejections
            ),
            "anchor_translation_error_m": anchor_error.translation_m,
            "anchor_rotation_error_deg": anchor_error.rotation_deg,
            "vio_bridge_samples": bridge_samples,
            "reacquired_after_occlusion": reacquired,
            "reacquisition_anchor_verdict": (
                reacquisition_estimate.verdict
                if reacquisition_estimate is not None
                else "NOT_RUN/timeline_short"
            ),
            "reacquisition_anchor_accepted": reacquired,
            "reacquisition_candidates": (
                reacquisition_estimate.candidate_count
                if reacquisition_estimate is not None
                else 0
            ),
            "reacquisition_inliers": (
                len(reacquisition_estimate.inlier_timestamps_ns)
                if reacquisition_estimate is not None
                else 0
            ),
            "reacquisition_consensus_outliers": reacquisition_outliers,
        },
    }


def _mount(device_id: str) -> TagMount:
    if device_id not in _DEVICES:
        raise ValueError(f"unknown UMI device: {device_id}")
    side = 1.0 if device_id == "left" else -1.0
    return TagMount(
        device_id=device_id,
        family=_FAMILY,
        tag_id=0 if device_id == "left" else 1,
        tag_from_gripper=transform_from_xyz_rpy(
            (0.012, side * 0.004, 0.034), (math.pi, 0.0, side * 0.04)
        ),
        calibration_id=f"simulation-{device_id}-tag-mount-v1",
    )


def _ego_camera_pose(time_s: float) -> np.ndarray:
    yaw = 0.025 * math.sin(0.35 * time_s)
    world_from_camera = np.eye(4, dtype=np.float64)
    world_from_camera[:3, :3] = transform_from_xyz_rpy(
        (0.0, 0.0, 0.0), (0.0, 0.0, yaw)
    )[:3, :3] @ _BASE_CAMERA_ROTATION
    world_from_camera[:3, 3] = (
        0.05 * math.sin(0.22 * time_s),
        0.03 * math.sin(0.31 * time_s),
        1.55 + 0.015 * math.sin(0.4 * time_s),
    )
    return validate_transform(world_from_camera)


def _gripper_pose(device_id: str, time_s: float) -> np.ndarray:
    side = 1.0 if device_id == "left" else -1.0
    if time_s < 5.0:
        motion = 0.0
    elif time_s < 12.0:
        motion = (time_s - 5.0) / 7.0 if device_id == "left" else 0.0
    elif time_s < 19.0:
        motion = 1.0 if device_id == "left" else (time_s - 12.0) / 7.0
    else:
        motion = 1.0 + 0.35 * math.sin(0.8 * (time_s - 19.0) + side)
    return transform_from_xyz_rpy(
        (
            0.62 + 0.10 * motion,
            side * (0.27 - 0.06 * motion),
            1.13 + 0.07 * math.sin(0.6 * time_s + side),
        ),
        (
            0.08 * math.sin(0.5 * time_s),
            side * 0.12 * math.sin(0.37 * time_s),
            side * (0.18 + 0.16 * motion),
        ),
    )


def _tag_visible(device_id: str, time_s: float) -> bool:
    if device_id == "left":
        return not (23.0 <= time_s < 25.0)
    return not (25.0 <= time_s < 28.0)


def _camera_from_tag(
    world_from_camera: np.ndarray,
    world_from_gripper: np.ndarray,
    mount: TagMount,
) -> np.ndarray:
    return compose(
        invert(world_from_camera),
        world_from_gripper,
        invert(mount.tag_from_gripper),
    )


def _pose_payload(transform: np.ndarray) -> dict:
    value = validate_transform(transform)
    return {
        "translation_m": value[:3, 3].tolist(),
        "quaternion_xyzw": _quaternion_xyzw(value[:3, :3]),
        "matrix_4x4_row_major": value.reshape(-1).tolist(),
    }


def _quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray((x, y, z, w), dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion.tolist()


def _render_video(path: Path, rows: list[dict], *, fps: int) -> None:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required to render the DEMO video") from exc

    width, height = 1280, 720
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the MP4 DEMO writer")
    trails = {device_id: [] for device_id in ("ego", "left", "right")}
    colors = {"ego": (80, 220, 255), "left": (80, 220, 80), "right": (80, 80, 255)}
    try:
        for row in rows:
            image = np.full((height, width, 3), 24, dtype=np.uint8)
            cv2.putText(image, "SIMULATION DEMO - EGO WORLD W", (34, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            time_s = row["timestamp_ns"] / 1e9
            cv2.putText(image, f"t={time_s:05.2f}s   Ego + dual UMI", (880, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1)
            cv2.rectangle(image, (30, 70), (850, 680), (110, 110, 110), 1)
            cv2.putText(image, "Top view / meters", (48, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.line(image, (140, 590), (760, 590), (90, 90, 90), 1)
            cv2.line(image, (140, 590), (140, 150), (90, 90, 90), 1)
            cv2.putText(image, "+X", (765, 596), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(image, "+Y", (126, 142), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            for device_id in ("ego", "left", "right"):
                pose = row["poses"][device_id]["T_world_device"]
                x, y, _z = pose["translation_m"]
                point = (int(180 + 680 * x), int(590 - 650 * y))
                trails[device_id].append(point)
                trail = trails[device_id][-300:]
                if len(trail) > 1:
                    cv2.polylines(image, [np.asarray(trail, dtype=np.int32)], False, colors[device_id], 2)
                cv2.circle(image, point, 8, colors[device_id], -1)
                cv2.putText(image, device_id.upper(), (point[0] + 10, point[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[device_id], 1)

            y_cursor = 105
            for device_id in ("left", "right"):
                pose_row = row["poses"][device_id]
                visible = pose_row["tag_visible"]
                cv2.rectangle(image, (885, y_cursor - 28), (1245, y_cursor + 82), (70, 70, 70), 1)
                cv2.putText(image, f"UMI {device_id.upper()} / tag ID {0 if device_id == 'left' else 1}", (900, y_cursor), cv2.FONT_HERSHEY_SIMPLEX, 0.58, colors[device_id], 1)
                status = "TAG VISIBLE" if visible else "TAG OCCLUDED - VIO BRIDGE"
                cv2.putText(image, status, (900, y_cursor + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (80, 220, 80) if visible else (0, 180, 255), 1)
                xyz = pose_row["T_world_device"]["translation_m"]
                cv2.putText(image, f"W xyz [{xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f}] m", (900, y_cursor + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (190, 190, 190), 1)
                y_cursor += 135
            cv2.putText(image, "T_W_G = T_W_CE * T_CE_A * T_A_G", (890, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (220, 220, 220), 1)
            cv2.putText(image, "Tags align UMI odometry; Ego defines W", (890, 474), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1)
            cv2.putText(image, "NOT HIL / NOT REAL SENSOR VIDEO", (890, 650), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 180, 255), 2)
            writer.write(image)
    finally:
        writer.release()
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError("DEMO video writer produced no file")


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _write_jsonl(path: Path, rows: Iterable[dict]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def _file_evidence(path: Path) -> dict:
    payload = path.read_bytes()
    return {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}


def _source_hashes() -> dict:
    paths = (Path(__file__), Path(__file__).with_name("apriltag_alignment.py"))
    return {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args(argv)
    report = export_apriltag_demo(
        args.output,
        duration_s=args.duration,
        fps=args.fps,
        render_video=not args.no_video,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["verdict"] == "PASS/simulation" else 1


if __name__ == "__main__":
    raise SystemExit(main())
