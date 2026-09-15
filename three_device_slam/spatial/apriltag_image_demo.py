"""Deterministic pixel-to-AprilTag-to-Ego-world dual-UMI replay DEMO."""

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
    estimate_tag_anchor_window,
    reconcile_anchor,
)
from .apriltag_detector import (
    AprilTagDetectorConfig,
    CameraCalibration,
    detect_apriltags,
)
from .se3 import compose, invert, pose_error, transform_from_xyz_rpy, validate_transform


_SCHEMA = "three-device-slam.apriltag-image-demo.v1"
_DEVICES = ("left", "right")
_ROOT = Path(__file__).resolve().parents[2]


def export_apriltag_image_demo(
    output: Path | str,
    *,
    duration_s: float = 6.0,
    fps: int = 10,
) -> dict:
    """Render tag pixels, detect/PnP them, and export replay evidence."""
    output_path = Path(output)
    if output_path.exists():
        raise FileExistsError(f"image DEMO output already exists: {output_path}")
    if (
        isinstance(duration_s, bool)
        or not isinstance(duration_s, (int, float))
        or not math.isfinite(float(duration_s))
        or float(duration_s) <= 0.0
    ):
        raise ValueError("duration_s must be positive and finite")
    if isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0:
        raise ValueError("fps must be a positive integer")
    frame_count = int(round(float(duration_s) * fps))
    if frame_count < 30:
        raise ValueError("image DEMO needs at least 30 frames")

    import cv2

    for relative in ("calibration", "derived", "exports", "quality"):
        (output_path / relative).mkdir(parents=True, exist_ok=False)
    calibration = _camera_calibration()
    config = AprilTagDetectorConfig(
        family="tag36h11",
        tag_size_m=0.040,
        allowed_tag_ids=(0, 1),
        max_reprojection_error_px=1.0,
    )
    mounts = {device_id: _mount(device_id) for device_id in _DEVICES}
    tag_images = {device_id: _load_tag_image(cv2, device_id) for device_id in _DEVICES}
    timestamps_ns = tuple(round(index * 1_000_000_000 / fps) for index in range(frame_count))
    ego_samples = tuple(PoseSample(timestamp, np.eye(4)) for timestamp in timestamps_ns)
    truth = {device_id: [] for device_id in _DEVICES}
    odom = {device_id: [] for device_id in _DEVICES}
    observations = {
        device_id: {"initial": [], "reacquisition": []} for device_id in _DEVICES
    }
    detection_rows: list[dict] = []
    false_ids = 0
    video_path = output_path / "exports/image_detection_overlay.mp4"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (calibration.width, calibration.height),
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open image DEMO video writer")
    try:
        for index, timestamp_ns in enumerate(timestamps_ns):
            fraction = index / max(1, frame_count - 1)
            camera_from_tag = {
                device_id: _camera_from_tag(device_id, fraction)
                for device_id in _DEVICES
            }
            visible = {
                device_id: _visible(device_id, fraction) for device_id in _DEVICES
            }
            image = np.full(
                (calibration.height, calibration.width), 210, dtype=np.uint8
            )
            for device_id in _DEVICES:
                world_from_gripper = compose(
                    camera_from_tag[device_id], mounts[device_id].tag_from_gripper
                )
                truth[device_id].append(world_from_gripper)
                odom_from_world = _odom_from_world(device_id)
                odom[device_id].append(
                    PoseSample(
                        timestamp_ns,
                        compose(odom_from_world, world_from_gripper),
                    )
                )
                if visible[device_id]:
                    _warp_tag(
                        cv2,
                        image,
                        tag_images[device_id],
                        calibration,
                        camera_from_tag[device_id],
                    )
            batch = detect_apriltags(
                image,
                calibration=calibration,
                config=config,
                acquisition_timestamp_ns=timestamp_ns,
                detector_completed_ns=timestamp_ns + 18_000_000,
            )
            by_id = {item.tag_id: item for item in batch.detections}
            for device_id in _DEVICES:
                tag_id = mounts[device_id].tag_id
                detection = by_id.get(tag_id)
                phase = _phase(device_id, fraction)
                if detection is not None and detection.accepted:
                    if phase in observations[device_id]:
                        observations[device_id][phase].append(detection.observation)
                    row_reason = "accepted"
                    reprojection_error = detection.observation.reprojection_error_px
                    transform_payload = _pose_payload(
                        detection.observation.camera_from_tag
                    )
                    corners = detection.corners_px.tolist()
                else:
                    row_reason = (
                        detection.reason if detection is not None else "tag_not_detected"
                    )
                    reprojection_error = None
                    transform_payload = None
                    corners = None if detection is None else detection.corners_px.tolist()
                candidate_payloads = (
                    []
                    if detection is None
                    else [_candidate_payload(item) for item in detection.candidates]
                )
                detection_rows.append(
                    {
                        "schema": "three-device-slam.apriltag-image-detection-row.v1",
                        "frame_index": index,
                        "acquisition_timestamp_ns": timestamp_ns,
                        "detector_completed_ns": timestamp_ns + 18_000_000,
                        "detector_latency_ns": 18_000_000,
                        "device_id": device_id,
                        "family": "tag36h11",
                        "tag_id": tag_id,
                        "visible_in_renderer": visible[device_id],
                        "phase": phase,
                        "reason": row_reason,
                        "corners_px": corners,
                        "T_camera_tag": transform_payload,
                        "reprojection_error_px": reprojection_error,
                        "pnp_candidates": candidate_payloads,
                        "selected_index": (
                            None if detection is None else detection.selected_index
                        ),
                        "detector_backend": batch.backend,
                    }
                )
            for detection in batch.detections:
                if detection.tag_id in (0, 1):
                    continue
                detection_rows.append(
                    {
                        "schema": "three-device-slam.apriltag-image-detection-row.v1",
                        "frame_index": index,
                        "acquisition_timestamp_ns": timestamp_ns,
                        "detector_completed_ns": timestamp_ns + 18_000_000,
                        "detector_latency_ns": 18_000_000,
                        "device_id": "unassigned",
                        "family": "tag36h11",
                        "tag_id": detection.tag_id,
                        "visible_in_renderer": None,
                        "phase": "identity_rejection",
                        "reason": detection.reason,
                        "corners_px": detection.corners_px.tolist(),
                        "T_camera_tag": None,
                        "reprojection_error_px": None,
                        "pnp_candidates": [
                            _candidate_payload(item)
                            for item in detection.candidates
                        ],
                        "selected_index": detection.selected_index,
                        "detector_backend": batch.backend,
                    }
                )
            false_ids += sum(item.tag_id not in (0, 1) for item in batch.detections)
            overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            for detection in batch.detections:
                color = (40, 220, 40) if detection.accepted else (0, 80, 255)
                polygon = np.rint(detection.corners_px).astype(np.int32)
                cv2.polylines(overlay, [polygon], True, color, 2)
                origin = tuple(polygon[0].tolist())
                cv2.putText(
                    overlay,
                    f"ID {detection.tag_id} {detection.reason}",
                    (origin[0], max(24, origin[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    color,
                    1,
                )
            cv2.rectangle(overlay, (0, 0), (1279, 58), (15, 15, 15), -1)
            cv2.putText(
                overlay,
                "SIMULATION IMAGE REPLAY - pixels -> tag36h11 -> IPPE -> Ego W",
                (20, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                overlay,
                f"frame {index:03d}  t={timestamp_ns / 1e9:05.2f}s  NOT HIL",
                (20, 49),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 190, 255),
                1,
            )
            writer.write(overlay)
    finally:
        writer.release()

    chains = {}
    trajectories = {}
    policy = TagObservationPolicy(
        min_inliers=5,
        anchor_translation_threshold_m=0.08,
        anchor_rotation_threshold_deg=8.0,
    )
    for device_id in _DEVICES:
        initial_observations = tuple(observations[device_id]["initial"][:10])
        reacquisition_observations = tuple(
            observations[device_id]["reacquisition"][:10]
        )
        initial = estimate_tag_anchor_window(
            ego_samples,
            odom[device_id],
            mounts[device_id],
            initial_observations,
            policy=policy,
            observation_delay_ns=0,
            max_interpolation_gap_ns=round(1_500_000_000 / fps),
        )
        if initial.verdict != "PASS/replay" or initial.world_from_odom is None:
            raise RuntimeError(f"{device_id} image initial anchor failed")
        current = AnchorAcceptance.initial(
            device_id=device_id,
            world_from_odom=initial.world_from_odom,
            accepted_at_ns=max(initial.inlier_timestamps_ns),
        )
        reacquisition = estimate_tag_anchor_window(
            ego_samples,
            odom[device_id],
            mounts[device_id],
            reacquisition_observations,
            policy=policy,
            observation_delay_ns=0,
            max_interpolation_gap_ns=round(1_500_000_000 / fps),
        )
        if (
            reacquisition.verdict != "PASS/replay"
            or reacquisition.world_from_odom is None
        ):
            raise RuntimeError(
                f"{device_id} image reacquisition anchor failed: "
                f"candidates={reacquisition.candidate_count}, "
                f"inliers={len(reacquisition.inlier_timestamps_ns)}, "
                f"rejections={[item.reason for item in reacquisition.rejections]}"
            )
        candidate = AnchorAcceptance.initial(
            device_id=device_id,
            world_from_odom=reacquisition.world_from_odom,
            accepted_at_ns=max(reacquisition.inlier_timestamps_ns),
        )
        reconciliation = reconcile_anchor(
            current,
            candidate,
            max_translation_update_m=0.08,
            max_rotation_update_deg=8.0,
        )
        if not reconciliation.accepted:
            raise RuntimeError(f"{device_id} image anchor continuity failed")
        initial_trajectory = propagate_world_trajectory(
            current.world_from_odom, odom[device_id]
        )
        reacquired_trajectory = propagate_world_trajectory(
            reconciliation.state.world_from_odom, odom[device_id]
        )
        switch_timestamp_ns = reconciliation.state.accepted_at_ns
        trajectories[device_id] = tuple(
            after if after.timestamp_ns >= switch_timestamp_ns else before
            for before, after in zip(initial_trajectory, reacquired_trajectory)
        )
        anchor_truth = invert(_odom_from_world(device_id))
        error = pose_error(initial.world_from_odom, anchor_truth)
        chains[device_id] = {
            "tag_id": mounts[device_id].tag_id,
            "initial_anchor_verdict": initial.verdict,
            "initial_anchor_candidates": initial.candidate_count,
            "initial_anchor_inliers": len(initial.inlier_timestamps_ns),
            "reacquisition_anchor_verdict": reacquisition.verdict,
            "reacquisition_candidates": reacquisition.candidate_count,
            "reacquisition_inliers": len(reacquisition.inlier_timestamps_ns),
            "reacquisition_accepted": reconciliation.accepted,
            "reacquisition_switch_timestamp_ns": switch_timestamp_ns,
            "occluded_frames": sum(
                not _visible(device_id, index / max(1, frame_count - 1))
                for index in range(frame_count)
            ),
            "anchor_translation_error_m": error.translation_m,
            "anchor_rotation_error_deg": error.rotation_deg,
            "initial_T_world_odom": _pose_payload(initial.world_from_odom),
            "reacquisition_T_world_odom": _pose_payload(
                reconciliation.state.world_from_odom
            ),
        }

    trajectory_rows = []
    for index, timestamp_ns in enumerate(timestamps_ns):
        trajectory_rows.append(
            {
                "schema": "three-device-slam.ego-world-trajectory-row.v1",
                "timestamp_ns": timestamp_ns,
                "devices": ["ego", "left", "right"],
                "poses": {
                    "ego": {
                        "pose_source": "ego_slam_simulation",
                        "T_world_device": _pose_payload(np.eye(4)),
                    },
                    **{
                        device_id: {
                            "pose_source": (
                                "image_tag_anchor_plus_vio"
                                if _visible(
                                    device_id,
                                    index / max(1, frame_count - 1),
                                )
                                else "umi_vio_bridge"
                            ),
                            "T_world_device": _pose_payload(
                                trajectories[device_id][index].transform
                            ),
                            "T_odom_device": _pose_payload(
                                odom[device_id][index].transform
                            ),
                        }
                        for device_id in _DEVICES
                    },
                },
            }
        )

    camera_payload = {
        "schema": "three-device-slam.camera-calibration.v1",
        "scope": "simulation_image_replay_using_d435i_factory_profile_shape",
        "calibration_id": calibration.calibration_id,
        "frame_id": calibration.frame_id,
        "width": calibration.width,
        "height": calibration.height,
        "camera_matrix": calibration.camera_matrix.tolist(),
        "distortion_coefficients": calibration.distortion_coefficients.tolist(),
    }
    tag_payload = {
        "schema": "three-device-slam.apriltag-mounts.v1",
        "scope": "simulation_only_not_installation_calibration",
        "family": "tag36h11",
        "effective_black_square_m": 0.040,
        "full_quiet_zone_square_m": 0.050,
        "asset_manifest_sha256": _sha256(
            _ROOT / "assets/calibration_tags/manifest.json"
        ),
        "mounts": [
            {
                "device_id": device_id,
                "tag_id": mounts[device_id].tag_id,
                "calibration_id": mounts[device_id].calibration_id,
                "T_tag_gripper": _pose_payload(
                    mounts[device_id].tag_from_gripper
                ),
            }
            for device_id in _DEVICES
        ],
    }
    report = {
        "schema": "three-device-slam.apriltag-image-demo-acceptance.v1",
        "mode": "deterministic_image_replay",
        "verdict": "PASS/simulation_image_replay",
        "hil_status": "NOT_RUN",
        "detector_backend": "opencv_aruco_reference",
        "production_detector_status": "NOT_RUN/AprilRobotics_backend_not_installed",
        "timeline": {
            "duration_s": float(duration_s),
            "fps": fps,
            "frames": frame_count,
        },
        "image_pipeline": {
            "frames_processed": frame_count,
            "false_id_detections": false_ids,
            "input": "projectively rendered official tag bitmaps",
            "pose_solver": "SOLVEPNP_IPPE_SQUARE",
        },
        "world_frame": "Ego SLAM world W; tags initialize UMI odometry anchors",
        "chains": chains,
        "limitations": [
            "All camera images and trajectories are deterministic simulation.",
            "OpenCV ArUco is a reference/replay detector, not the production AprilRobotics backend.",
            "Mount extrinsics are simulation fixtures and must be physically calibrated after installation.",
            "This evidence is not HIL and is not training-data acceptance.",
        ],
    }
    _write_json(output_path / "calibration/camera.json", camera_payload)
    _write_json(output_path / "calibration/tag_mounts.json", tag_payload)
    _write_jsonl(output_path / "derived/image_detections.jsonl", detection_rows)
    _write_jsonl(
        output_path / "derived/trajectories_ego_world.jsonl", trajectory_rows
    )
    _write_json(output_path / "quality/alignment_report.json", report)
    evidence_paths = (
        "calibration/camera.json",
        "calibration/tag_mounts.json",
        "derived/image_detections.jsonl",
        "derived/trajectories_ego_world.jsonl",
        "exports/image_detection_overlay.mp4",
        "quality/alignment_report.json",
    )
    manifest = {
        "schema": _SCHEMA,
        "mode": "deterministic_image_replay",
        "verdict": report["verdict"],
        "hil_status": "NOT_RUN",
        "detector_backend": "opencv_aruco_reference",
        "generator_sources": _source_hashes(),
        "files": {
            relative: _file_evidence(output_path / relative)
            for relative in evidence_paths
        },
    }
    _write_json(output_path / "manifest.json", manifest)
    return report


def _camera_calibration() -> CameraCalibration:
    return CameraCalibration(
        calibration_id="D435I_FACTORY_327122078613_20260828_IMAGE_SIM",
        frame_id="ego_ir_left_optical",
        width=1280,
        height=720,
        camera_matrix=np.array(
            (
                (644.0672, 0.0, 642.2068),
                (0.0, 644.0672, 367.0062),
                (0.0, 0.0, 1.0),
            ),
            dtype=np.float64,
        ),
        distortion_coefficients=np.zeros(5, dtype=np.float64),
    )


def _mount(device_id: str) -> TagMount:
    side = 1.0 if device_id == "left" else -1.0
    return TagMount(
        device_id=device_id,
        family="tag36h11",
        tag_id=0 if device_id == "left" else 1,
        tag_from_gripper=transform_from_xyz_rpy(
            (0.012, side * 0.004, 0.034), (math.pi, 0.0, side * 0.04)
        ),
        calibration_id=f"simulation-{device_id}-tag-mount-image-v1",
    )


def _camera_from_tag(device_id: str, fraction: float) -> np.ndarray:
    if device_id == "left":
        return transform_from_xyz_rpy(
            (-0.12 + 0.008 * math.sin(5.0 * fraction), -0.035, 0.50),
            (math.pi - 0.25, 0.18, -0.10 + 0.08 * fraction),
        )
    return transform_from_xyz_rpy(
        (0.14 + 0.008 * math.sin(4.0 * fraction), 0.04, 0.50),
        (math.pi + 0.30, -0.22, 0.14 - 0.07 * fraction),
    )


def _odom_from_world(device_id: str) -> np.ndarray:
    return (
        transform_from_xyz_rpy((2.7, -1.1, 0.5), (0.08, -0.04, 0.65))
        if device_id == "left"
        else transform_from_xyz_rpy((-2.2, 3.4, 0.7), (-0.06, 0.03, -0.72))
    )


def _visible(device_id: str, fraction: float) -> bool:
    if device_id == "left":
        return not (0.35 <= fraction < 0.50)
    return not (0.50 <= fraction < 0.65)


def _phase(device_id: str, fraction: float) -> str:
    if fraction < 0.30:
        return "initial"
    end = 0.50 if device_id == "left" else 0.65
    if fraction >= end:
        return "reacquisition"
    return "bridge_or_transition"


def _load_tag_image(cv2, device_id: str) -> np.ndarray:
    tag_id = 0 if device_id == "left" else 1
    path = (
        _ROOT
        / "assets/calibration_tags"
        / f"umi_{device_id}_tag36h11_id{tag_id}_40mm.png"
    )
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"cannot load tag asset: {path}")
    return np.rot90(image, 2).copy()


def _warp_tag(cv2, canvas, tag_image, calibration, camera_from_tag) -> None:
    # The official PNG covers 50 mm including quiet zone; its inner 8/10
    # black square is the 40 mm metric size supplied to the PnP solver.
    outer_half = 0.025
    object_corners = np.array(
        (
            (-outer_half, outer_half, 0.0),
            (outer_half, outer_half, 0.0),
            (outer_half, -outer_half, 0.0),
            (-outer_half, -outer_half, 0.0),
        ),
        dtype=np.float64,
    )
    rotation_vector, _ = cv2.Rodrigues(camera_from_tag[:3, :3])
    projected, _ = cv2.projectPoints(
        object_corners,
        rotation_vector,
        camera_from_tag[:3, 3],
        calibration.camera_matrix,
        calibration.distortion_coefficients,
    )
    source = np.array(
        (
            (0.0, 0.0),
            (tag_image.shape[1] - 1.0, 0.0),
            (tag_image.shape[1] - 1.0, tag_image.shape[0] - 1.0),
            (0.0, tag_image.shape[0] - 1.0),
        ),
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(
        source, projected.reshape(4, 2).astype(np.float32)
    )
    warped = cv2.warpPerspective(
        tag_image,
        homography,
        (calibration.width, calibration.height),
        flags=cv2.INTER_NEAREST,
        borderValue=255,
    )
    mask = cv2.warpPerspective(
        np.full(tag_image.shape, 255, dtype=np.uint8),
        homography,
        (calibration.width, calibration.height),
        flags=cv2.INTER_NEAREST,
        borderValue=0,
    )
    canvas[mask > 0] = warped[mask > 0]


def _pose_payload(transform: np.ndarray) -> dict:
    value = validate_transform(transform)
    return {"matrix_4x4_row_major": value.reshape(-1).tolist()}


def _candidate_payload(candidate) -> dict:
    return {
        "T_camera_tag": _pose_payload(candidate.camera_from_tag),
        "reprojection_error_px": candidate.reprojection_error_px,
        "positive_depth": candidate.positive_depth,
    }


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _write_jsonl(path: Path, rows: Iterable[dict]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _file_evidence(path: Path) -> dict:
    payload = path.read_bytes()
    return {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}


def _source_hashes() -> dict:
    paths = (
        Path(__file__),
        Path(__file__).with_name("apriltag_detector.py"),
        Path(__file__).with_name("apriltag_alignment.py"),
    )
    return {path.name: _sha256(path) for path in paths}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args(argv)
    report = export_apriltag_image_demo(
        args.output, duration_s=args.duration, fps=args.fps
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["verdict"] == "PASS/simulation_image_replay" else 1


if __name__ == "__main__":
    raise SystemExit(main())
