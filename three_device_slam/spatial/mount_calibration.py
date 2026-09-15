"""Tag-to-UMI mount calibration via a common external tag (co-observation).

The AprilTag on the UMI's back cannot be seen by the UMI's own cameras, so
the mount transform ``T_umiLeft_tag`` is calibrated indirectly:

1. The Ego color camera observes both the UMI-back mount tag and a second
   (external) tag placed where both devices can see it.
2. The UMI color camera observes the same external tag.
3. ``T_ego_umiColor = T_egoColor_ext * inv(T_umiColor_ext)`` links the two
   camera frames at capture time (static rig, so any timestamp works).
4. ``T_umiColor_mount = inv(T_ego_umiColor) * T_egoColor_mount`` and the
   factory color->ir_left extrinsics move the result into the UMI stereo
   (VIO) camera frame.

Both observation directions reuse the reference AprilTag pipeline from
:mod:`three_device_slam.spatial.pair_tag_alignment`.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from three_device_slam.devices.realsense_extrinsics import load_color_to_ir_snapshot

from . import pair_tag_alignment as pta
from .apriltag_detector import AprilTagDetectorConfig, detect_apriltags
from .se3 import compose, invert, validate_transform


SCHEMA = "ego.two_device.mount_calibration.v1"


def _xyz_rpy(transform: np.ndarray) -> dict:
    rotation = np.asarray(transform[:3, :3], dtype=np.float64)
    translation = np.asarray(transform[:3, 3], dtype=np.float64)
    sy = math.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    singular = sy < 1e-9
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = (
        math.atan2(-rotation[2, 0], sy)
        if not singular
        else math.atan2(-rotation[2, 0], 0.0)
    )
    yaw = (
        math.atan2(rotation[1, 0], rotation[0, 0])
        if not singular
        else math.atan2(-rotation[1, 2], rotation[1, 1])
    )
    return {
        "translation_m": [round(float(v), 6) for v in translation],
        "rotation_rpy_deg": [round(math.degrees(v), 3) for v in (roll, pitch, yaw)],
        "rotation_row_major": [
            round(float(v), 9) for v in rotation.reshape(-1)
        ],
    }


def _detect(
    session: Path,
    device_id: str,
    stream_id: str,
    *,
    tag_id: int,
    tag_size_m: float,
    stride: int,
):
    calibration = pta.load_device_calibration(session, device_id, stream_id)
    rows = pta.load_formal_index_rows(session, device_id, stream_id)
    if not rows:
        raise ValueError(f"no formal rows for {stream_id}")
    config = AprilTagDetectorConfig(
        family="tag36h11",
        tag_size_m=tag_size_m,
        allowed_tag_ids=(tag_id,),
        ambiguity_rotation_deg=2.0,
        ambiguity_translation_m=0.002,
    )
    observations = pta.detect_tag_in_frames(
        session,
        rows,
        device_id=device_id,
        stream_id=stream_id,
        calibration=calibration,
        config=config,
        stride=stride,
    )
    if not observations:
        raise ValueError(f"tag id={tag_id} not observed on {stream_id}")
    return observations


def _color_to_ir_transform(session: Path, device_id: str) -> np.ndarray:
    path = Path(session) / device_id / "calibration.json"
    transform, _ = load_color_to_ir_snapshot(path)
    return validate_transform(transform, name="color_to_ir_left")


def _reject_translation_outliers(observations, *, min_inliers: int = 8):
    """Drop pose outliers from one static-rig observation series.

    The reference detector occasionally double-detects: a second quad near
    another tag misdecodes as the wanted id and passes the ambiguity checks
    with a wildly wrong pose. On a static rig the true observations form one
    tight translation cluster, so a median/MAD gate removes the junk before
    the medoid reference pose is chosen.
    """
    if len(observations) < min_inliers:
        return list(observations)
    translations = np.array(
        [obs.camera_from_tag[:3, 3] for obs in observations], dtype=np.float64
    )
    median = np.median(translations, axis=0)
    deviations = np.linalg.norm(translations - median, axis=1)
    mad = float(np.median(deviations)) * 1.4826
    gate = max(3.0 * mad, 0.01)
    kept = [
        obs for obs, dev in zip(observations, deviations) if dev <= gate
    ]
    if len(kept) < min_inliers:
        return list(observations)
    return kept


def _reference_transform(observations, *, min_robust: int = 8) -> np.ndarray:
    """Robust static-rig reference pose for one observation series.

    Unlike the single-sample medoid in ``pair_tag_alignment``, this pools
    the whole inlier set: component-wise median translation plus the
    Markley quaternion mean of the rotation-consistent subset. Small or
    distant tags (e.g. the external tag seen at ~50 px) put tens of
    degrees of frame-to-frame noise on individual rotations, so averaging
    the consistent majority is materially tighter than picking one frame.
    """
    if len(observations) < min_robust:
        return pta.reference_transform(observations)
    from .se3 import pose_error

    translations = np.array(
        [obs.camera_from_tag[:3, 3] for obs in observations], dtype=np.float64
    )
    median_translation = np.median(translations, axis=0)
    seed = pta.reference_transform(observations)
    rotation_errors = [
        pose_error(obs.camera_from_tag, seed).rotation_deg for obs in observations
    ]
    gate = float(np.median(rotation_errors)) + 2.0
    gate = max(gate, 5.0)
    rotations = np.array(
        [
            obs.camera_from_tag[:3, :3]
            for obs, err in zip(observations, rotation_errors)
            if err <= gate
        ]
    )
    if len(rotations) < min_robust // 2:
        return seed
    # Markley mean: dominant eigenvector of sum(outer(q, q)); the outer
    # product is sign-invariant, so quaternion sign flips need no fixing.
    quaternions = []
    for rotation in rotations:
        quaternions.append(
            np.array(
                (
                    np.trace(rotation) + 1.0,
                    rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                    rotation[1, 0] - rotation[0, 1],
                )
            )
        )
    quaternions = np.array(quaternions, dtype=np.float64)
    quaternions /= np.linalg.norm(quaternions, axis=1, keepdims=True)
    _, eigenvectors = np.linalg.eigh(quaternions.T @ quaternions)
    q = eigenvectors[:, -1]
    w, x, y, z = q
    mean_rotation = np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = mean_rotation
    transform[:3, 3] = median_translation
    return validate_transform(transform, name="reference transform")


def compute_mount_calibration(
    session: Path,
    *,
    mount_tag_id: int,
    external_tag_id: int,
    tag_size_m: float = pta.DEFAULT_TAG_SIZE_M,
    external_tag_size_m: float = pta.DEFAULT_TAG_SIZE_M,
    stride: int = 1,
) -> dict:
    """Calibrate ``T_umiLeft_tag`` from a dual-color capture session."""
    session = Path(session)
    ego_mount = _reject_translation_outliers(
        _detect(
            session, "ego", "ego.color", tag_id=mount_tag_id, tag_size_m=tag_size_m, stride=stride
        )
    )
    ego_external = _reject_translation_outliers(
        _detect(
            session,
            "ego",
            "ego.color",
            tag_id=external_tag_id,
            tag_size_m=external_tag_size_m,
            stride=stride,
        )
    )
    umi_external = _reject_translation_outliers(
        _detect(
            session,
            "left",
            "left.color",
            tag_id=external_tag_id,
            tag_size_m=external_tag_size_m,
            stride=stride,
        )
    )

    t_ego_mount = _reference_transform(ego_mount)
    t_ego_external = _reference_transform(ego_external)
    t_umi_external = _reference_transform(umi_external)

    t_ego_umi = compose(t_ego_external, invert(t_umi_external))
    t_umi_mount = compose(invert(t_ego_umi), t_ego_mount)
    t_ir_color = _color_to_ir_transform(session, "left")
    t_ir_mount = compose(t_ir_color, t_umi_mount)

    report = {
        "schema": SCHEMA,
        "session": session.name,
        "mount_tag_id": mount_tag_id,
        "external_tag_id": external_tag_id,
        "tag_size_m": tag_size_m,
        "external_tag_size_m": external_tag_size_m,
        "counts": {
            "ego_mount_observations": len(ego_mount),
            "ego_external_observations": len(ego_external),
            "umi_external_observations": len(umi_external),
        },
        "ego_mount_stability": _stability_subset(ego_mount),
        "ego_external_stability": _stability_subset(ego_external),
        "umi_external_stability": _stability_subset(umi_external),
        "transforms": {
            "ego_color_from_mount_tag": _xyz_rpy(t_ego_mount),
            "ego_color_from_external_tag": _xyz_rpy(t_ego_external),
            "umi_color_from_external_tag": _xyz_rpy(t_umi_external),
            "ego_color_from_umi_color": _xyz_rpy(t_ego_umi),
            "umi_color_from_mount_tag": _xyz_rpy(t_umi_mount),
            "umi_ir_left_from_mount_tag": _xyz_rpy(t_ir_mount),
        },
        "convention": "T_a_b maps points from frame b into frame a",
    }
    output = session / "spatial"
    output.mkdir(parents=True, exist_ok=True)
    (output / "mount_calibration_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    return report


def _stability_subset(observations) -> dict:
    summary = pta.summarize_pose_series(observations)
    return {
        "translation_rms_m": summary["translation_rms_m"],
        "translation_max_m": summary["translation_max_m"],
        "rotation_rms_deg": summary["rotation_rms_deg"],
        "reprojection_median_px": summary["reprojection_median_px"],
    }


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mount-id", type=int, required=True)
    parser.add_argument("--external-id", type=int, required=True)
    parser.add_argument("--tag-size-mm", type=float, default=40.0)
    parser.add_argument("--external-tag-size-mm", type=float, default=40.0)
    parser.add_argument("--stride", type=int, default=1)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    report = compute_mount_calibration(
        args.session,
        mount_tag_id=args.mount_id,
        external_tag_id=args.external_id,
        tag_size_m=args.tag_size_mm / 1000.0,
        external_tag_size_m=args.external_tag_size_mm / 1000.0,
        stride=args.stride,
    )
    print(
        json.dumps(
            report["transforms"]["umi_ir_left_from_mount_tag"],
            indent=2,
            sort_keys=True,
        )
    )
    print(f"report={args.session / 'spatial' / 'mount_calibration_report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
