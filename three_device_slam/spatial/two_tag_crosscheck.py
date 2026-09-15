"""Two-external-tag cross-check for the mount calibration chain.

A single external tag leaves the ego<->umi relative rotation with no
redundancy: any systematic error in one tag observation rotates the
recovered mount vector and there is no second opinion.  This module
computes the full chain twice -- once per external tag -- and reports
how well the two independent estimates of ``T_ego_umi`` and of the final
``T_umiLeft_mount`` agree.

Required capture geometry (one static rig, ~20 s dual-color capture via
``scripts/pair_color_tag_capture.py``):

* Ego color sees: the UMI-back mount tag AND both external tags.
* UMI color sees: both external tags.

Placement rules inherited from the single-tag calibration: every tag
should face each observing camera at <45 deg incidence and subtend
>=60 px; the two external tags should be >=20 cm apart so their UMI
triangulation separates cleanly.

When the second external tag shares its id with the mount tag (only
assets ``id0``/``id1`` exist today, and the mount tag is ``id1``), the
ego stream contains two physical tags with the same id.  Ego
observations of the external id are therefore filtered by range: any
observation whose camera distance is within ``mount_margin_m`` of the
mount-tag distance belongs to the mount tag and is dropped.

Verdict thresholds are intentionally conservative: ``pass`` means the
two chains agree to <=3 deg and <=5 mm, i.e. the single-tag result is
not rotation-limited.  A real capture that disagrees beyond that is
telling you one of the four tag observations is bad (usually an
oblique/small external tag flipping IPPE modes) -- inspect the per-tag
counts and stability, then re-place the offending tag.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from . import mount_calibration as mc
from . import pair_tag_alignment as pta
from .se3 import compose, invert, pose_error, validate_transform


SCHEMA = "ego.two_device.mount_calibration.two_tag_crosscheck.v1"

PASS_ROTATION_DEG = 3.0
PASS_Z_DIFFERENCE_M = 0.005


def _mean_transform(transforms: list[np.ndarray]) -> np.ndarray:
    """Component-wise median translation plus Markley quaternion mean."""
    if not transforms:
        raise ValueError("no transforms to average")
    if len(transforms) == 1:
        return transforms[0]
    translations = np.array([t[:3, 3] for t in transforms], dtype=np.float64)
    median_translation = np.median(translations, axis=0)
    quaternions = []
    for transform in transforms:
        rotation = transform[:3, :3]
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
    w, x, y, z = eigenvectors[:, -1]
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
    return validate_transform(transform, name="mean transform")


def _nearer_cluster(observations, min_gap_m: float = 0.08):
    """Keep the nearer of two range clusters (same-id collision on ego).

    When the second external tag shares the mount tag's id, the ego
    stream holds two physical tags under one id.  The 3D component-wise
    median then lands between the clusters and both pass a MAD gate, so
    the reference pose must be built from one cluster only.  The mount
    tag is on the UMI (~30 cm from ego); a dual-visible external tag is
    always placed farther out, so the nearer cluster is the mount.
    With a single cluster (no gap >= min_gap_m) the series is returned
    untouched.
    """
    if len(observations) < 4:
        return list(observations)
    distances = np.sort(
        np.array([np.linalg.norm(o.camera_from_tag[:3, 3]) for o in observations])
    )
    gaps = np.diff(distances)
    split = int(np.argmax(gaps))
    if gaps[split] < min_gap_m:
        return list(observations)
    cut = (distances[split] + distances[split + 1]) / 2.0
    return [
        o for o in observations if np.linalg.norm(o.camera_from_tag[:3, 3]) <= cut
    ]


def _drop_mount_cluster(observations, mount_distance_m: float, margin_m: float):
    """Drop same-id observations that belong to the mount tag, not the ext."""
    kept = []
    dropped = 0
    for obs in observations:
        distance = float(np.linalg.norm(obs.camera_from_tag[:3, 3]))
        if abs(distance - mount_distance_m) <= margin_m:
            dropped += 1
        else:
            kept.append(obs)
    return kept, dropped


def _chain(session, t_ego_mount, ego_ext, umi_ext, t_ir_color):
    t_ego_ext = mc._reference_transform(ego_ext)
    t_umi_ext = mc._reference_transform(umi_ext)
    t_ego_umi = compose(t_ego_ext, invert(t_umi_ext))
    t_umi_mount = compose(invert(t_ego_umi), t_ego_mount)
    t_ir_mount = compose(t_ir_color, t_umi_mount)
    return t_ego_umi, t_ir_mount


def compute_two_tag_crosscheck(
    session: Path,
    *,
    mount_tag_id: int,
    ext0_tag_id: int,
    ext1_tag_id: int,
    tag_size_m: float = pta.DEFAULT_TAG_SIZE_M,
    mount_margin_m: float = 0.10,
    stride: int = 1,
    min_observations: int = 8,
) -> dict:
    """Cross-check the mount calibration through two external tags."""
    session = Path(session)
    if ext0_tag_id == ext1_tag_id:
        raise ValueError("external tag ids must differ")

    ego_mount = mc._reject_translation_outliers(
        _nearer_cluster(
            mc._detect(
                session, "ego", "ego.color",
                tag_id=mount_tag_id, tag_size_m=tag_size_m, stride=stride,
            )
        )
    )
    t_ego_mount = mc._reference_transform(ego_mount)
    mount_distance = float(np.linalg.norm(t_ego_mount[:3, 3]))

    chains = {}
    counts = {
        "ego_mount_observations": len(ego_mount),
    }
    stability = {}
    t_ego_umi_pair = []
    t_ir_pair = []
    for label, ext_id in (("ext0", ext0_tag_id), ("ext1", ext1_tag_id)):
        ego_ext_raw = mc._detect(
            session, "ego", "ego.color",
            tag_id=ext_id, tag_size_m=tag_size_m, stride=stride,
        )
        # Split physical same-ID tags before applying a single-cluster MAD gate.
        # Distinct IDs are already disambiguated and may share the mount range.
        ego_ext, dropped = (
            _drop_mount_cluster(ego_ext_raw, mount_distance, mount_margin_m)
            if ext_id == mount_tag_id else (ego_ext_raw, 0)
        )
        ego_ext_before_outliers = len(ego_ext)
        ego_ext = mc._reject_translation_outliers(ego_ext)
        umi_ext = mc._reject_translation_outliers(
            mc._detect(
                session, "left", "left.color",
                tag_id=ext_id, tag_size_m=tag_size_m, stride=stride,
            )
        )
        counts[f"ego_{label}_observations_raw"] = len(ego_ext_raw)
        counts[f"ego_{label}_observations_used"] = len(ego_ext)
        counts[f"ego_{label}_mount_cluster_dropped"] = dropped
        counts[f"ego_{label}_outliers_dropped"] = ego_ext_before_outliers - len(ego_ext)
        counts[f"umi_{label}_observations"] = len(umi_ext)
        if len(ego_ext) < min_observations or len(umi_ext) < min_observations:
            raise ValueError(
                f"{label}: too few observations after filtering "
                f"(ego={len(ego_ext)}, umi={len(umi_ext)}, "
                f"min={min_observations})"
            )
        stability[f"ego_{label}"] = mc._stability_subset(ego_ext)
        stability[f"umi_{label}"] = mc._stability_subset(umi_ext)
        t_ego_umi, t_ir_mount = _chain(
            session, t_ego_mount, ego_ext, umi_ext,
            mc._color_to_ir_transform(session, "left"),
        )
        t_ego_umi_pair.append(t_ego_umi)
        t_ir_pair.append(t_ir_mount)
        chains[label] = {
            "external_tag_id": ext_id,
            "ego_color_from_umi_color": mc._xyz_rpy(t_ego_umi),
            "umi_ir_left_from_mount_tag": mc._xyz_rpy(t_ir_mount),
        }

    rel_pose = pose_error(t_ego_umi_pair[0], t_ego_umi_pair[1])
    z0 = t_ir_pair[0][2, 3]
    z1 = t_ir_pair[1][2, 3]
    agreement = {
        "ego_color_from_umi_color_rotation_deg": round(rel_pose.rotation_deg, 3),
        "ego_color_from_umi_color_translation_m": round(rel_pose.translation_m, 6),
        "mount_z_difference_m": round(abs(float(z0 - z1)), 6),
        "mount_norm_difference_m": round(
            abs(float(np.linalg.norm(t_ir_pair[0][:3, 3]) - np.linalg.norm(t_ir_pair[1][:3, 3]))),
            6,
        ),
    }
    verdict = (
        "pass"
        if rel_pose.rotation_deg <= PASS_ROTATION_DEG
        and agreement["mount_z_difference_m"] <= PASS_Z_DIFFERENCE_M
        else "review"
    )

    consensus = _mean_transform(t_ir_pair)
    report = {
        "schema": SCHEMA,
        "session": session.name,
        "mount_tag_id": mount_tag_id,
        "external_tag_ids": [ext0_tag_id, ext1_tag_id],
        "tag_size_m": tag_size_m,
        "mount_margin_m": mount_margin_m,
        "counts": counts,
        "stability": stability,
        "chains": chains,
        "agreement": agreement,
        "consensus_umi_ir_left_from_mount_tag": mc._xyz_rpy(consensus),
        "verdict": verdict,
        "convention": "T_a_b maps points from frame b into frame a",
    }
    output = session / "spatial"
    output.mkdir(parents=True, exist_ok=True)
    (output / "two_tag_crosscheck_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    return report


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mount-id", type=int, required=True)
    parser.add_argument("--ext0-id", type=int, required=True)
    parser.add_argument("--ext1-id", type=int, required=True)
    parser.add_argument("--tag-size-mm", type=float, default=40.0)
    parser.add_argument("--mount-margin-mm", type=float, default=100.0)
    parser.add_argument("--stride", type=int, default=1)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    report = compute_two_tag_crosscheck(
        args.session,
        mount_tag_id=args.mount_id,
        ext0_tag_id=args.ext0_id,
        ext1_tag_id=args.ext1_id,
        tag_size_m=args.tag_size_mm / 1000.0,
        mount_margin_m=args.mount_margin_mm / 1000.0,
        stride=args.stride,
    )
    print(json.dumps(report["agreement"], indent=2, sort_keys=True))
    print(f"verdict={report['verdict']}")
    for label in ("ext0", "ext1"):
        t = report["chains"][label]["umi_ir_left_from_mount_tag"]
        print(f"{label} z={t['translation_m'][2]:+.4f} m")
    print(
        f"consensus z={report['consensus_umi_ir_left_from_mount_tag']['translation_m'][2]:+.4f} m"
    )
    print(f"report={args.session / 'spatial' / 'two_tag_crosscheck_report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
