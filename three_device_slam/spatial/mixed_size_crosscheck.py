"""Static bench cross-check: external ID1/2 80 mm, mount ID1 40 mm.

ID2 provides only discrete identity association, NOT a pose-fitting constraint.
Independent external-tag PnP estimates remain unchanged for the two chains.
This diagnostic does not activate a calibration or certify absolute accuracy.
"""
from __future__ import annotations

import argparse
import json
import time
import zlib
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from three_device_slam.devices.realsense_extrinsics import load_color_to_ir_snapshot

from .apriltag_detector import (
    AprilTagDetectorConfig, CORNER_REFINEMENTS, detect_apriltags,
    validate_corner_refinement,
    project_camera_points,
)
from . import mount_calibration as mc
from . import pair_tag_alignment as pta
from .se3 import invert, pose_error

ROLES = ("ego_mount", "ego_ext1", "ego_ext2", "umi_ext1", "umi_ext2")
DEFAULT_CORNER_REFINEMENT = "apriltag"
THRESHOLDS = dict(min_edge_px=60, max_incidence_deg_exclusive=45,
                  min_external_spacing_m=0.20, association_max_error_px=80,
                  association_min_margin_px=40, max_pair_skew_ms=20,
                  mount_max_distance_from_umi_origin_m=0.15,
                  min_association_fraction=0.90, min_observations=8,
                  max_chain_rotation_deg=3, max_mount_z_difference_m=0.005)


def _pose(detection):
    return detection.observation.camera_from_tag


def associate(ego80, ego40, umi80, ego_calibration):
    """Assign physical roles without using incorrectly scaled range sorting."""
    def matches(detections, tag_id):
        return [d for d in detections if d.tag_id == tag_id]

    candidates = matches(ego80, 1)
    anchors = [matches(ego80, 2), matches(umi80, 2), matches(umi80, 1)]
    if len(candidates) < 2 or any(len(a) != 1 for a in anchors):
        return {}, {"reason": "need_two_ego_ID1_and_unique_external_ID2_and_umi_ID1",
                    "detected_ids": {"ego": [d.tag_id for d in ego80],
                                     "umi": [d.tag_id for d in umi80]}}
    e2, u2, u1 = [a[0] for a in anchors]
    if not all(d.accepted for d in (e2, u2, u1)):
        return {}, {"reason": "anchor_or_umi_external_pose_rejected"}
    ego_from_umi = _pose(e2) @ invert(_pose(u2))
    predicted = ego_from_umi @ _pose(u1)
    if predicted[2, 3] <= 0:
        return {}, {"reason": "predicted_external_behind_camera"}
    # Project tag origin. A perspective quad's arithmetic center is not its
    # projected physical center; use diagonal intersection via homography.
    projected = project_camera_points(predicted[:3, 3].reshape(1, 3), ego_calibration)
    centers = []
    unit = np.float32([[-1, -1], [1, -1], [1, 1], [-1, 1]])
    for d in candidates:
        h = cv2.getPerspectiveTransform(unit, d.corners_px.astype(np.float32))
        centers.append(h[:2, 2] / h[2, 2])
    errors = np.linalg.norm(np.asarray(centers) - projected.reshape(2), axis=1)
    order = np.argsort(errors)
    info = dict(predicted_external_center_px=projected.reshape(2).tolist(),
                candidate_center_errors_px=errors.tolist(),
                margin_px=float(errors[order[1]] - errors[order[0]]))
    if (errors[order[0]] > THRESHOLDS["association_max_error_px"] or
            info["margin_px"] < THRESHOLDS["association_min_margin_px"]):
        return {}, dict(info, reason="ambiguous_external_ID1_identity")
    external = candidates[order[0]]
    if not external.accepted:
        return {}, dict(info, reason="correct_size_external_or_mount_pose_rejected")
    info["resolved_external_id1_corners_px"] = external.corners_px.tolist()
    # Broad rig-proximity prior, NOT the fitted mounting transform. Never pick
    # the nearest/best-looking candidate when multiple candidates are plausible.
    # External-tag PnP estimates are not adjusted by this identity check.
    mounts, checks = [], []
    for quad in candidates:
        if quad is external:
            continue
        matched = [d for d in matches(ego40, 1)
                   if np.max(np.abs(d.corners_px - quad.corners_px)) < 0.1]
        distance = None
        plausible = False
        if len(matched) == 1 and matched[0].accepted:
            distance = float(np.linalg.norm(_pose(matched[0])[:3, 3] - ego_from_umi[:3, 3]))
            plausible = distance <= THRESHOLDS["mount_max_distance_from_umi_origin_m"]
            if plausible:
                mounts.append(matched[0])
        checks.append(dict(corners_px=quad.corners_px.tolist(),
                           distance_from_umi_origin_m=distance, plausible_mount=plausible,
                           unresolved_pose=distance is None))
    info["mount_candidate_checks"] = checks
    if any(c["unresolved_pose"] for c in checks):
        return {}, dict(info, reason="unresolved_mount_candidate")
    if len(mounts) != 1:
        return {}, dict(info, reason="ambiguous_mount_identity" if mounts else "no_mount_near_umi")
    info["ignored_ego_id1_count"] = len(candidates) - 2
    return dict(zip(ROLES, (mounts[0], external, e2, u1, u2))), dict(info, reason="accepted")


def detect_pair(ego, umi, ego_calibration, umi_calibration, ego_stamp, umi_stamp,
                *, corner_refinement=DEFAULT_CORNER_REFINEMENT):
    validate_corner_refinement(corner_refinement)
    def detect(image, calibration, size, stamp):
        return detect_apriltags(
            image, calibration=calibration,
            config=AprilTagDetectorConfig(family="tag36h11", tag_size_m=size,
                allowed_tag_ids=(1, 2), ambiguity_rotation_deg=2,
                ambiguity_translation_m=0.002, corner_refinement=corner_refinement),
            acquisition_timestamp_ns=int(stamp),
            detector_completed_ns=max(time.monotonic_ns(), int(stamp)),
        ).detections
    e80 = detect(ego, ego_calibration, 0.08, ego_stamp)
    e40 = detect(ego, ego_calibration, 0.04, ego_stamp)
    u80 = detect(umi, umi_calibration, 0.08, umi_stamp)
    roles, info = associate(e80, e40, u80, ego_calibration)
    return roles, info, {"ego": e80, "umi": u80}


def role_geometry(role, d):
    t = _pose(d)
    distance = float(np.linalg.norm(t[:3, 3]))
    angle = float(np.degrees(np.arccos(np.clip(
        abs(t[:3, 2] @ t[:3, 3]) / distance, 0, 1))))
    edge = float(np.min(np.linalg.norm(d.corners_px - np.roll(d.corners_px, 1, axis=0), axis=1)))
    return dict(tag_id=d.tag_id, size_mm=40 if role == "ego_mount" else 80,
                min_edge_px=edge, incidence_deg=angle, range_m=distance,
                corners_px=d.corners_px.tolist(), camera_from_tag=t.tolist(),
                good=edge >= 60 and angle < 45)


def preview_resolved_roles(roles, info, raw):
    """Partial labels for display only; never feed them to calibration."""
    if roles:
        return roles
    partial = {}
    for camera, tag_id, role in (("umi", 1, "umi_ext1"), ("umi", 2, "umi_ext2"),
                                 ("ego", 2, "ego_ext2")):
        found = [d for d in raw[camera] if d.tag_id == tag_id]
        if len(found) == 1 and found[0].accepted:
            partial[role] = found[0]
    corners = info.get("resolved_external_id1_corners_px")
    if corners is not None:
        found = [d for d in raw["ego"] if d.tag_id == 1 and d.accepted
                 and np.max(np.abs(d.corners_px - corners)) < 0.1]
        if len(found) == 1:
            partial["ego_ext1"] = found[0]
    return partial


def geometry(roles):
    result = {role: role_geometry(role, d) for role, d in roles.items()}
    spacing = {camera: float(np.linalg.norm(
        _pose(roles[f"{camera}_ext1"])[:3, 3] -
        _pose(roles[f"{camera}_ext2"])[:3, 3])) for camera in ("ego", "umi")}
    good = all(r["good"] for r in result.values()) and min(spacing.values()) >= 0.20
    return dict(roles=result, external_spacing_m=spacing, good=good)


def mean_pose(poses):
    """Median translation and rotation mean, including exact 180 degree poses."""
    result = np.eye(4)
    result[:3, :3] = Rotation.from_matrix(np.array([p[:3, :3] for p in poses])).mean().as_matrix()
    result[:3, 3] = np.median([p[:3, 3] for p in poses], axis=0)
    return result


def summarize_chains(series, ir_from_color):
    reference = {role: mean_pose(values) for role, values in series.items()}
    chains, ego_umi, ir_mount = {}, [], []
    for tag_id in (1, 2):
        relative = reference[f"ego_ext{tag_id}"] @ invert(reference[f"umi_ext{tag_id}"])
        mount = ir_from_color @ invert(relative) @ reference["ego_mount"]
        ego_umi.append(relative)
        ir_mount.append(mount)
        chains[f"id{tag_id}"] = dict(
            ego_color_from_umi_color=mc._xyz_rpy(relative),
            umi_ir_left_from_mount_tag=mc._xyz_rpy(mount))
    error = pose_error(*ego_umi)
    delta = ir_mount[0][:3, 3] - ir_mount[1][:3, 3]
    agreement = dict(rotation_deg=error.rotation_deg,
                     relative_translation_m=error.translation_m,
                     mount_delta_xyz_m=delta.tolist(),
                     mount_full_translation_difference_m=float(np.linalg.norm(delta)),
                     mount_z_difference_m=abs(float(delta[2])))
    return dict(chains=chains, agreement=agreement,
                consensus_umi_ir_left_from_mount_tag=mc._xyz_rpy(mean_pose(ir_mount)),
                chain_verdict="pass" if error.rotation_deg <= 3 and abs(delta[2]) <= 0.005 else "review")


def audit_stream(session, device):
    stream = f"{device}.color"
    rows = [json.loads(line) for line in (session / device / f"{stream}.jsonl").read_text().splitlines() if line.strip()]
    if any(r["stream_id"] != stream for r in rows):
        raise ValueError("unexpected stream_id")
    formal = [r for r in rows if not r.get("warmup") and r.get("valid")]
    bad = 0
    with (session / device / f"{stream}.bin").open("rb") as f:
        for r in rows:
            f.seek(r["offset"])
            payload = f.read(r["size"])
            bad += int(len(payload) != r["size"] or zlib.crc32(payload) != r["crc32"])
    stamps = np.array([r["acquisition_ns"] for r in formal], dtype=np.int64)
    diffs = np.diff(stamps)
    report = dict(formal_frames=len(formal), crc_checked=len(rows), bad_payloads=bad,
                  invalid_formal_frames=sum(not r.get("warmup") and not r.get("valid") for r in rows),
                  unsupported_clock_rows=sum(
                      r.get("clock_domain") != "realsense_global_time_mapped_to_host_monotonic" or
                      r.get("metadata", {}).get("timestamp_domain") != "global_time"
                      for r in formal),
                  nonincreasing_timestamps=int(np.sum(diffs <= 0)))
    if len(diffs) and stamps[-1] > stamps[0]:
        report.update(hz=float(len(diffs) * 1e9 / (stamps[-1] - stamps[0])),
                      interval_ms_p50_p99_max=np.percentile(diffs / 1e6, [50, 99, 100]).tolist())
    return formal, report


def compute(session, stride=10, *, corner_refinement=DEFAULT_CORNER_REFINEMENT):
    validate_corner_refinement(corner_refinement)
    if stride < 1:
        raise ValueError("stride must be positive")
    session = Path(session)
    erows, eaudit = audit_stream(session, "ego")
    urows, uaudit = audit_stream(session, "left")
    report = dict(schema="ego.mixed_size_two_tag_crosscheck.v1", session=str(session.resolve()),
                  sizes_mm=dict(external_id1=80, external_id2=80, mount_id1=40),
                  thresholds=THRESHOLDS, stride=stride, streams=dict(ego=eaudit, umi=uaudit),
                  detector=dict(backend="opencv_aruco_reference", opencv_version=cv2.__version__,
                                corner_refinement=corner_refinement, pose_solver="IPPE_SQUARE"),
                  activation="NOT_ACTIVATED", absolute_accuracy="NOT_VERIFIED",
                  timestamp_note="host-mapped acquisition pairing; offline completion is not a latency measurement",
                  association_note="ID2 is used only to choose the ID1 quad, not to fit either chain",
                  verdict="blocked")
    if any(a["bad_payloads"] or a["nonincreasing_timestamps"] or a["invalid_formal_frames"] or
           a["unsupported_clock_rows"] or a["formal_frames"] < 8
           for a in (eaudit, uaudit)):
        report["reason"] = "raw_integrity_or_timestamp_gate_failed"
        return report, []
    ec = pta.load_device_calibration(session, "ego", "ego.color")
    uc = pta.load_device_calibration(session, "left", "left.color")
    report['camera_models'] = {name: dict(distortion_model=cal.distortion_model,
        factory_values_modified=False, ray_backend='librealsense' if
        cal.distortion_model == 'distortion.inverse_brown_conrady' else 'opencv')
        for name, cal in (('ego', ec), ('left', uc))}
    ir_from_color, layout_evidence = load_color_to_ir_snapshot(session / "left/calibration.json")
    report['color_to_ir_read'] = layout_evidence
    ustamps = np.array([r["acquisition_ns"] for r in urows], dtype=np.int64)
    audit, series, used = [], {role: [] for role in ROLES}, set()
    for row in erows[::stride]:
        j = int(np.argmin(abs(ustamps - row["acquisition_ns"])))
        skew = int(ustamps[j]) - row["acquisition_ns"]
        entry = dict(ego_sequence=row["sequence"], umi_sequence=urows[j]["sequence"], skew_ms=skew / 1e6)
        audit.append(entry)
        if abs(skew) > 20_000_000 or j in used:
            entry["reason"] = "pair_skew_or_reused_umi_frame"
            continue
        used.add(j)
        roles, info, _ = detect_pair(
            pta.load_frame_image(session, "ego", "ego.color", row),
            pta.load_frame_image(session, "left", "left.color", urows[j]),
            ec, uc, row["acquisition_ns"], int(ustamps[j]),
            corner_refinement=corner_refinement)
        entry.update(info)
        if not roles:
            continue
        entry["geometry"] = geometry(roles)
        # Keep ALL associated poses, even geometry failures; report failures
        # separately, never cherry-pick a cross-check into passing.
        for role in ROLES:
            series[role].append(_pose(roles[role]))
    count = len(series["ego_mount"])
    fraction = count / max(1, len(audit))
    geo_bad = sum(not e["geometry"]["good"] for e in audit if "geometry" in e)
    report.update(sampled_pairs=len(audit), associated_pairs=count,
                  association_fraction=fraction, geometry_failed_pairs=geo_bad,
                  geometry_verdict="pass" if count and not geo_bad else "review")
    if count >= 8:
        report.update(summarize_chains(series, ir_from_color))
        report["verdict"] = ("pass" if report["chain_verdict"] == "pass" and
                              not geo_bad and fraction >= 0.90 else "review")
    else:
        report["reason"] = "too_few_associated_pairs"
    return report, audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=10)
    parser.add_argument("--corner-refinement", choices=CORNER_REFINEMENTS,
                        default=DEFAULT_CORNER_REFINEMENT,
                        help="apriltag: refined frontend (default); none: legacy integer corners")
    args = parser.parse_args()
    dirname = "mixed80_40" if args.corner_refinement == "none" else "mixed80_40_apriltag"
    dirname += "_factory_model_v3"
    out = args.session / "spatial" / dirname
    out.mkdir(parents=True, exist_ok=False)
    print(f"ANALYSIS: corner_refinement={args.corner_refinement} stride={args.stride}; no calibration activation", flush=True)
    report, audit = compute(args.session, args.stride, corner_refinement=args.corner_refinement)
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "pair_audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    print(f"REPORT: {out / 'report.json'}", flush=True)
    return 0 if report["verdict"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
