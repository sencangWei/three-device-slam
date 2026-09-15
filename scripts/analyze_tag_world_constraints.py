#!/usr/bin/env python3
"""Build Tag-ID-aware world constraints and test one fixed world transform.

This is an offline diagnostic over an accepted D435i Ego + D405 UMI pair run.
It never writes a candidate into runtime configuration.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.color_aprilgrid_capture import detect_grid, make_detector
from scripts.fixed_factory_board_diagnostic import valid_detections
from three_device_slam.spatial.se3 import pose_error
from three_device_slam.spatial.tag_world_constraints import (
    TagWorldConstraint,
    analyze_fixed_transform_stability,
    make_world_constraint,
    solve_grid_pose,
)


PHASES = (
    ("initial_static", 0.0, 10.0),
    ("translation", 10.0, 30.0),
    ("yaw", 30.0, 50.0),
    ("pitch", 50.0, 70.0),
    ("roll", 70.0, 85.0),
    ("return", 85.0, 95.0),
    ("final_static", 95.0, 105.0),
)


def load_jsonl(path: Path) -> list[dict]:
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def load_poses(path: Path) -> tuple[np.ndarray, list[np.ndarray]]:
    stamps, poses = [], []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            value = np.eye(4)
            value[:3, :3] = Rotation.from_quat(
                [float(row[key]) for key in ("qx", "qy", "qz", "qw")]
            ).as_matrix()
            value[:3, 3] = [float(row[key]) for key in ("x", "y", "z")]
            stamps.append(int(row["timestamp_ns"]))
            poses.append(value)
    values = np.asarray(stamps, dtype=np.int64)
    if not len(values) or np.any(np.diff(values) <= 0):
        raise ValueError(f"invalid VIO pose timestamps: {path}")
    return values, poses


def nearest_index(stamps: np.ndarray, stamp: int) -> int:
    position = int(np.searchsorted(stamps, stamp))
    choices = (max(0, position - 1), min(len(stamps) - 1, position))
    return min(choices, key=lambda index: abs(int(stamps[index]) - stamp))


def read_image(handle, row: dict) -> np.ndarray:
    handle.seek(int(row["offset"]))
    payload = handle.read(int(row["size"]))
    height, width = int(row["metadata"]["height"]), int(row["metadata"]["width"])
    if len(payload) != height * width or row["metadata"]["encoding"] != "Y8":
        raise ValueError("expected an exact Y8 image payload")
    return np.frombuffer(payload, np.uint8).reshape(height, width)


def detect_grid_scaled(detector, image: np.ndarray, image_scale: int):
    """Detect small board tags at a fixed scale and return source-pixel corners."""
    if image_scale == 1:
        return detect_grid(detector, image)
    scaled = cv2.resize(
        image,
        None,
        fx=image_scale,
        fy=image_scale,
        interpolation=cv2.INTER_LANCZOS4,
    )
    detections = detect_grid(detector, scaled)
    for detection in detections:
        detection.corners = np.asarray(detection.corners, dtype=np.float64) / image_scale
    return detections


def camera_matrix(intrinsics: dict) -> np.ndarray:
    return np.array(
        ((intrinsics["fx"], 0.0, intrinsics["ppx"]),
         (0.0, intrinsics["fy"], intrinsics["ppy"]),
         (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )


def phase_name(elapsed_s: float) -> str | None:
    for name, begin, end in PHASES:
        if begin <= elapsed_s < end:
            return name
    return None


def distribution(values) -> dict:
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(len(data)),
        "median": None if not len(data) else float(np.median(data)),
        "p95": None if not len(data) else float(np.percentile(data, 95)),
        "max": None if not len(data) else float(np.max(data)),
    }


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


def known_id_epipolar_errors(
    fixed_transform: np.ndarray,
    item: dict,
    matrices: dict[str, np.ndarray],
    distortions: dict[str, np.ndarray],
    body_from_camera: dict[str, np.ndarray],
) -> list[float]:
    ego_camera = item["ego_body"] @ body_from_camera["ego"]
    right_camera = fixed_transform @ item["right_body"] @ body_from_camera["right"]
    right_from_ego = np.linalg.inv(right_camera) @ ego_camera
    essential = skew(right_from_ego[:3, 3]) @ right_from_ego[:3, :3]
    focal = np.mean([
        matrices[role][axis, axis] for role in ("ego", "right") for axis in (0, 1)
    ])
    errors = []
    for tag_id in sorted(set(item["ego_pixels"]) & set(item["right_pixels"])):
        first = cv2.undistortPoints(
            np.asarray(item["ego_pixels"][tag_id]).reshape(-1, 1, 2),
            matrices["ego"], distortions["ego"],
        ).reshape(-1, 2)
        second = cv2.undistortPoints(
            np.asarray(item["right_pixels"][tag_id]).reshape(-1, 1, 2),
            matrices["right"], distortions["right"],
        ).reshape(-1, 2)
        x1 = np.column_stack((first, np.ones(len(first))))
        x2 = np.column_stack((second, np.ones(len(second))))
        ex1 = (essential @ x1.T).T
        etx2 = (essential.T @ x2.T).T
        numerator = np.abs(np.sum(x2 * ex1, axis=1))
        denominator = np.sqrt(np.maximum(
            ex1[:, 0] ** 2 + ex1[:, 1] ** 2
            + etx2[:, 0] ** 2 + etx2[:, 1] ** 2,
            1e-18,
        ))
        errors.extend((numerator / denominator * focal).tolist())
    return errors


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def write_plot(path: Path, rows: list[dict], fixed: np.ndarray) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    translation, rotation = [], []
    for row in rows:
        error = pose_error(row["constraint"].ego_world_from_right_world, fixed)
        translation.append(1000.0 * error.translation_m)
        rotation.append(error.rotation_deg)
    colors = {name: plt.get_cmap("tab10")(index) for index, (name, _, _) in enumerate(PHASES)}
    figure, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for axis, values, label in zip(
        axes, (translation, rotation), ("Translation residual (mm)", "Rotation residual (deg)")
    ):
        for phase, _, _ in PHASES:
            selected = [index for index, row in enumerate(rows) if row["constraint"].phase == phase]
            if selected:
                axis.scatter(
                    [rows[index]["elapsed_s"] for index in selected],
                    [values[index] for index in selected], s=16, color=colors[phase], label=phase,
                )
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.25)
    axes[0].legend(ncol=4, fontsize=8)
    axes[1].set_xlabel("Elapsed time (s)")
    figure.suptitle("Tag-ID-aware fixed-transform residuals")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def analyze(
    session: Path,
    run: Path,
    output_dir: Path,
    *,
    stride: int,
    image_scale: int = 1,
) -> dict:
    session, run, output_dir = session.resolve(), run.resolve(), output_dir.resolve()
    if output_dir.exists():
        raise ValueError("output directory must be new")
    if stride < 1:
        raise ValueError("stride must be positive")
    if image_scale not in (1, 2):
        raise ValueError("image_scale must be 1 or 2")
    pair_path = session / "pair_acceptance.json"
    pair = json.loads(pair_path.read_text())
    if pair.get("status") != "PASS":
        raise ValueError("pair acquisition is not accepted")
    start, end = int(pair["formal_start_ns"]), int(pair["formal_end_ns"])
    max_pair_delta_ns = int(
        float(pair["cross_camera_timing"]["threshold_ms_max"]) * 1e6
    )
    if abs((end - start) / 1e9 - 105.0) > 0.1:
        raise ValueError("this phase protocol expects the 105 second capture")
    ego_run_report = json.loads((run / "ego_vins_run/report.json").read_text())
    right_run_report = json.loads((run / "right_vins_run/report.json").read_text())
    if any(report.get("status") != "VIO_REPLAY_PASS"
           for report in (ego_run_report, right_run_report)):
        raise ValueError("both source VIO runs must pass")
    input_reports = {
        "ego": json.loads((run / "ego_vins_input/input_report.json").read_text()),
        "right": json.loads((run / "right_vins_input/input_report.json").read_text()),
    }
    for role in ("ego", "right"):
        if Path(input_reports[role]["source"]).resolve() != session / role:
            raise ValueError(f"{role} VIO input does not belong to the requested session")
        recorded_hash = (ego_run_report if role == "ego" else right_run_report)[
            "source_input_report"
        ]["sha256"]
        if digest(run / f"{role}_vins_input/input_report.json") != recorded_hash:
            raise ValueError(f"{role} VIO input report changed after replay")

    ego_cal = json.loads((session / "ego/calibration.json").read_text())
    right_cal = json.loads((session / "right/calibration.json").read_text())
    matrices = {
        "ego": camera_matrix(ego_cal["intrinsics"]["ir_left"]),
        "right": camera_matrix(right_cal["streams"]["infrared_left"]),
    }
    distortions = {
        "ego": np.asarray(ego_cal["intrinsics"]["ir_left"].get("coefficients", [0.0] * 5)),
        "right": np.asarray(right_cal["streams"]["infrared_left"].get("coefficients", [0.0] * 5)),
    }
    body_from_camera = {
        "ego": np.asarray(input_reports["ego"]["T_body_cam0"]),
        "right": np.asarray(json.loads(
            (run / "covins_right_input/report.json").read_text()
        )["T_body_camera"]),
    }
    image_rows = {
        "ego": [row for row in load_jsonl(session / "ego/ego.ir_left.jsonl")
                if row["valid"] and not row["warmup"]
                and start <= int(row["acquisition_ns"]) < end],
        "right": [row for row in load_jsonl(session / "right/right.ir_left.jsonl")
                  if row["valid"] and not row["warmup"]
                  and start <= int(row["acquisition_ns"]) < end],
    }
    right_image_stamps = np.asarray(
        [row["acquisition_ns"] for row in image_rows["right"]], dtype=np.int64
    )
    pose_stamps, poses = {}, {}
    for role in ("ego", "right"):
        pose_stamps[role], poses[role] = load_poses(run / f"{role}_vins_run/odometry_raw.csv")
    detector = make_detector()
    rows = []
    rejected_duplicate_id_frames = 0
    handles = {
        "ego": (session / "ego/ego.ir_left.bin").open("rb"),
        "right": (session / "right/right.ir_left.bin").open("rb"),
    }
    try:
        for ego_row in image_rows["ego"][::stride]:
            ego_stamp = int(ego_row["acquisition_ns"])
            right_row = image_rows["right"][nearest_index(right_image_stamps, ego_stamp)]
            right_stamp = int(right_row["acquisition_ns"])
            if abs(right_stamp - ego_stamp) > max_pair_delta_ns:
                continue
            try:
                detected = {
                    role: valid_detections(detect_grid_scaled(
                        detector, read_image(handles[role], row), image_scale
                    ))
                    for role, row in (("ego", ego_row), ("right", right_row))
                }
            except ValueError:
                rejected_duplicate_id_frames += 1
                continue
            solved = {
                role: solve_grid_pose(
                    detected[role], matrices[role], distortions[role]
                )
                for role in ("ego", "right")
            }
            if any(value is None or value.reprojection_rms_px > 3.0
                   for value in solved.values()):
                continue
            indices = {
                "ego": nearest_index(pose_stamps["ego"], ego_stamp),
                "right": nearest_index(pose_stamps["right"], right_stamp),
            }
            if any(abs(int(pose_stamps[role][indices[role]]) - stamp) > 1000
                   for role, stamp in (("ego", ego_stamp), ("right", right_stamp))):
                continue
            phase = phase_name((ego_stamp - start) / 1e9)
            if phase is None:
                continue
            constraint = make_world_constraint(
                ego_stamp_ns=ego_stamp, right_stamp_ns=right_stamp, phase=phase,
                ego_world_from_body=poses["ego"][indices["ego"]],
                right_world_from_body=poses["right"][indices["right"]],
                ego_body_from_camera=body_from_camera["ego"],
                right_body_from_camera=body_from_camera["right"],
                ego_grid_pose=solved["ego"], right_grid_pose=solved["right"],
            )
            rows.append({
                "elapsed_s": (ego_stamp - start) / 1e9,
                "constraint": constraint,
                "ego_body": poses["ego"][indices["ego"]],
                "right_body": poses["right"][indices["right"]],
                "ego_pixels": detected["ego"], "right_pixels": detected["right"],
            })
    finally:
        for handle in handles.values():
            handle.close()
    constraints = [row["constraint"] for row in rows]
    stability = analyze_fixed_transform_stability(
        constraints,
        phase_order=[name for name, _, _ in PHASES],
        required_phases=("initial_static", "final_static"),
    )
    fixed = np.asarray(stability["fixed_transform"])
    # Match the deterministic phase-local holdout split used by the pure module.
    heldout_rows = []
    for name, _, _ in PHASES:
        heldout_rows.extend([row for row in rows if row["constraint"].phase == name][1::2])
    epipolar_by_phase = {}
    for name, _, _ in PHASES:
        errors = [value for row in heldout_rows if row["constraint"].phase == name
                  for value in known_id_epipolar_errors(
                      fixed, row, matrices, distortions, body_from_camera
                  )]
        epipolar_by_phase[name] = distribution(errors)
    epipolar = distribution([
        value for row in heldout_rows
        for value in known_id_epipolar_errors(
            fixed, row, matrices, distortions, body_from_camera
        )
    ])
    phase_anchors = {
        name: np.asarray(value)
        for name, value in stability["phase_anchors"].items()
    }
    phase_anchor_epipolar_by_phase = {}
    phase_holdout = {}
    for name, _, _ in PHASES:
        selected = [
            row for row in heldout_rows if row["constraint"].phase == name
        ]
        anchor = phase_anchors.get(name)
        errors = [] if anchor is None else [
            value
            for row in selected
            for value in known_id_epipolar_errors(
                anchor, row, matrices, distortions, body_from_camera
            )
        ]
        phase_anchor_epipolar_by_phase[name] = distribution(errors)
        residuals = [] if anchor is None else [
            pose_error(row["constraint"].ego_world_from_right_world, anchor)
            for row in selected
        ]
        phase_holdout[name] = {
            "translation_m": distribution(
                [residual.translation_m for residual in residuals]
            ),
            "rotation_deg": distribution(
                [residual.rotation_deg for residual in residuals]
            ),
        }
    phase_anchor_epipolar_values = [
        value
        for row in heldout_rows
        if row["constraint"].phase in phase_anchors
        for value in known_id_epipolar_errors(
            phase_anchors[row["constraint"].phase],
            row,
            matrices,
            distortions,
            body_from_camera,
        )
    ]
    phase_anchor_epipolar = distribution(phase_anchor_epipolar_values)
    policy = stability["policy"]
    updates = []
    previous_name = None
    for name, _, _ in PHASES:
        if name not in phase_anchors:
            continue
        if previous_name is not None:
            error = pose_error(phase_anchors[name], phase_anchors[previous_name])
            updates.append({
                "from": previous_name,
                "to": name,
                "translation_m": error.translation_m,
                "rotation_deg": error.rotation_deg,
                "accepted": (
                    error.translation_m
                    <= policy["max_holdout_translation_p95_m"]
                    and error.rotation_deg
                    <= policy["max_holdout_rotation_p95_deg"]
                ),
            })
        previous_name = name
    observed_holdout = [
        value for value in phase_holdout.values()
        if value["translation_m"]["count"] > 0
    ]
    incremental_checks = {
        "required_phase_coverage": stability["selection"]
        ["required_phase_coverage"]["initial_static"]
        and stability["selection"]["required_phase_coverage"]["final_static"],
        "all_incremental_updates_accepted": bool(updates)
        and all(update["accepted"] for update in updates),
        "each_observed_phase_translation_p95_within_policy":
        bool(observed_holdout) and all(
            value["translation_m"]["p95"]
            <= policy["max_holdout_translation_p95_m"]
            for value in observed_holdout
        ),
        "each_observed_phase_rotation_p95_within_policy":
        bool(observed_holdout) and all(
            value["rotation_deg"]["p95"]
            <= policy["max_holdout_rotation_p95_deg"]
            for value in observed_holdout
        ),
    }
    incremental = {
        "status": (
            "PASS_PHASE_ANCHOR_RELOCALIZATION_DIAGNOSTIC"
            if all(incremental_checks.values())
            else "FAIL_PHASE_ANCHOR_RELOCALIZATION_DIAGNOSTIC"
        ),
        "checks": incremental_checks,
        "updates": updates,
        "holdout_by_phase": phase_holdout,
        "known_id_holdout_epipolar_px": {
            "overall": phase_anchor_epipolar,
            "phases": phase_anchor_epipolar_by_phase,
        },
        "policy": policy,
    }
    pnp = {
        role: distribution([getattr(item, f"{role}_pnp_rms_px") for item in constraints])
        for role in ("ego", "right")
    }
    checks = {
        "source_capture_pass": True,
        "both_vio_runs_pass": True,
        "pnp_rms_p95_le_3px": max(pnp["ego"]["p95"], pnp["right"]["p95"]) <= 3.0,
        "fixed_transform_stability_and_endpoint_coverage":
            stability["status"] == "PASS_FIXED_TRANSFORM_STABLE_DIAGNOSTIC",
        "known_id_epipolar_p95_le_1p5px_diagnostic_reference": epipolar["p95"] <= 1.5,
    }
    status = "PASS_CANDIDATE_NOT_ACTIVATED" if all(checks.values()) \
        else "REJECT_FIXED_TRANSFORM_NOT_ACTIVATED"
    output_dir.mkdir(parents=True)
    constraints_path = output_dir / "tag_world_constraints.jsonl"
    with constraints_path.open("w") as stream:
        for row in rows:
            item: TagWorldConstraint = row["constraint"]
            error = pose_error(item.ego_world_from_right_world, fixed)
            rotation = Rotation.from_matrix(
                np.array(item.ego_world_from_right_world[:3, :3], copy=True)
            )
            stream.write(json.dumps({
                "ego_stamp_ns": item.ego_stamp_ns,
                "right_stamp_ns": item.right_stamp_ns,
                "time_delta_ms": (item.right_stamp_ns - item.ego_stamp_ns) / 1e6,
                "elapsed_s": row["elapsed_s"], "phase": item.phase,
                "ego_tag_ids": item.ego_tag_ids, "right_tag_ids": item.right_tag_ids,
                "common_tag_ids": sorted(set(item.ego_tag_ids) & set(item.right_tag_ids)),
                "ego_pnp_rms_px": item.ego_pnp_rms_px,
                "right_pnp_rms_px": item.right_pnp_rms_px,
                "ego_world_from_right_world": item.ego_world_from_right_world.tolist(),
                "ego_world_from_grid": item.ego_world_from_grid.tolist(),
                "right_world_from_grid": item.right_world_from_grid.tolist(),
                "translation_m": item.ego_world_from_right_world[:3, 3].tolist(),
                "quaternion_xyzw": rotation.as_quat().tolist(),
                "fixed_residual_translation_m": error.translation_m,
                "fixed_residual_rotation_deg": error.rotation_deg,
            }, sort_keys=True) + "\n")
    plot_path = output_dir / "constraint_stability.png"
    write_plot(plot_path, rows, fixed)
    source_files = [
        Path(__file__).resolve(),
        Path(__file__).resolve().parents[1]
        / "three_device_slam/spatial/tag_world_constraints.py",
        pair_path,
        session / "ego/calibration.json", session / "right/calibration.json",
        run / "ego_vins_run/report.json", run / "right_vins_run/report.json",
        run / "ego_vins_run/odometry_raw.csv", run / "right_vins_run/odometry_raw.csv",
        run / "ego_vins_input/input_report.json", run / "covins_right_input/report.json",
    ]
    report = {
        "schema": "three-device-slam.tag-world-constraints.v1",
        "status": status,
        "activation": "NOT_ACTIVATED",
        "session": str(session), "run": str(run),
        "board": {
            "family": "tag36h11", "rows": 6, "columns": 6,
            "tag_size_m": 0.0352, "gap_m": 0.01056, "marker_border_bits": 2,
            "association": "decoded tag ID to frozen metric grid corners",
        },
        "selection": {
            "stride": stride,
            "image_scale": image_scale,
            "constraint_count": len(constraints),
            "rejected_duplicate_id_frames": rejected_duplicate_id_frames,
        },
        "checks": checks,
        "fixed_transform_stability": stability,
        "known_id_holdout_epipolar_px": {"overall": epipolar, "phases": epipolar_by_phase},
        "incremental_phase_anchor_relocalization": incremental,
        "pnp_rms_px": pnp,
        "outputs": {
            "constraints_jsonl": str(constraints_path),
            "stability_plot": str(plot_path),
        },
        "provenance": {str(path): digest(path) for path in source_files},
        "decision": (
            "Do not freeze the fixed transform. Required endpoint coverage is missing."
            if not incremental_checks["required_phase_coverage"]
            else "Do not freeze one whole-run transform. Evaluate the separately reported "
            "quality-gated incremental phase-anchor relocalization path."
        ),
        "recommended_model": "TAG_FACTOR_OR_TIME_VARYING_WORLD_CORRECTION",
        "policy_note": (
            "The 1.5 px value is retained only as the prior per-correspondence diagnostic "
            "reference; it is not a frozen whole-run product release requirement."
        ),
    }
    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--image-scale", type=int, choices=(1, 2), default=1)
    args = parser.parse_args()
    report = analyze(
        args.session,
        args.run,
        args.output_dir,
        stride=args.stride,
        image_scale=args.image_scale,
    )
    print(json.dumps({
        "status": report["status"], "checks": report["checks"],
        "selection": report["selection"],
        "fixed_transform_stability": report["fixed_transform_stability"],
        "known_id_holdout_epipolar_px": report["known_id_holdout_epipolar_px"],
        "incremental_phase_anchor_relocalization":
            report["incremental_phase_anchor_relocalization"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
