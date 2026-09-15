#!/usr/bin/env python3
"""Accept the 45 s device3 ID1 anchor HIL against the fixed AprilGrid.

The AprilGrid defines the independent validation world.  The Ego camera and
grid are physically fixed during the formal window.  Device3 may move, while
its camera, IMU and mounted tag remain one rigid assembly.  This separates a
device3 mount/anchor result from the provisional D435i VINS static yaw drift.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.analyze_tag_world_constraints import detect_grid_scaled
from scripts.color_aprilgrid_capture import make_detector
from scripts.fixed_factory_board_diagnostic import valid_detections
from three_device_slam.devices.d405_umi.pair_vins_export import IMU_RECORD
from three_device_slam.spatial.alignment import PoseSample, sample_pose_at_time
from three_device_slam.spatial.apriltag_detector import (
    AprilTagDetectorConfig,
    detect_apriltags,
)
from three_device_slam.spatial.pair_tag_alignment import (
    load_device_calibration,
    load_formal_index_rows,
)
from three_device_slam.spatial.se3 import compose, invert, pose_error, validate_transform
from three_device_slam.spatial.tag_world_constraints import aggregate_transform, solve_grid_pose


PHASES = (
    ("initial_static", 0.0, 10.0),
    ("translation", 10.0, 22.0),
    ("rotation", 22.0, 32.0),
    ("occlusion", 32.0, 37.0),
    ("return_final", 37.0, 45.0),
)
POLICY = {
    "stride": 3,
    "image_scale": 2,
    "max_pair_delta_ms": 20.0,
    "max_pose_interpolation_gap_ms": 40.0,
    "stationary_gyro_max_deg_s": 3.0,
    "initial_consensus_window_s": [2.0, 10.0],
    "final_consensus_window_s": [43.0, 45.0],
    "anchor_translation_max_m": 0.03,
    "anchor_rotation_max_deg": 3.0,
    "static_crosscheck_translation_p95_max_m": 0.01,
    "static_crosscheck_rotation_p95_max_deg": 3.0,
    "minimum_occlusion_s": 1.5,
    "maximum_occlusion_s": 4.0,
}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def phase_name(elapsed_s: float) -> str | None:
    return next((name for name, begin, end in PHASES if begin <= elapsed_s < end), None)


def distribution(values) -> dict:
    data = np.asarray(tuple(values), dtype=np.float64)
    if not len(data):
        return {"count": 0, "median": None, "p95": None, "max": None}
    return {
        "count": int(len(data)),
        "median": float(np.median(data)),
        "p95": float(np.percentile(data, 95)),
        "max": float(np.max(data)),
    }


def consensus_anchor(transforms: list[np.ndarray]) -> tuple[np.ndarray, list[int]]:
    """Use the runtime 30 mm / 3 degree medoid consensus policy."""
    if not transforms:
        raise ValueError("anchor consensus has no candidates")
    best = None
    for index, candidate in enumerate(transforms):
        errors = [pose_error(other, candidate) for other in transforms]
        inliers = [
            item for item, error in enumerate(errors)
            if error.translation_m <= POLICY["anchor_translation_max_m"]
            and error.rotation_deg <= POLICY["anchor_rotation_max_deg"]
        ]
        cost = sum(
            errors[item].translation_m / POLICY["anchor_translation_max_m"]
            + errors[item].rotation_deg / POLICY["anchor_rotation_max_deg"]
            for item in inliers
        )
        key = (-len(inliers), cost, index)
        if best is None or key < best[0]:
            best = (key, candidate, inliers)
    return np.array(best[1], copy=True), best[2]


def largest_observation_gap(elapsed_s: list[float]) -> dict:
    if len(elapsed_s) < 2:
        return {"duration_s": 0.0, "before_s": None, "after_s": None}
    before, after = max(zip(elapsed_s, elapsed_s[1:]), key=lambda pair: pair[1] - pair[0])
    return {"duration_s": float(after - before), "before_s": before, "after_s": after}


def _load_poses(path: Path) -> tuple[PoseSample, ...]:
    samples = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_quat(
                [float(row[key]) for key in ("qx", "qy", "qz", "qw")]
            ).as_matrix()
            transform[:3, 3] = [float(row[key]) for key in ("x", "y", "z")]
            samples.append(PoseSample(int(row["timestamp_ns"]), transform))
    return tuple(samples)


def _read_image(handle, row: dict) -> np.ndarray:
    handle.seek(int(row["offset"]))
    payload = handle.read(int(row["size"]))
    height = int(row["metadata"]["height"])
    width = int(row["metadata"]["width"])
    if row["metadata"]["encoding"] != "Y8" or len(payload) != height * width:
        raise ValueError("anchor HIL requires exact Y8 image payloads")
    return np.frombuffer(payload, np.uint8).reshape(height, width)


def _nearest_index(stamps: np.ndarray, stamp: int) -> int:
    position = int(np.searchsorted(stamps, stamp))
    choices = (max(0, position - 1), min(len(stamps) - 1, position))
    return min(choices, key=lambda index: abs(int(stamps[index]) - stamp))


def _load_gyro(path: Path) -> tuple[np.ndarray, np.ndarray]:
    payload = path.read_bytes()
    if len(payload) % IMU_RECORD.size:
        raise ValueError("external IMU payload has a partial record")
    rows = [
        IMU_RECORD.unpack_from(payload, offset)
        for offset in range(0, len(payload), IMU_RECORD.size)
    ]
    stamps = np.asarray([round(row[0] * 1e9) for row in rows], dtype=np.int64)
    norms = np.asarray([np.linalg.norm(row[2:5]) for row in rows], dtype=np.float64)
    if len(stamps) < 2 or np.any(np.diff(stamps) <= 0):
        raise ValueError("external IMU timestamps are not strictly increasing")
    return stamps, norms


def _residual_summary(rows: list[dict], key: str, anchor: np.ndarray) -> dict:
    errors = [pose_error(row[key], anchor) for row in rows]
    return {
        "translation_m": distribution(error.translation_m for error in errors),
        "rotation_deg": distribution(error.rotation_deg for error in errors),
    }


def _crosscheck_summary(rows: list[dict]) -> dict:
    errors = [row["mount_vs_board"] for row in rows]
    return {
        "translation_m": distribution(error.translation_m for error in errors),
        "rotation_deg": distribution(error.rotation_deg for error in errors),
    }


def _write_plot(
    path: Path,
    rows: list[dict],
    mount_initial: np.ndarray,
    board_initial: np.ndarray,
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed = [row["elapsed_s"] for row in rows]
    mount_errors = [pose_error(row["mount_anchor"], mount_initial) for row in rows]
    board_errors = [pose_error(row["board_anchor"], board_initial) for row in rows]
    figure, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(elapsed, [1000 * item.translation_m for item in mount_errors], ".", ms=3, label="ID1 mount path")
    axes[0].plot(elapsed, [1000 * item.translation_m for item in board_errors], ".", ms=3, label="direct AprilGrid path")
    axes[0].axhline(30, color="black", ls="--", lw=1, label="30 mm policy")
    axes[0].set_ylabel("anchor translation (mm)")
    axes[1].plot(elapsed, [item.rotation_deg for item in mount_errors], ".", ms=3, label="ID1 mount path")
    axes[1].plot(elapsed, [item.rotation_deg for item in board_errors], ".", ms=3, label="direct AprilGrid path")
    axes[1].axhline(3, color="black", ls="--", lw=1, label="3 deg policy")
    axes[1].set_ylabel("anchor rotation (deg)")
    axes[2].plot(elapsed, [row["gyro_norm_deg_s"] for row in rows], ".", ms=3, color="tab:purple")
    axes[2].axhline(POLICY["stationary_gyro_max_deg_s"], color="black", ls="--", lw=1)
    axes[2].set_ylabel("device3 gyro (deg/s)")
    axes[2].set_xlabel("formal elapsed time (s)")
    colors = plt.get_cmap("Pastel1")
    for index, (name, begin, end) in enumerate(PHASES):
        for axis in axes:
            axis.axvspan(begin, end, color=colors(index), alpha=0.16)
            axis.grid(True, alpha=0.25)
        axes[0].text((begin + end) / 2, 31, name, ha="center", va="bottom", fontsize=8)
    axes[0].legend(loc="upper left", fontsize=8)
    axes[1].legend(loc="upper left", fontsize=8)
    figure.suptitle("Device3 ID1 anchor HIL: fixed-AprilGrid cross-check")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def analyze(session: Path, run: Path, candidate_path: Path, output: Path) -> dict:
    session, run, candidate_path, output = map(
        lambda path: Path(path).resolve(), (session, run, candidate_path, output)
    )
    if output.exists():
        raise ValueError("output directory must be new")
    pair = json.loads((session / "pair_acceptance.json").read_text())
    coordinator = json.loads((session / "coordinator.json").read_text())
    sync = json.loads((session / "sync/manifest.json").read_text())
    if pair.get("status") != "PASS" or pair.get("mode") != "device3_anchor_hil":
        raise ValueError("source is not an accepted device3 anchor HIL")
    start, end = int(pair["formal_start_ns"]), int(pair["formal_end_ns"])
    if abs((end - start) / 1e9 - 45.0) > 0.1:
        raise ValueError("device3 anchor HIL must be 45 seconds")
    reports = {
        role: json.loads((run / f"{role}_vins_run/report.json").read_text())
        for role in ("ego", "right")
    }
    input_reports = {
        role: json.loads((run / f"{role}_vins_input/input_report.json").read_text())
        for role in ("ego", "right")
    }
    for role in reports:
        if reports[role].get("status") != "VIO_REPLAY_PASS":
            raise ValueError(f"{role} VIO replay is not accepted")
        if Path(input_reports[role]["source"]).resolve() != session / role:
            raise ValueError(f"{role} VIO input belongs to another session")
        if digest(run / f"{role}_vins_input/input_report.json") != reports[role][
            "source_input_report"
        ]["sha256"]:
            raise ValueError(f"{role} VIO input report changed after replay")
    candidate = json.loads(candidate_path.read_text())
    if (
        candidate.get("activation") != "NOT_ACTIVATED"
        or candidate.get("tag", {}).get("id") != 1
        or candidate.get("tag", {}).get("nominal_black_square_m") != 0.04
    ):
        raise ValueError("unexpected device3 mount candidate identity")
    tag_from_body = validate_transform(
        np.asarray(candidate["transforms"]["T_mount_tag_from_vins_body"]),
        name="T_mount_tag_from_vins_body",
    )
    right_body_from_camera = validate_transform(
        np.asarray(candidate["transforms"]["T_vins_body_from_right_ir_left"]),
        name="T_vins_body_from_right_ir_left",
    )
    ego_body_from_camera = validate_transform(
        np.asarray(input_reports["ego"]["T_body_cam0"]), name="ego T_body_cam0"
    )
    ego_poses = _load_poses(run / "ego_vins_run/odometry_raw.csv")
    right_poses = _load_poses(run / "right_vins_run/odometry_raw.csv")
    image_rows = {
        "ego": [
            row for row in load_formal_index_rows(session, "ego", "ego.ir_left")
            if start <= int(row["acquisition_ns"]) < end
        ],
        "right": [
            row for row in load_formal_index_rows(session, "right", "right.ir_left")
            if start <= int(row["acquisition_ns"]) < end
        ],
    }
    right_stamps = np.asarray(
        [row["acquisition_ns"] for row in image_rows["right"]], dtype=np.int64
    )
    calibrations = {
        "ego": load_device_calibration(session, "ego", "ego.ir_left"),
        "right": load_device_calibration(session, "right", "right.infrared_left"),
    }
    tag_config = AprilTagDetectorConfig(
        family="tag36h11", tag_size_m=0.04, allowed_tag_ids=(1,),
        max_reprojection_error_px=1.5, ambiguity_error_gap_px=0.05,
        ambiguity_translation_m=0.005, ambiguity_rotation_deg=5.0,
        corner_refinement="apriltag",
    )
    board_detector = make_detector()
    gyro_stamps, gyro_norms = _load_gyro(session / "right/external_imu/imu.bin")
    accepted_tag_times = []
    rejection_reasons: dict[str, int] = {}
    phase_frames: dict[str, int] = {name: 0 for name, _, _ in PHASES}
    rows = []
    max_pair_delta_ns = int(POLICY["max_pair_delta_ms"] * 1e6)
    max_pose_gap_ns = int(POLICY["max_pose_interpolation_gap_ms"] * 1e6)
    with (session / "ego/ego.ir_left.bin").open("rb") as ego_file, (
        session / "right/right.ir_left.bin"
    ).open("rb") as right_file:
        for ego_row in image_rows["ego"][:: POLICY["stride"]]:
            ego_stamp = int(ego_row["acquisition_ns"])
            elapsed = (ego_stamp - start) / 1e9
            phase = phase_name(elapsed)
            if phase is None:
                continue
            phase_frames[phase] += 1
            right_row = image_rows["right"][_nearest_index(right_stamps, ego_stamp)]
            right_stamp = int(right_row["acquisition_ns"])
            if abs(right_stamp - ego_stamp) > max_pair_delta_ns:
                continue
            images = {
                "ego": _read_image(ego_file, ego_row),
                "right": _read_image(right_file, right_row),
            }
            batch = detect_apriltags(
                images["ego"], calibration=calibrations["ego"], config=tag_config,
                acquisition_timestamp_ns=ego_stamp,
                detector_completed_ns=time.monotonic_ns(),
            )
            tag_detections = [item for item in batch.detections if item.tag_id == 1]
            for detection in tag_detections:
                rejection_reasons[detection.reason] = (
                    rejection_reasons.get(detection.reason, 0) + 1
                )
            accepted = [item for item in tag_detections if item.accepted]
            if len(accepted) == 1:
                accepted_tag_times.append(elapsed)
            else:
                continue
            grids = {}
            for role in ("ego", "right"):
                try:
                    detections = valid_detections(detect_grid_scaled(
                        board_detector, images[role], POLICY["image_scale"]
                    ))
                except ValueError:
                    detections = {}
                grids[role] = solve_grid_pose(
                    detections,
                    calibrations[role].camera_matrix,
                    calibrations[role].distortion_coefficients,
                )
            if any(pose is None or pose.reprojection_rms_px > 3.0 for pose in grids.values()):
                continue
            try:
                right_at_ego = sample_pose_at_time(
                    right_poses, ego_stamp, max_gap_ns=max_pose_gap_ns
                )
                right_at_right = sample_pose_at_time(
                    right_poses, right_stamp, max_gap_ns=max_pose_gap_ns
                )
                ego_at_ego = sample_pose_at_time(
                    ego_poses, ego_stamp, max_gap_ns=max_pose_gap_ns
                )
            except ValueError:
                continue
            gyro_index = _nearest_index(gyro_stamps, ego_stamp)
            grid_from_right_body_mount = compose(
                invert(grids["ego"].camera_from_grid),
                accepted[0].observation.camera_from_tag,
                tag_from_body,
            )
            grid_from_right_body_board = compose(
                invert(grids["right"].camera_from_grid),
                invert(right_body_from_camera),
            )
            mount_anchor = compose(grid_from_right_body_mount, invert(right_at_ego))
            board_anchor = compose(grid_from_right_body_board, invert(right_at_right))
            ego_vins_anchor = compose(
                ego_at_ego,
                ego_body_from_camera,
                accepted[0].observation.camera_from_tag,
                tag_from_body,
                invert(right_at_ego),
            )
            rows.append({
                "ego_stamp_ns": ego_stamp,
                "right_stamp_ns": right_stamp,
                "elapsed_s": elapsed,
                "phase": phase,
                "pair_delta_ms": (right_stamp - ego_stamp) / 1e6,
                "gyro_norm_deg_s": float(gyro_norms[gyro_index]),
                "stationary": bool(
                    gyro_norms[gyro_index] <= POLICY["stationary_gyro_max_deg_s"]
                ),
                "tag_reprojection_px": accepted[0].observation.reprojection_error_px,
                "ego_grid_reprojection_px": grids["ego"].reprojection_rms_px,
                "right_grid_reprojection_px": grids["right"].reprojection_rms_px,
                "mount_anchor": mount_anchor,
                "board_anchor": board_anchor,
                "ego_vins_anchor": ego_vins_anchor,
                "mount_vs_board": pose_error(
                    grid_from_right_body_mount, grid_from_right_body_board
                ),
            })
    initial = [
        row for row in rows
        if POLICY["initial_consensus_window_s"][0] <= row["elapsed_s"]
        < POLICY["initial_consensus_window_s"][1] and row["stationary"]
    ]
    final = [
        row for row in rows
        if POLICY["final_consensus_window_s"][0] <= row["elapsed_s"]
        < POLICY["final_consensus_window_s"][1] and row["stationary"]
    ]
    mount_initial, mount_initial_inliers = consensus_anchor(
        [row["mount_anchor"] for row in initial]
    )
    board_initial, board_initial_inliers = consensus_anchor(
        [row["board_anchor"] for row in initial]
    )
    ego_vins_initial, _ = consensus_anchor([row["ego_vins_anchor"] for row in initial])
    mount_final, mount_final_inliers = consensus_anchor(
        [row["mount_anchor"] for row in final]
    )
    board_final, board_final_inliers = consensus_anchor(
        [row["board_anchor"] for row in final]
    )
    ego_vins_final, _ = consensus_anchor([row["ego_vins_anchor"] for row in final])
    endpoint = {
        "mount_tag_path": pose_error(mount_final, mount_initial),
        "direct_board_path": pose_error(board_final, board_initial),
        "provisional_ego_vins_path": pose_error(ego_vins_final, ego_vins_initial),
    }
    initial_residual = _residual_summary(initial, "mount_anchor", mount_initial)
    final_residual = _residual_summary(final, "mount_anchor", mount_final)
    static_crosscheck = _crosscheck_summary(initial + final)
    gap = largest_observation_gap(accepted_tag_times)
    phase_metrics = {}
    for name, _, _ in PHASES:
        selected = [row for row in rows if row["phase"] == name]
        phase_metrics[name] = {
            "processed_frames": phase_frames[name],
            "joint_solution_count": len(selected),
            "mount_anchor_from_initial": _residual_summary(
                selected, "mount_anchor", mount_initial
            ),
            "board_anchor_from_initial": _residual_summary(
                selected, "board_anchor", board_initial
            ),
            "mount_vs_direct_board": _crosscheck_summary(selected),
            "tag_reprojection_px": distribution(
                row["tag_reprojection_px"] for row in selected
            ),
            "right_gyro_deg_s": distribution(
                row["gyro_norm_deg_s"] for row in selected
            ),
        }
    checks = {
        "source_transport_coordinator_and_sync_pass": (
            pair["status"] == "PASS"
            and coordinator.get("status") == "PASS"
            and sync.get("row_count") == 1350
        ),
        "both_vio_replays_pass": all(
            report["status"] == "VIO_REPLAY_PASS" for report in reports.values()
        ),
        "static_mount_candidate_and_print_scale_pass": all((
            candidate["checks"]["cprime_independent_all_gates_pass"],
            candidate["checks"]["print_pipeline_40mm_actual_size_hash_verified"],
            candidate["checks"]["operator_confirms_installed_tag_matches_printed_asset"],
        )),
        "initial_stationary_joint_samples_ge_30": len(initial) >= 30,
        "final_stationary_joint_samples_ge_10": len(final) >= 10,
        "initial_anchor_consensus_ge_90pct": (
            len(mount_initial_inliers) >= 0.9 * len(initial)
        ),
        "final_anchor_consensus_ge_90pct": (
            len(mount_final_inliers) >= 0.9 * len(final)
        ),
        "initial_anchor_translation_p95_le_30mm": (
            initial_residual["translation_m"]["p95"]
            <= POLICY["anchor_translation_max_m"]
        ),
        "initial_anchor_rotation_p95_le_3deg": (
            initial_residual["rotation_deg"]["p95"]
            <= POLICY["anchor_rotation_max_deg"]
        ),
        "id1_occlusion_1p5_to_4s": (
            POLICY["minimum_occlusion_s"] <= gap["duration_s"]
            <= POLICY["maximum_occlusion_s"]
        ),
        "id1_reacquired_in_final_window": bool(final),
        "final_anchor_update_translation_le_30mm": (
            endpoint["mount_tag_path"].translation_m
            <= POLICY["anchor_translation_max_m"]
        ),
        "final_anchor_update_rotation_le_3deg": (
            endpoint["mount_tag_path"].rotation_deg
            <= POLICY["anchor_rotation_max_deg"]
        ),
        "final_anchor_translation_p95_le_30mm": (
            final_residual["translation_m"]["p95"]
            <= POLICY["anchor_translation_max_m"]
        ),
        "final_anchor_rotation_p95_le_3deg": (
            final_residual["rotation_deg"]["p95"]
            <= POLICY["anchor_rotation_max_deg"]
        ),
        "static_mount_board_crosscheck_translation_p95_le_10mm": (
            static_crosscheck["translation_m"]["p95"]
            <= POLICY["static_crosscheck_translation_p95_max_m"]
        ),
        "static_mount_board_crosscheck_rotation_p95_le_3deg": (
            static_crosscheck["rotation_deg"]["p95"]
            <= POLICY["static_crosscheck_rotation_p95_max_deg"]
        ),
        "direct_board_vio_endpoint_translation_le_30mm": (
            endpoint["direct_board_path"].translation_m
            <= POLICY["anchor_translation_max_m"]
        ),
        "direct_board_vio_endpoint_rotation_le_3deg": (
            endpoint["direct_board_path"].rotation_deg
            <= POLICY["anchor_rotation_max_deg"]
        ),
    }
    status = (
        "PASS_DEVICE3_ANCHOR_HIL"
        if all(checks.values())
        else "FAIL_DEVICE3_ANCHOR_HIL_NOT_ACTIVATED"
    )
    output.mkdir(parents=True)
    plot_path = output / "anchor_continuity.png"
    _write_plot(plot_path, rows, mount_initial, board_initial)
    rows_path = output / "anchor_constraints.jsonl"
    with rows_path.open("w") as stream:
        for row in rows:
            item = {key: value for key, value in row.items() if key not in {
                "mount_anchor", "board_anchor", "ego_vins_anchor", "mount_vs_board"
            }}
            item.update({
                "mount_anchor": row["mount_anchor"].tolist(),
                "board_anchor": row["board_anchor"].tolist(),
                "ego_vins_anchor": row["ego_vins_anchor"].tolist(),
                "mount_vs_board_translation_m": row["mount_vs_board"].translation_m,
                "mount_vs_board_rotation_deg": row["mount_vs_board"].rotation_deg,
            })
            stream.write(json.dumps(item, sort_keys=True) + "\n")
    sources = [
        Path(__file__).resolve(), session / "session.seal.json",
        session / "pair_acceptance.json", session / "coordinator.json",
        session / "sync/manifest.json", run / "ego_vins_run/report.json",
        run / "right_vins_run/report.json", run / "ego_vins_run/odometry_raw.csv",
        run / "right_vins_run/odometry_raw.csv", candidate_path,
    ]
    report = {
        "schema": "three-device-slam.device3-anchor-hil.v1",
        "status": status,
        "activation": "NOT_ACTIVATED",
        "session": str(session),
        "run": str(run),
        "candidate": str(candidate_path),
        "world_definition": "fixed AprilGrid; T_A_B maps frame B into frame A",
        "policy": POLICY,
        "checks": checks,
        "selection": {
            "formal_frames": len(image_rows["ego"]),
            "processed_frames": sum(phase_frames.values()),
            "accepted_id1_observations": len(accepted_tag_times),
            "joint_tag_board_vio_solutions": len(rows),
            "initial_stationary_solutions": len(initial),
            "initial_consensus_inliers": len(mount_initial_inliers),
            "final_stationary_solutions": len(final),
            "final_consensus_inliers": len(mount_final_inliers),
            "tag_detection_reasons": rejection_reasons,
        },
        "occlusion": gap,
        "endpoint_anchor_update": {
            name: {
                "translation_m": error.translation_m,
                "rotation_deg": error.rotation_deg,
            }
            for name, error in endpoint.items()
        },
        "initial_anchor_residual": initial_residual,
        "final_anchor_residual": final_residual,
        "static_mount_vs_direct_board": static_crosscheck,
        "phase_metrics": phase_metrics,
        "transforms": {
            "T_aprilgrid_from_right_world_initial_mount_path": mount_initial.tolist(),
            "T_aprilgrid_from_right_world_final_mount_path": mount_final.tolist(),
            "T_aprilgrid_from_right_world_initial_direct_board_path": board_initial.tolist(),
            "T_aprilgrid_from_right_world_final_direct_board_path": board_final.tolist(),
        },
        "diagnostics": {
            "provisional_ego_vins_endpoint_is_not_a_device3_gate": {
                "translation_m": endpoint["provisional_ego_vins_path"].translation_m,
                "rotation_deg": endpoint["provisional_ego_vins_path"].rotation_deg,
                "reason": "Ego and AprilGrid were physically fixed; the fixed-board path removes provisional D435i VINS static drift from the device3 calibration verdict.",
            },
            "motion_observation_policy": "Use consensus anchors from low-angular-rate windows; preserve the accepted anchor through fast rotation and occlusion.",
        },
        "outputs": {
            "constraints_jsonl": str(rows_path),
            "continuity_plot": str(plot_path),
        },
        "provenance": {str(path): digest(path) for path in sources},
        "decision": (
            "Device3 ID1 mount and quality-gated initial/reacquisition anchor pass. "
            "This does not activate the deferred three-device group."
            if status == "PASS_DEVICE3_ANCHOR_HIL"
            else "Do not activate the device3 world mount candidate."
        ),
    }
    report_path = output / "report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.session, args.run, args.candidate, args.output)
    print(json.dumps({
        "status": report["status"],
        "checks": report["checks"],
        "selection": report["selection"],
        "occlusion": report["occlusion"],
        "endpoint_anchor_update": report["endpoint_anchor_update"],
        "static_mount_vs_direct_board": report["static_mount_vs_direct_board"],
        "diagnostics": report["diagnostics"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
