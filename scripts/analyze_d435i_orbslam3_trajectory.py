#!/usr/bin/env python3
"""Apply local-trajectory gates to one completed D435i ORB-SLAM3 replay."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


GATES = {
    "min_frame_coverage": 0.85,
    "max_initialization_s": 15.0,
    "max_end_lag_s": 0.1,
    "max_position_step_m": 0.1,
    "max_endpoint_translation_m": 0.03,
    "max_endpoint_rotation_deg": 3.0,
    "max_final_static_radius_m": 0.01,
    "max_final_static_rotation_deg": 1.0,
}


def _quaternion_angle_deg(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    dot = abs(sum(a * b for a, b in zip(left, right))) / (left_norm * right_norm)
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def trajectory_metrics(
    trajectory: Path, *, input_start_ns: int, input_end_ns: int
) -> dict:
    rows = [line.split() for line in trajectory.read_text().splitlines() if line.strip()]
    if len(rows) < 2 or any(len(row) != 8 for row in rows):
        raise ValueError("trajectory must contain at least two EuRoC pose rows")
    timestamps_ns = [float(row[0]) for row in rows]
    positions = [tuple(float(value) for value in row[1:4]) for row in rows]
    quaternions = [tuple(float(value) for value in row[4:8]) for row in rows]
    position_steps = [math.dist(a, b) for a, b in zip(positions, positions[1:])]
    static_indices = [
        index
        for index, timestamp_ns in enumerate(timestamps_ns)
        if timestamp_ns >= input_start_ns + 95e9
    ]
    if not static_indices:
        raise ValueError("trajectory does not cover the final static phase")
    static_start = static_indices[0]
    return {
        "rows": len(rows),
        "duration_s": (timestamps_ns[-1] - timestamps_ns[0]) / 1e9,
        "initialization_s": (timestamps_ns[0] - input_start_ns) / 1e9,
        "end_lag_s": (input_end_ns - timestamps_ns[-1]) / 1e9,
        "max_excursion_m": max(math.dist(positions[0], value) for value in positions),
        "path_length_m": sum(position_steps),
        "max_position_step_m": max(position_steps),
        "endpoint_translation_m": math.dist(positions[0], positions[-1]),
        "endpoint_rotation_deg": _quaternion_angle_deg(
            quaternions[0], quaternions[-1]
        ),
        "final_static_duration_s": (
            timestamps_ns[-1] - timestamps_ns[static_start]
        )
        / 1e9,
        "final_static_max_radius_m": max(
            math.dist(positions[static_start], value)
            for value in positions[static_start:]
        ),
        "final_static_max_rotation_deg": max(
            _quaternion_angle_deg(quaternions[static_start], value)
            for value in quaternions[static_start:]
        ),
    }


def analyze(replay_dir: Path) -> dict:
    replay_dir = replay_dir.resolve()
    replay = json.loads((replay_dir / "replay_report.json").read_text())
    dataset = Path(replay["dataset"])
    export = json.loads((dataset / "export_report.json").read_text())
    source = Path(export["source"])
    ego = json.loads((source / "acceptance.json").read_text())
    pair = json.loads((source.parent / "pair_acceptance.json").read_text())
    stdout = (replay_dir / "orbslam3.stdout.log").read_text(errors="replace")
    metrics = trajectory_metrics(
        replay_dir / "f_d435i_ego.txt",
        input_start_ns=int(export["formal_start_ns"]),
        input_end_ns=int(export["formal_end_ns"]),
    )
    row_coverage = metrics["rows"] / int(replay["input_counts"]["stereo_pairs"])
    checks = {
        "source_ego_pass": ego["status"] == "PASS",
        "source_pair_pass": pair["status"] == "PASS",
        "replay_complete": replay["status"] == "REPLAY_COMPLETE_UNASSESSED",
        "one_continuous_map": stdout.count("New Map created") == 1,
        "no_bad_imu_reset": replay["evidence"]["bad_imu_map_resets"] == 0,
        "viba1_complete": "end VIBA 1" in stdout,
        "viba2_complete": "end VIBA 2" in stdout,
        "frame_coverage": row_coverage >= GATES["min_frame_coverage"],
        "initialization_bounded": metrics["initialization_s"] <= GATES["max_initialization_s"],
        "trajectory_reaches_end": metrics["end_lag_s"] <= GATES["max_end_lag_s"],
        "position_steps_bounded": metrics["max_position_step_m"] <= GATES["max_position_step_m"],
        "endpoint_translation": metrics["endpoint_translation_m"] <= GATES["max_endpoint_translation_m"],
        "endpoint_rotation": metrics["endpoint_rotation_deg"] <= GATES["max_endpoint_rotation_deg"],
        "final_static_position": metrics["final_static_max_radius_m"] <= GATES["max_final_static_radius_m"],
        "final_static_rotation": metrics["final_static_max_rotation_deg"] <= GATES["max_final_static_rotation_deg"],
    }
    report = {
        "schema": "three-device-slam.d435i-orbslam3-local-acceptance.v1",
        "status": "PROVISIONAL_LOCAL_SLAM_PASS" if all(checks.values()) else "FAIL",
        "replay": str(replay_dir),
        "runtime_manifest_sha256": replay["engine"]["runtime_manifest_sha256"],
        "init_translation_thresholds_m": export["engine"]["init_translation_thresholds_m"],
        "gates": GATES,
        "checks": checks,
        "metrics": metrics | {"frame_row_coverage": row_coverage},
        "limitations": [
            "D435i-specific IMU noise and camera/IMU timing remain unaccepted",
            "local trajectory acceptance does not establish T_world_orb",
            "live single-owner TrackStereo adapter remains to be implemented and accepted",
        ],
    }
    (replay_dir / "trajectory_acceptance.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.replay)
    print(json.dumps(report, indent=2))
    if report["status"] == "FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
