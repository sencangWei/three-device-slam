"""Deterministic evidence simulation for Ego-world UMI alignment."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .alignment import (
    PoseSample,
    RelativePoseObservation,
    body_pose_from_camera_pose,
    estimate_world_vio_anchor,
    initialize_ego_world,
    propagate_world_trajectory,
)
from .se3 import compose, invert, pose_error, transform_from_xyz_rpy


_KNOWN_DEVICES = ("left", "right")
_SAMPLE_PERIOD_NS = 20_000_000
_OBSERVATION_DELAY_NS = 60_000_000
_MAX_INTERPOLATION_GAP_NS = 40_000_000
_EGO_FORWARD_AXIS = np.array((1.0, 0.0, 0.0))


def run_alignment_simulation(device_ids: Iterable[str] = ("left", "right")) -> dict:
    """Run a noiseless, deterministic contract test and return JSON-ready evidence."""
    devices = tuple(device_ids)
    if not devices or len(set(devices)) != len(devices):
        raise ValueError("device_ids must be non-empty and unique")
    unknown = set(devices) - set(_KNOWN_DEVICES)
    if unknown:
        raise ValueError(f"unknown simulated device IDs: {sorted(unknown)}")

    timestamps = tuple(index * _SAMPLE_PERIOD_NS for index in range(101))
    ego_truth = tuple(_ego_pose(index) for index in range(len(timestamps)))

    source_from_world = transform_from_xyz_rpy(
        (2.4, -1.3, 0.7), (0.16, -0.09, 0.62)
    )
    gravity_world = np.array((0.0, 0.0, -9.81))
    gravity_source = source_from_world[:3, :3] @ gravity_world
    ego_source_samples = tuple(
        PoseSample(timestamp, compose(source_from_world, pose))
        for timestamp, pose in zip(timestamps, ego_truth)
    )
    initialization = initialize_ego_world(
        ego_source_samples,
        recording_start_ns=0,
        gravity_in_source=gravity_source,
        forward_axis_in_ego=_EGO_FORWARD_AXIS,
    )
    ego_world_samples = tuple(
        PoseSample(
            sample.timestamp_ns,
            compose(initialization.world_from_source, sample.transform),
        )
        for sample in ego_source_samples
    )
    gravity_after = initialization.world_from_source[:3, :3] @ gravity_source
    gravity_angle_deg = _vector_angle_deg(gravity_after, gravity_world)
    origin_error_m = float(np.linalg.norm(ego_world_samples[0].transform[:3, 3]))

    chains = {
        device_id: _simulate_device(device_id, timestamps, ego_world_samples)
        for device_id in devices
    }
    passed = (
        gravity_angle_deg < 1e-8
        and origin_error_m < 1e-10
        and all(
            chain["corrected_translation_rmse_m"] < 1e-9
            and chain["corrected_rotation_rmse_deg"] < 1e-7
            and chain["uncorrected_translation_rmse_m"] > 0.01
            and chain["camera_body_roundtrip_max_m"] < 1e-10
            and chain["camera_body_roundtrip_max_deg"] < 1e-7
            for chain in chains.values()
        )
    )
    return {
        "schema": "three-device-slam.spatial-alignment-simulation.v1",
        "mode": "deterministic_simulation",
        "verdict": "PASS/simulation" if passed else "FAIL/simulation",
        "hil_status": "NOT_RUN",
        "frame_convention": "T_A_B maps coordinates from frame B into frame A",
        "world_definition": "first healthy post-recording Ego pose; gravity=-Z; projected Ego forward=+X",
        "devices": list(devices),
        "ego_world": {
            "anchor_timestamp_ns": initialization.anchor_timestamp_ns,
            "origin_error_m": origin_error_m,
            "gravity_alignment_error_deg": gravity_angle_deg,
            "forward_axis_in_ego": _EGO_FORWARD_AXIS.tolist(),
        },
        "chains": chains,
        "future_one_click_topology": {
            "shared": ["recording barrier", "Ego world", "session manifest"],
            "per_umi": ["clock model", "body_T_cam", "T_world_vio anchor", "health state"],
        },
    }


def _simulate_device(
    device_id: str,
    timestamps: tuple[int, ...],
    ego_world_samples: tuple[PoseSample, ...],
) -> dict:
    device_index = _KNOWN_DEVICES.index(device_id)
    umi_truth = tuple(_umi_pose(index, device_index) for index in range(len(timestamps)))
    vio_from_world = transform_from_xyz_rpy(
        (4.0 + 1.7 * device_index, -3.0 + 0.4 * device_index, 0.8),
        (0.08, -0.05, 0.9 - 0.35 * device_index),
    )
    body_from_camera = transform_from_xyz_rpy(
        (0.032, -0.011 + 0.003 * device_index, 0.019),
        (0.008, -0.014, 0.004),
    )
    camera_poses = tuple(
        compose(vio_from_world, world_pose, body_from_camera)
        for world_pose in umi_truth
    )
    recovered_vio_body = tuple(
        body_pose_from_camera_pose(camera_pose, body_from_camera)
        for camera_pose in camera_poses
    )
    roundtrip_errors = tuple(
        pose_error(recovered, compose(vio_from_world, truth))
        for recovered, truth in zip(recovered_vio_body, umi_truth)
    )
    vio_samples = tuple(
        PoseSample(timestamp, pose)
        for timestamp, pose in zip(timestamps, recovered_vio_body)
    )

    true_index = 40 + 5 * device_index
    acquisition_timestamp = timestamps[true_index]
    stamped_timestamp = acquisition_timestamp + _OBSERVATION_DELAY_NS
    relative_pose = compose(
        invert(ego_world_samples[true_index].transform), umi_truth[true_index]
    )
    observation = RelativePoseObservation(stamped_timestamp, relative_pose)
    uncorrected_anchor = estimate_world_vio_anchor(
        ego_world_samples,
        vio_samples,
        observation,
        max_interpolation_gap_ns=_MAX_INTERPOLATION_GAP_NS,
    )
    corrected_anchor = estimate_world_vio_anchor(
        ego_world_samples,
        vio_samples,
        observation,
        observation_delay_ns=_OBSERVATION_DELAY_NS,
        max_interpolation_gap_ns=_MAX_INTERPOLATION_GAP_NS,
    )
    uncorrected_trajectory = propagate_world_trajectory(
        uncorrected_anchor, vio_samples
    )
    corrected_trajectory = propagate_world_trajectory(corrected_anchor, vio_samples)
    uncorrected_errors = tuple(
        pose_error(sample.transform, truth)
        for sample, truth in zip(uncorrected_trajectory, umi_truth)
    )
    corrected_errors = tuple(
        pose_error(sample.transform, truth)
        for sample, truth in zip(corrected_trajectory, umi_truth)
    )
    return {
        "anchor_id": f"{device_id}:T_world_vio",
        "acquisition_timestamp_ns": acquisition_timestamp,
        "stamped_timestamp_ns": stamped_timestamp,
        "timestamp_correction_ns": _OBSERVATION_DELAY_NS,
        "max_interpolation_gap_ns": _MAX_INTERPOLATION_GAP_NS,
        "camera_body_roundtrip_max_m": max(
            error.translation_m for error in roundtrip_errors
        ),
        "camera_body_roundtrip_max_deg": max(
            error.rotation_deg for error in roundtrip_errors
        ),
        "uncorrected_translation_rmse_m": _rmse(
            error.translation_m for error in uncorrected_errors
        ),
        "uncorrected_rotation_rmse_deg": _rmse(
            error.rotation_deg for error in uncorrected_errors
        ),
        "corrected_translation_rmse_m": _rmse(
            error.translation_m for error in corrected_errors
        ),
        "corrected_rotation_rmse_deg": _rmse(
            error.rotation_deg for error in corrected_errors
        ),
    }


def _ego_pose(index: int) -> np.ndarray:
    time_s = index * _SAMPLE_PERIOD_NS / 1e9
    return transform_from_xyz_rpy(
        (0.38 * time_s, 0.12 * math.sin(1.1 * time_s), 0.015 * math.sin(0.7 * time_s)),
        (0.01 * math.sin(time_s), 0.015 * math.sin(0.6 * time_s), 0.22 * time_s),
    )


def _umi_pose(index: int, device_index: int) -> np.ndarray:
    time_s = index * _SAMPLE_PERIOD_NS / 1e9
    side = 1.0 if device_index == 0 else -1.0
    return transform_from_xyz_rpy(
        (
            0.55 + 0.21 * time_s,
            side * (0.28 + 0.07 * math.sin(1.4 * time_s)),
            0.14 + 0.03 * math.sin(0.8 * time_s),
        ),
        (
            0.035 * math.sin(0.9 * time_s),
            side * 0.04 * math.sin(0.5 * time_s),
            side * (0.18 - 0.13 * time_s),
        ),
    )


def _rmse(values: Iterable[float]) -> float:
    array = np.asarray(tuple(values), dtype=np.float64)
    return float(math.sqrt(float(np.mean(array * array))))


def _vector_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(first @ second) / (
        float(np.linalg.norm(first)) * float(np.linalg.norm(second))
    )
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate UMI-to-Ego-world SE(3) and timestamp alignment in simulation."
    )
    parser.add_argument(
        "--devices",
        default="left,right",
        help="Comma-separated simulated UMI IDs (product default: left,right)",
    )
    parser.add_argument("--output", type=Path, help="Optional JSON evidence path")
    args = parser.parse_args(argv)
    report = run_alignment_simulation(
        tuple(part.strip() for part in args.devices.split(",") if part.strip())
    )
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["verdict"] == "PASS/simulation" else 1


if __name__ == "__main__":
    raise SystemExit(main())
