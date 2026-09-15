"""Timestamp-aware Ego-world and UMI-local-VIO alignment."""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .se3 import compose, interpolate_transform, invert, validate_transform


def _timestamp_ns(value: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _positive_duration_ns(value: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _stored_transform(value: np.ndarray, *, name: str) -> np.ndarray:
    transform = validate_transform(value, name=name)
    transform.setflags(write=False)
    return transform


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    transform: np.ndarray
    healthy: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", _timestamp_ns(self.timestamp_ns, name="timestamp_ns"))
        if not isinstance(self.healthy, bool):
            raise ValueError("healthy must be a boolean")
        object.__setattr__(self, "transform", _stored_transform(self.transform, name="pose rotation/transform"))


@dataclass(frozen=True)
class RelativePoseObservation:
    timestamp_ns: int
    ego_from_umi: np.ndarray
    valid: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", _timestamp_ns(self.timestamp_ns, name="timestamp_ns"))
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")
        object.__setattr__(self, "ego_from_umi", _stored_transform(self.ego_from_umi, name="ego_from_umi rotation/transform"))


@dataclass(frozen=True)
class WorldInitialization:
    anchor_timestamp_ns: int
    world_from_source: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "anchor_timestamp_ns", _timestamp_ns(self.anchor_timestamp_ns, name="anchor_timestamp_ns"))
        object.__setattr__(self, "world_from_source", _stored_transform(self.world_from_source, name="world_from_source"))


def body_pose_from_camera_pose(
    reference_from_camera: np.ndarray, body_from_camera: np.ndarray
) -> np.ndarray:
    """Apply D405/VINS ``body_T_cam`` to recover the body pose."""
    return compose(reference_from_camera, invert(body_from_camera))


def initialize_ego_world(
    ego_samples_in_source: Iterable[PoseSample],
    *,
    recording_start_ns: int,
    gravity_in_source: np.ndarray,
    forward_axis_in_ego: np.ndarray,
) -> WorldInitialization:
    """Create a Z-up, zero-yaw Ego world from the first usable recording pose."""
    start = _timestamp_ns(recording_start_ns, name="recording_start_ns")
    samples = _ordered_samples(ego_samples_in_source)
    anchor = next(
        (sample for sample in samples if sample.healthy and sample.timestamp_ns >= start),
        None,
    )
    if anchor is None:
        raise ValueError("no healthy Ego pose exists at or after recording start")

    gravity = np.asarray(gravity_in_source, dtype=np.float64)
    if gravity.shape != (3,) or not np.all(np.isfinite(gravity)):
        raise ValueError("gravity_in_source must contain three finite values")
    gravity_norm = float(np.linalg.norm(gravity))
    if gravity_norm < 1e-6:
        raise ValueError("gravity_in_source magnitude is too small")

    forward_axis = np.asarray(forward_axis_in_ego, dtype=np.float64)
    if forward_axis.shape != (3,) or not np.all(np.isfinite(forward_axis)):
        raise ValueError("forward_axis_in_ego must contain three finite values")
    forward_axis_norm = float(np.linalg.norm(forward_axis))
    if forward_axis_norm < 1e-6:
        raise ValueError("forward_axis_in_ego magnitude is too small")

    up_in_source = -gravity / gravity_norm
    ego_forward = anchor.transform[:3, :3] @ (forward_axis / forward_axis_norm)
    forward_horizontal = ego_forward - up_in_source * float(up_in_source @ ego_forward)
    forward_norm = float(np.linalg.norm(forward_horizontal))
    if forward_norm < 1e-6:
        raise ValueError("Ego forward axis is parallel to gravity")
    x_world_in_source = forward_horizontal / forward_norm
    y_world_in_source = np.cross(up_in_source, x_world_in_source)
    y_world_in_source /= np.linalg.norm(y_world_in_source)
    source_from_world_rotation = np.column_stack(
        (x_world_in_source, y_world_in_source, up_in_source)
    )
    world_from_source = np.eye(4, dtype=np.float64)
    world_from_source[:3, :3] = source_from_world_rotation.T
    world_from_source[:3, 3] = -(
        world_from_source[:3, :3] @ anchor.transform[:3, 3]
    )
    return WorldInitialization(anchor.timestamp_ns, world_from_source)


def sample_pose_at_time(
    samples: Iterable[PoseSample], timestamp_ns: int, *, max_gap_ns: int
) -> np.ndarray:
    ordered = _ordered_samples(samples)
    target = _timestamp_ns(timestamp_ns, name="timestamp_ns")
    maximum_gap = _positive_duration_ns(max_gap_ns, name="max_gap_ns")
    timestamps = [sample.timestamp_ns for sample in ordered]
    index = bisect_left(timestamps, target)
    if index < len(ordered) and ordered[index].timestamp_ns == target:
        if not ordered[index].healthy:
            raise ValueError("pose at requested timestamp is unhealthy")
        return ordered[index].transform.copy()
    if index == 0 or index == len(ordered):
        raise ValueError("requested timestamp lies outside pose range")
    before, after = ordered[index - 1], ordered[index]
    if not before.healthy or not after.healthy:
        raise ValueError("cannot interpolate across an unhealthy pose")
    if after.timestamp_ns - before.timestamp_ns > maximum_gap:
        raise ValueError("pose interpolation gap exceeds max_gap_ns")
    fraction = (target - before.timestamp_ns) / (
        after.timestamp_ns - before.timestamp_ns
    )
    return interpolate_transform(before.transform, after.transform, fraction)


def estimate_world_vio_anchor(
    ego_world_samples: Iterable[PoseSample],
    umi_vio_samples: Iterable[PoseSample],
    observation: RelativePoseObservation,
    *,
    observation_delay_ns: int = 0,
    max_interpolation_gap_ns: int,
) -> np.ndarray:
    """Estimate ``T_world_vio`` at corrected acquisition time.

    ``observation_delay_ns`` is signed and follows
    ``acquisition_timestamp = stamped_timestamp - observation_delay``.
    """
    if isinstance(observation_delay_ns, bool) or not isinstance(observation_delay_ns, int):
        raise ValueError("observation_delay_ns must be an integer")
    if not observation.valid:
        raise ValueError("relative pose observation is invalid")
    acquisition_timestamp = observation.timestamp_ns - observation_delay_ns
    if acquisition_timestamp < 0:
        raise ValueError("corrected acquisition timestamp cannot be negative")
    maximum_gap = _positive_duration_ns(
        max_interpolation_gap_ns, name="max_interpolation_gap_ns"
    )
    world_from_ego = sample_pose_at_time(
        ego_world_samples, acquisition_timestamp, max_gap_ns=maximum_gap
    )
    vio_from_umi = sample_pose_at_time(
        umi_vio_samples, acquisition_timestamp, max_gap_ns=maximum_gap
    )
    world_from_umi = compose(world_from_ego, observation.ego_from_umi)
    return compose(world_from_umi, invert(vio_from_umi))


def propagate_world_trajectory(
    world_from_vio: np.ndarray, umi_vio_samples: Iterable[PoseSample]
) -> tuple[PoseSample, ...]:
    anchor = validate_transform(world_from_vio, name="world_from_vio")
    return tuple(
        PoseSample(sample.timestamp_ns, compose(anchor, sample.transform), sample.healthy)
        for sample in _ordered_samples(umi_vio_samples)
    )


def _ordered_samples(samples: Iterable[PoseSample]) -> tuple[PoseSample, ...]:
    ordered = tuple(samples)
    if not ordered:
        raise ValueError("pose samples cannot be empty")
    previous = -1
    for sample in ordered:
        if not isinstance(sample, PoseSample):
            raise TypeError("pose samples must contain PoseSample values")
        if sample.timestamp_ns <= previous:
            raise ValueError("pose timestamps must be strictly increasing")
        previous = sample.timestamp_ns
    return ordered
