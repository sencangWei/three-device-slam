"""Quality-gated AprilTag anchors for independent UMI odometry chains.

The detector-facing pose is ``T_camera_tag``.  A rigid mount supplies
``T_tag_gripper`` and the UMI VIO supplies ``T_odom_gripper``.  The Ego SLAM
trajectory supplies ``T_world_camera``.  All transforms follow the package
convention ``T_A_B`` and therefore:

``T_world_gripper = T_world_camera @ T_camera_tag @ T_tag_gripper``
``T_world_odom = T_world_gripper @ inverse(T_odom_gripper)``
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .alignment import (
    PoseSample,
    RelativePoseObservation,
    estimate_world_vio_anchor,
)
from .se3 import compose, pose_error, validate_transform


def _nonempty_string(value: str, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _timestamp_ns(value: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _integer(value: int, *, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _finite_float(value: float, *, name: str, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite and >= {minimum}")
    converted = float(value)
    if not math.isfinite(converted) or converted < minimum:
        raise ValueError(f"{name} must be finite and >= {minimum}")
    return converted


def _stored_transform(value: np.ndarray, *, name: str) -> np.ndarray:
    transform = validate_transform(value, name=name)
    transform.setflags(write=False)
    return transform


@dataclass(frozen=True)
class TagMount:
    """One immutable left/right tag-to-gripper installation calibration."""

    device_id: str
    family: str
    tag_id: int
    tag_from_gripper: np.ndarray
    calibration_id: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "device_id", _nonempty_string(self.device_id, name="device_id")
        )
        object.__setattr__(
            self, "family", _nonempty_string(self.family, name="family")
        )
        object.__setattr__(self, "tag_id", _integer(self.tag_id, name="tag_id"))
        object.__setattr__(
            self,
            "tag_from_gripper",
            _stored_transform(self.tag_from_gripper, name="tag_from_gripper"),
        )
        object.__setattr__(
            self,
            "calibration_id",
            _nonempty_string(self.calibration_id, name="calibration_id"),
        )


@dataclass(frozen=True)
class TagPoseObservation:
    """One detector result with acquisition-domain and completion timestamps."""

    stamped_timestamp_ns: int
    detector_completed_ns: int
    family: str
    tag_id: int
    camera_from_tag: np.ndarray
    reprojection_error_px: float
    hamming: int = 0

    def __post_init__(self) -> None:
        stamped = _timestamp_ns(
            self.stamped_timestamp_ns, name="stamped_timestamp_ns"
        )
        completed = _timestamp_ns(
            self.detector_completed_ns, name="detector_completed_ns"
        )
        if completed < stamped:
            raise ValueError("detector_completed_ns cannot precede stamped_timestamp_ns")
        object.__setattr__(self, "stamped_timestamp_ns", stamped)
        object.__setattr__(self, "detector_completed_ns", completed)
        object.__setattr__(
            self, "family", _nonempty_string(self.family, name="family")
        )
        object.__setattr__(self, "tag_id", _integer(self.tag_id, name="tag_id"))
        object.__setattr__(
            self,
            "camera_from_tag",
            _stored_transform(self.camera_from_tag, name="camera_from_tag"),
        )
        object.__setattr__(
            self,
            "reprojection_error_px",
            _finite_float(
                self.reprojection_error_px, name="reprojection_error_px"
            ),
        )
        object.__setattr__(
            self, "hamming", _integer(self.hamming, name="hamming")
        )

    @property
    def detector_latency_ns(self) -> int:
        return self.detector_completed_ns - self.stamped_timestamp_ns


@dataclass(frozen=True)
class TagObservationPolicy:
    max_reprojection_error_px: float = 1.5
    max_hamming: int = 0
    min_depth_m: float = 0.05
    max_depth_m: float = 3.0
    min_inliers: int = 3
    anchor_translation_threshold_m: float = 0.03
    anchor_rotation_threshold_deg: float = 3.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "max_reprojection_error_px",
            _finite_float(
                self.max_reprojection_error_px,
                name="max_reprojection_error_px",
            ),
        )
        object.__setattr__(
            self,
            "max_hamming",
            _integer(self.max_hamming, name="max_hamming"),
        )
        min_depth = _finite_float(self.min_depth_m, name="min_depth_m")
        max_depth = _finite_float(self.max_depth_m, name="max_depth_m")
        if max_depth <= min_depth:
            raise ValueError("max_depth_m must exceed min_depth_m")
        object.__setattr__(self, "min_depth_m", min_depth)
        object.__setattr__(self, "max_depth_m", max_depth)
        object.__setattr__(
            self,
            "min_inliers",
            _integer(self.min_inliers, name="min_inliers", minimum=1),
        )
        translation = _finite_float(
            self.anchor_translation_threshold_m,
            name="anchor_translation_threshold_m",
        )
        rotation = _finite_float(
            self.anchor_rotation_threshold_deg,
            name="anchor_rotation_threshold_deg",
        )
        if translation <= 0.0 or rotation <= 0.0:
            raise ValueError("anchor consensus thresholds must be positive")
        object.__setattr__(self, "anchor_translation_threshold_m", translation)
        object.__setattr__(self, "anchor_rotation_threshold_deg", rotation)


@dataclass(frozen=True)
class RejectedTagObservation:
    stamped_timestamp_ns: int
    tag_id: int
    reason: str


@dataclass(frozen=True)
class TagAnchorEstimate:
    device_id: str
    verdict: str
    world_from_odom: np.ndarray | None
    candidate_count: int
    inlier_timestamps_ns: tuple[int, ...]
    acquisition_timestamps_ns: tuple[int, ...]
    rejections: tuple[RejectedTagObservation, ...]
    max_detector_latency_ns: int

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "device_id", _nonempty_string(self.device_id, name="device_id")
        )
        if self.world_from_odom is not None:
            object.__setattr__(
                self,
                "world_from_odom",
                _stored_transform(self.world_from_odom, name="world_from_odom"),
            )


@dataclass(frozen=True)
class AnchorAcceptance:
    device_id: str
    world_from_odom: np.ndarray
    accepted_at_ns: int

    @classmethod
    def initial(
        cls,
        *,
        device_id: str,
        world_from_odom: np.ndarray,
        accepted_at_ns: int,
    ) -> "AnchorAcceptance":
        return cls(device_id, world_from_odom, accepted_at_ns)

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "device_id", _nonempty_string(self.device_id, name="device_id")
        )
        object.__setattr__(
            self,
            "world_from_odom",
            _stored_transform(self.world_from_odom, name="world_from_odom"),
        )
        object.__setattr__(
            self,
            "accepted_at_ns",
            _timestamp_ns(self.accepted_at_ns, name="accepted_at_ns"),
        )


@dataclass(frozen=True)
class AnchorReconciliation:
    state: AnchorAcceptance
    accepted: bool
    reason: str


@dataclass(frozen=True)
class _AnchorCandidate:
    observation: TagPoseObservation
    acquisition_timestamp_ns: int
    world_from_odom: np.ndarray


def tag_observation_to_camera_gripper(
    observation: TagPoseObservation, mount: TagMount
) -> RelativePoseObservation:
    """Apply one fixed installation extrinsic without changing the timestamp."""
    if observation.family != mount.family or observation.tag_id != mount.tag_id:
        raise ValueError("tag observation does not match mount identity")
    return RelativePoseObservation(
        observation.stamped_timestamp_ns,
        compose(observation.camera_from_tag, mount.tag_from_gripper),
    )


def estimate_tag_anchor_window(
    world_from_ego_camera_samples: Iterable[PoseSample],
    odom_from_gripper_samples: Iterable[PoseSample],
    mount: TagMount,
    observations: Iterable[TagPoseObservation],
    *,
    policy: TagObservationPolicy,
    observation_delay_ns: int,
    max_interpolation_gap_ns: int,
) -> TagAnchorEstimate:
    """Estimate one robust ``T_world_odom`` from a synchronized tag window."""
    if isinstance(observation_delay_ns, bool) or not isinstance(
        observation_delay_ns, int
    ):
        raise ValueError("observation_delay_ns must be an integer")
    if (
        isinstance(max_interpolation_gap_ns, bool)
        or not isinstance(max_interpolation_gap_ns, int)
        or max_interpolation_gap_ns <= 0
    ):
        raise ValueError("max_interpolation_gap_ns must be a positive integer")
    ego_samples = tuple(world_from_ego_camera_samples)
    odom_samples = tuple(odom_from_gripper_samples)
    ordered_observations = tuple(observations)
    candidates: list[_AnchorCandidate] = []
    rejections: list[RejectedTagObservation] = []
    accepted_acquisition_timestamps: set[int] = set()

    for observation in ordered_observations:
        if not isinstance(observation, TagPoseObservation):
            raise TypeError("observations must contain TagPoseObservation values")
        reason = _quality_rejection(observation, mount, policy)
        if reason is not None:
            rejections.append(_rejection(observation, reason))
            continue
        acquisition_timestamp_ns = (
            observation.stamped_timestamp_ns - observation_delay_ns
        )
        if acquisition_timestamp_ns < 0:
            rejections.append(_rejection(observation, "negative_acquisition_time"))
            continue
        if acquisition_timestamp_ns in accepted_acquisition_timestamps:
            rejections.append(
                _rejection(observation, "duplicate_acquisition_time")
            )
            continue
        relative = tag_observation_to_camera_gripper(observation, mount)
        try:
            world_from_odom = estimate_world_vio_anchor(
                ego_samples,
                odom_samples,
                relative,
                observation_delay_ns=observation_delay_ns,
                max_interpolation_gap_ns=max_interpolation_gap_ns,
            )
        except ValueError:
            rejections.append(_rejection(observation, "pose_time_unavailable"))
            continue
        candidates.append(
            _AnchorCandidate(
                observation,
                acquisition_timestamp_ns,
                world_from_odom,
            )
        )
        accepted_acquisition_timestamps.add(acquisition_timestamp_ns)

    max_latency = max(
        (observation.detector_latency_ns for observation in ordered_observations),
        default=0,
    )
    if not candidates:
        return TagAnchorEstimate(
            mount.device_id,
            "BLOCKED/no_consensus",
            None,
            len(candidates),
            (),
            tuple(candidate.acquisition_timestamp_ns for candidate in candidates),
            tuple(rejections),
            max_latency,
        )

    medoid_index, inlier_indices = _consensus(candidates, policy)
    inlier_set = set(inlier_indices)
    for index, candidate in enumerate(candidates):
        if index not in inlier_set:
            rejections.append(
                _rejection(candidate.observation, "anchor_consensus_outlier")
            )
    inliers = tuple(candidates[index] for index in inlier_indices)
    if len(inlier_indices) < policy.min_inliers:
        return TagAnchorEstimate(
            mount.device_id,
            "BLOCKED/no_consensus",
            None,
            len(candidates),
            tuple(
                candidate.observation.stamped_timestamp_ns
                for candidate in inliers
            ),
            tuple(candidate.acquisition_timestamp_ns for candidate in inliers),
            tuple(rejections),
            max_latency,
        )
    return TagAnchorEstimate(
        mount.device_id,
        "PASS/replay",
        candidates[medoid_index].world_from_odom,
        len(candidates),
        tuple(candidate.observation.stamped_timestamp_ns for candidate in inliers),
        tuple(candidate.acquisition_timestamp_ns for candidate in inliers),
        tuple(rejections),
        max_latency,
    )


def reconcile_anchor(
    current: AnchorAcceptance,
    candidate: AnchorAcceptance,
    *,
    max_translation_update_m: float,
    max_rotation_update_deg: float,
) -> AnchorReconciliation:
    """Keep the previous anchor across occlusion or a discontinuous relocalization."""
    if current.device_id != candidate.device_id:
        raise ValueError("cannot reconcile anchors from different devices")
    translation_limit = _finite_float(
        max_translation_update_m, name="max_translation_update_m"
    )
    rotation_limit = _finite_float(
        max_rotation_update_deg, name="max_rotation_update_deg"
    )
    if translation_limit <= 0.0 or rotation_limit <= 0.0:
        raise ValueError("anchor update thresholds must be positive")
    if candidate.accepted_at_ns <= current.accepted_at_ns:
        return AnchorReconciliation(current, False, "stale_anchor")
    error = pose_error(candidate.world_from_odom, current.world_from_odom)
    if (
        error.translation_m > translation_limit
        or error.rotation_deg > rotation_limit
    ):
        return AnchorReconciliation(current, False, "relocalization_jump")
    return AnchorReconciliation(candidate, True, "accepted")


def _quality_rejection(
    observation: TagPoseObservation,
    mount: TagMount,
    policy: TagObservationPolicy,
) -> str | None:
    if observation.family != mount.family or observation.tag_id != mount.tag_id:
        return "tag_identity_mismatch"
    if observation.reprojection_error_px > policy.max_reprojection_error_px:
        return "reprojection_error"
    if observation.hamming > policy.max_hamming:
        return "hamming"
    depth_m = float(observation.camera_from_tag[2, 3])
    if depth_m < policy.min_depth_m or depth_m > policy.max_depth_m:
        return "depth_out_of_range"
    return None


def _consensus(
    candidates: list[_AnchorCandidate], policy: TagObservationPolicy
) -> tuple[int, tuple[int, ...]]:
    best_key: tuple[int, float, int, int] | None = None
    best_index = 0
    best_inliers: tuple[int, ...] = ()
    for index, candidate in enumerate(candidates):
        inliers = []
        cost = 0.0
        for other_index, other in enumerate(candidates):
            error = pose_error(other.world_from_odom, candidate.world_from_odom)
            if (
                error.translation_m <= policy.anchor_translation_threshold_m
                and error.rotation_deg <= policy.anchor_rotation_threshold_deg
            ):
                inliers.append(other_index)
                cost += (
                    error.translation_m / policy.anchor_translation_threshold_m
                    + error.rotation_deg / policy.anchor_rotation_threshold_deg
                )
        key = (
            -len(inliers),
            cost,
            candidate.observation.stamped_timestamp_ns,
            index,
        )
        if best_key is None or key < best_key:
            best_key = key
            best_index = index
            best_inliers = tuple(inliers)
    return best_index, best_inliers


def _rejection(
    observation: TagPoseObservation, reason: str
) -> RejectedTagObservation:
    return RejectedTagObservation(
        observation.stamped_timestamp_ns,
        observation.tag_id,
        reason,
    )
