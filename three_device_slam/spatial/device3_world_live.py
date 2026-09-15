"""Dependency-light primitives shared by the Ego+device3 live world adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np
from scipy.spatial.transform import Rotation

from .device3_world_runtime import AnchorCandidate
from .se3 import validate_transform


@dataclass(frozen=True)
class TimedTransform:
    timestamp_ns: int
    transform: np.ndarray

    def __post_init__(self) -> None:
        if isinstance(self.timestamp_ns, bool) or int(self.timestamp_ns) < 0:
            raise ValueError("timestamp_ns must be a non-negative integer")
        transform = validate_transform(self.transform, name="timed transform")
        transform.setflags(write=False)
        object.__setattr__(self, "timestamp_ns", int(self.timestamp_ns))
        object.__setattr__(self, "transform", transform)


def matrix_from_pose_components(
    position_xyz: Iterable[float], quaternion_xyzw: Iterable[float]
) -> np.ndarray:
    """Convert ROS pose components into ``T_parent_child``."""
    position = np.asarray(tuple(position_xyz), dtype=np.float64)
    quaternion = np.asarray(tuple(quaternion_xyzw), dtype=np.float64)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("position must contain three finite values")
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("quaternion norm must be non-zero")
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    transform[:3, 3] = position
    return validate_transform(transform, name="pose transform")


def pose_components_from_matrix(
    transform: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return position XYZ and normalized quaternion XYZW."""
    value = validate_transform(transform)
    position = np.array(value[:3, 3], copy=True)
    quaternion = Rotation.from_matrix(value[:3, :3]).as_quat()
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return position, quaternion


def nearest_transform(
    samples: Iterable[TimedTransform],
    timestamp_ns: int,
    *,
    max_delta_ns: int,
) -> TimedTransform:
    """Select the nearest transform and fail closed when it is stale."""
    values = tuple(samples)
    if not values:
        raise ValueError("no timed transforms are available")
    if max_delta_ns <= 0:
        raise ValueError("max_delta_ns must be positive")
    target = int(timestamp_ns)
    selected = min(values, key=lambda item: abs(item.timestamp_ns - target))
    delta = abs(selected.timestamp_ns - target)
    if delta > max_delta_ns:
        raise ValueError(
            f"nearest timed transform is stale by {delta / 1e6:.3f} ms"
        )
    return selected


class AnchorCandidateWindow:
    """Collect non-overlapping live candidates before calling the controller."""

    def __init__(self, *, window_ns: int, maximum_candidates: int = 30):
        if window_ns <= 0:
            raise ValueError("window_ns must be positive")
        if maximum_candidates < 3:
            raise ValueError("maximum_candidates must be at least three")
        self.window_ns = int(window_ns)
        self.maximum_candidates = int(maximum_candidates)
        self._values: list[AnchorCandidate] = []

    @property
    def pending_count(self) -> int:
        return len(self._values)

    def add(
        self, candidate: AnchorCandidate
    ) -> tuple[AnchorCandidate, ...] | None:
        if not isinstance(candidate, AnchorCandidate):
            raise TypeError("candidate must be an AnchorCandidate")
        if self._values and candidate.timestamp_ns <= self._values[-1].timestamp_ns:
            raise ValueError("candidate timestamps must be strictly increasing")
        self._values.append(candidate)
        elapsed = candidate.timestamp_ns - self._values[0].timestamp_ns
        if (
            elapsed < self.window_ns
            and len(self._values) < self.maximum_candidates
        ):
            return None
        batch = tuple(self._values)
        self._values.clear()
        return batch

    def clear(self) -> None:
        self._values.clear()
