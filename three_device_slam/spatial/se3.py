"""Small, dependency-light SE(3) helpers with strict boundary validation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


_ATOL = 1e-8


@dataclass(frozen=True)
class PoseError:
    translation_m: float
    rotation_deg: float


def validate_transform(transform: np.ndarray, *, name: str = "transform") -> np.ndarray:
    """Return a defensive float64 copy after validating a rigid transform."""
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4):
        raise ValueError(f"{name} must have shape (4, 4)")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain only finite values")
    if not np.allclose(value[3], (0.0, 0.0, 0.0, 1.0), atol=_ATOL):
        raise ValueError(f"{name} has an invalid homogeneous row")
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=_ATOL):
        raise ValueError(f"{name} rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=_ATOL):
        raise ValueError(f"{name} rotation determinant must be +1")
    return value.copy()


def compose(*transforms: np.ndarray) -> np.ndarray:
    """Compose transforms whose adjacent named frames cancel."""
    result = np.eye(4, dtype=np.float64)
    for index, transform in enumerate(transforms):
        result = result @ validate_transform(transform, name=f"transform[{index}]")
    return validate_transform(result, name="composed transform")


def invert(transform: np.ndarray) -> np.ndarray:
    value = validate_transform(transform)
    rotation = value[:3, :3]
    translation = value[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ translation)
    return inverse


def transform_from_xyz_rpy(
    xyz: Iterable[float], rpy_rad: Iterable[float] = (0.0, 0.0, 0.0)
) -> np.ndarray:
    translation = np.asarray(tuple(xyz), dtype=np.float64)
    rpy = np.asarray(tuple(rpy_rad), dtype=np.float64)
    if translation.shape != (3,) or rpy.shape != (3,):
        raise ValueError("xyz and rpy_rad must each contain exactly three values")
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(rpy)):
        raise ValueError("xyz and rpy_rad must be finite")
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_x = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)))
    rotation_y = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)))
    rotation_z = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)))
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation_z @ rotation_y @ rotation_x
    result[:3, 3] = translation
    return validate_transform(result)


def interpolate_transform(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    """Interpolate translation linearly and rotation on SO(3)."""
    start_value = validate_transform(start, name="start")
    end_value = validate_transform(end, name="end")
    if not math.isfinite(fraction) or fraction < 0.0 or fraction > 1.0:
        raise ValueError("fraction must be finite and within [0, 1]")
    relative_rotation = start_value[:3, :3].T @ end_value[:3, :3]
    rotation_vector = _rotation_log(relative_rotation)
    rotation = start_value[:3, :3] @ _rotation_exp(fraction * rotation_vector)
    translation = (
        (1.0 - fraction) * start_value[:3, 3]
        + fraction * end_value[:3, 3]
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return validate_transform(result)


def pose_error(actual: np.ndarray, expected: np.ndarray) -> PoseError:
    delta = compose(invert(expected), actual)
    rotation = delta[:3, :3]
    cosine = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
    sine = 0.5 * float(
        np.linalg.norm(
            (
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            )
        )
    )
    angle = math.atan2(sine, cosine)
    return PoseError(
        translation_m=float(np.linalg.norm(delta[:3, 3])),
        rotation_deg=math.degrees(angle),
    )


def _rotation_log(rotation: np.ndarray) -> np.ndarray:
    cosine = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-12:
        return np.zeros(3, dtype=np.float64)
    if math.pi - angle < 1e-7:
        eigenvalues, eigenvectors = np.linalg.eigh((rotation + np.eye(3)) / 2.0)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        axis /= np.linalg.norm(axis)
        return angle * axis
    axis = np.array(
        (
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        )
    ) / (2.0 * math.sin(angle))
    return angle * axis


def _rotation_exp(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-12:
        return np.eye(3, dtype=np.float64)
    axis = rotation_vector / angle
    skew = np.array(
        ((0.0, -axis[2], axis[1]), (axis[2], 0.0, -axis[0]), (-axis[1], axis[0], 0.0))
    )
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)
