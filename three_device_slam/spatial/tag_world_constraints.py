"""Tag-ID-aware constraints between two independent VIO worlds.

The static grid is the common geometric object.  Decoded tag IDs select fixed
3-D grid corners, so repeated black/white texture is never matched by visual
descriptor similarity.  Every synchronized dual-camera observation produces
one ``T_egoWorld_rightWorld`` sample that can be tested for temporal stability.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .se3 import compose, invert, pose_error, validate_transform


@dataclass(frozen=True)
class GridPose:
    camera_from_grid: np.ndarray
    tag_ids: tuple[int, ...]
    reprojection_rms_px: float

    def __post_init__(self) -> None:
        transform = validate_transform(self.camera_from_grid, name="camera_from_grid")
        transform.setflags(write=False)
        object.__setattr__(self, "camera_from_grid", transform)
        ids = tuple(int(value) for value in self.tag_ids)
        if not ids or len(ids) != len(set(ids)) or any(value < 0 for value in ids):
            raise ValueError("tag_ids must be unique non-negative integers")
        object.__setattr__(self, "tag_ids", ids)
        error = float(self.reprojection_rms_px)
        if not np.isfinite(error) or error < 0.0:
            raise ValueError("reprojection_rms_px must be finite and non-negative")
        object.__setattr__(self, "reprojection_rms_px", error)


@dataclass(frozen=True)
class TagWorldConstraint:
    ego_stamp_ns: int
    right_stamp_ns: int
    phase: str
    ego_world_from_grid: np.ndarray
    right_world_from_grid: np.ndarray
    ego_world_from_right_world: np.ndarray
    ego_tag_ids: tuple[int, ...]
    right_tag_ids: tuple[int, ...]
    ego_pnp_rms_px: float
    right_pnp_rms_px: float

    def __post_init__(self) -> None:
        if self.ego_stamp_ns < 0 or self.right_stamp_ns < 0:
            raise ValueError("timestamps must be non-negative")
        if not self.phase:
            raise ValueError("phase must be non-empty")
        for name in (
            "ego_world_from_grid",
            "right_world_from_grid",
            "ego_world_from_right_world",
        ):
            value = validate_transform(getattr(self, name), name=name)
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        for name in ("ego_tag_ids", "right_tag_ids"):
            values = tuple(int(value) for value in getattr(self, name))
            if not values or len(values) != len(set(values)) or any(value < 0 for value in values):
                raise ValueError(f"{name} must contain unique non-negative IDs")
            object.__setattr__(self, name, values)
        for name in ("ego_pnp_rms_px", "right_pnp_rms_px"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
            object.__setattr__(self, name, value)


def aprilgrid_object_corners(
    tag_id: int,
    *,
    rows: int = 6,
    columns: int = 6,
    tag_size_m: float = 0.0352,
    gap_m: float = 0.01056,
) -> np.ndarray:
    """Return the frozen legacy-grid corner order for one decoded tag ID."""
    if (isinstance(tag_id, bool) or not isinstance(tag_id, int)
            or not 0 <= tag_id < rows * columns):
        raise ValueError("tag_id is outside the configured grid")
    if rows < 1 or columns < 1 or tag_size_m <= 0.0 or gap_m < 0.0:
        raise ValueError("invalid AprilGrid geometry")
    center = np.array((tag_id % columns, tag_id // columns, 0.0)) * (
        tag_size_m + gap_m
    )
    offsets = 0.5 * tag_size_m * np.array(
        ((1.0, -1.0, 0.0), (-1.0, -1.0, 0.0),
         (-1.0, 1.0, 0.0), (1.0, 1.0, 0.0))
    )
    return center + offsets


def solve_grid_pose(
    detections_by_id: Mapping[int, np.ndarray],
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray | None = None,
    *,
    rows: int = 6,
    columns: int = 6,
    tag_size_m: float = 0.0352,
    gap_m: float = 0.01056,
    min_tags: int = 6,
    min_grid_rows: int = 3,
    min_grid_columns: int = 3,
) -> GridPose | None:
    """Solve one grid pose using decoded IDs and the matching metric corners."""
    detected: dict[int, np.ndarray] = {}
    for raw_id, raw_corners in detections_by_id.items():
        tag_id = int(raw_id)
        if tag_id != raw_id or not 0 <= tag_id < rows * columns:
            raise ValueError("detection contains an unknown grid tag ID")
        corners = np.asarray(raw_corners, dtype=np.float64)
        if corners.shape != (4, 2) or not np.all(np.isfinite(corners)):
            raise ValueError("each detection must contain four finite pixel corners")
        detected[tag_id] = corners
    ids = tuple(sorted(detected))
    if (len(ids) < min_tags
            or len({tag_id // columns for tag_id in ids}) < min_grid_rows
            or len({tag_id % columns for tag_id in ids}) < min_grid_columns):
        return None
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("camera_matrix must be finite 3x3")
    distortion = np.zeros(5) if distortion_coefficients is None else np.asarray(
        distortion_coefficients, dtype=np.float64
    ).reshape(-1)
    objects = np.concatenate([
        aprilgrid_object_corners(
            tag_id, rows=rows, columns=columns,
            tag_size_m=tag_size_m, gap_m=gap_m,
        )
        for tag_id in ids
    ])
    pixels = np.concatenate([detected[tag_id] for tag_id in ids])
    seeds = cv2.solvePnPGeneric(
        objects, pixels, matrix, distortion, flags=cv2.SOLVEPNP_IPPE
    )
    candidates = []
    for seed_index, (rvec, tvec) in enumerate(zip(seeds[1], seeds[2])):
        rvec, tvec = cv2.solvePnPRefineLM(
            objects, pixels, matrix, distortion, rvec.copy(), tvec.copy()
        )
        rotation = cv2.Rodrigues(rvec)[0]
        translation = tvec.reshape(3)
        if np.min((objects @ rotation.T + translation)[:, 2]) <= 0.0:
            continue
        projected = cv2.projectPoints(
            objects, rvec, tvec, matrix, distortion
        )[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected - pixels) ** 2, axis=1))))
        if np.isfinite(rms):
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = rotation, translation
            candidates.append((rms, seed_index, transform))
    if not candidates:
        return None
    rms, _, transform = min(candidates, key=lambda item: (item[0], item[1]))
    return GridPose(transform, ids, rms)


def make_world_constraint(
    *,
    ego_stamp_ns: int,
    right_stamp_ns: int,
    phase: str,
    ego_world_from_body: np.ndarray,
    right_world_from_body: np.ndarray,
    ego_body_from_camera: np.ndarray,
    right_body_from_camera: np.ndarray,
    ego_grid_pose: GridPose,
    right_grid_pose: GridPose,
) -> TagWorldConstraint:
    """Build one ``T_egoWorld_rightWorld`` observation through the static grid."""
    ego_grid = compose(
        ego_world_from_body, ego_body_from_camera, ego_grid_pose.camera_from_grid
    )
    right_grid = compose(
        right_world_from_body, right_body_from_camera, right_grid_pose.camera_from_grid
    )
    candidate = compose(ego_grid, invert(right_grid))
    return TagWorldConstraint(
        int(ego_stamp_ns), int(right_stamp_ns), phase,
        ego_grid, right_grid, candidate,
        ego_grid_pose.tag_ids, right_grid_pose.tag_ids,
        ego_grid_pose.reprojection_rms_px, right_grid_pose.reprojection_rms_px,
    )


def aggregate_transform(transforms: Sequence[np.ndarray]) -> np.ndarray:
    """Aggregate SE(3) samples without flattening or averaging rotation matrices."""
    values = [validate_transform(value) for value in transforms]
    if not values:
        raise ValueError("at least one transform is required")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_matrix(
        np.asarray([value[:3, :3] for value in values])
    ).mean().as_matrix()
    result[:3, 3] = np.median(
        np.asarray([value[:3, 3] for value in values]), axis=0
    )
    return validate_transform(result)


def _distribution(values: Sequence[float]) -> dict:
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(len(data)),
        "median": None if not len(data) else float(np.median(data)),
        "p95": None if not len(data) else float(np.percentile(data, 95)),
        "max": None if not len(data) else float(np.max(data)),
    }


def _error_summary(reference: np.ndarray, constraints: Sequence[TagWorldConstraint]) -> dict:
    errors = [pose_error(item.ego_world_from_right_world, reference) for item in constraints]
    return {
        "translation_m": _distribution([item.translation_m for item in errors]),
        "rotation_deg": _distribution([item.rotation_deg for item in errors]),
    }


def analyze_fixed_transform_stability(
    constraints: Sequence[TagWorldConstraint],
    *,
    phase_order: Sequence[str],
    required_phases: Sequence[str],
    min_observations_per_required_phase: int = 4,
    max_holdout_translation_p95_m: float = 0.03,
    max_holdout_rotation_p95_deg: float = 3.0,
    max_phase_anchor_translation_span_m: float = 0.03,
    max_phase_anchor_rotation_span_deg: float = 3.0,
) -> dict:
    """Cross-check whether one fixed world transform explains every phase.

    Even-indexed observations inside each phase form the candidate; odd-indexed
    observations are held out.  Phase anchors are estimated only from their
    training rows.  Thresholds are explicit caller policy rather than hidden
    product-release constants.
    """
    values = sorted(constraints, key=lambda item: item.ego_stamp_ns)
    if len(values) < 4:
        raise ValueError("at least four tag-world constraints are required")
    if any(b.ego_stamp_ns <= a.ego_stamp_ns for a, b in zip(values, values[1:])):
        raise ValueError("constraint Ego timestamps must be strictly increasing")
    phase_order = tuple(phase_order)
    required_phases = tuple(required_phases)
    if not phase_order or len(set(phase_order)) != len(phase_order):
        raise ValueError("phase_order must contain unique names")
    if not set(required_phases).issubset(phase_order):
        raise ValueError("required phases must be present in phase_order")
    unknown = sorted({item.phase for item in values} - set(phase_order))
    if unknown:
        raise ValueError(f"unknown constraint phases: {unknown}")
    if min_observations_per_required_phase < 2:
        raise ValueError("required phases need at least two observations")
    thresholds = (
        max_holdout_translation_p95_m,
        max_holdout_rotation_p95_deg,
        max_phase_anchor_translation_span_m,
        max_phase_anchor_rotation_span_deg,
    )
    if any(not np.isfinite(value) or value <= 0.0 for value in thresholds):
        raise ValueError("stability thresholds must be finite and positive")

    groups = {phase: [item for item in values if item.phase == phase]
              for phase in phase_order}
    train, heldout = [], []
    phase_train = {}
    for phase in phase_order:
        phase_train[phase] = groups[phase][::2]
        train.extend(phase_train[phase])
        heldout.extend(groups[phase][1::2])
    if not train or not heldout:
        raise ValueError("deterministic split produced an empty train or holdout set")
    fixed = aggregate_transform([item.ego_world_from_right_world for item in train])
    phase_anchors = {
        phase: aggregate_transform([
            item.ego_world_from_right_world for item in phase_train[phase]
        ])
        for phase in phase_order if phase_train[phase]
    }
    pairwise = []
    names = tuple(phase_anchors)
    for index, first in enumerate(names):
        for second in names[index + 1:]:
            error = pose_error(phase_anchors[first], phase_anchors[second])
            pairwise.append({
                "first": first,
                "second": second,
                "translation_m": error.translation_m,
                "rotation_deg": error.rotation_deg,
            })
    translation_span = max((row["translation_m"] for row in pairwise), default=0.0)
    rotation_span = max((row["rotation_deg"] for row in pairwise), default=0.0)
    counts = {phase: len(groups[phase]) for phase in phase_order}
    coverage = {
        phase: counts[phase] >= min_observations_per_required_phase
        for phase in required_phases
    }
    holdout_summary = _error_summary(fixed, heldout)
    holdout_by_phase = {
        phase: _error_summary(fixed, [item for item in heldout if item.phase == phase])
        for phase in phase_order
    }
    scored_phases = [
        summary for summary in holdout_by_phase.values()
        if summary["translation_m"]["count"]
    ]
    checks = {
        "required_phase_coverage": all(coverage.values()),
        "holdout_translation_p95_within_policy":
            holdout_summary["translation_m"]["p95"] <= max_holdout_translation_p95_m,
        "holdout_rotation_p95_within_policy":
            holdout_summary["rotation_deg"]["p95"] <= max_holdout_rotation_p95_deg,
        "each_observed_phase_translation_p95_within_policy": all(
            summary["translation_m"]["p95"] <= max_holdout_translation_p95_m
            for summary in scored_phases
        ),
        "each_observed_phase_rotation_p95_within_policy": all(
            summary["rotation_deg"]["p95"] <= max_holdout_rotation_p95_deg
            for summary in scored_phases
        ),
        "phase_anchor_translation_span_within_policy":
            translation_span <= max_phase_anchor_translation_span_m,
        "phase_anchor_rotation_span_within_policy":
            rotation_span <= max_phase_anchor_rotation_span_deg,
    }
    geometry_stable = all(value for key, value in checks.items()
                          if key != "required_phase_coverage")
    if not geometry_stable:
        status = "FAIL_FIXED_TRANSFORM_TIME_VARYING"
    elif not checks["required_phase_coverage"]:
        status = "INCONCLUSIVE_MISSING_REQUIRED_PHASES"
    else:
        status = "PASS_FIXED_TRANSFORM_STABLE_DIAGNOSTIC"
    return {
        "status": status,
        "fixed_transform": fixed.tolist(),
        "selection": {
            "observations": len(values), "train": len(train), "heldout": len(heldout),
            "phase_counts": counts, "required_phase_coverage": coverage,
        },
        "policy": {
            "min_observations_per_required_phase": min_observations_per_required_phase,
            "max_holdout_translation_p95_m": max_holdout_translation_p95_m,
            "max_holdout_rotation_p95_deg": max_holdout_rotation_p95_deg,
            "max_phase_anchor_translation_span_m": max_phase_anchor_translation_span_m,
            "max_phase_anchor_rotation_span_deg": max_phase_anchor_rotation_span_deg,
            "scope": "diagnostic caller policy; not a frozen product release contract",
        },
        "checks": checks,
        "holdout": holdout_summary,
        "holdout_by_phase": holdout_by_phase,
        "phase_anchors": {phase: value.tolist() for phase, value in phase_anchors.items()},
        "phase_anchor_pairwise": pairwise,
        "phase_anchor_translation_span_m": translation_span,
        "phase_anchor_rotation_span_deg": rotation_span,
    }
