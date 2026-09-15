"""Image-level AprilTag reference detection and square-IPPE pose solving.

This module deliberately labels its OpenCV ArUco implementation as a
reference/replay backend.  The product detector remains AprilRobotics
AprilTag; both backends must preserve the same pose contract:
``T_camera_tag`` maps points from the tag frame into the optical camera frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .apriltag_alignment import TagPoseObservation
from .se3 import pose_error, validate_transform


BACKEND = "opencv_aruco_reference"
EVIDENCE_CLASS = "simulation_image_replay"
CORNER_REFINEMENTS = ("none", "apriltag")


def validate_corner_refinement(value: str) -> str:
    if not isinstance(value, str) or value not in CORNER_REFINEMENTS:
        raise ValueError("corner_refinement must be 'none' or 'apriltag'")
    return value


def _nonempty(value: str, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _positive_integer(value: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _timestamp(value: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _positive_float(value: float, *, name: str, allow_zero: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite and positive")
    result = float(value)
    if not math.isfinite(result) or result < 0.0 or (not allow_zero and result == 0.0):
        raise ValueError(f"{name} must be finite and positive")
    return result


def _readonly_array(
    value: np.ndarray | Iterable[float], *, name: str, shape: tuple[int, ...]
) -> np.ndarray:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class CameraCalibration:
    calibration_id: str
    frame_id: str
    width: int
    height: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    distortion_model: str = "opencv_brown_conrady"

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "calibration_id", _nonempty(self.calibration_id, name="calibration_id")
        )
        object.__setattr__(self, "frame_id", _nonempty(self.frame_id, name="frame_id"))
        object.__setattr__(self, "width", _positive_integer(self.width, name="width"))
        object.__setattr__(self, "height", _positive_integer(self.height, name="height"))
        matrix = _readonly_array(
            self.camera_matrix, name="camera_matrix", shape=(3, 3)
        )
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0 or not np.allclose(
            matrix[2], (0.0, 0.0, 1.0), atol=1e-12
        ):
            raise ValueError("camera_matrix must contain positive focal lengths")
        distortion = np.array(
            self.distortion_coefficients, dtype=np.float64, copy=True
        ).reshape(-1)
        if distortion.size not in {4, 5, 8, 12, 14} or not np.all(
            np.isfinite(distortion)
        ):
            raise ValueError("distortion_coefficients has an unsupported shape")
        distortion.setflags(write=False)
        if self.distortion_model not in (
            "opencv_brown_conrady", "distortion.none", "distortion.brown_conrady",
            "distortion.inverse_brown_conrady",
        ):
            raise ValueError(f"unsupported distortion_model: {self.distortion_model}")
        if self.distortion_model.startswith("distortion.") and distortion.size != 5:
            raise ValueError("RealSense distortion requires exactly five coefficients")
        if self.distortion_model == "distortion.none" and np.any(distortion):
            raise ValueError("distortion.none requires zero coefficients")
        object.__setattr__(self, "camera_matrix", matrix)
        object.__setattr__(self, "distortion_coefficients", distortion)


def _sdk_intrinsics(calibration):
    # Geometry functions only: importing the SDK does not open a camera.
    # No silent OpenCV fallback when the declared factory model needs SDK math.
    import pyrealsense2 as rs

    intr = rs.intrinsics()
    intr.width, intr.height = calibration.width, calibration.height
    k = calibration.camera_matrix
    intr.fx, intr.fy, intr.ppx, intr.ppy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    intr.coeffs = calibration.distortion_coefficients.tolist()
    intr.model = getattr(rs.distortion, calibration.distortion_model.split(".")[-1])
    return rs, intr


def normalized_camera_points(pixels, calibration):
    import cv2

    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    if calibration.distortion_model == "distortion.inverse_brown_conrady":
        rs, intr = _sdk_intrinsics(calibration)
        return np.array([rs.rs2_deproject_pixel_to_point(intr, p.tolist(), 1.)[:2]
                         for p in points], dtype=np.float64)
    return cv2.undistortPoints(points.reshape(-1, 1, 2), calibration.camera_matrix,
                              calibration.distortion_coefficients).reshape(-1, 2)


def project_camera_points(points, calibration):
    import cv2

    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if calibration.distortion_model == "distortion.inverse_brown_conrady":
        rs, intr = _sdk_intrinsics(calibration)
        return np.array([rs.rs2_project_point_to_pixel(intr, p.tolist()) for p in points])
    return cv2.projectPoints(points, np.zeros(3), np.zeros(3), calibration.camera_matrix,
                             calibration.distortion_coefficients)[0].reshape(-1, 2)


@dataclass(frozen=True)
class AprilTagDetectorConfig:
    family: str
    tag_size_m: float
    allowed_tag_ids: tuple[int, ...]
    max_reprojection_error_px: float = 1.5
    ambiguity_error_gap_px: float = 0.05
    ambiguity_translation_m: float = 0.005
    ambiguity_rotation_deg: float = 5.0
    # Preserve the legacy shared detector default. Bench callers opt in.
    corner_refinement: str = "none"

    def __post_init__(self) -> None:
        validate_corner_refinement(self.corner_refinement)
        family = _nonempty(self.family, name="family")
        if family != "tag36h11":
            raise ValueError("OpenCV reference backend supports only tag36h11")
        ids = tuple(self.allowed_tag_ids)
        if not ids or any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in ids
        ):
            raise ValueError("allowed_tag_ids must contain non-negative integers")
        if len(set(ids)) != len(ids):
            raise ValueError("allowed_tag_ids must be unique")
        object.__setattr__(self, "family", family)
        object.__setattr__(self, "allowed_tag_ids", ids)
        for name in (
            "tag_size_m",
            "max_reprojection_error_px",
            "ambiguity_error_gap_px",
            "ambiguity_translation_m",
            "ambiguity_rotation_deg",
        ):
            object.__setattr__(
                self,
                name,
                _positive_float(
                    getattr(self, name),
                    name=name,
                    allow_zero=name == "ambiguity_error_gap_px",
                ),
            )


@dataclass(frozen=True)
class PnPCandidate:
    camera_from_tag: np.ndarray
    reprojection_error_px: float
    positive_depth: bool

    def __post_init__(self) -> None:
        transform = validate_transform(self.camera_from_tag, name="camera_from_tag")
        transform.setflags(write=False)
        object.__setattr__(self, "camera_from_tag", transform)
        object.__setattr__(
            self,
            "reprojection_error_px",
            _positive_float(
                self.reprojection_error_px,
                name="reprojection_error_px",
                allow_zero=True,
            ),
        )
        if not isinstance(self.positive_depth, bool):
            raise ValueError("positive_depth must be bool")


@dataclass(frozen=True)
class PnPSelection:
    selected_index: int | None
    reason: str

    def __post_init__(self) -> None:
        if self.selected_index is not None and (
            isinstance(self.selected_index, bool)
            or not isinstance(self.selected_index, int)
            or self.selected_index < 0
        ):
            raise ValueError("selected_index must be a non-negative integer or None")
        object.__setattr__(self, "reason", _nonempty(self.reason, name="reason"))
        if (self.reason == "accepted") != (self.selected_index is not None):
            raise ValueError("accepted selection must have exactly one selected index")


@dataclass(frozen=True)
class AprilTagImageDetection:
    tag_id: int
    corners_px: np.ndarray
    candidates: tuple[PnPCandidate, ...]
    selected_index: int | None
    reason: str
    observation: TagPoseObservation | None

    def __post_init__(self) -> None:
        if isinstance(self.tag_id, bool) or not isinstance(self.tag_id, int) or self.tag_id < 0:
            raise ValueError("tag_id must be a non-negative integer")
        corners = _readonly_array(self.corners_px, name="corners_px", shape=(4, 2))
        object.__setattr__(self, "corners_px", corners)
        candidates = tuple(self.candidates)
        if any(not isinstance(candidate, PnPCandidate) for candidate in candidates):
            raise ValueError("candidates must contain PnPCandidate values")
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "reason", _nonempty(self.reason, name="reason"))
        if self.selected_index is not None and (
            isinstance(self.selected_index, bool)
            or not isinstance(self.selected_index, int)
            or not 0 <= self.selected_index < len(candidates)
        ):
            raise ValueError("selected_index does not identify a candidate")
        accepted = self.reason == "accepted"
        if accepted:
            if self.selected_index is None or self.observation is None:
                raise ValueError("accepted detection must bind a candidate and observation")
            selected = candidates[self.selected_index]
            if not np.allclose(
                self.observation.camera_from_tag,
                selected.camera_from_tag,
                rtol=0.0,
                atol=1e-12,
            ) or not math.isclose(
                self.observation.reprojection_error_px,
                selected.reprojection_error_px,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise ValueError("observation must equal the selected PnP candidate")
        elif self.selected_index is not None or self.observation is not None:
            raise ValueError("rejected detection cannot bind a candidate or observation")
        if self.observation is not None and self.observation.tag_id != self.tag_id:
            raise ValueError("observation tag_id must match detection tag_id")

    @property
    def accepted(self) -> bool:
        return self.observation is not None and self.reason == "accepted"


@dataclass(frozen=True)
class AprilTagDetectionBatch:
    acquisition_timestamp_ns: int
    detector_completed_ns: int
    camera_frame_id: str
    calibration_id: str
    backend: str
    evidence_class: str
    detections: tuple[AprilTagImageDetection, ...]
    rejected_quad_count: int

    def __post_init__(self) -> None:
        acquisition = _timestamp(
            self.acquisition_timestamp_ns, name="acquisition_timestamp_ns"
        )
        completed = _timestamp(
            self.detector_completed_ns, name="detector_completed_ns"
        )
        if completed < acquisition:
            raise ValueError("detector_completed_ns cannot precede acquisition timestamp")
        object.__setattr__(self, "acquisition_timestamp_ns", acquisition)
        object.__setattr__(self, "detector_completed_ns", completed)
        for name in ("camera_frame_id", "calibration_id", "backend", "evidence_class"):
            object.__setattr__(self, name, _nonempty(getattr(self, name), name=name))
        detections = tuple(self.detections)
        if any(not isinstance(item, AprilTagImageDetection) for item in detections):
            raise ValueError("detections must contain AprilTagImageDetection values")
        object.__setattr__(self, "detections", detections)
        if (
            isinstance(self.rejected_quad_count, bool)
            or not isinstance(self.rejected_quad_count, int)
            or self.rejected_quad_count < 0
        ):
            raise ValueError("rejected_quad_count must be a non-negative integer")


def select_ippe_candidate(
    candidates: Iterable[PnPCandidate],
    *,
    max_reprojection_error_px: float,
    ambiguity_error_gap_px: float,
    ambiguity_translation_m: float,
    ambiguity_rotation_deg: float,
) -> PnPSelection:
    """Select one positive-depth IPPE solution or return an auditable rejection."""
    max_reprojection_error_px = _positive_float(
        max_reprojection_error_px, name="max_reprojection_error_px"
    )
    ambiguity_error_gap_px = _positive_float(
        ambiguity_error_gap_px, name="ambiguity_error_gap_px", allow_zero=True
    )
    ambiguity_translation_m = _positive_float(
        ambiguity_translation_m, name="ambiguity_translation_m"
    )
    ambiguity_rotation_deg = _positive_float(
        ambiguity_rotation_deg, name="ambiguity_rotation_deg"
    )
    values = tuple(candidates)
    if any(not isinstance(candidate, PnPCandidate) for candidate in values):
        raise ValueError("candidates must contain PnPCandidate values")
    indexed = sorted(
        (
            (index, candidate)
            for index, candidate in enumerate(values)
            if candidate.positive_depth
        ),
        key=lambda item: item[1].reprojection_error_px,
    )
    if not indexed:
        return PnPSelection(None, "no_positive_depth_candidate")
    best_index, best = indexed[0]
    if best.reprojection_error_px > max_reprojection_error_px:
        return PnPSelection(None, "reprojection_error")
    if len(indexed) > 1:
        _second_index, second = indexed[1]
        if (
            second.reprojection_error_px - best.reprojection_error_px
            <= ambiguity_error_gap_px
        ):
            difference = pose_error(best.camera_from_tag, second.camera_from_tag)
            if (
                difference.translation_m >= ambiguity_translation_m
                or difference.rotation_deg >= ambiguity_rotation_deg
            ):
                return PnPSelection(None, "planar_pose_ambiguity")
    return PnPSelection(best_index, "accepted")


def detect_apriltags(
    image: np.ndarray,
    *,
    calibration: CameraCalibration,
    config: AprilTagDetectorConfig,
    acquisition_timestamp_ns: int,
    detector_completed_ns: int,
) -> AprilTagDetectionBatch:
    """Detect `tag36h11` pixels and solve metric ``T_camera_tag`` poses."""
    acquisition = _timestamp(
        acquisition_timestamp_ns, name="acquisition_timestamp_ns"
    )
    completed = _timestamp(detector_completed_ns, name="detector_completed_ns")
    if completed < acquisition:
        raise ValueError("detector_completed_ns cannot precede acquisition timestamp")
    pixels = np.asarray(image)
    if pixels.shape[:2] != (calibration.height, calibration.width) or pixels.ndim not in {
        2,
        3,
    }:
        raise ValueError(
            f"image shape must begin with {(calibration.height, calibration.width)}"
        )
    if pixels.ndim == 3 and pixels.shape[2] not in {3, 4}:
        raise ValueError("image shape must be grayscale, BGR, or BGRA")
    if pixels.dtype != np.uint8:
        raise ValueError("image must use uint8 pixels")

    import cv2

    if pixels.ndim == 3:
        conversion = cv2.COLOR_BGR2GRAY if pixels.shape[2] == 3 else cv2.COLOR_BGRA2GRAY
        grayscale = cv2.cvtColor(pixels, conversion)
    else:
        grayscale = pixels
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = (
        cv2.aruco.CORNER_REFINE_APRILTAG
        if config.corner_refinement == "apriltag" else cv2.aruco.CORNER_REFINE_NONE
    )
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejected = detector.detectMarkers(grayscale)
    detections: list[AprilTagImageDetection] = []
    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
            tag_id = int(marker_id)
            image_corners = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            if tag_id not in config.allowed_tag_ids:
                detections.append(
                    AprilTagImageDetection(
                        tag_id=tag_id,
                        corners_px=image_corners,
                        candidates=(),
                        selected_index=None,
                        reason="unknown_tag_id",
                        observation=None,
                    )
                )
                continue
            candidates = _solve_ippe_candidates(
                image_corners, calibration=calibration, tag_size_m=config.tag_size_m
            )
            selection = select_ippe_candidate(
                candidates,
                max_reprojection_error_px=config.max_reprojection_error_px,
                ambiguity_error_gap_px=config.ambiguity_error_gap_px,
                ambiguity_translation_m=config.ambiguity_translation_m,
                ambiguity_rotation_deg=config.ambiguity_rotation_deg,
            )
            observation = None
            if selection.selected_index is not None:
                chosen = candidates[selection.selected_index]
                observation = TagPoseObservation(
                    stamped_timestamp_ns=acquisition,
                    detector_completed_ns=completed,
                    family=config.family,
                    tag_id=tag_id,
                    camera_from_tag=chosen.camera_from_tag,
                    reprojection_error_px=chosen.reprojection_error_px,
                    hamming=0,
                )
            detections.append(
                AprilTagImageDetection(
                    tag_id=tag_id,
                    corners_px=image_corners,
                    candidates=candidates,
                    selected_index=selection.selected_index,
                    reason=selection.reason,
                    observation=observation,
                )
            )
    detections.sort(key=lambda item: item.tag_id)
    return AprilTagDetectionBatch(
        acquisition_timestamp_ns=acquisition,
        detector_completed_ns=completed,
        camera_frame_id=calibration.frame_id,
        calibration_id=calibration.calibration_id,
        backend=BACKEND,
        evidence_class=EVIDENCE_CLASS,
        detections=tuple(detections),
        rejected_quad_count=len(rejected),
    )


def _solve_ippe_candidates(
    image_corners: np.ndarray,
    *,
    calibration: CameraCalibration,
    tag_size_m: float,
) -> tuple[PnPCandidate, ...]:
    import cv2

    half = tag_size_m / 2.0
    object_points = np.array(
        (
            (-half, half, 0.0),
            (half, half, 0.0),
            (half, -half, 0.0),
            (-half, -half, 0.0),
        ),
        dtype=np.float64,
    )
    inverse_factory = calibration.distortion_model == "distortion.inverse_brown_conrady"
    solved, rotations, translations, _errors = cv2.solvePnPGeneric(
        object_points,
        normalized_camera_points(image_corners, calibration) if inverse_factory else image_corners,
        np.eye(3) if inverse_factory else calibration.camera_matrix,
        np.zeros(5) if inverse_factory else calibration.distortion_coefficients,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not solved:
        return ()
    candidates: list[PnPCandidate] = []
    for rotation_vector, translation_vector in zip(rotations, translations):
        rotation_vector = np.asarray(rotation_vector, dtype=np.float64).reshape(3)
        translation = np.asarray(translation_vector, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(rotation_vector)) or not np.all(np.isfinite(translation)):
            continue
        rotation, _ = cv2.Rodrigues(rotation_vector)
        transformed = (rotation @ object_points.T).T + translation
        if inverse_factory:
            projected = project_camera_points(transformed, calibration)
        else:
            projected, _ = cv2.projectPoints(
                object_points, rotation_vector, translation, calibration.camera_matrix,
                calibration.distortion_coefficients,
            )
        residual = projected.reshape(4, 2) - image_corners
        rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        candidates.append(
            PnPCandidate(
                camera_from_tag=transform,
                reprojection_error_px=rmse,
                positive_depth=bool(np.all(transformed[:, 2] > 0.0)),
            )
        )
    return tuple(candidates)
