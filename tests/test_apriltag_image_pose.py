import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from three_device_slam.spatial.apriltag_detector import (
    AprilTagDetectorConfig,
    AprilTagImageDetection,
    CameraCalibration,
    PnPCandidate,
    detect_apriltags,
    select_ippe_candidate,
)
from three_device_slam.spatial.apriltag_alignment import TagPoseObservation
from three_device_slam.spatial.se3 import pose_error, transform_from_xyz_rpy


_ROOT = Path(__file__).resolve().parents[1]


def test_pixel_detection_contracts_run_in_an_isolated_interpreter():
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            "from tests.test_apriltag_image_pose import _run_pixel_contract_checks; _run_pixel_contract_checks()",
        ],
        cwd=_ROOT,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr


def _run_pixel_contract_checks():
    _check_pixels_produce_left_and_right_metric_camera_poses()
    _check_tag_size_is_black_square_and_controls_metric_scale()
    _check_unknown_tag_id_is_retained_as_auditable_rejection()
    _check_blank_or_occluded_image_has_no_false_observation()


def _check_pixels_produce_left_and_right_metric_camera_poses():
    calibration = _calibration()
    left_truth = transform_from_xyz_rpy(
        (-0.12, -0.03, 0.50), (math.pi - 0.25, 0.18, -0.10)
    )
    right_truth = transform_from_xyz_rpy(
        (0.14, 0.04, 0.60), (math.pi + 0.22, -0.16, 0.14)
    )
    image = _render_scene(
        calibration,
        (("left", left_truth), ("right", right_truth)),
    )

    batch = detect_apriltags(
        image,
        calibration=calibration,
        config=_config(),
        acquisition_timestamp_ns=1_000_000_000,
        detector_completed_ns=1_018_000_000,
    )

    accepted = {item.tag_id: item for item in batch.detections if item.accepted}
    assert set(accepted) == {0, 1}
    assert batch.backend == "opencv_aruco_reference"
    assert batch.evidence_class == "simulation_image_replay"
    for tag_id, truth in ((0, left_truth), (1, right_truth)):
        observation = accepted[tag_id].observation
        error = pose_error(observation.camera_from_tag, truth)
        assert observation.stamped_timestamp_ns == 1_000_000_000
        assert observation.detector_completed_ns == 1_018_000_000
        assert observation.reprojection_error_px < 0.8
        assert error.translation_m < 0.018
        assert error.rotation_deg < 3.0


def _check_tag_size_is_black_square_and_controls_metric_scale():
    calibration = _calibration()
    truth = transform_from_xyz_rpy((0.0, 0.0, 0.5), (math.pi - 0.25, 0.16, 0.0))
    image = _render_scene(calibration, (("left", truth),))

    correct = detect_apriltags(
        image,
        calibration=calibration,
        config=_config(tag_size_m=0.040),
        acquisition_timestamp_ns=10,
        detector_completed_ns=20,
    )
    wrong = detect_apriltags(
        image,
        calibration=calibration,
        config=_config(tag_size_m=0.020),
        acquisition_timestamp_ns=10,
        detector_completed_ns=20,
    )

    correct_z = _accepted(correct, 0).observation.camera_from_tag[2, 3]
    wrong_z = _accepted(wrong, 0).observation.camera_from_tag[2, 3]
    assert abs(correct_z - 0.5) < 0.015
    assert wrong_z / correct_z == pytest.approx(0.5, abs=0.01)


def _check_unknown_tag_id_is_retained_as_auditable_rejection():
    calibration = _calibration()
    image = _render_scene(
        calibration,
        (("right", transform_from_xyz_rpy((0.0, 0.0, 0.7), (math.pi, 0.0, 0.0))),),
    )

    batch = detect_apriltags(
        image,
        calibration=calibration,
        config=_config(allowed_tag_ids=(0,)),
        acquisition_timestamp_ns=100,
        detector_completed_ns=120,
    )

    assert len(batch.detections) == 1
    assert batch.detections[0].tag_id == 1
    assert batch.detections[0].accepted is False
    assert batch.detections[0].reason == "unknown_tag_id"
    assert batch.detections[0].observation is None


def _check_blank_or_occluded_image_has_no_false_observation():
    calibration = _calibration()
    image = np.full((calibration.height, calibration.width), 220, dtype=np.uint8)

    batch = detect_apriltags(
        image,
        calibration=calibration,
        config=_config(),
        acquisition_timestamp_ns=100,
        detector_completed_ns=120,
    )

    assert batch.detections == ()
    assert batch.rejected_quad_count == 0


def test_planar_ambiguity_and_negative_depth_are_rejected_by_pure_gate():
    best = PnPCandidate(
        camera_from_tag=transform_from_xyz_rpy((0.0, 0.0, 0.8), (math.pi, 0.0, 0.0)),
        reprojection_error_px=0.20,
        positive_depth=True,
    )
    distinct_near_tie = PnPCandidate(
        camera_from_tag=transform_from_xyz_rpy(
            (0.01, 0.0, 0.8), (math.pi, math.radians(12.0), 0.0)
        ),
        reprojection_error_px=0.22,
        positive_depth=True,
    )
    negative_depth = PnPCandidate(
        camera_from_tag=transform_from_xyz_rpy((0.0, 0.0, -0.8)),
        reprojection_error_px=0.01,
        positive_depth=False,
    )

    ambiguous = select_ippe_candidate(
        (negative_depth, best, distinct_near_tie),
        max_reprojection_error_px=1.0,
        ambiguity_error_gap_px=0.05,
        ambiguity_translation_m=0.005,
        ambiguity_rotation_deg=5.0,
    )
    selected = select_ippe_candidate(
        (negative_depth, best),
        max_reprojection_error_px=1.0,
        ambiguity_error_gap_px=0.05,
        ambiguity_translation_m=0.005,
        ambiguity_rotation_deg=5.0,
    )

    assert ambiguous.reason == "planar_pose_ambiguity"
    assert ambiguous.selected_index is None
    assert selected.reason == "accepted"
    assert selected.selected_index == 1


def test_input_contract_rejects_wrong_shape_and_completion_regression():
    calibration = _calibration()
    config = _config()

    with pytest.raises(ValueError, match="image shape"):
        detect_apriltags(
            np.zeros((360, 640), dtype=np.uint8),
            calibration=calibration,
            config=config,
            acquisition_timestamp_ns=10,
            detector_completed_ns=20,
        )
    with pytest.raises(ValueError, match="cannot precede"):
        detect_apriltags(
            np.zeros((720, 1280), dtype=np.uint8),
            calibration=calibration,
            config=config,
            acquisition_timestamp_ns=20,
            detector_completed_ns=10,
        )


def test_detection_model_binds_observation_to_exact_selected_candidate():
    candidate = PnPCandidate(
        camera_from_tag=transform_from_xyz_rpy((0.0, 0.0, 0.5)),
        reprojection_error_px=0.2,
        positive_depth=True,
    )
    mismatched = TagPoseObservation(
        stamped_timestamp_ns=10,
        detector_completed_ns=20,
        family="tag36h11",
        tag_id=0,
        camera_from_tag=transform_from_xyz_rpy((1.0, 0.0, 0.5)),
        reprojection_error_px=0.2,
    )

    with pytest.raises(ValueError, match="selected PnP candidate"):
        AprilTagImageDetection(
            tag_id=0,
            corners_px=np.zeros((4, 2)),
            candidates=(candidate,),
            selected_index=0,
            reason="accepted",
            observation=mismatched,
        )
    with pytest.raises(ValueError, match="rejected detection"):
        AprilTagImageDetection(
            tag_id=0,
            corners_px=np.zeros((4, 2)),
            candidates=(candidate,),
            selected_index=0,
            reason="planar_pose_ambiguity",
            observation=None,
        )


def _calibration() -> CameraCalibration:
    return CameraCalibration(
        calibration_id="D435I_FACTORY_327122078613_20260828",
        frame_id="ego_ir_left_optical",
        width=1280,
        height=720,
        camera_matrix=np.array(
            ((644.0672, 0.0, 642.2068), (0.0, 644.0672, 367.0062), (0.0, 0.0, 1.0)),
            dtype=np.float64,
        ),
        distortion_coefficients=np.zeros(5, dtype=np.float64),
    )


def _config(**overrides) -> AprilTagDetectorConfig:
    values = {
        "family": "tag36h11",
        "tag_size_m": 0.040,
        "allowed_tag_ids": (0, 1),
        "max_reprojection_error_px": 1.0,
    }
    values.update(overrides)
    return AprilTagDetectorConfig(**values)


def _accepted(batch, tag_id):
    return next(item for item in batch.detections if item.tag_id == tag_id and item.accepted)


def _render_scene(calibration, tags):
    import cv2

    image = np.full((calibration.height, calibration.width), 210, dtype=np.uint8)
    for label, camera_from_tag in tags:
        source = cv2.imread(
            str(_ROOT / "assets" / "calibration_tags" / f"umi_{label}_tag36h11_id{0 if label == 'left' else 1}_40mm.png"),
            cv2.IMREAD_GRAYSCALE,
        )
        assert source is not None
        # The official bitmap's raster orientation is rotated 180 degrees
        # relative to the detector's canonical tag axes.  Normalize it here
        # so the supplied truth is exactly T_camera_tag, not a paper-image
        # coordinate frame.
        source = np.rot90(source, 2).copy()
        # The PNG is a 10x10 grid: 50 mm including quiet zone. Detection
        # corners surround the inner 8x8 black square, whose physical size is
        # the configured 40 mm tag size.
        outer = 0.025
        object_corners = np.array(
            ((-outer, outer, 0.0), (outer, outer, 0.0), (outer, -outer, 0.0), (-outer, -outer, 0.0)),
            dtype=np.float64,
        )
        rotation, _ = cv2.Rodrigues(camera_from_tag[:3, :3])
        projected, _ = cv2.projectPoints(
            object_corners,
            rotation,
            camera_from_tag[:3, 3],
            calibration.camera_matrix,
            calibration.distortion_coefficients,
        )
        source_corners = np.array(
            ((0.0, 0.0), (source.shape[1] - 1.0, 0.0), (source.shape[1] - 1.0, source.shape[0] - 1.0), (0.0, source.shape[0] - 1.0)),
            dtype=np.float32,
        )
        homography = cv2.getPerspectiveTransform(
            source_corners, projected.reshape(4, 2).astype(np.float32)
        )
        warped = cv2.warpPerspective(
            source,
            homography,
            (calibration.width, calibration.height),
            flags=cv2.INTER_NEAREST,
            borderValue=255,
        )
        mask = cv2.warpPerspective(
            np.full(source.shape, 255, dtype=np.uint8),
            homography,
            (calibration.width, calibration.height),
            flags=cv2.INTER_NEAREST,
            borderValue=0,
        )
        image[mask > 0] = warped[mask > 0]
    return image
