import numpy as np
from scipy.spatial.transform import Rotation

from scripts.analyze_tag_world_constraints import detect_grid_scaled
from three_device_slam.spatial.se3 import compose, invert, transform_from_xyz_rpy
from three_device_slam.spatial.tag_world_constraints import (
    GridPose,
    aggregate_transform,
    analyze_fixed_transform_stability,
    aprilgrid_object_corners,
    make_world_constraint,
    solve_grid_pose,
)


PHASES = ("initial_static", "translation", "yaw", "final_static")


def test_scaled_detection_returns_original_pixel_coordinates():
    class Detector:
        def detectMarkers(self, image):
            assert image.shape == (8, 12)
            corners = np.array([[[2.0, 4.0], [6.0, 4.0], [6.0, 8.0], [2.0, 8.0]]])
            return [corners], np.array([[7]]), []

    detections = detect_grid_scaled(Detector(), np.zeros((4, 6), np.uint8), 2)

    assert len(detections) == 1
    assert detections[0].tag_id == 7
    assert np.array_equal(
        detections[0].corners,
        np.array([[1.0, 2.0], [3.0, 2.0], [3.0, 4.0], [1.0, 4.0]]),
    )


def _grid_detections(camera_from_grid):
    import cv2

    matrix = np.array(((600.0, 0.0, 640.0), (0.0, 600.0, 360.0), (0.0, 0.0, 1.0)))
    result = {}
    rvec = Rotation.from_matrix(camera_from_grid[:3, :3]).as_rotvec()
    for tag_id in (0, 2, 4, 12, 14, 16, 24, 26, 28):
        result[tag_id] = cv2.projectPoints(
            aprilgrid_object_corners(tag_id), rvec, camera_from_grid[:3, 3],
            matrix, np.zeros(5),
        )[0].reshape(4, 2)
    return matrix, result


def test_grid_pose_uses_decoded_ids_for_metric_corners():
    truth = transform_from_xyz_rpy((0.03, -0.04, 0.55), (0.12, -0.18, 0.07))
    matrix, detections = _grid_detections(truth)

    solved = solve_grid_pose(dict(reversed(tuple(detections.items()))), matrix)

    assert solved is not None
    assert solved.tag_ids == tuple(sorted(detections))
    assert solved.reprojection_rms_px < 1e-5
    assert np.allclose(solved.camera_from_grid, truth, atol=1e-6)


def test_world_constraint_has_explicit_ego_from_right_direction():
    ego_world_from_right = transform_from_xyz_rpy((0.4, -0.2, 0.1), (0.02, -0.04, 0.3))
    right_world_from_body = transform_from_xyz_rpy((0.1, 0.2, -0.1), (0.1, 0.2, -0.1))
    body_from_camera = transform_from_xyz_rpy((0.02, 0.0, 0.01))
    camera_from_grid = transform_from_xyz_rpy((0.0, 0.0, 0.7), (0.1, 0.0, 0.0))
    right_grid = compose(right_world_from_body, body_from_camera, camera_from_grid)
    ego_world_from_body = compose(
        ego_world_from_right, right_grid, invert(camera_from_grid), invert(body_from_camera)
    )
    pose = GridPose(camera_from_grid, (0, 1, 6, 7, 12, 13), 0.2)

    constraint = make_world_constraint(
        ego_stamp_ns=100, right_stamp_ns=103, phase="translation",
        ego_world_from_body=ego_world_from_body,
        right_world_from_body=right_world_from_body,
        ego_body_from_camera=body_from_camera,
        right_body_from_camera=body_from_camera,
        ego_grid_pose=pose, right_grid_pose=pose,
    )

    assert np.allclose(constraint.ego_world_from_right_world, ego_world_from_right)


def _constraints(*, phase_shift_m=0.0, include_endpoints=True):
    values = []
    fixed = transform_from_xyz_rpy((0.3, -0.2, 0.1), (0.01, -0.02, 0.15))
    phases = PHASES if include_endpoints else PHASES[1:3]
    stamp = 0
    pose = GridPose(np.eye(4), (0, 1, 6, 7, 12, 13), 0.2)
    for phase in phases:
        for index in range(6):
            shift = phase_shift_m if phase == "yaw" else 0.0
            observed = fixed.copy()
            observed[0, 3] += shift + (index - 2.5) * 0.0001
            right_body = transform_from_xyz_rpy((index * 0.01, 0.0, 0.0))
            ego_body = compose(observed, right_body)
            values.append(make_world_constraint(
                ego_stamp_ns=stamp, right_stamp_ns=stamp + 1_000_000, phase=phase,
                ego_world_from_body=ego_body, right_world_from_body=right_body,
                ego_body_from_camera=np.eye(4), right_body_from_camera=np.eye(4),
                ego_grid_pose=pose, right_grid_pose=pose,
            ))
            stamp += 10_000_000
    return values, fixed


def test_stability_passes_constant_transform_with_all_required_phases():
    constraints, truth = _constraints()
    result = analyze_fixed_transform_stability(
        constraints, phase_order=PHASES,
        required_phases=("initial_static", "final_static"),
    )
    assert result["status"] == "PASS_FIXED_TRANSFORM_STABLE_DIAGNOSTIC"
    assert all(result["checks"].values())
    assert np.allclose(aggregate_transform(
        [item.ego_world_from_right_world for item in constraints]
    ), truth, atol=3e-4)


def test_stability_rejects_motion_phase_dependent_transform():
    constraints, _ = _constraints(phase_shift_m=0.05)
    result = analyze_fixed_transform_stability(
        constraints, phase_order=PHASES,
        required_phases=("initial_static", "final_static"),
    )
    assert result["status"] == "FAIL_FIXED_TRANSFORM_TIME_VARYING"
    assert not result["checks"]["phase_anchor_translation_span_within_policy"]
    assert not result["checks"]["each_observed_phase_translation_p95_within_policy"]


def test_stability_reports_missing_static_endpoint_coverage():
    constraints, _ = _constraints(include_endpoints=False)
    result = analyze_fixed_transform_stability(
        constraints, phase_order=PHASES,
        required_phases=("initial_static", "final_static"),
    )
    assert result["status"] == "INCONCLUSIVE_MISSING_REQUIRED_PHASES"
    assert result["selection"]["required_phase_coverage"] == {
        "initial_static": False, "final_static": False
    }
