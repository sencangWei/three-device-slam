from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from scripts import fixed_factory_board_diagnostic as diagnostic
from scripts.color_aprilgrid_capture import make_detector, detect_grid
from three_device_slam.spatial.apriltag_detector import CameraCalibration, project_camera_points


def calibration():
    return CameraCalibration('test', 'camera', 1280, 720,
                             np.array([[700., 0, 640], [0, 700, 360], [0, 0, 1]]), np.zeros(5))


def observations():
    cal = calibration()
    rotation = cv2.Rodrigues(np.array([2.9, .15, -.1]))[0]
    translation = np.array([-.1, .1, .65])
    return cal, {i: project_camera_points(diagnostic.object_corners(i) @ rotation.T + translation, cal)
                 for i in range(36)}


def test_exact_projection_and_checkerboard_holdout():
    cal, detected = observations()
    for parity in (0, 1):
        result = diagnostic.fit_fold(detected, cal, parity)
        assert result['status'] == 'OK'
        assert result['holdout_rms_px'] < 1e-6
        assert set(result['train_ids']).isdisjoint(result['holdout_ids'])
        assert all((i % 6 + i // 6) % 2 == parity for i in result['train_ids'])
        assert not cal.camera_matrix.flags.writeable


def test_heldout_points_cannot_change_pose_or_branch():
    cal, detected = observations()
    first = diagnostic.fit_fold(detected, cal, 0)
    shifted = {i: corners + ([2., -3.] if (i % 6 + i // 6) % 2 else [0., 0.])
               for i, corners in detected.items()}
    second = diagnostic.fit_fold(shifted, cal, 0)
    np.testing.assert_array_equal(first['camera_from_board'], second['camera_from_board'])
    assert first['selected_seed'] == second['selected_seed']
    assert second['holdout_rms_px'] == pytest.approx(np.sqrt(13), abs=1e-6)


def test_insufficient_training_spread_is_explicit():
    cal, detected = observations()
    result = diagnostic.fit_fold({i: d for i, d in detected.items() if i < 12}, cal, 0)
    assert result['status'] == 'SKIPPED_TRAIN_COVERAGE'


@pytest.mark.parametrize('rotation', [0, 2])
def test_legacy_board_corner_convention_with_independent_render(rotation):
    # Legacy physical board: IDs increase right/up; each canonical marker is
    # upside down in the normal image. Pixel geometry independently specified.
    gray = np.full((720, 1280), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    expected = {}
    for row in range(6):
        for col in range(6):
            tag = row * 6 + col
            x, y = 200 + col * 78, 60 + (5-row) * 78
            marker = cv2.aruco.generateImageMarker(dictionary, tag, 60, borderBits=2)
            gray[y:y+60, x:x+60] = np.rot90(marker, 2)
            expected[tag] = np.array([[x+59, y+59], [x, y+59], [x, y], [x+59, y]], float)
    if rotation:
        gray = np.ascontiguousarray(np.rot90(gray, 2))
        expected = {i: np.array([1279., 719.]) - p for i, p in expected.items()}
    detections = detect_grid(make_detector(), gray)
    assert len(detections) == 36
    for d in detections:
        np.testing.assert_allclose(d.corners, expected[d.tag_id], atol=1.)
        offsets = diagnostic.object_corners(d.tag_id)[:, :2] - np.array([d.tag_id % 6, d.tag_id // 6]) * .04576
        np.testing.assert_array_equal(np.sign(offsets), [[1, -1], [-1, -1], [-1, 1], [1, 1]])


def test_duplicate_ids_fail_closed():
    _, detected = observations()
    ds = [SimpleNamespace(tag_id=0, corners=detected[0])] * 2
    with pytest.raises(ValueError, match='duplicate'):
        diagnostic.valid_detections(ds)


def test_fixed_window_recovers_corners_on_legacy_black_junction_grid():
    gray = np.full((720, 1280), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    truth = {}
    for row in range(6):
        for col in range(6):
            tag = row*6+col
            x, y, size, gap = 200+col*65, 80+(5-row)*65, 50, 15
            gray[y:y+size, x:x+size] = np.rot90(cv2.aruco.generateImageMarker(dictionary, tag, size, borderBits=2), 2)
            # Actual board has filled inter-tag junction squares, not isolated
            # tiny corner dots. This is different from a plain AprilTag grid.
            for xx, yy in [(x-gap,y-gap),(x+size,y-gap),(x-gap,y+size),(x+size,y+size)]:
                gray[yy:yy+gap, xx:xx+gap] = 0
            truth[tag] = np.array([[x+size-.5,y+size-.5],[x-.5,y+size-.5],[x-.5,y-.5],[x+size-.5,y-.5]])
    gray = cv2.GaussianBlur(gray, (5,5), .8)
    detected = diagnostic.valid_detections(detect_grid(make_detector(), gray))
    assert len(detected) == 36
    original = {i: c.copy() for i, c in detected.items()}
    fixed = diagnostic.refine_legacy_corners(gray, detected)
    assert set(fixed) == set(detected)
    raw_rms = np.sqrt(np.mean(np.concatenate([np.sum((detected[i]-truth[i])**2,axis=1) for i in detected])))
    fixed_rms = np.sqrt(np.mean(np.concatenate([np.sum((fixed[i]-truth[i])**2,axis=1) for i in fixed])))
    assert raw_rms > 2.
    assert fixed_rms < .01
    for i in detected:
        np.testing.assert_array_equal(detected[i], original[i])
