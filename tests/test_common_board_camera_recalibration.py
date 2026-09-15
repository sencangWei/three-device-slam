import copy
from dataclasses import replace
import numpy as np
import pytest
from scripts import common_board_calibration as cb
from test_common_board_calibration import scene


def test_ego_intrinsics_training_only_recovers_known_camera():
    from scripts.common_board_camera_recalibration import fit_camera
    cal, _, windows = scene()
    truth = replace(cal['ego'], distortion_model='opencv_brown_conrady',
                    distortion_coefficients=np.array([.05, 0, 0, 0, 0]))
    for w in windows:
        pose = cb.fit_pose(w['objects'], w['pixels']['ego'], cal['ego'])
        w['pixels']['ego'] = cb.project(w['objects'], pose, truth)
    fitted, metadata = fit_camera(windows, cal['ego'], 'K4_k1')
    np.testing.assert_allclose(fitted.camera_matrix, truth.camera_matrix, atol=.02)
    np.testing.assert_allclose(fitted.distortion_coefficients, truth.distortion_coefficients, atol=1e-4)
    bad = copy.deepcopy(windows)
    for w in bad:
        if w['split']=='holdout': w['pixels']['ego'] += 20
    other, _ = fit_camera(bad, cal['ego'], 'K4_k1')
    np.testing.assert_array_equal(other.camera_matrix, fitted.camera_matrix)
    np.testing.assert_array_equal(other.distortion_coefficients, fitted.distortion_coefficients)
    assert metadata['training_slots'] == [w['slot'] for w in windows if w['split']=='train']
    with pytest.raises(ValueError): fit_camera(windows, cal['ego'], 'unknown')


def test_new_mount_uses_candidate_camera_not_factory_pose():
    from scripts.common_board_camera_recalibration import mount_pose
    cal, _, _ = scene()
    truth = cb.unpack(np.array([3., .1, .02, .15, .12, .6]))
    modified = replace(cal['ego'], distortion_model='opencv_brown_conrady',
                       distortion_coefficients=np.array([.1, 0, 0, 0, 0]))
    pix = cb.project(cb.MOUNT_OBJECTS, truth, modified)
    np.testing.assert_allclose(mount_pose(pix, modified), truth, atol=1e-5)
    assert cb.difference(mount_pose(pix, cal['ego']), truth)[0] > .1
