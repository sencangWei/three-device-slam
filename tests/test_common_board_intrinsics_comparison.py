import copy

import numpy as np
import pytest

from scripts import common_board_intrinsics_comparison as diagnostic
from scripts import common_board_calibration as cb
from test_common_board_calibration import scene


def test_candidate_keeps_factory_arrays_and_distortion():
    cal, _, _ = scene()
    before = cal['ego'].camera_matrix.copy()
    new = diagnostic.changed_camera(cal['ego'], np.array([np.log(1.02)]), 'focal')
    np.testing.assert_array_equal(cal['ego'].camera_matrix, before)
    np.testing.assert_array_equal(new.distortion_coefficients, cal['ego'].distortion_coefficients)
    np.testing.assert_allclose(new.camera_matrix[0, 0], before[0, 0]*1.02)


def test_known_focal_error_recovered_without_holdout_leakage():
    cal, truth, windows = scene()
    actual = diagnostic.changed_camera(cal['ego'], np.array([np.log(1.02)]), 'focal')
    for w in windows:
        b = cb.fit_pose(w['objects'], w['pixels']['left'], cal['left'])
        w['pixels']['ego'] = cb.project(w['objects'], truth @ b, actual)
    result = diagnostic.fit_case(windows, cal, 'ego', 'focal')
    assert result['optimizer']['success']
    assert abs(result['candidate_K']['ego'][0][0]/cal['ego'].camera_matrix[0, 0]-1.02) < 1e-5
    assert max(v['p95_px'] for v in result['holdout']) < .005
    changed = copy.deepcopy(windows)
    changed[2]['pixels']['ego'] += [8., 0.]
    second = diagnostic.fit_case(changed, cal, 'ego', 'focal')
    np.testing.assert_array_equal(second['T_Ecolor_Ucolor'], result['T_Ecolor_Ucolor'])
    np.testing.assert_array_equal(second['candidate_K'], result['candidate_K'])
    assert max(v['p95_px'] for v in second['holdout']) > 5
    assert result['status'] == 'DIAGNOSTIC_ONLY'


def test_sdk_inverse_brown_K4_recovery_and_derivative_step_stability():
    from test_factory_camera_model import factory as factory_spec, load
    pytest.importorskip('pyrealsense2')
    cal, truth, windows = scene()
    sdk = load(factory_spec())
    original_left = cal['left']
    cal['left'] = sdk
    actual = diagnostic.changed_camera(sdk, np.array([np.log(1.01), np.log(.99), 3., -2.]), 'K4')
    for w in windows:
        b = cb.fit_pose(w['objects'], w['pixels']['left'], original_left)
        w['pixels']['left'] = cb.project(w['objects'], b, actual)
    snapshots = {r: c.camera_matrix.copy() for r, c in cal.items()}
    results = [diagnostic.fit_case(windows, cal, 'left', 'K4', k_derivative_scale=s) for s in (1., 2.)]
    for result in results:
        assert result['optimizer']['success']
        np.testing.assert_allclose(result['candidate_K']['left'], actual.camera_matrix, atol=.03, rtol=0)
        assert max(v['p95_px'] for v in result['holdout']) < .01
        for r in cal:
            np.testing.assert_array_equal(cal[r].camera_matrix, snapshots[r])
    np.testing.assert_allclose(results[0]['candidate_K']['left'], results[1]['candidate_K']['left'], atol=.03, rtol=0)


def test_sdk_radial_error_recovered_without_changing_K():
    from dataclasses import replace
    pytest.importorskip('pyrealsense2')
    cal, truth, windows = scene()
    cal['ego'] = replace(cal['ego'], distortion_model='distortion.inverse_brown_conrady')
    actual = diagnostic.changed_camera(cal['ego'], np.array([.08]), 'k1')
    for w in windows:
        b = cb.fit_pose(w['objects'], w['pixels']['left'], cal['left'])
        w['pixels']['ego'] = cb.project(w['objects'], truth @ b, actual)
    result = diagnostic.fit_case(windows, cal, 'ego', 'k1')
    assert abs(result['candidate_D']['ego'][0]-.08) < .001
    np.testing.assert_array_equal(result['candidate_K']['ego'], cal['ego'].camera_matrix)
    np.testing.assert_array_equal(cal['ego'].distortion_coefficients, np.zeros(5))
    assert max(v['p95_px'] for v in result['holdout']) < .02


def test_coupled_focal_radial_recovery():
    from dataclasses import replace
    pytest.importorskip('pyrealsense2')
    cal, truth, windows = scene()
    cal['ego'] = replace(cal['ego'], distortion_model='distortion.inverse_brown_conrady')
    actual = diagnostic.changed_camera(cal['ego'], np.array([np.log(1.02), .08]), 'focal_k1')
    for w in windows:
        b = cb.fit_pose(w['objects'], w['pixels']['left'], cal['left'])
        w['pixels']['ego'] = cb.project(w['objects'], truth @ b, actual)
    result = diagnostic.fit_case(windows, cal, 'ego', 'focal_k1')
    assert abs(result['candidate_D']['ego'][0]-.08) < .001
    np.testing.assert_allclose(result['candidate_K']['ego'], actual.camera_matrix, atol=.03, rtol=0)
    assert max(v['p95_px'] for v in result['holdout']) < .02
