import copy

import numpy as np
import pytest

from scripts import common_board_calibration as cb
from three_device_slam.spatial.apriltag_detector import CameraCalibration


def camera():
    return CameraCalibration('synthetic', 'color', 1280, 720,
                             np.array([[900., 0, 640], [0, 905., 360], [0, 0, 1]]),
                             np.zeros(5), 'distortion.none')


def scene():
    cal = {r: camera() for r in cb.ROLES}
    x = cb.unpack(np.array([.02, -.09, .01, .15, .025, .02]))
    obj = np.concatenate([cb.object_corners(i) for i in range(36)])
    windows = []
    for i, (_, split) in enumerate(cb.BOARD_PLAN):
        b = cb.unpack(np.array([-.25 + .12*(i % 4), -.24 + .2*(i % 3), .03,
                                -.15 + .035*(i % 4), -.14, .7 + .06*(i % 3)]))
        windows.append(dict(slot=i, split=split, objects=obj,
                            pixels={'left': cb.project(obj, b, cal['left']),
                                    'ego': cb.project(obj, x @ b, cal['ego'])}))
    return cal, x, windows


def test_fixed_factory_joint_x_and_bidirectional_holdout():
    cal, truth, windows = scene()
    before = {r: c.camera_matrix.copy() for r, c in cal.items()}
    report = cb.solve_board(windows, cal)
    assert report['status'] == 'PASS_BOARD_ONLY'
    assert np.allclose(report['T_Ecolor_Ucolor'], truth, atol=2e-6)
    assert len(report['holdout']) == 8
    assert max(v['p95_px'] for v in report['holdout']) < .001
    for r in cal:
        assert np.array_equal(before[r], cal[r].camera_matrix)


def test_right_role_solver_preserves_role_and_geometry():
    cal, truth, windows = scene()
    cal['right'] = cal.pop('left')
    for window in windows:
        window['pixels']['right'] = window['pixels'].pop('left')
    report = cb.solve_board(windows, cal)
    assert report['status'] == 'PASS_BOARD_ONLY'
    assert report['umi_role'] == 'right'
    assert {row['role'] for row in report['training']} == {'ego', 'right'}
    assert {(row['source'], row['destination']) for row in report['holdout']} == {
        ('right', 'ego'), ('ego', 'right')}
    np.testing.assert_allclose(report['T_Ecolor_Ucolor'], truth, atol=2e-6)


def test_holdout_fault_does_not_change_training_x():
    cal, _, windows = scene()
    good = cb.solve_board(windows, cal)
    bad = copy.deepcopy(windows)
    bad[2]['pixels']['ego'] += [8., 0.]
    report = cb.solve_board(bad, cal)
    assert report['status'] == 'REVIEW'
    assert np.allclose(report['T_Ecolor_Ucolor'], good['T_Ecolor_Ucolor'], atol=1e-10)
    assert max(v['p95_px'] for v in report['holdout']) > 5


def test_duplicate_pose_windows_and_nonfinite_rejected():
    cal, _, windows = scene()
    windows[1]['slot'] = windows[0]['slot']
    with pytest.raises(ValueError, match='slots'):
        cb.solve_board(windows, cal)
    cal, _, windows = scene()
    windows[0]['pixels']['left'][0, 0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        cb.solve_board(windows, cal)


def test_no_diversity_cannot_pass():
    cal, _, windows = scene()
    for w in windows:
        w['pixels'] = copy.deepcopy(windows[0]['pixels'])
    report = cb.solve_board(windows, cal)
    assert report['status'] == 'REVIEW'
    assert not report['checks']['board_diversity']


def test_repeated_training_pose_is_not_independent_holdout():
    cal, _, windows = scene()
    for w in windows:
        if w['split'] == 'holdout':
            w['pixels'] = copy.deepcopy(windows[0]['pixels'])
    report = cb.solve_board(windows, cal)
    assert report['status'] == 'REVIEW'
    assert not report['checks']['holdout_pose_novelty']


def test_repeated_novel_holdout_is_not_four_independent_poses():
    cal, _, windows = scene()
    for w in windows:
        if w['split'] == 'holdout':
            w['pixels'] = copy.deepcopy(windows[2]['pixels'])
    report = cb.solve_board(windows, cal)
    assert report['status'] == 'REVIEW'
    assert not report['checks']['holdout_pose_novelty']


def test_single_camera_motion_cannot_pass():
    cal, _, windows = scene()
    windows[3]['pixels']['ego'] += [10., 0.]
    report = cb.solve_board(windows, cal)
    assert report['status'] == 'REVIEW'


def test_after_mount_board_closure_is_held_out_and_catches_camera_shift():
    cal, _, windows = scene()
    original = cb.solve_board(windows, cal)
    closure = copy.deepcopy(windows[2])
    closure.update(slot=15, split='holdout')
    closure['pixels']['ego'] += [5., 0.]
    report = cb.solve_board(windows + [closure], cal)
    assert report['status'] == 'REVIEW'
    np.testing.assert_array_equal(report['T_Ecolor_Ucolor'], original['T_Ecolor_Ucolor'])
    assert any(w['slot'] == 15 and w['p95_px'] > 3 for w in report['holdout'])


def test_sdk_inverse_brown_joint_solver_no_hardware_opened():
    from test_factory_camera_model import factory, load
    pytest.importorskip('pyrealsense2')
    calibrations, truth, windows = scene()
    for r in cb.ROLES:
        old = calibrations[r]
        new = load(factory())
        for w in windows:
            rays = cb.normalized_camera_points(w['pixels'][r], old)
            w['pixels'][r] = cb.project_camera_points(np.c_[rays, np.ones(len(rays))], new)
        calibrations[r] = new
    result = cb.solve_board(windows, calibrations)
    assert result['status'] == 'PASS_BOARD_ONLY'
    assert cb.difference(truth, np.asarray(result['T_Ecolor_Ucolor']))[0] < .02
    assert max(w['p95_px'] for w in result['holdout']) < .01


def test_finite_noise_and_heldout_error_statistics():
    cal, truth, windows = scene()
    rng = np.random.default_rng(726)
    for w in windows:
        for r in cb.ROLES:
            w['pixels'][r] += rng.normal(0, .1, w['pixels'][r].shape)
    result = cb.solve_board(windows, cal)
    assert result['status'] == 'PASS_BOARD_ONLY'
    assert cb.difference(truth, np.asarray(result['T_Ecolor_Ucolor']))[0] < 1.
    assert max(w['p95_px'] for w in result['holdout']) < .5


def test_mount_training_and_independent_prediction():
    cal = camera()
    x = cb.unpack(np.array([.03, -.1, .02, .12, .01, .03]))
    # Tilted tag so IPPE branches are separated; camera sees tag in front.
    e_mount = cb.unpack(np.array([2.7, .15, .03, .015, -.02, .55]))
    pixels = [cb.project(cb.MOUNT_OBJECTS, e_mount, cal)] * 3
    trained = cb.solve_mount(pixels, cal, x)
    assert trained['status'] == 'CANDIDATE_NEEDS_INDEPENDENT_SETUP'
    expected = np.linalg.inv(x) @ e_mount
    assert np.allclose(trained['T_Ucolor_mount'], expected, atol=1e-5)
    x2 = cb.unpack(np.array([.02, .15, -.03, -.1, .01, .06]))
    other = [cb.project(cb.MOUNT_OBJECTS, x2 @ expected, cal)] * 3
    check = cb.solve_mount(other, cal, x2, reference=expected)
    assert check['status'] == 'PASS_INDEPENDENT_MOUNT'
    shifted = expected.copy()
    shifted[0, 3] += .015
    fail = cb.solve_mount(other, cal, x2, reference=shifted)
    assert fail['status'] == 'REVIEW'
    assert fail['repeatability_translation_mm'] > 10


def test_window_gate_uses_whole_window_not_adjacent_drift():
    base = {r: {i: np.array([[10., 10.], [30., 10.], [30., 30.], [10., 30.]])
                for i in range(36)} for r in cb.ROLES}
    samples = []
    for i in range(4):
        ds = copy.deepcopy(base)
        for role in ds:
            for tag in ds[role]:
                ds[role][tag] += [i*.4, 0]
        samples.append(dict(t=i*.4, arrival_gap_ms=10., detected=ds, reasons=[]))
    assert 'window_motion' in cb.window_reasons(samples, 'board')
    samples[0]['arrival_gap_ms'] = 100.
    assert 'arrival_gap' in cb.window_reasons(samples, 'board')


def test_parse_output_must_be_new_inside_artifacts(tmp_path):
    with pytest.raises(ValueError):
        cb.new_output(cb.ROOT / 'outside_artifacts')
    with pytest.raises(ValueError):
        cb.new_output(tmp_path)


def test_cli_reference_binding_freezes_M_and_rejects_invalid_reference(tmp_path, monkeypatch):
    import json
    cal, x, windows = scene()
    closure = copy.deepcopy(windows[2])
    closure['slot'] = 15
    windows.append(closure)
    e_mount = cb.unpack(np.array([2.7, .15, .03, .015, -.02, .55]))
    m = np.linalg.inv(x) @ e_mount
    setup = dict(devices={r: dict(serial=r, intrinsics='synthetic only') for r in cb.ROLES})
    hashes = {'setup.json': 'first-setup', 'attempts.jsonl': 'index', 'capture_report.json': 'report'}
    monkeypatch.setattr(cb, 'load_session', lambda _: (setup, cal, windows,
                        [cb.project(cb.MOUNT_OBJECTS, e_mount, cal['ego'])]*3, hashes, []))
    first = tmp_path/'first'
    assert cb.main(['--session', str(tmp_path/'raw1'), '--output', str(first)]) == 0
    reference = first/'report.json'
    training = json.loads(reference.read_text())
    assert training['status'] == 'CANDIDATE_NEEDS_INDEPENDENT_SETUP'
    second_x = cb.unpack(np.array([.02, .15, -.03, -.1, .01, .06]))
    second_windows = copy.deepcopy(windows)
    for w in second_windows:
        b = cb.fit_pose(w['objects'], w['pixels']['left'], cal['left'])
        w['pixels']['ego'] = cb.project(w['objects'], second_x @ b, cal['ego'])
    second_hashes = dict(hashes, **{'setup.json': 'independent-setup'})
    monkeypatch.setattr(cb, 'load_session', lambda _: (setup, cal, second_windows,
                        [cb.project(cb.MOUNT_OBJECTS, second_x @ m, cal['ego'])]*3, second_hashes, []))
    second = tmp_path/'second'
    assert cb.main(['--session', str(tmp_path/'raw2'), '--output', str(second), '--reference', str(reference)]) == 0
    checked = json.loads((second/'report.json').read_text())
    assert checked['status'] == 'PASS_INDEPENDENT_MOUNT'
    assert checked['reference']['sha256'] == cb.digest(reference)
    np.testing.assert_array_equal(checked['mount']['fixed_reference_M'], training['mount']['T_Ucolor_mount'])
    training['board']['status'] = 'REVIEW'
    cb.write_json(reference, training)
    with pytest.raises(ValueError, match='passing candidate'):
        cb.main(['--session', str(tmp_path/'raw2'), '--output', str(tmp_path/'bad'), '--reference', str(reference)])
    assert not (tmp_path/'bad').exists()
