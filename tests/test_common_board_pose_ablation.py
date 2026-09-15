import numpy as np
import copy
import pytest
from scripts import common_board_calibration as cb
from scripts.common_board_pose_ablation import analyze, pixel_pose
from scripts.common_board_static_analysis import pose_options
from test_common_board_calibration import scene
from test_factory_camera_model import factory, load


def test_reference_cannot_drop_duplicate_reorder_or_change_source():
    from scripts.common_board_pose_ablation import validate_reference_frames
    rows = [dict(accepted=True, attempt=2, slot=0, pairs=[{'ego': 'a'}, {'ego': 'b'}])]
    good = dict(frames=[dict(attempt=2, slot=0, pair=i, source=p) for i,p in enumerate(rows[0]['pairs'])])
    validate_reference_frames(good, rows)
    for frames in (good['frames'][:1], good['frames'][::-1], good['frames']*2):
        with pytest.raises(ValueError): validate_reference_frames(dict(frames=frames), rows)
    bad = copy.deepcopy(good); bad['frames'][0]['source']['ego'] = 'c'
    with pytest.raises(ValueError): validate_reference_frames(bad, rows)


def test_reference_mount_must_match_current_observation():
    from scripts.common_board_pose_ablation import observed_ego_mount
    cal, x, _ = scene()
    pose = cb.unpack(np.array([3., .1, .01, .05, 0, .6]))
    pix = cb.project(cb.MOUNT_OBJECTS, pose, cal['ego'])
    candidates = sorted((c for c in cb._solve_ippe_candidates(pix, calibration=cal['ego'], tag_size_m=.04)
                         if c.positive_depth), key=lambda c:c.reprojection_error_px)
    observed = candidates[0].camera_from_tag
    ref = dict(T_Ecolor_Ucolor=x.tolist(), mount=dict(T_Ucolor_mount=(np.linalg.inv(x)@observed).tolist()))
    np.testing.assert_allclose(observed_ego_mount(pix, cal['ego'], ref), observed)
    bad = copy.deepcopy(ref); bad['mount']['T_Ucolor_mount'][0][3] += .01
    with pytest.raises(AssertionError): observed_ego_mount(pix, cal['ego'], bad)


def test_synthetic_regional_chain_recovers_same_mount():
    cal, x, windows = scene()
    w = windows[2]
    m = cb.unpack(np.array([.1, -.1, .02, -.1, 0, .55]))
    result = analyze(w['pixels'], w['objects'], cal, x@m,
                     lambda o,p,c: pose_options(o,p,c)[0]['pose'])
    for g in result['groups'].values():
        np.testing.assert_allclose(g['mount']['T_Ucolor_mount'], m, atol=1e-5)
    assert max(c['translation_mm'] for c in result['contrasts'].values()) < .01


def test_pixel_objective_preserves_factory_and_cannot_worsen_sse():
    cal = load(factory())
    obj = np.concatenate([cb.object_corners(i) for i in range(36)])
    truth = cb.unpack(np.array([.2, -.1, .03, -.12, -.11, .6]))
    pix = cb.project(obj, truth, cal)
    noisy = pix + np.random.default_rng(20260906).normal(0, .15, pix.shape)
    before = pose_options(obj, noisy, cal)[0]['score']['rms_px']
    k, d = cal.camera_matrix.copy(), cal.distortion_coefficients.copy()
    fitted = pixel_pose(obj, noisy, cal)
    assert cb.errors(cb.project(obj, fitted, cal), noisy)['rms_px'] <= before
    exact = pixel_pose(obj, pix, cal)
    np.testing.assert_allclose(exact, truth, atol=1e-5)
    np.testing.assert_array_equal(cal.camera_matrix, k)
    np.testing.assert_array_equal(cal.distortion_coefficients, d)
