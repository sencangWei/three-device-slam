import numpy as np
from dataclasses import replace

from scripts.ego_stereo_color_validate import stereo_projection, transform
from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics
from types import SimpleNamespace
import pytest
from scripts import common_board_calibration as cb
from test_common_board_calibration import camera


def test_stereo_projection_is_metric_without_planar_target_or_size():
    cal = {r: camera() for r in ('color', 'ir_left', 'ir_right')}
    right = np.eye(4)
    right[0, 3] = -.05
    color = cb.unpack(np.array([.02, .01, -.01, .014, .003, -.002]))
    rng = np.random.default_rng(503)
    points = rng.uniform([-.2, -.1, .5], [.2, .1, 1.1], (64, 3))
    pixels = {r: cb.project(points, t, cal[r]) for r, t in
              (('ir_left', np.eye(4)), ('ir_right', right), ('color', color))}
    result = stereo_projection(pixels, cal, right, color)
    np.testing.assert_allclose(result['points_ir_left_m'], points, atol=1e-10)
    assert result['projections']['color']['p95_px'] < 1e-8
    k = cal['color'].camera_matrix.copy()
    k[0, 0] *= 1.02
    wrong = dict(cal, color=replace(cal['color'], camera_matrix=k))
    bad = stereo_projection(pixels, wrong, right, color)
    np.testing.assert_allclose(bad['points_ir_left_m'], points, atol=1e-10)
    assert bad['projections']['color']['p95_px'] > 2.


@pytest.mark.parametrize('defect', ['reflection', 'nan_rotation', 'nan_translation', 'direction'])
def test_extrinsics_reject_invalid_or_ambiguous_snapshot(defect):
    snapshot = serialize_sdk_extrinsics(SimpleNamespace(rotation=np.eye(3).ravel().tolist(), translation=[0., 0., 0.]))
    if defect == 'reflection':
        snapshot['rotation_row_major'][0] = -1.
    elif defect == 'nan_rotation':
        snapshot['rotation_row_major'][0] = float('nan')
    elif defect == 'nan_translation':
        snapshot['translation_m'][0] = float('nan')
    else:
        snapshot['convention'] = 'target_to_source'
    with pytest.raises(ValueError):
        transform(snapshot)


def test_stereo_rejects_point_at_infinity():
    cal = {r: camera() for r in ('color', 'ir_left', 'ir_right')}
    right = np.eye(4)
    right[0, 3] = -.05
    pixels = {r: np.array([[640., 360.], [650., 370.]]) for r in cal}
    with pytest.raises(ValueError, match='infinity'):
        stereo_projection(pixels, cal, right, np.eye(4))


def test_sdk_local_se3_fit_uses_resolvable_jacobian():
    from scripts.analyze_ego_stereo_color_probe import fit_local_transform
    cal = replace(camera(), distortion_model='distortion.inverse_brown_conrady',
                  distortion_coefficients=np.array([.056, 0., 0., 0., 0.]))
    q = np.random.default_rng(76).uniform([-.25, -.2, .5], [.25, .2, 1.2], (80, 3))
    expected = np.array([.005, -.003, .004, .002, -.001, .003])
    observed = cb.project(q, cb.unpack(expected), cal)
    for step in (1e-5, 2e-5):
        fit = fit_local_transform(q, observed, np.eye(4), cal, step)
        assert fit.success
        np.testing.assert_allclose(fit.x, expected, atol=1e-6)
