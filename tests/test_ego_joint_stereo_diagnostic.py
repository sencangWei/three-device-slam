import numpy as np
import copy
import pytest
import json
from scripts import common_board_calibration as cb
from test_factory_camera_model import factory, load


def test_missing_drift_is_null_not_unserializable_infinity():
    from scripts.ego_joint_stereo_diagnostic import drift_summary
    result=drift_summary([dict(drift_px={'color':{'board':.2,'mount':.3}})])
    assert result['ir_left']['board'] is None
    assert result['color']['board']==.2
    json.dumps(result,allow_nan=False)


def test_diagnostic_small_mount_is_retained_but_fails_size_not_identity():
    from scripts.ego_joint_stereo_diagnostic import diagnostic_targets
    from test_common_board_joint import scene
    board,mount=scene()
    small=copy.deepcopy(mount)
    center=small.corners.mean(axis=0)
    small.corners[:]=center+(small.corners-center)*.4
    ds,q,proof=diagnostic_targets(board,[small])
    assert len(ds)==35 and not proof['mount_size_pass']
    np.testing.assert_array_equal(q,small.corners)
    with pytest.raises(ValueError): diagnostic_targets(board,[small,small])
    with pytest.raises(ValueError): diagnostic_targets(board+[mount],[small])


def test_ir_to_color_fit_recovers_nonplanar_truth_without_changing_k():
    from scripts.ego_joint_stereo_diagnostic import fit_extrinsic
    cal = load(factory())
    points = np.random.default_rng(71).uniform([-.15,-.1,.4],[.15,.15,.9],(100,3))
    true = cb.unpack(np.array([.01,-.02,.005,.015,-.001,.002]))
    pix = cb.project(points,true,cal)
    k,d = cal.camera_matrix.copy(),cal.distortion_coefficients.copy()
    result = fit_extrinsic(points,pix,cal,np.eye(4))
    np.testing.assert_allclose(result,true,atol=2e-5)
    np.testing.assert_array_equal(k,cal.camera_matrix)
    np.testing.assert_array_equal(d,cal.distortion_coefficients)


def test_board_shape_diagnostic_recovers_scale_and_shear():
    from scripts.ego_joint_stereo_diagnostic import board_shape
    ids=list(range(36));o=np.concatenate([cb.object_corners(i) for i in ids])
    x=cb.unpack(np.array([.1,.2,.3,.01,.02,.6]))
    result=board_shape(o,cb.transform_points(o,x))
    assert abs(result['axis_scale_x']-1)<1e-10
    assert abs(result['axis_scale_y']-1)<1e-10
    assert abs(result['axis_angle_deg']-90)<1e-8
    warped=o.copy();warped[:,0]*=1.01;warped[:,1]*=.99
    result=board_shape(o,cb.transform_points(warped,x))
    assert abs(result['axis_scale_x']-1.01)<1e-10
    assert abs(result['axis_scale_y']-.99)<1e-10
