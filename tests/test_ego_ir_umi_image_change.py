import numpy as np
import pytest
from scripts import common_board_calibration as cb
from scripts.ego_ir_umi_image_change import flow, patch_shift, epipolar_errors
from test_factory_camera_model import load, factory


def test_raw_template_shift_independent_of_detector_and_brightness():
    rng=np.random.default_rng(95)
    a=rng.integers(30,180,(200,240),dtype=np.uint8)
    b=np.zeros_like(a);b[3:,7:]=(a[:-3,:-7].astype(float)*1.1+10).astype(np.uint8)
    result=patch_shift(a,b,(60,60,130,130),radius=15)
    assert result['status']=='DIAGNOSTIC' and result['ncc']>.99
    np.testing.assert_allclose(result['delta_px'],[7,3],atol=.03)


def test_textureless_region_is_unknown_not_stable():
    blank=np.zeros((200,240),dtype=np.uint8)
    assert flow(blank,blank,(20,20,100,100))['median_delta_px'] is None
    assert patch_shift(blank,blank,(50,50,100,100))['status']=='NO_TEXTURE'


def test_epipolar_constraint_independent_of_depth_along_ray():
    cal=load(factory('opencv_brown_conrady'))
    rng=np.random.default_rng(96);points=rng.uniform([-.1,-.1,.4],[.1,.1,.9],(100,3))
    pose=cb.unpack(np.array([.01,-.03,.02,.1,0.,.01]))
    ir=cb.project(points,np.eye(4),cal)
    changed_depth=points*rng.uniform(.7,1.3,(100,1))
    umi=cb.project(changed_depth,pose,cal)
    assert epipolar_errors(ir,umi,cal,cal,pose)['p95_undistorted_px']<1e-5
    assert epipolar_errors(ir,umi+[0.,8.],cal,cal,pose)['p95_undistorted_px']>7
    with pytest.raises(ValueError,match='degenerate'):epipolar_errors(ir,umi,cal,cal,np.eye(4))
