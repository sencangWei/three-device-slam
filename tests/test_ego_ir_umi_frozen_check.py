import copy
import numpy as np
import pytest
from scripts import common_board_calibration as cb
from scripts.ego_ir_umi_frozen_check import fixed_score, same_geometry, validate_gates
from test_factory_camera_model import factory, load


def test_frozen_prediction_does_not_fit_to_new_pixels():
    cal=load(factory('opencv_brown_conrady'))
    xyz=np.random.default_rng(94).uniform([-.1,-.1,.5],[.1,.1,.9],(100,3))
    frozen=cb.unpack(np.array([.02,-.01,.005,.01,.002,-.03]))
    prior=frozen.copy();pixels=cb.project(xyz,frozen,cal)
    assert fixed_score(xyz,pixels,cal,frozen)['p95_px']<1e-10
    assert fixed_score(xyz,pixels+[12.,0.],cal,frozen)['p95_px']==pytest.approx(12.)
    np.testing.assert_array_equal(frozen,prior)


@pytest.mark.parametrize('key',['schema','serials','intrinsics','right_from_left','native_fps'])
def test_cross_capture_geometry_change_rejected(key):
    old=dict(schema='v1',serials={'ego':'ego','umi':'umi'},intrinsics={'ir':{'fx':600}},right_from_left={'x':-.05},native_fps=30)
    new=copy.deepcopy(old)
    same_geometry(old,new)
    new[key]='changed'
    with pytest.raises(ValueError,match='camera geometry changed'):same_geometry(old,new)


def test_nonrigid_frozen_transform_rejected():
    cal=load(factory('opencv_brown_conrady'))
    bad=np.eye(4);bad[0,0]=2
    with pytest.raises(ValueError):fixed_score(np.array([[0.,0.,1.]]),np.zeros((1,2)),cal,bad)


def test_frozen_gate_mismatch_rejected():
    config=dict(projection_p95_gate_px=1.,mount_static_drift_px=.75)
    validate_gates(config)
    for key in config:
        wrong=dict(config);wrong[key]=999
        with pytest.raises(ValueError,match='frozen gates'):validate_gates(wrong)
