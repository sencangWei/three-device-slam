import copy
from dataclasses import replace
import numpy as np
import pytest
from scripts import common_board_calibration as cb
from scripts.ego_multidepth_camera_diagnostic import fit_multidepth,HOLDOUT
from test_common_board_calibration import camera


def test_geometry_signature_rejects_device_or_factory_change():
    from scripts.ego_multidepth_camera_diagnostic import geometry_signature
    from types import SimpleNamespace
    from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics
    snapshot=serialize_sdk_extrinsics(SimpleNamespace(rotation=np.eye(3).ravel().tolist(),translation=[.05,0,0]))
    setup=dict(schema='ego.stereo_color_probe.v1',serial='327122078613',activation='NOT_ACTIVATED',
        factory_values_modified=False,intrinsics={r:dict(fx=900) for r in ('color','ir_left','ir_right')},
        extrinsics={r:copy.deepcopy(snapshot) for r in ('color','ir_right')})
    signature=geometry_signature(setup)
    for field,value in [('serial','other'),('activation','ACTIVE'),('factory_values_modified',True),('schema','unknown')]:
        bad=copy.deepcopy(setup);bad[field]=value
        with pytest.raises(ValueError):geometry_signature(bad)
    bad=copy.deepcopy(setup);bad['intrinsics']['ir_left']['fx']+=1
    assert geometry_signature(bad)!=signature


def test_multidepth_recovers_model_and_excludes_holdouts_and_mounts():
    factory=camera();k=factory.camera_matrix.copy();k[0,0]*=.99;k[1,1]*=.99
    truth=replace(factory,camera_matrix=k,distortion_coefficients=np.array([.045,0,0,0,0]),distortion_model='opencv_brown_conrady')
    pose=cb.unpack(np.array([.01,-.02,.005,.015,0,.01]));rng=np.random.default_rng(20260906)
    data={}
    for n in range(1,7):
        xyz=rng.uniform([-.15,-.15,.35+.05*n],[.15,.15,.40+.05*n],(100,3))
        data[n]=dict(frames=[dict(board_points=xyz,board_pixels=cb.project(xyz,pose,truth),mount_pixels=[[1,2]])])
    cal,t,fit=fit_multidepth(data,factory)
    np.testing.assert_allclose(cal.camera_matrix,k,atol=.01)
    np.testing.assert_allclose(cal.distortion_coefficients,truth.distortion_coefficients,atol=1e-4)
    np.testing.assert_allclose(t,pose,atol=1e-5)
    altered=copy.deepcopy(data)
    for n in HOLDOUT:altered[n]['frames'][0]['board_pixels']+=100
    for n in altered:altered[n]['frames'][0]['mount_pixels']=[[999,999]]
    other,ot,_=fit_multidepth(altered,factory)
    np.testing.assert_array_equal(other.camera_matrix,cal.camera_matrix)
    np.testing.assert_array_equal(ot,t)
    assert fit['training_points']==400
