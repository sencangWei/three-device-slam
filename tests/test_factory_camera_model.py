from dataclasses import replace

import numpy as np
import pytest

from three_device_slam.spatial import apriltag_detector as detector
from three_device_slam.spatial.pair_tag_alignment import _camera_calibration_from_intrinsics


def factory(model='distortion.inverse_brown_conrady'):
    return dict(width=1280,height=720,fx=656.2879,fy=654.4534,ppx=641.5363,ppy=354.3078,
                coefficients=[-.0530364,.0568782,.0001357,.0001059,-.0190484],distortion_model=model)


def load(data):
    return _camera_calibration_from_intrinsics(calibration_id='factory',frame_id='left.color',intrinsics=data)


def test_factory_model_and_coefficients_are_preserved():
    data=factory();cal=load(data)
    assert cal.distortion_model==data['distortion_model']
    np.testing.assert_array_equal(cal.distortion_coefficients,data['coefficients'])
    assert not cal.distortion_coefficients.flags.writeable


@pytest.mark.parametrize('model',['distortion.unknown','distortion.kannala_brandt4','distortion.none'])
def test_unsupported_or_inconsistent_model_is_rejected(model):
    with pytest.raises(ValueError,match='distortion'):load(factory(model))


def test_sdk_rays_and_projection_match_native_without_opening_hardware():
    rs=pytest.importorskip('pyrealsense2')
    data=factory();cal=load(data);intr=rs.intrinsics()
    for name in ('width','height','fx','fy','ppx','ppy'):setattr(intr,name,data[name])
    intr.coeffs=data['coefficients'];intr.model=rs.distortion.inverse_brown_conrady
    pixels=np.array([[0.,0.],[1279.,719.],[100.,400.],[640.,360.]])
    expected=np.array([rs.rs2_deproject_pixel_to_point(intr,p.tolist(),1)[:2] for p in pixels])
    np.testing.assert_array_equal(detector.normalized_camera_points(pixels,cal),expected)
    points=np.column_stack([expected,np.ones(len(expected))])
    projected=np.array([rs.rs2_project_point_to_pixel(intr,p.tolist()) for p in points])
    np.testing.assert_array_equal(detector.project_camera_points(points,cal),projected)
    legacy=replace(cal,distortion_model='opencv_brown_conrady')
    assert np.max(np.abs(detector.normalized_camera_points(pixels,legacy)-expected))>1e-6


def test_ippe_reprojection_uses_sdk_model():
    rs=pytest.importorskip('pyrealsense2')
    cal=load(factory());h=.04
    obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    import cv2
    rotation=cv2.Rodrigues(np.array([2.9,.2,-.1]))[0]
    points=(rotation@obj.T).T+np.array([.12,.02,.5])
    corners=detector.project_camera_points(points,cal)
    candidates=detector._solve_ippe_candidates(corners,calibration=cal,tag_size_m=.08)
    best=min(candidates,key=lambda c:c.reprojection_error_px)
    np.testing.assert_allclose(best.camera_from_tag[:3,3],[.12,.02,.5],atol=2e-6)
    assert best.reprojection_error_px<.001


def test_legacy_missing_model_keeps_explicit_opencv_default():
    data=factory();data.pop('distortion_model')
    assert load(data).distortion_model=='opencv_brown_conrady'


@pytest.mark.parametrize('script',['live_mixed_placement_preview.py','live_placement_preview.py'])
def test_actual_preview_constructor_preserves_sdk_model_without_running_preview(script):
    import ast
    from pathlib import Path
    from types import SimpleNamespace
    path=Path(__file__).resolve().parents[1]/'scripts'/script
    tree=ast.parse(path.read_text())
    calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call) and
           isinstance(node.func,ast.Name) and node.func.id=='CameraCalibration']
    assert len(calls)==1
    data=factory()
    intr=SimpleNamespace(**{k:v for k,v in data.items() if k not in ('coefficients','distortion_model')},
                         coeffs=data['coefficients'],model=data['distortion_model'])
    # Evaluate only the construction expression, never top-level camera startup.
    code=compile(ast.Expression(calls[0]),str(path),'eval')
    namespace=dict(CameraCalibration=detector.CameraCalibration,np=np,intr=intr,serial='test',role='umi',title='UMI')
    assert eval(code,namespace).distortion_model=='distortion.inverse_brown_conrady'
    intr.model='distortion.unknown'
    with pytest.raises(ValueError,match='distortion'):eval(code,namespace)
