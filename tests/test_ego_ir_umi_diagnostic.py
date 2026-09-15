import numpy as np
import pytest
from scripts import common_board_calibration as cb
from scripts.ego_ir_umi_diagnostic import triangulate, fit_group, fit_observed_group, GROUPS, source
from test_factory_camera_model import factory, load


def synthetic():
    cal=load(factory('opencv_brown_conrady'))
    obj=np.concatenate([cb.object_corners(i) for i in range(36)])
    points=cb.transform_points(obj,cb.unpack(np.array([.2,.1,.02,-.1,-.1,.8])))
    truth=cb.unpack(np.array([.03,-.04,.01,.02,.01,-.1]))
    mount=np.array([[-.02,-.02,.4],[.02,-.02,.4],[.02,.02,.4],[-.02,.02,.4]])
    return cal,points,truth,mount


def test_triangulation_recovers_metric_points_and_direction():
    cal,points,_,_=synthetic()
    right=np.eye(4);right[0,3]=-.05
    xyz,scores=triangulate(cb.project(points,np.eye(4),cal),cb.project(points,right,cal),cal,cal,right)
    np.testing.assert_allclose(xyz,points,atol=1e-8)
    assert max(s['p95_px'] for s in scores.values())<1e-6
    with pytest.raises(ValueError):
        triangulate(cb.project(points,np.eye(4),cal),cb.project(points,np.eye(4),cal),cal,cal,right)


def test_direct_group_truth_and_no_heldout_or_mount_fit_leakage():
    cal,points,truth,mount=synthetic()
    pixels=cb.project(points,truth,cal)
    ids=GROUPS['horizontal_0']
    base=fit_group(points,pixels,mount,cal,np.eye(4),ids)
    np.testing.assert_allclose(base['T_Ucolor_Eir'],truth,atol=1e-6)
    expected=cb.transform_points(mount.mean(0)[None],truth)[0]*1000
    np.testing.assert_allclose(base['mount_center_Ucolor_mm'],expected,atol=1e-4)
    poisoned=pixels.copy();poisoned[72:]+=10
    altered=fit_group(points,poisoned,mount+np.array([0,0,.1]),cal,np.eye(4),ids)
    np.testing.assert_array_equal(base['T_Ucolor_Eir'],altered['T_Ucolor_Eir'])
    assert altered['heldout']['p95_px']>10
    assert not np.allclose(base['mount_center_Ucolor_mm'],altered['mount_center_Ucolor_mm'])


def test_frozen_groups_cover_all36_disjoint_halves():
    assert len(GROUPS)==7
    for name in ('horizontal','vertical','checker'):
        a,b=set(GROUPS[name+'_0']),set(GROUPS[name+'_1'])
        assert len(a)==len(b)==18 and not a&b and a|b==set(range(36))


def test_group_initialization_is_also_independent_of_heldout():
    cal,points,truth,mount=synthetic()
    pixels=cb.project(points,truth,cal)
    ir=cb.project(points,np.eye(4),cal)
    ids=GROUPS['horizontal_0']
    base=fit_observed_group(points,pixels,ir,mount,cal,cal,ids)
    altered=pixels.copy();altered[72:]+=50
    changed=ir.copy();changed[72:]-=20
    poison=fit_observed_group(points,altered,changed,mount,cal,cal,ids)
    np.testing.assert_array_equal(base['T_Ucolor_Eir'],poison['T_Ucolor_Eir'])
    assert poison['heldout']['p95_px']>50


def test_source_tamper_fails_before_image_decode(tmp_path):
    setup=dict(schema='ego.ir_umi_capture.v1',serials=dict(ego='327122078613',umi='260322279785'),
               factory_values_modified=False,activation='NOT_ACTIVATED',native_fps=30,
               sensor_settings={'ego':{'Emitter Enabled':0}})
    cb.write_json(tmp_path/'setup.json',setup)
    cb.write_json(tmp_path/'capture_report.json',dict(status='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION',
        failure=None,cleanup_errors=[],hashes={'setup.json':'wrong'}))
    with pytest.raises(ValueError,match='source hash mismatch'):source(tmp_path)


def test_joint_rays_recover_truth_without_modifying_intrinsics():
    from scripts.ego_ir_umi_bundle_diagnostic import bundle_fit
    cal,points,truth,_=synthetic()
    right=np.eye(4);right[0,3]=-.05
    points=points[::4]
    pixels={k:cb.project(points,t,cal) for k,t in (
        ('ir_left',np.eye(4)),('ir_right',right),('umi_color',truth))}
    original=cal.camera_matrix.copy()
    seed=truth.copy();seed[:3,3]+=[.002,-.001,.001]
    noisy=points+np.random.default_rng(93).normal(0,.0001,points.shape)
    result=bundle_fit(pixels,{k:cal for k in pixels},right,seed,noisy)
    assert result['optimizer']['success']
    np.testing.assert_allclose(result['T_Ucolor_Eir'],truth,atol=1e-7)
    np.testing.assert_array_equal(cal.camera_matrix,original)
