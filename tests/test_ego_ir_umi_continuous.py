import numpy as np
import pytest
from scripts.ego_ir_umi_continuous import ContinuousAB


def test_two_static_phases_require_changed_pose_and_do_not_restart():
    gate=ContinuousAB();points=np.zeros((3,144,2))
    for now in (0.,.5,1.):assert gate.update(now,points)[0] is None
    assert gate.update(1.5,points)[0]=='A'
    assert gate.update(11.,None)[0]=='A'  # keep failures during raw recording
    assert gate.update(11.5,points)[0] is None and gate.state=='WAIT_B'
    for now in (12.,12.5,13.,13.5):assert gate.update(now,points)[0] is None
    changed=points+30
    for now in (14.,14.5,15.):assert gate.update(now,changed)[0] is None
    assert gate.update(15.5,changed)[0]=='B'
    assert gate.update(25.5,changed)[0] is None and gate.state=='DONE'
    assert gate.update(26.,points)==(None,'DONE')


def test_missing_tags_or_jitter_cannot_start_recording():
    gate=ContinuousAB();points=np.zeros((3,144,2))
    for now in (0.,.5,1.):gate.update(now,points)
    gate.update(1.5,None)
    assert gate.update(2.,points)[0] is None
    for i in range(1,12):
        jitter=points.copy();jitter[0,0,0]=i%2*2.
        assert gate.update(2.+i*.5,jitter)[0] is None


def test_b_requires_change_in_all_views_not_one():
    gate=ContinuousAB();points=np.zeros((3,144,2))
    for i in range(4):gate.update(i*.5,points)
    gate.update(11.5,points)
    changed=points.copy();changed[0]+=30
    for i in range(10):assert gate.update(12.+i*.5,changed)[0] is None


def test_offline_fit_rejects_phase_b_and_recovers_a_truth():
    from scripts.ego_ir_umi_continuous_check import fit_phase_a
    from scripts import common_board_calibration as cb
    from test_factory_camera_model import load,factory
    cal=load(factory('opencv_brown_conrady'))
    points=cb.transform_points(np.concatenate([cb.object_corners(i) for i in range(36)]),
        cb.unpack(np.array([.1,.2,.01,-.1,-.1,.8])))
    right=np.eye(4);right[0,3]=-.05
    truth=cb.unpack(np.array([.02,-.03,.01,.04,0.,-.1]))
    pixels={k:cb.project(points,t,cal) for k,t in [('ir_left',np.eye(4)),('ir_right',right),('umi_color',truth)]}
    a=dict(phase='A',pixels=pixels,mount_ir_m=points[:4])
    fit,_=fit_phase_a([a],{k:cal for k in pixels},right)
    np.testing.assert_allclose(fit['T_Ucolor_Eir'],truth,atol=1e-6)
    with pytest.raises(ValueError,match='phase A only'):fit_phase_a([dict(a,phase='B')],{k:cal for k in pixels},right)


def test_static_board_does_not_hide_moving_mount():
    from scripts.ego_ir_umi_continuous_check import check_static,fit_phase_a
    first={};points={'ir_left':np.zeros((144,2))};mount={'ir_left':np.zeros((4,2))}
    a=dict(phase='A',failures=[])
    check_static(a,'A',first,points,mount)
    bad=dict(phase='A',failures=[])
    check_static(bad,'A',first,points,{'ir_left':np.ones((4,2))})
    assert bad['board_drift_px']['ir_left']==0
    assert 'ir_left:mount_static_drift' in bad['failures']
    with pytest.raises(ValueError,match='quality failed'):fit_phase_a([bad],{},np.eye(4))


def test_aba_requires_return_to_original_then_records_c():
    gate=ContinuousAB(return_to_a=True);a=np.zeros((3,144,2));b=a+30
    for i in range(4):gate.update(i*.5,a)
    gate.update(11.5,a)
    for i in range(4):gate.update(12.+i*.5,b)
    assert gate.state=='RECORD_B'
    gate.update(23.5,b)
    assert gate.state=='WAIT_C'
    for i in range(4):assert gate.update(24.+i*.5,b)[0] is None
    for i in range(4):value=gate.update(27.+i*.5,a+1)
    assert value[0]=='C' and gate.state=='RECORD_C'
    assert gate.update(29,None)[0]=='C'
    gate.update(38.5,a)
    assert gate.state=='DONE' and gate.phases==('A','B','C')


def test_aba_return_checks_all_views_and_outlier_corners():
    gate=ContinuousAB(return_to_a=True);a=np.zeros((3,144,2))
    gate.reference=a;gate.state='WAIT_C'
    for bad in (a+6, a.copy()):
        if bad.max()==0:bad[1,0]=11
        for i in range(4):assert gate.update(i*.5,bad)[0] is None


def test_tilt_gate_rejects_translation_and_requires_all_views():
    gate=ContinuousAB(return_to_a=True,tilt_contrast=True)
    points=np.zeros((3,144,2));normals=np.tile([0.,0.,1.],(3,1))
    for i in range(4):gate.update(i*.5,points,normals)
    assert gate.state=='RECORD_A'
    gate.update(11.5,points,normals)
    for i in range(4):assert gate.update(12.+i*.5,points+100,normals)[0] is None
    tilted=np.tile([np.sin(np.deg2rad(15)),0.,np.cos(np.deg2rad(15))],(3,1))
    partial=tilted.copy();partial[2]=normals[2]
    for i in range(4):assert gate.update(14.+i*.5,points,partial)[0] is None
    for i in range(4):value=gate.update(16.+i*.5,points,tilted)
    assert value[0]=='B'  # genuine tilt can start without >20px translation
    assert gate.update(18,None,None)[0]=='B'  # retain failures after start
    gate.update(27.5,points,tilted)
    for i in range(4):value=gate.update(28.+i*.5,points,normals)
    assert value[0]=='C'


def test_tilt_gate_fails_closed_on_missing_or_invalid_normals():
    with pytest.raises(ValueError,match='locked ABA'):ContinuousAB(tilt_contrast=True)
    gate=ContinuousAB(return_to_a=True,tilt_contrast=True);points=np.zeros((3,144,2))
    for i in range(4):assert gate.update(i*.5,points)[0] is None
    with pytest.raises(ValueError,match='normals'):gate.update(2,points,np.zeros((3,3)))


def test_offline_tilt_policy_cannot_silently_relax_threshold():
    from scripts.ego_ir_umi_continuous_check import phase_rows
    setup=dict(acquisition_mode='continuous_aba',tilt_policy=dict(min_normal_change_deg=1.,all_views=True))
    with pytest.raises(ValueError,match='tilt gate mismatch'):
        phase_rows(setup,dict(phase_state='DONE',continuous_stream=True),[])
