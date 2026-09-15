from types import SimpleNamespace as NS
import copy
import numpy as np
import pytest
from scripts import common_board_calibration as cb


def scene():
    board=[NS(tag_id=i,corners=cb.object_corners(i)[:,:2]*1200+[160,170]) for i in range(36)]
    mount=NS(tag_id=1,corners=np.array([[700.,300],[780,300],[780,380],[700,380]]))
    return board,mount


def test_simultaneous_same_id_keeps_mount_and35_board_tags():
    from scripts.common_board_joint import resolve_targets
    board,mount=scene()
    ds,m,proof=resolve_targets(board+[mount],[board[1],mount])
    assert set(ds)==set(range(36))-{1}
    np.testing.assert_array_equal(m,mount.corners)
    assert proof['excluded_board_ids']==[1]


@pytest.mark.parametrize('case',['only_board','two_mounts','overlap','other_duplicate','unmatched_alias','missing_board'])
def test_ambiguous_targets_never_selected_by_expected_distance(case):
    from scripts.common_board_joint import resolve_targets
    board,mount=scene();raw=board+[mount];tags=[board[1],mount]
    if case=='only_board':raw=board;tags=[board[1]]
    if case=='two_mounts':tags.append(NS(tag_id=1,corners=mount.corners+[100,0]))
    if case=='overlap':mount.corners-=np.array([350,100])
    if case=='other_duplicate':raw.append(copy.deepcopy(board[2]))
    if case=='unmatched_alias':raw.append(NS(tag_id=1,corners=mount.corners+[120,0]))
    if case=='missing_board':raw=[mount]
    with pytest.raises(ValueError):resolve_targets(raw,tags)


def test_joint_window_requires_board_and_mount_still_in_every_frame():
    from scripts.common_board_joint import joint_window_quality
    board,mount=scene();ds={d.tag_id:d.corners for d in board if d.tag_id!=1}
    ss=[dict(t=j*.25,arrival_gap_ms=.3,reasons=[],detected={
        '_board':{r:copy.deepcopy(ds) for r in cb.ROLES},
        '_mount':{'ego':{1:mount.corners.copy()},'left':{}}}) for j in range(6)]
    assert not joint_window_quality(ss)['reasons']
    ss[-1]['detected']['_mount']['ego'][1]+=[1.,0]
    assert 'mount:window_motion' in joint_window_quality(ss)['reasons']


def test_board_only_or_mount_only_cannot_pass_joint_sample(monkeypatch):
    from scripts.common_board_joint import detect_joint_sample
    board,mount=scene();b,t=object(),object()
    monkeypatch.setattr(cb,'detect_grid',lambda det,img:board if det is b else [board[1]])
    monkeypatch.setattr(cb,'refine_legacy_corners',lambda image,ds:ds)
    ds,reasons=detect_joint_sample({r:np.zeros((720,1280),np.uint8) for r in cb.ROLES},'board',b,t)
    assert reasons and not ds['_mount']['ego']


def test_actual_decoders_separate_board_and_mount_same_id():
    import cv2
    from scripts.common_board_detector import make_board_detector
    from scripts.common_board_joint import detect_joint_sample
    image=np.full((720,1280),255,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for i in range(36):
        x,y=650-i%6*104,70+i//6*104
        image[y:y+80,x:x+80]=cv2.aruco.generateImageMarker(dictionary,i,80,borderBits=2)
    # The physical legacy board has black squares in the inter-tag junctions,
    # not isolated markers (isolated markers deliberately retain nested conflicts).
    for row in range(5):
        for col in range(5):
            x,y=130+col*104+80,70+row*104+80
            image[y:y+24,x:x+24]=0
    image[300:380,1000:1080]=cv2.aruco.generateImageMarker(dictionary,1,80,borderBits=1)
    # Both cameras share board; the same image is sufficient to exercise real
    # decoders and same-ID namespaces, not a synthetic hardware acceptance.
    ds,reasons=detect_joint_sample({r:image for r in cb.ROLES},'board',
                                  make_board_detector('umi-aprilgrid'),cb.mount_detector())
    assert not reasons,reasons
    assert 1 not in ds['ego'] and 1 not in ds['left']
    assert len(ds['ego'])==35
    np.testing.assert_allclose(ds['_mount']['ego'][1].mean(axis=0),[1040,340],atol=1)


def test_simultaneous_capture_replay_and_no_remove_board_prompts(tmp_path,monkeypatch,capsys):
    import itertools,json
    from scripts import common_board_capture as cc
    from test_common_board_capture import fake_rig
    from scripts.common_board_joint import POLICY
    started,stopped=fake_rig(monkeypatch,itertools.repeat(ord('r')))
    def detect(images,kind,*args,**kw):
        assert kw.get('simultaneous')
        board,m=scene();boards={r:{d.tag_id:d.corners.copy() for d in board if d.tag_id!=1} for r in cb.ROLES}
        mount={'ego':{1:m.corners},'left':{}}
        return dict(boards if kind=='board' else mount,_board=boards,_mount=mount),[]
    monkeypatch.setattr(cc,'detect_sample',detect);monkeypatch.setattr(cb,'detect_sample',detect)
    output=tmp_path/'joint'
    assert cc.main(['--output',str(output),'--simultaneous'])==0
    setup,_,boards,mounts,_,audit=cb.load_session(output)
    assert setup['identity_policy']==POLICY
    assert len(boards)==13 and len(mounts)==3 and len(audit)==16
    assert all('same_frame_mount_corners' in a for a in audit)
    assert all(len(w['objects'])==35*4 for w in boards)
    assert started==stopped
    assert '移走公共板' not in capsys.readouterr().out
    report=json.loads((output/'capture_report.json').read_text())
    setup['identity_policy']['excluded_board_ids']=[]
    cb.write_json(output/'setup.json',setup)
    report['setup_sha256']=cb.digest(output/'setup.json');cb.write_json(output/'capture_report.json',report)
    with pytest.raises(ValueError,match='identity/plan mismatch'):cb.load_session(output)


def test_same_frame_transform_and_frozen_mount_fault():
    from scripts.common_board_joint import same_frame_check
    from test_common_board_calibration import scene as calibrated_scene
    cal,x,windows=calibrated_scene()
    m=cb.unpack(np.array([.1,-.1,.02,-.1,0,.55]))
    pixels=cb.project(cb.MOUNT_OBJECTS,x@m,cal['ego'])
    report=same_frame_check(windows[2],pixels,cal,m)
    assert report['passed']
    np.testing.assert_allclose(report['T_Ecolor_Ucolor'],x,atol=1e-5)
    assert min(np.linalg.norm(np.asarray(c['T_Ucolor_mount'])-m) for c in report['diagnostic_mount_candidates'])<1e-4
    wrong=m.copy();wrong[0,3]+=.01
    failed=same_frame_check(windows[2],pixels,cal,wrong)
    assert not failed['passed'] and failed['fixed_M_prediction']['p95_px']>5
    np.testing.assert_array_equal(failed['T_Ecolor_Ucolor'],report['T_Ecolor_Ucolor'])


def test_joint_detection_and_same_frame_check_support_right_role(monkeypatch):
    from scripts.common_board_joint import detect_joint_sample, same_frame_check
    board, mount = scene()
    board_detector, mount_detector = object(), object()
    monkeypatch.setattr(cb, 'detect_grid',
                        lambda detector, _image: board + [mount] if detector is board_detector else [board[1], mount])
    monkeypatch.setattr(cb, 'refine_legacy_corners', lambda _image, detections: detections)
    images = {'ego': np.zeros((720,1280), np.uint8),
              'right': np.zeros((720,1280), np.uint8)}
    detected, reasons = detect_joint_sample(images, 'board', board_detector, mount_detector)
    assert not reasons
    assert set(detected['_board']) == {'ego', 'right'}
    assert detected['_mount']['right'] == {}

    from test_common_board_calibration import scene as calibrated_scene
    calibrations, truth, windows = calibrated_scene()
    calibrations['right'] = calibrations.pop('left')
    for window in windows:
        window['pixels']['right'] = window['pixels'].pop('left')
    fixed_mount = cb.unpack(np.array([.1,-.1,.02,-.1,0,.55]))
    pixels = cb.project(cb.MOUNT_OBJECTS, truth @ fixed_mount, calibrations['ego'])
    report = same_frame_check(windows[2], pixels, calibrations, fixed_mount)
    assert report['passed']
    np.testing.assert_allclose(report['T_Ecolor_Ucolor'], truth, atol=1e-5)


def test_cannot_resume_sequential_data_in_simultaneous_mode(tmp_path,monkeypatch):
    import itertools
    from scripts import common_board_capture as cc
    from test_common_board_resume import stopped_prefix
    from test_common_board_capture import fake_rig
    parent=stopped_prefix(tmp_path,monkeypatch)
    started,stopped=fake_rig(monkeypatch,itertools.repeat(ord('r')))
    with pytest.raises(ValueError,match='original observation mode'):
        cc.main(['--output',str(tmp_path/'child'),'--simultaneous',
                 '--resume-from',str(parent),'--confirm-unchanged'])
    assert not started and not stopped
