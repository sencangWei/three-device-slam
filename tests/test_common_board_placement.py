import itertools
import json
import numpy as np
import pytest
from scripts.common_board_placement import PlacementCheck
from scripts import common_board_capture as cc
from scripts import common_board_calibration as cb
from test_common_board_capture import fake_rig


def test_requires_stability_and_rejects_changed_mount_view():
    p=PlacementCheck()
    d={'ego':{1:np.array([[100.,100],[180,100],[180,180],[100,180]])}}
    assert p.advance({'reasons':['not_still']},d)[0] is False
    assert p.step==0
    assert p.advance({'reasons':[]},d)[0]
    assert p.kind=='board'
    assert p.advance({'reasons':[]},d)[0]
    d['ego'][1]+=[2.,0]
    assert not p.advance({'reasons':[]},d)[0]
    assert p.step==2


def test_three_visibility_checks_never_become_calibration(tmp_path,monkeypatch):
    started,stopped=fake_rig(monkeypatch,itertools.repeat(ord('r')))
    output=tmp_path/'placement'
    assert cc.main(['--output',str(output),'--placement-check-only'])==0
    report=json.loads((output/'capture_report.json').read_text())
    assert report['placement_complete'] and not report['completed']
    assert report['completed_steps']==report['attempts']==0
    assert [x['kind'] for x in report['placement_checks']]==['mount','board','mount']
    assert len(list(output.glob('placement_*.png')))==6
    assert started==stopped
    with pytest.raises(ValueError,match='not a calibration'):
        cb.load_session(output,allow_partial=True)


def test_current_invalid_frame_cannot_use_previous_ready_window():
    from test_common_board_visibility import flicker_samples
    samples=flicker_samples()
    assert not cb.window_quality(samples,'board')['reasons']
    current=dict(t=samples[-1]['t']+.05,arrival_gap_ms=.3,
                 detected={'ego':{},'left':{}},reasons=['ego:duplicate_board_id'])
    q=cc.current_preview_quality(samples,current,'board')
    assert 'ego:duplicate_board_id' in q['reasons']
    assert 'ego:anchor_coverage' in q['reasons']
    p=PlacementCheck();p.step=1
    assert not p.advance(q,current['detected'])[0]
    assert p.step==1
