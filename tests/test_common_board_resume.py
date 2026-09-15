import itertools
import json

import pytest

from scripts import common_board_capture as cc
from scripts import common_board_calibration as cb
from test_common_board_capture import fake_rig


def stopped_prefix(tmp_path,monkeypatch):
    fake_rig(monkeypatch,itertools.repeat(ord('r')))
    parent=tmp_path/'parent'
    assert cc.main(['--output',str(parent)])==0
    rows=(parent/'attempts.jsonl').read_text().splitlines()
    (parent/'attempts.jsonl').write_text('\n'.join(rows[:13])+'\n')
    report=json.loads((parent/'capture_report.json').read_text())
    report.update(completed=False,status='NOT_COMPLETED',completed_steps=13,attempts=13,
                  attempts_sha256=cb.digest(parent/'attempts.jsonl'))
    cb.write_json(parent/'capture_report.json',report)
    return parent


def test_resume_requires_explicit_unchanged_confirmation(tmp_path):
    with pytest.raises(SystemExit):
        cc.parse_args(['--output',str(tmp_path/'child'),'--resume-from',str(tmp_path/'parent')])


def test_resume_prefix_bound_new_directory_and_loader_combines_all16(tmp_path,monkeypatch,capsys):
    parent=stopped_prefix(tmp_path,monkeypatch)
    hashes={f:cb.digest(parent/f) for f in ('setup.json','capture_report.json','attempts.jsonl')}
    started,stopped=fake_rig(monkeypatch,itertools.repeat(ord('r')))
    child=tmp_path/'child'
    assert cc.main(['--output',str(child),'--resume-from',str(parent),'--confirm-unchanged'])==0
    assert started==stopped
    setup,cal,boards,mounts,h,audit=cb.load_session(child)
    assert len(boards)==13 and len(mounts)==3 and len(audit)==16
    assert setup['resume_from']['next_slot']==13
    assert setup['resume_from']['source_hashes']==hashes
    assert [json.loads(l)['slot'] for l in (child/'attempts.jsonl').read_text().splitlines()]==[13,14,15]
    assert '从第14步开始' in capsys.readouterr().out
    assert hashes=={f:cb.digest(parent/f) for f in hashes}
    report=json.loads((parent/'capture_report.json').read_text())
    report['note']='changed after continuation'
    cb.write_json(parent/'capture_report.json',report)
    with pytest.raises(ValueError,match='source/step changed'):
        cb.load_session(child)


def test_incomplete_prefix_cannot_be_solved_as_complete(tmp_path,monkeypatch):
    parent=stopped_prefix(tmp_path,monkeypatch)
    with pytest.raises(ValueError,match='incomplete'):
        cb.load_session(parent)
    assert len(cb.load_session(parent,allow_partial=True)[2])==12


def test_detector_change_on_resume_rejected_before_camera_open(tmp_path,monkeypatch):
    parent=stopped_prefix(tmp_path,monkeypatch)
    started,stopped=fake_rig(monkeypatch,itertools.repeat(ord('r')))
    with pytest.raises(ValueError,match='original board detector'):
        cc.main(['--output',str(tmp_path/'child'),'--resume-from',str(parent),
                 '--confirm-unchanged','--board-detector','umi-aprilgrid'])
    assert started==stopped==[]


def test_offline_continuation_rejects_different_detector(tmp_path,monkeypatch):
    parent=stopped_prefix(tmp_path,monkeypatch)
    fake_rig(monkeypatch,itertools.repeat(ord('r')))
    child=tmp_path/'child'
    assert cc.main(['--output',str(child),'--resume-from',str(parent),'--confirm-unchanged'])==0
    setup=json.loads((child/'setup.json').read_text())
    setup['board_detector']='umi-aprilgrid'
    cb.write_json(child/'setup.json',setup)
    report=json.loads((child/'capture_report.json').read_text())
    report['setup_sha256']=cb.digest(child/'setup.json')
    cb.write_json(child/'capture_report.json',report)
    with pytest.raises(ValueError,match='detector mismatch'):
        cb.load_session(child)


def test_missing_final_report_prevents_camera_open(tmp_path,monkeypatch):
    started,stopped=fake_rig(monkeypatch,itertools.repeat(ord('r')))
    parent=tmp_path/'still_running'
    parent.mkdir()
    cb.write_json(parent/'setup.json',{})
    with pytest.raises(FileNotFoundError):
        cc.main(['--output',str(tmp_path/'child'),'--resume-from',str(parent),'--confirm-unchanged'])
    assert started==stopped==[]
