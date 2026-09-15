import json
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from scripts import common_board_capture as cc
from scripts import common_board_calibration as cb


def fake_rig(monkeypatch, keys, *, fail_second=False, fail_read=False, moving=False):
    now = [100.]
    started, stopped = [], []
    intr = NS(width=1280, height=720, fx=900, fy=900, ppx=640, ppy=360,
              coeffs=[0.]*5, model='distortion.none')
    sensor = NS(set_option=lambda *a: None, supports=lambda *a: True)

    class Config:
        def enable_device(self, serial): self.serial = serial
        def enable_stream(self, *args): pass

    class Pipeline:
        def start(self, cfg):
            if fail_second and started:
                raise RuntimeError('injected second start failure')
            self.serial, self.number = cfg.serial, 0
            started.append(self.serial)
            device = NS(get_info=lambda *_: self.serial, first_color_sensor=lambda: sensor, first_depth_sensor=lambda: sensor)
            profile = NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intr))
            return NS(get_device=lambda: device, get_stream=lambda *_: profile)

        def stop(self): stopped.append(self.serial)

        def wait_for_frames(self, *_):
            if fail_read:
                raise RuntimeError('injected read failure')
            self.number += 1
            frame = NS(get_frame_number=lambda: self.number, get_timestamp=lambda: self.number*33.3,
                       get_frame_timestamp_domain=lambda: 'hardware_clock',
                       get_data=lambda: np.zeros((720, 1280, 3), np.uint8))
            return NS(get_color_frame=lambda: frame)

    rs = NS(pipeline=Pipeline, config=Config, stream=NS(color=1), format=NS(rgb8=1),
            camera_info=NS(serial_number=1), option=NS(enable_auto_exposure=1, emitter_enabled=2, exposure=3, gain=4))
    monkeypatch.setitem(sys.modules, 'pyrealsense2', rs)
    for name in ('namedWindow', 'imshow', 'destroyAllWindows'):
        monkeypatch.setattr(cc.cv2, name, lambda *a: None)
    monkeypatch.setattr(cc.cv2, 'getWindowProperty', lambda *a: 1.)
    calls = [0]

    def key(_):
        calls[0] += 1
        if calls[0] == 1:  # Initial GUI initialization, not an operator step.
            return -1
        now[0] += .25
        return next(keys)

    monkeypatch.setattr(cc.cv2, 'waitKey', key)
    monkeypatch.setattr(cc.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(cc.time, 'monotonic_ns', lambda: int(now[0]*1e9))
    monkeypatch.setenv('DISPLAY', ':fake')

    def detection(images, kind, *_, **kwargs):
        roles = tuple(images)
        umi_role = next(r for r in roles if r != 'ego')
        if kind == 'mount':
            return {'ego': {1: np.array([[500.,200], [580,200], [580,280], [500,280]])}, umi_role: {}}, []
        tags = {i: np.array([[500.,200], [550,200], [550,250], [500,250]]) + [i%6*60, i//6*60]
                for i in range(36)}
        if moving:
            tags = {i: q + [now[0]*2, 0] for i, q in tags.items()}
        return {r: {i:q.copy() for i,q in tags.items()} for r in roles}, []

    monkeypatch.setattr(cc, 'detect_sample', detection)
    monkeypatch.setattr(cb, 'detect_sample', detection)
    return started, stopped


def test_no_invisible_start_and_no_external_output(tmp_path, monkeypatch):
    monkeypatch.delenv('DISPLAY', raising=False)
    out = tmp_path/'session'
    with pytest.raises(RuntimeError, match='DISPLAY'):
        cc.main(['--output', str(out)])
    assert not out.exists()
    with pytest.raises(SystemExit):
        cc.parse_args(['--output', '/tmp/not-authorized'])
    with pytest.raises(SystemExit):
        cc.parse_args(['--output', str(out), '--preview-hz', 'nan'])


def test_q_before_r_only_preview_no_raw(tmp_path, monkeypatch):
    started, stopped = fake_rig(monkeypatch, iter([ord('q')]))
    out = tmp_path/'session'
    assert cc.main(['--output', str(out)]) == 0
    report = json.loads((out/'capture_report.json').read_text())
    assert report['attempts'] == 0 and not report['completed']
    assert not list(out.rglob('*.png'))
    assert started == stopped == list(cc.DEVICES.values())


@pytest.mark.parametrize('failure', ['second', 'read'])
def test_hardware_failure_stops_every_successfully_opened_camera(tmp_path, monkeypatch, failure):
    started, stopped = fake_rig(monkeypatch, iter([]), fail_second=failure=='second', fail_read=failure=='read')
    out = tmp_path/'session'
    assert cc.main(['--output', str(out)]) == 2
    assert started == stopped
    report = json.loads((out/'capture_report.json').read_text())
    assert 'injected' in report['failure'] and not report['completed']


def test_interrupt_retains_raw_rejection_without_advancing(tmp_path, monkeypatch):
    started, stopped = fake_rig(monkeypatch, iter([ord('r'), ord('q')]))
    out = tmp_path/'session'
    assert cc.main(['--output', str(out)]) == 2
    row = json.loads((out/'attempts.jsonl').read_text())
    assert not row['accepted'] and 'operator_aborted' in row['reasons']
    assert len(list(out.rglob('*.png'))) == 2
    assert started == stopped


def test_motion_is_rejected_kept_and_same_step_repeated(tmp_path, monkeypatch, capsys):
    keys = iter([ord('r')] + [-1]*6 + [ord('q')])
    fake_rig(monkeypatch, keys, moving=True)
    out = tmp_path/'session'
    assert cc.main(['--output', str(out)]) == 2
    row = json.loads((out/'attempts.jsonl').read_text())
    assert not row['accepted'] and 'window_motion' in row['reasons']
    assert row['slot'] == 0 and len(row['pairs']) >= 3
    assert 'RETRY' in capsys.readouterr().out


def test_complete_capture_load_hashes_and_final_board_closure(tmp_path, monkeypatch, capsys):
    import itertools
    started, stopped = fake_rig(monkeypatch, itertools.repeat(ord('r')))
    out = tmp_path/'session'
    assert cc.main(['--output', str(out)]) == 0
    assert started == stopped
    setup, cal, windows, mounts, hashes, audit = cb.load_session(out)
    assert len(windows) == 13 and windows[-1]['slot'] == 15
    assert len(mounts) == 3 and len(audit) == 16
    assert sum(w['split']=='train' for w in windows) == 8
    assert setup['factory_values_modified'] is False
    assert setup['activation'] == 'NOT_ACTIVATED'
    assert 'NEXT' in capsys.readouterr().out
    image = out/'attempt_000/ego_000.png'
    image.write_bytes(b'corrupt raw bytes')
    with pytest.raises(ValueError, match='hash'):
        cb.load_session(out)


def test_right_role_is_preserved_in_capture_paths_and_offline_loader(tmp_path, monkeypatch):
    import itertools
    started, stopped = fake_rig(monkeypatch, itertools.repeat(ord('r')))
    out = tmp_path/'right_session'
    assert cc.main(['--output', str(out), '--umi-role', 'right',
                    '--umi-serial', 'device3-d405']) == 0
    setup, calibrations, windows, mounts, _, _ = cb.load_session(out)
    assert setup['umi_role'] == 'right'
    assert set(setup['devices']) == {'ego', 'right'}
    assert set(calibrations) == {'ego', 'right'}
    assert set(windows[0]['pixels']) == {'ego', 'right'}
    assert list(out.glob('attempt_*/right_*.png'))
    assert started == stopped == [cc.DEFAULT_EGO_SERIAL, 'device3-d405']


def test_failed_index_close_does_not_prevent_hardware_release(tmp_path, monkeypatch):
    from pathlib import Path
    started, stopped = fake_rig(monkeypatch, iter([ord('q')]))
    original = Path.open

    class BadClose:
        def __init__(self, file): self.file = file
        def close(self):
            self.file.close()
            raise OSError('injected index close')

    def open_file(path, *a, **kw):
        file = original(path, *a, **kw)
        return BadClose(file) if path.name=='attempts.jsonl' and a==('x',) else file

    monkeypatch.setattr(Path, 'open', open_file)
    out = tmp_path/'session'
    assert cc.main(['--output', str(out)]) == 2
    assert started == stopped
    report = json.loads((out/'capture_report.json').read_text())
    assert any('injected index close' in s for s in report['cleanup_errors'])
