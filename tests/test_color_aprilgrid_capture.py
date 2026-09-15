import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from scripts import color_aprilgrid_capture as capture


def grid():
    result=[]
    for row in range(3):
        for col in range(4):
            origin=np.array([150+col*52,100+row*52])
            corners=origin+np.array([[0,0],[40,0],[40,40],[0,40]])
            result.append(NS(tag_id=row*6+col,corners=corners))
    return result


def test_preview_geometry_does_not_accept_duplicates_or_single_tags():
    detections=grid()
    report=capture.grid_summary(detections)
    assert report['good'] and report['tags']==12
    assert 'not_calibration_acceptance'in report['reason']
    assert not capture.grid_summary(detections+[detections[0]])['good']
    assert not capture.grid_summary(detections[:3])['good']
    detections[-1].corners=detections[-1].corners+[30,90]
    assert not capture.grid_summary(detections)['good']


def test_cli_rejects_external_writes_and_bad_duration(tmp_path):
    with pytest.raises(SystemExit):capture.parse_args(['--output','/tmp/not-authorized'])
    local=Path(capture.__file__).resolve().parents[1]/'artifacts/test-grid-placeholder'
    with pytest.raises(SystemExit):capture.parse_args(['--output',str(local),'--duration','nan'])
    with pytest.raises(SystemExit):capture.parse_args(['--output',str(local),'--preview-hz','0'])


def fake_hardware(monkeypatch,keys):
    stopped=[];started=[]
    intr=NS(width=1280,height=720,fx=900,fy=900,ppx=640,ppy=360,coeffs=[0]*5,model='none')
    sensor=NS(set_option=lambda *a:None,supports=lambda *a:True)
    class Config:
        def enable_device(self,serial):self.serial=serial
        def enable_stream(self,*a):pass
    class Pipeline:
        def start(self,config):
            self.serial=config.serial;self.number=0;started.append(self.serial)
            dev=NS(get_info=lambda *_:self.serial,first_color_sensor=lambda:sensor,first_depth_sensor=lambda:sensor)
            stream=NS(as_video_stream_profile=lambda:NS(get_intrinsics=lambda:intr))
            return NS(get_device=lambda:dev,get_stream=lambda *_:stream)
        def stop(self):stopped.append(self.serial)
        def wait_for_frames(self,*a):
            self.number+=1
            frame=NS(get_frame_number=lambda:self.number,get_timestamp=lambda:self.number*33.3,
                     get_frame_timestamp_domain=lambda:'hardware_clock',get_data=lambda:np.zeros((720,1280,3),np.uint8))
            return NS(get_color_frame=lambda:frame)
    rs=NS(pipeline=Pipeline,config=Config,stream=NS(color=1),format=NS(rgb8=1),
          camera_info=NS(serial_number=1),option=NS(enable_auto_exposure=1,emitter_enabled=2,exposure=3,gain=4))
    monkeypatch.setitem(sys.modules,'pyrealsense2',rs)
    monkeypatch.setattr(capture,'make_detector',lambda:None)
    monkeypatch.setattr(capture,'detect_grid',lambda *_:grid())
    for name in ('namedWindow','imshow','destroyAllWindows'):
        monkeypatch.setattr(capture.cv2,name,lambda *a:None)
    monkeypatch.setattr(capture.signal,'signal',lambda *a:None)
    monkeypatch.setattr(capture.cv2,'waitKey',lambda *_:next(keys))
    monkeypatch.setenv('DISPLAY',':fake')
    return started,stopped


@pytest.mark.parametrize('record',[False,True])
def test_preview_never_records_until_r_and_q_preserves_partial(tmp_path,monkeypatch,record):
    args=NS(output=tmp_path/'session',duration=90.,preview_hz=5.)
    monkeypatch.setattr(capture,'parse_args',lambda *_:args)
    started,stopped=fake_hardware(monkeypatch,iter([ord('r'),ord('q')]if record else[ord('q')]))
    assert capture.main([])==(2 if record else 0)
    report=json.loads((args.output/'capture_report.json').read_text())
    assert report['started']==record
    assert report['operator_aborted']==record
    assert report['activation']=='NOT_ACTIVATED'
    assert set(started)==set(stopped)==set(capture.DEVICES.values())
    for role in capture.DEVICES:
        assert report['counts'][role]==int(record)
        index=args.output/role/'images.jsonl'
        assert index.exists()==record
        if record:
            row=json.loads(index.read_text())
            assert row['sdk_timestamp_domain']=='hardware_clock'
            assert (args.output/role/row['file']).exists()


def test_missing_display_fails_before_output_creation(tmp_path,monkeypatch):
    args=NS(output=tmp_path/'session',duration=90.,preview_hz=5.)
    monkeypatch.setattr(capture,'parse_args',lambda *_:args)
    monkeypatch.delenv('DISPLAY',raising=False)
    with pytest.raises(RuntimeError,match='DISPLAY'):capture.main([])
    assert not args.output.exists()


@pytest.mark.parametrize('rotation', range(4))
def test_real_detector_finds_all_standard_tags_and_black_corners(rotation):
    cv2=capture.cv2
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    gray=np.full((720,1280),255,np.uint8)
    for row in range(6):
        for col in range(6):
            gray[60+row*78:120+row*78,160+col*78:220+col*78]=cv2.aruco.generateImageMarker(dictionary,row*6+col,60,borderBits=2)
    gray=np.ascontiguousarray(np.rot90(gray,rotation))
    detections=capture.detect_grid(capture.make_detector(),gray)
    assert sorted(d.tag_id for d in detections)==list(range(36))
    report=capture.grid_summary(detections,width=gray.shape[1],height=gray.shape[0])
    assert report['good']
    assert 58 <= report['min_tag_edge_px'] <= 61


def test_file_close_failure_still_stops_both_cameras(tmp_path,monkeypatch):
    args=NS(output=tmp_path/'session',duration=90.,preview_hz=5.)
    monkeypatch.setattr(capture,'parse_args',lambda *_:args)
    started,stopped=fake_hardware(monkeypatch,iter([ord('r'),ord('q')]))
    original=Path.open
    class BadClose:
        def __init__(self,handle):self.handle=handle
        def write(self,*a):return self.handle.write(*a)
        def flush(self):return self.handle.flush()
        def close(self):
            self.handle.close()
            raise OSError('injected close failure')
    def open_file(path,*a,**kw):
        handle=original(path,*a,**kw)
        return BadClose(handle) if path.name=='images.jsonl' and a==('x',) else handle
    monkeypatch.setattr(Path,'open',open_file)
    assert capture.main([])==2
    assert set(started)==set(stopped)==set(capture.DEVICES.values())
    report=json.loads((args.output/'capture_report.json').read_text())
    assert report['status']=='NOT_COMPLETED'
    assert 'injected close failure' in report['failure']


@pytest.mark.parametrize('role',['ego','left'])
def test_single_camera_fixed_factory_capture_binds_stage_and_setup(tmp_path,monkeypatch,role):
    import hashlib
    args=NS(output=tmp_path/'session',duration=30.,preview_hz=5.,role=role,board_orientation='normal')
    monkeypatch.setattr(capture,'parse_args',lambda *_:args)
    started,stopped=fake_hardware(monkeypatch,iter([ord('r'),ord('q')]))
    assert capture.main([])==2
    assert started==stopped==[capture.DEVICES[role]]
    setup=args.output/'setup.json'
    data=json.loads(setup.read_text())
    assert 'NEVER FIT K/D'in data['scope']
    assert not data['factory_values_modified']
    assert list(data['devices'])==[role]
    row=json.loads((args.output/role/'images.jsonl').read_text())
    assert row['setup_sha256']==hashlib.sha256(setup.read_bytes()).hexdigest()
    assert row['board_orientation_operator_label']=='normal'


def test_guidance_and_next_steps_do_not_request_intrinsic_solving():
    assert capture.pose_guidance(0,30).startswith('CENTER')
    assert capture.pose_guidance(5,30).startswith('LEFT')
    assert capture.pose_guidance(100,30).startswith('NEAR/FAR')
    assert 'rotate the BOARD180' in capture.next_stage('ego','normal')
    assert 'UMI' in capture.next_stage('ego','rotated180')
    assert 'FACTORY K/D FIXED' in capture.next_stage('left','rotated180')


def test_timed_stage_closes_camera_and_announces_review_before_next(tmp_path,monkeypatch,capsys):
    args=NS(output=tmp_path/'session',duration=30.,preview_hz=5.,role='ego',board_orientation='normal')
    monkeypatch.setattr(capture,'parse_args',lambda *_:args)
    started,stopped=fake_hardware(monkeypatch,iter([]))
    now=[100.];keys=iter([ord('r'),0])
    def key(_):
        value=next(keys)
        now[0]+=.1 if value==ord('r') else 31.
        return value
    monkeypatch.setattr(capture.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(capture.cv2,'waitKey',key)
    assert capture.main([])==0
    report=json.loads((args.output/'capture_report.json').read_text())
    assert report['status']=='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION'
    assert report['counts']=={'ego':1}
    assert started==stopped==[capture.DEVICES['ego']]
    assert 'NEXT (after image quality review)' in capsys.readouterr().out
