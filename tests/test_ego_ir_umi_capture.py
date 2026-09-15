import json
import sys
from types import SimpleNamespace as NS
import pytest
from scripts import ego_ir_umi_capture as capture


OPTIONS=NS(emitter_enabled='emitter',enable_auto_exposure='auto',exposure='exposure',gain='gain')


def test_tilt_requires_locked_aba_before_output_or_device_access(monkeypatch,tmp_path):
    monkeypatch.setitem(sys.modules,'pyrealsense2',NS())
    out=tmp_path/'must_not_exist'
    monkeypatch.setattr(sys,'argv',['capture','--output',str(out),'--record-now','--tilt-contrast'])
    with pytest.raises(SystemExit) as exc:capture.main()
    assert exc.value.code==2 and not out.exists()


class Sensor:
    def __init__(self,fail=None):self.values={};self.fail=fail
    def set_option(self,o,v):self.values[o]=v
    def get_option(self,o):
        if o==self.fail:raise RuntimeError('PIPE')
        return self.values[o]


def test_only_umi_gain_readback_can_be_waived():
    rs=NS(option=OPTIONS)
    value=capture.configure_sensor(Sensor('gain'),rs,'umi')
    assert value['gain']==dict(requested=96,readback=None,error='PIPE')
    for role,option in [('ego','emitter'),('umi','exposure'),('umi','auto')]:
        with pytest.raises(RuntimeError):capture.configure_sensor(Sensor(option),rs,role)


def test_second_start_failure_releases_first_camera(monkeypatch,tmp_path):
    class Device:
        def __init__(self,serial):self.serial=serial;self.sensor=Sensor()
        def get_info(self,_):return self.serial
        def first_depth_sensor(self):return self.sensor
    devices=[Device(s) for s in capture.SERIALS.values()]
    class Profile:
        def get_device(self):return devices[0]
        def get_stream(self,*_):return self
        def as_video_stream_profile(self):return self
    pipes=[]
    class Pipeline:
        def __init__(self,*_):self.index=len(pipes);self.stopped=False;pipes.append(self)
        def start(self,*_):
            if self.index:raise RuntimeError('second camera start failed')
            return Profile()
        def stop(self):self.stopped=True
    rs=NS(option=OPTIONS,context=lambda:NS(query_devices=lambda:devices),pipeline=Pipeline,
          config=lambda:NS(enable_device=lambda *_:None,enable_stream=lambda *_:None),
          stream=NS(infrared='ir',color='rgb'),format=NS(y8='y8',rgb8='rgb8'),camera_info=NS(serial_number='serial'))
    monkeypatch.setitem(sys.modules,'pyrealsense2',rs)
    for name in ('namedWindow','imshow','destroyAllWindows'):monkeypatch.setattr(capture.cv2,name,lambda *_:None)
    monkeypatch.setattr(capture.cv2,'waitKey',lambda *_:-1)
    out=tmp_path/'capture'
    monkeypatch.setattr(capture.cb,'new_output',lambda _:out)
    monkeypatch.setattr(sys,'argv',['capture','--output',str(out),'--record-now'])
    assert capture.main()==2
    assert pipes[0].stopped and not pipes[1].stopped
    report=json.loads((out/'capture_report.json').read_text())
    assert report['samples']==0 and report['cleanup_errors']==[]
    assert 'second camera start failed' in report['failure']
