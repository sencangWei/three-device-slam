from types import SimpleNamespace as NS
import pytest
from scripts.ego_ir_lock import read_settings,apply_lock,verify_lock,restore_settings,check_frame_evidence,frame_evidence

RS=NS(option=NS(enable_auto_exposure='enable_auto_exposure',exposure='exposure',gain='gain'),
      frame_metadata_value=NS(actual_exposure='actual_exposure',gain_level='gain_level',auto_exposure='auto_exposure'))


class Sensor:
    def __init__(self):self.values=dict(enable_auto_exposure=1.,exposure=8500.,gain=16.);self.writes=[]
    def get_option(self,key):return self.values[key]
    def set_option(self,key,value):self.values[key]=value;self.writes.append(key)


def test_lock_readback_restore_order():
    sensor=Sensor();original=read_settings(sensor,RS);expected=apply_lock(sensor,RS,original)
    assert expected==dict(enable_auto_exposure=0.,exposure=8500.,gain=16.)
    sensor.values['exposure']=9000
    with pytest.raises(ValueError,match='changed'):verify_lock(sensor,RS,expected)
    restore_settings(sensor,RS,original)
    assert sensor.values==original
    assert sensor.writes[-3:]==['exposure','gain','enable_auto_exposure']


def test_frame_metadata_missing_or_changed_is_not_pass():
    expected=dict(exposure=8500.,gain=16.,enable_auto_exposure=0.)
    good={k:dict(supported=True,value=v) for k,v in dict(actual_exposure=8500.,gain_level=16.,auto_exposure=0.).items()}
    check_frame_evidence(good,expected)
    for k,bad in [('actual_exposure',dict(supported=False,value=None)),('gain_level',dict(supported=True,value=17)),
                  ('actual_exposure',dict(supported=True,value=float('nan'))),('auto_exposure',dict(supported=True,value=1))]:
        with pytest.raises(ValueError):check_frame_evidence(dict(good,**{k:bad}),expected)
    class Unsupported:
        def supports_frame_metadata(self,key):return False
    assert all(not v['supported'] for v in frame_evidence(Unsupported(),RS).values())


def test_restore_attempts_auto_mode_after_exposure_failure():
    class Failing(Sensor):
        def set_option(self,key,value):
            if key=='exposure':
                self.writes.append(key)
                raise RuntimeError('PIPE')
            super().set_option(key,value)
    sensor=Failing();original=read_settings(sensor,RS)
    sensor.values['enable_auto_exposure']=0.
    with pytest.raises(RuntimeError,match='exposure:PIPE'):restore_settings(sensor,RS,original)
    assert sensor.values['enable_auto_exposure']==1
    assert sensor.writes==['exposure','gain','enable_auto_exposure']


def test_completed_aba_requires_three_ordered_spans_and_verified_ir():
    import copy
    from scripts.ego_ir_umi_continuous_check import phase_rows
    expected=dict(enable_auto_exposure=0.,exposure=8500.,gain=16.)
    lock=dict(original=dict(expected,enable_auto_exposure=1.),expected=expected)
    setup=dict(acquisition_mode='continuous_aba',ir_lock=lock,return_policy=dict(phase='C',median_corner_distance_px=5.,max_corner_distance_px=10.,origin_translation_mm=10.,normal_deg=2.))
    capture=dict(phase_state='DONE',continuous_stream=True,phase_samples=dict(A=10,B=10,C=10),ir_lock=lock,
                 ir_final_readback=expected,ir_original_settings_restored=True)
    md={k:dict(supported=True,value=v) for k,v in dict(actual_exposure=8500.,gain_level=16.,auto_exposure=0.).items()}
    rows=[dict(phase=p,streams={r:dict(host_arrival_monotonic_ns=(j*20+i)*10**9,ir_metadata=md) for r in ('ir_left','ir_right')}) for j,p in enumerate('ABC') for i in range(10)]
    assert list(phase_rows(setup,capture,rows))==list('ABC')
    with pytest.raises(ValueError):phase_rows(setup,capture,rows[:20])
    with pytest.raises(ValueError):phase_rows(setup,capture,rows[::-1])
    bad=copy.deepcopy(rows);bad[20]['streams']['ir_left']['ir_metadata']['actual_exposure']['value']=9000
    with pytest.raises(ValueError,match='mismatch'):phase_rows(setup,capture,bad)


def test_warmup_uses_stable_live_values_not_idle_defaults():
    from scripts.ego_ir_lock import warmup_choice
    history=[]
    row={k:dict(supported=True,value=v) for k,v in dict(actual_exposure=30000.,gain_level=64.).items()}
    evidence=dict(ir_left=row,ir_right=row)
    for now in (0.,.5,1.,1.5):assert warmup_choice(history,now,evidence,True,now) is None
    history.clear()
    for now in (6.,6.5,7.):assert warmup_choice(history,now,evidence,True,now) is None
    assert warmup_choice(history,7.5,evidence,True,7.5)==dict(enable_auto_exposure=0.,exposure=30000.,gain=64.)
    assert warmup_choice(history,8.,evidence,False,8.) is None and not history


def test_transient_option_read_retries_but_persistent_failure_stops(monkeypatch):
    from scripts import ego_ir_lock as lock
    class Transient:
        def __init__(self):self.calls=0
        def get_option(self,key):
            self.calls+=1
            if self.calls<3:raise RuntimeError('PIPE')
            return 16.
    ticks=[0.]
    monkeypatch.setattr(lock.time,'monotonic',lambda:ticks[0])
    monkeypatch.setattr(lock.time,'sleep',lambda seconds:ticks.__setitem__(0,ticks[0]+seconds))
    sensor=Transient()
    assert lock.read_option(sensor,'gain')==16 and sensor.calls==3
    class Permanent:
        def get_option(self,key):raise RuntimeError('PIPE')
    with pytest.raises(RuntimeError,match='PIPE'):lock.read_option(Permanent(),'gain')
    assert ticks[0]<.7
def test_tilt_placement_wait_is_separate_from_bounded_convergence():
    from scripts.ego_ir_lock import warmup_timed_out
    assert warmup_timed_out(31,0,None)  # original ABA retains start+30
    assert not warmup_timed_out(200,0,None,True)  # outer capture still capped300
    assert not warmup_timed_out(225,0,200,True)
    assert warmup_timed_out(231,0,200,True)
