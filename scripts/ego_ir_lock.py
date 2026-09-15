"""Checked temporary IR settings and per-frame exposure evidence; no calibration writes."""
import math
import time


def warmup_timed_out(now,stream_start,placement_ready,wait_for_board=False):
    """Tilt placement may use the outer300s budget; started warmup never resets."""
    start=placement_ready if wait_for_board else stream_start
    return start is not None and now-start>30.


def read_option(sensor,option):
    """Bound transient immediate SET->GET PIPE errors; never waive readback."""
    deadline=time.monotonic()+.5
    while True:
        try:return sensor.get_option(option)
        except RuntimeError:
            if time.monotonic()>=deadline:raise
            time.sleep(.05)


def read_settings(sensor,rs):
    return {name:float(read_option(sensor,getattr(rs.option,name)))
            for name in ('enable_auto_exposure','exposure','gain')}


def apply_lock(sensor,rs,original):
    expected=dict(original,enable_auto_exposure=0.)
    if any(not math.isfinite(v) for v in expected.values()) or expected['exposure']<=0:
        raise ValueError('invalid IR settings')
    for name in ('enable_auto_exposure','exposure','gain'):
        sensor.set_option(getattr(rs.option,name),expected[name])
    verify_lock(sensor,rs,expected)
    return expected


def verify_lock(sensor,rs,expected):
    actual=read_settings(sensor,rs)
    if actual!=expected:raise ValueError('IR settings changed: '+str(actual))
    return actual


def restore_settings(sensor,rs,original):
    # Exposure/gain writes may disable auto mode, so restore auto mode last.
    errors=[]
    for name in ('exposure','gain'):
        try:
            sensor.set_option(getattr(rs.option,name),original[name])
            if read_option(sensor,getattr(rs.option,name))!=original[name]:
                raise ValueError('readback mismatch')
        except Exception as exc:errors.append(name+':'+str(exc))
    # Always attempt auto-mode restoration even if an exposure/gain SET failed.
    try:
        sensor.set_option(rs.option.enable_auto_exposure,original['enable_auto_exposure'])
        if read_option(sensor,rs.option.enable_auto_exposure)!=original['enable_auto_exposure']:
            raise ValueError('readback mismatch')
    except Exception as exc:errors.append('enable_auto_exposure:'+str(exc))
    if errors:raise RuntimeError('IR restore failed: '+'; '.join(errors))


def frame_evidence(frame,rs):
    result={}
    for key in ('actual_exposure','gain_level','auto_exposure'):
        item=getattr(rs.frame_metadata_value,key)
        try:
            result[key]=dict(supported=bool(frame.supports_frame_metadata(item)),value=None)
            if result[key]['supported']:result[key]['value']=float(frame.get_frame_metadata(item))
        except RuntimeError as exc:result[key]=dict(supported=False,value=None,error=str(exc))
    return result


def check_frame_evidence(evidence,expected):
    for name,target in (('actual_exposure',expected['exposure']),('gain_level',expected['gain'])):
        entry=evidence.get(name,{})
        if not entry.get('supported') or entry.get('value') is None:
            raise ValueError('IR metadata unavailable: '+name)
        value=entry['value']
        tolerance=1. if name=='actual_exposure' else 0.
        if not math.isfinite(value) or abs(value-target)>tolerance:
            raise ValueError('IR frame setting mismatch: '+name+' '+str(value))
    auto=evidence.get('auto_exposure',{})
    if auto.get('supported') and auto.get('value')!=0:
        raise ValueError('IR frame auto exposure enabled')


def warmup_choice(history,now,evidence,visible,elapsed):
    """Freeze converged LIVE metadata, not idle sensor option defaults."""
    pairs=[]
    for role in ('ir_left','ir_right'):
        row=evidence[role]
        values=[]
        for key in ('actual_exposure','gain_level'):
            entry=row[key]
            if not entry['supported'] or entry['value'] is None or not math.isfinite(entry['value']):
                raise ValueError('IR warmup metadata unavailable: '+role+':'+key)
            values.append(entry['value'])
        pairs.append(values)
    if pairs[0]!=pairs[1] or not visible:
        history.clear();return None
    history.append((now,pairs[0]))
    history[:]=[v for v in history if now-v[0]<=2.]
    if elapsed<6 or len(history)<4 or now-history[0][0]<1.5:return None
    for i in range(2):
        values=[v[1][i] for v in history]
        if max(values)-min(values)>abs(values[-1])*.02:return None
    return dict(enable_auto_exposure=0.,exposure=pairs[0][0],gain=pairs[0][1])
