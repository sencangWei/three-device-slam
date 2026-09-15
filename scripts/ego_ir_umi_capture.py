#!/usr/bin/env python3
"""One authorized 10s static capture: Ego stereo IR and UMI color, no calibration writes."""
import argparse
import json
from pathlib import Path
import threading
import time
import traceback

import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.pair_color_tag_capture import _intrinsics_dict
from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics

SERIALS = dict(ego='327122078613', umi='260322279785')
STREAMS = ('ir_left', 'ir_right', 'umi_color')
WINDOW = 'EGO IR LEFT | EGO IR RIGHT | UMI COLOR - automatic 10s - Q stop'


def configure_sensor(sensor, rs, role):
    options = ((rs.option.emitter_enabled, 0),) if role == 'ego' else (
        (rs.option.enable_auto_exposure, 0), (rs.option.exposure, 30000), (rs.option.gain, 96))
    settings = {}
    for option, value in options:
        sensor.set_option(option, value)
        try:
            actual = sensor.get_option(option)
        except RuntimeError as exc:
            # Optional gain telemetry only; emitter safety is never waived.
            if role != 'umi' or option != rs.option.gain:
                raise
            settings[str(option)] = dict(requested=value, readback=None, error=str(exc))
            print('WARNING: UMI gain SET succeeded; GET failed:', str(exc), flush=True)
            continue
        if actual != value:
            raise ValueError('pre-stream setting mismatch: '+role+str(option))
        settings[str(option)] = actual
    return settings


def main():
    import pyrealsense2 as rs
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record-now', action='store_true', required=True)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--continuous-ab',action='store_true',help='One uninterrupted stream, automatic A/B 10s phases; max300s')
    modes.add_argument('--continuous-aba',action='store_true',help='Locked Ego IR; A/B/C(return A) each10s; max300s')
    parser.add_argument('--tilt-contrast',action='store_true',help='With locked ABA: B requires >=12deg board normal change in every view')
    args = parser.parse_args()
    if args.tilt_contrast and not args.continuous_aba:parser.error('--tilt-contrast requires --continuous-aba')
    out = cb.new_output(args.output)
    out.mkdir(parents=True)
    pipes, latest, rows, previous = {}, {}, [], {}
    lock = threading.Lock()
    handle = None
    first = None
    stream_start_request=None
    controller=observer=None
    observation=None
    message='WARMUP'
    observation_at=-1.
    announced_state=None
    ir_sensor=ir_original=ir_expected=None
    ir_warmup=[]
    ir_warmup_start=None
    if args.continuous_ab or args.continuous_aba:
        from scripts.ego_ir_umi_continuous import ContinuousAB,BoardObserver
        controller,observer=ContinuousAB(return_to_a=args.continuous_aba,tilt_contrast=args.tilt_contrast),BoardObserver()
    if args.continuous_aba:
        from scripts.ego_ir_lock import read_settings,apply_lock,verify_lock,restore_settings,frame_evidence,check_frame_evidence,warmup_choice,warmup_timed_out
    report = dict(status='INCOMPLETE', activation='NOT_ACTIVATED', failure=None, cleanup_errors=[])

    def callback(role):
        def receive(frame):
            arrival = time.monotonic_ns()
            with lock:
                latest[role] = (frame.as_frameset(), arrival)
        return receive

    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(WINDOW, np.zeros((270, 1440, 3), np.uint8))
        cv2.waitKey(1)
        profiles = {}
        settings = {}
        context = rs.context()
        devices = {d.get_info(rs.camera_info.serial_number): d for d in context.query_devices()}
        for role, serial in SERIALS.items():
            sensor = devices[serial].first_depth_sensor()
            settings[role] = configure_sensor(sensor, rs, role)
            if role=='ego' and args.continuous_aba:
                ir_sensor=sensor;ir_original=read_settings(sensor,rs)
                sensor.set_option(rs.option.enable_auto_exposure,1)
                if sensor.get_option(rs.option.enable_auto_exposure)!=1:raise ValueError('IR auto warmup unavailable')
                report['ir_warmup_original']=ir_original
                print('IR WARMUP: auto exposure before sampling; will lock LIVE frame exposure/gain after convergence.',flush=True)
        print('PRE-STREAM: settings saved with readback status; Ego emitter OFF confirmed.', flush=True)
        for role, serial in SERIALS.items():
            print('STARTING:', role, serial, flush=True)
            cfg = rs.config()
            cfg.enable_device(serial)
            if role == 'ego':
                for index in (1, 2):
                    cfg.enable_stream(rs.stream.infrared, index, 1280, 720, rs.format.y8, 30)
            else:
                cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
            pipe = rs.pipeline(context)
            if stream_start_request is None:stream_start_request=time.monotonic()
            profile = pipe.start(cfg, callback(role))
            pipes[role] = pipe
            print('STREAMING:', role, flush=True)
            device = profile.get_device()
            if device.get_info(rs.camera_info.serial_number) != serial:
                raise ValueError('wrong device: '+role)
            sensor = device.first_depth_sensor()
            if role == 'ego':
                sensor.set_option(rs.option.emitter_enabled, 0)
                if sensor.get_option(rs.option.emitter_enabled) != 0:
                    raise ValueError('emitter off not confirmed')
                if args.continuous_aba:
                    ir_sensor=sensor
                    if sensor.get_option(rs.option.enable_auto_exposure)!=1:raise ValueError('IR warmup mode changed')
                for index, name in ((1, 'ir_left'), (2, 'ir_right')):
                    profiles[name] = profile.get_stream(rs.stream.infrared, index).as_video_stream_profile()
            else:
                profiles['umi_color'] = profile.get_stream(rs.stream.color).as_video_stream_profile()
            print('OPEN:', role, serial, '1280x720@30', flush=True)
        for name, profile in profiles.items():
            expected = rs.format.rgb8 if name == 'umi_color' else rs.format.y8
            if (profile.width(), profile.height(), profile.fps(), profile.format()) != (1280, 720, 30, expected):
                raise ValueError('profile mismatch: '+name)
        setup = dict(schema='ego.ir_umi_capture.v1', serials=SERIALS, factory_values_modified=False,
            activation='NOT_ACTIVATED', intrinsics={r: _intrinsics_dict(p.get_intrinsics()) for r,p in profiles.items()},
            right_from_left=serialize_sdk_extrinsics(profiles['ir_left'].get_extrinsics_to(profiles['ir_right'])),
            sensor_settings=settings, native_fps=30, duration_seconds=10, storage_sample_hz=2,
            timing='Independent SDK callbacks; host arrival gap is NOT cross-device exposure synchronization',
            code_sha256=cb.digest(Path(__file__).resolve()))
        if controller:
            setup.update(acquisition_mode='continuous_ab',duration_seconds=None,phase_duration_seconds=10,
                maximum_stream_seconds=300,phase_start_policy='all36 eachview, 1.5s <=.75px stable; B >20px perview median board change',
                controller_sha256=cb.digest(Path(__file__).with_name('ego_ir_umi_continuous.py')))
        if args.continuous_aba:
            setup.update(acquisition_mode='continuous_aba',ir_lock=dict(original=ir_original,expected=ir_expected),
                ir_warmup_policy=dict(min_elapsed_s=6.,stable_span_s=1.5,relative_range_max=.02,max_elapsed_s=30.,
                    all36_each_view=True,max_host_age_ms=250.,max_host_gap_ms=75.,post_lock_drain_s=2.),
                return_policy=dict(phase='C',median_corner_distance_px=5.,max_corner_distance_px=10.,
                                   origin_translation_mm=10.,normal_deg=2.),
                ir_lock_code_sha256=cb.digest(Path(__file__).with_name('ego_ir_lock.py')))
        setup_hash=None
        tilt_cal=None
        if args.tilt_contrast:
            setup.update(tilt_policy=dict(min_normal_change_deg=12.,all_views=True),
                phase_start_policy='all36 eachview, 1.5s <=.75px stable; B >=12deg normal change in each view; C return A')
            tilt_cal={k:cb._camera_calibration_from_intrinsics(calibration_id='live_factory',frame_id=k,intrinsics=v)
                      for k,v in setup['intrinsics'].items()}
            tilt_objects=np.concatenate([cb.object_corners(i) for i in range(36)])
            setup['ir_warmup_policy']['deadline_origin']='first_all36_fresh_after_placement; never resets; outer300s unchanged'
        if not args.continuous_aba:
            cb.write_json(out/'setup.json', setup)
            setup_hash = cb.digest(out/'setup.json')
        handle = (out/'frames.jsonl').open('x')
        ready = time.monotonic()+2
        timeout = stream_start_request+300 if controller else ready+15
        saved_at = preview_at = -1.
        while time.monotonic() < timeout:
            now = time.monotonic()
            if args.continuous_aba and ir_expected is None and warmup_timed_out(now,stream_start_request,ir_warmup_start,args.tilt_contrast):
                raise RuntimeError('IR auto warmup exceeded30s without verified live convergence')
            if not controller and first is not None and now-first >= 10:
                report['status'] = 'CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION'
                report['recording_elapsed_s'] = now-first
                break
            if cv2.waitKey(1) & 255 in (27, ord('q'), ord('Q')):
                raise RuntimeError('operator_aborted')
            with lock:
                snapshot = dict(latest)
            if set(snapshot) != set(SERIALS):
                continue
            frames = dict(ir_left=snapshot['ego'][0].get_infrared_frame(1),
                          ir_right=snapshot['ego'][0].get_infrared_frame(2),
                          umi_color=snapshot['umi'][0].get_color_frame())
            if not all(frames.values()):
                continue
            if not controller and first is None and now >= ready:
                first = now
                print('RECORDING: 10 seconds; keep both cameras, board and mount static. No R needed.', flush=True)
            save = first is not None and now-saved_at >= .5
            preview = now-preview_at >= .125
            if not (save or preview):
                continue
            arrays = {r: np.asanyarray(f.get_data()).copy() for r,f in frames.items()}
            arrays['umi_color'] = cv2.cvtColor(arrays['umi_color'], cv2.COLOR_RGB2BGR)
            phase=None
            if controller:
                save=False
                if now>=ready and now-observation_at>=.5:
                    ir_metadata={}
                    if args.continuous_aba:
                        for role in ('ir_left','ir_right'):
                            ir_metadata[role]=frame_evidence(frames[role],rs)
                            report['last_ir_frame_metadata']=ir_metadata
                            if ir_expected is not None:check_frame_evidence(ir_metadata[role],ir_expected)
                    points,observation=observer.observe(arrays)
                    normals=None
                    if args.tilt_contrast and points is not None:
                        try:
                            normals=np.array([cb.fit_pose(tilt_objects,points[i],tilt_cal[k])[:3,2] for i,k in enumerate(STREAMS)])
                            observation['board_normals']=normals.tolist()
                        except (ValueError,cv2.error) as exc:
                            observation['board_orientation_failure']=str(exc)
                    age_ms=(time.monotonic_ns()-min(v[1] for v in snapshot.values()))/1e6
                    gap_ms=abs(snapshot['ego'][1]-snapshot['umi'][1])/1e6
                    if age_ms>250 or gap_ms>75:points=None
                    if args.continuous_aba and ir_expected is None:
                        elapsed=now-stream_start_request
                        if args.tilt_contrast and points is not None and ir_warmup_start is None:
                            ir_warmup_start=now
                            report['placement_ready_elapsed_s']=elapsed
                        message=('PLACE BOARD: need all36 in EACH view; PREVIEW ONLY' if args.tilt_contrast and ir_warmup_start is None
                                 else 'IR WARMUP: hold board still; exposure convergence')
                        chosen=warmup_choice(ir_warmup,now,ir_metadata,points is not None,elapsed)
                        report['ir_warmup_evidence']=ir_warmup.copy()
                        if chosen is not None:
                            ir_expected=apply_lock(ir_sensor,rs,chosen)
                            report['ir_lock']=dict(original=ir_original,expected=ir_expected)
                            setup['ir_lock']=report['ir_lock']
                            setup['ir_warmup_seconds']=elapsed
                            cb.write_json(out/'setup.json',setup);setup_hash=cb.digest(out/'setup.json')
                            ready=time.monotonic()+2
                            print('IR LOCK: live-frame exposure/gain locked and read back:',ir_expected,flush=True)
                        elif warmup_timed_out(now,stream_start_request,ir_warmup_start,args.tilt_contrast):raise RuntimeError('IR auto warmup failed to converge with all36 visible')
                        observation_at=now
                        print('IR WARMUP: no sampling yet; counts',{k:len(v['ids']) for k,v in observation.items() if isinstance(v,dict) and 'ids' in v},flush=True)
                        continue
                    phase,message=controller.update(now,points,normals)
                    if controller.state!=announced_state:
                        instructions={'WAIT_A':'等待姿态 A：只调整公共板，让三幅画面都完整看到36码，停稳后自动采。',
                               'RECORD_A':'正在采姿态 A：10秒内请保持不动。',
                               'WAIT_B':'姿态 A 已采完，相机没有停止！现在只移动/转动公共板，再停稳，保持两台相机不动。',
                               'RECORD_B':'正在采姿态 B：10秒内请保持不动。',
                               'WAIT_C':'姿态 B 已采完！只移动公共板，放回紫色A轮廓，停稳后自动采返回段C。相机不动。',
                               'RECORD_C':'正在采返回 A 的对照段 C：10秒内请保持不动。',
                               'DONE':'各段已采完，正在停止两台相机；还需离线检验。'}
                        if args.tilt_contrast:instructions['WAIT_B']='A已采完！两相机不动，只把板的一侧向前/后转约15–20度；保持36码共视。角度>=12度并停稳后自动采B。'
                        print(instructions[controller.state],flush=True)
                        announced_state=controller.state
                    observation.update(host_age_ms=age_ms,host_gap_ms=gap_ms)
                    if args.continuous_aba:observation['ir_metadata']=ir_metadata
                    observation_at=now
                    print(message,'COUNTS',{k:len(v['ids']) for k,v in observation.items() if isinstance(v,dict) and 'ids' in v},flush=True)
                    if phase:
                        if first is None:first=now
                        save=True
                    if controller.state=='DONE':
                        report['status']='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION'
                        report['recording_elapsed_s']=now-first
                        break
            if preview:
                tiles = []
                for name in STREAMS:
                    tile = arrays[name] if name == 'umi_color' else cv2.cvtColor(arrays[name], cv2.COLOR_GRAY2BGR)
                    tile = cv2.resize(tile, (480,270))
                    cv2.putText(tile, name, (10,25), 0,.7,(0,255,255),2)
                    if controller:
                        count=len(observation.get(name,{}).get('ids',[])) if observation else 0
                        cv2.putText(tile,f'{controller.state} tags {count}/36',(10,55),0,.6,(0,255,0) if count==36 else (0,0,255),2)
                        if controller.state=='WAIT_C' and controller.reference is not None:
                            ref=controller.reference[STREAMS.index(name)]
                            for quad in ref.reshape(36,4,2):
                                cv2.polylines(tile,[np.rint(quad*.375).astype(np.int32)],True,(255,0,255),1)
                            cv2.putText(tile,'RETURN BOARD TO PURPLE A',(10,80),0,.5,(255,0,255),1)
                    tiles.append(tile)
                canvas=np.hstack(tiles)
                if args.tilt_contrast:
                    footer=np.zeros((45,canvas.shape[1],3),np.uint8)
                    cv2.putText(footer,message,(10,28),0,.65,(0,255,255),1)
                    canvas=np.vstack([canvas,footer])
                cv2.imshow(WINDOW, canvas)
                preview_at = now
            if save:
                row = dict(sample=len(rows), setup_sha256=setup_hash, streams={},
                    host_arrival_gap_ms=abs(snapshot['ego'][1]-snapshot['umi'][1])/1e6)
                if controller:row.update(phase=phase,live_observation=observation)
                for name, frame in frames.items():
                    stamp, count, domain = frame.get_timestamp(), frame.get_frame_number(), str(frame.get_frame_timestamp_domain())
                    arrival = snapshot['umi' if name == 'umi_color' else 'ego'][1]
                    old = previous.get(name)
                    if not np.isfinite(stamp) or (old and (stamp<=old[0] or count<=old[1] or domain!=old[2] or arrival<=old[3])):
                        raise ValueError('non-increasing frame metadata: '+name)
                    previous[name] = stamp,count,domain,arrival
                    path = out/f'{len(rows):03d}_{name}.png'
                    if not cv2.imwrite(str(path), arrays[name]):
                        raise RuntimeError('PNG write failed')
                    row['streams'][name] = dict(file=path.name, sha256=cb.digest(path), sdk_timestamp_ms=stamp,
                        sdk_timestamp_domain=domain, frame_number=count, host_arrival_monotonic_ns=arrival)
                    if args.continuous_aba and name in ir_metadata:
                        row['streams'][name]['ir_metadata']=ir_metadata[name]
                handle.write(json.dumps(row)+'\n')
                handle.flush()
                rows.append(row)
                saved_at = now
                print(f'SAVED {len(rows)} triples; phase={phase or "single"}; host arrival gap {row["host_arrival_gap_ms"]:.2f}ms', flush=True)
        if report['status'] == 'INCOMPLETE':
            raise RuntimeError('capture_timeout')
    except (Exception, KeyboardInterrupt) as exc:
        report['failure'] = type(exc).__name__+':'+str(exc)
        report['traceback'] = traceback.format_exc()
    finally:
        if ir_sensor is not None and ir_expected is not None:
            try:report['ir_final_readback']=verify_lock(ir_sensor,rs,ir_expected)
            except Exception as exc:report['cleanup_errors'].append('ir_final_readback:'+str(exc))
        for role, pipe in pipes.items():
            try:
                pipe.stop()
            except Exception as exc:
                report['cleanup_errors'].append(role+':'+str(exc))
        if ir_sensor is not None and ir_original is not None:
            try:
                restore_settings(ir_sensor,rs,ir_original)
                report['ir_original_settings_restored']=True
            except Exception as exc:report['cleanup_errors'].append('ir_restore:'+str(exc))
        if handle:
            try:
                handle.close()
            except Exception as exc:
                report['cleanup_errors'].append('index:'+str(exc))
        try:
            cv2.destroyAllWindows()
        except Exception as exc:
            report['cleanup_errors'].append('preview:'+str(exc))
        report.update(samples=len(rows), hashes={p.name: cb.digest(p) for p in
            (out/'setup.json', out/'frames.jsonl') if p.exists()})
        if controller:report.update(phase_state=controller.state,phase_events=controller.events,
            phase_samples={k:sum(r.get('phase')==k for r in rows) for k in controller.phases},continuous_stream=True)
        cb.write_json(out/'capture_report.json', report)
        print('STOPPED:', json.dumps(report), flush=True)
    return 0 if report['status'].startswith('CAPTURE_COMPLETE') and not report['cleanup_errors'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
