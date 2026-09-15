#!/usr/bin/env python3
"""Guided common AprilGrid capture: preview first, R per static window, Q stop."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import time

import cv2
import numpy as np

from scripts.common_board_detector import make_board_detector
from scripts.common_board_placement import PlacementCheck
from scripts.pair_color_tag_capture import _intrinsics_dict
from scripts.common_board_calibration import (
    SCHEMA, TARGET, MOUNT_TARGET, GATES, STEPS, new_output, write_json,
    digest, mount_detector, detect_sample, window_quality, load_session,
)

DEFAULT_EGO_SERIAL = '327122078613'
DEFAULT_UMI_SERIAL = '260322279785'
DEVICES = {'ego': DEFAULT_EGO_SERIAL, 'left': DEFAULT_UMI_SERIAL}  # Legacy public default.
WINDOW = 'COMMON BOARD | EGO blue - UMI green | R sample - Q stop'
COLORS = {'ego': (255, 160, 0), 'left': (0, 220, 0), 'right': (0, 220, 0)}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--ego-serial', default=DEFAULT_EGO_SERIAL)
    parser.add_argument('--umi-role', choices=('left','right'), default='left')
    parser.add_argument('--umi-serial', default=DEFAULT_UMI_SERIAL)
    parser.add_argument('--preview-hz', type=float, default=8.)
    parser.add_argument('--board-detector', choices=('opencv','umi-aprilgrid'), default='opencv')
    parser.add_argument('--placement-check-only', action='store_true', help='Check mount -> board -> mount before any16step calibration')
    parser.add_argument('--simultaneous',action='store_true',help='Ego sees board AND mount in every frame; UMI sees board')
    parser.add_argument('--resume-from',type=Path,help='Stopped partial session after all12 board steps; original files read-only')
    parser.add_argument('--confirm-unchanged',action='store_true',help='Operator confirms cameras AND mount have not moved since source capture')
    args = parser.parse_args(argv)
    if not np.isfinite(args.preview_hz) or not 2 <= args.preview_hz <= 15:
        parser.error('preview-hz must be 2..15')
    if args.resume_from and not args.confirm_unchanged:
        parser.error('resume requires --confirm-unchanged; if cameras/mount moved, capture a new full setup')
    if args.resume_from and args.placement_check_only:
        parser.error('placement check is not a calibration continuation')
    if args.simultaneous and args.placement_check_only:
        parser.error('simultaneous capture does not use remove-board placement checks')
    try:
        args.output = new_output(args.output)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def instruction(slot,simultaneous=False):
    guide, kind, split = STEPS[slot]
    actions = ('板放在两幅画面的中部', '板稍向UMI画面左侧移动', '板在左侧再增加一些倾斜',
               '板稍向UMI画面右侧移动', '板稍向画面上方移动', '板在上方再增加一些倾斜',
               '板稍向画面下方移动', '板向左转约20度', '板向右转约20度',
               '板靠近一点并向上倾斜', '板远离一点并向下倾斜', '板在远处再换一个侧倾角度')
    action = actions[slot] if slot < 12 else '移走公共板，只露出UMI背码' if kind == 'mount' else '放回公共板，复核两台相机是否被碰动'
    chinese = ('两台相机都别动；只移动公共板，停稳后按 R。' if kind == 'board'
               else '两台相机都别动；移走公共板及外部散码，只让 Ego 看 UMI 背码，停稳后按 R。')
    detail = f'本步：{action}，保证两机共视。\n' if kind == 'board' else ''
    if simultaneous:
        from scripts.common_board_joint import STEPS as JOINT_STEPS
        guide=JOINT_STEPS[slot][0]
        action=actions[slot] if slot<12 else '保持板和背码都可见，静态复测' if kind=='mount' else '换回中间板姿态，最后复核'
        detail=f'本步：{action}。\n'
        chinese='公共板与背码一起留在Ego画面；UMI看板。只移动板，勿遮背码；两相机保持不动，停稳按R。'
    return f'STEP {slot+1}/{len(STEPS)} [{split}]: {guide}\n{detail}操作：{chinese} Q/ESC 退出并保留数据。'


def quality_text(quality):
    """OpenCV uses ASCII; terminal gets an actionable Chinese explanation."""
    reasons = quality['reasons']
    roles = quality['roles']
    names = {'ego':'Ego','left':'UMI-left','right':'UMI-right','mount':'Mount'}
    counts = '/'.join(f'{names[r]}:{v["min_anchor_tags"]}' for r,v in roles.items())
    drift = max((v['max_drift_px'] for v in roles.values()),default=0.)
    if not reasons:
        return (f'READY R | anchor {counts} | max drift {drift:.2f}/0.75px',
                f'稳定预检通过；共同跟踪码 {counts}，最大漂移 {drift:.2f}/0.75px。按 R 采当前一步。')
    explanations = []
    for reason in reasons:
        role = 'UMI' if reason.startswith(('left:','right:')) else 'Ego' if reason.startswith('ego:') else ''
        if 'need_unique_mount1_outside_board' in reason:
            message='公共板保持可见；让Ego在板旁看到唯一背码ID1，不要与板重叠；额外散码需遮住'
        elif 'mount_overlaps_board_outline' in reason:
            message='背码与公共板轮廓重叠或靠得太近；只调整板到背码旁边，不用移出视野'
        elif 'unmatched_external_id1' in reason:
            message='发现无法归属的额外ID1；保留公共板和UMI背码，遮住其他散码'
        elif 'board_identity_geometry' in reason:
            message='板码排列与既有6x6几何不符；检查完整板面、成像和额外散码'
        elif 'duplicate board ID' in reason:
            message='公共板出现ID1以外的重号冲突，黄色框标出；检查额外散码，公共板和背码仍可同框'
        elif reason.endswith('need_3_pairs_spanning_1s'):
            message = '稳定观察时长不足，请保持不动约1秒'
        elif reason.endswith('window_motion'):
            message = f'角点漂移 {drift:.2f}px 超过0.75px，请停稳；持续超限再检查成像'
        elif 'duplicate_board_id' in reason:
            message = f'{role}同编号码出现多处；黄色标出冲突。遮住背码/多余散码，不能把冲突当整板消失'
        elif 'coverage' in reason or 'need_12' in reason:
            message = f'{role}共同跟踪码不足或分布太窄，请让更多整块板进入画面'
        elif reason == 'arrival_gap':
            message = '两帧主机到达差超过75ms，请等待采集稳定'
        elif 'visibility_changed' in reason:
            message = f'{role}背码未持续可见，请保持完整露出'
        elif 'remove_board' in reason:
            message = f'{role}检测到背码之外的板码，请移走或遮住公共板'
        elif 'show_only_mount' in reason:
            message = 'Ego画面只保留UMI背码ID1，移走同编号外码'
        elif 'mount_edge' in reason:
            message = '背码小于60像素，请先调整可见尺寸'
        elif reason == 'operator_aborted':
            message = '操作者提前结束，本段不计入'
        else:
            message = reason
        explanations.append(message)
    state = 'WAIT' if 'need_3_pairs_spanning_1s' in reasons else 'HOLD'
    return (f'{state} | anchor {counts} | drift {drift:.2f}/0.75px | '+','.join(reasons),
            '；'.join(dict.fromkeys(explanations)))


def current_preview_quality(samples, current, kind, simultaneous=False):
    # R confirms the displayed image, including frames between the0.2s samples.
    window = samples if samples and samples[-1] is current else [*samples,current]
    return window_quality(window,kind,simultaneous=True) if simultaneous else window_quality(window,kind)


def main(argv=None):
    args = parse_args(argv)
    roles = ('ego', args.umi_role)
    devices = {'ego': args.ego_serial, args.umi_role: args.umi_serial}
    if not os.environ.get('DISPLAY'):
        raise RuntimeError('DISPLAY unavailable; no invisible capture')
    continuation, base = None, None
    initial_slot = 0
    if args.resume_from:
        print('VERIFY：只读校验已停止的前段原图和步骤，尚未打开相机。',flush=True)
        base,_,boards,mounts,hashes,_ = load_session(args.resume_from,allow_partial=True)
        if args.board_detector != base.get('board_detector','opencv'):
            raise ValueError('resume must use original board detector')
        if args.simultaneous != (base.get('observation_mode','sequential')=='simultaneous'):
            raise ValueError('resume must use original observation mode')
        initial_slot = len(boards)+len(mounts)
        if not 12 <= initial_slot < 16:
            raise ValueError('resume supported only after12 board steps and before completion')
        continuation = dict(session=str(args.resume_from.resolve()),source_hashes=hashes,
                            next_slot=initial_slot,operator_confirmed_unchanged=True)
        print(f'RESUME：保留前{initial_slot}步，新目录从第{initial_slot+1}步开始；最后仍需放回板复核。',flush=True)
    import pyrealsense2 as rs

    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    pipes, factory, cleanup = {}, {}, []
    setup_hash = None
    failure = None
    slot, attempt_count = initial_slot, 0
    placement = PlacementCheck() if args.placement_check_only else None
    placement_evidence = []
    handle = None
    current = None
    samples = []
    preview_samples = []
    last_meta = {}
    previous_handler = signal.getsignal(signal.SIGTERM)

    def stop(_signal, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)

    def finish_attempt(extra_reason=None):
        nonlocal current, samples, slot, attempt_count, preview_samples
        quality = window_quality(samples, current['kind'],simultaneous=True) if args.simultaneous else window_quality(samples, current['kind'])
        reasons = list(quality['reasons'])
        if extra_reason:
            reasons.append(extra_reason)
        current.update(accepted=not reasons, reasons=sorted(set(reasons)), window_quality=quality)
        handle.write(json.dumps(current, allow_nan=False)+'\n')
        handle.flush()
        attempt_count += 1
        if not reasons:
            slot += 1
            print(f'WINDOW SAVED {slot}/{len(STEPS)}：静态采样通过；尚不是标定精度通过。', flush=True)
        else:
            print('RETRY：本段未通过，原图已保留。'+quality_text(dict(quality,reasons=reasons))[1], flush=True)
        current, samples = None, []
        preview_samples = []
        if slot < len(STEPS):
            print(instruction(slot,args.simultaneous), flush=True)

    try:
        # Establish a functioning GUI before taking ownership of either camera.
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(WINDOW, np.zeros((180, 960, 3), np.uint8))
        cv2.waitKey(1)
        for role in roles:
            pipe, config = rs.pipeline(), rs.config()
            config.enable_device(devices[role])
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
            profile = pipe.start(config)
            pipes[role] = pipe
            device = profile.get_device()
            if device.get_info(rs.camera_info.serial_number) != devices[role]:
                raise RuntimeError('device identity mismatch')
            if role == 'ego':
                device.first_color_sensor().set_option(rs.option.enable_auto_exposure, 1)
                sensor = device.first_depth_sensor()
                if sensor.supports(rs.option.emitter_enabled):
                    sensor.set_option(rs.option.emitter_enabled, 0)
            else:
                sensor = device.first_depth_sensor()
                sensor.set_option(rs.option.enable_auto_exposure, 0)
                sensor.set_option(rs.option.exposure, 30000)
                sensor.set_option(rs.option.gain, 96)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            factory[role] = dict(serial=devices[role], intrinsics=_intrinsics_dict(intr),
                                 format='rgb8', configured_hz=30)
            print(f'OPEN: {role} {devices[role]} 1280x720@30', flush=True)
        if base is not None and factory != base['devices']:
            raise ValueError('resumed camera factory/profile differs from source')
        setup = dict(schema=SCHEMA, devices=factory, target=TARGET, mount_target=MOUNT_TARGET,
                     resume_from=continuation,
                     gates=GATES, plan=STEPS, created_host_monotonic_ns=time.monotonic_ns(),
                     factory_values_modified=False, activation='NOT_ACTIVATED',
                     umi_role=args.umi_role,
                     camera_settings=dict(ego_auto_exposure=True, ego_emitter=0,
                                          umi_auto_exposure=False, umi_exposure_us=30000, umi_gain=96),
                     timing='independent SDK clocks retained; host-arrival static proxy only, NOT exposure sync',
                     operator_contract='R confirms cameras fixed for whole setup; board rigid/flat; mount unchanged; hide duplicate physical ID1',
                     code_sha256={str(p): digest(p) for p in (Path(__file__).resolve(),
                         Path(__file__).with_name('common_board_calibration.py').resolve())},
                     selected_pair='middle raw pair of each accepted static window; all attempts retained')
        setup['board_detector'] = args.board_detector
        setup['placement_check_only'] = args.placement_check_only
        if args.simultaneous:
            from scripts.common_board_joint import POLICY,STEPS as JOINT_STEPS
            setup.update(observation_mode='simultaneous',identity_policy=POLICY,plan=JOINT_STEPS,
                         operator_contract='R confirms cameras fixed; Ego sees board+mount together; UMI sees board; board ID1 excluded upfront')
            p=Path(__file__).with_name('common_board_joint.py')
            setup['code_sha256'][str(p)]=digest(p)
        setup['code_sha256'].update({str(p):digest(p) for p in (
            Path(__file__).with_name('common_board_detector.py'),Path(__file__).with_name('common_board_placement.py'))})
        write_json(args.output/'setup.json', setup)
        setup_hash = digest(args.output/'setup.json')
        handle = (args.output/'attempts.jsonl').open('x')
        detector, tag_detector = make_board_detector(args.board_detector), mount_detector()
        print('READY：双画面已启动。Ego 蓝框，UMI 绿框；'+
              ('摆位检查，按 R 确认当前摆位，不是标定采集。' if placement else '按 R 采当前一步。'), flush=True)
        if args.simultaneous:
            print('同帧模式：Ego蓝色板框+紫色背码，UMI绿色板框；公共板ID1灰色仅显示，不参与求解。首次R之前可调整摆位，之后两相机固定。',flush=True)
        print(placement.instruction if placement else instruction(slot,args.simultaneous), flush=True)
        began, logged = time.monotonic(), 0.
        while slot < len(STEPS):
            tick = time.monotonic()
            if tick-began > (300 if placement else 1800) or attempt_count >= 90:
                raise RuntimeError('bounded_session_limit; preserve data and review placement')
            images, meta = {}, {}
            # Grab both frames BEFORE detection/PNG work; do not compare the
            # independent devices' hardware_clock epochs to manufacture sync.
            for role, pipe in pipes.items():
                frame = pipe.wait_for_frames(1000).get_color_frame()
                if not frame:
                    raise RuntimeError(role+':missing color frame')
                arrival = time.monotonic_ns()
                item = dict(frame_number=frame.get_frame_number(), sdk_timestamp_ms=frame.get_timestamp(),
                            sdk_timestamp_domain=str(frame.get_frame_timestamp_domain()), host_arrival_monotonic_ns=arrival)
                for key in ('frame_number', 'sdk_timestamp_ms', 'host_arrival_monotonic_ns'):
                    if not np.isfinite(item[key]) or item[key] <= last_meta.get((role, key), -1):
                        raise RuntimeError(role+':timestamp/counter regression')
                    last_meta[role, key] = item[key]
                if (role, 'domain') in last_meta and last_meta[role, 'domain'] != item['sdk_timestamp_domain']:
                    raise RuntimeError(role+':timestamp domain changed')
                last_meta[role, 'domain'] = item['sdk_timestamp_domain']
                images[role] = cv2.cvtColor(np.asanyarray(frame.get_data()), cv2.COLOR_RGB2GRAY)
                meta[role] = item
            kind = placement.kind if placement else STEPS[slot][1]
            display = {}
            detected, reasons = detect_sample(images, kind, detector, tag_detector, display=display,simultaneous=True) if args.simultaneous else detect_sample(images, kind, detector, tag_detector, display=display)
            arrivals = [meta[r]['host_arrival_monotonic_ns'] for r in roles]
            stamp = max(arrivals)/1e9
            gap = abs(arrivals[0]-arrivals[1])/1e6
            sample = dict(t=stamp, arrival_gap_ms=gap, detected=detected, reasons=reasons)
            if not preview_samples or stamp-preview_samples[-1]['t'] >= .2:
                preview_samples.append(sample)
                preview_samples = [s for s in preview_samples if stamp-s['t'] <= 1.4]
            if current is not None and (not samples or stamp-samples[-1]['t'] >= .2):
                n = len(samples)
                for role in roles:
                    filename = f'attempt_{attempt_count:03d}/{role}_{n:03d}.png'
                    path = args.output/filename
                    if not cv2.imwrite(str(path), images[role]):
                        raise RuntimeError('PNG write failed')
                    meta[role].update(file=filename, sha256=digest(path))
                current['pairs'].append(meta)
                samples.append(sample)
                if stamp-samples[0]['t'] >= 1.1:
                    finish_attempt()
                    if slot == len(STEPS):
                        break
            panels = []
            for role in roles:
                panel = cv2.cvtColor(images[role], cv2.COLOR_GRAY2BGR)
                raw = display.get(role,{}).get('raw',[]) if kind=='board' or args.simultaneous else []
                duplicates = display.get(role,{}).get('duplicate_ids',[]) if kind=='board' or args.simultaneous else []
                visible = [(int(d.tag_id),np.asarray(d.corners)) for d in raw] if raw else list(detected[role].items())
                for tag, quad in visible:
                    color = (0,220,255) if tag in duplicates else COLORS[role]
                    if args.simultaneous and tag==1:color=(160,160,160)
                    cv2.polylines(panel, [quad.astype(np.int32)], True, color, 2)
                    cv2.putText(panel, str(tag), tuple(quad[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
                if args.simultaneous and role=='ego':
                    for tag,quad in detected['_mount']['ego'].items():
                        cv2.polylines(panel,[quad.astype(np.int32)],True,(255,0,255),3)
                        cv2.putText(panel,'UMI MOUNT 1',tuple(quad[0].astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,0,255),2)
                panel = cv2.resize(panel, (800, 450))
                valid=len(detected['_board'][role]) if args.simultaneous else len(detected[role])
                cv2.putText(panel, f'{role.upper()} visible={len(visible)} valid={valid} dup={duplicates}',
                            (12,28), cv2.FONT_HERSHEY_SIMPLEX,.65,COLORS[role],2)
                panels.append(panel)
            canvas = np.vstack([np.hstack(panels), np.zeros((110, 1600, 3), np.uint8)])
            state = f'RECORDING {len(samples)} pairs' if current else ('PREVIEW ONLY | R confirm | Q exit' if placement else 'PREVIEW ONLY | R sample | Q exit')
            title = f'PLACEMENT {placement.step+1}/3 {kind.upper()} - R confirm fixed cameras' if placement else f'{slot+1}/{len(STEPS)} {setup["plan"][slot][0]}'
            cv2.putText(canvas, f'{title} | {state}', (12, 478), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1)
            if placement or (args.simultaneous and not current):
                quality = current_preview_quality(preview_samples,sample,kind,args.simultaneous)
            elif args.simultaneous:
                quality = window_quality(samples,kind,simultaneous=True)
            else:
                quality = window_quality(samples if current else preview_samples,kind)
            text, explanation = quality_text(quality)
            cv2.putText(canvas, text[:170], (12, 507), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,220,255) if quality['reasons'] else (0,255,0), 1)
            cv2.putText(canvas, f'host arrival gap {gap:.1f}ms (NOT hardware sync); static drift limit 0.75px', (12, 537), cv2.FONT_HERSHEY_SIMPLEX, .55, (210,210,210), 1)
            cv2.imshow(WINDOW, canvas)
            if tick-logged >= 5:
                label = f'PLACEMENT {placement.step+1}/3' if placement else f'LIVE {slot+1}/{len(STEPS)}'
                print(f'{label} {state}；{explanation}；到达差={gap:.1f}ms', flush=True)
                logged = tick
            key = cv2.waitKey(max(1, int(1000*(1/args.preview_hz-(time.monotonic()-tick))))) & 255
            if key in (ord('q'), ord('Q'), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                if current:
                    finish_attempt('operator_aborted')
                break
            if key in (ord('r'), ord('R')) and current is None:
                if placement:
                    step = placement.step
                    ok,message = placement.advance(quality,detected)
                    print(message,flush=True)
                    if not ok:
                        continue
                    for role in roles:
                        filename=f'placement_{step}_{role}.png'
                        if not cv2.imwrite(str(args.output/filename),images[role]):
                            raise RuntimeError('placement PNG write failed')
                        meta[role].update(file=filename,sha256=digest(args.output/filename))
                    placement_evidence.append(dict(step=step,kind=kind,quality=quality,streams=meta,
                        operator_confirmed_fixed_since_first=(step>0),
                        detected={r:{str(i):q.tolist() for i,q in ds.items()} for r,ds in detected.items()}))
                    preview_samples=[]
                    if placement.step==3:
                        break
                    continue
                if quality['reasons']:
                    print('提示：预检尚未通过，本次仍保留原图；'+explanation, flush=True)
                (args.output/f'attempt_{attempt_count:03d}').mkdir()
                current = dict(attempt=attempt_count, slot=slot, kind=STEPS[slot][1],
                               split=STEPS[slot][2], operator_confirmed_fixed_cameras=True, pairs=[])
                print(f'RECORDING STEP {slot+1}: keep still ~1.2s；失败会原地提示重试。', flush=True)
    except KeyboardInterrupt:
        failure = 'operator_interrupt'
    except Exception as exc:
        failure = f'{type(exc).__name__}: {exc}'
    finally:
        # Stop camera ownership before any failing disk finalization can prevent it.
        for role, pipe in pipes.items():
            try:
                pipe.stop()
            except Exception as exc:
                cleanup.append(role+':'+str(exc))
        if current is not None:
            try:
                finish_attempt('interrupted')
            except Exception as exc:
                cleanup.append('attempt_finalize:'+str(exc))
        if handle is not None:
            try:
                handle.close()
            except Exception as exc:
                cleanup.append('index_close:'+str(exc))
        try:
            cv2.destroyAllWindows()
        except Exception as exc:
            cleanup.append('preview_close:'+str(exc))
        signal.signal(signal.SIGTERM, previous_handler)
        completed = slot == len(STEPS) and failure is None and not cleanup
        report = dict(schema=SCHEMA, completed=completed, completed_steps=slot, attempts=attempt_count,
                      failure=failure, cleanup_errors=cleanup, activation='NOT_ACTIVATED',
                      status='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION' if completed else 'NOT_COMPLETED',
                      setup_sha256=setup_hash,
                      attempts_sha256=digest(args.output/'attempts.jsonl') if (args.output/'attempts.jsonl').exists() else None)
        if placement:
            report['placement_complete'] = placement.step==3 and failure is None and not cleanup
            report['status'] = 'PLACEMENT_CHECK_COMPLETE_NOT_CALIBRATION' if report['placement_complete'] else 'PLACEMENT_NOT_COMPLETE'
            report['placement_checks'] = placement_evidence
            report['limitation'] = 'Operator-fixed assumption; visibility and mount pixel closure do not independently prove both cameras did not move'
        write_json(args.output/'capture_report.json', report)
    print(f'{report["status"]}: {args.output}', flush=True)
    if failure or cleanup:
        print(f'BLOCKED: {failure}; cleanup={cleanup}', flush=True)
    if completed:
        print('NEXT：相机已停止；运行 common_board_calibration 求解，勿把采完当标定通过。', flush=True)
        print(f'PYTHONPATH=. /opt/three-device-slam/venv/bin/python scripts/common_board_calibration.py --session {args.output} --output {args.output}/solve', flush=True)
    return 0 if completed or report.get('placement_complete') or (slot == 0 and attempt_count == 0 and not failure and not cleanup) else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError, cv2.error) as exc:
        print(f'BLOCKED: {type(exc).__name__}: {exc}; no automatic retry.', flush=True)
        raise SystemExit(2)
