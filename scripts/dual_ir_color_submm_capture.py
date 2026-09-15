#!/usr/bin/env python3
"""Dual-device IR+color co-capture for the sub-mm mount-z experiment.

Streams (per sampled frame, all six):
  ego D435i : color RGB8 + ir_left Y8 + ir_right Y8 (emitter OFF)
  umi D405  : color YUYV->gray + ir_left Y8 + ir_right Y8 (manual exposure)

Protocol: devices and UMI-back mount tag stay RIGID for the whole session.
The 6x6 board is placed static per pose; R records 10s@2Hz per pose (up to 3
poses with distinct board placements/tilts), Q aborts.  Preview tiles show
per-stream board/mount detection counts so the operator can place the board
before recording.

Why this capture dissolves the remaining few mm: the board's 3D is
triangulated in-session by the UMI factory IR stereo (metric anchor, no
print dependence, no ego-IR scale error), the ego-IR inflation factor is
solved in-session as a similarity between the two devices' board
triangulations, and the mount tag is averaged over hundreds of ego-color
frames with the accepted k1/k2 model.  Nothing here activates any
calibration; factory values are only read and snapshotted.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.pair_color_tag_capture import _find_profile, _intrinsics_dict, _decode_color
from scripts.submm_detection import (make_board_detector_opencv, detect_board_union,
                                     detect_mount_ladder, reject_board_id1)
from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics

EGO_SERIAL = '327122078613'
UMI_SERIAL = '260322279785'
STREAMS = ('ego.color', 'ego.ir_left', 'ego.ir_right', 'umi.color', 'umi.ir_left', 'umi.ir_right')
IR_STREAMS = ('ego.ir_left', 'ego.ir_right', 'umi.ir_left', 'umi.ir_right')
POSES = 3
POSE_SECONDS = 12.0
SAMPLE_HZ = 5.0
WINDOW = 'SUB-MM CAPTURE | R record pose | T retune | Q abort'


def main(argv=None):
    import pyrealsense2 as rs

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--umi-exposure-us', type=float, default=30000.0)
    parser.add_argument('--umi-gain', type=float, default=96.0)
    parser.add_argument('--poses', type=int, default=POSES,
                        help='pose count; use 1 for a supplemental single-pose session')
    args = parser.parse_args(argv)
    poses_target = args.poses
    session = args.session.resolve()
    if session.exists():
        raise FileExistsError(f'session exists: {session}')
    session.mkdir(parents=True)

    context = rs.context()
    devices = {d.get_info(rs.camera_info.serial_number): d for d in context.query_devices()}
    if EGO_SERIAL not in devices or UMI_SERIAL not in devices:
        raise RuntimeError(f'devices missing: ego={EGO_SERIAL in devices} umi={UMI_SERIAL in devices}')
    ego_device, umi_device = devices[EGO_SERIAL], devices[UMI_SERIAL]

    # --- ego profiles (single pipeline, 3 streams) ---
    ego_cfg = rs.config()
    ego_cfg.enable_device(EGO_SERIAL)
    ego_cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    ego_cfg.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
    ego_cfg.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)
    ego_pipe = rs.pipeline(context)
    ego_profile = ego_pipe.start(ego_cfg)
    ego_dev = ego_profile.get_device()
    if ego_dev.get_info(rs.camera_info.serial_number) != EGO_SERIAL:
        raise ValueError('wrong ego device')
    ego_sensor = ego_dev.first_depth_sensor()
    ego_sensor.set_option(rs.option.emitter_enabled, 0)
    if ego_sensor.get_option(rs.option.emitter_enabled) != 0:
        raise ValueError('ego emitter off not confirmed')
    ego_color_sensor = ego_dev.first_color_sensor()
    ep = dict(color=ego_profile.get_stream(rs.stream.color).as_video_stream_profile(),
              ir_left=ego_profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile(),
              ir_right=ego_profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile())

    # --- umi profiles (sensor.open, 3 streams on the depth sensor) ---
    umi_sensor = umi_device.first_depth_sensor()
    umi_color_profile = _find_profile(umi_sensor, rs, rs.stream.color)
    umi_ir1_profile = _find_profile(umi_sensor, rs, rs.stream.infrared, 1)
    umi_ir2_profile = _find_profile(umi_sensor, rs, rs.stream.infrared, 2)
    try:
        umi_sensor.set_option(rs.option.emitter_enabled, 0.0)
        if umi_sensor.get_option(rs.option.emitter_enabled) != 0.0:
            raise ValueError('umi emitter off not confirmed')
    except RuntimeError:
        pass  # D405 exposes no emitter option (no projector); nothing to disable
    umi_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
    umi_sensor.set_option(rs.option.exposure, args.umi_exposure_us)
    umi_sensor.set_option(rs.option.gain, args.umi_gain)
    umi_sensor.open([umi_color_profile, umi_ir1_profile, umi_ir2_profile])
    umi_queue = rs.frame_queue(64, keep_frames=True)
    umi_sensor.start(umi_queue)

    # --- startup auto-tune: pick the umi exposure/gain that maximizes the
    # UMI IR streams' board counts (they respond to exposure choice), with the
    # ego IR pair as tie-break.  ego.ir_right is a known soft-focus lens and
    # must NOT hold the whole decision hostage.  Scoring uses the same robust
    # detection ladder as the analysis, sampled over 3 frames per candidate.
    board_detector = make_board_detector_opencv()
    ir_board = lambda g: len(detect_board_union(board_detector, g))

    def score_current(tag):
        best_counts = None
        for _ in range(3):  # 3 independent samplings; per-stream max
            ego_frames = {}
            latest = {}
            for _ in range(90):
                try:
                    fs = ego_pipe.wait_for_frames(200)
                except RuntimeError:
                    continue
                ego_frames = dict(color=fs.get_color_frame(), ir_left=fs.get_infrared_frame(1),
                                  ir_right=fs.get_infrared_frame(2))
                drain_umi_queue(umi_queue, rs, latest)
            if not (all(ego_frames.values()) and
                    all(k in latest for k in ('umi.color', 'umi.ir_left', 'umi.ir_right'))):
                return None
            eg = lambda f: np.asanyarray(f.get_data())
            eg_color = cv2.cvtColor(eg(ego_frames['color']), cv2.COLOR_RGB2BGR)
            eg_gray = cv2.cvtColor(eg_color, cv2.COLOR_BGR2GRAY)
            board_color = detect_board_union(board_detector, eg_gray)
            mount_color = len(reject_board_id1(detect_mount_ladder(eg_gray), board_color))
            clip_frac = float((eg_gray >= 250).mean())
            counts = dict(
                ego_ir_left=ir_board(eg(ego_frames['ir_left'])),
                ego_ir_right=ir_board(eg(ego_frames['ir_right'])),
                umi_ir_left=ir_board(latest['umi.ir_left'][0]),
                umi_ir_right=ir_board(latest['umi.ir_right'][0]),
                ego_color_mount=mount_color,
                color_clip=round(clip_frac, 5),
            )
            best_counts = counts if best_counts is None else {k: max(v, counts[k]) for k, v in best_counts.items()}
        umi_floor = min(best_counts['umi_ir_left'], best_counts['umi_ir_right'])
        ego_floor = min(best_counts['ego_ir_left'], best_counts['ego_ir_right'])
        unclipped = 1 if best_counts['color_clip'] <= 0.01 else 0
        print(f"tune {tag}: ir={ {k: v for k, v in best_counts.items() if k not in ('ego_color_mount', 'color_clip')} } "
              f"mount={best_counts['ego_color_mount']} clip={best_counts['color_clip']} "
              f"umi_floor={umi_floor} ego_floor={ego_floor}", flush=True)
        return (umi_floor, best_counts['ego_color_mount'], unclipped, ego_floor)

    def set_opt(sensor, option, value, what):
        # USB control transfers on the D405 can transiently stall
        # (RS2_USB_STATUS_PIPE) when options are poked right after start;
        # back off and retry instead of aborting the whole capture.
        for attempt in range(5):
            try:
                sensor.set_option(option, value)
                return
            except RuntimeError as exc:
                if attempt == 4:
                    raise RuntimeError(f'set_option failed on {what}: {exc}') from exc
                time.sleep(0.3 * (attempt + 1))

    def drain_stale():
        stash = {}
        drain_umi_queue(umi_queue, rs, stash)

    CANDIDATES = (
        # label, umi_exposure_us, umi_gain, ego_manual_or_None
        # ego_manual None = leave both ego auto-exposures alone; a tuple fixes
        # (ir_exposure_us, ir_gain, color_exposure, color_gain) where color
        # exposure is in the RGB sensor's 100us units (50 = 5ms); each color
        # field None leaves that sensor's AE on.
        # Scoring: (umi_floor, mount, unclipped_color, ego_floor) -- the mount
        # tag decides manual-vs-auto only when it actually decodes; otherwise
        # an unclipped auto-exposure frame beats a locked-too-dark manual one.
        ('umi30k/g96 ego-auto', 30000.0, 96.0, None),
        ('umi20k/g32 ego-auto', 20000.0, 32.0, None),
        ('umi20k/g32 ego-ir5ms-colorAE', 20000.0, 32.0, (5000.0, 16.0, None, None)),
        ('umi20k/g32 ego-lit5ms', 20000.0, 32.0, (5000.0, 16.0, 50.0, 64.0)),
        ('umi20k/g32 ego-lit10ms', 20000.0, 32.0, (10000.0, 24.0, 80.0, 64.0)),
        ('umi30k/g96 ego-dark33ms', 30000.0, 96.0, (33333.0, 32.0, None, None)),
    )

    def apply_ego(mode):
        if mode is None:
            set_opt(ego_sensor, rs.option.enable_auto_exposure, 1.0, 'ego ir ae on')
            set_opt(ego_color_sensor, rs.option.enable_auto_exposure, 1.0, 'ego color ae on')
        else:
            ir_exp, ir_gn, color_exp, color_gn = mode
            set_opt(ego_sensor, rs.option.enable_auto_exposure, 0.0, 'ego ir ae off')
            set_opt(ego_sensor, rs.option.exposure, ir_exp, 'ego ir exposure')
            set_opt(ego_sensor, rs.option.gain, ir_gn, 'ego ir gain')
            if color_exp is None:
                set_opt(ego_color_sensor, rs.option.enable_auto_exposure, 1.0, 'ego color ae on')
            else:
                set_opt(ego_color_sensor, rs.option.enable_auto_exposure, 0.0, 'ego color ae off')
                set_opt(ego_color_sensor, rs.option.exposure, color_exp, 'ego color exposure')
                set_opt(ego_color_sensor, rs.option.gain, color_gn, 'ego color gain')

    def run_auto_tune(setup_box=None):
        best = None
        for label, umi_exp, umi_gn, ego_manual in CANDIDATES:
            try:
                set_opt(umi_sensor, rs.option.exposure, umi_exp, f'{label} exposure')
                set_opt(umi_sensor, rs.option.gain, umi_gn, f'{label} gain')
                apply_ego(ego_manual)
            except RuntimeError as exc:
                print(f'tune {label}: SKIP ({exc})', flush=True)
                continue
            time.sleep(1.2)  # exposure/gain settle
            drain_stale()  # drop frames captured at the previous settings
            s = score_current(label)
            if s is not None and (best is None or s > best[0]):
                best = (s, label, umi_exp, umi_gn, ego_manual)
        if best is None:
            raise RuntimeError('auto-tune found no frames')
        _, label, umi_exp, umi_gn, ego_manual = best
        set_opt(umi_sensor, rs.option.exposure, umi_exp, 'winner exposure')
        set_opt(umi_sensor, rs.option.gain, umi_gn, 'winner gain')
        apply_ego(ego_manual)
        time.sleep(0.5)
        print(f'AUTO-TUNE winner: {label}', flush=True)
        if setup_box is not None:  # mid-session re-tune: keep setup.json honest
            setup_box['setup']['umi_manual_exposure_us'] = umi_exp
            setup_box['setup']['umi_manual_gain'] = umi_gn
            setup_box['setup']['auto_tune_winner'] = label
            setup_box['setup']['ego_ir_manual'] = ego_manual is not None
            cb.write_json(session / 'setup.json', setup_box['setup'])
            setup_box['hash'] = cb.digest(session / 'setup.json')
        return label, umi_exp, umi_gn, ego_manual, best[0]

    setup_box = {}
    label, umi_exp, umi_gn, ego_manual, tune_score = run_auto_tune()
    up = dict(color=umi_color_profile.as_video_stream_profile(),
              ir_left=umi_ir1_profile.as_video_stream_profile(),
              ir_right=umi_ir2_profile.as_video_stream_profile())

    def snap(p):
        return _intrinsics_dict(p.get_intrinsics())

    setup = dict(
        schema='ego.dual_ir_color_submm_capture.v1',
        factory_values_modified=False, activation='NOT_ACTIVATED',
        devices=dict(ego=dict(serial=EGO_SERIAL, intrinsics={r: snap(ep[r]) for r in ep}),
                     umi=dict(serial=UMI_SERIAL, intrinsics={r: snap(up[r]) for r in up})),
        extrinsics=dict(
            ego_ir_right_from_ir_left=serialize_sdk_extrinsics(ep['ir_left'].get_extrinsics_to(ep['ir_right'])),
            ego_color_from_ir_left=serialize_sdk_extrinsics(ep['ir_left'].get_extrinsics_to(ep['color'])),
            umi_ir_right_from_ir_left=serialize_sdk_extrinsics(up['ir_left'].get_extrinsics_to(up['ir_right'])),
            umi_color_from_ir_left=serialize_sdk_extrinsics(up['ir_left'].get_extrinsics_to(up['color'])),
        ),
        umi_manual_exposure_us=umi_exp, umi_manual_gain=umi_gn,
        auto_tune_winner=label, ego_ir_manual=ego_manual is not None,
        auto_tune_score_umi_floor_mount_unclipped_egofloor=list(tune_score) if tune_score else None,
        emitter_off=True, poses=poses_target, pose_seconds=POSE_SECONDS, sample_hz=SAMPLE_HZ,
        operator_contract='devices+mount rigid whole session; board static per pose; R records; Q aborts',
        timing='SDK global-time stamps per frame; cross-device alignment by timestamp, not sync cable',
        code_sha256=cb.digest(Path(__file__).resolve()),
    )
    cb.write_json(session / 'setup.json', setup)
    setup_box.update(setup=setup, hash=cb.digest(session / 'setup.json'))
    setup_hash = setup_box['hash']

    board_detector = make_board_detector_opencv()
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(WINDOW, np.zeros((180, 1440, 3), np.uint8))
    cv2.waitKey(1)

    def retune():
        return run_auto_tune(setup_box)

    poses = []
    report = dict(status='INCOMPLETE', activation='NOT_ACTIVATED', failure=None,
                  cleanup_errors=[], poses=[])
    try:
        for pose_index in range(poses_target):
            print(f'POSE {pose_index}: place board static, check detections, then R; T retune; Q aborts.', flush=True)
            recorded = preview_and_record(ego_pipe, umi_queue, board_detector,
                                          session, pose_index, setup_box, report,
                                          run_auto_tune=retune)
            poses.append(recorded)
            if pose_index + 1 < poses_target:
                print('Now MOVE the board to a new static placement/tilt.', flush=True)
                time.sleep(1.0)
        report['status'] = 'CAPTURE_COMPLETE'
    except Exception as exc:  # operator abort or hardware fault: keep partial evidence
        report['failure'] = f'{type(exc).__name__}: {exc}'
    finally:
        try:
            umi_sensor.stop()
            umi_sensor.close()
        except Exception as exc:
            report['cleanup_errors'].append(f'umi: {exc}')
        try:
            ego_pipe.stop()
        except Exception as exc:
            report['cleanup_errors'].append(f'ego: {exc}')
        cv2.destroyAllWindows()
        for pose in poses:
            try:
                frames_file = session / f'pose_{pose:03d}' / 'frames.jsonl'
                rows = [json.loads(t) for t in frames_file.read_text().splitlines()]
                for row in rows:
                    for stream in row['streams'].values():
                        stream['sha256'] = cb.digest(frames_file.parent / stream['file'])
                frames_file.write_text('\n'.join(json.dumps(r, sort_keys=True) for r in rows) + '\n')
            except Exception as exc:  # never let hashing mask the original failure
                report['cleanup_errors'].append(f'backfill pose {pose}: {exc}')
        report['hashes'] = dict(setup=setup_hash)
        cb.write_json(session / 'capture_report.json', report)
    print('STATUS', report['status'], 'failure', report['failure'], 'poses', poses, flush=True)
    return 0 if report['status'] == 'CAPTURE_COMPLETE' else 2


def _decode_color_bgr(frame):
    """YUYV payload -> BGR color (storage/display); detection still uses gray."""
    data = np.asanyarray(frame.get_data())
    if data.dtype == np.uint16 and data.size == 1280 * 720:
        yuyv = data.reshape(720, 1280).view(np.uint8).reshape(720, 1280, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
    if data.size == 1280 * 720 * 2:
        return cv2.cvtColor(data.reshape(720, 1280, 2), cv2.COLOR_YUV2BGR_YUY2)
    if data.size == 1280 * 720 * 3:
        return cv2.cvtColor(data.reshape(720, 1280, 3), cv2.COLOR_RGB2BGR)
    return cv2.cvtColor(data.reshape(720, 1280), cv2.COLOR_GRAY2BGR)


def drain_umi_queue(umi_queue, rs, latest):
    """The D405 low-level sensor API delivers individual frames, not framesets.

    The rig and board are static per pose, so per-stream latest-frame
    sampling is sufficient; each stream's own SDK timestamp is recorded.
    """
    for _ in range(64):
        try:
            f = umi_queue.poll_for_frame()
        except RuntimeError:
            break
        if not f:
            break
        p = f.get_profile()
        if p.stream_type() == rs.stream.color:
            latest['umi.color'] = (_decode_color_bgr(f), (f.get_timestamp(), f.get_frame_number(), str(f.get_frame_timestamp_domain())))
        elif p.stream_type() == rs.stream.infrared and p.stream_index() == 1:
            latest['umi.ir_left'] = (np.asanyarray(f.get_data()).copy(), (f.get_timestamp(), f.get_frame_number(), str(f.get_frame_timestamp_domain())))
        elif p.stream_type() == rs.stream.infrared and p.stream_index() == 2:
            latest['umi.ir_right'] = (np.asanyarray(f.get_data()).copy(), (f.get_timestamp(), f.get_frame_number(), str(f.get_frame_timestamp_domain())))


def preview_and_record(ego_pipe, umi_queue, board_detector, session, pose_index, setup_box, report, run_auto_tune=None):
    import pyrealsense2 as rs
    pose_dir = session / f'pose_{pose_index:03d}'
    pose_dir.mkdir()
    handle = (pose_dir / 'frames.jsonl').open('x')
    rows, first, last_saved, last_preview = [], None, -1.0, -1.0
    previous = {}
    latest = {}
    ready_at = time.monotonic() + 2.0
    deadline = ready_at + 600.0
    while time.monotonic() < deadline:
        try:
            fs = ego_pipe.wait_for_frames(1000)
        except RuntimeError:
            continue
        ego_frames = dict(color=fs.get_color_frame(), ir_left=fs.get_infrared_frame(1),
                          ir_right=fs.get_infrared_frame(2))
        if not all(ego_frames.values()):
            continue
        drain_umi_queue(umi_queue, rs, latest)
        if not all(k in latest for k in ('umi.color', 'umi.ir_left', 'umi.ir_right')):
            continue
        now = time.monotonic()
        arrays = dict(**{f'ego.{r}': np.asanyarray(f.get_data()).copy() for r, f in ego_frames.items()},
                      **{k: v[0] for k, v in latest.items()})
        arrays['ego.color'] = cv2.cvtColor(arrays['ego.color'], cv2.COLOR_RGB2BGR)
        metas = dict(**{f'ego.{r}': (f.get_timestamp(), f.get_frame_number(), str(f.get_frame_timestamp_domain()))
                        for r, f in ego_frames.items()},
                     **{k: v[1] for k, v in latest.items()})
        key = cv2.waitKey(1) & 255
        if key in (27, ord('q'), ord('Q')):
            handle.close()
            raise RuntimeError('operator_aborted')
        if first is None and key in (ord('t'), ord('T')) and run_auto_tune is not None:
            print('RE-TUNE: keep the board still and lit...', flush=True)
            try:
                run_auto_tune()
            except RuntimeError as exc:
                print(f'RE-TUNE failed: {exc}', flush=True)
        if first is None and now >= ready_at and key in (ord('r'), ord('R')):
            first = now
            print(f'RECORDING pose {pose_index}: keep still {POSE_SECONDS}s @ {SAMPLE_HZ}Hz.', flush=True)
        if now - last_preview >= 0.15:
            tiles = []
            for stream in STREAMS:
                arr = arrays[stream]
                gray = arr if arr.ndim == 2 else cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
                board = detect_board_union(board_detector, gray)
                canvas = arr if arr.ndim == 3 else cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                for q in board.values():
                    cv2.polylines(canvas, [np.round(q).astype(np.int32)], True, (0, 220, 0), 2)
                label = f'{stream}: board {len(board)}'
                if stream == 'ego.color':
                    mount = reject_board_id1(detect_mount_ladder(gray), board)
                    for q in mount:
                        cv2.polylines(canvas, [np.round(q).astype(np.int32)], True, (0, 140, 255), 2)
                    edge = 0.0
                    if mount:
                        q = np.asarray(mount[0], dtype=float).reshape(4, 2)
                        edge = float(np.linalg.norm(q - np.roll(q, 1, axis=0), axis=1).min())
                    label += f' mount {len(mount)} ({edge:.0f}px)'
                tile = cv2.resize(canvas, (480, 270))
                cv2.putText(tile, label, (8, 24), 0, 0.6, (0, 255, 255), 2)
                tiles.append(tile)
            view = np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])])
            state = 'PREVIEW' if first is None else f'RECORDING {POSE_SECONDS - (now - first):.0f}s'
            cv2.putText(view, f'pose {pose_index} | {state} | R record / Q abort', (8, 530), 0, 0.7, (50, 255, 50), 2)
            cv2.imshow(WINDOW, view)
            last_preview = now
        if first is not None and now - last_saved >= 1.0 / SAMPLE_HZ and now - first <= POSE_SECONDS:
            row = dict(sample=len(rows), setup_sha256=setup_box['hash'], streams={})
            for stream in STREAMS:
                stamp, count, domain = metas[stream]
                if not np.isfinite(stamp) or (stream in previous and (stamp < previous[stream][0] or count < previous[stream][1])):
                    handle.close()
                    raise RuntimeError(f'stamp regression on {stream}')
                previous[stream] = (stamp, count)
                path = Path(f'pose_{pose_index:03d}') / f'{stream}_{len(rows):04d}.png'
                ok = cv2.imwrite(str(pose_dir / path.name), arrays[stream])
                if not ok:
                    handle.close()
                    raise RuntimeError(f'imwrite failed: {path}')
                row['streams'][stream] = dict(file=path.name, sdk_timestamp_ms=stamp, frame_number=count,
                                              sdk_timestamp_domain=domain)
            handle.write(json.dumps(row, sort_keys=True) + '\n')
            handle.flush()
            rows.append(row)
            last_saved = now
        if first is not None and now - first > POSE_SECONDS:
            break
    handle.close()
    report['poses'].append(dict(pose=pose_index, samples=len(rows)))
    print(f'POSE {pose_index} done: {len(rows)} samples.', flush=True)
    return pose_index


if __name__ == '__main__':
    raise SystemExit(main())
