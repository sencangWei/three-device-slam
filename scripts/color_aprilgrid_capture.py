#!/usr/bin/env python3
"""Preview-first fixed-factory board diagnostic sampler, not SLAM capture.

R starts a bounded90s capture of independent color images at up to2Hz/camera.
Q/ESC closes and preserves partial evidence. Never fits K/D or activates calibration.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time
from types import SimpleNamespace

import cv2
import numpy as np

from scripts.pair_color_tag_capture import _intrinsics_dict

DEVICES = {'ego': '327122078613', 'left': '260322279785'}
WINDOW = 'EGO | LEFT UMI - AprilGrid: R record, Q stop'

POSE_GUIDANCE = ('CENTER - hold still', 'LEFT area - hold still', 'RIGHT area - hold still',
                 'UP area - hold still', 'DOWN area - hold still', 'TILT left - hold still',
                 'TILT right - hold still', 'NEAR/FAR - pause after moving')


def pose_guidance(elapsed, duration):
    return POSE_GUIDANCE[min(7, max(0, int(8 * elapsed / duration)))]


def next_stage(role, orientation):
    if role == 'ego' and orientation == 'normal':
        return 'EGO: rotate the BOARD180deg in its plane, keep printed face visible; await next preview.'
    if role == 'ego' and orientation == 'rotated180':
        return 'UMI: restore board normal orientation; await UMI preview.'
    if role == 'left' and orientation == 'normal':
        return 'UMI: rotate the BOARD180deg in its plane, keep printed face visible; await next preview.'
    return 'Review image quality and held-out tag prediction with FACTORY K/D FIXED. No recalibration.'


def make_detector():
    parameters = cv2.aruco.DetectorParameters()
    # Existing physical Kalibr board uses a two-bit black border, unlike
    # the separate one-bit external alignment tags. Never auto-mix layouts.
    parameters.markerBorderBits = 2
    # The legacy board has black junction squares touching the tag corners.
    # APRILTAG quad segmentation misses most tags on this board; contour
    # detection plus subpixel refinement preserves its separate tag quads.
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), parameters)


def detect_grid(detector, gray):
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    return [SimpleNamespace(tag_id=int(tag_id), corners=quad.reshape(4, 2))
            for tag_id, quad in zip(ids.flatten(), corners)]


def grid_summary(detections, width=1280, height=720):
    selected = [d for d in detections if 0 <= int(d.tag_id) < 36]
    ids = [int(d.tag_id) for d in selected]
    result = dict(tags=len(ids), ids=ids, good=False, reason='show_6x6_grid', center_cell=None)
    if len(ids) != len(set(ids)):
        return dict(result, reason='duplicate_ID_hide_old_tags')
    if len(ids) < 12:
        return result
    corners = np.array([d.corners for d in selected], dtype=float).reshape(-1, 4, 2)
    if not np.isfinite(corners).all():
        return dict(result, reason='invalid_corners')
    edge = float(np.linalg.norm(corners-np.roll(corners, 1, axis=1), axis=2).min())
    xy = corners.reshape(-1, 2)
    center = xy.mean(axis=0)
    cell = [int(np.clip(center[0]*3/width, 0, 2)), int(np.clip(center[1]*3/height, 0, 2))]
    area = float(np.prod(np.ptp(xy, axis=0)) / (width*height))
    # Preview-only lattice check: do not assume physical tag orientation or
    # metric spacing before the actual board has been verified offline.
    lattice = np.array([[tag_id % 6, tag_id // 6] for tag_id in ids], dtype=float)
    centers = corners.mean(axis=1)
    homography, _ = cv2.findHomography(lattice, centers, 0)
    consistent = False
    if homography is not None and np.isfinite(homography).all():
        projected = cv2.perspectiveTransform(lattice.reshape(-1, 1, 2), homography).reshape(-1, 2)
        consistent = bool(np.sqrt(np.mean(np.sum((projected-centers)**2, axis=1))) <= 3.)
    good = consistent and edge >= 15
    return dict(result, good=good, reason='grid_visible_not_calibration_acceptance' if good else 'check_size_or_grid_layout',
                min_tag_edge_px=edge, bbox_area_fraction=area, center_cell=cell)


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--preview-hz', type=float, default=5.)
    parser.add_argument('--role', choices=('both','ego','left'), default='both')
    parser.add_argument('--board-orientation', choices=('unspecified','normal','rotated180'), default='unspecified')
    args=parser.parse_args(argv)
    if not np.isfinite(args.duration) or not 1 <= args.duration <= 300:
        parser.error('duration must be1..300seconds')
    if not np.isfinite(args.preview_hz) or not 1 <= args.preview_hz <= 15:
        parser.error('preview-hz must be1..15')
    args.output=args.output.resolve()
    if not args.output.is_relative_to(Path(__file__).resolve().parents[1]/'artifacts'):
        parser.error('output must be inside this repository artifacts directory')
    return args


def main(argv=None):
    args=parse_args(argv)
    selected_role=getattr(args,'role','both')
    orientation=getattr(args,'board_orientation','unspecified')
    devices=DEVICES if selected_role=='both' else {selected_role:DEVICES[selected_role]}
    if not os.environ.get('DISPLAY'):
        raise RuntimeError('DISPLAY unavailable; no invisible capture')
    import pyrealsense2 as rs

    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    detector=make_detector()
    pipelines={};handles={};factory={};counts={role:0 for role in devices}
    good_counts=dict(counts);cells={role:set() for role in DEVICES}
    started=None;completed=False;failure=None
    last_saved={role:-1 for role in devices};last_frame={role:-1 for role in devices}
    setup_sha256=None;last_guidance=None
    def stop(_signum, _frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        for role,serial in devices.items():
            pipeline=rs.pipeline();config=rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color,1280,720,rs.format.rgb8,30)
            profile=pipeline.start(config);pipelines[role]=pipeline
            device=profile.get_device()
            actual=device.get_info(rs.camera_info.serial_number)
            if actual!=serial: raise RuntimeError('device identity mismatch')
            if role=='ego':
                device.first_color_sensor().set_option(rs.option.enable_auto_exposure,1)
                sensor=device.first_depth_sensor()
                if sensor.supports(rs.option.emitter_enabled):sensor.set_option(rs.option.emitter_enabled,0)
            else:
                sensor=device.first_depth_sensor()
                sensor.set_option(rs.option.enable_auto_exposure,0)
                sensor.set_option(rs.option.exposure,30000)
                sensor.set_option(rs.option.gain,96)
            intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            factory[role]=dict(serial=serial,intrinsics=_intrinsics_dict(intr),format='rgb8',configured_hz=30)
            (args.output/role).mkdir()
            print(f'PREVIEW: {role} {serial} opened1280x720@30; no recording yet.',flush=True)
        (args.output/'setup.json').write_text(json.dumps(dict(schema='ego.color_aprilgrid_capture.v1',
            devices=factory,target=dict(family='t36h11',marker_border_bits=2,rows=6,columns=6,nominal_tag_m=.0352,nominal_gap_m=.01056,
            physical_dimensions_verified=False),sample_hz=2.,duration_s=args.duration,
            scope='fixed-factory board prediction diagnostic; NEVER FIT K/D; not synchronized raw SLAM',
            selected_role=selected_role,board_orientation_operator_label=orientation,
            factory_values_modified=False,activation='NOT_ACTIVATED'),indent=2)+'\n')
        setup_sha256=hashlib.sha256((args.output/'setup.json').read_bytes()).hexdigest()
        cv2.namedWindow(WINDOW,cv2.WINDOW_AUTOSIZE)
        print(f'READY: preview only. Put the flat grid in view; R starts{args.duration:g}s, Q/ESC closes. Move slowly, pause at each pose.',flush=True)
        logged=0.
        while True:
            tick=time.monotonic();canvases=[];summaries={}
            if started is not None and tick-started>=args.duration:
                completed=True;break
            for role,pipeline in pipelines.items():
                frame=pipeline.wait_for_frames(1000).get_color_frame()
                arrived=time.monotonic_ns()
                if not frame:raise RuntimeError(f'{role}: missing color frame')
                frame_number=int(frame.get_frame_number())
                if frame_number<=last_frame[role]:raise RuntimeError(f'{role}: stale/regressed frame counter')
                last_frame[role]=frame_number
                rgb=np.asanyarray(frame.get_data()).copy()
                gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
                detections=detect_grid(detector,gray);summary=grid_summary(detections);summaries[role]=summary
                now=time.monotonic()
                if started is not None and now-started<args.duration and now-last_saved[role]>=.5:
                    filename=f'{counts[role]:06d}.png';path=args.output/role/filename
                    if not cv2.imwrite(str(path),gray):raise RuntimeError('PNG write failed')
                    record=dict(sample=counts[role],file=filename,frame_number=frame_number,
                        sdk_timestamp_ms=float(frame.get_timestamp()),sdk_timestamp_domain=str(frame.get_frame_timestamp_domain()),
                        host_arrival_monotonic_ns=arrived,preview_quality=summary,
                        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        setup_sha256=setup_sha256,board_orientation_operator_label=orientation)
                    handles[role].write(json.dumps(record)+'\n');handles[role].flush()
                    counts[role]+=1;good_counts[role]+=int(summary['good']);last_saved[role]=now
                    if summary['good']:cells[role].add(tuple(summary['center_cell']))
                canvas=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
                color=(0,220,0)if summary['good']else(0,180,255)
                for d in detections:
                    quad=np.asarray(d.corners).reshape(4,2).astype(np.int32)
                    cv2.polylines(canvas,[quad],True,color,2)
                    cv2.putText(canvas,str(int(d.tag_id)),tuple(quad.mean(axis=0).astype(int)),0,.6,color,2)
                canvas=cv2.resize(canvas,(800,450))
                cv2.rectangle(canvas,(0,0),(800,64),(20,20,20),-1)
                elapsed=0 if started is None else now-started
                state='PREVIEW ONLY - R:record'if started is None else f'RECORDING {elapsed:.0f}/{args.duration:.0f}s'
                cue='FIXED FACTORY K/D - no recording'if started is None else pose_guidance(elapsed,args.duration)
                cv2.putText(canvas,f'{role.upper()} | {state} | Q:stop',(10,22),0,.6,color,1)
                cv2.putText(canvas,f"tags {summary['tags']}/36 | saved {counts[role]} | cells {len(cells[role])}/9 | {summary['reason']}",(10,49),0,.43,color,1)
                cv2.rectangle(canvas,(0,425),(800,450),(20,20,20),-1)
                cv2.putText(canvas,f'{orientation} | {cue}',(10,443),0,.52,color,1)
                canvases.append(canvas)
            if started is not None:
                cue=pose_guidance(time.monotonic()-started,args.duration)
                if cue!=last_guidance:
                    print(f'ACTION: {cue}; keep entire board visible, pause2seconds. FACTORY K/D FIXED.',flush=True)
                    last_guidance=cue
            displayed=np.hstack(canvases);cv2.imshow(WINDOW,displayed)
            if tick-logged>=3:
                snapshot=dict(recording=started is not None,counts=counts,quality=summaries,
                              sampled_good_counts=good_counts,coverage_cells={r:sorted(v) for r,v in cells.items()},
                              physical_dimensions_verified=False,activation='NOT_ACTIVATED',
                              selected_role=selected_role,board_orientation_operator_label=orientation,
                              factory_values_modified=False,setup_sha256=setup_sha256)
                temp=args.output/'latest.tmp';temp.write_text(json.dumps(snapshot,indent=2)+'\n');temp.replace(args.output/'latest.json')
                cv2.imwrite(str(args.output/'preview.jpg'),displayed)
                print('RECORDING'if started is not None else'PREVIEW',json.dumps({r:dict(tags=s['tags'],reason=s['reason'],saved=counts[r])for r,s in summaries.items()}),flush=True)
                logged=tick
            key=cv2.waitKey(max(1,int(1000*(1/args.preview_hz-(time.monotonic()-tick)))))&255
            if key in (27,ord('q')):break
            if key in (ord('r'),ord('R')) and started is None:
                for r in devices:
                    handles[r]=(args.output/r/'images.jsonl').open('x')
                started=time.monotonic()
                print(f'RECORDING STARTED: {args.duration:.0f}s; move slowly through center/corners, near/far and tilt; all sampled frames retained.',flush=True)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        failure=f'{type(exc).__name__}:{exc}';raise
    finally:
        cleanup_errors=[]
        for role,handle in handles.items():
            try:handle.close()
            except Exception as exc:cleanup_errors.append(f'{role} close: {type(exc).__name__}:{exc}')
        for pipeline in pipelines.values():
            try:pipeline.stop()
            except Exception as exc:cleanup_errors.append(f'camera stop: {type(exc).__name__}:{exc}')
        try:cv2.destroyAllWindows()
        except Exception as exc:cleanup_errors.append(f'window close: {type(exc).__name__}:{exc}')
        if cleanup_errors:
            completed=False
            failure='; '.join(([failure] if failure else [])+cleanup_errors)
        report=dict(status='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION'if completed else'NOT_COMPLETED',
            started=started is not None,operator_aborted=started is not None and not completed and failure is None,
            failure=failure,counts=counts,good_counts=good_counts,coverage_cells={r:sorted(v)for r,v in cells.items()},
            physical_dimensions_verified=False,activation='NOT_ACTIVATED',
            selected_role=selected_role,board_orientation_operator_label=orientation,
            factory_values_modified=False,setup_sha256=setup_sha256)
        (args.output/'capture_report.json').write_text(json.dumps(report,indent=2)+'\n')
        print('CLOSED: camera ownership released. '+report['status'],flush=True)
        if completed:
            print('本段计时完成，图像质量尚待检查；不会修改出厂内参或畸变。',flush=True)
            print('NEXT (after image quality review): '+next_stage(selected_role,orientation),flush=True)
    return 0 if failure is None and (completed or started is None) else 2


if __name__=='__main__':raise SystemExit(main())
