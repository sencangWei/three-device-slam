#!/usr/bin/env python3
"""Independent new-board-view scoring of a frozen old IR-to-UMI transform."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_joint import board_geometry
from scripts.common_board_resampling_analysis import restore_board_id1
from scripts.ego_joint_stereo_diagnostic import diagnostic_targets
from scripts.ego_ir_umi_diagnostic import source, triangulate, GATES
from scripts.ego_stereo_color_validate import transform


def fixed_score(points, pixels, calibration, frozen):
    cb.validate_transform(frozen)
    return cb.errors(cb.project(points,frozen,calibration),pixels)


def same_geometry(old,new):
    for key in ('schema','serials','intrinsics','right_from_left','native_fps'):
        if old[key]!=new[key]:raise ValueError('camera geometry changed: '+key)


def validate_gates(config):
    if (config['projection_p95_gate_px']!=GATES['projection_p95_px'] or
        config['mount_static_drift_px']!=GATES['static_drift_px']):
        raise ValueError('frozen gates disagree with evaluator')


def observe(session, row, detector, mount_detector):
    board={};mount={};proof={};failures=[]
    for role,meta in row['streams'].items():
        gray=cv2.imread(str(session/meta['file']),0)
        if gray is None or gray.shape!=(720,1280):raise ValueError('image decode/shape')
        try:
            raw=cb.detect_grid(detector,gray)
            if role=='umi_color':
                ds,_,p95=board_geometry(raw);proof[role]=dict(board_h_p95_px=p95)
            else:
                ds,mount[role],proof[role]=diagnostic_targets(raw,cb.detect_grid(mount_detector,gray))
                if not proof[role]['mount_size_pass']:failures.append(role+':mount_edge_below_60px')
            ds[1],proof[role]['board_id1']=restore_board_id1(raw)
            board[role]=cb.refine_legacy_corners(gray,ds)
            if set(ds)!=set(range(36)):failures.append(role+':missing_board_ids')
        except ValueError as exc:failures.append(role+':'+str(exc))
    return board,mount,proof,failures


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--reference-session',type=Path,required=True)
    p.add_argument('--frozen',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=cb.new_output(args.output)
    setup,capture,rows=source(args.session)
    old,old_capture,old_rows=source(args.reference_session);same_geometry(old,setup)
    config=json.loads(args.frozen.read_text());reference=Path(config['reference_report'])
    validate_gates(config)
    if cb.digest(reference)!=config['reference_report_sha256'] or config['source_hashes']!=old_capture['hashes']:
        raise ValueError('frozen binding mismatch')
    report=json.loads(reference.read_text())
    if report['source_hashes']!=old_capture['hashes'] or config['method']!='direct_mean' or config['group']!='all36':
        raise ValueError('unexpected reference method/source')
    frozen=np.array(config['T_Ucolor_Eir'])
    if not np.array_equal(frozen,report['fits']['all36']['direct_mean']['fit']['T_Ucolor_Eir']):
        raise ValueError('frozen transform mismatch')
    cal={k:cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],frame_id=k,intrinsics=v)
         for k,v in setup['intrinsics'].items()}
    right=transform(setup['right_from_left'])
    detector=make_board_detector('umi-aprilgrid');md=cb.mount_detector()
    prior,prior_mount,_,_=observe(args.reference_session,old_rows[0],detector,md)
    first={};frames=[]
    for row in rows:
        board,mount,proof,failures=observe(args.session,row,detector,md)
        lm,rm,um=[row['streams'][k] for k in ('ir_left','ir_right','umi_color')]
        skew=abs(lm['sdk_timestamp_ms']-rm['sdk_timestamp_ms'])
        gap=abs(lm['host_arrival_monotonic_ns']-um['host_arrival_monotonic_ns'])/1e6
        if lm['sdk_timestamp_domain']!=rm['sdk_timestamp_domain'] or skew>GATES['ir_skew_ms']:failures.append('ir_timestamp')
        if gap>GATES['host_arrival_gap_ms']:failures.append('host_arrival_gap')
        frame=dict(sample=row['sample'],source=row,failures=failures,identity=proof,ir_skew_ms=skew,host_arrival_gap_ms=gap,
            detected_ids={k:sorted(v) for k,v in board.items()},static_drift_px={},mount_reference_drift_px={},board_novelty={})
        for role,ds in board.items():
            base=first.setdefault(role,ds)
            if set(base)!=set(ds):failures.append(role+':visibility_changed')
            common=sorted(set(base)&set(ds))
            drift=max(float(np.linalg.norm(ds[i]-base[i],axis=1).max()) for i in common) if common else None
            frame['static_drift_px'][role]=drift
            if drift is None or drift>GATES['static_drift_px']:failures.append(role+':static_drift')
            if role in mount:
                delta=float(np.linalg.norm(mount[role]-prior_mount[role],axis=1).max())
                frame['mount_reference_drift_px'][role]=delta
                if delta>GATES['static_drift_px']:failures.append(role+':mount_projection_changed_since_reference')
            common_old=sorted(set(ds)&set(prior[role]))
            objects=np.concatenate([cb.object_corners(i) for i in common_old])
            a=cb.fit_pose(objects,np.concatenate([prior[role][i] for i in common_old]),cal[role])
            b=cb.fit_pose(objects,np.concatenate([ds[i] for i in common_old]),cal[role])
            shift=float(np.linalg.norm(a[:3,3]-b[:3,3])*1000)
            angle=float(np.degrees(np.arccos(np.clip(a[:3,2]@b[:3,2],-1,1))))
            frame['board_novelty'][role]=dict(origin_translation_mm=shift,normal_deg=angle)
            if shift<config['board_novel_center_mm'] and angle<config['board_novel_normal_deg']:
                failures.append(role+':board_pose_not_novel')
        if set(board)==set(cal):
            ids=sorted(set.intersection(*(set(v) for v in board.values())))
            frame['common_ids']=ids
            if len(ids)>=12 and 'ir_timestamp' not in failures:
                pixels={k:np.concatenate([v[i] for i in ids]) for k,v in board.items()}
                xyz,scores=triangulate(pixels['ir_left'],pixels['ir_right'],cal['ir_left'],cal['ir_right'],right)
                error=fixed_score(xyz,pixels['umi_color'],cal['umi_color'],frozen)
                frame.update(frozen_score=error,stereo_scores=scores,points_ir_m=xyz.tolist(),pixels={k:v.tolist() for k,v in pixels.items()})
                if error['p95_px']>GATES['projection_p95_px']:failures.append('frozen_projection')
                if any(v['p95_px']>GATES['projection_p95_px'] for v in scores.values()):failures.append('stereo_projection')
            else:failures.append('insufficient_common_ids_or_timing')
        else:failures.append('missing_stream_observations')
        frames.append(frame)
        print('FRAME',row['sample'],'common',len(frame.get('common_ids',[])),'frozen',frame.get('frozen_score'),'failures',failures,flush=True)
    scores=[f['frozen_score']['p95_px'] for f in frames if 'frozen_score' in f]
    summary=dict(frames=len(frames),scored=len(scores),failed_frames=sum(bool(f['failures']) for f in frames),
        frozen_p95_px_median=float(np.median(scores)) if scores else None,frozen_p95_px_max=max(scores) if scores else None)
    out.mkdir()
    cb.write_json(out/'report.json',dict(status='PASS_FIXED_TRANSFORM_VIEW_ONLY' if scores and len(scores)==len(frames) and not any(f['failures'] for f in frames) else 'FAIL',
        activation='NOT_ACTIVATED',summary=summary,frames=frames,frozen=config,gates=GATES,
        source_hashes=capture['hashes'],reference_source_hashes=old_capture['hashes'],frozen_sha256=cb.digest(args.frozen),
        code_sha256=cb.digest(Path(__file__).resolve()),
        limitations=['Missing tags remain FAIL; visible intersections scored diagnostically, no residual pruning',
            'No fit of X/K/D on new images; board-only PnP used exclusively for novelty, never prediction',
            'Mount image stability is a relative-motion proxy, not proof of unchanged entire setup',
            'One changed board pose is not full product/world/SLAM acceptance']))
    print('SUMMARY',json.dumps(summary),flush=True)


if __name__=='__main__':main()
