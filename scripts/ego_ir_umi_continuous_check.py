#!/usr/bin/env python3
"""Fit only phase A; score phase B from a single uninterrupted camera session."""
import argparse
import json
from pathlib import Path
import numpy as np
from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.ego_ir_umi_frozen_check import observe,fixed_score
from scripts.ego_ir_umi_diagnostic import source,triangulate,fit_observed_group,GATES
from scripts.ego_stereo_color_validate import transform
from scripts.ego_ir_lock import check_frame_evidence


def phase_rows(setup,capture,rows):
    if setup.get('acquisition_mode') not in ('continuous_ab','continuous_aba') or capture.get('phase_state')!='DONE' or not capture.get('continuous_stream'):
        raise ValueError('requires completed continuous A/B session')
    names=('A','B','C') if setup['acquisition_mode']=='continuous_aba' else ('A','B')
    if 'tilt_policy' in setup and (setup['acquisition_mode']!='continuous_aba' or setup['tilt_policy']!=dict(min_normal_change_deg=12.,all_views=True)):
        raise ValueError('tilt gate mismatch')
    phases={k:[r for r in rows if r.get('phase')==k] for k in names}
    if [r.get('phase') for r in rows]!=[k for k in names for _ in phases[k]]:
        raise ValueError('invalid phase order')
    for k,v in phases.items():
        if len(v)<10 or len(v)!=capture['phase_samples'][k]:raise ValueError('phase samples missing')
        span=(v[-1]['streams']['ir_left']['host_arrival_monotonic_ns']-v[0]['streams']['ir_left']['host_arrival_monotonic_ns'])/1e9
        if not 8.<=span<=10.5:raise ValueError('phase stored span invalid')
    if 'C' in phases:
        expected=setup['ir_lock']['expected']
        if expected['enable_auto_exposure']!=0 or capture.get('ir_final_readback')!=expected:
            raise ValueError('IR lock not verified')
        if capture.get('ir_lock')!=setup['ir_lock'] or not capture.get('ir_original_settings_restored'):
            raise ValueError('IR lock/restore evidence inconsistent')
        if setup.get('return_policy')!=dict(phase='C',median_corner_distance_px=5.,max_corner_distance_px=10.,origin_translation_mm=10.,normal_deg=2.):
            raise ValueError('return gate mismatch')
        for r in rows:
            for role in ('ir_left','ir_right'):
                check_frame_evidence(r['streams'][role].get('ir_metadata',{}),expected)
    return phases


def fit_phase_a(train,cal,right):
    if not train or any(f['phase']!='A' for f in train):raise ValueError('phase A only')
    if any(any(not e.endswith(':mount_edge_below_60px') for e in f.get('failures',[])) for f in train):
        raise ValueError('phase A quality failed')
    pixels={k:np.mean([f['pixels'][k] for f in train],axis=0) for k in cal}
    xyz,_=triangulate(pixels['ir_left'],pixels['ir_right'],cal['ir_left'],cal['ir_right'],right)
    mount=np.mean([f['mount_ir_m'] for f in train],axis=0)
    return fit_observed_group(xyz,pixels['umi_color'],pixels['ir_left'],mount,cal['umi_color'],cal['ir_left'],list(range(36))),pixels


def check_static(frame,phase,first,pixels,mount):
    for target,views in (('board',pixels),('mount',mount)):
        frame[target+'_drift_px']={}
        for role,points in views.items():
            baseline=first.setdefault((phase,role,target),points.copy())
            drift=float(np.linalg.norm(points-baseline,axis=1).max())
            frame[target+'_drift_px'][role]=drift
            if drift>GATES['static_drift_px']:frame['failures'].append(role+':'+target+'_static_drift')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=cb.new_output(args.output)
    setup,capture,rows=source(args.session);phases=phase_rows(setup,capture,rows)
    cal={k:cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],frame_id=k,intrinsics=v)
         for k,v in setup['intrinsics'].items()}
    right=transform(setup['right_from_left']);detector=make_board_detector('umi-aprilgrid');md=cb.mount_detector()
    frames=[];first={}
    for row in rows:
        phase=row['phase'];board,mount,identity,failures=observe(args.session,row,detector,md)
        frame=dict(sample=row['sample'],phase=phase,source=row,identity=identity,failures=failures,drift_px={})
        lm,rm,um=[row['streams'][k] for k in ('ir_left','ir_right','umi_color')]
        if lm['sdk_timestamp_domain']!=rm['sdk_timestamp_domain'] or abs(lm['sdk_timestamp_ms']-rm['sdk_timestamp_ms'])>GATES['ir_skew_ms']:
            failures.append('ir_timestamp')
        if abs(lm['host_arrival_monotonic_ns']-um['host_arrival_monotonic_ns'])/1e6>GATES['host_arrival_gap_ms']:
            failures.append('host_gap')
        if set(board)==set(cal) and all(set(v)==set(range(36)) for v in board.values()) and len(mount)==2:
            pixels={k:np.concatenate([v[i] for i in range(36)]) for k,v in board.items()}
            frame.update(pixels={k:v.tolist() for k,v in pixels.items()},mount_pixels={k:v.tolist() for k,v in mount.items()})
            check_static(frame,phase,first,pixels,mount)
            try:
                xyz,score=triangulate(pixels['ir_left'],pixels['ir_right'],cal['ir_left'],cal['ir_right'],right)
                mx,mscore=triangulate(mount['ir_left'],mount['ir_right'],cal['ir_left'],cal['ir_right'],right)
                frame.update(points_ir_m=xyz.tolist(),mount_ir_m=mx.tolist(),stereo_scores=score,stereo_mount_scores=mscore)
                if any(e['p95_px']>1 for s in (score,mscore) for e in s.values()):failures.append('stereo_projection')
            except ValueError as exc:failures.append(str(exc))
        else:failures.append('incomplete_board_or_mount')
        frames.append(frame)
    train=[f for f in frames if f['phase']=='A'];test=[f for f in frames if f['phase']=='B']
    fitted=None
    if all('points_ir_m' in f and all(e.endswith(':mount_edge_below_60px') for e in f['failures']) for f in train):
        fitted,pixels=fit_phase_a(train,cal,right)
        frozen=np.array(fitted['T_Ucolor_Eir'])
        objects=np.concatenate([cb.object_corners(i) for i in range(36)])
        prior={k:cb.fit_pose(objects,v,cal[k]) for k,v in pixels.items()}
        prior_mount={k:np.mean([f['mount_pixels'][k] for f in train],axis=0) for k in ('ir_left','ir_right')}
        for f in frames:
            if 'points_ir_m' not in f:continue
            f['frozen_score']=fixed_score(np.array(f['points_ir_m']),np.array(f['pixels']['umi_color']),cal['umi_color'],frozen)
            f['mount_center_Ucolor_mm']=(cb.transform_points(np.mean(f['mount_ir_m'],axis=0)[None],frozen)[0]*1000).tolist()
            if f['frozen_score']['p95_px']>1:f['failures'].append('frozen_projection')
            if f['phase'] in ('B','C'):
                f['novelty']={};f['mount_projection_change_px']={}
                for k in cal:
                    pose=cb.fit_pose(objects,np.array(f['pixels'][k]),cal[k])
                    shift=float(np.linalg.norm(pose[:3,3]-prior[k][:3,3])*1000)
                    angle=float(np.degrees(np.arccos(np.clip(pose[:3,2]@prior[k][:3,2],-1,1))))
                    f['novelty'][k]=dict(origin_translation_mm=shift,normal_deg=angle)
                    if f['phase']=='B' and shift<20 and angle<7:f['failures'].append(k+':insufficient_board_pose_change')
                    if f['phase']=='B' and 'tilt_policy' in setup and angle<setup['tilt_policy']['min_normal_change_deg']:
                        f['failures'].append(k+':insufficient_board_tilt')
                    if f['phase']=='C':
                        delta=np.linalg.norm(np.array(f['pixels'][k])-pixels[k],axis=1)
                        f.setdefault('return_pixels',{})[k]=dict(median_px=float(np.median(delta)),max_px=float(delta.max()))
                        if shift>10 or angle>2 or np.median(delta)>5 or delta.max()>10:
                            f['failures'].append(k+':board_return_mismatch')
                for k in prior_mount:
                    delta=float(np.linalg.norm(np.array(f['mount_pixels'][k])-prior_mount[k],axis=1).max())
                    f['mount_projection_change_px'][k]=delta
                    if delta>.75:f['failures'].append(k+':mount_projection_changed')
    scores=[f['frozen_score']['p95_px'] for f in test if 'frozen_score' in f]
    summary=dict(phase_samples={k:sum(f['phase']==k for f in frames) for k in phases},B_scored=len(scores),
        B_p95_median_px=float(np.median(scores)) if scores else None,B_p95_max_px=max(scores) if scores else None,
        failed_frames=sum(bool(f['failures']) for f in frames))
    if 'C' in phases:
        cs=[f['frozen_score']['p95_px'] for f in frames if f['phase']=='C' and 'frozen_score' in f]
        summary.update(C_scored=len(cs),C_p95_median_px=float(np.median(cs)) if cs else None,C_p95_max_px=max(cs) if cs else None)
    out.mkdir();cb.write_json(out/'report.json',dict(status='PASS_FIXED_TRANSFORM_VIEW_ONLY' if fitted and not any(f['failures'] for f in frames) else 'FAIL',
        activation='NOT_ACTIVATED',source_hashes=capture['hashes'],capture_sha256=cb.digest(args.session/'capture_report.json'),
        fit_A_only=fitted,training_samples=[f['sample'] for f in train],summary=summary,frames=frames,
        code_sha256=cb.digest(Path(__file__).resolve()),limitations=['One stream eliminates inter-window restart, not all confounders',
            'Factory fixed; B never fitted X/K/D; board PnP only measures novelty',
            'Small mount remains size failure; no physical movement inference from projection change']))
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
