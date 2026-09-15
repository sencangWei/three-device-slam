#!/usr/bin/env python3
"""Same-frame RGB/IR board and independent mount check. Diagnostic, not activation."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_joint import board_geometry, location, POLICY
from scripts.common_board_resampling_analysis import restore_board_id1, ID1_POLICY
from scripts.ego_stereo_color_validate import transform, stereo_projection

STREAMS = ('color','ir_left','ir_right')
GATES = dict(max_static_drift_px=.75, max_ir_skew_ms=.2, max_projection_p95_px=1.,
             all_board_tags=36, fit_board_frames=5)


def diagnostic_targets(raw,tags):
    """Preserve identifiable small mount for diagnosis; size failure is NOT waived."""
    ds,footprint,p95=board_geometry(raw)
    candidates=[d for d in tags if d.tag_id==1 and location(d.corners,footprint)=='outside']
    if len(candidates)!=1:raise ValueError('need_unique_mount1_outside_board')
    chosen=candidates[0]
    for d in raw:
        if d.tag_id==1 and location(d.corners,footprint)=='outside' and not cb.same_mount_quad(d,chosen):
            raise ValueError('unmatched_external_id1')
    q=np.asarray(chosen.corners,dtype=float)
    edge=float(np.linalg.norm(q-np.roll(q,1,axis=0),axis=1).min())
    return ds,q,dict(footprint=footprint.tolist(),board_h_p95_px=p95,mount_min_edge_px=edge,
                    mount_size_pass=edge>=POLICY['mount_min_edge_px'])


def fit_extrinsic(points,pixels,calibration,seed):
    rays=cb.normalized_camera_points(pixels,calibration)
    r,t=cv2.solvePnPRefineLM(np.asarray(points,dtype=float),rays,np.eye(3),np.zeros(5),
        cv2.Rodrigues(seed[:3,:3])[0].copy(),seed[:3,3].copy(),
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,100,1e-12))
    result=cb.unpack(np.r_[r.ravel(),t.ravel()])
    if not np.isfinite(result).all() or np.any(cb.transform_points(points,result)[:,2]<=0):
        raise ValueError('invalid fitted pose')
    return result


def board_shape(objects,points):
    # Independent affine axes; no board dimensions enter stereo triangulation.
    design=np.c_[objects[:,:2],np.ones(len(objects))]
    coefficients=np.linalg.lstsq(design,points,rcond=None)[0]
    a,b=coefficients[:2];sa,sb=np.linalg.norm(a),np.linalg.norm(b)
    return dict(axis_scale_x=float(sa),axis_scale_y=float(sb),
        axis_angle_deg=float(np.rad2deg(np.arccos(np.clip(a@b/(sa*sb),-1,1)))),
        affine_residual_rms_mm=float(np.sqrt(np.mean(np.sum((design@coefficients-points)**2,axis=1)))*1000))


def drift_summary(frames):
    result={}
    for role in STREAMS:
        result[role]={}
        for target in ('board','mount'):
            values=[f['drift_px'].get(role,{}).get(target) for f in frames]
            result[role][target]=max(values) if values and all(v is not None and np.isfinite(v) for v in values) else None
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv);out=cb.new_output(args.output)
    s=args.session;setup=json.loads((s/'setup.json').read_text());capture=json.loads((s/'capture_report.json').read_text())
    if setup['schema']!='ego.stereo_color_probe.v1' or setup['serial']!='327122078613':
        raise ValueError('wrong source')
    if capture['status']!='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION' or capture['failure'] or capture['cleanup_errors']:
        raise ValueError('incomplete source')
    for name in ('setup.json','frames.jsonl'):
        if cb.digest(s/name)!=capture['hashes'][name]:raise ValueError('capture hash mismatch')
    rows=[json.loads(t) for t in (s/'frames.jsonl').read_text().splitlines()]
    if len(rows)!=capture['samples'] or len(rows)<=GATES['fit_board_frames']:
        raise ValueError('insufficient or incomplete rows')
    cal={r:cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],
        frame_id='ego.'+r,intrinsics=setup['intrinsics'][r]) for r in STREAMS}
    right,color=transform(setup['extrinsics']['ir_right']),transform(setup['extrinsics']['color'])
    candidate_report=json.loads(args.candidate.read_text())
    if candidate_report['activation']!='NOT_ACTIVATED' or candidate_report['status']!='CAMERA_RECALIBRATION_DEVELOPMENT_ONLY':
        raise ValueError('wrong candidate schema')
    if candidate_report['devices']['ego']['serial']!=setup['serial'] or candidate_report['devices']['ego']['intrinsics']!=setup['intrinsics']['color']:
        raise ValueError('candidate camera identity mismatch')
    candidate=next(c['candidate'] for c in candidate_report['cases'] if c['candidate']['mode']=='K4_k1')
    changed=replace(cal['color'],camera_matrix=np.array(candidate['K']),distortion_coefficients=np.array(candidate['D']),
                    distortion_model=candidate['distortion_model'])
    detector=make_board_detector('umi-aprilgrid');mount_detector=cb.mount_detector()
    frames=[];first={};last={};objects=np.concatenate([cb.object_corners(i) for i in range(36)])
    for index,row in enumerate(rows):
        if row['sample']!=index or row['setup_sha256']!=capture['hashes']['setup.json'] or set(row['streams'])!=set(STREAMS):
            raise ValueError('row binding mismatch')
        board={};mount={};identity={};drifts={};failures=[]
        for role in STREAMS:
            meta=row['streams'][role];path=s/meta['file']
            if path.parent.resolve()!=s.resolve() or cb.digest(path)!=meta['sha256']:raise ValueError('raw mismatch')
            previous=last.get(role)
            if previous and (meta['sdk_timestamp_ms']<=previous['sdk_timestamp_ms'] or meta['frame_number']<=previous['frame_number'] or meta['sdk_timestamp_domain']!=previous['sdk_timestamp_domain']):
                raise ValueError('timestamp regression')
            last[role]=meta
            gray=cv2.imread(str(path),0)
            if gray is None or gray.shape!=(720,1280):raise ValueError('image shape mismatch')
            try:
                raw=cb.detect_grid(detector,gray);tags=cb.detect_grid(mount_detector,gray)
                ds,mount[role],proof=diagnostic_targets(raw,tags)
                ds[1],id1=restore_board_id1(raw)
                if set(ds)!=set(range(36)):raise ValueError('need_all36_no_pruning')
                ds=cb.refine_legacy_corners(gray,ds)
                board[role]=np.concatenate([ds[i] for i in range(36)])
                identity[role]=dict(board=proof,id1=id1)
                if not proof['mount_size_pass']:failures.append(role+':mount_edge_below_60px')
                baseline=first.setdefault(role,dict(board=board[role].copy(),mount=mount[role].copy()))
                drifts[role]={target:float(np.max(np.linalg.norm(points-baseline[target],axis=1)))
                              for target,points in (('board',board[role]),('mount',mount[role]))}
                if max(drifts[role].values())>GATES['max_static_drift_px']:failures.append(role+':static_drift')
            except ValueError as exc:failures.append(role+':'+str(exc))
        left_meta,right_meta=row['streams']['ir_left'],row['streams']['ir_right']
        skew=abs(left_meta['sdk_timestamp_ms']-right_meta['sdk_timestamp_ms'])
        if left_meta['sdk_timestamp_domain']!=right_meta['sdk_timestamp_domain'] or skew>GATES['max_ir_skew_ms']:
            failures.append('ir_timestamp_mismatch')
        frame=dict(sample=index,source=row,identity=identity,drift_px=drifts,failures=failures,ir_skew_ms=skew,
                   board_pixels={r:q.tolist() for r,q in board.items()},mount_pixels={r:q.tolist() for r,q in mount.items()})
        if set(board)==set(STREAMS) and set(mount)==set(STREAMS) and 'ir_timestamp_mismatch' not in failures:
            for target,pixels in (('board',board),('mount',mount)):
                try:
                    result=stereo_projection(pixels,cal,right,color)
                    points=np.asarray(result['points_ir_left_m'])
                    result['candidate_factory_extrinsic']=cb.errors(cb.project(points,color,changed),pixels['color'])
                    if target=='board':result['board_shape']=board_shape(objects,points)
                    frame[target]=result
                except ValueError as exc:failures.append(target+':'+str(exc))
        frames.append(frame)
        print('FRAME',index,'failures',failures,'RGB p95',frame.get('board',{}).get('projections',{}).get('color',{}).get('p95_px'),flush=True)
    fits={}
    training=frames[:GATES['fit_board_frames']]
    # Small mount never enters this board-only fit; preserve its independent FAIL.
    if all('board' in f and not any(not reason.endswith(':mount_edge_below_60px') for reason in f['failures']) for f in training):
        xyz=np.concatenate([f['board']['points_ir_left_m'] for f in training])
        pix=np.concatenate([f['board_pixels']['color'] for f in training])
        for name,camera in (('factory',cal['color']),('K4_k1',changed)):
            pose=fit_extrinsic(xyz,pix,camera,color)
            scores=[]
            for f in frames:
                scores.append(dict(sample=f['sample'],training_board=f['sample']<GATES['fit_board_frames'],
                    **{target:cb.errors(cb.project(np.asarray(f[target]['points_ir_left_m']),pose,camera),np.asarray(f[target+'_pixels']['color']))
                       for target in ('board','mount') if target in f}))
            fits[name]=dict(T_color_ir_left=pose.tolist(),scores=scores,training_board_samples=list(range(GATES['fit_board_frames'])),
                            mount_never_used_in_fit=True,activation='NOT_ACTIVATED')
    measured=[f for f in frames if 'board' in f and 'mount' in f]
    drift=drift_summary(frames)
    static_pass=all(v is not None and v<=GATES['max_static_drift_px'] for r in drift.values() for v in r.values())
    geometry_pass=len(measured)==len(frames) and all(not f['failures'] for f in frames)
    factory_projection_pass=len(measured)==len(frames) and all(
        f[t]['projections'][r]['p95_px']<=GATES['max_projection_p95_px']
        for f in measured for t in ('board','mount') for r in STREAMS)
    summary=dict(frames=len(frames),measured=len(measured),static_pass=static_pass,
                 factory_projection_pass=factory_projection_pass,geometry_pass=geometry_pass,drift_max_px=drift)
    out.mkdir();cb.write_json(out/'report.json',dict(status='PASS_REFERENCE_CHECK_ONLY' if static_pass and geometry_pass and factory_projection_pass else 'FAIL',
        activation='NOT_ACTIVATED',summary=summary,gates=GATES,source=str(s.resolve()),source_hashes=capture['hashes'],
        capture_report_sha256=cb.digest(s/'capture_report.json'),candidate_sha256=cb.digest(args.candidate),candidate=candidate,
        identity_policy=POLICY,derived_id1_policy=ID1_POLICY,board_detector='umi-aprilgrid',legacy_corners='fixed5',
        frames=frames,board_only_extrinsic_fits=fits,
        limitations=['IR calibration is a reference, not absolute metrology',
            'candidate K/D frozen before new capture; fitted extrinsics are separate diagnostic only',
            'first5 board frames fit extrinsic; mount never fitted; same static scene is not multi-view acceptance',
            'print dimensions used only in identity/shape diagnostic, not IR triangulation or extrinsic fit',
            'all frames retained, static failures block fitting first5; no factory modification',
            'identifiable mounts below60px retained for diagnosis with explicit size failure; cannot pass acceptance'],
        code_sha256={str(p.relative_to(cb.ROOT)):cb.digest(p) for p in (Path(__file__).resolve(),
            cb.ROOT/'scripts/ego_stereo_color_validate.py',cb.ROOT/'scripts/common_board_joint.py',
            cb.ROOT/'scripts/common_board_resampling_analysis.py',cb.ROOT/'scripts/common_board_detector.py',
            cb.ROOT/'scripts/common_board_calibration.py',cb.ROOT/'scripts/fixed_factory_board_diagnostic.py',
            cb.ROOT/'three_device_slam/spatial/apriltag_detector.py')}))
    print('SUMMARY',summary,flush=True)


if __name__=='__main__':main()
