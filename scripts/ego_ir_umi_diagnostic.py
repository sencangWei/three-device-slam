#!/usr/bin/env python3
"""Fixed-factory Ego IR3D -> UMI color2D regional diagnostic; never activates calibration."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_joint import board_geometry
from scripts.common_board_resampling_analysis import restore_board_id1, groups, spread
from scripts.ego_joint_stereo_diagnostic import diagnostic_targets, fit_extrinsic
from scripts.ego_stereo_color_validate import transform
from scripts.ego_ir_umi_capture import STREAMS, SERIALS

GROUPS = {k:v for k,v in groups().items() if k == 'all36' or k.startswith(('horizontal_', 'vertical_', 'checker_'))}
GATES = dict(static_drift_px=.75, ir_skew_ms=.2, host_arrival_gap_ms=75.,
             projection_p95_px=1., mount_edge_px=60., regional_center_mm=2., min_samples=10)


def triangulate(left, right, cal_left, cal_right, right_from_left):
    a = cb.normalized_camera_points(left, cal_left)
    b = cb.normalized_camera_points(right, cal_right)
    q = cv2.triangulatePoints(np.eye(3,4), right_from_left[:3], a.T, b.T)
    if np.any(np.abs(q[3]) < 1e-10):
        raise ValueError('triangulation at infinity')
    xyz = (q[:3]/q[3]).T
    scores = {name: cb.errors(cb.project(xyz,t,c),p) for name,t,c,p in (
        ('ir_left',np.eye(4),cal_left,left), ('ir_right',right_from_left,cal_right,right))}
    return xyz, scores


def fit_group(points, pixels, mount, calibration, seed, ids):
    if len(set(ids)) != len(ids) or len(ids)<4 or not set(ids)<=set(range(36)):
        raise ValueError('invalid tag group')
    if points.shape != (144,3) or pixels.shape != (144,2) or mount.shape != (4,3):
        raise ValueError('all36 and four mount corners required')
    mask = np.repeat([i in ids for i in range(36)],4)
    pose = fit_extrinsic(points[mask],pixels[mask],calibration,seed)
    center = cb.transform_points(mount.mean(axis=0)[None],pose)[0]
    return dict(ids=ids,T_Ucolor_Eir=pose.tolist(),mount_center_Ucolor_mm=(center*1000).tolist(),
        mount_distance_mm=float(np.linalg.norm(center)*1000),
        training=cb.errors(cb.project(points[mask],pose,calibration),pixels[mask]),
        heldout=cb.errors(cb.project(points[~mask],pose,calibration),pixels[~mask]) if np.any(~mask) else None)


def fit_observed_group(points, pixels, ir_pixels, mount, calibration, ir_calibration, ids):
    """Both initialization and optimization use training IDs only."""
    mask=np.repeat([i in ids for i in range(36)],4)
    objects=np.concatenate([cb.object_corners(i) for i in range(36)])
    seed=cb.fit_pose(objects[mask],pixels[mask],calibration)@np.linalg.inv(
        cb.fit_pose(objects[mask],ir_pixels[mask],ir_calibration))
    return fit_group(points,pixels,mount,calibration,seed,ids)


def source(session):
    setup=json.loads((session/'setup.json').read_text())
    capture=json.loads((session/'capture_report.json').read_text())
    if (setup['schema']!='ego.ir_umi_capture.v1' or setup['serials']!=SERIALS or
        setup['factory_values_modified'] or setup['activation']!='NOT_ACTIVATED' or
        setup['sensor_settings']['ego']['Emitter Enabled']!=0 or setup['native_fps']!=30):
        raise ValueError('source identity/settings mismatch')
    if capture['status']!='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION' or capture['failure'] or capture['cleanup_errors']:
        raise ValueError('incomplete capture')
    for name in ('setup.json','frames.jsonl'):
        if cb.digest(session/name)!=capture['hashes'][name]:raise ValueError('source hash mismatch')
    rows=[json.loads(line) for line in (session/'frames.jsonl').read_text().splitlines()]
    if len(rows)!=capture['samples'] or len(rows)<GATES['min_samples']:
        raise ValueError('sample count mismatch')
    last={}
    for i,row in enumerate(rows):
        if row['sample']!=i or row['setup_sha256']!=capture['hashes']['setup.json'] or set(row['streams'])!=set(STREAMS):
            raise ValueError('row binding mismatch')
        for role,meta in row['streams'].items():
            path=session/meta['file']
            if path.parent.resolve()!=session.resolve() or cb.digest(path)!=meta['sha256']:
                raise ValueError('raw hash/path mismatch')
            stamp,count,arrival=meta['sdk_timestamp_ms'],meta['frame_number'],meta['host_arrival_monotonic_ns']
            if not np.isfinite([stamp,count,arrival]).all():raise ValueError('nonfinite metadata')
            old=last.get(role)
            if old and (stamp<=old['sdk_timestamp_ms'] or count<=old['frame_number'] or arrival<=old['host_arrival_monotonic_ns'] or meta['sdk_timestamp_domain']!=old['sdk_timestamp_domain']):
                raise ValueError('frame metadata regression')
            last[role]=meta
    return setup,capture,rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=cb.new_output(args.output)
    setup,capture,rows=source(args.session)
    cal={r:cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],
        frame_id=r,intrinsics=setup['intrinsics'][r]) for r in STREAMS}
    right=transform(setup['right_from_left'])
    detector=make_board_detector('umi-aprilgrid');md=cb.mount_detector()
    objects=np.concatenate([cb.object_corners(i) for i in range(36)])
    frames=[];first={}
    for row in rows:
        failures=[];board={};mount={};identity={};drift={}
        for role,meta in row['streams'].items():
            gray=cv2.imread(str(args.session/meta['file']),0)
            if gray is None or gray.shape!=(720,1280):raise ValueError('image dimensions/decode mismatch')
            try:
                raw=cb.detect_grid(detector,gray)
                if role=='umi_color':
                    ds,_,p95=board_geometry(raw)
                    proof=dict(board_h_p95_px=p95)
                else:
                    ds,mount[role],proof=diagnostic_targets(raw,cb.detect_grid(md,gray))
                    if not proof['mount_size_pass']:failures.append(role+':mount_edge_below_60px')
                ds[1],id1=restore_board_id1(raw)
                if set(ds)!=set(range(36)):raise ValueError('need_all36_no_pruning')
                ds=cb.refine_legacy_corners(gray,ds)
                board[role]=np.concatenate([ds[i] for i in range(36)])
                identity[role]=dict(proof=proof,id1=id1)
                targets=dict(board=board[role])
                if role in mount:targets['mount']=mount[role]
                baseline=first.setdefault(role,{k:v.copy() for k,v in targets.items()})
                drift[role]={k:float(np.max(np.linalg.norm(v-baseline[k],axis=1))) for k,v in targets.items()}
                if max(drift[role].values())>GATES['static_drift_px']:failures.append(role+':static_drift')
            except ValueError as exc:failures.append(role+':'+str(exc))
        lm,rm,um=[row['streams'][r] for r in STREAMS]
        skew=abs(lm['sdk_timestamp_ms']-rm['sdk_timestamp_ms'])
        gap=abs(lm['host_arrival_monotonic_ns']-um['host_arrival_monotonic_ns'])/1e6
        if lm['sdk_timestamp_domain']!=rm['sdk_timestamp_domain'] or skew>GATES['ir_skew_ms']:
            failures.append('ir_timestamp_mismatch')
        if gap>GATES['host_arrival_gap_ms']:failures.append('host_arrival_gap')
        frame=dict(sample=row['sample'],source=row,failures=failures,identity=identity,drift_px=drift,
                   ir_skew_ms=skew,host_arrival_gap_ms=gap,board_pixels={k:v.tolist() for k,v in board.items()},
                   mount_pixels={k:v.tolist() for k,v in mount.items()})
        if set(board)==set(STREAMS) and set(mount)=={'ir_left','ir_right'} and 'ir_timestamp_mismatch' not in failures:
            try:
                xyz,score=triangulate(board['ir_left'],board['ir_right'],cal['ir_left'],cal['ir_right'],right)
                mx,mscore=triangulate(mount['ir_left'],mount['ir_right'],cal['ir_left'],cal['ir_right'],right)
                frame.update(board_ir3d_m=xyz.tolist(),mount_ir3d_m=mx.tolist(),stereo_board=score,stereo_mount=mscore)
                # Printed geometry supplies only a seed, never the final objective or scale.
                fits={k:fit_observed_group(xyz,board['umi_color'],board['ir_left'],mx,
                    cal['umi_color'],cal['ir_left'],ids) for k,ids in GROUPS.items()}
                frame['groups']=fits
                frame['contrasts']={k:dict(center_mm=float(np.linalg.norm(np.array(fits[k+'_0']['mount_center_Ucolor_mm'])-fits[k+'_1']['mount_center_Ucolor_mm'])),
                    rotation_deg=cb.difference(np.array(fits[k+'_0']['T_Ucolor_Eir']),np.array(fits[k+'_1']['T_Ucolor_Eir']))[1])
                    for k in ('horizontal','vertical','checker')}
                if any(e['p95_px']>GATES['projection_p95_px'] for s in (score,mscore) for e in s.values()):
                    failures.append('stereo_reprojection')
                if any(f['training']['p95_px']>GATES['projection_p95_px'] or (f['heldout'] and f['heldout']['p95_px']>GATES['projection_p95_px']) for f in fits.values()):
                    failures.append('umi_projection_or_spatial_holdout')
                if any(c['center_mm']>GATES['regional_center_mm'] for c in frame['contrasts'].values()):
                    failures.append('regional_center_disagreement')
            except (ValueError,cv2.error) as exc:failures.append('geometry:'+str(exc))
        frames.append(frame)
        print('FRAME',row['sample'],'failures',failures,'contrasts',frame.get('contrasts'),flush=True)
    measured=[f for f in frames if 'groups' in f]
    summary=dict(frames=len(frames),measured=len(measured),frames_with_failures=sum(bool(f['failures']) for f in frames))
    if measured:
        summary['regional_center_mm']={k:dict(median=float(np.median([f['contrasts'][k]['center_mm'] for f in measured])),
            max=max(f['contrasts'][k]['center_mm'] for f in measured)) for k in ('horizontal','vertical','checker')}
        summary['all36_center']=spread([f['groups']['all36']['mount_center_Ucolor_mm'] for f in measured])
        summary['all36_training_p95_px_max']=max(f['groups']['all36']['training']['p95_px'] for f in measured)
        summary['spatial_holdout_p95_px_max']=max(g['heldout']['p95_px'] for f in measured for g in f['groups'].values() if g['heldout'])
    out.mkdir()
    cb.write_json(out/'report.json',dict(status='DIAGNOSTIC_ONLY_REQUIRES_INDEPENDENT_VIEW' if len(measured)==len(frames) and not any(f['failures'] for f in frames) else 'FAIL',
        activation='NOT_ACTIVATED',summary=summary,gates=GATES,groups=GROUPS,source=str(args.session.resolve()),
        source_hashes=capture['hashes'],capture_report_sha256=cb.digest(args.session/'capture_report.json'),frames=frames,
        code_sha256={str(p.relative_to(cb.ROOT)):cb.digest(p) for p in (Path(__file__).resolve(),cb.ROOT/'scripts/ego_joint_stereo_diagnostic.py',
            cb.ROOT/'scripts/common_board_calibration.py',cb.ROOT/'scripts/common_board_detector.py',cb.ROOT/'scripts/common_board_resampling_analysis.py')},
        limitations=['No Ego RGB, no K/D fit; native fixed5 board corners; all frames/failures retained',
            'Independent host arrivals are not exposure synchronization; static diagnostic only',
            'Mount excluded from extrinsic fitting; center is mean of four stereo reconstructed corners',
            'No independent mount ground truth, no complete mount rotation or SLAM/world acceptance',
            'IR calibration and triangulation noise remain reference uncertainties',
            'Same static scene/spatial holdout does not replace independent-view acceptance']))
    print('SUMMARY',json.dumps(summary),flush=True)


if __name__=='__main__':main()
