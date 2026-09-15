#!/usr/bin/env python3
"""All-frame36-tag diagnostic; never modifies the captured35-tag policy."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_joint import board_geometry,location
from scripts.common_board_static_analysis import pose_options,mount_summary,inspect_static

ID1_POLICY=dict(name='offline_unique_board_id1_v1',max_canonical_corner_error_px=3.,
                fit_support='all non1board tags, no RANSAC',activation='NOT_ACTIVATED')


def restore_board_id1(raw):
    ds,footprint,_=board_geometry(raw)
    candidates=[d for d in raw if d.tag_id==1 and location(d.corners,footprint)=='board']
    if len(candidates)!=1:raise ValueError('boardID1 must have exactly one inside candidate; never choose closest')
    ids=sorted(ds);obj=np.concatenate([cb.object_corners(i)[:,:2] for i in ids]);pix=np.concatenate([ds[i] for i in ids])
    h,_=cv2.findHomography(obj,pix,0)
    predicted=cv2.perspectiveTransform(cb.object_corners(1)[:,:2].reshape(-1,1,2),h).reshape(4,2)
    quad=np.asarray(candidates[0].corners,dtype=float)
    error=float(np.max(np.linalg.norm(quad-predicted,axis=1)))
    if not np.isfinite(error) or error>ID1_POLICY['max_canonical_corner_error_px']:
        raise ValueError('boardID1 canonical geometry mismatch')
    return quad,dict(inside_candidates=1,max_corner_error_px=error,quad=quad.tolist())


def groups():
    all_ids=list(range(36));result={'all36':all_ids,'all35':[i for i in all_ids if i!=1]}
    for name,selector in [('checker',lambda i:(i//6+i%6)%2),('horizontal',lambda i:int(i//6>=3)),
                          ('vertical',lambda i:int(i%6>=3))]:
        for side in (0,1):result[f'{name}_{side}']=[i for i in all_ids if selector(i)==side]
    for axis,selector in [('row',lambda i:i//6),('column',lambda i:i%6)]:
        for line in range(6):
            result[f'{axis}_{line}']=[i for i in all_ids if selector(i)==line]
            result[f'without_{axis}_{line}']=[i for i in all_ids if selector(i)!=line]
    for i in all_ids:result[f'without_tag_{i}']=[j for j in all_ids if j!=i]
    return result


def fit_group(window,ids,cal,ego_mount):
    if len(ids)!=len(set(ids)) or not set(ids)<=set(range(36)) or len(ids)<4:
        raise ValueError('invalid fixed group')
    expected=np.concatenate([cb.object_corners(i) for i in range(36)])
    np.testing.assert_array_equal(window['objects'],expected)
    mask=np.repeat([i in ids for i in range(36)],4)
    options={r:pose_options(window['objects'][mask],window['pixels'][r][mask],cal[r]) for r in cb.ROLES}
    x=options['ego'][0]['pose']@np.linalg.inv(options['left'][0]['pose'])
    return dict(ids=ids,T_Ecolor_Ucolor=x.tolist(),mount=mount_summary(np.linalg.inv(x)@ego_mount),
        fit={r:options[r][0]['score'] for r in cb.ROLES},
        heldout={r:cb.errors(cb.project(window['objects'][~mask],options[r][0]['pose'],cal[r]),window['pixels'][r][~mask])
                 for r in cb.ROLES} if np.any(~mask) else None,
        pose_branches={r:[dict(T_camera_board=o['pose'].tolist(),score=o['score']) for o in options[r]] for r in cb.ROLES})


def spread(xyz):
    xyz=np.asarray(xyz,dtype=float);norm=np.linalg.norm(xyz,axis=1)
    return dict(n=len(xyz),mean_xyz_mm=xyz.mean(0).tolist(),axis_std_mm=xyz.std(0).tolist(),
        distance_range_mm=[float(norm.min()),float(norm.max())],
        max_pairwise_mm=float(np.linalg.norm(xyz[:,None]-xyz[None,:],axis=2).max()))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv);out=cb.new_output(args.output)
    setup,cal,_,_,hashes,audit=cb.load_session(args.session,allow_partial=True)
    if setup.get('observation_mode')!='simultaneous' or setup.get('resume_from'):
        raise ValueError('requires single simultaneous source; do not silently join sessions')
    frozen=groups();detector=make_board_detector(setup['board_detector']);mount_detector=cb.mount_detector()
    rows=[json.loads(line) for line in (args.session/'attempts.jsonl').read_text().splitlines()]
    frames=[];gates=[]
    for row in rows:
        if not row['accepted']:continue
        samples=[]
        for number,pair in enumerate(row['pairs']):
            images={}
            for role in cb.ROLES:
                path=args.session/pair[role]['file']
                if cb.digest(path)!=pair[role]['sha256']:raise ValueError('raw hash mismatch')
                images[role]=cv2.imread(str(path),0)
            display={}
            detected,reasons=cb.detect_sample(images,'board',detector,mount_detector,display=display,simultaneous=True)
            if reasons:raise ValueError('recorded pair no longer joint-valid')
            restored={};board={r:dict(detected['_board'][r]) for r in cb.ROLES}
            for role in cb.ROLES:
                q,restored[role]=restore_board_id1(display[role]['raw'])
                board[role][1]=cb.refine_legacy_corners(images[role],{1:q})[1]
                if set(board[role])!=set(range(36)):raise ValueError('all36 analysis requires all36 in EVERY frame; no frame pruning')
            arrivals=[pair[r]['host_arrival_monotonic_ns'] for r in cb.ROLES]
            samples.append(dict(t=max(arrivals)/1e9,arrival_gap_ms=abs(arrivals[0]-arrivals[1])/1e6,detected=board,reasons=[]))
            window=dict(slot=row['slot'],objects=np.concatenate([cb.object_corners(i) for i in range(36)]),
                        pixels={r:np.concatenate([board[r][i] for i in range(36)]) for r in cb.ROLES})
            mount_pixels=detected['_mount']['ego'][1]
            static=inspect_static(window,range(36),mount_pixels,cal)
            ego_mount=np.asarray(static['whole_board']['T_Ecolor_Ucolor'])@np.asarray(static['whole_board']['mount']['T_Ucolor_mount'])
            fits={name:fit_group(window,ids,cal,ego_mount) for name,ids in frozen.items()}
            for f in fits.values():
                f['delta_all36_mm'],f['rotation_all36_deg']=cb.difference(np.asarray(f['mount']['T_Ucolor_mount']),np.asarray(static['whole_board']['mount']['T_Ucolor_mount']))
            contrasts={name:dict(zip(('translation_mm','rotation_deg'),cb.difference(
                np.asarray(fits[name+'_0']['mount']['T_Ucolor_mount']),np.asarray(fits[name+'_1']['mount']['T_Ucolor_mount']))))
                for name in ('checker','horizontal','vertical')}
            frames.append(dict(attempt=row['attempt'],slot=row['slot'],pair=number,source=pair,restored_id1=restored,
                               static=static,groups=fits,contrasts=contrasts))
            print('frame',len(frames),'all36norm',round(static['whole_board']['mount']['distance_mm'],3),
                  'contrasts',{k:round(v['translation_mm'],3) for k,v in contrasts.items()},flush=True)
        gates.append(dict(slot=row['slot'],restored36_static=cb.window_quality(samples,'board')))
    if not frames:raise ValueError('no accepted frames')
    summaries={name:dict(positions=spread([f['groups'][name]['mount']['xyz_mm'] for f in frames]),
                        delta_all36_p50_mm=float(np.median([f['groups'][name]['delta_all36_mm'] for f in frames])),
                        delta_all36_max_mm=float(max(f['groups'][name]['delta_all36_mm'] for f in frames))) for name in frozen}
    contrasts={name:dict(min_mm=float(min(f['contrasts'][name]['translation_mm'] for f in frames)),
                        median_mm=float(np.median([f['contrasts'][name]['translation_mm'] for f in frames])),
                        max_mm=float(max(f['contrasts'][name]['translation_mm'] for f in frames))) for name in ('checker','horizontal','vertical')}
    out.mkdir();cb.write_json(out/'report.json',dict(status='RESAMPLING_DIAGNOSTIC_NOT_CALIBRATION',activation='NOT_ACTIVATED',
        source_session=str(args.session.resolve()),source_hashes=hashes,original_capture_policy=setup['identity_policy'],
        derived_id1_policy=ID1_POLICY,groups=frozen,frame_count=len(frames),group_fit_count=len(frames)*len(frozen),
        frames=frames,group_summary=summaries,contrast_summary=contrasts,restored_static_checks=gates,
        limitations=['all frames reused from two existing windows; correlated, not new independent poses',
            'ID1 restored only for derived diagnostic; original capture policy/acceptance unchanged',
            'thin row/column supports are sensitivity diagnostics, not equal-accuracy calibrations',
            'no outcome-driven grouping/pruning/factory fit/activation; dispersion is not absolute error'],
        code_sha256={str(s.relative_to(cb.ROOT)):cb.digest(s) for s in (Path(__file__).resolve(),
            cb.ROOT/'scripts/common_board_static_analysis.py',cb.ROOT/'scripts/common_board_calibration.py',
            cb.ROOT/'scripts/common_board_joint.py',cb.ROOT/'scripts/common_board_detector.py')}))
    print('SUMMARY',json.dumps(dict(all36=summaries['all36'],all35=summaries['all35'],contrasts=contrasts)),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
