#!/usr/bin/env python3
"""Static same-frame per-tag analysis, not calibration acceptance or activation."""
import argparse
import itertools
from pathlib import Path
import cv2
import numpy as np
from scripts import common_board_calibration as cb


def pose_options(objects,pixels,calibration):
    rays=cb.normalized_camera_points(pixels,calibration)
    seeds=cv2.solvePnPGeneric(objects,rays,np.eye(3),np.zeros(5),flags=cv2.SOLVEPNP_IPPE)
    results=[]
    for index,(r,t) in enumerate(zip(seeds[1],seeds[2])):
        r,t=cv2.solvePnPRefineLM(objects,rays,np.eye(3),np.zeros(5),r.copy(),t.copy())
        pose=cb.unpack(np.r_[r.ravel(),t.ravel()])
        if np.isfinite(pose).all() and np.min(cb.transform_points(objects,pose)[:,2])>0:
            score=cb.errors(cb.project(objects,pose,calibration),pixels)
            results.append(dict(seed=index,pose=pose,score=score))
    if not results:raise ValueError('no positive board/tag pose')
    return sorted(results,key=lambda r:(r['score']['rms_px'],r['seed']))


def mount_summary(m):
    return dict(T_Ucolor_mount=m.tolist(),xyz_mm=(m[:3,3]*1000).tolist(),
                z_mm=float(m[2,3]*1000),distance_mm=float(np.linalg.norm(m[:3,3])*1000))


def inspect_static(window,ids,mount_pixels,cal):
    ids=list(ids)
    if len(ids)!=len(set(ids)) or len(window['objects'])!=4*len(ids):
        raise ValueError('IDs/object support mismatch')
    np.testing.assert_allclose(window['objects'],np.concatenate([cb.object_corners(i) for i in ids]),atol=0,rtol=0)
    poses={r:pose_options(window['objects'],window['pixels'][r],cal[r]) for r in cb.ROLES}
    x=poses['ego'][0]['pose']@np.linalg.inv(poses['left'][0]['pose'])
    mounts=sorted((c for c in cb._solve_ippe_candidates(mount_pixels,calibration=cal['ego'],tag_size_m=.04) if c.positive_depth),
                  key=lambda c:c.reprojection_error_px)
    if not mounts:raise ValueError('no positive mount pose')
    m=np.linalg.inv(x)@mounts[0].camera_from_tag
    whole=dict(T_Ecolor_Ucolor=x.tolist(),mount=mount_summary(m),
        fit={r:poses[r][0]['score'] for r in cb.ROLES},
        mount_branches=[dict(mount_summary(np.linalg.inv(x)@c.camera_from_tag),
                             pixel_error=c.reprojection_error_px) for c in mounts])
    per_tag=[]
    for j,tag in enumerate(ids):
        sl=slice(4*j,4*j+4);obj=window['objects'][sl]
        options={r:pose_options(obj,window['pixels'][r][sl],cal[r]) for r in cb.ROLES}
        xi=options['ego'][0]['pose']@np.linalg.inv(options['left'][0]['pose'])
        mi=np.linalg.inv(xi)@mounts[0].camera_from_tag
        td,rd=cb.difference(mi,m)
        alternatives=[]
        for (e,ep),(u,up),(b,mp) in itertools.product(enumerate(options['ego']),enumerate(options['left']),enumerate(mounts)):
            localx=ep['pose']@np.linalg.inv(up['pose'])
            alternatives.append(dict(ego_rank=e,umi_rank=u,mount_rank=b,
                                     **mount_summary(np.linalg.inv(localx)@mp.camera_from_tag)))
        per_tag.append(dict(id=tag,**mount_summary(mi),delta_whole_mm=td,delta_whole_rotation_deg=rd,
            selected_by='lowest own-image reprojection only, never expected mount distance',
            image_center_px={r:window['pixels'][r][sl].mean(0).tolist() for r in cb.ROLES},
            fits={r:[o['score'] for o in options[r]] for r in cb.ROLES},branches=alternatives))
    folds=[]
    for parity in (0,1):
        train=[i for i in ids if (i//6+i%6)%2==parity];held=sorted(set(ids)-set(train))
        if len(train)<6 or not held:
            folds.append(dict(parity=parity,status='SKIPPED_INSUFFICIENT_SUPPORT'));continue
        mask=np.repeat([i in train for i in ids],4)
        fp={r:pose_options(window['objects'][mask],window['pixels'][r][mask],cal[r])[0]['pose'] for r in cb.ROLES}
        fx=fp['ego']@np.linalg.inv(fp['left'])
        folds.append(dict(parity=parity,status='DIAGNOSTIC',train_ids=train,heldout_ids=held,
            mount=mount_summary(np.linalg.inv(fx)@mounts[0].camera_from_tag),
            prediction={r:cb.errors(cb.project(window['objects'][~mask],fp[r],cal[r]),window['pixels'][r][~mask]) for r in cb.ROLES}))
    xyz=np.array([r['xyz_mm'] for r in per_tag])
    summary=dict(tags=len(ids),distance_range_mm=[min(r['distance_mm'] for r in per_tag),max(r['distance_mm'] for r in per_tag)],
        max_pairwise_mm=float(np.linalg.norm(xyz[:,None]-xyz[None,:],axis=2).max()),
        delta_whole_p50_mm=float(np.median([r['delta_whole_mm'] for r in per_tag])),
        delta_whole_p95_mm=float(np.percentile([r['delta_whole_mm'] for r in per_tag],95)))
    return dict(slot=window['slot'],ids=ids,whole_board=whole,per_tag=per_tag,disjoint_folds=folds,summary=summary)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv);out=cb.new_output(args.output)
    setup,cal,windows,_,hashes,audit=cb.load_session(args.session,allow_partial=True)
    if setup.get('observation_mode')!='simultaneous' or not windows:
        raise ValueError('requires saved simultaneous static board windows')
    report=dict(status='STATIC_DIAGNOSTIC_NOT_CALIBRATION',activation='NOT_ACTIVATED',
        factory_values_modified=False,source_session=str(args.session.resolve()),source_hashes=hashes,
        devices=setup['devices'],target=setup['target'],identity_policy=setup['identity_policy'],
        chain='X=T_E_B inverse(T_U_B); M=inverse(X) T_E_mount',
        limits=['same static images, no independent pose/metrology validation',
                'all recorded selected board IDs used; ID1 excluded upfront by capture policy',
                'single-tag 4-corner pose is ill-conditioned; all positive refined branches retained',
                'optical color frame z/distance are not case/glass-to-paper caliper distance',
                'fit residual is not absolute accuracy; no factory/size calibration fitted'],windows=[])
    for w in windows:
        a=next(a for a in audit if a['accepted'] and a['slot']==w['slot'])
        item=inspect_static(w,a['window_quality']['selected_common_ids'],a['same_frame_mount_corners'],cal)
        item['static_quality']=a['window_quality'];report['windows'].append(item)
        print('slot',w['slot'],'whole',item['whole_board']['mount']['xyz_mm'],
              'norm_mm',item['whole_board']['mount']['distance_mm'],'per_tag',item['summary'],flush=True)
    report['between_window_comparison']=[dict(a=a['slot'],b=b['slot'],
        mount_translation_mm=cb.difference(np.asarray(a['whole_board']['mount']['T_Ucolor_mount']),np.asarray(b['whole_board']['mount']['T_Ucolor_mount']))[0],
        x_translation_mm=cb.difference(np.asarray(a['whole_board']['T_Ecolor_Ucolor']),np.asarray(b['whole_board']['T_Ecolor_Ucolor']))[0])
        for a,b in itertools.combinations(report['windows'],2)]
    report['code_sha256']={str(s.relative_to(cb.ROOT)):cb.digest(s) for s in (Path(__file__).resolve(),
        cb.ROOT/'scripts/common_board_calibration.py',cb.ROOT/'scripts/common_board_joint.py',
        cb.ROOT/'scripts/common_board_detector.py',cb.ROOT/'three_device_slam/spatial/apriltag_detector.py')}
    out.mkdir();cb.write_json(out/'report.json',report)
    print('DIAGNOSTIC:',out/'report.json',flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
