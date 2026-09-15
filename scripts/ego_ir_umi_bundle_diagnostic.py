#!/usr/bin/env python3
"""Posthoc errors-in-variables diagnostic; train first10 static frames, test last10."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scripts import common_board_calibration as cb
from scripts.ego_ir_umi_diagnostic import source, triangulate, fit_observed_group, GROUPS, GATES
from scripts.ego_stereo_color_validate import transform


def bundle_fit(pixels, cal, right, seed, points):
    """Fit one rigid link and latent points against ALL THREE measured image rays."""
    n=len(points)
    rays={k:cb.normalized_camera_points(v,cal[k]) for k,v in pixels.items()}
    scales={k:np.diag(cal[k].camera_matrix)[:2] for k in pixels}
    def fun(parameters):
        pose=cb.unpack(parameters[:6]);xyz=parameters[6:].reshape(n,3)
        projected=[]
        for k,t in (('ir_left',np.eye(4)),('ir_right',right),('umi_color',pose)):
            q=cb.transform_points(xyz,t)
            if not np.isfinite(q).all() or np.any(q[:,2]<=0):
                raise ValueError('bundle nonpositive depth')
            projected.append(((q[:,:2]/q[:,2,None]-rays[k])*scales[k]).ravel())
        return np.concatenate(projected)
    sparsity=lil_matrix((6*n,6+3*n),dtype=int)
    for stream in range(3):
        for i in range(n):
            rows=slice(stream*2*n+i*2,stream*2*n+i*2+2)
            sparsity[rows,6+i*3:6+i*3+3]=1
            if stream==2:sparsity[rows,:6]=1
    result=least_squares(fun,np.r_[cb.pack(seed),points.ravel()],jac_sparsity=sparsity.tocsr(),
        x_scale='jac',max_nfev=200,ftol=1e-9,xtol=1e-9,gtol=1e-9,
        tr_options={'atol':1e-10,'btol':1e-10})
    return dict(T_Ucolor_Eir=cb.unpack(result.x[:6]).tolist(),points_ir_m=result.x[6:].reshape(n,3).tolist(),
        optimizer=dict(success=bool(result.success),nfev=result.nfev,message=result.message,cost=float(result.cost)),
        objective='all IR-L/IR-R/UMI normalized rays scaled by factory fx/fy; linear, no outlier pruning')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--direct-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=cb.new_output(args.output)
    setup,capture,rows=source(args.session)
    direct=json.loads(args.direct_report.read_text());frames=direct['frames']
    if direct['source_hashes']!=capture['hashes'] or direct['capture_report_sha256']!=cb.digest(args.session/'capture_report.json'):
        raise ValueError('derived report source binding mismatch')
    if len(frames)!=20 or [f['source'] for f in frames]!=rows:
        raise ValueError('exact20 complete frames required')
    for f in frames:
        if (f['ir_skew_ms']>GATES['ir_skew_ms'] or f['host_arrival_gap_ms']>GATES['host_arrival_gap_ms'] or
            any(v>GATES['static_drift_px'] for s in f['drift_px'].values() for v in s.values())):
            raise ValueError('static or timing failed')
    cal={k:cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],frame_id=k,intrinsics=v)
         for k,v in setup['intrinsics'].items()}
    right=transform(setup['right_from_left'])
    pixels={k:np.mean([f['board_pixels'][k] for f in frames[:10]],axis=0) for k in cal}
    points,_=triangulate(pixels['ir_left'],pixels['ir_right'],cal['ir_left'],cal['ir_right'],right)
    # Placeholder mount only for seed-return convenience; never optimized.
    seed_mount=np.array(frames[0]['mount_ir3d_m'])
    fits={}
    for name,ids in GROUPS.items():
        mask=np.repeat([i in ids for i in range(36)],4)
        plain=fit_observed_group(points,pixels['umi_color'],pixels['ir_left'],seed_mount,cal['umi_color'],cal['ir_left'],ids)
        bundle=bundle_fit({k:v[mask] for k,v in pixels.items()},cal,right,np.array(plain['T_Ucolor_Eir']),points[mask])
        cases={}
        for method,result in (('direct_mean',plain),('joint_rays',bundle)):
            pose=np.array(result['T_Ucolor_Eir']);scores=[]
            for f in frames[10:]:
                xyz=np.array(f['board_ir3d_m']);observed=np.array(f['board_pixels']['umi_color'])
                center=cb.transform_points(np.mean(f['mount_ir3d_m'],axis=0)[None],pose)[0]*1000
                scores.append(dict(sample=f['sample'],all_board=cb.errors(cb.project(xyz,pose,cal['umi_color']),observed),
                    spatial_holdout=cb.errors(cb.project(xyz[~mask],pose,cal['umi_color']),observed[~mask]) if np.any(~mask) else None,
                    mount_center_Ucolor_mm=center.tolist()))
            cases[method]=dict(fit=result,holdout=scores)
        fits[name]=cases
        print('FIT',name,bundle['optimizer'],flush=True)
    summary={}
    for method in ('direct_mean','joint_rays'):
        summary[method]=dict(all36_holdout_max_p95_px=max(f['all_board']['p95_px'] for f in fits['all36'][method]['holdout']),
            spatial_holdout_max_p95_px=max(f['spatial_holdout']['p95_px'] for name in fits if name!='all36' for f in fits[name][method]['holdout']),
            contrasts={k:[float(np.linalg.norm(np.array(a['mount_center_Ucolor_mm'])-b['mount_center_Ucolor_mm']))
                for a,b in zip(fits[k+'_0'][method]['holdout'],fits[k+'_1'][method]['holdout'])] for k in ('horizontal','vertical','checker')})
    out.mkdir()
    cb.write_json(out/'report.json',dict(status='POSTHOC_DIAGNOSTIC_NOT_ACCEPTANCE',activation='NOT_ACTIVATED',summary=summary,fits=fits,
        train_samples=list(range(10)),holdout_samples=list(range(10,20)),source_hashes=capture['hashes'],
        direct_report_sha256=cb.digest(args.direct_report),code_sha256=cb.digest(Path(__file__).resolve()),
        limitations=['Posthoc algorithm development; last10 not unseen experiment; same scene not independent-view validation',
            'Triangulated heldout IR points never refitted using heldout UMI pixels; mount excluded from link fitting',
            'Direct source size/projection failures remain; no gates relaxed or factory calibration changed']))
    print('SUMMARY',json.dumps(summary),flush=True)


if __name__=='__main__':main()
