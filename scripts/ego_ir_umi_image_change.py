#!/usr/bin/env python3
"""Offline raw-texture and epipolar diagnostics; never infer operator actions."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.ego_ir_umi_diagnostic import source
from scripts.ego_ir_umi_frozen_check import same_geometry

ROIS=dict(bottle=(200,160,430,520),left_background=(20,200,170,480),
          left_gripper=(200,525,470,700),right_gripper=(920,545,1220,700))


def flow(a,b,roi):
    mask=np.zeros_like(a);x0,y0,x1,y1=roi;mask[y0:y1,x0:x1]=255
    p=cv2.goodFeaturesToTrack(a,200,.01,8,mask=mask)
    if p is None:return dict(detected=0,retained=0,median_delta_px=None)
    q,st,err=cv2.calcOpticalFlowPyrLK(a,b,p,None,winSize=(21,21),maxLevel=3)
    z,back,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,winSize=(21,21),maxLevel=3)
    fb=np.linalg.norm(z-p,axis=2).ravel()
    valid=(st.ravel()==1)&(back.ravel()==1)&np.isfinite(q).all(axis=(1,2))&(fb<.3)&(err.ravel()<20)
    delta=(q-p).reshape(-1,2)[valid]
    return dict(detected=len(p),retained=len(delta),median_delta_px=np.median(delta,0).tolist() if len(delta) else None,
        p95_displacement_px=float(np.percentile(np.linalg.norm(delta,axis=1),95)) if len(delta) else None,
        original_points_px=p.reshape(-1,2)[valid].tolist(),new_points_px=q.reshape(-1,2)[valid].tolist(),
        note='fewer than10 tracks insufficient for a region-stability conclusion')


def patch_shift(a,b,roi,radius=24):
    x0,y0,x1,y1=map(int,roi)
    if x0-radius<0 or y0-radius<0 or x1+radius>b.shape[1] or y1+radius>b.shape[0]:
        raise ValueError('patch search outside image')
    template=a[y0:y1,x0:x1]
    if np.std(template)<1:return dict(status='NO_TEXTURE',delta_px=None)
    search=b[y0-radius:y1+radius,x0-radius:x1+radius]
    result=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED)
    _,score,_,(x,y)=cv2.minMaxLoc(result)
    delta=np.array([x-radius,y-radius],dtype=float)
    if 0<x<2*radius and 0<y<2*radius:
        for axis,values in enumerate((result[y,x-1:x+2],result[y-1:y+2,x])):
            denom=float(values[0]-2*values[1]+values[2])
            if abs(denom)>1e-8:delta[axis]+=.5*float(values[0]-values[2])/denom
    return dict(status='DIAGNOSTIC' if score>=.8 and 0<x<2*radius and 0<y<2*radius else 'UNRELIABLE',
        delta_px=delta.tolist(),ncc=float(score),roi=list(roi),radius=radius)


def epipolar_errors(ir_pixels,umi_pixels,ir_cal,umi_cal,pose):
    cb.validate_transform(pose)
    a=np.c_[cb.normalized_camera_points(ir_pixels,ir_cal),np.ones(len(ir_pixels))]
    b=cb.normalized_camera_points(umi_pixels,umi_cal)
    t=pose[:3,3];cross=np.array([[0.,-t[2],t[1]],[t[2],0.,-t[0]],[-t[1],t[0],0.]])
    line=a@(cross@pose[:3,:3]).T
    # Exact point-to-line metric in undistorted pinhole pixel coordinates.
    fx,fy=umi_cal.camera_matrix[0,0],umi_cal.camera_matrix[1,1]
    denominator=np.sqrt((line[:,0]/fx)**2+(line[:,1]/fy)**2)
    if np.any(denominator<1e-12):raise ValueError('degenerate epipolar baseline')
    distance=(np.sum(b*line[:,:2],axis=1)+line[:,2])/denominator
    return dict(p95_undistorted_px=float(np.percentile(abs(distance),95)),signed_median_px=float(np.median(distance)),
                signed_distances_px=distance.tolist())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('old','new','frozen-report','old-direct','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();out=cb.new_output(args.output)
    old,old_capture,old_rows=source(args.old);new,new_capture,new_rows=source(args.new);same_geometry(old,new)
    report=json.loads(args.frozen_report.read_text());prior=json.loads(args.old_direct.read_text())
    if report['source_hashes']!=new_capture['hashes'] or report['reference_source_hashes']!=old_capture['hashes']:
        raise ValueError('frozen report source mismatch')
    if prior['source_hashes']!=old_capture['hashes'] or [f['source'] for f in prior['frames']]!=old_rows:
        raise ValueError('old direct source mismatch')
    if [f['source'] for f in report['frames']]!=new_rows:raise ValueError('new report frame mismatch')
    cal={k:cb._camera_calibration_from_intrinsics(calibration_id=new_capture['hashes']['setup.json'],frame_id=k,intrinsics=v)
         for k,v in new['intrinsics'].items()}
    pose=np.array(report['frozen']['T_Ucolor_Eir'])
    original={k:cv2.imread(str(args.old/old_rows[0]['streams'][k]['file']),0) for k in cal}
    frames=[]
    for i,row in enumerate(new_rows):
        image={k:cv2.imread(str(args.new/row['streams'][k]['file']),0) for k in cal}
        frame=dict(sample=i,flow={k:flow(original['umi_color'],image['umi_color'],v) for k,v in ROIS.items()},
                   bottle_template=patch_shift(original['umi_color'],image['umi_color'],(265,300,330,400)),mount_template={})
        for role in ('ir_left','ir_right'):
            q=np.array(prior['frames'][0]['mount_pixels'][role]);lo=np.ceil(q.min(0)+8).astype(int);hi=np.floor(q.max(0)-8).astype(int)
            frame['mount_template'][role]=patch_shift(original[role],image[role],np.r_[lo,hi].tolist())
        obs=report['frames'][i]
        frame['epipolar']=epipolar_errors(obs['pixels']['ir_left'],obs['pixels']['umi_color'],cal['ir_left'],cal['umi_color'],pose)
        frames.append(frame)
        print(i,'bottle',frame['bottle_template'],'epi',frame['epipolar']['p95_undistorted_px'],flush=True)
    # Within-old-window control, same algorithms/ROI and a temporally separated image.
    control_image=cv2.imread(str(args.old/old_rows[10]['streams']['umi_color']['file']),0)
    control=dict(flow={k:flow(original['umi_color'],control_image,v) for k,v in ROIS.items()},
                 bottle_template=patch_shift(original['umi_color'],control_image,(265,300,330,400)))
    out.mkdir();cb.write_json(out/'report.json',dict(status='DIAGNOSTIC_PHYSICAL_CAUSE_UNRESOLVED',activation='NOT_ACTIVATED',
        operator_statement='Neither camera moved or touched; only public board repositioned',rois=ROIS,frames=frames,within_old_control=control,
        old_hashes=old_capture['hashes'],new_hashes=new_capture['hashes'],frozen_report_sha256=cb.digest(args.frozen_report),
        old_direct_sha256=cb.digest(args.old_direct),code_sha256=cb.digest(Path(__file__).resolve()),
        limitations=['ROIs selected posthoc from raw images; not physical motion ground truth',
            'Natural-texture flow/NCC independent of AprilTag corner detection; low counts and failed scores retained',
            'Bottle may itself move; gripper appearance may change; do not infer operator action',
            'Nonzero epipolar residual rules out correcting only per-point IR depth under unchanged image rays and pose',
            'No compensation applied to images, calibration, or SLAM']))


if __name__=='__main__':main()
