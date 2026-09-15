#!/usr/bin/env python3
"""Frozen multi-depth IR3D -> RGB model test, no target dimensions or activation."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.ego_stereo_color_validate import transform

TRAIN=(1,2,4,5)
HOLDOUT=(3,6)


def geometry_signature(setup):
    if (setup.get('schema')!='ego.stereo_color_probe.v1' or setup.get('serial')!='327122078613'
            or setup.get('activation')!='NOT_ACTIVATED' or setup.get('factory_values_modified') is not False):
        raise ValueError('unexpected device/schema or modified factory calibration')
    if set(setup['intrinsics'])!={'color','ir_left','ir_right'} or set(setup['extrinsics'])!={'color','ir_right'}:
        raise ValueError('missing stereo geometry')
    for snapshot in setup['extrinsics'].values():transform(snapshot)
    return dict(serial=setup['serial'],intrinsics=setup['intrinsics'],extrinsics=setup['extrinsics'])


def load_frames(root,number):
    session=root/f'ego_stereo_color_probe_20260906_run{number}'
    setup=json.loads((session/'setup.json').read_text())
    signature=geometry_signature(setup)
    capture=json.loads((session/'capture_report.json').read_text())
    if capture['failure'] or capture['cleanup_errors'] or not capture['status'].startswith('CAPTURE_COMPLETE'):
        raise ValueError('invalid capture')
    for name in ('setup.json','frames.jsonl'):
        if cb.digest(session/name)!=capture['hashes'][name]:raise ValueError('capture hash mismatch')
    rows=[json.loads(t) for t in (session/'frames.jsonl').read_text().splitlines()]
    if len(rows)!=capture['samples']:raise ValueError('row count mismatch')
    for i,row in enumerate(rows):
        if row['sample']!=i or row['setup_sha256']!=capture['hashes']['setup.json']:raise ValueError('row binding mismatch')
        for stream in row['streams'].values():
            path=session/stream['file']
            if path.parent.resolve()!=session.resolve() or cb.digest(path)!=stream['sha256']:raise ValueError('image mismatch')
    sub='joint_stereo_diagnostic_v3' if number==6 else ('solve_scale_aware_noid1' if number==5 else 'solve_ir_wide_3x_noid1')
    path=session/sub/'report.json';report=json.loads(path.read_text())
    expected=report['capture_report_sha256'] if number==6 else report['source_report_sha256']
    if cb.digest(session/'capture_report.json')!=expected:raise ValueError('derived report mismatch')
    if number!=6 and report['setup']!=setup:raise ValueError('derived setup mismatch')
    if [f['sample'] for f in report['frames']]!=list(range(len(rows))):raise ValueError('derived support incomplete or reordered')
    factory=cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],frame_id='ego.color',intrinsics=setup['intrinsics']['color'])
    t=transform(setup['extrinsics']['color']);frames=[]
    for f in report['frames']:
        if number==6:
            if 'board' not in f or f['source']!=rows[f['sample']]:raise ValueError('missing current geometry/source')
            frame=dict(sample=f['sample'],board_points=f['board']['points_ir_left_m'],board_pixels=f['board_pixels']['color'],
                mount_points=f['mount']['points_ir_left_m'],mount_pixels=f['mount_pixels']['color'],
                source_failures=f['failures'],static=f['drift_px'])
        else:
            if f['stream_metadata']!=rows[f['sample']]['streams']:raise ValueError('derived frame binding mismatch')
            if 'points_ir_left_m' not in f:
                frames.append(dict(sample=f['sample'],status=f['status']));continue
            xyz=np.asarray(f['points_ir_left_m'])
            observed=cb.project(xyz,t,factory)-np.asarray(f['projections']['color']['residual_px'])
            frame=dict(sample=f['sample'],board_points=xyz.tolist(),board_pixels=observed.tolist(),
                       source_status=f['status'],static=f['drift_from_first_px'])
        frames.append(frame)
    return factory,t,frames,signature,dict(setup=capture['hashes']['setup.json'],index=capture['hashes']['frames.jsonl'],
                                derived_report=cb.digest(path),capture=cb.digest(session/'capture_report.json'))


def fit_multidepth(datasets,factory):
    points=np.concatenate([f['board_points'] for n in TRAIN for f in datasets[n]['frames'] if 'board_points' in f]).astype(np.float32)
    pixels=np.concatenate([f['board_pixels'] for n in TRAIN for f in datasets[n]['frames'] if 'board_points' in f]).astype(np.float32)
    flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_ZERO_TANGENT_DIST|cv2.CALIB_FIX_K2|cv2.CALIB_FIX_K3
    rms,k,d,rv,tv=cv2.calibrateCamera([points],[pixels],(1280,720),factory.camera_matrix.copy(),np.zeros(5),
        flags=flags,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,200,1e-10))
    candidate=replace(factory,camera_matrix=k,distortion_coefficients=d.ravel(),distortion_model='opencv_brown_conrady')
    candidate_pose=cb.unpack(np.r_[rv[0].ravel(),tv[0].ravel()])
    return candidate,candidate_pose,dict(training_points=len(points),train_rms_px=float(rms))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv);out=cb.new_output(args.output)
    root=cb.ROOT/'artifacts/spatial_bench';datasets={};hashes={};factory=None;signature=None
    for number in range(1,7):
        camera,pose,frames,current_signature,hashes[number]=load_frames(root,number)
        if signature is not None and current_signature!=signature:raise ValueError('cross-session stereo geometry mismatch')
        signature=current_signature
        if factory is not None:
            np.testing.assert_array_equal(camera.camera_matrix,factory.camera_matrix)
            np.testing.assert_array_equal(camera.distortion_coefficients,factory.distortion_coefficients)
        factory=camera;datasets[number]=dict(frames=frames,factory_pose=pose)
    # A single rigid sensor transform across every depth; no per-board-pose fitting.
    candidate,candidate_pose,fit=fit_multidepth(datasets,factory)
    k,d=candidate.camera_matrix,candidate.distortion_coefficients
    results=[]
    for number,data in datasets.items():
        scores=[]
        for frame in data['frames']:
            score=dict(sample=frame['sample'],source_status=frame.get('source_status',frame.get('status')),
                       source_static=frame.get('static'),source_failures=frame.get('source_failures'))
            for target in ('board','mount'):
                if target+'_points' not in frame:continue
                xyz=np.asarray(frame[target+'_points']);observed=np.asarray(frame[target+'_pixels'])
                score[target]={name:cb.errors(cb.project(xyz,t,c),observed)
                               for name,c,t in [('factory',factory,data['factory_pose']),('multidepth',candidate,candidate_pose)]}
            scores.append(score)
        summary={target:{name:max(f[target][name]['p95_px'] for f in scores if target in f)
                         for name in ('factory','multidepth')} for target in ('board','mount') if any(target in f for f in scores)}
        results.append(dict(session=number,split='train' if number in TRAIN else 'holdout',frames=scores,max_p95_px=summary))
        print(number,results[-1]['split'],summary,flush=True)
    out.mkdir();cb.write_json(out/'report.json',dict(status='DEVELOPMENT_ONLY_NOT_ACCEPTANCE',activation='NOT_ACTIVATED',
        training_sessions=TRAIN,holdout_sessions=HOLDOUT,source_hashes=hashes,**fit,
        camera_serial=signature['serial'],factory_color_intrinsics=signature['intrinsics']['color'],geometry_signature=signature,
        candidate=dict(K=k.tolist(),D=d.ravel().tolist(),model='opencv_brown_conrady',T_color_ir_left=candidate_pose.tolist()),
        results=results,limitations=['legacy derived corner policy differs by session and failures retained',
            'historical poses already inspected; this is not independent product acceptance',
            'triangulation is noisy, errors-in-variables not modeled; no print dimensions enter fit',
            'no outlier rejection or per-view extrinsics; currentrun6 and every mount excluded from fitting'],
        code_sha256={str(p.relative_to(cb.ROOT)):cb.digest(p) for p in (Path(__file__).resolve(),
            cb.ROOT/'scripts/ego_stereo_color_validate.py',cb.ROOT/'scripts/common_board_calibration.py',
            cb.ROOT/'three_device_slam/spatial/apriltag_detector.py')}))
    print('CANDIDATE',k.tolist(),d.ravel().tolist(),flush=True)


if __name__=='__main__':main()
