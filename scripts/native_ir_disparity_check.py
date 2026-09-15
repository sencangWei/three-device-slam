#!/usr/bin/env python3
"""Independent local photometric disparity check; never update calibration/data."""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.crosscheck_ego_stereo_color_poses import load_bound_validation
from scripts.ego_stereo_color_validate import transform, stereo_projection
from scripts.validate_ego_stereo_color_frozen_fit import FROZEN_SHA, PRIMARY


def patch_shift(left, right, p_left, p_right, size):
    template = cv2.getRectSubPix(left.astype(np.float32), (size, size), tuple(map(float, p_left)))
    template -= template.mean()
    norm = np.linalg.norm(template)
    if norm < 1:
        raise ValueError('no template contrast')
    shifts = np.linspace(-2., 2., 81)
    costs = []
    right = right.astype(np.float32)
    for shift in shifts:
        patch = cv2.getRectSubPix(right, (size, size), (float(p_right[0]+shift), float(p_right[1])))
        patch -= patch.mean()
        denominator = norm*np.linalg.norm(patch)
        costs.append(1-float(np.sum(template*patch)/denominator) if denominator > 1 else 2.)
    best = int(np.argmin(costs))
    shift = shifts[best]
    if 0 < best < len(shifts)-1:
        a, b, c = costs[best-1:best+2]
        if a-2*b+c > 1e-12:
            shift += .05*np.clip(.5*(a-c)/(a-2*b+c), -1., 1.)
    return dict(right_x_shift_px=float(shift), correlation=float(1-costs[best]),
                search_bound_hit=best in (0, len(shifts)-1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validations', nargs='+', type=Path, required=True)
    parser.add_argument('--frozen-fit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = cb.new_output(args.output)
    if cb.digest(args.frozen_fit) != FROZEN_SHA:
        raise ValueError('frozen fit changed')
    fit = json.loads(args.frozen_fit.read_text())
    candidate = next(a for a in fit['cases'] if (a['label'], a['jac_step']) == PRIMARY)
    result = dict(status='POSTHOC_DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED',
        frozen_fit_sha256=FROZEN_SHA, sessions=[],
        limitations=['first previously valid nondevelopment frame per pose; no residual selection',
            'local horizontal translation NCC omits patch perspective warp and radiometric differences',
            'patch match not ground truth; never changes the frozen third-pose FAIL result',
            'initial IR coordinates reconstructed from triangulated points, vertical coordinate averages stereo rays',
            'all corners retained; correlations and search-bound hits recorded without pruning'])
    for path in args.validations:
        r = load_bound_validation(path)
        session = Path(r['source'])
        frame = next(f for f in r['frames'] if f['status']=='DIAGNOSTIC_MEASURED' and not f['detector_development_sample'])
        cal = {role: cb._camera_calibration_from_intrinsics(calibration_id='bound', frame_id=role, intrinsics=i)
               for role, i in r['setup']['intrinsics'].items()}
        right_t, color_t = [transform(r['setup']['extrinsics'][role]) for role in ('ir_right','color')]
        q = np.asarray(frame['points_ir_left_m'])
        lp = cb.project(q, np.eye(4), cal['ir_left'])
        rp = cb.project(q, right_t, cal['ir_right'])
        color_obs = cb.project(q, color_t, cal['color'])-np.asarray(frame['projections']['color']['residual_px'])
        images = {role: cv2.imread(str(session/frame['stream_metadata'][role]['file']), 0) for role in ('ir_left','ir_right')}
        frozen_cal = replace(cal['color'], camera_matrix=np.array(candidate['candidate_K']),
                             distortion_coefficients=np.array(candidate['candidate_D']))
        candidate_t = np.array(candidate['T_color_ir_left'])
        record = dict(source=str(session), validation_sha256=cb.digest(path), sample=frame['sample'],
                      common_ids=frame['common_ids'], baseline=cb.errors(cb.project(q, candidate_t, frozen_cal), color_obs), arms=[])
        for size in (11, 19):
            matches = [patch_shift(images['ir_left'], images['ir_right'], a,b,size) for a,b in zip(lp,rp)]
            new_rp = rp.copy()
            new_rp[:,0] += [m['right_x_shift_px'] for m in matches]
            geometry = stereo_projection(dict(ir_left=lp, ir_right=new_rp, color=color_obs), cal, right_t, color_t)
            new_q = np.array(geometry['points_ir_left_m'])
            score = cb.errors(cb.project(new_q, candidate_t, frozen_cal), color_obs)
            record['arms'].append(dict(patch_size=size, matches=matches, corrected_projection=score,
                depth_delta_mm=((new_q[:,2]-q[:,2])*1000).tolist()))
            print(session.name, size, 'RGB p95', record['baseline']['p95_px'], '->',score['p95_px'],
                  'median shift',np.median([m['right_x_shift_px'] for m in matches]), flush=True)
        result['sessions'].append(record)
    result['code_sha256'] = cb.digest(Path(__file__).resolve())
    out.mkdir(parents=True, exist_ok=False)
    cb.write_json(out/'report.json', result)


if __name__ == '__main__':
    main()
