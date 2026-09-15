#!/usr/bin/env python3
"""Frozen candidate crosscheck against IR triangulation; never activate calibration."""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from scripts import common_board_calibration as cb
from scripts.ego_stereo_color_validate import transform


def fit_local_transform(points, observed, reference, calibration, step):
    def residual(x):
        return (cb.project(points, cb.unpack(x)@reference, calibration)-observed).ravel()
    def jacobian(x):
        offsets = np.eye(6)*step
        return np.column_stack([(residual(x+v)-residual(x-v))/(2*step) for v in offsets])
    return least_squares(residual, np.zeros(6), jac=jacobian)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validation', type=Path, required=True)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--jac-step', type=float, default=1e-5,
                        help='Central SE3 finite difference radians/meters, above SDK float32 pixel quantization')
    args = parser.parse_args()
    if not 1e-6 <= args.jac_step <= 1e-3:
        raise ValueError('jac step out of diagnostic range')
    out = cb.new_output(args.output)
    report = json.loads(args.validation.read_text())
    comparison = json.loads(args.comparison.read_text())
    source = Path(report['source'])
    if cb.digest(source/'capture_report.json') != report['source_report_sha256']:
        raise ValueError('capture report binding mismatch')
    capture = json.loads((source/'capture_report.json').read_text())
    for name in ('setup.json', 'frames.jsonl'):
        if cb.digest(source/name) != capture['hashes'][name]:
            raise ValueError('capture source changed')
    setup = report['setup']
    if setup != json.loads((source/'setup.json').read_text()):
        raise ValueError('embedded setup changed')
    factory = cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],
        frame_id='ego.color', intrinsics=setup['intrinsics']['color'])
    baseline = next(c for c in comparison['cases'] if c['mode'] == 'factory')
    candidate = next(c for c in comparison['cases'] if c['role'] == 'ego' and c['mode'] == 'k1')
    if candidate['bound_hit'] or not candidate['optimizer']['success']:
        raise ValueError('invalid diagnostic candidate')
    np.testing.assert_array_equal(baseline['candidate_K']['ego'], factory.camera_matrix)
    np.testing.assert_array_equal(baseline['candidate_D']['ego'], factory.distortion_coefficients)
    np.testing.assert_array_equal(candidate['candidate_K']['ego'], factory.camera_matrix)
    np.testing.assert_array_equal(candidate['candidate_D']['ego'][1:], factory.distortion_coefficients[1:])
    frozen = replace(factory, distortion_coefficients=np.array(candidate['candidate_D']['ego']))
    t = transform(setup['extrinsics']['color'])
    frames = []
    excluded = []
    for f in report['frames']:
        if f['status'] != 'DIAGNOSTIC_MEASURED' or f['detector_development_sample']:
            excluded.append(dict(sample=f['sample'], status=f['status'], development=f['detector_development_sample']))
            continue
        q = np.array(f['points_ir_left_m'])
        y = cb.project(q, t, factory)-np.array(f['projections']['color']['residual_px'])
        parity = np.repeat([(i % 6+i//6) % 2 for i in f['common_ids']], 4)
        frames.append((f, q, y, parity))
    if not frames:
        raise ValueError('no independent diagnostic frames')
    # One first non-development frame and even checker parity only. Odd tags
    # remain spatially held out; later frames are correlated, not new poses.
    first, q, y, parity = frames[0]
    mask = parity == 0
    if mask.sum() < 12 or (~mask).sum() < 12:
        raise ValueError('insufficient parity support')
    output = dict(status='DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED',
        validation_sha256=cb.digest(args.validation), comparison_sha256=cb.digest(args.comparison),
        code_sha256=cb.digest(Path(__file__).resolve()), excluded=excluded, cases=[],
        local_pose_fit=dict(sample=first['sample'], train_tag_parity=0, heldout_tag_parity=1,
                            central_jac_step=args.jac_step),
        limitations=['single stationary board pose: cannot uniquely separate RGB K/D, RGB-IR extrinsics and IR systematic errors',
            'IR epipolar reprojection does not bound horizontal disparity bias or metric depth accuracy',
            'time samples correlated; detector development sample5 excluded, missing supports explicitly retained',
            'local SE3 fitting is diagnostic only, cannot become a new device calibration',
            'all projections use same fixed IR triangulated points; no board dimensions or planar pose assumption'])
    for label, cal in (('factory', factory), ('frozen_paired_board_k1', frozen)):
        fit = fit_local_transform(q[mask], y[mask], t, cal, args.jac_step)
        arm = dict(label=label, candidate_K=cal.camera_matrix.tolist(), candidate_D=cal.distortion_coefficients.tolist(),
            local_delta_rotvec_translation=fit.x.tolist(), optimizer_success=bool(fit.success),
            optimizer_optimality=float(fit.optimality), frames=[])
        for f, points, observed, pt in frames:
            odd = pt == 1
            base_pred = cb.project(points, t, cal)
            arm['frames'].append(dict(sample=f['sample'], common_ids=f['common_ids'],
                ir_left=f['projections']['ir_left'], drift=f['drift_from_first_px'],
                fixed_extrinsics=cb.errors(base_pred, observed), mean_residual_px=(base_pred-observed).mean(0).tolist(),
                local_extrinsic_fit_odd_tags=cb.errors(cb.project(points[odd], cb.unpack(fit.x)@t, cal), observed[odd])))
        output['cases'].append(arm)
        print(label, 'fixed p95 range', min(f['fixed_extrinsics']['p95_px'] for f in arm['frames']),
              max(f['fixed_extrinsics']['p95_px'] for f in arm['frames']),
              'local SE3 heldout worst', max(f['local_extrinsic_fit_odd_tags']['p95_px'] for f in arm['frames']), flush=True)
    out.mkdir(parents=True, exist_ok=False)
    cb.write_json(out/'report.json', output)


if __name__ == '__main__':
    main()
