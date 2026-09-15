#!/usr/bin/env python3
"""Exploratory two-pose SE3 fit with fixed K/D and spatially held-out corners."""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from scripts import common_board_calibration as cb
from scripts.ego_stereo_color_validate import transform
from scripts.crosscheck_ego_stereo_color_poses import load_bound_validation
from scripts.analyze_ego_stereo_color_probe import fit_local_transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validations', type=Path, nargs=2, required=True)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = cb.new_output(args.output)
    reports = [load_bound_validation(p) for p in args.validations]
    if len({Path(r['source']).resolve() for r in reports}) != 2:
        raise ValueError('requires two distinct capture sessions')
    for key in ('serial', 'intrinsics', 'extrinsics'):
        if reports[0]['setup'][key] != reports[1]['setup'][key]:
            raise ValueError('factory geometry mismatch')
    for key in ('ir_wide_threshold', 'ir_detection_scale', 'edge_corners', 'identity_excluded_ids'):
        if reports[0].get(key) != reports[1].get(key):
            raise ValueError('detector/identity policy mismatch')
    if reports[0].get('legacy_corner_policy', 'fixed5') != reports[1].get('legacy_corner_policy', 'fixed5'):
        raise ValueError('corner policy mismatch')
    comparison = json.loads(args.comparison.read_text())
    candidate = next(c for c in comparison['cases'] if c['role']=='ego' and c['mode']=='k1')
    baseline = next(c for c in comparison['cases'] if c['mode']=='factory')
    if candidate['bound_hit'] or not candidate['optimizer']['success']:
        raise ValueError('invalid frozen candidate')
    factory = cb._camera_calibration_from_intrinsics(calibration_id='bound_factory', frame_id='ego.color',
        intrinsics=reports[0]['setup']['intrinsics']['color'])
    np.testing.assert_array_equal(factory.camera_matrix, candidate['candidate_K']['ego'])
    np.testing.assert_array_equal(factory.camera_matrix, baseline['candidate_K']['ego'])
    np.testing.assert_array_equal(factory.distortion_coefficients, baseline['candidate_D']['ego'])
    frozen = replace(factory, distortion_coefficients=np.array(candidate['candidate_D']['ego']))
    extrinsic = transform(reports[0]['setup']['extrinsics']['color'])
    training, groups, exclusions = [], [], []
    for report in reports:
        rows = []
        for f in report['frames']:
            if f['status'] != 'DIAGNOSTIC_MEASURED' or f['detector_development_sample']:
                exclusions.append(dict(source=report['source'], sample=f['sample'], status=f['status'],
                                       detector_development=f['detector_development_sample']))
                continue
            q = np.array(f['points_ir_left_m'])
            y = cb.project(q, extrinsic, factory)-np.array(f['projections']['color']['residual_px'])
            odd = np.repeat([(i % 6+i//6) % 2 == 1 for i in f['common_ids']], 4)
            if (~odd).sum() < 12 or odd.sum() < 12:
                raise ValueError('insufficient parity support; no silent removal')
            rows.append((f, q, y, odd))
        if not rows:
            raise ValueError('no valid rows')
        training.append(dict(source=report['source'], sample=rows[0][0]['sample'], tag_parity=0))
        groups.append(rows)
    q_train = np.concatenate([g[0][1][~g[0][3]] for g in groups])
    y_train = np.concatenate([g[0][2][~g[0][3]] for g in groups])
    result = dict(status='EXPLORATORY_REQUIRES_THIRD_POSE', activation='NOT_ACTIVATED',
        source_validations={str(p.resolve()): cb.digest(p) for p in args.validations},
        comparison_sha256=cb.digest(args.comparison), setup=reports[0]['setup'], training=training,
        excluded=exclusions, cases=[], code_sha256=cb.digest(Path(__file__).resolve()),
        limitations=['both poses already inspected; this is NOT independent pose validation',
            'only first valid frame/even tags per pose train SE3; K/D frozen from older paired-board train8',
            'odd tags spatially held out, repeated frames correlated',
            'IR calibration/localization errors and RGB model error remain confounded; cannot activate'])
    for label, cal in (('factory', factory), ('frozen_paired_board_k1', frozen)):
        for step in (1e-5, 2e-5):
            fit = fit_local_transform(q_train, y_train, extrinsic, cal, step)
            arm = dict(label=label, jac_step=step, local_delta_rotvec_translation=fit.x.tolist(),
                T_color_ir_left=(cb.unpack(fit.x)@extrinsic).tolist(), candidate_K=cal.camera_matrix.tolist(),
                candidate_D=cal.distortion_coefficients.tolist(), optimizer_success=bool(fit.success),
                cost=float(fit.cost), optimality=float(fit.optimality), groups=[])
            for report, rows in zip(reports, groups):
                measurements = [dict(sample=f['sample'], odd_tag_heldout=cb.errors(
                    cb.project(q[odd], cb.unpack(fit.x)@extrinsic, cal), y[odd])) for f,q,y,odd in rows]
                arm['groups'].append(dict(source=report['source'], frames=measurements,
                    worst_p95_px=max(f['odd_tag_heldout']['p95_px'] for f in measurements)))
            result['cases'].append(arm)
            print(label, step, [g['worst_p95_px'] for g in arm['groups']], flush=True)
    out.mkdir(parents=True, exist_ok=False)
    cb.write_json(out/'report.json', result)


if __name__ == '__main__':
    main()
