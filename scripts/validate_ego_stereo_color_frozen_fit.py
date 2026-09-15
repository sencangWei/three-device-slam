#!/usr/bin/env python3
"""Third-pose validation of the frozen two-pose candidate; no parameter fitting."""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from scripts import common_board_calibration as cb
from scripts.crosscheck_ego_stereo_color_poses import load_bound_validation
from scripts.ego_stereo_color_validate import transform

FROZEN_SHA = '84cd4e3b4e70aa822b2000d48c4931adc475b1b5572fa03650edc243daac7e5d'
PRIMARY = ('frozen_paired_board_k1', 1e-5)


def score_frame(frame, factory, factory_t, candidate, candidate_t):
    points = np.asarray(frame['points_ir_left_m'])
    observed = cb.project(points, factory_t, factory)-np.asarray(frame['projections']['color']['residual_px'])
    prediction = cb.project(points, candidate_t, candidate)
    residual = prediction-observed
    metrics = cb.errors(prediction, observed)
    drift = frame['drift_from_first_px']
    static_ok = set(drift) == {'color', 'ir_left', 'ir_right'} and all(
        v is not None and np.isfinite(v) and v <= .75 for v in drift.values())
    meta = frame['stream_metadata']
    left, right = meta['ir_left'], meta['ir_right']
    stereo_ok = (left['sdk_timestamp_domain'] == right['sdk_timestamp_domain']
                 and abs(left['sdk_timestamp_ms']-right['sdk_timestamp_ms']) <= .2)
    return dict(sample=frame['sample'], common_ids=frame['common_ids'], metrics=metrics,
                residual_px=residual.tolist(), mean_residual_px=residual.mean(axis=0).tolist(),
                static_ok=static_ok, stereo_timestamp_ok=stereo_ok, drift_px=drift,
                geometry_support_ok=len(frame['common_ids']) >= 8, baseline=frame['projections']['color'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frozen-fit', type=Path, required=True)
    parser.add_argument('--validation', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = cb.new_output(args.output)
    if cb.digest(args.frozen_fit) != FROZEN_SHA:
        raise ValueError('candidate differs from pre-capture frozen report')
    fit = json.loads(args.frozen_fit.read_text())
    if sum((a['label'], a['jac_step']) == PRIMARY for a in fit['cases']) != 1:
        raise ValueError('primary candidate must be unique')
    target = load_bound_validation(args.validation)
    for path, digest in fit['source_validations'].items():
        if cb.digest(Path(path)) != digest:
            raise ValueError('training validation changed')
        source = load_bound_validation(Path(path))
        if Path(source['source']).resolve() == Path(target['source']).resolve():
            raise ValueError('new pose overlaps training capture')
        for key in ('serial', 'intrinsics', 'extrinsics'):
            if source['setup'][key] != target['setup'][key]:
                raise ValueError('factory geometry changed')
        for key in ('ir_wide_threshold', 'ir_detection_scale', 'edge_corners', 'identity_excluded_ids'):
            if source[key] != target[key]:
                raise ValueError('localization or identity policy changed')
        if source.get('legacy_corner_policy', 'fixed5') != target.get('legacy_corner_policy', 'fixed5'):
            raise ValueError('corner policy changed; posthoc results cannot be frozen acceptance')
    if any(f['detector_development_sample'] for f in target['frames']):
        raise ValueError('new pose was used for detector development')
    factory = cb._camera_calibration_from_intrinsics(calibration_id='bound_factory', frame_id='ego.color',
        intrinsics=target['setup']['intrinsics']['color'])
    factory_t = transform(target['setup']['extrinsics']['color'])
    result = dict(status='BLOCKED', scope='frozen_RGB_IR_projection_diagnostic_only',
        activation='NOT_ACTIVATED', mounting_calibration_accepted=False, parameters_refitted=False,
        frozen_fit_sha256=FROZEN_SHA, validation_sha256=cb.digest(args.validation),
        primary=dict(label=PRIMARY[0], jac_step=PRIMARY[1]),
        gates=dict(min_frames=3, min_common_tags=8, max_drift_px=.75, max_each_frame_p95_px=1., max_ir_gap_ms=.2),
        unmeasured=[dict(sample=f['sample'], status=f['status']) for f in target['frames']
                    if f['status'] != 'DIAGNOSTIC_MEASURED'], cases=[],
        limitations=['one heldout pose with correlated temporal samples, not final mount/SLAM acceptance',
            'IR horizontal disparity and metric depth bias remain confounded with RGB model',
            'candidate chosen before capture, all valid target corners scored, no refitting or residual pruning'])
    for arm in fit['cases']:
        if not arm['optimizer_success']:
            raise ValueError('frozen optimizer did not terminate')
        candidate = replace(factory, camera_matrix=np.array(arm['candidate_K']),
                            distortion_coefficients=np.array(arm['candidate_D']))
        candidate_t = np.array(arm['T_color_ir_left'])
        np.testing.assert_allclose(candidate_t, cb.unpack(arm['local_delta_rotvec_translation'])@factory_t, atol=1e-12)
        scored = [score_frame(f, factory, factory_t, candidate, candidate_t) for f in target['frames']
                  if f['status']=='DIAGNOSTIC_MEASURED']
        passed = len(scored) >= 3 and all(f['static_ok'] and f['stereo_timestamp_ok'] and
                    f['geometry_support_ok'] and f['metrics']['p95_px'] <= 1 for f in scored)
        item = dict(label=arm['label'], jac_step=arm['jac_step'], status='PASS' if passed else 'FAIL',
                    frames=scored, worst_p95_px=max((f['metrics']['p95_px'] for f in scored), default=None))
        result['cases'].append(item)
        if (arm['label'], arm['jac_step']) == PRIMARY:
            result['status'] = item['status']
        print(item['label'], item['jac_step'], item['status'], len(scored), item['worst_p95_px'], flush=True)
    result['code_sha256'] = cb.digest(Path(__file__).resolve())
    out.mkdir(parents=True, exist_ok=False)
    cb.write_json(out/'report.json', result)
    return 0 if result['status']=='PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
