#!/usr/bin/env python3
"""Evaluate a previously fitted local transform on a new pose, without refitting."""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from scripts import common_board_calibration as cb
from scripts.ego_stereo_color_validate import transform


def load_bound_validation(path):
    report = json.loads(path.read_text())
    session = Path(report['source'])
    capture = json.loads((session/'capture_report.json').read_text())
    if cb.digest(session/'capture_report.json') != report['source_report_sha256']:
        raise ValueError('capture report changed')
    if capture['failure'] or capture['cleanup_errors'] or not capture['status'].startswith('CAPTURE_COMPLETE'):
        raise ValueError('incomplete capture')
    for name in ('setup.json', 'frames.jsonl'):
        if cb.digest(session/name) != capture['hashes'][name]:
            raise ValueError('capture binding changed')
    if json.loads((session/'setup.json').read_text()) != report['setup']:
        raise ValueError('embedded setup changed')
    rows = [json.loads(line) for line in (session/'frames.jsonl').read_text().splitlines()]
    if len(rows) != capture['samples'] or len(rows) != len(report['frames']):
        raise ValueError('frame count mismatch')
    previous = {}
    for index, row in enumerate(rows):
        derived = report['frames'][index]
        if derived['sample'] != row['sample'] or derived['stream_metadata'] != row['streams']:
            raise ValueError('derived frame metadata mismatch')
        if derived['status'] == 'DIAGNOSTIC_MEASURED':
            count = 4*len(derived['common_ids'])
            points = np.asarray(derived['points_ir_left_m'])
            residual = np.asarray(derived['projections']['color']['residual_px'])
            if (points.shape != (count, 3) or residual.shape != (count, 2)
                    or not np.isfinite(points).all() or not np.isfinite(residual).all()):
                raise ValueError('derived point/residual count or finiteness mismatch')
        if row['sample'] != index or row['setup_sha256'] != capture['hashes']['setup.json']:
            raise ValueError('sample identity mismatch')
        if set(row['streams']) != {'color', 'ir_left', 'ir_right'}:
            raise ValueError('stream identity mismatch')
        for role, meta in row['streams'].items():
            if meta['file'] != f'{index:03d}_{role}.png' or cb.digest(session/meta['file']) != meta['sha256']:
                raise ValueError('image identity/hash mismatch')
            stamp, count, domain = meta['sdk_timestamp_ms'], meta['frame_number'], meta['sdk_timestamp_domain']
            if not np.isfinite(stamp) or (role in previous and
                    (stamp <= previous[role][0] or count <= previous[role][1] or domain != previous[role][2])):
                raise ValueError('timestamp/counter regression')
            previous[role] = stamp, count, domain
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-validation', type=Path, required=True)
    parser.add_argument('--frozen-fit', type=Path, required=True)
    parser.add_argument('--validation', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = cb.new_output(args.output)
    source = load_bound_validation(args.source_validation)
    target = load_bound_validation(args.validation)
    fit = json.loads(args.frozen_fit.read_text())
    if fit['validation_sha256'] != cb.digest(args.source_validation):
        raise ValueError('frozen fit source mismatch')
    if not fit['local_pose_fit'].get('central_jac_step'):
        raise ValueError('obsolete quantized default-jacobian fit')
    if source['source'] == target['source']:
        raise ValueError('requires different capture')
    for key in ('serial', 'intrinsics', 'extrinsics'):
        if source['setup'][key] != target['setup'][key]:
            raise ValueError('factory geometry mismatch')
    for key in ('ir_wide_threshold', 'ir_detection_scale', 'edge_corners'):
        # The original validator predates the opt-in edge implementation.
        if source.get(key, False) != target.get(key, False):
            raise ValueError('detector/localization changed')
    if source.get('legacy_corner_policy', 'fixed5') != target.get('legacy_corner_policy', 'fixed5'):
        raise ValueError('corner policy changed')
    # ID1 is odd parity and did not enter the old even-parity local fit.
    if any((i % 6+i//6) % 2 == fit['local_pose_fit']['train_tag_parity']
           for i in target.get('identity_excluded_ids', [])):
        raise ValueError('new identity exclusion overlaps frozen fit training parity')
    setup = target['setup']
    factory = cb._camera_calibration_from_intrinsics(calibration_id='bound_factory', frame_id='ego.color',
        intrinsics=setup['intrinsics']['color'])
    extrinsic = transform(setup['extrinsics']['color'])
    result = dict(status='DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED', parameters_refitted=False,
        source_validation_sha256=cb.digest(args.source_validation), target_validation_sha256=cb.digest(args.validation),
        frozen_fit_sha256=cb.digest(args.frozen_fit), code_sha256=cb.digest(Path(__file__).resolve()),
        source=str(args.source_validation.resolve()), target=str(args.validation.resolve()),
        threshold_px=1., cases=[],
        identity_excluded_ids=target.get('identity_excluded_ids', []),
        limitations=['IR epipolar residual cannot bound horizontal disparity bias or absolute metric depth',
            'one newly heldout board pose, correlated temporal samples; not a full mounting calibration acceptance',
            'local SE3 only from prior pose/even tags; all new-pose corners scored, no refit or residual pruning'])
    for arm in fit['cases']:
        if not arm['optimizer_success']:
            raise ValueError('frozen fit did not terminate')
        cal = replace(factory, camera_matrix=np.array(arm['candidate_K']), distortion_coefficients=np.array(arm['candidate_D']))
        corrected = cb.unpack(arm['local_delta_rotvec_translation'])@extrinsic
        records = []
        for frame in target['frames']:
            item = dict(sample=frame['sample'], input_status=frame['status'], common_ids=frame['common_ids'])
            if frame['status'] != 'DIAGNOSTIC_MEASURED' or frame['detector_development_sample']:
                records.append(dict(item, status='UNMEASURED'))
                continue
            points = np.array(frame['points_ir_left_m'])
            observed = cb.project(points, extrinsic, factory)-np.array(frame['projections']['color']['residual_px'])
            baseline = cb.errors(cb.project(points, extrinsic, factory), observed)
            predicted = cb.errors(cb.project(points, corrected, cal), observed)
            records.append(dict(item, status='DIAGNOSTIC_MEASURED', factory=baseline, frozen_candidate=predicted,
                ir_left=frame['projections']['ir_left'], drift=frame['drift_from_first_px'],
                median_depth_m=frame['median_depth_m']))
        measured = [f for f in records if f['status']=='DIAGNOSTIC_MEASURED']
        if not measured:
            raise ValueError('no new-pose measurements')
        worst = max(f['frozen_candidate']['p95_px'] for f in measured)
        result['cases'].append(dict(label=arm['label'], frames=records, measured_frames=len(measured),
            worst_p95_px=worst, prediction_threshold_status='FAIL' if worst>1 else 'WITHIN_PIXEL_BOUND_ONLY'))
        print(arm['label'], len(measured), 'frames; worst heldout p95', worst, flush=True)
    out.mkdir(parents=True, exist_ok=False)
    cb.write_json(out/'report.json', result)


if __name__ == '__main__':
    main()
