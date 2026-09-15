#!/usr/bin/env python3
"""Freeze paired-train Ego k1 and test prior mono clips, without refitting K/D."""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts import fixed_factory_board_diagnostic as fd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = cb.new_output(args.output)
    comparison = json.loads(args.comparison.read_text())
    case = next(c for c in comparison['cases'] if c['role'] == 'ego' and c['mode'] == 'k1')
    if comparison['experiment'] != 'D' or case['bound_hit'] or not case['optimizer']['success']:
        raise ValueError('expected frozen, unbounded Ego k1 diagnostic')
    report = dict(status='DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED',
                  comparison_sha256=fd.digest(args.comparison), comparison=str(args.comparison.resolve()),
                  frozen_candidate_D=case['candidate_D']['ego'], sessions=[],
                  limitations=['same physical board; not independent target metrology',
                               'frames/folds/corners are correlated, not independent sample counts',
                               'previously inspected clips, no candidate tuning on them',
                               'only per-image source pose fitted, K/D frozen',
                               'development sample0 of first Ego clip excluded from aggregate'])
    for name in fd.SESSIONS[:2]:
        path = fd.ROOT/'artifacts/spatial_bench'/name
        setup, rows, hashes = fd.verified_session(path)
        intr = setup['devices']['ego']['intrinsics']
        if intr != comparison['devices']['ego']['intrinsics']:
            raise ValueError('prior clip factory intrinsics differ')
        factory = cb._camera_calibration_from_intrinsics(calibration_id=hashes['setup.json'], frame_id='ego.color', intrinsics=intr)
        np.testing.assert_array_equal(case['candidate_K']['ego'], factory.camera_matrix)
        candidate = replace(factory, calibration_id=factory.calibration_id+'.DIAGNOSTIC',
                            distortion_coefficients=np.asarray(case['candidate_D']['ego']))
        detector, frames = fd.make_detector(), []
        aggregate = {'factory': [], 'frozen_k1': []}
        for row in rows:
            gray = cv2.imread(str(path/'ego'/row['file']), 0)
            frame = dict(sample=row['sample'], image_sha256=row['sha256'],
                         excluded_development=(name == fd.SESSIONS[0] and row['sample'] == 0))
            try:
                ds = fd.refine_legacy_corners(gray, fd.valid_detections(fd.detect_grid(detector, gray)))
            except ValueError as exc:
                frames.append(dict(frame, status='SKIPPED', reason=str(exc)))
                continue
            folds = {label: [fd.fit_fold(ds, cal, parity) for parity in (0, 1)]
                     for label, cal in (('factory', factory), ('frozen_k1', candidate))}
            frame['folds'] = folds
            for a, b in zip(folds['factory'], folds['frozen_k1']):
                if (a['status'], a['train_ids'], a['holdout_ids']) != (b['status'], b['train_ids'], b['holdout_ids']):
                    raise ValueError('paired fold support mismatch; do not compare different supports')
            for label, values in folds.items():
                if not frame['excluded_development']:
                    for fold in values:
                        if fold['status'] == 'OK':
                            aggregate[label].extend(c['residual_px'] for c in fold['corners'])
            frames.append(frame)
        summary = {}
        for label, values in aggregate.items():
            norm = np.linalg.norm(np.asarray(values), axis=1)
            summary[label] = dict(corners=len(norm), rms_px=float(np.sqrt(np.mean(norm**2))),
                                  p95_px=float(np.percentile(norm, 95)))
        report['sessions'].append(dict(source=str(path), source_hashes=hashes, summary=summary, frames=frames))
        print(name, json.dumps(summary), flush=True)
    report['code_sha256'] = {str(p.relative_to(fd.ROOT)): fd.digest(p) for p in (
        Path(__file__).resolve(), fd.ROOT/'scripts/fixed_factory_board_diagnostic.py',
        fd.ROOT/'scripts/color_aprilgrid_capture.py', fd.ROOT/'three_device_slam/spatial/apriltag_detector.py')}
    output.mkdir(parents=True, exist_ok=False)
    cb.write_json(output/'report.json', report)
    print('DIAGNOSTIC_ONLY: '+str(output/'report.json'), flush=True)


if __name__ == '__main__':
    main()
