#!/usr/bin/env python3
"""Posthoc fixed-image corner-window contrast; original acceptance is immutable."""
import argparse
import copy
import json
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.fixed_factory_board_diagnostic import refine_legacy_corners_scaled


def selected_scaled_corners(gray, detector, ids, original):
    raw = cb.valid_detections(cb.detect_grid(detector, gray))
    if not set(ids).issubset(raw):
        raise ValueError('original selected IDs missing; no support pruning allowed')
    raw = {i: raw[i] for i in ids}
    baseline = cb.refine_legacy_corners(gray, raw)
    np.testing.assert_array_equal(np.concatenate([baseline[i] for i in ids]), original)
    scaled = refine_legacy_corners_scaled(gray, raw)
    return np.concatenate([scaled[i] for i in ids])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = cb.new_output(args.output)
    print('AUDIT: replay original raw hashes, static gates, and selected IDs.', flush=True)
    setup, factory, windows, _, hashes, audit = cb.load_session(args.session)
    if setup.get('resume_from'):
        raise ValueError('requires single complete capture')
    rows = [json.loads(t) for t in (args.session/'attempts.jsonl').read_text().splitlines()]
    altered = copy.deepcopy(windows)
    detector, shifts = cb.make_detector(), []
    for window in altered:
        accepted = next(a for a in audit if a['accepted'] and a['slot'] == window['slot'])
        ids = accepted['window_quality']['selected_common_ids']
        pairs = rows[accepted['attempt']]['pairs']
        pair = pairs[len(pairs)//2]
        for role in cb.ROLES:
            gray = cv2.imread(str(args.session/pair[role]['file']), 0)
            original = window['pixels'][role].copy()
            window['pixels'][role] = selected_scaled_corners(gray, detector, ids, original)
            shifts.append(dict(slot=window['slot'], role=role, ids=ids,
                shifts_px=(window['pixels'][role]-original).tolist()))
    report = dict(status='POSTHOC_DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED',
        source=str(args.session.resolve()), source_hashes=hashes, corner_shifts=shifts,
        original_raw_audit=audit, cases=[],
        limitations=['same original accepted windows, middle images, selected IDs and train/holdout split',
                     'only X and training board poses refitted; factory K/D, board geometry unchanged',
                     'changed corner algorithm is development, not independent calibration validation',
                     'original static gates replayed with fixed5; new-policy static acceptance not claimed',
                     'mount and original external-tag chain discrepancy not solved by this diagnostic'])
    for label, data in (('fixed5_factory', windows), ('scale_aware_factory', altered)):
        result = cb.solve_board(data, factory)
        report['cases'].append(dict(label=label, result=result))
        print(label, result['status'], 'worst_holdout_p95',
              max(v['p95_px'] for v in result['holdout']), flush=True)
    report['code_sha256'] = {str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
        Path(__file__).resolve(), cb.ROOT/'scripts/common_board_calibration.py',
        cb.ROOT/'scripts/fixed_factory_board_diagnostic.py')}
    output.mkdir(parents=True, exist_ok=False)
    cb.write_json(output/'report.json', report)


if __name__ == '__main__':
    main()
