#!/usr/bin/env python3
"""Diagnostic edge-intersection corners, frozen images/IDs, no runtime changes."""
import argparse
import copy
import json
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb


def edge_corners(gray, initial):
    corners = np.asarray(initial, dtype=float).reshape(4, 2)
    lines = []
    offsets = np.arange(-4., 4.001, .25)
    for i in range(4):
        a, b = corners[i], corners[(i+1) % 4]
        tangent = b-a
        normal = np.array([tangent[1], -tangent[0]])/np.linalg.norm(tangent)
        if normal @ ((a+b)/2-corners.mean(axis=0)) < 0:
            normal = -normal
        centers = a + np.linspace(.2, .8, 40)[:, None]*tangent
        def sample(delta):
            q = centers[:, None, :] + (offsets[None, :, None]+delta)*normal
            return cv2.remap(gray.astype(np.float32), q[:, :, 0].astype(np.float32),
                             q[:, :, 1].astype(np.float32), cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REPLICATE)
        weights = np.maximum(sample(.5)-sample(-.5), 0.)
        total = weights.sum(axis=1)
        if np.min(total) < 5:
            raise ValueError('edge contrast insufficient; do not silently discard corners')
        shift = (weights*offsets).sum(axis=1)/total
        points = centers+shift[:, None]*normal
        center = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points-center, full_matrices=False)
        n = vt[-1]
        lines.append(np.r_[n, -n@center])
    result = []
    for i in range(4):
        q = np.cross(lines[i-1], lines[i])
        if abs(q[2]) < 1e-8:
            raise ValueError('parallel edges')
        result.append(q[:2]/q[2])
    result = np.asarray(result)
    if not np.isfinite(result).all() or np.max(np.linalg.norm(result-corners, axis=1)) > 3:
        raise ValueError('edge solution too far from decoded corners')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = cb.new_output(args.output)
    setup, factory, windows, _, hashes, audit = cb.load_session(args.session)
    if setup.get('resume_from'):
        raise ValueError('this diagnostic requires a single complete capture, not a continuation')
    comparison = json.loads(args.comparison.read_text())
    k1 = next(c for c in comparison['cases'] if c['role'] == 'ego' and c['mode'] == 'k1')
    if comparison['source_hashes'] != hashes or comparison['devices'] != setup['devices'] or k1['bound_hit'] or not k1['optimizer']['success']:
        raise ValueError('frozen candidate source/optimizer mismatch')
    np.testing.assert_array_equal(k1['candidate_K']['ego'], factory['ego'].camera_matrix)
    np.testing.assert_array_equal(np.asarray(k1['candidate_D']['ego'])[1:], factory['ego'].distortion_coefficients[1:])
    modified = dict(factory, ego=replace(factory['ego'], distortion_coefficients=np.array(k1['candidate_D']['ego'])))
    edge_windows = copy.deepcopy(windows)
    rows = [json.loads(t) for t in (args.session/'attempts.jsonl').read_text().splitlines()]
    shifts = []
    for w in edge_windows:
        item = next(a for a in audit if a['slot'] == w['slot'] and a['accepted'])
        pairs = rows[item['attempt']]['pairs']
        pair = pairs[len(pairs)//2]
        for r in cb.ROLES:
            gray = cv2.imread(str(args.session/pair[r]['file']), 0)
            old = w['pixels'][r].copy()
            w['pixels'][r] = np.concatenate([edge_corners(gray, c) for c in old.reshape(-1, 4, 2)])
            shifts.append(dict(slot=w['slot'], role=r, shifts_px=(w['pixels'][r]-old).tolist()))
    report = dict(status='DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED', source_hashes=hashes,
                  comparison_sha256=cb.digest(args.comparison), cases=[], corner_shifts=shifts,
                  parameters=dict(edge_fraction=[.2, .8], edge_samples=40, normal_halfwidth_px=4.,
                                  normal_spacing_px=.25, gradient_halfstep_px=.5),
                  limitations=['same frozen IDs/middle pairs and splits; no outlier deletion',
                               'edge method is independent of checker-junction cornerSubPix but not physical metrology',
                               'Ego k1 previously fitted on the same train8; no retuning'])
    for label, ws, cal in (('corner5_factory', windows, factory), ('edge_factory', edge_windows, factory),
                           ('corner5_frozen_k1', windows, modified), ('edge_frozen_k1', edge_windows, modified)):
        result = cb.solve_board(ws, cal)
        report['cases'].append(dict(label=label, result=result))
        print(label, result['status'], 'worst_holdout_p95', max(v['p95_px'] for v in result['holdout']), flush=True)
    report['code_sha256'] = {str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
        Path(__file__).resolve(), cb.ROOT/'scripts/common_board_calibration.py',
        cb.ROOT/'scripts/fixed_factory_board_diagnostic.py', cb.ROOT/'three_device_slam/spatial/apriltag_detector.py')}
    output.mkdir(parents=True, exist_ok=False)
    cb.write_json(output/'report.json', report)


if __name__ == '__main__':
    main()
