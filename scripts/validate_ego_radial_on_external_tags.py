#!/usr/bin/env python3
"""Freeze paired-board Ego k1; replay archived one-bit external-tag corners."""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from scripts import common_board_calibration as cb
from three_device_slam.spatial import mixed_size_crosscheck as mix
from three_device_slam.spatial.apriltag_detector import _solve_ippe_candidates, select_ippe_candidate, AprilTagDetectorConfig

SESSIONS = (
    ('mixed80_crosscheck_20260905T215930', 'mixed80_40_apriltag'),
    ('mixed80_newview_20260905T225115', 'mixed80_40_apriltag_factory_model_v3'),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('k1', 'focal_k1'), default='k1')
    args = parser.parse_args()
    output = cb.new_output(args.output)
    prior = json.loads(args.comparison.read_text())
    case = next(c for c in prior['cases'] if c['role'] == 'ego' and c['mode'] == args.mode)
    if case['bound_hit'] or not case['optimizer']['success']:
        raise ValueError('unbounded terminated candidate required')
    report = dict(status='DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED',
                  comparison_sha256=cb.digest(args.comparison), candidate_mode=args.mode,
                  frozen_K=case['candidate_K']['ego'], frozen_D=case['candidate_D']['ego'],
                  sessions=[], limitations=['archived accepted identities/corners held fixed in both arms',
                      'raw CRC checked; original rejected pairs not reselected',
                      'best positive pose retained for diagnostics even if ambiguity/reprojection gate rejects; rejects counted',
                      'full3D chain consistency is not absolute accuracy; no deployment'])
    for name, directory in SESSIONS:
        session = cb.ROOT/'artifacts/spatial_bench'/name
        path = session/'spatial'/directory/'pair_audit.json'
        rows = json.loads(path.read_text())
        audits = {r: mix.audit_stream(session, r)[1] for r in ('ego', 'left')}
        if any(a['bad_payloads'] or a['nonincreasing_timestamps'] for a in audits.values()):
            raise ValueError('raw integrity failure')
        factory = {r: mix.pta.load_device_calibration(session, r, r+'.color') for r in ('ego', 'left')}
        base = next(c for c in prior['cases'] if c['mode'] == 'factory')
        np.testing.assert_array_equal(factory['ego'].camera_matrix, base['candidate_K']['ego'])
        np.testing.assert_array_equal(factory['ego'].distortion_coefficients, base['candidate_D']['ego'])
        candidate = dict(factory, ego=replace(factory['ego'], camera_matrix=np.array(case['candidate_K']['ego']),
                                             distortion_coefficients=np.array(case['candidate_D']['ego'])))
        ir, ir_evidence = mix.load_color_to_ir_snapshot(session/'left/calibration.json')
        result = dict(session=str(session), audit_sha256=cb.digest(path), raw_audit=audits,
                      calibration_sha256={r: cb.digest(session/r/'calibration.json') for r in factory},
                      ir_evidence=ir_evidence, arms={})
        for label, cameras in (('factory', factory), ('frozen_k1', candidate)):
            series = {r: [] for r in mix.ROLES}
            rejected = []
            for row in rows:
                if 'geometry' not in row:
                    continue
                for role in mix.ROLES:
                    item = row['geometry']['roles'][role]
                    cam = cameras['ego' if role.startswith('ego') else 'left']
                    size = item['size_mm']/1000
                    corners = np.asarray(item['corners_px'])
                    candidates = _solve_ippe_candidates(corners, calibration=cam, tag_size_m=size)
                    cfg = AprilTagDetectorConfig(family='tag36h11', allowed_tag_ids=(1, 2), tag_size_m=size, ambiguity_rotation_deg=2,
                                                ambiguity_translation_m=.002)
                    selection = select_ippe_candidate(candidates, max_reprojection_error_px=cfg.max_reprojection_error_px,
                        ambiguity_error_gap_px=cfg.ambiguity_error_gap_px,
                        ambiguity_translation_m=cfg.ambiguity_translation_m, ambiguity_rotation_deg=cfg.ambiguity_rotation_deg)
                    if selection.selected_index is None:
                        rejected.append(dict(ego_sequence=row['ego_sequence'], role=role, reason=selection.reason))
                    best = min((c for c in candidates if c.positive_depth), key=lambda c: c.reprojection_error_px)
                    series[role].append(best.camera_from_tag)
            result['arms'][label] = dict(associated_pairs=len(series['ego_mount']),
                                         pose_gate_rejections=rejected, **mix.summarize_chains(series, ir))
            print(name, label, json.dumps(result['arms'][label]['agreement']), 'rejections', len(rejected), flush=True)
        report['sessions'].append(result)
    report['code_sha256'] = {str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
        Path(__file__).resolve(), cb.ROOT/'three_device_slam/spatial/apriltag_detector.py',
        cb.ROOT/'three_device_slam/spatial/mixed_size_crosscheck.py')}
    output.mkdir(parents=True, exist_ok=False)
    cb.write_json(output/'report.json', report)


if __name__ == '__main__':
    main()
