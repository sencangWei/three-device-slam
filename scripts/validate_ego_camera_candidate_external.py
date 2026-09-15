#!/usr/bin/env python3
"""Replay fixed external-tag support with a frozen single-camera candidate."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
from scripts import common_board_calibration as cb
from scripts.validate_ego_radial_on_external_tags import SESSIONS
from three_device_slam.spatial import mixed_size_crosscheck as mix
from three_device_slam.spatial.apriltag_detector import _solve_ippe_candidates, select_ippe_candidate


def frozen_candidate(prior):
    if prior.get('activation')!='NOT_ACTIVATED':raise ValueError('candidate must be diagnostic only')
    if prior.get('status')=='CAMERA_RECALIBRATION_DEVELOPMENT_ONLY':
        c=next(c['candidate'] for c in prior['cases'] if c['candidate']['mode']=='K4_k1')
        return c,prior['devices'],'K4_k1'
    if prior.get('status')=='DEVELOPMENT_ONLY_NOT_ACCEPTANCE' and 'training_sessions' in prior:
        c=prior['candidate']
        return dict(K=c['K'],D=c['D'],distortion_model=c['model']),dict(ego=dict(
            serial=prior['camera_serial'],intrinsics=prior['factory_color_intrinsics'])),'multidepth_K4_k1'
    raise ValueError('unexpected candidate source')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    out = cb.new_output(args.output)
    prior = json.loads(args.candidate.read_text())
    c,expected_devices,candidate_label=frozen_candidate(prior)
    results = []
    for name, directory in SESSIONS:
        session = cb.ROOT/'artifacts/spatial_bench'/name
        path = session/'spatial'/directory/'pair_audit.json'
        rows = json.loads(path.read_text())
        audits = {r: mix.audit_stream(session, r)[1] for r in ('ego', 'left')}
        if any(a['bad_payloads'] or a['nonincreasing_timestamps'] for a in audits.values()):
            raise ValueError('raw integrity failure')
        cal = {r: mix.pta.load_device_calibration(session, r, r+'.color') for r in ('ego', 'left')}
        for role in expected_devices:
            snap = json.loads((session/role/'calibration.json').read_text())
            if snap['device']['serial'] != expected_devices[role]['serial']:
                raise ValueError('wrong camera identity')
            expected = cb._camera_calibration_from_intrinsics(calibration_id='expected', frame_id=role+'.color',
                      intrinsics=expected_devices[role]['intrinsics'])
            np.testing.assert_array_equal(cal[role].camera_matrix, expected.camera_matrix)
            np.testing.assert_array_equal(cal[role].distortion_coefficients, expected.distortion_coefficients)
            if (cal[role].width, cal[role].height) != (expected.width, expected.height):
                raise ValueError('image size mismatch')
        candidate = dict(cal, ego=replace(cal['ego'], camera_matrix=np.array(c['K']),
                distortion_coefficients=np.array(c['D']), distortion_model=c['distortion_model']))
        ir, evidence = mix.load_color_to_ir_snapshot(session/'left/calibration.json')
        result = dict(session=str(session), audit_sha256=cb.digest(path), raw_audit=audits,
                      calibration_sha256={r: cb.digest(session/r/'calibration.json') for r in cal},
                      ir_evidence=evidence, arms={})
        for label, cameras in (('factory', cal), (candidate_label, candidate)):
            series = {r: [] for r in mix.ROLES}
            rejected = []
            for row in rows:
                if 'geometry' not in row:
                    continue
                for role in mix.ROLES:
                    item = row['geometry']['roles'][role]
                    cam = cameras['ego' if role.startswith('ego') else 'left']
                    opts = _solve_ippe_candidates(np.array(item['corners_px']), calibration=cam,
                                                  tag_size_m=item['size_mm']/1000)
                    selection = select_ippe_candidate(opts, max_reprojection_error_px=1.5,
                        ambiguity_error_gap_px=.05, ambiguity_translation_m=.002, ambiguity_rotation_deg=2.)
                    if selection.selected_index is None:
                        rejected.append(dict(sequence=row['ego_sequence'], role=role, reason=selection.reason))
                    best = min((o for o in opts if o.positive_depth), key=lambda o:o.reprojection_error_px)
                    series[role].append(best.camera_from_tag)
            result['arms'][label] = dict(pose_gate_rejections=rejected,
                                        associated_pairs=len(series['ego_mount']), **mix.summarize_chains(series, ir))
            print(name, label, result['arms'][label]['agreement'], 'rejects', len(rejected), flush=True)
        results.append(result)
    out.mkdir()
    cb.write_json(out/'report.json', dict(status='DEVELOPMENT_REGRESSION_NOT_ACCEPTANCE', activation='NOT_ACTIVATED',
        candidate_sha256=cb.digest(args.candidate), candidate=c, sessions=results,
        limitations=['same archived identities/corners in both arms; original rejected pairs not reselected',
            'pose-gate failures retained and counted, not silently pruned',
            'previously inspected sessions, not independent metrology; no runtime changes'],
        code_sha256={str(p.relative_to(cb.ROOT)):cb.digest(p) for p in (
            Path(__file__).resolve(), cb.ROOT/'scripts/validate_ego_radial_on_external_tags.py',
            cb.ROOT/'three_device_slam/spatial/apriltag_detector.py',
            cb.ROOT/'three_device_slam/spatial/mixed_size_crosscheck.py')}))


if __name__ == '__main__':
    main()
