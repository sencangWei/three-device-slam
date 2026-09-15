#!/usr/bin/env python3
"""Paired same-image refinement comparison; always posthoc, never acceptance."""
import argparse
import json
from dataclasses import replace
from pathlib import Path
import numpy as np

from scripts import common_board_calibration as cb
from scripts.crosscheck_ego_stereo_color_poses import load_bound_validation
from scripts.ego_stereo_color_validate import transform
from scripts.validate_ego_stereo_color_frozen_fit import FROZEN_SHA, PRIMARY


def verify_pair(old, new):
    if old['source_report_sha256'] != new['source_report_sha256'] or old['setup'] != new['setup']:
        raise ValueError('different raw source')
    if old.get('legacy_corner_policy', 'fixed5') != 'fixed5' or new.get('legacy_corner_policy') != 'scale_aware_v1':
        raise ValueError('unexpected corner policies')
    for key in ('ir_wide_threshold', 'ir_detection_scale', 'edge_corners', 'identity_excluded_ids'):
        if old[key] != new[key]:
            raise ValueError('non-corner policy changed')
    if len(old['frames']) != len(new['frames']):
        raise ValueError('paired frame count changed')
    for a, b in zip(old['frames'], new['frames']):
        for key in ('sample', 'common_ids', 'status', 'detector_development_sample'):
            if a[key] != b[key]:
                raise ValueError('paired identities/support changed; do not compare pruned results')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before', type=Path, required=True)
    p.add_argument('--after', type=Path, required=True)
    p.add_argument('--frozen-fit', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    out = cb.new_output(args.output)
    old, new = [load_bound_validation(path) for path in (args.before, args.after)]
    verify_pair(old, new)
    if cb.digest(args.frozen_fit) != FROZEN_SHA:
        raise ValueError('candidate changed')
    arm = next(a for a in json.loads(args.frozen_fit.read_text())['cases'] if (a['label'],a['jac_step']) == PRIMARY)
    factory = cb._camera_calibration_from_intrinsics(calibration_id='bound', frame_id='ego.color',
        intrinsics=old['setup']['intrinsics']['color'])
    t = transform(old['setup']['extrinsics']['color'])
    candidate = replace(factory, camera_matrix=np.array(arm['candidate_K']), distortion_coefficients=np.array(arm['candidate_D']))
    result = dict(status='POSTHOC_DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED',
        parameters_refitted=False, source_capture_sha256=old['source_report_sha256'],
        before_sha256=cb.digest(args.before), after_sha256=cb.digest(args.after), frozen_fit_sha256=FROZEN_SHA, arms=[],
        limitations=['same image identities, no residual filtering, corner algorithm changed',
                     'development capture cannot become independent frozen-model validation',
                     'repair of corner jumps does not prove full spatial-alignment root cause fixed'])
    for label, report in (('fixed5',old),('scale_aware_v1',new)):
        records=[]
        for f in report['frames']:
            if f['status'] != 'DIAGNOSTIC_MEASURED':
                records.append(dict(sample=f['sample'],status=f['status']))
                continue
            q=np.array(f['points_ir_left_m'])
            y=cb.project(q,t,factory)-np.array(f['projections']['color']['residual_px'])
            records.append(dict(sample=f['sample'],status=f['status'],common_ids=f['common_ids'],
                factory=f['projections']['color'], drift=f['drift_from_first_px'],
                candidate=cb.errors(cb.project(q,np.array(arm['T_color_ir_left']),candidate),y)))
        result['arms'].append(dict(label=label,frames=records))
    result['code_sha256']=cb.digest(Path(__file__).resolve())
    out.mkdir(parents=True,exist_ok=False)
    cb.write_json(out/'report.json',result)


if __name__=='__main__':
    main()
