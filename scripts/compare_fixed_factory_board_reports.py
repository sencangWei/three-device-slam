#!/usr/bin/env python3
"""Frozen-association comparison, excluding the development frame from confirmation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def metrics(residuals):
    norm = np.linalg.norm(np.asarray(residuals).reshape(-1, 2), axis=1)
    return dict(corners=len(norm), rms_px=float(np.sqrt(np.mean(norm**2))),
                p95_px=float(np.percentile(norm, 95))) if len(norm) else dict(corners=0)


def compare(before, after):
    results = []
    assert len(before['sessions']) == len(after['sessions']) == 4
    for a, b in zip(before['sessions'], after['sessions']):
        assert a['source'] == b['source'] and a['source_hashes'] == b['source_hashes']
        assert a['factory_snapshot'] == b['factory_snapshot'] and len(a['frames']) == len(b['frames'])
        residuals = [[], []]
        frame_count, improved, compared = 0, 0, 0
        excluded = []
        for fa, fb in zip(a['frames'], b['frames']):
            for key in ('sample', 'image_sha256', 'status', 'valid_tag_ids', 'preview_quality'):
                assert fa.get(key) == fb.get(key), key
            assert len(fa.get('folds', [])) == len(fb.get('folds', []))
            is_dev = Path(a['source']).name == 'board_factory_ego_normal_20260906T002443' and fa['sample'] == 0
            if is_dev:
                excluded.append(0)
            else:
                frame_count += 1
            for xa, xb in zip(fa.get('folds', []), fb.get('folds', [])):
                for key in ('train_parity', 'train_ids', 'holdout_ids', 'status'):
                    assert xa[key] == xb[key], key
                ca, cb = xa.get('corners', []), xb.get('corners', [])
                assert [(c['tag_id'],c['corner']) for c in ca] == [(c['tag_id'],c['corner']) for c in cb]
                if is_dev or xa['status'] != 'OK':
                    continue
                compared += 1
                improved += xb['holdout_rms_px'] < xa['holdout_rms_px']
                residuals[0].extend(c['residual_px'] for c in ca)
                residuals[1].extend(c['residual_px'] for c in cb)
        results.append(dict(session=Path(a['source']).name, confirmation_images=frame_count,
                            excluded_development_samples=excluded, matched_folds=compared, improved_folds=improved,
                            baseline=metrics(residuals[0]), refined5=metrics(residuals[1])))
    return dict(schema='ego.fixed_factory_board_refinement_comparison.v1',
                status='PAIRED_DIAGNOSTIC_COMPLETE_NOT_SPATIAL_ACCEPTANCE',
                all_source_hashes_ids_splits_statuses_equal=True, confirmation_images=sum(r['confirmation_images'] for r in results),
                sessions=results, factory_values_modified=False, activation='NOT_ACTIVATED',
                limitations=['199 images include frames with no sufficient training coverage; see matched folds',
                             'temporal frames/corners are correlated', 'legacy two-bit filled-junction board only',
                             'does not resolve the separate one-bit external-tag chain discrepancy'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root/'artifacts') or args.output.exists():
        raise ValueError('output must be a NEW file under repository artifacts')
    result = compare(json.loads(args.before.read_text()), json.loads(args.after.read_text()))
    result['input_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.before,args.after)}
    result['comparison_code_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with args.output.open('x') as handle:
        handle.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
