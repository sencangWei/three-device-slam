#!/usr/bin/env python3
"""Replay rejected v1 windows with v2 visibility gate; never relabel raw capture."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb


def audit(session):
    session = Path(session).resolve()
    setup = json.loads((session/'setup.json').read_text())
    capture = json.loads((session/'capture_report.json').read_text())
    if setup['schema'] != 'ego.common_board_static_capture.v1':
        raise ValueError('this comparison expects preserved version1 evidence')
    if setup['target'] != cb.TARGET or setup['factory_values_modified']:
        raise ValueError('target/factory mismatch')
    hashes = {name:cb.digest(session/name) for name in ('setup.json','attempts.jsonl','capture_report.json')}
    if hashes['setup.json'] != capture['setup_sha256'] or hashes['attempts.jsonl'] != capture['attempts_sha256']:
        raise ValueError('source binding mismatch')
    rows = [json.loads(line) for line in (session/'attempts.jsonl').read_text().splitlines()]
    if len(rows) != capture['attempts']:
        raise ValueError('source count mismatch')
    detector, mount_detector = cb.make_detector(),cb.mount_detector()
    results,previous = [],{}
    for row in rows:
        samples,image_hashes = [],{}
        for j,pair in enumerate(row['pairs']):
            images = {}
            for role in cb.ROLES:
                meta = pair[role]
                expected = f'attempt_{row["attempt"]:03d}/{role}_{j:03d}.png'
                if meta['file'] != expected or cb.digest(session/expected) != meta['sha256']:
                    raise ValueError('image binding mismatch')
                image_hashes[expected] = meta['sha256']
                image = cv2.imread(str(session/expected),0)
                intr = setup['devices'][role]['intrinsics']
                if image is None or image.shape != (intr['height'],intr['width']):
                    raise ValueError('image shape mismatch')
                for field in ('frame_number','sdk_timestamp_ms','host_arrival_monotonic_ns'):
                    if not np.isfinite(meta[field]) or meta[field] <= previous.get((role,field),-1):
                        raise ValueError('timestamp/counter regression')
                    previous[role,field] = meta[field]
                images[role] = image
            detected,reasons = cb.detect_sample(images,row['kind'],detector,mount_detector)
            arrivals = [pair[r]['host_arrival_monotonic_ns'] for r in cb.ROLES]
            samples.append(dict(t=max(arrivals)/1e9,arrival_gap_ms=abs(arrivals[0]-arrivals[1])/1e6,
                                detected=detected,reasons=reasons))
        quality = cb.window_quality(samples,row['kind'])
        results.append(dict(attempt=row['attempt'],slot=row['slot'],original_accepted=row['accepted'],
                            original_reasons=row['reasons'],v2_static_window=quality,
                            image_hashes=image_hashes))
    return dict(schema='ego.common_board_visibility_replay.v1',status='DIAGNOSTIC_ONLY',
                source=str(session),source_hashes=hashes,original_capture=capture,
                original_gates=setup['gates'],new_gates=cb.GATES,attempts=results,
                code_sha256={str(p):cb.digest(p) for p in (Path(__file__).resolve(),Path(cb.__file__).resolve())},
                activation='NOT_ACTIVATED',
                limitations=['development replay of3 same-pose attempts, not independent HIL acceptance',
                             'v1 raw FAIL is preserved; no completed calibration session or parameters produced'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = cb.new_output(args.output)
    report = audit(args.session)
    output.mkdir(parents=True,exist_ok=False)
    cb.write_json(output/'report.json',report)
    for a in report['attempts']:
        q=a['v2_static_window']
        print(json.dumps(dict(attempt=a['attempt'],old=a['original_reasons'],new=q['reasons'],
                              metrics=q['roles'],selected_common=len(q['selected_common_ids']))),flush=True)
    print('DIAGNOSTIC ONLY:',output/'report.json',flush=True)


if __name__ == '__main__':
    main()
