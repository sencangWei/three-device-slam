#!/usr/bin/env python3
"""Triangulate IR corners and project into RGB without using printed dimensions."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from three_device_slam.devices.realsense_extrinsics import CONVENTION


def transform(snapshot):
    if (snapshot['schema'] != 'ego.realsense.extrinsics.v2' or snapshot['rotation_layout'] != 'row_major'
            or snapshot.get('convention') != CONVENTION or snapshot.get('source_rotation_layout') != 'column_major'):
        raise ValueError('unverified extrinsics layout')
    t = np.eye(4)
    t[:3, :3] = np.asarray(snapshot['rotation_row_major']).reshape(3, 3)
    t[:3, 3] = snapshot['translation_m']
    if (not np.isfinite(t).all() or abs(np.linalg.det(t[:3, :3])-1) > 1e-5
            or np.max(np.abs(t[:3, :3].T@t[:3, :3]-np.eye(3))) > 1e-5):
        raise ValueError('invalid rotation')
    return t


def stereo_projection(pixels, calibrations, right_from_left, color_from_left):
    """No board model, tag size, planar PnP, or fitted transform enters here."""
    rays = {r: cb.normalized_camera_points(pixels[r], calibrations[r]) for r in ('ir_left', 'ir_right')}
    q = cv2.triangulatePoints(np.eye(3, 4), right_from_left[:3], rays['ir_left'].T, rays['ir_right'].T)
    if np.any(np.abs(q[3]) < 1e-10):
        raise ValueError('triangulation at infinity')
    q = (q[:3]/q[3]).T
    if not np.isfinite(q).all() or np.any(q[:, 2] <= 0):
        raise ValueError('nonpositive triangulated depth')
    result = dict(points_ir_left_m=q.tolist(), median_depth_m=float(np.median(q[:, 2])), projections={})
    for r, t in (('ir_left', np.eye(4)), ('ir_right', right_from_left), ('color', color_from_left)):
        pred = cb.project(q, t, calibrations[r])
        result['projections'][r] = dict(**cb.errors(pred, pixels[r]), residual_px=(pred-pixels[r]).tolist())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ir-wide-threshold', action='store_true', help='Diagnostic IR detector: C=11, max window53; developed on sample5, not production')
    parser.add_argument('--ir-detection-scale', type=int, choices=(1, 3), default=1,
                        help='Decode enlarged IR only; refine corners on unchanged native pixels. Developed on sample5.')
    parser.add_argument('--edge-corners', action='store_true', help='Independent native edge intersection after same detections; diagnostic only')
    parser.add_argument('--detector-development-sample', type=int,
                        help='Explicit development sample in THIS session only; new independent sessions have none')
    parser.add_argument('--exclude-board-id', type=int, choices=range(36), action='append', default=[],
                        help='Explicit identity ambiguity exclusion across ALL streams/frames; never residual-based')
    parser.add_argument('--scale-aware-corners', action='store_true', help='Opt-in legacy board scale-aware refinement; posthoc, not frozen acceptance')
    args = parser.parse_args()
    out = cb.new_output(args.output)
    setup = json.loads((args.session/'setup.json').read_text())
    capture = json.loads((args.session/'capture_report.json').read_text())
    if setup['schema'] != 'ego.stereo_color_probe.v1' or setup['serial'] != '327122078613':
        raise ValueError('unexpected capture identity')
    if not capture['status'].startswith('CAPTURE_COMPLETE') or capture['failure'] or capture['cleanup_errors']:
        raise ValueError('incomplete capture')
    if set(capture['hashes']) != {'setup.json', 'frames.jsonl'} or set(setup['intrinsics']) != {'color', 'ir_left', 'ir_right'}:
        raise ValueError('required capture bindings missing')
    for name, h in capture['hashes'].items():
        if cb.digest(args.session/name) != h:
            raise ValueError('capture binding mismatch')
    cal = {r: cb._camera_calibration_from_intrinsics(calibration_id=capture['hashes']['setup.json'],
            frame_id='ego.'+r, intrinsics=intr) for r, intr in setup['intrinsics'].items()}
    right, color = [transform(setup['extrinsics'][r]) for r in ('ir_right', 'color')]
    rows = [json.loads(t) for t in (args.session/'frames.jsonl').read_text().splitlines()]
    if len(rows) != capture['samples']:
        raise ValueError('sample count mismatch')
    detector, results, first_detections = cb.make_detector(), [], {}
    ir_detector = cb.make_detector()
    if args.ir_wide_threshold:
        params = ir_detector.getDetectorParameters()
        params.adaptiveThreshConstant = 11
        params.adaptiveThreshWinSizeMax = 53
        ir_detector.setDetectorParameters(params)
    for row in rows:
        if row['setup_sha256'] != capture['hashes']['setup.json'] or set(row['streams']) != set(cal):
            raise ValueError('row stream/setup binding mismatch')
        ds = {}
        for r, meta in row['streams'].items():
            path = args.session/meta['file']
            if path.parent.resolve() != args.session.resolve() or cb.digest(path) != meta['sha256']:
                raise ValueError('PNG binding mismatch')
            gray = cv2.imread(str(path), 0)
            if gray is None or gray.shape != (720, 1280):
                raise ValueError('PNG shape/decode mismatch')
            scale = args.ir_detection_scale if r != 'color' else 1
            decode_image = gray if scale == 1 else cv2.resize(gray, None, fx=scale, fy=scale)
            detections = cb.detect_grid(ir_detector if r != 'color' else detector, decode_image)
            detections = [d for d in detections if d.tag_id not in args.exclude_board_id]
            for detection in detections:
                detection.corners = (detection.corners+.5)/scale-.5
            if args.scale_aware_corners:
                from scripts.fixed_factory_board_diagnostic import refine_legacy_corners_scaled
                ds[r] = refine_legacy_corners_scaled(gray, cb.valid_detections(detections))
            else:
                ds[r] = cb.refine_legacy_corners(gray, cb.valid_detections(detections))
        ids = sorted(set.intersection(*(set(d) for d in ds.values())))
        result = dict(sample=row['sample'], common_ids=ids, stream_metadata=row['streams'],
                      detector_development_sample=(row['sample'] == args.detector_development_sample))
        drift = {}
        for role, detections in ds.items():
            first = first_detections.setdefault(role, detections)
            repeated = sorted(set(first) & set(detections))
            drift[role] = float(max(np.linalg.norm(detections[i]-first[i], axis=1).max() for i in repeated)) if repeated else None
        result['drift_from_first_px'] = drift
        if len(ids) < 8:
            results.append(dict(result, status='INSUFFICIENT_COMMON_TAGS'))
            continue
        if args.edge_corners:
            from scripts.common_board_edge_diagnostic import edge_corners
            try:
                for role in ds:
                    gray = cv2.imread(str(args.session/row['streams'][role]['file']), 0)
                    ds[role] = {i: edge_corners(gray, ds[role][i]) for i in ids}
            except ValueError as exc:
                results.append(dict(result, status='EDGE_LOCALIZATION_FAILED', reason=str(exc)))
                continue
        l, r = row['streams']['ir_left'], row['streams']['ir_right']
        if l['sdk_timestamp_domain'] != r['sdk_timestamp_domain'] or abs(l['sdk_timestamp_ms']-r['sdk_timestamp_ms']) > .2:
            results.append(dict(result, status='IR_STEREO_TIMESTAMP_MISMATCH'))
            continue
        pixels = {r: np.concatenate([d[i] for i in ids]) for r, d in ds.items()}
        try:
            result.update(stereo_projection(pixels, cal, right, color), status='DIAGNOSTIC_MEASURED')
        except ValueError as exc:
            result.update(status='INVALID_GEOMETRY', reason=str(exc))
        results.append(result)
        print(row['sample'], result['status'], result.get('projections', {}).get('color', {}).get('p95_px'), flush=True)
    report = dict(status='DIAGNOSTIC_ONLY', activation='NOT_ACTIVATED', source=str(args.session.resolve()),
                  source_report_sha256=cb.digest(args.session/'capture_report.json'), setup=setup, frames=results,
                  code_sha256=cb.digest(Path(__file__).resolve()), ir_wide_threshold=args.ir_wide_threshold,
                  ir_detection_scale=args.ir_detection_scale,
                  edge_corners=args.edge_corners,
                  detector_development_sample=args.detector_development_sample,
                  identity_excluded_ids=args.exclude_board_id,
                  legacy_corner_policy='scale_aware_v1' if args.scale_aware_corners else 'fixed5',
                  print_dimensions_used=False, calibration_parameters_fitted=False,
                  limitations=['IR stereo calibration/extrinsics remain a reference, not absolute metrology',
                               'RGB exposure need not coincide with IR; static drift is recorded, inspect it',
                               'RGB residual can include RGB-to-IR extrinsics and localization errors, not only lens distortion'])
    out.mkdir(parents=True, exist_ok=False)
    cb.write_json(out/'report.json', report)


if __name__ == '__main__':
    main()
