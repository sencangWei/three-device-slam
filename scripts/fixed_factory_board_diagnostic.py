#!/usr/bin/env python3
"""Offline legacy-board prediction. Factory K/D fixed; never activate calibration."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from scripts.color_aprilgrid_capture import make_detector, detect_grid
from three_device_slam.spatial.apriltag_detector import normalized_camera_points, project_camera_points
from three_device_slam.spatial.pair_tag_alignment import _camera_calibration_from_intrinsics

ROOT = Path(__file__).resolve().parents[1]
SESSIONS = (
    'board_factory_ego_normal_20260906T002443',
    'board_factory_ego_rot180_run2_20260906T0030',
    'board_factory_umi_normal_20260906T003607',
    'board_factory_umi_rot180_20260906T0040',
)
TAG_M, GAP_M = .0352, .01056


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def object_corners(tag_id):
    if not 0 <= tag_id < 36:
        raise ValueError('unsupported board tag')
    center = np.array([tag_id % 6, tag_id // 6, 0.]) * (TAG_M + GAP_M)
    # Board x follows increasing column, y increasing row. Frozen legacy print
    # convention, NOT selected by evaluating any test-image held-out corners.
    return center + .5 * TAG_M * np.array([[1., -1, 0], [-1, -1, 0], [-1, 1, 0], [1, 1, 0]])


def valid_detections(detections):
    result, seen = {}, set()
    for detection in detections:
        tag = int(detection.tag_id)
        if tag not in range(36):
            continue
        if tag in seen:
            raise ValueError('duplicate board ID')
        seen.add(tag)
        corners = np.asarray(detection.corners, dtype=float).reshape(4, 2)
        # Per-tag gate only: no all-board pose or lattice fit used as a gate.
        if np.isfinite(corners).all() and np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1).min() >= 15:
            result[tag] = corners
    return result


def refine_legacy_corners(gray, detected):
    """Only for this two-bit board with filled black junction squares.

    Explicit5px half-window avoids ArUco's relative-module window clamp.
    Preserve the original decoded IDs and input arrays for paired comparisons.
    Not a general-purpose refinement for the separate one-bit spatial tags.
    """
    return {tag: cv2.cornerSubPix(gray, np.array(corners, dtype=np.float32).reshape(-1,1,2),
                                  (5,5), (-1,-1),
                                  (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, .001)).reshape(4,2).astype(float)
            for tag, corners in detected.items()}


def refine_legacy_corners_scaled(gray, detected):
    """Opt-in scale-aware recovery for the two-bit filled-junction board only.

    Coarse contours can land inside the outer border. A fixed5px search fails
    on large tags; 1.5 code cells of the ten-cell black square reaches the edge.
    Original fixed5 function stays unchanged for frozen calibration evidence.
    """
    result = {}
    for tag, corners in detected.items():
        corners = np.asarray(corners, dtype=float).reshape(4, 2)
        edge = np.median(np.linalg.norm(corners-np.roll(corners, 1, axis=0), axis=1))
        window = max(5, int(np.ceil(.15*edge)))
        result[tag] = cv2.cornerSubPix(gray, corners.astype(np.float32).reshape(-1, 1, 2),
            (window, window), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, .001)).reshape(4, 2).astype(float)
    return result


def projected(objects, rotation, translation, calibration):
    return project_camera_points(objects @ rotation.T + translation, calibration)


def fit_fold(detected, calibration, parity):
    train = sorted(i for i in detected if (i % 6 + i // 6) % 2 == parity)
    held = sorted(set(detected) - set(train))
    result = dict(train_parity=parity, train_ids=train, holdout_ids=held)
    if len(train) < 6 or len({i % 6 for i in train}) < 3 or len({i // 6 for i in train}) < 3:
        return dict(result, status='SKIPPED_TRAIN_COVERAGE')
    objects = np.concatenate([object_corners(i) for i in train])
    pixels = np.concatenate([detected[i] for i in train])
    rays = normalized_camera_points(pixels, calibration)
    seeds = cv2.solvePnPGeneric(objects, rays, np.eye(3), np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for index, (rvec, tvec) in enumerate(zip(seeds[1], seeds[2])):
        # Only training rays enter refinement. Optimize normalized rays using
        # the declared SDK model, then rank seeds by TRAINING pixel residual.
        rvec, tvec = cv2.solvePnPRefineLM(objects, rays, np.eye(3), np.zeros(5), rvec.copy(), tvec.copy())
        rotation, translation = cv2.Rodrigues(rvec)[0], tvec.reshape(3)
        if not np.isfinite(translation).all() or np.min((objects @ rotation.T + translation)[:, 2]) <= 0:
            continue
        error = projected(objects, rotation, translation, calibration) - pixels
        rms = float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))))
        if np.isfinite(rms):
            candidates.append((rms, index, rotation, translation))
    if not candidates:
        return dict(result, status='SKIPPED_NO_POSITIVE_TRAIN_POSE')
    candidates.sort(key=lambda c: (c[0], c[1]))
    rms, index, rotation, translation = candidates[0]
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation
    observations, residuals = [], []
    for tag in held:
        obj = object_corners(tag)
        pred = projected(obj, rotation, translation, calibration)
        residual = detected[tag] - pred
        # Local board-plane displacement equivalent, NOT a measured print error.
        epsilon = 1e-4
        jac = np.stack([(projected(obj + np.eye(3)[axis] * epsilon, rotation, translation, calibration)
                         - projected(obj - np.eye(3)[axis] * epsilon, rotation, translation, calibration))
                        / (2 * epsilon) for axis in (0, 1)], axis=2)
        for corner in range(4):
            local = np.linalg.lstsq(jac[corner], residual[corner], rcond=None)[0]
            observations.append(dict(tag_id=tag, corner=corner, board_xy_m=obj[corner, :2].tolist(),
                                     observed_px=detected[tag][corner].tolist(), predicted_px=pred[corner].tolist(),
                                     residual_px=residual[corner].tolist(), apparent_board_plane_residual_m=local.tolist()))
        residuals.extend(residual.tolist())
    held_rms = float(np.sqrt(np.mean(np.sum(np.array(residuals)**2, axis=1)))) if residuals else None
    return dict(result, status='OK' if held else 'NO_HOLDOUT', selected_seed=index,
                train_rms_px=rms, seed_train_rms_px=[c[0] for c in candidates],
                camera_from_board=transform.tolist(), holdout_rms_px=held_rms, corners=observations)


def verified_session(path):
    setup = json.loads((path / 'setup.json').read_text())
    report = json.loads((path / 'capture_report.json').read_text())
    role = setup['selected_role']
    if report['operator_aborted'] or report['failure'] or not report['started']:
        raise ValueError(f'incomplete capture: {path}')
    if setup['factory_values_modified'] or setup['activation'] != 'NOT_ACTIVATED':
        raise ValueError('modified calibration')
    target = setup['target']
    if (target['rows'], target['columns'], target['marker_border_bits'], target['nominal_tag_m'], target['nominal_gap_m']) != (6, 6, 2, TAG_M, GAP_M):
        raise ValueError('unexpected target layout')
    source_hashes = {name: digest(path / name) for name in ('setup.json', 'capture_report.json', f'{role}/images.jsonl')}
    rows = [json.loads(line) for line in (path / role / 'images.jsonl').read_text().splitlines()]
    if not rows or len(rows) != report['counts'][role]:
        raise ValueError('image count mismatch')
    for n, row in enumerate(rows):
        if row['file'] != f'{n:06d}.png' or row['sample'] != n or row['setup_sha256'] != source_hashes['setup.json']:
            raise ValueError('index/setup binding mismatch')
        image_path = path / role / row['file']
        if digest(image_path) != row['sha256']:
            raise ValueError('image hash mismatch')
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        intr = setup['devices'][role]['intrinsics']
        if image is None or image.shape != (intr['height'], intr['width']):
            raise ValueError('image shape mismatch')
        for field in ('frame_number', 'sdk_timestamp_ms', 'host_arrival_monotonic_ns'):
            if not np.isfinite(row[field]) or (n and row[field] <= rows[n-1][field]):
                raise ValueError('timestamp/counter regression')
    return setup, rows, source_hashes


def analyze_session(path, refine5=False):
    setup, rows, source_hashes = verified_session(path)
    role = setup['selected_role']
    calibration = _camera_calibration_from_intrinsics(calibration_id=source_hashes['setup.json'], frame_id=role+'.color',
                                                      intrinsics=setup['devices'][role]['intrinsics'])
    detector, frames = make_detector(), []
    for row in rows:
        gray = cv2.imread(str(path / role / row['file']), 0)
        ds = detect_grid(detector, gray)
        frame = dict(sample=row['sample'], image_sha256=row['sha256'], preview_quality=row['preview_quality'])
        try:
            detected = valid_detections(ds)
        except ValueError as exc:
            frames.append(dict(frame, status='SKIPPED_DUPLICATE_IDS', reason=str(exc)))
            continue
        if refine5:
            detected = refine_legacy_corners(gray, detected)
        folds = [fit_fold(detected, calibration, parity) for parity in (0, 1)]
        frames.append(dict(frame, status='ANALYZED', valid_tag_ids=sorted(detected), folds=folds))
    successful = [fold for frame in frames for fold in frame.get('folds', []) if fold['status'] == 'OK']
    errors = np.array([corner['residual_px'] for fold in successful for corner in fold['corners']]).reshape(-1, 2)
    norms = np.linalg.norm(errors, axis=1)
    return dict(source=str(path), role=role, orientation=setup['board_orientation_operator_label'],
                factory_snapshot=setup['devices'][role], source_hashes=source_hashes, frames=frames,
                summary=dict(images=len(rows), preview_good=sum(r['preview_quality']['good'] for r in rows),
                             successful_folds=len(successful), heldout_corners=len(errors),
                             corner_rms_px=float(np.sqrt(np.mean(norms**2))) if len(norms) else None,
                             corner_p95_px=float(np.percentile(norms, 95)) if len(norms) else None,
                             median_frame_fold_rms_px=float(np.median([f['holdout_rms_px'] for f in successful])) if successful else None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--refine5', action='store_true', help='Legacy junction board only: explicit5px corner window, same decoded IDs')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'artifacts') or output.exists():
        raise ValueError('output must be a NEW directory under repository artifacts')
    result = dict(schema='ego.fixed_factory_board_prediction.v1', status='DIAGNOSTIC_ONLY_NOT_CALIBRATION_ACCEPTANCE',
                  factory_values_modified=False, activation='NOT_ACTIVATED',
                  corner_pipeline='legacy_explicit5px' if args.refine5 else 'original_aruco_relative_window',
                  development_frame='board_factory_ego_normal_20260906T002443/ego/000000.png; exclude from refinement confirmation aggregate',
                  convention='legacy6x6: center=(column,row)*0.04576m; canonical corners=(+,-),(-,-),(-,+),(+,+)*0.0176m',
                  pose_objective='training-only normalized SDK rays; IPPE seeds and LM; rank by training pixel RMS',
                  gates='per-tag finite and min15px; training >=6 tags spanning >=3 rows and >=3 columns; no preview-quality/all-tag pose gate',
                  limitations=['documented board dimensions, not fresh metrology', 'limited pose coverage and partial occlusion',
                               'unweighted correlated corner samples, not independent precision estimates',
                               'no unique attribution to lens/print/algorithm from residual alone'],
                  code_sha256={str(p.relative_to(ROOT)): digest(p) for p in (
                      Path(__file__).resolve(), ROOT/'scripts/color_aprilgrid_capture.py',
                      ROOT/'three_device_slam/spatial/apriltag_detector.py', ROOT/'tests/test_fixed_factory_board_diagnostic.py')}, sessions=[])
    for name in SESSIONS:
        session = analyze_session(ROOT / 'artifacts/spatial_bench' / name, args.refine5)
        result['sessions'].append(session)
        print(name, json.dumps(session['summary']), flush=True)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print('DIAGNOSTIC SAVED:', output / 'report.json', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
