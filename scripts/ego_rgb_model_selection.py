#!/usr/bin/env python3
"""Ego RGB model selection via leave-scene-out CV on IR-anchored 3D points.

Decides which distortion/intrinsic model best explains the Ego RGB
observations, without touching factory values or activating anything.

Anchor: stereo-IR-triangulated 3D points (factory IR intrinsics are the
precise calibration on this D435i; the RGB block reports all-zero
distortion, which this test quantifies).  Board print dimensions never
enter the fit, so print scale cannot bias K.

Method: for every leave-one-scene-out fold and every model variant, fit
K/D with cv2.calibrateCamera on the training scenes' per-frame 3D->2D
correspondences (per-view poses estimated jointly), then score the held
-out scene by reprojection p95/max of the IR-3D points through a
per-frame SQPNP pose.  Mount-tag corners are excluded from fitting
(Codex policy) but scored wherever available (run6) as the end-use
metric.  Decision rule: lowest worst-fold mount p95, board p95 as the
secondary criterion; the factory zero-D profile must lose for the
zero-distortion hypothesis to be rejected.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.ego_multidepth_camera_diagnostic import load_frames


VARIANTS = {
    # name: (flags, description)
    'K_only': (
        cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST
        | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
        'refit fx/fy/cx/cy only, distortion stays zero'),
    'k1': (
        cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST
        | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
        'zero tangential, k1 free (Codex multidepth mode)'),
    'k1k2': (
        cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST
        | cv2.CALIB_FIX_K3,
        'zero tangential, k1/k2 free'),
    'k1k2k3': (
        cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST,
        'zero tangential, k1/k2/k3 free'),
    'k1_p1p2': (
        cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
        'k1 + tangential p1/p2 free'),
    'full': (
        cv2.CALIB_USE_INTRINSIC_GUESS,
        'k1/k2/k3 + tangential free'),
}

CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-10)


def fit_variant(train_frames, factory, name):
    if name == 'factory':
        return factory, {}
    flags, _ = VARIANTS[name]
    points = [np.asarray(f['board_points'], dtype=np.float32) for f in train_frames]
    pixels = [np.asarray(f['board_pixels'], dtype=np.float32) for f in train_frames]
    rms, k, d, _, _ = cv2.calibrateCamera(
        points, pixels, (factory.width, factory.height),
        factory.camera_matrix.copy(), np.zeros(5), flags=flags, criteria=CRITERIA)
    from dataclasses import replace
    candidate = replace(factory, camera_matrix=k, distortion_coefficients=d.ravel(),
                        distortion_model='opencv_brown_conrady')
    return candidate, dict(train_rms_px=float(rms))


def frame_pose(points, pixels, calibration):
    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(points, dtype=np.float64), np.asarray(pixels, dtype=np.float64),
        calibration.camera_matrix, calibration.distortion_coefficients,
        flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise ValueError('solvePnP failed')
    return cb.unpack(np.r_[rvec.ravel(), tvec.ravel()])


def score_frames(frames, calibration):
    """Board pose per frame; mount scored through the SAME rigid pose.

    Per-mount-tag PnP is meaningless here: the archived 4-corner pairing
    of the tiny IR mount quad is not internally consistent (any per-tag
    pose lands at ~16 px).  The end use is exactly the chain structure --
    a pose from the board, evaluated at the mount's image location -- so
    that is what gets scored.  With a static scene (run6) the board pose
    also transfers across frames.
    """
    out = {}
    board_frames = [f for f in frames if f.get('board_points') is not None]
    board_errs = []
    mount_errs = []
    for f in board_frames:
        pose = frame_pose(f['board_points'], f['board_pixels'], calibration)
        predicted = cb.project(np.asarray(f['board_points']), pose, calibration)
        board_errs.append(cb.errors(predicted, np.asarray(f['board_pixels'])))
        if f.get('mount_points') is not None:
            pm = cb.project(np.asarray(f['mount_points']), pose, calibration)
            r = np.linalg.norm(pm - np.asarray(f['mount_pixels']), axis=1)
            mount_errs.append(dict(p95_px=float(np.percentile(r, 95)),
                                   max_px=float(r.max())))
    if board_errs:
        out['board'] = dict(
            frames=len(board_errs),
            p95_px=float(np.median([e['p95_px'] for e in board_errs])),
            max_px=float(np.median([e['max_px'] for e in board_errs])),
            worst_p95_px=float(max(e['p95_px'] for e in board_errs)),
        )
    if mount_errs:
        out['mount'] = dict(
            frames=len(mount_errs),
            p95_px=float(np.median([e['p95_px'] for e in mount_errs])),
            max_px=float(np.median([e['max_px'] for e in mount_errs])),
            worst_p95_px=float(max(e['p95_px'] for e in mount_errs)),
        )
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    out = cb.new_output(args.output)

    root = cb.ROOT / 'artifacts/spatial_bench'
    datasets = {}
    factory = None
    hashes = {}
    signature = None
    for number in range(1, 7):
        camera, pose, frames, current_signature, hashes[number] = load_frames(root, number)
        if signature is not None and current_signature != signature:
            raise ValueError('cross-session stereo geometry mismatch')
        signature = current_signature
        factory = camera if factory is None else factory
        datasets[number] = [f for f in frames if f.get('board_points') is not None]

    scenes = sorted(datasets)
    folds = []
    for held in scenes:
        train = [f for n in scenes if n != held for f in datasets[n]]
        test = datasets[held]
        row = dict(held_out=held, train_scenes=[n for n in scenes if n != held],
                   train_frames=len(train), test_frames=len(test))
        for name in ('factory', *VARIANTS):
            candidate, fit_meta = fit_variant(train, factory, name)
            scores = score_frames(test, candidate)
            row[name] = dict(scores=scores,
                             K=np.round(candidate.camera_matrix, 4).tolist(),
                             D=np.round(candidate.distortion_coefficients, 6).tolist(),
                             **fit_meta)
            print('fold', held, name, json.dumps(scores), flush=True)
        folds.append(row)

    summary = {}
    for name in ('factory', *VARIANTS):
        board_worst = max(
            folds[i][name]['scores']['board']['worst_p95_px'] for i in range(len(scenes)))
        board_med = float(np.median(
            [folds[i][name]['scores']['board']['p95_px'] for i in range(len(scenes))]))
        mount_runs = [folds[i][name]['scores'].get('mount') for i in range(len(scenes))]
        mount_runs = [m for m in mount_runs if m]
        mount_worst = max(m['worst_p95_px'] for m in mount_runs) if mount_runs else None
        mount_med = float(np.median([m['p95_px'] for m in mount_runs])) if mount_runs else None
        summary[name] = dict(board_worst_p95_px=board_worst, board_median_p95_px=board_med,
                             mount_worst_p95_px=mount_worst, mount_median_p95_px=mount_med)
    ranked = sorted(summary, key=lambda n: (
        summary[n]['mount_worst_p95_px'] is None, summary[n]['mount_worst_p95_px'] or 0,
        summary[n]['board_worst_p95_px']))
    decision = ranked[0]
    print('DECISION', decision, json.dumps(summary[decision]), flush=True)

    final_train = [f for n in scenes for f in datasets[n]]
    final_model, final_fit = fit_variant(final_train, factory, decision)
    print('FINAL', json.dumps(final_model.camera_matrix.tolist()),
          json.dumps(final_model.distortion_coefficients.tolist()), flush=True)

    out.mkdir()
    cb.write_json(out / 'report.json', dict(
        status='MODEL_SELECTION_DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        objective='leave-scene-out CV on IR-triangulated 3D points; factory IR geometry anchor',
        anchor='stereo IR factory intrinsics (precise); RGB zero-D block quantified',
        variants={k: v[1] for k, v in VARIANTS.items()},
        scenes=scenes, folds=folds, summary=summary,
        decision=decision,
        final_model=dict(
            mode=decision, K=final_model.camera_matrix.tolist(),
            D=final_model.distortion_coefficients.tolist(),
            model='opencv_brown_conrady', **final_fit,
            factory_values_modified=False),
        limitations=[
            'run6 is a static scene: mount scores there measure self-consistency, not generalization',
            'IR triangulation noise is not modeled (errors-in-variables ignored)',
            'per-frame poses re-solved with SQPNP on the evaluated model',
            'decision is development evidence, not product acceptance; nothing activated'],
        source_hashes=hashes,
        code_sha256={str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
            Path(__file__).resolve(),
            cb.ROOT / 'scripts/ego_multidepth_camera_diagnostic.py',
            cb.ROOT / 'scripts/common_board_calibration.py',
            cb.ROOT / 'scripts/ego_stereo_color_validate.py',
            cb.ROOT / 'three_device_slam/spatial/apriltag_detector.py')}))


if __name__ == '__main__':
    main()
