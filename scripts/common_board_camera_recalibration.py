#!/usr/bin/env python3
"""Offline Ego camera calibration candidates; never overwrites factory or activates."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_pose_ablation import analyze, observed_ego_mount, validate_reference_frames
from scripts.common_board_resampling_analysis import restore_board_id1
from scripts.common_board_joint import board_geometry
from scripts.common_board_static_analysis import pose_options

MODES = {
    'K4': cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
    'K4_k1': cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
    'K4_k1k2': cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3,
}


def fit_camera(windows, factory, mode):
    if mode not in MODES:
        raise ValueError('unsupported calibration mode')
    train = [w for w in windows if w['split'] == 'train']
    expected = [i for i, (_, split) in enumerate(cb.BOARD_PLAN) if split == 'train']
    if [w['slot'] for w in train] != expected:
        raise ValueError('requires exact frozen training slots')
    if np.any(factory.distortion_coefficients):
        raise ValueError('this candidate calibration requires the recorded zero-D Ego profile')
    rms, k, d, _, _ = cv2.calibrateCamera(
        [w['objects'].astype(np.float32) for w in train],
        [w['pixels']['ego'].astype(np.float32) for w in train],
        (factory.width, factory.height), factory.camera_matrix.copy(), np.zeros(5),
        flags=MODES[mode] | cv2.CALIB_USE_INTRINSIC_GUESS,
        criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-10))
    candidate = replace(factory, camera_matrix=k, distortion_coefficients=d.ravel(),
                        distortion_model='opencv_brown_conrady')
    return candidate, dict(mode=mode, training_slots=expected, train_rms_px=float(rms),
        K=k.tolist(), D=d.ravel().tolist(), distortion_model=candidate.distortion_model,
        factory_values_modified=False, activation='NOT_ACTIVATED',
        objective='OpenCV per-camera pixel reprojection; independent pose per training window',
        flags=int(MODES[mode] | cv2.CALIB_USE_INTRINSIC_GUESS))


def mount_pose(pixels, calibration):
    candidates = [c for c in cb._solve_ippe_candidates(pixels, calibration=calibration,
                  tag_size_m=.04) if c.positive_depth]
    if not candidates:
        raise ValueError('no positive mount pose')
    return min(candidates, key=lambda c: c.reprojection_error_px).camera_from_tag


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--training-session', type=Path, required=True)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    out = cb.new_output(args.output)
    train_setup, factory, windows, _, train_hashes, _ = cb.load_session(args.training_session)
    setup, cal, _, _, hashes, _ = cb.load_session(args.session, allow_partial=True)
    if train_setup.get('resume_from') or setup.get('resume_from') or setup.get('observation_mode') != 'simultaneous':
        raise ValueError('requires non-resumed complete training and simultaneous comparison')
    if train_setup['devices'] != setup['devices'] or train_setup['target'] != setup['target']:
        raise ValueError('camera/target identity mismatch')
    reference = json.loads(args.reference.read_text())
    if reference['source_hashes'] != hashes:
        raise ValueError('source mismatch')
    validate_reference_frames(reference, [json.loads(t) for t in (args.session/'attempts.jsonl').read_text().splitlines()])
    datasets = []
    detector, tag_detector = make_board_detector(setup['board_detector']), cb.mount_detector()
    objects = np.concatenate([cb.object_corners(i) for i in range(36)])
    for f in reference['frames']:
        images = {}
        for r in cb.ROLES:
            p = args.session/f['source'][r]['file']
            if cb.digest(p) != f['source'][r]['sha256']:
                raise ValueError('raw hash mismatch')
            images[r] = cv2.imread(str(p), 0)
        display = {}
        detected, reasons = cb.detect_sample(images, 'board', detector, tag_detector, display=display, simultaneous=True)
        if reasons:
            raise ValueError(str(reasons))
        pixels = {}
        for r in cb.ROLES:
            raw, _, _ = board_geometry(display[r]['raw'])
            raw[1], _ = restore_board_id1(display[r]['raw'])
            if set(raw) != set(range(36)):
                raise ValueError('all36 required; no pruning')
            refined = cb.refine_legacy_corners(images[r], raw)
            pixels[r] = np.concatenate([refined[i] for i in range(36)])
        mount_pixels = detected['_mount']['ego'][1]
        observed_ego_mount(mount_pixels, cal['ego'], f['static']['whole_board'])
        datasets.append(dict(slot=f['slot'], pair=f['pair'], pixels=pixels, mount_pixels=mount_pixels))
    cases = []
    for mode in ('factory', *MODES):
        candidate, meta = (factory['ego'], dict(mode='factory')) if mode == 'factory' else fit_camera(windows, factory['ego'], mode)
        candidate_cal = dict(cal, ego=candidate)
        print('CASE', mode, 'K', candidate.camera_matrix.tolist(), 'D', candidate.distortion_coefficients.tolist(), flush=True)
        joint = cb.solve_board(windows, candidate_cal)
        frames = []
        for w in datasets:
            em = mount_pose(w['mount_pixels'], candidate)
            result = analyze(w['pixels'], objects, candidate_cal, em, lambda o,p,c: pose_options(o,p,c)[0]['pose'])
            frames.append(dict(slot=w['slot'], pair=w['pair'], result=result, T_Ecolor_mount=em.tolist()))
        contrasts = {name: dict(min_mm=float(min(f['result']['contrasts'][name]['translation_mm'] for f in frames)),
             median_mm=float(np.median([f['result']['contrasts'][name]['translation_mm'] for f in frames])),
             max_mm=float(max(f['result']['contrasts'][name]['translation_mm'] for f in frames)))
             for name in ('horizontal', 'vertical', 'checker')}
        case = dict(candidate=meta, old_joint=joint, new_frames=frames, contrasts=contrasts,
            old_single_camera_holdout=[dict(slot=w['slot'], **pose_options(w['objects'],w['pixels']['ego'],candidate)[0]['score'])
                                      for w in windows if w['split']=='holdout'])
        cases.append(case)
        print(mode, 'old worst joint holdout', max(h['p95_px'] for h in joint['holdout']), 'new', contrasts, flush=True)
    out.mkdir()
    cb.write_json(out/'report.json', dict(status='CAMERA_RECALIBRATION_DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        training_session=str(args.training_session.resolve()), training_hashes=train_hashes,
        source_session=str(args.session.resolve()), source_hashes=hashes, reference_sha256=cb.digest(args.reference),
        devices=setup['devices'], target=setup['target'], cases=cases,
        limitations=['no factory files/runtime/SLAM configuration changed',
            'only old8 frozen train poses fit intrinsic parameters; all holdouts excluded',
            'all inspected datasets are development; model comparison is not independent acceptance',
            'new mount pose recomputed with each candidate camera, not reused factory mount',
            'no distance-based branch choice or frame/tag pruning; two IPPE seeds refined and best own-image pixel score selected',
            'camera/board/corner systematic bias not uniquely physically identified'],
        code_sha256={str(p.relative_to(cb.ROOT)):cb.digest(p) for p in (
            Path(__file__).resolve(), cb.ROOT/'scripts/common_board_calibration.py',
            cb.ROOT/'scripts/common_board_pose_ablation.py', cb.ROOT/'scripts/common_board_static_analysis.py',
            cb.ROOT/'scripts/common_board_resampling_analysis.py', cb.ROOT/'scripts/common_board_detector.py',
            cb.ROOT/'scripts/common_board_joint.py', cb.ROOT/'scripts/fixed_factory_board_diagnostic.py',
            cb.ROOT/'three_device_slam/spatial/apriltag_detector.py')}))


if __name__ == '__main__':
    main()
