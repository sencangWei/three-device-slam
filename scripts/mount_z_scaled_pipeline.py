#!/usr/bin/env python3
"""Redo the Ego RGB model selection + joint-chain z after correcting the
ego-IR stereo scale error, then rerun the joint 12-frame chain.

2026-09-07 caliper verdict: the joint/probe board is the exact product
print (black edge 35.2-35.3mm, nominal).  The ego-IR triangulation that
anchored scripts/ego_rgb_model_selection.py is uniformly inflated by
s = 1.01014 +/- 0.0001 (20-frame lattice similarity fit; board, pitch and
mount measurements all agree).  The inflated 3D points biased every fitted
fx by ~1/s (k1k2k3's 892.62 corresponds to ~901.5 true metric).

Pipeline:
1. rescale all probe IR points by 1/s and redo the LOSO selection
2. refit k1k2 / k1k2k3 on the corrected points
3. rerun the joint 12-frame chain with NOMINAL board geometry (the board
   is exact) and the corrected candidates; UMI stays factory (Intel, and
   self-consistent with the exact board).

Nothing is activated; factory values untouched.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_pose_ablation import analyze
from scripts.common_board_joint import board_geometry
from scripts.common_board_resampling_analysis import restore_board_id1
from scripts.common_board_static_analysis import pose_options
from scripts.ego_rgb_model_selection import fit_variant, score_frames
from scripts.ego_multidepth_camera_diagnostic import load_frames as load_probe_frames
from scripts.mount_z_decision import joint_frames, REFERENCE_BANDS_MM
from three_device_slam.spatial.mount_calibration import load_color_to_ir_snapshot

IR_SCALE_MEASURED_OVER_NOMINAL = 1.010142  # median, 20 frames, std 0.0001
CORRECTION = 1.0 / IR_SCALE_MEASURED_OVER_NOMINAL


def scaled_probe_datasets(root):
    datasets, factory, hashes = {}, None, {}
    signature = None
    for number in range(1, 7):
        camera, pose, frames, current_signature, hashes[number] = load_probe_frames(root, number)
        if signature is not None and current_signature != signature:
            raise ValueError('cross-session stereo geometry mismatch')
        signature = current_signature
        factory = camera if factory is None else factory
        scaled = []
        for f in frames:
            g = dict(f)
            for key in ('board_points', 'mount_points'):
                if g.get(key) is not None:
                    g[key] = (np.asarray(g[key], dtype=float) * CORRECTION).tolist()
            scaled.append(g)
        datasets[number] = [f for f in scaled if f.get('board_points') is not None]
    return datasets, factory, hashes


def model_selection(root, out):
    datasets, factory, hashes = scaled_probe_datasets(root)
    scenes = sorted(datasets)
    names = ('factory', 'k1', 'k1k2', 'k1k2k3')
    folds = []
    for held in scenes:
        train = [f for n in scenes if n != held for f in datasets[n]]
        test = datasets[held]
        row = dict(held_out=held)
        for name in names:
            candidate, fit_meta = fit_variant(train, factory, name)
            row[name] = dict(scores=score_frames(test, candidate),
                             K=np.round(candidate.camera_matrix, 4).tolist(),
                             D=np.round(candidate.distortion_coefficients, 6).tolist(),
                             **fit_meta)
        folds.append(row)
        print('fold', held, {n: folds[-1][n]['scores'].get('mount', folds[-1][n]['scores']['board']) for n in names}, flush=True)
    final_train = [f for n in scenes for f in datasets[n]]
    final = {}
    for name in names:
        model, meta = fit_variant(final_train, factory, name)
        final[name] = (model, meta)
        print(f'FINAL {name} fx={model.camera_matrix[0,0]:.2f} D={np.round(model.distortion_coefficients,6).tolist()}', flush=True)
    out.mkdir(parents=True, exist_ok=True)
    cb.write_json(out / 'report.json', dict(
        status='MODEL_SELECTION_IR_SCALE_CORRECTED_DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        ir_scale_correction=CORRECTION,
        ir_scale_evidence='board is exact product print (2026-09-07 caliper 35.2-35.3mm); '
                          'ego-IR triangulation inflated s=1.01014+/-0.0001 (lattice similarity, 20 frames)',
        folds=folds, factory_values_modified=False))
    return final


def joint_chain(session, umi_calibration, models, out):
    setup, cal, _, _, hashes, _ = cb.load_session(session, allow_partial=True)
    t_ir_ucolor, _ = load_color_to_ir_snapshot(umi_calibration)
    detector = make_board_detector(setup['board_detector'])
    mount_detector = cb.mount_detector()
    solver = lambda o, p, c: pose_options(o, p, c)[0]['pose']
    objects = np.concatenate([cb.object_corners(i) for i in range(36)])

    pixels_per_frame = []
    for f in joint_frames(session):
        images = {}
        for role in cb.ROLES:
            path = session / f['source'][role]['file']
            if cb.digest(path) != f['source'][role]['sha256']:
                raise ValueError('raw hash mismatch')
            images[role] = cv2.imread(str(path), 0)
        display = {}
        detected, reasons = cb.detect_sample(images, 'board', detector, mount_detector,
                                             display=display, simultaneous=True)
        if reasons:
            raise ValueError(str(reasons))
        pixels = {}
        for role in cb.ROLES:
            raw, _, _ = board_geometry(display[role]['raw'])
            raw[1], _ = restore_board_id1(display[role]['raw'])
            if set(raw) != set(range(36)):
                raise ValueError('all36 required')
            refined = cb.refine_legacy_corners(images[role], raw)
            pixels[role] = np.concatenate([refined[i] for i in range(36)])
        pixels_per_frame.append(dict(pixels=pixels, mount_pixels=detected['_mount']['ego'][1]))

    cases = {}
    for name, (candidate, meta) in models.items():
        candidate_cal = dict(cal, ego=candidate)
        rows = []
        for entry in pixels_per_frame:
            cands = sorted((c for c in cb._solve_ippe_candidates(
                entry['mount_pixels'], calibration=candidate, tag_size_m=0.04) if c.positive_depth),
                key=lambda c: c.reprojection_error_px)
            em = cands[0].camera_from_tag
            group = analyze(entry['pixels'], objects, candidate_cal, em, solver)['groups']['all36']
            m_u = np.asarray(group['mount']['T_Ucolor_mount'])
            m_i = t_ir_ucolor @ m_u
            rows.append(dict(z_ucolor_mm=float(m_u[2, 3] * 1000),
                             norm_ucolor_mm=float(np.linalg.norm(m_u[:3, 3]) * 1000),
                             z_irleft_mm=float(m_i[2, 3] * 1000),
                             norm_irleft_mm=float(np.linalg.norm(m_i[:3, 3]) * 1000)))
        zc = [r['z_ucolor_mm'] for r in rows]
        nc = [r['norm_ucolor_mm'] for r in rows]
        cases[name] = dict(
            K=np.round(candidate.camera_matrix, 4).tolist(),
            D=np.round(candidate.distortion_coefficients, 6).tolist(), frames=rows,
            summary=dict(z_ucolor_median_mm=float(np.median(zc)),
                         norm_ucolor_median_mm=float(np.median(nc)),
                         z_irleft_median_mm=float(np.median([r['z_irleft_mm'] for r in rows])),
                         norm_irleft_median_mm=float(np.median([r['norm_irleft_mm'] for r in rows])),
                         z_spread_mm=float(max(zc) - min(zc))))
        s = cases[name]['summary']
        print(f"{name:12s} Ucolor z={s['z_ucolor_median_mm']:7.2f} norm={s['norm_ucolor_median_mm']:6.2f} | "
              f"IRleft z={s['z_irleft_median_mm']:7.2f} norm={s['norm_irleft_median_mm']:6.2f}", flush=True)
    out.mkdir(parents=True, exist_ok=True)
    cb.write_json(out / 'report.json', dict(
        status='MOUNT_Z_IR_SCALE_CORRECTED_DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        geometry='nominal (board verified exact by 2026-09-07 caliper)',
        ir_scale_correction=CORRECTION, reference_bands_mm=REFERENCE_BANDS_MM,
        cases=cases, session=session.name, session_hashes=hashes))
    return cases


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--umi-calibration', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, default=cb.ROOT / 'artifacts/spatial_bench')
    args = parser.parse_args(argv)

    final = model_selection(args.output_root, args.output_root / 'ego_rgb_model_selection_scaled_run1')
    models = {'factory': final['factory'], 'k1k2_scaled': final['k1k2'], 'k1k2k3_scaled': final['k1k2k3']}
    joint_chain(args.session, args.umi_calibration, models,
                args.output_root / 'mount_z_scaled_run1')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
