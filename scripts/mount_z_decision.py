#!/usr/bin/env python3
"""End-to-end mount-tag z with candidate Ego RGB intrinsics.

Recomputes the full mount calibration chain on the joint 12-frame
board+mount dataset (common_board_joint_20260906_run1, both attempts)
for several Ego RGB calibration candidates:

* factory   -- recorded zero-distortion profile (the status quo)
* k1k2_ir   -- full-data IR-anchored refit, k1/k2 free   (LOSO runner-up)
* k1k2k3_ir -- full-data IR-anchored refit, k1/k2/k3 free (LOSO decision)

The IR-anchored candidates come from scripts/ego_rgb_model_selection.py
(fit_variant over ego_stereo_color_probe runs 1-6); they never touch the
factory profile and are NOT activated.

Per frame: all36 board pose per camera -> T_Ecolor_Ucolor; mount pose
from the ego mount quad with the candidate; T_Ucolor_mount = inv(x) @ em;
then T_Uirleft_mount via the UMI factory color->ir_left snapshot.  The
report tabulates z axial / norm / xyz in BOTH the UMI color and UMI
ir_left frames so numbers stay comparable with the caliper measurement
(color optical center datum) and the historical IR-frame consensus.

Nothing is written back to any session; factory calibration untouched.
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
from scripts.ego_rgb_model_selection import fit_variant
from scripts.ego_multidepth_camera_diagnostic import load_frames as load_probe_frames
from three_device_slam.spatial.mount_calibration import load_color_to_ir_snapshot


MOUNT_TAG_SIZE_M = 0.04

REFERENCE_BANDS_MM = {
    # Physical caliper chain: glass->tag 52.1mm minus 3.7mm optical-center
    # datum (glass is in FRONT of the OC, so only subtraction is valid).
    'caliper_color_oc_to_tag': 48.4,
    # Codex new candidate in the umi ir_left frame.
    'codex_candidate_irleft_z': -46.54,
    # Historical consensus (now dead) in the umi ir_left frame.
    'old_consensus_irleft_z': -61.3,
}


def candidate_models(factory):
    """factory + the two IR-anchored full-data refits."""
    root = cb.ROOT / 'artifacts/spatial_bench'
    datasets = {}
    signature = None
    for number in range(1, 7):
        _, _, frames, current_signature, _ = load_probe_frames(root, number)
        if signature is not None and current_signature != signature:
            raise ValueError('cross-session stereo geometry mismatch')
        signature = current_signature
        datasets[number] = [f for f in frames if f.get('board_points') is not None]
    train = [f for n in sorted(datasets) for f in datasets[n]]
    out = {'factory': (factory, dict(source='recorded zero-D profile'))}
    for name in ('k1k2', 'k1k2k3'):
        model, meta = fit_variant(train, factory, name)
        out[f'{name}_ir'] = (model, dict(source='IR-anchored LOSO selection full-data refit', **meta))
    return out


def joint_frames(session):
    """Rebuild the 12 accepted (attempt, pair) frames from attempts.jsonl."""
    rows = [json.loads(line) for line in (session / 'attempts.jsonl').read_text().splitlines()]
    frames = []
    for row in rows:
        if not row.get('accepted'):
            continue
        for pair_index, pair in enumerate(row['pairs']):
            frames.append(dict(attempt=row['attempt'], slot=row['slot'], pair=pair_index, source=pair))
    return frames


def mount_candidates(mount_pixels, calibration):
    cands = sorted((c for c in cb._solve_ippe_candidates(
        mount_pixels, calibration=calibration, tag_size_m=MOUNT_TAG_SIZE_M)
        if c.positive_depth), key=lambda c: c.reprojection_error_px)
    if not cands:
        raise ValueError('no positive mount pose')
    return cands


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True,
                        help='joint board+mount session dir')
    parser.add_argument('--umi-calibration', type=Path, required=True,
                        help='any capture session left/calibration.json carrying the factory color_to_ir_left snapshot')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)

    out = cb.new_output(args.output)
    setup, cal, _, _, hashes, _ = cb.load_session(args.session, allow_partial=True)
    if setup.get('resume_from') or setup.get('observation_mode') != 'simultaneous':
        raise ValueError('requires non-resumed simultaneous session')

    t_ir_ucolor, extrinsic_meta = load_color_to_ir_snapshot(args.umi_calibration)
    objects = np.concatenate([cb.object_corners(i) for i in range(36)])
    detector = make_board_detector(setup['board_detector'])
    mount_detector = cb.mount_detector()
    solver = lambda o, p, c: pose_options(o, p, c)[0]['pose']

    frames = joint_frames(args.session)
    pixels_per_frame = []
    for f in frames:
        images = {}
        for role in cb.ROLES:
            path = args.session / f['source'][role]['file']
            if cb.digest(path) != f['source'][role]['sha256']:
                raise ValueError('raw hash mismatch')
            images[role] = cv2.imread(str(path), 0)
        display = {}
        detected, reasons = cb.detect_sample(images, 'board', detector, mount_detector,
                                             display=display, simultaneous=True)
        if reasons:
            raise ValueError(f"attempt {f['attempt']} pair {f['pair']}: {reasons}")
        pixels = {}
        for role in cb.ROLES:
            raw, _, _ = board_geometry(display[role]['raw'])
            raw[1], _ = restore_board_id1(display[role]['raw'])
            if set(raw) != set(range(36)):
                raise ValueError('all36 required; no pruning')
            refined = cb.refine_legacy_corners(images[role], raw)
            pixels[role] = np.concatenate([refined[i] for i in range(36)])
        pixels_per_frame.append(dict(frame=f, pixels=pixels,
                                     mount_pixels=detected['_mount']['ego'][1]))

    cases = {}
    for name, (candidate, meta) in candidate_models(cal['ego']).items():
        candidate_cal = dict(cal, ego=candidate)
        per_frame = []
        for entry in pixels_per_frame:
            cands = mount_candidates(entry['mount_pixels'], candidate)
            em = cands[0].camera_from_tag
            result = analyze(entry['pixels'], objects, candidate_cal, em, solver)
            group = result['groups']['all36']
            m_ucolor = np.asarray(group['mount']['T_Ucolor_mount'])
            m_irleft = t_ir_ucolor @ m_ucolor
            ego_fit = group['fit']['ego']
            per_frame.append(dict(
                attempt=entry['frame']['attempt'], slot=entry['frame']['slot'],
                pair=entry['frame']['pair'],
                z_ucolor_mm=float(m_ucolor[2, 3] * 1000),
                norm_ucolor_mm=float(np.linalg.norm(m_ucolor[:3, 3]) * 1000),
                xyz_ucolor_mm=(m_ucolor[:3, 3] * 1000).round(3).tolist(),
                z_irleft_mm=float(m_irleft[2, 3] * 1000),
                norm_irleft_mm=float(np.linalg.norm(m_irleft[:3, 3]) * 1000),
                xyz_irleft_mm=(m_irleft[:3, 3] * 1000).round(3).tolist(),
                board_p95_px=float(ego_fit['p95_px']),
                mount_reproj_px=float(cands[0].reprojection_error_px),
                mount_ippe_candidates=len(cands),
                mount_second_mode_gap_px=(
                    float(cands[1].reprojection_error_px - cands[0].reprojection_error_px)
                    if len(cands) > 1 else None),
            ))
        zc = [r['z_ucolor_mm'] for r in per_frame]
        zi = [r['z_irleft_mm'] for r in per_frame]
        nc = [r['norm_ucolor_mm'] for r in per_frame]
        ni = [r['norm_irleft_mm'] for r in per_frame]
        xc = [r['xyz_ucolor_mm'][0] for r in per_frame]
        cases[name] = dict(
            meta=dict(meta, K=np.round(candidate.camera_matrix, 4).tolist(),
                      D=np.round(candidate.distortion_coefficients, 6).tolist(),
                      distortion_model=candidate.distortion_model,
                      factory_values_modified=False, activation='NOT_ACTIVATED'),
            frames=per_frame,
            summary=dict(
                x_ucolor_median_mm=float(np.median(xc)),
                z_ucolor_median_mm=float(np.median(zc)),
                z_ucolor_spread_mm=float(max(zc) - min(zc)),
                norm_ucolor_median_mm=float(np.median(nc)),
                z_irleft_median_mm=float(np.median(zi)),
                z_irleft_spread_mm=float(max(zi) - min(zi)),
                norm_irleft_median_mm=float(np.median(ni)),
                board_p95_median_px=float(np.median([r['board_p95_px'] for r in per_frame])),
                mount_reproj_median_px=float(np.median([r['mount_reproj_px'] for r in per_frame])),
            ),
        )
        s = cases[name]['summary']
        print(f"{name:10s} Ucolor z={s['z_ucolor_median_mm']:7.2f} (spread {s['z_ucolor_spread_mm']:5.2f}) "
              f"norm={s['norm_ucolor_median_mm']:6.2f} | IRleft z={s['z_irleft_median_mm']:7.2f} "
              f"norm={s['norm_irleft_median_mm']:6.2f} | board p95 {s['board_p95_median_px']:.2f}px "
              f"mount {s['mount_reproj_median_px']:.2f}px", flush=True)

    out.mkdir()
    cb.write_json(out / 'report.json', dict(
        status='MOUNT_Z_RECOMPUTATION_DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        objective='per-frame all36 board pose + ego mount pose, chained to umi color and ir_left',
        session=args.session.name, session_hashes=hashes,
        frames=len(frames), reference_bands_mm=REFERENCE_BANDS_MM,
        umi_color_to_ir_left_snapshot=extrinsic_meta,
        candidates=cases,
        limitations=[
            'single static rig: 12 frames share one geometry, spread is detection noise not scene diversity',
            'UMI color coefficients still misused as forward Brown (deferred, small residual)',
            'board print scale enters the board pose; IR-anchored candidates remove Ego print dependence but left-camera print dependence remains',
            'development evidence only; nothing activated'],
        code_sha256={str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
            Path(__file__).resolve(),
            cb.ROOT / 'scripts/common_board_calibration.py',
            cb.ROOT / 'scripts/common_board_pose_ablation.py',
            cb.ROOT / 'scripts/ego_rgb_model_selection.py')}))
    print(f"report={out / 'report.json'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
