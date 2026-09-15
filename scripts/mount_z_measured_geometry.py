#!/usr/bin/env python3
"""Joint-chain mount z with MEASURED print geometry.

The IR triangulation probe (ego_stereo_color_probe_20260906_run6,
joint_stereo_diagnostic_v3, factory stereo IR) measures the physical
board at tag 35.56mm, pitch x 46.18mm / y 46.26mm, and the mount tag
at 40.39mm -- every print ~1% larger than the nominal 35.2 / 45.76 /
40.0mm the chain has been assuming.  This driver replays the joint
12-frame chain (scripts/mount_z_decision.py) with measured object
geometry and mount size.

Self-consistency note: the UMI factory color calibration was fitted by
its vendor pipeline against this same physical board, so UMI factory
intrinsics + measured geometry are the matched pair; swapping in
"corrected" UMI intrinsics without re-fitting would double-count.
The Ego candidates are IR-anchored (true metric), so they pair with
measured geometry directly.
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
from scripts.mount_z_decision import candidate_models, joint_frames
from three_device_slam.spatial.mount_calibration import load_color_to_ir_snapshot

MEASURED = dict(tag_m=0.03556, pitch_x_m=0.04618, pitch_y_m=0.04626, mount_m=0.040386)
NOMINAL = dict(tag_m=0.0352, pitch_x_m=0.04576, pitch_y_m=0.04576, mount_m=0.040)


def measured_object_corners(tag_id, geom):
    if not 0 <= tag_id < 36:
        raise ValueError('unsupported board tag')
    center = np.array([tag_id % 6, tag_id // 6, 0.]) * [geom['pitch_x_m'], geom['pitch_y_m'], 1.0]
    return center + .5 * geom['tag_m'] * np.array([[1., -1, 0], [-1, -1, 0], [-1, 1, 0], [1, 1, 0]])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--umi-calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)

    out = cb.new_output(args.output)
    setup, cal, _, _, hashes, _ = cb.load_session(args.session, allow_partial=True)
    t_ir_ucolor, extrinsic_meta = load_color_to_ir_snapshot(args.umi_calibration)
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

    models = candidate_models(cal['ego'])
    cases = {}
    combos = (
        ('factory_nominal', 'factory', NOMINAL),
        ('k1k2k3_nominal', 'k1k2k3_ir', NOMINAL),
        ('k1k2k3_measured', 'k1k2k3_ir', MEASURED),
        ('k1k2_measured', 'k1k2_ir', MEASURED),
        # attribution probes: board-only vs mount-only geometry correction
        ('k1k2k3_board_only', 'k1k2k3_ir', dict(NOMINAL, mount_m=MEASURED['mount_m'])),
        ('k1k2k3_mount_only', 'k1k2k3_ir', dict(MEASURED, mount_m=NOMINAL['mount_m'])),
    )
    for case_name, model_name, geom in combos:
        candidate, meta = models[model_name]
        candidate_cal = dict(cal, ego=candidate)
        objects = np.concatenate([measured_object_corners(i, geom) for i in range(36)])
        rows = []
        for entry in pixels_per_frame:
            cands = sorted((c for c in cb._solve_ippe_candidates(
                entry['mount_pixels'], calibration=candidate, tag_size_m=geom['mount_m'])
                if c.positive_depth), key=lambda c: c.reprojection_error_px)
            if not cands:
                raise ValueError('no positive mount pose')
            em = cands[0].camera_from_tag
            result = analyze(entry['pixels'], objects, candidate_cal, em, solver)
            group = result['groups']['all36']
            m_ucolor = np.asarray(group['mount']['T_Ucolor_mount'])
            m_irleft = t_ir_ucolor @ m_ucolor
            rows.append(dict(
                z_ucolor_mm=float(m_ucolor[2, 3] * 1000),
                norm_ucolor_mm=float(np.linalg.norm(m_ucolor[:3, 3]) * 1000),
                z_irleft_mm=float(m_irleft[2, 3] * 1000),
                norm_irleft_mm=float(np.linalg.norm(m_irleft[:3, 3]) * 1000),
                board_p95_px=float(group['fit']['ego']['p95_px']),
            ))
        zc = [r['z_ucolor_mm'] for r in rows]
        zi = [r['z_irleft_mm'] for r in rows]
        nc = [r['norm_ucolor_mm'] for r in rows]
        ni = [r['norm_irleft_mm'] for r in rows]
        cases[case_name] = dict(model=model_name, geometry=geom, frames=rows,
                                summary=dict(
            z_ucolor_median_mm=float(np.median(zc)),
            norm_ucolor_median_mm=float(np.median(nc)),
            z_irleft_median_mm=float(np.median(zi)),
            norm_irleft_median_mm=float(np.median(ni)),
            z_spread_mm=float(max(zc) - min(zc)),
            board_p95_median_px=float(np.median([r['board_p95_px'] for r in rows]))))
        s = cases[case_name]['summary']
        print(f"{case_name:20s} Ucolor z={s['z_ucolor_median_mm']:7.2f} norm={s['norm_ucolor_median_mm']:6.2f} | "
              f"IRleft z={s['z_irleft_median_mm']:7.2f} norm={s['norm_irleft_median_mm']:6.2f} | "
              f"board p95 {s['board_p95_median_px']:.2f}px", flush=True)

    out.mkdir()
    cb.write_json(out / 'report.json', dict(
        status='MOUNT_Z_MEASURED_GEOMETRY_DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        measured_geometry_m=MEASURED, nominal_geometry_m=NOMINAL,
        measurement_source='ego_stereo_color_probe_20260906_run6/joint_stereo_diagnostic_v3 '
                           'IR triangulation (n=600 pitches, n=80 mount sides)',
        session=args.session.name, session_hashes=hashes,
        umi_color_to_ir_left_snapshot=extrinsic_meta,
        cases=cases,
        limitations=[
            'measured geometry comes from a different capture than the joint session (same physical board)',
            'UMI factory intrinsics + measured geometry treated as the matched pair (vendor fit used this board)',
            'single static rig; spread is detection noise not scene diversity',
            'development evidence only; nothing activated'],
        code_sha256={str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
            Path(__file__).resolve(),
            cb.ROOT / 'scripts/mount_z_decision.py')}))
    print(f"report={out / 'report.json'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
