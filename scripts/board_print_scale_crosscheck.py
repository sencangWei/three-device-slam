#!/usr/bin/env python3
"""Independent print-scale check: board geometry from the OTHER device's IR stereo.

Triangulates the shared 6x6 board in the D405 S/N 260322273737 stereo IR
pair (camera_bench_results 20260821_115114 d405_stereo_candidate solved
camchain, radtan) and measures physical tag size / pitch.  Ego IR
(ego_stereo_color_probe run6) already measured tag 35.56mm, pitch x 46.18
/ y 46.26 (+1% vs nominal 35.2 / 45.76).  If this unit's IR stereo agrees,
the +1% print scale is a property of the physical board, not of any one
device's intrinsics, and the measured-geometry joint-chain run stands.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

BENCH = Path('/home/robot/ego_vio_calib_kit/camera_bench_results/20260821_115114_645650_d405_stereo_candidate')
RECS = BENCH / 'training/recordings/calib_20260821_115119/left_hand'
CAMCHAIN = BENCH / 'solve/stereo-camchain.yaml'
JOINT_SETUP = Path('/home/robot/three-device-slam/artifacts/spatial_bench/common_board_joint_20260906_run1/setup.json')
FRAMES = list(range(200, 3200, 300))  # sparse sample across the capture

sys.path.insert(0, '/home/robot/three-device-slam')
from scripts.common_board_detector import make_board_detector  # noqa: E402
from scripts.fixed_factory_board_diagnostic import refine_legacy_corners  # noqa: E402


def load_camchain():
    cc = yaml.safe_load(CAMCHAIN.read_text())
    cams = []
    for name in ('cam0', 'cam1'):
        c = cc[name]
        k = np.eye(3)
        intr = c['intrinsics']
        k[0, 0], k[1, 1], k[0, 2], k[1, 2] = intr
        d = np.array(c['distortion_coeffs'], dtype=float)  # radtan k1 k2 p1 p2
        d5 = np.array([d[0], d[1], d[2], d[3], 0.0])
        t = np.asarray(c['T_cn_cnm1'], dtype=float) if name == 'cam1' else np.eye(4)
        cams.append(dict(K=k, D=d5, T=t))
    return cams  # T[1] maps cam0 coords -> cam1 coords


def detect_board(gray, detector):
    corners, ids, _ = detector.detectMarkers(gray)
    out = {}
    if ids is None:
        return out
    for cid, quad in zip(ids.ravel(), corners):
        out[int(cid)] = np.asarray(quad, dtype=float).reshape(4, 2)
    return out


def main():
    cams = load_camchain()
    detector = make_board_detector(json.loads(JOINT_SETUP.read_text())['board_detector'])
    sides, h_pitches, v_pitches = [], [], []
    used = 0
    for idx in FRAMES:
        p0 = RECS / 'frames' / f'{idx:06d}.jpg'
        p1 = RECS / 'frames_right' / f'{idx:06d}.jpg'
        if not p0.exists() or not p1.exists():
            continue
        g0 = cv2.imread(str(p0), 0)
        g1 = cv2.imread(str(p1), 0)
        d0 = detect_board(g0, detector)
        d1 = detect_board(g1, detector)
        common = sorted(set(d0) & set(d1) & set(range(36)))
        if len(common) < 30:
            continue
        r0 = refine_legacy_corners(g0, {i: d0[i] for i in common})
        r1 = refine_legacy_corners(g1, {i: d1[i] for i in common})
        obj, pts0, pts1 = [], [], []
        for order, i in enumerate(common):
            obj += [order * 4 + j for j in range(4)]
            pts0.extend(r0[i].tolist())
            pts1.extend(r1[i].tolist())
        pts0 = np.asarray(pts0, dtype=float).reshape(-1, 1, 2)
        pts1 = np.asarray(pts1, dtype=float).reshape(-1, 1, 2)
        u0 = cv2.undistortPoints(pts0, cams[0]['K'], cams[0]['D']).reshape(-1, 2)
        u1 = cv2.undistortPoints(pts1, cams[1]['K'], cams[1]['D']).reshape(-1, 2)
        P0 = cams[0]['K'] @ np.hstack([np.eye(3), np.zeros((3, 1))])
        T10 = cams[1]['T']
        R, t = T10[:3, :3], T10[:3, 3:]
        P1 = cams[1]['K'] @ np.hstack([R, t])
        xyz = cv2.triangulatePoints(P0, P1, u0.T, u1.T).T
        xyz = (xyz[:, :3] / xyz[:, 3:]).reshape(-1, 4, 3)
        centers = xyz.mean(axis=1)
        q = np.roll(xyz, -1, axis=1)
        sides.extend(np.linalg.norm(xyz - q, axis=2).ravel().tolist())
        grid = {cid: c for cid, c in zip(common, centers)}
        rows = max(c // 6 for c in common) + 1
        cols = max(c % 6 for c in common) + 1
        for cid, c in grid.items():
            r, col = cid // 6, cid % 6
            if col + 1 < cols and cid + 1 in grid:
                h_pitches.append(float(np.linalg.norm(c - grid[cid + 1])))
            if r + 1 < rows and cid + 6 in grid:
                v_pitches.append(float(np.linalg.norm(c - grid[cid + 6])))
        used += 1
    sides, h, v = np.array(sides), np.array(h_pitches), np.array(v_pitches)
    print(f'frames used: {used} (corners per frame ~{len(sides) / max(used, 1):.0f})')
    print(f'tag side (nominal 35.2mm):   median {np.median(sides) * 1000:.3f} mm  (n={len(sides)})')
    print(f'horizontal pitch (45.76):    median {np.median(h) * 1000:.4f} mm  (n={len(h)})')
    print(f'vertical   pitch (45.76):    median {np.median(v) * 1000:.4f} mm  (n={len(v)})')


if __name__ == '__main__':
    main()
