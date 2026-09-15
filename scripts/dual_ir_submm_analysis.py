#!/usr/bin/env python3
"""Sub-mm mount-z analysis for the dual-IR/color co-capture session.

Chain design -- everything anchored in-session, nothing trusted from prints
or factory stereo scale:

  * The 6x6 board is caliper-verified exact nominal geometry; it is the
    metric transfer standard.
  * Per pose and per device, the board is triangulated in that device's own
    IR stereo (raw, possibly inflated by the device's stereo scale error),
    then a 7-DOF similarity to the nominal lattice solves the device's
    inflation factor AND yields a corrected metric board cloud in that
    device's ir_left frame.  (For the Ego D435i the inflation is expected
    near +1.01%; for the UMI D405 near 1.0.)
  * T_egoIr_umiIr per pose = rigid fit between the two corrected clouds;
    averaged over poses on SE(3).
  * The mount tag is NOT visible to the ego IR pair at this geometry, so it
    rides the color path: per-frame mono PnP of the board in ego.color
    (accepted k1k2k3 model) and ego.ir_left (factory K/D -- the D435i IR
    stereo scale error does not enter a mono PnP), composed into T_ir_color
    per frame and robustly medianed on SE(3) with 3mm translation-outlier
    rejection (soft-focus IR decode noise gives rare 9-20mm outlier frames
    over a ~0.6mm inlier cloud).  T_egoIr_mount = T_ir_color @ T_color_mount
    where T_color_mount is an IPPE_SQUARE PnP of the median mount quad in
    ego.color.  This never trusts the factory color<->IR extrinsic for the
    answer -- only as cross-check 1 (factory_dt).
  * Cross-check 2: ego IR stereo triangulation of the mount tag when the IR
    pair does decode it (usually starved at this geometry -> None).

Outputs T_umiIrleft_mount per pose and combined on SE(3), with per-pose
spreads and both cross-checks.  No calibration activation.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.submm_detection import (make_board_detector_aprilgrid, detect_board_union,
                                     detect_mount_ladder, reject_board_id1)
from scripts.common_board_resampling_analysis import restore_board_id1
from scripts.fixed_factory_board_diagnostic import refine_legacy_corners
from three_device_slam.spatial.apriltag_detector import normalized_camera_points

TAG_M = 0.0352
PITCH_M = 0.04576
MOUNT_M = 0.040
BOOTSTRAP = 200
# Accepted ego RGB model = LOSO-selected k1k2k3, refit on the IR-scale-
# corrected probe points (mount_z_scaled_run1/report.json).  The stale
# hand-copied constant it replaced (fx 889.87 / cy 376.34) mixed k1k2 fx
# with k1k2k3 cy -- 0.9px cy error alone shifts the mount PnP by ~1mm.
K1K2 = dict(K=[[892.6243, 0.0, 664.9773], [0.0, 892.8378, 376.3447], [0.0, 0.0, 1.0]],
            D=[0.078681, 0.017895, 0.0, 0.0, -0.545334])


def nominal_corners():
    out = []
    for i in range(36):
        center = np.array([i % 6, i // 6, 0.]) * PITCH_M
        out.append(center + .5 * TAG_M * np.array([[1., -1, 0], [-1, -1, 0], [-1, 1, 0], [1, 1, 0]]))
    return np.concatenate(out)


def load_calibration(setup, device, stream):
    intr = setup['devices'][device]['intrinsics'][stream]
    from three_device_slam.spatial.pair_tag_alignment import _camera_calibration_from_intrinsics
    return _camera_calibration_from_intrinsics(
        calibration_id=f"submm:{device}:{stream}", frame_id=f'{device}.{stream}', intrinsics=intr)


def transform_from_snapshot(item):
    r = np.asarray(item['rotation_row_major'], dtype=float).reshape(3, 3)
    t = np.asarray(item['translation_m'], dtype=float)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = r, t
    return T


def umeyama(src, dst, with_scale):
    """src ~ s*R*dst + t (returns s, R, t fitting src from dst)."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    H = (dst - cd).T @ (src - cs)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U @ Vt))
    R = Vt.T @ np.diag([1., 1., d]) @ U.T
    s = S.sum() / ((dst - cd) ** 2).sum() if with_scale else 1.0
    t = cs - s * R @ cd
    return s, R, t


def triangulate(cal_l, cal_r, T_r_l, px_l, px_r):
    u_l = normalized_camera_points(px_l, cal_l)
    u_r = normalized_camera_points(px_r, cal_r)
    P0 = np.hstack([np.eye(3), np.zeros((3, 1))])
    P1 = T_r_l[:3]
    q = cv2.triangulatePoints(P0, P1, np.asarray(u_l).T, np.asarray(u_r).T).T
    return q[:, :3] / q[:, 3:]


def kabsch_pose(src3d, dst3d):
    """Rigid transform mapping dst3d -> src3d (row-wise paired)."""
    _, R, t = umeyama(src3d, dst3d, with_scale=False)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def detect_board(gray, detector):
    # Union of raw + CLAHE passes (submm_detection), then id1 corner restore.
    merged = detect_board_union(detector, gray)
    # id1 disambiguation: the mount tag shares id1 with the board.  The union
    # dict keeps one quad per id, so when both decode the choice is a coin
    # flip -- drop the instance that falls OUTSIDE the hull of the other board
    # tags (that one is the mount; the mount ladder picks it up separately).
    if 1 in merged and len(merged) >= 5:
        others = [np.asarray(q, np.float32).reshape(4, 2).mean(axis=0, keepdims=True)
                  for t, q in merged.items() if t != 1]
        hull = cv2.convexHull(np.concatenate(others).astype(np.float32))
        c1 = np.asarray(merged[1], np.float32).reshape(4, 2).mean(axis=0)
        if cv2.pointPolygonTest(hull, (float(c1[0]), float(c1[1])), False) < 0:
            del merged[1]
    if 1 in merged:
        # restore_board_id1 refits the id1 quad from the grid homography; on
        # weak IR frames the fit can be under-supported -- keep the raw decode.
        try:
            from types import SimpleNamespace
            dets = [SimpleNamespace(tag_id=tid, corners=q) for tid, q in merged.items()]
            fixed, _ = restore_board_id1(dets)
            if fixed is not None:
                merged[1] = np.asarray(fixed, dtype=float).reshape(4, 2)
        except (ValueError, cv2.error):
            pass
    return merged


def detect_mount(gray):
    return [np.asarray(getattr(q, 'corners', q), dtype=float).reshape(4, 2) for q in detect_mount_ladder(gray)]


def pnp_pose(obj_pts, img_pts, K, D, square=False):
    flags = cv2.SOLVEPNP_IPPE_SQUARE if square else cv2.SOLVEPNP_SQPNP
    ok, rvec, tvec = cv2.solvePnP(np.asarray(obj_pts, float), np.asarray(img_pts, float),
                                  np.asarray(K, float), np.asarray(D, float), flags=flags)
    if not ok:
        return None
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(rvec)
    T[:3, 3] = tvec.ravel()
    return T


def quat_mean(Ts):
    """SE(3) median of rigid transforms: Shepperd DCM->quat, sign-aligned
    eigen-mean for rotation, coordinate-wise median for translation."""
    if len(Ts) == 1:
        return Ts[0]

    def dcm_to_quat(R):
        m00, m01, m02 = R[0]
        m10, m11, m12 = R[1]
        m20, m21, m22 = R[2]
        tr = m00 + m11 + m22
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            return np.array([s / 4, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s])
        if m00 > m11 and m00 > m22:
            s = np.sqrt(1.0 + m00 - m11 - m22) * 2
            return np.array([(m21 - m12) / s, s / 4, (m01 + m10) / s, (m02 + m20) / s])
        if m11 > m22:
            s = np.sqrt(1.0 + m11 - m00 - m22) * 2
            return np.array([(m02 - m20) / s, (m01 + m10) / s, s / 4, (m12 + m21) / s])
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2
        return np.array([(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4])

    Q = np.array([dcm_to_quat(T[:3, :3]) for T in Ts])
    Q = Q / np.linalg.norm(Q, axis=1, keepdims=True)
    for i in range(1, len(Q)):
        if Q[i] @ Q[0] < 0:
            Q[i] = -Q[i]
    _, vecs = np.linalg.eigh(Q.T @ Q)
    w, x, y, z = vecs[:, -1]
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, np.median(np.array([T[:3, 3] for T in Ts]), axis=0)
    return T


def se3_median_robust(Ts, gate_mm=3.0):
    """Median of rigid transforms with translation-outlier rejection.

    Soft-focus ego IR decode noise produces occasional 9-20mm outlier
    frames while the inlier half scatters ~0.6mm (pose-0 probe).  Gate on
    distance to the provisional translation median, then quat_mean the
    inliers.  Falls back to all frames when the gate would starve the fit
    (degenerate capture); the raw spread is always reported alongside.
    """
    t = np.array([T[:3, 3] for T in Ts])
    d = np.linalg.norm(t - np.median(t, axis=0), axis=1)
    mask = d * 1000 <= gate_mm
    if mask.sum() < max(3, len(Ts) // 2):
        mask = np.ones(len(Ts), bool)
    inliers = [T for T, m in zip(Ts, mask) if m]
    return quat_mean(inliers), int(mask.sum()), float(d.max() * 1000), float(d[mask].max() * 1000)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--k1k2-json', '--ego-rgb-json', dest='k1k2_json', type=Path, default=None,
                        help='accepted ego RGB model json {K,D} (else built-in k1k2k3_scaled)')
    args = parser.parse_args(argv)
    out = cb.new_output(args.output)

    setup = json.loads((args.session / 'setup.json').read_text())
    cals = {(d, s): load_calibration(setup, d, s)
            for d in ('ego', 'umi') for s in ('color', 'ir_left', 'ir_right')}
    if args.k1k2_json:
        model = json.loads(args.k1k2_json.read_text())
        K, D = np.asarray(model['K'], float), np.asarray(model['D'], float)
    else:
        K, D = np.asarray(K1K2['K'], float), np.asarray(K1K2['D'], float)
    from dataclasses import replace
    cals[('ego', 'color')] = replace(cals[('ego', 'color')], camera_matrix=K,
                                     distortion_coefficients=D, distortion_model='opencv_brown_conrady')
    T_ego_ir_r = transform_from_snapshot(setup['extrinsics']['ego_ir_right_from_ir_left'])
    T_ego_color = transform_from_snapshot(setup['extrinsics']['ego_color_from_ir_left'])
    T_umi_ir_r = transform_from_snapshot(setup['extrinsics']['umi_ir_right_from_ir_left'])
    nominal = nominal_corners()

    detector = make_board_detector_aprilgrid()

    poses = sorted(p.name for p in args.session.glob('pose_*') if p.is_dir())
    MIN_TAGS = 15  # ego IR is soft-focus; 15 spread tags still over-constrain the 7-DoF fit

    per_pose = {}
    for pose in poses:
        rows = [json.loads(t) for t in (args.session / pose / 'frames.jsonl').read_text().splitlines()]
        streams = ('ego.ir_left', 'ego.ir_right', 'ego.color', 'umi.ir_left', 'umi.ir_right')
        tag_corners = {s: {i: [] for i in range(36)} for s in streams}
        mount_px = {'ego.ir_left': [], 'ego.ir_right': [], 'ego.color': []}
        for row in rows:
            images = {s: cv2.imread(str(args.session / pose / row['streams'][s]['file']), 0) for s in streams}
            for s in streams:
                board = detect_board(images[s], detector)
                if len(board) >= MIN_TAGS:
                    r = refine_legacy_corners(images[s], {i: board[i] for i in board})
                    for i, q in r.items():
                        tag_corners[s][i].append(q)
            for s in mount_px:
                for q in reject_board_id1(detect_mount(images[s]), detect_board(images[s], detector)):
                    mount_px[s].append(q)
        med = {}
        for s in streams:
            med[s] = {i: np.median(v, axis=0) for i, v in tag_corners[s].items() if len(v) >= max(3, 0.3 * len(rows))}
        common = sorted(set.intersection(*[set(med[s]) for s in streams]))
        if len(common) < MIN_TAGS:
            raise ValueError(f'{pose}: only {len(common)} common tags across streams')
        drift = {s: max((float(np.max(np.linalg.norm(np.asarray(tag_corners[s][i]) - med[s][i], axis=(1, 2))))
                         for i in common if len(tag_corners[s][i]) > 1), default=0.0)
                 for s in streams}

        # per-device board triangulation + scale correction to nominal.  The
        # similarity fit raw ~= s*R*nominal + t is T_device_nominal; chaining
        # the two fits gives T_egoIr_umiIr (kabsch between nominal-corrected
        # clouds would wrongly recover ~identity -- both clouds were aligned
        # to the same nominal frame).
        idx = [j for t in common for j in range(4 * t, 4 * t + 4)]
        nominal_common = nominal[idx]
        fits = {}
        scales = {}
        for dev, T_r, key_l, key_r in (('ego', T_ego_ir_r, 'ego.ir_left', 'ego.ir_right'),
                                       ('umi', T_umi_ir_r, 'umi.ir_left', 'umi.ir_right')):
            px_l = np.concatenate([med[key_l][i] for i in common])
            px_r = np.concatenate([med[key_r][i] for i in common])
            raw = triangulate(cals[(dev, 'ir_left')], cals[(dev, 'ir_right')], T_r, px_l, px_r)
            fits[dev] = umeyama(raw, nominal_common, with_scale=True)
            scales[dev] = fits[dev][0]
        s_e, R_e, t_e = fits['ego']
        s_u, R_u, t_u = fits['umi']
        R_eu = R_e @ R_u.T
        t_eu = t_e - (s_e / s_u) * R_eu @ t_u
        T_egoIr_umiIr = np.eye(4)
        T_egoIr_umiIr[:3, :3], T_egoIr_umiIr[:3, 3] = R_eu, t_eu
        ego_corr = (s_e, R_e, t_e)

        # --- in-session T_ir_color from board co-observation (per frame, then
        # SE(3) median). Mono PnP on both streams: ego.color with the accepted
        # k1k2k3 model, ego.ir_left with factory K/D. The ego-IR stereo scale
        # error does not enter a mono PnP, so this transfer is metric-clean.
        # (The ego-IR pair cannot see the mount tag at this geometry, so the
        # mount rides: T_ir_mount = T_ir_color @ T_color_mount.)
        K_ir = np.asarray(cals[('ego', 'ir_left')].camera_matrix, float)
        D_ir = np.asarray(cals[('ego', 'ir_left')].distortion_coefficients, float)
        per_frame_T_ir_color = []
        for fi, row in enumerate(rows):
            imgs = {s: cv2.imread(str(args.session / pose / row['streams'][s]['file']), 0) for s in streams}
            boards_f = {}
            for s in ('ego.color', 'ego.ir_left'):
                boards_f[s] = detect_board(imgs[s], detector)
            common_f = sorted(set(boards_f['ego.color']) & set(boards_f['ego.ir_left']))
            if len(common_f) < MIN_TAGS:
                continue
            obj_f = np.concatenate([nominal[4 * t:4 * t + 4] for t in common_f])
            img_c = np.concatenate([refine_legacy_corners(imgs['ego.color'],
                                 {t: boards_f['ego.color'][t] for t in common_f})[t] for t in common_f])
            img_i = np.concatenate([refine_legacy_corners(imgs['ego.ir_left'],
                                 {t: boards_f['ego.ir_left'][t] for t in common_f})[t] for t in common_f])
            T_c_b = pnp_pose(obj_f, img_c, K, D)
            T_i_b = pnp_pose(obj_f, img_i, K_ir, D_ir)
            if T_c_b is not None and T_i_b is not None:
                per_frame_T_ir_color.append(T_i_b @ np.linalg.inv(T_c_b))
        if not per_frame_T_ir_color:
            raise ValueError(f'{pose}: no frame yielded board PnP in both ego.color and ego.ir_left')
        T_ir_color, n_ir_color_inliers, t_spread_raw, t_spread = se3_median_robust(per_frame_T_ir_color)

        # mount: ego.color PnP (IPPE_SQUARE on the median quad), then transfer
        half = MOUNT_M / 2
        canonical = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
        T_color_mount = None
        if mount_px['ego.color']:
            m_c = np.median(np.asarray(mount_px['ego.color']), axis=0)
            T_color_mount = pnp_pose(canonical, m_c, K, D, square=True)
        if T_color_mount is None:
            raise ValueError(f'{pose}: mount tag never detected in ego.color')
        T_egoIr_mount = T_ir_color @ T_color_mount

        # cross-check 1: factory color->ir snapshot vs measured
        T_factory = transform_from_snapshot(setup['extrinsics']['ego_color_from_ir_left'])
        T_ir_color_factory = np.linalg.inv(T_factory)
        factory_dt = float(np.linalg.norm((np.linalg.inv(T_ir_color) @ T_ir_color_factory)[:3, 3]) * 1000)
        # same answer if the transfer rides the FACTORY extrinsic instead of
        # the in-session measured one -- the pair brackets the extrinsic
        # uncertainty (factory_dt is their direct delta).
        T_egoIr_mount_f = T_ir_color_factory @ T_color_mount
        T_umiIr_mount_f = np.linalg.inv(T_egoIr_umiIr) @ T_egoIr_mount_f

        # cross-check 2: ego IR stereo triangulation of the mount (usually
        # starved by the soft-focus IR pair; None when undetected)
        ir_stereo_mount = None
        n_mount = min(len(mount_px['ego.ir_left']), len(mount_px['ego.ir_right']))
        if n_mount > 0:
            m_l = np.median(np.asarray(mount_px['ego.ir_left'][:n_mount]), axis=0)
            m_r = np.median(np.asarray(mount_px['ego.ir_right'][:n_mount]), axis=0)
            m_raw = triangulate(cals[('ego', 'ir_left')], cals[('ego', 'ir_right')], T_ego_ir_r, m_l, m_r)
            s_e, R_e, t_e = ego_corr
            m_corr = (m_raw - t_e) @ R_e / s_e
            ir_stereo_mount = kabsch_pose(m_corr, canonical)

        T_final = np.linalg.inv(T_egoIr_umiIr) @ T_egoIr_mount
        per_pose[pose] = dict(scales=scales, drift_px=drift, n_frames=len(rows),
                              n_mount_frames=len(mount_px['ego.color']),
                              n_ir_color_frames=len(per_frame_T_ir_color),
                              n_ir_color_inliers=n_ir_color_inliers,
                              ir_color_translation_spread_mm=t_spread,
                              ir_color_translation_spread_all_frames_mm=t_spread_raw,
                              T_egoIr_umiIr=T_egoIr_umiIr.tolist(), T_egoIr_mount=T_egoIr_mount.tolist(),
                              T_umiIr_mount=T_final.tolist(),
                              T_umiIr_mount_via_factory_extrinsic=T_umiIr_mount_f.tolist(),
                              ir_color_vs_factory_dt_mm=factory_dt,
                              ir_stereo_mount=(ir_stereo_mount.tolist() if ir_stereo_mount is not None else None))
        z = T_final[2, 3] * 1000
        n = np.linalg.norm(T_final[:3, 3]) * 1000
        print(f"{pose}: s_ego={scales['ego']:.6f} s_umi={scales['umi']:.6f} "
              f"drift={max(drift.values()):.2f}px mount_frames={len(mount_px['ego.color'])} "
              f"TirColor_spread={t_spread:.3f}mm(raw {t_spread_raw:.3f}, "
              f"{n_ir_color_inliers}/{len(per_frame_T_ir_color)} inl) factory_dt={factory_dt:.3f}mm | "
              f"z={z:.3f}mm norm={n:.3f}mm", flush=True)

    # combine poses on SE(3) (quat_mean: Shepperd + eigen-mean + median t)
    Ts = [np.asarray(p['T_umiIr_mount']) for p in per_pose.values()]
    T_mean = quat_mean(Ts)

    z_spread = max(float(T[2, 3]) for T in Ts) - min(float(T[2, 3]) for T in Ts)
    t_mean = T_mean[:3, 3]
    summary = dict(xyz_mm=(t_mean * 1000).round(4).tolist(),
                   z_mm=float(T_mean[2, 3] * 1000), norm_mm=float(np.linalg.norm(t_mean) * 1000),
                   per_pose_z_spread_mm=float(z_spread * 1000),
                   per_pose=dict((k, dict(z_mm=float(np.asarray(v['T_umiIr_mount'])[2, 3] * 1000),
                                          norm_mm=float(np.linalg.norm(np.asarray(v['T_umiIr_mount'])[:3, 3]) * 1000)))
                                 for k, v in per_pose.items()))
    print('COMBINED', json.dumps(summary), flush=True)

    out.mkdir()
    cb.write_json(out / 'report.json', dict(
        status='SUBMM_MOUNT_Z_DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        method=__doc__.splitlines()[0],
        nominal_geometry_m=dict(tag=TAG_M, pitch=PITCH_M, mount=MOUNT_M),
        ego_rgb_model=dict(name='k1k2k3_scaled', K=K.tolist(), D=D.tolist()),
        poses=per_pose, combined=summary,
        factory_values_modified=False,
        code_sha256=cb.digest(Path(__file__).resolve())))
    print(f'report={out / "report.json"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
