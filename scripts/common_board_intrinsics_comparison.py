#!/usr/bin/env python3
"""Offline, operator-authorized imaging comparison; never export deployable calibration."""
from __future__ import annotations

import argparse
from dataclasses import replace
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from scripts import common_board_calibration as cb

CASES = ((None, 'factory'), ('ego', 'focal'), ('left', 'focal'),
         ('ego', 'K4'), ('left', 'K4'))
D_CASES = ((None, 'factory'), ('ego', 'k1'), ('left', 'k1'),
           ('ego', 'D5'), ('left', 'D5'))
KD_CASES = ((None, 'factory'), ('ego', 'focal_k1'), ('left', 'focal_k1'),
            ('ego', 'K4_D5'), ('left', 'K4_D5'))


def parameter_spec(mode):
    if mode in ('focal_k1', 'K4_D5'):
        a, b = ('focal', 'k1') if mode == 'focal_k1' else ('K4', 'D5')
        return tuple(x+y for x, y in zip(parameter_spec(a), parameter_spec(b)))
    if mode == 'factory':
        return [], [], []
    if mode == 'focal':
        return [np.log(.9)], [np.log(1.1)], [1e-4]
    if mode == 'K4':
        return [np.log(.9)]*2 + [-40.]*2, [np.log(1.1)]*2 + [40.]*2, [1e-4, 1e-4, .02, .02]
    if mode == 'k1':
        return [-.2], [.2], [1e-4]
    if mode == 'D5':
        bounds = np.array([.3, .5, .02, .02, .5])
        return (-bounds).tolist(), bounds.tolist(), [1e-4]*5
    raise ValueError('unknown imaging mode')


def changed_camera(factory, delta, mode):
    """Make a diagnostic copy, keeping the original distortion convention."""
    if mode in ('focal_k1', 'K4_D5'):
        a, b, n = ('focal', 'k1', 1) if mode == 'focal_k1' else ('K4', 'D5', 4)
        return changed_camera(changed_camera(factory, delta[:n], a), delta[n:], b)
    k = factory.camera_matrix.copy()
    d = factory.distortion_coefficients.copy()
    if mode == 'focal':
        k[0, 0] *= np.exp(delta[0])
        k[1, 1] *= np.exp(delta[0])
    elif mode == 'K4':
        k[0, 0] *= np.exp(delta[0])
        k[1, 1] *= np.exp(delta[1])
        k[0, 2] += delta[2]
        k[1, 2] += delta[3]
    elif mode == 'k1':
        d[0] += delta[0]
    elif mode == 'D5':
        d += delta
    else:
        raise ValueError('unknown intrinsics mode')
    return replace(factory, calibration_id=factory.calibration_id + '.DIAGNOSTIC_ONLY',
                   camera_matrix=k, distortion_coefficients=d)


def fit_case(windows, factory, role, mode, *, k_derivative_scale=1.):
    if not np.isfinite(k_derivative_scale) or k_derivative_scale <= 0:
        raise ValueError('positive derivative scale required')
    if (role, mode) not in CASES + D_CASES + KD_CASES:
        raise ValueError('unregistered comparison case')
    if [w['slot'] for w in windows] not in (list(range(12)), list(range(12)) + [15]):
        raise ValueError('frozen slots required')
    if any(w['split'] != cb.STEPS[w['slot']][2] for w in windows):
        raise ValueError('frozen split required')
    train = [w for w in windows if w['split'] == 'train']
    initial = [{r: cb.fit_pose(w['objects'], w['pixels'][r], factory[r])
                for r in cb.ROLES} for w in train]
    xs = [p['ego'] @ np.linalg.inv(p['left']) for p in initial]
    x0 = min(xs, key=lambda a: sum(sum(cb.difference(a, b)) for b in xs))
    pose_start = np.concatenate([cb.pack(x0)] + [cb.pack(p['left']) for p in initial])
    offset = len(pose_start)
    lower_k, upper_k, parameter_steps = parameter_spec(mode)
    count = len(lower_k)
    start = np.r_[pose_start, np.zeros(count)]

    @lru_cache(maxsize=32)
    def geometry(delta):
        cameras = dict(factory)
        if count:
            cameras[role] = changed_camera(factory[role], delta, mode)
        rays = [{r: cb.normalized_camera_points(w['pixels'][r], cameras[r])
                 for r in cb.ROLES} for w in train]
        scales = {r: np.diag(cameras[r].camera_matrix)[:2] for r in cb.ROLES}
        return cameras, rays, scales

    def residual(vector):
        _, rays, scales = geometry(tuple(vector[offset:]))
        x = cb.unpack(vector[:6])
        values = []
        for j, w in enumerate(train):
            b = cb.unpack(vector[6+6*j:12+6*j])
            for r, transform in (('ego', x @ b), ('left', b)):
                q = cb.transform_points(w['objects'], transform)
                values.extend(((q[:, :2]/np.maximum(q[:, 2:], 1e-6)-rays[j][r])*scales[r]).ravel())
        return np.asarray(values)

    def jacobian(vector):
        # SDK deprojection uses float32. Explicit central steps avoid zero/noisy
        # K derivatives; poses are differentiated in float64 normalized rays.
        steps = np.full(len(vector), 1e-6)
        if count:
            steps[offset:] = parameter_steps
            steps[offset:] *= k_derivative_scale
        columns = []
        for i, step in enumerate(steps):
            a, b = vector.copy(), vector.copy()
            a[i] += step
            b[i] -= step
            columns.append((residual(a)-residual(b))/(2*step))
        return np.column_stack(columns)

    fit = least_squares(residual, start, jac=jacobian, x_scale='jac', loss='linear',
                        bounds=(np.r_[np.full(offset, -np.inf), lower_k],
                                np.r_[np.full(offset, np.inf), upper_k]),
                        max_nfev=150, ftol=1e-9, xtol=1e-9, gtol=1e-8)
    cameras, _, _ = geometry(tuple(fit.x[offset:]))
    x = cb.unpack(fit.x[:6])
    training = []
    for j, w in enumerate(train):
        b = cb.unpack(fit.x[6+6*j:12+6*j])
        for r, transform in (('ego', x @ b), ('left', b)):
            training.append(dict(slot=w['slot'], role=r,
                                 **cb.errors(cb.project(w['objects'], transform, cameras[r]), w['pixels'][r])))
    heldout = []
    for w in windows:
        if w['split'] != 'holdout':
            continue
        for source, destination, link in (('left', 'ego', x), ('ego', 'left', np.linalg.inv(x))):
            b = cb.fit_pose(w['objects'], w['pixels'][source], cameras[source])
            heldout.append(dict(slot=w['slot'], source=source, destination=destination,
                                **cb.errors(cb.project(w['objects'], link @ b, cameras[destination]),
                                            w['pixels'][destination])))
    norms = np.linalg.norm(fit.jac, axis=0)
    sv = np.linalg.svd(fit.jac/np.maximum(norms, 1e-30), compute_uv=False)
    bound_hit = bool(count and np.any((fit.x[offset:]-lower_k < 1e-4) | (upper_k-fit.x[offset:] < 1e-4)))
    return dict(status='DIAGNOSTIC_ONLY', role=role, mode=mode, activation='NOT_ACTIVATED',
                k_derivative_scale=k_derivative_scale,
                optimizer=dict(success=bool(fit.success), message=fit.message, nfev=fit.nfev,
                               cost=fit.cost, optimality=fit.optimality),
                bound_hit=bound_hit, column_scaled_jacobian_condition=float(sv[0]/sv[-1]),
                candidate_K={r: cameras[r].camera_matrix.tolist() for r in cb.ROLES},
                candidate_D={r: cameras[r].distortion_coefficients.tolist() for r in cb.ROLES},
                delta=fit.x[offset:].tolist(), T_Ecolor_Ucolor=x.tolist(),
                training=training, holdout=heldout,
                all_holdout_p95_le_1px=all(v['p95_px'] <= 1. for v in heldout))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--k-derivative-scale', type=float, default=1.)
    parser.add_argument('--experiment', choices=('K', 'D', 'KD'), default='K')
    args = parser.parse_args(argv)
    output = cb.new_output(args.output)
    setup, factory, windows, _, hashes, audit = cb.load_session(args.session)
    output.mkdir(parents=True, exist_ok=False)
    reports = []
    cases = {'K': CASES, 'D': D_CASES, 'KD': KD_CASES}[args.experiment]
    for role, mode in cases:
        name = 'factory' if role is None else role + '_' + mode
        print('COMPARE: ' + name + '; train8 only, fixed layout/size/model enum', flush=True)
        result = fit_case(windows, factory, role, mode, k_derivative_scale=args.k_derivative_scale)
        cb.write_json(output/(name + '.json'), result)
        reports.append(result)
        print(f"DONE: {name}; worst holdout p95={max(v['p95_px'] for v in result['holdout']):.3f}px; bound_hit={result['bound_hit']}", flush=True)
    report = dict(schema='ego.common_board_intrinsics_comparison.v1', status='DIAGNOSTIC_ONLY',
                  activation='NOT_ACTIVATED', factory_files_modified=False,
                  source_session=str(args.session.resolve()), source_hashes=hashes,
                  devices=setup['devices'], target=setup['target'], audit=audit,
                  experiment=args.experiment, cases=reports, case_order=[(r, m) for r, m in cases],
                  frozen_bounds=dict(focal_scale=[.9, 1.1], principal_delta_px=[-40, 40]),
                  distortion_delta_bounds={m: parameter_spec(m)[:2] for m in ('k1', 'D5')},
                  objective='linear normalized candidate-camera ray residuals weighted by candidate fx/fy; cost is not exact pixel SSE',
                  scoring='actual SDK-model pixel projection; all frozen whole-window bidirectional holdouts',
                  code_sha256={str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
                      Path(__file__).resolve(), cb.ROOT/'scripts/common_board_calibration.py',
                      cb.ROOT/'scripts/fixed_factory_board_diagnostic.py',
                      cb.ROOT/'scripts/color_aprilgrid_capture.py',
                      cb.ROOT/'three_device_slam/spatial/apriltag_detector.py')},
                  limitations=['one camera at a time; K fixed in D experiment, D fixed in K experiment, both free in KD',
                               'optimizer success means termination; inspect optimality and derivative-step sensitivity',
                               'reused diagnostic holdouts, not new independent acceptance',
                               'step6 and step7 have near-duplicate poses; no relabel/deletion',
                               'improvement may absorb board/corner/model bias, not proof of factory error',
                               'no mounting solve, no parameter export, no deployment'])
    cb.write_json(output/'report.json', report)
    print('DIAGNOSTIC_ONLY: ' + str(output/'report.json'), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
