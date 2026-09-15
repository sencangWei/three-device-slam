"""Exploratory shared-external-layout contrast on an already-seen validation set."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from . import mount_bundle_diagnostic as b


def shared_fit(sessions):
    seed = b.fit(sessions)
    if not seed['success']:
        raise ValueError('initial fit failed')
    initial = np.concatenate([b.vector(seed['mount']),
                    *[b.vector(b.mean_pose([e[i] for _, e in seed['poses']])) for i in (1, 2)],
                    *[b.vector(x) for x, _ in seed['poses']]])

    def unpack(v):
        return (b.matrix(v[:6]), {1: b.matrix(v[6:12]), 2: b.matrix(v[12:18])},
                [b.matrix(v[18+6*i:24+6*i]) for i in range(len(sessions))])

    def residual(v):
        m, ext, xs = unpack(v)
        parts = []
        for s, x in zip(sessions, xs):
            for key, group in s['groups'].items():
                device, label = key.split('_')
                t = m if label == 'mount' else ext[int(label[-1])]
                if device == 'ego':
                    t = x @ t
                corners = np.array([o['corners'] for o in group])
                try:
                    parts.append((b.project(t, s['calibration'][device])-corners).ravel())
                except ValueError:
                    parts.append(np.full(corners.size, 1e6))
        return np.concatenate(parts)

    fit = least_squares(residual, initial, loss='soft_l1', f_scale=1., x_scale='jac',
                        max_nfev=300, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    if not fit.success:
        raise ValueError('shared layout fit did not converge: '+fit.message)
    m, ext, xs = unpack(fit.x)
    return {'mount': m, 'externals': ext, 'ego_poses': xs, 'nfev': fit.nfev}


def register_external_only(session, externals):
    """Fit only Ego pose from Ego external corners; no mount or UMI pose seeds."""
    seeds = [b.mean_pose([o['pose'] for o in session['groups'][f'ego_ext{i}']])
             @ np.linalg.inv(externals[i]) for i in (1, 2)]

    def residual(v):
        x = b.matrix(v)
        parts = []
        for i in (1, 2):
            corners = np.array([o['corners'] for o in session['groups'][f'ego_ext{i}']])
            try:
                parts.append((b.project(x@externals[i], session['calibration']['ego'])-corners).ravel())
            except ValueError:
                parts.append(np.full(corners.size, 1e6))
        return np.concatenate(parts)
    fit = least_squares(residual, b.vector(b.mean_pose(seeds)), loss='soft_l1', f_scale=1.,
                        x_scale='jac', max_nfev=300, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    if not fit.success:
        raise ValueError('validation external registration did not converge')
    return b.matrix(fit.x)


def run(sessions):
    if len(sessions) != 3 or len({s['session'] for s in sessions}) != 3:
        raise ValueError('three distinct sessions required')
    free = b.fit(sessions[:2]); nuisance = b.fit([sessions[2]], use_mount=False)
    if not free['success'] or not nuisance['success']:
        raise ValueError('comparison fit did not converge')
    shared = shared_fit(sessions[:2])
    fixed_x = register_external_only(sessions[2], shared['externals'])
    registrations = {'free': nuisance['poses'][0][0], 'fixed_layout': fixed_x}
    mounts = {'free': free['mount'], 'fixed_layout': shared['mount']}
    table = {f'mount={m},registration={x}': b.score_mount(sessions[2], pose, mount)
             for m, mount in mounts.items() for x, pose in registrations.items()}
    groups = {}
    for s, x in list(zip(sessions[:2], shared['ego_poses']))+[(sessions[2], fixed_x)]:
        for device in ('ego', 'left'):
            for i in (1, 2):
                t = shared['externals'][i]
                if device == 'ego':
                    t = x@t
                corners = np.array([o['corners'] for o in s['groups'][f'{device}_ext{i}']])
                groups[f"{s['session']}/{device}_ext{i}"] = b.pixel_metrics(b.project(t, s['calibration'][device])-corners)
    return {'schema': 'ego.mount.fixed_layout_diagnostic.v1',
            'training_sessions': [s['session'] for s in sessions[:2]],
            'validation_session': sessions[2]['session'],
            'validation_status': 'REUSED_VALIDATION_NOT_FRESH_TEST',
            'score_table': table, 'external_residuals': groups,
            'mount_umi_ir_left': b.mc._xyz_rpy(np.array(sessions[0]['ir_from_color'])@shared['mount']),
            'mount_umi_color': shared['mount'].tolist(),
            'fixed_external_umi_color': {str(i): t.tolist() for i,t in shared['externals'].items()},
            'validation_ego_from_umi_color': fixed_x.tolist(),
            'activation': 'NOT_ACTIVATED', 'verdict': 'DIAGNOSTIC_ONLY'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--corners', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = run(json.loads(args.corners.read_text()))
    (args.output/'report.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({'score_table': result['score_table'], 'mount': result['mount_umi_ir_left']}, indent=2))


if __name__ == '__main__':
    main()
