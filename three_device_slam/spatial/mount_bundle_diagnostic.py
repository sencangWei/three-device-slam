"""Offline fixed-intrinsics corner fit; diagnostic only, never activates calibration.

UMI color is the gauge. M maps mount to UMI color; X_s maps UMI to Ego;
A_sj maps external tag j to UMI. Ego predictions are X_s M and X_s A_sj.
Held-out nuisance fits MUST exclude mount observations, including their seeds.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from . import mount_calibration as mc
from . import pair_tag_alignment as pta
from .apriltag_detector import AprilTagDetectorConfig, detect_apriltags


def matrix(v):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    t[:3, 3] = v[3:]
    return t


def vector(t):
    return np.r_[Rotation.from_matrix(t[:3, :3]).as_rotvec(), t[:3, 3]]


def mean_pose(poses):
    poses = np.asarray(poses)
    t = np.eye(4)
    t[:3, :3] = Rotation.from_matrix(poses[:, :3, :3]).mean().as_matrix()
    t[:3, 3] = np.median(poses[:, :3, 3], axis=0)
    return t


def project(t, calibration, size=.04):
    import cv2
    h = size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    depths = (obj @ t[:3, :3].T + t[:3, 3])[:, 2]
    if np.any(depths <= 0):
        raise ValueError("nonpositive projected depth")
    return cv2.projectPoints(obj, vector(t)[:3], t[:3, 3],
                             np.array(calibration['K']), np.array(calibration['D']))[0].reshape(4, 2)


def selected_pose(detection):
    return detection.candidates[detection.selected_index].camera_from_tag


def label_detections(accepted, device):
    """Fixed bench range bands; external sampling never requires a mount detection.

    This diagnostic's Ego mount is <300mm and same-ID external is >350mm.
    Ambiguous/multiple candidates in a band are rejected, never inferred from
    a held-out mount pose. These bands are not a general tracking policy.
    """
    tagged = {}
    for d in accepted:
        if d.tag_id == 2:
            label = 'ext2'
        elif d.tag_id == 1 and device == 'left':
            label = 'ext1'
        elif d.tag_id == 1:
            distance = np.linalg.norm(selected_pose(d)[:3, 3])
            if distance < .30:
                label = 'mount'
            elif distance > .35:
                label = 'ext1'
            else:
                continue
        else:
            continue
        tagged.setdefault(label, []).append(d)
    return [(key, values[0]) for key, values in tagged.items() if len(values) == 1]


def extract(session: Path, stride=20):
    """Extract accepted corners using fixed, independent identity bands."""
    if stride < 1:
        raise ValueError('stride must be positive')
    result = {'session': session.name, 'groups': {}, 'calibration': {}, 'counts': {},
              'ir_from_color': mc._color_to_ir_transform(session, 'left').tolist()}
    config = AprilTagDetectorConfig(family='tag36h11', tag_size_m=.04,
                                   allowed_tag_ids=(1, 2), ambiguity_rotation_deg=2.,
                                   ambiguity_translation_m=.002)
    for device, stream in [('ego', 'ego.color'), ('left', 'left.color')]:
        cal = pta.load_device_calibration(session, device, stream)
        result['calibration'][device] = {'K': cal.camera_matrix.tolist(),
                                         'D': cal.distortion_coefficients.tolist()}
        rows = pta.load_formal_index_rows(session, device, stream)[::stride]
        counts = {'sampled_frames': len(rows), 'missing_or_ambiguous_groups': 0,
                  'geometry_rejected_observations': 0}
        for row in rows:
            batch = detect_apriltags(pta.load_frame_image(session, device, stream, row),
                                    calibration=cal, config=config,
                                    acquisition_timestamp_ns=row['acquisition_ns'],
                                    # Offline replay may run after a host reboot.
                                    # This synthetic completion is NOT a latency measurement.
                                    detector_completed_ns=max(time.monotonic_ns(), row['acquisition_ns']))
            accepted = [d for d in batch.detections if d.accepted]
            labelled = label_detections(accepted, device)
            counts['missing_or_ambiguous_groups'] += (3 if device == 'ego' else 2) - len(labelled)
            for label, d in labelled:
                t = selected_pose(d)
                edge = np.linalg.norm(d.corners_px - np.roll(d.corners_px, 1, axis=0), axis=1).min()
                angle = np.degrees(np.arccos(np.clip(abs(t[:3, 2] @ t[:3, 3]) / np.linalg.norm(t[:3, 3]), 0, 1)))
                if edge < 60 or angle >= 45:
                    counts['geometry_rejected_observations'] += 1
                    continue
                group = result['groups'].setdefault(device + '_' + label, [])
                group.append({'sequence': row['sequence'], 'corners': d.corners_px.tolist(),
                              'pose': t.tolist(), 'min_edge_px': float(edge),
                              'incidence_deg': float(angle)})
        result['counts'][device] = counts
    for key in ['ego_mount', 'ego_ext1', 'ego_ext2', 'left_ext1', 'left_ext2']:
        if len(result['groups'].get(key, [])) < 8:
            raise ValueError('too few accepted observations: ' + key)
    return result


def fit(sessions, *, mount=None, use_mount=True, external_ids=(1, 2)):
    """Fit training mount, or external-only nuisance poses if use_mount=False."""
    if not sessions or not external_ids or any(i not in (1, 2) for i in external_ids):
        raise ValueError('sessions and supported external ids required')
    if not use_mount and mount is None:
        # A placeholder is returned but never optimized or used in residuals.
        mount = np.eye(4)
    fixed = mount is not None
    initial = []
    seeds = []
    for s in sessions:
        groups = s['groups']
        ext = {i: mean_pose([o['pose'] for o in groups[f'left_ext{i}']]) for i in external_ids}
        xs = [mean_pose([o['pose'] for o in groups[f'ego_ext{i}']]) @ np.linalg.inv(ext[i]) for i in external_ids]
        x = mean_pose(xs)
        seeds.append((x, ext))
    if not fixed:
        mount = mean_pose([np.linalg.inv(x) @ mean_pose([o['pose'] for o in s['groups']['ego_mount']])
                           for s, (x, _) in zip(sessions, seeds)])
        initial.extend(vector(mount))
    for x, ext in seeds:
        initial.extend(vector(x))
        for i in external_ids:
            initial.extend(vector(ext[i]))

    def unpack(v):
        m = mount if fixed else matrix(v[:6])
        offset = 0 if fixed else 6
        poses = []
        for _ in sessions:
            x = matrix(v[offset:offset+6]); offset += 6
            ext = {}
            for i in external_ids:
                ext[i] = matrix(v[offset:offset+6]); offset += 6
            poses.append((x, ext))
        return m, poses

    observations = []
    for index, s in enumerate(sessions):
        keys = [f'{device}_ext{i}' for device in ('ego', 'left') for i in external_ids]
        if use_mount:
            keys.append('ego_mount')
        for key in keys:
            observations.append((index, key, np.array([o['corners'] for o in s['groups'][key]])))

    def residual(v):
        m, poses = unpack(v)
        parts = []
        for index, key, corners in observations:
            x, ext = poses[index]
            device, tag = key.split('_')
            t = m if tag == 'mount' else ext[int(tag[-1])]
            if device == 'ego':
                t = x @ t
            try:
                predicted = project(t, sessions[index]['calibration'][device])
                parts.append((predicted - corners).ravel())
            except ValueError:
                # Keep residual dimension fixed and reject nonphysical trials.
                parts.append(np.full(corners.size, 1e6))
        return np.concatenate(parts)

    solution = least_squares(residual, np.array(initial), loss='soft_l1', f_scale=1.,
                             x_scale='jac', max_nfev=300, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    m, poses = unpack(solution.x)
    metrics = {}
    for index, key, corners in observations:
        x, ext = poses[index]; device, tag = key.split('_')
        t = m if tag == 'mount' else ext[int(tag[-1])]
        if device == 'ego':
            t = x @ t
        errors = project(t, sessions[index]['calibration'][device]) - corners
        metrics[f"{sessions[index]['session']}/{key}"] = pixel_metrics(errors)
    return {'mount': m, 'mount_used': use_mount, 'external_ids': external_ids,
            'poses': poses, 'success': bool(solution.success),
            'nfev': int(solution.nfev), 'message': solution.message, 'groups': metrics}


def pixel_metrics(errors):
    e = np.linalg.norm(errors, axis=-1)
    return {'corner_rmse_px': float(np.sqrt(np.mean(e**2))),
            'corner_p95_px': float(np.percentile(e, 95)),
            'mean_residual_xy_px': np.mean(errors.reshape(-1, 2), axis=0).tolist()}


def score_mount(session, ego_from_umi, mount):
    prediction = project(ego_from_umi @ mount, session['calibration']['ego'])
    observed = np.array([o['corners'] for o in session['groups']['ego_mount']])
    return pixel_metrics(prediction - observed)


def serialize_fit(fit_result, ir_from_color):
    result = {'success': fit_result['success'], 'nfev': fit_result['nfev'],
            'message': fit_result['message'], 'groups': fit_result['groups'],
            'session_poses': [{'ego_from_umi_color': x.tolist(),
                               'umi_color_from_external': {str(i): t.tolist() for i, t in ext.items()}}
                              for x, ext in fit_result['poses']],
            'mount_observations_used': fit_result['mount_used']}
    if fit_result['mount_used']:
        result.update(umi_color_from_mount=fit_result['mount'].tolist(),
                      umi_ir_left_from_mount=mc._xyz_rpy(ir_from_color @ fit_result['mount']))
    return result


def run(sessions, baseline_ir):
    """First two sessions train; final session is the prespecified holdout."""
    ir = np.array(sessions[0]['ir_from_color'])
    if any(not np.allclose(s['ir_from_color'], ir, atol=1e-8) for s in sessions):
        raise ValueError('factory color/IR extrinsics differ across sessions')
    baseline = np.linalg.inv(ir) @ baseline_ir
    training = fit(sessions[:-1])
    held = fit([sessions[-1]], use_mount=False)
    x = held['poses'][0][0]
    result = {'schema': 'ego.mount.corner_joint_diagnostic.v1',
              'training_sessions': [s['session'] for s in sessions[:-1]],
              'holdout_session': sessions[-1]['session'], 'training': serialize_fit(training, ir),
              'holdout_external_fit': serialize_fit(held, ir),
              'holdout_joint': score_mount(sessions[-1], x, training['mount']),
              'holdout_baseline': score_mount(sessions[-1], x, baseline),
              'leave_one_out': [], 'single_tag_ablations': {},
              'activation': 'NOT_ACTIVATED', 'absolute_accuracy': 'NOT_VERIFIED'}
    for i, test in enumerate(sessions):
        train = fit([s for j, s in enumerate(sessions) if i != j])
        nuisance = fit([test], use_mount=False)
        result['leave_one_out'].append({'holdout': test['session'], 'train': serialize_fit(train, ir),
                                        'external_fit_success': nuisance['success'],
                                        'joint': score_mount(test, nuisance['poses'][0][0], train['mount']),
                                        'baseline': score_mount(test, nuisance['poses'][0][0], baseline)})
    for i in (1, 2):
        train = fit(sessions[:-1], external_ids=(i,))
        result['single_tag_ablations'][str(i)] = {'train': serialize_fit(train, ir),
                                                 'holdout_mount': score_mount(sessions[-1], x, train['mount'])}
    result['primary_holdout_improved'] = result['holdout_joint']['corner_rmse_px'] < result['holdout_baseline']['corner_rmse_px']
    result['all_fits_converged'] = (training['success'] and held['success']
        and all(x['train']['success'] and x['external_fit_success'] for x in result['leave_one_out'])
        and all(x['train']['success'] for x in result['single_tag_ablations'].values()))
    result['verdict'] = 'DIAGNOSTIC_ONLY' if result['all_fits_converged'] else 'FIT_FAILED'
    if not result['all_fits_converged']:
        result['primary_holdout_improved'] = None
    result['single_tag_scoring_contract'] = 'Single-tag-trained mount evaluated using same frozen two-tag holdout registration, not an independent single-tag registration accuracy claim.'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions', nargs=3, required=True, type=Path)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if len({p.resolve() for p in args.sessions}) != 3:
        raise ValueError('training and holdout session paths must differ')
    args.output.mkdir(parents=True, exist_ok=False)
    sessions = []
    for path in args.sessions:
        print('EXTRACT:', path.name, flush=True)
        sessions.append(extract(path))
    (args.output/'corners.json').write_text(json.dumps(sessions, indent=2))
    source = json.loads(args.baseline.read_text())['consensus']
    baseline = np.eye(4)
    baseline[:3, :3] = Rotation.from_matrix(np.array(source['rotation_row_major']).reshape(3, 3)).as_matrix()
    baseline[:3, 3] = source['translation_m']
    print('FIT: fixed primary split; held-out mount corners excluded from camera registration', flush=True)
    result = run(sessions, baseline)
    result['baseline_sha256'] = hashlib.sha256(args.baseline.read_bytes()).hexdigest()
    result['configuration'] = {'stride': 20, 'tag_size_m': .04, 'loss': 'soft_l1', 'f_scale_px': 1.,
                                'fixed_intrinsics': True, 'fixed_factory_color_ir': True,
                                'ego_mount_range_max_m_exclusive': .30,
                                'ego_same_id_external_range_min_m_exclusive': .35,
                                'geometry_min_edge_px': 60, 'geometry_max_incidence_deg_exclusive': 45}
    result['configuration']['detector_completion_clock'] = 'synthetic offline max(now, acquisition); not latency evidence'
    (args.output/'report.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k: result[k] for k in ('holdout_joint', 'holdout_baseline', 'primary_holdout_improved')}, indent=2))


if __name__ == '__main__':
    main()
