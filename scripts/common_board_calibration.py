#!/usr/bin/env python3
"""Fixed-factory common-board calibration. Offline only; never activate results."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from scripts.fixed_factory_board_diagnostic import (
    ROOT, TAG_M, GAP_M, digest, object_corners, valid_detections, refine_legacy_corners,
)
from scripts.color_aprilgrid_capture import make_detector, detect_grid
from three_device_slam.spatial.apriltag_detector import (
    normalized_camera_points, project_camera_points, _solve_ippe_candidates,
)
from three_device_slam.spatial.pair_tag_alignment import _camera_calibration_from_intrinsics
from three_device_slam.spatial.se3 import validate_transform

ARTIFACTS_ROOT = (ROOT / 'artifacts').resolve()
ROLES = ('ego', 'left')  # Legacy default; new sessions may bind the UMI as right.
UMI_ROLES = ('left', 'right')
SCHEMA = 'ego.common_board_static_capture.v3'
BOARD_PLAN = (
    ('CENTER', 'train'), ('LEFT', 'train'), ('LEFT + TILT', 'holdout'),
    ('RIGHT', 'train'), ('UP', 'train'), ('UP + TILT', 'holdout'),
    ('DOWN', 'train'), ('TILT LEFT 20deg', 'train'), ('TILT RIGHT 20deg', 'holdout'),
    ('NEAR + TILT UP', 'train'), ('FAR + TILT DOWN', 'train'), ('FAR + SIDE TILT', 'holdout'),
)
TARGET = dict(family='tag36h11', border_bits=2, rows=6, columns=6,
              tag_m=TAG_M, gap_m=GAP_M, layout='legacy_filled_junction_v1')
MOUNT_TARGET = dict(family='tag36h11', border_bits=1, tag_id=1, tag_m=.04)
GATES = dict(min_pairs=3, span_s=1., max_window_s=2., max_drift_px=.75,
             max_arrival_gap_ms=75., min_board_tags=12, min_board_edge_px=15.,
             min_mount_edge_px=60., max_mount_tilt_deg=45., p95_px=1.,
             translation_mm=5., rotation_deg=3., normal_span_deg=15., center_span_m=.08,
             heldout_novel_center_mm=20., heldout_novel_normal_deg=7.,
             board_visibility_policy='anchor_overlap_all_reobservations_v2')
V2_GATES = dict(GATES)
GATES.update(mount_identity_policy='same_id_same_quad_v3', mount_alias_min_iou=.65,
             mount_alias_max_center_edge_ratio=.1)
MOUNT_OBJECTS = .02 * np.array([[-1., 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]])
STEPS = [(name, 'board', split) for name, split in BOARD_PLAN] + [
    ('REMOVE BOARD; SHOW ONLY UMI MOUNT', 'mount', 'mount') for _ in range(3)
] + [('RETURN BOARD; CHECK CAMERAS DID NOT MOVE', 'board', 'holdout')]


def role_pair(mapping):
    """Return the frozen Ego/UMI pair without aliasing right as left."""
    keys = set(mapping)
    umi = [role for role in UMI_ROLES if role in keys]
    if 'ego' not in keys or len(umi) != 1 or keys - {'ego', umi[0]}:
        raise ValueError('exactly ego and one UMI role (left or right) required')
    return 'ego', umi[0]


def new_output(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ARTIFACTS_ROOT) or path.exists():
        raise ValueError('output must be a NEW directory under repository artifacts')
    return path


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def unpack(vector):
    result = np.eye(4)
    result[:3, :3] = cv2.Rodrigues(np.asarray(vector[:3], dtype=float))[0]
    result[:3, 3] = vector[3:6]
    return result


def pack(transform):
    return np.r_[cv2.Rodrigues(transform[:3, :3])[0].ravel(), transform[:3, 3]]


def transform_points(objects, transform):
    return np.asarray(objects) @ transform[:3, :3].T + transform[:3, 3]


def project(objects, transform, calibration):
    points = transform_points(objects, transform)
    if np.any(points[:, 2] <= 0) or not np.isfinite(points).all():
        raise ValueError('nonpositive or nonfinite projection depth')
    return project_camera_points(points, calibration)


def errors(predicted, observed):
    values = np.linalg.norm(np.asarray(predicted) - observed, axis=1)
    return dict(rms_px=float(np.sqrt(np.mean(values**2))), p95_px=float(np.percentile(values, 95)),
                max_px=float(values.max()), corners=len(values))


def difference(a, b):
    delta = np.linalg.inv(a) @ b
    return float(np.linalg.norm(a[:3, 3]-b[:3, 3])*1000), float(np.linalg.norm(cv2.Rodrigues(delta[:3, :3])[0])*180/np.pi)


def fit_pose(objects, pixels, calibration):
    rays = normalized_camera_points(pixels, calibration)
    # Detect incompatible SDK model adaptation before attempting optimization.
    roundtrip = project_camera_points(np.c_[rays, np.ones(len(rays))], calibration)
    if np.max(np.linalg.norm(roundtrip-pixels, axis=1)) > .05:
        raise ValueError('factory ray roundtrip exceeds 0.05px')
    seeds = cv2.solvePnPGeneric(objects, rays, np.eye(3), np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for r, t in zip(seeds[1], seeds[2]):
        r, t = cv2.solvePnPRefineLM(objects, rays, np.eye(3), np.zeros(5), r.copy(), t.copy())
        pose = unpack(np.r_[r.ravel(), t.ravel()])
        if np.min(transform_points(objects, pose)[:, 2]) > 0:
            candidates.append((errors(project(objects, pose, calibration), pixels)['rms_px'], pose))
    if not candidates:
        raise ValueError('no positive board pose')
    return min(candidates, key=lambda x: x[0])[1]


def solve_board(windows, calibrations):
    roles = role_pair(calibrations)
    umi_role = roles[1]
    slots = [w['slot'] for w in windows]
    if slots not in (list(range(len(BOARD_PLAN))), list(range(len(BOARD_PLAN))) + [15]):
        raise ValueError('board slots must be exactly the 12 frozen whole windows')
    for w in windows:
        split = STEPS[w['slot']][2]
        if w['split'] != split:
            raise ValueError('frozen window split mismatch')
        obj = np.asarray(w['objects'])
        if obj.ndim != 2 or obj.shape[1] != 3 or len(obj) < 48 or not np.isfinite(obj).all():
            raise ValueError('objects must be finite and cover >=12 tags')
        for role in roles:
            pix = np.asarray(w['pixels'][role])
            if pix.shape != (len(obj), 2) or not np.isfinite(pix).all():
                raise ValueError('pixels must be finite and match objects')
    train = [w for w in windows if w['split'] == 'train']
    initial = [{r: fit_pose(w['objects'], w['pixels'][r], calibrations[r]) for r in roles} for w in train]
    xs = [p['ego'] @ np.linalg.inv(p[umi_role]) for p in initial]
    # Medoid initializer only; the final shared X is fitted from all training corners.
    x0 = min(xs, key=lambda a: sum(sum(difference(a, b)) for b in xs))
    start = np.concatenate([pack(x0)] + [pack(p[umi_role]) for p in initial])
    rays = [{r: normalized_camera_points(w['pixels'][r], calibrations[r]) for r in roles} for w in train]
    scales = {r: np.diag(calibrations[r].camera_matrix)[:2] for r in roles}

    def residual(vector):
        x = unpack(vector[:6])
        values = []
        for j, w in enumerate(train):
            b = unpack(vector[6+6*j:12+6*j])
            for role, pose in (('ego', x @ b), (umi_role, b)):
                q = transform_points(w['objects'], pose)
                values.extend(((q[:, :2]/np.maximum(q[:, 2:], 1e-6)-rays[j][role])*scales[role]).ravel())
        return np.asarray(values)

    # Float64 normalized SDK rays avoid finite-difference derivatives through
    # librealsense's float32 projection. Pixel-space scores below use the SDK.
    sparsity = lil_matrix((sum(4*len(w['objects']) for w in train), len(start)), dtype=int)
    row = 0
    for j, w in enumerate(train):
        count = 2*len(w['objects'])
        sparsity[row:row+count, :6] = 1
        sparsity[row:row+2*count, 6+6*j:12+6*j] = 1
        row += 2*count
    result = least_squares(residual, start, jac_sparsity=sparsity.tocsr(), x_scale='jac',
                           loss='linear', max_nfev=300, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    x = unpack(result.x[:6])
    poses = [unpack(result.x[6+6*j:12+6*j]) for j in range(len(train))]
    training = [dict(slot=w['slot'], role=r, **errors(project(w['objects'], x @ b if r=='ego' else b, calibrations[r]), w['pixels'][r]))
                for w, b in zip(train, poses) for r in roles]
    heldout, novelty, earlier_heldout = [], [], []
    for w in windows:
        if w['split'] != 'holdout':
            continue
        for source, dest, link in ((umi_role, 'ego', x), ('ego', umi_role, np.linalg.inv(x))):
            b = fit_pose(w['objects'], w['pixels'][source], calibrations[source])
            if source == umi_role and w['slot'] != 15:
                # Source-camera poses only: never use Ego held-out residuals
                # to select which images get counted as independent validation.
                reference_poses = [(tw['slot'], p[umi_role]) for tw, p in zip(train, initial)] + earlier_heldout
                comparisons = [dict(reference_slot=slot, translation_mm=float(np.linalg.norm(b[:3,3]-prior[:3,3])*1000),
                                    normal_deg=float(np.degrees(np.arccos(np.clip(b[:3,2] @ prior[:3,2], -1, 1)))))
                               for slot, prior in reference_poses]
                novel = all(c['translation_mm'] >= GATES['heldout_novel_center_mm'] or
                            c['normal_deg'] >= GATES['heldout_novel_normal_deg'] for c in comparisons)
                novelty.append(dict(slot=w['slot'], novel=novel, comparisons=comparisons))
                earlier_heldout.append((w['slot'], b))
            predicted = project(w['objects'], link @ b, calibrations[dest])
            heldout.append(dict(slot=w['slot'], source=source, destination=dest,
                                **errors(predicted, w['pixels'][dest]),
                                residual_px=(predicted-w['pixels'][dest]).tolist()))
    normal_span = max(float(np.degrees(np.arccos(np.clip(a[:3, 2] @ b[:3, 2], -1, 1)))) for a in poses for b in poses)
    center_span = max(float(np.linalg.norm(a[:3, 3]-b[:3, 3])) for a in poses for b in poses)
    spread = [dict(slot=w['slot'], translation_mm=difference(x, xi)[0], rotation_deg=difference(x, xi)[1]) for w, xi in zip(train, xs)]
    checks = dict(optimizer_converged=bool(result.success),
                  training_residual=all(v['p95_px'] <= GATES['p95_px'] for v in training),
                  bidirectional_whole_window_holdout=all(v['p95_px'] <= GATES['p95_px'] for v in heldout),
                  holdout_pose_novelty=all(v['novel'] for v in novelty),
                  board_diversity=normal_span >= GATES['normal_span_deg'] and center_span >= GATES['center_span_m'],
                  fixed_camera_consistency=all(v['translation_mm'] <= GATES['translation_mm'] and v['rotation_deg'] <= GATES['rotation_deg'] for v in spread))
    return dict(status='PASS_BOARD_ONLY' if all(checks.values()) else 'REVIEW', checks=checks,
                umi_role=umi_role, T_Ecolor_Ucolor=x.tolist(), training=training, holdout=heldout, holdout_novelty=novelty, camera_consistency=spread,
                board_normal_span_deg=normal_span, board_center_span_m=center_span,
                optimizer=dict(success=bool(result.success), message=result.message, nfev=result.nfev),
                fitted_parameters='X and training board poses only; fixed K/D/layout/size',
                objective='linear normalized factory rays scaled by fx/fy; no outlier deletion')


def solve_mount(pixels, calibration, x, reference=None):
    x = np.asarray(x, dtype=float)
    validate_transform(x)
    values = np.asarray(pixels, dtype=float)
    if values.shape != (3, 4, 2) or not np.isfinite(values).all():
        raise ValueError('exactly three finite mount windows required')
    candidates = sorted((c for c in _solve_ippe_candidates(values.mean(axis=0), calibration=calibration, tag_size_m=.04)
                         if c.positive_depth), key=lambda c: c.reprojection_error_px)
    if not candidates:
        raise ValueError('no positive mount pose')
    pose = candidates[0].camera_from_tag
    ambiguous = False
    if len(candidates) > 1:
        td, rd = difference(pose, candidates[1].camera_from_tag)
        ambiguous = candidates[1].reprojection_error_px-candidates[0].reprojection_error_px < .05 and (td > 5 or rd > 3)
    mount = np.linalg.inv(x) @ pose
    tilt = float(np.degrees(np.arccos(np.clip(abs(pose[2, 2]), 0, 1))))
    fits = [errors(project(MOUNT_OBJECTS, pose, calibration), p) for p in values]
    checks = dict(unambiguous=not ambiguous, tilt=tilt <= GATES['max_mount_tilt_deg'],
                  all_window_fit=all(e['p95_px'] <= GATES['p95_px'] for e in fits))
    out = dict(T_Ucolor_mount=mount.tolist(), mount_tilt_deg=tilt, checks=checks, training_windows=fits,
               seed_errors_px=[c.reprojection_error_px for c in candidates],
               note='Mount observations never fitted X. No absolute metrology or activation.')
    if reference is not None:
        reference = np.asarray(reference, dtype=float)
        validate_transform(reference)
        # The reference M, not the local refitted M above, predicts these corners.
        predictions = [errors(project(MOUNT_OBJECTS, x @ reference, calibration), p) for p in values]
        td, rd = difference(reference, mount)
        checks.update(independent_fixed_M=all(e['p95_px'] <= GATES['p95_px'] for e in predictions),
                      full3d_repeatability=td <= GATES['translation_mm'] and rd <= GATES['rotation_deg'])
        out.update(independent_prediction=predictions, repeatability_translation_mm=td,
                   repeatability_rotation_deg=rd, fixed_reference_M=reference.tolist())
    out['status'] = ('PASS_INDEPENDENT_MOUNT' if reference is not None else 'CANDIDATE_NEEDS_INDEPENDENT_SETUP') if all(checks.values()) else 'REVIEW'
    return out


def mount_detector():
    p = cv2.aruco.DetectorParameters()
    p.markerBorderBits = 1
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)


def same_mount_quad(board_detection, mount_detection):
    """Same image instance, not a global ID exemption (external ID1 exists)."""
    if board_detection.tag_id != 1 or mount_detection.tag_id != 1:
        return False
    a,b = (np.asarray(d.corners,dtype=np.float32).reshape(4,2) for d in (board_detection,mount_detection))
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        return False
    area_a,area_b = abs(cv2.contourArea(a)),abs(cv2.contourArea(b))
    intersection = cv2.intersectConvexConvex(a,b)[0]
    union = area_a+area_b-intersection
    edge = float(np.linalg.norm(b-np.roll(b,1,axis=0),axis=1).min())
    return union > 0 and edge > 0 and intersection/union >= GATES['mount_alias_min_iou'] and \
        np.linalg.norm(a.mean(axis=0)-b.mean(axis=0))/edge <= GATES['mount_alias_max_center_edge_ratio']


def detect_sample(images, kind, board_detector, tag_detector, *, legacy_mount_check=False, display=None, simultaneous=False):
    if simultaneous:
        from scripts.common_board_joint import detect_joint_sample
        return detect_joint_sample(images,kind,board_detector,tag_detector,display=display)
    roles = role_pair(images)
    umi_role = roles[1]
    ds, reasons = {}, []
    tags = detect_grid(tag_detector, images['ego']) if kind == 'mount' else []
    for role in roles:
        raw = detect_grid(board_detector, images[role])
        if display is not None:
            ids = [int(d.tag_id) for d in raw if 0 <= int(d.tag_id) < 36]
            display[role] = dict(raw=raw, duplicate_ids=sorted(i for i in set(ids) if ids.count(i)>1))
        if kind == 'board':
            try:
                ds[role] = refine_legacy_corners(images[role], valid_detections(raw))
            except ValueError:
                ds[role] = {}
                reasons.append(role+':duplicate_board_id')
            ids = list(ds[role])
            if len(ids) < 12 or len({i//6 for i in ids}) < 3 or len({i%6 for i in ids}) < 3:
                reasons.append(role+':need_12_tags_3rows_3cols')
        else:
            others = raw
            if not legacy_mount_check and role == 'ego' and len(tags) == 1 and tags[0].tag_id == 1:
                others = [d for d in raw if not same_mount_quad(d,tags[0])]
            if others:
                reasons.append(role+':remove_board_for_mount_stage')
            ds[role] = {}
    if kind == 'mount':
        if len(tags) != 1 or tags[0].tag_id != 1:
            reasons.append('ego:show_only_mount_id1_hide_external_tags')
        else:
            quad = np.asarray(tags[0].corners)
            ds['ego'][1] = quad
            if not np.isfinite(quad).all() or np.linalg.norm(quad-np.roll(quad, 1, axis=0), axis=1).min() < 60:
                reasons.append('ego:mount_edge_below_60px')
    else:
        common = set(ds['ego']) & set(ds[umi_role])
        if len(common) < 12 or len({i//6 for i in common}) < 3 or len({i%6 for i in common}) < 3:
            reasons.append('need_12_common_tags_3rows_3cols')
    return ds, reasons


def board_coverage(ids):
    return len(ids) >= GATES['min_board_tags'] and len({i//6 for i in ids}) >= 3 and len({i%6 for i in ids}) >= 3


def window_quality(samples, kind, *, simultaneous=False):
    """Visibility is not motion. Check all reobserved tags; never trim by drift.

    Every board image needs spatially spread overlap with the FIRST image,
    preventing sliding/chained tracks from hiding cumulative motion. Intermittent
    IDs get their own first-observation anchor, even if absent in image zero.
    """
    if simultaneous:
        from scripts.common_board_joint import joint_window_quality
        return joint_window_quality(samples)
    reasons = sorted({v for s in samples for v in s['reasons']})
    if len(samples) < GATES['min_pairs'] or samples[-1]['t']-samples[0]['t'] < GATES['span_s']:
        reasons.append('need_3_pairs_spanning_1s')
    if samples and samples[-1]['t']-samples[0]['t'] > GATES['max_window_s']:
        reasons.append('window_too_long')
    if any(not np.isfinite(s['arrival_gap_ms']) or s['arrival_gap_ms'] > GATES['max_arrival_gap_ms'] for s in samples):
        reasons.append('arrival_gap')
    roles = role_pair(samples[0]['detected']) if samples else ROLES
    umi_role = roles[1]
    metrics, repeated = {}, {}
    for role in roles if kind == 'board' else ('ego',):
        if not samples:
            continue
        first = samples[0]['detected'][role]
        if not first:
            reasons.append(role+':no_target')
        anchors, counts, overlaps, drift = {}, {}, [], 0.
        for j,s in enumerate(samples):
            current = s['detected'][role]
            common = set(current) & set(first)
            if j:
                overlaps.append(len(common))
            if kind == 'board' and not board_coverage(common):
                reasons.append(role+':anchor_coverage')
            elif kind != 'board' and (set(current) != set(first) or set(current) != {1}):
                reasons.append(role+':visibility_changed')
            for tag, quad in current.items():
                quad = np.asarray(quad)
                if quad.shape != (4,2) or not np.isfinite(quad).all():
                    reasons.append(role+':invalid_corners')
                    continue
                counts[tag] = counts.get(tag,0)+1
                if tag in anchors:
                    value = float(np.max(np.linalg.norm(quad-anchors[tag], axis=1)))
                    drift = max(drift,value)
                    if value > GATES['max_drift_px']:
                        reasons.append('window_motion')
                else:
                    anchors[tag] = quad
        repeated[role] = {tag for tag,n in counts.items() if n>=2}
        metrics[role] = dict(min_anchor_tags=min(overlaps) if overlaps else len(first),
                             max_drift_px=drift, frame_tag_counts=[len(s['detected'][role]) for s in samples],
                             anchor_ids=sorted(first), reobserved_ids=sorted(repeated[role]))
    selected_ids = []
    if kind == 'board' and samples:
        middle = samples[len(samples)//2]['detected']
        selected_ids = sorted(set(middle['ego']) & set(middle[umi_role]) & repeated.get('ego',set()) & repeated.get(umi_role,set()))
        if not board_coverage(selected_ids):
            reasons.append('selected_pair_tracked_coverage')
    return dict(reasons=sorted(set(reasons)), roles=metrics, selected_common_ids=selected_ids,
                pairs=len(samples), span_s=samples[-1]['t']-samples[0]['t'] if samples else 0.)


def window_reasons(samples, kind):
    return window_quality(samples,kind)['reasons']


def load_session(path, *, allow_partial=False, _ancestors=()):
    path = Path(path).resolve()
    if path in _ancestors or len(_ancestors) >= 4:
        raise ValueError('invalid continuation cycle/depth')
    setup = json.loads((path/'setup.json').read_text())
    report = json.loads((path/'capture_report.json').read_text())
    legacy = setup['schema'] == 'ego.common_board_static_capture.v2'
    mode=setup.get('observation_mode','sequential')
    if mode not in ('sequential','simultaneous'):
        raise ValueError('unknown observation mode')
    simultaneous=mode=='simultaneous'
    if simultaneous:
        from scripts.common_board_joint import POLICY,STEPS as JOINT_STEPS
        if legacy or setup.get('identity_policy')!=POLICY or setup.get('plan')!=[list(s) for s in JOINT_STEPS]:
            raise ValueError('simultaneous identity/plan mismatch')
    if setup.get('placement_check_only'):
        raise ValueError('placement check is not a calibration dataset')
    if setup['schema'] not in (SCHEMA,'ego.common_board_static_capture.v2') or setup['target'] != TARGET or setup['mount_target'] != MOUNT_TARGET or setup['gates'] != (V2_GATES if legacy else GATES):
        raise ValueError('unsupported paired capture/target/gates; monocular clips cannot be paired')
    if setup['factory_values_modified'] or setup['activation'] != 'NOT_ACTIVATED':
        raise ValueError('factory/activation violation')
    hashes = {f: digest(path/f) for f in ('setup.json', 'attempts.jsonl', 'capture_report.json')}
    if report['cleanup_errors'] or (not allow_partial and (not report['completed'] or report['failure'])) or \
            (allow_partial and report['failure'] not in (None,'operator_interrupt')):
        raise ValueError('capture incomplete; preserve and recapture')
    if report['setup_sha256'] != hashes['setup.json'] or report['attempts_sha256'] != hashes['attempts.jsonl']:
        raise ValueError('session binding mismatch')
    roles = role_pair(setup['devices'])
    calibrations = {r: _camera_calibration_from_intrinsics(calibration_id=hashes['setup.json'], frame_id=r+'.color',
                    intrinsics=setup['devices'][r]['intrinsics']) for r in roles}
    rows = [json.loads(line) for line in (path/'attempts.jsonl').read_text().splitlines()]
    if len(rows) != report['attempts']:
        raise ValueError('attempt count mismatch')
    from scripts.common_board_detector import make_board_detector
    board_detector, tag_detector = make_board_detector(setup.get('board_detector','opencv')), mount_detector()
    accepted, mount, audit = [], [], []
    slot = 0
    continuation = setup.get('resume_from')
    if continuation:
        if legacy or continuation.get('operator_confirmed_unchanged') is not True:
            raise ValueError('continuation requires unchanged camera/mount confirmation')
        base, _, accepted, mount, base_hashes, audit = load_session(continuation['session'],allow_partial=True,
                                                                  _ancestors=(*_ancestors,path))
        slot = len(accepted)+len(mount)
        if not 12 <= slot < 16 or slot != continuation['next_slot'] or base_hashes != continuation['source_hashes']:
            raise ValueError('continuation source/step changed')
        if setup['devices'] != base['devices']:
            raise ValueError('continuation factory/device mismatch')
        if setup.get('board_detector','opencv') != base.get('board_detector','opencv'):
            raise ValueError('continuation board detector mismatch')
        if mode != base.get('observation_mode','sequential'):
            raise ValueError('continuation observation mode mismatch')
    previous = {}
    for attempt_id, row in enumerate(rows):
        kind = STEPS[slot][1] if slot < len(STEPS) else None
        if slot >= len(STEPS) or row['attempt'] != attempt_id or row['slot'] != slot or row['kind'] != kind:
            raise ValueError('attempt order or frozen stage mismatch')
        samples = []
        for j, pair in enumerate(row['pairs']):
            images = {}
            arrivals = []
            for r in roles:
                meta = pair[r]
                expected = f'attempt_{attempt_id:03d}/{r}_{j:03d}.png'
                if meta['file'] != expected or digest(path/expected) != meta['sha256']:
                    raise ValueError('raw image hash/path mismatch')
                image = cv2.imread(str(path/expected), 0)
                c = calibrations[r]
                if image is None or image.shape != (c.height, c.width):
                    raise ValueError('image shape mismatch')
                for key in ('frame_number', 'sdk_timestamp_ms', 'host_arrival_monotonic_ns'):
                    value = meta[key]
                    if not np.isfinite(value) or value <= previous.get((r, key), -1):
                        raise ValueError('timestamp/counter regression')
                    previous[r, key] = value
                if (r, 'domain') in previous and previous[r, 'domain'] != meta['sdk_timestamp_domain']:
                    raise ValueError('timestamp domain changed')
                previous[r, 'domain'] = meta['sdk_timestamp_domain']
                arrivals.append(meta['host_arrival_monotonic_ns'])
                images[r] = image
            if legacy:
                ds, reasons = detect_sample(images, kind, board_detector, tag_detector, legacy_mount_check=True)
            elif simultaneous:
                ds, reasons = detect_sample(images, kind, board_detector, tag_detector, simultaneous=True)
            else:
                ds, reasons = detect_sample(images, kind, board_detector, tag_detector)
            samples.append(dict(t=max(arrivals)/1e9, arrival_gap_ms=abs(arrivals[0]-arrivals[1])/1e6,
                                detected=ds, reasons=reasons))
        quality = window_quality(samples, kind, simultaneous=True) if simultaneous else window_quality(samples, kind)
        reasons = quality['reasons']
        terminal_abort = (not row['accepted'] and set(row['reasons'])-set(reasons) <= {'operator_aborted','interrupted'}
                          and bool(set(row['reasons'])-set(reasons)))
        if row['accepted'] != (not reasons) and not terminal_abort:
            raise ValueError('offline/static gate differs from capture; do not silently reselect')
        if row.get('window_quality') != quality:
            raise ValueError('window quality/selected IDs changed on replay')
        audit.append(dict(source_session=str(path),attempt=attempt_id, slot=slot, accepted=row['accepted'],
                          reasons=row['reasons'], raw_pairs=len(samples), window_quality=quality))
        if not row['accepted']:
            continue
        selected = samples[len(samples)//2]['detected']
        if simultaneous:
            audit[-1]['same_frame_mount_corners']=selected['_mount']['ego'][1].tolist()
        if kind == 'board':
            ids = quality['selected_common_ids']
            accepted.append(dict(slot=slot, split=STEPS[slot][2],
                                 objects=np.concatenate([object_corners(i) for i in ids]),
                                 pixels={r: np.concatenate([selected[r][i] for i in ids]) for r in roles}))
        else:
            mount.append(selected['ego'][1])
        slot += 1
    if slot != report['completed_steps']:
        raise ValueError('completed step count mismatch')
    if not allow_partial and (len(accepted) != 13 or len(mount) != 3):
        raise ValueError('missing complete board/mount windows')
    return setup, calibrations, accepted, mount, hashes, audit


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, help='First independent setup report; freeze its M')
    args = parser.parse_args(argv)
    output = new_output(args.output)
    setup, cal, windows, pixels, hashes, audit = load_session(args.session)
    umi_role = role_pair(setup['devices'])[1]
    reference, binding = None, None
    if args.reference:
        ref = json.loads(args.reference.read_text())
        if (ref['schema'] != 'ego.common_board_calibration.v1' or ref['status'] != 'CANDIDATE_NEEDS_INDEPENDENT_SETUP'
                or ref.get('gates') != GATES or ref.get('factory_values_modified') is not False
                or ref.get('activation') != 'NOT_ACTIVATED' or ref['board']['status'] != 'PASS_BOARD_ONLY'
                or ref['mount']['status'] != 'CANDIDATE_NEEDS_INDEPENDENT_SETUP'):
            raise ValueError('reference must be a complete passing candidate, not REVIEW')
        if ref['devices'] != setup['devices'] or ref['target'] != TARGET or ref['mount_target'] != MOUNT_TARGET:
            raise ValueError('reference device/model/target mismatch')
        if ref.get('observation_mode','sequential') != setup.get('observation_mode','sequential') or ref.get('identity_policy') != setup.get('identity_policy'):
            raise ValueError('reference observation/identity policy mismatch')
        if ref['source_hashes']['setup.json'] == hashes['setup.json']:
            raise ValueError('same setup cannot be its own independent validation')
        reference = np.asarray(ref['mount']['T_Ucolor_mount'])
        binding = dict(path=str(args.reference.resolve()), sha256=digest(args.reference))
    print('SOLVE: only 8 training board windows fit X; 4 whole windows held out.', flush=True)
    board = solve_board(windows, cal)
    mount = solve_mount(pixels, cal['ego'], np.asarray(board['T_Ecolor_Ucolor']), reference)
    independent_geometry = True
    if args.reference:
        td, rd = difference(np.asarray(ref['board']['T_Ecolor_Ucolor']), np.asarray(board['T_Ecolor_Ucolor']))
        independent_geometry = td >= 50 or rd >= 10
        mount['independent_camera_change'] = dict(translation_mm=td, rotation_deg=rd, passed=independent_geometry)
    status = mount['status'] if board['status'] == 'PASS_BOARD_ONLY' and independent_geometry else 'REVIEW'
    same_frame=None
    if setup.get('observation_mode')=='simultaneous':
        from scripts.common_board_joint import same_frame_check
        fixed=reference if reference is not None else np.asarray(mount['T_Ucolor_mount'])
        same_frame=[same_frame_check(w,next(a['same_frame_mount_corners'] for a in audit if a['accepted'] and a['slot']==w['slot']),cal,fixed) for w in windows]
        if not all(v['passed'] for v in same_frame):status='REVIEW'
    result = dict(schema='ego.common_board_calibration.v1', status=status, activation='NOT_ACTIVATED',
                  factory_values_modified=False, devices=setup['devices'], umi_role=umi_role,
                  target=TARGET, mount_target=MOUNT_TARGET,
                  board_detector=setup.get('board_detector','opencv'),
                  observation_mode=setup.get('observation_mode','sequential'),
                  identity_policy=setup.get('identity_policy'),
                  gates=GATES, source_session=str(args.session.resolve()), source_hashes=hashes,
                  reference=binding, attempts=audit, board=board, mount=mount,
                  same_frame_mount_checks=same_frame,
                  code_sha256={str(p.relative_to(ROOT)): digest(p) for p in (Path(__file__).resolve(),
                      ROOT/'scripts/fixed_factory_board_diagnostic.py', ROOT/'scripts/color_aprilgrid_capture.py',
                      ROOT/'scripts/common_board_detector.py',
                      ROOT/'scripts/common_board_joint.py',
                      ROOT/'three_device_slam/spatial/apriltag_detector.py', ROOT/'scripts/common_board_capture.py')},
                  limitations=['static host-arrival pairing, not exposure synchronization or calibrated td',
                               'window samples correlated; repeatability is not absolute metrology',
                               'temporary X invalid after camera movement; M not activated',
                               'color optical frames only; audited color-to-VIO/body conversion still required'])
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'report.json', result)
    print(f'{status}: {output / "report.json"}', flush=True)
    print('NEXT: independent Ego placement and new paired capture.' if status == 'CANDIDATE_NEEDS_INDEPENDENT_SETUP'
          else 'NEXT: review report; no automatic activation or runtime change.', flush=True)
    return 0 if status != 'REVIEW' else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError, KeyError, cv2.error) as exc:
        print(f'BLOCKED: {type(exc).__name__}: {exc}; original session preserved; no activation.', flush=True)
        raise SystemExit(2)
