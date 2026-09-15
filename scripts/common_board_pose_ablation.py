#!/usr/bin/env python3
"""Fixed-input regional pose diagnosis; no calibration activation or pruning."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_edge_diagnostic import edge_corners
from scripts.common_board_joint import board_geometry
from scripts.common_board_resampling_analysis import groups, restore_board_id1
from scripts.common_board_static_analysis import pose_options, mount_summary
from scripts.fixed_factory_board_diagnostic import refine_legacy_corners_scaled


def validate_reference_frames(reference, rows):
    expected = [dict(attempt=row['attempt'], slot=row['slot'], pair=i, source=pair)
                for row in rows if row['accepted'] for i, pair in enumerate(row['pairs'])]
    actual = [{k: f[k] for k in ('attempt', 'slot', 'pair', 'source')} for f in reference['frames']]
    if actual != expected or not expected:
        raise ValueError('reference must contain every accepted raw pair in original order')


def observed_ego_mount(pixels, calibration, reference_whole):
    candidates = sorted((c for c in cb._solve_ippe_candidates(pixels, calibration=calibration,
                         tag_size_m=.04) if c.positive_depth), key=lambda c: c.reprojection_error_px)
    if not candidates:
        raise ValueError('no positive observed mount pose')
    observed = candidates[0].camera_from_tag
    reference = np.asarray(reference_whole['T_Ecolor_Ucolor']) @ np.asarray(reference_whole['mount']['T_Ucolor_mount'])
    np.testing.assert_allclose(observed, reference, atol=1e-10, rtol=0,
                               err_msg='reference mount must match current raw detection')
    return observed


def pixel_pose(objects, pixels, calibration):
    """Refine both existing seeds in actual SDK pixels, with explicit central Jacobian.

    SDK projection is float32; scipy's default tiny finite differences are invalid.
    Retain seeds as candidates so optimization cannot silently worsen own pixel SSE.
    """
    options = pose_options(objects, pixels, calibration)
    candidates = [o['pose'] for o in options]
    def residual(v):
        return (cb.project(objects, cb.unpack(v), calibration)-pixels).ravel()
    step = np.array([1e-5]*3+[1e-6]*3)
    def jac(v):
        return np.column_stack([(residual(v+np.eye(6)[i]*step[i])-
                                 residual(v-np.eye(6)[i]*step[i]))/(2*step[i]) for i in range(6)])
    for o in options:
        result = least_squares(residual, cb.pack(o['pose']), jac=jac, max_nfev=80,
                               ftol=1e-9, xtol=1e-9, gtol=1e-7)
        pose = cb.unpack(result.x)
        if np.min(cb.transform_points(objects, pose)[:, 2]) > 0:
            candidates.append(pose)
    return min(candidates, key=lambda t: cb.errors(cb.project(objects, t, calibration), pixels)['rms_px'])


def analyze(pixels, objects, cal, ego_mount, solver):
    frozen = {k: v for k, v in groups().items() if k in (
        'all36', 'horizontal_0', 'horizontal_1', 'vertical_0', 'vertical_1', 'checker_0', 'checker_1')}
    results = {}
    for name, ids in frozen.items():
        mask = np.repeat([i in ids for i in range(36)], 4)
        poses = {r: solver(objects[mask], pixels[r][mask], cal[r]) for r in cb.ROLES}
        x = poses['ego'] @ np.linalg.inv(poses['left'])
        results[name] = dict(mount=mount_summary(np.linalg.inv(x) @ ego_mount),
            camera_board={r: poses[r].tolist() for r in cb.ROLES},
            fit={r: cb.errors(cb.project(objects[mask], poses[r], cal[r]), pixels[r][mask]) for r in cb.ROLES},
            heldout={r: cb.errors(cb.project(objects[~mask], poses[r], cal[r]), pixels[r][~mask]) for r in cb.ROLES}
                if np.any(~mask) else None)
    contrasts = {}
    for name in ('horizontal', 'vertical', 'checker'):
        a, b = (results[name+'_'+str(i)] for i in (0, 1))
        d, rot = cb.difference(np.asarray(a['mount']['T_Ucolor_mount']), np.asarray(b['mount']['T_Ucolor_mount']))
        contrasts[name] = dict(translation_mm=d, rotation_deg=rot,
            camera_rotation_deg={r: cb.difference(np.asarray(a['camera_board'][r]),
                                                  np.asarray(b['camera_board'][r]))[1] for r in cb.ROLES})
    return dict(groups=results, contrasts=contrasts)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    out = cb.new_output(args.output)
    setup, cal, _, _, hashes, _ = cb.load_session(args.session, allow_partial=True)
    ref = json.loads(args.reference.read_text())
    if ref['source_hashes'] != hashes or setup.get('observation_mode') != 'simultaneous':
        raise ValueError('source mismatch')
    rows = [json.loads(line) for line in (args.session/'attempts.jsonl').read_text().splitlines()]
    validate_reference_frames(ref, rows)
    objects = np.concatenate([cb.object_corners(i) for i in range(36)])
    detector = make_board_detector(setup['board_detector'])
    mount_detector = cb.mount_detector()
    ray_solver = lambda obj, pix, c: pose_options(obj, pix, c)[0]['pose']
    output = []
    for f in ref['frames']:
        images = {}
        for r in cb.ROLES:
            source = f['source'][r]
            path = args.session/source['file']
            if cb.digest(path) != source['sha256']:
                raise ValueError('raw hash mismatch')
            images[r] = cv2.imread(str(path), 0)
        display = {}
        detected, reasons = cb.detect_sample(images, 'board', detector, mount_detector,
                                             display=display, simultaneous=True)
        if reasons:
            raise ValueError(str(reasons))
        variants = {name: {} for name in ('fixed5', 'native', 'scaled', 'edge')}
        for r in cb.ROLES:
            raw, _, _ = board_geometry(display[r]['raw'])
            raw[1], _ = restore_board_id1(display[r]['raw'])
            if set(raw) != set(range(36)):
                raise ValueError('must retain all36, no tag pruning')
            refined = cb.refine_legacy_corners(images[r], raw)
            variants['fixed5'][r] = np.concatenate([refined[i] for i in range(36)])
            variants['native'][r] = np.concatenate([raw[i] for i in range(36)])
            scaled = refine_legacy_corners_scaled(images[r], raw)
            variants['scaled'][r] = np.concatenate([scaled[i] for i in range(36)])
            variants['edge'][r] = np.concatenate([edge_corners(images[r], refined[i]) for i in range(36)])
        whole = f['static']['whole_board']
        ego_mount = observed_ego_mount(detected['_mount']['ego'][1], cal['ego'], whole)
        results = {name: analyze(p, objects, cal, ego_mount, ray_solver) for name, p in variants.items()}
        np.testing.assert_allclose(results['fixed5']['groups']['all36']['mount']['T_Ucolor_mount'],
                                   whole['mount']['T_Ucolor_mount'], atol=1e-10, rtol=0)
        results['pixel_objective'] = analyze(variants['fixed5'], objects, cal, ego_mount, pixel_pose)
        ideal = {r: cb.project(objects, np.array(results['fixed5']['groups']['all36']['camera_board'][r]), cal[r])
                 for r in cb.ROLES}
        results['ideal_both'] = analyze(ideal, objects, cal, ego_mount, ray_solver)
        for r in cb.ROLES:
            # Counterfactual diagnostic only: ideal pixels from whole-board fit.
            mixed = dict(variants['fixed5'], **{r: ideal[r]})
            results['ideal_'+r] = analyze(mixed, objects, cal, ego_mount, ray_solver)
        output.append(dict(slot=f['slot'], pair=f['pair'], results=results,
            corners={k: {r: v[r].tolist() for r in cb.ROLES} for k, v in variants.items()}))
        print('FRAME', len(output), {k: round(v['contrasts']['horizontal']['translation_mm'], 3)
                                    for k, v in results.items()}, flush=True)
    summary = {name: {contrast: dict(
        median_mm=float(np.median([f['results'][name]['contrasts'][contrast]['translation_mm'] for f in output])),
        max_mm=float(max(f['results'][name]['contrasts'][contrast]['translation_mm'] for f in output)))
        for contrast in ('horizontal', 'vertical', 'checker')} for name in output[0]['results']}
    out.mkdir()
    cb.write_json(out/'report.json', dict(status='DIAGNOSTIC_NOT_CALIBRATION', activation='NOT_ACTIVATED',
        source_hashes=hashes, reference_sha256=cb.digest(args.reference), frames=output, summary=summary,
        limitations=['all12 existing frames reused; not independent acceptance',
            'ideal cases are counterfactual localization, never deployable corrections',
            'all36 IDs and factoryKD retained, no target or mount fitting',
            'native/scaled/edge are frozen algorithm controls, not residual-chosen per tag'],
        code_sha256={str(p.relative_to(cb.ROOT)): cb.digest(p) for p in (
            Path(__file__).resolve(), cb.ROOT/'scripts/common_board_static_analysis.py',
            cb.ROOT/'scripts/common_board_calibration.py', cb.ROOT/'scripts/common_board_edge_diagnostic.py',
            cb.ROOT/'scripts/fixed_factory_board_diagnostic.py', cb.ROOT/'scripts/common_board_detector.py',
            cb.ROOT/'scripts/common_board_resampling_analysis.py', cb.ROOT/'scripts/common_board_joint.py',
            cb.ROOT/'three_device_slam/spatial/apriltag_detector.py')}))
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
