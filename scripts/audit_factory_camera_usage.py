"""Offline fixed-factory audit; never fits or activates intrinsics/distortion.

Compare the current OpenCV ray model against the installed RealSense SDK,
using every accepted tag corner in two sealed bench sessions.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from three_device_slam.spatial import mixed_size_crosscheck as mixed
from three_device_slam.spatial import pair_tag_alignment as pta
from three_device_slam.spatial.apriltag_detector import _solve_ippe_candidates
from three_device_slam.devices.realsense_extrinsics import load_color_to_ir_snapshot

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('mixed80_crosscheck_20260905T215930', 'mixed80_newview_20260905T225115')


def sdk_intrinsics(document):
    intr = rs.intrinsics()
    for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
        setattr(intr, key, document[key])
    intr.coeffs = document['coefficients']
    intr.model = getattr(rs.distortion, document['distortion_model'].split('.')[-1])
    return intr


def rays(intr, pixels):
    return np.array([rs.rs2_deproject_pixel_to_point(intr, p.tolist(), 1.)[:2]
                     for p in np.asarray(pixels).reshape(-1, 2)])


def stats(values):
    a = np.asarray(values)
    return dict(count=int(a.size), max=float(a.max()), p50=float(np.median(a)),
                p95=float(np.percentile(a, 95)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts'):
        raise ValueError('output must be under repository artifacts')
    output.mkdir(parents=True, exist_ok=False)
    files = [Path(__file__)]
    report = dict(schema='ego.factory_camera_usage_audit.v1', activation='NOT_ACTIVATED',
                  factory_values_modified=False, sessions={}, opencv=cv2.__version__,
                  sdk_version=getattr(rs, '__version__', 'unknown'),
                  diagnostic_limit='SDK-ray IPPE candidates matched to baseline branch; not production acceptance')
    for name in NAMES:
        session = ROOT/'artifacts/spatial_bench'/name
        audit = session/'spatial/mixed80_40_apriltag/pair_audit.json'
        files.append(audit)
        rows = [r for r in json.loads(audit.read_text()) if 'geometry' in r]
        cameras = {}; cals = {}; sdks = {}
        for device in ('ego', 'left'):
            path = session/device/'calibration.json'; files.append(path)
            doc = json.loads(path.read_text())
            intr = doc['intrinsics' if device == 'ego' else 'streams']['color']
            cal = pta.load_device_calibration(session, device, f'{device}.color')
            cals[device] = cal; sdks[device] = sdk_intrinsics(intr)
            index = session/device/f'{device}.color.jsonl'; files.append(index)
            index_rows = pta.load_formal_index_rows(session, device, f'{device}.color')
            shape_ok = all(r['metadata']['width'] == cal.width and
                           r['metadata']['height'] == cal.height and
                           r['size'] == cal.width*cal.height and
                           r['metadata']['encoding'] == 'Y8' for r in index_rows)
            np.testing.assert_array_equal(cal.distortion_coefficients, intr['coefficients'])
            field = np.array([[x, y] for x in np.linspace(0, cal.width-1, 33)
                              for y in np.linspace(0, cal.height-1, 19)])
            normalized = cv2.undistortPoints(field.reshape(-1, 1, 2), cal.camera_matrix,
                                            cal.distortion_coefficients).reshape(-1, 2)
            diff = (rays(sdks[device], field)-normalized)*np.diag(cal.camera_matrix)[:2]
            cameras[device] = dict(serial=doc['device']['serial'], stream=f'{device}.color',
                dimensions=[cal.width, cal.height], formal_frames=len(index_rows),
                frame_metadata_matches=shape_ok, factory_snapshot_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                model_in_snapshot=intr['distortion_model'], model_in_loaded_calibration=getattr(cal, 'distortion_model', None),
                K=cal.camera_matrix.tolist(), D=cal.distortion_coefficients.tolist(),
                field_sdk_vs_opencv_ray_difference_px=stats(np.linalg.norm(diff, axis=1)))
        baseline = {r: [] for r in mixed.ROLES}; sdk_series = {r: [] for r in mixed.ROLES}
        role_errors = {r: [] for r in mixed.ROLES}; pose_changes = {r: [] for r in mixed.ROLES}
        for row in rows:
            for role in mixed.ROLES:
                item = row['geometry']['roles'][role]
                corners = np.array(item['corners_px']); size = item['size_mm']/1000
                device = 'ego' if role.startswith('ego') else 'left'
                cal = cals[device]; intr = sdks[device]
                normalized = cv2.undistortPoints(corners.reshape(-1, 1, 2), cal.camera_matrix,
                                                cal.distortion_coefficients).reshape(-1, 2)
                sdk_xy = rays(intr, corners)
                diff = (sdk_xy-normalized)*np.diag(cal.camera_matrix)[:2]
                role_errors[role].extend(np.linalg.norm(diff, axis=1))
                original = np.array(item['camera_from_tag'])
                legacy_cal = replace(cal, distortion_model='opencv_brown_conrady')
                current = _solve_ippe_candidates(corners, calibration=legacy_cal, tag_size_m=size)
                old = min(current, key=lambda c: np.linalg.norm(c.camera_from_tag-original))
                np.testing.assert_allclose(old.camera_from_tag, original, atol=1e-10, rtol=0)
                h = size/2
                obj = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
                ok, rotations, translations, _ = cv2.solvePnPGeneric(obj, sdk_xy, np.eye(3),
                    np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
                assert ok
                candidates = []
                for rv, tv in zip(rotations, translations):
                    t = np.eye(4);t[:3,:3] = cv2.Rodrigues(rv)[0];t[:3,3] = tv.reshape(3)
                    if np.all(((t[:3,:3]@obj.T).T+t[:3,3])[:,2] > 0):candidates.append(t)
                # Isolate only the ray-model change, not ambiguity policy or detection.
                selected = min(candidates, key=lambda t: np.linalg.norm(t-original))
                pose_changes[role].append(float(np.linalg.norm(selected[:3,3]-original[:3,3])*1000))
                baseline[role].append(original);sdk_series[role].append(selected)
        transform, provenance = load_color_to_ir_snapshot(session/'left/calibration.json')
        report['sessions'][name] = dict(samples=len(rows), cameras=cameras,
            role_ray_difference_px={r:stats(v) for r,v in role_errors.items()},
            role_pose_translation_change_mm={r:stats(v) for r,v in pose_changes.items()},
            baseline=mixed.summarize_chains(baseline, transform)['agreement'],
            sdk_rays=mixed.summarize_chains(sdk_series, transform)['agreement'],
            extrinsics_read=provenance)
        print(name, json.dumps(report['sessions'][name]), flush=True)
    report['sha256'] = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':main()
