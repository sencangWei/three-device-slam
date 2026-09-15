#!/usr/bin/env python3
"""Re-run the ORIGINAL tag-chain mount calibration with candidate Ego intrinsics.

The 61.3mm consensus was produced by the single-external-tag chain on
three 20260905 sessions (mount_calib_20260905T013727 / 024412 / 024608,
z = -59.1 / -61.1 / -61.6 mm).  This driver replays exactly that chain --
same detections, same outlier gates, same reference-transform averaging --
with the Ego RGB calibration swapped for the IR-anchored candidates from
scripts/ego_rgb_model_selection.py, to test whether the corrected model
also dissolves the original paradox.  Nothing is written into the source
sessions; factory calibration files stay untouched.

Usage: run from repo root with the project venv; ~minutes per session.
"""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from three_device_slam.spatial import mount_calibration as mc
from three_device_slam.spatial import pair_tag_alignment as pta
from three_device_slam.spatial.se3 import compose, invert

from scripts.mount_z_decision import candidate_models

SESSIONS = (
    'mount_calib_20260905T013727',
    'mount_calib_20260905T024412',
    'mount_calib_20260905T024608',
)
MOUNT_TAG_ID = 1
EXTERNAL_TAG_ID = 0
TAG_SIZE_M = 0.04


def run_chain(session: Path, ego_calibration):
    """Replay mc.compute_mount_calibration's chain with a swapped ego cal."""
    original = pta.load_device_calibration

    def patched(session_dir, device_id, stream_id):
        if device_id == 'ego':
            return replace(ego_calibration, calibration_id=f'candidate:{ego_calibration.calibration_id}')
        return original(session_dir, device_id, stream_id)

    pta.load_device_calibration = patched
    try:
        ego_mount = mc._reject_translation_outliers(
            mc._detect(session, 'ego', 'ego.color', tag_id=MOUNT_TAG_ID, tag_size_m=TAG_SIZE_M, stride=1))
        t_ego_mount = mc._reference_transform(ego_mount)
        ego_ext = mc._reject_translation_outliers(
            mc._detect(session, 'ego', 'ego.color', tag_id=EXTERNAL_TAG_ID, tag_size_m=TAG_SIZE_M, stride=1))
        umi_ext = mc._reject_translation_outliers(
            mc._detect(session, 'left', 'left.color', tag_id=EXTERNAL_TAG_ID, tag_size_m=TAG_SIZE_M, stride=1))
        t_ego_ext = mc._reference_transform(ego_ext)
        t_umi_ext = mc._reference_transform(umi_ext)
        t_ego_umi = compose(t_ego_ext, invert(t_umi_ext))
        t_umi_mount = compose(invert(t_ego_umi), t_ego_mount)
        t_ir_color = mc._color_to_ir_transform(session, 'left')
        t_ir_mount = compose(t_ir_color, t_umi_mount)
    finally:
        pta.load_device_calibration = original
    return t_ego_umi, t_umi_mount, t_ir_mount, len(ego_mount), len(ego_ext), len(umi_ext)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('artifacts/spatial_bench'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)

    # The factory profile is read from any one session's sealed snapshot;
    # candidate refits reuse the IR-anchored probe data.
    from scripts import common_board_calibration as cb
    _, factory, _, _, _, _ = cb.load_session(args.root / 'common_board_joint_20260906_run1', allow_partial=True)
    models = candidate_models(factory['ego'])

    rows = {}
    for session_name in SESSIONS:
        session = args.root / session_name
        for model_name, (calibration, meta) in models.items():
            t_ego_umi, t_umi_mount, t_ir_mount, n_mount, n_ego_ext, n_umi_ext = run_chain(session, calibration)
            key = f'{session_name}/{model_name}'
            rows[key] = dict(
                n_observations=dict(ego_mount=n_mount, ego_ext=n_ego_ext, umi_ext=n_umi_ext),
                ucolor_mount_xyz_mm=(t_umi_mount[:3, 3] * 1000).round(3).tolist(),
                ucolor_mount_z_mm=float(t_umi_mount[2, 3] * 1000),
                ucolor_mount_norm_mm=float(np.linalg.norm(t_umi_mount[:3, 3]) * 1000),
                irleft_mount_xyz_mm=(t_ir_mount[:3, 3] * 1000).round(3).tolist(),
                irleft_mount_z_mm=float(t_ir_mount[2, 3] * 1000),
                irleft_mount_norm_mm=float(np.linalg.norm(t_ir_mount[:3, 3]) * 1000),
                ego_color_from_umi_color_rpy_deg=mc._xyz_rpy(t_ego_umi)['rotation_rpy_deg'],
            )
            r = rows[key]
            print(f"{session_name} {model_name:10s} IRleft z={r['irleft_mount_z_mm']:7.2f} "
                  f"norm={r['irleft_mount_norm_mm']:6.2f} | Ucolor z={r['ucolor_mount_z_mm']:7.2f} "
                  f"xyz={r['ucolor_mount_xyz_mm']}", flush=True)

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'report.json').write_text(json.dumps(dict(
        schema='ego.two_device.mount_calibration.original_chain_replay.v1',
        status='DEVELOPMENT_ONLY', activation='NOT_ACTIVATED',
        sessions=list(SESSIONS), mount_tag_id=MOUNT_TAG_ID, external_tag_id=EXTERNAL_TAG_ID,
        historical_consensus_irleft_z_mm=(-59.1, -61.1, -61.6),
        candidates={k: dict(source=v[1].get('source'), K=np.round(v[0].camera_matrix, 4).tolist(),
                            D=np.round(v[0].distortion_coefficients, 6).tolist()) for k, v in models.items()},
        rows=rows), indent=2, sort_keys=True), encoding='utf-8')
    print(f"report={args.output / 'report.json'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
