"""Explicit SDK column-major serialization and audited legacy snapshot reading.

No hardware import or access. SDK rotations map source points into the target;
changing array storage order is NOT inverting that transform.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


SCHEMA = 'ego.realsense.extrinsics.v2'
CONVENTION = 'SDK source_to_target: p_target = R*p_source + t'
# Exact whole-file bytes from the audited pair_color_tag_capture.py snapshots.
# Both 21:59 and22:51 sessions contain these same snapshots. Never infer layout
# from device serial, factory_calibration.v1, filename, or rotation magnitude.
LEGACY_COLUMN_MAJOR_SNAPSHOTS = frozenset({
    '6beb8a6b092319eb1848fdf1dff07639f9ef13eebc0cfeff6e182795c74b2d61',
    '9c4860a1d0bb666ecd32c827cda3965e83a1b920515647755f58e04686b1ce0c',
})


def serialize_sdk_extrinsics(value) -> dict:
    rotation = [float(v) for v in value.rotation]
    translation = [float(v) for v in value.translation]
    if len(rotation) != 9 or len(translation) != 3 or not all(
        math.isfinite(v) for v in rotation + translation
    ):
        raise ValueError('invalid SDK extrinsics dimensions or values')
    return dict(schema=SCHEMA, rotation_layout='row_major',
                source_rotation_layout='column_major', source='librealsense',
                rotation_row_major=[rotation[i] for i in (0, 3, 6, 1, 4, 7, 2, 5, 8)],
                translation_m=translation, convention=CONVENTION)


def load_color_to_ir_snapshot(path: Path):
    """Return a proper transform and explicit read-policy evidence.

    Unknown SDK-marked legacy snapshots are ambiguous and require a separate
    source audit, not an automatic transpose. Unmarked historical row-major
    documents retain their field's declared semantics.
    """
    import numpy as np

    payload = Path(path).read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    document = json.loads(payload)
    item = document['extrinsics']['color_to_ir_left']
    if 'schema' in item or 'rotation_layout' in item or 'source_rotation_layout' in item:
        if (item.get('schema') != SCHEMA or item.get('rotation_layout') != 'row_major'
                or item.get('source_rotation_layout') != 'column_major'
                or item.get('source') != 'librealsense' or item.get('convention') != CONVENTION):
            raise ValueError('unsupported or conflicting extrinsics metadata')
        policy = 'explicit_v2_row_major'
    elif digest in LEGACY_COLUMN_MAJOR_SNAPSHOTS:
        policy = 'audited_legacy_sdk_column_major'
    elif ('SDK' in item.get('convention', '') or item.get('source') == 'librealsense'
          or 'librealsense' in document.get('provenance', '')
          or 'SDK' in document.get('extrinsic_convention', '')):
        raise ValueError('unknown legacy SDK rotation layout; audit snapshot SHA256 before use')
    elif 'convention' in item or 'extrinsic_convention' in document:
        raise ValueError('unknown or conflicting legacy transform convention')
    else:
        policy = 'legacy_declared_row_major'
    raw = np.asarray(item['rotation_row_major'], dtype=np.float64)
    translation = np.asarray(item['translation_m'], dtype=np.float64)
    if raw.shape != (9,) or translation.shape != (3,) or not (
        np.isfinite(raw).all() and np.isfinite(translation).all()
    ):
        raise ValueError('invalid extrinsics dimensions or values')
    rotation = raw.reshape(3, 3)
    if policy == 'audited_legacy_sdk_column_major':
        rotation = rotation.T
    # Permit float32 roundoff, not arbitrary shear/reflection silently repaired
    # by SVD. Store no rewritten snapshot; projection is read-time only.
    if (np.max(np.abs(rotation.T @ rotation - np.eye(3))) > 1e-5
            or abs(np.linalg.det(rotation) - 1) > 1e-5):
        raise ValueError('extrinsics rotation is not near SO(3)')
    u, _, vt = np.linalg.svd(rotation)
    transform = np.eye(4)
    transform[:3, :3] = u @ np.diag((1., 1., np.linalg.det(u @ vt))) @ vt
    transform[:3, 3] = translation
    return transform, dict(schema='ego.realsense.extrinsics_read.v2',
                           snapshot_sha256=digest, rotation_policy=policy,
                           transform_direction='ir_left_from_color',
                           raw_snapshot_modified=False)
