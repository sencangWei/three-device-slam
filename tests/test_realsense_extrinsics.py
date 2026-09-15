"""Nonsymmetric SDK rotations must not be mistaken for row-major arrays."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.pair_color_tag_capture import _extrinsics_dict
from three_device_slam.devices.d435i_ego.worker import _extrinsics
from three_device_slam.spatial import mount_calibration as mc
from three_device_slam.spatial.se3 import transform_from_xyz_rpy


ROOT = Path(__file__).resolve().parents[1]
SESSION = ROOT / 'artifacts/spatial_bench/mixed80_newview_20260905T225115'
TRUTH = transform_from_xyz_rpy((.015, -.002, .001), (.2, -.3, .4))


@pytest.mark.parametrize('serializer', [_extrinsics_dict, _extrinsics])
def test_sdk_serializer_converts_column_major_at_source(serializer):
    sdk = SimpleNamespace(rotation=TRUTH[:3, :3].flatten(order='F'),
                          translation=TRUTH[:3, 3])
    result = serializer(sdk)
    np.testing.assert_allclose(np.array(result['rotation_row_major']).reshape(3, 3),
                               TRUTH[:3, :3], atol=1e-12)
    assert result['rotation_layout'] == 'row_major'
    assert result['source_rotation_layout'] == 'column_major'
    assert result['schema'] == 'ego.realsense.extrinsics.v2'
    np.testing.assert_array_equal(result['translation_m'], TRUTH[:3, 3])


def test_known_sealed_sdk_capture_matches_native_point_transform():
    rs = pytest.importorskip('pyrealsense2')
    if not SESSION.exists():
        pytest.skip('local sealed hardware evidence not present')
    raw = json.loads((SESSION / 'left/calibration.json').read_text())['extrinsics']['color_to_ir_left']
    sdk = rs.extrinsics()
    sdk.rotation = raw['rotation_row_major']
    sdk.translation = raw['translation_m']
    got = mc._color_to_ir_transform(SESSION, 'left')
    for point in ([.015, -.01, -.05], [.2, .1, .5], [0, 0, 0]):
        expected = rs.rs2_transform_point_to_point(sdk, point)
        np.testing.assert_allclose(got[:3, :3] @ point + got[:3, 3], expected, atol=5e-8)


def test_true_legacy_row_major_not_transposed(tmp_path):
    root = tmp_path / 'left'
    root.mkdir()
    (root / 'calibration.json').write_text(json.dumps({'extrinsics': {'color_to_ir_left': {
        'rotation_row_major': TRUTH[:3, :3].flatten().tolist(),
        'translation_m': TRUTH[:3, 3].tolist()}}}))
    np.testing.assert_allclose(mc._color_to_ir_transform(tmp_path, 'left'), TRUTH, atol=1e-12)


def write_snapshot(tmp_path, item):
    from three_device_slam.devices.realsense_extrinsics import load_color_to_ir_snapshot
    path = tmp_path / 'calibration.json'
    path.write_text(json.dumps({'extrinsics': {'color_to_ir_left': item}}))
    return load_color_to_ir_snapshot(path)


def test_new_roundtrip_and_inverse_direction(tmp_path):
    from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics
    item = serialize_sdk_extrinsics(SimpleNamespace(rotation=TRUTH[:3, :3].flatten(order='F'),
                                                    translation=TRUTH[:3, 3]))
    got, evidence = write_snapshot(tmp_path, item)
    np.testing.assert_allclose(got, TRUTH, atol=1e-12)
    point = np.array([.1, -.2, .3, 1])
    np.testing.assert_allclose(np.linalg.inv(got) @ (TRUTH @ point), point, atol=1e-12)
    assert evidence['rotation_policy'] == 'explicit_v2_row_major'
    assert not evidence['raw_snapshot_modified']


@pytest.mark.parametrize('change', [
    {'schema': 'unknown'}, {'rotation_layout': 'column_major'},
    {'source_rotation_layout': 'row_major'}, {'source': 'unknown'},
    {'convention': 'target_to_source'}, {'rotation_row_major': [1] * 8},
    {'rotation_row_major': [float('nan')] * 9}, {'translation_m': [0, 0]},
    {'rotation_row_major': [2, 0, 0, 0, 1, 0, 0, 0, 1]},
    {'rotation_row_major': [-1, 0, 0, 0, 1, 0, 0, 0, 1]},
])
def test_bad_metadata_or_nonrotation_not_silently_repaired(tmp_path, change):
    item = _extrinsics_dict(SimpleNamespace(rotation=TRUTH[:3, :3].flatten(order='F'),
                                           translation=TRUTH[:3, 3]))
    item.update(change)
    with pytest.raises(ValueError):
        write_snapshot(tmp_path, item)


def test_unknown_sdk_snapshot_and_reformatted_known_bytes_fail_closed(tmp_path):
    from three_device_slam.devices.realsense_extrinsics import CONVENTION, load_color_to_ir_snapshot
    item = {'rotation_row_major': TRUTH[:3, :3].flatten().tolist(),
            'translation_m': TRUTH[:3, 3].tolist(), 'convention': CONVENTION}
    with pytest.raises(ValueError, match='unknown legacy SDK'):
        write_snapshot(tmp_path, item)
    if SESSION.exists():
        path = tmp_path / 'calibration.json'
        path.write_text(json.dumps(json.loads((SESSION / 'left/calibration.json').read_text())))
        with pytest.raises(ValueError, match='unknown legacy SDK'):
            load_color_to_ir_snapshot(path)


@pytest.mark.parametrize('convention', ['target_to_source', 'p_source=R*p_target+t', 'unknown'])
def test_unversioned_conflicting_direction_rejected(tmp_path, convention):
    with pytest.raises(ValueError, match='convention'):
        write_snapshot(tmp_path, dict(rotation_row_major=TRUTH[:3, :3].flatten().tolist(),
                                     translation_m=TRUTH[:3, 3].tolist(), convention=convention))


def test_d405_group_snapshot_uses_same_row_major_serializer(monkeypatch):
    from three_device_slam.acquisition import rsusb_pair as pair
    intr = SimpleNamespace(fx=1, fy=1, ppx=0, ppy=0, model='none', coeffs=[0] * 5)
    sdk = SimpleNamespace(rotation=TRUTH[:3, :3].flatten(order='F'), translation=TRUTH[:3, 3])
    class Profile:
        def __init__(self, key): self.key = key
        def as_video_stream_profile(self): return self
        def get_intrinsics(self): return intr
        def width(self): return 1280
        def height(self): return 720
        def fps(self): return 30
        def format(self): return 'Y8'
        def get_extrinsics_to(self, other): return sdk
    monkeypatch.setattr(pair.d405, 'stream_key_from_profile', lambda p: p.key)
    monkeypatch.setattr(pair.d435, '_device_snapshot', lambda *a: {})
    result = pair._d405_calibration_snapshot(None, None,
            [Profile('infrared_left'), Profile('infrared_right')], {})
    np.testing.assert_allclose(np.array(result['left_to_right']['rotation_row_major']).reshape(3, 3),
                               TRUTH[:3, :3], atol=1e-12)
    assert result['left_to_right']['schema'] == 'ego.realsense.extrinsics.v2'
