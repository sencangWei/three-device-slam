"""Two-external-tag cross-check tests: independent chain agreement."""

import json
import math

import numpy as np

from three_device_slam.spatial import mount_calibration as mc
from three_device_slam.spatial import two_tag_crosscheck as ttc
from three_device_slam.spatial.se3 import compose, invert, transform_from_xyz_rpy

from test_mount_calibration import (
    EGO_COLOR_CALIBRATION,
    UMI_COLOR_CALIBRATION,
    TRUTH_EGO_MOUNT,
    TRUTH_IR_COLOR,
    _render_tags,
    _write_stream,
)


# Tilted truths keep planar IPPE ambiguity unambiguous.  ext1 shares
# id1 with the mount tag on purpose: that collision is the real-world
# case (only id0/id1 assets exist) and must be filtered by range.
TRUTH_EGO_EXT0 = transform_from_xyz_rpy(
    (-0.12, 0.06, 0.55), (math.pi - 0.50, -0.30, -0.20)
)
TRUTH_EGO_EXT1 = transform_from_xyz_rpy(
    (0.10, 0.04, 0.60), (math.pi - 0.45, 0.35, 0.15)
)
TRUTH_UMI_EXT0 = transform_from_xyz_rpy(
    (0.01, 0.03, 0.50), (math.pi - 0.35, 0.28, -0.15)
)
# Rigid-rig constraint: both external tags must imply the SAME ego<->umi
# relative pose, so derive the second UMI truth from the first rather
# than choosing an independent orientation.
TRUTH_EGO_UMI = compose(TRUTH_EGO_EXT0, invert(TRUTH_UMI_EXT0))
TRUTH_UMI_EXT1 = compose(invert(TRUTH_EGO_UMI), TRUTH_EGO_EXT1)


def _write_two_tag_session(tmp_path):
    import json as _json
    from pathlib import Path

    session = Path(tmp_path) / "two_tag_session"
    session.mkdir()
    (session / "ego").mkdir()
    (session / "left").mkdir()
    (session / "ego" / "calibration.json").write_text(
        _json.dumps(
            {
                "schema": "ego.d435i.factory_calibration.v1",
                "device": {"serial": "327122078613"},
                "intrinsics": {
                    "color": {
                        "width": 1280,
                        "height": 720,
                        "fx": 906.93,
                        "fy": 906.87,
                        "ppx": 659.93,
                        "ppy": 377.74,
                        "coefficients": [0.0] * 5,
                    }
                },
                "extrinsics": {
                    "color_to_ir_left": {
                        "rotation_row_major": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        "translation_m": [0.0, 0.0, 0.0],
                    }
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    (session / "left" / "calibration.json").write_text(
        _json.dumps(
            {
                "schema": "umi.d405.factory_calibration.v1",
                "device": {"serial": "260322279785"},
                "streams": {
                    "color": {
                        "width": 1280,
                        "height": 720,
                        "fx": 656.29,
                        "fy": 654.45,
                        "ppx": 641.54,
                        "ppy": 354.31,
                        "coefficients": [0.0] * 5,
                    }
                },
                "extrinsics": {
                    "color_to_ir_left": {
                        "rotation_row_major": [
                            round(float(v), 9)
                            for v in TRUTH_IR_COLOR[:3, :3].reshape(-1)
                        ],
                        "translation_m": [
                            round(float(v), 6) for v in TRUTH_IR_COLOR[:3, 3]
                        ],
                    }
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    ego_images = [
        _render_tags(
            EGO_COLOR_CALIBRATION,
            (
                ("umi_right_tag36h11_id1_40mm.png", TRUTH_EGO_MOUNT),
                ("umi_left_tag36h11_id0_40mm.png", TRUTH_EGO_EXT0),
                ("umi_right_tag36h11_id1_40mm.png", TRUTH_EGO_EXT1),
            ),
        )
        for _ in range(5)
    ]
    umi_images = [
        _render_tags(
            UMI_COLOR_CALIBRATION,
            (
                ("umi_left_tag36h11_id0_40mm.png", TRUTH_UMI_EXT0),
                ("umi_right_tag36h11_id1_40mm.png", TRUTH_UMI_EXT1),
            ),
        )
        for _ in range(4)
    ]
    _write_stream(session, "ego", "ego.color", ego_images, 10_000_000_000)
    _write_stream(session, "left", "left.color", umi_images, 10_001_000_000)
    return session


def _expected_ir(ego_ext_truth, umi_ext_truth):
    t_ego_umi = compose(ego_ext_truth, invert(umi_ext_truth))
    t_umi_mount = compose(invert(t_ego_umi), TRUTH_EGO_MOUNT)
    return compose(TRUTH_IR_COLOR, t_umi_mount)


def test_two_tag_crosscheck_recovers_both_chains(tmp_path):
    session = _write_two_tag_session(tmp_path)

    report = ttc.compute_two_tag_crosscheck(
        session, mount_tag_id=1, ext0_tag_id=0, ext1_tag_id=1,
        min_observations=3,
    )

    assert report["schema"] == ttc.SCHEMA
    # The id1 collision on ego must have been split by range: every
    # near-cluster (mount) observation removed from the ext1 series must
    # account for the raw-vs-used difference.
    dropped1 = report["counts"]["ego_ext1_mount_cluster_dropped"]
    used1 = report["counts"]["ego_ext1_observations_used"]
    raw1 = report["counts"]["ego_ext1_observations_raw"]
    assert dropped1 + used1 == raw1
    assert dropped1 >= 3
    assert used1 >= 3
    assert report["counts"]["ego_mount_observations"] >= 3

    expected0 = _expected_ir(TRUTH_EGO_EXT0, TRUTH_UMI_EXT0)
    expected1 = _expected_ir(TRUTH_EGO_EXT1, TRUTH_UMI_EXT1)
    # Translation recovery on ~60 px synthetic tags carries ~1 deg of
    # IPPE bias (a few mm per hop, ~1 cm through the full chain); what
    # matters for the cross-check is that BOTH chains land within that
    # same noise, which the agreement assertions below pin down.
    for label, expected in (("ext0", expected0), ("ext1", expected1)):
        got = report["chains"][label]["umi_ir_left_from_mount_tag"]
        assert np.allclose(got["translation_m"], expected[:3, 3], atol=0.010)

    # Both chains observe the same static rig, so their T_ego_umi
    # estimates must agree far tighter than either agrees with truth.
    # Synthetic tags are only ~60 px, which carries ~1 deg of IPPE
    # recovery bias per chain (see test_mount_calibration), so keep the
    # bounds at the synthetic-noise level, not at the capture level.
    assert report["agreement"]["ego_color_from_umi_color_rotation_deg"] < 3.0
    assert report["agreement"]["mount_z_difference_m"] < 0.012

    persisted = json.loads(
        (session / "spatial" / "two_tag_crosscheck_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert persisted["schema"] == ttc.SCHEMA
    assert persisted["verdict"] == report["verdict"]


def test_drop_mount_cluster_keeps_far_series():
    from test_mount_calibration import _observation

    mount = [_observation((0.05, -0.02, 0.45)) for _ in range(30)]
    far = [_observation((0.10, 0.04, 0.60)) for _ in range(25)]
    kept, dropped = ttc._drop_mount_cluster(mount + far, 0.451, 0.10)
    assert dropped == 30
    assert len(kept) == 25


def test_drop_mount_cluster_no_false_drop_when_well_separated():
    from test_mount_calibration import _observation

    far = [_observation((0.10, 0.04, 0.60)) for _ in range(25)]
    kept, dropped = ttc._drop_mount_cluster(far, 0.451, 0.10)
    assert dropped == 0
    assert kept == far


def test_mean_transform_single_entry():
    from three_device_slam.spatial.se3 import transform_from_xyz_rpy

    truth = transform_from_xyz_rpy((0.01, 0.02, 0.03), (0.1, 0.2, 0.3))
    assert np.allclose(ttc._mean_transform([truth]), truth)


def test_crosscheck_preserves_minority_same_id_cluster(tmp_path, monkeypatch):
    from test_mount_calibration import _observation

    near = [_observation((0.05, -0.02, 0.30)) for _ in range(10)]
    far = [_observation((0.10, 0.04, 0.60)) for _ in range(30)]
    ext0 = [_observation((-0.12, 0.06, 0.55)) for _ in range(30)]

    def detect(session, device, stream, *, tag_id, **kwargs):
        if device == 'ego' and tag_id == 1:
            return near + far
        return far if tag_id == 1 else ext0

    monkeypatch.setattr(mc, '_detect', detect)
    monkeypatch.setattr(mc, '_color_to_ir_transform', lambda *args: np.eye(4))
    report = ttc.compute_two_tag_crosscheck(
        tmp_path, mount_tag_id=1, ext0_tag_id=0, ext1_tag_id=1,
    )
    assert report['counts']['ego_mount_observations'] == 10
    assert report['counts']['ego_ext1_observations_used'] == 30
    assert report['counts']['ego_ext1_mount_cluster_dropped'] == 10


def test_crosscheck_keeps_distinct_id_at_mount_range(tmp_path, monkeypatch):
    from test_mount_calibration import _observation

    near = [_observation((0.05, -0.02, 0.30)) for _ in range(10)]
    far = [_observation((0.10, 0.04, 0.60)) for _ in range(10)]
    ext0 = [_observation((-0.12, 0.06, 0.30)) for _ in range(10)]

    def detect(session, device, stream, *, tag_id, **kwargs):
        if device == 'ego' and tag_id == 1:
            return near + far
        return far if tag_id == 1 else ext0

    monkeypatch.setattr(mc, '_detect', detect)
    monkeypatch.setattr(mc, '_color_to_ir_transform', lambda *args: np.eye(4))
    report = ttc.compute_two_tag_crosscheck(
        tmp_path, mount_tag_id=1, ext0_tag_id=0, ext1_tag_id=1,
    )
    assert report['counts']['ego_ext0_observations_used'] == 10
    assert report['counts']['ego_ext0_mount_cluster_dropped'] == 0
