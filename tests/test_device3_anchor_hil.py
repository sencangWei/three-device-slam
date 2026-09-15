import numpy as np

from scripts.analyze_device3_anchor_hil import (
    consensus_anchor,
    largest_observation_gap,
    phase_name,
)
from three_device_slam.spatial.se3 import transform_from_xyz_rpy


def test_short_anchor_phase_boundaries():
    assert phase_name(0) == "initial_static"
    assert phase_name(10) == "translation"
    assert phase_name(22) == "rotation"
    assert phase_name(32) == "occlusion"
    assert phase_name(37) == "return_final"
    assert phase_name(45) is None


def test_consensus_anchor_rejects_one_large_rotation_jump():
    values = [
        transform_from_xyz_rpy((index * 0.001, 0, 0), (0, 0, index * 0.002))
        for index in range(5)
    ]
    values.append(transform_from_xyz_rpy((0.2, 0, 0), (0, 0, np.deg2rad(20))))
    anchor, inliers = consensus_anchor(values)
    assert len(inliers) == 5
    assert np.linalg.norm(anchor[:3, 3]) < 0.01


def test_largest_observation_gap_reports_occlusion_edges():
    assert largest_observation_gap([0.0, 0.1, 2.5, 2.6]) == {
        "duration_s": 2.4,
        "before_s": 0.1,
        "after_s": 2.5,
    }
