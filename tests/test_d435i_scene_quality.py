import numpy as np
import pytest

from three_device_slam.devices.d435i_ego.scene_quality import aggregate_support, evaluate_pair, support_statistics


def supported_points():
    xx, yy = np.meshgrid(np.linspace(100, 1100, 8), np.linspace(100, 600, 5))
    return np.c_[xx.ravel(), yy.ravel()]


def test_spread_positive_disparity_support():
    left = supported_points()
    assert support_statistics(left, left-[20,0], 1280,720)["supported"]


def test_narrow_strip_is_not_spatial_support():
    left = supported_points(); left[:,1] /= 10
    result = support_statistics(left, left-[20,0],1280,720)
    assert result["checks"]["unique_matches"]
    assert not result["checks"]["vertical_coverage"]
    assert not result["supported"]


@pytest.mark.parametrize("offset", [[-20,0], [20,5], [1,0]])
def test_wrong_epipolar_geometry(offset):
    left = supported_points()
    assert support_statistics(left,left-offset,1280,720)["unique_matches"] == 0


def test_duplicate_keypoint_orientations_do_not_inflate_support():
    left = np.repeat(supported_points()[:14], 4, axis=0)
    result = support_statistics(left,left-[20,0],1280,720)
    assert result["unique_matches"] == 14
    assert not result["supported"]


def test_nonfinite_and_empty_points_fail_closed():
    assert not support_statistics([],[],1280,720)["supported"]
    assert support_statistics([[np.nan, 10]],[[0,10]],1280,720)["unique_matches"] == 0
    with pytest.raises(ValueError):
        support_statistics([[1,2]],[],1280,720)


def test_blank_stereo_and_invalid_encoding():
    blank = np.zeros((720,1280),np.uint8)
    assert not evaluate_pair(blank,blank)["supported"]
    with pytest.raises(ValueError,match="mono8"):
        evaluate_pair(blank.astype(float),blank)


def test_textured_shifted_stereo_passes_real_matcher():
    rng = np.random.default_rng(17)
    left = rng.integers(0,256,(360,640),dtype=np.uint8)
    # Exact rectified correspondence: x_left - x_right = 20 pixels.
    right = np.zeros_like(left)
    right[:, :-20] = left[:, 20:]
    result = evaluate_pair(left,right)
    assert result["supported"], result
    assert result["unique_matches"] >= 30
    assert not evaluate_pair(left,left)["supported"]  # zero baseline/disparity


@pytest.mark.parametrize("count,good,expected",[(0,0,False),(9,9,False),(10,7,False),(10,8,True),(10,10,True)])
def test_aggregate_support_boundaries(count,good,expected):
    verdict,fraction = aggregate_support([{"supported":i<good} for i in range(count)])
    assert verdict is expected
    assert fraction == (good/count if count else 0.)
