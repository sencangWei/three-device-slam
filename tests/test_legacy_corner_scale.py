import cv2
import numpy as np
import pytest

from scripts.fixed_factory_board_diagnostic import refine_legacy_corners, refine_legacy_corners_scaled


def test_large_tag_inward_seed_outside_fixed_window_is_recovered():
    gray = np.full((220, 220), 220, np.uint8)
    gray[60:140, 60:140] = 25
    gray = cv2.GaussianBlur(gray, (3, 3), .5)
    truth = np.array([[139.5, 59.5], [59.5, 59.5], [59.5, 139.5], [139.5, 139.5]])
    seeds = truth + np.sign(99.5-truth)*7
    old = refine_legacy_corners(gray, {12: seeds})[12]
    assert np.linalg.norm(old-truth, axis=1).max() > 5
    fixed = refine_legacy_corners_scaled(gray, {12: seeds})[12]
    assert np.linalg.norm(fixed-truth, axis=1).max() < .2


def test_small_tag_retains_exact_fixed5_behavior():
    gray = np.full((100, 100), 220, np.uint8)
    gray[30:60, 30:60] = 25
    seeds = np.array([[59., 30.], [30., 30.], [30., 59.], [59., 59.]])
    np.testing.assert_array_equal(refine_legacy_corners_scaled(gray, {2: seeds})[2],
                                  refine_legacy_corners(gray, {2: seeds})[2])


@pytest.mark.parametrize('destination', [
    [[145, 70], [70, 60], [50, 130], [140, 145]],
    [[160, 90], [80, 60], [80, 140], [160, 120]],
])
def test_perspective_warp_keeps_recovered_outer_corners(destination):
    gray = np.full((240, 240), 220, np.uint8)
    gray[60:140, 60:140] = 25
    gray = cv2.GaussianBlur(gray, (3, 3), .5)
    truth = np.array([[139.5, 59.5], [59.5, 59.5], [59.5, 139.5], [139.5, 139.5]], np.float32)
    target = np.array(destination, np.float32)
    homography = cv2.getPerspectiveTransform(truth, target)
    warped = cv2.warpPerspective(gray, homography, (240, 240), borderValue=220)
    seeds = target + np.sign(target.mean(0)-target)*7
    corrected = refine_legacy_corners_scaled(warped, {0: seeds})[0]
    assert np.linalg.norm(corrected-target, axis=1).max() < .45
