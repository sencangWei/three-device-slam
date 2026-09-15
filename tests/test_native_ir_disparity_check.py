import cv2
import numpy as np
import pytest

from scripts.native_ir_disparity_check import patch_shift


@pytest.mark.parametrize('size', [11, 19])
def test_native_patch_shift_recovers_known_subpixel_disparity(size):
    g = cv2.GaussianBlur(np.random.default_rng(44).uniform(0, 255, (100, 100)).astype(np.float32), (0, 0), 2)
    x, y = np.meshgrid(np.arange(100, dtype=np.float32), np.arange(100, dtype=np.float32))
    right = cv2.remap(g, x+.25, y, cv2.INTER_LINEAR)
    result = patch_shift(g, right, (50., 50.), (50., 50.), size)
    assert result['right_x_shift_px'] == pytest.approx(-.25, abs=.04)
    assert result['correlation'] > .99
    assert not result['search_bound_hit']


def test_no_contrast_is_not_a_valid_match():
    with pytest.raises(ValueError, match='contrast'):
        patch_shift(np.zeros((40, 40)), np.zeros((40, 40)), (20., 20.), (20., 20.), 11)
