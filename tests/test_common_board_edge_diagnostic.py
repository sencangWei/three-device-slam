import cv2
import numpy as np

from scripts.common_board_edge_diagnostic import edge_corners


def test_blurred_square_edge_intersections_recover_pixel_boundaries():
    gray = np.full((240, 240), 220, np.uint8)
    gray[60:181, 60:181] = 20
    gray = cv2.GaussianBlur(gray, (7, 7), 1.)
    truth = np.array([[59.5, 59.5], [180.5, 59.5], [180.5, 180.5], [59.5, 180.5]])
    initial = truth + [[.2, -.3], [-.2, .3], [.3, .2], [-.3, -.2]]
    recovered = edge_corners(gray, initial)
    assert np.max(np.linalg.norm(recovered-truth, axis=1)) < .04
    np.testing.assert_array_equal(gray[100:110, 100:110], np.full((10, 10), 20, np.uint8))
