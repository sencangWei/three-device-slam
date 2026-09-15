import numpy as np
import pytest

from scripts import common_board_corner_scale_diagnostic as diagnostic


def test_selection_preserves_original_id_order_and_arrays(monkeypatch):
    raw = {3: np.full((4, 2), 3.), 2: np.full((4, 2), 2.), 4: np.full((4, 2), 4.)}
    monkeypatch.setattr(diagnostic.cb, 'detect_grid', lambda *args: raw)
    monkeypatch.setattr(diagnostic.cb, 'valid_detections', lambda value: value)
    monkeypatch.setattr(diagnostic.cb, 'refine_legacy_corners', lambda gray, ds: ds)
    monkeypatch.setattr(diagnostic, 'refine_legacy_corners_scaled',
                        lambda gray, ds: {i: q+.1 for i, q in ds.items()})
    original = np.concatenate([raw[2], raw[3]])
    result = diagnostic.selected_scaled_corners(None, None, [2, 3], original)
    np.testing.assert_allclose(result, original+.1)
    np.testing.assert_array_equal(raw[2], np.full((4, 2), 2.))
    with pytest.raises(ValueError, match='missing'):
        diagnostic.selected_scaled_corners(None, None, [2, 5], original)
    with pytest.raises(AssertionError):
        diagnostic.selected_scaled_corners(None, None, [3, 2], original)
