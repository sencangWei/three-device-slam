import copy

import numpy as np
import pytest

from scripts import common_board_calibration as cb
from scripts.validate_ego_stereo_color_frozen_fit import score_frame
from test_common_board_calibration import camera


@pytest.mark.parametrize('defect', [None, 'drift', 'missing_drift', 'timestamps', 'pixel_error'])
def test_frozen_scoring_does_not_fit_away_error(defect):
    cal = camera()
    points = np.random.default_rng(80).uniform([-.2, -.2, .7], [.2, .2, .9], (32, 3))
    meta = dict(sdk_timestamp_ms=12., sdk_timestamp_domain='hardware_clock')
    frame = dict(sample=0, points_ir_left_m=points.tolist(), common_ids=list(range(8)),
        projections={'color': {'residual_px': np.zeros((32, 2)).tolist()}},
        drift_from_first_px=dict(color=.1, ir_left=.1, ir_right=.1),
        stream_metadata=dict(ir_left=copy.deepcopy(meta), ir_right=copy.deepcopy(meta)))
    if defect == 'drift':
        frame['drift_from_first_px']['color'] = .751
    elif defect == 'missing_drift':
        frame['drift_from_first_px']['ir_left'] = None
    elif defect == 'timestamps':
        frame['stream_metadata']['ir_right']['sdk_timestamp_ms'] += 1.
    elif defect == 'pixel_error':
        frame['projections']['color']['residual_px'] = np.tile([2., 0.], (32, 1)).tolist()
    scored = score_frame(frame, cal, np.eye(4), cal, np.eye(4))
    assert scored['static_ok'] == (defect not in ('drift', 'missing_drift'))
    assert scored['stereo_timestamp_ok'] == (defect != 'timestamps')
    assert scored['metrics']['p95_px'] == pytest.approx(2. if defect == 'pixel_error' else 0.)
