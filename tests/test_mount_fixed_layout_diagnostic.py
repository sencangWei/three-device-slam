import copy

import numpy as np

from three_device_slam.spatial import mount_fixed_layout_diagnostic as f
from test_mount_bundle_diagnostic import fixture


def test_shared_layout_recovers_truth_and_predicts_unseen_mount():
    sessions, truth = fixture()
    result = f.shared_fit(sessions[:2])
    np.testing.assert_allclose(result['mount'], truth, atol=1e-6)
    x = f.register_external_only(sessions[2], result['externals'])
    assert f.b.score_mount(sessions[2], x, result['mount'])['corner_rmse_px'] < 1e-5


def test_fixed_registration_does_not_read_mount_or_left_observations():
    sessions, _ = fixture()
    result = f.shared_fit(sessions[:2])
    x = f.register_external_only(sessions[2], result['externals'])
    changed = copy.deepcopy(sessions[2])
    changed['groups'] = {k: v for k,v in changed['groups'].items() if k.startswith('ego_ext')}
    np.testing.assert_array_equal(x, f.register_external_only(changed, result['externals']))
