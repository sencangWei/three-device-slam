import copy
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from three_device_slam.spatial import mount_bundle_diagnostic as b


def pose(xyz, angles):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    t[:3, 3] = xyz
    return t


def fixture():
    mount = pose([.015, -.01, -.046], [177, 1, 179])
    external = {1: pose([-.10, -.06, .3], [160, -15, 0]),
                2: pose([.13, -.03, .34], [155, 20, 10])}
    sessions = []
    for i in range(3):
        x = pose([.02 + .03*i, .015, .23], [18+i*3, -5+i*4, 2-i*5])
        s = {'session': str(i), 'groups': {}, 'ir_from_color': np.eye(4).tolist(),
             'calibration': {d: {'K': [[850., 0, 640], [0, 850., 360], [0, 0, 1]],
                                'D': [0.]*5} for d in ('ego', 'left')}}
        for key, truth in [('ego_mount', x@mount)] + [(f'{d}_ext{j}', x@a if d=='ego' else a)
                                                      for d in ('ego', 'left') for j, a in external.items()]:
            # Perturb every initializer; observations retain exact independent truth.
            seed = truth.copy(); seed[:3, 3] += [.0007, -.0008, .001]
            s['groups'][key] = [{'pose': seed.tolist(), 'corners': b.project(truth, s['calibration'][key.split('_')[0]]).tolist()} for _ in range(3)]
        sessions.append(s)
    return sessions, mount


def test_known_mount_recovered_from_joint_corners():
    sessions, truth = fixture()
    fit = b.fit(sessions[:2])
    assert fit['success']
    np.testing.assert_allclose(fit['mount'], truth, atol=1e-6)
    held = b.fit([sessions[2]], use_mount=False)
    assert b.score_mount(sessions[2], held['poses'][0][0], fit['mount'])['corner_rmse_px'] < 1e-5


def test_holdout_registration_never_uses_mount_corners_or_seeds():
    sessions, truth = fixture()
    original = b.fit([sessions[2]], use_mount=False)
    changed = copy.deepcopy(sessions[2])
    for o in changed['groups']['ego_mount']:
        o['corners'] = (np.array(o['corners']) + 100).tolist()
        o['pose'] = np.full((4, 4), float('nan')).tolist()
    second = b.fit([changed], use_mount=False)
    np.testing.assert_array_equal(original['poses'][0][0], second['poses'][0][0])
    assert b.score_mount(changed, second['poses'][0][0], truth)['corner_rmse_px'] > 100


def test_holdout_scores_wrong_mount_worse_and_near_pi_rotation_finite():
    sessions, truth = fixture()
    held = b.fit([sessions[2]], use_mount=False)
    wrong = truth.copy(); wrong[0, 3] += .01
    assert b.score_mount(sessions[2], held['poses'][0][0], wrong)['corner_rmse_px'] > 20
    p = pose([0, 0, .2], [180, 0, 0])
    np.testing.assert_allclose(b.mean_pose([p, p]), p, atol=1e-12)


def test_external_sample_labels_independent_of_mount_presence_and_pose():
    def detection(tag, z):
        return SimpleNamespace(tag_id=tag, selected_index=0,
                               candidates=[SimpleNamespace(camera_from_tag=pose([0, 0, z], [180, 0, 0]))])
    a, c, mount = detection(1, .45), detection(2, .5), detection(1, .2)
    for inputs in ([a, c, mount], [a, c], [a, c, detection(1, .32)]):
        ext = dict((k, v) for k, v in b.label_detections(inputs, 'ego') if k.startswith('ext'))
        assert ext == {'ext1': a, 'ext2': c}
    assert 'ext1' not in dict(b.label_detections([a, c, detection(1, .46)], 'ego'))


def test_external_only_serialization_has_no_fake_mount():
    sessions, _ = fixture()
    result = b.serialize_fit(b.fit([sessions[-1]], use_mount=False), np.eye(4))
    assert not result['mount_observations_used']
    assert 'umi_ir_left_from_mount' not in result
    assert 'umi_color_from_mount' not in result


def test_nonconverged_fit_cannot_report_validated_improvement(monkeypatch):
    sessions, truth = fixture()
    original = b.fit
    def unsuccessful(*args, **kwargs):
        result = original(*args, **kwargs)
        result['success'] = False
        return result
    monkeypatch.setattr(b, 'fit', unsuccessful)
    report = b.run(sessions, truth)
    assert report['verdict'] == 'FIT_FAILED'
    assert report['primary_holdout_improved'] is None
