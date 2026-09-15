from copy import deepcopy

import pytest

from scripts.compare_fixed_factory_board_reports import compare


def reports():
    corner = dict(tag_id=1, corner=0, residual_px=[1., 0.])
    fold = dict(train_parity=0, train_ids=[0], holdout_ids=[1], status='OK',
                holdout_rms_px=1., corners=[corner])
    frame = dict(sample=0, image_sha256='image', status='ANALYZED', valid_tag_ids=[0,1],
                 preview_quality={}, folds=[fold])
    session = dict(source='board_factory_ego_normal_20260906T002443', source_hashes={}, factory_snapshot={},
                   frames=[frame, dict(deepcopy(frame), sample=1)])
    before = dict(sessions=[deepcopy(session) for _ in range(4)])
    for i in range(1,4):
        before['sessions'][i]['source'] = 'other'+str(i)
    after = deepcopy(before)
    for s in after['sessions']:
        for f in s['frames']:
            f['folds'][0]['corners'][0]['residual_px'] = [.1,0.]
            f['folds'][0]['holdout_rms_px'] = .1
    return before, after


def test_excludes_development_frame_and_keeps_matching_splits():
    before, after = reports()
    after['sessions'][0]['frames'][0]['folds'][0]['corners'][0]['residual_px'] = [999.,0.]
    result = compare(before, after)
    assert result['confirmation_images'] == 7
    first = result['sessions'][0]
    assert first['excluded_development_samples'] == [0]
    assert first['refined5']['rms_px'] == pytest.approx(.1)


@pytest.mark.parametrize('mutation', ['split', 'image_hash'])
def test_rejects_different_associations_or_sources(mutation):
    before, after = reports()
    f = after['sessions'][0]['frames'][0]
    if mutation == 'split':
        f['folds'][0]['holdout_ids'] = [2]
    else:
        f['image_sha256'] = 'changed'
    with pytest.raises(AssertionError):
        compare(before, after)
