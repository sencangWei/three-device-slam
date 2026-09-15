import copy

import pytest

from scripts.compare_legacy_corner_replays import verify_pair


@pytest.mark.parametrize('defect', [None, 'count', 'ids', 'policy', 'scale', 'development'])
def test_paired_comparison_rejects_non_corner_changes(defect):
    old = dict(source_report_sha256='raw', setup={}, ir_wide_threshold=True,
               ir_detection_scale=3, edge_corners=False, identity_excluded_ids=[1],
               frames=[dict(sample=0, common_ids=[2, 3], status='DIAGNOSTIC_MEASURED',
                            detector_development_sample=False)])
    new = copy.deepcopy(old)
    new['legacy_corner_policy'] = 'scale_aware_v1'
    if defect == 'count':
        new['frames'] = []
    elif defect == 'ids':
        new['frames'][0]['common_ids'] = [2]
    elif defect == 'policy':
        new['legacy_corner_policy'] = 'fixed5'
    elif defect == 'scale':
        new['ir_detection_scale'] = 2
    elif defect == 'development':
        new['frames'][0]['detector_development_sample'] = True
    if defect:
        with pytest.raises(ValueError):
            verify_pair(old, new)
    else:
        verify_pair(old, new)
