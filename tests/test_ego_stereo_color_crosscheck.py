import copy
import json

import pytest

from scripts import common_board_calibration as cb
from scripts.crosscheck_ego_stereo_color_poses import load_bound_validation


def fixture_report(tmp_path):
    setup = {'schema': 'synthetic_test'}
    cb.write_json(tmp_path/'setup.json', setup)
    streams = {}
    for role in ('color', 'ir_left', 'ir_right'):
        path = tmp_path/f'000_{role}.png'
        path.write_bytes(b'hashed-fixture; PNG decode checked by upstream validator')
        streams[role] = dict(file=path.name, sha256=cb.digest(path), sdk_timestamp_ms=123.,
                             frame_number=1, sdk_timestamp_domain='hardware_clock')
    row = dict(sample=0, setup_sha256=cb.digest(tmp_path/'setup.json'), streams=streams)
    (tmp_path/'frames.jsonl').write_text(json.dumps(row)+'\n')
    capture = dict(status='CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION', failure=None, cleanup_errors=[],
                   samples=1, hashes={p: cb.digest(tmp_path/p) for p in ('setup.json', 'frames.jsonl')})
    cb.write_json(tmp_path/'capture_report.json', capture)
    frame = dict(sample=0, stream_metadata=copy.deepcopy(streams), status='DIAGNOSTIC_MEASURED',
                 common_ids=[2], points_ir_left_m=[[0., 0., 1.]]*4,
                 projections={'color': {'residual_px': [[0., 0.]]*4}})
    return dict(source=str(tmp_path), source_report_sha256=cb.digest(tmp_path/'capture_report.json'),
                setup=setup, frames=[frame])


@pytest.mark.parametrize('defect', [None, 'sample', 'metadata', 'point_count', 'nonfinite'])
def test_derived_frame_remains_bound_to_raw(tmp_path, defect):
    report = fixture_report(tmp_path)
    frame = report['frames'][0]
    if defect == 'sample':
        frame['sample'] = 1
    elif defect == 'metadata':
        frame['stream_metadata']['color']['frame_number'] = 2
    elif defect == 'point_count':
        frame['points_ir_left_m'].pop()
    elif defect == 'nonfinite':
        frame['points_ir_left_m'][0][0] = float('nan')
    path = tmp_path/'validation.json'
    path.write_text(json.dumps(report))
    if defect is None:
        assert load_bound_validation(path)['frames'][0]['sample'] == 0
    else:
        with pytest.raises(ValueError, match='derived'):
            load_bound_validation(path)
