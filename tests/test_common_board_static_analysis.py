import numpy as np
import pytest
from scripts import common_board_calibration as cb
from scripts.common_board_static_analysis import inspect_static
from test_common_board_calibration import scene


def test_static_chain_recovers_known_mount_and_retains_all_tags():
    cal,x,windows=scene();w=windows[2]
    m=cb.unpack(np.array([.1,-.1,.02,-.1,0,.55]))
    pix=cb.project(cb.MOUNT_OBJECTS,x@m,cal['ego'])
    report=inspect_static(w,range(36),pix,cal)
    assert len(report['per_tag'])==36
    np.testing.assert_allclose(report['whole_board']['mount']['T_Ucolor_mount'],m,atol=1e-5)
    assert report['summary']['delta_whole_p95_mm']<.01
    for row in report['per_tag']:assert row['branches']
    for fold in report['disjoint_folds']:
        assert not set(fold['train_ids'])&set(fold['heldout_ids'])
        assert max(r['p95_px'] for r in fold['prediction'].values())<.01


def test_fault_is_not_pruned_and_wrong_identity_support_rejected():
    cal,x,windows=scene();w=windows[2]
    m=cb.unpack(np.array([.1,-.1,.02,-.1,0,.55]))
    pix=cb.project(cb.MOUNT_OBJECTS,x@m,cal['ego'])
    w['pixels']['ego'][4*7:4*8]+=[4.,0]
    report=inspect_static(w,range(36),pix,cal)
    assert report['per_tag'][7]['id']==7 and report['per_tag'][7]['delta_whole_mm']>1
    assert len(report['per_tag'])==36
    with pytest.raises(ValueError):inspect_static(w,[1]*36,pix,cal)
