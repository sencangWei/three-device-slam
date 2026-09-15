import copy
import numpy as np
import pytest
from scripts import common_board_calibration as cb
from test_common_board_joint import scene


def test_restore_board_id1_not_external_mount_and_never_guess_duplicate():
    from scripts.common_board_resampling_analysis import restore_board_id1
    board,mount=scene()
    quad,proof=restore_board_id1(board+[mount])
    np.testing.assert_allclose(quad,board[1].corners)
    assert proof['inside_candidates']==1
    for raw in (board[:1]+board[2:]+[mount],board+[copy.deepcopy(board[1]),mount]):
        with pytest.raises(ValueError):restore_board_id1(raw)
    bad=copy.deepcopy(board);bad[1].corners+=[5,0]
    with pytest.raises(ValueError):restore_board_id1(bad+[mount])


def test_groups_frozen_geometry_not_fitted_error():
    from scripts.common_board_resampling_analysis import groups
    g=groups()
    assert len(g)==68 and g['all36']==list(range(36)) and 1 not in g['all35']
    for name in ('checker','horizontal','vertical'):
        a,b=set(g[name+'_0']),set(g[name+'_1'])
        assert not a&b and a|b==set(range(36)) and len(a)==len(b)==18
    for i in range(36):assert g[f'without_tag_{i}']==[j for j in range(36) if j!=i]


def test_group_fit_synthetic_truth_and_heldout_fault_not_refitted():
    from scripts.common_board_resampling_analysis import fit_group
    from test_common_board_calibration import scene as calibrated
    cal,x,windows=calibrated();w=windows[2]
    m=cb.unpack(np.array([.1,-.1,.02,-.1,0,.55]));em=x@m
    selected=list(range(18))
    result=fit_group(w,selected,cal,em)
    np.testing.assert_allclose(result['mount']['T_Ucolor_mount'],m,atol=1e-5)
    w['pixels']['ego'][4*30:4*31]+=[8,0]
    bad=fit_group(w,selected,cal,em)
    np.testing.assert_array_equal(bad['mount']['T_Ucolor_mount'],result['mount']['T_Ucolor_mount'])
    assert bad['heldout']['ego']['max_px']>7
