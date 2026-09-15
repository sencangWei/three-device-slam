from types import SimpleNamespace as NS

import numpy as np
import pytest

from scripts import common_board_calibration as cb


def detection(tag=1, dx=0, size=80):
    return NS(tag_id=tag,corners=np.array([[0.,0],[size,0],[size,size],[0,size]])+[100+dx,100])


def run(monkeypatch, boards, mounts):
    board, mount = object(),object()
    calls = [0]
    def detector(det,image):
        if det is mount:
            return mounts
        calls[0] += 1
        return boards if calls[0] == 1 else []
    monkeypatch.setattr(cb,'detect_grid',detector)
    return cb.detect_sample({r:np.zeros((300,300),np.uint8) for r in cb.ROLES},'mount',board,mount)


def test_same_physical_mount_decoded_by_both_borders_not_board(monkeypatch):
    ds,reasons=run(monkeypatch,[detection(dx=-5,size=90)],[detection()])
    assert not reasons
    np.testing.assert_array_equal(ds['ego'][1],detection().corners)


@pytest.mark.parametrize('boards,mounts',[
    ([detection(dx=160)],[detection()]),
    ([detection(tag=2)],[detection()]),
    ([detection(),detection(tag=2,dx=160)],[detection()]),
    ([detection()],[]),
    ([detection()],[detection(),detection(dx=160)]),
])
def test_other_or_ambiguous_tags_remain_blocked(monkeypatch,boards,mounts):
    ds,reasons=run(monkeypatch,boards,mounts)
    assert 'ego:remove_board_for_mount_stage' in reasons
