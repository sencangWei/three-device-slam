import copy

import numpy as np

from scripts import common_board_calibration as cb
from scripts import common_board_capture as cc


def flicker_samples():
    samples = []
    for j in range(6):
        detected = {}
        for role in cb.ROLES:
            # Missing a different row per frame: zero IDs last all6frames,
            # but each has24 well-spread tags in common with the anchor frame.
            detected[role] = {i: np.array([[0.,0],[20,0],[20,20],[0,20]])+[30*(i%6),30*(i//6)]
                              for i in range(36) if i//6 != j}
        samples.append(dict(t=j*.25, arrival_gap_ms=.3, detected=detected, reasons=[]))
    return samples


def test_static_id_flicker_no_longer_implies_motion():
    assert cb.window_reasons(flicker_samples(), 'board') == []
    q = cb.window_quality(flicker_samples(), 'board')
    assert q['roles']['left']['min_anchor_tags'] == 24
    assert q['roles']['left']['max_drift_px'] == 0
    assert q['selected_common_ids']


def test_motion_on_a_flickering_tag_not_discarded():
    ss = flicker_samples()
    ss[4]['detected']['left'][6] += [1., 0.]
    assert 'window_motion' in cb.window_reasons(ss, 'board')


def test_newly_appearing_tag_motion_also_checked():
    ss = flicker_samples()
    # ID0 absent first frame, seen thereafter, must get its own anchor.
    ss[3]['detected']['left'][0] += [1., 0.]
    assert 'window_motion' in cb.window_reasons(ss, 'board')


def test_changing_visibility_cannot_hide_slow_accumulated_motion():
    ss = flicker_samples()
    for j,s in enumerate(ss):
        for quad in s['detected']['left'].values():
            quad += [j*.3,0]
    assert 'window_motion' in cb.window_reasons(ss,'board')


def test_disconnected_or_narrow_anchor_visibility_rejected():
    ss = flicker_samples()
    ss[2]['detected']['left'] = {i:q for i,q in ss[2]['detected']['left'].items() if i//6 < 2}
    assert 'left:anchor_coverage' in cb.window_reasons(ss,'board')


def test_untracked_single_appearance_tag_excluded_from_solver_not_raw():
    ss = flicker_samples()
    for s in ss:
        for role in cb.ROLES:
            s['detected'][role].pop(35,None)
    for role in cb.ROLES:
        ss[3]['detected'][role][35] = np.array([[0.,0],[20,0],[20,20],[0,20]])
    q = cb.window_quality(ss,'board')
    assert not q['reasons']
    assert 35 not in q['selected_common_ids']
    assert 35 in ss[3]['detected']['ego']


def test_nonfinite_corner_never_passes_static_comparison():
    ss = flicker_samples()
    ss[2]['detected']['left'][6][0,0] = np.nan
    assert 'left:invalid_corners' in cb.window_reasons(ss,'board')


def test_mount_id_flicker_still_rejected():
    ss = flicker_samples()
    for s in ss:
        s['detected'] = {'ego':{1:np.array([[0.,0],[80,0],[80,80],[0,80]])},'left':{}}
    ss[2]['detected']['ego'] = {}
    assert 'ego:visibility_changed' in cb.window_reasons(ss,'mount')


def test_ui_separates_visible_from_stable_and_explains_retry():
    q = cb.window_quality(flicker_samples()[:1],'board')
    english,chinese = cc.quality_text(q)
    assert 'WAIT' in english and '不足' in chinese
    q = cb.window_quality(flicker_samples(),'board')
    english,chinese = cc.quality_text(q)
    assert 'READY' in english and '稳定' in chinese and '24' in english
    ss = flicker_samples()
    ss[4]['detected']['left'][6] += [1,0]
    english,chinese = cc.quality_text(cb.window_quality(ss,'board'))
    assert 'HOLD' in english and '漂移' in chinese
