"""Validated frontend selection; no changes to pose/identity acceptance gates."""
from dataclasses import replace
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from three_device_slam.spatial import apriltag_detector as detector
from three_device_slam.spatial import mixed_size_crosscheck as mixed
from test_mixed_size_crosscheck import CAL, render, transform


def config(method="none", **kwargs):
    return detector.AprilTagDetectorConfig("tag36h11", .08, (1, 2),
                                         corner_refinement=method, **kwargs)


def detect(image, cfg):
    return detector.detect_apriltags(image, calibration=CAL, config=cfg,
                                     acquisition_timestamp_ns=100,
                                     detector_completed_ns=200)


def test_shared_default_stays_legacy_mixed_default_is_refined():
    assert detector.AprilTagDetectorConfig("tag36h11", .08, (1,)).corner_refinement == "none"
    assert mixed.DEFAULT_CORNER_REFINEMENT == "apriltag"


@pytest.mark.parametrize("value", ["subpix", "auto", "APRILTAG", None, 3, [], True])
def test_invalid_mode_rejected_before_data_access(value):
    with pytest.raises(ValueError, match="corner_refinement"):
        config(value)
    with pytest.raises(ValueError, match="corner_refinement"):
        mixed.compute("/nonexistent/does-not-get-read", corner_refinement=value)


@pytest.mark.parametrize("method", ["none", "apriltag"])
def test_all_three_mixed_detection_passes_receive_explicit_mode(monkeypatch, method):
    calls=[]
    def fake(image, **kwargs):
        calls.append(kwargs['config'])
        return SimpleNamespace(detections=[])
    monkeypatch.setattr(mixed, "detect_apriltags", fake)
    roles, info, _ = mixed.detect_pair(None, None, CAL, CAL, 100, 100,
                                       corner_refinement=method)
    assert not roles and info['reason'] != 'accepted'
    assert [c.tag_size_m for c in calls] == [.08, .04, .08]
    assert all(c.corner_refinement == method for c in calls)
    assert all(c.max_reprojection_error_px == 1.5 for c in calls)
    assert all(c.ambiguity_rotation_deg == 2 for c in calls)


def test_refined_pixels_change_but_metric_scale_and_candidate_binding_remain():
    truth=transform((-.10, -.03, .40))
    image=render([(1, truth, .08)])
    old=next(d for d in detect(image, config()).detections if d.accepted)
    new=next(d for d in detect(image, config('apriltag')).detections if d.accepted)
    assert np.max(np.abs(new.corners_px-old.corners_px)) > .01
    np.testing.assert_allclose(new.observation.camera_from_tag[:3,3],truth[:3,3],atol=.002)
    np.testing.assert_array_equal(new.candidates[new.selected_index].camera_from_tag,
                                   new.observation.camera_from_tag)
    half=next(d for d in detect(image, replace(config('apriltag'),tag_size_m=.04)).detections if d.accepted)
    np.testing.assert_allclose(half.observation.camera_from_tag[:3,3],
                               new.observation.camera_from_tag[:3,3]/2,atol=1e-8)
    assert new.observation.stamped_timestamp_ns == 100


@pytest.mark.parametrize("method", ["none", "apriltag"])
def test_blank_unknown_id_and_reprojection_gates_remain(monkeypatch, method):
    blank=np.full((CAL.height,CAL.width),255,np.uint8)
    assert not detect(blank,config(method)).detections
    image=render([(1,transform((-.10,-.03,.40)),.08)])
    unknown=detect(image,replace(config(method),allowed_tag_ids=(2,))).detections
    assert len(unknown) == 1 and unknown[0].reason == 'unknown_tag_id'
    original=detector._solve_ippe_candidates
    def poor_fit(*args,**kwargs):
        return tuple(replace(c,reprojection_error_px=10.) for c in original(*args,**kwargs))
    monkeypatch.setattr(detector,'_solve_ippe_candidates',poor_fit)
    rejected=detect(image,config(method)).detections
    assert len(rejected) == 1 and rejected[0].reason == 'reprojection_error'
    assert rejected[0].observation is None


def test_cli_segregates_modes_and_refuses_overwrite(tmp_path,monkeypatch):
    seen=[]
    def fake(session,stride,*,corner_refinement):
        seen.append(corner_refinement)
        return dict(verdict='blocked',detector=dict(corner_refinement=corner_refinement)),[]
    monkeypatch.setattr(mixed,'compute',fake)
    monkeypatch.setattr(sys,'argv',['mixed','--session',str(tmp_path)])
    assert mixed.main() == 2
    refined=tmp_path/'spatial/mixed80_40_apriltag_factory_model_v3/report.json'
    assert json.loads(refined.read_text())['detector']['corner_refinement']=='apriltag'
    before=refined.read_bytes()
    monkeypatch.setattr(sys,'argv',['mixed','--session',str(tmp_path),'--corner-refinement','none'])
    assert mixed.main() == 2
    assert (tmp_path/'spatial/mixed80_40_factory_model_v3/report.json').is_file()
    assert refined.read_bytes()==before
    with pytest.raises(FileExistsError):
        mixed.main()
    assert seen==['apriltag','none']
