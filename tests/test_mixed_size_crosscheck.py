"""Mixed physical sizes and duplicate ID identity must not corrupt scale."""
from dataclasses import replace
import json

import cv2
import numpy as np
import pytest

from three_device_slam.spatial import mixed_size_crosscheck as m
from three_device_slam.spatial.apriltag_alignment import TagPoseObservation
from three_device_slam.spatial.apriltag_detector import AprilTagImageDetection, PnPCandidate
from three_device_slam.spatial.se3 import transform_from_xyz_rpy, invert
from test_mount_calibration import EGO_COLOR_CALIBRATION as CAL, _write_stream


def transform(xyz):
    return transform_from_xyz_rpy(xyz, (np.pi - 0.3, 0.12, 0.03))


def detection(tag_id, pose, size, corners=None):
    if corners is None:
        h = size / 2
        points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
        corners = cv2.projectPoints(points, cv2.Rodrigues(pose[:3, :3])[0],
                                   pose[:3, 3], CAL.camera_matrix, CAL.distortion_coefficients)[0].reshape(4, 2)
    obs = TagPoseObservation(1, 1, "tag36h11", tag_id, pose, 0.01)
    return AprilTagImageDetection(tag_id, corners, (PnPCandidate(pose, 0.01, True),), 0, "accepted", obs)


def fixture():
    # Physical mount range .26m; external .40m. Incorrectly solving the mount
    # as 80mm yields .52m, INVERTING the old nearest-range identity heuristic.
    mount = detection(1, transform((0.075, 0.035, 0.26)), .04)
    wrong_pose = m._pose(mount).copy()
    wrong_pose[:3, 3] *= 2
    wrong_mount = detection(1, wrong_pose, .08, mount.corners_px)
    ext1 = detection(1, transform((-.10, -.03, .40)), .08)
    ext2 = detection(2, transform((.14, -.06, .50)), .08)
    wrong_external = m._pose(ext1).copy()
    wrong_external[:3, 3] /= 2
    e40 = [mount, detection(1, wrong_external, .04, ext1.corners_px)]
    ego_from_umi = np.eye(4)
    ego_from_umi[2, 3] = .15
    umi = [detection(d.tag_id, invert(ego_from_umi) @ m._pose(d), .08)
           for d in (ext1, ext2)]
    return [ext1, wrong_mount, ext2], e40, umi, mount


def test_identity_survives_inverted_wrong_size_ranges():
    e80, e40, u80, mount = fixture()
    roles, info = m.associate(e80, e40, u80, CAL)
    assert info["reason"] == "accepted"
    assert roles["ego_mount"] is mount
    assert roles["ego_ext1"] is e80[0]
    assert np.linalg.norm(m._pose(e80[1])[:3, 3]) > np.linalg.norm(m._pose(e80[0])[:3, 3])
    assert m.geometry(roles)["roles"]["ego_mount"]["size_mm"] == 40


@pytest.mark.parametrize("failure", ["missing_anchor", "duplicate_anchor", "ambiguous", "missing_mount"])
def test_identity_fails_closed(failure):
    e80, e40, u80, _ = fixture()
    if failure == "missing_anchor":
        u80 = u80[:1]
    elif failure == "duplicate_anchor":
        u80.append(u80[1])
    elif failure == "ambiguous":
        e80[1] = replace(e80[1], corners_px=e80[0].corners_px + (10, 0))
    else:
        e40 = []
    roles, info = m.associate(e80, e40, u80, CAL)
    assert not roles
    assert info["reason"] != "accepted"


def test_chains_recover_mount_and_keep_independent_errors():
    e80, e40, u80, mount = fixture()
    roles, _ = m.associate(e80, e40, u80, CAL)
    series = {k: [m._pose(v)] * 8 for k, v in roles.items()}
    report = m.summarize_chains(series, np.eye(4))
    np.testing.assert_allclose(report["consensus_umi_ir_left_from_mount_tag"]["translation_m"],
                               m._pose(mount)[:3, 3] - [0, 0, .15], atol=1e-6)
    perturbed = m._pose(roles["ego_ext1"]).copy()
    perturbed[0, 3] += .012
    series["ego_ext1"] = [perturbed] * 8
    report = m.summarize_chains(series, np.eye(4))
    assert report["agreement"]["mount_full_translation_difference_m"] == pytest.approx(.012)
    # z-only legacy agreement is explicitly NOT a full-3D 5mm accuracy gate.
    assert report["chain_verdict"] == "pass"


def test_mean_supports_exact_180_degree_rotation():
    t = transform_from_xyz_rpy((0, 0, .2), (np.pi, 0, 0))
    np.testing.assert_allclose(m.mean_pose([t, t]), t, atol=1e-12)


def test_crc_and_timestamp_corruption_are_detected(tmp_path):
    images = [np.zeros((720, 1280), np.uint8)] * 10
    _write_stream(tmp_path, "ego", "ego.color", images, 100000)
    rows, audit = m.audit_stream(tmp_path, "ego")
    assert audit["bad_payloads"] == 0
    path = tmp_path / "ego" / "ego.color.jsonl"
    document = [json.loads(line) for line in path.read_text().splitlines()]
    document[2]["crc32"] = 123
    document[3]["acquisition_ns"] = document[2]["acquisition_ns"] - 1
    path.write_text("\n".join(json.dumps(r) for r in document))
    _, audit = m.audit_stream(tmp_path, "ego")
    assert audit["bad_payloads"] == 1
    assert audit["nonincreasing_timestamps"] == 1


@pytest.mark.parametrize("clock", [None, "device_clock"])
def test_aligned_numbers_without_host_clock_provenance_block(tmp_path, clock):
    for device in ("ego", "left"):
        _write_stream(tmp_path, device, f"{device}.color", [np.zeros((720, 1280), np.uint8)] * 10, 100000)
    path = tmp_path / "left" / "left.color.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    for row in rows:
        row["clock_domain"] = clock
    path.write_text("\n".join(json.dumps(r) for r in rows))
    report, _ = m.compute(tmp_path)
    assert report["verdict"] == "blocked"
    assert report["streams"]["umi"]["unsupported_clock_rows"] == 9


def render(tags):
    image = np.full((CAL.height, CAL.width), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id, pose, size in tags:
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 400)
        corners = detection(tag_id, pose, size).corners_px.astype(np.float32)
        homography = cv2.getPerspectiveTransform(np.float32([[0, 0], [399, 0], [399, 399], [0, 399]]), corners)
        warped = cv2.warpPerspective(marker, homography, (CAL.width, CAL.height), borderValue=255)
        image = np.minimum(image, warped)
    return image


@pytest.mark.parametrize("corner_refinement", ["none", "apriltag"])
def test_rendered_mixed_size_detector_recovers_metric_mount(corner_refinement):
    mount = transform((.075, .035, .26))
    ext1, ext2 = transform((-.10, -.03, .40)), transform((.14, -.06, .50))
    ego = render([(1, mount, .04), (1, ext1, .08), (2, ext2, .08)])
    ego_from_umi = np.eye(4)
    ego_from_umi[2, 3] = .15
    umi = render([(1, invert(ego_from_umi) @ ext1, .08),
                  (2, invert(ego_from_umi) @ ext2, .08)])
    roles, info, _ = m.detect_pair(ego, umi, CAL, CAL, 1, 1,
                                 corner_refinement=corner_refinement)
    assert info["reason"] == "accepted"
    np.testing.assert_allclose(m._pose(roles["ego_mount"])[:3, 3], mount[:3, 3], atol=.002)
    np.testing.assert_allclose(m._pose(roles["ego_ext1"])[:3, 3], ext1[:3, 3], atol=.002)


def add_id1_candidate(e80, e40, xyz):
    correct = detection(1, transform(xyz), .04)
    wrong_pose = m._pose(correct).copy()
    wrong_pose[:3, 3] *= 2
    e40.append(correct)
    e80.append(detection(1, wrong_pose, .08, correct.corners_px))
    return correct


def test_extra_far_id1_is_ignored_without_changing_chain():
    e80, e40, u80, mount = fixture()
    baseline, _ = m.associate(e80, e40, u80, CAL)
    extra = add_id1_candidate(e80, e40, (.16, .05, .65))
    roles, info = m.associate(e80, e40, u80, CAL)
    assert info['reason'] == 'accepted'
    assert roles['ego_mount'] is mount
    assert info['ignored_ego_id1_count'] == 1
    assert sum(x['plausible_mount'] for x in info['mount_candidate_checks']) == 1
    for role in m.ROLES:
        np.testing.assert_array_equal(m._pose(roles[role]), m._pose(baseline[role]))


def test_two_plausible_mounts_reject_instead_of_picking_nearest():
    e80, e40, u80, _ = fixture()
    add_id1_candidate(e80, e40, (.035, .03, .20))
    roles, info = m.associate(e80, e40, u80, CAL)
    assert not roles
    assert info['reason'] == 'ambiguous_mount_identity'
    labels = m.preview_resolved_roles(roles, info, {'ego': e80, 'umi': u80})
    assert set(labels) == {'ego_ext1', 'ego_ext2', 'umi_ext1', 'umi_ext2'}
    assert 'ego_mount' not in labels


def test_missing_mount_does_not_promote_far_spare_tag():
    e80, e40, u80, mount = fixture()
    e80.pop(1)
    e40.remove(mount)
    add_id1_candidate(e80, e40, (.16, .05, .65))
    roles, info = m.associate(e80, e40, u80, CAL)
    assert not roles and info['reason'] == 'no_mount_near_umi'


@pytest.mark.parametrize('failure', ['rejected_pose', 'unmatched_quad'])
def test_unresolved_true_mount_does_not_promote_nearby_spare(failure):
    e80, e40, u80, mount = fixture()
    add_id1_candidate(e80, e40, (.035, .03, .20))
    if failure == 'rejected_pose':
        e40[0] = replace(mount, selected_index=None, observation=None,
                         reason='planar_pose_ambiguity')
    else:
        e40.pop(0)
    roles, info = m.associate(e80, e40, u80, CAL)
    assert not roles and info['reason'] == 'unresolved_mount_candidate'
