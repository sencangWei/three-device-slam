"""Mount calibration tests: common external tag co-observation."""

import json
import math
from zlib import crc32

import numpy as np

from three_device_slam.spatial import mount_calibration as mc
from three_device_slam.spatial import pair_tag_alignment as pta
from three_device_slam.spatial.apriltag_detector import CameraCalibration
from three_device_slam.spatial.se3 import compose, invert, transform_from_xyz_rpy


_ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]

EGO_COLOR_CALIBRATION = CameraCalibration(
    calibration_id="ego.d435i.factory_calibration.v1:327122078613",
    frame_id="ego.color",
    width=1280,
    height=720,
    camera_matrix=np.array(
        ((906.93, 0.0, 659.93), (0.0, 906.87, 377.74), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    ),
    distortion_coefficients=np.zeros(5, dtype=np.float64),
)
UMI_COLOR_CALIBRATION = CameraCalibration(
    calibration_id="umi.d405.factory_calibration.v1:260322279785",
    frame_id="left.color",
    width=1280,
    height=720,
    camera_matrix=np.array(
        ((656.29, 0.0, 641.54), (0.0, 654.45, 354.31), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    ),
    distortion_coefficients=np.zeros(5, dtype=np.float64),
)

# Tilted truths keep planar IPPE ambiguity unambiguous.
TRUTH_EGO_MOUNT = transform_from_xyz_rpy(
    (0.05, -0.02, 0.45), (math.pi - 0.40, 0.25, 0.10)
)
TRUTH_EGO_EXTERNAL = transform_from_xyz_rpy(
    (-0.12, 0.06, 0.55), (math.pi - 0.50, -0.30, -0.20)
)
TRUTH_UMI_EXTERNAL = transform_from_xyz_rpy(
    (0.01, 0.03, 0.50), (math.pi - 0.35, 0.28, -0.15)
)
# Factory color -> ir_left extrinsic (source=color, target=ir_left).
TRUTH_IR_COLOR = transform_from_xyz_rpy((0.015, -0.002, 0.001), (0.01, -0.02, 0.005))


def _render_tags(calibration, tags):
    """tags: tuple of (asset_name, camera_from_tag)."""
    import cv2

    image = np.full((calibration.height, calibration.width), 210, dtype=np.uint8)
    for asset_name, camera_from_tag in tags:
        source = cv2.imread(
            str(_ROOT / "assets" / "calibration_tags" / asset_name),
            cv2.IMREAD_GRAYSCALE,
        )
        assert source is not None
        source = np.rot90(source, 2).copy()
        outer = 0.025
        object_corners = np.array(
            (
                (-outer, outer, 0.0),
                (outer, outer, 0.0),
                (outer, -outer, 0.0),
                (-outer, -outer, 0.0),
            ),
            dtype=np.float64,
        )
        rotation, _ = cv2.Rodrigues(camera_from_tag[:3, :3])
        projected, _ = cv2.projectPoints(
            object_corners,
            rotation,
            camera_from_tag[:3, 3],
            calibration.camera_matrix,
            calibration.distortion_coefficients,
        )
        source_corners = np.array(
            (
                (0.0, 0.0),
                (source.shape[1] - 1.0, 0.0),
                (source.shape[1] - 1.0, source.shape[0] - 1.0),
                (0.0, source.shape[0] - 1.0),
            ),
            dtype=np.float32,
        )
        homography = cv2.getPerspectiveTransform(
            source_corners, projected.reshape(4, 2).astype(np.float32)
        )
        size = (calibration.width, calibration.height)
        warped = cv2.warpPerspective(
            source, homography, size,
            flags=cv2.INTER_NEAREST, borderValue=255,
        )
        mask = cv2.warpPerspective(
            np.full(source.shape, 255, dtype=np.uint8),
            homography, size,
            flags=cv2.INTER_NEAREST, borderValue=0,
        )
        image[mask > 0] = warped[mask > 0]
    return image


def _write_stream(session, device_dir, stream_id, images, start_ns):
    payload = bytearray()
    rows = []
    for index, image in enumerate(images):
        frame = image.tobytes()
        offset = len(payload)
        payload.extend(frame)
        rows.append(
            {
                "stream_id": stream_id,
                "sequence": index,
                "acquisition_ns": start_ns + index * 33_333_333,
                "arrival_ns": start_ns + index * 33_333_333 + 4_000_000,
                "clock_domain": "realsense_global_time_mapped_to_host_monotonic",
                "warmup": index < 1,
                "valid": True,
                "offset": offset,
                "size": len(frame),
                "crc32": crc32(frame) & 0xFFFFFFFF,
                "metadata": {
                    "timestamp_domain": "global_time",
                    "encoding": "Y8",
                    "width": 1280,
                    "height": 720,
                    "clock_rate_ppm": 3.0,
                },
            }
        )
    device = session / device_dir
    device.mkdir(parents=True, exist_ok=True)
    with (device / f"{stream_id}.jsonl").open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    (device / f"{stream_id}.bin").write_bytes(bytes(payload))


def _write_mount_session(tmp_path):
    import cv2

    session = tmp_path / "mount_session"
    session.mkdir()
    (session / "ego").mkdir()
    (session / "left").mkdir()
    (session / "ego" / "calibration.json").write_text(
        json.dumps(
            {
                "schema": "ego.d435i.factory_calibration.v1",
                "device": {"serial": "327122078613"},
                "intrinsics": {
                    "color": {
                        "width": 1280,
                        "height": 720,
                        "fx": 906.93,
                        "fy": 906.87,
                        "ppx": 659.93,
                        "ppy": 377.74,
                        "coefficients": [0.0] * 5,
                    }
                },
                "extrinsics": {
                    "color_to_ir_left": {
                        "rotation_row_major": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        "translation_m": [0.0, 0.0, 0.0],
                    }
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    (session / "left" / "calibration.json").write_text(
        json.dumps(
            {
                "schema": "umi.d405.factory_calibration.v1",
                "device": {"serial": "260322279785"},
                "streams": {
                    "color": {
                        "width": 1280,
                        "height": 720,
                        "fx": 656.29,
                        "fy": 654.45,
                        "ppx": 641.54,
                        "ppy": 354.31,
                        "coefficients": [0.0] * 5,
                    }
                },
                "extrinsics": {
                    "color_to_ir_left": {
                        "rotation_row_major": [
                            round(float(v), 9)
                            for v in TRUTH_IR_COLOR[:3, :3].reshape(-1)
                        ],
                        "translation_m": [
                            round(float(v), 6) for v in TRUTH_IR_COLOR[:3, 3]
                        ],
                    }
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    rng = np.random.default_rng(5)
    ego_images = []
    for _ in range(5):
        jitter = transform_from_xyz_rpy(
            (0, 0, 0),
            tuple(rng.normal(0.0, math.radians(0.05), size=3)),
        )
        ego_images.append(
            _render_tags(
                EGO_COLOR_CALIBRATION,
                (
                    ("umi_right_tag36h11_id1_40mm.png", TRUTH_EGO_MOUNT @ jitter),
                    ("umi_left_tag36h11_id0_40mm.png", TRUTH_EGO_EXTERNAL @ jitter),
                ),
            )
        )
    umi_images = [
        _render_tags(UMI_COLOR_CALIBRATION, (("umi_left_tag36h11_id0_40mm.png", TRUTH_UMI_EXTERNAL),))
        for _ in range(4)
    ]
    _write_stream(session, "ego", "ego.color", ego_images, 10_000_000_000)
    _write_stream(session, "left", "left.color", umi_images, 10_001_000_000)
    return session


def _expected_mount_ir():
    t_ego_umi = compose(TRUTH_EGO_EXTERNAL, invert(TRUTH_UMI_EXTERNAL))
    t_umi_mount = compose(invert(t_ego_umi), TRUTH_EGO_MOUNT)
    return compose(TRUTH_IR_COLOR, t_umi_mount)


def test_mount_calibration_recovers_mount_transform(tmp_path):
    session = _write_mount_session(tmp_path)

    report = mc.compute_mount_calibration(
        session, mount_tag_id=1, external_tag_id=0
    )

    assert report["schema"] == mc.SCHEMA
    assert report["counts"]["ego_mount_observations"] == 4
    assert report["counts"]["ego_external_observations"] == 4
    assert report["counts"]["umi_external_observations"] == 3
    expected = _expected_mount_ir()
    got = report["transforms"]["umi_ir_left_from_mount_tag"]
    assert np.allclose(got["translation_m"], expected[:3, 3], atol=0.005)
    # Each single-tag IPPE observation carries ~1 deg systematic recovery
    # error at synthetic tag sizes; three composed transforms accumulate to
    # a few degrees. Real captures use much larger tags (>=180 px).
    assert np.allclose(
        got["rotation_row_major"],
        np.asarray(expected[:3, :3].reshape(-1)),
        atol=0.06,
    )
    persisted = json.loads(
        (session / "spatial" / "mount_calibration_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert persisted["schema"] == mc.SCHEMA


def _observation(translation, tag_id=1, stamp=1_000_000):
    from three_device_slam.spatial.apriltag_alignment import TagPoseObservation

    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = translation
    return TagPoseObservation(
        stamped_timestamp_ns=stamp,
        detector_completed_ns=stamp + 1_000_000,
        family="tag36h11",
        tag_id=tag_id,
        camera_from_tag=transform,
        reprojection_error_px=0.3,
    )


def test_reject_translation_outliers_removes_double_detect_junk():
    good = [_observation((0.01, -0.02, 0.36), stamp=1_000_000 + i) for i in range(40)]
    junk = [_observation((0.3, 0.1, 0.9), stamp=2_000_000 + i) for i in range(35)]
    kept = mc._reject_translation_outliers(good + junk)
    assert len(kept) == len(good)
    assert all(abs(obs.camera_from_tag[2, 3] - 0.36) < 1e-9 for obs in kept)


def test_reject_translation_outliers_keeps_small_series_untouched():
    series = [_observation((0.0, 0.0, 0.5), stamp=1_000_000 + i) for i in range(3)]
    assert mc._reject_translation_outliers(series) == series


def test_mount_calibration_fails_closed_when_tag_missing(tmp_path):
    session = _write_mount_session(tmp_path)
    try:
        mc.compute_mount_calibration(session, mount_tag_id=1, external_tag_id=7)
    except ValueError as error:
        assert "not observed" in str(error)
    else:  # pragma: no cover - defensive
        raise AssertionError("expected ValueError")
