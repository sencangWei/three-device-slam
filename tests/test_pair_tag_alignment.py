"""Offline spatial-link analysis tests (Ego IR frames observing the UMI tag)."""

import json
import math
from zlib import crc32

import numpy as np

from three_device_slam.spatial import pair_tag_alignment as pta
from three_device_slam.spatial.apriltag_detector import CameraCalibration
from three_device_slam.spatial.se3 import transform_from_xyz_rpy


_ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]

TRUTH = transform_from_xyz_rpy((0.05, -0.02, 0.45), (math.pi - 0.1, 0.05, 0.08))
CALIBRATION = CameraCalibration(
    calibration_id="ego.d435i.factory_calibration.v1:327122078613",
    frame_id="ego.ir_left",
    width=1280,
    height=720,
    camera_matrix=np.array(
        ((644.0672, 0.0, 642.2068), (0.0, 644.0672, 367.0062), (0.0, 0.0, 1.0)),
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
        ((656.2879, 0.0, 641.5363), (0.0, 654.4534, 354.3078), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    ),
    distortion_coefficients=np.zeros(5, dtype=np.float64),
)

# Deliberately tilted so planar IPPE ambiguity is not borderline.
TRUTH_UMI_COLOR = transform_from_xyz_rpy(
    (0.05, -0.02, 0.45), (math.pi - 0.45, 0.30, 0.15)
)


def _render_single_tag(camera_from_tag, calibration=CALIBRATION):
    import cv2

    source = cv2.imread(
        str(_ROOT / "assets" / "calibration_tags" / "umi_left_tag36h11_id0_40mm.png"),
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
    warped = cv2.warpPerspective(
        source,
        homography,
        (calibration.width, calibration.height),
        flags=cv2.INTER_NEAREST,
        borderValue=255,
    )
    mask = cv2.warpPerspective(
        np.full(source.shape, 255, dtype=np.uint8),
        homography,
        (calibration.width, calibration.height),
        flags=cv2.INTER_NEAREST,
        borderValue=0,
    )
    image = np.full(
        (calibration.height, calibration.width), 210, dtype=np.uint8
    )
    image[mask > 0] = warped[mask > 0]
    return image


def _write_synthetic_session(tmp_path, *, frames=6, warmup=2):
    session = tmp_path / "session"
    (session / "ego").mkdir(parents=True)
    (session / "ego" / "calibration.json").write_text(
        json.dumps(
            {
                "schema": "ego.d435i.factory_calibration.v1",
                "device": {"serial": "327122078613"},
                "intrinsics": {
                    "ir_left": {
                        "width": 1280,
                        "height": 720,
                        "fx": 644.0672,
                        "fy": 644.0672,
                        "ppx": 642.2068,
                        "ppy": 367.0062,
                        "coefficients": [0.0] * 5,
                    }
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    rng = np.random.default_rng(7)
    payload = bytearray()
    rows = []
    total = frames + warmup
    for index in range(total):
        noise_rpy = rng.normal(0.0, math.radians(0.1), size=3)
        pose = TRUTH @ transform_from_xyz_rpy((0, 0, 0), tuple(noise_rpy))
        image = _render_single_tag(pose)
        offset = len(payload)
        frame = image.tobytes()
        payload.extend(frame)
        rows.append(
            {
                "stream_id": "ego.ir_left",
                "sequence": index,
                "acquisition_ns": 10_000_000_000 + index * 33_333_333,
                "arrival_ns": 10_004_000_000 + index * 33_333_333,
                "clock_domain": "realsense_global_time_mapped_to_host_monotonic",
                "warmup": index < warmup,
                "valid": True,
                "offset": offset,
                "size": len(frame),
                "crc32": crc32(frame) & 0xFFFFFFFF,
                "metadata": {
                    "timestamp_domain": "global_time",
                    "encoding": "Y8",
                    "width": 1280,
                    "height": 720,
                    "clock_rate_ppm": 12.5,
                    "clock_mapping_model": "continuous_per_device",
                },
            }
        )
    with (session / "ego" / "ego.ir_left.jsonl").open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    (session / "ego" / "ego.ir_left.bin").write_bytes(bytes(payload))
    return session


def test_analyze_session_detects_tag_and_measures_stability(tmp_path):
    session = _write_synthetic_session(tmp_path)

    report = pta.analyze_session(session, annotate=2)

    assert report["schema"] == pta.SCHEMA
    assert report["detection_rate"] == 1.0
    assert report["summary"]["observed_tag_ids"] == [0]
    median_t = report["summary"]["reference_camera_from_tag"]["translation_m"]
    assert np.allclose(median_t, TRUTH[:3, 3], atol=0.01)
    # Small synthetic jitter keeps deviations tight.
    assert report["summary"]["translation_max_m"] < 0.01
    assert report["summary"]["rotation_max_deg"] < 1.0
    assert report["observed_clock_mapping"]["clock_rate_ppm"] == 12.5
    report_path = session / "spatial" / "spatial_tag_report.json"
    persisted = json.loads(report_path.read_text(encoding="utf-8"))
    assert persisted["schema"] == pta.SCHEMA
    assert persisted["device_id"] == "ego"
    assert persisted["stream_id"] == "ego.ir_left"
    assert len(persisted["annotated_images"]) == 2
    for rel in persisted["annotated_images"]:
        assert (session / rel).is_file()


def _write_umi_color_session(tmp_path, *, frames=5, warmup=1):
    """Synthetic UMI color (YUYV) session observing the tag on the Ego."""
    import cv2

    session = tmp_path / "session_umi"
    (session / "left").mkdir(parents=True)
    (session / "left" / "calibration.json").write_text(
        json.dumps(
            {
                "schema": "umi.d405.factory_calibration.v1",
                "device": {"serial": "260322279785"},
                "streams": {
                    "color": {
                        "width": 1280,
                        "height": 720,
                        "fx": 656.2879,
                        "fy": 654.4534,
                        "ppx": 641.5363,
                        "ppy": 354.3078,
                        "coefficients": [0.0] * 5,
                        "format": "format.yuyv",
                    }
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    rng = np.random.default_rng(11)
    payload = bytearray()
    rows = []
    total = frames + warmup
    for index in range(total):
        noise_rpy = rng.normal(0.0, math.radians(0.1), size=3)
        pose = TRUTH_UMI_COLOR @ transform_from_xyz_rpy((0, 0, 0), tuple(noise_rpy))
        gray = _render_single_tag(pose, UMI_COLOR_CALIBRATION)
        height, width = gray.shape
        yuyv = np.zeros((height, width, 2), dtype=np.uint8)
        yuyv[:, 0::2, 0] = gray[:, 0::2]
        yuyv[:, 1::2, 0] = gray[:, 1::2]
        yuyv[:, :, 1] = 128
        frame = yuyv.tobytes()
        offset = len(payload)
        payload.extend(frame)
        rows.append(
            {
                "stream_id": "left.color",
                "sequence": index,
                "acquisition_ns": 20_000_000_000 + index * 33_333_333,
                "arrival_ns": 20_004_000_000 + index * 33_333_333,
                "clock_domain": "realsense_global_time_mapped_to_host_monotonic",
                "warmup": index < warmup,
                "valid": True,
                "offset": offset,
                "size": len(frame),
                "crc32": crc32(frame) & 0xFFFFFFFF,
                "metadata": {
                    "timestamp_domain": "global_time",
                    "encoding": "YUYV",
                    "width": 1280,
                    "height": 720,
                    "clock_rate_ppm": -439.1,
                },
            }
        )
    with (session / "left" / "left.color.jsonl").open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    (session / "left" / "left.color.bin").write_bytes(bytes(payload))
    return session


def test_analyze_session_supports_umi_color_reverse_path(tmp_path):
    session = _write_umi_color_session(tmp_path)

    report = pta.analyze_session(
        session, device_id="left", stream_id="left.color", annotate=1
    )

    assert report["device_id"] == "left"
    assert report["stream_id"] == "left.color"
    assert report["detection_rate"] == 1.0
    median_t = report["summary"]["reference_camera_from_tag"]["translation_m"]
    assert np.allclose(median_t, TRUTH_UMI_COLOR[:3, 3], atol=0.01)
    assert report["summary"]["translation_max_m"] < 0.01
    assert report["observed_clock_mapping"]["clock_rate_ppm"] == -439.1
    assert len(report["annotated_images"]) == 1


def test_analyze_session_rejects_wrong_calibration_schema(tmp_path):
    session = _write_synthetic_session(tmp_path)
    calibration_path = session / "ego" / "calibration.json"
    document = json.loads(calibration_path.read_text(encoding="utf-8"))
    document["schema"] = "something.else.v1"
    calibration_path.write_text(json.dumps(document), encoding="utf-8")

    try:
        pta.analyze_session(session)
    except ValueError as error:
        assert "schema" in str(error)
    else:  # pragma: no cover - defensive
        raise AssertionError("expected ValueError")


def test_load_formal_index_rows_skips_warmup(tmp_path):
    session = _write_synthetic_session(tmp_path, frames=4, warmup=3)
    rows = pta.load_formal_index_rows(session, "ego", "ego.ir_left")
    assert len(rows) == 4
    assert all(not row["warmup"] for row in rows)
