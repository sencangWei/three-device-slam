"""Offline Ego<->UMI spatial-link analysis via an AprilTag mounted on one rig.

Physical setup (temporary D435i Ego + single D405 UMI): a printed tag36h11
tag is rigidly attached to one device and observed by the other camera,
yielding a time series ``T_observer_tag(t)``. While the rig is static this
transform is constant; its temporal stability is the measurable evidence
that both devices live in one common spatial frame.

Two practical observation paths exist:

* default: the Ego left IR camera observes a tag on the UMI's back;
* reversed: the UMI D405 color camera observes a tag mounted on the Ego
  (preferred when the Ego lens is too soft to decode the tag pixels).

This module is an offline analysis tool over a sealed pair session. It reads
the observer's append-only stream (jsonl index + bin payloads), runs the
OpenCV reference AprilTag detector, and publishes a JSON report plus
annotated images under ``<session>/spatial/``.

Completing the chain to the other device's camera frame additionally
requires the one-time tag-to-host mount transform, which is a product
calibration artifact and is not estimated here.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from .apriltag_alignment import TagPoseObservation
from .apriltag_detector import (
    AprilTagDetectorConfig,
    CameraCalibration,
    detect_apriltags,
)
from .se3 import pose_error


SCHEMA = "ego.two_device.spatial_tag.v1"
EGO_CALIBRATION_SCHEMA = "ego.d435i.factory_calibration.v1"
UMI_CALIBRATION_SCHEMA = "umi.d405.factory_calibration.v1"
DEFAULT_DEVICE_ID = "ego"
DEFAULT_STREAM_ID = "ego.ir_left"
DEFAULT_TAG_SIZE_M = 0.040
DEFAULT_ALLOWED_TAG_IDS = (0, 1)


def _camera_calibration_from_intrinsics(
    *, calibration_id: str, frame_id: str, intrinsics: dict
) -> CameraCalibration:
    coefficients = intrinsics.get("coefficients") or [0.0] * 5
    camera = np.array(
        (
            (intrinsics["fx"], 0.0, intrinsics["ppx"]),
            (0.0, intrinsics["fy"], intrinsics["ppy"]),
            (0.0, 0.0, 1.0),
        ),
        dtype=np.float64,
    )
    return CameraCalibration(
        calibration_id=calibration_id,
        frame_id=frame_id,
        width=int(intrinsics["width"]),
        height=int(intrinsics["height"]),
        camera_matrix=camera,
        distortion_coefficients=np.array(coefficients, dtype=np.float64),
        distortion_model=intrinsics.get("distortion_model", "opencv_brown_conrady"),
    )


def load_device_calibration(
    session: Path, device_id: str, stream_id: str
) -> CameraCalibration:
    """Build the detector calibration from a sealed device calibration.json."""
    suffix = stream_id.split(".", 1)[1]
    path = Path(session) / device_id / "calibration.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    if device_id == "ego":
        if document.get("schema") != EGO_CALIBRATION_SCHEMA:
            raise ValueError(f"unsupported ego calibration schema in {path}")
        intrinsics = document["intrinsics"][suffix]
        serial = document.get("device", {}).get("serial", "unknown")
        calibration_id = f"{EGO_CALIBRATION_SCHEMA}:{serial}"
    elif device_id in ("left", "right"):
        if document.get("schema") != UMI_CALIBRATION_SCHEMA:
            raise ValueError(f"unsupported UMI calibration schema in {path}")
        intrinsics = document["streams"][suffix]
        serial = document.get("device", {}).get("serial", "unknown")
        calibration_id = f"{UMI_CALIBRATION_SCHEMA}:{serial}"
    else:
        raise ValueError(f"unsupported device_id: {device_id}")
    return _camera_calibration_from_intrinsics(
        calibration_id=calibration_id, frame_id=stream_id, intrinsics=intrinsics
    )


def load_ego_ir_left_calibration(session: Path) -> CameraCalibration:
    """Backward-compatible helper for the default Ego IR-left stream."""
    return load_device_calibration(session, DEFAULT_DEVICE_ID, DEFAULT_STREAM_ID)


def load_formal_index_rows(
    session: Path, device_id: str, stream_id: str
) -> list[dict]:
    """Load non-warmup, valid append-only index rows for one stream."""
    path = Path(session) / device_id / f"{stream_id}.jsonl"
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if row.get("stream_id") != stream_id:
                raise ValueError(f"stream_id mismatch in {path}")
            if row.get("warmup") or not row.get("valid"):
                continue
            rows.append(row)
    rows.sort(key=lambda row: row["acquisition_ns"])
    return rows


def load_frame_image(
    session: Path, device_id: str, stream_id: str, row: dict
) -> np.ndarray:
    """Decode one Y8 or YUYV payload from the contiguous bin archive."""
    metadata = row.get("metadata", {})
    width = int(metadata.get("width") or 0)
    height = int(metadata.get("height") or 0)
    if width <= 0 or height <= 0:
        raise ValueError("index row is missing frame dimensions")
    offset = int(row["offset"])
    size = int(row["size"])
    path = Path(session) / device_id / f"{stream_id}.bin"
    with path.open("rb") as handle:
        handle.seek(offset)
        payload = handle.read(size)
    encoding = str(metadata.get("encoding") or "Y8")
    if encoding == "Y8":
        expected = width * height
        if len(payload) != expected:
            raise ValueError(
                f"payload size {len(payload)} does not match {width}x{height}"
            )
        return np.frombuffer(payload, dtype=np.uint8).reshape(height, width).copy()
    if encoding == "YUYV":
        import cv2

        expected = width * height * 2
        if len(payload) != expected:
            raise ValueError(
                f"payload size {len(payload)} does not match {width}x{height} YUYV"
            )
        yuyv = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2GRAY_YUY2)
    raise ValueError(f"unsupported frame encoding: {encoding}")


def detect_tag_in_frames(
    session: Path,
    rows: list[dict],
    *,
    device_id: str,
    stream_id: str,
    calibration: CameraCalibration,
    config: AprilTagDetectorConfig,
    stride: int = 1,
) -> list[TagPoseObservation]:
    """Run the reference detector over formal frames and collect observations."""
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError("stride must be a positive integer")
    observations = []
    for row in rows[::stride]:
        image = load_frame_image(session, device_id, stream_id, row)
        completed = time.monotonic_ns()
        batch = detect_apriltags(
            image,
            calibration=calibration,
            config=config,
            acquisition_timestamp_ns=int(row["acquisition_ns"]),
            detector_completed_ns=completed,
        )
        for detection in batch.detections:
            if detection.accepted:
                observations.append(detection.observation)
    return observations


def reference_transform(observations: list[TagPoseObservation]) -> np.ndarray:
    """Robust reference pose: translation-wise geometric median sample."""
    translations = np.array(
        [obs.camera_from_tag[:3, 3] for obs in observations], dtype=np.float64
    )
    median_translation = np.median(translations, axis=0)
    best = min(
        observations,
        key=lambda obs: float(
            np.linalg.norm(obs.camera_from_tag[:3, 3] - median_translation)
        ),
    )
    return best.camera_from_tag


# Backward-compatible private alias.
_reference_transform = reference_transform


def summarize_pose_series(
    observations: list[TagPoseObservation],
) -> dict:
    """Temporal stability statistics of T_egoIrLeft_tag(t)."""
    if not observations:
        raise ValueError("no accepted tag observations")
    reference = _reference_transform(observations)
    errors = [pose_error(obs.camera_from_tag, reference) for obs in observations]
    translations = np.array(
        [obs.camera_from_tag[:3, 3] for obs in observations], dtype=np.float64
    )
    reprojections = [obs.reprojection_error_px for obs in observations]
    tag_ids = sorted({obs.tag_id for obs in observations})

    def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
        sy = math.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
        singular = sy < 1e-9
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = (
            math.atan2(-rotation[2, 0], sy)
            if not singular
            else math.atan2(-rotation[2, 0], 0.0)
        )
        yaw = (
            math.atan2(rotation[1, 0], rotation[0, 0])
            if not singular
            else math.atan2(-rotation[1, 2], rotation[1, 1])
        )
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    roll, pitch, yaw = _rpy(reference[:3, :3])
    return {
        "accepted_observations": len(observations),
        "observed_tag_ids": tag_ids,
        "reference_camera_from_tag": {
            "translation_m": [round(float(v), 6) for v in reference[:3, 3]],
            "rotation_rpy_deg": [round(v, 3) for v in (roll, pitch, yaw)],
            "rotation_row_major": [
                round(float(v), 9) for v in np.asarray(reference[:3, :3]).reshape(-1)
            ],
        },
        "translation_rms_m": round(
            float(np.sqrt(np.mean([e.translation_m ** 2 for e in errors]))), 6
        ),
        "translation_max_m": round(max(e.translation_m for e in errors), 6),
        "rotation_rms_deg": round(
            float(np.sqrt(np.mean([e.rotation_deg ** 2 for e in errors]))), 4
        ),
        "rotation_max_deg": round(max(e.rotation_deg for e in errors), 4),
        "reprojection_median_px": round(float(np.median(reprojections)), 4),
        "reprojection_max_px": round(max(reprojections), 4),
        "translation_component_std_m": [
            round(float(v), 6) for v in np.std(translations, axis=0)
        ],
    }


def _annotate_frame(
    image: np.ndarray,
    observation: TagPoseObservation,
    corners_px: np.ndarray,
) -> np.ndarray:
    import cv2

    annotated = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    points = corners_px.reshape(-1, 1, 2).astype(np.int32)
    cv2.polylines(annotated, [points], True, (0, 255, 0), 2)
    t = observation.camera_from_tag[:3, 3]
    label = (
        f"id={observation.tag_id} z={t[2]:.3f}m "
        f"rerr={observation.reprojection_error_px:.2f}px"
    )
    cv2.putText(
        annotated,
        label,
        (int(corners_px[0][0]), max(20, int(corners_px[0][1]) - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )
    return annotated


def write_annotated_images(
    session: Path,
    rows: list[dict],
    observations: list[TagPoseObservation],
    *,
    device_id: str,
    stream_id: str,
    calibration: CameraCalibration,
    tag_size_m: float,
    allowed_tag_ids: tuple[int, ...],
    limit: int = 6,
) -> list[str]:
    """Save annotated PNGs for the first `limit` accepted observations."""
    import cv2

    output = Path(session) / "spatial" / "annotated"
    output.mkdir(parents=True, exist_ok=True)
    by_stamp = {obs.stamped_timestamp_ns: obs for obs in observations}
    written = []
    for row in rows:
        stamp = int(row["acquisition_ns"])
        observation = by_stamp.get(stamp)
        if observation is None:
            continue
        image = load_frame_image(session, device_id, stream_id, row)
        batch = detect_apriltags(
            image,
            calibration=calibration,
            config=AprilTagDetectorConfig(
                family="tag36h11",
                tag_size_m=tag_size_m,
                allowed_tag_ids=allowed_tag_ids,
            ),
            acquisition_timestamp_ns=stamp,
            detector_completed_ns=time.monotonic_ns(),
        )
        corners = None
        for detection in batch.detections:
            if detection.accepted:
                corners = detection.corners_px
                break
        if corners is None:
            continue
        name = f"annotated_{device_id}_{stamp}.png"
        cv2.imwrite(
            str(output / name), _annotate_frame(image, observation, corners)
        )
        written.append(f"spatial/annotated/{name}")
        if len(written) >= limit:
            break
    return written


def analyze_session(
    session: Path,
    *,
    device_id: str = DEFAULT_DEVICE_ID,
    stream_id: str = DEFAULT_STREAM_ID,
    tag_size_m: float = DEFAULT_TAG_SIZE_M,
    allowed_tag_ids: tuple[int, ...] = DEFAULT_ALLOWED_TAG_IDS,
    stride: int = 1,
    annotate: int = 6,
) -> dict:
    """Detect the rig-mounted tag and measure temporal spatial stability.

    ``device_id``/``stream_id`` select the observing camera. The default is
    the Ego IR-left stream observing a tag on the UMI; the practical reverse
    path is ``device_id="left", stream_id="left.color"`` — the D405 color
    camera observing a tag mounted on the Ego (used when the Ego lens is too
    soft for tag decoding).
    """
    session = Path(session)
    calibration = load_device_calibration(session, device_id, stream_id)
    rows = load_formal_index_rows(session, device_id, stream_id)
    if not rows:
        raise ValueError(f"no formal rows for {stream_id}")
    config = AprilTagDetectorConfig(
        family="tag36h11",
        tag_size_m=tag_size_m,
        allowed_tag_ids=tuple(allowed_tag_ids),
        # Static-rig analysis: a rigid mount cannot drift degrees between
        # frames, so planar IPPE ambiguities are rejected aggressively.
        ambiguity_rotation_deg=2.0,
        ambiguity_translation_m=0.002,
    )
    observations = detect_tag_in_frames(
        session,
        rows,
        device_id=device_id,
        stream_id=stream_id,
        calibration=calibration,
        config=config,
        stride=stride,
    )
    summary = summarize_pose_series(observations)
    annotated = (
        write_annotated_images(
            session,
            rows,
            observations,
            device_id=device_id,
            stream_id=stream_id,
            calibration=calibration,
            tag_size_m=tag_size_m,
            allowed_tag_ids=tuple(allowed_tag_ids),
            limit=annotate,
        )
        if annotate
        else []
    )
    first_meta = rows[0].get("metadata", {})
    report = {
        "schema": SCHEMA,
        "session": session.name,
        "device_id": device_id,
        "stream_id": stream_id,
        "calibration_id": calibration.calibration_id,
        "detector_backend": "opencv_aruco_reference",
        "evidence_class": "real_capture",
        "tag_family": "tag36h11",
        "tag_size_m": tag_size_m,
        "frames_processed": len(rows[::stride]),
        "detection_rate": round(len(observations) / len(rows[::stride]), 4),
        "observed_clock_mapping": {
            "clock_rate_ppm": first_meta.get("clock_rate_ppm"),
            "clock_mapping_model": first_meta.get("clock_mapping_model"),
        },
        "summary": summary,
        "annotated_images": annotated,
        "chain_note": (
            "Full ego-to-UMI chain: T_ego_umi = T_ego_tag * T_tag_umi when "
            "the Ego observes the tag, or T_umi_ego = T_umi_tag * "
            "inv(T_ego_tag) when the UMI observes a tag mounted on the Ego. "
            "The tag-to-host mount transform is a one-time product "
            "calibration artifact and is not estimated by this analysis."
        ),
    }
    output = session / "spatial"
    output.mkdir(parents=True, exist_ok=True)
    (output / "spatial_tag_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    return report


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
    )
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE_ID,
        choices=("ego", "left", "right"),
        help="observing device directory",
    )
    parser.add_argument(
        "--stream",
        default=DEFAULT_STREAM_ID,
        help="observing stream id, e.g. ego.ir_left or left.color",
    )
    parser.add_argument("--tag-size-mm", type=float, default=40.0)
    parser.add_argument(
        "--allowed-ids", type=int, nargs="+", default=list(DEFAULT_ALLOWED_TAG_IDS)
    )
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--annotate", type=int, default=6)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    report = analyze_session(
        args.session,
        device_id=args.device,
        stream_id=args.stream,
        tag_size_m=args.tag_size_mm / 1000.0,
        allowed_tag_ids=tuple(args.allowed_ids),
        stride=args.stride,
        annotate=args.annotate,
    )
    print(json.dumps(report["summary"], indent=2, sort_keys=True))
    print(f"report={args.session / 'spatial' / 'spatial_tag_report.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
