#!/usr/bin/env python3
"""Live dual-camera preview with tag overlay for tag placement (local display)."""

import time
import math
import signal
import argparse

import cv2
import numpy as np
import pyrealsense2 as rs

from three_device_slam.spatial.apriltag_detector import (
    AprilTagDetectorConfig, CameraCalibration, detect_apriltags,
)

cv2.setNumThreads(1)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--external-ids", nargs=2, type=int, default=(0, 1))
args = parser.parse_args()
external_ids = tuple(args.external_ids)
if len(set(external_ids)) != 2 or min(external_ids) < 0:
    parser.error("external IDs must be distinct non-negative integers")
pose_config = AprilTagDetectorConfig(
    family="tag36h11", tag_size_m=0.04, allowed_tag_ids=tuple(sorted({1, *external_ids})),
    ambiguity_rotation_deg=2.0, ambiguity_translation_m=0.002,
)
calibrations = {}


def stop_preview(signum, frame):
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, stop_preview)

pipelines = {}
for serial, title in (("327122078613", "EGO (place tags here)"), ("260322279785", "UMI")):
    p = rs.pipeline()
    c = rs.config()
    c.enable_device(serial)
    c.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    profile = p.start(c)
    pipelines[title] = p
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    calibrations[title] = CameraCalibration(
        calibration_id=serial, frame_id=title, width=intr.width, height=intr.height,
        camera_matrix=np.array(((intr.fx, 0, intr.ppx), (0, intr.fy, intr.ppy), (0, 0, 1))),
        distortion_coefficients=np.array(intr.coeffs),
        distortion_model=str(intr.model),
    )
    if title == "UMI":
        sensor = profile.get_device().first_depth_sensor()
        sensor.set_option(rs.option.enable_auto_exposure, 0.0)
        sensor.set_option(rs.option.exposure, 30000.0)
        sensor.set_option(rs.option.gain, 96.0)
    cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow(title, 840 if title == "UMI" else 20, 60)

print("preview started; press q in a window to quit", flush=True)
print(f"PREVIEW: external IDs={external_ids}; mount ID=1", flush=True)
shown = set()
try:
    while True:
        started = time.monotonic()
        for title, p in pipelines.items():
            try:
                frames = p.wait_for_frames(200)
            except RuntimeError:
                missing = np.zeros((450, 800, 3), dtype=np.uint8)
                cv2.putText(missing, "NO FRESH FRAME", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow(title, missing)
                continue
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            stamp = time.monotonic_ns()
            batch = detect_apriltags(
                gray, calibration=calibrations[title], config=pose_config,
                acquisition_timestamp_ns=stamp, detector_completed_ns=stamp,
            )
            small = cv2.resize(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), (800, 450))
            good_ids, id1_ranges = [], []
            for detection in batch.detections:
                corners = detection.corners_px
                size = float(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1).min())
                angle, distance = None, None
                if detection.candidates:
                    index = detection.selected_index
                    if index is None:
                        index = min(range(len(detection.candidates)), key=lambda i: detection.candidates[i].reprojection_error_px)
                    transform = detection.candidates[index].camera_from_tag
                    distance = float(np.linalg.norm(transform[:3, 3]))
                    cosine = abs(float(transform[:3, 2] @ transform[:3, 3])) / distance
                    angle = math.degrees(math.acos(float(np.clip(cosine, 0, 1))))
                good = detection.accepted and size >= 60 and angle is not None and angle < 45
                if good:
                    good_ids.append(detection.tag_id)
                    if detection.tag_id == 1:
                        id1_ranges.append(distance)
                tint = (0, 220, 0) if good else (0, 200, 255)
                points = (corners * (800 / 1280)).astype(np.int32)
                cv2.polylines(small, [points], True, tint, 2)
                label = f"id{detection.tag_id}: {size:.0f}px"
                if angle is not None:
                    label += f" {angle:.1f}deg"
                if not detection.accepted:
                    label += " POSE?"
                origin = (int(np.clip(points[:, 0].min(), 5, 480)), int(np.clip(points[:, 1].min() - 8, 70, 430)))
                cv2.putText(small, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.55, tint, 2)
            required = sorted(external_ids if title == "UMI" else (*external_ids, 1))
            ready = sorted(good_ids) == required
            if title != "UMI" and ready and 1 in external_ids:
                ready = max(id1_ranges) - min(id1_ranges) > 0.10
            cv2.rectangle(small, (0, 0), (800, 56), (20, 20, 20), -1)
            status = "GEOMETRY OK" if ready else "ADJUST / CHECK MISSING TAG"
            tint = (0, 220, 0) if ready else (0, 200, 255)
            cv2.putText(small, status + " | >=60px, <45deg", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, tint, 2)
            hint = f"Need: external id{external_ids[0]} + external id{external_ids[1]}" + (" + back id1" if title != "UMI" else "")
            cv2.putText(small, hint + " | q: close", (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            cv2.imshow(title, small)
            if title not in shown:
                print(f"PREVIEW: {title} live frames displayed.", flush=True)
                shown.add(title)
        delay = max(1, int(100 - (time.monotonic() - started) * 1000))
        if cv2.waitKey(delay) & 0xFF in (ord("q"), 27):
            break
except KeyboardInterrupt:
    pass
finally:
    for p in pipelines.values():
        try:
            p.stop()
        except Exception:
            pass
    cv2.destroyAllWindows()
    print("PREVIEW: closed; camera ownership released.", flush=True)
