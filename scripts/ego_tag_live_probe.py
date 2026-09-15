#!/usr/bin/env python3
"""Live probe: find the D435i Ego focus plane and validate tag detection.

Streams the Ego left IR camera, runs tag36h11 detection every frame, and
prints apparent tag size / estimated distance whenever a tag is accepted.
Also prints a rolling Laplacian sharpness measure so the operator can sweep
a target through distances and locate the focus plane.

Usage (venv python, from the repo root):

    /opt/three-device-slam/venv/bin/python scripts/ego_tag_live_probe.py \
        --serial 327122078613 --tag-size-mm 150 --duration 120
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="327122078613")
    parser.add_argument(
        "--stream",
        choices=("ir", "color"),
        default="ir",
        help="ir: left infrared (Y8); color: RGB sensor (sharp on D435i)",
    )
    parser.add_argument("--tag-size-mm", type=float, default=40.0)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    if args.stream == "ir":
        config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
    else:
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    profile = pipeline.start(config)
    if args.stream == "ir":
        intr = (
            profile.get_stream(rs.stream.infrared, 1)
            .as_video_stream_profile()
            .get_intrinsics()
        )
    else:
        intr = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
    fx = float(intr.fx)
    tag_size_m = args.tag_size_mm / 1000.0
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    detector = cv2.aruco.ArucoDetector(
        dictionary, cv2.aruco.DetectorParameters()
    )
    lap_history: list[float] = []
    try:
        from time import monotonic, sleep

        deadline = monotonic() + args.duration + args.warmup
        warmup_until = monotonic() + args.warmup
        frame_index = 0
        while monotonic() < deadline:
            try:
                frames = pipeline.wait_for_frames(1000)
            except RuntimeError:
                continue
            if frames is None:
                continue
            ir = (
                frames.get_infrared_frame(1)
                if args.stream == "ir"
                else frames.get_color_frame()
            )
            if not ir:
                continue
            frame_index += 1
            if monotonic() < warmup_until:
                continue
            image = np.asanyarray(ir.get_data())
            if image.ndim == 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            lap = cv2.Laplacian(image, cv2.CV_64F).var()
            lap_history.append(lap)
            corners, ids, _ = detector.detectMarkers(image)
            marker = ""
            if ids is not None:
                for corner_set, tag_id in zip(corners, ids.ravel()):
                    edge = float(
                        np.linalg.norm(corner_set[0][0] - corner_set[0][1])
                        + np.linalg.norm(corner_set[0][2] - corner_set[0][3])
                    ) / 2.0
                    # black square edge ~ 8/8 of detected quad edge for our
                    # print layout (detector corners surround the black square)
                    distance = fx * tag_size_m / edge
                    marker += (
                        f" TAG id={tag_id} edge_px={edge:.0f}"
                        f" z~{distance:.2f}m"
                    )
            if frame_index % 10 == 0 or marker:
                lap_med = (
                    float(np.median(lap_history[-30:])) if lap_history else 0.0
                )
                print(
                    f"frame={frame_index} lap_var={lap_med:7.1f}{marker}",
                    flush=True,
                )
            sleep(0.001)
    finally:
        pipeline.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
