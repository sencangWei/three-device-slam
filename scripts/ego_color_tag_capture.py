#!/usr/bin/env python3
"""Standalone Ego-color tag capture for the spatial-link bench validation.

The temporary D435i Ego's IR imagers are too soft for AprilTag decoding, so
this script records the sharp RGB sensor instead. It writes a minimal
append-only session (``ego/ego.color`` jsonl + bin + calibration.json) that
``three_device_slam.spatial.pair_tag_alignment`` can consume directly with
``--device ego --stream ego.color``.

Frames are stored as grayscale Y8 (the detector works on gray pixels).

Usage (venv python, from the repo root):

    /opt/three-device-slam/venv/bin/python scripts/ego_color_tag_capture.py \
        --serial 327122078613 --session /path/to/session --duration 20
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from zlib import crc32

import numpy as np


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="327122078613")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    import time

    import cv2
    import pyrealsense2 as rs

    from three_device_slam.synchronization.clock_mapping import DeviceClockTracker

    args = parse_args(argv)
    session = args.session.resolve()
    if session.exists():
        raise FileExistsError(f"session already exists: {session}")
    (session / "ego").mkdir(parents=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    profile = pipeline.start(config)
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()

    # startup wall->monotonic intercept, same model as rsusb_pair
    candidates = []
    for _ in range(9):
        wall_before = time.time_ns()
        monotonic = time.monotonic_ns()
        wall_after = time.time_ns()
        candidates.append((wall_before + wall_after) // 2 - monotonic)
    intercept_ns = min(candidates)
    tracker = DeviceClockTracker(
        intercept_ns=intercept_ns, expected_frame_delta_ns=33_333_333
    )

    calibration = {
        "schema": "ego.d435i.factory_calibration.v1",
        "device": {
            "serial": args.serial,
            "name": profile.get_device().get_info(rs.camera_info.name),
            "firmware": profile.get_device().get_info(
                rs.camera_info.firmware_version
            ),
        },
        "intrinsics": {
            "color": {
                "width": int(intr.width),
                "height": int(intr.height),
                "fx": float(intr.fx),
                "fy": float(intr.fy),
                "ppx": float(intr.ppx),
                "ppy": float(intr.ppy),
                "coefficients": [float(c) for c in intr.coeffs],
                "distortion_model": str(intr.model),
            }
        },
        "note": "color stream used for AprilTag observation (IR imagers soft)",
    }
    (session / "ego" / "calibration.json").write_text(
        json.dumps(calibration, indent=2, sort_keys=True), encoding="utf-8"
    )

    payload = bytearray()
    rows = []
    warmup_until = time.monotonic() + args.warmup
    deadline = warmup_until + args.duration
    index = 0
    while time.monotonic() < deadline:
        frames = pipeline.wait_for_frames(1000)
        if frames is None:
            continue
        color = frames.get_color_frame()
        if not color:
            continue
        acquisition_ms = float(color.get_timestamp())
        domain = str(color.get_frame_timestamp_domain()).rsplit(".", 1)[-1]
        valid = domain == "global_time"
        if not valid:
            continue
        tracker.add_sample(index, acquisition_ms)
        gray = cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2GRAY)
        frame = gray.tobytes()
        offset = len(payload)
        payload.extend(frame)
        rows.append(
            {
                "stream_id": "ego.color",
                "sequence": index,
                "acquisition_ns": tracker.map_acquisition_ns(acquisition_ms),
                "arrival_ns": time.monotonic_ns(),
                "clock_domain": "realsense_global_time_mapped_to_host_monotonic",
                "warmup": time.monotonic() < warmup_until,
                "valid": True,
                "offset": offset,
                "size": len(frame),
                "crc32": crc32(frame) & 0xFFFFFFFF,
                "metadata": {
                    "timestamp_domain": "global_time",
                    "device_timestamp_ms": acquisition_ms,
                    "encoding": "Y8",
                    "width": 1280,
                    "height": 720,
                    **tracker.mapping_evidence(),
                },
            }
        )
        index += 1
    pipeline.stop()

    with (session / "ego" / "ego.color.jsonl").open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    (session / "ego" / "ego.color.bin").write_bytes(bytes(payload))
    formal = [row for row in rows if not row["warmup"]]
    print(f"session={session}")
    print(f"frames_total={len(rows)} formal={len(formal)}")
    if formal:
        span_s = (formal[-1]["acquisition_ns"] - formal[0]["acquisition_ns"]) / 1e9
        print(f"formal_span_s={span_s:.2f} hz={len(formal)/span_s:.2f}")
        print(f"clock_rate_ppm={formal[-1]['metadata']['clock_rate_ppm']:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
