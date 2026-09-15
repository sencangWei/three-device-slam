#!/usr/bin/env python3
"""Dual color-stream capture (Ego D435i + UMI D405) for tag mount calibration.

Both RealSense devices are owned by this single process/thread (the RSUSB
ownership rule). Records ``ego/ego.color`` and ``left/left.color`` in the
append-only session format plus per-device calibration.json snapshots that
include the color->ir_left extrinsics needed to express the calibrated
mount in the UMI stereo (VIO) camera frame.

Frames are stored as grayscale Y8.

Usage (venv python, from the repo root):

    PYTHONPATH=. /opt/three-device-slam/venv/bin/python \
        scripts/pair_color_tag_capture.py --session /path/to/session \
        --ego-serial 327122078613 --umi-serial 260322279785 --duration 20
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from zlib import crc32

import numpy as np

from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ego-serial", default="327122078613")
    parser.add_argument("--umi-serial", default="260322279785")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument(
        "--umi-gain",
        type=float,
        default=96.0,
        help="manual color gain for the UMI D405 (AE off); 0 keeps auto-exposure",
    )
    parser.add_argument(
        "--umi-exposure-us",
        type=float,
        default=30000.0,
        help="manual color exposure for the UMI D405 when --umi-gain > 0",
    )
    return parser.parse_args(argv)


def _find_profile(sensor, rs, stream_type, index=0, width=1280, height=720):
    for profile in sensor.get_stream_profiles():
        video = profile.as_video_stream_profile()
        if (
            profile.stream_type() == stream_type
            and video.stream_index() == index
            and video.width() == width
            and video.height() == height
            and profile.fps() == 30
        ):
            return profile
    raise RuntimeError(f"no {stream_type} {width}x{height}@30 profile")


def _intrinsics_dict(intr) -> dict:
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "coefficients": [float(c) for c in intr.coeffs],
        "distortion_model": str(intr.model),
    }


def _extrinsics_dict(extrinsics) -> dict:
    return serialize_sdk_extrinsics(extrinsics)


def _decode_color(frame) -> "np.ndarray":
    """Return a grayscale 1280x720 uint8 image from a color frame.

    The SDK hands back YUYV payloads either as a (720, 1280) uint16 view
    (two bytes per pixel) or as a (720, 1280, 2) uint8 array, depending on
    the profile; RGB8 arrives as (720, 1280, 3). Anything else is assumed
    to be single-channel already.
    """
    import cv2

    data = np.asanyarray(frame.get_data())
    if data.dtype == np.uint16 and data.size == 1280 * 720:
        yuyv = data.reshape(720, 1280).view(np.uint8).reshape(720, 1280, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2GRAY_YUY2)
    if data.size == 1280 * 720 * 2:
        yuyv = data.reshape(720, 1280, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2GRAY_YUY2)
    if data.size == 1280 * 720 * 3:
        return cv2.cvtColor(data.reshape(720, 1280, 3), cv2.COLOR_RGB2GRAY)
    return data.reshape(720, 1280)


def main(argv=None) -> int:
    import time

    import pyrealsense2 as rs

    from three_device_slam.devices.d405_umi import worker as d405
    from three_device_slam.devices.d435i_ego import worker as d435
    from three_device_slam.synchronization.clock_mapping import DeviceClockTracker

    args = parse_args(argv)
    session = args.session.resolve()
    if session.exists():
        raise FileExistsError(f"session already exists: {session}")
    (session / "ego").mkdir(parents=True)
    (session / "left").mkdir(parents=True)

    rs = d435._load_realsense()
    d405._load_hardware_modules()
    context = rs.context()
    devices = {
        device.get_info(rs.camera_info.serial_number): device
        for device in context.query_devices()
    }
    if args.ego_serial not in devices:
        raise RuntimeError(f"ego D435i not found: {args.ego_serial}")
    if args.umi_serial not in devices:
        raise RuntimeError(f"UMI D405 not found: {args.umi_serial}")
    ego_device = devices[args.ego_serial]
    umi_device = devices[args.umi_serial]

    d435_global = d435._enable_global_time(rs, ego_device)
    ego_sensor = ego_device.first_color_sensor()
    ego_color_profile = _find_profile(
        ego_sensor, rs, rs.stream.color
    )
    ego_ir_profile = _find_profile(
        ego_device.first_depth_sensor(), rs, rs.stream.infrared, 1
    )
    ego_color_to_ir = ego_color_profile.get_extrinsics_to(ego_ir_profile)

    umi_sensor = umi_device.first_depth_sensor()
    d405.configure_global_time(umi_sensor)
    umi_color_profile = _find_profile(
        umi_sensor, rs, rs.stream.color
    )
    umi_ir_profile = _find_profile(umi_sensor, rs, rs.stream.infrared, 1)
    umi_color_to_ir = umi_color_profile.get_extrinsics_to(umi_ir_profile)

    (session / "ego" / "calibration.json").write_text(
        json.dumps(
            {
                "schema": "ego.d435i.factory_calibration.v1",
                "device": {
                    "serial": args.ego_serial,
                    "name": ego_device.get_info(rs.camera_info.name),
                },
                "intrinsics": {"color": _intrinsics_dict(ego_color_profile.as_video_stream_profile().get_intrinsics())},
                "extrinsics": {"color_to_ir_left": _extrinsics_dict(ego_color_to_ir)},
                "global_time": d435_global,
                "note": "color stream used for AprilTag observation",
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    (session / "left" / "calibration.json").write_text(
        json.dumps(
            {
                "schema": "umi.d405.factory_calibration.v1",
                "device": {
                    "serial": args.umi_serial,
                    "name": umi_device.get_info(rs.camera_info.name),
                },
                "streams": {"color": _intrinsics_dict(umi_color_profile.as_video_stream_profile().get_intrinsics())},
                "extrinsics": {"color_to_ir_left": _extrinsics_dict(umi_color_to_ir)},
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    candidates = []
    for _ in range(9):
        wall_before = time.time_ns()
        monotonic = time.monotonic_ns()
        wall_after = time.time_ns()
        candidates.append((wall_before + wall_after) // 2 - monotonic)
    intercept_ns = min(candidates)
    ego_tracker = DeviceClockTracker(
        intercept_ns=intercept_ns, expected_frame_delta_ns=33_333_333
    )
    umi_tracker = DeviceClockTracker(
        intercept_ns=intercept_ns, expected_frame_delta_ns=33_333_333
    )

    pipeline = rs.pipeline(context)
    config = rs.config()
    config.enable_device(args.ego_serial)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    pipeline.start(config)
    # The D405's auto-exposure parks at minimum gain in a dim scene, which
    # leaves the AprilTag below the detector's contrast floor. Manual
    # exposure/gain (set after open, before start) lifts the tag into range.
    if args.umi_gain > 0:
        umi_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
        umi_sensor.set_option(rs.option.exposure, args.umi_exposure_us)
        umi_sensor.set_option(rs.option.gain, args.umi_gain)
    umi_sensor.open([umi_color_profile])
    queue = rs.frame_queue(64, keep_frames=True)
    umi_sensor.start(queue)

    buffers = {"ego.color": bytearray(), "left.color": bytearray()}
    rows = {"ego.color": [], "left.color": []}
    sequences = {"ego.color": 0, "left.color": 0}
    trackers = {"ego.color": ego_tracker, "left.color": umi_tracker}

    def record(stream_id, device_dir, gray, acquisition_ms, warmup):
        tracker = trackers[stream_id]
        valid = True
        tracker.add_sample(sequences[stream_id], acquisition_ms)
        frame = gray.tobytes()
        offset = len(buffers[stream_id])
        buffers[stream_id].extend(frame)
        rows[stream_id].append(
            {
                "stream_id": stream_id,
                "sequence": sequences[stream_id],
                "acquisition_ns": tracker.map_acquisition_ns(acquisition_ms),
                "arrival_ns": time.monotonic_ns(),
                "clock_domain": "realsense_global_time_mapped_to_host_monotonic",
                "warmup": warmup,
                "valid": valid,
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
        sequences[stream_id] += 1

    warmup_until = time.monotonic() + args.warmup
    deadline = warmup_until + args.duration
    try:
        while time.monotonic() < deadline:
            warmup = time.monotonic() < warmup_until
            try:
                frames = pipeline.wait_for_frames(200)
            except RuntimeError:
                frames = None
            if frames is not None:
                color = frames.get_color_frame()
                if color:
                    gray = _decode_color(color)
                    record(
                        "ego.color",
                        "ego",
                        gray,
                        float(color.get_timestamp()),
                        warmup,
                    )
            for _ in range(64):
                frame = queue.poll_for_frame()
                if not frame:
                    break
                color = frame.as_frameset().get_color_frame() if frame.is_frameset() else frame
                if not color:
                    continue
                gray = _decode_color(color)
                record(
                    "left.color",
                    "left",
                    gray,
                    float(color.get_timestamp()),
                    warmup,
                )
    finally:
        try:
            umi_sensor.stop()
            umi_sensor.close()
        except Exception:
            pass
        pipeline.stop()

    for stream_id, device_dir in (("ego.color", "ego"), ("left.color", "left")):
        with (session / device_dir / f"{stream_id}.jsonl").open(
            "w", encoding="utf-8"
        ) as handle:
            for row in rows[stream_id]:
                handle.write(json.dumps(row, sort_keys=True) + "\n")
        (session / device_dir / f"{stream_id}.bin").write_bytes(
            bytes(buffers[stream_id])
        )
    formal_ego = [r for r in rows["ego.color"] if not r["warmup"]]
    formal_umi = [r for r in rows["left.color"] if not r["warmup"]]
    print(f"session={session}")
    print(f"ego.color formal={len(formal_ego)} left.color formal={len(formal_umi)}")
    if formal_ego and formal_umi:
        print(
            "ego rate ppm:", round(formal_ego[-1]["metadata"]["clock_rate_ppm"], 2),
            "umi rate ppm:", round(formal_umi[-1]["metadata"]["clock_rate_ppm"], 2),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
