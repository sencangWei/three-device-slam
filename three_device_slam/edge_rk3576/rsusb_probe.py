"""Bounded, no-payload D405 continuity probe for an RSUSB librealsense build."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


WIDTH = 1280
HEIGHT = 720
FPS = 30
MAX_ARRIVAL_INTERVAL_MS = 250.0
PAYLOAD_BYTES = {
    "color": WIDTH * HEIGHT * 2,
    "infrared_left": WIDTH * HEIGHT,
    "infrared_right": WIDTH * HEIGHT,
}


@dataclass
class StreamContinuity:
    expected_payload_bytes: int
    received: int = 0
    first_sequence: int | None = None
    last_sequence: int | None = None
    first_timestamp_ms: float | None = None
    last_timestamp_ms: float | None = None
    last_arrival_ns: int | None = None
    sequence_gaps: int = 0
    repeated_sequences: int = 0
    sequence_regressions: int = 0
    timestamp_regressions: int = 0
    payload_size_errors: int = 0
    maximum_arrival_interval_ms: float = 0.0

    def observe(
        self,
        *,
        sequence: int,
        timestamp_ms: float,
        arrival_ns: int,
        payload_bytes: int,
    ) -> None:
        if self.last_sequence is not None:
            difference = sequence - self.last_sequence
            if difference > 1:
                self.sequence_gaps += difference - 1
            elif difference == 0:
                self.repeated_sequences += 1
            elif difference < 0:
                self.sequence_regressions += 1
        if self.last_timestamp_ms is not None and timestamp_ms <= self.last_timestamp_ms:
            self.timestamp_regressions += 1
        if self.last_arrival_ns is not None:
            interval_ms = (arrival_ns - self.last_arrival_ns) / 1_000_000.0
            self.maximum_arrival_interval_ms = max(
                self.maximum_arrival_interval_ms, interval_ms
            )
        if payload_bytes != self.expected_payload_bytes:
            self.payload_size_errors += 1
        if self.first_sequence is None:
            self.first_sequence = sequence
            self.first_timestamp_ms = timestamp_ms
        self.received += 1
        self.last_sequence = sequence
        self.last_timestamp_ms = timestamp_ms
        self.last_arrival_ns = arrival_ns

    def report(self) -> dict[str, int | float | None]:
        timestamp_span_s = (
            (self.last_timestamp_ms - self.first_timestamp_ms) / 1000.0
            if self.first_timestamp_ms is not None
            and self.last_timestamp_ms is not None
            and self.last_timestamp_ms >= self.first_timestamp_ms
            else 0.0
        )
        return {
            "received": self.received,
            "first_sequence": self.first_sequence,
            "last_sequence": self.last_sequence,
            "timestamp_span_s": round(timestamp_span_s, 9),
            "observed_rate_hz": round(
                (self.received - 1) / timestamp_span_s, 6
            )
            if timestamp_span_s > 0 and self.received > 1
            else 0.0,
            "sequence_gaps": self.sequence_gaps,
            "repeated_sequences": self.repeated_sequences,
            "sequence_regressions": self.sequence_regressions,
            "timestamp_regressions": self.timestamp_regressions,
            "payload_size_errors": self.payload_size_errors,
            "max_arrival_interval_ms": round(
                self.maximum_arrival_interval_ms, 3
            ),
        }


def evaluate_streams(
    streams: dict[str, dict[str, Any]], *, expected_frames: int
) -> dict[str, str | list[str]]:
    reasons: list[str] = []
    for name in ("color", "infrared_left", "infrared_right"):
        metrics = streams[name]
        if metrics["received"] != expected_frames:
            reasons.append(
                f"{name}:frames={metrics['received']} expected={expected_frames}"
            )
        for key in (
            "sequence_gaps",
            "repeated_sequences",
            "sequence_regressions",
            "timestamp_regressions",
            "payload_size_errors",
        ):
            if metrics[key]:
                reasons.append(f"{name}:{key}={metrics[key]}")
        if metrics["max_arrival_interval_ms"] > MAX_ARRIVAL_INTERVAL_MS:
            reasons.append(
                f"{name}:max_arrival_interval_ms="
                f"{metrics['max_arrival_interval_ms']:.3f}"
            )
    return {"status": "PASS" if not reasons else "FAIL", "reasons": reasons}


def _positive_duration(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or not 0 < duration <= 600:
        raise argparse.ArgumentTypeError("duration must be within (0, 600] seconds")
    return duration


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--duration", type=_positive_duration, required=True)
    parser.add_argument("--warmup", type=_positive_duration, default=3.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def _stream_key(rs, frame) -> str | None:
    profile = frame.get_profile()
    stream = profile.stream_type()
    index = profile.as_video_stream_profile().stream_index()
    if stream == rs.stream.color:
        return "color"
    if stream == rs.stream.infrared and index == 1:
        return "infrared_left"
    if stream == rs.stream.infrared and index == 2:
        return "infrared_right"
    return None


def _select_profiles(rs, sensor) -> list[Any]:
    selected: dict[str, Any] = {}
    for profile in sensor.get_stream_profiles():
        video = profile.as_video_stream_profile()
        if video.width() != WIDTH or video.height() != HEIGHT or profile.fps() != FPS:
            continue
        stream = profile.stream_type()
        index = video.stream_index()
        if stream == rs.stream.color and profile.format() == rs.format.yuyv:
            selected["color"] = profile
        elif (
            stream == rs.stream.infrared
            and profile.format() == rs.format.y8
            and index in (1, 2)
        ):
            selected[f"infrared_{'left' if index == 1 else 'right'}"] = profile
    if set(selected) != set(PAYLOAD_BYTES):
        raise RuntimeError(f"D405 profiles incomplete: {sorted(selected)}")
    return [selected[name] for name in PAYLOAD_BYTES]


def _module_evidence(rs) -> dict[str, str]:
    module_path = Path(rs.__file__).resolve()
    return {
        "path": str(module_path),
        "sha256": hashlib.sha256(module_path.read_bytes()).hexdigest(),
    }


def run_probe(args: argparse.Namespace) -> dict[str, Any]:
    import pyrealsense2 as rs

    expected_frames = int(round(args.duration * FPS))
    if not math.isclose(expected_frames / FPS, args.duration, abs_tol=1e-9):
        raise ValueError("duration must resolve to an integer number of 30 Hz frames")
    context = rs.context()
    matches = [
        device
        for device in context.query_devices()
        if device.get_info(rs.camera_info.serial_number) == args.serial
    ]
    if len(matches) != 1:
        raise RuntimeError(f"expected one D405 serial {args.serial}, found {len(matches)}")
    device = matches[0]
    model = device.get_info(rs.camera_info.name)
    if "D405" not in model.upper():
        raise RuntimeError(f"serial {args.serial} is not a D405: {model}")
    sensor = device.first_depth_sensor()
    profiles = _select_profiles(rs, sensor)
    frame_queue = rs.frame_queue(128, keep_frames=False)
    continuity = {
        name: StreamContinuity(expected_payload_bytes=size)
        for name, size in PAYLOAD_BYTES.items()
    }

    opened = started = False
    formal_start_ns = formal_stop_ns = None
    try:
        sensor.open(profiles)
        opened = True
        sensor.start(frame_queue)
        started = True
        warmup_deadline = time.monotonic() + args.warmup
        while time.monotonic() < warmup_deadline:
            frame = frame_queue.wait_for_frame(1000)
            bytes(frame.get_data())

        formal_start_ns = time.monotonic_ns()
        bounded_deadline = time.monotonic() + args.duration + 10.0
        while not all(item.received >= expected_frames for item in continuity.values()):
            if time.monotonic() >= bounded_deadline:
                break
            frame = frame_queue.wait_for_frame(1000)
            key = _stream_key(rs, frame)
            if key is None or continuity[key].received >= expected_frames:
                continue
            payload = bytes(frame.get_data())
            continuity[key].observe(
                sequence=int(frame.get_frame_number()),
                timestamp_ms=float(frame.get_timestamp()),
                arrival_ns=time.monotonic_ns(),
                payload_bytes=len(payload),
            )
        formal_stop_ns = time.monotonic_ns()
    finally:
        if started:
            sensor.stop()
        if opened:
            sensor.close()

    streams = {name: item.report() for name, item in continuity.items()}
    decision = evaluate_streams(streams, expected_frames=expected_frames)
    return {
        "schema": "rk3576.d405.rsusb_probe.v1",
        "status": decision["status"],
        "reasons": decision["reasons"],
        "mode": "bench_no_motion_driver_only_no_payload_storage",
        "backend": "librealsense_rsusb",
        "host": {
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "module": _module_evidence(rs),
        "device": {
            "serial": args.serial,
            "name": model,
            "firmware": device.get_info(rs.camera_info.firmware_version),
            "usb_type": device.get_info(rs.camera_info.usb_type_descriptor),
        },
        "requested": {
            "duration_s": args.duration,
            "warmup_s": args.warmup,
            "frames_per_stream": expected_frames,
            "width": WIDTH,
            "height": HEIGHT,
            "fps": FPS,
        },
        "formal_host_span_s": round(
            (formal_stop_ns - formal_start_ns) / 1_000_000_000.0, 9
        ),
        "streams": streams,
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.output.exists():
        raise FileExistsError(f"output already exists: {args.output}")
    result = run_probe(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
