from __future__ import annotations

import argparse
import json
import statistics
import threading
import time

from .capture import TwoUQ2Packet, _CtypesXuBridge, _load_gst, build_pipeline_description


def evaluate_rates(
    *,
    duration_s: float,
    hardware_video_frames: int,
    output_video_frames: int,
    imu_packets: int,
    imu_samples: int,
    xu_failures: int,
    timestamp_regressions: int,
) -> dict:
    rates = {
        "camera_hardware_hz": hardware_video_frames / duration_s,
        "camera_output_hz": output_video_frames / duration_s,
        "imu_packet_hz": imu_packets / duration_s,
        "imu_sample_hz": imu_samples / duration_s,
    }
    checks = {
        "camera_hardware_hz": (57.0 <= rates["camera_hardware_hz"] <= 63.0),
        "camera_output_hz": (57.0 <= rates["camera_output_hz"] <= 63.0),
        "imu_packet_hz": (190.0 <= rates["imu_packet_hz"] <= 210.0),
        "imu_sample_hz": (190.0 <= rates["imu_sample_hz"] <= 210.0),
        "one_main_sample_per_packet": imu_samples == imu_packets,
        "xu_failures": xu_failures == 0,
        "timestamp_regressions": timestamp_regressions == 0,
    }
    thresholds = {
        "camera_hardware_hz": "57.0 <= value <= 63.0",
        "camera_output_hz": "57.0 <= value <= 63.0",
        "imu_packet_hz": "190.0 <= value <= 210.0",
        "imu_sample_hz": "190.0 <= value <= 210.0",
        "one_main_sample_per_packet": "imu_samples == imu_packets",
        "xu_failures": "== 0",
        "timestamp_regressions": "== 0",
    }
    measurements = {
        **rates,
        "one_main_sample_per_packet": (
            imu_samples / imu_packets if imu_packets else None
        ),
        "xu_failures": xu_failures,
        "timestamp_regressions": timestamp_regressions,
    }
    return {
        "schema": "ego.2uq2.rate-probe.v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "scope": "transport_rate",
        "rates": rates,
        "counts": {
            "hardware_video_frames": hardware_video_frames,
            "output_video_frames": output_video_frames,
            "imu_packets": imu_packets,
            "imu_samples": imu_samples,
            "xu_failures": xu_failures,
            "timestamp_regressions": timestamp_regressions,
        },
        "checks": {
            name: {
                "status": "PASS" if passed else "FAIL",
                "threshold": thresholds[name],
                "measurement": measurements[name],
            }
            for name, passed in checks.items()
        },
    }


class _VideoCounter:
    def __init__(self, gst):
        self._gst = gst
        self._lock = threading.Lock()
        self._active = False
        self._hardware_index = 0
        self._hardware_frames = 0
        self._output_frames = 0

    def begin_window(self) -> None:
        with self._lock:
            self._hardware_frames = 0
            self._output_frames = 0
            self._active = True

    def end_window(self) -> tuple[int, int]:
        with self._lock:
            self._active = False
            return self._hardware_frames, self._output_frames

    def callback(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return self._gst.FlowReturn.ERROR
        with self._lock:
            self._hardware_index += 1
            if self._active:
                self._hardware_frames += 1
                self._output_frames += 1
        return self._gst.FlowReturn.OK


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    return sorted(values)[int(fraction * (len(values) - 1))]


def probe(device: str, xu_library: str, duration_s: float) -> dict:
    gst = _load_gst()
    gst.init(None)
    pipeline = gst.parse_launch(build_pipeline_description(device))
    sink = pipeline.get_by_name("sink")
    if sink is None:
        raise RuntimeError("2UQ2 pipeline has no appsink")
    video = _VideoCounter(gst)
    handler = sink.connect("new-sample", video.callback)
    bridge = _CtypesXuBridge(xu_library)
    descriptor = None
    call_ms = []
    sample_intervals_ms = []
    imu_packets = 0
    imu_samples = 0
    xu_failures = 0
    timestamp_regressions = 0
    duplicate_main_payloads = 0
    last_main_payload = None
    last_sample_ns = None
    try:
        state_result = pipeline.set_state(gst.State.PLAYING)
        if state_result == gst.StateChangeReturn.FAILURE:
            raise RuntimeError("gstreamer_start_failed")
        pipeline.get_state(5 * gst.SECOND)
        descriptor = bridge.open(device)

        window_start_ns = time.monotonic_ns()
        window_deadline_ns = window_start_ns + int(duration_s * 1_000_000_000)
        video.begin_window()
        while time.monotonic_ns() < window_deadline_ns:
            read_started_ns = time.monotonic_ns()
            try:
                raw = bridge.read_imu27(descriptor)
            except OSError:
                xu_failures += 1
                continue
            read_ended_ns = time.monotonic_ns()
            packet = TwoUQ2Packet.parse(raw)
            acquisition_ns = (read_started_ns + read_ended_ns) // 2
            imu_packets += 1
            imu_samples += 1
            duplicate_main_payloads += int(
                last_main_payload is not None and packet.main_raw == last_main_payload
            )
            last_main_payload = packet.main_raw
            call_ms.append((read_ended_ns - read_started_ns) / 1_000_000)
            if last_sample_ns is not None:
                interval_ms = (acquisition_ns - last_sample_ns) / 1_000_000
                sample_intervals_ms.append(interval_ms)
                timestamp_regressions += int(interval_ms < 0)
            last_sample_ns = acquisition_ns
        window_end_ns = time.monotonic_ns()
        hardware_frames, output_frames = video.end_window()
    finally:
        if descriptor is not None:
            bridge.close(descriptor)
        pipeline.set_state(gst.State.NULL)
        sink.disconnect(handler)

    measured_duration_s = (window_end_ns - window_start_ns) / 1_000_000_000
    report = evaluate_rates(
        duration_s=measured_duration_s,
        hardware_video_frames=hardware_frames,
        output_video_frames=output_frames,
        imu_packets=imu_packets,
        imu_samples=imu_samples,
        xu_failures=xu_failures,
        timestamp_regressions=timestamp_regressions,
    )
    report["duration_s"] = measured_duration_s
    report["timing"] = {
        "imu_timestamp_source": "host_monotonic_control_transfer_midpoint",
        "published_group": "main_bytes_3_14_only",
        "call_ms_p50": statistics.median(call_ms) if call_ms else None,
        "call_ms_p95": _percentile(call_ms, 0.95),
        "call_ms_max": max(call_ms) if call_ms else None,
        "sample_interval_ms_p50": (
            statistics.median(sample_intervals_ms) if sample_intervals_ms else None
        ),
        "sample_interval_ms_p95": _percentile(sample_intervals_ms, 0.95),
        "sample_interval_ms_max": (
            max(sample_intervals_ms) if sample_intervals_ms else None
        ),
    }
    report["packet_evidence"] = {
        "duplicate_main_payloads": duplicate_main_payloads,
        "physical_imu_count": 1,
        "backup_group_policy": "raw_packet_only_not_published",
    }
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Measure 2UQ2 60 Hz camera and 200 Hz single-IMU transport"
    )
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument(
        "--xu-library", default="/opt/three-device-slam/lib/libtwo_uq2_xu.so"
    )
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    try:
        report = probe(args.device, args.xu_library, args.duration)
    except Exception as exc:
        print(
            json.dumps(
                {
                    "schema": "ego.2uq2.rate-probe.v1",
                    "status": "BLOCKED",
                    "reason": type(exc).__name__,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 3
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
