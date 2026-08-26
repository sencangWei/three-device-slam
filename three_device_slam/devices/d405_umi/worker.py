#!/usr/bin/env python3
"""Record a D405 720p stereo-IR sensor pack and the external IMU.

The RealSense streams are stored losslessly in a rosbag2 sqlite file. The SDK
backend is selected by the launcher. Color remains in camera-native YUYV; BGR
conversion is preview-only. The timestamp CSV is reconstructed from the
recorded DB3 so a slow preview cannot alter the SLAM input timeline. The
external IMU remains in the project's established imu.bin/imu_ts.csv format.
"""

from __future__ import annotations

import argparse
import csv
import gc
import hashlib
import json
import math
import os
import re
import shutil
import sqlite3
import struct
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from types import SimpleNamespace
from pathlib import Path
from contextlib import nullcontext

import numpy as np


cv2 = None
rs = SimpleNamespace(
    option=SimpleNamespace(
        auto_exposure_limit="auto exposure limit",
        auto_exposure_limit_toggle="auto exposure limit toggle",
        auto_gain_limit="auto gain limit",
        auto_gain_limit_toggle="auto gain limit toggle",
        enable_auto_exposure="enable auto exposure",
        global_time_enabled="global time enabled",
    ),
    timestamp_domain=SimpleNamespace(
        global_time="global_time",
        system_time="system_time",
        hardware_clock="hardware_clock",
    ),
    stream=SimpleNamespace(color="color", depth="depth", infrared="infrared"),
    format=SimpleNamespace(y8="y8", yuyv="yuyv", z16="z16"),
)


def _load_hardware_modules() -> None:
    global cv2, rs, STREAMS, DEPTH_STEREO_STREAMS, STREAM_KEYS
    import cv2 as cv2_module
    import pyrealsense2 as rs_module

    cv2 = cv2_module
    rs = rs_module
    STREAMS = (
        ("color", rs.stream.color, 0, rs.format.yuyv),
        ("infrared_left", rs.stream.infrared, 1, rs.format.y8),
        ("infrared_right", rs.stream.infrared, 2, rs.format.y8),
    )
    DEPTH_STEREO_STREAMS = (
        ("depth", rs.stream.depth, 0, rs.format.z16),
        ("infrared_left", rs.stream.infrared, 1, rs.format.y8),
        ("infrared_right", rs.stream.infrared, 2, rs.format.y8),
    )
    STREAM_KEYS = tuple(name for name, *_ in STREAMS)

from .imu.imu_reader import ImuReader
from .imu.calibration import IMUCalibration
from .camera.realsense_capture import CameraFrame
from .recorder.recorder import IMU_PACK_SIZE, UnitRecorder
from three_device_slam.core import BarrierDirectory, DeviceHeartbeat
from three_device_slam.core.session_lifecycle import producer_claim
from .gripper.training_sync import (
    MAX_CAMERA_GRIPPER_DELTA_MS,
    MAX_ENCODER_PAIR_GAP_US,
    analyze_gripper_csv,
    write_gripper_camera_alignment,
)


STREAMS = (
    ("color", rs.stream.color, 0, rs.format.yuyv),
    ("infrared_left", rs.stream.infrared, 1, rs.format.y8),
    ("infrared_right", rs.stream.infrared, 2, rs.format.y8),
)

DEPTH_STEREO_STREAMS = (
    ("depth", rs.stream.depth, 0, rs.format.z16),
    ("infrared_left", rs.stream.infrared, 1, rs.format.y8),
    ("infrared_right", rs.stream.infrared, 2, rs.format.y8),
)

STREAM_KEYS = tuple(name for name, *_ in STREAMS)
IMU_WARMUP_FRAMES = 500
IMU_TRANSPORT_COUNTER_KEYS = (
    "frames_ok",
    "frames_bad",
    "resyncs",
    "dropped_frames",
    "counter_resets",
    "counter_stalls",
    "sequence_gaps",
    "invalid_imu_flags",
    "queue_overflow_flags",
    "serial_errors",
    "serial_reconnects",
)
MIN_CAMERA_RATE_HZ = 29.5
MAX_CAMERA_GAP_RATIO = 0.001
MIN_IMU_RATE_HZ = 399.0
MAX_IMU_RATE_HZ = 401.0
SYNC_TOLERANCE_MS = 2.0
PRODUCT_CAMERA_IMU_TD_S = -0.009312
CAMERA_RAW_BYTES_PER_SECOND = 1280 * 720 * (2 + 1 + 1) * 30
STAGING_HEADROOM_RATIO = 1.15
MONITOR_QUEUE_CAPACITY = 64
JOINT_HEARTBEAT_INTERVAL_NS = 50_000_000
JOINT_MINIMUM_START_GUARD_NS = 100_000_000
JOINT_CAMERA_STALE_NS = 250_000_000
JOINT_IMU_STALE_NS = 100_000_000
JOINT_OUTPUT_GRACE_NS = 2_000_000_000
JOINT_OUTPUT_STALE_NS = 2_000_000_000
JOINT_READINESS_WINDOW_NS = 3_000_000_000
DEFAULT_IR_AUTO_EXPOSURE_LIMIT_US = 8000.0
DEFAULT_IR_AUTO_GAIN_LIMIT = 248.0
IR_EXPOSURE_LIMIT_TOLERANCE_US = 100.0
FRAME_NUMBER_RE = re.compile(r"(?:^|;)Frame number=(\d+)")
TIMESTAMP_RE = re.compile(r"(?:^|;)timestamp=([0-9.]+)")
ACTUAL_EXPOSURE_RE = re.compile(r"(?:^|;)Actual Exposure=([0-9.]+)")
GAIN_LEVEL_RE = re.compile(r"(?:^|;)Gain Level=([0-9.]+)")
METADATA_TOPICS = {
    "color": "/device_0/sensor_0/Color_0/image/metadata",
    "depth": "/device_0/sensor_0/Depth_0/image/metadata",
    "infrared_left": "/device_0/sensor_0/Infrared_1/image/metadata",
    "infrared_right": "/device_0/sensor_0/Infrared_2/image/metadata",
}


def capture_streams_for_mode(mode: str) -> tuple:
    if mode == "rgb_stereo_ir":
        return STREAMS
    if mode == "depth_stereo_ir":
        return DEPTH_STEREO_STREAMS
    raise ValueError(f"unsupported capture mode: {mode}")


@dataclass
class StreamContinuity:
    received: int = 0
    first_number: int | None = None
    last_number: int | None = None
    first_timestamp_ms: float | None = None
    last_timestamp_ms: float | None = None
    skipped_frames: int = 0
    gap_events: int = 0
    repeated_frames: int = 0
    frame_number_resets: int = 0
    timestamp_regressions: int = 0

    def add(self, number: int, timestamp_ms: float) -> None:
        if self.first_number is None:
            self.first_number = number
            self.first_timestamp_ms = timestamp_ms
        if self.last_number is not None:
            if number > self.last_number + 1:
                self.skipped_frames += number - self.last_number - 1
                self.gap_events += 1
            elif number == self.last_number:
                self.repeated_frames += 1
            elif number < self.last_number:
                self.frame_number_resets += 1
        if self.last_timestamp_ms is not None and timestamp_ms < self.last_timestamp_ms:
            self.timestamp_regressions += 1
        self.received += 1
        self.last_number = number
        self.last_timestamp_ms = timestamp_ms

    def report(self) -> dict:
        number_span = (
            self.last_number - self.first_number
            if self.first_number is not None and self.last_number is not None
            else 0
        )
        timestamp_span_s = (
            (self.last_timestamp_ms - self.first_timestamp_ms) / 1000.0
            if self.first_timestamp_ms is not None
            and self.last_timestamp_ms is not None
            and self.last_timestamp_ms >= self.first_timestamp_ms
            else 0.0
        )
        return {
            "received": self.received,
            "first_frame_number": self.first_number,
            "last_frame_number": self.last_number,
            "skipped_frames": self.skipped_frames,
            "gap_events": self.gap_events,
            "gap_ratio": round(self.skipped_frames / number_span, 9) if number_span else 0.0,
            "rate_hz": round(number_span / timestamp_span_s, 6) if timestamp_span_s else 0.0,
            "repeated_frames": self.repeated_frames,
            "frame_number_resets": self.frame_number_resets,
            "timestamp_regressions": self.timestamp_regressions,
        }


@dataclass(frozen=True)
class MetadataFrame:
    number: int
    device_ms: float
    exposure_us: float | None = None
    gain: float | None = None


def configure_global_time(sensor) -> dict:
    option = rs.option.global_time_enabled
    requested = 1.0
    supported = bool(sensor.supports(option))
    evidence = {
        "supported": supported,
        "requested": requested,
        "readback": None,
        "verified": False,
    }
    if not supported:
        return evidence
    sensor.set_option(option, requested)
    readback = float(sensor.get_option(option))
    evidence["readback"] = readback
    evidence["verified"] = readback == requested
    return evidence


def normalize_frame_timestamp_domain(domain) -> str:
    for attribute, name in (
        ("global_time", "global_time"),
        ("system_time", "system_time"),
        ("hardware_clock", "hardware_clock"),
    ):
        known = getattr(rs.timestamp_domain, attribute, None)
        if known is not None and domain == known:
            return name
    return "unrecognized"


class CameraClockEvidence:
    def __init__(self, stream_keys, configuration: dict):
        self.stream_keys = tuple(stream_keys)
        self.configuration = dict(configuration)
        self.observed_domains = {key: {} for key in self.stream_keys}

    def observe(self, stream_key: str, domain) -> None:
        if stream_key not in self.observed_domains:
            raise ValueError(f"unknown camera stream: {stream_key}")
        name = normalize_frame_timestamp_domain(domain)
        counts = self.observed_domains[stream_key]
        counts[name] = counts.get(name, 0) + 1

    def report(self) -> dict:
        observed = {
            stream: dict(sorted(counts.items()))
            for stream, counts in self.observed_domains.items()
        }
        all_global = all(
            counts and set(counts) == {"global_time"}
            for counts in observed.values()
        )
        return {
            "configuration": dict(self.configuration),
            "required_streams": list(self.stream_keys),
            "observed_domains": observed,
            "verified": self.configuration.get("verified") is True and all_global,
        }


class JointWorkerFailure(RuntimeError):
    exit_code = 2

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(reason)


def barrier_root(path: Path) -> Path:
    path = Path(path)
    return path.parent if path.name == "barrier" else path


def build_joint_start_acceptance(
    *,
    device_id: str,
    barrier_dir: Path,
    formal_start_host_monotonic_ns: int | None,
    first_formal_camera_device_ms: float | None,
    first_formal_imu_counter: int | None,
    clock_domain: str,
) -> dict:
    return {
        "device_id": device_id,
        "barrier_directory": str(barrier_root(barrier_dir) / "barrier"),
        "formal_start_host_monotonic_ns": formal_start_host_monotonic_ns,
        "clock_domain": clock_domain,
        "first_formal_camera_device_ms": first_formal_camera_device_ms,
        "first_formal_imu_counter": first_formal_imu_counter,
        "raw_recording_is_append_only": True,
        "raw_warmup_preserved": True,
        "classification_metadata": "d405_frames.csv:warmup",
    }


def classify_d405_result(
    camera_ok: bool,
    imu_ok: bool,
    gripper_ok: bool,
    live_vins_ok: bool,
    camera_clock_verified: bool,
) -> str:
    if not (camera_ok and imu_ok and gripper_ok and live_vins_ok):
        return "FAIL"
    return "PASS" if camera_clock_verified else "BLOCKED"


def build_raw_file_manifest(root: Path, paths) -> dict:
    root = Path(root).resolve()
    result = {}
    for candidate in paths:
        path = Path(candidate)
        if not path.is_file():
            continue
        resolved = path.resolve()
        relative = resolved.relative_to(root).as_posix()
        digest = hashlib.sha256()
        with resolved.open("rb") as stream:
            for block in iter(lambda: stream.read(65536), b""):
                digest.update(block)
        result[relative] = {
            "size_bytes": resolved.stat().st_size,
            "sha256": digest.hexdigest(),
        }
    return result


def write_atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def shutdown_capture_outputs(
    *,
    pause_recorder,
    stop_sensor,
    close_sensor,
    stop_imu,
    imu_recorder,
) -> dict:
    errors = []
    camera_error = None
    camera_clean = True
    imu_clean = True
    recorder_write_error = None

    if pause_recorder is not None:
        try:
            camera_error = pause_recorder()
        except Exception as exc:
            camera_error = f"{type(exc).__name__}: {exc}"
        if camera_error is not None:
            camera_clean = False
            errors.append(f"camera_pause: {camera_error}")

    for name, action in (
        ("sensor_stop", stop_sensor),
        ("sensor_close", close_sensor),
        ("imu_stop", stop_imu),
    ):
        if action is None:
            continue
        try:
            action()
        except Exception as exc:
            errors.append(f"{name}: {type(exc).__name__}: {exc}")

    if imu_recorder is not None:
        try:
            imu_clean = bool(imu_recorder.stop())
        except Exception as exc:
            imu_clean = False
            errors.append(f"recorder_stop: {type(exc).__name__}: {exc}")
        recorder_write_error = imu_recorder.first_write_error
        if not imu_clean and recorder_write_error is None:
            errors.append("recorder_stop: recorder shutdown timed out")
        if recorder_write_error is not None:
            errors.append(f"recorder_write: {recorder_write_error}")

    return {
        "camera_clean": camera_clean,
        "camera_error": camera_error,
        "imu_clean": imu_clean,
        "recorder_write_error": recorder_write_error,
        "errors": tuple(errors),
    }


def finalize_joint_outputs(
    *,
    session: Path,
    report: dict,
    raw_paths,
    camera_recorder_clean_shutdown: bool,
    imu_recorder_clean_shutdown: bool,
    recorder_write_error: str | None,
    camera_recorder_shutdown_error: str | None,
    stage_move_error: str | None,
    shutdown_errors=(),
) -> dict:
    finalization_errors = tuple(
        error
        for error in (
            None
            if camera_recorder_clean_shutdown
            else camera_recorder_shutdown_error or "camera recorder shutdown failed",
            None
            if imu_recorder_clean_shutdown
            else recorder_write_error or "IMU recorder shutdown failed",
            recorder_write_error,
            stage_move_error,
            *shutdown_errors,
        )
        if error is not None
    )
    if finalization_errors:
        report["result"] = "FAIL"
        report["reason"] = "output_finalization_failed"
        report["finalization_errors"] = list(dict.fromkeys(finalization_errors))
    outputs_sealed = (
        camera_recorder_clean_shutdown
        and imu_recorder_clean_shutdown
        and recorder_write_error is None
        and camera_recorder_shutdown_error is None
        and stage_move_error is None
    )
    if not outputs_sealed:
        report["raw_files"] = {}
    else:
        try:
            report["raw_files"] = build_raw_file_manifest(session, raw_paths)
        except (OSError, ValueError) as exc:
            report["result"] = "FAIL"
            report["reason"] = "raw_manifest_failed"
            report["raw_files"] = {}
            report["finalization_errors"] = [f"{type(exc).__name__}: {exc}"]
    write_atomic_json(Path(session) / "acceptance.json", report)
    return report


def run_prestart_step(
    controller, reason: str, action, *, clock_ns=time.monotonic_ns
):
    try:
        return action()
    except Exception as exc:
        if controller is None:
            raise
        controller.barrier.latch_failure(controller.device_id, reason, clock_ns())
        raise JointWorkerFailure(reason) from exc


def pause_joint_recorder_for_shutdown(
    recorder_device, controller, *, clock_ns=time.monotonic_ns
) -> str | None:
    try:
        recorder_device.pause()
        return None
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        try:
            controller.barrier.latch_failure(
                controller.device_id, "recorder_error", clock_ns()
            )
        except (OSError, ValueError):
            pass
        return error


class JointBarrierController:
    """Small testable boundary between a D405 worker and BarrierDirectory."""

    HEALTH_REASONS = {
        "camera_disconnect",
        "imu_disconnect",
        "recorder_error",
        "timestamp_regression",
        "storage_error",
        "imu_error",
        "readiness_window",
    }

    def __init__(self, barrier, device_id: str, stream_keys=None):
        if device_id not in {"left", "right"}:
            raise ValueError("joint D405 device_id must be left or right")
        self.barrier = barrier
        self.device_id = device_id
        self.stream_keys = tuple(STREAM_KEYS if stream_keys is None else stream_keys)
        self.start_ns: int | None = None
        self.last_heartbeat_ns: int | None = None
        self.first_formal_camera_device_ms: float | None = None
        self.first_formal_imu_counter: int | None = None
        self._camera_backlog_drained = False
        self._formal_camera_candidates: dict[str, float] = {}

    def fail(self, reason: str, now_ns: int) -> None:
        self.barrier.latch_failure(self.device_id, reason, now_ns)
        raise JointWorkerFailure(reason)

    def poll(
        self,
        now_ns: int,
        healthy: bool,
        reasons: tuple[str, ...],
        terminal_reason: str | None = None,
    ) -> int | None:
        unknown = set(reasons) - self.HEALTH_REASONS
        if unknown:
            raise ValueError(f"unsupported heartbeat health reasons: {sorted(unknown)}")
        candidate = self.barrier.read_start_ns()
        if self.start_ns is None and candidate is not None:
            if candidate - now_ns < JOINT_MINIMUM_START_GUARD_NS:
                self.fail("stale_start", now_ns)
            self.start_ns = candidate
        elif self.start_ns is not None and candidate != self.start_ns:
            self.fail("changed_start", now_ns)
        if terminal_reason is not None:
            self.fail(terminal_reason, now_ns)
        if (
            self.last_heartbeat_ns is None
            or now_ns - self.last_heartbeat_ns >= JOINT_HEARTBEAT_INTERVAL_NS
        ):
            self.barrier.write_heartbeat(
                DeviceHeartbeat(self.device_id, now_ns, healthy, reasons)
            )
            self.last_heartbeat_ns = now_ns
        if self.start_ns is not None and now_ns >= self.start_ns:
            return self.start_ns
        return None

    def mark_camera_backlog_drained(self) -> None:
        self._camera_backlog_drained = True
        self._formal_camera_candidates.clear()

    def observe_camera(self, key: str, device_ms: float) -> None:
        if not self._camera_backlog_drained or self.first_formal_camera_device_ms is not None:
            return
        if key not in self.stream_keys:
            raise ValueError(f"unknown camera stream: {key}")
        self._formal_camera_candidates[key] = device_ms
        if set(self._formal_camera_candidates) == set(self.stream_keys):
            timestamps = list(self._formal_camera_candidates.values())
            if timestamps_aligned(timestamps, SYNC_TOLERANCE_MS):
                self.first_formal_camera_device_ms = max(timestamps)

    def observe_imu(self, host_ns: int, counter: int) -> None:
        if (
            self.start_ns is not None
            and host_ns >= self.start_ns
            and self.first_formal_imu_counter is None
        ):
            self.first_formal_imu_counter = counter


class JointCaptureHealth:
    """Track only the health signals a joint D405 worker may publish."""

    def __init__(self, stream_keys: tuple[str, ...]):
        self.stream_keys = tuple(stream_keys)
        self.last_camera_ns: dict[str, int] = {}
        self.last_camera_device_ms: dict[str, float] = {}
        self.last_imu_ns: int | None = None
        self.last_imu_acquisition_s: float | None = None
        self.timestamp_regression = False
        self._camera_window = {key: deque() for key in self.stream_keys}
        self._imu_window = deque()

    def observe_camera(
        self, key: str, device_ms: float, host_ns: int, frame_number: int | None = None
    ) -> None:
        previous = self.last_camera_device_ms.get(key)
        if previous is not None and device_ms < previous:
            self.timestamp_regression = True
        self.last_camera_device_ms[key] = device_ms
        self.last_camera_ns[key] = host_ns
        self._camera_window[key].append((host_ns, frame_number))
        cutoff = host_ns - JOINT_READINESS_WINDOW_NS
        while self._camera_window[key] and self._camera_window[key][0][0] < cutoff:
            self._camera_window[key].popleft()

    def observe_imu(
        self, host_ns: int, acquisition_s: float, counter: int | None = None
    ) -> None:
        if (
            self.last_imu_acquisition_s is not None
            and acquisition_s < self.last_imu_acquisition_s
        ):
            self.timestamp_regression = True
        self.last_imu_ns = host_ns
        self.last_imu_acquisition_s = acquisition_s
        acquisition_ns = int(round(acquisition_s * 1_000_000_000))
        self._imu_window.append((host_ns, acquisition_ns, counter))
        cutoff = host_ns - JOINT_READINESS_WINDOW_NS
        while self._imu_window and self._imu_window[0][0] < cutoff:
            self._imu_window.popleft()

    def joint_ready(self, now_ns: int, *, clock_verified: bool) -> bool:
        if not clock_verified or self.timestamp_regression:
            return False
        cutoff = now_ns - JOINT_READINESS_WINDOW_NS
        maximum_frame_period_ns = int(1_000_000_000 / MIN_CAMERA_RATE_HZ)
        for key in self.stream_keys:
            window = self._camera_window[key]
            while window and window[0][0] < cutoff:
                window.popleft()
            if not window or now_ns - window[0][0] < (
                JOINT_READINESS_WINDOW_NS - maximum_frame_period_ns
            ):
                return False
            elapsed = window[-1][0] - window[0][0]
            if elapsed <= 0 or (len(window) - 1) * 1_000_000_000 / elapsed < MIN_CAMERA_RATE_HZ:
                return False
            numbers = [number for _, number in window]
            if any(number is None for number in numbers):
                return False
            if any(right != left + 1 for left, right in zip(numbers, numbers[1:])):
                return False
        imu_window = self._imu_window
        while imu_window and imu_window[0][0] < cutoff:
            imu_window.popleft()
        maximum_imu_period_ns = int(1_000_000_000 / MIN_IMU_RATE_HZ)
        required_window_coverage_ns = (
            JOINT_READINESS_WINDOW_NS - maximum_imu_period_ns
        )
        required_sample_span_ns = (
            required_window_coverage_ns - JOINT_IMU_STALE_NS
        )
        if not imu_window:
            return False
        host_elapsed = imu_window[-1][0] - imu_window[0][0]
        acquisition_elapsed = imu_window[-1][1] - imu_window[0][1]
        if now_ns - imu_window[0][0] < required_window_coverage_ns:
            return False
        if (
            host_elapsed < required_sample_span_ns
            or acquisition_elapsed < required_sample_span_ns
        ):
            return False
        latest_host_age_ns = now_ns - imu_window[-1][0]
        latest_acquisition_age_ns = now_ns - imu_window[-1][1]
        if not 0 <= latest_host_age_ns <= JOINT_IMU_STALE_NS:
            return False
        if not 0 <= latest_acquisition_age_ns <= JOINT_IMU_STALE_NS:
            return False
        rate_numerator = (len(imu_window) - 1) * 1_000_000_000
        imu_rate_hz = rate_numerator / acquisition_elapsed
        rate_quantization_tolerance_hz = max(
            rate_numerator / (acquisition_elapsed - 1) - imu_rate_hz,
            imu_rate_hz - rate_numerator / (acquisition_elapsed + 1),
        )
        if imu_rate_hz < MIN_IMU_RATE_HZ and not math.isclose(
            imu_rate_hz,
            MIN_IMU_RATE_HZ,
            rel_tol=0.0,
            abs_tol=rate_quantization_tolerance_hz,
        ):
            return False
        if imu_rate_hz > MAX_IMU_RATE_HZ and not math.isclose(
            imu_rate_hz,
            MAX_IMU_RATE_HZ,
            rel_tol=0.0,
            abs_tol=rate_quantization_tolerance_hz,
        ):
            return False
        counters = [counter for _, _, counter in imu_window]
        if any(counter is None for counter in counters):
            return False
        if any(
            ((right - left) & 0xFFFFFFFF) != 1
            for left, right in zip(counters, counters[1:])
        ):
            return False
        return True

    def evaluate(
        self,
        now_ns: int,
        *,
        imu_stats: dict,
        recorder_healthy: bool,
        storage_healthy: bool,
        disconnect_is_terminal: bool,
    ) -> tuple[bool, tuple[str, ...], str | None]:
        reasons = []
        camera_connected = all(
            key in self.last_camera_ns
            and now_ns - self.last_camera_ns[key] <= JOINT_CAMERA_STALE_NS
            for key in self.stream_keys
        )
        if not camera_connected:
            reasons.append("camera_disconnect")
        if self.last_imu_ns is None or now_ns - self.last_imu_ns > JOINT_IMU_STALE_NS:
            reasons.append("imu_disconnect")
        if self.timestamp_regression:
            reasons.append("timestamp_regression")
        imu_error = any(
            int(imu_stats.get(key, 0)) > 0
            for key in IMU_TRANSPORT_COUNTER_KEYS
            if key != "frames_ok"
        )
        if imu_error:
            reasons.append("imu_error")
        if not recorder_healthy:
            reasons.append("recorder_error")
        if not storage_healthy:
            reasons.append("storage_error")
        terminal = next(
            (
                reason
                for reason in (
                    "timestamp_regression",
                    "imu_error",
                    "recorder_error",
                    "storage_error",
                )
                if reason in reasons
            ),
            None,
        )
        if terminal is None and disconnect_is_terminal:
            terminal = next(
                (
                    reason
                    for reason in ("camera_disconnect", "imu_disconnect")
                    if reason in reasons
                ),
                None,
            )
        return not reasons, tuple(reasons), terminal


class OutputProgress:
    """Require an output file to appear and keep growing after startup grace."""

    def __init__(self, path: Path, *, grace_ns: int, stale_ns: int):
        self.path = Path(path)
        self.grace_ns = grace_ns
        self.stale_ns = stale_ns
        self.started_ns: int | None = None
        self.last_size = 0
        self.last_growth_ns: int | None = None

    @property
    def observed_growth(self) -> bool:
        return self.last_growth_ns is not None

    def healthy(self, now_ns: int) -> bool:
        if self.started_ns is None:
            self.started_ns = now_ns
        try:
            size = self.path.stat().st_size
        except FileNotFoundError:
            size = 0
        except OSError:
            return False
        if size > self.last_size:
            self.last_size = size
            self.last_growth_ns = now_ns
        if self.last_growth_ns is None:
            return now_ns - self.started_ns <= self.grace_ns
        return now_ns - self.last_growth_ns <= self.stale_ns


def persisted_imu_covers_formal(
    *, clean_shutdown: bool, persisted_samples: int | None, formal_samples: int
) -> bool:
    """Raw IMU includes warmup, so persistence is a formal-count lower bound."""
    return (
        clean_shutdown
        and persisted_samples is not None
        and persisted_samples >= formal_samples
    )


def stats_delta(start: dict, end: dict) -> dict:
    return {key: end.get(key, 0) - value for key, value in start.items()}


def imu_transport_accepted(
    protocol: str, rate_hz: float, formal: dict, recorder_drops: int
) -> bool:
    """Apply protocol-aware IMU transport gates to the formal capture window."""
    if protocol not in {"kt_ex9_37", "stm32_combined_v1"}:
        return False
    common_keys = (
        "frames_bad",
        "resyncs",
        "dropped_frames",
        "counter_resets",
        "counter_stalls",
        "serial_errors",
        "serial_reconnects",
    )
    if not (
        MIN_IMU_RATE_HZ <= rate_hz <= MAX_IMU_RATE_HZ
        and all(int(formal.get(key, -1)) == 0 for key in common_keys)
        and recorder_drops == 0
    ):
        return False
    if protocol == "stm32_combined_v1":
        return all(
            int(formal.get(key, -1)) == 0
            for key in (
                "sequence_gaps",
                "invalid_imu_flags",
                "queue_overflow_flags",
            )
        )
    return True


def timestamps_aligned(timestamps_ms: list[float], tolerance_ms: float) -> bool:
    return bool(timestamps_ms) and max(timestamps_ms) - min(timestamps_ms) <= tolerance_ms


def live_vins_timestamp_monotonic(device_ms: float, epoch_offset: float) -> float:
    """Map the recorded D405 global-time exposure timestamp to monotonic time."""
    return device_ms / 1000.0 - epoch_offset


def decode_cdr_string(blob: bytes) -> str:
    if len(blob) < 8:
        raise ValueError("CDR string 数据不足 8 字节")
    little_endian = blob[1] == 1
    length = struct.unpack_from("<I" if little_endian else ">I", blob, 4)[0]
    if length < 1 or 8 + length > len(blob):
        raise ValueError(f"CDR string 长度无效: {length}")
    return blob[8:8 + length - 1].decode("utf-8")


def metadata_frame_from_text(text: str) -> MetadataFrame | None:
    number_match = FRAME_NUMBER_RE.search(text)
    timestamp_match = TIMESTAMP_RE.search(text)
    if not number_match or not timestamp_match:
        return None
    exposure_match = ACTUAL_EXPOSURE_RE.search(text)
    gain_match = GAIN_LEVEL_RE.search(text)
    return MetadataFrame(
        number=int(number_match.group(1)),
        device_ms=float(timestamp_match.group(1)),
        exposure_us=float(exposure_match.group(1)) if exposure_match else None,
        gain=float(gain_match.group(1)) if gain_match else None,
    )


def read_db3_metadata(bag_path: Path) -> dict[str, list[MetadataFrame]]:
    records = {key: [] for key in STREAM_KEYS}
    topic_to_key = {
        topic: key
        for key, topic in METADATA_TOPICS.items()
        if key in STREAM_KEYS
    }
    placeholders = ",".join("?" for _ in topic_to_key)
    query = f"""
        SELECT topics.name, messages.data
        FROM messages JOIN topics ON messages.topic_id = topics.id
        WHERE topics.name IN ({placeholders})
        ORDER BY messages.id
    """
    with sqlite3.connect(bag_path) as connection:
        for topic, blob in connection.execute(query, tuple(topic_to_key)):
            text = decode_cdr_string(blob)
            frame = metadata_frame_from_text(text)
            if frame is not None:
                records[topic_to_key[topic]].append(frame)
    return records


def analyze_ir_exposure(
    records: dict[str, list[MetadataFrame]],
    limit_us: float,
    tolerance_us: float = IR_EXPOSURE_LIMIT_TOLERANCE_US,
) -> dict:
    streams = {}
    for key in ("infrared_left", "infrared_right"):
        stream_records = records.get(key, [])
        exposures = [
            record.exposure_us
            for record in stream_records
            if record.exposure_us is not None
        ]
        gains = [record.gain for record in stream_records if record.gain is not None]
        complete = (
            bool(stream_records)
            and len(exposures) == len(stream_records)
            and len(gains) == len(stream_records)
        )
        within_limit = bool(exposures) and max(exposures) <= limit_us + tolerance_us
        streams[key] = {
            "result": "PASS" if complete and within_limit else "FAIL",
            "metadata_complete": complete,
            "metadata_frames": len(stream_records),
            "exposure_samples": len(exposures),
            "gain_samples": len(gains),
            "exposure_us": {
                "min": float(np.min(exposures)) if exposures else None,
                "median": float(np.median(exposures)) if exposures else None,
                "p95": float(np.percentile(exposures, 95)) if exposures else None,
                "max": float(np.max(exposures)) if exposures else None,
            },
            "gain": {
                "min": float(np.min(gains)) if gains else None,
                "median": float(np.median(gains)) if gains else None,
                "p95": float(np.percentile(gains, 95)) if gains else None,
                "max": float(np.max(gains)) if gains else None,
            },
        }
    return {
        "result": (
            "PASS"
            if all(stream["result"] == "PASS" for stream in streams.values())
            else "FAIL"
        ),
        "requested_limit_us": limit_us,
        "tolerance_us": tolerance_us,
        "streams": streams,
    }


def configure_ir_auto_exposure(
    sensor, exposure_limit_us: float, gain_limit: float
) -> dict:
    required_options = (
        rs.option.enable_auto_exposure,
        rs.option.auto_exposure_limit_toggle,
        rs.option.auto_exposure_limit,
        rs.option.auto_gain_limit_toggle,
        rs.option.auto_gain_limit,
    )
    unsupported = [str(option) for option in required_options if not sensor.supports(option)]
    if unsupported:
        raise RuntimeError(f"D405缺少自动曝光限制选项: {unsupported}")

    exposure_range = sensor.get_option_range(rs.option.auto_exposure_limit)
    gain_range = sensor.get_option_range(rs.option.auto_gain_limit)
    if not exposure_range.min <= exposure_limit_us <= exposure_range.max:
        raise ValueError(
            f"IR自动曝光上限超出范围: {exposure_limit_us} not in "
            f"[{exposure_range.min}, {exposure_range.max}]"
        )
    if not gain_range.min <= gain_limit <= gain_range.max:
        raise ValueError(
            f"IR自动增益上限超出范围: {gain_limit} not in "
            f"[{gain_range.min}, {gain_range.max}]"
        )

    sensor.set_option(rs.option.enable_auto_exposure, 1.0)
    sensor.set_option(rs.option.auto_exposure_limit_toggle, 1.0)
    sensor.set_option(rs.option.auto_exposure_limit, float(exposure_limit_us))
    sensor.set_option(rs.option.auto_gain_limit_toggle, 1.0)
    sensor.set_option(rs.option.auto_gain_limit, float(gain_limit))
    applied = {
        "enable_auto_exposure": sensor.get_option(rs.option.enable_auto_exposure),
        "auto_exposure_limit_toggle": sensor.get_option(
            rs.option.auto_exposure_limit_toggle
        ),
        "auto_exposure_limit_us": sensor.get_option(rs.option.auto_exposure_limit),
        "auto_gain_limit_toggle": sensor.get_option(rs.option.auto_gain_limit_toggle),
        "auto_gain_limit": sensor.get_option(rs.option.auto_gain_limit),
    }
    exposure_tolerance = max(float(getattr(exposure_range, "step", 0.0)), 1e-6)
    gain_tolerance = max(float(getattr(gain_range, "step", 0.0)), 1e-6)
    mismatches = []
    if applied["enable_auto_exposure"] < 0.5:
        mismatches.append("enable_auto_exposure")
    if applied["auto_exposure_limit_toggle"] < 0.5:
        mismatches.append("auto_exposure_limit_toggle")
    if (
        abs(applied["auto_exposure_limit_us"] - exposure_limit_us)
        > exposure_tolerance
    ):
        mismatches.append("auto_exposure_limit_us")
    if applied["auto_gain_limit_toggle"] < 0.5:
        mismatches.append("auto_gain_limit_toggle")
    if abs(applied["auto_gain_limit"] - gain_limit) > gain_tolerance:
        mismatches.append("auto_gain_limit")
    if mismatches:
        raise RuntimeError(f"IR自动曝光配置读回值不一致: {mismatches}; applied={applied}")
    return {
        "result": "PASS",
        "requested": {
            "auto_exposure_limit_us": exposure_limit_us,
            "auto_gain_limit": gain_limit,
        },
        "applied": applied,
        "takes_effect": "next_streaming_session",
    }


def analyze_metadata_records(
    records: dict[str, list[MetadataFrame]],
) -> dict[str, dict]:
    stats = {key: StreamContinuity() for key in STREAM_KEYS}
    for key, stream_records in records.items():
        for record in stream_records:
            stats[key].add(record.number, record.device_ms)
    return {key: value.report() for key, value in stats.items()}


def pair_metadata_frames(
    records: dict[str, list[MetadataFrame]], tolerance_ms: float
) -> list[dict[str, MetadataFrame]]:
    indexes = {key: 0 for key in STREAM_KEYS}
    pairs = []
    while all(indexes[key] < len(records[key]) for key in STREAM_KEYS):
        current = {key: records[key][indexes[key]] for key in STREAM_KEYS}
        timestamps = [record.device_ms for record in current.values()]
        if timestamps_aligned(timestamps, tolerance_ms):
            pairs.append(current)
            for key in STREAM_KEYS:
                indexes[key] += 1
            continue
        earliest = min(timestamps)
        for key, record in current.items():
            if record.device_ms == earliest:
                indexes[key] += 1
    return pairs


def write_frames_csv(
    path: Path,
    records: dict[str, list[MetadataFrame]],
    epoch_offset: float,
    tolerance_ms: float,
    formal_start_device_ms: float | None = None,
    timestamp_domain: str = "global_time",
) -> int:
    if timestamp_domain not in {"global_time", "unverified"}:
        raise ValueError(f"unsupported CSV timestamp domain: {timestamp_domain}")
    fields = ["set_index", "arrival_mono", "arrival_wall", "warmup"]
    for name in STREAM_KEYS:
        fields.extend(
            [f"{name}_frame_number", f"{name}_device_ms", f"{name}_mono", f"{name}_domain"]
        )
    pairs = pair_metadata_frames(records, tolerance_ms)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        for index, pair in enumerate(pairs):
            # librealsense recorder writes messages.timestamp relative to bag
            # start, not Unix epoch. When global_time is verified, the latest
            # exposure timestamp is the stable wall/mono proxy.
            arrival_wall = max(record.device_ms for record in pair.values()) / 1000.0
            row = {
                "set_index": index,
                "arrival_mono": f"{arrival_wall - epoch_offset:.9f}",
                "arrival_wall": f"{arrival_wall:.6f}",
                "warmup": int(
                    formal_start_device_ms is not None
                    and arrival_wall * 1000.0 < formal_start_device_ms
                ),
            }
            for name, record in pair.items():
                row[f"{name}_frame_number"] = record.number
                row[f"{name}_device_ms"] = f"{record.device_ms:.6f}"
                row[f"{name}_mono"] = f"{record.device_ms / 1000.0 - epoch_offset:.9f}"
                row[f"{name}_domain"] = timestamp_domain
            writer.writerow(row)
    return len(pairs)


def analyze_db3_metadata(bag_path: Path) -> dict[str, dict]:
    return analyze_metadata_records(read_db3_metadata(bag_path))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="D405 720p三路(RGB+双IR)＋外置IMU采集")
    parser.add_argument("--serial", required=True)
    parser.add_argument("--imu-port", required=True)
    parser.add_argument("--imu-baud", type=int, default=921600)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument(
        "--capture-mode",
        choices=("rgb_stereo_ir", "depth_stereo_ir"),
        default="rgb_stereo_ir",
        help=(
            "rgb_stereo_ir保留彩色录制；depth_stereo_ir使用硬件Depth+"
            "双IR，用于平面因子和Z误差验收"
        ),
    )
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument(
        "--ir-auto-exposure-limit-us",
        type=float,
        default=DEFAULT_IR_AUTO_EXPOSURE_LIMIT_US,
        help="双IR自动曝光上限；必须在开流前设置，默认8000us",
    )
    parser.add_argument(
        "--ir-auto-gain-limit",
        type=float,
        default=DEFAULT_IR_AUTO_GAIN_LIMIT,
        help="双IR自动增益上限，默认248",
    )
    parser.add_argument(
        "--output-root", type=Path, default=Path.cwd() / "recordings"
    )
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument(
        "--publish-vins",
        action="store_true",
        help="unsupported in the isolated phase-one capture worker",
    )
    parser.add_argument(
        "--vins-imu-calibration",
        type=Path,
        default=None,
        help=(
            "实时发布前显式应用的已验收IMU内参标定；"
            "缺省发布原始IMU，原始IMU落盘始终不改写"
        ),
    )
    parser.add_argument(
        "--no-ram-stage",
        action="store_true",
        help="直接写输出硬盘；默认先写/dev/shm，停止采集后再搬运DB3",
    )
    parser.add_argument(
        "--ram-stage-root",
        type=Path,
        default=Path("/dev/shm/three_device_slam_d405"),
    )
    parser.add_argument("--device-id", choices=("d405", "left", "right"), default="d405")
    parser.add_argument("--session", type=Path)
    parser.add_argument("--barrier-dir", type=Path)
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration must be finite and positive")
    if args.publish_vins:
        parser.error("--publish-vins is unavailable in phase-one capture")
    if args.barrier_dir is not None:
        if args.device_id not in {"left", "right"}:
            parser.error("--device-id must be left or right with --barrier-dir")
        if args.session is None:
            parser.error("--session is required with --barrier-dir")
    elif args.device_id != "d405":
        parser.error("left/right --device-id requires --barrier-dir")
    return args


def capture_session_directory(args, stamp: str) -> Path:
    if args.barrier_dir is not None:
        return args.session.resolve() / args.device_id
    return args.output_root.resolve() / f"d405_720p_{args.capture_mode}_{stamp}"


def imu_reader_warmup_frames(joint_mode: bool) -> int:
    return 0 if joint_mode else IMU_WARMUP_FRAMES


def formal_window_complete(*, now_ns, start_ns, duration_s, stop_record):
    if duration_s is not None and (
        not math.isfinite(duration_s) or duration_s <= 0
    ):
        raise ValueError("duration_s must be finite and positive")
    if stop_record is not None:
        return True
    if duration_s is None:
        return False
    return now_ns >= start_ns + int(duration_s * 1_000_000_000)


def poll_joint_formal_window(
    controller,
    *,
    now_ns: int,
    healthy: bool,
    reasons: tuple[str, ...],
    terminal_reason: str | None,
    start_ns: int,
    duration_s: float | None,
) -> bool:
    controller.poll(
        now_ns,
        healthy,
        reasons,
        terminal_reason=terminal_reason,
    )
    stop_record = controller.barrier.read_stop()
    return formal_window_complete(
        now_ns=now_ns,
        start_ns=start_ns,
        duration_s=duration_s,
        stop_record=stop_record,
    )


def required_staging_bytes(duration_s: float | None) -> int:
    if duration_s is None or duration_s <= 0:
        return 0
    return int(CAMERA_RAW_BYTES_PER_SECOND * duration_s * STAGING_HEADROOM_RATIO)


def stream_key(frame) -> str | None:
    profile = frame.get_profile()
    stream = profile.stream_type()
    index = profile.as_video_stream_profile().stream_index()
    if stream == rs.stream.color:
        return "color"
    if stream == rs.stream.depth:
        return "depth"
    if stream == rs.stream.infrared and index == 1:
        return "infrared_left"
    if stream == rs.stream.infrared and index == 2:
        return "infrared_right"
    return None


def select_profiles(sensor) -> list:
    profiles = []
    for profile in sensor.get_stream_profiles():
        video = profile.as_video_stream_profile()
        if video.width() != 1280 or video.height() != 720 or profile.fps() != 30:
            continue
        if (
            "color" in STREAM_KEYS
            and profile.stream_type() == rs.stream.color
            and profile.format() == rs.format.yuyv
        ):
            profiles.append(profile)
        elif (
            "depth" in STREAM_KEYS
            and profile.stream_type() == rs.stream.depth
            and profile.format() == rs.format.z16
        ):
            profiles.append(profile)
        elif (
            profile.stream_type() == rs.stream.infrared
            and profile.format() == rs.format.y8
            and video.stream_index() in (1, 2)
        ):
            profiles.append(profile)
    selected = {stream_key_from_profile(profile) for profile in profiles}
    if selected != set(STREAM_KEYS):
        raise RuntimeError(f"D405 720p 三路 profile 不完整: {sorted(selected)}")
    return profiles


def stream_key_from_profile(profile) -> str | None:
    stream = profile.stream_type()
    index = profile.as_video_stream_profile().stream_index()
    if stream == rs.stream.color:
        return "color"
    if stream == rs.stream.depth:
        return "depth"
    if stream == rs.stream.infrared and index == 1:
        return "infrared_left"
    if stream == rs.stream.infrared and index == 2:
        return "infrared_right"
    return None


def preview_mosaic(frame_map: dict) -> np.ndarray:
    left = np.asanyarray(frame_map["infrared_left"].get_data())
    right = np.asanyarray(frame_map["infrared_right"].get_data())
    left_bgr = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
    right_bgr = cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)

    if "color" in frame_map:
        color_yuyv = np.asanyarray(frame_map["color"].get_data())
        if color_yuyv.dtype == np.uint16:
            color_yuyv = color_yuyv.view(np.uint8).reshape(color_yuyv.shape + (2,))
        primary_label = "RGB"
        primary = cv2.cvtColor(color_yuyv, cv2.COLOR_YUV2BGR_YUY2)
    else:
        primary_label = "Depth"
        primary = np.asanyarray(
            rs.colorizer().colorize(frame_map["depth"]).get_data()
        )

    tiles = []
    for label, image in (
        (primary_label, primary),
        ("IR Left", left_bgr),
        ("IR Right", right_bgr),
    ):
        tile = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)
        cv2.putText(tile, label, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        tiles.append(tile)
    return np.hstack(tiles)


def _main() -> int:
    args = parse_args()
    claim = (
        producer_claim(args.session, args.device_id)
        if args.session is not None
        else nullcontext()
    )
    with claim:
        return _run_claimed(args)


def _run_claimed(args) -> int:
    global STREAMS, STREAM_KEYS, CAMERA_RAW_BYTES_PER_SECOND
    _load_hardware_modules()
    STREAMS = capture_streams_for_mode(args.capture_mode)
    STREAM_KEYS = tuple(name for name, *_ in STREAMS)
    CAMERA_RAW_BYTES_PER_SECOND = 1280 * 720 * (2 + 1 + 1) * 30
    realsense_backend = os.environ.get("THREE_DEVICE_SLAM_REALSENSE_BACKEND", "system-default")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    joint_mode = args.barrier_dir is not None
    joint_controller = None
    joint_health = None
    if joint_mode:
        barrier = BarrierDirectory(barrier_root(args.barrier_dir))
        joint_controller = JointBarrierController(barrier, args.device_id, STREAM_KEYS)
        joint_health = JointCaptureHealth(STREAM_KEYS)
    session = capture_session_directory(args, stamp)
    run_prestart_step(
        joint_controller,
        "storage_error",
        lambda: session.mkdir(parents=True, exist_ok=False),
    )
    bag_path = session / f"d405_720p_{args.capture_mode}.db3"
    frame_csv_path = session / "d405_frames.csv"
    use_ram_stage = (
        not args.no_ram_stage
        and args.duration is not None
        and args.duration > 0
    )
    record_path = bag_path
    staging_required_bytes = required_staging_bytes(args.duration)
    output_free = run_prestart_step(
        joint_controller, "storage_error", lambda: shutil.disk_usage(session).free
    )
    if staging_required_bytes and output_free < staging_required_bytes:
        if joint_controller is not None:
            joint_controller.fail("storage_error", time.monotonic_ns())
        raise RuntimeError(
            "输出硬盘空间不足: "
            f"需要约 {staging_required_bytes / 1e9:.2f}GB, "
            f"可用 {output_free / 1e9:.2f}GB"
        )
    if use_ram_stage:
        run_prestart_step(
            joint_controller,
            "storage_error",
            lambda: args.ram_stage_root.mkdir(parents=True, exist_ok=True),
        )
        staging_free = run_prestart_step(
            joint_controller,
            "storage_error",
            lambda: shutil.disk_usage(args.ram_stage_root).free,
        )
        if staging_free < staging_required_bytes:
            if joint_controller is not None:
                joint_controller.fail("storage_error", time.monotonic_ns())
            raise RuntimeError(
                "内存盘空间不足: "
                f"需要约 {staging_required_bytes / 1e9:.2f}GB, "
                f"可用 {staging_free / 1e9:.2f}GB；可缩短时长或使用--no-ram-stage"
            )
        record_path = args.ram_stage_root / f"{session.name}_{bag_path.name}"

    bag_output_progress = (
        OutputProgress(
            record_path,
            grace_ns=JOINT_OUTPUT_GRACE_NS,
            stale_ns=JOINT_OUTPUT_STALE_NS,
        )
        if joint_mode
        else None
    )
    imu_output_progress = (
        OutputProgress(
            session / "external_imu" / "imu.bin",
            grace_ns=JOINT_OUTPUT_GRACE_NS,
            stale_ns=JOINT_OUTPUT_STALE_NS,
        )
        if joint_mode
        else None
    )

    imu_recorder = UnitRecorder(
        "external_imu", session, save_depth=False, record_gripper=True, max_queue=8000
    )
    recording_active = threading.Event()
    vins_bridge = None
    vins_imu_calibration = None
    live_vins_frames_forwarded = 0
    live_vins_transport = {}

    def current_joint_health(now_ns: int, disconnect_is_terminal: bool):
        imu_health_stats = {
            key: int(getattr(imu, key, 0)) for key in IMU_TRANSPORT_COUNTER_KEYS
        }
        recorder_thread = getattr(imu_recorder, "_thread", None)
        bag_progress_healthy = bag_output_progress.healthy(now_ns)
        imu_progress_healthy = imu_output_progress.healthy(now_ns)
        output_growth_observed = (
            bag_output_progress.observed_growth
            and imu_output_progress.observed_growth
        )
        storage_healthy = (
            imu_recorder.first_write_error is None
            and bag_progress_healthy
            and imu_progress_healthy
            and output_growth_observed
        )
        healthy, reasons, terminal = joint_health.evaluate(
            now_ns,
            imu_stats=imu_health_stats,
            recorder_healthy=(
                imu_recorder.dropped == 0
                and recorder_thread is not None
                and recorder_thread.is_alive()
            ),
            storage_healthy=storage_healthy,
            disconnect_is_terminal=disconnect_is_terminal,
        )
        clock_verified = bool(
            camera_clock_evidence is not None
            and camera_clock_evidence.report()["verified"]
        )
        if not joint_health.joint_ready(now_ns, clock_verified=clock_verified):
            healthy = False
            reasons = tuple((*reasons, "readiness_window"))
        if (
            terminal == "storage_error"
            and imu_recorder.first_write_error is None
            and bag_progress_healthy
            and imu_progress_healthy
        ):
            terminal = None
        return healthy, reasons, terminal

    def record_imu(sample) -> None:
        if recording_active.is_set():
            imu_recorder.put_imu(sample)
        if joint_controller is not None:
            sample_host_ns = int(sample.rx_time * 1_000_000_000)
            joint_controller.observe_imu(sample_host_ns, int(sample.counter))
            joint_health.observe_imu(
                sample_host_ns, float(sample.ts), counter=int(sample.counter)
            )
        if vins_bridge is not None:
            corrected = (
                vins_imu_calibration.apply(sample)
                if vins_imu_calibration is not None
                else sample
            )
            vins_bridge.feed_imu(corrected)

    imu = ImuReader(
        args.imu_port,
        baud=args.imu_baud,
        warmup_frames=imu_reader_warmup_frames(joint_mode),
        on_sample=record_imu,
        on_raw_packet=imu_recorder.put_raw_imu_packet,
        name="all_streams_imu",
    )

    context = run_prestart_step(
        joint_controller, "camera_disconnect", rs.context
    )
    base_device = run_prestart_step(
        joint_controller,
        "camera_disconnect",
        lambda: next(
            (
                device for device in context.query_devices()
                if device.get_info(rs.camera_info.serial_number) == args.serial
            ),
            None,
        ),
    )
    if base_device is None:
        if joint_controller is not None:
            joint_controller.barrier.latch_failure(
                args.device_id, "camera_disconnect", time.monotonic_ns()
            )
            return 2
        raise RuntimeError(f"找不到 D405(serial={args.serial})")
    firmware = run_prestart_step(
        joint_controller,
        "camera_disconnect",
        lambda: base_device.get_info(rs.camera_info.firmware_version),
    )
    usb_type = run_prestart_step(
        joint_controller,
        "camera_disconnect",
        lambda: base_device.get_info(rs.camera_info.usb_type_descriptor),
    )

    recorder_device = run_prestart_step(
        joint_controller,
        "recorder_error",
        lambda: rs.recorder(str(record_path), base_device),
    )
    run_prestart_step(joint_controller, "recorder_error", recorder_device.pause)
    sensor = run_prestart_step(
        joint_controller, "camera_disconnect", recorder_device.first_depth_sensor
    )
    profiles = run_prestart_step(
        joint_controller, "camera_disconnect", lambda: select_profiles(sensor)
    )
    frame_queue = run_prestart_step(
        joint_controller,
        "camera_disconnect",
        lambda: rs.frame_queue(MONITOR_QUEUE_CAPACITY, keep_frames=True),
    )

    stream_stats = {key: StreamContinuity() for key in STREAM_KEYS}
    warmup_counts = {key: 0 for key in STREAM_KEYS}
    warmup_cutoff = {key: -1 for key in STREAM_KEYS}
    latest_frames: dict[str, object] = {}
    last_complete_signature = None
    complete_sets = 0
    formal_start_mono = None
    formal_stop_mono = None
    formal_start_host_ns = None
    formal_start_device_ms = None
    formal_start_imu_counter = None
    imu_stats_start = {}
    imu_stats_end = {}
    imu_warmup_stats = {}
    capture_error = None
    imu_recorder_started = False
    imu_recorder_clean_shutdown = True
    camera_recorder_clean_shutdown = True
    camera_recorder_shutdown_error = None
    shutdown_errors = ()
    imu_started = False
    sensor_opened = False
    sensor_started = False
    recorder_resumed = False
    stage_move_error = None
    stage_move_duration_s = 0.0
    ir_exposure_configuration = None
    camera_clock_evidence = None
    epoch_offset = time.time() - time.monotonic()
    prestart_failure_reason = "camera_disconnect" if joint_mode else None
    try:
        prestart_failure_reason = "storage_error" if joint_mode else None
        run_prestart_step(joint_controller, "storage_error", imu_recorder.start)
        imu_recorder_started = True
        if joint_mode:
            recording_active.set()
        prestart_failure_reason = "imu_disconnect" if joint_mode else None
        imu_opened = run_prestart_step(
            joint_controller, "imu_disconnect", imu.start
        )
        if not imu_opened:
            if joint_controller is not None:
                joint_controller.fail("imu_disconnect", time.monotonic_ns())
            raise RuntimeError(f"无法打开IMU串口: {args.imu_port}")
        imu_started = True

        prestart_failure_reason = "camera_disconnect" if joint_mode else None
        ir_exposure_configuration = run_prestart_step(
            joint_controller,
            "camera_disconnect",
            lambda: configure_ir_auto_exposure(
                sensor,
                exposure_limit_us=args.ir_auto_exposure_limit_us,
                gain_limit=args.ir_auto_gain_limit,
            ),
        )
        if joint_mode:
            global_time_configuration = run_prestart_step(
                joint_controller,
                "camera_disconnect",
                lambda: configure_global_time(sensor),
            )
            camera_clock_evidence = CameraClockEvidence(
                STREAM_KEYS, global_time_configuration
            )
        run_prestart_step(
            joint_controller, "camera_disconnect", lambda: sensor.open(profiles)
        )
        sensor_opened = True
        if joint_mode:
            prestart_failure_reason = "recorder_error"
            run_prestart_step(
                joint_controller, "recorder_error", recorder_device.resume
            )
            recorder_resumed = True
        prestart_failure_reason = "camera_disconnect" if joint_mode else None
        run_prestart_step(
            joint_controller,
            "camera_disconnect",
            lambda: sensor.start(frame_queue),
        )
        sensor_started = True

        print(f"[全流采集] 输出目录: {session}")
        print("[全流采集] 原生 recorder + sensor frame_queue（录制与预览解耦）")
        if use_ram_stage:
            print(
                f"[全流采集] DB3实时写入内存盘，结束后搬到输出目录；"
                f"预估占用 {staging_required_bytes / 1e9:.2f}GB"
            )
        stream_description = (
            "彩色YUYV + 左IR + 右IR"
            if args.capture_mode == "rgb_stereo_ir"
            else "Depth Z16 + 左IR + 右IR"
        )
        print(f"[全流采集] 1280x720@30: {stream_description}；IMU 400Hz")
        print(
            "[全流采集] 双IR自动曝光上限 "
            f"{args.ir_auto_exposure_limit_us:.0f}us，"
            f"自动增益上限 {args.ir_auto_gain_limit:.0f}"
        )
        warmup_storage_note = (
            "预热 raw bag/IMU 保留，并由 acceptance 元数据标记正式边界"
            if joint_mode
            else "预热数据不写入正式文件"
        )
        print(
            f"[全流采集] 预热: 相机每路 {max(0, args.warmup_frames)} 帧，"
            f"IMU {IMU_WARMUP_FRAMES} 帧；{warmup_storage_note}"
        )

        warmup_deadline = time.monotonic() + 15.0
        if joint_mode:
            warmup_complete = False
            while True:
                frame = run_prestart_step(
                    joint_controller,
                    "camera_disconnect",
                    frame_queue.poll_for_frame,
                )
                now_ns = time.monotonic_ns()
                if frame:
                    key = stream_key(frame)
                    if key is not None:
                        camera_clock_evidence.observe(
                            key, frame.get_frame_timestamp_domain()
                        )
                        device_ms = float(frame.get_timestamp())
                        warmup_counts[key] += 1
                        warmup_cutoff[key] = int(frame.get_frame_number())
                        joint_health.observe_camera(
                            key,
                            device_ms,
                            now_ns,
                            frame_number=int(frame.get_frame_number()),
                        )
                        joint_controller.observe_camera(key, device_ms)
                camera_ready = all(
                    count >= max(0, args.warmup_frames)
                    for count in warmup_counts.values()
                )
                imu_ready = imu.frames_ok >= IMU_WARMUP_FRAMES
                warmup_complete = warmup_complete or (camera_ready and imu_ready)
                healthy, reasons, terminal_reason = current_joint_health(
                    now_ns, warmup_complete
                )
                if time.monotonic() >= warmup_deadline and not warmup_complete:
                    terminal_reason = (
                        "camera_disconnect" if not camera_ready else "imu_disconnect"
                    )
                crossed_start_ns = joint_controller.poll(
                    now_ns, healthy, reasons, terminal_reason=terminal_reason
                )
                if crossed_start_ns is not None:
                    if not warmup_complete:
                        joint_controller.fail("warmup_incomplete", now_ns)
                    formal_start_host_ns = crossed_start_ns
                    formal_start_mono = crossed_start_ns / 1_000_000_000
                    while True:
                        queued_frame = run_prestart_step(
                            joint_controller,
                            "camera_disconnect",
                            frame_queue.poll_for_frame,
                        )
                        if not queued_frame:
                            break
                        key = stream_key(queued_frame)
                        if key is not None:
                            camera_clock_evidence.observe(
                                key, queued_frame.get_frame_timestamp_domain()
                            )
                            device_ms = float(queued_frame.get_timestamp())
                            warmup_counts[key] += 1
                            warmup_cutoff[key] = int(queued_frame.get_frame_number())
                            joint_health.observe_camera(
                                key,
                                device_ms,
                                time.monotonic_ns(),
                                frame_number=int(queued_frame.get_frame_number()),
                            )
                            joint_controller.observe_camera(key, device_ms)
                    joint_controller.mark_camera_backlog_drained()
                    prestart_failure_reason = None
                    break
                if not frame:
                    time.sleep(0.005)
        while not joint_mode:
            frame = frame_queue.wait_for_frame(timeout_ms=2000)
            key = stream_key(frame)
            if key is not None:
                warmup_counts[key] += 1
                warmup_cutoff[key] = int(frame.get_frame_number())
            camera_ready = all(
                count >= max(0, args.warmup_frames)
                for count in warmup_counts.values()
            )
            if camera_ready and imu.warmup_stats():
                break
            if time.monotonic() >= warmup_deadline:
                raise RuntimeError(
                    f"预热超时: camera={warmup_counts}, imu={imu.frames_ok}"
                )

        imu_warmup_stats = (
            {
                key: int(getattr(imu, key, 0))
                for key in IMU_TRANSPORT_COUNTER_KEYS
            }
            if joint_mode
            else imu.warmup_stats()
        )

        while not joint_mode:
            queued_frame = frame_queue.poll_for_frame()
            if not queued_frame:
                break
            key = stream_key(queued_frame)
            if key is not None:
                warmup_cutoff[key] = int(queued_frame.get_frame_number())

        imu_stats_snapshot = imu.stats_since_warmup()
        imu_stats_start = {
            key: imu_stats_snapshot.get(key, 0)
            for key in IMU_TRANSPORT_COUNTER_KEYS
        }
        if not recorder_resumed:
            recorder_device.resume()
            recorder_resumed = True
        if not joint_mode:
            formal_start_mono = time.monotonic()
            formal_start_host_ns = int(formal_start_mono * 1_000_000_000)
            recording_active.set()
        capture_window = (
            "直到 stop 请求"
            if args.duration is None
            else f"{args.duration:.1f} 秒"
        )
        print(f"[全流采集] 正式采集 {capture_window}；按 q 可提前结束")

        next_preview_mono = formal_start_mono
        stop_requested = False
        while True:
            if joint_mode:
                frame = frame_queue.poll_for_frame()
                now_ns = time.monotonic_ns()
                healthy, reasons, terminal_reason = current_joint_health(now_ns, True)
                if poll_joint_formal_window(
                    joint_controller,
                    now_ns=now_ns,
                    healthy=healthy,
                    reasons=reasons,
                    terminal_reason=terminal_reason,
                    start_ns=formal_start_host_ns,
                    duration_s=args.duration,
                ):
                    break
                if not frame:
                    time.sleep(0.005)
                    continue
            else:
                frame = frame_queue.wait_for_frame(timeout_ms=2000)
            now_mono = time.monotonic()
            if (
                args.duration is not None
                and now_mono - formal_start_mono >= args.duration
            ):
                break
            key = stream_key(frame)
            if key is None:
                continue
            if camera_clock_evidence is not None:
                camera_clock_evidence.observe(
                    key, frame.get_frame_timestamp_domain()
                )
            number = int(frame.get_frame_number())
            if number <= warmup_cutoff[key]:
                continue
            device_ms = float(frame.get_timestamp())
            if joint_mode:
                now_ns = time.monotonic_ns()
                joint_health.observe_camera(
                    key, device_ms, now_ns, frame_number=number
                )
                joint_controller.observe_camera(key, device_ms)
            stream_stats[key].add(number, device_ms)
            latest_frames[key] = frame

            if len(latest_frames) == len(STREAM_KEYS):
                timestamps_ms = [
                    float(latest_frames[name].get_timestamp())
                    for name in STREAM_KEYS
                ]
                signature = tuple(
                    int(latest_frames[name].get_frame_number())
                    for name in STREAM_KEYS
                )
                aligned = timestamps_aligned(timestamps_ms, SYNC_TOLERANCE_MS)
            else:
                signature = None
                aligned = False

            if aligned and signature != last_complete_signature:
                complete_sets += 1
                if vins_bridge is not None:
                    left_frame = latest_frames["infrared_left"]
                    right_frame = latest_frames["infrared_right"]
                    left_image = np.asanyarray(left_frame.get_data())
                    right_image = np.asanyarray(right_frame.get_data())
                    vins_bridge.feed_camera(
                        CameraFrame(
                            ts=live_vins_timestamp_monotonic(
                                float(left_frame.get_timestamp()), epoch_offset
                            ),
                            color=left_image,
                            depth=None,
                            frame_idx=complete_sets,
                            ts_arrival=now_mono,
                            ts_domain="global_time",
                            frame_number=int(left_frame.get_frame_number()),
                            infrared_left=left_image,
                            infrared_right=right_image,
                        )
                    )
                    live_vins_frames_forwarded += 1
                if not args.no_preview and now_mono >= next_preview_mono:
                    cv2.imshow(
                        f"D405 720p: {args.capture_mode} | IR Left | IR Right",
                        preview_mosaic(latest_frames),
                    )
                    next_preview_mono = now_mono + 0.1
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        stop_requested = True
                last_complete_signature = signature
            if stop_requested:
                break
        formal_stop_mono = time.monotonic()
    except Exception as exc:
        capture_error = f"{type(exc).__name__}: {exc}"
        if joint_controller is not None and not isinstance(exc, JointWorkerFailure):
            try:
                joint_controller.barrier.latch_failure(
                    args.device_id,
                    prestart_failure_reason or "capture_error",
                    time.monotonic_ns(),
                )
            except (OSError, ValueError):
                pass
        print(f"[全流采集] ERROR: {capture_error}")
    finally:
        recording_active.clear()
        if formal_start_mono is not None and formal_stop_mono is None:
            formal_stop_mono = time.monotonic()
        if imu_started:
            imu_stats_end = imu.stats_since_warmup()
        pause_recorder = None
        if recorder_resumed:
            if joint_controller is not None:
                pause_recorder = lambda: pause_joint_recorder_for_shutdown(
                    recorder_device, joint_controller
                )
            else:
                pause_recorder = recorder_device.pause
        shutdown = shutdown_capture_outputs(
            pause_recorder=pause_recorder,
            stop_sensor=sensor.stop if sensor_started else None,
            close_sensor=sensor.close if sensor_opened else None,
            stop_imu=imu.stop if imu_started else None,
            imu_recorder=imu_recorder if imu_recorder_started else None,
        )
        camera_recorder_clean_shutdown = shutdown["camera_clean"]
        camera_recorder_shutdown_error = shutdown["camera_error"]
        imu_recorder_clean_shutdown = shutdown["imu_clean"]
        recorder_shutdown_error = shutdown["recorder_write_error"]
        shutdown_errors = shutdown["errors"]
        if shutdown_errors:
            if capture_error is None:
                capture_error = "; ".join(shutdown_errors)
            if joint_controller is not None:
                try:
                    joint_controller.barrier.latch_failure(
                        args.device_id,
                        (
                            "storage_error"
                            if not imu_recorder_clean_shutdown
                            or recorder_shutdown_error is not None
                            else "capture_error"
                        ),
                        time.monotonic_ns(),
                    )
                except (OSError, ValueError):
                    pass
        if vins_bridge is not None:
            live_vins_transport = vins_bridge.transport_stats()
            vins_bridge.close()
        cv2.destroyAllWindows()
        sensor = None
        recorder_device = None
        base_device = None
        gc.collect()

    if joint_controller is not None:
        formal_start_device_ms = joint_controller.first_formal_camera_device_ms
        formal_start_imu_counter = joint_controller.first_formal_imu_counter

    if (
        use_ram_stage
        and camera_recorder_clean_shutdown
        and record_path.exists()
    ):
        move_start = time.monotonic()
        try:
            print(f"[全流采集] 相机已停止，正在搬运DB3到: {bag_path}")
            shutil.move(str(record_path), str(bag_path))
            stage_move_duration_s = time.monotonic() - move_start
            print(f"[全流采集] DB3搬运完成，用时 {stage_move_duration_s:.1f}s")
        except Exception as exc:
            stage_move_error = f"{type(exc).__name__}: {exc}"
            if capture_error is None:
                capture_error = f"DB3搬运失败: {stage_move_error}；暂存文件保留在 {record_path}"

    camera_clock_report = (
        camera_clock_evidence.report()
        if camera_clock_evidence is not None
        else None
    )
    camera_clock_verified = (
        camera_clock_report is not None and camera_clock_report["verified"]
    )
    joint_timestamp_domain = (
        "global_time" if camera_clock_verified else "unverified"
    )
    joint_clock_domain = (
        "host_monotonic" if camera_clock_verified else "unverified"
    )
    duration = (
        max(formal_stop_mono - formal_start_mono, 1e-9)
        if formal_start_mono is not None and formal_stop_mono is not None
        else 0.0
    )
    source_streams = {key: value.report() for key, value in stream_stats.items()}
    bag_error = None
    bag_streams = {}
    db3_records = {}
    acceptance_records = {}
    csv_error = None
    csv_rows = 0
    formal_csv_rows = 0
    camera_output_stable = (
        camera_recorder_clean_shutdown and stage_move_error is None
    )
    if not camera_output_stable:
        bag_error = (
            camera_recorder_shutdown_error
            or stage_move_error
            or "camera recorder output is not stable"
        )
        csv_error = "authoritative DB3 analysis skipped: output is not stable"
    elif bag_path.exists() and bag_path.stat().st_size > 0:
        try:
            db3_records = read_db3_metadata(bag_path)
            csv_rows = write_frames_csv(
                frame_csv_path,
                db3_records,
                epoch_offset,
                SYNC_TOLERANCE_MS,
                formal_start_device_ms=formal_start_device_ms,
                timestamp_domain=(
                    joint_timestamp_domain if joint_mode else "global_time"
                ),
            )
            if joint_mode and formal_start_device_ms is not None:
                formal_pairs = [
                    pair
                    for pair in pair_metadata_frames(db3_records, SYNC_TOLERANCE_MS)
                    if max(item.device_ms for item in pair.values())
                    >= formal_start_device_ms
                ]
                acceptance_records = {
                    key: [pair[key] for pair in formal_pairs] for key in STREAM_KEYS
                }
                formal_csv_rows = len(formal_pairs)
            elif not joint_mode:
                acceptance_records = db3_records
                formal_csv_rows = csv_rows
            bag_streams = analyze_metadata_records(acceptance_records)
        except Exception as exc:
            bag_error = f"{type(exc).__name__}: {exc}"
            csv_error = bag_error
    else:
        bag_error = "db3 不存在或为空"
    ir_exposure_acceptance = analyze_ir_exposure(
        acceptance_records,
        limit_us=args.ir_auto_exposure_limit_us,
    )

    imu_formal = stats_delta(
        {key: imu_stats_start.get(key, 0) for key in IMU_TRANSPORT_COUNTER_KEYS},
        {key: imu_stats_end.get(key, 0) for key in IMU_TRANSPORT_COUNTER_KEYS},
    ) if imu_stats_start and imu_stats_end else {}
    imu_protocol = str(imu_stats_end.get("protocol", "unknown"))
    imu_path = session / "external_imu" / "imu.bin"
    imu_samples_written = (
        imu_path.stat().st_size // IMU_PACK_SIZE
        if imu_recorder_clean_shutdown and imu_path.exists()
        else (0 if imu_recorder_clean_shutdown else None)
    )
    formal_imu_samples = int(imu_formal.get("frames_ok", 0))
    persisted_formal_lower_bound_ok = persisted_imu_covers_formal(
        clean_shutdown=imu_recorder_clean_shutdown,
        persisted_samples=imu_samples_written,
        formal_samples=formal_imu_samples,
    )
    imu_rate_samples = (
        formal_imu_samples if joint_mode else int(imu_samples_written or 0)
    )
    imu_rate_hz = imu_rate_samples / duration if duration > 0 else 0.0
    recorder_write_error = imu_recorder.first_write_error
    gripper_path = session / "external_imu" / "gripper_encoder.csv"
    gripper_alignment_path = session / "gripper_camera_alignment.csv"
    if not imu_recorder_clean_shutdown:
        gripper_stream = {
            "result": "FAIL",
            "reason": "recorder did not terminate cleanly; output is not stable",
            "rows": 0,
        }
        gripper_alignment = {"result": "SKIPPED", "rows": 0}
        gripper_ok = False
    elif imu_protocol == "stm32_combined_v1":
        gripper_stream = analyze_gripper_csv(gripper_path)
        gripper_alignment = write_gripper_camera_alignment(
            frame_csv_path,
            gripper_path,
            gripper_alignment_path,
            camera_to_imu_td_s=PRODUCT_CAMERA_IMU_TD_S,
        )
        gripper_ok = (
            gripper_stream["result"] == "PASS"
            and gripper_stream["rows"] == imu_samples_written
            and gripper_alignment["result"] == "PASS"
            and gripper_alignment["rows"] == csv_rows
        )
    else:
        gripper_stream = {
            "result": "SKIPPED",
            "reason": f"protocol={imu_protocol}; gripper requires stm32_combined_v1",
            "rows": 0,
        }
        gripper_alignment = {"result": "SKIPPED", "rows": 0}
        gripper_ok = True

    camera_ok = (
        capture_error is None
        and bag_error is None
        and csv_error is None
        and ir_exposure_configuration is not None
        and ir_exposure_acceptance["result"] == "PASS"
        and formal_csv_rows > 1
        and set(bag_streams) == set(STREAM_KEYS)
        and all(
            stream["received"] > 1
            and stream["rate_hz"] >= MIN_CAMERA_RATE_HZ
            and stream["gap_ratio"] <= MAX_CAMERA_GAP_RATIO
            and stream["repeated_frames"] == 0
            and stream["frame_number_resets"] == 0
            and stream["timestamp_regressions"] == 0
            for stream in bag_streams.values()
        )
    )
    imu_ok = (
        (
            not joint_mode
            or (
                imu_recorder_clean_shutdown
                and recorder_write_error is None
                and persisted_formal_lower_bound_ok
            )
        )
        and bool(imu_formal)
        and imu_transport_accepted(
            imu_protocol, imu_rate_hz, imu_formal, imu_recorder.dropped
        )
    )
    live_vins_ok = (
        not args.publish_vins
        or (
            live_vins_frames_forwarded > 1
            and live_vins_transport.get("ros_cam_drop", 1) == 0
            and live_vins_transport.get("ros_imu_pub", 0)
            >= int(imu_samples_written or 0)
        )
    )
    result = classify_d405_result(
        camera_ok,
        imu_ok,
        gripper_ok,
        live_vins_ok,
        camera_clock_verified if joint_mode else True,
    )
    raw_paths = (
        bag_path,
        session / "external_imu" / "raw_imu_packets.bin",
        session / "external_imu" / "imu.bin",
        session / "external_imu" / "imu_ts.csv",
        session / "external_imu" / "gripper_encoder.csv",
        frame_csv_path,
        gripper_alignment_path,
    )
    report = {
        "result": result,
        "reason": "common_clock_unverified" if result == "BLOCKED" else None,
        "session": str(session),
        "bag": str(bag_path),
        "capture_error": capture_error,
        "resolution": [1280, 720],
        "fps_requested": 30,
        "capture_mode": args.capture_mode,
        "color_record_format": "YUYV" if "color" in STREAM_KEYS else None,
        "depth_record_format": "Z16" if "depth" in STREAM_KEYS else None,
        "preview_color_format": "BGR" if "color" in STREAM_KEYS else "Depth colorized",
        "capture_backend": (
            f"rs.recorder + direct sensor frame_queue ({realsense_backend})"
        ),
        "realsense_python_module": str(Path(rs.__file__).resolve()),
        "monitor_queue_capacity": MONITOR_QUEUE_CAPACITY,
        "camera_storage": {
            "ram_staged": use_ram_stage,
            "staging_path": str(record_path) if use_ram_stage else None,
            "staging_required_bytes": staging_required_bytes if use_ram_stage else 0,
            "move_duration_s": stage_move_duration_s,
            "move_error": stage_move_error,
            "recorder_clean_shutdown": camera_recorder_clean_shutdown,
            "recorder_shutdown_error": camera_recorder_shutdown_error,
            "authoritative_output_stable": camera_output_stable,
        },
        "camera_serial": args.serial,
        "camera_firmware": firmware,
        "usb_type": usb_type,
        "warmup_frames": max(0, args.warmup_frames),
        "imu_warmup_frames": IMU_WARMUP_FRAMES,
        "warmup_recorded": joint_mode,
        "complete_framesets_for_csv": csv_rows,
        "monitor_complete_framesets": complete_sets,
        "csv_sync_tolerance_ms": SYNC_TOLERANCE_MS,
        "duration_s": duration,
        "thresholds": {
            "camera_min_rate_hz": MIN_CAMERA_RATE_HZ,
            "camera_max_gap_ratio": MAX_CAMERA_GAP_RATIO,
            "ir_actual_exposure_max_us": args.ir_auto_exposure_limit_us,
            "ir_exposure_tolerance_us": IR_EXPOSURE_LIMIT_TOLERANCE_US,
            "imu_rate_hz": [MIN_IMU_RATE_HZ, MAX_IMU_RATE_HZ],
            "encoder_sensor_pair_delta_us": [0, MAX_ENCODER_PAIR_GAP_US],
            "camera_gripper_nearest_max_ms": MAX_CAMERA_GRIPPER_DELTA_MS,
            "camera_to_imu_td_s": PRODUCT_CAMERA_IMU_TD_S,
        },
        "camera": {
            "result": "PASS" if camera_ok else "FAIL",
            "monitor_queue_streams": source_streams,
            "db3_streams": bag_streams,
            "db3_analysis_error": bag_error,
            "csv_rebuild_error": csv_error,
            "authoritative_stream_stats": "db3_streams",
            "ir_auto_exposure_configuration": ir_exposure_configuration,
            "ir_exposure_acceptance": ir_exposure_acceptance,
            "db3_size_bytes": (
                bag_path.stat().st_size
                if camera_output_stable and bag_path.exists()
                else None
            ),
        },
        "imu": {
            "result": "PASS" if imu_ok else "FAIL",
            "protocol": imu_protocol,
            "warmup_cumulative_stats": imu_warmup_stats,
            "pre_formal_post_warmup_stats": imu_stats_start,
            "formal_window_stats": imu_formal,
            "samples_written": imu_samples_written,
            "recorder_clean_shutdown": imu_recorder_clean_shutdown,
            "rate_hz": imu_rate_hz,
            "recorder_drops": imu_recorder.dropped,
        },
        "gripper_encoder": {
            "required_for_training": imu_protocol == "stm32_combined_v1",
            "result": "PASS" if gripper_ok else "FAIL",
            "stream": gripper_stream,
            "camera_alignment": gripper_alignment,
            "samples_written_by_recorder": imu_recorder.gripper_samples_written,
            "invalid_samples_seen_by_recorder": imu_recorder.gripper_invalid_samples,
            "timestamp_contract": (
                "encoder_ts_mono uses the same frozen STM32 MCU-to-host monotonic "
                "mapping as imu_ts_mono; camera rows carry explicit timestamp "
                "domains and verified global_time maps to host monotonic; "
                "camera association queries "
                "encoder at camera_ts_mono + calibrated td"
            ),
            "training_semantics": (
                "raw_count/angle_deg/closure_ratio are valid under load; "
                "estimated_no_load_gap_mm is not loaded object size"
            ),
        },
        "live_vins": {
            "enabled": args.publish_vins,
            "result": "PASS" if live_vins_ok else "FAIL",
            "frames_forwarded": live_vins_frames_forwarded,
            "imu_calibration": (
                vins_imu_calibration.calibration_id
                if vins_imu_calibration is not None
                else None
            ),
            "transport": live_vins_transport,
        },
        "raw_integrity_semantics": (
            "raw_imu_packets.bin stores header/length-framed wire candidates before "
            "parsing; KT checksum/STM32 CRC remain separate parser aggregate evidence, "
            "not an invented per-record CRC field"
        ),
    }
    if joint_mode:
        report["camera_clock"] = camera_clock_report
        report["formal_complete_framesets_for_csv"] = formal_csv_rows
        report["imu"]["formal_samples"] = imu_rate_samples
        report["imu"]["recorder_write_error"] = recorder_write_error
        report["imu"]["persisted_formal_lower_bound_ok"] = (
            persisted_formal_lower_bound_ok
        )
        report["imu"]["persisted_evidence"] = (
            "imu.bin includes raw warmup samples, so samples_written must be "
            ">= formal_samples"
        )
        report["joint_start"] = build_joint_start_acceptance(
            device_id=args.device_id,
            barrier_dir=args.barrier_dir,
            formal_start_host_monotonic_ns=formal_start_host_ns,
            first_formal_camera_device_ms=formal_start_device_ms,
            first_formal_imu_counter=formal_start_imu_counter,
            clock_domain=joint_clock_domain,
        )
    report = finalize_joint_outputs(
        session=session,
        report=report,
        raw_paths=raw_paths,
        camera_recorder_clean_shutdown=camera_recorder_clean_shutdown,
        imu_recorder_clean_shutdown=imu_recorder_clean_shutdown,
        recorder_write_error=recorder_write_error,
        camera_recorder_shutdown_error=camera_recorder_shutdown_error,
        stage_move_error=stage_move_error,
        shutdown_errors=shutdown_errors,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return {"PASS": 0, "FAIL": 2, "BLOCKED": 3}[report["result"]]


def main() -> int:
    try:
        return _main()
    except JointWorkerFailure as exc:
        return exc.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
