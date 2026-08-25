#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import sys
import threading
import time
from collections import deque
from pathlib import Path

from .capture import (
    CaptureRuntimeError,
    DeviceUnavailableError,
    TwoUQ2Capture,
    TwoUQ2Frame,
    VendorXuLinkError,
)
from three_device_slam.core import BarrierDirectory
from three_device_slam.core import DeviceHeartbeat, SensorRecord
from three_device_slam.core import AppendOnlySessionWriter


MINIMUM_START_GUARD_NS = 100_000_000
LOCAL_WARMUP_NS = 500_000_000
FORMAL_EDGE_LAG_NS = 50_000_000
JOINT_READINESS_WINDOW_NS = 3_000_000_000
MIN_EGO_RATE_HZ = 59.0


class EgoFrameRecorder:
    def __init__(self, writer: AppendOnlySessionWriter):
        self._writer = writer
        self._lock = threading.Lock()
        self._start_ns = None
        self._frame_index = 0
        self._warmup_frames = 0
        self._formal_frames = 0
        self._candidate_written = False
        self._last_valid = False
        self._last_arrival_ns = None
        self._storage_failed = False
        self._readiness_frames = deque()
        self._formal_last_sequence = None
        self._formal_stats = {
            "observed_frames": 0,
            "total_frames": 0,
            "valid_frames": 0,
            "comparable_frames": 0,
            "first_acquisition_ns": None,
            "last_acquisition_ns": None,
            "first_arrival_ns": None,
            "last_arrival_ns": None,
            "measured_width": None,
            "measured_height": None,
            "resolution_mismatches": 0,
            "valid_sequences": 0,
            "duplicate_sequences": 0,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
            "xu_failures": 0,
            "bad_jpegs": 0,
            "timestamp_regressions": 0,
            "timestamp_unavailable": 0,
        }

    def set_start_ns(self, start_ns: int) -> None:
        if isinstance(start_ns, bool) or not isinstance(start_ns, int) or start_ns < 0:
            raise ValueError("start_ns must be non-negative integer nanoseconds")
        with self._lock:
            if self._start_ns is not None and self._start_ns != start_ns:
                raise RuntimeError("scheduled start changed")
            self._start_ns = start_ns

    def handle(self, frame: TwoUQ2Frame) -> None:
        with self._lock:
            self._frame_index += 1
            record_sequence = self._frame_index
            timestamp_available = frame.acquisition_ns is not None
            warmup = (
                not timestamp_available
                or self._start_ns is None
                or frame.acquisition_ns < self._start_ns
            )
            observed_after_start = self._start_ns is not None and (
                frame.acquisition_ns >= self._start_ns
                if timestamp_available
                else frame.arrival_ns >= self._start_ns
            )
            if warmup:
                self._warmup_frames += 1
            else:
                self._formal_frames += 1
            candidate = not warmup and frame.valid and not self._candidate_written
            if candidate:
                self._candidate_written = True
            common_metadata = {
                "vendor_sequence": frame.sequence,
                "task_start_candidate": candidate,
                "error_reason": frame.error_reason,
                "acquisition_available": timestamp_available,
            }
            stored_acquisition_ns = frame.acquisition_ns if timestamp_available else 0
            clock_domain = (
                "host_monotonic"
                if timestamp_available
                else "gstreamer_timestamp_unavailable"
            )
            self._append(
                SensorRecord(
                    stream_id="ego.video",
                    sequence=record_sequence,
                    acquisition_ns=stored_acquisition_ns,
                    arrival_ns=frame.arrival_ns,
                    clock_domain=clock_domain,
                    warmup=warmup,
                    valid=frame.valid,
                    metadata={
                        **common_metadata,
                        "encoding": "MJPG",
                        "width": frame.width,
                        "height": frame.height,
                    },
                ),
                frame.jpeg,
            )
            self._append(
                SensorRecord(
                    stream_id="ego.xu",
                    sequence=record_sequence,
                    acquisition_ns=stored_acquisition_ns,
                    arrival_ns=frame.arrival_ns,
                    clock_domain=clock_domain,
                    warmup=warmup,
                    valid=frame.valid,
                    metadata={**common_metadata, "size_expected": 27},
                ),
                frame.xu_raw,
            )
            self._last_valid = frame.valid
            self._last_arrival_ns = frame.arrival_ns
            self._readiness_frames.append(
                (
                    frame.arrival_ns,
                    frame.acquisition_ns,
                    frame.sequence,
                    frame.valid,
                    frame.error_reason,
                )
            )
            self._prune_readiness(frame.arrival_ns)
            if observed_after_start:
                self._observe_formal(frame)

    def _observe_formal(self, frame: TwoUQ2Frame) -> None:
        stats = self._formal_stats
        stats["observed_frames"] += 1
        stats["total_frames"] += 1
        stats["valid_frames"] += int(frame.valid)
        if stats["first_arrival_ns"] is None:
            stats["first_arrival_ns"] = frame.arrival_ns
        stats["last_arrival_ns"] = frame.arrival_ns
        stats["measured_width"] = frame.width
        stats["measured_height"] = frame.height
        if (frame.width, frame.height) != (3840, 1080):
            stats["resolution_mismatches"] += 1

        errors = set(filter(None, frame.error_reason.split(",")))
        stats["xu_failures"] += int("xu_read_failed" in errors)
        stats["bad_jpegs"] += int("bad_jpeg" in errors)
        stats["timestamp_unavailable"] += int("timestamp_unavailable" in errors)

        if frame.acquisition_ns is not None:
            previous = stats["last_acquisition_ns"]
            if previous is None:
                stats["first_acquisition_ns"] = frame.acquisition_ns
            elif frame.acquisition_ns < previous:
                stats["timestamp_regressions"] += 1
            stats["last_acquisition_ns"] = frame.acquisition_ns
            stats["comparable_frames"] += 1

        if frame.sequence is not None:
            stats["valid_sequences"] += 1
            if self._formal_last_sequence is not None:
                delta = (frame.sequence - self._formal_last_sequence) & 0xFFFFFF
                if delta == 0:
                    stats["duplicate_sequences"] += 1
                elif delta < 0x800000:
                    stats["sequence_gaps"] += max(0, delta - 1)
                else:
                    stats["sequence_regressions"] += 1
            self._formal_last_sequence = frame.sequence

    def _append(self, record: SensorRecord, payload: bytes) -> None:
        try:
            self._writer.append(record, payload)
        except (OSError, RuntimeError):
            self._storage_failed = True
            raise

    def raise_if_storage_failed(self) -> None:
        with self._lock:
            failed = self._storage_failed
        if failed:
            raise StorageIoFailure()

    def counts(self) -> dict:
        with self._lock:
            return {
                "warmup_frames": self._warmup_frames,
                "formal_frames": self._formal_frames,
            }

    def formal_stats(self) -> dict:
        with self._lock:
            result = dict(self._formal_stats)
        first_ns = result["first_acquisition_ns"]
        last_ns = result["last_acquisition_ns"]
        elapsed_ns = last_ns - first_ns if first_ns is not None and last_ns is not None else 0
        result["rate_hz"] = (
            (result["comparable_frames"] - 1) * 1_000_000_000 / elapsed_ns
            if result["comparable_frames"] > 1 and elapsed_ns > 0
            else 0.0
        )
        result["gap_ratio"] = (
            result["sequence_gaps"] / result["valid_sequences"]
            if result["valid_sequences"]
            else 0.0
        )
        return result

    def healthy(self, now_ns: int | None = None) -> bool:
        with self._lock:
            recent = (
                now_ns is None
                or self._last_arrival_ns is not None
                and now_ns - self._last_arrival_ns <= 200_000_000
            )
            return self._frame_index > 0 and self._last_valid and recent

    def _prune_readiness(self, now_ns: int) -> None:
        cutoff = now_ns - JOINT_READINESS_WINDOW_NS
        while self._readiness_frames and self._readiness_frames[0][0] < cutoff:
            self._readiness_frames.popleft()

    def joint_ready(self, now_ns: int) -> bool:
        with self._lock:
            self._prune_readiness(now_ns)
            rows = tuple(self._readiness_frames)
        maximum_frame_period_ns = int(1_000_000_000 / MIN_EGO_RATE_HZ)
        if not rows or now_ns - rows[0][0] < (
            JOINT_READINESS_WINDOW_NS - maximum_frame_period_ns
        ):
            return False
        elapsed = rows[-1][0] - rows[0][0]
        if elapsed <= 0 or (len(rows) - 1) * 1_000_000_000 / elapsed < MIN_EGO_RATE_HZ:
            return False
        if any(not valid or error for _, _, _, valid, error in rows):
            return False
        acquisitions = [row[1] for row in rows]
        if any(value is None for value in acquisitions):
            return False
        if any(right < left for left, right in zip(acquisitions, acquisitions[1:])):
            return False
        sequences = [row[2] for row in rows]
        if any(value is None for value in sequences):
            return False
        return all(
            ((right - left) & 0xFFFFFF) == 1
            for left, right in zip(sequences, sequences[1:])
        )


_CHECKS = {
    "resolution": (
        "== 3840x1080",
        lambda stats: [stats.get("measured_width"), stats.get("measured_height")],
        lambda value: value == [3840, 1080],
    ),
    "resolution_mismatches": ("== 0", lambda stats: int(stats.get("resolution_mismatches", 0)), lambda value: value == 0),
    "rate_hz": (
        ">= 59.0",
        lambda stats: stats.get("window_rate_hz"),
        lambda value: value is not None and value >= 59.0,
    ),
    "timestamp_regressions": ("== 0", lambda stats: int(stats.get("timestamp_regressions", 0)), lambda value: value == 0),
    "timestamp_unavailable": ("== 0", lambda stats: int(stats.get("timestamp_unavailable", 0)), lambda value: value == 0),
    "xu_failures": ("== 0", lambda stats: int(stats.get("xu_failures", 0)), lambda value: value == 0),
    "duplicate_sequences": ("== 0", lambda stats: int(stats.get("duplicate_sequences", 0)), lambda value: value == 0),
    "sequence_regressions": ("== 0", lambda stats: int(stats.get("sequence_regressions", 0)), lambda value: value == 0),
    "gap_ratio": (
        "<= 0.001",
        lambda stats: int(stats.get("sequence_gaps", 0)) / max(1, int(stats.get("valid_sequences", 0))),
        lambda value: value <= 0.001,
    ),
    "bad_jpegs": ("== 0", lambda stats: int(stats.get("bad_jpegs", 0)), lambda value: value == 0),
}


def build_acceptance(
    *,
    stats: dict,
    formal_stats: dict | None = None,
    counts: dict,
    manifest: dict,
    evidence: dict,
    blocked_reason: str | None = None,
    failure=None,
    requested_duration_s: float | None = None,
    measured_formal_duration_s: float | None = None,
    formal_start_ns: int | None = None,
    formal_deadline_ns: int | None = None,
    formal_end_ns: int | None = None,
) -> dict:
    qualification = dict(formal_stats or {})
    formal_measurements_available = int(qualification.get("observed_frames", 0)) > 0
    if not formal_measurements_available:
        window_rate_hz = None
    elif requested_duration_s is not None and requested_duration_s > 0:
        window_rate_hz = int(qualification.get("valid_frames", 0)) / requested_duration_s
    else:
        window_rate_hz = qualification.get("rate_hz")
    qualification["window_rate_hz"] = window_rate_hz
    if failure is not None:
        forced_status = failure.status
        forced_reason = failure.reason
    else:
        forced_status = None
        forced_reason = None
    if failure is None and blocked_reason is None and formal_measurements_available and (
        int(qualification.get("duplicate_sequences", 0)) > 0
        or int(qualification.get("sequence_regressions", 0)) > 0
    ):
        blocked_reason = "2uq2_xu_frame_relation"
    checks = {}
    for name, (threshold, measure, accepted) in _CHECKS.items():
        measurement = (
            None
            if not formal_measurements_available
            else measure(qualification)
        )
        if forced_status is not None:
            check_status = forced_status
        elif blocked_reason:
            check_status = "BLOCKED"
        elif not formal_measurements_available:
            check_status = "FAIL"
        else:
            check_status = "PASS" if accepted(measurement) else "FAIL"
        checks[name] = {
            "status": check_status,
            "threshold": threshold,
            "measurement": measurement,
            "scope": "formal_window",
        }

    valid_formal_frames = (
        int(qualification.get("valid_frames", 0))
        if formal_measurements_available
        else None
    )
    checks["valid_formal_frames"] = {
        "status": (
            forced_status
            if forced_status is not None
            else "BLOCKED"
            if blocked_reason
            else "PASS"
            if valid_formal_frames is not None and valid_formal_frames >= 1
            else "FAIL"
        ),
        "threshold": ">= 1",
        "measurement": valid_formal_frames,
        "scope": "formal_window",
    }

    first_formal_ns = (
        qualification.get("first_acquisition_ns")
        if formal_measurements_available
        else None
    )
    start_covered = (
        first_formal_ns is not None
        and formal_start_ns is not None
        and first_formal_ns <= formal_start_ns + FORMAL_EDGE_LAG_NS
    )
    checks["formal_start_coverage"] = {
        "status": (
            forced_status
            if forced_status is not None
            else "BLOCKED"
            if blocked_reason
            else "PASS"
            if start_covered
            else "FAIL"
        ),
        "threshold": f"first comparable frame within {FORMAL_EDGE_LAG_NS} ns of start",
        "measurement": first_formal_ns,
        "scope": "formal_window",
    }

    last_formal_ns = (
        qualification.get("last_acquisition_ns")
        if formal_measurements_available
        else None
    )
    formal_end_boundary_ns = (
        formal_deadline_ns if formal_deadline_ns is not None else formal_end_ns
    )
    end_covered = (
        last_formal_ns is not None
        and formal_end_boundary_ns is not None
        and last_formal_ns >= formal_end_boundary_ns - FORMAL_EDGE_LAG_NS
    )
    checks["formal_end_coverage"] = {
        "status": (
            forced_status
            if forced_status is not None
            else "BLOCKED"
            if blocked_reason
            else "PASS"
            if end_covered
            else "FAIL"
        ),
        "threshold": (
            f"last comparable frame within {FORMAL_EDGE_LAG_NS} ns of "
            f"{'deadline' if formal_deadline_ns is not None else 'stop'}"
        ),
        "measurement": last_formal_ns,
        "scope": "formal_window",
    }


    for name in ("callback_exceptions", "capture_errors"):
        value = int(stats[name]) if name in stats else 0
        diagnostic_available = name in stats and (
            int(stats.get("frames", 0)) > 0 or value != 0
        )
        measurement = value if diagnostic_available else None
        if measurement is None or forced_status == "BLOCKED" or blocked_reason:
            diagnostic_status = "BLOCKED"
        else:
            diagnostic_status = "PASS" if measurement == 0 else "FAIL"
        checks[name] = {
            "status": diagnostic_status,
            "threshold": "== 0",
            "measurement": measurement,
            "scope": "whole_run",
        }

    if requested_duration_s is not None:
        if forced_status is not None:
            duration_status = forced_status
        elif blocked_reason:
            duration_status = "BLOCKED"
        else:
            duration_status = (
                "PASS"
                if measured_formal_duration_s is not None
                and measured_formal_duration_s >= requested_duration_s
                else "FAIL"
            )
        checks["formal_duration_s"] = {
            "status": duration_status,
            "threshold": f">= {requested_duration_s}",
            "measurement": measured_formal_duration_s,
            "scope": "formal_window",
        }

    if forced_status is not None:
        status = forced_status
    elif blocked_reason:
        status = "BLOCKED"
    elif any(check["status"] == "FAIL" for check in checks.values()):
        status = "FAIL"
    elif any(check["status"] == "BLOCKED" for check in checks.values()):
        status = "BLOCKED"
    else:
        status = "PASS"
    hashes = {key: value for key, value in evidence.items() if key.endswith("_sha256")}
    hashes["streams"] = manifest.get("streams", {})
    tool_versions = {key: value for key, value in evidence.items() if not key.endswith("_sha256")}
    measurements_available = formal_measurements_available
    report = {
        "schema": "ego.2uq2.acceptance.v1",
        "status": status,
        "checks": checks,
        "counts": dict(counts),
        "formal_window": {
            "requested_duration_s": requested_duration_s,
            "measured_duration_s": measured_formal_duration_s,
            "formal_frames": counts.get("formal_frames", 0),
            "start_ns": formal_start_ns,
            "deadline_ns": formal_deadline_ns,
            "edge_lag_threshold_ns": FORMAL_EDGE_LAG_NS,
        },
        "formal_evidence": dict(qualification),
        "formal_evidence_scope": "formal_window",
        "capture_diagnostics": dict(stats),
        "capture_diagnostics_scope": "whole_run",
        "rates": {
            "video_hz": (
                float(window_rate_hz)
                if window_rate_hz is not None
                else None
            ),
            "observed_span_hz": (
                float(qualification.get("rate_hz", 0.0))
                if measurements_available
                else None
            ),
            "scope": "formal_window",
        },
        "sequence_evidence": {
            key: qualification.get(key, 0) if measurements_available else None
            for key in (
                "valid_sequences",
                "duplicate_sequences",
                "sequence_gaps",
                "sequence_regressions",
            )
        },
        "xu_failures": qualification.get("xu_failures", 0) if measurements_available else None,
        "bad_jpegs": qualification.get("bad_jpegs", 0) if measurements_available else None,
        "hashes": hashes,
        "tool_versions": tool_versions,
    }
    if forced_reason or blocked_reason:
        report["reason"] = forced_reason or blocked_reason
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Capture encoded 2UQ2 MJPEG and frame-linked XU data")
    parser.add_argument("--device", required=True)
    parser.add_argument("--xu-library", required=True)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--barrier-dir", type=Path)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--device-id", choices=("ego",), default="ego")
    args = parser.parse_args(argv)
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration must be positive")
    return args


def _sha256_if_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def _evidence(xu_library: Path) -> dict:
    evidence = {"python": platform.python_version(), "gstreamer": "unavailable"}
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        evidence["gstreamer"] = Gst.version_string()
    except (ImportError, ValueError):
        pass
    library_hash = _sha256_if_file(xu_library)
    if library_hash is not None:
        evidence["xu_library_sha256"] = library_hash
    return evidence


def _barrier_root(path: Path) -> Path:
    return path.parent if path.name == "barrier" else path


def _write_acceptance(path: Path, report: dict) -> None:
    temporary = path.with_suffix(".json.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as file:
            json.dump(report, file, sort_keys=True, indent=2)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary.exists():
            temporary.unlink()


def acceptance_exit_code(status: str) -> int:
    return {"PASS": 0, "FAIL": 2, "BLOCKED": 3}[status]


class WorkerFailure(RuntimeError):
    status = "FAIL"
    reason = "worker_failure"

    def __init__(self, detail: str = ""):
        self.detail = detail
        super().__init__(f"{self.reason}:{detail}" if detail else self.reason)


class VendorXuLinkFailure(WorkerFailure):
    status = "BLOCKED"
    reason = "vendor_xu_link"


class HardwareUnavailableFailure(WorkerFailure):
    status = "BLOCKED"
    reason = "2uq2_hardware_unavailable"


class BarrierIoFailure(WorkerFailure):
    reason = "barrier_io"


class StorageIoFailure(WorkerFailure):
    reason = "storage_io"


class CaptureRuntimeFailure(WorkerFailure):
    reason = "2uq2_capture_runtime"

    def __init__(self, terminal_reason: str):
        self.terminal_reason = terminal_reason
        super().__init__(terminal_reason)


class StaleScheduledStart(WorkerFailure):
    status = "BLOCKED"
    reason = "stale_scheduled_start"

    def __init__(self):
        super().__init__()


def capture_start_failure(exc: Exception) -> WorkerFailure:
    if isinstance(exc, VendorXuLinkError):
        return VendorXuLinkFailure()
    if isinstance(exc, DeviceUnavailableError):
        return HardwareUnavailableFailure()
    return CaptureRuntimeFailure("capture_start")


def _check_capture_terminal(capture: TwoUQ2Capture, recorder=None) -> None:
    if recorder is not None:
        recorder.raise_if_storage_failed()
    reason = capture.poll_terminal()
    if reason is not None:
        raise CaptureRuntimeFailure(reason)


def wait_for_scheduled_start(
    barrier,
    capture,
    *,
    clock_ns=time.monotonic_ns,
    sleep=time.sleep,
    on_poll=None,
    recorder=None,
) -> int:
    while True:
        _check_capture_terminal(capture, recorder)
        try:
            start_ns = barrier.read_start_ns()
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            raise BarrierIoFailure() from exc
        if start_ns is not None:
            if start_ns - clock_ns() < MINIMUM_START_GUARD_NS:
                raise StaleScheduledStart()
            if recorder is not None:
                recorder.set_start_ns(start_ns)
            return start_ns
        now_ns = clock_ns()
        if on_poll is not None:
            on_poll(now_ns)
        sleep(0.02)


def run_formal_window(
    capture,
    *,
    barrier,
    start_ns: int,
    duration_s: float | None,
    clock_ns=time.monotonic_ns,
    sleep=time.sleep,
    on_poll=None,
    recorder=None,
) -> int:
    while True:
        _check_capture_terminal(capture, recorder)
        now_ns = clock_ns()
        try:
            stop_record = barrier.read_stop()
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            raise BarrierIoFailure() from exc
        if formal_window_complete(
            now_ns=now_ns,
            start_ns=start_ns,
            duration_s=duration_s,
            stop_record=stop_record,
        ):
            return now_ns
        if on_poll is not None:
            on_poll(now_ns)
        sleep(0.02)


def schedule_local_start(barrier, *, clock_ns=time.monotonic_ns) -> int:
    start_ns = clock_ns() + LOCAL_WARMUP_NS
    try:
        barrier.schedule_start(start_ns)
    except OSError as exc:
        raise BarrierIoFailure() from exc
    return start_ns


def formal_deadline_ns(start_ns: int, duration_s: float) -> int:
    return start_ns + int(duration_s * 1_000_000_000)


def formal_window_complete(*, now_ns, start_ns, duration_s, stop_record):
    if stop_record is not None:
        return True
    if duration_s is None:
        return False
    return now_ns >= start_ns + int(duration_s * 1_000_000_000)


def setup_storage(session: Path, *, writer_factory=AppendOnlySessionWriter):
    ego_directory = Path(session) / "ego"
    try:
        ego_directory.mkdir(parents=True, exist_ok=True)
        writer = writer_factory(ego_directory)
    except (OSError, RuntimeError) as exc:
        raise StorageIoFailure() from exc
    return ego_directory, writer


def run(args) -> int:
    try:
        ego_directory, writer = setup_storage(args.session)
    except StorageIoFailure as storage_failure:
        ego_directory = args.session / "ego"
        try:
            ego_directory.mkdir(parents=True, exist_ok=True)
            _write_acceptance(
                ego_directory / "acceptance.json",
                build_acceptance(
                    stats={},
                    formal_stats={},
                    counts={"warmup_frames": 0, "formal_frames": 0},
                    manifest={"streams": {}},
                    evidence={"python": platform.python_version()},
                    failure=storage_failure,
                    requested_duration_s=args.duration,
                ),
            )
        except (OSError, RuntimeError):
            pass
        return acceptance_exit_code("FAIL")
    barrier_path = args.barrier_dir if args.barrier_dir is not None else args.session
    failure = None
    try:
        barrier = BarrierDirectory(_barrier_root(barrier_path))
    except OSError:
        barrier = None
        failure = BarrierIoFailure()
    recorder = EgoFrameRecorder(writer)
    capture = TwoUQ2Capture(args.device, args.xu_library, recorder.handle)
    capture_started = False
    start_ns = None
    deadline_ns = None
    formal_end_ns = None
    last_heartbeat_ns = 0

    def heartbeat(now_ns: int) -> None:
        nonlocal last_heartbeat_ns
        if now_ns - last_heartbeat_ns < 50_000_000:
            return
        capture_stats = capture.stats()
        healthy = (
            recorder.joint_ready(now_ns)
            and capture_stats["callback_exceptions"] == 0
            and capture_stats["capture_errors"] == 0
        )
        try:
            barrier.write_heartbeat(
                DeviceHeartbeat(
                    "ego", now_ns, healthy, () if healthy else ("no_healthy_frame",)
                )
            )
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            raise BarrierIoFailure() from exc
        last_heartbeat_ns = now_ns

    try:
        if failure is not None:
            raise failure
        capture.start()
        capture_started = True
        if args.barrier_dir is None:
            try:
                schedule_local_start(barrier)
            except FileExistsError:
                pass

        start_ns = wait_for_scheduled_start(
            barrier, capture, on_poll=heartbeat, recorder=recorder
        )
        deadline_ns = (
            formal_deadline_ns(start_ns, args.duration)
            if args.duration is not None
            else None
        )
        formal_end_ns = run_formal_window(
            capture,
            barrier=barrier,
            start_ns=start_ns,
            duration_s=args.duration,
            on_poll=heartbeat,
            recorder=recorder,
        )
    except WorkerFailure as exc:
        failure = exc
    except (
        VendorXuLinkError,
        DeviceUnavailableError,
        CaptureRuntimeError,
        OSError,
        RuntimeError,
    ) as exc:
        failure = (
            CaptureRuntimeFailure("capture_exception")
            if capture_started
            else capture_start_failure(exc)
        )
    finally:
        if start_ns is not None and formal_end_ns is None:
            formal_end_ns = time.monotonic_ns()
        try:
            capture.stop()
        except (OSError, RuntimeError):
            failure = failure or CaptureRuntimeFailure("capture_stop")
        try:
            manifest = writer.close()
        except (OSError, RuntimeError):
            manifest = {"streams": {}}
            failure = StorageIoFailure()

    stats = capture.stats()
    report = build_acceptance(
        stats=stats,
        formal_stats=recorder.formal_stats(),
        counts=recorder.counts(),
        manifest=manifest,
        evidence=_evidence(Path(args.xu_library)),
        failure=failure,
        requested_duration_s=args.duration,
        measured_formal_duration_s=(
            max(0.0, (formal_end_ns - start_ns) / 1_000_000_000)
            if start_ns is not None and formal_end_ns is not None
            else None
        ),
        formal_start_ns=start_ns,
        formal_deadline_ns=deadline_ns,
        formal_end_ns=formal_end_ns,
    )
    try:
        _write_acceptance(ego_directory / "acceptance.json", report)
    except (OSError, RuntimeError):
        return acceptance_exit_code("FAIL")
    if report["status"] != "PASS":
        reason = report.get("reason") or next(
            name for name, check in report["checks"].items() if check["status"] == "FAIL"
        )
        if barrier is not None:
            try:
                barrier.latch_failure("ego", reason, time.monotonic_ns())
            except OSError:
                pass
    return acceptance_exit_code(report["status"])


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
