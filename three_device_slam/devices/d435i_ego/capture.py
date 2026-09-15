"""Pure contracts and acceptance metrics for the temporary D435i Ego."""

from __future__ import annotations

import statistics
from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence


@dataclass(frozen=True)
class D435iContract:
    width: int = 1280
    height: int = 720
    camera_hz: int = 30
    gyro_hz: int = 200
    accel_hz: int = 200
    writer_queue_depth: int = 512


@dataclass(frozen=True)
class StreamSample:
    sequence: int
    acquisition_ns: int
    arrival_ns: int
    timestamp_domain: str
    width: int | None = None
    height: int | None = None


def configure_streams(rs, config, serial: str, contract: D435iContract) -> None:
    """Apply the exact temporary-Ego stream contract to an SDK config."""
    if not isinstance(serial, str) or not serial.strip():
        raise ValueError("serial must be a non-empty string")
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.infrared,
        1,
        contract.width,
        contract.height,
        rs.format.y8,
        contract.camera_hz,
    )
    config.enable_stream(
        rs.stream.infrared,
        2,
        contract.width,
        contract.height,
        rs.format.y8,
        contract.camera_hz,
    )
    config.enable_stream(
        rs.stream.gyro, rs.format.motion_xyz32f, contract.gyro_hz
    )
    config.enable_stream(
        rs.stream.accel, rs.format.motion_xyz32f, contract.accel_hz
    )


def derive_monotonic_acquisition_ns(
    *, device_timestamp_ms: float, wall_to_monotonic_offset_ns: int
) -> int:
    """Map RealSense global_time (wall milliseconds) into host monotonic ns."""
    if device_timestamp_ms < 0:
        raise ValueError("device_timestamp_ms must be non-negative")
    whole_ms = int(device_timestamp_ms)
    fractional_ns = int(round((device_timestamp_ms - whole_ms) * 1_000_000))
    device_timestamp_ns = whole_ms * 1_000_000 + fractional_ns
    acquisition_ns = device_timestamp_ns - int(wall_to_monotonic_offset_ns)
    if acquisition_ns < 0:
        raise ValueError("mapped acquisition timestamp is negative")
    return acquisition_ns


def is_warmup_sample(
    *, formal_start_ns: int | None, acquisition_ns: int, timestamp_valid: bool
) -> bool:
    """Classify the formal edge by acquisition time, never USB arrival time."""
    return (
        formal_start_ns is None
        or not timestamp_valid
        or acquisition_ns < formal_start_ns
    )


def evaluate_stream(
    samples: Sequence[StreamSample],
    *,
    expected_hz: float,
    hz_tolerance: float,
    expected_resolution: tuple[int, int] | None = None,
) -> dict:
    """Evaluate one formal-window stream without reordering its evidence."""
    if expected_hz <= 0 or not 0 <= hz_tolerance < 1:
        raise ValueError("invalid rate contract")

    timestamp_regressions = 0
    timestamp_repeats = 0
    sequence_gaps = 0
    sequence_regressions = 0
    sequence_repeats = 0
    positive_intervals_ms = []
    for previous, current in zip(samples, samples[1:]):
        timestamp_delta = current.acquisition_ns - previous.acquisition_ns
        if timestamp_delta < 0:
            timestamp_regressions += 1
        elif timestamp_delta == 0:
            timestamp_repeats += 1
        else:
            positive_intervals_ms.append(timestamp_delta / 1_000_000)

        sequence_delta = current.sequence - previous.sequence
        if sequence_delta < 0:
            sequence_regressions += 1
        elif sequence_delta == 0:
            sequence_repeats += 1
        elif sequence_delta > 1:
            sequence_gaps += sequence_delta - 1

    resolution_mismatches = 0
    if expected_resolution is not None:
        resolution_mismatches = sum(
            (sample.width, sample.height) != expected_resolution for sample in samples
        )

    measured_hz = 0.0
    if len(samples) >= 2:
        span_ns = samples[-1].acquisition_ns - samples[0].acquisition_ns
        if span_ns > 0:
            measured_hz = (len(samples) - 1) * 1_000_000_000 / span_ns

    minimum_hz = expected_hz * (1 - hz_tolerance)
    maximum_hz = expected_hz * (1 + hz_tolerance)
    domains = sorted({sample.timestamp_domain for sample in samples})
    status = "PASS"
    if not (
        len(samples) >= 2
        and minimum_hz <= measured_hz <= maximum_hz
        and timestamp_regressions == 0
        and timestamp_repeats == 0
        and sequence_gaps == 0
        and sequence_regressions == 0
        and sequence_repeats == 0
        and resolution_mismatches == 0
        and domains == ["global_time"]
    ):
        status = "FAIL"

    return {
        "status": status,
        "samples": len(samples),
        "measured_hz": measured_hz,
        "expected_hz": expected_hz,
        "threshold_hz": [minimum_hz, maximum_hz],
        "interval_ms_p50": _percentile(positive_intervals_ms, 0.50),
        "interval_ms_p95": _percentile(positive_intervals_ms, 0.95),
        "interval_ms_max": max(positive_intervals_ms, default=None),
        "timestamp_regressions": timestamp_regressions,
        "timestamp_repeats": timestamp_repeats,
        "sequence_gaps": sequence_gaps,
        "sequence_regressions": sequence_regressions,
        "sequence_repeats": sequence_repeats,
        "resolution_mismatches": resolution_mismatches,
        "timestamp_domains": domains,
    }


def evaluate_stereo_pairing(
    left: Sequence[StreamSample], right: Sequence[StreamSample]
) -> dict:
    left_by_sequence = {sample.sequence: sample for sample in left}
    right_by_sequence = {sample.sequence: sample for sample in right}
    shared = sorted(left_by_sequence.keys() & right_by_sequence.keys())
    skew_ms = [
        abs(
            left_by_sequence[sequence].acquisition_ns
            - right_by_sequence[sequence].acquisition_ns
        )
        / 1_000_000
        for sequence in shared
    ]
    unmatched = len(left) + len(right) - 2 * len(shared)
    maximum_skew_ms = max(skew_ms, default=None)
    status = (
        "PASS"
        if shared
        and unmatched == 0
        and maximum_skew_ms is not None
        and maximum_skew_ms <= 1.0
        else "FAIL"
    )
    return {
        "status": status,
        "paired_frames": len(shared),
        "unmatched_frames": unmatched,
        "skew_ms_p95": _percentile(skew_ms, 0.95),
        "skew_ms_max": maximum_skew_ms,
        "threshold": "all frames paired by sequence and max skew <= 1.0 ms",
    }


def build_capture_acceptance(
    streams: Mapping[str, Mapping],
    *,
    writer_queue_drops: int,
    operator_aborted: bool,
    capture_error: str | None,
    stereo_pairing: Mapping | None = None,
    formal_start_ns: int | None = None,
    first_acquisition_ns: int | None = None,
) -> dict:
    required = ("ir_left", "ir_right", "gyro", "accel")
    missing = [name for name in required if name not in streams]
    checks = {
        "required_streams": {"status": "PASS" if not missing else "FAIL", "missing": missing},
        "writer_queue_drops": {
            "status": "PASS" if writer_queue_drops == 0 else "FAIL",
            "measurement": writer_queue_drops,
            "threshold": "== 0",
        },
        "operator_aborted": {
            "status": "PASS" if not operator_aborted else "FAIL",
            "measurement": operator_aborted,
            "threshold": "== false",
        },
        "capture_error": {
            "status": "PASS" if capture_error is None else "FAIL",
            "measurement": capture_error,
            "threshold": "== null",
        },
    }
    if stereo_pairing is not None:
        checks["stereo_pairing"] = dict(stereo_pairing)
    if formal_start_ns is not None:
        start_lag_ns = (
            first_acquisition_ns - formal_start_ns
            if first_acquisition_ns is not None
            else None
        )
        checks["formal_start_coverage"] = {
            "status": (
                "PASS"
                if start_lag_ns is not None and 0 <= start_lag_ns <= 50_000_000
                else "FAIL"
            ),
            "measurement_first_acquisition_ns": first_acquisition_ns,
            "formal_start_ns": formal_start_ns,
            "lag_ns": start_lag_ns,
            "threshold": "0 <= first acquisition - formal start <= 50,000,000 ns",
        }

    stream_pass = not missing and all(
        streams[name].get("status") == "PASS" for name in required
    )
    check_pass = all(check.get("status") == "PASS" for check in checks.values())
    return {
        "status": "PASS" if stream_pass and check_pass else "FAIL",
        "checks": checks,
    }


def _percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    index = int(round(fraction * (len(ordered) - 1)))
    return ordered[index]
