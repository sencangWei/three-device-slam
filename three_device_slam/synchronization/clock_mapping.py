"""Per-device continuous clock mapping (design spec section 7).

RealSense ``global_time`` is wall-clock based, but individual devices drift
relative to the host wall clock at their own rate (observed ~170 ppm on the
D405 UMI vs ~40 ppm on the D435i Ego over 30 s). Sharing one startup
``wall_to_monotonic_offset_ns`` across devices therefore leaks milliseconds
into cross-device pairing span.

Each device owns a :class:`DeviceClockTracker`. It keeps the startup
wall-to-monotonic intercept (a host-clock quantity, safe to share across
devices) and continuously estimates the *rate error* of that device's
``global_time`` clock from the spacing of its own frame sequence numbers.
USB arrival latency never enters the estimate, because only device timestamps
and their sequence numbers are used.

Model::

    acquisition_ns(device_ms) =
        device_ns - intercept_ns
        - round(rate_ppm * (device_ns - anchor_ns) / 1e6)

where ``rate_ppm > 0`` means the device clock runs fast relative to the host
wall clock, so the correction is subtracted. The anchor is the first tracked
sample, so the correction is exactly zero at capture start and grows linearly
with device-clock drift. Every recorded frame keeps the raw
``device_timestamp_ms`` plus the mapping parameters and sample count as
audit evidence.
"""

from __future__ import annotations

from collections import deque
import math


CLOCK_MAPPING_MODEL = (
    "acquisition_monotonic_ns = device_global_time_ns - intercept_ns "
    "- rate_ppm * (device_global_time_ns - anchor_ns) / 1e6"
)


def device_ms_to_ns(device_timestamp_ms: float) -> int:
    """Convert RealSense wall milliseconds to integer nanoseconds."""
    if isinstance(device_timestamp_ms, bool) or not isinstance(
        device_timestamp_ms, (int, float)
    ):
        raise ValueError("device_timestamp_ms must be a number")
    if not math.isfinite(device_timestamp_ms) or device_timestamp_ms < 0:
        raise ValueError("device_timestamp_ms must be finite and non-negative")
    whole_ms = int(device_timestamp_ms)
    fractional_ns = int(round((device_timestamp_ms - whole_ms) * 1_000_000))
    return whole_ms * 1_000_000 + fractional_ns


class DeviceClockTracker:
    """Continuous per-device mapping from global_time to host monotonic ns.

    Parameters
    ----------
    intercept_ns:
        Startup ``wall_to_monotonic_offset_ns`` (host clock; shared across
        devices is intentional).
    expected_frame_delta_ns:
        Nominal device-clock interval of one frame (e.g. 33_333_333 for
        30 Hz). The device clock paces frames, so deviation of the fitted
        sequence slope from this nominal value is the device rate error.
    window:
        Number of samples kept for the slope fit.
    min_samples:
        Samples required before the first rate update.
    update_interval_ns:
        Minimum device-time between rate re-fits.
    max_rate_ppm:
        Absolute clamp for the estimated rate error.
    smoothing:
        EMA factor applied to successive rate estimates.
    """

    def __init__(
        self,
        *,
        intercept_ns: int,
        expected_frame_delta_ns: int,
        window: int = 300,
        min_samples: int = 60,
        update_interval_ns: int = 1_000_000_000,
        max_rate_ppm: float = 2_000.0,
        smoothing: float = 0.3,
    ) -> None:
        if intercept_ns < 0:
            raise ValueError("intercept_ns must be non-negative")
        if expected_frame_delta_ns <= 0:
            raise ValueError("expected_frame_delta_ns must be positive")
        if window < 4 or min_samples < 4 or min_samples > window:
            raise ValueError("require 4 <= min_samples <= window")
        if update_interval_ns < 0:
            raise ValueError("update_interval_ns must be non-negative")
        if max_rate_ppm <= 0:
            raise ValueError("max_rate_ppm must be positive")
        if not 0.0 < smoothing <= 1.0:
            raise ValueError("smoothing must be in (0, 1]")
        self._intercept_ns = int(intercept_ns)
        self._expected_delta_ns = int(expected_frame_delta_ns)
        self._min_samples = int(min_samples)
        self._update_interval_ns = int(update_interval_ns)
        self._max_rate_ppm = float(max_rate_ppm)
        self._smoothing = float(smoothing)
        self._samples: deque[tuple[int, int]] = deque(maxlen=int(window))
        self._anchor_ns: int | None = None
        self._rate_ppm = 0.0
        self._updates = 0
        self._last_update_device_ns: int | None = None

    @property
    def intercept_ns(self) -> int:
        return self._intercept_ns

    @property
    def rate_ppm(self) -> float:
        return self._rate_ppm

    @property
    def samples_used(self) -> int:
        return len(self._samples)

    def add_sample(self, sequence: int, device_timestamp_ms: float) -> None:
        """Feed one valid (global_time) frame for rate estimation."""
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            raise ValueError("sequence must be an integer")
        device_ns = device_ms_to_ns(device_timestamp_ms)
        if self._anchor_ns is None:
            self._anchor_ns = device_ns
        previous = self._samples[-1] if self._samples else None
        if previous is not None and sequence <= previous[0]:
            # Sequence regression (stream restart): start a fresh window.
            self._samples.clear()
        self._samples.append((sequence, device_ns))
        self._maybe_update_rate(device_ns)

    def _maybe_update_rate(self, device_ns: int) -> None:
        if len(self._samples) < self._min_samples:
            return
        if (
            self._last_update_device_ns is not None
            and device_ns - self._last_update_device_ns < self._update_interval_ns
        ):
            return
        raw = self._fit_rate_ppm()
        raw = max(-self._max_rate_ppm, min(self._max_rate_ppm, raw))
        if self._updates == 0:
            self._rate_ppm = raw
        else:
            self._rate_ppm = (
                self._smoothing * raw + (1.0 - self._smoothing) * self._rate_ppm
            )
        self._updates += 1
        self._last_update_device_ns = device_ns

    def _fit_rate_ppm(self) -> float:
        """Least-squares slope of device_ns over sequence, minus nominal.

        Sequence gaps (dropped frames) advance both axes together, so they do
        not bias the slope; per-frame timestamp jitter averages down over the
        window.
        """
        samples = list(self._samples)
        count = len(samples)
        mean_seq = sum(seq for seq, _ in samples) / count
        mean_ns = sum(ns for _, ns in samples) / count
        numerator = sum(
            (seq - mean_seq) * (ns - mean_ns) for seq, ns in samples
        )
        denominator = sum((seq - mean_seq) ** 2 for seq, _ in samples)
        if denominator <= 0:
            return 0.0
        fitted_delta_ns = numerator / denominator
        return (fitted_delta_ns / self._expected_delta_ns - 1.0) * 1.0e6

    def map_acquisition_ns(self, device_timestamp_ms: float) -> int:
        """Map a device global_time sample to host monotonic ns."""
        device_ns = device_ms_to_ns(device_timestamp_ms)
        anchor = self._anchor_ns if self._anchor_ns is not None else device_ns
        correction_ns = int(
            round(self._rate_ppm * (device_ns - anchor) / 1.0e6)
        )
        acquisition_ns = device_ns - self._intercept_ns - correction_ns
        if acquisition_ns < 0:
            raise ValueError("mapped acquisition timestamp is negative")
        return acquisition_ns

    def mapping_evidence(self) -> dict:
        """Audit evidence embedded into per-frame metadata."""
        return {
            "clock_mapping_model": CLOCK_MAPPING_MODEL,
            "clock_intercept_ns": self._intercept_ns,
            "clock_anchor_device_ns": self._anchor_ns,
            "clock_rate_ppm": self._rate_ppm,
            "clock_mapping_samples": len(self._samples),
            "clock_mapping_updates": self._updates,
            "clock_expected_frame_delta_ns": self._expected_delta_ns,
        }
