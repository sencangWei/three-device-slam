"""Unit tests for per-device continuous clock mapping (spec section 7)."""

import pytest

from three_device_slam.synchronization.clock_mapping import (
    CLOCK_MAPPING_MODEL,
    DeviceClockTracker,
    device_ms_to_ns,
)


INTERCEPT_NS = 1_000_000_000_000
EXPECTED_DELTA_NS = 33_333_333
# 30 Hz device timeline with +170 ppm rate error (the observed D405 drift).
DRIFT_PPM = 170.0


def _device_series(count, *, rate_ppm=0.0, start_ns=2_000_000_000_000):
    """(sequence, device_ms) pairs on a drifted 30 Hz grid."""
    factor = 1.0 + rate_ppm / 1.0e6
    series = []
    for index in range(count):
        device_ns = start_ns + int(round(index * EXPECTED_DELTA_NS * factor))
        series.append((index, device_ns / 1_000_000.0))
    return series


def _make_tracker(**overrides):
    options = {
        "intercept_ns": INTERCEPT_NS,
        "expected_frame_delta_ns": EXPECTED_DELTA_NS,
        "min_samples": 10,
        "update_interval_ns": 0,
    }
    options.update(overrides)
    return DeviceClockTracker(**options)


def test_ms_to_ns_rounding_matches_legacy_contract():
    assert device_ms_to_ns(1234.5) == 1_234_500_000
    assert device_ms_to_ns(0.25) == 250_000
    with pytest.raises(ValueError):
        device_ms_to_ns(-1.0)
    with pytest.raises(ValueError):
        device_ms_to_ns(float("nan"))


def test_zero_drift_matches_fixed_mapping():
    tracker = _make_tracker()
    series = _device_series(40)
    for sequence, device_ms in series:
        tracker.add_sample(sequence, device_ms)
    assert abs(tracker.rate_ppm) < 5.0
    for sequence, device_ms in series:
        expected = device_ms_to_ns(device_ms) - INTERCEPT_NS
        assert tracker.map_acquisition_ns(device_ms) == expected


def test_rate_estimate_recovers_linear_drift():
    tracker = _make_tracker()
    for sequence, device_ms in _device_series(200, rate_ppm=DRIFT_PPM):
        tracker.add_sample(sequence, device_ms)
    assert abs(tracker.rate_ppm - DRIFT_PPM) < 5.0


def test_mapping_corrects_drift_end_to_end():
    tracker = _make_tracker()
    series = _device_series(600, rate_ppm=DRIFT_PPM)
    for sequence, device_ms in series:
        tracker.add_sample(sequence, device_ms)
    # With the full series fed, late samples map back onto the nominal grid:
    # residual after correction stays well under 1 ms.
    for sequence, device_ms in series[-30:]:
        nominal_ns = (
            device_ms_to_ns(series[0][1])
            + sequence * EXPECTED_DELTA_NS
            - INTERCEPT_NS
        )
        residual = abs(tracker.map_acquisition_ns(device_ms) - nominal_ns)
        assert residual < 1_000_000


def test_anchor_locks_correction_to_zero_at_start():
    tracker = _make_tracker()
    series = _device_series(200, rate_ppm=DRIFT_PPM)
    for sequence, device_ms in series:
        tracker.add_sample(sequence, device_ms)
    first_ms = series[0][1]
    first_ns = device_ms_to_ns(first_ms)
    assert tracker.map_acquisition_ns(first_ms) == first_ns - INTERCEPT_NS


def test_sequence_gap_does_not_bias_slope():
    tracker = _make_tracker()
    start_ns = 2_000_000_000_000
    factor = 1.0 + DRIFT_PPM / 1.0e6
    index = 0
    device_index = 0
    while index < 200:
        device_ns = start_ns + int(round(device_index * EXPECTED_DELTA_NS * factor))
        tracker.add_sample(device_index, device_ns / 1_000_000.0)
        device_index += 2 if index == 100 else 1  # one dropped frame
        index += 1
    assert abs(tracker.rate_ppm - DRIFT_PPM) < 5.0


def test_sequence_regression_resets_window():
    tracker = _make_tracker()
    for sequence, device_ms in _device_series(20):
        tracker.add_sample(sequence, device_ms)
    tracker.add_sample(2, 3_000_000.0)  # stale/late frame: regression
    assert tracker.samples_used == 1


def test_rate_is_clamped():
    tracker = _make_tracker(max_rate_ppm=500.0)
    for sequence, device_ms in _device_series(200, rate_ppm=50_000.0):
        tracker.add_sample(sequence, device_ms)
    assert abs(tracker.rate_ppm) <= 500.0


def test_evidence_carries_audit_fields():
    tracker = _make_tracker()
    for sequence, device_ms in _device_series(30):
        tracker.add_sample(sequence, device_ms)
    evidence = tracker.mapping_evidence()
    assert evidence["clock_mapping_model"] == CLOCK_MAPPING_MODEL
    assert evidence["clock_intercept_ns"] == INTERCEPT_NS
    assert evidence["clock_mapping_samples"] == 30
    assert evidence["clock_mapping_updates"] >= 1
    assert evidence["clock_anchor_device_ns"] == device_ms_to_ns(
        _device_series(1)[0][1]
    )


def test_validation():
    with pytest.raises(ValueError):
        _make_tracker(intercept_ns=-1)
    with pytest.raises(ValueError):
        _make_tracker(expected_frame_delta_ns=0)
    with pytest.raises(ValueError):
        _make_tracker(min_samples=3)
    with pytest.raises(ValueError):
        DeviceClockTracker(
            intercept_ns=0,
            expected_frame_delta_ns=EXPECTED_DELTA_NS,
            smoothing=0.0,
        )
    with pytest.raises(ValueError):
        _make_tracker().add_sample(1.5, 1000.0)
