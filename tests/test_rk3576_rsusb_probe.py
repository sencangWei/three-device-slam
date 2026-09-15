from three_device_slam.edge_rk3576.rsusb_probe import (
    StreamContinuity,
    evaluate_streams,
)


def _complete_stream() -> dict:
    continuity = StreamContinuity(expected_payload_bytes=100)
    for sequence in range(3):
        continuity.observe(
            sequence=sequence,
            timestamp_ms=sequence * 33.333,
            arrival_ns=1_000_000_000 + sequence * 33_333_000,
            payload_bytes=100,
        )
    return continuity.report()


def test_exact_three_stream_probe_passes() -> None:
    streams = {
        name: _complete_stream()
        for name in ("color", "infrared_left", "infrared_right")
    }

    result = evaluate_streams(streams, expected_frames=3)

    assert result == {"status": "PASS", "reasons": []}


def test_sequence_gap_and_stale_interval_fail() -> None:
    streams = {
        name: _complete_stream()
        for name in ("color", "infrared_left", "infrared_right")
    }
    broken = StreamContinuity(expected_payload_bytes=100)
    broken.observe(
        sequence=0,
        timestamp_ms=0.0,
        arrival_ns=1_000_000_000,
        payload_bytes=100,
    )
    broken.observe(
        sequence=2,
        timestamp_ms=300.0,
        arrival_ns=1_300_000_000,
        payload_bytes=99,
    )
    streams["infrared_left"] = broken.report()

    result = evaluate_streams(streams, expected_frames=3)

    assert result["status"] == "FAIL"
    assert result["reasons"] == [
        "infrared_left:frames=2 expected=3",
        "infrared_left:sequence_gaps=1",
        "infrared_left:payload_size_errors=1",
        "infrared_left:max_arrival_interval_ms=300.000",
    ]
