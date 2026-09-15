"""Deterministic common-clock matching for Ego and two D405 frame streams."""

from bisect import bisect_left
from collections.abc import Sequence
from dataclasses import dataclass

from three_device_slam.core.model import FrameStamp, Triplet


GRID_NS = 33_333_333
TARGET_SPAN_NS = 10_000_000
HARD_SPAN_NS = 16_700_000


@dataclass(frozen=True)
class PairRow:
    sample_ns: int
    ego: FrameStamp
    left: FrameStamp
    span_ns: int
    trainable: bool
    reason: str


def build_pairs(
    ego: Sequence[FrameStamp],
    left: Sequence[FrameStamp],
    umi_device_id: str = "left",
) -> list[PairRow]:
    """Match Ego and one UMI frame stream on the frozen 30 Hz common grid."""
    if umi_device_id not in {"left", "right"}:
        raise ValueError("umi_device_id must be left or right")
    streams = (
        _validate_stream(ego, "ego"),
        _validate_stream(left, umi_device_id),
    )
    if any(not stream for stream in streams):
        return []

    overlap_start = max(stream[0].acquisition_ns for stream in streams)
    overlap_end = min(stream[-1].acquisition_ns for stream in streams)
    if overlap_start > overlap_end:
        return []

    timestamp_streams = [
        [frame.acquisition_ns for frame in stream] for stream in streams
    ]
    pairs = []
    grid_ns = overlap_start
    while grid_ns <= overlap_end:
        selected = tuple(
            _nearest_frame(stream, timestamps, grid_ns)
            for stream, timestamps in zip(streams, timestamp_streams)
        )
        timestamps = sorted(frame.acquisition_ns for frame in selected)
        sample_ns = (timestamps[0] + timestamps[-1]) // 2
        span_ns = timestamps[-1] - timestamps[0]
        trainable, reason = _classify(span_ns)
        pairs.append(
            PairRow(
                sample_ns=sample_ns,
                ego=selected[0],
                left=selected[1],
                span_ns=span_ns,
                trainable=trainable,
                reason=reason,
            )
        )
        grid_ns += GRID_NS
    return pairs


def build_triplets(
    ego: Sequence[FrameStamp],
    left: Sequence[FrameStamp],
    right: Sequence[FrameStamp],
) -> list[Triplet]:
    """Match supplied frame streams on the frozen 30 Hz common-clock grid."""
    streams = (
        _validate_stream(ego, "ego"),
        _validate_stream(left, "left"),
        _validate_stream(right, "right"),
    )
    if any(not stream for stream in streams):
        return []

    overlap_start = max(stream[0].acquisition_ns for stream in streams)
    overlap_end = min(stream[-1].acquisition_ns for stream in streams)
    if overlap_start > overlap_end:
        return []

    timestamp_streams = [
        [frame.acquisition_ns for frame in stream] for stream in streams
    ]
    triplets = []
    grid_ns = overlap_start
    while grid_ns <= overlap_end:
        selected = tuple(
            _nearest_frame(stream, timestamps, grid_ns)
            for stream, timestamps in zip(streams, timestamp_streams)
        )
        selected_timestamps = sorted(frame.acquisition_ns for frame in selected)
        sample_ns = selected_timestamps[1]
        span_ns = selected_timestamps[-1] - selected_timestamps[0]
        trainable, reason = _classify(span_ns)
        triplets.append(
            Triplet(
                sample_ns=sample_ns,
                ego=selected[0],
                left=selected[1],
                right=selected[2],
                span_ns=span_ns,
                trainable=trainable,
                reason=reason,
            )
        )
        grid_ns += GRID_NS
    return triplets


def _validate_stream(
    supplied: Sequence[FrameStamp], expected_device_id: str
) -> list[FrameStamp]:
    stream = list(supplied)
    seen_sequences = set()
    previous_timestamp = None
    for index, frame in enumerate(stream):
        if not isinstance(frame, FrameStamp):
            raise ValueError(f"{expected_device_id}[{index}] must be a FrameStamp")
        if frame.device_id != expected_device_id:
            raise ValueError(
                f"{expected_device_id}[{index}] device_id must be {expected_device_id}"
            )
        _nonnegative_integer(frame.sequence, f"{expected_device_id}[{index}] sequence")
        _nonnegative_integer(
            frame.acquisition_ns,
            f"{expected_device_id}[{index}] acquisition_ns",
        )
        if not isinstance(frame.payload_ref, str) or not frame.payload_ref:
            raise ValueError(
                f"{expected_device_id}[{index}] payload_ref must be non-empty"
            )
        if frame.sequence in seen_sequences:
            raise ValueError(
                f"duplicate ({expected_device_id}, {frame.sequence}) sequence"
            )
        if (
            previous_timestamp is not None
            and frame.acquisition_ns < previous_timestamp
        ):
            raise ValueError(f"{expected_device_id} timestamp regression")
        seen_sequences.add(frame.sequence)
        previous_timestamp = frame.acquisition_ns
    return stream


def _nonnegative_integer(value, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")
    return value


def _nearest_frame(
    stream: Sequence[FrameStamp], timestamps: Sequence[int], target_ns: int
) -> FrameStamp:
    position = bisect_left(timestamps, target_ns)
    if position < len(timestamps) and timestamps[position] == target_ns:
        return stream[position]
    if position == 0:
        selected_timestamp = timestamps[0]
    elif position == len(timestamps):
        selected_timestamp = timestamps[-1]
    else:
        before = timestamps[position - 1]
        after = timestamps[position]
        selected_timestamp = min(
            (before, after), key=lambda timestamp: (abs(timestamp - target_ns), timestamp)
        )
    return stream[bisect_left(timestamps, selected_timestamp)]


def _classify(span_ns: int) -> tuple[bool, str]:
    if span_ns <= TARGET_SPAN_NS:
        return True, "within_target"
    if span_ns <= HARD_SPAN_NS:
        return True, "within_hard_limit"
    return False, "span_over_hard_limit"
