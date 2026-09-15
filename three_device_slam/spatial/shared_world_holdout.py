"""Deterministic, phase-stratified pose-pair selection for shared-world holdout."""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class HoldoutPair:
    phase: str
    ego_index: int
    right_index: int
    ego_stamp_ns: int
    right_stamp_ns: int

    def as_dict(self) -> dict:
        result = asdict(self)
        result["time_delta_ms"] = (
            self.right_stamp_ns - self.ego_stamp_ns
        ) / 1e6
        return result


def motion_phases(formal_start_ns: int, formal_end_ns: int) -> tuple:
    """Return the four phases shown by the 60 s shared-world capture UI."""
    duration_ns = formal_end_ns - formal_start_ns
    if duration_ns < 30_000_000_000:
        raise ValueError("shared-world holdout requires at least 30 seconds")
    ten_seconds = 10_000_000_000
    translation_end = min(formal_start_ns + 30_000_000_000,
                          formal_end_ns - ten_seconds)
    return (
        ("initial_static", formal_start_ns, formal_start_ns + ten_seconds),
        ("translation", formal_start_ns + ten_seconds, translation_end),
        ("rotation_return", translation_end, formal_end_ns - ten_seconds),
        ("final_static", formal_end_ns - ten_seconds, formal_end_ns),
    )


def _nearest_index(stamps: list[int], value: int) -> int:
    index = bisect_left(stamps, value)
    choices = []
    if index:
        choices.append(index - 1)
    if index < len(stamps):
        choices.append(index)
    if not choices:
        raise ValueError("right pose stream is empty")
    return min(choices, key=lambda candidate: abs(stamps[candidate] - value))


def _away_from_keyframes(stamp: int, keyframes: np.ndarray,
                         separation_ns: int) -> bool:
    return not len(keyframes) or int(np.min(np.abs(keyframes - stamp))) >= separation_ns


def select_phase_stratified_pairs(
    ego_stamps_ns: list[int],
    right_stamps_ns: list[int],
    *,
    formal_start_ns: int,
    formal_end_ns: int,
    max_pair_delta_ns: int,
    ego_keyframes_ns: list[int] = (),
    right_keyframes_ns: list[int] = (),
    keyframe_separation_ns: int = 20_000_000,
    pairs_per_phase: int = 20,
) -> tuple[list[HoldoutPair], dict[str, int]]:
    """Select uniformly spread non-keyframe pairs in every capture phase.

    ``max_pair_delta_ns`` must come from the accepted capture contract.  Callers
    must not silently substitute a tighter value because that can remove later
    free-running camera phases and bias the holdout toward startup.
    """
    ego = [int(value) for value in ego_stamps_ns]
    right = [int(value) for value in right_stamps_ns]
    if (not ego or not right or any(b <= a for a, b in zip(ego, ego[1:]))
            or any(b <= a for a, b in zip(right, right[1:]))):
        raise ValueError("pose timestamps must be nonempty and strictly increasing")
    if max_pair_delta_ns <= 0 or pairs_per_phase <= 0:
        raise ValueError("pair tolerance and per-phase count must be positive")
    ego_kf = np.asarray(ego_keyframes_ns, dtype=np.int64)
    right_kf = np.asarray(right_keyframes_ns, dtype=np.int64)
    selected: list[HoldoutPair] = []
    counts: dict[str, int] = {}
    for phase, begin, end in motion_phases(formal_start_ns, formal_end_ns):
        candidates = []
        for ego_index, ego_stamp in enumerate(ego):
            if not begin <= ego_stamp < end:
                continue
            right_index = _nearest_index(right, ego_stamp)
            right_stamp = right[right_index]
            if abs(right_stamp - ego_stamp) > max_pair_delta_ns:
                continue
            if not _away_from_keyframes(
                    ego_stamp, ego_kf, keyframe_separation_ns):
                continue
            if not _away_from_keyframes(
                    right_stamp, right_kf, keyframe_separation_ns):
                continue
            candidates.append(HoldoutPair(
                phase, ego_index, right_index, ego_stamp, right_stamp))
        counts[phase] = len(candidates)
        if candidates:
            indices = np.linspace(
                0, len(candidates) - 1,
                min(pairs_per_phase, len(candidates)), dtype=int)
            selected.extend(candidates[int(index)] for index in indices)
    return selected, counts
