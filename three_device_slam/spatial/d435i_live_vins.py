"""Dependency-light primitives for live D435i stereo/internal-IMU VINS."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .device3_world_runtime import AnchorCandidate, RuntimeDecision
from .se3 import compose, invert, validate_transform


@dataclass(frozen=True)
class CombinedImu:
    timestamp_ns: int
    gyro_rad_s: np.ndarray
    accel_m_s2: np.ndarray

    def __post_init__(self) -> None:
        timestamp = int(self.timestamp_ns)
        gyro = np.asarray(self.gyro_rad_s, dtype=np.float64)
        accel = np.asarray(self.accel_m_s2, dtype=np.float64)
        if timestamp < 0 or gyro.shape != (3,) or accel.shape != (3,):
            raise ValueError("invalid combined IMU sample")
        if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel)):
            raise ValueError("combined IMU sample must be finite")
        gyro = np.array(gyro, copy=True)
        accel = np.array(accel, copy=True)
        gyro.setflags(write=False)
        accel.setflags(write=False)
        object.__setattr__(self, "timestamp_ns", timestamp)
        object.__setattr__(self, "gyro_rad_s", gyro)
        object.__setattr__(self, "accel_m_s2", accel)


class OnlineImuCombiner:
    """Interpolate accel into the gyro/body frame at original gyro stamps."""

    def __init__(self, accel_to_gyro: np.ndarray, *, max_gap_ns: int = 10_000_000):
        rotation = np.asarray(accel_to_gyro, dtype=np.float64)
        if (
            rotation.shape != (3, 3)
            or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)
        ):
            raise ValueError("accel_to_gyro must be a proper rotation")
        if max_gap_ns <= 0:
            raise ValueError("max_gap_ns must be positive")
        self.accel_to_gyro = np.array(rotation, copy=True)
        self.max_gap_ns = int(max_gap_ns)
        self.accel: deque[tuple[int, np.ndarray]] = deque(maxlen=512)
        self.gyro: deque[tuple[int, np.ndarray]] = deque(maxlen=512)
        self.last_accel_ns: int | None = None
        self.last_gyro_ns: int | None = None
        self.published = 0
        self.leading_gyro_dropped = 0
        self.interpolation_gap_dropped = 0
        self.failure: str | None = None

    @staticmethod
    def _vector(value) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError("motion vector must contain three finite values")
        return np.array(vector, copy=True)

    def push_accel(self, timestamp_ns: int, accel_m_s2) -> tuple[CombinedImu, ...]:
        timestamp = int(timestamp_ns)
        if self.last_accel_ns is not None and timestamp <= self.last_accel_ns:
            self.failure = "accel_timestamp_not_strictly_increasing"
            raise ValueError(self.failure)
        self.last_accel_ns = timestamp
        self.accel.append((timestamp, self._vector(accel_m_s2)))
        return self._drain()

    def push_gyro(self, timestamp_ns: int, gyro_rad_s) -> tuple[CombinedImu, ...]:
        timestamp = int(timestamp_ns)
        if self.last_gyro_ns is not None and timestamp <= self.last_gyro_ns:
            self.failure = "gyro_timestamp_not_strictly_increasing"
            raise ValueError(self.failure)
        self.last_gyro_ns = timestamp
        if len(self.gyro) == self.gyro.maxlen:
            self.failure = "gyro_interpolation_queue_overflow"
            raise RuntimeError(self.failure)
        self.gyro.append((timestamp, self._vector(gyro_rad_s)))
        return self._drain()

    def _drain(self) -> tuple[CombinedImu, ...]:
        output = []
        while self.gyro and self.accel:
            stamp, gyro = self.gyro[0]
            if stamp < self.accel[0][0]:
                self.gyro.popleft()
                self.leading_gyro_dropped += 1
                continue
            if stamp > self.accel[-1][0]:
                break
            hi = next(
                index
                for index, (accel_stamp, _value) in enumerate(self.accel)
                if accel_stamp >= stamp
            )
            if self.accel[hi][0] == stamp:
                accel = self.accel[hi][1]
            else:
                if hi == 0:
                    self.gyro.popleft()
                    self.leading_gyro_dropped += 1
                    continue
                lo_stamp, lo_value = self.accel[hi - 1]
                hi_stamp, hi_value = self.accel[hi]
                gap = hi_stamp - lo_stamp
                if gap > self.max_gap_ns:
                    self.gyro.popleft()
                    self.interpolation_gap_dropped += 1
                    continue
                fraction = (stamp - lo_stamp) / gap
                accel = (1.0 - fraction) * lo_value + fraction * hi_value
            self.gyro.popleft()
            output.append(
                CombinedImu(
                    stamp,
                    gyro,
                    self.accel_to_gyro @ accel,
                )
            )
            self.published += 1
        return tuple(output)

    def stats(self) -> dict:
        return {
            "combined_imu_published": self.published,
            "leading_gyro_dropped": self.leading_gyro_dropped,
            "interpolation_gap_dropped": self.interpolation_gap_dropped,
            "pending_gyro": len(self.gyro),
            "accel_buffer": len(self.accel),
            "failure": self.failure,
        }


def ego_world_anchor_candidate(
    *,
    timestamp_ns: int,
    gyro_rad_s: np.ndarray,
    world_from_camera: np.ndarray,
    body_from_camera: np.ndarray,
    odom_from_body: np.ndarray,
) -> AnchorCandidate:
    """Build ``T_world_ego_odom`` from a synchronized grid and local VINS pose."""
    gyro = np.asarray(gyro_rad_s, dtype=np.float64)
    if gyro.shape != (3,) or not np.all(np.isfinite(gyro)):
        raise ValueError("gyro_rad_s must contain three finite values")
    world_from_body = compose(
        validate_transform(world_from_camera, name="world_from_camera"),
        invert(validate_transform(body_from_camera, name="body_from_camera")),
    )
    world_from_odom = compose(
        world_from_body,
        invert(validate_transform(odom_from_body, name="odom_from_body")),
    )
    return AnchorCandidate(
        timestamp_ns=int(timestamp_ns),
        gyro_norm_deg_s=math.degrees(float(np.linalg.norm(gyro))),
        world_from_odom=world_from_odom,
    )


class FrozenSessionAnchorController:
    """Initialize an existing quality-gated controller once, then hold its frame."""

    def __init__(self, controller):
        self.controller = controller

    @property
    def current(self):
        return self.controller.current

    def process_window(self, candidates) -> RuntimeDecision:
        values = tuple(candidates)
        if self.current is None:
            return self.controller.process_window(values)
        return RuntimeDecision(
            status="HELD",
            reason="session_anchor_frozen",
            world_from_odom=self.current.world_from_odom,
            candidate_count=len(values),
            eligible_count=0,
            inlier_count=0,
        )

    def world_from_body(self, odom_from_body: np.ndarray) -> np.ndarray:
        return self.controller.world_from_body(odom_from_body)
