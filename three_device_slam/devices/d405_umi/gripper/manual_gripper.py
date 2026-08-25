"""Convert the UMI magnetic encoder angle into training-friendly gripper state.

The millimetre output is a no-load soft-pad gap estimate.  Raw angle and
closure ratio remain the authoritative model-training signals when gripping an
object because pad compression was not force-calibrated.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


DEFAULT_PROFILE = Path(__file__).with_name("umi_manual_gripper_20260824.yaml")
_DEFAULT_PROFILE_TEXT = """\
format_version: 1
profile_id: UMI_MANUAL_GRIPPER_20260824_V1
product_mode: manual_hand_operated
fully_open_gap_mm: 66.900
nominal_radius_mm: 25.150
direction_deadband_deg: 0.200
no_load_uncertainty_mm: 1.500
loaded_object_size_valid: false
distance_semantics:
  estimated_no_load_gap_mm: Estimated soft-pad face gap without an object load.
  dual_closing_distance_mm: fully_open_gap_mm minus estimated_no_load_gap_mm.
  single_jaw_travel_mm: One half of dual_closing_distance_mm for the symmetric jaws.
  closure_ratio: dual_closing_distance_mm divided by fully_open_gap_mm.
  warning: Raw angle and closure ratio are valid manual-state signals; gap is not loaded object size.
source:
  date: 2026-08-24
  device_protocol: stm32_combined_v1
  firmware_sha256: 1030e78c9fe4f7b949350b100d6389f219a8b650c649c834693619006953cd8a
  evidence: product_calibration/evidence/umi_manual_gripper_20260824.yaml
  method: Hand-operated caliper points; direction-specific piecewise-linear interpolation.
curves:
  closing:
    - {angle_deg: 12.969031, gap_mm: 0.000}
    - {angle_deg: 13.857045, gap_mm: 4.000}
    - {angle_deg: 21.869696, gap_mm: 8.000}
    - {angle_deg: 29.259646, gap_mm: 16.720}
    - {angle_deg: 48.320859, gap_mm: 33.450}
    - {angle_deg: 68.408441, gap_mm: 50.174}
    - {angle_deg: 91.171230, gap_mm: 66.900}
  opening:
    - {angle_deg: 12.969031, gap_mm: 0.000}
    - {angle_deg: 15.144568, gap_mm: 4.000}
    - {angle_deg: 19.111518, gap_mm: 8.000}
    - {angle_deg: 30.477789, gap_mm: 16.720}
    - {angle_deg: 48.857102, gap_mm: 33.450}
    - {angle_deg: 69.158545, gap_mm: 50.174}
    - {angle_deg: 91.135558, gap_mm: 66.900}
quality:
  transport: PASS
  manual_state_calibration: PASS_FOR_MANUAL_STATE_AND_NO_LOAD_GAP_ESTIMATE
  loaded_object_size: NOT_SUPPORTED_WITHOUT_FORCE_COMPRESSION_MODEL
  blind_holdout:
    result: PASS
    count: 3
    maximum_absolute_error_mm: 1.195
    mean_absolute_error_mm: 0.829
    frozen_acceptance:
      maximum_absolute_error_mm: 1.500
      mean_absolute_error_mm: 1.000
    evidence: product_calibration/evidence/umi_manual_gripper_20260824.yaml
  notes:
    - Fixed-radius conversion was rejected by intermediate caliper points.
    - Opening 8 mm used the midpoint of two held observations affected by hand-force drift.
    - Values below contact clamp to zero gap while raw encoder angle remains available.
"""


class CalibrationError(ValueError):
    pass


def shortest_angle_delta_deg(current: float, previous: float) -> float:
    return (float(current) - float(previous) + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class ManualGripperState:
    angle_deg: float
    direction: str
    estimated_no_load_gap_mm: float | None
    dual_closing_distance_mm: float | None
    single_jaw_travel_mm: float | None
    closure_ratio: float | None
    no_load_uncertainty_mm: float | None
    loaded_object_size_valid: bool
    status: str


class ManualGripperCalibration:
    def __init__(self, document: dict[str, Any]) -> None:
        self.document = document
        self.profile_id = str(document["profile_id"])
        self.fully_open_gap_mm = float(document["fully_open_gap_mm"])
        self.direction_deadband_deg = float(document["direction_deadband_deg"])
        self.no_load_uncertainty_mm = float(document["no_load_uncertainty_mm"])
        self.curves = {
            name: self._validate_curve(name, document["curves"][name])
            for name in ("closing", "opening")
        }

    @classmethod
    def load(cls, path: Path = DEFAULT_PROFILE) -> "ManualGripperCalibration":
        profile_path = Path(path)
        try:
            profile_text = profile_path.read_text(encoding="utf-8")
        except FileNotFoundError:
            if profile_path != DEFAULT_PROFILE:
                raise
            profile_text = _DEFAULT_PROFILE_TEXT
        document = yaml.safe_load(profile_text) or {}
        if document.get("format_version") != 1:
            raise CalibrationError("unsupported manual gripper profile format")
        return cls(document)

    def _validate_curve(self, name: str, raw_points: list[dict[str, Any]]):
        points = tuple(
            (float(item["angle_deg"]), float(item["gap_mm"]))
            for item in raw_points
        )
        if len(points) < 2:
            raise CalibrationError(f"{name} curve needs at least two points")
        if any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(points, points[1:])):
            raise CalibrationError(f"{name} curve must be strictly increasing")
        return points

    def estimate_gap_mm(self, angle_deg: float, direction: str) -> float:
        points = self.curves[direction]
        reference = (points[0][0] + points[-1][0]) / 2.0
        angle = float(angle_deg) + 360.0 * round((reference - float(angle_deg)) / 360.0)
        angles = [point[0] for point in points]
        if angle <= angles[0]:
            return points[0][1]
        if angle >= angles[-1]:
            return points[-1][1]
        upper = bisect.bisect_right(angles, angle)
        angle0, gap0 = points[upper - 1]
        angle1, gap1 = points[upper]
        return gap0 + (angle - angle0) / (angle1 - angle0) * (gap1 - gap0)

    def state(self, angle_deg: float, direction: str) -> ManualGripperState:
        gap = min(self.fully_open_gap_mm, max(0.0, self.estimate_gap_mm(angle_deg, direction)))
        dual = self.fully_open_gap_mm - gap
        return ManualGripperState(
            angle_deg=float(angle_deg) % 360.0,
            direction=direction,
            estimated_no_load_gap_mm=gap,
            dual_closing_distance_mm=dual,
            single_jaw_travel_mm=dual / 2.0,
            closure_ratio=dual / self.fully_open_gap_mm,
            no_load_uncertainty_mm=self.no_load_uncertainty_mm,
            loaded_object_size_valid=False,
            status="MANUAL_NO_LOAD_ESTIMATE",
        )

    def unknown_direction_state(self, angle_deg: float) -> ManualGripperState:
        gap = (
            self.estimate_gap_mm(angle_deg, "closing")
            + self.estimate_gap_mm(angle_deg, "opening")
        ) / 2.0
        gap = min(self.fully_open_gap_mm, max(0.0, gap))
        dual = self.fully_open_gap_mm - gap
        return ManualGripperState(
            angle_deg=float(angle_deg) % 360.0,
            direction="unknown",
            estimated_no_load_gap_mm=gap,
            dual_closing_distance_mm=dual,
            single_jaw_travel_mm=dual / 2.0,
            closure_ratio=dual / self.fully_open_gap_mm,
            no_load_uncertainty_mm=self.no_load_uncertainty_mm,
            loaded_object_size_valid=False,
            status="MANUAL_NO_LOAD_ESTIMATE_DIRECTION_UNKNOWN",
        )


class ManualGripperTracker:
    def __init__(self, calibration: ManualGripperCalibration) -> None:
        self.calibration = calibration
        self._anchor: float | None = None
        self._direction: str | None = None

    def update(self, angle_deg: float, *, encoder_valid: bool = True) -> ManualGripperState:
        angle = float(angle_deg) % 360.0
        if not encoder_valid:
            return ManualGripperState(
                angle, self._direction or "unknown", None, None, None, None,
                None, False, "ENCODER_INVALID",
            )
        if self._anchor is None:
            self._anchor = angle
            return self.calibration.unknown_direction_state(angle)
        delta = shortest_angle_delta_deg(angle, self._anchor)
        if delta <= -self.calibration.direction_deadband_deg:
            self._direction, self._anchor = "closing", angle
        elif delta >= self.calibration.direction_deadband_deg:
            self._direction, self._anchor = "opening", angle
        if self._direction is None:
            return self.calibration.unknown_direction_state(angle)
        return self.calibration.state(angle, self._direction)
