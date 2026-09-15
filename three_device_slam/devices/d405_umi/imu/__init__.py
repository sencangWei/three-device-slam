"""D405 UMI inertial helpers with dependency-light lazy imports."""

from __future__ import annotations

__all__ = [
    "IMUCalibration",
    "ImuReader",
    "ImuSample",
    "find_frames",
    "parse_frame",
    "verify_checksum",
]


def __getattr__(name: str):
    if name == "IMUCalibration":
        from .calibration import IMUCalibration

        return IMUCalibration
    if name in {"ImuReader", "ImuSample", "find_frames", "parse_frame", "verify_checksum"}:
        from .imu_reader import ImuReader, ImuSample, find_frames, parse_frame, verify_checksum

        return {
            "ImuReader": ImuReader,
            "ImuSample": ImuSample,
            "find_frames": find_frames,
            "parse_frame": parse_frame,
            "verify_checksum": verify_checksum,
        }[name]
    raise AttributeError(name)
