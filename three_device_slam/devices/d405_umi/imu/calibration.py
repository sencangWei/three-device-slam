"""IMU 内参标定加载与运行时应用。"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml

from .imu_reader import ImuSample


@dataclass(frozen=True)
class IMUCalibration:
    calibration_id: str
    accel_matrix: np.ndarray
    accel_offset_g: np.ndarray
    gyro_matrix: np.ndarray
    gyro_bias_deg_s: np.ndarray

    @classmethod
    def load(cls, path: str | Path) -> "IMUCalibration":
        path = Path(path)
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        acceptance = raw.get("acceptance", {})
        if acceptance.get("status") == "FAIL" or acceptance.get("runtime_applied") is False:
            raise ValueError(f"IMU标定未通过运行时门禁: {path}")
        gyro_raw = raw["gyroscope"]
        gyro_matrix = gyro_raw.get("matrix", gyro_raw.get("matrix_candidate"))
        if gyro_matrix is None:
            raise ValueError("gyroscope.matrix 或 matrix_candidate 缺失")
        calibration = cls(
            calibration_id=str(raw.get("calibration_id", path.stem)),
            accel_matrix=np.asarray(raw["accelerometer"]["matrix"], dtype=float),
            accel_offset_g=np.asarray(raw["accelerometer"]["offset_g"], dtype=float),
            gyro_matrix=np.asarray(gyro_matrix, dtype=float),
            gyro_bias_deg_s=np.asarray(gyro_raw["bias_deg_s"], dtype=float),
        )
        if calibration.accel_matrix.shape != (3, 3):
            raise ValueError("accelerometer.matrix 必须是 3x3")
        if calibration.gyro_matrix.shape != (3, 3):
            raise ValueError("gyroscope.matrix 必须是 3x3")
        if calibration.accel_offset_g.shape != (3,):
            raise ValueError("accelerometer.offset_g 必须有 3 项")
        if calibration.gyro_bias_deg_s.shape != (3,):
            raise ValueError("gyroscope.bias_deg_s 必须有 3 项")
        return calibration

    def apply(self, sample: ImuSample) -> ImuSample:
        accel = self.accel_matrix @ np.asarray(
            [sample.ax, sample.ay, sample.az], dtype=float
        ) + self.accel_offset_g
        gyro = self.gyro_matrix @ (
            np.asarray([sample.gx, sample.gy, sample.gz], dtype=float)
            - self.gyro_bias_deg_s
        )
        return replace(
            sample,
            ax=float(accel[0]),
            ay=float(accel[1]),
            az=float(accel[2]),
            gx=float(gyro[0]),
            gy=float(gyro[1]),
            gz=float(gyro[2]),
        )
