
import numpy as np


from three_device_slam.devices.d405_umi.imu.calibration import IMUCalibration
from three_device_slam.devices.d405_umi.imu.imu_reader import ImuSample


def test_calibration_changes_values_but_preserves_sample_identity(tmp_path):
    calibration_path = tmp_path / "imu.yaml"
    calibration_path.write_text(
        """
calibration_id: test-calibration
accelerometer:
  matrix: [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
  offset_g: [0.1, 0.2, 0.3]
gyroscope:
  matrix: [[5, 0, 0], [0, 6, 0], [0, 0, 7]]
  bias_deg_s: [1, 2, 3]
""",
        encoding="utf-8",
    )
    calibration = IMUCalibration.load(calibration_path)
    raw = ImuSample(
        ts=12.5,
        rx_time=12.6,
        counter=42,
        gx=2.0,
        gy=4.0,
        gz=6.0,
        ax=1.0,
        ay=2.0,
        az=3.0,
        temp=24.0,
    )

    corrected = calibration.apply(raw)

    assert np.allclose([corrected.ax, corrected.ay, corrected.az], [2.1, 6.2, 12.3])
    assert np.allclose([corrected.gx, corrected.gy, corrected.gz], [5.0, 12.0, 21.0])
    assert (corrected.ts, corrected.rx_time, corrected.counter, corrected.temp) == (
        raw.ts,
        raw.rx_time,
        raw.counter,
        raw.temp,
    )
    assert corrected is not raw


def test_calibration_accepts_manual_gyro_matrix_candidate(tmp_path):
    calibration_path = tmp_path / "manual_candidate.yaml"
    calibration_path.write_text(
        """
accelerometer:
  matrix: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
  offset_g: [0, 0, 0]
gyroscope:
  matrix_candidate: [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
  bias_deg_s: [0, 0, 0]
""",
        encoding="utf-8",
    )

    calibration = IMUCalibration.load(calibration_path)

    assert np.allclose(calibration.gyro_matrix, np.diag([2.0, 3.0, 4.0]))


def test_calibration_rejects_explicitly_failed_candidate(tmp_path):
    calibration_path = tmp_path / "failed_candidate.yaml"
    calibration_path.write_text(
        """
accelerometer:
  matrix: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
  offset_g: [0, 0, 0]
gyroscope:
  matrix_candidate: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
  bias_deg_s: [0, 0, 0]
acceptance:
  status: FAIL
  runtime_applied: false
""",
        encoding="utf-8",
    )

    with np.testing.assert_raises_regex(ValueError, "未通过运行时门禁"):
        IMUCalibration.load(calibration_path)
