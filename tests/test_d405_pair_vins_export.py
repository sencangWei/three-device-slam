from pathlib import Path
import struct

import pytest

from three_device_slam.devices.d405_umi.pair_vins_export import (
    IMU_RECORD,
    VINS_IMU_ROTATION,
    imu_to_vins_axes,
    load_formal_imu,
    render_runtime_config,
    validate_imu_index,
)
import numpy as np


def test_formal_imu_selection_preserves_shared_clock_and_boundaries(tmp_path):
    path = tmp_path / "imu.bin"
    path.write_bytes(b"".join(IMU_RECORD.pack(t, i, *([float(i)] * 7))
                              for i, t in enumerate((1.0, 1.0025, 1.005))))
    rows = load_formal_imu(path, 1_002_500_000, 1_005_000_000)
    assert [row[0] for row in rows] == [1.0025, 1.005]
    assert rows[0][2:5] == pytest.approx((1., 1., 1.))


def test_imu_conversion_matches_frozen_product_replay_axes():
    row = (1.0, 7, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 25.0)
    gyro, accel = imu_to_vins_axes(row)

    assert np.linalg.det(VINS_IMU_ROTATION) == pytest.approx(1.0, abs=1e-6)
    assert gyro == pytest.approx(
        VINS_IMU_ROTATION @ (np.array([1.0, 2.0, 3.0]) * np.pi / 180.0)
    )
    assert accel == pytest.approx(
        VINS_IMU_ROTATION @ (np.array([0.1, 0.2, 0.3]) * 9.80665)
    )


def test_formal_imu_rejects_partial_or_nonmonotonic_payload(tmp_path):
    path = tmp_path / "imu.bin"
    path.write_bytes(b"partial")
    with pytest.raises(ValueError, match="partial"):
        load_formal_imu(path, 0, 2_000_000_000)
    path.write_bytes(b"".join(IMU_RECORD.pack(t, i, *([0.] * 7))
                              for i, t in enumerate((1.0, 1.0))))
    with pytest.raises(ValueError, match="strictly increasing"):
        load_formal_imu(path, 0, 2_000_000_000)


def test_imu_index_must_match_binary_counter_and_timestamp(tmp_path):
    rows = [(1.0, 7, *([0.] * 7)), (1.0025, 8, *([0.] * 7))]
    index = tmp_path / "imu_ts.csv"
    index.write_text("counter,ts_mono,rx_mono,ts_wall\n7,1.000000000,1,2\n8,1.002500000,1,2\n")
    validate_imu_index(index, rows)
    index.write_text("counter,ts_mono,rx_mono,ts_wall\n7,1.000000000,1,2\n9,1.002500000,1,2\n")
    with pytest.raises(ValueError, match="mismatch at row 1"):
        validate_imu_index(index, rows)


def test_runtime_config_binds_right_topics_and_derived_output(tmp_path):
    source = ('%YAML:1.0\nimu_topic: "/imu0"\nimage0_topic: "/cam0/image_raw"\n'
              'image1_topic: "/cam1/image_raw"\noutput_path: "/old/"\n')
    rendered = render_runtime_config(source, tmp_path, "right_probe")
    assert 'imu_topic: "/right_probe/imu0"' in rendered
    assert 'image0_topic: "/right_probe/cam0/image_raw"' in rendered
    assert f'output_path: "{tmp_path / "solver_output"}/"' in rendered
