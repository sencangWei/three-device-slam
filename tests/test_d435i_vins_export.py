import numpy as np
import pytest

from three_device_slam.devices.d435i_ego.vins_export import (
    combine_imu,
    factory_transforms,
    interpolation_support_rows,
    validate_capture_window,
)


def test_bracket_interpolation_preserves_gyro_and_rotates_accel():
    rotation = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    values, audit = combine_imu([5, 10], [[1., 2, 3], [4, 5, 6]],
        [0, 10, 20], [[0., 0, 10], [2, 0, 10], [4, 0, 10]], rotation)
    np.testing.assert_array_equal(values[:, :3], [[1, 2, 3], [4, 5, 6]])
    np.testing.assert_allclose(values[:, 3:], [[0, 1, 10], [0, 2, 10]])
    assert audit == [(5, 0, 10, .5), (10, 10, 10, 0)]


@pytest.mark.parametrize("gt,at,error", [
    ([-1, 5], [0, 10], "bracket"), ([5, 11], [0, 10], "bracket"),
    ([5, 5], [0, 10], "increasing"), ([1, 2], [0, 0], "increasing"),
    ([1, 2], [0, 20_000_000], "gap"),
])
def test_no_extrapolation_gaps_or_regressions(gt, at, error):
    with pytest.raises(ValueError, match=error):
        combine_imu(gt, np.zeros((2, 3)), at, np.zeros((2, 3)), np.eye(3))


def test_factory_extrinsics_direction():
    def entry(x):
        return {"rotation_row_major": np.eye(3).reshape(-1).tolist(), "translation_m": [x, 0, 0]}
    cal = {"extrinsics": {"gyro_to_ir_left": entry(.005), "accel_to_ir_left": entry(.005),
                          "gyro_to_accel": entry(0), "ir_left_to_ir_right": entry(-.05)}}
    c0, c1, rotation = factory_transforms(cal)
    assert c0[0, 3] == -.005
    assert c1[0, 3] == pytest.approx(.045)
    np.testing.assert_allclose(np.linalg.inv(c1) @ c0, np.array([
        [1, 0, 0, -.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
    cal["extrinsics"]["gyro_to_accel"] = entry(.01)
    with pytest.raises(ValueError, match="lever-arm"):
        factory_transforms(cal)


def test_invalid_imu_values():
    with pytest.raises(ValueError, match="nonfinite"):
        combine_imu([1, 2], np.full((2, 3), np.nan), [0, 3], np.zeros((2, 3)), np.eye(3))


def test_interpolation_support_keeps_warmup_sample_that_brackets_formal_gyro():
    rows = [
        {"acquisition_ns": 0, "warmup": True},
        {"acquisition_ns": 10, "warmup": False},
        {"acquisition_ns": 20, "warmup": False},
        {"acquisition_ns": 30, "warmup": False},
    ]
    support = interpolation_support_rows(rows, [5, 25])
    assert [row["acquisition_ns"] for row in support] == [0, 10, 20, 30]
    assert support[0]["warmup"] is True


def test_interpolation_support_rejects_unbracketed_target():
    rows = [{"acquisition_ns": 10}, {"acquisition_ns": 20}]
    with pytest.raises(ValueError, match="no extrapolation"):
        interpolation_support_rows(rows, [5, 15])


def test_capture_window_accepts_multiaxis_protocol_and_remains_bounded():
    assert validate_capture_window(10, 105_000_000_010) == 105_000_000_000
    with pytest.raises(ValueError, match="<=120s"):
        validate_capture_window(10, 120_000_000_011)
