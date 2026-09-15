import numpy as np
import pytest

from three_device_slam.spatial.d435i_live_vins import (
    FrozenSessionAnchorController,
    OnlineImuCombiner,
    ego_world_anchor_candidate,
)
from three_device_slam.spatial.device3_world_runtime import RuntimeDecision
from three_device_slam.spatial.se3 import compose, transform_from_xyz_rpy


def test_online_imu_combiner_interpolates_accel_at_gyro_stamp():
    combiner = OnlineImuCombiner(np.eye(3))
    combiner.push_accel(100, (0.0, 0.0, 9.0))
    assert combiner.push_gyro(150, (0.1, 0.2, 0.3)) == ()
    samples = combiner.push_accel(200, (0.0, 2.0, 11.0))

    assert len(samples) == 1
    assert samples[0].timestamp_ns == 150
    assert np.allclose(samples[0].gyro_rad_s, (0.1, 0.2, 0.3))
    assert np.allclose(samples[0].accel_m_s2, (0.0, 1.0, 10.0))


def test_online_imu_combiner_rotates_accel_without_rotating_gyro():
    rotation = np.array(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    combiner = OnlineImuCombiner(rotation)
    combiner.push_accel(100, (1.0, 2.0, 3.0))
    sample = combiner.push_gyro(100, (4.0, 5.0, 6.0))[0]

    assert np.allclose(sample.gyro_rad_s, (4.0, 5.0, 6.0))
    assert np.allclose(sample.accel_m_s2, (-2.0, 1.0, 3.0))


def test_online_imu_combiner_rejects_timestamp_regression():
    combiner = OnlineImuCombiner(np.eye(3))
    combiner.push_accel(100, (0.0, 0.0, 9.8))
    with pytest.raises(ValueError, match="accel_timestamp"):
        combiner.push_accel(100, (0.0, 0.0, 9.8))


def test_ego_anchor_recovers_world_from_local_odom():
    world_from_camera = transform_from_xyz_rpy((1.0, 2.0, 3.0), (0.1, 0.2, 0.3))
    body_from_camera = transform_from_xyz_rpy((0.02, 0.0, 0.01))
    odom_from_body = transform_from_xyz_rpy((0.4, -0.2, 0.1), (-0.1, 0.05, 0.2))

    candidate = ego_world_anchor_candidate(
        timestamp_ns=123,
        gyro_rad_s=np.array((0.01, 0.02, 0.03)),
        world_from_camera=world_from_camera,
        body_from_camera=body_from_camera,
        odom_from_body=odom_from_body,
    )

    expected_world_from_body = compose(world_from_camera, np.linalg.inv(body_from_camera))
    assert np.allclose(
        compose(candidate.world_from_odom, odom_from_body),
        expected_world_from_body,
    )


def test_frozen_session_anchor_initializes_once_and_holds():
    transform = transform_from_xyz_rpy((1.0, 2.0, 3.0))

    class Controller:
        current = None

        def __init__(self):
            self.calls = 0

        def process_window(self, values):
            self.calls += 1
            self.current = type("Anchor", (), {"world_from_odom": transform})()
            return RuntimeDecision("INITIALIZED", "accepted", transform, len(values), len(values), len(values))

        def world_from_body(self, value):
            return compose(self.current.world_from_odom, value)

    inner = Controller()
    frozen = FrozenSessionAnchorController(inner)
    first = frozen.process_window((object(), object(), object()))
    second = frozen.process_window((object(), object(), object()))

    assert first.status == "INITIALIZED"
    assert second.status == "HELD"
    assert second.reason == "session_anchor_frozen"
    assert inner.calls == 1
