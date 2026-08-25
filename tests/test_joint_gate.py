import pytest

from three_device_slam.core.joint_gate import GateState, JointWarmupGate
from three_device_slam.core.model import DeviceHeartbeat


DEVICES = ("ego", "left", "right")


def feed_all(gate, seconds, healthy=True):
    for device in DEVICES:
        gate.observe(DeviceHeartbeat(device, int(seconds * 1e9), healthy))


def make_gate():
    return JointWarmupGate(DEVICES, 5_000_000_000, 3_000_000_000, 30_000_000_000)


def test_requires_five_total_and_three_continuous_healthy():
    gate = make_gate()
    feed_all(gate, 0)
    feed_all(gate, 2.9)
    assert gate.state == GateState.WARMING
    feed_all(gate, 5.0)
    assert gate.state == GateState.JOINT_READY


def test_bad_heartbeat_restarts_continuous_window():
    gate = make_gate()
    feed_all(gate, 0)
    gate.observe(DeviceHeartbeat("left", 2_000_000_000, False, ("frame_gap",)))
    feed_all(gate, 4.9)
    assert gate.state == GateState.WARMING
    feed_all(gate, 7.0)
    assert gate.state == GateState.WARMING
    feed_all(gate, 7.9)
    assert gate.state == GateState.JOINT_READY


def test_first_healthy_ego_frame_after_request_is_task_anchor():
    gate = make_gate()
    feed_all(gate, 0)
    feed_all(gate, 5)
    gate.request_recording(5_100_000_000)
    assert gate.observe_ego_frame(5_100_000_001, True) == 5_100_000_001
    assert gate.task_start_ego_frame_ns == 5_100_000_001


def test_timeout_is_blocked():
    gate = make_gate()
    feed_all(gate, 0)
    gate.tick(30_000_000_000)
    assert gate.state == GateState.BLOCKED


def test_unhealthy_heartbeat_revokes_ready_and_requires_fresh_recovery():
    gate = make_gate()
    feed_all(gate, 0)
    feed_all(gate, 5)
    assert gate.state == GateState.JOINT_READY
    gate.observe(DeviceHeartbeat("left", 6_000_000_000, False, ("frame_gap",)))
    assert gate.state == GateState.WARMING
    with pytest.raises(RuntimeError, match="JOINT_READY"):
        gate.request_recording(6_000_000_000)
    feed_all(gate, 6)
    gate.tick(8_999_999_999)
    assert gate.state == GateState.WARMING
    gate.tick(9_000_000_000)
    assert gate.state == GateState.JOINT_READY


def test_warmup_starts_when_last_worker_is_first_seen():
    gate = make_gate()
    gate.observe(DeviceHeartbeat("ego", 0, True))
    gate.observe(DeviceHeartbeat("left", 2_000_000_000, True))
    gate.observe(DeviceHeartbeat("right", 3_000_000_000, True))
    gate.tick(7_999_999_999)
    assert gate.state == GateState.WARMING
    gate.tick(8_000_000_000)
    assert gate.state == GateState.JOINT_READY


def test_timeout_is_anchored_to_first_worker_when_a_worker_is_missing():
    gate = make_gate()
    gate.observe(DeviceHeartbeat("ego", 0, True))
    gate.observe(DeviceHeartbeat("left", 1_000_000_000, True))
    gate.tick(30_000_000_000)
    assert gate.state == GateState.BLOCKED


def test_warmup_uses_latest_first_seen_timestamp_despite_callback_order():
    gate = make_gate()
    gate.observe(DeviceHeartbeat("ego", 3_000_000_000, True))
    gate.observe(DeviceHeartbeat("left", 2_000_000_000, True))
    gate.observe(DeviceHeartbeat("right", 1_000_000_000, True))
    gate.tick(6_000_000_000)
    assert gate.state == GateState.WARMING
    gate.tick(8_000_000_000)
    assert gate.state == GateState.JOINT_READY


def test_recovery_uses_latest_healthy_run_start_despite_callback_order():
    gate = make_gate()
    feed_all(gate, 0)
    gate.observe(DeviceHeartbeat("left", 2_000_000_000, False, ("frame_gap",)))
    gate.observe(DeviceHeartbeat("right", 2_000_000_000, False, ("frame_gap",)))
    gate.observe(DeviceHeartbeat("right", 5_100_000_000, True))
    gate.observe(DeviceHeartbeat("left", 5_000_000_000, True))
    gate.tick(8_000_000_000)
    assert gate.state == GateState.WARMING
    gate.tick(8_100_000_000)
    assert gate.state == GateState.JOINT_READY


def test_timeout_uses_earliest_first_seen_timestamp_despite_callback_order():
    gate = make_gate()
    gate.observe(DeviceHeartbeat("ego", 3_000_000_000, True))
    gate.observe(DeviceHeartbeat("left", 2_000_000_000, True))
    gate.tick(32_000_000_000)
    assert gate.state == GateState.BLOCKED


def test_blocked_gate_stays_blocked_when_delayed_workers_arrive():
    gate = make_gate()
    gate.observe(DeviceHeartbeat("ego", 0, True))
    gate.tick(30_000_000_000)
    assert gate.state == GateState.BLOCKED
    gate.observe(DeviceHeartbeat("left", 1_000_000_000, True))
    assert gate.state == GateState.BLOCKED
    gate.observe(DeviceHeartbeat("right", 2_000_000_000, True))
    gate.tick(7_000_000_000)
    assert gate.state == GateState.BLOCKED


def test_rejects_unknown_devices_and_timestamp_regression():
    gate = make_gate()
    with pytest.raises(ValueError, match="unknown"):
        gate.observe(DeviceHeartbeat("other", 0, True))
    gate.observe(DeviceHeartbeat("ego", 1, True))
    with pytest.raises(ValueError, match="regression"):
        gate.observe(DeviceHeartbeat("ego", 0, True))


def test_recording_request_is_only_permitted_when_joint_ready():
    with pytest.raises(RuntimeError, match="JOINT_READY"):
        make_gate().request_recording(0)


def test_only_first_healthy_ego_frame_at_or_after_request_latches_anchor():
    gate = make_gate()
    feed_all(gate, 0)
    feed_all(gate, 5)
    gate.request_recording(5_100_000_000)
    assert gate.observe_ego_frame(5_100_000_000, False) is None
    assert gate.observe_ego_frame(5_099_999_999, True) is None
    assert gate.observe_ego_frame(5_100_000_001, True) == 5_100_000_001
    assert gate.observe_ego_frame(5_100_000_002, True) is None
