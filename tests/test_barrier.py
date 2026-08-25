import json
import threading

import pytest

from three_device_slam.core.barrier import BarrierDirectory
from three_device_slam.core.model import DeviceHeartbeat


def test_round_trip_has_no_partial_file(tmp_path):
    barrier = BarrierDirectory(tmp_path)

    barrier.write_heartbeat(DeviceHeartbeat("ego", 10, True))

    assert barrier.read_heartbeats()["ego"].healthy
    barrier.schedule_start(20)
    assert barrier.read_start_ns() == 20
    assert not list((tmp_path / "barrier").glob("*.tmp"))


def test_schedule_start_is_exclusive(tmp_path):
    barrier = BarrierDirectory(tmp_path)

    barrier.schedule_start(20)

    with pytest.raises(FileExistsError):
        barrier.schedule_start(30)
    assert barrier.read_start_ns() == 20


def test_schedule_start_has_one_concurrent_first_writer(tmp_path):
    barrier = BarrierDirectory(tmp_path)
    rendezvous = threading.Barrier(2)
    outcomes = []

    def schedule(start_ns):
        rendezvous.wait()
        try:
            barrier.schedule_start(start_ns)
            outcomes.append(("scheduled", start_ns))
        except FileExistsError:
            outcomes.append(("already_scheduled", start_ns))

    workers = [threading.Thread(target=schedule, args=(start_ns,)) for start_ns in (20, 30)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join()

    assert sorted(result for result, _ in outcomes) == ["already_scheduled", "scheduled"]
    winner_ns = next(start_ns for result, start_ns in outcomes if result == "scheduled")
    assert barrier.read_start_ns() == winner_ns
    assert json.loads((tmp_path / "barrier" / "start.json").read_text()) == {"start_ns": winner_ns}
    assert not list((tmp_path / "barrier").glob("*.tmp"))


def test_failure_is_first_error_latched(tmp_path):
    barrier = BarrierDirectory(tmp_path)

    barrier.latch_failure("left", "timestamp_regression", 30)
    barrier.latch_failure("left", "later_error", 40)

    assert barrier.read_failures()["left"] == {
        "device_id": "left",
        "reason": "timestamp_regression",
        "at_ns": 30,
    }


@pytest.mark.parametrize("timestamp", [-1, True])
def test_invalid_start_timestamp_does_not_claim_schedule(tmp_path, timestamp):
    barrier = BarrierDirectory(tmp_path)

    with pytest.raises(ValueError):
        barrier.schedule_start(timestamp)

    assert not (tmp_path / "barrier" / "start.lock").exists()
    assert barrier.read_start_ns() is None
    barrier.schedule_start(20)
    assert barrier.read_start_ns() == 20


@pytest.mark.parametrize(
    "device_id",
    ["", "/", "\\", "..", ":", "other", "ego/other", r"ego\other", "ego:other"],
)
def test_invalid_device_ids_do_not_create_barrier_records(tmp_path, device_id):
    barrier = BarrierDirectory(tmp_path)

    with pytest.raises(ValueError):
        barrier.write_heartbeat(DeviceHeartbeat(device_id, 10, True))
    with pytest.raises(ValueError):
        barrier.latch_failure(device_id, "timestamp_regression", 30)

    assert not list((tmp_path / "barrier").iterdir())
    barrier.write_heartbeat(DeviceHeartbeat("ego", 10, True))
    barrier.latch_failure("left", "timestamp_regression", 30)
    assert set(barrier.read_heartbeats()) == {"ego"}
    assert set(barrier.read_failures()) == {"left"}


@pytest.mark.parametrize("timestamp", [-1, True])
def test_invalid_failure_timestamp_does_not_latch(tmp_path, timestamp):
    barrier = BarrierDirectory(tmp_path)

    with pytest.raises(ValueError):
        barrier.latch_failure("left", "timestamp_regression", timestamp)

    assert not (tmp_path / "barrier" / "failure.left.lock").exists()
    assert not barrier.read_failures()
    barrier.latch_failure("left", "timestamp_regression", 30)
    assert barrier.read_failures()["left"]["at_ns"] == 30


@pytest.mark.parametrize("reason", ["", 42, object()])
def test_invalid_failure_reason_does_not_latch(tmp_path, reason):
    barrier = BarrierDirectory(tmp_path)

    with pytest.raises(ValueError):
        barrier.latch_failure("left", reason, 30)

    assert not (tmp_path / "barrier" / "failure.left.lock").exists()
    assert not barrier.read_failures()
    barrier.latch_failure("left", "timestamp_regression", 30)
    assert barrier.read_failures()["left"]["reason"] == "timestamp_regression"


def test_publish_crash_keeps_start_claim_terminal(tmp_path, monkeypatch):
    barrier = BarrierDirectory(tmp_path)

    def crash(*_args):
        raise OSError("simulated publish crash")

    monkeypatch.setattr(barrier, "_write_atomic", crash)

    with pytest.raises(OSError, match="simulated publish crash"):
        barrier.schedule_start(20)
    with pytest.raises(FileExistsError):
        barrier.schedule_start(30)
    assert barrier.read_start_ns() is None


def test_failure_publish_crash_releases_claim_for_retry(tmp_path, monkeypatch):
    barrier = BarrierDirectory(tmp_path)
    original = barrier._write_atomic
    attempts = 0

    def fail_once(*args):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise OSError("simulated failure publish crash")
        return original(*args)

    monkeypatch.setattr(barrier, "_write_atomic", fail_once)

    with pytest.raises(OSError, match="simulated failure publish crash"):
        barrier.latch_failure("left", "camera_disconnect", 10)

    assert not (tmp_path / "barrier" / "failure.left.lock").exists()
    barrier.latch_failure("left", "imu_disconnect", 20)
    assert barrier.read_failures()["left"]["reason"] == "imu_disconnect"


def test_stop_request_is_first_writer_latched(tmp_path):
    barrier = BarrierDirectory(tmp_path)
    barrier.request_stop(100, "operator_interrupt")
    barrier.request_stop(200, "later_reason")
    assert barrier.read_stop() == {"at_ns": 100, "reason": "operator_interrupt"}


def test_invalid_stop_request_writes_nothing(tmp_path):
    barrier = BarrierDirectory(tmp_path)
    with pytest.raises(ValueError):
        barrier.request_stop(-1, "operator_interrupt")
    with pytest.raises(ValueError):
        barrier.request_stop(1, "")
    assert barrier.read_stop() is None
