import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from three_device_slam.core.barrier import BarrierDirectory
from three_device_slam.core.model import DeviceHeartbeat
from three_device_slam.acquisition import coordinator


DEVICES = ("ego", "left", "right")


class FakeClock:
    def __init__(self, tick_on_call_ns=0):
        self.now_ns = 0
        self.tick_on_call_ns = tick_on_call_ns

    def __call__(self):
        now_ns = self.now_ns
        self.now_ns += self.tick_on_call_ns
        return now_ns

    def sleep(self, seconds):
        self.now_ns += int(seconds * 1_000_000_000)


class FakeProcess:
    def __init__(self, device_id, barrier, clock, duration_s, behavior):
        self.device_id = device_id
        self.barrier = barrier
        self.clock = clock
        self.duration_ns = int(duration_s * 1_000_000_000)
        self.behavior = behavior
        self.returncode = None
        self.terminate_called = False
        self.kill_called = False
        self.finalized_naturally = False
        self.start_observations = []
        self._poll_count = 0
        self._last_heartbeat_write_ns = None
        self._last_health = None
        self._wait_calls = 0

    def poll(self):
        if self.returncode is not None:
            return self.returncode
        now_ns = self.clock()
        start_ns = self.barrier.read_start_ns()
        self.start_observations.append((now_ns, start_ns))
        self._poll_count += 1

        first_heartbeat_ns = self.behavior.get("first_heartbeat_ns", 0)
        stop_after_start_ns = self.behavior.get("stop_heartbeat_after_start_ns")
        heartbeat_stopped = (
            start_ns is not None
            and stop_after_start_ns is not None
            and now_ns >= start_ns + stop_after_start_ns
        )
        if (
            now_ns >= first_heartbeat_ns
            and not self.behavior.get("no_heartbeat", False)
            and not heartbeat_stopped
        ):
            healthy = self.behavior.get("healthy", lambda _now: True)(now_ns)
            interval_ns = self.behavior.get("heartbeat_interval_ns", 500_000_000)
            write_due = (
                self._last_heartbeat_write_ns is None
                or now_ns - self._last_heartbeat_write_ns >= interval_ns
                or healthy != self._last_health
            )
            if write_due:
                timestamp = self.behavior.get(
                    "heartbeat_timestamp", lambda now, _count: now
                )(now_ns, self._poll_count)
                reasons = () if healthy else ("warming",)
                self.barrier.write_heartbeat(
                    DeviceHeartbeat(self.device_id, timestamp, healthy, reasons)
                )
                self._last_heartbeat_write_ns = now_ns
                self._last_health = healthy

        exit_ns = self.behavior.get("exit_ns")
        if start_ns is not None:
            offset = self.behavior.get(
                "exit_after_start_ns", self.duration_ns + 50_000_000
            )
            exit_ns = start_ns + offset
        if exit_ns is not None and now_ns >= exit_ns:
            failure_reason = self.behavior.get("failure_reason")
            if failure_reason:
                self.barrier.latch_failure(self.device_id, failure_reason, now_ns)
            self.returncode = self.behavior.get("exit_code", 0)
            self.finalized_naturally = True
            self._write_acceptance(start_ns)
        return self.returncode

    def terminate(self):
        self.terminate_called = True
        if self.returncode is None:
            self.returncode = -15

    def wait(self, timeout=None):
        self._wait_calls += 1
        failures = self.behavior.get("wait_failures", ())
        if self._wait_calls <= len(failures):
            failure = failures[self._wait_calls - 1]
            if failure == "timeout":
                raise subprocess.TimeoutExpired([self.device_id], timeout)
            if failure == "oserror":
                raise OSError("wait unavailable")
        return self.returncode

    def kill(self):
        self.kill_called = True
        self.terminate()

    def _write_acceptance(self, start_ns):
        if self.behavior.get("omit_acceptance", False):
            return
        device_directory = self.barrier.path.parent / self.device_id
        device_directory.mkdir(parents=True, exist_ok=True)
        if "acceptance_payload" in self.behavior:
            payload = self.behavior["acceptance_payload"]
        elif self.device_id == "ego":
            payload = {
                "schema": "ego.2uq2.acceptance.v1",
                "status": self.behavior.get("acceptance_status", "PASS"),
                "formal_evidence": {
                    "first_acquisition_ns": None if start_ns is None else start_ns + 10
                },
                "hashes": {"xu_library_sha256": "ego-xu-evidence"},
            }
        else:
            payload = {
                "result": self.behavior.get("acceptance_status", "PASS"),
                "live_vins": {"imu_calibration": None},
            }
        (device_directory / "acceptance.json").write_text(json.dumps(payload))


def run_fake_session(
    tmp_path,
    monkeypatch,
    *,
    behaviors=None,
    duration_s=1.0,
    commands=None,
    auto_start=True,
    on_prompt=None,
    clock=None,
    stop_requested=lambda: False,
):
    session = tmp_path / "session"
    clock = clock or FakeClock()
    barrier = BarrierDirectory(session)
    behaviors = behaviors or {}
    commands = commands or {device: [device] for device in DEVICES}
    processes = {}
    launches = []

    def fake_popen(command, **kwargs):
        device_id = command[0]
        launches.append((list(command), dict(kwargs)))
        process = FakeProcess(
            device_id,
            barrier,
            clock,
            duration_s,
            behaviors.get(device_id, {}),
        )
        processes[device_id] = process
        return process

    monkeypatch.setattr(coordinator, "_popen", fake_popen)
    monkeypatch.setattr(coordinator, "_sleep", clock.sleep)
    if on_prompt is not None:
        monkeypatch.setattr(
            coordinator,
            "_input",
            lambda _message: on_prompt(clock, barrier),
        )
    report = coordinator.run_coordinator(
        session,
        commands,
        duration_s,
        clock_ns=clock,
        auto_start=auto_start,
        stop_requested=stop_requested,
    )
    return session, clock, processes, launches, report


def transition_time(report, state):
    return next(row["at_ns"] for row in report["transitions"] if row["state"] == state)


def test_product_run_never_prompts_twice(tmp_path, monkeypatch):
    monkeypatch.setattr(
        coordinator,
        "_input",
        lambda _message: (_ for _ in ()).throw(AssertionError("unexpected prompt")),
    )
    _, _, _, _, report = run_fake_session(tmp_path, monkeypatch, auto_start=True)
    assert report["scheduled_start_ns"] is not None


def test_stop_callback_publishes_operator_stop(tmp_path, monkeypatch):
    session, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=1.0,
        stop_requested=lambda: True,
    )
    payload = json.loads((session / "barrier" / "stop.json").read_text())
    assert payload["reason"] == "operator_interrupt"
    assert all(process.finalized_naturally for process in processes.values())
    assert report["reason"] == "operator_stop"


def test_stop_callback_timeout_terminates_unresponsive_worker(tmp_path, monkeypatch):
    _, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=1.0,
        stop_requested=lambda: True,
        behaviors={"left": {"exit_after_start_ns": 100_000_000_000}},
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_stop_timeout"
    assert processes["left"].terminate_called


def test_operator_cancel_uses_one_timestamp_for_error_and_transition(
    tmp_path, monkeypatch
):
    clock = FakeClock(tick_on_call_ns=1)

    def cancel(_clock, _barrier):
        raise EOFError

    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        auto_start=False,
        on_prompt=cancel,
        clock=clock,
    )

    assert report["reason"] == "operator_cancelled"
    assert report["first_error"]["at_ns"] == transition_time(report, "BLOCKED")


def test_start_waits_for_last_first_heartbeat_warmup_and_continuous_health(
    tmp_path, monkeypatch
):
    session, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={"right": {"first_heartbeat_ns": 2_000_000_000}},
    )

    assert transition_time(report, "JOINT_READY") == 7_000_000_000
    assert report["scheduled_start_ns"] == 7_500_000_000
    assert all(
        observed_start is None
        for process in processes.values()
        for now_ns, observed_start in process.start_observations
        if now_ns < 7_000_000_000
    )
    assert json.loads((session / "barrier" / "start.json").read_text()) == {
        "start_ns": 7_500_000_000
    }


def test_unhealthy_heartbeat_requires_a_fresh_full_recovery_window(tmp_path, monkeypatch):
    unhealthy_from = 4_000_000_000
    healthy_again = 4_500_000_000
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "healthy": lambda now: not (unhealthy_from <= now < healthy_again)
            }
        },
    )

    assert transition_time(report, "JOINT_READY") == 7_500_000_000
    assert report["scheduled_start_ns"] == 8_000_000_000


@pytest.mark.parametrize(
    "behaviors",
    [
        {"right": {"no_heartbeat": True}},
        {"right": {"healthy": lambda _now: False}},
    ],
    ids=("zero-heartbeat", "never-healthy"),
)
def test_launch_anchored_timeout_blocks_without_start(tmp_path, monkeypatch, behaviors):
    session, clock, _, _, report = run_fake_session(
        tmp_path, monkeypatch, behaviors=behaviors
    )

    assert clock() == 30_000_000_000
    assert report["status"] == "BLOCKED"
    assert report["reason"] == "joint_warmup_timeout"
    assert report["scheduled_start_ns"] is None
    assert not (session / "barrier" / "start.json").exists()


def test_prestart_timeout_does_not_mask_trusted_failed_acceptance(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(
        coordinator,
        "_load_acceptances",
        lambda _session: (
            {
                "ego": {
                    "schema": "ego.2uq2.acceptance.v1",
                    "status": "FAIL",
                    "reason": "storage_io",
                    "formal_evidence": {},
                    "hashes": {},
                },
                "left": {"result": "PASS", "live_vins": {}},
            },
            [],
        ),
    )

    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={"right": {"no_heartbeat": True}},
    )

    assert report["status"] == "FAIL"
    assert report["worker_acceptance"]["ego"]["reason"] == "storage_io"


def test_start_is_published_once_at_now_plus_500_ms_and_shell_is_false(
    tmp_path, monkeypatch
):
    commands = {
        "ego": ["ego", "--token", "do-not-record"],
        "left": ["left"],
        "right": ["right"],
    }
    session, _, _, launches, report = run_fake_session(
        tmp_path, monkeypatch, commands=commands
    )

    ready_ns = transition_time(report, "JOINT_READY")
    assert report["scheduled_start_ns"] == ready_ns + 500_000_000
    assert all(kwargs == {"shell": False} for _, kwargs in launches)
    assert report["worker_commands"]["ego"] == ["ego", "--token", "<redacted>"]
    assert len(list((session / "barrier").glob("start.json"))) == 1


def test_prompt_rechecks_revoked_health_once_before_later_recovery(tmp_path, monkeypatch):
    prompts = []

    def revoke_health(clock, barrier):
        prompts.append(clock())
        clock.sleep(0.05)
        barrier.write_heartbeat(
            DeviceHeartbeat("left", clock(), False, ("warming",))
        )

    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        auto_start=False,
        on_prompt=revoke_health,
        behaviors={
            "left": {"healthy": lambda now: not (5_000_000_000 < now < 5_500_000_000)}
        },
    )

    assert prompts == [5_000_000_000]
    assert report["scheduled_start_ns"] == 9_000_000_000


def test_prompt_requires_new_three_second_health_window_after_confirmation(
    tmp_path, monkeypatch
):
    confirmation_times = []

    def transient_unhealthy_then_healthy(clock, barrier):
        clock.sleep(0.05)
        barrier.write_heartbeat(
            DeviceHeartbeat("left", clock(), False, ("warming",))
        )
        clock.sleep(0.05)
        barrier.write_heartbeat(DeviceHeartbeat("left", clock(), True))
        confirmation_times.append(clock())

    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        auto_start=False,
        on_prompt=transient_unhealthy_then_healthy,
    )

    assert report["status"] == "PASS"
    assert report["scheduled_start_ns"] >= (
        confirmation_times[0]
        + coordinator.CONTINUOUS_HEALTHY_NS
        + coordinator.START_GUARD_NS
    )


def test_prompt_does_not_schedule_from_stale_pre_prompt_heartbeats(tmp_path, monkeypatch):
    def disconnect_during_prompt(clock, _barrier):
        clock.sleep(1.0)

    session, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        auto_start=False,
        on_prompt=disconnect_during_prompt,
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_disconnect"
    assert report["scheduled_start_ns"] is None
    assert not (session / "barrier" / "start.json").exists()


def test_prompt_recheck_records_timestamp_regression_instead_of_escaping(
    tmp_path, monkeypatch
):
    def regress_during_prompt(clock, barrier):
        clock.sleep(0.05)
        barrier.write_heartbeat(DeviceHeartbeat("left", 1, True))

    session, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        auto_start=False,
        on_prompt=regress_during_prompt,
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "timestamp_regression"
    assert report["coordinator_errors"][0]["device_id"] == "left"
    assert not (session / "barrier" / "start.json").exists()


def test_prompt_return_at_launch_deadline_blocks_before_schedule(tmp_path, monkeypatch):
    def return_at_deadline_with_fresh_health(clock, barrier):
        clock.sleep(25.0)
        for device in DEVICES:
            barrier.write_heartbeat(DeviceHeartbeat(device, clock(), True))

    session, clock, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        auto_start=False,
        on_prompt=return_at_deadline_with_fresh_health,
    )

    assert clock() == 30_000_000_000
    assert report["status"] == "BLOCKED"
    assert report["reason"] == "joint_warmup_timeout"
    assert report["scheduled_start_ns"] is None
    assert report["transitions"][-1] == {"state": "BLOCKED", "at_ns": clock()}
    assert not (session / "barrier" / "start.json").exists()


def test_recording_worker_failure_does_not_kill_remaining_finalizers(
    tmp_path, monkeypatch
):
    _, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=3.0,
        behaviors={
            "left": {
                "exit_after_start_ns": 500_000_000,
                "exit_code": 7,
                "failure_reason": "camera_disconnect",
                "acceptance_status": "FAIL",
            }
        },
    )

    assert report["status"] == "FAIL"
    assert report["workers"]["left"] == {
        "exit_code": 7,
        "early_exit": True,
        "observed_running_at_or_after_formal_end": False,
    }
    assert report["barrier_failures"]["left"]["reason"] == "camera_disconnect"
    assert all(process.finalized_naturally for process in processes.values())
    assert not any(process.terminate_called for process in processes.values())


def test_recording_heartbeat_disconnect_fails_but_allows_natural_finalization(
    tmp_path, monkeypatch
):
    _, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=2.0,
        behaviors={"left": {"stop_heartbeat_after_start_ns": 100_000_000}},
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_disconnect"
    assert report["barrier_failures"]["left"]["reason"] == "worker_disconnect"
    assert all(process.finalized_naturally for process in processes.values())
    assert not any(process.terminate_called for process in processes.values())


def test_recording_failure_survives_barrier_latch_io_error(tmp_path, monkeypatch):
    def fail_latch(_self, _device_id, _reason, _at_ns):
        raise OSError("barrier unavailable")

    monkeypatch.setattr(BarrierDirectory, "latch_failure", fail_latch)
    session, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=2.0,
        behaviors={"left": {"stop_heartbeat_after_start_ns": 100_000_000}},
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_disconnect"
    assert report["first_error"]["reason"] == "worker_disconnect"
    assert any(error["reason"] == "barrier_io" for error in report["coordinator_errors"])
    assert (session / "coordinator.json").exists()
    assert all(process.finalized_naturally for process in processes.values())


def test_post_deadline_heartbeat_stops_while_worker_finalizes_normally(
    tmp_path, monkeypatch
):
    duration_s = 2.0
    _, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=duration_s,
        behaviors={
            "left": {
                "stop_heartbeat_after_start_ns": int(duration_s * 1_000_000_000),
                "exit_after_start_ns": int((duration_s + 0.8) * 1_000_000_000),
            }
        },
    )

    assert report["status"] == "PASS"
    assert report["workers"]["left"] == {
        "exit_code": 0,
        "early_exit": False,
        "observed_running_at_or_after_formal_end": True,
    }
    assert processes["left"].finalized_naturally
    assert not processes["left"].terminate_called


def test_exit_just_before_deadline_observed_on_deadline_is_fail_closed(
    tmp_path, monkeypatch
):
    duration_s = 1.0
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=duration_s,
        behaviors={
            "left": {
                "exit_after_start_ns": int(duration_s * 1_000_000_000) - 1
            }
        },
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_exit"
    assert report["workers"]["left"]["exit_code"] == 0
    assert report["workers"]["left"]["early_exit"] is True
    assert report["workers"]["left"]["observed_running_at_or_after_formal_end"] is False


def test_finalization_timeout_is_bounded_without_killing_completed_workers(
    tmp_path, monkeypatch
):
    duration_s = 1.0
    _, clock, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=duration_s,
        behaviors={
            "left": {
                "stop_heartbeat_after_start_ns": int(duration_s * 1_000_000_000),
                "exit_after_start_ns": 100_000_000_000,
            }
        },
    )

    formal_end_ns = report["scheduled_start_ns"] + int(duration_s * 1_000_000_000)
    assert clock() == formal_end_ns + coordinator.FINALIZATION_GRACE_NS
    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_finalize_timeout"
    assert report["workers"]["left"] == {
        "exit_code": -15,
        "early_exit": False,
        "observed_running_at_or_after_formal_end": True,
    }
    assert processes["left"].terminate_called
    assert not processes["left"].finalized_naturally
    assert processes["ego"].finalized_naturally
    assert processes["right"].finalized_naturally
    assert not processes["ego"].terminate_called
    assert not processes["right"].terminate_called


@pytest.mark.parametrize("second_wait_failure", ["timeout", "oserror"])
def test_finalization_second_wait_failure_still_writes_terminal_report(
    tmp_path, monkeypatch, second_wait_failure
):
    session, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=1.0,
        behaviors={
            "left": {
                "exit_after_start_ns": 100_000_000_000,
                "wait_failures": ("timeout", second_wait_failure),
            }
        },
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_finalize_timeout"
    assert report["first_error"]["reason"] == "worker_finalize_timeout"
    assert report["workers"]["left"]["exit_code"] is None
    assert processes["left"].kill_called
    assert json.loads((session / "coordinator.json").read_text()) == report


def test_prestart_second_wait_failure_preserves_blocked_report(tmp_path, monkeypatch):
    session, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {"wait_failures": ("timeout", "timeout")},
            "right": {"no_heartbeat": True},
        },
    )

    assert report["status"] == "BLOCKED"
    assert report["reason"] == "joint_warmup_timeout"
    assert report["first_error"]["reason"] == "joint_warmup_timeout"
    assert report["workers"]["left"]["exit_code"] is None
    assert processes["left"].kill_called
    assert json.loads((session / "coordinator.json").read_text()) == report


def test_heartbeat_timestamp_regression_is_terminal_and_recorded(tmp_path, monkeypatch):
    def regressing_timestamp(_now, poll_count):
        return 1 if poll_count == 1 else 0

    session, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "heartbeat_timestamp": regressing_timestamp,
                "heartbeat_interval_ns": 50_000_000,
            }
        },
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "timestamp_regression"
    assert report["coordinator_errors"][0]["device_id"] == "left"
    assert not (session / "barrier" / "start.json").exists()


def test_final_report_is_atomic_durable_and_contains_provenance(tmp_path, monkeypatch):
    session, _, _, _, report = run_fake_session(tmp_path, monkeypatch)

    stored = json.loads((session / "coordinator.json").read_text())
    assert stored == report
    assert stored["schema"] == "ego.three_device.coordinator.v1"
    assert stored["status"] == "PASS"
    assert stored["reason"] == "acquisition_timing"
    assert stored["task_start_ego_frame_ns"] == stored["scheduled_start_ns"] + 10
    assert stored["transitions"][-1]["state"] == "OFFLINE_QC"
    assert set(stored["workers"]) == set(DEVICES)
    assert set(stored["calibration_evidence_ids"]) == set(DEVICES)
    assert stored["host"]["platform"]
    assert set(stored["git"]) == {"hash", "dirty"}
    assert not list(session.glob("*.tmp"))


def test_first_report_write_failure_populates_storage_first_error(tmp_path, monkeypatch):
    original_write = coordinator._write_atomic_json
    calls = []

    def fail_once(path, payload):
        calls.append(path)
        if len(calls) == 1:
            raise OSError("first write failed")
        return original_write(path, payload)

    monkeypatch.setattr(coordinator, "_write_atomic_json", fail_once)
    session, _, _, _, report = run_fake_session(tmp_path, monkeypatch)

    assert len(calls) == 2
    assert report["status"] == "FAIL"
    assert report["reason"] == "storage_io"
    assert report["first_error"]["reason"] == "storage_io"
    assert json.loads((session / "coordinator.json").read_text()) == report


def test_storage_failure_overrides_blocked_clock_worker(tmp_path, monkeypatch):
    original_write = coordinator._write_atomic_json
    calls = []

    def fail_once(path, payload):
        calls.append(path)
        if len(calls) == 1:
            raise OSError("first write failed")
        return original_write(path, payload)

    monkeypatch.setattr(coordinator, "_write_atomic_json", fail_once)
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "acceptance_payload": {
                    "result": "BLOCKED",
                    "reason": "common_clock_unverified",
                },
                "exit_code": 3,
            }
        },
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "storage_io"
    assert any(error["reason"] == "storage_io" for error in report["coordinator_errors"])


def test_missing_worker_acceptance_is_not_a_pass(tmp_path, monkeypatch):
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={"right": {"omit_acceptance": True}},
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_acceptance_missing"
    assert report["first_error"]["device_id"] == "right"


def test_d405_unverified_clock_acceptance_propagates_blocked_after_recording(
    tmp_path, monkeypatch
):
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "acceptance_payload": {
                    "result": "BLOCKED",
                    "reason": "common_clock_unverified",
                },
                "exit_code": 3,
            }
        },
    )

    assert report["status"] == "BLOCKED"
    assert report["reason"] == "common_clock_unverified"


def test_blocked_clock_worker_does_not_mask_barrier_failure(tmp_path, monkeypatch):
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "acceptance_payload": {
                    "result": "BLOCKED",
                    "reason": "common_clock_unverified",
                },
                "exit_code": 3,
            },
            "right": {"failure_reason": "timestamp_regression"},
        },
    )

    assert report["status"] == "FAIL"
    assert report["barrier_failures"]["right"]["reason"] == "timestamp_regression"


def test_blocked_clock_worker_does_not_mask_late_barrier_io(
    tmp_path, monkeypatch
):
    original_read_failures = coordinator.BarrierDirectory.read_failures
    post_acceptance_reads = 0

    def fail_after_all_acceptances_exist(barrier):
        nonlocal post_acceptance_reads
        session = barrier.path.parent
        if all((session / device / "acceptance.json").exists() for device in DEVICES):
            post_acceptance_reads += 1
            if post_acceptance_reads == 2:
                raise OSError("final barrier read failed")
        return original_read_failures(barrier)

    monkeypatch.setattr(
        coordinator.BarrierDirectory, "read_failures", fail_after_all_acceptances_exist
    )
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "acceptance_payload": {
                    "result": "BLOCKED",
                    "reason": "common_clock_unverified",
                },
                "exit_code": 3,
            }
        },
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "barrier_io"
    assert report["first_error"]["reason"] == "common_clock_unverified"
    assert any(
        error["reason"] == "barrier_io" for error in report["coordinator_errors"]
    )


def test_late_barrier_failure_reason_overrides_old_blocked_first_error(
    tmp_path, monkeypatch
):
    original_read_failures = coordinator.BarrierDirectory.read_failures
    post_acceptance_reads = 0

    def inject_final_failure(barrier):
        nonlocal post_acceptance_reads
        session = barrier.path.parent
        if all((session / device / "acceptance.json").exists() for device in DEVICES):
            post_acceptance_reads += 1
            if post_acceptance_reads == 2:
                return {
                    "right": {
                        "device_id": "right",
                        "reason": "timestamp_regression",
                        "at_ns": 1_200_000_000,
                    }
                }
        return original_read_failures(barrier)

    monkeypatch.setattr(
        coordinator.BarrierDirectory, "read_failures", inject_final_failure
    )
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "acceptance_payload": {
                    "result": "BLOCKED",
                    "reason": "common_clock_unverified",
                },
                "exit_code": 3,
            }
        },
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "timestamp_regression"
    assert report["first_error"]["reason"] == "common_clock_unverified"


def test_blocked_worker_does_not_mask_another_missing_acceptance(tmp_path, monkeypatch):
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={
            "left": {
                "acceptance_payload": {
                    "result": "BLOCKED",
                    "reason": "common_clock_unverified",
                },
                "exit_code": 3,
            },
            "right": {"omit_acceptance": True},
        },
    )

    assert report["status"] == "FAIL"
    assert report["reason"] != "common_clock_unverified"


@pytest.mark.parametrize(
    ("device_id", "payload"),
    [
        ("left", []),
        ("right", {"result": "UNKNOWN"}),
        ("ego", {"schema": "wrong.schema", "status": "PASS"}),
        ("left", {"status": "PASS"}),
    ],
    ids=("nonmapping", "unknown-status", "bad-ego-schema", "d405-wrong-field"),
)
def test_invalid_worker_acceptance_is_fail_closed(
    tmp_path, monkeypatch, device_id, payload
):
    _, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={device_id: {"acceptance_payload": payload}},
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_acceptance_invalid"
    assert report["first_error"]["device_id"] == device_id


@pytest.mark.parametrize(
    ("device_id", "payload"),
    [
        (
            "ego",
            {
                "schema": "ego.2uq2.acceptance.v1",
                "status": "PASS",
                "formal_evidence": [],
            },
        ),
        (
            "ego",
            {
                "schema": "ego.2uq2.acceptance.v1",
                "status": "PASS",
                "hashes": [],
            },
        ),
        ("left", {"result": "PASS", "live_vins": []}),
        (
            "ego",
            {
                "schema": "wrong.schema",
                "status": "PASS",
                "formal_evidence": {"first_acquisition_ns": 99_000_000_000},
                "hashes": {"xu_library_sha256": "untrusted"},
            },
        ),
    ],
    ids=("formal-evidence-list", "hashes-list", "live-vins-list", "wrong-schema"),
)
def test_malformed_nested_acceptance_never_escapes_or_contributes_evidence(
    tmp_path, monkeypatch, device_id, payload
):
    session, _, _, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        behaviors={device_id: {"acceptance_payload": payload}},
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "worker_acceptance_invalid"
    assert report["first_error"]["device_id"] == device_id
    assert json.loads((session / "coordinator.json").read_text()) == report
    evidence = report["calibration_evidence_ids"][device_id]
    assert evidence["available"] is False
    if device_id == "ego":
        assert report["task_start_ego_frame_ns"] is None
        assert evidence["evidence_id"] is None


def test_cli_builds_current_session_root_worker_contract(tmp_path, monkeypatch):
    config = SimpleNamespace(
        left=SimpleNamespace(serial="left-serial", imu_path="/dev/left-imu"),
        right=SimpleNamespace(serial="right-serial", imu_path="/dev/right-imu"),
        ego=SimpleNamespace(video_device="/dev/video0", xu_library="/opt/libxu.so"),
        duration_s=12.5,
        output_root=tmp_path,
    )
    session = tmp_path / "one-session"
    monkeypatch.chdir(tmp_path)

    commands = coordinator._build_worker_commands(config, session)

    for device_id in ("left", "right"):
        command = commands[device_id]
        assert command[0] == coordinator.sys.executable
        assert command[0:3] == [
            coordinator.sys.executable,
            "-m",
            "three_device_slam.devices.d405_umi.worker",
        ]
        assert command[command.index("--session") + 1] == str(session)
        assert command[command.index("--barrier-dir") + 1] == str(session / "barrier")
        assert command[command.index("--device-id") + 1] == device_id
        assert "--no-ram-stage" in command
        assert "--no-preview" in command
        assert command[command.index("--duration") + 1] == "12.5"
    assert commands["ego"][0:3] == [
        coordinator.sys.executable,
        "-m",
        "three_device_slam.devices.two_uq2.worker",
    ]
    assert commands["ego"][commands["ego"].index("--device-id") + 1] == "ego"
    assert commands["ego"][commands["ego"].index("--duration") + 1] == "12.5"


def test_worker_commands_omit_duration_when_config_has_none(tmp_path):
    config = SimpleNamespace(
        left=SimpleNamespace(serial="left-serial", imu_path="/dev/left-imu"),
        right=SimpleNamespace(serial="right-serial", imu_path="/dev/right-imu"),
        ego=SimpleNamespace(video_device="/dev/video0", xu_library="/opt/libxu.so"),
        duration_s=None,
        output_root=tmp_path,
    )

    commands = coordinator._build_worker_commands(config, tmp_path / "session")

    assert all("--duration" not in command for command in commands.values())


def test_run_product_capture_returns_created_session_and_report(tmp_path, monkeypatch):
    config = SimpleNamespace(
        left=SimpleNamespace(serial="left-serial", imu_path="/dev/left-imu"),
        right=SimpleNamespace(serial="right-serial", imu_path="/dev/right-imu"),
        ego=SimpleNamespace(video_device="/dev/video0", xu_library="/opt/libxu.so"),
        duration_s=None,
        output_root=tmp_path,
    )
    monkeypatch.setattr(coordinator, "_new_session", lambda _root: tmp_path / "session")
    monkeypatch.setattr(
        coordinator,
        "run_coordinator",
        lambda session, commands, duration_s, **kwargs: {
            "session": str(session),
            "commands": commands,
            "duration_s": duration_s,
            "options": kwargs,
        },
    )

    result = coordinator.run_product_capture(config, lambda: False)

    assert result.session == tmp_path / "session"
    assert result.report["duration_s"] is None
    assert result.report["options"]["auto_start"] is True


def test_coordinator_module_help_runs_directly_from_repository_root():
    result = subprocess.run(
        [sys.executable, "-m", "three_device_slam.acquisition.coordinator", "--help"],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_git_provenance_is_rooted_at_repository(tmp_path, monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        stdout = "abc123\n" if command[-1] == "HEAD" else " M tracked.py\n"
        return subprocess.CompletedProcess(command, 0, stdout=stdout, stderr="")

    monkeypatch.setattr(coordinator.subprocess, "run", fake_run)
    monkeypatch.chdir(tmp_path)

    provenance = coordinator._git_provenance()

    assert provenance == {"hash": "abc123", "dirty": True}
    assert len(calls) == 2
    assert all(call_kwargs["cwd"] == coordinator.ROOT for _, call_kwargs in calls)


def test_session_subdirectory_creation_failure_writes_storage_report(
    tmp_path, monkeypatch
):
    session = tmp_path / "session"
    session.mkdir()
    (session / "left").write_text("not a directory")
    monkeypatch.setattr(
        coordinator,
        "_popen",
        lambda *_args, **_kwargs: pytest.fail("workers must not launch"),
    )

    report = coordinator.run_coordinator(
        session,
        {device: [device] for device in DEVICES},
        1.0,
        clock_ns=lambda: 123,
        auto_start=True,
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "storage_io"
    assert json.loads((session / "coordinator.json").read_text()) == report


def test_unwritable_session_path_still_returns_explicit_storage_failure(
    tmp_path, monkeypatch
):
    parent_file = tmp_path / "not-a-directory"
    parent_file.write_text("occupied")
    session = parent_file / "session"
    monkeypatch.setattr(
        coordinator,
        "_popen",
        lambda *_args, **_kwargs: pytest.fail("workers must not launch"),
    )

    report = coordinator.run_coordinator(
        session,
        {device: [device] for device in DEVICES},
        1.0,
        clock_ns=lambda: 456,
        auto_start=True,
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "storage_io"
    assert report["first_error"]["at_ns"] == 456
    assert not (session / "coordinator.json").exists()


def test_cli_output_root_creation_failure_returns_fail_exit(tmp_path, monkeypatch):
    output_root = tmp_path / "not-a-directory"
    output_root.write_text("occupied")
    monkeypatch.setattr(
        coordinator,
        "_popen",
        lambda *_args, **_kwargs: pytest.fail("workers must not launch"),
    )

    exit_code = coordinator.main(
        [
            "--left-serial",
            "left",
            "--left-imu",
            "/dev/left",
            "--right-serial",
            "right",
            "--right-imu",
            "/dev/right",
            "--ego-device",
            "/dev/video0",
            "--ego-xu-library",
            "/opt/libxu.so",
            "--duration",
            "1",
            "--output-root",
            str(output_root),
            "--auto-start",
        ]
    )

    assert exit_code == 2


@pytest.mark.parametrize(
    "extra_args",
    [
        ["--duration", "nan"],
        ["--duration", "0"],
        ["--left-serial", "same", "--right-serial", "same"],
        ["--left-imu", "/dev/same", "--right-imu", "/dev/same"],
        ["--left-serial", ""],
        ["--left-imu", ""],
    ],
)
def test_cli_rejects_invalid_duration_or_non_distinct_d405_inputs(extra_args):
    values = {
        "--left-serial": "left",
        "--left-imu": "/dev/left",
        "--right-serial": "right",
        "--right-imu": "/dev/right",
        "--ego-device": "/dev/video0",
        "--ego-xu-library": "/opt/libxu.so",
        "--duration": "1",
        "--output-root": "/tmp/output",
    }
    for option, value in zip(extra_args[::2], extra_args[1::2]):
        values[option] = value
    argv = [item for pair in values.items() for item in pair]

    with pytest.raises(SystemExit):
        coordinator.parse_args(argv)
