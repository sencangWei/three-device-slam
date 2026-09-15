#!/usr/bin/env python3
"""Launch and coordinate the Ego, left D405, and right D405 capture workers."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import re
import subprocess
import sys
import time
import uuid
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Callable

ROOT = Path(__file__).resolve().parents[2]

from three_device_slam.core.barrier import BarrierDirectory
from three_device_slam.core.joint_gate import GateState, JointWarmupGate
from three_device_slam.core.session_lifecycle import producer_claim


REQUIRED_DEVICES = ("ego", "left", "right")
POLL_SECONDS = 0.05
WARMUP_NS = 5_000_000_000
CONTINUOUS_HEALTHY_NS = 3_000_000_000
LAUNCH_TIMEOUT_NS = 30_000_000_000
START_GUARD_NS = 500_000_000
HEARTBEAT_STALE_NS = 500_000_000
FINALIZATION_GRACE_NS = 30_000_000_000
SCHEMA = "ego.three_device.coordinator.v1"

_popen = subprocess.Popen
_sleep = time.sleep
_input = input


def run_coordinator(
    session: Path,
    worker_commands: dict[str, list[str]],
    duration_s: float | None,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    *,
    auto_start: bool = True,
    stop_requested: Callable[[], bool] = lambda: False,
    expected_calibration_ids: Mapping[str, str] | None = None,
) -> dict:
    """Run the three capture workers and return their durable coordinator report."""
    _validate_calibration_expectations(expected_calibration_ids)
    try:
        with producer_claim(session, "coordinator"):
            return _run_coordinator_claimed(
                session,
                worker_commands,
                duration_s,
                clock_ns,
                auto_start=auto_start,
                stop_requested=stop_requested,
                expected_calibration_ids=expected_calibration_ids,
            )
    except OSError as exc:
        return _initial_storage_failure_report(
            Path(session),
            worker_commands,
            clock_ns(),
            type(exc).__name__,
            expected_calibration_ids,
        )


def _run_coordinator_claimed(
    session: Path,
    worker_commands: dict[str, list[str]],
    duration_s: float | None,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    *,
    auto_start: bool = True,
    stop_requested: Callable[[], bool] = lambda: False,
    expected_calibration_ids: Mapping[str, str] | None = None,
) -> dict:
    session = Path(session)
    _validate_run_inputs(worker_commands, duration_s)
    launch_ns = clock_ns()
    try:
        for directory in (*REQUIRED_DEVICES, "barrier"):
            (session / directory).mkdir(parents=True, exist_ok=True)
        barrier = BarrierDirectory(session)
    except OSError as exc:
        return _initial_storage_failure_report(
            session,
            worker_commands,
            launch_ns,
            type(exc).__name__,
            expected_calibration_ids,
        )
    gate = JointWarmupGate(
        REQUIRED_DEVICES,
        WARMUP_NS,
        CONTINUOUS_HEALTHY_NS,
        LAUNCH_TIMEOUT_NS,
    )
    transitions = [{"state": "COLD", "at_ns": launch_ns}]
    processes = {}
    worker_results = {
        device: {
            "exit_code": None,
            "early_exit": None,
            "observed_running_at_or_after_formal_end": False,
        }
        for device in REQUIRED_DEVICES
    }
    last_heartbeat_ns = {}
    barrier_failures = {}
    coordinator_errors = []
    first_error = None
    scheduled_start_ns = None
    operator_confirmed = bool(auto_start)
    post_confirmation_ns = None
    prompted = False
    prestart_terminal = False
    stop_boundary_ns = None
    operator_stop_published = False

    def add_transition(state: str, at_ns: int) -> None:
        if transitions[-1]["state"] != state:
            transitions.append({"state": state, "at_ns": at_ns})

    def latch_error(reason: str, at_ns: int, **details) -> None:
        nonlocal first_error
        error = {"reason": reason, "at_ns": at_ns, **details}
        if first_error is None:
            first_error = error

    def publish_stop(at_ns: int, reason: str, alive_devices) -> None:
        nonlocal stop_boundary_ns, operator_stop_published
        if stop_boundary_ns is not None:
            return
        stop_boundary_ns = at_ns
        operator_stop_published = reason == "operator_interrupt"
        for device in alive_devices:
            worker_results[device][
                "observed_running_at_or_after_formal_end"
            ] = True
        try:
            barrier.request_stop(at_ns, reason)
        except (OSError, ValueError) as exc:
            coordinator_errors.append(
                {
                    "reason": "barrier_io",
                    "at_ns": at_ns,
                    "detail": type(exc).__name__,
                }
            )
            latch_error("barrier_io", at_ns)

    def fail_stop(reason: str, at_ns: int, alive_devices, **details) -> None:
        latch_error(reason, at_ns, **details)
        publish_stop(at_ns, "worker_failure", alive_devices)

    try:
        for device in REQUIRED_DEVICES:
            processes[device] = _popen(worker_commands[device], shell=False)
    except (OSError, ValueError) as exc:
        now_ns = clock_ns()
        coordinator_errors.append(
            {"reason": "worker_launch", "at_ns": now_ns, "detail": type(exc).__name__}
        )
        latch_error("worker_launch", now_ns)
        prestart_terminal = True

    while processes and not prestart_terminal:
        now_ns = clock_ns()
        formal_end_ns = (
            stop_boundary_ns
            if stop_boundary_ns is not None
            else None
            if scheduled_start_ns is None or duration_s is None
            else scheduled_start_ns + int(duration_s * 1_000_000_000)
        )
        newly_exited = []
        alive_this_poll = set()
        for device, process in processes.items():
            exit_code = process.poll()
            if exit_code is None:
                alive_this_poll.add(device)
            if (
                exit_code is None
                and formal_end_ns is not None
                and now_ns >= formal_end_ns
            ):
                worker_results[device][
                    "observed_running_at_or_after_formal_end"
                ] = True
            if exit_code is not None and worker_results[device]["exit_code"] is None:
                newly_exited.append((device, exit_code))

        try:
            observed_failures = barrier.read_failures()
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            coordinator_errors.append(
                {"reason": "barrier_io", "at_ns": now_ns, "detail": type(exc).__name__}
            )
            observed_failures = {}
            if scheduled_start_ns is None:
                latch_error("barrier_io", now_ns)
                prestart_terminal = True
            else:
                fail_stop("barrier_io", now_ns, alive_this_poll)
        for device, failure in observed_failures.items():
            if device not in barrier_failures:
                barrier_failures[device] = failure
                if scheduled_start_ns is None:
                    latch_error(
                        failure["reason"], failure["at_ns"], device_id=device
                    )
                    prestart_terminal = True
                else:
                    fail_stop(
                        failure["reason"],
                        failure["at_ns"],
                        alive_this_poll,
                        device_id=device,
                    )

        for device, exit_code in newly_exited:
            proof = worker_results[device][
                "observed_running_at_or_after_formal_end"
            ]
            early = formal_end_ns is None or not proof
            worker_results[device] = {
                "exit_code": exit_code,
                "early_exit": early,
                "observed_running_at_or_after_formal_end": proof,
            }
            if device not in barrier_failures and (early or exit_code != 0):
                if (
                    scheduled_start_ns is not None
                    and stop_boundary_ns is None
                    and (formal_end_ns is None or now_ns < formal_end_ns)
                ):
                    fail_stop(
                        "worker_exit",
                        now_ns,
                        alive_this_poll,
                        device_id=device,
                        exit_code=exit_code,
                    )
                else:
                    latch_error(
                        "worker_exit", now_ns, device_id=device, exit_code=exit_code
                    )
            if scheduled_start_ns is None:
                prestart_terminal = True

        if scheduled_start_ns is not None:
            if stop_boundary_ns is None and stop_requested():
                publish_stop(now_ns, "operator_interrupt", alive_this_poll)
            if stop_boundary_ns is not None:
                formal_end_ns = stop_boundary_ns
            if formal_end_ns is None or now_ns < formal_end_ns:
                try:
                    recording_heartbeats = barrier.read_heartbeats()
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    coordinator_errors.append(
                        {
                            "reason": "barrier_io",
                            "at_ns": now_ns,
                            "detail": type(exc).__name__,
                        }
                    )
                    fail_stop("barrier_io", now_ns, alive_this_poll)
                    recording_heartbeats = {}
                for device, heartbeat in recording_heartbeats.items():
                    if heartbeat.at_ns == last_heartbeat_ns.get(device):
                        continue
                    try:
                        gate.observe(heartbeat)
                    except ValueError:
                        fail_stop(
                            "timestamp_regression",
                            now_ns,
                            alive_this_poll,
                            device_id=device,
                        )
                        if device not in barrier_failures:
                            if _safe_latch_failure(
                                barrier,
                                device,
                                "timestamp_regression",
                                now_ns,
                                coordinator_errors,
                            ):
                                barrier_failures[device] = {
                                    "device_id": device,
                                    "reason": "timestamp_regression",
                                    "at_ns": now_ns,
                                }
                        coordinator_errors.append(
                            {
                                "reason": "timestamp_regression",
                                "at_ns": now_ns,
                                "device_id": device,
                            }
                        )
                        continue
                    last_heartbeat_ns[device] = heartbeat.at_ns
                for device, heartbeat_ns in last_heartbeat_ns.items():
                    if (
                        now_ns - heartbeat_ns > HEARTBEAT_STALE_NS
                        and device not in barrier_failures
                    ):
                        fail_stop(
                            "worker_disconnect",
                            now_ns,
                            alive_this_poll,
                            device_id=device,
                        )
                        if _safe_latch_failure(
                            barrier,
                            device,
                            "worker_disconnect",
                            now_ns,
                            coordinator_errors,
                        ):
                            barrier_failures[device] = {
                                "device_id": device,
                                "reason": "worker_disconnect",
                                "at_ns": now_ns,
                            }
            if all(result["exit_code"] is not None for result in worker_results.values()):
                break
            if (
                formal_end_ns is not None
                and now_ns >= formal_end_ns + FINALIZATION_GRACE_NS
            ):
                timeout_reason = (
                    "worker_stop_timeout"
                    if stop_boundary_ns is not None
                    else "worker_finalize_timeout"
                )
                coordinator_errors.append(
                    {"reason": timeout_reason, "at_ns": now_ns}
                )
                latch_error(timeout_reason, now_ns)
                for device, process in processes.items():
                    if worker_results[device]["exit_code"] is not None:
                        continue
                    exit_code = _stop_process(process)
                    worker_results[device] = {
                        "exit_code": exit_code,
                        "early_exit": False,
                        "observed_running_at_or_after_formal_end": True,
                    }
                break
            _sleep(POLL_SECONDS)
            continue

        if prestart_terminal:
            break

        if now_ns - launch_ns >= LAUNCH_TIMEOUT_NS:
            latch_error("joint_warmup_timeout", now_ns)
            add_transition("BLOCKED", now_ns)
            prestart_terminal = True
            break

        try:
            heartbeats = barrier.read_heartbeats()
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            coordinator_errors.append(
                {"reason": "barrier_io", "at_ns": now_ns, "detail": type(exc).__name__}
            )
            latch_error("barrier_io", now_ns)
            prestart_terminal = True
            break

        for device in REQUIRED_DEVICES:
            heartbeat = heartbeats.get(device)
            if heartbeat is None or heartbeat.at_ns == last_heartbeat_ns.get(device):
                continue
            if (
                post_confirmation_ns is not None
                and heartbeat.at_ns < post_confirmation_ns
            ):
                continue
            try:
                gate.observe(heartbeat)
            except ValueError as exc:
                reason = (
                    "timestamp_regression"
                    if "regression" in str(exc)
                    else "invalid_heartbeat"
                )
                coordinator_errors.append(
                    {"reason": reason, "at_ns": now_ns, "device_id": device}
                )
                latch_error(reason, now_ns, device_id=device)
                _safe_latch_failure(
                    barrier, device, reason, now_ns, coordinator_errors
                )
                prestart_terminal = True
                break
            last_heartbeat_ns[device] = heartbeat.at_ns
            add_transition(gate.state.name, now_ns)
        if prestart_terminal:
            break

        for device, heartbeat_ns in last_heartbeat_ns.items():
            if now_ns - heartbeat_ns > HEARTBEAT_STALE_NS:
                latch_error("worker_disconnect", now_ns, device_id=device)
                _safe_latch_failure(
                    barrier,
                    device,
                    "worker_disconnect",
                    now_ns,
                    coordinator_errors,
                )
                prestart_terminal = True
                break
        if prestart_terminal:
            break

        gate.tick(now_ns)
        add_transition(gate.state.name, now_ns)
        if gate.state is GateState.BLOCKED:
            latch_error("joint_warmup_timeout", now_ns)
            prestart_terminal = True
            break

        if gate.state is GateState.JOINT_READY and not operator_confirmed:
            if not prompted:
                prompted = True
                try:
                    _input("All three devices are healthy. Press Enter to start recording: ")
                except (EOFError, KeyboardInterrupt):
                    cancelled_ns = clock_ns()
                    latch_error("operator_cancelled", cancelled_ns)
                    add_transition("BLOCKED", cancelled_ns)
                    prestart_terminal = True
                    break
                operator_confirmed = True
                post_confirmation_ns = clock_ns()
            try:
                refresh_error = _refresh_readiness(
                    barrier, gate, last_heartbeat_ns, clock_ns()
                )
            except (OSError, json.JSONDecodeError):
                refresh_error = (None, "barrier_io")
            refreshed_now_ns = clock_ns()
            if refreshed_now_ns - launch_ns >= LAUNCH_TIMEOUT_NS:
                latch_error("joint_warmup_timeout", refreshed_now_ns)
                add_transition("BLOCKED", refreshed_now_ns)
                prestart_terminal = True
                break
            if refresh_error is not None:
                device, reason = refresh_error
                coordinator_errors.append(
                    {
                        "reason": reason,
                        "at_ns": refreshed_now_ns,
                        "device_id": device,
                    }
                )
                latch_error(reason, refreshed_now_ns, device_id=device)
                if device is not None:
                    _safe_latch_failure(
                        barrier,
                        device,
                        reason,
                        refreshed_now_ns,
                        coordinator_errors,
                    )
                prestart_terminal = True
                break
            add_transition(gate.state.name, refreshed_now_ns)
            stale_device = next(
                (
                    device
                    for device, heartbeat_ns in last_heartbeat_ns.items()
                    if refreshed_now_ns - heartbeat_ns > HEARTBEAT_STALE_NS
                ),
                None,
            )
            if stale_device is not None:
                latch_error(
                    "worker_disconnect", refreshed_now_ns, device_id=stale_device
                )
                _safe_latch_failure(
                    barrier,
                    stale_device,
                    "worker_disconnect",
                    refreshed_now_ns,
                    coordinator_errors,
                )
                prestart_terminal = True
                break
            gate = JointWarmupGate(
                REQUIRED_DEVICES,
                0,
                CONTINUOUS_HEALTHY_NS,
                LAUNCH_TIMEOUT_NS,
            )

        if gate.state is GateState.JOINT_READY and operator_confirmed:
            schedule_now_ns = clock_ns()
            if schedule_now_ns - launch_ns >= LAUNCH_TIMEOUT_NS:
                latch_error("joint_warmup_timeout", schedule_now_ns)
                add_transition("BLOCKED", schedule_now_ns)
                prestart_terminal = True
                break
            candidate_start_ns = schedule_now_ns + START_GUARD_NS
            try:
                barrier.schedule_start(candidate_start_ns)
                gate.request_recording(candidate_start_ns)
            except (OSError, FileExistsError, RuntimeError, ValueError) as exc:
                coordinator_errors.append(
                    {"reason": "barrier_io", "at_ns": schedule_now_ns, "detail": type(exc).__name__}
                )
                latch_error("barrier_io", schedule_now_ns)
                prestart_terminal = True
                break
            scheduled_start_ns = candidate_start_ns
            add_transition("RECORDING", schedule_now_ns)

        _sleep(POLL_SECONDS)

    if scheduled_start_ns is not None:
        add_transition("OFFLINE_QC", clock_ns())
    if prestart_terminal:
        _terminate_prestart(processes, worker_results)

    acceptances, acceptance_errors = _load_acceptances(session)
    acceptance_statuses = {}
    trusted_acceptances = {}
    coordinator_errors.extend(acceptance_errors)
    if acceptance_errors and first_error is None:
        first_error = acceptance_errors[0]
    for device in REQUIRED_DEVICES:
        if device not in acceptances:
            error = {
                "reason": "worker_acceptance_missing",
                "at_ns": clock_ns(),
                "device_id": device,
            }
            coordinator_errors.append(error)
            if first_error is None:
                first_error = error
            continue
        status = _validated_acceptance_status(device, acceptances[device])
        if status is None:
            error = {
                "reason": "worker_acceptance_invalid",
                "at_ns": clock_ns(),
                "device_id": device,
            }
            coordinator_errors.append(error)
            if first_error is None:
                first_error = error
            continue
        acceptance_statuses[device] = status
        trusted_acceptances[device] = acceptances[device]
    for device, status in acceptance_statuses.items():
        if status in {"FAIL", "BLOCKED"} and first_error is None:
            acceptance = acceptances[device]
            first_error = {
                "reason": acceptance.get("reason", "worker_acceptance"),
                "at_ns": clock_ns(),
                "device_id": device,
                "worker_status": status,
            }
    if (
        isinstance(first_error, Mapping)
        and first_error.get("reason") == "worker_exit"
        and first_error.get("exit_code") == 3
        and not barrier_failures
        and not coordinator_errors
        and set(acceptance_statuses) == set(REQUIRED_DEVICES)
        and all(
            (
                worker_results[device]["exit_code"] == 0
                and acceptance_statuses[device] == "PASS"
            )
            or (
                worker_results[device]["exit_code"] == 3
                and acceptance_statuses[device] == "BLOCKED"
            )
            for device in REQUIRED_DEVICES
        )
    ):
        device = first_error.get("device_id")
        acceptance = trusted_acceptances.get(device, {})
        first_error = {
            "reason": acceptance.get("reason", "worker_blocked"),
            "at_ns": first_error["at_ns"],
            "device_id": device,
            "worker_status": "BLOCKED",
        }

    try:
        barrier_failures.update(
            {
                device: failure
                for device, failure in barrier.read_failures().items()
                if device not in barrier_failures
            }
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        coordinator_errors.append(
            {
                "reason": "barrier_io",
                "at_ns": clock_ns(),
                "detail": type(exc).__name__,
            }
        )

    status, reason = _final_status(
        scheduled_start_ns,
        first_error,
        trusted_acceptances,
        worker_results,
        barrier_failures,
        coordinator_errors,
    )
    if status == "PASS" and operator_stop_published:
        reason = "operator_stop"
    task_start_ns = _task_start_ego_frame_ns(
        trusted_acceptances.get("ego"), scheduled_start_ns
    )
    report = {
        "schema": SCHEMA,
        "status": status,
        "reason": reason,
        "worker_commands": {
            device: _sanitize_command(worker_commands[device])
            for device in REQUIRED_DEVICES
        },
        "workers": worker_results,
        "scheduled_start_ns": scheduled_start_ns,
        "task_start_ego_frame_ns": task_start_ns,
        "transitions": transitions,
        "host": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "git": _git_provenance(),
        "calibration_evidence_ids": _calibration_evidence_ids(trusted_acceptances),
        "worker_acceptance": acceptances,
        "barrier_failures": barrier_failures,
        "coordinator_errors": coordinator_errors,
        "first_error": first_error,
    }
    calibration_expectations = _calibration_expectations(
        expected_calibration_ids, trusted_acceptances
    )
    if calibration_expectations is not None:
        report["calibration_expectations"] = calibration_expectations
    try:
        _write_atomic_json(session / "coordinator.json", report)
    except OSError as exc:
        storage_error = {
            "reason": "storage_io",
            "at_ns": clock_ns(),
            "detail": type(exc).__name__,
        }
        report["status"] = "FAIL"
        report["reason"] = "storage_io"
        report["coordinator_errors"].append(storage_error)
        if report["first_error"] is None:
            report["first_error"] = storage_error
        try:
            _write_atomic_json(session / "coordinator.json", report)
        except OSError:
            pass
    return report


def _safe_latch_failure(
    barrier, device_id: str, reason: str, at_ns: int, coordinator_errors: list
) -> bool:
    try:
        barrier.latch_failure(device_id, reason, at_ns)
    except OSError as exc:
        coordinator_errors.append(
            {
                "reason": "barrier_io",
                "at_ns": at_ns,
                "device_id": device_id,
                "detail": type(exc).__name__,
            }
        )
        return False
    return True


def _initial_storage_failure_report(
    session: Path,
    worker_commands: dict[str, list[str]],
    at_ns: int,
    detail: str,
    expected_calibration_ids: Mapping[str, str] | None = None,
) -> dict:
    error = {"reason": "storage_io", "at_ns": at_ns, "detail": detail}
    report = {
        "schema": SCHEMA,
        "status": "FAIL",
        "reason": "storage_io",
        "worker_commands": {
            device: _sanitize_command(worker_commands[device])
            for device in REQUIRED_DEVICES
        },
        "workers": {
            device: {
                "exit_code": None,
                "early_exit": None,
                "observed_running_at_or_after_formal_end": False,
            }
            for device in REQUIRED_DEVICES
        },
        "scheduled_start_ns": None,
        "task_start_ego_frame_ns": None,
        "transitions": [{"state": "FAIL", "at_ns": at_ns}],
        "host": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "git": _git_provenance(),
        "calibration_evidence_ids": _calibration_evidence_ids({}),
        "worker_acceptance": {},
        "barrier_failures": {},
        "coordinator_errors": [error],
        "first_error": error,
    }
    calibration_expectations = _calibration_expectations(
        expected_calibration_ids, {}
    )
    if calibration_expectations is not None:
        report["calibration_expectations"] = calibration_expectations
    try:
        _write_atomic_json(session / "coordinator.json", report)
    except OSError:
        pass
    return report


def _refresh_readiness(barrier, gate, last_heartbeat_ns, now_ns):
    for device, heartbeat in barrier.read_heartbeats().items():
        if heartbeat.at_ns != last_heartbeat_ns.get(device):
            try:
                gate.observe(heartbeat)
            except ValueError as exc:
                reason = (
                    "timestamp_regression"
                    if "regression" in str(exc)
                    else "invalid_heartbeat"
                )
                return device, reason
            last_heartbeat_ns[device] = heartbeat.at_ns
    gate.tick(now_ns)
    return None


def _terminate_prestart(processes, worker_results):
    for device, process in processes.items():
        exit_code = _stop_process(process)
        worker_results[device] = {
            "exit_code": exit_code,
            "early_exit": True,
            "observed_running_at_or_after_formal_end": False,
        }


def _stop_process(process):
    try:
        running = process.poll() is None
    except OSError:
        running = True
    if running:
        try:
            process.terminate()
        except OSError:
            pass
    try:
        return process.wait(timeout=5)
    except (subprocess.TimeoutExpired, OSError):
        try:
            process.kill()
        except OSError:
            return None
        try:
            return process.wait(timeout=5)
        except (subprocess.TimeoutExpired, OSError):
            return None


def _final_status(
    scheduled_start_ns,
    first_error,
    acceptances,
    worker_results,
    barrier_failures,
    coordinator_errors,
):
    acceptance_statuses = {
        device: _validated_acceptance_status(device, acceptance)
        for device, acceptance in acceptances.items()
    }
    has_failed_acceptance = "FAIL" in acceptance_statuses.values()
    has_blocked_acceptance = "BLOCKED" in acceptance_statuses.values()
    first_error_is_blocked_candidate = first_error is not None and (
        first_error.get("reason") in {"joint_warmup_timeout", "operator_cancelled"}
        or first_error.get("worker_status") == "BLOCKED"
        or (
            first_error.get("reason") == "worker_exit"
            and first_error.get("exit_code") == 3
        )
    )
    if (
        first_error is not None
        and scheduled_start_ns is None
        and first_error["reason"] in {"joint_warmup_timeout", "operator_cancelled"}
        and not has_failed_acceptance
        and not barrier_failures
        and all(
            error.get("reason") == "worker_acceptance_missing"
            for error in coordinator_errors
        )
    ):
        return "BLOCKED", first_error["reason"]
    timeout_error = next(
        (
            error
            for error in coordinator_errors
            if error.get("reason")
            in {"worker_stop_timeout", "worker_finalize_timeout"}
        ),
        None,
    )
    if timeout_error is not None:
        return "FAIL", timeout_error["reason"]
    if coordinator_errors:
        reason = (
            coordinator_errors[0]["reason"]
            if first_error is None or first_error_is_blocked_candidate
            else first_error["reason"]
        )
        return "FAIL", reason
    if barrier_failures:
        failure = next(iter(barrier_failures.values()))
        reason = (
            failure["reason"]
            if first_error is None or first_error_is_blocked_candidate
            else first_error["reason"]
        )
        return "FAIL", reason
    if first_error is not None:
        reason = first_error["reason"]
        if (
            set(acceptance_statuses) == set(REQUIRED_DEVICES)
            and has_blocked_acceptance
            and not has_failed_acceptance
            and first_error.get("worker_status") == "BLOCKED"
            and first_error.get("device_id") in acceptance_statuses
            and acceptance_statuses[first_error["device_id"]] == "BLOCKED"
            and worker_results[first_error["device_id"]]["exit_code"] == 3
            and reason
            == acceptances[first_error["device_id"]].get("reason", "worker_blocked")
            and all(
                result["exit_code"]
                == (3 if acceptance_statuses[device] == "BLOCKED" else 0)
                and not result["early_exit"]
                for device, result in worker_results.items()
            )
        ):
            blocked = next(
                acceptance
                for device, acceptance in acceptances.items()
                if acceptance_statuses[device] == "BLOCKED"
            )
            return "BLOCKED", blocked.get("reason", reason)
        return "FAIL", reason
    if any(
        result["exit_code"] != 0 or result["early_exit"]
        for result in worker_results.values()
    ):
        return "FAIL", "worker_exit"
    return "PASS", "acquisition_timing"


def _load_acceptances(session: Path):
    reports = {}
    errors = []
    for device in REQUIRED_DEVICES:
        path = session / device / "acceptance.json"
        if not path.exists():
            continue
        try:
            with path.open(encoding="utf-8") as file_handle:
                reports[device] = json.load(file_handle)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            errors.append(
                {
                    "reason": "worker_acceptance_io",
                    "at_ns": time.monotonic_ns(),
                    "device_id": device,
                    "detail": type(exc).__name__,
                }
            )
    return reports, errors


def _validated_acceptance_status(device_id: str, acceptance) -> str | None:
    if not isinstance(acceptance, Mapping):
        return None
    reason = acceptance.get("reason")
    if reason is not None and (
        not isinstance(reason, str) or not reason.strip()
    ):
        return None
    if device_id == "ego":
        if acceptance.get("schema") not in {
            "ego.2uq2.acceptance.v1",
            "ego.d435i.acceptance.v1",
        }:
            return None
        if any(
            key in acceptance and not isinstance(acceptance[key], Mapping)
            for key in ("formal_evidence", "hashes")
        ):
            return None
        formal_evidence = acceptance.get("formal_evidence", {})
        first_acquisition_ns = formal_evidence.get("first_acquisition_ns")
        if first_acquisition_ns is not None and (
            isinstance(first_acquisition_ns, bool)
            or not isinstance(first_acquisition_ns, int)
            or first_acquisition_ns < 0
        ):
            return None
        hashes = acceptance.get("hashes", {})
        xu_library_sha256 = hashes.get("xu_library_sha256")
        if xu_library_sha256 is not None and (
            not isinstance(xu_library_sha256, str)
            or re.fullmatch(r"[0-9a-fA-F]{64}", xu_library_sha256) is None
        ):
            return None
        calibration_id = acceptance.get("calibration_id")
        if calibration_id is not None and (
            not isinstance(calibration_id, str) or not calibration_id.strip()
        ):
            return None
        status = acceptance.get("status")
    else:
        if "live_vins" in acceptance and not isinstance(
            acceptance["live_vins"], Mapping
        ):
            return None
        live_vins = acceptance.get("live_vins", {})
        calibration_id = live_vins.get("imu_calibration")
        if calibration_id is not None and (
            not isinstance(calibration_id, str) or not calibration_id.strip()
        ):
            return None
        status = acceptance.get("result")
    if not isinstance(status, str) or status not in {"PASS", "FAIL", "BLOCKED"}:
        return None
    if status == "BLOCKED" and reason is None:
        return None
    return status


def _task_start_ego_frame_ns(acceptance, scheduled_start_ns):
    if not isinstance(acceptance, Mapping) or scheduled_start_ns is None:
        return None
    if acceptance.get("status") != "PASS":
        return None
    formal_evidence = acceptance.get("formal_evidence")
    if not isinstance(formal_evidence, Mapping):
        return None
    candidate = formal_evidence.get("first_acquisition_ns")
    if isinstance(candidate, bool) or not isinstance(candidate, int):
        return None
    return candidate if candidate >= scheduled_start_ns else None


def _calibration_evidence_ids(acceptances, devices=REQUIRED_DEVICES):
    result = {}
    for device in devices:
        acceptance = acceptances.get(device, {})
        if not isinstance(acceptance, Mapping):
            acceptance = {}
        if device == "ego":
            calibration_id = acceptance.get("calibration_id")
            result[device] = {
                "calibration_id": calibration_id,
                "evidence_id": None,
                "available": calibration_id is not None,
            }
        else:
            live_vins = acceptance.get("live_vins")
            calibration_id = (
                live_vins.get("imu_calibration")
                if isinstance(live_vins, Mapping)
                else None
            )
            result[device] = {
                "calibration_id": calibration_id,
                "evidence_id": None,
                "available": calibration_id is not None,
            }
    return result


def _validate_calibration_expectations(
    expected_calibration_ids, devices=REQUIRED_DEVICES
) -> None:
    if expected_calibration_ids is None:
        return
    if not isinstance(expected_calibration_ids, Mapping) or set(
        expected_calibration_ids
    ) != set(devices):
        raise ValueError(
            "expected calibration IDs must contain exactly "
            + ", ".join(devices)
        )
    if any(
        not isinstance(value, str) or not value.strip()
        for value in expected_calibration_ids.values()
    ):
        raise ValueError("expected calibration IDs must be non-empty strings")


def _calibration_expectations(
    expected_calibration_ids, acceptances, devices=REQUIRED_DEVICES
):
    if expected_calibration_ids is None:
        return None
    result = {}
    for device in devices:
        acceptance = acceptances.get(device)
        observed = None
        if isinstance(acceptance, Mapping):
            if device == "ego":
                observed = acceptance.get("calibration_id")
            else:
                live_vins = acceptance.get("live_vins")
                if isinstance(live_vins, Mapping):
                    observed = live_vins.get("imu_calibration")
        expected = expected_calibration_ids[device]
        if observed is None:
            result[device] = {
                "expected": expected,
                "observed": None,
                "status": "BLOCKED",
                "reason": "evidence_unavailable",
            }
        elif observed != expected:
            result[device] = {
                "expected": expected,
                "observed": observed,
                "status": "FAIL",
                "reason": "mismatch",
            }
        else:
            result[device] = {
                "expected": expected,
                "observed": observed,
                "status": "PASS",
            }
    return result


def _sanitize_command(command):
    sanitized = []
    redact_next = False
    for argument in command:
        value = str(argument)
        if redact_next:
            sanitized.append("<redacted>")
            redact_next = False
            continue
        option = value.lower().lstrip("-")
        if "=" in option:
            name, _ = option.split("=", 1)
            if _secret_option(name):
                sanitized.append(value.split("=", 1)[0] + "=<redacted>")
                continue
        sanitized.append(value)
        if value.startswith("--") and _secret_option(option):
            redact_next = True
    return sanitized


def _secret_option(option):
    normalized = option.replace("_", "-")
    return any(
        marker in normalized
        for marker in ("password", "passwd", "token", "secret", "api-key", "credential")
    )


def _git_provenance():
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )
    except (OSError, subprocess.CalledProcessError):
        commit, dirty = None, None
    return {"hash": commit, "dirty": dirty}


def _write_atomic_json(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary_path.open("x", encoding="utf-8") as file_handle:
            json.dump(payload, file_handle, ensure_ascii=False, indent=2)
            file_handle.write("\n")
            file_handle.flush()
            os.fsync(file_handle.fileno())
        temporary_path.replace(path)
        descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def _validate_run_inputs(worker_commands, duration_s):
    if set(worker_commands) != set(REQUIRED_DEVICES):
        raise ValueError("worker_commands must contain exactly ego, left, and right")
    if duration_s is not None and (
        not math.isfinite(duration_s) or duration_s <= 0
    ):
        raise ValueError("duration_s must be positive and finite")
    if any(not isinstance(command, list) or not command for command in worker_commands.values()):
        raise ValueError("each worker command must be a non-empty list")


def _nonempty(value):
    if not value.strip():
        raise argparse.ArgumentTypeError("value must be non-empty")
    return value


def _positive_duration(value):
    try:
        duration = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("duration must be a number") from exc
    if not math.isfinite(duration) or duration <= 0:
        raise argparse.ArgumentTypeError("duration must be positive and finite")
    return duration


def build_parser():
    parser = argparse.ArgumentParser(description="Coordinate three-device acquisition")
    parser.add_argument("--left-serial", required=True, type=_nonempty)
    parser.add_argument("--left-imu", required=True, type=_nonempty)
    parser.add_argument("--right-serial", required=True, type=_nonempty)
    parser.add_argument("--right-imu", required=True, type=_nonempty)
    parser.add_argument("--ego-device", required=True, type=_nonempty)
    parser.add_argument("--ego-xu-library", required=True, type=_nonempty)
    parser.add_argument("--duration", required=True, type=_positive_duration)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--auto-start", action="store_true")
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.left_serial == args.right_serial:
        parser.error("left and right D405 serials must be distinct")
    if args.left_imu == args.right_imu:
        parser.error("left and right IMU paths must be distinct")
    return args


def _build_worker_commands(config, session: Path):
    session_text = str(session)
    barrier_text = str(session / "barrier")
    commands = {}
    for device, serial, imu_path in (
        ("left", config.left.serial, config.left.imu_path),
        ("right", config.right.serial, config.right.imu_path),
    ):
        commands[device] = [
            sys.executable,
            "-m",
            "three_device_slam.devices.d405_umi.worker",
            "--serial",
            serial,
            "--imu-port",
            imu_path,
            "--imu-protocol",
            getattr(getattr(config, device), "imu_protocol", "auto"),
            "--device-id",
            device,
            "--session",
            session_text,
            "--barrier-dir",
            barrier_text,
            "--no-ram-stage",
            "--no-preview",
        ]
    ego_type = getattr(config.ego, "type", "2uq2")
    if ego_type == "d435i":
        commands["ego"] = [
            sys.executable,
            "-m",
            "three_device_slam.devices.d435i_ego.worker",
            "--serial",
            config.ego.serial,
            "--device-id",
            "ego",
            "--session",
            session_text,
            "--barrier-dir",
            barrier_text,
            "--calibration-id",
            config.ego.calibration_id,
        ]
    elif ego_type == "2uq2":
        commands["ego"] = [
            sys.executable,
            "-m",
            "three_device_slam.devices.two_uq2.worker",
            "--device",
            config.ego.video_device,
            "--xu-library",
            config.ego.xu_library,
            "--device-id",
            "ego",
            "--session",
            session_text,
            "--barrier-dir",
            barrier_text,
        ]
    else:
        raise ValueError("unsupported ego type")
    if config.duration_s is not None:
        duration_arguments = ["--duration", str(config.duration_s)]
        for command in commands.values():
            command.extend(duration_arguments)
    return commands


def _capture_devices(config) -> tuple[str, ...]:
    """Devices present for this product configuration."""
    devices = ["ego"]
    devices.extend(
        role for role in ("left", "right") if getattr(config, role, None) is not None
    )
    return tuple(devices)


def _capture_topology(config) -> str:
    return "single_umi" if len(_capture_devices(config)) == 2 else "dual_umi"


def _build_rsusb_owner_command(config, session: Path) -> list[str]:
    if getattr(config.ego, "type", "2uq2") != "d435i":
        raise ValueError("RSUSB owner requires d435i Ego")
    if config.duration_s is None:
        raise ValueError("RSUSB owner requires a finite duration")
    umi_configs = tuple(
        (role, getattr(config, role, None))
        for role in ("left", "right")
        if getattr(config, role, None) is not None
    )
    if not umi_configs:
        raise ValueError("RSUSB owner requires at least one UMI")
    protocols = tuple(
        getattr(umi, "imu_protocol", "auto") for _, umi in umi_configs
    )
    if "auto" in protocols:
        raise ValueError("RSUSB owner requires explicit UMI protocols")
    command = [
        sys.executable,
        "-m",
        "three_device_slam.acquisition.rsusb_pair",
        "--session",
        str(session),
        "--ego-serial",
        config.ego.serial,
        "--ego-calibration-id",
        config.ego.calibration_id,
        "--umi-device-id",
        umi_configs[0][0],
        "--umi-serial",
        umi_configs[0][1].serial,
        "--umi-port",
        umi_configs[0][1].imu_path,
        "--umi-calibration-id",
        umi_configs[0][1].calibration_id,
        "--umi-protocol",
        getattr(umi_configs[0][1], "imu_protocol", "auto"),
    ]
    if len(umi_configs) == 2:
        command.extend(
            [
                "--right-serial",
                config.right.serial,
                "--right-port",
                config.right.imu_path,
                "--right-calibration-id",
                config.right.calibration_id,
                "--right-protocol",
                getattr(config.right, "imu_protocol", "auto"),
            ]
        )
    command.extend(["--duration", str(config.duration_s)])
    return command


def _run_rsusb_owner_capture(config, session: Path, stop_requested) -> dict:
    """Run D435i and one or both D405 units under one RealSense lifecycle owner."""
    from three_device_slam.acquisition import rsusb_pair

    devices = _capture_devices(config)
    command = _build_rsusb_owner_command(config, session)
    args = rsusb_pair.parse_args(command[3:])
    expected = {"ego": config.ego.calibration_id}
    expected.update(
        {
            role: getattr(config, role).calibration_id
            for role in ("left", "right")
            if getattr(config, role, None) is not None
        }
    )
    try:
        group = rsusb_pair.run_pair(args, stop_requested=stop_requested)
    except (rsusb_pair.RequiredHardwareUnavailable, rsusb_pair.CaptureCancelled) as exc:
        report = _single_owner_terminal_report(
            session,
            command,
            status="BLOCKED",
            reason=(
                "operator_cancelled"
                if isinstance(exc, rsusb_pair.CaptureCancelled)
                else "required_hardware_unavailable"
            ),
            expected_calibration_ids=expected,
            devices=devices,
            topology=_capture_topology(config),
        )
        _write_atomic_json(session / "coordinator.json", report)
        return report
    acceptances, acceptance_errors = _load_acceptances(session)
    report = _single_owner_coordinator_report(
        session,
        command,
        group,
        acceptances,
        acceptance_errors,
        expected,
        devices=devices,
        topology=_capture_topology(config),
    )
    _write_atomic_json(session / "coordinator.json", report)
    return report


def _single_owner_terminal_report(
    session: Path,
    command: list[str],
    *,
    status: str,
    reason: str,
    expected_calibration_ids: Mapping[str, str],
    devices: tuple[str, ...] = REQUIRED_DEVICES,
    topology: str = "dual_umi",
) -> dict:
    session.mkdir(parents=True, exist_ok=True)
    now_ns = time.monotonic_ns()
    error = {"reason": reason, "at_ns": now_ns}
    return {
        "schema": SCHEMA,
        "status": status,
        "reason": reason,
        "owner_model": "single_process_single_thread_realsense_lifecycle",
        "capture_topology": topology,
        "owner_command": _sanitize_command(command),
        "worker_commands": {
            device: _sanitize_command(command) for device in devices
        },
        "workers": {
            device: {
                "exit_code": 3 if status == "BLOCKED" else 2,
                "early_exit": False,
                "observed_running_at_or_after_formal_end": False,
            }
            for device in devices
        },
        "scheduled_start_ns": None,
        "task_start_ego_frame_ns": None,
        "transitions": [{"state": "COLD", "at_ns": now_ns}],
        "host": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "git": _git_provenance(),
        "calibration_evidence_ids": _calibration_evidence_ids({}, devices),
        "calibration_expectations": {
            device: {
                "expected": calibration_id,
                "observed": None,
                "status": "BLOCKED",
                "reason": "evidence_unavailable",
            }
            for device, calibration_id in expected_calibration_ids.items()
        },
        "worker_acceptance": {},
        "barrier_failures": {},
        "coordinator_errors": [],
        "first_error": error,
    }


def _single_owner_coordinator_report(
    session: Path,
    command: list[str],
    group: dict,
    acceptances: dict,
    acceptance_errors: list,
    expected_calibration_ids: Mapping[str, str],
    devices: tuple[str, ...] = REQUIRED_DEVICES,
    topology: str = "dual_umi",
) -> dict:
    start_ns = group.get("formal_start_ns")
    end_ns = group.get("formal_end_ns")
    warmup_started_ns = group.get("warmup_started_ns")
    joint_ready_ns = group.get("joint_ready_ns")
    statuses = {
        device: _validated_acceptance_status(device, acceptances.get(device))
        for device in devices
    }
    complete = (
        group.get("status") == "PASS"
        and set(acceptances) == set(devices)
        and all(status == "PASS" for status in statuses.values())
        and not acceptance_errors
        and isinstance(start_ns, int)
        and isinstance(end_ns, int)
        and end_ns > start_ns
        and isinstance(warmup_started_ns, int)
        and isinstance(joint_ready_ns, int)
        and warmup_started_ns <= joint_ready_ns < start_ns
    )
    status = "PASS" if complete else "FAIL"
    reason = "acquisition_timing" if complete else "single_owner_acceptance"
    first_error = None if complete else {"reason": reason, "at_ns": time.monotonic_ns()}
    task_start_ns = _task_start_ego_frame_ns(acceptances.get("ego"), start_ns)
    transitions = []
    if isinstance(warmup_started_ns, int):
        transitions.append({"state": "COLD", "at_ns": warmup_started_ns})
    if isinstance(joint_ready_ns, int):
        transitions.append({"state": "JOINT_READY", "at_ns": joint_ready_ns})
    if isinstance(start_ns, int):
        transitions.append({"state": "RECORDING", "at_ns": start_ns})
    if isinstance(end_ns, int):
        transitions.append({"state": "COMPLETE", "at_ns": end_ns})
    report = {
        "schema": SCHEMA,
        "status": status,
        "reason": reason,
        "owner_model": "single_process_single_thread_realsense_lifecycle",
        "capture_topology": topology,
        "owner_command": _sanitize_command(command),
        "owner_acceptance": group,
        "worker_commands": {
            device: _sanitize_command(command) for device in devices
        },
        "workers": {
            device: {
                "exit_code": 0 if statuses[device] == "PASS" else 2,
                "early_exit": False,
                "observed_running_at_or_after_formal_end": isinstance(end_ns, int),
            }
            for device in devices
        },
        "scheduled_start_ns": start_ns,
        "task_start_ego_frame_ns": task_start_ns,
        "transitions": transitions,
        "host": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "git": _git_provenance(),
        "calibration_evidence_ids": _calibration_evidence_ids(acceptances, devices),
        "worker_acceptance": acceptances,
        "barrier_failures": {},
        "coordinator_errors": acceptance_errors,
        "first_error": first_error,
    }
    report["calibration_expectations"] = _calibration_expectations(
        expected_calibration_ids, acceptances, devices
    )
    return report


@dataclass(frozen=True)
class CaptureResult:
    session: Path
    report: dict


def run_product_capture(config, stop_requested) -> CaptureResult:
    session = _new_session(Path(config.output_root))
    if getattr(config.ego, "type", "2uq2") == "d435i":
        report = _run_rsusb_owner_capture(config, session, stop_requested)
        return CaptureResult(session=session, report=report)
    commands = _build_worker_commands(config, session)
    expected_calibration_ids = {
        "ego": config.ego.calibration_id,
        "left": config.left.calibration_id,
        "right": config.right.calibration_id,
    }
    report = run_coordinator(
        session,
        commands,
        config.duration_s,
        auto_start=True,
        stop_requested=stop_requested,
        expected_calibration_ids=expected_calibration_ids,
    )
    return CaptureResult(session=session, report=report)


def _new_session(output_root: Path):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return output_root / f"three_device_{stamp}_{uuid.uuid4().hex[:8]}"


def main(argv=None):
    args = parse_args(argv)
    session = _new_session(args.output_root)
    config = SimpleNamespace(
        left=SimpleNamespace(serial=args.left_serial, imu_path=args.left_imu),
        right=SimpleNamespace(serial=args.right_serial, imu_path=args.right_imu),
        ego=SimpleNamespace(
            video_device=args.ego_device,
            xu_library=args.ego_xu_library,
        ),
        duration_s=args.duration,
    )
    report = run_coordinator(
        session,
        _build_worker_commands(config, session),
        args.duration,
        auto_start=args.auto_start,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return {"PASS": 0, "FAIL": 2, "BLOCKED": 3}[report["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
