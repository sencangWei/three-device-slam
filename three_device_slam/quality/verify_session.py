#!/usr/bin/env python3
"""Verify one completed three-device acquisition session without opening devices."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import re
import sys
import uuid
from collections import Counter
from collections.abc import Mapping
from pathlib import Path
from zlib import crc32


from three_device_slam.synchronization import build_index as sync_builder
from three_device_slam.synchronization.sync_index import (
    HARD_SPAN_NS,
    TARGET_SPAN_NS,
    build_triplets,
)


SCHEMA = "ego.three_device.acceptance.v1"
STATUS_ORDER = {"PASS": 0, "BLOCKED": 1, "FAIL": 2}
REQUIRED_SECTIONS = (
    "provenance",
    "ego",
    "left",
    "right",
    "joint_gate",
    "clock_domains",
    "sync_index",
    "storage",
)
REQUIRED_FILES = {
    "coordinator": "coordinator.json",
    "ego_acceptance": "ego/acceptance.json",
    "left_acceptance": "left/acceptance.json",
    "right_acceptance": "right/acceptance.json",
    "ego_manifest": "ego/manifest.json",
    "ego_video_index": "ego/ego.video.jsonl",
    "left_frames": "left/d405_frames.csv",
    "right_frames": "right/d405_frames.csv",
    "sync_manifest": "sync/manifest.json",
    "sync_csv": "sync/common_30hz.csv",
}
CAMERA_STREAMS = ("color", "infrared_left", "infrared_right")
D405_TIMESTAMP_DOMAINS = {
    "global_time",
    "system_time",
    "hardware_clock",
    "unrecognized",
    "unverified",
}
IMU_ZERO_COUNTERS = (
    "frames_bad",
    "resyncs",
    "dropped_frames",
    "counter_resets",
    "counter_stalls",
    "sequence_gaps",
    "invalid_imu_flags",
    "queue_overflow_flags",
    "serial_errors",
    "serial_reconnects",
)
HASH_PATTERN = re.compile(r"^[0-9a-f]{64}$")
STREAM_PATTERN = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9._-]*[A-Za-z0-9])?$")


class Section:
    def __init__(self):
        self.failures = []
        self.blockers = []
        self.evidence = {}
        self.measurements = {}

    def fail(self, reason):
        self.failures.append(str(reason))

    def block(self, reason):
        self.blockers.append(str(reason))

    def report(self):
        status = "FAIL" if self.failures else "BLOCKED" if self.blockers else "PASS"
        return {
            "status": status,
            "reasons": sorted(set(self.failures + self.blockers)),
            "measurements": self.measurements,
            "evidence": self.evidence,
        }


def _evaluate_session(session: Path) -> dict:
    session = Path(session)
    sections = {name: Section() for name in REQUIRED_SECTIONS}
    documents = {}
    paths = {}
    missing_derived = set()

    try:
        session_root = session.resolve(strict=True)
        if not session_root.is_dir():
            raise ValueError("session root is not a directory")
    except (OSError, ValueError) as exc:
        sections["storage"].fail(f"session_root:{type(exc).__name__}")
        return _finish(sections)

    for label, relative in REQUIRED_FILES.items():
        try:
            paths[label] = _safe_regular_file(session_root, relative)
        except (OSError, ValueError) as exc:
            if label in {"sync_manifest", "sync_csv"}:
                missing_derived.add(label)
            else:
                sections["storage"].fail(
                    f"required_file:{relative}:{type(exc).__name__}"
                )

    document_sections = {
        "coordinator": "joint_gate",
        "ego_acceptance": "ego",
        "left_acceptance": "left",
        "right_acceptance": "right",
        "ego_manifest": "storage",
        "sync_manifest": "sync_index",
    }
    for label, section_name in document_sections.items():
        if label not in paths:
            continue
        try:
            documents[label] = _read_json_object(paths[label], label)
        except (OSError, ValueError) as exc:
            sections[section_name].fail(f"malformed_json:{label}:{type(exc).__name__}")

    coordinator = documents.get("coordinator")
    acceptances = {
        device: documents.get(f"{device}_acceptance")
        for device in ("ego", "left", "right")
    }
    sync_manifest = documents.get("sync_manifest")

    clock_evidence = _check_clock_domains(
        sections["clock_domains"], paths, coordinator, acceptances
    )
    allow_missing_derived = bool(
        clock_evidence and clock_evidence["allow_missing_derived"]
    )
    _check_provenance(
        sections["provenance"],
        coordinator,
        sync_manifest,
        acceptances,
        allow_missing_derived,
    )
    raw = clock_evidence["raw"] if clock_evidence else {}
    _check_ego(sections["ego"], acceptances["ego"], raw.get("ego"))
    _check_d405(sections["left"], acceptances["left"], "left", raw.get("left"))
    _check_d405(
        sections["right"], acceptances["right"], "right", raw.get("right")
    )
    _check_joint_gate(sections["joint_gate"], coordinator, acceptances)
    _check_sync_index(
        sections["sync_index"],
        sections["storage"],
        session_root,
        paths,
        sync_manifest,
        coordinator,
        clock_evidence,
        missing_derived,
    )
    _check_storage(
        sections["storage"], session_root, paths, documents.get("ego_manifest"), acceptances
    )
    return _finish(sections)


def verify_session(session: Path) -> dict:
    session = Path(session)
    try:
        report = _evaluate_session(session)
    except Exception as exc:
        sections = {name: Section() for name in REQUIRED_SECTIONS}
        sections["storage"].fail(f"verification_exception:{type(exc).__name__}")
        report = _finish(sections)
    _publish_quality_reports(session, report)
    return report


def _check_provenance(
    section, coordinator, sync_manifest, acceptances, allow_missing_derived
):
    if coordinator is None:
        section.fail("provenance_inputs_unavailable")
        return
    inputs = [("coordinator", coordinator)]
    if sync_manifest is None:
        if allow_missing_derived:
            section.block("sync_provenance_unavailable_due_to_clock")
        else:
            section.fail("sync_provenance_unavailable")
    else:
        inputs.append(("sync_index", sync_manifest))
    for label, document in inputs:
        try:
            git = _mapping(document.get("git"), f"{label}.git")
            commit = git.get("hash")
            dirty = git.get("dirty")
            if not isinstance(commit, str) or not commit.strip():
                raise ValueError(f"{label}.git.hash")
            if not isinstance(dirty, bool):
                raise ValueError(f"{label}.git.dirty")
            section.evidence[f"{label}_git"] = {"hash": commit, "dirty": dirty}
        except ValueError as exc:
            section.fail(exc)
    embedded = coordinator.get("worker_acceptance")
    if not isinstance(embedded, Mapping):
        section.fail("coordinator.worker_acceptance")
    else:
        for device, actual in acceptances.items():
            if actual is not None and embedded.get(device) != actual:
                section.fail(f"worker_acceptance_mismatch:{device}")


def _check_ego(section, acceptance, raw):
    if acceptance is None:
        section.fail("ego_acceptance_unavailable")
        return
    try:
        if acceptance.get("schema") != "ego.2uq2.acceptance.v1":
            raise ValueError("ego.schema")
        status = _status(acceptance.get("status"), "ego.status")
        if status == "FAIL":
            section.fail(f"worker_fail:{acceptance.get('reason', 'ego_acceptance')}")
        elif status == "BLOCKED":
            section.block(f"worker_blocked:{acceptance.get('reason', 'ego_acceptance')}")
        checks = _mapping(acceptance.get("checks"), "ego.checks")
        for name, value in checks.items():
            check = _mapping(value, f"ego.checks.{name}")
            check_status = _status(check.get("status"), f"ego.checks.{name}.status")
            if check_status == "FAIL":
                section.fail(f"check_fail:{name}")
            elif check_status == "BLOCKED":
                section.block(f"check_blocked:{name}")
        window = _mapping(acceptance.get("formal_window"), "ego.formal_window")
        start_ns = _nonnegative_int(window.get("start_ns"), "ego.formal_start_ns")
        counts = _mapping(acceptance.get("counts"), "ego.counts")
        warmup_frames = _nonnegative_int(counts.get("warmup_frames"), "ego.warmup_frames")
        formal_frames = _nonnegative_int(counts.get("formal_frames"), "ego.formal_frames")
        if status == "PASS" and formal_frames == 0:
            section.fail("ego_no_formal_frames")
        observed = _ego_observed_summary(raw, start_ns) if raw is not None else None
        if raw is None:
            section.fail("ego_raw_summary_unavailable")
        else:
            if raw["warmup_rows"] < 1:
                section.fail("raw_warmup_missing")
            if raw["warmup_rows"] != warmup_frames:
                section.fail("raw_warmup_count_mismatch")
            if raw["formal_rows"] != formal_frames:
                section.fail("raw_formal_count_mismatch")
            if status == "PASS" and observed["host_formal_rows"] < 2:
                section.fail("insufficient_comparable_formal_rows_for_rate")
        formal = _mapping(acceptance.get("formal_evidence"), "ego.formal_evidence")
        rate = _finite_number(formal.get("window_rate_hz"), "ego.window_rate_hz")
        if rate < 59.0:
            section.fail("formal_rate_below_59_hz")
        for name in ("timestamp_regressions", "xu_failures", "bad_jpegs"):
            if _nonnegative_int(formal.get(name), f"ego.{name}") != 0:
                section.fail(f"nonzero:{name}")
        timestamp_unavailable = _nonnegative_int(
            formal.get("timestamp_unavailable"), "ego.timestamp_unavailable"
        )
        if timestamp_unavailable:
            if status == "BLOCKED":
                section.block("formal_acquisition_clock_unavailable")
            else:
                section.fail("nonzero:timestamp_unavailable")
        for name in ("duplicate_sequences", "sequence_gaps", "sequence_regressions"):
            if _nonnegative_int(formal.get(name), f"ego.{name}") != 0:
                if status == "BLOCKED":
                    section.block(f"unverified_sequence_relation:{name}")
                else:
                    section.fail(f"nonzero:{name}")
        if observed is not None:
            for field, raw_field in (
                ("observed_frames", "observed_rows"),
                ("total_frames", "observed_rows"),
                ("valid_frames", "valid_formal_rows"),
                ("comparable_frames", "host_formal_rows"),
                ("timestamp_unavailable", "unavailable_formal_rows"),
            ):
                if _nonnegative_int(formal.get(field), f"ego.{field}") != observed[raw_field]:
                    section.fail(f"raw_formal_evidence_mismatch:{field}")
            if formal.get("first_acquisition_ns") != observed["first_host_formal_ns"]:
                section.fail("raw_first_acquisition_mismatch")
            if formal.get("last_acquisition_ns") != observed["last_host_formal_ns"]:
                section.fail("raw_last_acquisition_mismatch")
            observed_rate = _finite_number(formal.get("rate_hz"), "ego.rate_hz")
            if not math.isclose(observed_rate, observed["span_rate_hz"], rel_tol=1e-6, abs_tol=1e-6):
                section.fail("raw_span_rate_mismatch")
            sequence = _mapping(
                acceptance.get("sequence_evidence"), "ego.sequence_evidence"
            )
            for field in (
                "valid_sequences",
                "duplicate_sequences",
                "sequence_gaps",
                "sequence_regressions",
            ):
                expected = observed[field]
                if _nonnegative_int(formal.get(field), f"ego.{field}") != expected:
                    section.fail(f"raw_formal_evidence_mismatch:{field}")
                if _nonnegative_int(sequence.get(field), f"ego.sequence.{field}") != expected:
                    section.fail(f"raw_sequence_evidence_mismatch:{field}")
        requested_duration = _finite_number(
            window.get("requested_duration_s"), "ego.requested_duration_s"
        )
        measured_duration = _finite_number(
            window.get("measured_duration_s"), "ego.measured_duration_s"
        )
        if requested_duration <= 0 or measured_duration < requested_duration:
            section.fail("formal_duration")
        if _nonnegative_int(window.get("formal_frames"), "ego.window.formal_frames") != formal_frames:
            section.fail("formal_window_count_mismatch")
        expected_window_rate = (
            observed["valid_formal_rows"] / requested_duration
            if observed is not None
            else None
        )
        if expected_window_rate is not None and not math.isclose(
            rate, expected_window_rate, rel_tol=1e-9, abs_tol=1e-9
        ):
            section.fail("formal_window_rate_mismatch")
        deadline_ns = _nonnegative_int(
            window.get("deadline_ns"), "ego.formal_deadline_ns"
        )
        if observed is not None and observed["host_formal_rows"]:
            if observed["first_host_formal_ns"] > start_ns + 50_000_000:
                section.fail("raw_formal_start_not_covered")
            if observed["last_host_formal_ns"] < deadline_ns - 50_000_000:
                section.fail("raw_formal_end_not_covered")
        diagnostics = _mapping(acceptance.get("capture_diagnostics"), "ego.capture_diagnostics")
        for name in ("callback_exceptions", "capture_errors"):
            if _nonnegative_int(diagnostics.get(name), f"ego.{name}") != 0:
                section.fail(f"nonzero:{name}")
        hashes = _mapping(acceptance.get("hashes"), "ego.hashes")
        _hash(hashes.get("xu_library_sha256"), "ego.xu_library_sha256")
        _mapping(hashes.get("streams"), "ego.hashes.streams")
        section.measurements.update(
            video_rate_hz=rate,
            warmup_frames=warmup_frames,
            formal_frames=formal_frames,
        )
    except ValueError as exc:
        section.fail(exc)


def _check_d405(section, acceptance, device, raw):
    if acceptance is None:
        section.fail(f"{device}_acceptance_unavailable")
        return
    try:
        status = _status(acceptance.get("result"), f"{device}.result")
        if status == "FAIL":
            section.fail(f"worker_fail:{device}")
        elif status == "BLOCKED":
            section.block(f"worker_blocked:{device}")
        if acceptance.get("capture_error") is not None:
            section.fail("capture_error")
        if acceptance.get("warmup_recorded") is not True:
            section.fail("raw_warmup_not_recorded")
        formal_framesets = _positive_int(
            acceptance.get("formal_complete_framesets_for_csv"),
            f"{device}.formal_complete_framesets_for_csv",
        )
        if formal_framesets < 2:
            section.fail("insufficient_formal_framesets")
        if raw is None:
            section.fail("d405_raw_summary_unavailable")
        else:
            if raw["warmup_rows"] < 1:
                section.fail("raw_warmup_missing")
            if raw["formal_rows"] != formal_framesets:
                section.fail("raw_formal_count_mismatch")
        storage = _mapping(acceptance.get("camera_storage"), f"{device}.camera_storage")
        if storage.get("recorder_clean_shutdown") is not True:
            section.fail("camera_recorder_unclean")
        if storage.get("authoritative_output_stable") is not True:
            section.fail("camera_output_unstable")
        for name in ("recorder_shutdown_error", "move_error"):
            if storage.get(name) is not None:
                section.fail(f"camera_storage:{name}")

        camera = _mapping(acceptance.get("camera"), f"{device}.camera")
        if _status(camera.get("result"), f"{device}.camera.result") != "PASS":
            section.fail("camera_result")
        if camera.get("authoritative_stream_stats") != "db3_streams":
            section.fail("camera_stats_not_authoritative")
        if camera.get("db3_analysis_error") is not None or camera.get("csv_rebuild_error") is not None:
            section.fail("camera_analysis_error")
        stream_reports = _mapping(camera.get("db3_streams"), f"{device}.camera.db3_streams")
        if set(stream_reports) != set(CAMERA_STREAMS):
            raise ValueError(f"{device}.camera.streams")
        rates = {}
        for stream_name in CAMERA_STREAMS:
            stream = _mapping(stream_reports[stream_name], f"{device}.{stream_name}")
            received = _positive_int(
                stream.get("received"), f"{device}.{stream_name}.received"
            )
            if received < 2:
                section.fail(f"camera_received:{stream_name}")
            if received != formal_framesets:
                section.fail(f"camera_raw_count_mismatch:{stream_name}")
            rate = _finite_number(stream.get("rate_hz"), f"{device}.{stream_name}.rate_hz")
            rates[stream_name] = rate
            if rate < 29.5:
                section.fail(f"camera_rate:{stream_name}")
            if _finite_number(stream.get("gap_ratio"), f"{device}.{stream_name}.gap_ratio") > 0.001:
                section.fail(f"camera_gap_ratio:{stream_name}")
            for name in ("repeated_frames", "frame_number_resets", "timestamp_regressions"):
                if _nonnegative_int(stream.get(name), f"{device}.{stream_name}.{name}") != 0:
                    section.fail(f"camera_continuity:{stream_name}:{name}")

        imu = _mapping(acceptance.get("imu"), f"{device}.imu")
        if _status(imu.get("result"), f"{device}.imu.result") != "PASS":
            section.fail("imu_result")
        if imu.get("protocol") not in {"kt_ex9_37", "stm32_combined_v1"}:
            section.fail("imu_protocol")
        rate = _finite_number(imu.get("rate_hz"), f"{device}.imu.rate_hz")
        if not 399.0 <= rate <= 401.0:
            section.fail("imu_rate")
        formal = _mapping(imu.get("formal_window_stats"), f"{device}.imu.formal_window_stats")
        if _positive_int(formal.get("frames_ok"), f"{device}.imu.frames_ok") < 1:
            section.fail("imu_no_formal_frames")
        for name in IMU_ZERO_COUNTERS:
            if _nonnegative_int(formal.get(name), f"{device}.imu.{name}") != 0:
                section.fail(f"imu_continuity:{name}")
        if _nonnegative_int(imu.get("recorder_drops"), f"{device}.imu.recorder_drops") != 0:
            section.fail("imu_recorder_drops")
        if imu.get("recorder_write_error") is not None:
            section.fail("imu_recorder_write_error")
        if imu.get("recorder_clean_shutdown") is not True:
            section.fail("imu_recorder_unclean")
        if imu.get("persisted_formal_lower_bound_ok") is not True:
            section.fail("imu_persistence")
        formal_samples = _positive_int(imu.get("formal_samples"), f"{device}.imu.formal_samples")
        samples_written = _positive_int(imu.get("samples_written"), f"{device}.imu.samples_written")
        if samples_written < formal_samples:
            section.fail("imu_persisted_count")

        joint = _mapping(acceptance.get("joint_start"), f"{device}.joint_start")
        if joint.get("device_id") != device:
            section.fail("joint_start_device")
        _nonnegative_int(
            joint.get("formal_start_host_monotonic_ns"),
            f"{device}.formal_start_host_monotonic_ns",
        )
        if joint.get("raw_recording_is_append_only") is not True:
            section.fail("raw_not_append_only")
        if joint.get("raw_warmup_preserved") is not True:
            section.fail("raw_warmup_not_preserved")
        if joint.get("classification_metadata") != "d405_frames.csv:warmup":
            section.fail("warmup_classification_missing")
        section.measurements.update(camera_rates_hz=rates, imu_rate_hz=rate)
    except ValueError as exc:
        section.fail(exc)


def _check_joint_gate(section, coordinator, acceptances):
    if coordinator is None:
        section.fail("coordinator_unavailable")
        return
    try:
        if coordinator.get("schema") != "ego.three_device.coordinator.v1":
            raise ValueError("coordinator.schema")
        status = _status(coordinator.get("status"), "coordinator.status")
        if status == "FAIL":
            section.fail(f"coordinator_fail:{coordinator.get('reason', 'unknown')}")
        elif status == "BLOCKED":
            section.block(f"coordinator_blocked:{coordinator.get('reason', 'unknown')}")
        workers = _mapping(coordinator.get("workers"), "coordinator.workers")
        if set(workers) != {"ego", "left", "right"}:
            raise ValueError("coordinator.workers.devices")
        acceptance_statuses = {
            device: _worker_acceptance_status(device, acceptances.get(device))
            for device in ("ego", "left", "right")
        }
        has_failed_acceptance = "FAIL" in acceptance_statuses.values()
        for device in ("ego", "left", "right"):
            worker = _mapping(workers[device], f"coordinator.workers.{device}")
            exit_code = worker.get("exit_code")
            expected_exit = (
                3
                if acceptance_statuses[device] == "BLOCKED"
                else 0 if acceptance_statuses[device] == "PASS" else None
            )
            if (
                isinstance(exit_code, bool)
                or not isinstance(exit_code, int)
                or exit_code != expected_exit
                or exit_code == 3
                and (status != "BLOCKED" or has_failed_acceptance)
            ):
                section.fail(f"worker_exit:{device}")
            if worker.get("early_exit") is not False:
                section.fail(f"worker_early_exit:{device}")
            if worker.get("observed_running_at_or_after_formal_end") is not True:
                section.fail(f"worker_incomplete:{device}")
        transitions = coordinator.get("transitions")
        if not isinstance(transitions, list):
            raise ValueError("coordinator.transitions")
        ready_times = []
        for index, transition in enumerate(transitions):
            item = _mapping(transition, f"coordinator.transitions.{index}")
            state = item.get("state")
            if isinstance(state, str) and state.upper() == "JOINT_READY":
                ready_times.append(_nonnegative_int(item.get("at_ns"), "JOINT_READY.at_ns"))
        if not ready_times:
            section.fail("joint_ready_missing")
        scheduled_value = coordinator.get("scheduled_start_ns")
        task_start_value = coordinator.get("task_start_ego_frame_ns")
        if scheduled_value is None and status == "BLOCKED":
            scheduled = None
            section.block("scheduled_start_unavailable")
        else:
            scheduled = _nonnegative_int(scheduled_value, "scheduled_start_ns")
        if task_start_value is None and status == "BLOCKED":
            task_start = None
            section.block("task_start_ego_frame_unavailable")
        else:
            task_start = _nonnegative_int(task_start_value, "task_start_ego_frame_ns")
        if ready_times and scheduled is not None and scheduled <= ready_times[-1]:
            section.fail("scheduled_start_not_after_joint_ready")
        if task_start is not None and scheduled is not None and task_start < scheduled:
            section.fail("task_start_before_scheduled_start")
        first_error = coordinator.get("first_error")
        if status == "BLOCKED" and first_error is None:
            section.fail("coordinator_error_missing")
        if first_error is not None:
            error = _mapping(first_error, "coordinator.first_error")
            reason = error.get("reason")
            device = error.get("device_id")
            blocked_worker_error = (
                status == "BLOCKED"
                and device in acceptance_statuses
                and acceptance_statuses[device] == "BLOCKED"
                and workers[device].get("exit_code") == 3
                and error.get("worker_status") == "BLOCKED"
                and reason == acceptances[device].get("reason")
                and not has_failed_acceptance
            )
            if (
                status != "BLOCKED"
                or reason not in {"joint_warmup_timeout", "operator_cancelled"}
                and not blocked_worker_error
            ):
                section.fail(f"coordinator_error:{reason}")
        errors = coordinator.get("coordinator_errors")
        if not isinstance(errors, list):
            raise ValueError("coordinator.coordinator_errors")
        if errors:
            section.fail("coordinator_errors_present")
        failures = _mapping(coordinator.get("barrier_failures"), "coordinator.barrier_failures")
        if failures:
            section.fail("barrier_failures_present")
        section.measurements.update(
            scheduled_start_ns=scheduled,
            task_start_ego_frame_ns=task_start,
            joint_ready_ns=ready_times[-1] if ready_times else None,
        )
    except ValueError as exc:
        section.fail(exc)


def _worker_acceptance_status(device, acceptance):
    if not isinstance(acceptance, Mapping):
        return None
    value = acceptance.get("status" if device == "ego" else "result")
    return value if value in STATUS_ORDER else None


def _check_clock_domains(section, paths, coordinator, acceptances):
    if coordinator is None:
        section.fail("coordinator_unavailable")
        return None
    try:
        payloads = {}
        for label in ("ego_video_index", "left_frames", "right_frames"):
            if label not in paths:
                raise ValueError(f"clock_source_unavailable:{label}")
            payloads[label] = paths[label].read_bytes()
        raw = {
            "ego": _summarize_ego_rows(payloads["ego_video_index"]),
            "left": _summarize_d405_rows(payloads["left_frames"], "left"),
            "right": _summarize_d405_rows(payloads["right_frames"], "right"),
        }
        ego_acceptance = acceptances.get("ego")
        ego_window = (
            ego_acceptance.get("formal_window")
            if isinstance(ego_acceptance, Mapping)
            else None
        )
        clock_start_ns = (
            ego_window.get("start_ns")
            if isinstance(ego_window, Mapping)
            else coordinator.get("scheduled_start_ns")
        )
        clock_start_ns = _nonnegative_int(clock_start_ns, "ego formal clock start")
        ego_observed = _ego_observed_summary(raw["ego"], clock_start_ns)
        ego_clock_unavailable = ego_observed["unavailable_formal_rows"] > 0
        if ego_clock_unavailable:
            section.block("common_clock_unverified:ego:acquisition_unavailable")
        for device in ("left", "right"):
            acceptance = acceptances.get(device)
            if not isinstance(acceptance, Mapping) or not isinstance(
                acceptance.get("joint_start"), Mapping
            ):
                section.block(f"common_clock_unverified:{device}:missing_joint_start")
                continue
            clock = _validate_d405_camera_clock(acceptance, raw[device], device)
            if clock is None:
                section.block(
                    f"common_clock_unverified:{device}:missing_camera_clock"
                )
                continue
            section.evidence[f"{device}_camera_clock"] = {
                "configuration": clock["configuration"],
                "observed_domains": clock["observed_domains"],
            }
            section.measurements[f"{device}_csv_domain_counts"] = (
                clock["csv_domain_counts"]
            )
            if not clock["verified"]:
                reason = (
                    "missing_joint_claim"
                    if clock["claim_missing"]
                    else "missing_csv_evidence"
                    if clock["csv_evidence_missing"]
                    else "camera_clock_unverified"
                )
                section.block(f"common_clock_unverified:{device}:{reason}")

        task_start_value = coordinator.get("task_start_ego_frame_ns")
        streams = None
        if ego_clock_unavailable:
            if task_start_value is None:
                section.block("task_start_unavailable_due_to_acquisition_clock")
            else:
                _nonnegative_int(task_start_value, "task_start_ego_frame_ns")
                section.block("formal_ego_stream_not_indexable_without_acquisition_clock")
        else:
            task_start = _nonnegative_int(task_start_value, "task_start_ego_frame_ns")
            streams = {
                "ego": sync_builder._parse_ego_rows(
                    payloads["ego_video_index"], task_start
                ),
                "left": sync_builder._parse_d405_rows(
                    payloads["left_frames"], "left", task_start
                ),
                "right": sync_builder._parse_d405_rows(
                    payloads["right_frames"], "right", task_start
                ),
            }
        if streams is not None and any(not stream for stream in streams.values()):
            section.block("formal_common_clock_evidence_missing")
        section.evidence["domains"] = {
            "ego": (
                "host_monotonic"
                if not ego_clock_unavailable
                else "unavailable"
            ),
            "left": _joint_clock_domain(acceptances.get("left")),
            "right": _joint_clock_domain(acceptances.get("right")),
        }
        section.measurements["formal_rows"] = {
            device: summary["formal_rows"] for device, summary in raw.items()
        }
        return {
            "streams": streams,
            "raw": raw,
            "allow_missing_derived": bool(section.blockers and not section.failures),
        }
    except (OSError, UnicodeError, ValueError) as exc:
        section.fail(f"clock_validation:{exc}")
        return None


def _summarize_ego_rows(payload):
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("ego.video.jsonl is not UTF-8") from exc
    seen_sequences = set()
    previous_host_ns = None
    summary = {
        "rows": [],
        "warmup_rows": 0,
        "formal_rows": 0,
        "valid_formal_rows": 0,
        "host_formal_rows": 0,
        "unavailable_formal_rows": 0,
        "unavailable_rows": 0,
        "first_host_formal_ns": None,
        "last_host_formal_ns": None,
        "span_rate_hz": 0.0,
    }
    for line_number, line in enumerate(text.splitlines(), start=1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"ego line {line_number} malformed") from exc
        row = _mapping(row, f"ego line {line_number}")
        if row.get("stream_id") != "ego.video":
            raise ValueError(f"ego line {line_number} stream_id")
        sequence = _nonnegative_int(row.get("sequence"), f"ego line {line_number} sequence")
        acquisition_ns = _nonnegative_int(
            row.get("acquisition_ns"), f"ego line {line_number} acquisition_ns"
        )
        arrival_ns = _nonnegative_int(
            row.get("arrival_ns"), f"ego line {line_number} arrival_ns"
        )
        warmup = row.get("warmup")
        valid = row.get("valid")
        if not isinstance(warmup, bool) or not isinstance(valid, bool):
            raise ValueError(f"ego line {line_number} warmup/valid")
        _nonnegative_int(row.get("offset"), f"ego line {line_number} offset")
        _nonnegative_int(row.get("size"), f"ego line {line_number} size")
        metadata = _mapping(row.get("metadata"), f"ego line {line_number} metadata")
        acquisition_available = metadata.get("acquisition_available")
        if not isinstance(acquisition_available, bool):
            raise ValueError(f"ego line {line_number} acquisition_available")
        vendor_sequence = metadata.get("vendor_sequence")
        if vendor_sequence is not None and (
            isinstance(vendor_sequence, bool)
            or not isinstance(vendor_sequence, int)
            or not 0 <= vendor_sequence <= 0xFFFFFF
        ):
            raise ValueError(f"ego line {line_number} vendor_sequence")
        if sequence in seen_sequences:
            raise ValueError("ego sequence duplicate")
        seen_sequences.add(sequence)
        domain = row.get("clock_domain")
        if domain not in {
            "host_monotonic",
            "gstreamer_timestamp_unavailable",
            "arrival_only",
        }:
            raise ValueError(f"ego line {line_number} clock_domain")
        if domain == "host_monotonic":
            if not acquisition_available:
                raise ValueError(f"ego line {line_number} acquisition metadata")
            if previous_host_ns is not None and acquisition_ns < previous_host_ns:
                raise ValueError("ego timestamp regression")
            previous_host_ns = acquisition_ns
        else:
            if acquisition_available:
                raise ValueError(f"ego line {line_number} unavailable metadata")
            summary["unavailable_rows"] += 1
        summary["rows"].append(
            {
                "acquisition_ns": acquisition_ns,
                "arrival_ns": arrival_ns,
                "clock_domain": domain,
                "warmup": warmup,
                "valid": valid,
                "vendor_sequence": vendor_sequence,
            }
        )
        if warmup:
            summary["warmup_rows"] += 1
            continue
        summary["formal_rows"] += 1
        summary["valid_formal_rows"] += int(valid)
        if domain == "host_monotonic":
            summary["host_formal_rows"] += 1
            if summary["first_host_formal_ns"] is None:
                summary["first_host_formal_ns"] = acquisition_ns
            summary["last_host_formal_ns"] = acquisition_ns
        else:
            summary["unavailable_formal_rows"] += 1
    first_ns = summary["first_host_formal_ns"]
    last_ns = summary["last_host_formal_ns"]
    if summary["host_formal_rows"] > 1 and last_ns > first_ns:
        summary["span_rate_hz"] = (
            (summary["host_formal_rows"] - 1) * 1_000_000_000 / (last_ns - first_ns)
        )
    return summary


def _ego_observed_summary(raw, start_ns):
    observed_rows = []
    for row in raw["rows"]:
        observed_ns = (
            row["acquisition_ns"]
            if row["clock_domain"] == "host_monotonic"
            else row["arrival_ns"]
        )
        if observed_ns >= start_ns:
            observed_rows.append(row)
    host_ns = [
        row["acquisition_ns"]
        for row in observed_rows
        if row["clock_domain"] == "host_monotonic"
    ]
    result = {
        "observed_rows": len(observed_rows),
        "valid_formal_rows": sum(bool(row["valid"]) for row in observed_rows),
        "host_formal_rows": len(host_ns),
        "unavailable_formal_rows": len(observed_rows) - len(host_ns),
        "first_host_formal_ns": host_ns[0] if host_ns else None,
        "last_host_formal_ns": host_ns[-1] if host_ns else None,
        "span_rate_hz": 0.0,
        "valid_sequences": 0,
        "duplicate_sequences": 0,
        "sequence_gaps": 0,
        "sequence_regressions": 0,
    }
    previous_vendor_sequence = None
    for row in observed_rows:
        sequence = row["vendor_sequence"]
        if sequence is None:
            continue
        result["valid_sequences"] += 1
        if previous_vendor_sequence is not None:
            delta = (sequence - previous_vendor_sequence) & 0xFFFFFF
            if delta == 0:
                result["duplicate_sequences"] += 1
            elif delta < 0x800000:
                result["sequence_gaps"] += max(0, delta - 1)
            else:
                result["sequence_regressions"] += 1
        previous_vendor_sequence = sequence
    if len(host_ns) > 1 and host_ns[-1] > host_ns[0]:
        result["span_rate_hz"] = (
            (len(host_ns) - 1) * 1_000_000_000 / (host_ns[-1] - host_ns[0])
        )
    return result


def _summarize_d405_rows(payload, device):
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{device} frames are not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    fieldnames = reader.fieldnames
    required = {"set_index", "arrival_mono", "warmup"}
    if fieldnames is None or not required.issubset(fieldnames):
        raise ValueError(f"{device} frames missing columns")
    if any(name is None or not name.strip() for name in fieldnames):
        raise ValueError(f"{device} frames empty header")
    if len(fieldnames) != len(set(fieldnames)):
        raise ValueError(f"{device} frames duplicate header")
    producer_fields = {"arrival_wall"}
    for stream_id in CAMERA_STREAMS:
        producer_fields.update(
            {
                f"{stream_id}_frame_number",
                f"{stream_id}_device_ms",
                f"{stream_id}_mono",
                f"{stream_id}_domain",
            }
        )
    producer_columns_present = bool(set(fieldnames) & producer_fields)
    if producer_columns_present and not producer_fields.issubset(fieldnames):
        raise ValueError(f"{device} frames incomplete producer columns")
    summary = {
        "warmup_rows": 0,
        "formal_rows": 0,
        "producer_columns_present": producer_columns_present,
        "producer_domain_counts": {
            stream_id: {} for stream_id in CAMERA_STREAMS
        },
    }
    seen_sequences = set()
    previous_sequence = None
    previous_arrival_ns = None
    for row in reader:
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f"{device} frames row width")
        warmup = row.get("warmup")
        if warmup not in {"0", "1"}:
            raise ValueError(f"{device} frames warmup")
        sequence = _csv_nonnegative_int(
            row.get("set_index"), f"{device}.set_index"
        )
        if sequence in seen_sequences:
            raise ValueError(f"{device} set_index duplicate")
        if previous_sequence is not None and sequence != previous_sequence + 1:
            raise ValueError(f"{device} set_index continuity")
        if previous_sequence is None and producer_columns_present and sequence != 0:
            raise ValueError(f"{device} producer set_index must start at zero")
        arrival_ns = sync_builder._decimal_seconds_to_ns(
            row.get("arrival_mono"), f"{device}.arrival_mono"
        )
        if previous_arrival_ns is not None and arrival_ns < previous_arrival_ns:
            raise ValueError(f"{device} arrival_mono regression")
        seen_sequences.add(sequence)
        previous_sequence = sequence
        previous_arrival_ns = arrival_ns
        summary["warmup_rows" if warmup == "1" else "formal_rows"] += 1
        if producer_columns_present:
            _finite_csv_number(row["arrival_wall"], f"{device}.arrival_wall")
            for stream_id in CAMERA_STREAMS:
                _csv_nonnegative_int(
                    row[f"{stream_id}_frame_number"],
                    f"{device}.{stream_id}.frame_number",
                )
                _finite_csv_number(
                    row[f"{stream_id}_device_ms"],
                    f"{device}.{stream_id}.device_ms",
                )
                sync_builder._decimal_seconds_to_ns(
                    row[f"{stream_id}_mono"], f"{device}.{stream_id}.mono"
                )
                domain = row[f"{stream_id}_domain"]
                if domain not in D405_TIMESTAMP_DOMAINS:
                    raise ValueError(f"{device}.{stream_id}.domain")
                counts = summary["producer_domain_counts"][stream_id]
                counts[domain] = counts.get(domain, 0) + 1
    return summary


def _validate_d405_camera_clock(acceptance, raw, device):
    evidence_value = acceptance.get("camera_clock")
    if evidence_value is None:
        return None
    evidence = _mapping(evidence_value, f"{device}.camera_clock")
    configuration = _mapping(
        evidence.get("configuration"), f"{device}.camera_clock.configuration"
    )
    supported = configuration.get("supported")
    configured = configuration.get("verified")
    if not isinstance(supported, bool) or not isinstance(configured, bool):
        raise ValueError(f"{device}.camera_clock.configuration booleans")
    requested = _finite_number(
        configuration.get("requested"), f"{device}.camera_clock.requested"
    )
    if requested != 1.0:
        raise ValueError(f"{device}.camera_clock.requested")
    readback = configuration.get("readback")
    if readback is not None:
        readback = _finite_number(readback, f"{device}.camera_clock.readback")
    if not supported and readback is not None:
        raise ValueError(f"{device}.camera_clock.unsupported_readback")
    computed_configuration = supported and readback == 1.0
    if configured != computed_configuration:
        raise ValueError(f"{device}.camera_clock.configuration contradiction")

    required_streams = evidence.get("required_streams")
    if not isinstance(required_streams, list) or tuple(required_streams) != CAMERA_STREAMS:
        raise ValueError(f"{device}.camera_clock.required_streams")
    observed_value = _mapping(
        evidence.get("observed_domains"), f"{device}.camera_clock.observed_domains"
    )
    if set(observed_value) != set(CAMERA_STREAMS):
        raise ValueError(f"{device}.camera_clock.observed_streams")
    observed = {}
    for stream in CAMERA_STREAMS:
        counts_value = _mapping(
            observed_value[stream], f"{device}.camera_clock.{stream}"
        )
        if not set(counts_value).issubset(D405_TIMESTAMP_DOMAINS - {"unverified"}):
            raise ValueError(f"{device}.camera_clock.{stream}.domain")
        counts = {
            domain: _positive_int(
                count, f"{device}.camera_clock.{stream}.{domain}"
            )
            for domain, count in counts_value.items()
        }
        observed[stream] = counts
    all_global = all(
        counts and set(counts) == {"global_time"}
        for counts in observed.values()
    )
    computed_camera_verified = computed_configuration and all_global
    declared_camera_verified = evidence.get("verified")
    if not isinstance(declared_camera_verified, bool):
        raise ValueError(f"{device}.camera_clock.verified")
    if declared_camera_verified != computed_camera_verified:
        raise ValueError(f"{device}.camera_clock.observation contradiction")

    joint = _mapping(acceptance.get("joint_start"), f"{device}.joint_start")
    declared_joint_domain = joint.get("clock_domain")
    expected_joint_domain = (
        "host_monotonic" if computed_camera_verified else "unverified"
    )
    claim_missing = declared_joint_domain is None
    if not claim_missing and declared_joint_domain != expected_joint_domain:
        raise ValueError(f"{device}.joint_start.clock_domain contradiction")

    csv_evidence_missing = not raw["producer_columns_present"]
    csv_domain_counts = raw["producer_domain_counts"]
    if not csv_evidence_missing:
        expected_csv_domain = (
            "global_time" if computed_camera_verified else "unverified"
        )
        expected_rows = raw["warmup_rows"] + raw["formal_rows"]
        for stream in CAMERA_STREAMS:
            if csv_domain_counts[stream] != {expected_csv_domain: expected_rows}:
                raise ValueError(f"{device}.{stream}.csv_domain contradiction")

    return {
        "verified": (
            computed_camera_verified
            and not claim_missing
            and not csv_evidence_missing
        ),
        "configuration": dict(configuration),
        "observed_domains": observed,
        "csv_domain_counts": csv_domain_counts,
        "claim_missing": claim_missing,
        "csv_evidence_missing": csv_evidence_missing,
    }


def _joint_clock_domain(acceptance):
    if not isinstance(acceptance, Mapping):
        return None
    joint = acceptance.get("joint_start")
    return joint.get("clock_domain") if isinstance(joint, Mapping) else None


def _check_sync_index(
    section,
    storage,
    session,
    paths,
    manifest,
    coordinator,
    clock_evidence,
    missing_derived,
):
    if missing_derived:
        if (
            "sync_manifest" in missing_derived
            and clock_evidence is not None
            and clock_evidence["allow_missing_derived"]
        ):
            section.block("derived_sync_unavailable_due_to_clock")
            storage.block("derived_sync_unavailable_due_to_clock")
        else:
            section.fail("sync_inputs_unavailable")
            storage.fail("derived_sync_required_files_missing")
        return
    if manifest is None or coordinator is None or clock_evidence is None:
        section.fail("sync_inputs_unavailable")
        return
    streams = clock_evidence["streams"]
    if streams is None:
        section.fail("derived_sync_contradicts_unavailable_clock")
        return
    try:
        if manifest.get("schema") != sync_builder.SCHEMA:
            raise ValueError("sync.schema")
        for name, expected in (
            ("grid_ns", sync_builder.GRID_NS),
            ("target_span_ns", TARGET_SPAN_NS),
            ("hard_span_ns", HARD_SPAN_NS),
        ):
            if _nonnegative_int(manifest.get(name), f"sync.{name}") != expected:
                raise ValueError(f"sync.{name}")
        expected_input_config = {
            "ego": {
                "source": "ego/ego.video.jsonl",
                "stream_id": "ego.video",
                "clock_domain": "host_monotonic",
                "timestamp_field": "acquisition_ns",
                "formal_filter": "warmup=false,valid=true",
            },
            "left": {
                "source": "left/d405_frames.csv",
                "clock_domain": "host_monotonic",
                "timestamp_field": "arrival_mono",
                "timestamp_unit": "decimal_seconds",
                "formal_filter": "warmup=0",
            },
            "right": {
                "source": "right/d405_frames.csv",
                "clock_domain": "host_monotonic",
                "timestamp_field": "arrival_mono",
                "timestamp_unit": "decimal_seconds",
                "formal_filter": "warmup=0",
            },
        }
        if manifest.get("input_config") != expected_input_config:
            raise ValueError("sync.input_config")
        expected_overlap = sync_builder._common_overlap(
            streams["ego"], streams["left"], streams["right"]
        )
        if manifest.get("common_overlap") != expected_overlap:
            raise ValueError("sync.common_overlap")
        task_start = _nonnegative_int(manifest.get("task_start_ego_frame_ns"), "sync.task_start")
        if task_start != coordinator.get("task_start_ego_frame_ns"):
            raise ValueError("sync.task_start_mismatch")
        expected_sources = set(sync_builder.SOURCE_PATHS)
        source_hashes = _mapping(manifest.get("source_sha256"), "sync.source_sha256")
        if set(source_hashes) != expected_sources:
            raise ValueError("sync.source_sha256.paths")
        for relative, expected_hash in source_hashes.items():
            _hash(expected_hash, f"sync.source_sha256.{relative}")
            source = _safe_regular_file(session, relative)
            if _sha256(source) != expected_hash:
                storage.fail(f"sync_source_hash_mismatch:{relative}")
                section.fail(f"source_hash_mismatch:{relative}")
        expected_output_hash = _hash(manifest.get("output_csv_sha256"), "sync.output_hash")
        if _sha256(paths["sync_csv"]) != expected_output_hash:
            storage.fail("sync_output_hash_mismatch")
            section.fail("output_hash_mismatch")

        rows = _strict_csv(paths["sync_csv"], sync_builder.CSV_HEADER)
        expected_triplets = build_triplets(streams["ego"], streams["left"], streams["right"])
        if len(rows) != len(expected_triplets):
            section.fail("triplet_row_count_mismatch")
        reasons = Counter()
        trainable_count = 0
        for index, (row, expected) in enumerate(zip(rows, expected_triplets), start=2):
            actual = _parse_triplet(row, index)
            expected_values = {
                "sample_ns": expected.sample_ns,
                "ego_sequence": expected.ego.sequence,
                "ego_acquisition_ns": expected.ego.acquisition_ns,
                "ego_ref": expected.ego.payload_ref,
                "left_sequence": expected.left.sequence,
                "left_acquisition_ns": expected.left.acquisition_ns,
                "left_ref": expected.left.payload_ref,
                "right_sequence": expected.right.sequence,
                "right_acquisition_ns": expected.right.acquisition_ns,
                "right_ref": expected.right.payload_ref,
                "span_ns": expected.span_ns,
                "trainable": expected.trainable,
                "reason": expected.reason,
            }
            if actual != expected_values:
                section.fail(f"triplet_mismatch:line_{index}")
            if actual["span_ns"] > HARD_SPAN_NS and actual["trainable"]:
                section.fail(f"over_hard_trainable:line_{index}")
            reasons[actual["reason"]] += 1
            trainable_count += int(actual["trainable"])
        stored_reasons = _mapping(manifest.get("reason_counts"), "sync.reason_counts")
        expected_reason_counts = {reason: reasons[reason] for reason in sync_builder.REASONS}
        if stored_reasons != expected_reason_counts:
            section.fail("reason_counts_mismatch")
        if _nonnegative_int(manifest.get("row_count"), "sync.row_count") != len(rows):
            section.fail("manifest_row_count_mismatch")
        if _nonnegative_int(manifest.get("trainable_count"), "sync.trainable_count") != trainable_count:
            section.fail("manifest_trainable_count_mismatch")
        if not rows:
            section.fail("sync_index_empty")
        if trainable_count < 1:
            section.fail("sync_index_has_no_trainable_rows")
        section.measurements.update(
            row_count=len(rows),
            trainable_count=trainable_count,
            reason_counts=expected_reason_counts,
        )
    except (OSError, UnicodeError, ValueError) as exc:
        section.fail(f"sync_validation:{exc}")


def _check_storage(section, session, paths, manifest, acceptances):
    if manifest is None:
        section.fail("ego_manifest_unavailable")
        return
    try:
        if manifest.get("schema") != "ego.three_device.raw_session.v1":
            raise ValueError("ego_manifest.schema")
        streams = _mapping(manifest.get("streams"), "ego_manifest.streams")
        if not {"ego.video", "ego.xu"}.issubset(streams):
            raise ValueError("ego_manifest.required_streams")
        for stream_id, hashes in streams.items():
            if not isinstance(stream_id, str) or not STREAM_PATTERN.fullmatch(stream_id) or ".." in stream_id:
                raise ValueError(f"ego_manifest.stream_id:{stream_id}")
            entry = _mapping(hashes, f"ego_manifest.streams.{stream_id}")
            for suffix, hash_name in (("bin", "payload_sha256"), ("jsonl", "index_sha256")):
                expected_hash = _hash(entry.get(hash_name), f"{stream_id}.{hash_name}")
                raw_path = _safe_regular_file(session, f"ego/{stream_id}.{suffix}")
                if _sha256(raw_path) != expected_hash:
                    section.fail(f"ego_raw_hash_mismatch:{stream_id}.{suffix}")
        ego = acceptances.get("ego")
        if isinstance(ego, Mapping):
            hashes = ego.get("hashes")
            if not isinstance(hashes, Mapping) or hashes.get("streams") != streams:
                section.fail("ego_acceptance_stream_hash_mismatch")
        _verify_ego_xu_payloads(session)
        for device in ("left", "right"):
            acceptance = acceptances.get(device)
            if not isinstance(acceptance, Mapping):
                continue
            camera_storage = acceptance.get("camera_storage")
            imu = acceptance.get("imu")
            if not isinstance(camera_storage, Mapping) or not isinstance(imu, Mapping):
                continue
            if (
                camera_storage.get("recorder_clean_shutdown") is not True
                or camera_storage.get("authoritative_output_stable") is not True
                or imu.get("recorder_clean_shutdown") is not True
                or imu.get("recorder_drops") != 0
                or imu.get("recorder_write_error") is not None
            ):
                section.fail(f"worker_storage_unclean:{device}")
            _verify_d405_raw_files(session, device, acceptance)
        section.evidence["required_files"] = sorted(REQUIRED_FILES.values())
    except (OSError, ValueError) as exc:
        section.fail(f"storage_validation:{exc}")


def _read_jsonl(path: Path, label: str) -> list[dict]:
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{label} line {line_number} malformed") from exc
        if not isinstance(row, dict):
            raise ValueError(f"{label} line {line_number} must be an object")
        rows.append(row)
    return rows


def _verify_ego_xu_payloads(session: Path) -> None:
    video_rows = _read_jsonl(
        _safe_regular_file(session, "ego/ego.video.jsonl"), "ego.video.jsonl"
    )
    xu_rows = _read_jsonl(
        _safe_regular_file(session, "ego/ego.xu.jsonl"), "ego.xu.jsonl"
    )
    if len(video_rows) != len(xu_rows):
        raise ValueError("ego video/xu index row count mismatch")
    _verify_indexed_payload_stream(session, "ego.video", video_rows)
    xu_payloads = _verify_indexed_payload_stream(session, "ego.xu", xu_rows)
    for line_number, (video, xu, raw) in enumerate(
        zip(video_rows, xu_rows, xu_payloads), 1
    ):
        if video.get("stream_id") != "ego.video" or xu.get("stream_id") != "ego.xu":
            raise ValueError(f"ego video/xu stream mismatch at line {line_number}")
        for field in (
            "sequence",
            "acquisition_ns",
            "arrival_ns",
            "clock_domain",
            "warmup",
            "valid",
        ):
            if xu.get(field) != video.get(field):
                raise ValueError(f"ego video/xu {field} mismatch at line {line_number}")
        size = _positive_int(xu.get("size"), f"ego.xu line {line_number} size")
        if size != 27:
            raise ValueError(f"ego.xu line {line_number} must cover one 27-byte packet")
        vendor_sequence = int.from_bytes(raw[:3], "big")
        xu_metadata = _mapping(xu.get("metadata"), f"ego.xu line {line_number} metadata")
        video_metadata = _mapping(
            video.get("metadata"), f"ego.video line {line_number} metadata"
        )
        if (
            xu_metadata.get("vendor_sequence") != vendor_sequence
            or video_metadata.get("vendor_sequence") != vendor_sequence
        ):
            raise ValueError(f"ego.xu line {line_number} 24-bit sequence mismatch")


def _verify_indexed_payload_stream(session: Path, stream_id: str, rows: list[dict]):
    payload_path = _safe_regular_file(session, f"ego/{stream_id}.bin")
    payloads = []
    cursor = 0
    with payload_path.open("rb") as payload:
        for line_number, row in enumerate(rows, 1):
            if row.get("stream_id") != stream_id:
                raise ValueError(f"{stream_id} line {line_number} stream mismatch")
            offset = _nonnegative_int(
                row.get("offset"), f"{stream_id} line {line_number} offset"
            )
            size = _positive_int(
                row.get("size"), f"{stream_id} line {line_number} size"
            )
            if offset != cursor:
                raise ValueError(
                    f"{stream_id} payload coverage mismatch at line {line_number}"
                )
            payload.seek(offset)
            raw = payload.read(size)
            if len(raw) != size:
                raise ValueError(f"{stream_id} payload truncated at line {line_number}")
            stored_crc = _nonnegative_int(
                row.get("crc32"), f"{stream_id} line {line_number} crc32"
            )
            if stored_crc > 0xFFFFFFFF or crc32(raw) & 0xFFFFFFFF != stored_crc:
                raise ValueError(f"{stream_id} payload crc32 mismatch at line {line_number}")
            payloads.append(raw)
            cursor += size
    if payload_path.stat().st_size != cursor:
        raise ValueError(f"{stream_id} payload has unindexed bytes")
    return payloads


def _verify_d405_raw_files(session: Path, device: str, acceptance: Mapping) -> None:
    manifest = _mapping(acceptance.get("raw_files"), f"{device}.raw_files")
    required = {
        "external_imu/raw_imu_packets.bin",
        "external_imu/imu.bin",
        "external_imu/imu_ts.csv",
        "d405_frames.csv",
    }
    if not required.issubset(manifest) or not any(name.endswith(".db3") for name in manifest):
        raise ValueError(f"d405_raw_manifest_required_files:{device}")
    for relative, evidence_value in manifest.items():
        if not isinstance(relative, str) or not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError(f"d405_raw_manifest_path:{device}:{relative}")
        evidence = _mapping(evidence_value, f"{device}.raw_files.{relative}")
        expected_size = _positive_int(
            evidence.get("size_bytes"), f"{device}.raw_files.{relative}.size_bytes"
        )
        expected_hash = _hash(
            evidence.get("sha256"), f"{device}.raw_files.{relative}.sha256"
        )
        path = _safe_regular_file(session, f"{device}/{relative}")
        if path.stat().st_size != expected_size or _sha256(path) != expected_hash:
            raise ValueError(f"d405_raw_integrity_mismatch:{device}:{relative}")
    imu = _mapping(acceptance.get("imu"), f"{device}.imu")
    packet_sizes = {"kt_ex9_37": 37, "stm32_combined_v1": 63}
    protocol = imu.get("protocol")
    if protocol not in packet_sizes:
        raise ValueError(f"d405_raw_packet_protocol:{device}")
    raw_size = manifest["external_imu/raw_imu_packets.bin"]["size_bytes"]
    packet_size = packet_sizes[protocol]
    samples_written = _positive_int(
        imu.get("samples_written"), f"{device}.imu.samples_written"
    )
    if raw_size % packet_size or raw_size // packet_size < samples_written:
        raise ValueError(f"d405_raw_packet_count_mismatch:{device}")


def _parse_triplet(row, line_number):
    result = {}
    for name in (
        "sample_ns",
        "ego_sequence",
        "ego_acquisition_ns",
        "left_sequence",
        "left_acquisition_ns",
        "right_sequence",
        "right_acquisition_ns",
        "span_ns",
    ):
        result[name] = _csv_nonnegative_int(row[name], f"triplets line {line_number} {name}")
    for name in ("ego_ref", "left_ref", "right_ref"):
        value = row[name]
        if not value:
            raise ValueError(f"triplets line {line_number} {name}")
        result[name] = value
    if row["trainable"] not in {"0", "1"}:
        raise ValueError(f"triplets line {line_number} trainable")
    result["trainable"] = row["trainable"] == "1"
    if row["reason"] not in sync_builder.REASONS:
        raise ValueError(f"triplets line {line_number} reason")
    result["reason"] = row["reason"]
    timestamps = sorted(
        result[name]
        for name in ("ego_acquisition_ns", "left_acquisition_ns", "right_acquisition_ns")
    )
    if result["sample_ns"] != timestamps[1] or result["span_ns"] != timestamps[-1] - timestamps[0]:
        raise ValueError(f"triplets line {line_number} timing")
    if result["span_ns"] <= TARGET_SPAN_NS:
        classification = (True, "within_target")
    elif result["span_ns"] <= HARD_SPAN_NS:
        classification = (True, "within_hard_limit")
    else:
        classification = (False, "span_over_hard_limit")
    if (result["trainable"], result["reason"]) != classification:
        raise ValueError(f"triplets line {line_number} classification")
    return result


def _strict_csv(path, header):
    text = path.read_text(encoding="utf-8")
    rows = list(csv.reader(io.StringIO(text, newline="")))
    if not rows or tuple(rows[0]) != tuple(header):
        raise ValueError(f"{path.name} header")
    if any(len(row) != len(header) for row in rows[1:]):
        raise ValueError(f"{path.name} row width")
    return [dict(zip(header, row)) for row in rows[1:]]


def _finish(sections):
    reports = {name: sections[name].report() for name in REQUIRED_SECTIONS}
    status = max((report["status"] for report in reports.values()), key=STATUS_ORDER.get)
    reasons = sorted(
        f"{name}:{reason}"
        for name, report in reports.items()
        if report["status"] == status
        for reason in report["reasons"]
    )
    return {"schema": SCHEMA, "status": status, "reasons": reasons, "sections": reports}


def _read_json_object(path, label):
    def reject_constant(value):
        raise ValueError(f"{label} contains non-finite number: {value}")

    value = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def _safe_regular_file(session, relative):
    relative_path = Path(relative)
    if relative_path.is_absolute() or ".." in relative_path.parts:
        raise ValueError(f"unsafe path: {relative}")
    path = (session / relative_path).resolve(strict=True)
    if not path.is_relative_to(session) or not path.is_file():
        raise ValueError(f"path escapes session or is not a file: {relative}")
    return path


def _mapping(value, name):
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _status(value, name):
    if not isinstance(value, str) or value not in STATUS_ORDER:
        raise ValueError(f"{name} must be PASS, FAIL, or BLOCKED")
    return value


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _nonnegative_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _positive_int(value, name):
    value = _nonnegative_int(value, name)
    if value == 0:
        raise ValueError(f"{name} must be positive")
    return value


def _csv_nonnegative_int(value, name):
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a non-negative integer") from exc
    if str(parsed) != value or parsed < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return parsed


def _finite_csv_number(value, name):
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be finite")
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(parsed) or parsed < 0:
        raise ValueError(f"{name} must be finite and non-negative")
    return parsed


def _hash(value, name):
    if not isinstance(value, str) or not HASH_PATTERN.fullmatch(value):
        raise ValueError(f"{name} must be a lowercase SHA-256")
    return value


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_atomic_json(path, report):
    payload = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode()
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        if temporary.exists():
            temporary.unlink()


def _publish_quality_reports(session: Path, report: dict) -> None:
    quality_directory = session / "quality"
    quality_directory.mkdir(exist_ok=True)
    acquisition_path = quality_directory / "acquisition_timing.json"
    product_path = quality_directory / "product_status.json"
    final_paths = (acquisition_path, product_path)
    _remove_quality_reports(quality_directory, final_paths)
    try:
        _write_atomic_json(acquisition_path, report)
        _write_atomic_json(product_path, compose_product_status(report["status"]))
    except BaseException:
        _remove_quality_reports(quality_directory, final_paths)
        raise


def _remove_quality_reports(directory: Path, paths: tuple[Path, ...]) -> None:
    removed = False
    for path in paths:
        try:
            path.unlink()
            removed = True
        except FileNotFoundError:
            pass
    if removed:
        _fsync_directory(directory)


def _fsync_directory(path):
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def compose_product_status(acquisition_timing: str) -> dict[str, str]:
    if acquisition_timing not in {"PASS", "FAIL", "BLOCKED"}:
        raise ValueError("invalid acquisition_timing status")
    pending = "BLOCKED/phase_not_delivered"
    return {
        "acquisition_timing": acquisition_timing,
        "ego_slam": pending,
        "markerless_tracking": pending,
        "fusion": pending,
        "spatial_accuracy": pending,
        "overall": "BLOCKED" if acquisition_timing == "PASS" else acquisition_timing,
    }


def exit_code_for_status(status):
    try:
        return {"PASS": 0, "FAIL": 2, "BLOCKED": 3}[status]
    except KeyError as exc:
        raise ValueError(f"unknown acceptance status: {status}") from exc


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Verify a completed three-device acquisition session offline."
    )
    parser.add_argument("--session", required=True, type=Path, help="session root")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    published = False
    try:
        report = verify_session(args.session)
        published = True
    except Exception as exc:
        sections = {name: Section() for name in REQUIRED_SECTIONS}
        sections["storage"].fail(f"verification_exception:{type(exc).__name__}")
        report = _finish(sections)
    try:
        if not published:
            _publish_quality_reports(args.session, report)
    except (OSError, ValueError) as exc:
        print(f"three-device verification failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "reasons": report["reasons"]}, sort_keys=True))
    return exit_code_for_status(report["status"])


if __name__ == "__main__":
    raise SystemExit(main())
