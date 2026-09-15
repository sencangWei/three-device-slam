#!/usr/bin/env python3
"""Verify one D435i-Ego + single-UMI (two-device) acquisition session offline.

This is the pair-topology counterpart of ``verify_session``.  It never claims
three-device product acceptance: the published schema, required topology and
device set are explicitly two-device, and the product status keeps every
undelivered spatial layer BLOCKED.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import sys
from collections import Counter
from collections.abc import Mapping
from pathlib import Path
from zlib import crc32


from three_device_slam.quality import verify_session as three_device_verifier
from three_device_slam.synchronization import build_index as sync_builder
from three_device_slam.synchronization.sync_index import (
    GRID_NS,
    HARD_SPAN_NS,
    TARGET_SPAN_NS,
    build_pairs,
)


SCHEMA = "ego.two_device.acceptance.v1"
ROLE_SCHEMA = "ego.two_device.acceptance.v2"
EGO_STREAMS = ("ego.ir_left", "ego.ir_right", "ego.gyro", "ego.accel")
EGO_ACCEPTANCE_SCHEMA = "ego.d435i.acceptance.v1"
UMI_ACCEPTANCE_SCHEMA = "umi.d405.append_only.acceptance.v1"
D435I_CLOCK_DOMAIN = sync_builder.D435I_CLOCK_DOMAIN
EGO_INPUT_CONFIG = {
    "source": "ego/ego.ir_left.jsonl",
    "stream_id": "ego.ir_left",
    "clock_domain": D435I_CLOCK_DOMAIN,
    "timestamp_field": "acquisition_ns",
    "formal_filter": "warmup=false,valid=true",
}


def _expected_pair_input_config(umi_role):
    return {
        "ego": EGO_INPUT_CONFIG,
        umi_role: {
            "source": f"{umi_role}/{umi_role}.ir_left.jsonl",
            "stream_id": f"{umi_role}.ir_left",
            "clock_domain": D435I_CLOCK_DOMAIN,
            "timestamp_field": "acquisition_ns",
            "formal_filter": "warmup=false,valid=true",
        },
    }


def _required_files(umi_role):
    return {
        "coordinator": "coordinator.json",
        "ego_acceptance": "ego/acceptance.json",
        "umi_acceptance": f"{umi_role}/acceptance.json",
        "ego_manifest": "ego/manifest.json",
        "umi_manifest": f"{umi_role}/manifest.json",
        "sync_manifest": "sync/manifest.json",
        "sync_csv": "sync/common_30hz.csv",
    }


def _required_sections(umi_role):
    return (
        "provenance",
        "ego",
        umi_role,
        "joint_gate",
        "clock_domains",
        "sync_index",
        "storage",
    )


LEGACY_EXPECTED_PAIR_INPUT_CONFIG = {
    "ego": {
        "source": "ego/ego.ir_left.jsonl",
        "stream_id": "ego.ir_left",
        "clock_domain": D435I_CLOCK_DOMAIN,
        "timestamp_field": "acquisition_ns",
        "formal_filter": "warmup=false,valid=true",
    },
    "left": {
        "source": "left/left.ir_left.jsonl",
        "stream_id": "left.ir_left",
        "clock_domain": D435I_CLOCK_DOMAIN,
        "timestamp_field": "acquisition_ns",
        "formal_filter": "warmup=false,valid=true",
    },
}


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


def _evaluate_pair_session(session: Path) -> dict:
    session = Path(session)
    paths = {}
    missing_derived = set()
    documents = {}

    try:
        session_root = session.resolve(strict=True)
        if not session_root.is_dir():
            raise ValueError("session root is not a directory")
    except (OSError, ValueError) as exc:
        sections = {name: Section() for name in _required_sections("left")}
        sections["storage"].fail(f"session_root:{type(exc).__name__}")
        return _finish(sections, _required_sections("left"), "left")

    umi_role = _infer_umi_role(session_root)
    required_files = _required_files(umi_role)
    required_sections = _required_sections(umi_role)
    sections = {name: Section() for name in required_sections}

    for label, relative in required_files.items():
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
        "umi_acceptance": umi_role,
        "ego_manifest": "storage",
        "umi_manifest": "storage",
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
    ego_acceptance = documents.get("ego_acceptance")
    umi_acceptance = documents.get("umi_acceptance")
    sync_manifest = documents.get("sync_manifest")

    if missing_derived:
        sections["sync_index"].block("derived_sync_unavailable")
        sections["storage"].block("derived_sync_unavailable")
    _check_joint_gate(
        sections["joint_gate"], coordinator, ego_acceptance, umi_acceptance, umi_role
    )
    _check_clock_domains(
        sections["clock_domains"], session_root, ego_acceptance, umi_acceptance, umi_role
    )
    _check_provenance(
        sections["provenance"], coordinator, sync_manifest, ego_acceptance,
        umi_acceptance, umi_role
    )
    _check_ego(
        sections["ego"],
        sections["storage"],
        session_root,
        ego_acceptance,
        documents.get("ego_manifest"),
    )
    _check_umi(
        sections[umi_role],
        sections["storage"],
        session_root,
        umi_acceptance,
        documents.get("umi_manifest"),
        umi_role,
    )
    _check_sync_index(
        sections["sync_index"],
        sections["storage"],
        session_root,
        paths,
        sync_manifest,
        coordinator,
        missing_derived,
        umi_role,
    )
    sections["storage"].evidence["required_files"] = sorted(required_files.values())
    return _finish(sections, required_sections, umi_role)


def verify_pair_session(session: Path) -> dict:
    with sync_builder._offline_session_lease(session) as session_root:
        boundary_error = None
        seal = None
        try:
            seal = sync_builder._load_existing_session_seal(session_root)
            sync_directory = session_root / "sync"
            if sync_builder.assert_safe_path(
                sync_directory, session_root, "sync", kind="directory"
            ) is not None:
                sync_builder._validate_existing_index(session_root, seal)
        except Exception as exc:  # noqa: BLE001 - boundary must fail closed
            boundary_error = exc
        try:
            report = _evaluate_pair_session(session_root)
        except Exception as exc:  # noqa: BLE001 - verification must fail closed
            try:
                umi_role = _infer_umi_role(session_root)
            except ValueError:
                umi_role = "left"
            required_sections = _required_sections(umi_role)
            sections = {name: Section() for name in required_sections}
            sections["storage"].fail(f"verification_exception:{type(exc).__name__}")
            report = _finish(sections, required_sections, umi_role)
        if boundary_error is not None:
            _force_protocol_failure(report, boundary_error)
        three_device_verifier._publish_quality_reports(
            session_root,
            report,
            seal if boundary_error is None else None,
        )
        return report


def _infer_umi_role(session):
    present = [
        role for role in ("left", "right")
        if (session / role / "acceptance.json").is_file()
    ]
    if len(present) != 1:
        raise ValueError("pair session must contain exactly one UMI role")
    return present[0]


def _check_provenance(
    section, coordinator, sync_manifest, ego_acceptance, umi_acceptance, umi_role
):
    if coordinator is None:
        section.fail("provenance_inputs_unavailable")
        return
    inputs = [("coordinator", coordinator)]
    if sync_manifest is None:
        section.block("sync_provenance_unavailable")
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
        return
    for device, actual in (("ego", ego_acceptance), (umi_role, umi_acceptance)):
        if actual is not None and embedded.get(device) != actual:
            section.fail(f"worker_acceptance_mismatch:{device}")


def _check_joint_gate(section, coordinator, ego_acceptance, umi_acceptance, umi_role):
    if coordinator is None:
        section.fail("coordinator_unavailable")
        return
    try:
        if coordinator.get("schema") != sync_builder.COORDINATOR_SCHEMA:
            raise ValueError("coordinator.schema")
        if coordinator.get("capture_topology") != "single_umi":
            raise ValueError("coordinator.capture_topology")
        status = _status(coordinator.get("status"), "coordinator.status")
        if status == "FAIL":
            section.fail(f"coordinator_fail:{coordinator.get('reason', 'unknown')}")
        elif status == "BLOCKED":
            section.block(f"coordinator_blocked:{coordinator.get('reason', 'unknown')}")
        workers = _mapping(coordinator.get("workers"), "coordinator.workers")
        pair_devices = ("ego", umi_role)
        if set(workers) != set(pair_devices):
            raise ValueError("coordinator.workers.devices")
        for device in pair_devices:
            worker = _mapping(workers[device], f"coordinator.workers.{device}")
            if worker.get("exit_code") != 0:
                section.fail(f"worker_exit:{device}")
            if worker.get("early_exit") is not False:
                section.fail(f"worker_early_exit:{device}")
            if worker.get("observed_running_at_or_after_formal_end") is not True:
                section.fail(f"worker_incomplete:{device}")
        transitions = coordinator.get("transitions")
        if not isinstance(transitions, list) or not transitions:
            raise ValueError("coordinator.transitions")
        states = [item.get("state") for item in transitions if isinstance(item, Mapping)]
        if "JOINT_READY" not in states or "RECORDING" not in states:
            section.fail("joint_ready_or_recording_missing")
        scheduled = _nonnegative_int(
            coordinator.get("scheduled_start_ns"), "scheduled_start_ns"
        )
        task_start = _nonnegative_int(
            coordinator.get("task_start_ego_frame_ns"), "task_start_ego_frame_ns"
        )
        if task_start < scheduled:
            section.fail("task_start_before_scheduled_start")
        expectations = _mapping(
            coordinator.get("calibration_expectations"),
            "coordinator.calibration_expectations",
        )
        if set(expectations) != set(pair_devices):
            raise ValueError("calibration_expectations.devices")
        observed_ids = {
            "ego": (
                ego_acceptance.get("calibration_id")
                if isinstance(ego_acceptance, Mapping)
                else None
            ),
            umi_role: (
                umi_acceptance.get("calibration_id")
                if isinstance(umi_acceptance, Mapping)
                else None
            ),
        }
        for device in pair_devices:
            item = _mapping(expectations[device], f"calibration_expectations.{device}")
            if item.get("status") != "PASS" or item.get("observed") != observed_ids[device]:
                section.fail(f"calibration_expectation:{device}")
        section.measurements.update(
            scheduled_start_ns=scheduled, task_start_ego_frame_ns=task_start
        )
    except ValueError as exc:
        section.fail(exc)


def _check_clock_domains(section, session, ego_acceptance, umi_acceptance, umi_role):
    """Record the common-clock evidence for the pair topology."""
    try:
        for device_dir, stream_id in (
            ("ego", "ego.ir_left"),
            (umi_role, f"{umi_role}.ir_left"),
        ):
            path = _safe_regular_file(session, f"{device_dir}/{stream_id}.jsonl")
            rows = 0
            for line in path.read_text(encoding="utf-8").splitlines():
                row = json.loads(line)
                if not isinstance(row, Mapping):
                    raise ValueError(f"{stream_id} row must be an object")
                if row.get("clock_domain") != D435I_CLOCK_DOMAIN:
                    raise ValueError(f"{stream_id} clock_domain")
                rows += 1
            if rows < 1:
                section.block(f"clock_rows_missing:{device_dir}")
            section.evidence[f"{device_dir}_clock_domain"] = D435I_CLOCK_DOMAIN
        ego_joint = (
            ego_acceptance.get("joint_start", {})
            if isinstance(ego_acceptance, Mapping)
            else {}
        )
        umi_joint = (
            umi_acceptance.get("joint_start", {})
            if isinstance(umi_acceptance, Mapping)
            else {}
        )
        section.evidence["domains"] = {
            "ego": D435I_CLOCK_DOMAIN,
            umi_role: (
                umi_joint.get("clock_domain")
                if isinstance(umi_joint, Mapping)
                else None
            ),
        }
        if not isinstance(umi_joint, Mapping) or umi_joint.get(
            "clock_domain"
        ) != "host_monotonic":
            section.block(f"{umi_role}_joint_clock_unverified")
    except (OSError, UnicodeError, ValueError) as exc:
        section.fail(f"clock_validation:{exc}")


def _check_ego(section, storage, session, acceptance, manifest):
    if acceptance is None:
        section.fail("ego_acceptance_unavailable")
        return
    try:
        if acceptance.get("schema") != EGO_ACCEPTANCE_SCHEMA:
            raise ValueError("ego.schema")
        status = _status(acceptance.get("status"), "ego.status")
        if status == "FAIL":
            section.fail(f"worker_fail:{acceptance.get('reason', 'ego_acceptance')}")
        elif status == "BLOCKED":
            section.block(f"worker_blocked:{acceptance.get('reason', 'ego_acceptance')}")
        if not isinstance(acceptance.get("calibration_id"), str) or not acceptance[
            "calibration_id"
        ].strip():
            raise ValueError("ego.calibration_id")
        streams = _mapping(acceptance.get("streams"), "ego.streams")
        if set(streams) != {"ir_left", "ir_right", "gyro", "accel"}:
            raise ValueError("ego.streams.set")
        rates = {}
        for name in ("ir_left", "ir_right", "gyro", "accel"):
            stream = _mapping(streams[name], f"ego.streams.{name}")
            if stream.get("status") != "PASS":
                section.fail(f"ego_stream:{name}")
            rates[name] = _finite_number(
                stream.get("measured_hz"), f"ego.streams.{name}.measured_hz"
            )
            if _nonnegative_int(
                stream.get("sequence_gaps"), f"ego.streams.{name}.sequence_gaps"
            ):
                section.fail(f"ego_stream_gaps:{name}")
            if _nonnegative_int(
                stream.get("sequence_regressions"),
                f"ego.streams.{name}.sequence_regressions",
            ):
                section.fail(f"ego_stream_regressions:{name}")
            for domain in stream.get("timestamp_domains", []):
                if domain != "global_time":
                    section.fail(f"ego_stream_domain:{name}")
        pairing = _mapping(acceptance.get("stereo_pairing"), "ego.stereo_pairing")
        if pairing.get("status") != "PASS":
            section.fail("ego_stereo_pairing")
        if pairing.get("unmatched_frames") != 0:
            section.fail("ego_stereo_unmatched")
        if _nonnegative_int(
            acceptance.get("writer_queue_drops"), "ego.writer_queue_drops"
        ):
            section.fail("ego_writer_queue_drops")
        formal = _mapping(acceptance.get("formal_evidence"), "ego.formal_evidence")
        first_ns = _nonnegative_int(
            formal.get("first_acquisition_ns"), "ego.first_acquisition_ns"
        )
        raw_formal = _summarize_stream_rows(session, "ego", "ego.ir_left")
        if raw_formal < 2:
            section.fail("ego_raw_formal_missing")
        section.evidence["ego_formal_ir_left_rows"] = raw_formal
        section.measurements.update(
            ego_ir_rate_hz=rates["ir_left"], ego_imu_rate_hz=rates["gyro"]
        )
        if manifest is not None:
            _verify_manifest_streams(storage, session, manifest, "ego", EGO_STREAMS)
    except ValueError as exc:
        section.fail(exc)


def _check_umi(section, storage, session, acceptance, manifest, umi_role):
    if acceptance is None:
        section.fail(f"{umi_role}_acceptance_unavailable")
        return
    try:
        if acceptance.get("schema") != UMI_ACCEPTANCE_SCHEMA:
            raise ValueError(f"{umi_role}.schema")
        status = _status(acceptance.get("result"), f"{umi_role}.result")
        if status == "FAIL":
            section.fail(f"worker_fail:{acceptance.get('reason', f'{umi_role}_acceptance')}")
        elif status == "BLOCKED":
            section.block(f"worker_blocked:{acceptance.get('reason', f'{umi_role}_acceptance')}")
        if not isinstance(acceptance.get("calibration_id"), str) or not acceptance[
            "calibration_id"
        ].strip():
            raise ValueError(f"{umi_role}.calibration_id")
        live_vins = _mapping(acceptance.get("live_vins"), f"{umi_role}.live_vins")
        if live_vins.get("imu_calibration") != acceptance["calibration_id"]:
            section.fail(f"{umi_role}.imu_calibration_mismatch")
        if acceptance.get("capture_error") is not None:
            section.fail(f"{umi_role}.capture_error")
        if _nonnegative_int(
            acceptance.get("writer_queue_drops"), f"{umi_role}.writer_queue_drops"
        ):
            section.fail(f"{umi_role}_writer_queue_drops")
        clock = _mapping(acceptance.get("camera_clock"), f"{umi_role}.camera_clock")
        if clock.get("verified") is not True:
            section.block(f"{umi_role}_camera_clock_unverified")
        streams = _mapping(acceptance.get("camera_streams"), f"{umi_role}.camera_streams")
        if set(streams) != {"color", "ir_left", "ir_right"}:
            raise ValueError("left.camera_streams.set")
        rates = {}
        for name in ("color", "ir_left", "ir_right"):
            stream = _mapping(streams[name], f"{umi_role}.camera_streams.{name}")
            if stream.get("status") != "PASS":
                section.fail(f"{umi_role}_stream:{name}")
            rates[name] = _finite_number(
                stream.get("measured_hz"), f"{umi_role}.camera_streams.{name}.measured_hz"
            )
            if _nonnegative_int(
                stream.get("sequence_gaps"), f"{umi_role}.camera_streams.{name}.sequence_gaps"
            ):
                section.fail(f"{umi_role}_stream_gaps:{name}")
            for domain in stream.get("timestamp_domains", []):
                if domain != "global_time":
                    section.fail(f"{umi_role}_stream_domain:{name}")
        imu = _mapping(acceptance.get("imu"), f"{umi_role}.imu")
        if imu.get("result") != "PASS":
            section.fail(f"{umi_role}_imu_result")
        imu_rate = _finite_number(imu.get("rate_hz"), f"{umi_role}.imu.rate_hz")
        if not 399.0 <= imu_rate <= 401.0:
            section.fail(f"{umi_role}_imu_rate")
        if _nonnegative_int(imu.get("recorder_drops"), f"{umi_role}.imu.recorder_drops"):
            section.fail(f"{umi_role}_imu_recorder_drops")
        if imu.get("formal_samples") is None:
            section.fail(f"{umi_role}_imu_formal_samples_missing")
        joint = _mapping(acceptance.get("joint_start"), f"{umi_role}.joint_start")
        if joint.get("clock_domain") != "host_monotonic":
            section.block(f"{umi_role}_joint_clock_unverified")
        raw_formal = _summarize_stream_rows(
            session, umi_role, f"{umi_role}.ir_left"
        )
        if raw_formal < 2:
            section.fail(f"{umi_role}_raw_formal_missing")
        section.measurements.update(
            **{
                f"{umi_role}_ir_rate_hz": rates["ir_left"],
                f"{umi_role}_imu_rate_hz": imu_rate,
            }
        )
        if manifest is not None:
            _verify_manifest_streams(
                storage,
                session,
                manifest,
                umi_role,
                tuple(f"{umi_role}.{name}" for name in ("color", "ir_left", "ir_right")),
            )
    except ValueError as exc:
        section.fail(exc)


def _check_sync_index(
    section, storage, session, paths, manifest, coordinator, missing_derived, umi_role
):
    if missing_derived:
        return
    if manifest is None or coordinator is None:
        section.fail("sync_inputs_unavailable")
        return
    try:
        expected_schema = (
            sync_builder.PAIR_SCHEMA
            if umi_role == "left"
            else sync_builder.PAIR_ROLE_SCHEMA
        )
        if manifest.get("schema") != expected_schema:
            raise ValueError("sync.schema")
        if manifest.get("capture_topology") != sync_builder.TOPOLOGY_PAIR:
            raise ValueError("sync.capture_topology")
        for name, expected in (
            ("grid_ns", GRID_NS),
            ("target_span_ns", TARGET_SPAN_NS),
            ("hard_span_ns", HARD_SPAN_NS),
        ):
            if _nonnegative_int(manifest.get(name), f"sync.{name}") != expected:
                raise ValueError(f"sync.{name}")
        expected_input = (
            LEGACY_EXPECTED_PAIR_INPUT_CONFIG
            if umi_role == "left"
            else _expected_pair_input_config(umi_role)
        )
        if manifest.get("input_config") != expected_input:
            raise ValueError("sync.input_config")
        if umi_role == "right" and manifest.get("umi_role") != "right":
            raise ValueError("sync.umi_role")
        task_start = _nonnegative_int(
            coordinator.get("task_start_ego_frame_ns"), "task_start_ego_frame_ns"
        )
        if _nonnegative_int(
            manifest.get("task_start_ego_frame_ns"), "sync.task_start"
        ) != task_start:
            raise ValueError("sync.task_start_mismatch")
        source_hashes = _mapping(manifest.get("source_sha256"), "sync.source_sha256")
        expected_sources = (
            sync_builder.PAIR_SOURCE_PATHS
            if umi_role == "left"
            else sync_builder.PAIR_RIGHT_SOURCE_PATHS
        )
        if set(source_hashes) != set(expected_sources):
            raise ValueError("sync.source_sha256.paths")
        for relative, expected_hash in source_hashes.items():
            _hash(expected_hash, f"sync.source_sha256.{relative}")
            source = _safe_regular_file(session, relative)
            if _sha256(source) != expected_hash:
                storage.fail(f"sync_source_hash_mismatch:{relative}")
                section.fail(f"source_hash_mismatch:{relative}")
        if "sync_csv" not in paths:
            raise ValueError("sync.csv_missing")
        expected_output_hash = _hash(
            manifest.get("output_csv_sha256"), "sync.output_hash"
        )
        if _sha256(paths["sync_csv"]) != expected_output_hash:
            storage.fail("sync_output_hash_mismatch")
            section.fail("output_hash_mismatch")
        ego = sync_builder._parse_d435i_ego_rows(
            _safe_regular_file(session, "ego/ego.ir_left.jsonl").read_bytes(),
            task_start,
        )
        umi = sync_builder._parse_pair_umi_rows(
            _safe_regular_file(
                session, f"{umi_role}/{umi_role}.ir_left.jsonl"
            ).read_bytes(),
            umi_role,
            task_start,
        )
        expected_pairs = build_pairs(ego, umi, umi_role)
        csv_header = (
            sync_builder.PAIR_CSV_HEADER
            if umi_role == "left"
            else sync_builder.PAIR_ROLE_CSV_HEADER
        )
        rows = _strict_csv(paths["sync_csv"], csv_header)
        if len(rows) != len(expected_pairs):
            section.fail("pair_row_count_mismatch")
        reasons = Counter()
        trainable_count = 0
        for index, (row, expected) in enumerate(zip(rows, expected_pairs), start=2):
            actual = _parse_pair_row(row, index, umi_role)
            expected_values = {
                "sample_ns": expected.sample_ns,
                "ego_sequence": expected.ego.sequence,
                "ego_acquisition_ns": expected.ego.acquisition_ns,
                "ego_ref": expected.ego.payload_ref,
                "span_ns": expected.span_ns,
                "trainable": expected.trainable,
                "reason": expected.reason,
            }
            if umi_role == "left":
                expected_values.update(
                    left_sequence=expected.left.sequence,
                    left_acquisition_ns=expected.left.acquisition_ns,
                    left_ref=expected.left.payload_ref,
                )
            else:
                expected_values.update(
                    umi_role=umi_role,
                    umi_sequence=expected.left.sequence,
                    umi_acquisition_ns=expected.left.acquisition_ns,
                    umi_ref=expected.left.payload_ref,
                )
            if actual != expected_values:
                section.fail(f"pair_mismatch:line_{index}")
            if actual["span_ns"] > HARD_SPAN_NS and actual["trainable"]:
                section.fail(f"over_hard_trainable:line_{index}")
            reasons[actual["reason"]] += 1
            trainable_count += int(actual["trainable"])
        stored_reasons = _mapping(manifest.get("reason_counts"), "sync.reason_counts")
        expected_reason_counts = {
            reason: reasons[reason] for reason in sync_builder.REASONS
        }
        if stored_reasons != expected_reason_counts:
            section.fail("reason_counts_mismatch")
        if _nonnegative_int(manifest.get("row_count"), "sync.row_count") != len(rows):
            section.fail("manifest_row_count_mismatch")
        if _nonnegative_int(
            manifest.get("trainable_count"), "sync.trainable_count"
        ) != trainable_count:
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


def _parse_pair_row(row, line_number, umi_role="left"):
    result = {}
    umi_sequence = "left_sequence" if umi_role == "left" else "umi_sequence"
    umi_acquisition = (
        "left_acquisition_ns" if umi_role == "left" else "umi_acquisition_ns"
    )
    for name in (
        "sample_ns", "ego_sequence", "ego_acquisition_ns",
        umi_sequence, umi_acquisition, "span_ns",
    ):
        result[name] = _csv_nonnegative_int(
            row[name], f"pairs line {line_number} {name}"
        )
    umi_ref = "left_ref" if umi_role == "left" else "umi_ref"
    for name in ("ego_ref", umi_ref):
        value = row[name]
        if not value:
            raise ValueError(f"pairs line {line_number} {name}")
        result[name] = value
    if row["trainable"] not in {"0", "1"}:
        raise ValueError(f"pairs line {line_number} trainable")
    result["trainable"] = row["trainable"] == "1"
    if row["reason"] not in sync_builder.REASONS:
        raise ValueError(f"pairs line {line_number} reason")
    result["reason"] = row["reason"]
    if umi_role == "right":
        if row.get("umi_role") != "right":
            raise ValueError(f"pairs line {line_number} umi_role")
        result["umi_role"] = "right"
    timestamps = sorted(
        (result["ego_acquisition_ns"], result[umi_acquisition])
    )
    if (
        result["sample_ns"]
        != (timestamps[0] + timestamps[-1]) // 2
        or result["span_ns"] != timestamps[-1] - timestamps[0]
    ):
        raise ValueError(f"pairs line {line_number} timing")
    if result["span_ns"] <= TARGET_SPAN_NS:
        classification = (True, "within_target")
    elif result["span_ns"] <= HARD_SPAN_NS:
        classification = (True, "within_hard_limit")
    else:
        classification = (False, "span_over_hard_limit")
    if (result["trainable"], result["reason"]) != classification:
        raise ValueError(f"pairs line {line_number} classification")
    return result


def _summarize_stream_rows(session, device_dir, stream_id):
    path = _safe_regular_file(session, f"{device_dir}/{stream_id}.jsonl")
    formal_rows = 0
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{stream_id} line {line_number} malformed") from exc
        if not isinstance(row, Mapping):
            raise ValueError(f"{stream_id} line {line_number} must be an object")
        if row.get("stream_id") != stream_id:
            raise ValueError(f"{stream_id} line {line_number} stream_id")
        if row.get("clock_domain") != D435I_CLOCK_DOMAIN:
            raise ValueError(f"{stream_id} line {line_number} clock_domain")
        if not isinstance(row.get("warmup"), bool) or not isinstance(
            row.get("valid"), bool
        ):
            raise ValueError(f"{stream_id} line {line_number} warmup/valid")
        if not row["warmup"] and row["valid"]:
            formal_rows += 1
    return formal_rows


def _verify_manifest_streams(storage, session, manifest, device_dir, stream_ids):
    try:
        if manifest.get("schema") != "ego.three_device.raw_session.v1":
            raise ValueError(f"{device_dir}.manifest.schema")
        streams = _mapping(manifest.get("streams"), f"{device_dir}.manifest.streams")
        for stream_id in stream_ids:
            entry = _mapping(streams.get(stream_id), f"{device_dir}.{stream_id}")
            for suffix, hash_name in (("bin", "payload_sha256"), ("jsonl", "index_sha256")):
                expected_hash = _hash(entry.get(hash_name), f"{stream_id}.{hash_name}")
                raw_path = _safe_regular_file(
                    session, f"{device_dir}/{stream_id}.{suffix}"
                )
                if _sha256(raw_path) != expected_hash:
                    storage.fail(f"raw_hash_mismatch:{stream_id}.{suffix}")
            _verify_payload_crc(session, device_dir, stream_id)
    except ValueError as exc:
        storage.fail(f"manifest_validation:{exc}")


def _verify_payload_crc(session, device_dir, stream_id):
    index_path = _safe_regular_file(session, f"{device_dir}/{stream_id}.jsonl")
    payload_path = _safe_regular_file(session, f"{device_dir}/{stream_id}.bin")
    cursor = 0
    with payload_path.open("rb") as payload:
        for line_number, line in enumerate(
            index_path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            row = json.loads(line)
            offset = _nonnegative_int(row.get("offset"), f"{stream_id}.offset")
            size = _nonnegative_int(row.get("size"), f"{stream_id}.size")
            stored_crc = _nonnegative_int(row.get("crc32"), f"{stream_id}.crc32")
            if offset != cursor:
                raise ValueError(f"{stream_id} payload coverage line {line_number}")
            payload.seek(offset)
            raw = payload.read(size)
            if len(raw) != size:
                raise ValueError(f"{stream_id} payload truncated line {line_number}")
            if crc32(raw) & 0xFFFFFFFF != stored_crc:
                raise ValueError(f"{stream_id} payload crc line {line_number}")
            cursor += size
    if payload_path.stat().st_size != cursor:
        raise ValueError(f"{stream_id} payload has unindexed bytes")


def _strict_csv(path, header):
    text = path.read_text(encoding="utf-8")
    rows = list(csv.reader(io.StringIO(text, newline="")))
    if not rows or tuple(rows[0]) != tuple(header):
        raise ValueError(f"{path.name} header")
    if any(len(row) != len(header) for row in rows[1:]):
        raise ValueError(f"{path.name} row width")
    return [dict(zip(header, row)) for row in rows[1:]]


def _finish(sections, required_sections, umi_role="left"):
    reports = {name: sections[name].report() for name in required_sections}
    status = max(
        (report["status"] for report in reports.values()),
        key=three_device_verifier.STATUS_ORDER.get,
    )
    reasons = sorted(
        f"{name}:{reason}"
        for name, report in reports.items()
        if report["status"] == status
        for reason in report["reasons"]
    )
    payload = {
        "schema": SCHEMA if umi_role == "left" else ROLE_SCHEMA,
        "status": status,
        "reasons": reasons,
        "sections": reports,
    }
    if umi_role == "right":
        payload["umi_role"] = umi_role
    return payload


def _force_protocol_failure(report: dict, error: Exception) -> None:
    three_device_verifier._force_protocol_failure(report, error)


def _read_json_object(path, label):
    return three_device_verifier._read_json_object(path, label)


def _safe_regular_file(session, relative):
    return three_device_verifier._safe_regular_file(session, relative)


def _mapping(value, name):
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _status(value, name):
    if not isinstance(value, str) or value not in three_device_verifier.STATUS_ORDER:
        raise ValueError(f"{name} must be PASS, FAIL, or BLOCKED")
    return value


def _finite_number(value, name):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite")
    return value


def _nonnegative_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _csv_nonnegative_int(value, name):
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a non-negative integer") from exc
    if str(parsed) != value or parsed < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return parsed


def _hash(value, name):
    return three_device_verifier._hash(value, name)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Verify a completed two-device (D435i Ego + one UMI) session offline."
    )
    parser.add_argument("--session", required=True, type=Path, help="session root")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        report = verify_pair_session(args.session)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"two-device verification failed: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {"status": report["status"], "reasons": report["reasons"]},
            sort_keys=True,
        )
    )
    return three_device_verifier.exit_code_for_status(report["status"])


if __name__ == "__main__":
    raise SystemExit(main())
