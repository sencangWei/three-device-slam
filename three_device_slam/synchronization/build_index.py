#!/usr/bin/env python3
"""Build the deterministic 30 Hz synchronization index for one raw session."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
import subprocess
import sys
import uuid
from collections import Counter
from contextlib import ExitStack, contextmanager
from decimal import Decimal, InvalidOperation, localcontext
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]

from three_device_slam.core.model import FrameStamp, Triplet
from three_device_slam.core.session_lifecycle import (
    SEAL_NAME,
    assert_safe_path,
    canonical_session_root,
    offline_claim,
)
from three_device_slam.synchronization.sync_index import (
    GRID_NS,
    HARD_SPAN_NS,
    TARGET_SPAN_NS,
    build_pairs,
    build_triplets,
)


SCHEMA = "ego.three_device.sync_index.v1"
PAIR_SCHEMA = "ego.two_device.sync_index.v1"
PAIR_ROLE_SCHEMA = "ego.two_device.sync_index.v2"
SEAL_SCHEMA = "three-device-slam.session-seal.v1"
COORDINATOR_SCHEMA = "ego.three_device.coordinator.v1"
SOURCE_PATHS = (
    "coordinator.json",
    "ego/ego.video.jsonl",
    "left/acceptance.json",
    "left/d405_frames.csv",
    "right/acceptance.json",
    "right/d405_frames.csv",
)
PAIR_SOURCE_PATHS = (
    "coordinator.json",
    "ego/ego.ir_left.jsonl",
    "left/acceptance.json",
    "left/left.ir_left.jsonl",
)
PAIR_RIGHT_SOURCE_PATHS = (
    "coordinator.json",
    "ego/ego.ir_left.jsonl",
    "right/acceptance.json",
    "right/right.ir_left.jsonl",
)
D435I_EGO_STREAM_ID = "ego.ir_left"
D435I_CLOCK_DOMAIN = "realsense_global_time_mapped_to_host_monotonic"
TOPOLOGY_LEGACY = "legacy_three_device"
TOPOLOGY_PAIR = "d435i_single_umi"
CSV_HEADER = (
    "sample_ns",
    "ego_sequence",
    "ego_acquisition_ns",
    "ego_ref",
    "left_sequence",
    "left_acquisition_ns",
    "left_ref",
    "right_sequence",
    "right_acquisition_ns",
    "right_ref",
    "span_ns",
    "trainable",
    "reason",
)
PAIR_CSV_HEADER = (
    "sample_ns",
    "ego_sequence",
    "ego_acquisition_ns",
    "ego_ref",
    "left_sequence",
    "left_acquisition_ns",
    "left_ref",
    "span_ns",
    "trainable",
    "reason",
)
PAIR_ROLE_CSV_HEADER = (
    "sample_ns",
    "ego_sequence",
    "ego_acquisition_ns",
    "ego_ref",
    "umi_role",
    "umi_sequence",
    "umi_acquisition_ns",
    "umi_ref",
    "span_ns",
    "trainable",
    "reason",
)
REASONS = ("within_target", "within_hard_limit", "span_over_hard_limit")


def _detect_topology(session: Path) -> str:
    if (session / "ego/ego.video.jsonl").is_file():
        return TOPOLOGY_LEGACY
    if (session / "ego/ego.ir_left.jsonl").is_file():
        return TOPOLOGY_PAIR
    raise ValueError(
        "session topology is unrecognized: missing ego/ego.video.jsonl "
        "and ego/ego.ir_left.jsonl"
    )


def _source_paths(topology: str) -> tuple[str, ...]:
    if topology == TOPOLOGY_LEGACY:
        return SOURCE_PATHS
    if topology == TOPOLOGY_PAIR:
        return PAIR_SOURCE_PATHS
    raise ValueError(f"unsupported session topology: {topology}")


def build_index(session: Path) -> dict:
    """Build and durably publish one session index, returning its manifest."""
    with _offline_session_lease(session) as session_root:
        _reject_active_writer_claims(session_root)
        seal = _ensure_session_seal(session_root)
        output_directory = session_root / "sync"
        existing = assert_safe_path(
            output_directory, session_root, "sync", kind="directory"
        )
        if existing is not None:
            return _validate_existing_index(session_root, seal)
        return _build_index_locked(session_root, seal)


def _build_index_locked(session: Path, seal: dict) -> dict:
    source_bytes = _read_sealed_sources(session, seal)
    csv_bytes, manifest = _build_artifacts(source_bytes, seal, _git_provenance())
    manifest_bytes = (
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    output_directory = session / "sync"
    temporary_directory = session / f".sync.{uuid.uuid4().hex}.tmp"
    temporary_directory.mkdir()
    published = False
    try:
        _write_directory_member(
            temporary_directory, "common_30hz.csv", csv_bytes
        )
        _write_directory_member(temporary_directory, "manifest.json", manifest_bytes)
        _fsync_directory(temporary_directory)
        _verify_sources_against_seal(session, seal)
        os.replace(temporary_directory, output_directory)
        published = True
        _fsync_directory(session)
        _validate_existing_index(session, seal)
    except BaseException:
        if published:
            _remove_published_directory(session, output_directory)
        raise
    finally:
        if temporary_directory.exists():
            _remove_published_directory(session, temporary_directory)
    return manifest


def _build_artifacts(
    source_bytes: dict[str, bytes], seal: dict, git: dict
) -> tuple[bytes, dict]:
    topology = _seal_topology(seal)
    source_sha256 = {
        relative: seal["sources"][relative]["sha256"]
        for relative in seal["sources"]
    }
    task_start_ns = _parse_coordinator(source_bytes["coordinator.json"])
    if topology == TOPOLOGY_PAIR:
        return _build_pair_artifacts(
            source_bytes, seal, git, source_sha256, task_start_ns
        )
    ego = _parse_ego_rows(source_bytes["ego/ego.video.jsonl"], task_start_ns)
    _validate_d405_clock(
        source_bytes["left/acceptance.json"],
        source_bytes["left/d405_frames.csv"],
        "left",
    )
    left = _parse_d405_rows(
        source_bytes["left/d405_frames.csv"], "left", task_start_ns
    )
    _validate_d405_clock(
        source_bytes["right/acceptance.json"],
        source_bytes["right/d405_frames.csv"],
        "right",
    )
    right = _parse_d405_rows(
        source_bytes["right/d405_frames.csv"], "right", task_start_ns
    )
    triplets = build_triplets(ego, left, right)
    csv_bytes = _serialize_csv(triplets)
    overlap = _common_overlap(ego, left, right)
    reason_counts = Counter(row.reason for row in triplets)
    manifest = {
        "schema": SCHEMA,
        "capture_topology": TOPOLOGY_LEGACY,
        "source_sha256": source_sha256,
        "grid_ns": GRID_NS,
        "target_span_ns": TARGET_SPAN_NS,
        "hard_span_ns": HARD_SPAN_NS,
        "input_config": {
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
        },
        "common_overlap": overlap,
        "row_count": len(triplets),
        "trainable_count": sum(row.trainable for row in triplets),
        "reason_counts": {reason: reason_counts[reason] for reason in REASONS},
        "task_start_ego_frame_ns": task_start_ns,
        "git": _validated_git_provenance(git),
        "session_seal_sha256": seal["content_sha256"],
        "output_csv_sha256": hashlib.sha256(csv_bytes).hexdigest(),
    }
    return csv_bytes, manifest


def _build_pair_artifacts(
    source_bytes: dict[str, bytes],
    seal: dict,
    git: dict,
    source_sha256: dict[str, str],
    task_start_ns: int,
) -> tuple[bytes, dict]:
    coordinator = _load_json_object(
        source_bytes["coordinator.json"], "coordinator.json"
    )
    umi_role = _pair_role_from_sources(source_bytes)
    _validate_pair_worker_acceptances(coordinator, umi_role)
    ego = _parse_d435i_ego_rows(
        source_bytes["ego/ego.ir_left.jsonl"], task_start_ns
    )
    umi = _parse_pair_umi_rows(
        source_bytes[f"{umi_role}/{umi_role}.ir_left.jsonl"], umi_role, task_start_ns
    )
    pairs = build_pairs(ego, umi, umi_role)
    csv_bytes = _serialize_pair_csv(pairs, umi_role)
    overlap = _pair_overlap(ego, umi)
    reason_counts = Counter(row.reason for row in pairs)
    manifest = {
        "schema": PAIR_SCHEMA if umi_role == "left" else PAIR_ROLE_SCHEMA,
        "capture_topology": TOPOLOGY_PAIR,
        "source_sha256": source_sha256,
        "grid_ns": GRID_NS,
        "target_span_ns": TARGET_SPAN_NS,
        "hard_span_ns": HARD_SPAN_NS,
        "input_config": {
            "ego": {
                "source": "ego/ego.ir_left.jsonl",
                "stream_id": D435I_EGO_STREAM_ID,
                "clock_domain": D435I_CLOCK_DOMAIN,
                "timestamp_field": "acquisition_ns",
                "formal_filter": "warmup=false,valid=true",
            },
            umi_role: {
                "source": f"{umi_role}/{umi_role}.ir_left.jsonl",
                "stream_id": f"{umi_role}.ir_left",
                "clock_domain": D435I_CLOCK_DOMAIN,
                "timestamp_field": "acquisition_ns",
                "formal_filter": "warmup=false,valid=true",
            },
        },
        "common_overlap": overlap,
        "row_count": len(pairs),
        "trainable_count": sum(row.trainable for row in pairs),
        "reason_counts": {reason: reason_counts[reason] for reason in REASONS},
        "task_start_ego_frame_ns": task_start_ns,
        "git": _validated_git_provenance(git),
        "session_seal_sha256": seal["content_sha256"],
        "output_csv_sha256": hashlib.sha256(csv_bytes).hexdigest(),
    }
    if umi_role == "right":
        manifest["umi_role"] = umi_role
    return csv_bytes, manifest


def _seal_topology(seal: dict) -> str:
    sources = seal.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ValueError("session seal sources are invalid")
    keys = set(sources)
    if keys == set(SOURCE_PATHS):
        return TOPOLOGY_LEGACY
    if keys in (set(PAIR_SOURCE_PATHS), set(PAIR_RIGHT_SOURCE_PATHS)):
        return TOPOLOGY_PAIR
    raise ValueError("session seal sources are invalid")


def _ensure_session_seal(session: Path) -> dict:
    seal_path = session / SEAL_NAME
    existing = assert_safe_path(seal_path, session, SEAL_NAME, kind="file")
    if existing is not None:
        seal = _load_json_object(seal_path.read_bytes(), SEAL_NAME)
        _validate_seal_document(seal)
        _verify_sources_against_seal(session, seal)
        return seal

    topology = _detect_topology(session)
    source_paths = (
        _pair_source_paths_for_session(session)
        if topology == TOPOLOGY_PAIR
        else _source_paths(topology)
    )
    captured = _capture_sources(session)
    seal = {
        "schema": SEAL_SCHEMA,
        "sources": {
            relative: {
                "sha256": captured[relative]["sha256"],
                "identity": list(captured[relative]["identity"]),
            }
            for relative in source_paths
        },
    }
    seal["content_sha256"] = _seal_content_sha256(seal)
    payload = (json.dumps(seal, indent=2, sort_keys=True) + "\n").encode("utf-8")
    temporary = session / f".{SEAL_NAME}.{uuid.uuid4().hex}.tmp"
    try:
        _write_directory_member(session, temporary.name, payload)
        _verify_captured_sources(session, captured)
        os.replace(temporary, seal_path)
        _fsync_directory(session)
        _verify_sources_against_seal(session, seal)
    finally:
        if temporary.exists():
            temporary.unlink()
    return seal


def _load_existing_session_seal(session: Path) -> dict:
    seal_path = session / SEAL_NAME
    if assert_safe_path(seal_path, session, SEAL_NAME, kind="file") is None:
        raise RuntimeError("session seal is missing")
    seal = _load_json_object(seal_path.read_bytes(), SEAL_NAME)
    _validate_seal_document(seal)
    _verify_sources_against_seal(session, seal)
    return seal


def _validate_seal_document(seal: dict) -> None:
    if seal.get("schema") != SEAL_SCHEMA:
        raise ValueError("session seal schema is invalid")
    sources = seal.get("sources")
    if not isinstance(sources, dict) or set(sources) not in (
        set(SOURCE_PATHS),
        set(PAIR_SOURCE_PATHS),
        set(PAIR_RIGHT_SOURCE_PATHS),
    ):
        raise ValueError("session seal sources are invalid")
    for relative, evidence in sources.items():
        if not isinstance(evidence, dict) or set(evidence) != {"sha256", "identity"}:
            raise ValueError(f"session seal evidence is invalid: {relative}")
        digest = evidence["sha256"]
        identity = evidence["identity"]
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
            or not isinstance(identity, list)
            or len(identity) != 5
            or any(isinstance(value, bool) or not isinstance(value, int) for value in identity)
        ):
            raise ValueError(f"session seal evidence is invalid: {relative}")
    if seal.get("content_sha256") != _seal_content_sha256(seal):
        raise ValueError("session seal content hash is invalid")


def _seal_content_sha256(seal: dict) -> str:
    addressed = {key: value for key, value in seal.items() if key != "content_sha256"}
    payload = json.dumps(
        addressed, ensure_ascii=False, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _capture_sources(session: Path, source_paths=None) -> dict[str, dict]:
    if source_paths is None:
        seal_path = session / SEAL_NAME
        if seal_path.is_file():
            source_paths = tuple(
                _load_json_object(seal_path.read_bytes(), SEAL_NAME)["sources"]
            )
        else:
            topology = _detect_topology(session)
            source_paths = (
                _pair_source_paths_for_session(session)
                if topology == TOPOLOGY_PAIR
                else _source_paths(topology)
            )
    captured = {}
    for relative in source_paths:
        path = session / relative
        assert_safe_path(path, session, relative, kind="file")
        with path.open("rb") as stream:
            before_descriptor = _file_identity(os.fstat(stream.fileno()))
            before_path = _file_identity(path.stat())
            payload = stream.read()
            after_descriptor = _file_identity(os.fstat(stream.fileno()))
            after_path = _file_identity(path.stat())
        if not all(
            observed == before_descriptor
            for observed in (before_path, after_descriptor, after_path)
        ):
            raise RuntimeError(f"source changed while sealing: {relative}")
        captured[relative] = {
            "payload": payload,
            "sha256": hashlib.sha256(payload).hexdigest(),
            "identity": before_descriptor,
        }
    return captured


def _verify_captured_sources(
    session: Path, expected: dict[str, dict], source_paths=None
) -> None:
    observed = (
        _capture_sources(session)
        if source_paths is None
        else _capture_sources(session, source_paths)
    )
    for relative in expected:
        if (
            observed[relative]["sha256"] != expected[relative]["sha256"]
            or observed[relative]["identity"] != expected[relative]["identity"]
        ):
            raise RuntimeError(f"source changed while sealing: {relative}")


def _verify_sources_against_seal(session: Path, seal: dict) -> dict[str, dict]:
    _validate_seal_document(seal)
    observed = _capture_sources(session)
    for relative in seal["sources"]:
        evidence = seal["sources"][relative]
        if (
            observed[relative]["sha256"] != evidence["sha256"]
            or list(observed[relative]["identity"]) != evidence["identity"]
        ):
            raise RuntimeError(f"source does not match session seal: {relative}")
    return observed


def _read_sealed_sources(session: Path, seal: dict) -> dict[str, bytes]:
    return {
        relative: evidence["payload"]
        for relative, evidence in _verify_sources_against_seal(session, seal).items()
    }


def _validate_existing_index(session: Path, seal: dict) -> dict:
    output_directory = session / "sync"
    assert_safe_path(output_directory, session, "sync", kind="directory")
    members = {path.name for path in output_directory.iterdir()}
    if members != {"common_30hz.csv", "manifest.json"}:
        raise RuntimeError("existing sync directory is incomplete or has extra members")
    csv_path = output_directory / "common_30hz.csv"
    manifest_path = output_directory / "manifest.json"
    assert_safe_path(csv_path, session, "sync/common_30hz.csv", kind="file")
    assert_safe_path(manifest_path, session, "sync/manifest.json", kind="file")
    manifest = _load_json_object(manifest_path.read_bytes(), "sync/manifest.json")
    source_bytes = _read_sealed_sources(session, seal)
    expected_csv, expected_manifest = _build_artifacts(
        source_bytes, seal, manifest.get("git")
    )
    if manifest != expected_manifest or csv_path.read_bytes() != expected_csv:
        raise RuntimeError("existing sync directory does not match sealed inputs")
    _verify_published_outputs(session, csv_path, manifest)
    return manifest


def _write_directory_member(directory: Path, name: str, payload: bytes) -> None:
    if Path(name).name != name or not name:
        raise ValueError("publication member name is unsafe")
    path = directory / name
    with path.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _remove_published_directory(session: Path, directory: Path) -> None:
    info = assert_safe_path(directory, session, directory.name, kind="directory")
    if info is None:
        return
    allowed = {"common_30hz.csv", "manifest.json"}
    members = list(directory.iterdir())
    if any(path.name not in allowed for path in members):
        raise RuntimeError(f"refusing to clean unexpected publication: {directory.name}")
    for path in members:
        assert_safe_path(path, session, path.name, kind="file")
        path.unlink()
    directory.rmdir()
    _fsync_directory(session)


def _reject_active_writer_claims(session: Path) -> None:
    for directory in (session, *(session / name for name in ("ego", "left", "right"))):
        claim = directory / ".writer.lock"
        if claim.exists() or claim.is_symlink():
            assert_safe_path(claim, session, str(claim.relative_to(session)), kind="file")
            raise RuntimeError("session has an active writer")


@contextmanager
def _offline_session_lease(session: Path):
    with offline_claim(session) as root:
        yield root


def _pair_source_paths_for_session(session: Path) -> tuple[str, ...]:
    present = [
        role
        for role in ("left", "right")
        if (session / role / f"{role}.ir_left.jsonl").is_file()
    ]
    if present == ["left"]:
        return PAIR_SOURCE_PATHS
    if present == ["right"]:
        return PAIR_RIGHT_SOURCE_PATHS
    raise ValueError("single-UMI session must contain exactly one left or right stream")


def _pair_role_from_sources(source_bytes: dict[str, bytes]) -> str:
    keys = set(source_bytes)
    if keys == set(PAIR_SOURCE_PATHS):
        return "left"
    if keys == set(PAIR_RIGHT_SOURCE_PATHS):
        return "right"
    raise ValueError("pair source set does not identify exactly one UMI role")


def _validate_pair_worker_acceptances(coordinator: dict, umi_role: str = "left") -> None:
    if coordinator.get("schema") != COORDINATOR_SCHEMA:
        raise ValueError(
            f"coordinator.json schema must be {COORDINATOR_SCHEMA}"
        )
    worker_acceptance = coordinator.get("worker_acceptance")
    if not isinstance(worker_acceptance, dict):
        raise ValueError("coordinator.json worker_acceptance is missing")
    ego = worker_acceptance.get("ego")
    if (
        not isinstance(ego, dict)
        or ego.get("schema") != "ego.d435i.acceptance.v1"
        or ego.get("status") != "PASS"
    ):
        raise ValueError("coordinator.json ego acceptance is not a d435i PASS")
    umi = worker_acceptance.get(umi_role)
    camera_clock = (
        umi.get("camera_clock") if isinstance(umi, dict) else None
    )
    if (
        not isinstance(umi, dict)
        or umi.get("result") != "PASS"
        or not isinstance(camera_clock, dict)
        or camera_clock.get("verified") is not True
    ):
        raise ValueError(
            f"coordinator.json {umi_role} UMI camera clock is not verified"
        )


def _parse_coordinator(payload: bytes) -> int:
    coordinator = _load_json_object(payload, "coordinator.json")
    if coordinator.get("schema") != COORDINATOR_SCHEMA:
        raise ValueError(f"coordinator.json schema must be {COORDINATOR_SCHEMA}")
    task_start_ns = coordinator.get("task_start_ego_frame_ns")
    return _nonnegative_integer(
        task_start_ns, "coordinator.json task_start_ego_frame_ns"
    )


def _load_json_object(payload: bytes, source: str) -> dict:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{source} is malformed JSON") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{source} must contain a JSON object")
    return value


def _validate_d405_clock(acceptance_payload: bytes, csv_payload: bytes, device: str) -> None:
    acceptance = _load_json_object(acceptance_payload, f"{device}/acceptance.json")
    camera_clock = acceptance.get("camera_clock")
    joint_start = acceptance.get("joint_start")
    if (
        acceptance.get("result") != "PASS"
        or not isinstance(camera_clock, dict)
        or camera_clock.get("verified") is not True
        or not isinstance(joint_start, dict)
        or joint_start.get("clock_domain") != "host_monotonic"
    ):
        raise ValueError(f"{device} camera clock is not verified")
    required = camera_clock.get("required_streams")
    if not isinstance(required, list) or not required:
        raise ValueError(f"{device} camera clock required streams are missing")
    try:
        text = csv_payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{device}/d405_frames.csv is not UTF-8") from exc
    rows = csv.DictReader(io.StringIO(text, newline=""))
    expected_columns = {f"{stream}_domain" for stream in required}
    if rows.fieldnames is None or not expected_columns.issubset(rows.fieldnames):
        raise ValueError(f"{device} camera clock CSV domains are missing")
    for line_number, row in enumerate(rows, start=2):
        if any(row.get(column) != "global_time" for column in expected_columns):
            raise ValueError(
                f"{device} camera clock CSV domain mismatch at line {line_number}"
            )


def _parse_ego_rows(payload: bytes, task_start_ns: int) -> list[FrameStamp]:
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("ego/ego.video.jsonl is not UTF-8") from exc
    frames = []
    seen_sequences = set()
    previous_timestamp = None
    for line_number, line in enumerate(text.splitlines(), start=1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"ego/ego.video.jsonl line {line_number} is malformed JSON"
            ) from exc
        if not isinstance(row, dict):
            raise ValueError(
                f"ego/ego.video.jsonl line {line_number} must be an object"
            )
        if row.get("stream_id") != "ego.video":
            raise ValueError(
                f"ego/ego.video.jsonl line {line_number} stream_id must be ego.video"
            )
        sequence = _nonnegative_integer(
            row.get("sequence"), f"ego line {line_number} sequence"
        )
        acquisition_ns = _nonnegative_integer(
            row.get("acquisition_ns"),
            f"ego line {line_number} acquisition_ns",
        )
        warmup = _boolean(row.get("warmup"), f"ego line {line_number} warmup")
        valid = _boolean(row.get("valid"), f"ego line {line_number} valid")
        clock_domain = row.get("clock_domain")
        if clock_domain not in {
            "host_monotonic",
            "gstreamer_timestamp_unavailable",
        }:
            raise ValueError(f"ego line {line_number} clock_domain is unsupported")
        candidate = not warmup and valid
        if candidate and clock_domain != "host_monotonic":
            raise ValueError(
                f"ego line {line_number} clock_domain must be host_monotonic"
            )
        offset = _nonnegative_integer(
            row.get("offset"), f"ego line {line_number} offset"
        )
        size = _nonnegative_integer(row.get("size"), f"ego line {line_number} size")
        if sequence in seen_sequences:
            raise ValueError(f"duplicate (ego, {sequence}) sequence")
        seen_sequences.add(sequence)
        if clock_domain == "host_monotonic":
            if previous_timestamp is not None and acquisition_ns < previous_timestamp:
                raise ValueError("ego timestamp regression")
            previous_timestamp = acquisition_ns
        if candidate and acquisition_ns >= task_start_ns:
            frames.append(
                FrameStamp(
                    "ego",
                    sequence,
                    acquisition_ns,
                    f"ego/ego.video.bin:{offset}:{size}",
                )
            )
    return frames


def _parse_d435i_ego_rows(payload: bytes, task_start_ns: int) -> list[FrameStamp]:
    return _parse_append_only_rows(
        payload, "ego/ego.ir_left.jsonl", D435I_EGO_STREAM_ID, "ego", task_start_ns
    )


def _parse_pair_umi_rows(
    payload: bytes, device_id: str, task_start_ns: int
) -> list[FrameStamp]:
    return _parse_append_only_rows(
        payload,
        f"{device_id}/{device_id}.ir_left.jsonl",
        f"{device_id}.ir_left",
        device_id,
        task_start_ns,
    )


def _parse_append_only_rows(
    payload: bytes, source: str, stream_id: str, device_id: str, task_start_ns: int
) -> list[FrameStamp]:
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{source} is not UTF-8") from exc
    frames = []
    seen_sequences = set()
    previous_timestamp = None
    for line_number, line in enumerate(text.splitlines(), start=1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{source} line {line_number} is malformed JSON") from exc
        if not isinstance(row, dict):
            raise ValueError(f"{source} line {line_number} must be an object")
        if row.get("stream_id") != stream_id:
            raise ValueError(
                f"{source} line {line_number} stream_id must be {stream_id}"
            )
        sequence = _nonnegative_integer(
            row.get("sequence"), f"{device_id} line {line_number} sequence"
        )
        acquisition_ns = _nonnegative_integer(
            row.get("acquisition_ns"),
            f"{device_id} line {line_number} acquisition_ns",
        )
        warmup = _boolean(row.get("warmup"), f"{device_id} line {line_number} warmup")
        valid = _boolean(row.get("valid"), f"{device_id} line {line_number} valid")
        clock_domain = row.get("clock_domain")
        if clock_domain != D435I_CLOCK_DOMAIN:
            raise ValueError(
                f"{device_id} line {line_number} clock_domain is unsupported"
            )
        offset = _nonnegative_integer(
            row.get("offset"), f"{device_id} line {line_number} offset"
        )
        size = _nonnegative_integer(row.get("size"), f"{device_id} line {line_number} size")
        if previous_timestamp is not None and acquisition_ns < previous_timestamp:
            raise ValueError(f"{device_id} timestamp regression")
        previous_timestamp = acquisition_ns
        candidate = not warmup and valid and acquisition_ns >= task_start_ns
        if candidate and sequence in seen_sequences:
            raise ValueError(f"duplicate ({device_id}, {sequence}) sequence")
        if candidate:
            seen_sequences.add(sequence)
            frames.append(
                FrameStamp(
                    device_id,
                    sequence,
                    acquisition_ns,
                    f"{source.removesuffix('.jsonl')}.bin:{offset}:{size}",
                )
            )
    return frames


def _parse_d405_rows(
    payload: bytes, device_id: str, task_start_ns: int
) -> list[FrameStamp]:
    source = f"{device_id}/d405_frames.csv"
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{source} is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    required_fields = {"set_index", "arrival_mono", "warmup"}
    fieldnames = reader.fieldnames
    if fieldnames is None or not required_fields.issubset(fieldnames):
        raise ValueError(f"{source} is missing required columns")
    if any(name is None or not name.strip() for name in fieldnames):
        raise ValueError(f"{source} has an empty header")
    if len(fieldnames) != len(set(fieldnames)):
        raise ValueError(f"{source} has duplicate headers")
    frames = []
    seen_sequences = set()
    previous_timestamp = None
    for row in reader:
        line_number = reader.line_num
        if None in row:
            raise ValueError(f"{source} line {line_number} has surplus fields")
        if any(value is None for value in row.values()):
            raise ValueError(f"{source} line {line_number} has too few fields")
        sequence = _csv_nonnegative_integer(
            row.get("set_index"), f"{source} line {line_number} set_index"
        )
        acquisition_ns = _decimal_seconds_to_ns(
            row.get("arrival_mono"),
            f"{source} line {line_number} arrival_mono",
        )
        warmup_text = row.get("warmup")
        if warmup_text not in {"0", "1"}:
            raise ValueError(f"{source} line {line_number} warmup must be 0 or 1")
        if sequence in seen_sequences:
            raise ValueError(f"duplicate ({device_id}, {sequence}) sequence")
        if previous_timestamp is not None and acquisition_ns < previous_timestamp:
            raise ValueError(f"{device_id} timestamp regression")
        seen_sequences.add(sequence)
        previous_timestamp = acquisition_ns
        if warmup_text == "0" and acquisition_ns >= task_start_ns:
            frames.append(
                FrameStamp(
                    device_id,
                    sequence,
                    acquisition_ns,
                    f"{source}:{line_number}:{sequence}",
                )
            )
    return frames


def _csv_nonnegative_integer(value, field_name: str) -> int:
    if not isinstance(value, str):
        raise ValueError(f"{field_name} must be a non-negative integer")
    try:
        parsed = int(value)
    except ValueError as exc:
        raise ValueError(f"{field_name} must be a non-negative integer") from exc
    if parsed < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")
    return parsed


def _decimal_seconds_to_ns(value, field_name: str) -> int:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field_name} must be decimal seconds")
    try:
        with localcontext() as context:
            context.prec = max(50, len(value) + 10)
            seconds = Decimal(value)
            nanoseconds = seconds * Decimal(1_000_000_000)
    except InvalidOperation as exc:
        raise ValueError(f"{field_name} must be finite decimal seconds") from exc
    if not seconds.is_finite() or seconds < 0:
        raise ValueError(f"{field_name} must be finite non-negative decimal seconds")
    if nanoseconds != nanoseconds.to_integral_value():
        raise ValueError(f"{field_name} is not an exact integer nanosecond")
    return int(nanoseconds)


def _nonnegative_integer(value, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")
    return value


def _boolean(value, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field_name} must be a boolean")
    return value


def _common_overlap(
    ego: list[FrameStamp], left: list[FrameStamp], right: list[FrameStamp]
) -> dict[str, int | None]:
    streams = (ego, left, right)
    if any(not stream for stream in streams):
        return {"start_ns": None, "end_ns": None}
    start_ns = max(stream[0].acquisition_ns for stream in streams)
    end_ns = min(stream[-1].acquisition_ns for stream in streams)
    if start_ns > end_ns:
        return {"start_ns": None, "end_ns": None}
    return {"start_ns": start_ns, "end_ns": end_ns}


def _pair_overlap(
    ego: list[FrameStamp], left: list[FrameStamp]
) -> dict[str, int | None]:
    streams = (ego, left)
    if any(not stream for stream in streams):
        return {"start_ns": None, "end_ns": None}
    start_ns = max(stream[0].acquisition_ns for stream in streams)
    end_ns = min(stream[-1].acquisition_ns for stream in streams)
    if start_ns > end_ns:
        return {"start_ns": None, "end_ns": None}
    return {"start_ns": start_ns, "end_ns": end_ns}


def _serialize_pair_csv(pairs: list, umi_role: str = "left") -> bytes:
    stream = io.StringIO(newline="")
    header = PAIR_CSV_HEADER if umi_role == "left" else PAIR_ROLE_CSV_HEADER
    writer = csv.DictWriter(stream, fieldnames=header, lineterminator="\n")
    writer.writeheader()
    for row in pairs:
        values = {
                "sample_ns": row.sample_ns,
                "ego_sequence": row.ego.sequence,
                "ego_acquisition_ns": row.ego.acquisition_ns,
                "ego_ref": row.ego.payload_ref,
                "span_ns": row.span_ns,
                "trainable": int(row.trainable),
                "reason": row.reason,
        }
        if umi_role == "left":
            values.update(
                left_sequence=row.left.sequence,
                left_acquisition_ns=row.left.acquisition_ns,
                left_ref=row.left.payload_ref,
            )
        else:
            values.update(
                umi_role=umi_role,
                umi_sequence=row.left.sequence,
                umi_acquisition_ns=row.left.acquisition_ns,
                umi_ref=row.left.payload_ref,
            )
        writer.writerow(values)
    return stream.getvalue().encode("utf-8")


def _serialize_csv(triplets: list[Triplet]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_HEADER, lineterminator="\n")
    writer.writeheader()
    for row in triplets:
        writer.writerow(
            {
                "sample_ns": row.sample_ns,
                "ego_sequence": row.ego.sequence,
                "ego_acquisition_ns": row.ego.acquisition_ns,
                "ego_ref": row.ego.payload_ref,
                "left_sequence": row.left.sequence,
                "left_acquisition_ns": row.left.acquisition_ns,
                "left_ref": row.left.payload_ref,
                "right_sequence": row.right.sequence,
                "right_acquisition_ns": row.right.acquisition_ns,
                "right_ref": row.right.payload_ref,
                "span_ns": row.span_ns,
                "trainable": int(row.trainable),
                "reason": row.reason,
            }
        )
    return stream.getvalue().encode("utf-8")


def _verify_published_outputs(session: Path, csv_path: Path, manifest: dict) -> None:
    expected_files = {
        "common_30hz.csv": (csv_path, manifest["output_csv_sha256"]),
        **{
            relative: (session / relative, expected_hash)
            for relative, expected_hash in manifest["source_sha256"].items()
        },
    }
    with ExitStack() as stack:
        opened = {
            label: (
                path,
                expected_hash,
                stack.enter_context(path.open("rb")),
            )
            for label, (path, expected_hash) in expected_files.items()
        }
        before_descriptors = {
            label: _file_identity(os.fstat(stream.fileno()))
            for label, (_, _, stream) in opened.items()
        }
        before_paths = {
            label: _file_identity(path.stat())
            for label, (path, _, _) in opened.items()
        }
        digests = {
            label: _hash_open_file(path, stream)
            for label, (path, _, stream) in opened.items()
        }
        after_descriptors = {
            label: _file_identity(os.fstat(stream.fileno()))
            for label, (_, _, stream) in opened.items()
        }
        after_paths = {
            label: _file_identity(path.stat())
            for label, (path, _, _) in opened.items()
        }
        for label, (_, expected_hash, _) in opened.items():
            identity = before_descriptors[label]
            if not all(
                observed == identity
                for observed in (
                    before_paths[label],
                    after_descriptors[label],
                    after_paths[label],
                )
            ):
                _raise_published_change(label)
            if digests[label] != expected_hash:
                _raise_published_change(label)


def _raise_published_change(label: str) -> None:
    if label == "common_30hz.csv":
        raise RuntimeError("published CSV changed while building")
    raise RuntimeError(f"source changed while building: {label}")


def _hash_open_file(path: Path, stream) -> str:
    del path
    stream.seek(0)
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(65536), b""):
        digest.update(block)
    return digest.hexdigest()


def _file_identity(stat_result) -> tuple[int, int, int, int, int]:
    return (
        stat_result.st_dev,
        stat_result.st_ino,
        stat_result.st_size,
        stat_result.st_mtime_ns,
        stat_result.st_ctime_ns,
    )


@contextmanager
def _exclusive_publication_lock(output_directory: Path):
    with _offline_session_lease(output_directory):
        yield


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _git_provenance() -> dict:
    executable = _git_executable()
    try:
        commit = subprocess.run(
            [executable, "rev-parse", "HEAD"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                [executable, "status", "--porcelain"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("Git provenance is unavailable") from exc
    return {"hash": commit, "dirty": dirty}


def _git_executable() -> str:
    if os.name != "posix":
        return "git"
    try:
        pointer = (ROOT / ".git").read_text(encoding="utf-8").strip()
    except (OSError, UnicodeDecodeError):
        return "git"
    if not pointer.startswith("gitdir: "):
        return "git"
    git_directory = pointer.removeprefix("gitdir: ")
    if (
        len(git_directory) >= 3
        and git_directory[1] == ":"
        and git_directory[2] in "/\\"
    ):
        return "git.exe"
    return "git"


def _validated_git_provenance(provenance) -> dict:
    if not isinstance(provenance, dict):
        raise RuntimeError("Git provenance must be an object")
    commit = provenance.get("hash")
    dirty = provenance.get("dirty")
    if not isinstance(commit, str) or not commit.strip():
        raise RuntimeError("Git provenance hash must be non-empty")
    if not isinstance(dirty, bool):
        raise RuntimeError("Git provenance dirty must be a boolean")
    return {"hash": commit.strip(), "dirty": dirty}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Build a deterministic 30 Hz three-device synchronization index."
    )
    parser.add_argument("--session", type=Path, required=True, help="raw session root")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        manifest = build_index(args.session)
    except Exception as exc:
        print(f"sync index build failed: {exc}", file=sys.stderr)
        return 1
    print(
        f"wrote {manifest['row_count']} rows to "
        f"{args.session / 'sync' / 'common_30hz.csv'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
