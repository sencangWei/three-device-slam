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
from three_device_slam.synchronization.sync_index import (
    GRID_NS,
    HARD_SPAN_NS,
    TARGET_SPAN_NS,
    build_triplets,
)


SCHEMA = "ego.three_device.sync_index.v1"
COORDINATOR_SCHEMA = "ego.three_device.coordinator.v1"
SOURCE_PATHS = (
    "coordinator.json",
    "ego/ego.video.jsonl",
    "left/acceptance.json",
    "left/d405_frames.csv",
    "right/acceptance.json",
    "right/d405_frames.csv",
)
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
REASONS = ("within_target", "within_hard_limit", "span_over_hard_limit")


def build_index(session: Path) -> dict:
    """Build and durably publish one session index, returning its manifest."""
    session = Path(session)
    output_directory = session / "sync"
    output_directory.mkdir(parents=True, exist_ok=True)
    with _exclusive_publication_lock(output_directory):
        return _build_index_locked(session, output_directory)


def _build_index_locked(session: Path, output_directory: Path) -> dict:
    manifest_path = output_directory / "manifest.json"
    csv_path = output_directory / "common_30hz.csv"
    _remove_success_manifest(manifest_path)

    source_bytes = {
        relative: (session / relative).read_bytes() for relative in SOURCE_PATHS
    }
    source_sha256 = {
        relative: hashlib.sha256(payload).hexdigest()
        for relative, payload in source_bytes.items()
    }

    task_start_ns = _parse_coordinator(source_bytes["coordinator.json"])
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
        "git": _validated_git_provenance(_git_provenance()),
        "output_csv_sha256": hashlib.sha256(csv_bytes).hexdigest(),
    }
    manifest_bytes = (
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")

    csv_temporary = _write_temporary(csv_path, csv_bytes)
    manifest_temporary = None
    try:
        manifest_temporary = _write_temporary(manifest_path, manifest_bytes)
        _verify_sources_unchanged(session, source_sha256)
        os.replace(csv_temporary, csv_path)
        _fsync_directory(output_directory)
        _verify_sources_unchanged(session, source_sha256)
        os.replace(manifest_temporary, manifest_path)
        _fsync_directory(output_directory)
        _verify_published_outputs(session, csv_path, manifest)
    except BaseException:
        _remove_success_manifest(manifest_path)
        raise
    finally:
        for temporary in (csv_temporary, manifest_temporary):
            if temporary is not None and temporary.exists():
                temporary.unlink()
    return manifest


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


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def _verify_sources_unchanged(
    session: Path, source_sha256: dict[str, str]
) -> None:
    for relative, expected_hash in source_sha256.items():
        if _sha256_file(session / relative) != expected_hash:
            raise RuntimeError(f"source changed while building: {relative}")


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
    lock_path = output_directory / ".publication.lock"
    with lock_path.open("a+b") as stream:
        try:
            _lock_stream(stream)
        except OSError as exc:
            raise RuntimeError("sync index build already in progress") from exc
        try:
            yield
        finally:
            _unlock_stream(stream)


def _lock_stream(stream) -> None:
    if os.name == "nt":
        import msvcrt

        stream.seek(0, os.SEEK_END)
        if stream.tell() == 0:
            stream.write(b"\0")
            stream.flush()
            os.fsync(stream.fileno())
        stream.seek(0)
        msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        return
    import fcntl

    fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)


def _unlock_stream(stream) -> None:
    if os.name == "nt":
        import msvcrt

        stream.seek(0)
        msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
        return
    import fcntl

    fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def _write_temporary(final_path: Path, payload: bytes) -> Path:
    temporary = final_path.with_name(f".{final_path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise
    return temporary


def _remove_success_manifest(manifest_path: Path) -> None:
    if not manifest_path.exists():
        return
    manifest_path.unlink()
    _fsync_directory(manifest_path.parent)


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
