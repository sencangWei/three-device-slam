from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import zlib


_SAFE_OUTPUT_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def select_camera_rows(rows: list[dict], *, frequency_hz: float) -> list[dict]:
    if frequency_hz <= 0:
        raise ValueError("frequency_hz must be positive")
    period_ns = int(round(1_000_000_000 / frequency_hz))
    selected = []
    previous_ns = None
    last_selected_ns = None
    for row in rows:
        acquisition_ns = int(row["acquisition_ns"])
        if previous_ns is not None and acquisition_ns <= previous_ns:
            raise ValueError("video timestamp regression or duplicate")
        previous_ns = acquisition_ns
        if (
            last_selected_ns is None
            or acquisition_ns - last_selected_ns >= period_ns
        ):
            selected.append(row)
            last_selected_ns = acquisition_ns
    return selected


def _load_source_rows(session: Path, manifest: dict) -> list[dict]:
    payload_path = session / "ego.video.bin"
    index_path = session / "ego.video.jsonl"
    stream = manifest.get("streams", {}).get("ego.video")
    if not isinstance(stream, dict):
        raise ValueError("manifest is missing ego.video")
    expected_payload = stream.get("payload_sha256")
    expected_index = stream.get("index_sha256")
    if _sha256_file(payload_path) != expected_payload:
        raise ValueError("ego.video.bin SHA-256 mismatch")
    if _sha256_file(index_path) != expected_index:
        raise ValueError("ego.video.jsonl SHA-256 mismatch")
    rows = []
    with index_path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"ego.video.jsonl line {line_number} is malformed"
                ) from exc
            if not isinstance(row, dict):
                raise ValueError(f"ego.video.jsonl line {line_number} is not an object")
            rows.append(row)
    if not rows:
        raise ValueError("ego.video.jsonl is empty")
    return rows


def export_stereo_dataset(
    *,
    session: Path,
    output: Path,
    frequency_hz: float = 5.0,
    expected_shape: tuple[int, int] = (1080, 3840),
    required_stage: str = "stereo",
    required_acceptance_status: str = "PASS",
) -> dict:
    import cv2
    import numpy as np

    session = session.resolve(strict=True)
    output = Path(os.path.abspath(output))
    if output.parent != session:
        raise ValueError("output must be a direct child of the source session")
    if not _SAFE_OUTPUT_NAME.fullmatch(output.name):
        raise ValueError("output directory name is unsafe")
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")

    capture_report_path = session / "capture_report.json"
    capture_report = json.loads(capture_report_path.read_text(encoding="utf-8"))
    if (
        capture_report.get("stage") != required_stage
        or capture_report.get("stage_acceptance", {}).get("status")
        != required_acceptance_status
    ):
        raise ValueError(
            f"source session did not satisfy {required_stage} "
            f"{required_acceptance_status} acceptance"
        )
    manifest_path = session / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    rows = _load_source_rows(session, manifest)
    selected = select_camera_rows(rows, frequency_hz=frequency_hz)

    output.mkdir()
    cam0 = output / "cam0"
    cam1 = output / "cam1"
    cam0.mkdir()
    cam1.mkdir()
    payload_path = session / "ego.video.bin"
    index_path = output / "export_index.jsonl"
    with payload_path.open("rb") as payload_file, index_path.open(
        "w", encoding="utf-8"
    ) as derived_index:
        for row in selected:
            if row.get("stream_id") != "ego.video" or not row.get("valid"):
                raise ValueError(f"invalid video row at sequence {row.get('sequence')}")
            metadata = row.get("metadata", {})
            if (
                metadata.get("encoding") != "MJPG"
                or int(metadata.get("height", 0)) != expected_shape[0]
                or int(metadata.get("width", 0)) != expected_shape[1]
            ):
                raise ValueError(
                    f"unexpected video contract at sequence {row.get('sequence')}"
                )
            offset = int(row["offset"])
            size = int(row["size"])
            payload_file.seek(offset)
            jpeg = payload_file.read(size)
            if len(jpeg) != size:
                raise ValueError(f"short video payload at sequence {row.get('sequence')}")
            if zlib.crc32(jpeg) & 0xFFFFFFFF != int(row["crc32"]):
                raise ValueError(f"CRC mismatch at video sequence {row.get('sequence')}")
            if len(jpeg) < 4 or jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9":
                raise ValueError(f"invalid JPEG boundaries at sequence {row.get('sequence')}")
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            if image is None or tuple(image.shape) != expected_shape:
                raise ValueError(f"JPEG shape mismatch at sequence {row.get('sequence')}")
            midpoint = expected_shape[1] // 2
            left = image[:, :midpoint]
            right = image[:, midpoint:]
            timestamp_ns = int(row["acquisition_ns"])
            filename = f"{timestamp_ns}.png"
            left_path = cam0 / filename
            right_path = cam1 / filename
            png_options = [cv2.IMWRITE_PNG_COMPRESSION, 3]
            if not cv2.imwrite(str(left_path), left, png_options):
                raise OSError(f"failed to write {left_path}")
            if not cv2.imwrite(str(right_path), right, png_options):
                raise OSError(f"failed to write {right_path}")
            derived = {
                "timestamp_ns": timestamp_ns,
                "source_sequence": int(row["sequence"]),
                "source_offset": offset,
                "source_size": size,
                "source_crc32": int(row["crc32"]),
                "cam0": f"cam0/{filename}",
                "cam1": f"cam1/{filename}",
                "cam0_sha256": _sha256_file(left_path),
                "cam1_sha256": _sha256_file(right_path),
            }
            derived_index.write(
                json.dumps(derived, ensure_ascii=False, sort_keys=True) + "\n"
            )
        derived_index.flush()
        os.fsync(derived_index.fileno())

    first_ns = int(selected[0]["acquisition_ns"])
    last_ns = int(selected[-1]["acquisition_ns"])
    duration_s = (last_ns - first_ns) / 1_000_000_000
    report = {
        "schema": "ego.2uq2.kalibr-stereo-export.v1",
        "status": "PASS",
        "source_session": str(session),
        "source_manifest_sha256": _sha256_file(manifest_path),
        "source_capture_report_sha256": _sha256_file(capture_report_path),
        "camera_mapping": {
            "cam0": "left half of 3840x1080 SBS",
            "cam1": "right half of 3840x1080 SBS",
        },
        "timestamp_model": "identical host_monotonic acquisition_ns for cam0 and cam1",
        "requested_hz": frequency_hz,
        "selected_pairs": len(selected),
        "first_timestamp_ns": first_ns,
        "last_timestamp_ns": last_ns,
        "duration_s": duration_s,
        "measured_hz": (len(selected) - 1) / duration_s if duration_s > 0 else 0.0,
        "derived_index_sha256": _sha256_file(index_path),
    }
    report_path = output / "export_report.json"
    with report_path.open("w", encoding="utf-8") as destination:
        json.dump(report, destination, ensure_ascii=False, indent=2, sort_keys=True)
        destination.write("\n")
        destination.flush()
        os.fsync(destination.fileno())
    return report


def export_imu_csv(*, session: Path, output: Path) -> dict:
    """Export validated SI IMU rows in kalibr_bagcreater CSV order."""
    session = session.resolve(strict=True)
    output = output.resolve(strict=True)
    if output.parent != session:
        raise ValueError("output must be a direct child of the source session")
    manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))
    stream = manifest.get("streams", {}).get("ego.imu")
    if not isinstance(stream, dict):
        raise ValueError("manifest is missing ego.imu")
    payload_path = session / "ego.imu.bin"
    index_path = session / "ego.imu.jsonl"
    if _sha256_file(payload_path) != stream.get("payload_sha256"):
        raise ValueError("ego.imu.bin SHA-256 mismatch")
    if _sha256_file(index_path) != stream.get("index_sha256"):
        raise ValueError("ego.imu.jsonl SHA-256 mismatch")

    csv_path = output / "imu0.csv"
    timestamps = []
    with index_path.open("r", encoding="utf-8") as source, csv_path.open(
        "w", encoding="utf-8", newline=""
    ) as destination:
        writer = csv.writer(destination, lineterminator="\n")
        writer.writerow(
            ("timestamp", "omega_x", "omega_y", "omega_z", "alpha_x", "alpha_y", "alpha_z")
        )
        previous_ns = None
        for line_number, line in enumerate(source, start=1):
            row = json.loads(line)
            timestamp_ns = int(row["acquisition_ns"])
            if previous_ns is not None and timestamp_ns <= previous_ns:
                raise ValueError(f"IMU timestamp regression at line {line_number}")
            if row.get("stream_id") != "ego.imu" or not row.get("valid"):
                raise ValueError(f"invalid IMU row at line {line_number}")
            metadata = row.get("metadata", {})
            gyro = [float(value) for value in metadata.get("gyro_rad_s", ())]
            accel = [float(value) for value in metadata.get("accel_m_s2", ())]
            if (
                len(gyro) != 3
                or len(accel) != 3
                or not all(math.isfinite(value) for value in gyro + accel)
            ):
                raise ValueError(f"invalid SI IMU vector at line {line_number}")
            writer.writerow((timestamp_ns, *gyro, *accel))
            timestamps.append(timestamp_ns)
            previous_ns = timestamp_ns
        destination.flush()
        os.fsync(destination.fileno())
    if len(timestamps) < 2:
        raise ValueError("at least two IMU rows are required")
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    duration_s = (timestamps[-1] - timestamps[0]) / 1_000_000_000
    return {
        "imu_samples": len(timestamps),
        "duration_s": duration_s,
        "measured_hz": (len(timestamps) - 1) / duration_s,
        "timestamp_regressions": 0,
        "max_interval_ms": max(intervals) / 1_000_000,
        "timestamp_model": "host_monotonic_control_transfer_midpoint",
        "units": {"omega": "rad/s", "alpha": "m/s^2"},
        "imu_csv_sha256": _sha256_file(csv_path),
    }


def export_camera_imu_dataset(
    *, session: Path, output: Path, frequency_hz: float = 10.0
) -> dict:
    report = export_stereo_dataset(
        session=session,
        output=output,
        frequency_hz=frequency_hz,
        required_stage="cam-imu-provisional",
        required_acceptance_status="PROVISIONAL_PASS",
    )
    report["schema"] = "ego.2uq2.kalibr-camera-imu-export.v1"
    report["acceptance"] = "PROVISIONAL_PASS"
    report["imu"] = export_imu_csv(session=session, output=output)
    report_path = Path(output) / "export_report.json"
    with report_path.open("w", encoding="utf-8") as destination:
        json.dump(report, destination, ensure_ascii=False, indent=2, sort_keys=True)
        destination.write("\n")
        destination.flush()
        os.fsync(destination.fileno())
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Export an accepted 2UQ2 SBS session for Kalibr bag creation"
    )
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frequency-hz", type=float, default=5.0)
    args = parser.parse_args(argv)
    try:
        report = export_stereo_dataset(
            session=args.session,
            output=args.output,
            frequency_hz=args.frequency_hz,
        )
    except Exception as exc:
        print(f"BLOCKED: {type(exc).__name__}: {exc}")
        return 3
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    print("NEXT: create ego_stereo_5hz.bag with the pinned Kalibr container.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
