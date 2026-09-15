#!/usr/bin/env python3
"""Capture 2UQ2 stereo MJPEG and the main XU IMU group on an X5 host."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import queue
import select
import shlex
import signal
import statistics
import sys
import threading
import time
from pathlib import Path
from zlib import crc32


STANDARD_GRAVITY_M_S2 = 9.80665


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def measured_rate(count: int, first_ns: int | None, last_ns: int | None) -> float:
    if count < 2 or first_ns is None or last_ns is None or last_ns <= first_ns:
        return 0.0
    return (count - 1) * 1_000_000_000.0 / (last_ns - first_ns)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_stage_acceptance(
    *,
    stage: str,
    requested_duration_seconds: float,
    duration_seconds: float,
    native_video_hz: float,
    retained_video_hz: float,
    video_sequence_gaps: int,
    imu_hz: float,
    xu_errors: int,
    operator_aborted: bool,
) -> dict:
    """Evaluate only the sensors required by the selected calibration stage."""
    if stage not in {"preflight", "stereo", "cam-imu"}:
        raise ValueError(f"unsupported stage: {stage}")

    imu_required = stage in {"preflight", "cam-imu"}
    definitions = {
        "duration_completed": (
            duration_seconds >= requested_duration_seconds * 0.99,
            ">= 99% of requested duration",
            duration_seconds,
            True,
        ),
        "native_video_hz": (
            58.0 <= native_video_hz <= 62.0,
            "58.0 <= value <= 62.0",
            native_video_hz,
            True,
        ),
        "retained_video_hz": (
            29.0 <= retained_video_hz <= 31.0,
            "29.0 <= value <= 31.0",
            retained_video_hz,
            True,
        ),
        "video_sequence_gaps": (
            video_sequence_gaps == 0,
            "== 0",
            video_sequence_gaps,
            True,
        ),
        "imu_hz": (
            190.0 <= imu_hz <= 220.0,
            "190.0 <= value <= 220.0",
            imu_hz,
            imu_required,
        ),
        "xu_errors": (
            xu_errors == 0,
            "== 0",
            xu_errors,
            imu_required,
        ),
        "operator_aborted": (
            not operator_aborted,
            "== false",
            operator_aborted,
            True,
        ),
    }
    checks = {}
    for name, (passed, threshold, measurement, required) in definitions.items():
        checks[name] = {
            "status": "PASS" if passed else ("FAIL" if required else "WARN"),
            "required": required,
            "threshold": threshold,
            "measurement": measurement,
        }

    camera_names = (
        "duration_completed",
        "native_video_hz",
        "retained_video_hz",
        "video_sequence_gaps",
        "operator_aborted",
    )
    imu_names = ("imu_hz", "xu_errors")
    camera_status = (
        "PASS" if all(checks[name]["status"] == "PASS" for name in camera_names) else "FAIL"
    )
    if all(checks[name]["status"] == "PASS" for name in imu_names):
        imu_status = "PASS"
    elif imu_required:
        imu_status = "FAIL"
    else:
        imu_status = "WARN"
    required_pass = all(
        check["status"] == "PASS" for check in checks.values() if check["required"]
    )
    return {
        "stage": stage,
        "status": "PASS" if required_pass else "FAIL",
        "components": {
            "camera": {"status": camera_status, "required": True},
            "imu": {"status": imu_status, "required": imu_required},
        },
        "checks": checks,
    }


def build_next_action(args: argparse.Namespace, stage_report: dict) -> dict:
    output = Path(args.output)
    if stage_report["status"] != "PASS":
        failed = [
            name
            for name, check in stage_report["checks"].items()
            if check["required"] and check["status"] == "FAIL"
        ]
        return {
            "state": "RERUN_REQUIRED",
            "instruction": "修正硬门槛失败项后，换一个新的 --output 路径重跑本步骤。",
            "failed_required_checks": failed,
        }
    if args.stage == "preflight":
        next_output = f"/tmp/ego-x5-stereo-calib-{time.strftime('%Y%m%d')}-run1"
        command = (
            "python3 /tmp/x5_2uq2_calibration_capture.py "
            f"--stage stereo --output {shlex.quote(next_output)} --duration-seconds 90"
        )
        return {
            "state": "READY",
            "instruction": "预检通过。固定标定板，移动 Ego 相机，采集双目标定数据。",
            "command": command,
        }
    if args.stage == "stereo":
        local_output = (
            Path(args.pc_artifact_root).expanduser().resolve() / output.name
        )
        command = (
            f"scp -r pi@{shlex.quote(args.x5_address)}:{shlex.quote(str(output))} "
            f"{shlex.quote(str(local_output))}"
        )
        return {
            "state": "PENDING_OFFLINE_APRILGRID",
            "instruction": "相机传输通过。退出 SSH，在电脑终端复制数据，再做 AprilGrid 覆盖检查。",
            "command": command,
        }
    return {
        "state": "PENDING_OFFLINE_KALIBR",
        "instruction": "相机与 IMU 传输通过。退出 SSH，复制数据并进行 Kalibr cam-imu 求解。",
    }


def print_stage_summary(report: dict) -> None:
    stage = report["stage_acceptance"]
    print("\n=== 本步验收 ===", flush=True)
    print(f"步骤: {stage['stage']}", flush=True)
    print(f"相机: {stage['components']['camera']['status']}", flush=True)
    imu = stage["components"]["imu"]
    suffix = "硬门槛" if imu["required"] else "本阶段仅告警"
    print(f"IMU: {imu['status']} ({suffix})", flush=True)
    for name, check in stage["checks"].items():
        if check["status"] in {"FAIL", "WARN"}:
            print(
                f"- {check['status']} {name}: {check['measurement']}，要求 {check['threshold']}",
                flush=True,
            )
    print(f"本步结果: {stage['status']}", flush=True)
    next_action = report["next_action"]
    print(f"下一步: {next_action['instruction']}", flush=True)
    if next_action.get("command"):
        print(next_action["command"], flush=True)


class VideoWriter:
    def __init__(self, output: Path, capacity: int = 128):
        self._payload_path = output / "ego.video.bin"
        self._index_path = output / "ego.video.jsonl"
        self._queue: queue.Queue = queue.Queue(maxsize=capacity)
        self._sentinel = object()
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, daemon=False)
        self._thread.start()

    def append(self, row: dict, payload: bytes) -> None:
        if self._error is not None:
            raise RuntimeError("video writer failed") from self._error
        try:
            self._queue.put_nowait((row, payload))
        except queue.Full as error:
            raise RuntimeError("video writer queue full") from error

    def close(self) -> None:
        self._queue.put(self._sentinel)
        self._thread.join()
        if self._error is not None:
            raise RuntimeError("video writer failed") from self._error

    def _run(self) -> None:
        try:
            with self._payload_path.open("xb") as payload_stream, self._index_path.open(
                "x", encoding="utf-8"
            ) as index_stream:
                while True:
                    item = self._queue.get()
                    try:
                        if item is self._sentinel:
                            break
                        row, payload = item
                        row = dict(row)
                        row["offset"] = payload_stream.tell()
                        row["size"] = len(payload)
                        row["crc32"] = crc32(payload) & 0xFFFFFFFF
                        payload_stream.write(payload)
                        index_stream.write(json.dumps(row, sort_keys=True) + "\n")
                    finally:
                        self._queue.task_done()
                payload_stream.flush()
                index_stream.flush()
        except BaseException as error:
            self._error = error


def write_sensor_stream(
    output: Path, stream_id: str, rows: list[dict], payloads: list[bytes]
) -> None:
    if len(rows) != len(payloads):
        raise ValueError("row/payload count mismatch")
    payload_path = output / f"{stream_id}.bin"
    index_path = output / f"{stream_id}.jsonl"
    with payload_path.open("xb") as payload_stream, index_path.open(
        "x", encoding="utf-8"
    ) as index_stream:
        for source_row, payload in zip(rows, payloads):
            row = dict(source_row)
            row["offset"] = payload_stream.tell()
            row["size"] = len(payload)
            row["crc32"] = crc32(payload) & 0xFFFFFFFF
            payload_stream.write(payload)
            index_stream.write(json.dumps(row, sort_keys=True) + "\n")


def capture(args: argparse.Namespace) -> dict:
    sys.path.insert(0, args.source_root)
    from src.recorder.two_uq2_source import (  # pylint: disable=import-error
        TwoUq2MediaSource,
        parse_imu_frame,
    )

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    stop_requested = threading.Event()

    def request_stop(_signum, _frame):
        stop_requested.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)

    video_writer = VideoWriter(output)
    source = TwoUq2MediaSource(args.device, target_fps=args.video_hz)
    poller = select.poll()
    imu_rows: list[dict] = []
    imu_payloads: list[bytes] = []
    xu_rows: list[dict] = []
    xu_payloads: list[bytes] = []
    imu_intervals_ms: list[float] = []
    video_intervals_ms: list[float] = []
    native_video_frames = 0
    retained_video_frames = 0
    xu_errors = 0
    duplicate_main_payloads = 0
    video_sequence_gaps = 0
    last_imu_ns = None
    last_video_ns = None
    last_native_sequence = None
    last_main_payload = None
    started_ns = ended_ns = None
    try:
        source.start(capture=False)
        poller.register(source._fd, select.POLLIN)
        warmup_deadline_ns = time.monotonic_ns() + int(
            args.warmup_seconds * 1_000_000_000
        )
        while time.monotonic_ns() < warmup_deadline_ns:
            source.read_imu_frame()
            if poller.poll(0):
                source._dequeue_frame()
        started_ns = time.monotonic_ns()
        deadline_ns = started_ns + int(args.duration_seconds * 1_000_000_000)
        while time.monotonic_ns() < deadline_ns and not stop_requested.is_set():
            read_started_ns = time.monotonic_ns()
            try:
                raw = source.read_imu_frame()
            except OSError:
                xu_errors += 1
                continue
            read_complete_ns = time.monotonic_ns()
            vendor_sequence, accel_g, gyro_dps = parse_imu_frame(raw)
            main_payload = raw[3:15]
            if last_main_payload is not None and main_payload == last_main_payload:
                duplicate_main_payloads += 1
            if last_imu_ns is not None:
                imu_intervals_ms.append((read_complete_ns - last_imu_ns) / 1_000_000)
            last_imu_ns = read_complete_ns
            last_main_payload = main_payload
            sample_index = len(imu_rows)
            accel_m_s2 = [value * STANDARD_GRAVITY_M_S2 for value in accel_g]
            gyro_rad_s = [math.radians(value) for value in gyro_dps]
            imu_rows.append(
                {
                    "sequence": sample_index,
                    "acquisition_ns": read_complete_ns,
                    "arrival_ns": read_complete_ns,
                    "clock_domain": "host_monotonic",
                    "timestamp_model": "xu_get_cur_read_complete",
                    "vendor_video_sequence": vendor_sequence,
                    "read_started_ns": read_started_ns,
                    "accel_m_s2": accel_m_s2,
                    "gyro_rad_s": gyro_rad_s,
                    "wire_group": "main_bytes_3_14",
                }
            )
            imu_payloads.append(main_payload)
            xu_rows.append(
                {
                    "sequence": sample_index,
                    "acquisition_ns": read_complete_ns,
                    "arrival_ns": read_complete_ns,
                    "clock_domain": "host_monotonic",
                    "vendor_video_sequence": vendor_sequence,
                }
            )
            xu_payloads.append(raw)

            if not poller.poll(0):
                continue
            frame = source._dequeue_frame()
            if frame is None:
                continue
            jpeg, driver_timestamp_ns, driver_sequence = frame
            native_video_frames += 1
            if last_native_sequence is not None and driver_sequence > last_native_sequence + 1:
                video_sequence_gaps += driver_sequence - last_native_sequence - 1
            last_native_sequence = driver_sequence
            if jpeg is None:
                continue
            retained_video_frames += 1
            if last_video_ns is not None:
                video_intervals_ms.append(
                    (driver_timestamp_ns - last_video_ns) / 1_000_000
                )
            last_video_ns = driver_timestamp_ns
            video_writer.append(
                {
                    "sequence": retained_video_frames - 1,
                    "driver_sequence": driver_sequence,
                    "acquisition_ns": driver_timestamp_ns,
                    "arrival_ns": time.monotonic_ns(),
                    "clock_domain": "v4l2_driver_monotonic",
                    "encoding": "MJPG",
                    "width": 3840,
                    "height": 1080,
                    "layout": "side_by_side_unverified_order",
                },
                jpeg,
            )
        ended_ns = time.monotonic_ns()
    finally:
        try:
            source.close(require_success=True)
        finally:
            video_writer.close()
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)

    write_sensor_stream(output, "ego.imu.main", imu_rows, imu_payloads)
    write_sensor_stream(output, "ego.xu.packet", xu_rows, xu_payloads)

    elapsed_seconds = (ended_ns - started_ns) / 1_000_000_000
    imu_hz = measured_rate(
        len(imu_rows),
        imu_rows[0]["acquisition_ns"] if imu_rows else None,
        imu_rows[-1]["acquisition_ns"] if imu_rows else None,
    )
    retained_video_hz = measured_rate(
        retained_video_frames,
        None if retained_video_frames == 0 else last_video_ns - int(sum(video_intervals_ms) * 1_000_000),
        last_video_ns,
    )
    native_video_hz = native_video_frames / elapsed_seconds
    stage_acceptance = build_stage_acceptance(
        stage=args.stage,
        requested_duration_seconds=args.duration_seconds,
        duration_seconds=elapsed_seconds,
        native_video_hz=native_video_hz,
        retained_video_hz=retained_video_hz,
        video_sequence_gaps=video_sequence_gaps,
        imu_hz=imu_hz,
        xu_errors=xu_errors,
        operator_aborted=stop_requested.is_set(),
    )
    report = {
        "schema": "ego.x5.2uq2.calibration_capture.v2",
        "status": stage_acceptance["status"],
        "stage": args.stage,
        "operator_aborted": stop_requested.is_set(),
        "duration_seconds": elapsed_seconds,
        "device": args.device,
        "video": {
            "native_frames": native_video_frames,
            "native_hz": native_video_hz,
            "retained_frames": retained_video_frames,
            "retained_hz": retained_video_hz,
            "sequence_gaps": video_sequence_gaps,
            "interval_ms_p50": statistics.median(video_intervals_ms) if video_intervals_ms else None,
            "interval_ms_p99": percentile(video_intervals_ms, 0.99),
            "interval_ms_max": max(video_intervals_ms) if video_intervals_ms else None,
        },
        "imu": {
            "samples": len(imu_rows),
            "hz": imu_hz,
            "xu_errors": xu_errors,
            "duplicate_main_payloads": duplicate_main_payloads,
            "interval_ms_p50": statistics.median(imu_intervals_ms) if imu_intervals_ms else None,
            "interval_ms_p95": percentile(imu_intervals_ms, 0.95),
            "interval_ms_p99": percentile(imu_intervals_ms, 0.99),
            "interval_ms_max": max(imu_intervals_ms) if imu_intervals_ms else None,
            "timestamp_model": "host_monotonic_xu_get_cur_read_complete",
            "published_group": "main_bytes_3_14_only",
            "backup_group_policy": "raw_packet_only_not_published",
        },
        "stage_acceptance": stage_acceptance,
    }
    report["next_action"] = build_next_action(args, stage_acceptance)
    report_path = output / "capture_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    files = sorted(path for path in output.iterdir() if path.is_file())
    manifest = {
        "schema": "ego.x5.2uq2.calibration_manifest.v1",
        "files": {
            path.name: {"size": path.stat().st_size, "sha256": sha256(path)}
            for path in files
        },
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/ego-recorder-camera")
    parser.add_argument("--source-root", default="/home/pi/slam")
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-seconds", type=float, required=True)
    parser.add_argument("--warmup-seconds", type=float, default=0.5)
    parser.add_argument("--video-hz", type=int, default=30)
    parser.add_argument(
        "--stage",
        choices=("preflight", "stereo", "cam-imu"),
        default="preflight",
    )
    parser.add_argument("--x5-address", default="192.168.113.32")
    parser.add_argument(
        "--pc-artifact-root",
        default="/home/robot/three-device-slam/artifacts/ego_calibration",
    )
    return parser.parse_args()


def main() -> int:
    report = capture(parse_args())
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    print_stage_summary(report)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
