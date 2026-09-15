#!/usr/bin/env python3
"""Record a D435i as the temporary Ego stereo-inertial device."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import queue
import struct
import threading
import time
from contextlib import nullcontext
from pathlib import Path

from three_device_slam.core import (
    AppendOnlySessionWriter,
    BarrierDirectory,
    DeviceHeartbeat,
    SensorRecord,
)
from three_device_slam.core.session_lifecycle import producer_claim
from three_device_slam.core.session_writer import prepare_session_writer_root
from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics

from .capture import (
    D435iContract,
    StreamSample,
    build_capture_acceptance,
    configure_streams,
    derive_monotonic_acquisition_ns,
    evaluate_stereo_pairing,
    evaluate_stream,
    is_warmup_sample,
)


SCHEMA = "ego.d435i.capture.v1"
WINDOW_NAME = "D435i Ego - IR Left | IR Right (q/ESC stop)"


class DeviceUnavailableError(RuntimeError):
    pass


class _CaptureState:
    def __init__(self):
        self.lock = threading.Lock()
        self.formal_start_ns: int | None = None
        self.samples = {name: [] for name in ("ir_left", "ir_right", "gyro", "accel")}
        self.all_samples = {name: [] for name in self.samples}
        self.total_counts = {name: 0 for name in self.samples}
        self.last_arrival_ns = {name: None for name in self.samples}
        self.latest_images: dict[str, tuple[bytes, int, int, int]] = {}
        self.latest_gyro_norm_rad_s = 0.0
        self.peak_gyro_norm_rad_s = 0.0
        self.writer_queue_drops = 0
        self.capture_error: str | None = None

    def fail(self, reason: str) -> None:
        with self.lock:
            if self.capture_error is None:
                self.capture_error = reason

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "formal_start_ns": self.formal_start_ns,
                "samples": {name: list(values) for name, values in self.samples.items()},
                "all_samples": {
                    name: list(values) for name, values in self.all_samples.items()
                },
                "total_counts": dict(self.total_counts),
                "last_arrival_ns": dict(self.last_arrival_ns),
                "latest_images": dict(self.latest_images),
                "latest_gyro_norm_rad_s": self.latest_gyro_norm_rad_s,
                "peak_gyro_norm_rad_s": self.peak_gyro_norm_rad_s,
                "writer_queue_drops": self.writer_queue_drops,
                "capture_error": self.capture_error,
            }


class _AsyncWriter:
    def __init__(
        self,
        writer: AppendOnlySessionWriter,
        depth: int,
        on_error,
    ):
        self._writer = writer
        self._queue: queue.Queue = queue.Queue(maxsize=depth)
        self._sentinel = object()
        self._on_error = on_error
        self._failed = False
        self._thread = threading.Thread(target=self._run, name="d435i-writer", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def submit(self, record: SensorRecord, payload: bytes) -> bool:
        try:
            self._queue.put_nowait((record, payload))
        except queue.Full:
            return False
        return True

    def close(self) -> None:
        self._queue.put(self._sentinel)
        self._thread.join()

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._sentinel:
                    return
                if not self._failed:
                    record, payload = item
                    try:
                        self._writer.append(record, payload)
                    except (OSError, RuntimeError, ValueError) as exc:
                        self._failed = True
                        self._on_error(f"storage_error:{type(exc).__name__}")
            finally:
                self._queue.task_done()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="D435i temporary Ego: stereo IR 1280x720@30 + built-in IMU 200Hz"
    )
    parser.add_argument("--serial", required=True, help="exact D435i serial number")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--barrier-dir", type=Path)
    parser.add_argument("--device-id", choices=("ego",), default="ego")
    parser.add_argument("--calibration-id")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--preview-hz", type=float, default=5.0)
    parser.add_argument("--no-preview", action="store_true")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.serial.strip():
        parser.error("--serial must be non-empty")
    for name in ("warmup", "preview_hz"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.preview_hz > 30:
        parser.error("--preview-hz must be <= 30")
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration must be finite and positive")
    joint_mode = args.session is not None or args.barrier_dir is not None
    if joint_mode:
        if args.session is None or args.barrier_dir is None:
            parser.error("--session and --barrier-dir are both required in joint mode")
        if args.output is not None:
            parser.error("--output cannot be used with --session")
        if not isinstance(args.calibration_id, str) or not args.calibration_id.strip():
            parser.error("--calibration-id is required in joint mode")
    else:
        if args.output is None:
            parser.error("--output is required in standalone mode")
        if args.duration is None:
            parser.error("--duration is required in standalone mode")
    return args


def capture(args: argparse.Namespace, *, realsense_context=None) -> dict:
    rs = _load_realsense()
    contract = D435iContract()
    context = realsense_context if realsense_context is not None else rs.context()
    matches = [
        device
        for device in context.query_devices()
        if _device_info(device, rs.camera_info.serial_number) == args.serial
    ]
    if len(matches) != 1:
        raise DeviceUnavailableError(
            f"D435i serial={args.serial} expected exactly once, observed={len(matches)}"
        )
    device = matches[0]
    name = _device_info(device, rs.camera_info.name)
    if "D435I" not in name.upper():
        raise DeviceUnavailableError(
            f"serial={args.serial} is {name!r}, not an Intel RealSense D435i"
        )

    cv2, np = _prepare_preview(args.no_preview)
    joint_mode = args.session is not None
    barrier = (
        BarrierDirectory(_barrier_root(args.barrier_dir)) if joint_mode else None
    )
    output = (args.session / "ego" if joint_mode else args.output).resolve()
    if not joint_mode and output.exists():
        _close_preview(cv2)
        raise FileExistsError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    mapping = _sample_wall_to_monotonic_mapping()
    emitter_configuration = _disable_infrared_emitter(rs, device)
    global_time_sensors = _enable_global_time(rs, device)
    state = _CaptureState()
    writer = AppendOnlySessionWriter(output)
    async_writer = _AsyncWriter(writer, contract.writer_queue_depth, state.fail)
    async_writer.start()
    pipeline = rs.pipeline(context)
    config = rs.config()
    configure_streams(rs, config, args.serial, contract)
    operator_aborted = False
    pipeline_started = False
    calibration = None
    manifest = None
    stop_ns = time.monotonic_ns()

    def callback(frame) -> None:
        try:
            frames = list(frame.as_frameset()) if frame.is_frameset() else [frame]
            with state.lock:
                formal_start_ns = state.formal_start_ns
            for item in frames:
                _record_frame(
                    rs,
                    item,
                    mapping,
                    formal_start_ns,
                    state,
                    async_writer,
                )
        except Exception as exc:  # SDK callbacks must not unwind into librealsense.
            state.fail(f"callback_error:{type(exc).__name__}")

    try:
        print(
            f"SETUP: D435i serial={args.serial} IR-left/right="
            "1280x720@30 Y8 gyro=200Hz accel=200Hz"
        )
        print(f"SETUP: output={output}")
        print(
            "PREVIEW: disabled; terminal progress remains enabled."
            if args.no_preview
            else f"PREVIEW: {args.preview_hz:g} Hz live stereo view; q/ESC stops."
        )
        active_profile = pipeline.start(config, callback)
        pipeline_started = True
        active_contract = _active_profile_contract(rs, active_profile)
        if not _profile_contract_matches(active_contract, contract):
            state.fail("active_profile_mismatch")
        calibration = _calibration_snapshot(
            rs,
            active_profile,
            device,
            active_contract,
            mapping,
            global_time_sensors,
            emitter_configuration,
        )
        _atomic_write_json(output / "calibration.json", calibration)
        print("WARMUP: streaming all four inputs for joint stabilization...")

        warmup_started_ns = time.monotonic_ns()
        next_preview_ns = warmup_started_ns
        next_heartbeat_ns = warmup_started_ns
        warmup_deadline_ns = warmup_started_ns + int(
            _warmup_timeout_seconds(args.warmup, joint_mode=joint_mode) * 1e9
        )
        while True:
            now_ns = time.monotonic_ns()
            snapshot = state.snapshot()
            if snapshot["capture_error"] is not None:
                break
            ready = all(snapshot["total_counts"][name] > 0 for name in snapshot["total_counts"])
            local_ready = ready and now_ns - warmup_started_ns >= int(args.warmup * 1e9)
            if barrier is not None and now_ns >= next_heartbeat_ns:
                _write_heartbeat(barrier, snapshot, now_ns, local_ready)
                next_heartbeat_ns = now_ns + 50_000_000
            if local_ready:
                if barrier is None:
                    scheduled_start_ns = now_ns
                else:
                    scheduled_start_ns = barrier.read_start_ns()
                if scheduled_start_ns is not None:
                    if barrier is not None and scheduled_start_ns - now_ns < 100_000_000:
                        state.fail("stale_scheduled_start")
                        break
                    with state.lock:
                        state.formal_start_ns = scheduled_start_ns
                    print(
                        "READY: local warmup PASS; waiting for coordinator start."
                        if barrier is not None
                        else "RECORDING: warmup PASS; formal acquisition started."
                    )
                    while barrier is not None and time.monotonic_ns() < scheduled_start_ns:
                        now_ns = time.monotonic_ns()
                        snapshot = state.snapshot()
                        if snapshot["capture_error"] is not None:
                            break
                        if now_ns >= next_heartbeat_ns:
                            _write_heartbeat(barrier, snapshot, now_ns, True)
                            next_heartbeat_ns = now_ns + 50_000_000
                        if cv2 is not None and now_ns >= next_preview_ns:
                            if _show_preview(cv2, np, snapshot, "JOINT READY"):
                                operator_aborted = True
                                barrier.request_stop(now_ns, "operator_interrupt")
                                break
                            next_preview_ns = now_ns + int(1e9 / args.preview_hz)
                        time.sleep(0.005)
                    if (
                        barrier is not None
                        and not operator_aborted
                        and state.snapshot()["capture_error"] is None
                    ):
                        print("RECORDING: coordinator start crossed; formal acquisition started.")
                    break
            if now_ns >= warmup_deadline_ns:
                state.fail("warmup_timeout")
                break
            if cv2 is not None and now_ns >= next_preview_ns:
                if _show_preview(cv2, np, snapshot, "WARMUP"):
                    operator_aborted = True
                    break
                next_preview_ns = now_ns + int(1e9 / args.preview_hz)
            time.sleep(0.005)

        formal_start_ns = state.snapshot()["formal_start_ns"]
        if formal_start_ns is not None and not operator_aborted:
            formal_end_ns = (
                formal_start_ns + int(args.duration * 1e9)
                if args.duration is not None
                else None
            )
            next_progress_ns = formal_start_ns + 5_000_000_000
            next_preview_ns = formal_start_ns
            next_heartbeat_ns = formal_start_ns
            while True:
                now_ns = time.monotonic_ns()
                snapshot = state.snapshot()
                stop_record = barrier.read_stop() if barrier is not None else None
                if (
                    snapshot["capture_error"] is not None
                    or (formal_end_ns is not None and now_ns >= formal_end_ns)
                    or stop_record is not None
                ):
                    break
                if barrier is not None and now_ns >= next_heartbeat_ns:
                    _write_heartbeat(barrier, snapshot, now_ns, True)
                    next_heartbeat_ns = now_ns + 50_000_000
                if cv2 is not None and now_ns >= next_preview_ns:
                    elapsed = (now_ns - formal_start_ns) / 1e9
                    if _show_preview(cv2, np, snapshot, f"REC {elapsed:5.1f}s"):
                        operator_aborted = True
                        if barrier is not None:
                            barrier.request_stop(now_ns, "operator_interrupt")
                        break
                    next_preview_ns = now_ns + int(1e9 / args.preview_hz)
                if now_ns >= next_progress_ns:
                    elapsed = (now_ns - formal_start_ns) / 1e9
                    counts = {name: len(values) for name, values in snapshot["samples"].items()}
                    print(
                        f"PROGRESS: {elapsed:.1f}/"
                        f"{args.duration if args.duration is not None else 'open'}s "
                        f"ir={counts['ir_left']}/{counts['ir_right']} "
                        f"gyro={counts['gyro']} accel={counts['accel']} "
                        f"queue_drops={snapshot['writer_queue_drops']}"
                    )
                    next_progress_ns += 5_000_000_000
                time.sleep(0.005)
    except KeyboardInterrupt:
        operator_aborted = True
    except Exception as exc:
        state.fail(f"capture_error:{type(exc).__name__}:{exc}")
    finally:
        stop_ns = time.monotonic_ns()
        if pipeline_started:
            try:
                pipeline.stop()
            except Exception as exc:
                state.fail(f"pipeline_stop_error:{type(exc).__name__}")
        async_writer.close()
        try:
            manifest = writer.close()
        except (OSError, RuntimeError) as exc:
            state.fail(f"storage_finalize_error:{type(exc).__name__}")
        _close_preview(cv2)

    snapshot = state.snapshot()
    streams = {
        "ir_left": evaluate_stream(
            snapshot["samples"]["ir_left"],
            expected_hz=contract.camera_hz,
            hz_tolerance=0.05,
            expected_resolution=(contract.width, contract.height),
        ),
        "ir_right": evaluate_stream(
            snapshot["samples"]["ir_right"],
            expected_hz=contract.camera_hz,
            hz_tolerance=0.05,
            expected_resolution=(contract.width, contract.height),
        ),
        "gyro": evaluate_stream(
            snapshot["samples"]["gyro"],
            expected_hz=contract.gyro_hz,
            hz_tolerance=0.05,
        ),
        "accel": evaluate_stream(
            snapshot["samples"]["accel"],
            expected_hz=contract.accel_hz,
            hz_tolerance=0.05,
        ),
    }
    stereo_pairing = evaluate_stereo_pairing(
        snapshot["samples"]["ir_left"], snapshot["samples"]["ir_right"]
    )
    formal_start_ns = snapshot["formal_start_ns"]
    first_acquisition_ns = (
        snapshot["samples"]["ir_left"][0].acquisition_ns
        if snapshot["samples"]["ir_left"]
        else None
    )
    acceptance = build_capture_acceptance(
        streams,
        writer_queue_drops=snapshot["writer_queue_drops"],
        operator_aborted=operator_aborted,
        capture_error=snapshot["capture_error"],
        stereo_pairing=stereo_pairing,
        formal_start_ns=formal_start_ns,
        first_acquisition_ns=first_acquisition_ns,
    )
    calibration_hash = (
        _sha256(output / "calibration.json") if calibration is not None else None
    )
    calibration_id = args.calibration_id or calibration_hash
    report = {
        "schema": "ego.d435i.acceptance.v1",
        "capture_schema": SCHEMA,
        "status": acceptance["status"],
        "mode": "temporary_d435i_ego",
        "device": _device_snapshot(rs, device),
        "contract": {
            "infrared_left": "1280x720 Y8 @ 30 Hz",
            "infrared_right": "1280x720 Y8 @ 30 Hz",
            "gyro": "motion_xyz32f @ 200 Hz (rad/s)",
            "accel": "motion_xyz32f @ 200 Hz (m/s^2)",
        },
        "output": str(output),
        "warmup_seconds": args.warmup,
        "requested_formal_seconds": args.duration,
        "observed_formal_seconds": (
            max(0.0, (stop_ns - formal_start_ns) / 1e9)
            if formal_start_ns is not None
            else 0.0
        ),
        "clock_mapping": mapping,
        "streams": streams,
        "stereo_pairing": stereo_pairing,
        "counts": snapshot["total_counts"],
        "acceptance": acceptance,
        "operator_aborted": operator_aborted,
        "capture_error": snapshot["capture_error"],
        "writer_queue_drops": snapshot["writer_queue_drops"],
        "manifest": manifest,
        "calibration_id": calibration_id,
        "calibration_sha256": calibration_hash,
        "formal_evidence": {
            "first_acquisition_ns": first_acquisition_ns,
            "last_acquisition_ns": (
                snapshot["samples"]["ir_left"][-1].acquisition_ns
                if snapshot["samples"]["ir_left"]
                else None
            ),
        },
        "hashes": {
            "calibration_sha256": calibration_hash,
            "streams": (manifest or {}).get("streams", {}),
        },
    }
    if report["status"] != "PASS":
        report["reason"] = snapshot["capture_error"] or "acceptance_check_failed"
    _atomic_write_json(output / "capture_report.json", report)
    if joint_mode:
        _atomic_write_json(output / "acceptance.json", report)
        if report["status"] != "PASS":
            try:
                barrier.latch_failure("ego", report["reason"], time.monotonic_ns())
            except OSError:
                pass
    return report


def _record_frame(rs, frame, mapping, formal_start_ns, state, async_writer) -> None:
    profile = frame.get_profile()
    stream = profile.stream_type()
    index = profile.stream_index()
    if stream == rs.stream.infrared and index == 1:
        name = "ir_left"
    elif stream == rs.stream.infrared and index == 2:
        name = "ir_right"
    elif stream == rs.stream.gyro:
        name = "gyro"
    elif stream == rs.stream.accel:
        name = "accel"
    else:
        return

    arrival_ns = time.monotonic_ns()
    sequence = int(frame.get_frame_number())
    timestamp_ms = float(frame.get_timestamp())
    timestamp_domain = str(frame.get_frame_timestamp_domain()).rsplit(".", 1)[-1]
    timestamp_valid = timestamp_domain == "global_time"
    acquisition_ns = (
        derive_monotonic_acquisition_ns(
            device_timestamp_ms=timestamp_ms,
            wall_to_monotonic_offset_ns=mapping["wall_to_monotonic_offset_ns"],
        )
        if timestamp_valid
        else 0
    )
    warmup = is_warmup_sample(
        formal_start_ns=formal_start_ns,
        acquisition_ns=acquisition_ns,
        timestamp_valid=timestamp_valid,
    )
    width = height = None
    if name.startswith("ir_"):
        video = profile.as_video_stream_profile()
        width, height = int(video.width()), int(video.height())
        payload = bytes(frame.get_data())
        encoding = "Y8"
        units = "uint8"
    else:
        motion = frame.as_motion_frame().get_motion_data()
        payload = struct.pack("<fff", float(motion.x), float(motion.y), float(motion.z))
        encoding = "float32_le_xyz"
        units = "rad/s" if name == "gyro" else "m/s^2"

    record = SensorRecord(
        stream_id=f"ego.{name}",
        sequence=sequence,
        acquisition_ns=acquisition_ns,
        arrival_ns=arrival_ns,
        clock_domain=(
            "realsense_global_time_mapped_to_host_monotonic"
            if timestamp_valid
            else f"realsense_unverified_{timestamp_domain}"
        ),
        warmup=warmup,
        valid=timestamp_valid,
        metadata={
            "device_timestamp_ms": timestamp_ms,
            "timestamp_domain": timestamp_domain,
            "wall_to_monotonic_offset_ns": mapping["wall_to_monotonic_offset_ns"],
            "mapping_uncertainty_ns": mapping["uncertainty_ns"],
            "encoding": encoding,
            "units": units,
            "width": width,
            "height": height,
            "temporary_ego": True,
        },
    )
    if not async_writer.submit(record, payload):
        with state.lock:
            state.writer_queue_drops += 1
        state.fail("writer_queue_overflow")
        return

    sample = StreamSample(
        sequence=sequence,
        acquisition_ns=acquisition_ns,
        arrival_ns=arrival_ns,
        timestamp_domain=timestamp_domain,
        width=width,
        height=height,
    )
    with state.lock:
        state.total_counts[name] += 1
        state.last_arrival_ns[name] = arrival_ns
        if timestamp_valid:
            state.all_samples[name].append(sample)
        if not warmup:
            state.samples[name].append(sample)
            if name == "gyro":
                gyro_norm = math.sqrt(
                    float(motion.x) ** 2
                    + float(motion.y) ** 2
                    + float(motion.z) ** 2
                )
                state.latest_gyro_norm_rad_s = gyro_norm
                state.peak_gyro_norm_rad_s = max(
                    state.peak_gyro_norm_rad_s, gyro_norm
                )
        if name.startswith("ir_"):
            state.latest_images[name] = (payload, width, height, sequence)


def _load_realsense():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise DeviceUnavailableError("pyrealsense2 is unavailable") from exc
    return rs


def _prepare_preview(disabled: bool):
    if disabled:
        return None, None
    if not os.environ.get("DISPLAY"):
        raise DeviceUnavailableError("DISPLAY is unavailable; use --no-preview")
    try:
        import cv2
        import numpy as np

        cv2.setNumThreads(1)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        blank = np.zeros((360, 1280, 3), dtype=np.uint8)
        cv2.imshow(WINDOW_NAME, blank)
        cv2.waitKey(1)
    except Exception as exc:
        raise DeviceUnavailableError(f"preview unavailable: {type(exc).__name__}") from exc
    return cv2, np


def _show_preview(cv2, np, snapshot: dict, label: str) -> bool:
    latest = snapshot["latest_images"]
    if "ir_left" in latest and "ir_right" in latest:
        tiles = []
        for name, title in (("ir_left", "IR LEFT"), ("ir_right", "IR RIGHT")):
            payload, width, height, sequence = latest[name]
            image = np.frombuffer(payload, dtype=np.uint8).reshape(height, width)
            tile = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            tile = cv2.resize(tile, (640, 360), interpolation=cv2.INTER_AREA)
            cv2.putText(
                tile,
                f"{title} frame={sequence}",
                (16, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            tiles.append(tile)
        mosaic = np.hstack(tiles)
        if label.isascii():
            cv2.putText(
                mosaic,
                label,
                (16, 345),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )
        else:
            from PIL import Image, ImageDraw, ImageFont

            rgb = cv2.cvtColor(mosaic, cv2.COLOR_BGR2RGB)
            canvas = Image.fromarray(rgb)
            draw = ImageDraw.Draw(canvas)
            font = ImageFont.truetype(
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 24
            )
            lines = label.splitlines()
            top = 292 if len(lines) > 1 else 322
            draw.multiline_text(
                (16, top), label, font=font, fill=(255, 255, 0), spacing=3
            )
            mosaic = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)
        cv2.imshow(WINDOW_NAME, mosaic)
    key = cv2.waitKey(1) & 0xFF
    return key in (27, ord("q"), ord("Q"))


def _close_preview(cv2) -> None:
    if cv2 is None:
        return
    try:
        cv2.destroyWindow(WINDOW_NAME)
        cv2.waitKey(1)
    except Exception:
        pass


def _sample_wall_to_monotonic_mapping() -> dict:
    candidates = []
    for _ in range(9):
        wall_before = time.time_ns()
        monotonic = time.monotonic_ns()
        wall_after = time.time_ns()
        candidates.append(
            {
                "wall_to_monotonic_offset_ns":
                    (wall_before + wall_after) // 2 - monotonic,
                "uncertainty_ns": max(0, (wall_after - wall_before) // 2),
            }
        )
    best = min(candidates, key=lambda value: value["uncertainty_ns"])
    return {
        **best,
        "model": "acquisition_monotonic_ns = realsense_global_time_ms*1e6 - offset_ns",
        "samples": len(candidates),
    }


def _barrier_root(path: Path) -> Path:
    return path.parent if path.name == "barrier" else path


def _warmup_timeout_seconds(warmup_seconds: float, *, joint_mode: bool) -> float:
    return max(35.0, warmup_seconds + 8.0) if joint_mode else warmup_seconds + 8.0


def _write_heartbeat(barrier, snapshot: dict, now_ns: int, locally_ready: bool) -> None:
    recent = all(
        arrival_ns is not None and now_ns - arrival_ns <= 500_000_000
        for arrival_ns in snapshot["last_arrival_ns"].values()
    )
    healthy = locally_ready and recent and snapshot["capture_error"] is None
    reasons = () if healthy else ("warming_or_stale_stream",)
    barrier.write_heartbeat(DeviceHeartbeat("ego", now_ns, healthy, reasons))


def _enable_global_time(rs, device) -> list[dict]:
    configured = []
    for sensor in device.query_sensors():
        name = _device_info(sensor, rs.camera_info.name)
        supported = sensor.supports(rs.option.global_time_enabled)
        if supported:
            sensor.set_option(rs.option.global_time_enabled, 1.0)
        configured.append({"sensor": name, "supported": bool(supported), "enabled": bool(supported)})
    return configured


def _disable_infrared_emitter(rs, device) -> dict:
    """Disable the D435i pattern projector and verify the device readback."""
    sensor = device.first_depth_sensor()
    option = rs.option.emitter_enabled
    if not sensor.supports(option):
        raise RuntimeError("D435i emitter disable unsupported")
    sensor.set_option(option, 0.0)
    readback = float(sensor.get_option(option))
    if not math.isfinite(readback) or abs(readback) > 1e-6:
        raise RuntimeError(f"D435i emitter disable readback mismatch: {readback}")
    return {
        "sensor": _device_info(sensor, rs.camera_info.name),
        "option": "emitter_enabled",
        "requested": 0.0,
        "readback": readback,
        "status": "PASS",
    }


def _active_profile_contract(rs, active_profile) -> dict:
    left = active_profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
    right = active_profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
    gyro = active_profile.get_stream(rs.stream.gyro)
    accel = active_profile.get_stream(rs.stream.accel)
    return {
        "ir_left": {"width": left.width(), "height": left.height(), "fps": left.fps(), "format": str(left.format())},
        "ir_right": {"width": right.width(), "height": right.height(), "fps": right.fps(), "format": str(right.format())},
        "gyro": {"fps": gyro.fps(), "format": str(gyro.format())},
        "accel": {"fps": accel.fps(), "format": str(accel.format())},
    }


def _profile_contract_matches(active: dict, expected: D435iContract) -> bool:
    for name in ("ir_left", "ir_right"):
        value = active[name]
        if (value["width"], value["height"], value["fps"]) != (
            expected.width,
            expected.height,
            expected.camera_hz,
        ) or "y8" not in value["format"].lower():
            return False
    return (
        active["gyro"]["fps"] == expected.gyro_hz
        and active["accel"]["fps"] == expected.accel_hz
        and "motion_xyz32f" in active["gyro"]["format"].lower()
        and "motion_xyz32f" in active["accel"]["format"].lower()
    )


def _calibration_snapshot(
    rs,
    active,
    device,
    active_contract,
    mapping,
    global_time_sensors,
    emitter_configuration,
):
    left = active.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
    right = active.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
    gyro = active.get_stream(rs.stream.gyro)
    accel = active.get_stream(rs.stream.accel)
    return {
        "schema": "ego.d435i.factory_calibration.v1",
        "provenance": "librealsense device factory calibration; temporary Ego bring-up only",
        "device": _device_snapshot(rs, device),
        "active_profiles": active_contract,
        "intrinsics": {
            "ir_left": _intrinsics(left.get_intrinsics()),
            "ir_right": _intrinsics(right.get_intrinsics()),
        },
        "extrinsics": {
            "ir_left_to_ir_right": _extrinsics(left.get_extrinsics_to(right)),
            "gyro_to_ir_left": _extrinsics(gyro.get_extrinsics_to(left)),
            "accel_to_ir_left": _extrinsics(accel.get_extrinsics_to(left)),
            "gyro_to_accel": _extrinsics(gyro.get_extrinsics_to(accel)),
        },
        "extrinsic_convention": "SDK source_to_target: p_target = R*p_source + t",
        "clock_mapping": mapping,
        "global_time_configuration": global_time_sensors,
        "infrared_emitter_configuration": emitter_configuration,
    }


def _intrinsics(value) -> dict:
    return {
        "width": value.width,
        "height": value.height,
        "fx": value.fx,
        "fy": value.fy,
        "ppx": value.ppx,
        "ppy": value.ppy,
        "distortion_model": str(value.model),
        "coefficients": list(value.coeffs),
    }


def _extrinsics(value) -> dict:
    return serialize_sdk_extrinsics(value)


def _device_snapshot(rs, device) -> dict:
    return {
        "name": _device_info(device, rs.camera_info.name),
        "serial": _device_info(device, rs.camera_info.serial_number),
        "firmware": _device_info(device, rs.camera_info.firmware_version),
        "usb": _device_info(device, rs.camera_info.usb_type_descriptor),
        "physical_port": _device_info(device, rs.camera_info.physical_port),
    }


def _device_info(device, key) -> str:
    return device.get_info(key) if device.supports(key) else "unavailable"


def _atomic_write_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("x", encoding="utf-8") as file:
        json.dump(payload, file, indent=2, sort_keys=True)
        file.write("\n")
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)
    directory = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(args: argparse.Namespace, *, realsense_context=None) -> dict:
    claim = producer_claim(args.session, "ego") if args.session is not None else nullcontext()
    with claim:
        return capture(args, realsense_context=realsense_context)


def _write_joint_terminal_acceptance(
    args: argparse.Namespace, *, status: str, reason: str
) -> None:
    if args.session is None or status not in {"FAIL", "BLOCKED"}:
        return
    ego_directory = prepare_session_writer_root(Path(args.session) / "ego")
    path = ego_directory / "acceptance.json"
    if path.exists():
        return
    report = {
        "schema": "ego.d435i.acceptance.v1",
        "status": status,
        "reason": reason,
        "calibration_id": args.calibration_id,
        "formal_evidence": {},
        "hashes": {},
    }
    _atomic_write_json(path, report)
    try:
        barrier = BarrierDirectory(_barrier_root(args.barrier_dir))
        barrier.latch_failure("ego", reason, time.monotonic_ns())
    except OSError:
        pass


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        report = run(args)
    except DeviceUnavailableError as exc:
        try:
            _write_joint_terminal_acceptance(
                args, status="BLOCKED", reason="d435i_hardware_unavailable"
            )
        except (OSError, RuntimeError, ValueError):
            pass
        print(f"BLOCKED: {type(exc).__name__}: {exc}")
        return 3
    except (FileExistsError, ValueError) as exc:
        print(f"BLOCKED: {type(exc).__name__}: {exc}")
        return 3
    print(json.dumps(report, indent=2, sort_keys=True))
    if report["status"] == "PASS":
        print("NEXT: D435i Ego capture PASS; bind this worker in the three-device coordinator.")
        return 0
    print("STOP: D435i Ego capture FAIL; preserve the report and inspect failed checks.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
