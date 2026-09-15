"""Capture one D435i Ego and one D405 UMI under one RSUSB owner thread."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import time
import uuid
from bisect import bisect_left
from pathlib import Path

from three_device_slam.core import AppendOnlySessionWriter, SensorRecord
from three_device_slam.devices.d405_umi import worker as d405
from three_device_slam.devices.d405_umi.imu.imu_reader import ImuReader
from three_device_slam.devices.d405_umi.recorder.recorder import UnitRecorder
from three_device_slam.devices.d435i_ego import worker as d435
from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics
from three_device_slam.devices.d435i_ego.capture import (
    D435iContract,
    StreamSample,
    build_capture_acceptance,
    configure_streams,
    evaluate_stereo_pairing,
    evaluate_stream,
    is_warmup_sample,
)
from three_device_slam.synchronization.clock_mapping import DeviceClockTracker


SCHEMA = "ego.rsusb_pair.acceptance.v2"
GROUP_SCHEMA = "ego.rsusb_group.acceptance.v1"
UMI_WRITER_QUEUE_DEPTH = 128
WARMUP_TIMEOUT_S = 20.0
START_GUARD_NS = 500_000_000
PAIR_NEAREST_MAX_NS = 20_000_000
IMU_STABLE_WARMUP_NS = 1_000_000_000
CAMERA_POSTROLL_NS = 50_000_000
MOTION_SIGNAL_MIN_DEG_S = 3.0
MOTION_SIGNAL_MAX_DEG_S = 25.0
EGO_RAW_BYTES_PER_SECOND = 2 * 1280 * 720 * 30
UMI_RAW_BYTES_PER_SECOND = (3 + 2) * 1280 * 720 * 30
CAPTURE_DISK_HEADROOM_BYTES = 2 * 1024**3
PREVIEW_WINDOW_NAME = "Ego 与 UMI 双机预览（q/ESC停止）"


class RequiredHardwareUnavailable(RuntimeError):
    pass


class CaptureCancelled(RuntimeError):
    pass


def required_capture_free_bytes(
    duration_s: float, start_delay_s: float, umi_count: int
) -> int:
    """Conservatively budget uncompressed frames through warmup and postroll."""
    recorded_s = (
        WARMUP_TIMEOUT_S
        + start_delay_s
        + duration_s
        + CAMERA_POSTROLL_NS / 1e9
    )
    raw_rate = EGO_RAW_BYTES_PER_SECOND + umi_count * UMI_RAW_BYTES_PER_SECOND
    return math.ceil(recorded_s * raw_rate) + CAPTURE_DISK_HEADROOM_BYTES


def _existing_storage_path(path: Path) -> Path:
    candidate = path
    while not candidate.exists() and candidate != candidate.parent:
        candidate = candidate.parent
    return candidate


def _positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("value must be positive and finite")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Shared-owner D435i Ego + one D405 UMI raw capture"
    )
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--ego-serial", required=True)
    parser.add_argument("--ego-calibration-id", required=True)
    parser.add_argument("--umi-device-id", choices=("left", "right"), default="left")
    parser.add_argument("--umi-serial", required=True)
    parser.add_argument("--umi-port", required=True)
    parser.add_argument("--umi-calibration-id", required=True)
    parser.add_argument(
        "--umi-protocol",
        choices=("kt_ex9_37", "stm32_combined_v1"),
        required=True,
    )
    parser.add_argument("--right-serial")
    parser.add_argument("--right-port")
    parser.add_argument("--right-calibration-id")
    parser.add_argument(
        "--right-protocol",
        choices=("kt_ex9_37", "stm32_combined_v1"),
    )
    parser.add_argument("--duration", type=_positive, required=True)
    parser.add_argument(
        "--mode",
        choices=("bench_no_motion", "shared_world_motion", "device3_anchor_hil"),
        default="bench_no_motion",
        help="label the physical protocol used during the formal window",
    )
    parser.add_argument("--preview-hz", type=_positive, default=5.0)
    parser.add_argument(
        "--start-delay",
        type=_positive,
        default=10.0,
        help="seconds between joint readiness and the formal capture",
    )
    parser.add_argument("--no-preview", action="store_true")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    args = build_parser().parse_args(argv)
    for name in (
        "ego_serial",
        "ego_calibration_id",
        "umi_serial",
        "umi_port",
        "umi_calibration_id",
    ):
        if not getattr(args, name).strip():
            raise SystemExit(f"--{name.replace('_', '-')} must be non-empty")
    if args.ego_serial == args.umi_serial:
        raise SystemExit("Ego and UMI serials must be distinct")
    right_values = (
        args.right_serial,
        args.right_port,
        args.right_calibration_id,
        args.right_protocol,
    )
    if any(value is not None for value in right_values):
        if not all(isinstance(value, str) and value.strip() for value in right_values):
            raise SystemExit("all right UMI arguments are required together")
        if args.umi_device_id != "left":
            raise SystemExit("two-UMI mode requires the primary UMI to be left")
        if args.right_serial in {args.ego_serial, args.umi_serial}:
            raise SystemExit("Ego, left UMI, and right UMI serials must be distinct")
        if args.right_port == args.umi_port:
            raise SystemExit("left and right UMI ports must be distinct")
    if args.preview_hz > 30:
        raise SystemExit("--preview-hz must be <= 30")
    if args.start_delay > 30:
        raise SystemExit("--start-delay must be <= 30")
    if args.mode == "shared_world_motion" and args.duration < 30:
        raise SystemExit("shared_world_motion requires at least 30 seconds")
    if args.mode == "device3_anchor_hil":
        if args.umi_device_id != "right" or args.right_serial is not None:
            raise SystemExit("device3_anchor_hil requires exactly one right UMI")
        if args.duration < 40:
            raise SystemExit("device3_anchor_hil requires at least 40 seconds")
    return args


def capture_cue(elapsed_s: float, duration_s: float, mode: str) -> str:
    """Return the operator instruction shown during the formal capture."""
    if mode == "device3_anchor_hil":
        if elapsed_s < 10.0:
            return "Ego和设备3完全静止，确认Ego看清设备3的ID1，建立世界锚点"
        if elapsed_s < 22.0:
            return "Ego保持固定；缓慢整体移动设备3，前后和左右各20至40厘米"
        if elapsed_s < 32.0:
            return "Ego保持固定；缓慢左右转动设备3约20至30度，再小幅上下俯仰"
        if elapsed_s < 37.0:
            return "Ego保持固定；让ID1短暂离开视野约2秒，然后重新完整出现"
        return "设备3回到起点附近并完全静止；Ego和AprilGrid都不要移动"
    if mode != "shared_world_motion":
        return "保持静止"
    if duration_s >= 100.0:
        if elapsed_s < 10.0:
            return "确认两台设备锁紧且都看见AprilGrid和远近纹理，整体静止"
        if elapsed_s < 30.0:
            return "两台设备锁紧后连续整体前后、左右平移，速度约0.15至0.30米/秒；各至少0.5米"
        if elapsed_s < 50.0:
            return "缓慢整体左右偏航，各方向至少25度；保持共同视野"
        if elapsed_s < 70.0:
            return "缓慢整体上下俯仰，各方向至少20度；保持共同视野"
        if elapsed_s < 85.0:
            return "缓慢整体左右侧倾，各方向至少15度；保持共同视野"
        if elapsed_s < 95.0:
            return "整体缓慢回到起点附近，禁止调整两台设备的安装"
        return "回到起点，让两台设备都看见AprilGrid并完全保持静止"
    if elapsed_s < 10.0:
        return "确认两台设备锁紧在同一刚性支架上，整体保持静止"
    if elapsed_s < 30.0:
        return "缓慢整体平移刚性支架至少0.5米，禁止分别移动两台设备"
    if elapsed_s < duration_s - 10.0:
        return "缓慢整体转动刚性支架，并沿原路返回；保持共同视野"
    return "整体回到起点附近，不调整安装，两台设备保持静止"


def pre_capture_cue(
    remaining_s: float,
    total_delay_s: float = 10.0,
    mode: str = "bench_no_motion",
) -> str:
    """Guide adjustment while preserving a short static settling interval."""
    settle_s = 5.0 if total_delay_s >= 15.0 else 2.0
    if remaining_s > settle_s:
        if mode == "device3_anchor_hil":
            return "可调整Ego和设备3的位置；不要改变设备3相机、IMU和ID1的安装"
        return "可调整标定板或刚性支架整体位置，不要改变两台相机的相对安装"
    return "停止调整，正式采集即将开始，请完全保持静止"


def motion_signal_label(
    mode: str,
    elapsed_s: float,
    ego_current_rad_s: float,
    umi_current_deg_s: dict[str, float],
) -> str:
    """Summarize current IMU motion without replacing the later VIO motion gate."""
    if mode not in {"shared_world_motion", "device3_anchor_hil"} or elapsed_s < 10.0:
        return ""
    if mode == "device3_anchor_hil":
        ego_deg_s = math.degrees(float(ego_current_rad_s))
        ego_state = "静止正常" if ego_deg_s < MOTION_SIGNAL_MIN_DEG_S else "发生移动，请保持固定"
        if elapsed_s >= 37.0:
            def umi_state(value: float) -> str:
                return "静止正常" if value < MOTION_SIGNAL_MIN_DEG_S else "仍在移动，请停稳"
        else:
            def umi_state(value: float) -> str:
                if value > MOTION_SIGNAL_MAX_DEG_S:
                    return "过快，请减速"
                return "已检测" if value >= MOTION_SIGNAL_MIN_DEG_S else "转动不足"
        display_names = {"left": "设备2", "right": "设备3"}
        umi_text = " ".join(
            f"{display_names.get(name, name)}={umi_state(float(value))}"
            for name, value in umi_current_deg_s.items()
        )
        return f"当前状态：Ego={ego_state} {umi_text}"
    signals = {
        "ego": math.degrees(float(ego_current_rad_s)),
        **{name: float(value) for name, value in umi_current_deg_s.items()},
    }
    display_names = {"ego": "Ego", "left": "设备2", "right": "设备3"}
    def state(value: float) -> str:
        if value > MOTION_SIGNAL_MAX_DEG_S:
            return "过快，请减速"
        if value >= MOTION_SIGNAL_MIN_DEG_S:
            return "已检测"
        return "不足"

    return "当前转速：" + " ".join(
        f"{display_names.get(name, name)}={state(value)}"
        for name, value in signals.items()
    )


def motion_signal_status(
    mode: str,
    ego_peak_gyro_deg_s: float,
    umi_peak_gyro_deg_s: dict[str, float],
) -> str:
    values = [float(ego_peak_gyro_deg_s), *map(float, umi_peak_gyro_deg_s.values())]
    if mode == "bench_no_motion":
        return "PASS" if values and max(values) <= MOTION_SIGNAL_MIN_DEG_S else "FAIL"
    if mode == "device3_anchor_hil":
        return (
            "PASS"
            if umi_peak_gyro_deg_s
            and float(ego_peak_gyro_deg_s) <= MOTION_SIGNAL_MIN_DEG_S
            and min(map(float, umi_peak_gyro_deg_s.values())) >= MOTION_SIGNAL_MIN_DEG_S
            else "FAIL"
        )
    return "PASS" if values and min(values) >= MOTION_SIGNAL_MIN_DEG_S else "FAIL"


def preview_sources(ego_snapshot: dict, runtimes: list[dict]) -> list[tuple]:
    """Select one left-IR image per physical camera for the operator preview."""
    selected = []
    ego_image = ego_snapshot.get("latest_images", {}).get("ir_left")
    if ego_image is not None:
        selected.append(("Ego 左IR", ego_image))
    display_names = {"left": "设备2 左IR", "right": "设备3 左IR"}
    for runtime in runtimes:
        image = runtime["state"].get("latest_images", {}).get("ir_left")
        if image is not None:
            device_id = runtime["spec"]["device_id"]
            selected.append((display_names.get(device_id, f"{device_id} 左IR"), image))
    return selected


def _prepare_pair_preview(disabled: bool):
    if disabled:
        return None, None
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("DISPLAY is unavailable; use --no-preview")
    try:
        import cv2
        import numpy as np

        cv2.setNumThreads(1)
        cv2.namedWindow(PREVIEW_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.imshow(PREVIEW_WINDOW_NAME, np.zeros((360, 1280, 3), dtype=np.uint8))
        cv2.waitKey(1)
    except Exception as exc:
        raise RuntimeError(f"preview unavailable: {type(exc).__name__}") from exc
    return cv2, np


def _show_pair_preview(
    cv2,
    np,
    ego_snapshot: dict,
    runtimes: list[dict],
    label: str,
) -> bool:
    sources = preview_sources(ego_snapshot, runtimes)
    if sources:
        tile_width = 640 if len(sources) <= 2 else 426
        tile_height = round(tile_width * 720 / 1280)
        tiles = []
        for _, (payload, width, height, _) in sources:
            image = np.frombuffer(payload, dtype=np.uint8).reshape(height, width)
            tile = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            tiles.append(
                cv2.resize(
                    tile,
                    (tile_width, tile_height),
                    interpolation=cv2.INTER_AREA,
                )
            )
        mosaic = np.hstack(tiles)
        cv2.rectangle(mosaic, (0, 0), (mosaic.shape[1], 42), (0, 0, 0), -1)
        bottom_height = 76 if "\n" in label else 46
        cv2.rectangle(
            mosaic,
            (0, mosaic.shape[0] - bottom_height),
            (mosaic.shape[1], mosaic.shape[0]),
            (0, 0, 0),
            -1,
        )

        from PIL import Image, ImageDraw, ImageFont

        canvas = Image.fromarray(cv2.cvtColor(mosaic, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(canvas)
        font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
        title_font = ImageFont.truetype(font_path, 23)
        label_font = ImageFont.truetype(font_path, 24)
        for index, (title, _) in enumerate(sources):
            draw.text(
                (index * tile_width + 14, 6),
                title,
                font=title_font,
                fill=(0, 255, 0),
            )
        draw.multiline_text(
            (14, mosaic.shape[0] - bottom_height + 6),
            label,
            font=label_font,
            fill=(255, 255, 0),
            spacing=3,
        )
        mosaic = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)
        cv2.imshow(PREVIEW_WINDOW_NAME, mosaic)
    key = cv2.waitKey(1) & 0xFF
    return key in (27, ord("q"), ord("Q"))


def _close_pair_preview(cv2) -> None:
    if cv2 is None:
        return
    try:
        cv2.destroyWindow(PREVIEW_WINDOW_NAME)
        cv2.waitKey(1)
    except Exception:
        pass


def _record_umi_imu_sample(recorder, state: dict, sample) -> None:
    recorder.put_imu(sample)
    formal_start_ns = state["formal_start_ns"]
    if formal_start_ns is None or int(sample.ts * 1e9) < formal_start_ns:
        return
    gyro_norm = math.sqrt(sample.gx**2 + sample.gy**2 + sample.gz**2)
    state["latest_gyro_norm_deg_s"] = gyro_norm
    state["peak_gyro_norm_deg_s"] = max(
        state["peak_gyro_norm_deg_s"], gyro_norm
    )


def umi_specs(args: argparse.Namespace) -> list[dict[str, str]]:
    specs = [
        {
            "device_id": args.umi_device_id,
            "serial": args.umi_serial,
            "port": args.umi_port,
            "calibration_id": args.umi_calibration_id,
            "protocol": args.umi_protocol,
        }
    ]
    if args.right_serial is not None:
        specs.append(
            {
                "device_id": "right",
                "serial": args.right_serial,
                "port": args.right_port,
                "calibration_id": args.right_calibration_id,
                "protocol": args.right_protocol,
            }
        )
    return specs


def nearest_timestamp_deltas_ns(reference_ns, candidate_ns) -> list[int]:
    candidates = sorted(int(value) for value in candidate_ns)
    if not candidates:
        return []
    deltas = []
    for value in reference_ns:
        value = int(value)
        index = bisect_left(candidates, value)
        choices = []
        if index < len(candidates):
            choices.append(abs(candidates[index] - value))
        if index:
            choices.append(abs(candidates[index - 1] - value))
        deltas.append(min(choices))
    return deltas


def formal_window_samples(samples, start_ns: int, end_ns: int) -> list:
    return [
        sample
        for sample in samples
        if start_ns <= sample.acquisition_ns < end_ns
    ]


def counter_delta(start: dict, end: dict) -> dict:
    """Return non-negative counter changes between two snapshots."""
    return {
        key: max(0, int(end.get(key, 0)) - int(value))
        for key, value in start.items()
    }


def _percentile(values: list[int], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def cross_timing_report(reference_samples, candidate_samples) -> dict:
    deltas = nearest_timestamp_deltas_ns(
        [sample.acquisition_ns for sample in reference_samples],
        [sample.acquisition_ns for sample in candidate_samples],
    )
    return {
        "pairs": len(deltas),
        "nearest_delta_ms_p50": _percentile(deltas, 50) / 1e6 if deltas else None,
        "nearest_delta_ms_p95": _percentile(deltas, 95) / 1e6 if deltas else None,
        "nearest_delta_ms_max": max(deltas) / 1e6 if deltas else None,
        "threshold_ms_max": PAIR_NEAREST_MAX_NS / 1e6,
        "status": (
            "PASS"
            if deltas and max(deltas) <= PAIR_NEAREST_MAX_NS
            else "FAIL"
        ),
    }


def _atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    with temporary.open("x", encoding="utf-8") as file_handle:
        json.dump(payload, file_handle, ensure_ascii=False, indent=2, sort_keys=True)
        file_handle.write("\n")
        file_handle.flush()
        os.fsync(file_handle.fileno())
    temporary.replace(path)


def _device_inventory(rs, context) -> tuple[dict[str, str], dict[str, object]]:
    names, devices = {}, {}
    for device in context.query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        names[serial] = device.get_info(rs.camera_info.name)
        devices[serial] = device
    return names, devices


def _d405_calibration_snapshot(rs, device, profiles, exposure) -> dict:
    streams, by_key = {}, {}
    for profile in profiles:
        key = d405.stream_key_from_profile(profile)
        video = profile.as_video_stream_profile()
        intrinsics = video.get_intrinsics()
        by_key[key] = profile
        streams[key] = {
            "width": video.width(),
            "height": video.height(),
            "fps": profile.fps(),
            "format": str(profile.format()),
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "ppx": intrinsics.ppx,
            "ppy": intrinsics.ppy,
            "distortion_model": str(intrinsics.model),
            "coefficients": list(intrinsics.coeffs),
        }
    extrinsics = by_key["infrared_left"].get_extrinsics_to(
        by_key["infrared_right"]
    )
    return {
        "schema": "umi.d405.factory_calibration.v1",
        "device": d435._device_snapshot(rs, device),
        "streams": streams,
        "left_to_right": serialize_sdk_extrinsics(extrinsics),
        "ir_auto_exposure": exposure,
    }


def _record_d405_frame(
    rs,
    frame,
    *,
    device_id: str,
    mapping: dict,
    tracker: DeviceClockTracker,
    formal_start_ns: int | None,
    state: dict,
    writer,
) -> None:
    name = {
        "color": "color",
        "infrared_left": "ir_left",
        "infrared_right": "ir_right",
    }.get(d405.stream_key(frame))
    if name is None:
        return
    profile = frame.get_profile().as_video_stream_profile()
    arrival_ns = time.monotonic_ns()
    sequence = int(frame.get_frame_number())
    timestamp_ms = float(frame.get_timestamp())
    domain = str(frame.get_frame_timestamp_domain()).rsplit(".", 1)[-1]
    valid = domain == "global_time"
    if valid and name == "ir_left":
        # Design spec section 7: each device maintains its own continuous
        # clock mapping. ir_left is the reference stream; the three D405
        # video streams share one device clock and one frame-number domain
        # would be corrupted by mixing, so only ir_left feeds the tracker.
        tracker.add_sample(sequence, timestamp_ms)
    acquisition_ns = (
        tracker.map_acquisition_ns(timestamp_ms)
        if valid
        else 0
    )
    warmup = is_warmup_sample(
        formal_start_ns=formal_start_ns,
        acquisition_ns=acquisition_ns,
        timestamp_valid=valid,
    )
    payload = bytes(frame.get_data())
    if name == "ir_left":
        state["latest_images"][name] = (
            payload,
            profile.width(),
            profile.height(),
            sequence,
        )
    record = SensorRecord(
        stream_id=f"{device_id}.{name}",
        sequence=sequence,
        acquisition_ns=acquisition_ns,
        arrival_ns=arrival_ns,
        clock_domain=(
            "realsense_global_time_mapped_to_host_monotonic"
            if valid
            else f"realsense_unverified_{domain}"
        ),
        warmup=warmup,
        valid=valid,
        metadata={
            "device_timestamp_ms": timestamp_ms,
            "timestamp_domain": domain,
            "wall_to_monotonic_offset_ns": mapping["wall_to_monotonic_offset_ns"],
            "mapping_uncertainty_ns": mapping["uncertainty_ns"],
            **tracker.mapping_evidence(),
            "encoding": "YUYV" if name == "color" else "Y8",
            "units": "uint8",
            "width": profile.width(),
            "height": profile.height(),
        },
    )
    if not writer.submit(record, payload):
        state["writer_queue_drops"] += 1
        state["capture_error"] = state["capture_error"] or "writer_queue_overflow"
        return
    sample = StreamSample(
        sequence=sequence,
        acquisition_ns=acquisition_ns,
        arrival_ns=arrival_ns,
        timestamp_domain=domain,
        width=profile.width(),
        height=profile.height(),
    )
    state["total_counts"][name] += 1
    if valid:
        state["all_samples"][name].append(sample)
    if not warmup:
        state["samples"][name].append(sample)


def _imu_error_counters(reader: ImuReader) -> dict:
    return {
        key: int(getattr(reader, key, 0))
        for key in d405.IMU_TRANSPORT_COUNTER_KEYS
        if key != "frames_ok"
    }


def run_pair(args: argparse.Namespace, stop_requested=lambda: False) -> dict:
    session = args.session.resolve()
    if session.exists():
        raise FileExistsError(f"session already exists: {session}")
    specs = umi_specs(args)
    required_bytes = required_capture_free_bytes(
        args.duration, args.start_delay, len(specs)
    )
    storage_path = _existing_storage_path(session.parent)
    free_bytes = shutil.disk_usage(storage_path).free
    if free_bytes < required_bytes:
        raise RuntimeError(
            "insufficient free storage for raw capture: "
            f"free={free_bytes / 1e9:.1f} GB "
            f"required={required_bytes / 1e9:.1f} GB "
            f"path={storage_path}"
        )
    session.mkdir(parents=True)
    ego_root = session / "ego"
    rs = d435._load_realsense()
    d405._load_hardware_modules()
    context = rs.context()
    inventory, devices = _device_inventory(rs, context)
    if "D435I" not in inventory.get(args.ego_serial, "").upper():
        raise RequiredHardwareUnavailable(
            f"D435i identity unavailable: {args.ego_serial}"
        )
    for spec in specs:
        if "D405" not in inventory.get(spec["serial"], "").upper():
            raise RequiredHardwareUnavailable(
                f"{spec['device_id']} D405 identity unavailable: {spec['serial']}"
            )

    mapping = d435._sample_wall_to_monotonic_mapping()
    # Design spec section 7: each UMI device maintains its own continuous
    # clock mapping (startup intercept + online rate estimate). The shared
    # `mapping` above remains the startup intercept source and the Ego's
    # fixed mapping (D435i drift observed ~1.2 ms/30 s, inside budget).
    contract = D435iContract()
    ego_state = d435._CaptureState()
    pipeline = rs.pipeline(context)
    config = rs.config()
    configure_streams(rs, config, args.ego_serial, contract)
    ego_device = devices[args.ego_serial]
    d435_emitter = d435._disable_infrared_emitter(rs, ego_device)
    d435_global = d435._enable_global_time(rs, ego_device)
    runtimes = []
    for spec in specs:
        root = session / spec["device_id"]
        state = {
            "samples": {name: [] for name in ("color", "ir_left", "ir_right")},
            "all_samples": {name: [] for name in ("color", "ir_left", "ir_right")},
            "total_counts": {name: 0 for name in ("color", "ir_left", "ir_right")},
            "writer_queue_drops": 0,
            "capture_error": None,
            "formal_start_ns": None,
            "latest_gyro_norm_deg_s": 0.0,
            "peak_gyro_norm_deg_s": 0.0,
            "latest_images": {},
        }
        device = devices[spec["serial"]]
        sensor = device.first_depth_sensor()
        profiles = d405.select_profiles(sensor)
        d405.configure_global_time(sensor)
        exposure = d405.configure_ir_auto_exposure(
            sensor,
            exposure_limit_us=d405.DEFAULT_IR_AUTO_EXPOSURE_LIMIT_US,
            gain_limit=d405.DEFAULT_IR_AUTO_GAIN_LIMIT,
        )
        recorder = UnitRecorder(
            "external_imu", root, save_depth=False, record_gripper=True, max_queue=8000
        )
        reader = ImuReader(
            spec["port"],
            baud=921600,
            warmup_frames=0,
            protocol=spec["protocol"],
            on_sample=lambda sample, recorder=recorder, state=state: (
                _record_umi_imu_sample(recorder, state, sample)
            ),
            on_raw_packet=recorder.put_raw_imu_packet,
            name=f"{spec['device_id']}_imu",
        )
        runtimes.append(
            {
                "spec": spec,
                "root": root,
                "state": state,
                "writer": None,
                "async": None,
                "device": device,
                "sensor": sensor,
                "profiles": profiles,
                "exposure": exposure,
                "queue": rs.frame_queue(
                    d405.MONITOR_QUEUE_CAPACITY, keep_frames=True
                ),
                "recorder": recorder,
                "reader": reader,
                "sensor_opened": False,
                "sensor_started": False,
                "imu_started": False,
                "manifest": None,
                "stable_errors": None,
                "stable_since_ns": None,
                "formal_error_start": None,
                "formal_error_end": None,
                "clock_tracker": DeviceClockTracker(
                    intercept_ns=mapping["wall_to_monotonic_offset_ns"],
                    expected_frame_delta_ns=33_333_333,
                ),
            }
        )

    cv2, np = _prepare_pair_preview(args.no_preview)
    ego_writer = None
    ego_async = None
    initialized_runtimes = []
    try:
        ego_writer = AppendOnlySessionWriter(ego_root)
        ego_async = d435._AsyncWriter(
            ego_writer, contract.writer_queue_depth, ego_state.fail
        )
        ego_async.start()
        for runtime in runtimes:
            runtime["writer"] = AppendOnlySessionWriter(runtime["root"])
            runtime["async"] = d435._AsyncWriter(
                runtime["writer"],
                UMI_WRITER_QUEUE_DEPTH,
                lambda reason, state=runtime["state"]: state.__setitem__(
                    "capture_error", reason
                ),
            )
            runtime["async"].start()
            initialized_runtimes.append(runtime)
    except BaseException:
        for runtime in initialized_runtimes:
            runtime["async"].close()
            runtime["writer"].close()
        if ego_async is not None:
            ego_async.close()
        if ego_writer is not None:
            ego_writer.close()
        _close_pair_preview(cv2)
        raise

    pipeline_started = False
    operator_aborted = False
    formal_start_ns = formal_end_ns = None
    warmup_started_ns = joint_ready_ns = None
    ego_manifest = None

    def ego_callback(frame) -> None:
        frames = list(frame.as_frameset()) if frame.is_frameset() else [frame]
        with ego_state.lock:
            start_ns = ego_state.formal_start_ns
        for item in frames:
            d435._record_frame(rs, item, mapping, start_ns, ego_state, ego_async)

    def process_umi_frames(start_ns) -> None:
        for runtime in runtimes:
            for frame in d405.drain_frame_queue(runtime["queue"]):
                _record_d405_frame(
                    rs,
                    frame,
                    device_id=runtime["spec"]["device_id"],
                    mapping=mapping,
                    tracker=runtime["clock_tracker"],
                    formal_start_ns=start_ns,
                    state=runtime["state"],
                    writer=runtime["async"],
                )

    try:
        active = pipeline.start(config, ego_callback)
        pipeline_started = True
        active_contract = d435._active_profile_contract(rs, active)
        if not d435._profile_contract_matches(active_contract, contract):
            raise RuntimeError("D435i active profile mismatch")
        _atomic_json(
            ego_root / "calibration.json",
            d435._calibration_snapshot(
                rs,
                active,
                ego_device,
                active_contract,
                mapping,
                d435_global,
                d435_emitter,
            ),
        )
        for runtime in runtimes:
            runtime["recorder"].start()
            if not runtime["reader"].start():
                raise RequiredHardwareUnavailable(
                    f"cannot open {runtime['spec']['device_id']} UMI IMU: "
                    f"{runtime['spec']['port']}"
                )
            runtime["imu_started"] = True
            runtime["sensor"].open(runtime["profiles"])
            runtime["sensor_opened"] = True
            runtime["sensor"].start(runtime["queue"])
            runtime["sensor_started"] = True
            _atomic_json(
                runtime["root"] / "calibration.json",
                _d405_calibration_snapshot(
                    rs,
                    runtime["device"],
                    runtime["profiles"],
                    runtime["exposure"],
                ),
            )
        print(
            "SETUP: one RSUSB owner started Ego first, then "
            + ", ".join(runtime["spec"]["device_id"] for runtime in runtimes)
        )
        print(f"SETUP: ego={args.ego_serial}")
        for runtime in runtimes:
            print(
                f"SETUP: {runtime['spec']['device_id']}="
                f"{runtime['spec']['serial']} "
                f"imu_protocol={runtime['spec']['protocol']}"
            )
        print(
            "PREVIEW: disabled; terminal progress remains enabled."
            if args.no_preview
            else f"PREVIEW: Ego + UMI left IR at {args.preview_hz:g} Hz."
        )

        warmup_started_ns = time.monotonic_ns()
        next_preview_ns = warmup_started_ns
        while True:
            if stop_requested():
                raise CaptureCancelled("operator cancelled during warmup")
            now_ns = time.monotonic_ns()
            process_umi_frames(None)
            ego_snapshot = ego_state.snapshot()
            base_ready = (
                all(value >= 30 for value in ego_snapshot["total_counts"].values())
                and all(
                    all(value >= 30 for value in runtime["state"]["total_counts"].values())
                    and runtime["reader"].frames_ok >= 500
                    and runtime["state"]["capture_error"] is None
                    for runtime in runtimes
                )
                and ego_snapshot["capture_error"] is None
            )
            for runtime in runtimes:
                errors = _imu_error_counters(runtime["reader"])
                runtime_ready = (
                    all(
                        value >= 30
                        for value in runtime["state"]["total_counts"].values()
                    )
                    and runtime["reader"].frames_ok >= 500
                    and runtime["state"]["capture_error"] is None
                )
                if runtime_ready:
                    if errors != runtime["stable_errors"]:
                        runtime["stable_errors"] = dict(errors)
                        runtime["stable_since_ns"] = now_ns
                else:
                    runtime["stable_errors"] = None
                    runtime["stable_since_ns"] = None
            ready = (
                base_ready
                and all(
                    runtime["stable_since_ns"] is not None
                    and now_ns - runtime["stable_since_ns"] >= IMU_STABLE_WARMUP_NS
                    for runtime in runtimes
                )
            )
            if ready and now_ns - warmup_started_ns >= 3_000_000_000:
                joint_ready_ns = now_ns
                formal_start_ns = now_ns + int(args.start_delay * 1e9)
                formal_end_ns = formal_start_ns + int(args.duration * 1e9)
                with ego_state.lock:
                    ego_state.formal_start_ns = formal_start_ns
                for runtime in runtimes:
                    runtime["state"]["formal_start_ns"] = formal_start_ns
                break
            if now_ns - warmup_started_ns >= int(WARMUP_TIMEOUT_S * 1e9):
                raise RuntimeError(
                    "RSUSB group warmup timeout: "
                    f"ego={ego_snapshot['total_counts']} "
                    + " ".join(
                        f"{runtime['spec']['device_id']}="
                        f"{runtime['state']['total_counts']} "
                        f"imu={runtime['reader'].frames_ok} "
                        f"imu_errors={_imu_error_counters(runtime['reader'])}"
                        for runtime in runtimes
                    )
                )
            if cv2 is not None and now_ns >= next_preview_ns:
                if _show_pair_preview(
                    cv2, np, ego_snapshot, runtimes, "正在初始化两台设备"
                ):
                    operator_aborted = True
                    break
                next_preview_ns = now_ns + int(1e9 / args.preview_hz)
            time.sleep(0.001)

        while not operator_aborted and time.monotonic_ns() < formal_start_ns:
            if stop_requested():
                raise CaptureCancelled("operator cancelled before formal start")
            process_umi_frames(formal_start_ns)
            now_ns = time.monotonic_ns()
            for runtime in runtimes:
                if _imu_error_counters(runtime["reader"]) != runtime["stable_errors"]:
                    raise RuntimeError(
                        f"{runtime['spec']['device_id']} UMI IMU transport "
                        "error during start guard"
                    )
            if cv2 is not None and now_ns >= next_preview_ns:
                remaining_s = max(0.0, (formal_start_ns - now_ns) / 1e9)
                if _show_pair_preview(
                    cv2,
                    np,
                    ego_state.snapshot(),
                    runtimes,
                    f"准备完成｜{remaining_s:04.1f}秒后开始｜"
                    f"{pre_capture_cue(remaining_s, args.start_delay, args.mode)}",
                ):
                    operator_aborted = True
                    break
                next_preview_ns = now_ns + int(1e9 / args.preview_hz)
            time.sleep(0.001)
        if not operator_aborted:
            # The setup countdown deliberately permits physical adjustment.  Only
            # formal-window motion belongs in the acceptance signal.
            with ego_state.lock:
                ego_state.peak_gyro_norm_rad_s = 0.0
            for runtime in runtimes:
                runtime["state"]["peak_gyro_norm_deg_s"] = 0.0
                runtime["formal_error_start"] = _imu_error_counters(
                    runtime["reader"]
                )
            print(f"RECORDING: shared formal_start_ns={formal_start_ns}")
        next_progress_ns = formal_start_ns + 1_000_000_000
        while not operator_aborted and time.monotonic_ns() < formal_end_ns:
            if stop_requested():
                raise CaptureCancelled("operator cancelled during formal capture")
            process_umi_frames(formal_start_ns)
            now_ns = time.monotonic_ns()
            if cv2 is not None and now_ns >= next_preview_ns:
                elapsed_s = (now_ns - formal_start_ns) / 1e9
                ego_snapshot = ego_state.snapshot()
                motion_label = motion_signal_label(
                    args.mode,
                    elapsed_s,
                    ego_snapshot["latest_gyro_norm_rad_s"],
                    {
                        runtime["spec"]["device_id"]: runtime["state"][
                            "latest_gyro_norm_deg_s"
                        ]
                        for runtime in runtimes
                    },
                )
                label = (
                    f"双机采集 {elapsed_s:04.1f}/{args.duration:.1f}秒｜"
                    f"{capture_cue(elapsed_s, args.duration, args.mode)}"
                )
                if motion_label:
                    label += f"\n{motion_label}"
                if _show_pair_preview(cv2, np, ego_snapshot, runtimes, label):
                    operator_aborted = True
                    break
                next_preview_ns = now_ns + int(1e9 / args.preview_hz)
            if now_ns >= next_progress_ns:
                elapsed_s = (now_ns - formal_start_ns) / 1e9
                ego_snapshot = ego_state.snapshot()
                motion_label = motion_signal_label(
                    args.mode,
                    elapsed_s,
                    ego_snapshot["latest_gyro_norm_rad_s"],
                    {
                        runtime["spec"]["device_id"]: runtime["state"][
                            "latest_gyro_norm_deg_s"
                        ]
                        for runtime in runtimes
                    },
                )
                print(
                    f"PROGRESS: {elapsed_s:.1f}/"
                    f"{args.duration:.1f}s "
                    f"action={capture_cue(elapsed_s, args.duration, args.mode)!r} "
                    f"motion={motion_label or 'WAIT'} "
                    f"ego={ego_snapshot['total_counts']} "
                    + " ".join(
                        f"{runtime['spec']['device_id']}="
                        f"{runtime['state']['total_counts']} "
                        f"imu={runtime['reader'].frames_ok}"
                        for runtime in runtimes
                    )
                )
                next_progress_ns += 1_000_000_000
            time.sleep(0.001)
        for runtime in runtimes:
            runtime["formal_error_end"] = _imu_error_counters(runtime["reader"])
        postroll_end_ns = formal_end_ns + CAMERA_POSTROLL_NS
        while time.monotonic_ns() < postroll_end_ns:
            process_umi_frames(formal_start_ns)
            time.sleep(0.001)
    finally:
        for runtime in reversed(runtimes):
            if runtime["sensor_started"]:
                try:
                    runtime["sensor"].stop()
                except Exception as exc:
                    runtime["state"]["capture_error"] = (
                        runtime["state"]["capture_error"]
                        or f"sensor_stop_error:{type(exc).__name__}"
                    )
            if runtime["sensor_opened"]:
                try:
                    runtime["sensor"].close()
                except Exception as exc:
                    runtime["state"]["capture_error"] = (
                        runtime["state"]["capture_error"]
                        or f"sensor_close_error:{type(exc).__name__}"
                    )
        if pipeline_started:
            try:
                pipeline.stop()
            except Exception as exc:
                ego_state.fail(f"pipeline_stop_error:{type(exc).__name__}")
        for runtime in runtimes:
            if runtime["imu_started"]:
                try:
                    runtime["reader"].stop()
                except Exception as exc:
                    runtime["state"]["capture_error"] = (
                        runtime["state"]["capture_error"]
                        or f"imu_stop_error:{type(exc).__name__}"
                    )
            try:
                runtime["recorder"].stop()
            except Exception as exc:
                runtime["state"]["capture_error"] = (
                    runtime["state"]["capture_error"]
                    or f"recorder_stop_error:{type(exc).__name__}"
                )
        try:
            ego_async.close()
        except Exception as exc:
            ego_state.fail(f"async_close_error:{type(exc).__name__}")
        for runtime in runtimes:
            try:
                runtime["async"].close()
            except Exception as exc:
                runtime["state"]["capture_error"] = (
                    runtime["state"]["capture_error"]
                    or f"async_close_error:{type(exc).__name__}"
                )
        try:
            ego_manifest = ego_writer.close()
        except Exception as exc:
            ego_state.fail(f"writer_close_error:{type(exc).__name__}")
        for runtime in runtimes:
            try:
                runtime["manifest"] = runtime["writer"].close()
            except Exception as exc:
                runtime["state"]["capture_error"] = (
                    runtime["state"]["capture_error"]
                    or f"writer_close_error:{type(exc).__name__}"
                )
        _close_pair_preview(cv2)

    ego_snapshot = ego_state.snapshot()
    ego_formal = {
        name: formal_window_samples(samples, formal_start_ns, formal_end_ns)
        for name, samples in ego_snapshot["samples"].items()
    }
    ego_streams = {
        "ir_left": evaluate_stream(
            ego_formal["ir_left"],
            expected_hz=30,
            hz_tolerance=0.05,
            expected_resolution=(1280, 720),
        ),
        "ir_right": evaluate_stream(
            ego_formal["ir_right"],
            expected_hz=30,
            hz_tolerance=0.05,
            expected_resolution=(1280, 720),
        ),
        "gyro": evaluate_stream(
            ego_formal["gyro"],
            expected_hz=200,
            hz_tolerance=0.05,
        ),
        "accel": evaluate_stream(
            ego_formal["accel"],
            expected_hz=200,
            hz_tolerance=0.05,
        ),
    }
    ego_pairing = evaluate_stereo_pairing(
        ego_formal["ir_left"], ego_formal["ir_right"]
    )
    ego_first_ns = (
        ego_formal["ir_left"][0].acquisition_ns
        if ego_formal["ir_left"]
        else None
    )
    ego_acceptance = build_capture_acceptance(
        ego_streams,
        writer_queue_drops=ego_snapshot["writer_queue_drops"],
        operator_aborted=operator_aborted,
        capture_error=ego_snapshot["capture_error"],
        stereo_pairing=ego_pairing,
        formal_start_ns=formal_start_ns,
        first_acquisition_ns=ego_first_ns,
    )
    ego_report = {
        "schema": "ego.d435i.acceptance.v1",
        "status": ego_acceptance["status"],
        "calibration_id": args.ego_calibration_id,
        "streams": ego_streams,
        "stereo_pairing": ego_pairing,
        "writer_queue_drops": ego_snapshot["writer_queue_drops"],
        "manifest": ego_manifest,
        "formal_evidence": {"first_acquisition_ns": ego_first_ns},
    }
    _atomic_json(ego_root / "acceptance.json", ego_report)

    umi_reports = {}
    umi_formal_by_device = {}
    for runtime in runtimes:
        spec = runtime["spec"]
        state = runtime["state"]
        formal = {
            name: formal_window_samples(samples, formal_start_ns, formal_end_ns)
            for name, samples in state["samples"].items()
        }
        umi_formal_by_device[spec["device_id"]] = formal
        streams = {
            name: evaluate_stream(
                samples,
                expected_hz=30,
                hz_tolerance=0.05,
                expected_resolution=(1280, 720),
            )
            for name, samples in formal.items()
        }
        imu_samples = d405.count_persisted_imu_samples(
            runtime["root"] / "external_imu" / "imu_ts.csv",
            start_ns=formal_start_ns,
            stop_ns=formal_end_ns,
        )
        imu_rate_hz = imu_samples / args.duration
        imu_errors = counter_delta(
            runtime["formal_error_start"] or {},
            runtime["formal_error_end"] or {},
        )
        recorder = runtime["recorder"]
        imu_ok = (
            399.0 <= imu_rate_hz <= 401.0
            and not any(imu_errors.values())
            and recorder.dropped == 0
            and recorder.first_write_error is None
        )
        umi_ok = (
            all(stream["status"] == "PASS" for stream in streams.values())
            and state["writer_queue_drops"] == 0
            and state["capture_error"] is None
            and runtime["manifest"] is not None
            and imu_ok
        )
        first_ns = formal["ir_left"][0].acquisition_ns if formal["ir_left"] else None
        report = {
            "schema": "umi.d405.append_only.acceptance.v1",
            "result": "PASS" if umi_ok else "FAIL",
            "calibration_id": spec["calibration_id"],
            "live_vins": {"imu_calibration": spec["calibration_id"]},
            "camera_streams": streams,
            "camera_clock": {
                "verified": all(
                    stream["timestamp_domains"] == ["global_time"]
                    for stream in streams.values()
                ),
                "required_streams": list(streams),
                "observed_domains": {
                    name: stream["timestamp_domains"]
                    for name, stream in streams.items()
                },
            },
            "joint_start": {
                "clock_domain": "host_monotonic",
                "scheduled_start_ns": formal_start_ns,
                "first_acquisition_ns": first_ns,
            },
            "imu": {
                "result": "PASS" if imu_ok else "FAIL",
                "protocol": spec["protocol"],
                "formal_samples": imu_samples,
                "rate_hz": imu_rate_hz,
                "errors": imu_errors,
                "startup_cumulative_errors": runtime["formal_error_start"],
                "recorder_drops": recorder.dropped,
            },
            "writer_queue_drops": state["writer_queue_drops"],
            "capture_error": state["capture_error"],
            "manifest": runtime["manifest"],
            "formal_evidence": {"first_acquisition_ns": first_ns},
        }
        umi_reports[spec["device_id"]] = report
        _atomic_json(runtime["root"] / "acceptance.json", report)

    cross_timings = {
        f"ego_to_{device_id}": cross_timing_report(
            ego_formal["ir_left"],
            next(
                runtime["state"]["all_samples"]["ir_left"]
                for runtime in runtimes
                if runtime["spec"]["device_id"] == device_id
            ),
        )
        for device_id, formal in umi_formal_by_device.items()
    }
    if set(umi_formal_by_device) == {"left", "right"}:
        cross_timings["left_to_right"] = cross_timing_report(
            umi_formal_by_device["left"]["ir_left"],
            next(
                runtime["state"]["all_samples"]["ir_left"]
                for runtime in runtimes
                if runtime["spec"]["device_id"] == "right"
            ),
        )
    ego_peak_gyro_deg_s = math.degrees(ego_snapshot["peak_gyro_norm_rad_s"])
    umi_peak_gyro_deg_s = {
        runtime["spec"]["device_id"]: runtime["state"]["peak_gyro_norm_deg_s"]
        for runtime in runtimes
    }
    motion_status = motion_signal_status(
        args.mode,
        ego_peak_gyro_deg_s,
        umi_peak_gyro_deg_s,
    )
    status = (
        "PASS"
        if ego_report["status"] == "PASS"
        and all(report["result"] == "PASS" for report in umi_reports.values())
        and all(timing["status"] == "PASS" for timing in cross_timings.values())
        and motion_status == "PASS"
        else "FAIL"
    )
    report = {
        "schema": GROUP_SCHEMA,
        "status": status,
        "mode": args.mode,
        "owner_model": "single_process_single_thread_realsense_lifecycle",
        "startup_order": ["ego", *[spec["device_id"] for spec in specs]],
        "formal_start_ns": formal_start_ns,
        "formal_end_ns": formal_end_ns,
        "warmup_started_ns": warmup_started_ns,
        "joint_ready_ns": joint_ready_ns,
        "duration_s": args.duration,
        "inventory": inventory,
        "ego_status": ego_report["status"],
        "umi_status": {
            device_id: item["result"] for device_id, item in umi_reports.items()
        },
        "cross_camera_timing": cross_timings,
        "operator_motion_signal": {
            "threshold_deg_s": MOTION_SIGNAL_MIN_DEG_S,
            "advisory_max_deg_s": MOTION_SIGNAL_MAX_DEG_S,
            "ego_peak_gyro_deg_s": ego_peak_gyro_deg_s,
            "umi_peak_gyro_deg_s": umi_peak_gyro_deg_s,
            "status": motion_status,
            "note": (
                "Formal-window static gate: every device peak must be <= threshold."
                if args.mode == "bench_no_motion"
                else (
                    "Ego peak must remain <= threshold while device3 must exceed it; "
                    "phase-level anchor continuity remains an offline gate."
                    if args.mode == "device3_anchor_hil"
                    else "Live operator aid only; VIO excursion remains the acceptance gate."
                )
            ),
        },
    }
    _atomic_json(session / "group_acceptance.json", report)
    if len(runtimes) == 1:
        compatibility = dict(report)
        compatibility["schema"] = SCHEMA
        compatibility["umi_status"] = next(iter(umi_reports.values()))["result"]
        compatibility["cross_camera_timing"] = next(iter(cross_timings.values()))
        _atomic_json(session / "pair_acceptance.json", compatibility)
        return compatibility
    return report


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        report = run_pair(args)
    except (FileExistsError, RuntimeError, OSError, ValueError) as exc:
        print(f"BLOCKED: {type(exc).__name__}: {exc}")
        return 3
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
