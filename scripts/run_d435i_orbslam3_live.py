#!/usr/bin/env python3
"""Run standalone D435i stereo-inertial ORB-SLAM3 with Chinese guidance."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from three_device_slam.devices.d435i_ego.capture import (  # noqa: E402
    D435iContract,
    configure_streams,
)
from three_device_slam.devices.d435i_ego.orbslam3_live import (  # noqa: E402
    OrbSlam3LiveProcess,
)
from three_device_slam.devices.d435i_ego.vins_export import factory_transforms  # noqa: E402
from three_device_slam.devices.d435i_ego.worker import (  # noqa: E402
    _disable_infrared_emitter,
    _enable_global_time,
    _load_realsense,
)
from three_device_slam.spatial.d435i_live_vins import OnlineImuCombiner  # noqa: E402


SERIAL = "327122078613"
CALIBRATION = (
    ROOT
    / "artifacts/product_sessions/"
    "three_device_orbslam3_ego_dynamic_recheck_20260915T060638.265192460Z_device3_right/"
    "ego/calibration.json"
)
ARTIFACT_ROOT = ROOT / "artifacts/live_hil"
WINDOW = "D435i Ego ORB-SLAM3 实时验收"
WINDOW_WIDTH = 1800
WINDOW_HEIGHT = 1000
FONT_PATHS = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.otf",
)
STATE_NAMES = {
    -1: "系统未就绪",
    0: "等待图像",
    1: "正在初始化",
    2: "跟踪正常",
    3: "短暂丢失",
    4: "跟踪丢失",
    5: "KLT跟踪正常",
}


def instruction(elapsed_s: float, duration_s: float) -> str:
    if elapsed_s < 10:
        return "保持Ego完全静止，不要移动"
    if elapsed_s < 25:
        return "平稳整体平移并做小角度转动，不要只原地旋转"
    if elapsed_s < duration_s - 10:
        return "缓慢走一段闭合路线，保持双红外看见有纹理环境"
    return "回到起点附近并完全静止"


def font(size: int):
    for path in FONT_PATHS:
        if Path(path).is_file():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


class LiveOwner:
    def __init__(self, rs, serial: str, calibration: dict, orb: OrbSlam3LiveProcess):
        self.rs = rs
        self.serial = serial
        self.calibration = calibration
        self.orb = orb
        _body_from_left, _body_from_right, accel_to_gyro = factory_transforms(calibration)
        self.accel_to_gyro = accel_to_gyro
        self.combiner = OnlineImuCombiner(self.accel_to_gyro)
        self.context = rs.context()
        matches = [
            item
            for item in self.context.query_devices()
            if item.get_info(rs.camera_info.serial_number) == serial
        ]
        if len(matches) != 1:
            raise RuntimeError(f"D435i {serial} 在线数量应为1，实际为{len(matches)}")
        self.device = matches[0]
        if "D435I" not in self.device.get_info(rs.camera_info.name).upper():
            raise RuntimeError("指定序列号不是D435i")
        self.pipeline = rs.pipeline(self.context)
        self.config = rs.config()
        configure_streams(rs, self.config, serial, D435iContract())
        self.lock = threading.RLock()
        self.latest_pair = None
        self.failure = None
        self.started_mono = None
        self.stereo_frames = 0
        self.gyro_frames = 0
        self.accel_frames = 0
        self.stereo_gaps = 0
        self.last_frame_number = None
        self.pipeline_started = False
        self.formal_active = False
        self.orb_active = False
        self.warmup_unmatched_stereo = 0
        self.formal_unmatched_stereo = 0

    @staticmethod
    def timestamp_ns(frame) -> int:
        return int(round(float(frame.get_timestamp()) * 1_000_000))

    def callback(self, frame) -> None:
        try:
            if frame.is_frameset():
                frameset = frame.as_frameset()
                left_frame = frameset.get_infrared_frame(1)
                right_frame = frameset.get_infrared_frame(2)
                if not left_frame or not right_frame:
                    return
                left_number = int(left_frame.get_frame_number())
                right_number = int(right_frame.get_frame_number())
                left_stamp = self.timestamp_ns(left_frame)
                right_stamp = self.timestamp_ns(right_frame)
                if left_number != right_number or abs(left_stamp - right_stamp) > 1_000_000:
                    with self.lock:
                        if self.formal_active:
                            self.formal_unmatched_stereo += 1
                        else:
                            self.warmup_unmatched_stereo += 1
                    return
                left = np.asanyarray(left_frame.get_data()).copy()
                right = np.asanyarray(right_frame.get_data()).copy()
                with self.lock:
                    self.latest_pair = (left, right)
                    formal_active = self.formal_active
                if not formal_active:
                    return
                with self.lock:
                    if self.last_frame_number is not None and left_number > self.last_frame_number + 1:
                        self.stereo_gaps += left_number - self.last_frame_number - 1
                    self.last_frame_number = left_number
                    self.stereo_frames += 1
                    orb_active = self.orb_active
                if orb_active:
                    self.orb.push_stereo(left_stamp, left, right)
                return
            motion = frame.as_motion_frame()
            data = motion.get_motion_data()
            vector = np.array((data.x, data.y, data.z), dtype=np.float64)
            timestamp_ns = self.timestamp_ns(frame)
            stream = frame.get_profile().stream_type()
            with self.lock:
                if not self.formal_active:
                    return
            if stream == self.rs.stream.gyro:
                with self.lock:
                    self.gyro_frames += 1
                    orb_active = self.orb_active
                samples = self.combiner.push_gyro(timestamp_ns, vector) if orb_active else ()
            elif stream == self.rs.stream.accel:
                with self.lock:
                    self.accel_frames += 1
                    orb_active = self.orb_active
                samples = self.combiner.push_accel(timestamp_ns, vector) if orb_active else ()
            else:
                return
            for sample in samples:
                self.orb.push_imu(sample)
        except Exception as exc:
            with self.lock:
                if self.failure is None:
                    self.failure = f"{type(exc).__name__}: {exc}"

    def start(self) -> None:
        global_time = _enable_global_time(self.rs, self.device)
        if not all(row.get("enabled") for row in global_time if row.get("supported")):
            raise RuntimeError("D435i global_time启用失败")
        emitter = _disable_infrared_emitter(self.rs, self.device)
        if emitter.get("readback") != 0:
            raise RuntimeError("D435i红外发射器关闭失败")
        active = self.pipeline.start(self.config, self.callback)
        self.pipeline_started = True
        actual = active.get_device().get_info(self.rs.camera_info.serial_number)
        if actual != self.serial:
            raise RuntimeError(f"打开了错误相机: {actual}")
        left = active.get_stream(self.rs.stream.infrared, 1).as_video_stream_profile()
        intrinsics = left.get_intrinsics()
        expected = self.calibration["intrinsics"]["ir_left"]
        observed = (intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        reference = (expected["width"], expected["height"], expected["fx"], expected["fy"], expected["ppx"], expected["ppy"])
        if not np.allclose(observed, reference, rtol=0, atol=1e-6):
            raise RuntimeError(f"D435i活动内参与冻结值不同: {observed}")
        self.started_mono = time.monotonic()

    def begin_formal(self) -> None:
        with self.lock:
            self.combiner = OnlineImuCombiner(self.accel_to_gyro)
            self.stereo_frames = 0
            self.gyro_frames = 0
            self.accel_frames = 0
            self.stereo_gaps = 0
            self.formal_unmatched_stereo = 0
            self.last_frame_number = None
            self.started_mono = time.monotonic()
            self.formal_active = True

    def begin_orb(self) -> None:
        with self.lock:
            if not self.formal_active or self.orb_active:
                raise RuntimeError("ORB输入阶段状态错误")
            self.combiner = OnlineImuCombiner(self.accel_to_gyro)
            self.orb_active = True

    def snapshot(self) -> dict:
        with self.lock:
            elapsed = max(1e-6, time.monotonic() - (self.started_mono or time.monotonic()))
            return {
                "failure": self.failure,
                "stereo_frames": self.stereo_frames,
                "stereo_rate_hz": self.stereo_frames / elapsed,
                "gyro_frames": self.gyro_frames,
                "gyro_rate_hz": self.gyro_frames / elapsed,
                "accel_frames": self.accel_frames,
                "accel_rate_hz": self.accel_frames / elapsed,
                "stereo_gaps": self.stereo_gaps,
                "warmup_unmatched_stereo": self.warmup_unmatched_stereo,
                "formal_unmatched_stereo": self.formal_unmatched_stereo,
                "imu_combiner": self.combiner.stats(),
            }

    def image_pair(self):
        with self.lock:
            return self.latest_pair

    def close(self) -> None:
        if self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False


def render(owner, orb, pose, elapsed_s: float, duration_s: float):
    pair = owner.image_pair()
    if pair is None:
        canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
    else:
        left, right = pair
        stereo = np.hstack((left, right))
        canvas = cv2.cvtColor(
            cv2.resize(stereo, (WINDOW_WIDTH, 506)), cv2.COLOR_GRAY2BGR
        )
        canvas = np.vstack(
            (
                canvas,
                np.zeros((WINDOW_HEIGHT - 506, WINDOW_WIDTH, 3), dtype=np.uint8),
            )
        )
    image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    large, small = font(48), font(32)
    draw.text((40, 535), f"{elapsed_s:05.1f}/{duration_s:.0f} 秒", font=large, fill=(255, 255, 255))
    draw.text((40, 605), instruction(elapsed_s, duration_s), font=large, fill=(255, 220, 0))
    if elapsed_s < 10:
        state = "静止测量阶段，暂不向ORB送图"
        xyz = "位置: --"
    elif pose is None:
        state = "等待ORB返回位姿"
        xyz = "位置: --"
    else:
        state = STATE_NAMES.get(pose.tracking_state, str(pose.tracking_state))
        value = pose.local_from_camera[:3, 3]
        xyz = f"局部位置 x={value[0]:+.3f}  y={value[1]:+.3f}  z={value[2]:+.3f} m"
    stats = orb.stats()
    draw.text((40, 685), f"ORB状态: {state}", font=small, fill=(80, 255, 120))
    draw.text((40, 735), xyz, font=small, fill=(255, 255, 255))
    draw.text(
        (40, 790),
        f"输入帧 {stats['stereo_sent']}  位姿 {stats['poses_received']}  "
        f"处理P95 {stats['processing_ms_p95'] or 0:.1f} ms  队列 {stats['write_queue']}",
        font=small,
        fill=(200, 220, 255),
    )
    draw.text((40, 900), "按 Q 或 ESC 可提前结束", font=small, fill=(180, 180, 180))
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def run(args) -> dict:
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    if calibration["device"]["serial"] != args.serial:
        raise ValueError("冻结标定与指定D435i序列号不同")
    orb = OrbSlam3LiveProcess(output / "orb")
    owner = None
    poses = deque(maxlen=6000)
    operator_aborted = False
    started = None
    try:
        print("正在加载ORB词典和地图引擎，请保持设备不动……", flush=True)
        orb.start()
        rs = _load_realsense()
        owner = LiveOwner(rs, args.serial, calibration, orb)
        owner.start()
        warmup_deadline = time.monotonic() + 2.0
        while time.monotonic() < warmup_deadline or owner.image_pair() is None:
            if owner.failure:
                raise RuntimeError(owner.failure)
            if time.monotonic() > warmup_deadline + 3.0:
                raise RuntimeError("D435i双红外预热超时")
            time.sleep(0.01)
        owner.begin_formal()
        started = time.monotonic()
        next_render = started
        orb_begun = False
        if not args.no_preview:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, WINDOW_WIDTH, WINDOW_HEIGHT)
            cv2.moveWindow(WINDOW, 20, 20)
        while time.monotonic() - started < args.duration:
            if owner.failure:
                raise RuntimeError(owner.failure)
            orb.check_alive()
            while True:
                pose = orb.poll_pose()
                if pose is None:
                    break
                poses.append(pose)
            now = time.monotonic()
            if not orb_begun and now - started >= 10.0:
                owner.begin_orb()
                orb_begun = True
            if not args.no_preview and now >= next_render:
                frame = render(owner, orb, poses[-1] if poses else None, now - started, args.duration)
                cv2.imshow(WINDOW, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    operator_aborted = True
                    break
                next_render = now + 1.0 / args.preview_hz
            time.sleep(0.002)
    finally:
        if owner is not None:
            owner.close()
        sensor = owner.snapshot() if owner is not None else {}
        orb.finish_input()
        deadline = time.monotonic() + 5.0
        while orb.poses_received < orb.stereo_sent and time.monotonic() < deadline:
            while True:
                pose = orb.poll_pose()
                if pose is None:
                    break
                poses.append(pose)
            time.sleep(0.01)
        before_stop = orb.stats()
        orb.stop()
        after_stop = orb.stats()
        cv2.destroyAllWindows()
    log_path = output / "orb/orbslam3.log"
    log = log_path.read_text(encoding="utf-8", errors="replace")
    elapsed = time.monotonic() - started if started is not None else 0.0
    checks = {
        "operator_completed": not operator_aborted,
        "sensor_callback": not sensor.get("failure"),
        "stereo_rate": 27.0 <= sensor.get("stereo_rate_hz", 0) <= 33.0,
        "gyro_rate": 180.0 <= sensor.get("gyro_rate_hz", 0) <= 220.0,
        "accel_rate": 180.0 <= sensor.get("accel_rate_hz", 0) <= 220.0,
        "stereo_no_gaps": sensor.get("stereo_gaps") == 0,
        "stereo_exact_pairing": sensor.get("formal_unmatched_stereo") == 0,
        "transport_complete": before_stop["stereo_sent"] == before_stop["poses_received"],
        "transport_no_failure": before_stop["failure"] is None,
        "child_clean_exit": orb._child is not None and orb._child.returncode == 0,
        "tracking_initialized": before_stop["tracking_ok_poses"] > 0,
        "no_bad_imu_reset": "Reset map because local mapper set the bad imu flag" not in log,
        "viba1_completed": "end VIBA 1" in log,
        "viba2_completed": "end VIBA 2" in log,
        "processing_realtime": (before_stop["processing_ms_p95"] or math.inf) <= 33.4,
        "latency_bounded": (before_stop["end_to_end_ms_p95"] or math.inf) <= 100.0,
    }
    report = {
        "schema": "three-device-slam.d435i-orbslam3-live-hil.v1",
        "status": "PROVISIONAL_LIVE_LOCAL_PASS" if all(checks.values()) else "FAIL",
        "serial": args.serial,
        "duration_s": elapsed,
        "checks": checks,
        "sensor": sensor,
        "orb": before_stop,
        "orb_after_stop": after_stop,
        "operator_aborted": operator_aborted,
        "limitations": [
            "local ORB pose is not yet aligned to the fixed AprilGrid world",
            "D435i-specific IMU noise and camera/IMU timing remain provisional",
        ],
    }
    (output / "live_hil_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default=SERIAL)
    parser.add_argument("--calibration", type=Path, default=CALIBRATION)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--preview-hz", type=float, default=10.0)
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.duration < 30 or not 1 <= args.preview_hz <= 30:
        parser.error("duration must be >=30 and preview-hz must be within [1,30]")
    report = run(args)
    return 0 if report["status"] == "PROVISIONAL_LIVE_LOCAL_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
