#!/usr/bin/env python3
"""Live Ego-AprilGrid world anchoring for the accepted device3/right mount.

This temporary pair-only adapter consumes the signed device3 VINS topics,
runs D435i stereo-inertial ORB-SLAM3 from the same RealSense owner, observes
the fixed AprilGrid and mounted ID1, and publishes both moving devices in the
AprilGrid world.  It never refits calibration.  After the two anchors freeze,
neither device needs to keep the board in view.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import sys
import threading
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.analyze_tag_world_constraints import detect_grid_scaled
from scripts.color_aprilgrid_capture import make_detector
from scripts.fixed_factory_board_diagnostic import valid_detections
from three_device_slam.spatial.apriltag_detector import (
    AprilTagDetectorConfig,
    CameraCalibration,
    detect_apriltags,
)
from three_device_slam.spatial.device3_world_live import (
    AnchorCandidateWindow,
    TimedTransform,
    matrix_from_pose_components,
    nearest_transform,
    pose_components_from_matrix,
)
from three_device_slam.spatial.device3_world_runtime import (
    Device3WorldAnchorController,
    anchor_candidate_from_synced_observation,
    load_device3_runtime_profile,
)
from three_device_slam.spatial.d435i_live_vins import (
    FrozenSessionAnchorController,
    OnlineImuCombiner,
    ego_world_anchor_candidate,
)
from three_device_slam.spatial.se3 import invert
from three_device_slam.spatial.tag_world_constraints import solve_grid_pose
from three_device_slam.devices.d435i_ego.capture import (
    D435iContract,
    configure_streams as configure_d435i_streams,
)
from three_device_slam.devices.d435i_ego.orbslam3_live import (
    OrbSlam3LiveProcess,
    OrbSlam3Pose,
)
from three_device_slam.devices.d435i_ego.vins_export import factory_transforms
from three_device_slam.devices.d405_umi import worker as d405
from three_device_slam.devices.d405_umi.camera.realsense_capture import CameraFrame
from three_device_slam.devices.d405_umi.imu.imu_reader import ImuReader


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / "config/device3_right_world_runtime_pair_20260915.json"
DEFAULT_EGO_CALIBRATION = (
    ROOT
    / "artifacts/product_sessions/three_device_anchor_hil_20260914T215952_device3_right_run24/ego/calibration.json"
)
WINDOW_NAME = "Ego + 设备3 世界坐标实时运行"
WINDOW_WIDTH = 1800
WINDOW_HEIGHT = 1000
FONT_PATHS = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.otf",
)
EGO_VIO_ROOT = Path("/home/robot/ego_vio_humble")
DEVICE3_IMU_BY_ID = Path(
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "cc06413ab505f0118b03921272aab386-if00-port0"
)
DEVICE3_IMU_LEAD_GUARD_MS = -6.812
DEVICE3_CAMERA_IMU_TD_S = -0.009084889
CROSS_CAMERA_MAX_DELTA_MS = 20.0


def _assert_ego_anchor_epoch_valid(
    anchor_initialized: bool, inertial_ba2_ready: bool
) -> None:
    if anchor_initialized and not inertial_ba2_ready:
        raise RuntimeError(
            "Ego ORB地图在世界锚建立后发生重置；旧世界变换已作废，"
            "本次运行停止以防继续输出错误坐标"
        )


class SharedPairSensors:
    """Own both RealSense cameras in one SDK context and publish D405 to ROS."""

    def __init__(
        self,
        profile,
        ego_calibration,
        ego_document,
        ego_orb,
        ego_imu_callback,
    ):
        if str(EGO_VIO_ROOT) not in sys.path:
            sys.path.insert(0, str(EGO_VIO_ROOT))
        from ego_vio.vio.openvins_ros2_bridge import OpenVINSROS2Bridge

        d405._load_hardware_modules()
        self.rs = d405.rs
        self.profile = profile
        self.ego_calibration = ego_calibration
        self.ego_document = ego_document
        self.ego_orb = ego_orb
        self.ego_imu_callback = ego_imu_callback
        _body_from_cam0, _body_from_cam1, accel_to_gyro = factory_transforms(
            ego_document
        )
        self.ego_imu_combiner = OnlineImuCombiner(accel_to_gyro)
        self.context = self.rs.context()
        devices = {
            item.get_info(self.rs.camera_info.serial_number): item
            for item in self.context.query_devices()
        }
        missing = [
            serial
            for serial in (profile.ego_serial, profile.right_serial)
            if serial not in devices
        ]
        if missing:
            raise RuntimeError(f"缺少指定RealSense设备: {missing}")
        ego_name = devices[profile.ego_serial].get_info(self.rs.camera_info.name)
        right_name = devices[profile.right_serial].get_info(self.rs.camera_info.name)
        if "D435I" not in ego_name.upper() or "D405" not in right_name.upper():
            raise RuntimeError(
                f"RealSense身份不匹配: Ego={ego_name!r}, device3={right_name!r}"
            )
        if not DEVICE3_IMU_BY_ID.exists():
            raise RuntimeError(f"设备3 STM32 不在线: {DEVICE3_IMU_BY_ID}")

        self.epoch_offset = time.time() - time.monotonic()
        self.bridge = OpenVINSROS2Bridge(
            name="device3_shared_pair",
            cam_topic="/cam0/image_raw",
            cam1_topic="/cam1/image_raw",
            imu_topic="/imu0",
            stereo=True,
            queue_size=100,
            qos_reliable=True,
            epoch_offset=self.epoch_offset,
            cam_latency_ms=0.0,
            imu_lead_guard_ms=DEVICE3_IMU_LEAD_GUARD_MS,
            gripper_topic="",
        )

        self.ego_pipeline = self.rs.pipeline(self.context)
        ego_config = self.rs.config()
        configure_d435i_streams(
            self.rs, ego_config, profile.ego_serial, D435iContract()
        )
        self.ego_config = ego_config
        self.ego_device = devices[profile.ego_serial]
        self.right_device = devices[profile.right_serial]
        self.right_sensor = self.right_device.first_depth_sensor()
        self.right_profiles = d405.select_profiles(self.right_sensor)
        self.right_queue = self.rs.frame_queue(
            d405.MONITOR_QUEUE_CAPACITY, keep_frames=True
        )
        self.imu = ImuReader(
            str(DEVICE3_IMU_BY_ID),
            baud=921600,
            warmup_frames=d405.IMU_WARMUP_FRAMES,
            protocol="stm32_combined_v1",
            on_sample=self._on_imu,
            name="device3_world_shared_imu",
        )
        self.latest_right_frames = {}
        self.latest_right_image = None
        self.latest_right_timestamp_ns = None
        self.timestamp_lock = threading.RLock()
        self.ego_lock = threading.RLock()
        self.latest_ego_frames = {}
        self.latest_ego_image = None
        self.latest_ego_timestamp_ns = None
        self.ego_camera_timestamps_ns = deque(maxlen=18000)
        self.right_camera_timestamps_ns = deque(maxlen=18000)
        self.ego_imu_timestamps_ns = deque(maxlen=72000)
        self.right_imu_timestamps_ns = deque(maxlen=144000)
        self.last_returned_ego_signature = None
        self.last_ego_pair_signature = None
        self.ego_callback_error = None
        self.ego_orb_active = False
        self.ego_gyro_frames = 0
        self.ego_accel_frames = 0
        self.last_complete_signature = None
        self.last_left_number = None
        self.ego_frames = 0
        self.right_frames = 0
        self.right_frame_gaps = 0
        self.imu_forwarded = 0
        self.started_mono = None
        self.ego_started = False
        self.right_opened = False
        self.right_started = False
        self.imu_started = False
        self.exposure = None
        self.global_time = None

    def _on_imu(self, sample) -> None:
        self.bridge.feed_imu(sample)
        with self.timestamp_lock:
            self.right_imu_timestamps_ns.append(
                int(round((float(sample.ts) + self.epoch_offset) * 1_000_000_000))
            )
        self.imu_forwarded += 1

    def _ego_timestamp_ns(self, frame) -> int:
        domain = str(frame.get_frame_timestamp_domain()).rsplit(".", 1)[-1]
        if domain != "global_time":
            raise RuntimeError(f"D435i timestamp domain is not global_time: {domain}")
        return int(round(float(frame.get_timestamp()) * 1_000_000))

    def _on_ego_frame(self, frame) -> None:
        try:
            if frame.is_frameset():
                for child in frame.as_frameset():
                    self._handle_ego_frame(child)
            else:
                self._handle_ego_frame(frame)
        except Exception as exc:
            with self.ego_lock:
                if self.ego_callback_error is None:
                    self.ego_callback_error = f"{type(exc).__name__}: {exc}"

    def _handle_ego_frame(self, frame) -> None:
        profile = frame.get_profile()
        stream = profile.stream_type()
        timestamp_ns = self._ego_timestamp_ns(frame)
        if stream == self.rs.stream.infrared:
            index = int(profile.stream_index())
            if index not in (1, 2):
                return
            image = np.asanyarray(frame.get_data()).copy()
            value = (int(frame.get_frame_number()), timestamp_ns, image)
            with self.ego_lock:
                self.latest_ego_frames[index] = value
                if 1 not in self.latest_ego_frames or 2 not in self.latest_ego_frames:
                    return
                left = self.latest_ego_frames[1]
                right = self.latest_ego_frames[2]
                signature = (left[0], right[0])
                if signature == self.last_ego_pair_signature:
                    return
                if left[0] != right[0] or abs(left[1] - right[1]) > 1_000_000:
                    return
                if self.ego_orb_active:
                    self.ego_orb.publish_stereo(left[1], left[2], right[2])
                self.latest_ego_image = left[2]
                self.latest_ego_timestamp_ns = left[1]
                with self.timestamp_lock:
                    self.ego_camera_timestamps_ns.append(left[1])
                self.last_ego_pair_signature = signature
                self.ego_frames += 1
            return
        if stream not in (self.rs.stream.gyro, self.rs.stream.accel):
            return
        motion = frame.as_motion_frame().get_motion_data()
        vector = np.array((motion.x, motion.y, motion.z), dtype=np.float64)
        with self.ego_lock:
            if stream == self.rs.stream.gyro:
                self.ego_gyro_frames += 1
                combined = self.ego_imu_combiner.push_gyro(timestamp_ns, vector)
            else:
                self.ego_accel_frames += 1
                combined = self.ego_imu_combiner.push_accel(timestamp_ns, vector)
            for sample in combined:
                with self.timestamp_lock:
                    self.ego_imu_timestamps_ns.append(int(sample.timestamp_ns))
                self.ego_imu_callback(sample)
                if self.ego_orb_active:
                    self.ego_orb.publish_imu(sample)

    def begin_ego_orb(self) -> None:
        """Start one fresh ORB timeline after the operator adjustment phase."""
        with self.ego_lock:
            if self.ego_orb_active:
                return
            self.ego_imu_combiner = OnlineImuCombiner(
                self.ego_imu_combiner.accel_to_gyro
            )
            self.ego_orb_active = True

    def start(self) -> None:
        for sensor in self.ego_device.query_sensors():
            if sensor.supports(self.rs.option.global_time_enabled):
                sensor.set_option(self.rs.option.global_time_enabled, 1.0)
        ego_depth = self.ego_device.first_depth_sensor()
        if not ego_depth.supports(self.rs.option.emitter_enabled):
            raise RuntimeError("Ego D435i emitter control unavailable")
        ego_depth.set_option(self.rs.option.emitter_enabled, 0.0)
        if abs(float(ego_depth.get_option(self.rs.option.emitter_enabled))) > 1e-6:
            raise RuntimeError("Ego D435i emitter disable readback failed")

        self.global_time = d405.configure_global_time(self.right_sensor)
        if not self.global_time.get("verified"):
            raise RuntimeError(f"设备3 D405 global_time 未通过读回: {self.global_time}")
        self.exposure = d405.configure_ir_auto_exposure(
            self.right_sensor,
            exposure_limit_us=d405.DEFAULT_IR_AUTO_EXPOSURE_LIMIT_US,
            gain_limit=d405.DEFAULT_IR_AUTO_GAIN_LIMIT,
        )

        active = self.ego_pipeline.start(self.ego_config, self._on_ego_frame)
        self.ego_started = True
        active_device = active.get_device()
        actual_serial = active_device.get_info(self.rs.camera_info.serial_number)
        if actual_serial != self.profile.ego_serial:
            raise RuntimeError(f"打开了错误的Ego相机: {actual_serial}")
        video_profile = active.get_stream(
            self.rs.stream.infrared, 1
        ).as_video_stream_profile()
        _verify_live_intrinsics(video_profile.get_intrinsics(), self.ego_calibration)
        active_contract = {
            "ir_left": active.get_stream(self.rs.stream.infrared, 1).fps(),
            "ir_right": active.get_stream(self.rs.stream.infrared, 2).fps(),
            "gyro": active.get_stream(self.rs.stream.gyro).fps(),
            "accel": active.get_stream(self.rs.stream.accel).fps(),
        }
        if active_contract != {
            "ir_left": 30,
            "ir_right": 30,
            "gyro": 200,
            "accel": 200,
        }:
            raise RuntimeError(f"D435i active stream contract mismatch: {active_contract}")

        if not self.imu.start():
            raise RuntimeError(f"无法打开设备3 STM32: {DEVICE3_IMU_BY_ID}")
        self.imu_started = True
        self.right_sensor.open(self.right_profiles)
        self.right_opened = True
        self.right_sensor.start(self.right_queue)
        self.right_started = True
        self.started_mono = time.monotonic()

    def poll(self):
        with self.ego_lock:
            if self.ego_callback_error is not None:
                raise RuntimeError(f"D435i live callback failed: {self.ego_callback_error}")
            signature = self.last_ego_pair_signature
            if signature is not None and signature != self.last_returned_ego_signature:
                ego_image = self.latest_ego_image
                ego_timestamp_ns = self.latest_ego_timestamp_ns
                self.last_returned_ego_signature = signature
            else:
                ego_image = None
                ego_timestamp_ns = None

        for frame in d405.drain_frame_queue(self.right_queue):
            key = d405.stream_key(frame)
            if key is not None:
                self.latest_right_frames[key] = frame
                self._forward_complete_right_set()
        return ego_image, ego_timestamp_ns

    def _forward_complete_right_set(self) -> None:
        if len(self.latest_right_frames) != len(d405.STREAM_KEYS):
            return
        timestamps_ms = [
            float(self.latest_right_frames[name].get_timestamp())
            for name in d405.STREAM_KEYS
        ]
        signature = tuple(
            int(self.latest_right_frames[name].get_frame_number())
            for name in d405.STREAM_KEYS
        )
        if (
            not d405.timestamps_aligned(timestamps_ms, d405.SYNC_TOLERANCE_MS)
            or signature == self.last_complete_signature
        ):
            return
        left_frame = self.latest_right_frames["infrared_left"]
        right_frame = self.latest_right_frames["infrared_right"]
        left_number = int(left_frame.get_frame_number())
        if self.last_left_number is not None and left_number > self.last_left_number + 1:
            self.right_frame_gaps += left_number - self.last_left_number - 1
        self.last_left_number = left_number
        left = np.asanyarray(left_frame.get_data()).copy()
        right = np.asanyarray(right_frame.get_data()).copy()
        right_timestamp_ns = int(round(float(left_frame.get_timestamp()) * 1_000_000))
        with self.timestamp_lock:
            self.right_camera_timestamps_ns.append(right_timestamp_ns)
        now = time.monotonic()
        self.bridge.feed_camera(
            CameraFrame(
                ts=d405.live_vins_timestamp_monotonic(
                    float(left_frame.get_timestamp()), self.epoch_offset
                ),
                color=left,
                depth=None,
                frame_idx=self.right_frames + 1,
                ts_arrival=now,
                ts_domain="global_time",
                frame_number=left_number,
                infrared_left=left,
                infrared_right=right,
            )
        )
        self.latest_right_image = left
        self.latest_right_timestamp_ns = right_timestamp_ns
        self.right_frames += 1
        self.last_complete_signature = signature

    def snapshot(self) -> dict:
        elapsed = max(1e-6, time.monotonic() - (self.started_mono or time.monotonic()))
        with self.timestamp_lock:
            cross_camera = _nearest_timestamp_metrics(
                self.ego_camera_timestamps_ns,
                self.right_camera_timestamps_ns,
            )
            cross_camera["threshold_ms_max"] = CROSS_CAMERA_MAX_DELTA_MS
            cross_camera["status"] = (
                "WAITING"
                if cross_camera["samples"] == 0
                else (
                    "PASS"
                    if cross_camera["max_ms"] <= CROSS_CAMERA_MAX_DELTA_MS
                    else "FAIL"
                )
            )
            timestamp_contract = {
                "domain": "librealsense_global_time_unix_epoch_ns",
                "cross_camera_nearest": cross_camera,
                "ego_camera_imu_nearest": _nearest_timestamp_metrics(
                    self.ego_camera_timestamps_ns,
                    self.ego_imu_timestamps_ns,
                ),
                "right_camera_imu_nearest_raw": _nearest_timestamp_metrics(
                    self.right_camera_timestamps_ns,
                    self.right_imu_timestamps_ns,
                ),
                "right_camera_imu_calibration": {
                    "td_s": DEVICE3_CAMERA_IMU_TD_S,
                    "estimate_td": 0,
                    "scope": "D405_260422274454_plus_STM32_cc06413a",
                },
                "note": (
                    "Nearest deltas verify clock-domain association only; device3 "
                    "physical camera/IMU alignment uses the frozen signed td."
                ),
            }
        return {
            "owner": "single_rsusb_context",
            "ego_serial": self.profile.ego_serial,
            "right_serial": self.profile.right_serial,
            "imu_by_id": str(DEVICE3_IMU_BY_ID),
            "ego_rate_hz": self.ego_frames / elapsed,
            "ego_gyro_rate_hz": self.ego_gyro_frames / elapsed,
            "ego_accel_rate_hz": self.ego_accel_frames / elapsed,
            "right_rate_hz": self.right_frames / elapsed,
            "right_frame_gaps": self.right_frame_gaps,
            "imu_rate_hz": self.imu_forwarded / elapsed,
            "imu": self.imu.stats(),
            "bridge": self.bridge.transport_stats(),
            "right_global_time": self.global_time,
            "right_exposure": self.exposure,
            "ego_imu_combiner": self.ego_imu_combiner.stats(),
            "ego_orb_active": self.ego_orb_active,
            "ego_orb_input": self.ego_orb.stats(),
            "timestamp_contract": timestamp_contract,
            "ego_callback_error": self.ego_callback_error,
        }

    def close(self) -> None:
        if self.right_started:
            try:
                self.right_sensor.stop()
            except Exception:
                pass
        if self.right_opened:
            try:
                self.right_sensor.close()
            except Exception:
                pass
        if self.imu_started:
            self.imu.stop()
        if self.ego_started:
            try:
                self.ego_pipeline.stop()
            except Exception:
                pass
        self.bridge.close()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _nearest_timestamp_metrics(reference, candidate) -> dict:
    """Summarize nearest absolute deltas over the overlapping clock interval."""
    left = np.asarray(tuple(reference), dtype=np.int64)
    right = np.asarray(tuple(candidate), dtype=np.int64)
    if left.size == 0 or right.size == 0:
        return {"samples": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
    overlap_start = max(int(left[0]), int(right[0]))
    overlap_end = min(int(left[-1]), int(right[-1]))
    left = left[(left >= overlap_start) & (left <= overlap_end)]
    right = right[(right >= overlap_start) & (right <= overlap_end)]
    if left.size == 0 or right.size == 0:
        return {"samples": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
    indices = np.searchsorted(right, left)
    hi = np.clip(indices, 0, right.size - 1)
    lo = np.clip(indices - 1, 0, right.size - 1)
    delta = np.minimum(np.abs(right[hi] - left), np.abs(right[lo] - left)) / 1e6
    return {
        "samples": int(delta.size),
        "p50_ms": float(np.percentile(delta, 50)),
        "p95_ms": float(np.percentile(delta, 95)),
        "max_ms": float(np.max(delta)),
    }


def _rolling_position_metrics(samples, window_s: float = 5.0) -> dict:
    """Measure recent endpoint change and radius without assuming stationarity."""
    values = tuple(samples)
    if not values:
        return {"samples": 0, "endpoint_delta_m": None, "radius_m": None}
    cutoff = values[-1].timestamp_ns - int(window_s * 1_000_000_000)
    points = np.asarray(
        [item.transform[:3, 3] for item in values if item.timestamp_ns >= cutoff],
        dtype=np.float64,
    )
    if len(points) < 2:
        return {"samples": int(len(points)), "endpoint_delta_m": None, "radius_m": None}
    center = np.mean(points, axis=0)
    return {
        "samples": int(len(points)),
        "endpoint_delta_m": float(np.linalg.norm(points[-1] - points[0])),
        "radius_m": float(np.max(np.linalg.norm(points - center, axis=1))),
    }


def _orb_local_from_body(
    local_from_camera: np.ndarray, body_from_camera: np.ndarray
) -> np.ndarray:
    """Convert ORB's left-camera pose into the calibrated Ego body pose."""
    return np.asarray(local_from_camera, dtype=np.float64) @ invert(
        np.asarray(body_from_camera, dtype=np.float64)
    )


class Device3WorldNode:
    def __init__(self, controller, ego_controller, ego_body_from_camera, output: Path):
        import rclpy
        from nav_msgs.msg import Odometry, Path as RosPath
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image as RosImage, Imu
        from std_msgs.msg import String

        class NodeImpl(Node):
            pass

        self.rclpy = rclpy
        self.Odometry = Odometry
        self.RosPath = RosPath
        self.String = String
        self.RosImage = RosImage
        self.Imu = Imu
        self.node = NodeImpl("device3_world_live_pair")
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=80,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(depth=20)
        self.world_pub = self.node.create_publisher(
            Odometry, "/device3/world_odometry", output_qos
        )
        self.path_pub = self.node.create_publisher(
            RosPath, "/device3/world_path", output_qos
        )
        self.ego_world_pub = self.node.create_publisher(
            Odometry, "/ego/world_odometry", output_qos
        )
        self.ego_path_pub = self.node.create_publisher(
            RosPath, "/ego/world_path", output_qos
        )
        self.status_pub = self.node.create_publisher(
            String, "/device3/world_status", output_qos
        )
        self.ego_status_pub = self.node.create_publisher(
            String, "/ego/world_status", output_qos
        )
        self.ego_image_pub = self.node.create_publisher(
            RosImage, "/ego/preview/image_raw", input_qos
        )
        self.right_image_pub = self.node.create_publisher(
            RosImage, "/device3/preview/image_raw", input_qos
        )
        self.ego_imu_pub = self.node.create_publisher(
            Imu, "/ego/imu_preview", input_qos
        )
        self.right_imu_pub = self.node.create_publisher(
            Imu, "/device3/imu_preview", input_qos
        )
        self.node.create_subscription(
            Odometry, "/odometry_rect", self._on_odometry, input_qos
        )
        self.node.create_subscription(Imu, "/imu0", self._on_imu, input_qos)
        self.node.create_subscription(
            String, "/vins/pose_integrity", self._on_pose_integrity, output_qos
        )
        self.controller = controller
        self.ego_controller = ego_controller
        self.ego_body_from_camera = np.asarray(
            ego_body_from_camera, dtype=np.float64
        )
        self.output = output
        self.lock = threading.RLock()
        self.odometry: deque[TimedTransform] = deque(maxlen=600)
        self.gyro: deque[tuple[int, float]] = deque(maxlen=800)
        self.ego_odometry: deque[TimedTransform] = deque(maxlen=600)
        self.ego_gyro: deque[tuple[int, np.ndarray]] = deque(maxlen=800)
        self.path_poses = deque(maxlen=600)
        self.plot_points = deque(maxlen=6000)
        self.ego_path_poses = deque(maxlen=600)
        self.ego_plot_points = deque(maxlen=6000)
        self.source_odom_count = 0
        self.world_odom_count = 0
        self.ego_world_odom_count = 0
        self.ego_source_odom_count = 0
        self.ego_source_imu_count = 0
        self.source_imu_count = 0
        self.last_odom_arrival_ns = 0
        self.last_ego_odom_arrival_ns = 0
        self.last_world_stamp_ns = 0
        self.last_path_publish_ns = 0
        self.last_ego_world_stamp_ns = 0
        self.last_ego_path_publish_ns = 0
        self.last_ego_observation_arrival_ns = 0
        self.last_ego_preview_stamp_ns = 0
        self.last_right_preview_stamp_ns = 0
        self.last_ego_imu_preview_stamp_ns = 0
        self.last_right_imu_preview_stamp_ns = 0
        self.latest_ego_imu_preview = None
        self.pose_integrity_failure = None
        self.ego_inertial_ba2_ready = False
        self.observation = {
            "state": "等待Ego观测",
            "board_tags": 0,
            "ego_tracking": False,
            "ego_anchor_status": "BLOCKED",
            "ego_anchor_reason": "not_initialized",
            "tag_detected": False,
            "anchor_status": "BLOCKED",
            "anchor_reason": "not_initialized",
        }
        self._csv_file = (output / "right_world_trajectory_live.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(("timestamp_ns", "x", "y", "z", "qw", "qx", "qy", "qz"))
        self._ego_csv_file = (output / "ego_world_trajectory_live.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._ego_csv = csv.writer(self._ego_csv_file)
        self._ego_csv.writerow(
            ("timestamp_ns", "x", "y", "z", "qw", "qx", "qy", "qz")
        )

    def _on_imu(self, message) -> None:
        stamp = _stamp_ns(message.header.stamp)
        vector = message.angular_velocity
        gyro_deg_s = math.degrees(
            math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
        )
        with self.lock:
            self.source_imu_count += 1
            self.gyro.append((stamp, gyro_deg_s))
            if (
                self.controller.current is not None
                and self.ego_controller.current is not None
                and stamp - self.last_right_imu_preview_stamp_ns >= 100_000_000
            ):
                self.right_imu_pub.publish(message)
                self.last_right_imu_preview_stamp_ns = stamp

    def _on_pose_integrity(self, message) -> None:
        with self.lock:
            try:
                self.pose_integrity_failure = json.loads(message.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                self.pose_integrity_failure = {"state": "SLAM_FAILED", "raw": message.data}

    def ingest_ego_imu(self, sample) -> None:
        stamp = int(sample.timestamp_ns)
        gyro = np.asarray(sample.gyro_rad_s, dtype=np.float64)
        accel = np.asarray(sample.accel_m_s2, dtype=np.float64)
        with self.lock:
            self.ego_source_imu_count += 1
            self.ego_gyro.append((stamp, np.array(gyro, copy=True)))
            self.latest_ego_imu_preview = (stamp, accel.copy(), gyro.copy())

    def _publish_preview(self, publisher, timestamp_ns: int, image: np.ndarray, frame_id: str) -> None:
        frame = np.ascontiguousarray(image, dtype=np.uint8)
        message = self.RosImage()
        message.header.stamp.sec = timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        message.header.frame_id = frame_id
        message.height, message.width = frame.shape
        message.encoding = "mono8"
        message.is_bigendian = False
        message.step = int(frame.shape[1])
        message.data = frame.tobytes()
        publisher.publish(message)

    def publish_previews(
        self,
        ego_timestamp_ns: int,
        ego_image: np.ndarray,
        right_timestamp_ns: int | None,
        right_image: np.ndarray | None,
    ) -> None:
        if self.controller.current is None or self.ego_controller.current is None:
            return
        if ego_timestamp_ns - self.last_ego_preview_stamp_ns >= 200_000_000:
            self._publish_preview(
                self.ego_image_pub, ego_timestamp_ns, ego_image, "ego_ir_left"
            )
            self.last_ego_preview_stamp_ns = ego_timestamp_ns
        if (
            right_timestamp_ns is not None
            and right_image is not None
            and right_timestamp_ns - self.last_right_preview_stamp_ns >= 200_000_000
        ):
            self._publish_preview(
                self.right_image_pub,
                right_timestamp_ns,
                right_image,
                "device3_ir_left",
            )
            self.last_right_preview_stamp_ns = right_timestamp_ns
        with self.lock:
            imu_sample = self.latest_ego_imu_preview
        if (
            imu_sample is not None
            and imu_sample[0] - self.last_ego_imu_preview_stamp_ns >= 100_000_000
        ):
            stamp, accel, gyro = imu_sample
            message = self.Imu()
            message.header.stamp.sec = stamp // 1_000_000_000
            message.header.stamp.nanosec = stamp % 1_000_000_000
            message.header.frame_id = "ego_imu"
            message.linear_acceleration.x = float(accel[0])
            message.linear_acceleration.y = float(accel[1])
            message.linear_acceleration.z = float(accel[2])
            message.angular_velocity.x = float(gyro[0])
            message.angular_velocity.y = float(gyro[1])
            message.angular_velocity.z = float(gyro[2])
            self.ego_imu_pub.publish(message)
            self.last_ego_imu_preview_stamp_ns = stamp

    def ingest_ego_orb_pose(self, pose: OrbSlam3Pose) -> None:
        with self.lock:
            self.ego_inertial_ba2_ready = bool(pose.inertial_ba2_ready)
        if not pose.tracking_ok:
            return
        stamp = int(pose.timestamp_ns)
        local_from_body = _orb_local_from_body(
            pose.local_from_camera, self.ego_body_from_camera
        )
        with self.lock:
            self.ego_source_odom_count += 1
            self.last_ego_odom_arrival_ns = time.monotonic_ns()
            self.ego_odometry.append(TimedTransform(stamp, local_from_body))
            if self.ego_controller.current is None:
                return
            world_from_body = self.ego_controller.world_from_body(local_from_body)
            world_from_camera = world_from_body @ self.ego_body_from_camera
        self.publish_ego_world(stamp, world_from_camera)

    def _on_odometry(self, source) -> None:
        stamp = _stamp_ns(source.header.stamp)
        p, q = source.pose.pose.position, source.pose.pose.orientation
        local = matrix_from_pose_components(
            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        )
        with self.lock:
            self.source_odom_count += 1
            self.last_odom_arrival_ns = time.monotonic_ns()
            self.odometry.append(TimedTransform(stamp, local))
            if self.controller.current is None:
                return
            world = self.controller.world_from_body(local)
            position, quaternion = pose_components_from_matrix(world)
            result = self.Odometry()
            result.header.stamp = source.header.stamp
            result.header.frame_id = "aprilgrid_world"
            result.child_frame_id = "device3_vins_body"
            result.pose.pose.position.x = float(position[0])
            result.pose.pose.position.y = float(position[1])
            result.pose.pose.position.z = float(position[2])
            result.pose.pose.orientation.x = float(quaternion[0])
            result.pose.pose.orientation.y = float(quaternion[1])
            result.pose.pose.orientation.z = float(quaternion[2])
            result.pose.pose.orientation.w = float(quaternion[3])
            self.world_pub.publish(result)
            self.world_odom_count += 1
            self.last_world_stamp_ns = stamp
            self.plot_points.append(position.copy())
            self._csv.writerow(
                (stamp, *position.tolist(), quaternion[3], *quaternion[:3].tolist())
            )
            if self.world_odom_count % 15 == 0:
                self._csv_file.flush()
            self._append_path(result)

    def publish_ego_world(self, timestamp_ns: int, world_from_camera: np.ndarray) -> None:
        """Publish the continuous D435i camera pose from frozen ORB world alignment."""
        position, quaternion = pose_components_from_matrix(world_from_camera)
        result = self.Odometry()
        result.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        result.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        result.header.frame_id = "aprilgrid_world"
        result.child_frame_id = "ego_ir_left"
        result.pose.pose.position.x = float(position[0])
        result.pose.pose.position.y = float(position[1])
        result.pose.pose.position.z = float(position[2])
        result.pose.pose.orientation.x = float(quaternion[0])
        result.pose.pose.orientation.y = float(quaternion[1])
        result.pose.pose.orientation.z = float(quaternion[2])
        result.pose.pose.orientation.w = float(quaternion[3])
        with self.lock:
            self.ego_world_pub.publish(result)
            self.ego_world_odom_count += 1
            self.last_ego_world_stamp_ns = timestamp_ns
            self.last_ego_observation_arrival_ns = time.monotonic_ns()
            self.ego_plot_points.append(position.copy())
            self._ego_csv.writerow(
                (timestamp_ns, *position.tolist(), quaternion[3], *quaternion[:3].tolist())
            )
            if self.ego_world_odom_count % 15 == 0:
                self._ego_csv_file.flush()
            self._append_ego_path(result)

    def _append_path(self, odometry) -> None:
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.path_poses.append(pose)
        if (
            self.last_world_stamp_ns - self.last_path_publish_ns < 1_000_000_000
            or self.path_pub.get_subscription_count() == 0
        ):
            return
        path = self.RosPath()
        path.header = odometry.header
        path.poses = list(self.path_poses)
        self.path_pub.publish(path)
        self.last_path_publish_ns = self.last_world_stamp_ns

    def _append_ego_path(self, odometry) -> None:
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.ego_path_poses.append(pose)
        if (
            self.last_ego_world_stamp_ns - self.last_ego_path_publish_ns
            < 1_000_000_000
            or self.ego_path_pub.get_subscription_count() == 0
        ):
            return
        path = self.RosPath()
        path.header = odometry.header
        path.poses = list(self.ego_path_poses)
        self.ego_path_pub.publish(path)
        self.last_ego_path_publish_ns = self.last_ego_world_stamp_ns

    def nearest_inputs(self, timestamp_ns: int):
        with self.lock:
            odometry = nearest_transform(
                tuple(self.odometry), timestamp_ns, max_delta_ns=150_000_000
            )
            if not self.gyro:
                raise ValueError("no gyro samples are available")
            gyro_stamp, gyro_value = min(
                self.gyro, key=lambda item: abs(item[0] - timestamp_ns)
            )
            gyro_delta = abs(gyro_stamp - timestamp_ns)
            if gyro_delta > 150_000_000:
                raise ValueError(f"nearest gyro sample is stale by {gyro_delta / 1e6:.3f} ms")
            return odometry, float(gyro_value), gyro_stamp

    def nearest_ego_inputs(self, timestamp_ns: int):
        with self.lock:
            odometry = nearest_transform(
                tuple(self.ego_odometry), timestamp_ns, max_delta_ns=150_000_000
            )
            if not self.ego_gyro:
                raise ValueError("no Ego gyro samples are available")
            gyro_stamp, gyro_value = min(
                self.ego_gyro, key=lambda item: abs(item[0] - timestamp_ns)
            )
            gyro_delta = abs(gyro_stamp - timestamp_ns)
            if gyro_delta > 150_000_000:
                raise ValueError(
                    f"nearest Ego gyro sample is stale by {gyro_delta / 1e6:.3f} ms"
                )
            return odometry, np.array(gyro_value, copy=True), gyro_stamp

    def apply_anchor_batch(self, candidates) -> dict:
        with self.lock:
            decision = self.controller.process_window(candidates)
            self.observation.update(
                anchor_status=decision.status,
                anchor_reason=decision.reason,
                candidate_count=decision.candidate_count,
                eligible_count=decision.eligible_count,
                inlier_count=decision.inlier_count,
            )
            return {
                "status": decision.status,
                "reason": decision.reason,
                "candidate_count": decision.candidate_count,
                "eligible_count": decision.eligible_count,
                "inlier_count": decision.inlier_count,
            }

    def apply_ego_anchor_batch(self, candidates) -> dict:
        with self.lock:
            decision = self.ego_controller.process_window(candidates)
            self.observation.update(
                ego_anchor_status=decision.status,
                ego_anchor_reason=decision.reason,
                ego_anchor_candidate_count=decision.candidate_count,
                ego_anchor_eligible_count=decision.eligible_count,
                ego_anchor_inlier_count=decision.inlier_count,
            )
            return {
                "status": decision.status,
                "reason": decision.reason,
                "candidate_count": decision.candidate_count,
                "eligible_count": decision.eligible_count,
                "inlier_count": decision.inlier_count,
            }

    def update_observation(self, **values) -> None:
        with self.lock:
            self.observation.update(values)

    def snapshot(self) -> dict:
        with self.lock:
            cross_odometry = _nearest_timestamp_metrics(
                (item.timestamp_ns for item in self.ego_odometry),
                (item.timestamp_ns for item in self.odometry),
            )
            return {
                **self.observation,
                "source_odom_count": self.source_odom_count,
                "world_odom_count": self.world_odom_count,
                "ego_world_odom_count": self.ego_world_odom_count,
                "source_imu_count": self.source_imu_count,
                "ego_source_odom_count": self.ego_source_odom_count,
                "ego_inertial_ba2_ready": self.ego_inertial_ba2_ready,
                "ego_source_imu_count": self.ego_source_imu_count,
                "timestamp_alignment": {
                    "ego_right_odometry_nearest": cross_odometry,
                },
                "recent_motion": {
                    "ego_5s": _rolling_position_metrics(self.ego_odometry, 5.0),
                    "right_5s": _rolling_position_metrics(self.odometry, 5.0),
                    "ego_10s": _rolling_position_metrics(self.ego_odometry, 10.0),
                    "right_10s": _rolling_position_metrics(self.odometry, 10.0),
                },
                "odom_age_s": (
                    None
                    if not self.last_odom_arrival_ns
                    else (time.monotonic_ns() - self.last_odom_arrival_ns) / 1e9
                ),
                "ego_source_odom_age_s": (
                    None
                    if not self.last_ego_odom_arrival_ns
                    else (time.monotonic_ns() - self.last_ego_odom_arrival_ns) / 1e9
                ),
                "trajectory_points": len(self.plot_points),
                "ego_trajectory_points": len(self.ego_plot_points),
                "ego_pose_age_s": (
                    None
                    if not self.last_ego_observation_arrival_ns
                    else (time.monotonic_ns() - self.last_ego_observation_arrival_ns)
                    / 1e9
                ),
                "ego_tracking": (
                    self.ego_controller.current is not None
                    and self.last_ego_observation_arrival_ns > 0
                    and time.monotonic_ns() - self.last_ego_observation_arrival_ns
                    <= 500_000_000
                ),
                "right_anchor_initialized": self.controller.current is not None,
                "pose_integrity_failure": self.pose_integrity_failure,
                "ego_anchor_initialized": self.ego_controller.current is not None,
                "anchor_initialized": (
                    self.controller.current is not None
                    and self.ego_controller.current is not None
                ),
                "latest_position_m": (
                    None if not self.plot_points else self.plot_points[-1].tolist()
                ),
                "ego_latest_position_m": (
                    None
                    if not self.ego_plot_points
                    else self.ego_plot_points[-1].tolist()
                ),
            }

    def points(self) -> np.ndarray:
        with self.lock:
            return np.asarray(tuple(self.plot_points), dtype=np.float64)

    def ego_points(self) -> np.ndarray:
        with self.lock:
            return np.asarray(tuple(self.ego_plot_points), dtype=np.float64)

    def publish_status(self, payload: dict) -> None:
        message = self.String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(message)
        self.ego_status_pub.publish(message)

    def close(self) -> None:
        with self.lock:
            self._csv_file.flush()
            self._csv_file.close()
            self._ego_csv_file.flush()
            self._ego_csv_file.close()
        self.node.destroy_node()


def _load_font(size: int):
    for path in FONT_PATHS:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _draw_text(image: np.ndarray, lines: list[tuple[str, tuple[int, int, int]]]) -> np.ndarray:
    canvas = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(canvas)
    font = _load_font(26)
    y = 18
    for line, color in lines:
        draw.rectangle((12, y - 3, 948, y + 34), fill=(20, 20, 20))
        draw.text((20, y), line, font=font, fill=color)
        y += 42
    return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)


def _trajectory_panel(
    device3_points: np.ndarray,
    ego_points: np.ndarray,
    snapshot: dict,
    height: int = 540,
) -> np.ndarray:
    width = 640
    panel = np.full((height, width, 3), 25, dtype=np.uint8)
    cv2.putText(
        panel,
        "AprilGrid World: X-Y",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (220, 220, 220),
        2,
    )
    available = [points for points in (device3_points, ego_points) if len(points)]
    if available:
        combined_xy = np.concatenate(available, axis=0)[:, :2]
        center = np.median(combined_xy, axis=0)
        extent = max(0.2, float(np.max(np.abs(combined_xy - center))) * 1.25)
        scale = min((width - 60) / (2 * extent), (height - 140) / (2 * extent))
        for offset in range(-4, 5):
            x = int(width / 2 + offset * 0.1 * scale)
            y = int(height / 2 - offset * 0.1 * scale)
            cv2.line(panel, (x, 55), (x, height - 75), (55, 55, 55), 1)
            cv2.line(panel, (30, y), (width - 30, y), (55, 55, 55), 1)

        def draw_track(points: np.ndarray, line_color, marker_color) -> None:
            if not len(points):
                return
            track_xy = points[:, :2]
            pixels = np.empty((len(track_xy), 2), dtype=np.int32)
            pixels[:, 0] = np.round(
                width / 2 + (track_xy[:, 0] - center[0]) * scale
            ).astype(int)
            pixels[:, 1] = np.round(
                height / 2 - (track_xy[:, 1] - center[1]) * scale
            ).astype(int)
            if len(pixels) > 1:
                cv2.polylines(
                    panel, [pixels.reshape(-1, 1, 2)], False, line_color, 3
                )
            cv2.circle(panel, tuple(pixels[-1]), 7, marker_color, -1)

        draw_track(device3_points, (40, 190, 255), (50, 255, 80))
        draw_track(ego_points, (255, 210, 40), (255, 255, 80))
        if len(device3_points):
            position = device3_points[-1]
            cv2.putText(
                panel,
                f"D3 X {position[0]:+.3f} Y {position[1]:+.3f} Z {position[2]:+.3f}m",
                (20, height - 42),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (40, 190, 255),
                1,
            )
        if len(ego_points):
            position = ego_points[-1]
            cv2.putText(
                panel,
                f"EGO X {position[0]:+.3f} Y {position[1]:+.3f} Z {position[2]:+.3f}m",
                (20, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 210, 40),
                1,
            )
    else:
        cv2.putText(
            panel,
            "WAITING FOR WORLD POSES",
            (130, height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 180, 255),
            2,
        )
    color = (60, 220, 60) if snapshot.get("anchor_initialized") else (0, 180, 255)
    cv2.putText(
        panel,
        f"anchor={snapshot.get('anchor_status')} reason={snapshot.get('anchor_reason')}",
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.53,
        color,
        1,
    )
    return panel


def _board_pose_is_live_usable(detections: dict, reprojection_rms_px: float) -> bool:
    ids = tuple(int(tag_id) for tag_id in detections)
    return (
        len(ids) >= 6
        and len({tag_id // 6 for tag_id in ids}) >= 3
        and len({tag_id % 6 for tag_id in ids}) >= 3
        and math.isfinite(reprojection_rms_px)
        and reprojection_rms_px <= 3.0
    )


def _camera_calibration(source: Path) -> tuple[CameraCalibration, dict]:
    document = json.loads(source.read_text(encoding="utf-8"))
    intrinsics = document["intrinsics"]["ir_left"]
    matrix = np.array(
        (
            (intrinsics["fx"], 0.0, intrinsics["ppx"]),
            (0.0, intrinsics["fy"], intrinsics["ppy"]),
            (0.0, 0.0, 1.0),
        ),
        dtype=np.float64,
    )
    calibration = CameraCalibration(
        calibration_id="D435I_FACTORY_327122078613_20260828",
        frame_id="ego.ir_left",
        width=int(intrinsics["width"]),
        height=int(intrinsics["height"]),
        camera_matrix=matrix,
        distortion_coefficients=np.asarray(intrinsics["coefficients"]),
        distortion_model=intrinsics["distortion_model"],
    )
    return calibration, document


def _verify_live_intrinsics(rs_intrinsics, frozen: CameraCalibration) -> None:
    actual = np.array(
        (rs_intrinsics.fx, rs_intrinsics.fy, rs_intrinsics.ppx, rs_intrinsics.ppy),
        dtype=np.float64,
    )
    expected = np.array(
        (
            frozen.camera_matrix[0, 0],
            frozen.camera_matrix[1, 1],
            frozen.camera_matrix[0, 2],
            frozen.camera_matrix[1, 2],
        )
    )
    if (rs_intrinsics.width, rs_intrinsics.height) != (frozen.width, frozen.height):
        raise RuntimeError("Ego D435i live stream shape differs from accepted run24")
    if not np.allclose(actual, expected, atol=1e-6, rtol=0.0):
        raise RuntimeError(
            f"Ego D435i factory intrinsics differ from run24: actual={actual}, expected={expected}"
        )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--ego-calibration", type=Path, default=DEFAULT_EGO_CALIBRATION)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until Q/Esc/Ctrl-C")
    parser.add_argument("--no-display", action="store_true")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"output directory already exists: {output}")
    output.mkdir(parents=True)
    profile = load_device3_runtime_profile(args.profile)
    ego_calibration, ego_document = _camera_calibration(args.ego_calibration)
    if ego_document["device"]["serial"] != profile.ego_serial:
        raise RuntimeError("Ego factory calibration belongs to another camera")
    ego_body_from_camera, _ego_body_from_camera1, _accel_to_gyro = factory_transforms(
        ego_document
    )
    controller = FrozenSessionAnchorController(Device3WorldAnchorController(profile))
    ego_controller = FrozenSessionAnchorController(
        Device3WorldAnchorController(profile)
    )
    window = AnchorCandidateWindow(window_ns=400_000_000, maximum_candidates=20)
    ego_window = AnchorCandidateWindow(window_ns=400_000_000, maximum_candidates=20)
    runtime_vins_library = os.environ.get("DEVICE3_WORLD_VINS_LIBRARY")
    provenance = {
        "schema": "three-device-slam.device3-world-live-pair.v4",
        "scope": "d435i_orbslam3_ego_plus_vins_right_live_trial",
        "three_device_group_ready": False,
        "profile": str(args.profile.resolve()),
        "profile_sha256": _sha256(args.profile.resolve()),
        "ego_factory_calibration": str(args.ego_calibration.resolve()),
        "ego_factory_calibration_sha256": _sha256(args.ego_calibration.resolve()),
        "device3_runtime_image": os.environ.get(
            "DEVICE3_WORLD_IMAGE",
            "umi-ego-vio:device3-cc06413a-d405-product-v1.0.2-20260913",
        ),
        "device3_runtime_vins_library": runtime_vins_library,
        "device3_runtime_vins_library_sha256": (
            _sha256(Path(runtime_vins_library).resolve())
            if runtime_vins_library
            else None
        ),
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "runtime_control": {
            "mode": "continuous_until_operator_stop" if args.duration == 0 else "fixed_duration",
            "configured_duration_s": None if args.duration == 0 else args.duration,
            "display": (
                "持续运行，直到按Q/Esc、关闭引导窗口或Ctrl-C"
                if args.duration == 0
                else f"限时运行 {args.duration:g} 秒，届时自动正常停止"
            ),
        },
        "world_frame": {
            "frame_id": "aprilgrid_world",
            "x": "increasing frozen AprilGrid column coordinate",
            "y": "increasing frozen AprilGrid row coordinate",
            "z": "printed-board normal toward the camera-facing side",
            "rerun_view_coordinates": "LDB",
        },
        "status": "STARTING",
    }
    _atomic_json(output / "run_status.json", provenance)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init(args=[])
    node = Device3WorldNode(
        controller, ego_controller, ego_body_from_camera, output
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node.node)
    ros_thread = threading.Thread(target=executor.spin, name="device3-world-ros", daemon=True)
    ros_thread.start()
    cv2.setNumThreads(1)
    ego_orb = OrbSlam3LiveProcess(
        output / "ego_orb_runtime",
        max_write_backlog=4,
        backpressure_after_frames=1200,
    )
    sensors = SharedPairSensors(
        profile,
        ego_calibration,
        ego_document,
        ego_orb,
        node.ingest_ego_imu,
    )

    board_detector = make_detector()
    tag_config = AprilTagDetectorConfig(
        family="tag36h11",
        tag_size_m=0.04,
        allowed_tag_ids=(1,),
        max_reprojection_error_px=1.5,
        ambiguity_error_gap_px=0.05,
        ambiguity_translation_m=0.005,
        ambiguity_rotation_deg=5.0,
        corner_refinement="apriltag",
    )
    stopped = threading.Event()
    stop_control = {"reason": None}

    def request_stop(signum, _frame):
        if stop_control["reason"] is None:
            stop_control["reason"] = f"signal_{signal.Signals(signum).name}"
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    start = time.monotonic()
    frame_count = 0
    processed_count = 0
    last_state_write = 0.0
    last_image = np.zeros((720, 1280), dtype=np.uint8)
    last_board = {}
    last_tag_corners = None
    last_error = None
    ego_orb_begun = False
    try:
        ego_orb.start()
        sensors.start()
        start = time.monotonic()
        last_sensor_snapshot = sensors.snapshot()
        if not args.no_display:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)
            cv2.moveWindow(WINDOW_NAME, 20, 20)
            cv2.imshow(
                WINDOW_NAME,
                np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8),
            )
            cv2.waitKey(1)
        provenance["status"] = "RUNNING_WAITING_FOR_ANCHOR"
        while not stopped.is_set():
            if args.duration > 0 and time.monotonic() - start >= args.duration:
                stop_control["reason"] = "configured_duration_elapsed"
                break
            elapsed = time.monotonic() - start
            if not ego_orb_begun and elapsed >= 15.0:
                sensors.begin_ego_orb()
                ego_orb_begun = True
            ego_orb.check_alive()
            integrity_failure = node.snapshot().get("pose_integrity_failure")
            if integrity_failure is not None:
                raise RuntimeError(
                    "设备3 VINS位姿完整性保护触发："
                    + json.dumps(integrity_failure, ensure_ascii=False)
                )
            while True:
                pose = ego_orb.poll_pose()
                if pose is None:
                    break
                _assert_ego_anchor_epoch_valid(
                    ego_controller.current is not None, pose.inertial_ba2_ready
                )
                node.ingest_ego_orb_pose(pose)
            ego_image, ego_frame_stamp_ns = sensors.poll()
            if ego_image is None:
                time.sleep(0.001)
                continue
            node.publish_previews(
                ego_frame_stamp_ns,
                ego_image,
                sensors.latest_right_timestamp_ns,
                sensors.latest_right_image,
            )
            frame_count += 1
            last_image = ego_image
            anchors_ready = (
                controller.current is not None and ego_controller.current is not None
            )
            ego_inertial_ready = node.snapshot().get("ego_inertial_ba2_ready", False)
            anchor_phase = elapsed >= 35.0 and ego_inertial_ready
            if frame_count % 3 == 0 and not anchors_ready:
                processed_count += 1
                observation_stamp_ns = time.monotonic_ns()
                board_pose = None
                tag_detection = None
                try:
                    last_board = valid_detections(
                        detect_grid_scaled(board_detector, last_image, 2)
                    )
                    board_pose = solve_grid_pose(
                        last_board,
                        ego_calibration.camera_matrix,
                        ego_calibration.distortion_coefficients,
                    )
                except (RuntimeError, ValueError) as exc:
                    last_board = {}
                    last_error = f"AprilGrid: {exc}"
                batch = detect_apriltags(
                    last_image,
                    calibration=ego_calibration,
                    config=tag_config,
                    acquisition_timestamp_ns=observation_stamp_ns,
                    detector_completed_ns=time.monotonic_ns(),
                )
                accepted = [
                    item
                    for item in batch.detections
                    if item.tag_id == 1 and item.accepted
                ]
                if len(accepted) == 1:
                    tag_detection = accepted[0]
                    last_tag_corners = tag_detection.corners_px
                else:
                    last_tag_corners = None
                board_ok = board_pose is not None and board_pose.reprojection_rms_px <= 3.0
                board_world_ok = (
                    board_pose is not None
                    and _board_pose_is_live_usable(
                        last_board, board_pose.reprojection_rms_px
                    )
                )
                if anchor_phase and board_world_ok:
                    try:
                        ego_local, ego_gyro_rad_s, _ego_gyro_stamp = (
                            node.nearest_ego_inputs(ego_frame_stamp_ns)
                        )
                        node.update_observation(
                            ego_anchor_odom_delta_ms=abs(
                                ego_local.timestamp_ns - ego_frame_stamp_ns
                            )
                            / 1e6,
                            ego_anchor_gyro_delta_ms=abs(
                                _ego_gyro_stamp - ego_frame_stamp_ns
                            )
                            / 1e6,
                        )
                        ego_candidate = ego_world_anchor_candidate(
                            timestamp_ns=observation_stamp_ns,
                            gyro_rad_s=ego_gyro_rad_s,
                            world_from_camera=invert(board_pose.camera_from_grid),
                            body_from_camera=ego_body_from_camera,
                            odom_from_body=ego_local.transform,
                        )
                        ego_ready = ego_window.add(ego_candidate)
                        if ego_ready is not None:
                            node.apply_ego_anchor_batch(ego_ready)
                    except ValueError as exc:
                        ego_window.clear()
                        last_error = f"Ego ORB: {exc}"
                else:
                    ego_window.clear()
                tag_ok = tag_detection is not None
                if elapsed < 15.0:
                    observation_state = "设备调整阶段：两台设备保持静止，调整到同时看见AprilGrid和ID1"
                elif elapsed < 35.0:
                    observation_state = "初始化Ego ORB：移动Ego连续平移并小幅转动，设备3保持不动"
                elif not ego_inertial_ready:
                    observation_state = "继续移动Ego完成惯性初始化（BA2），设备3保持不动"
                else:
                    observation_state = "停止移动两台设备，保持AprilGrid和设备3的ID1同时可见"
                if anchor_phase and board_ok and tag_ok:
                    try:
                        local, gyro_deg_s, gyro_stamp = node.nearest_inputs(
                            ego_frame_stamp_ns
                        )
                        node.update_observation(
                            right_anchor_odom_delta_ms=abs(
                                local.timestamp_ns - ego_frame_stamp_ns
                            )
                            / 1e6,
                            right_anchor_gyro_delta_ms=abs(
                                gyro_stamp - ego_frame_stamp_ns
                            )
                            / 1e6,
                        )
                        last_error = None
                        candidate = anchor_candidate_from_synced_observation(
                            profile,
                            timestamp_ns=observation_stamp_ns,
                            gyro_norm_deg_s=gyro_deg_s,
                            world_from_ego_camera=invert(board_pose.camera_from_grid),
                            ego_camera_from_mount_tag=(
                                tag_detection.observation.camera_from_tag
                            ),
                            right_odom_from_body=local.transform,
                        )
                        ready = window.add(candidate)
                        observation_state = (
                            "Ego和设备3请保持静止，正在建立世界锚"
                            if controller.current is None
                            else "锚已建立：Ego可移动但须保持AprilGrid可见"
                        )
                        if ready is not None:
                            decision = node.apply_anchor_batch(ready)
                            observation_state = (
                                "锚已建立：Ego可移动但须保持AprilGrid可见"
                                if controller.current is not None
                                else f"世界锚未通过：{decision['reason']}"
                            )
                    except ValueError as exc:
                        window.clear()
                        last_error = str(exc)
                        observation_state = "等待设备3实时VIO和IMU同步"
                else:
                    window.clear()
                if (
                    ego_controller.current is None
                    and board_world_ok
                    and node.snapshot().get("ego_source_odom_count", 0) > 0
                ):
                    observation_state = "Ego和设备3保持静止，正在建立两条世界锚"
                elif elapsed >= 35.0 and not ego_inertial_ready:
                    observation_state = (
                        "继续移动Ego完成惯性初始化（BA2）：连续平移并绕三轴小幅转动"
                    )
                elif (
                    elapsed >= 35.0
                    and node.snapshot().get("ego_source_odom_count", 0) == 0
                ):
                    observation_state = (
                        "Ego ORB尚未初始化：只移动Ego做启停平移和小幅转动"
                    )
                elif (
                    ego_controller.current is not None
                    and controller.current is not None
                ):
                    observation_state = "世界锚已建立，可以缓慢移动Ego和设备3"
                snapshot = node.snapshot()
                node.update_observation(
                    state=observation_state,
                    board_tags=len(last_board),
                    board_reprojection_px=(
                        None if board_pose is None else board_pose.reprojection_rms_px
                    ),
                    tag_detected=tag_ok,
                    board_tracking=board_world_ok,
                    tag_reprojection_px=(
                        None
                        if tag_detection is None
                        else tag_detection.observation.reprojection_error_px
                    ),
                    last_error=last_error,
                    processed_frames=processed_count,
                    ego_frames=frame_count,
                )
            elif anchors_ready:
                last_board = {}
                last_tag_corners = None
                last_error = None
                node.update_observation(
                    state="ORB/VINS世界锚已冻结，可移动；无需持续看到AprilGrid",
                    board_tags=0,
                    board_reprojection_px=None,
                    board_tracking=False,
                    tag_detected=False,
                    tag_reprojection_px=None,
                    last_error=None,
                    processed_frames=processed_count,
                    ego_frames=frame_count,
                )

            snapshot = node.snapshot()
            if not args.no_display:
                preview = cv2.cvtColor(last_image, cv2.COLOR_GRAY2BGR)
                for corners in last_board.values():
                    cv2.polylines(
                        preview,
                        [np.round(corners).astype(np.int32).reshape(-1, 1, 2)],
                        True,
                        (40, 220, 40),
                        2,
                    )
                if last_tag_corners is not None:
                    cv2.polylines(
                        preview,
                        [np.round(last_tag_corners).astype(np.int32).reshape(-1, 1, 2)],
                        True,
                        (0, 220, 255),
                        3,
                    )
                preview = cv2.resize(preview, (960, 540), interpolation=cv2.INTER_AREA)
                state_color = (80, 255, 100) if snapshot["anchor_initialized"] else (255, 210, 60)
                lines = [
                    (snapshot.get("state", ""), state_color),
                    (
                        provenance["runtime_control"]["display"],
                        (230, 230, 230),
                    ),
                    (
                        f"AprilGrid标签 {snapshot.get('board_tags', 0)} 个；设备3 ID1："
                        + ("已识别" if snapshot.get("tag_detected") else "未识别"),
                        (230, 230, 230),
                    ),
                    (
                        f"Ego ORB/世界 {snapshot.get('ego_source_odom_count', 0)}/"
                        f"{snapshot.get('ego_world_odom_count', 0)}；设备3世界 "
                        f"{snapshot.get('world_odom_count', 0)}；按 Q 或 Esc 停止",
                        (230, 230, 230),
                    ),
                ]
                timing = last_sensor_snapshot["timestamp_contract"][
                    "cross_camera_nearest"
                ]
                timing_color = (
                    (80, 255, 100)
                    if timing["status"] == "PASS"
                    else (70, 70, 255)
                    if timing["status"] == "FAIL"
                    else (255, 210, 60)
                )
                lines.append(
                    (
                        "跨相机时间戳："
                        + (
                            f"{timing['status']}，P95 {timing['p95_ms']:.2f} ms，"
                            f"最大 {timing['max_ms']:.2f} ms"
                            if timing["samples"]
                            else "等待样本"
                        ),
                        timing_color,
                    )
                )
                recent = snapshot["recent_motion"]
                ego_delta = recent["ego_5s"]["endpoint_delta_m"]
                right_delta = recent["right_5s"]["endpoint_delta_m"]
                lines.append(
                    (
                        "近5秒坐标位移（静止时用于判断漂移）："
                        f"Ego {ego_delta * 1000:.1f} mm；设备3 {right_delta * 1000:.1f} mm"
                        if ego_delta is not None and right_delta is not None
                        else "近5秒坐标位移：等待世界坐标样本",
                        (230, 230, 230),
                    )
                )
                preview = _draw_text(preview, lines)
                if sensors.latest_right_image is not None:
                    right_preview = cv2.resize(
                        sensors.latest_right_image,
                        (320, 180),
                        interpolation=cv2.INTER_AREA,
                    )
                    right_preview = cv2.cvtColor(right_preview, cv2.COLOR_GRAY2BGR)
                    cv2.rectangle(right_preview, (0, 0), (320, 30), (0, 0, 0), -1)
                    cv2.putText(
                        right_preview,
                        "DEVICE3 LEFT IR",
                        (8, 21),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 255),
                        1,
                    )
                    preview[-180:, -320:] = right_preview
                display = preview
                display = cv2.resize(
                    display, (WINDOW_WIDTH, WINDOW_HEIGHT), interpolation=cv2.INTER_LINEAR
                )
                cv2.imshow(WINDOW_NAME, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    stop_control["reason"] = "operator_key"
                    stopped.set()
                try:
                    if (
                        time.monotonic() - start > 2.0
                        and cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
                    ):
                        stop_control["reason"] = "guide_window_closed"
                        stopped.set()
                except cv2.error:
                    pass

            now = time.monotonic()
            if now - last_state_write >= 1.0:
                ego_orb.check_alive()
                snapshot = node.snapshot()
                last_sensor_snapshot = sensors.snapshot()
                state = {
                    **provenance,
                    "status": (
                        "WORLD_RUNNING"
                        if snapshot["anchor_initialized"]
                        else "WAITING_FOR_WORLD_ANCHOR"
                    ),
                    "elapsed_s": now - start,
                    "live": snapshot,
                    "sensors": last_sensor_snapshot,
                    "ego_orb_process": ego_orb.stats(),
                    "remaining_s": (
                        None
                        if args.duration == 0
                        else max(0.0, args.duration - (now - start))
                    ),
                }
                _atomic_json(output / "run_status.json", state)
                node.publish_status(state)
                last_state_write = now
    finally:
        final = node.snapshot()
        failure_type, failure_value, _ = sys.exc_info()
        failure = (
            None
            if failure_value is None
            else {
                "type": failure_type.__name__ if failure_type is not None else "Exception",
                "message": str(failure_value),
            }
        )
        if stop_control["reason"] is None:
            stop_control["reason"] = "runtime_exception" if failure else "stop_event"
        provenance.update(
            status=(
                "FAILED_RUNTIME"
                if failure is not None
                else "STOPPED_AFTER_WORLD_RUN"
                if final["anchor_initialized"]
                else "STOPPED_WITHOUT_WORLD_ANCHOR"
            ),
            stop_reason=stop_control["reason"],
            failure=failure,
            ended_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            elapsed_s=time.monotonic() - start,
            live=final,
            sensors=sensors.snapshot(),
            ego_orb_process=ego_orb.stats(),
        )
        _atomic_json(output / "run_status.json", provenance)
        try:
            sensors.close()
        finally:
            try:
                ego_orb.finish_input()
                ego_orb.stop()
                provenance["ego_orb_process_after_stop"] = ego_orb.stats()
                _atomic_json(output / "run_status.json", provenance)
            finally:
                executor.shutdown()
                node.close()
                rclpy.shutdown()
                ros_thread.join(timeout=2.0)
                if not args.no_display:
                    cv2.destroyAllWindows()
                    cv2.waitKey(1)
    return 0 if final["anchor_initialized"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
