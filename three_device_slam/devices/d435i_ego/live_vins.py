"""Provisional live ROS2 input and process wrapper for D435i internal-IMU VINS."""

from __future__ import annotations

import array
import hashlib
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time

import numpy as np

from three_device_slam.spatial.d435i_live_vins import CombinedImu


EGO_VINS_BUILD = Path(
    "/home/robot/worktrees/ego-loop-probe-20260904/"
    ".loop_probe_build/loop_ws/build/vins_fusion_ros2"
)
EGO_VINS_EXECUTABLE = EGO_VINS_BUILD / "vins_fusion_ros2_node"
EGO_VINS_LIBRARY = EGO_VINS_BUILD / "vins/libvins_lib.so"
EGO_VINS_TYPESUPPORT = (
    EGO_VINS_BUILD / "libvins_fusion_ros2__rosidl_typesupport_fastrtps_cpp.so"
)
EXPECTED_HASHES = {
    EGO_VINS_EXECUTABLE: "ca75d47ea58933d5dcdbc7b6e761395ee4b8ebc4d9b920ffa3c2538a8c76b461",
    EGO_VINS_LIBRARY: "aaff4287dec21ce72e7a5afb1aef91d994fe6f3522b62db31cebf97aaf82d762",
    EGO_VINS_TYPESUPPORT: "a8e14520eee82aac9b3273f26b0004d2f9e852b1b14d586353a812e389e7be46",
}
DEFAULT_CONFIG_SOURCE = Path(
    "/home/robot/three-device-slam/artifacts/spatial_bench/"
    "ego_internal_vins_probe_20260906_run3"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare_ego_live_config(source: Path, output: Path) -> Path:
    """Copy the frozen provisional calibration and bind it to live namespaced topics."""
    source, output = source.resolve(), output.resolve()
    if output.exists():
        raise FileExistsError(f"Ego VINS output already exists: {output}")
    for name in ("vins_config.yaml", "left.yaml", "right.yaml"):
        if not (source / name).is_file():
            raise FileNotFoundError(source / name)
    text = (source / "vins_config.yaml").read_text(encoding="utf-8")
    replacements = {
        'imu_topic: "/ego_probe/imu0"': 'imu_topic: "/ego_live/imu0"',
        'image0_topic: "/ego_probe/cam0/image_raw"': (
            'image0_topic: "/ego_live/cam0/image_raw"'
        ),
        'image1_topic: "/ego_probe/cam1/image_raw"': (
            'image1_topic: "/ego_live/cam1/image_raw"'
        ),
    }
    for old, new in replacements.items():
        if text.count(old) != 1:
            raise ValueError(f"cannot bind frozen Ego VINS topic: {old}")
        text = text.replace(old, new)
    if len(re.findall(r'^output_path:.*$', text, flags=re.MULTILINE)) != 1:
        raise ValueError("frozen Ego VINS config has ambiguous output_path")
    output.mkdir(parents=True)
    solver_output = output / "solver_output"
    solver_output.mkdir()
    text = re.sub(
        r'^output_path:.*$',
        f'output_path: "{solver_output}"',
        text,
        count=1,
        flags=re.MULTILINE,
    )
    (output / "vins_config.yaml").write_text(text, encoding="utf-8")
    for name in ("left.yaml", "right.yaml"):
        (output / name).write_bytes((source / name).read_bytes())
    return output / "vins_config.yaml"


class EgoVinsInputPublisher:
    """Publish native-SI D435i stereo/IMU inputs without STM32 conversion."""

    def __init__(self, node, *, image_shape=(720, 1280), imu_lead_ns=5_000_000):
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, Imu

        qos = QoSProfile(
            depth=2000,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.Image = Image
        self.Imu = Imu
        self.image_shape = tuple(image_shape)
        self.imu_lead_ns = int(imu_lead_ns)
        self.cam0_pub = node.create_publisher(
            Image, "/ego_live/cam0/image_raw", qos
        )
        self.cam1_pub = node.create_publisher(
            Image, "/ego_live/cam1/image_raw", qos
        )
        self.imu_pub = node.create_publisher(Imu, "/ego_live/imu0", qos)
        self._camera_queue: queue.Queue = queue.Queue(maxsize=3)
        self._condition = threading.Condition()
        self._latest_imu_ns = -1
        self._stop = threading.Event()
        self.camera_published = 0
        self.camera_queue_dropped = 0
        self.camera_wait_dropped = 0
        self.imu_published = 0
        self._thread = threading.Thread(
            target=self._camera_loop, name="ego-vins-camera-publisher", daemon=True
        )
        self._thread.start()

    @staticmethod
    def _stamp(message, timestamp_ns: int) -> None:
        message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)

    def publish_imu(self, sample: CombinedImu) -> None:
        message = self.Imu()
        self._stamp(message, sample.timestamp_ns)
        message.header.frame_id = "ego_imu_body"
        message.angular_velocity.x = float(sample.gyro_rad_s[0])
        message.angular_velocity.y = float(sample.gyro_rad_s[1])
        message.angular_velocity.z = float(sample.gyro_rad_s[2])
        message.linear_acceleration.x = float(sample.accel_m_s2[0])
        message.linear_acceleration.y = float(sample.accel_m_s2[1])
        message.linear_acceleration.z = float(sample.accel_m_s2[2])
        self.imu_pub.publish(message)
        self.imu_published += 1
        with self._condition:
            self._latest_imu_ns = max(self._latest_imu_ns, sample.timestamp_ns)
            self._condition.notify_all()

    def publish_stereo(self, timestamp_ns: int, left, right) -> None:
        left = np.ascontiguousarray(left, dtype=np.uint8)
        right = np.ascontiguousarray(right, dtype=np.uint8)
        if left.shape != self.image_shape or right.shape != self.image_shape:
            raise ValueError("D435i live stereo image shape mismatch")
        item = (int(timestamp_ns), left.tobytes(), right.tobytes())
        try:
            self._camera_queue.put_nowait(item)
        except queue.Full:
            try:
                self._camera_queue.get_nowait()
                self.camera_queue_dropped += 1
            except queue.Empty:
                pass
            self._camera_queue.put_nowait(item)

    def _camera_message(self, timestamp_ns: int, frame_id: str, data: bytes):
        message = self.Image()
        self._stamp(message, timestamp_ns)
        message.header.frame_id = frame_id
        message.height, message.width = self.image_shape
        message.encoding = "mono8"
        message.is_bigendian = 0
        message.step = self.image_shape[1]
        message.data[:] = array.array("B", data)
        return message

    def _camera_loop(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._camera_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                continue
            timestamp_ns, left, right = item
            with self._condition:
                ready = self._condition.wait_for(
                    lambda: self._stop.is_set()
                    or self._latest_imu_ns >= timestamp_ns + self.imu_lead_ns,
                    timeout=0.25,
                )
            if self._stop.is_set():
                break
            if not ready:
                self.camera_wait_dropped += 1
                continue
            self.cam0_pub.publish(
                self._camera_message(timestamp_ns, "ego_ir_left", left)
            )
            self.cam1_pub.publish(
                self._camera_message(timestamp_ns, "ego_ir_right", right)
            )
            self.camera_published += 1

    def subscribers_ready(self) -> bool:
        return all(
            publisher.get_subscription_count() >= 1
            for publisher in (self.cam0_pub, self.cam1_pub, self.imu_pub)
        )

    def stats(self) -> dict:
        return {
            "ros_ego_cam_pairs_published": self.camera_published,
            "ros_ego_imu_published": self.imu_published,
            "ros_ego_cam_queue_dropped": self.camera_queue_dropped,
            "ros_ego_cam_wait_dropped": self.camera_wait_dropped,
            "ros_ego_cam_queue": self._camera_queue.qsize(),
            "vins_subscribers_ready": self.subscribers_ready(),
        }

    def close(self) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        try:
            self._camera_queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=2.0)


class EgoVinsProcess:
    """Own the hash-pinned provisional Ego VINS child process."""

    def __init__(self, output: Path, config_source: Path = DEFAULT_CONFIG_SOURCE):
        self.output = output.resolve()
        self.config_source = config_source.resolve()
        self.config: Path | None = None
        self.child: subprocess.Popen | None = None
        self.log_file = None

    def start(self) -> None:
        for path, expected in EXPECTED_HASHES.items():
            if not path.is_file() or sha256(path) != expected:
                raise RuntimeError(f"Ego VINS pinned binary hash mismatch: {path}")
        self.config = prepare_ego_live_config(self.config_source, self.output)
        (self.output / "ros_logs").mkdir()
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = str(EGO_VINS_BUILD) + os.pathsep + env.get(
            "LD_LIBRARY_PATH", ""
        )
        env["ROS_LOG_DIR"] = str(self.output / "ros_logs")
        env.update(
            OMP_NUM_THREADS="1",
            OMP_DYNAMIC="FALSE",
            OMP_THREAD_LIMIT="1",
            OPENBLAS_NUM_THREADS="1",
        )
        args = [
            str(EGO_VINS_EXECUTABLE),
            "--ros-args",
            "-r",
            "__ns:=/ego_live",
            "-p",
            f"config_file:={self.config}",
            "-p",
            "world_frame_id:=ego/local_world",
            "-p",
            "body_frame_id:=ego/imu_body",
            "-p",
            "camera_frame_id:=ego/ir_left",
        ]
        self.log_file = (self.output / "vins.log").open("wb")
        self.child = subprocess.Popen(
            args,
            cwd=self.output,
            env=env,
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def check_alive(self) -> None:
        if self.child is None or self.child.poll() is not None:
            code = None if self.child is None else self.child.returncode
            raise RuntimeError(f"Ego VINS process is not alive: returncode={code}")

    def snapshot(self) -> dict:
        return {
            "status": (
                "RUNNING"
                if self.child is not None and self.child.poll() is None
                else "STOPPED"
            ),
            "pid": None if self.child is None else self.child.pid,
            "config": None if self.config is None else str(self.config),
            "calibration_status": "FACTORY_GEOMETRY_ONLY_TD_NOISE_UNVALIDATED",
            "binary_hashes": {
                str(path): expected for path, expected in EXPECTED_HASHES.items()
            },
        }

    def stop(self) -> None:
        if self.child is not None and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGTERM)
                try:
                    self.child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(self.child.pid, signal.SIGKILL)
                    self.child.wait(timeout=3)
        if self.log_file is not None:
            self.log_file.close()
