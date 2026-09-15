#!/usr/bin/env python3
"""Lightweight UMI-style Rerun viewer for the shared AprilGrid world."""

from __future__ import annotations

import argparse
from collections import deque
import json
import os
import shutil
import signal
import socket
import subprocess
import time

import numpy as np


RERUN_PORT = 9877
COLORS = {"ego": [70, 150, 255], "device3": [255, 150, 40]}
POSE_DRAW_PERIOD_S = 0.10
TRAJECTORY_DRAW_PERIOD_S = 0.25
POSE_HISTORY_PERIOD_S = 2.0
IMAGE_DRAW_PERIOD_S = 1.0 / 3.0
IMU_DRAW_PERIOD_S = 0.20

# ``aprilgrid_object_corners`` uses the frozen legacy board winding.  Seen
# from the printed side, its right-handed frame is X=left, Y=down, Z=back
# (the board normal points toward the cameras).  Declaring RDF here used to
# make Rerun interpret both X and Z in the opposite physical direction.
WORLD_VIEW_COORDINATES = "LDB"
APRILGRID_ROWS = 6
APRILGRID_COLUMNS = 6
APRILGRID_TAG_SIZE_M = 0.0352
APRILGRID_GAP_M = 0.01056


def _port_is_open(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.1):
            return True
    except OSError:
        return False


def _start_rerun(port: int) -> subprocess.Popen:
    if _port_is_open(port):
        raise RuntimeError(f"Rerun端口{port}已被旧窗口占用")
    executable = shutil.which("rerun")
    if executable is None:
        raise RuntimeError("找不到rerun可执行程序")
    rerun_threads = os.environ.get("DEVICE3_WORLD_RERUN_THREADS", "2")
    if not rerun_threads.isdigit() or int(rerun_threads) < 1:
        raise RuntimeError("DEVICE3_WORLD_RERUN_THREADS必须是正整数")
    process = subprocess.Popen(
        [
            executable,
            f"--port={port}",
            f"--threads={rerun_threads}",
            "--memory-limit=1GB",
            "--drop-at-latency=250ms",
            "--expect-data-soon",
        ],
        start_new_session=True,
    )
    for _ in range(100):
        if process.poll() is not None:
            raise RuntimeError(f"Rerun启动失败，退出码={process.returncode}")
        if _port_is_open(port):
            return process
        time.sleep(0.1)
    process.terminate()
    process.wait(timeout=2.0)
    raise RuntimeError("Rerun启动超时")


def _stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3.0)


def _fixed_grid(extent_m: float = 1.0, step_m: float = 0.1):
    values = np.arange(-extent_m, extent_m + step_m / 2.0, step_m)
    lines = []
    for value in values:
        lines.extend(
            [
                [[-extent_m, value, -extent_m], [extent_m, value, -extent_m]],
                [[value, -extent_m, -extent_m], [value, extent_m, -extent_m]],
                [[-extent_m, extent_m, value], [extent_m, extent_m, value]],
                [[value, extent_m, -extent_m], [value, extent_m, extent_m]],
                [[-extent_m, value, -extent_m], [-extent_m, value, extent_m]],
                [[-extent_m, -extent_m, value], [-extent_m, extent_m, value]],
            ]
        )
    return lines


def _grid_planes(extent_m: float = 1.0):
    low, high = -extent_m, extent_m
    vertices = [
        [low, low, low], [high, low, low], [high, high, low], [low, high, low],
        [low, high, low], [high, high, low], [high, high, high], [low, high, high],
        [low, low, low], [low, high, low], [low, high, high], [low, low, high],
    ]
    triangles = [
        [0, 1, 2], [0, 2, 3],
        [4, 5, 6], [4, 6, 7],
        [8, 9, 10], [8, 10, 11],
    ]
    return vertices, triangles


def _aprilgrid_reference():
    """Return the metric board plane and labeled corner-tag centers."""
    pitch = APRILGRID_TAG_SIZE_M + APRILGRID_GAP_M
    low = -APRILGRID_TAG_SIZE_M / 2.0
    high_x = (APRILGRID_COLUMNS - 1) * pitch + APRILGRID_TAG_SIZE_M / 2.0
    high_y = (APRILGRID_ROWS - 1) * pitch + APRILGRID_TAG_SIZE_M / 2.0
    vertices = [
        [low, low, 0.0],
        [high_x, low, 0.0],
        [high_x, high_y, 0.0],
        [low, high_y, 0.0],
    ]
    outline = [[vertices[index], vertices[(index + 1) % 4]] for index in range(4)]
    ids = (0, APRILGRID_COLUMNS - 1,
           (APRILGRID_ROWS - 1) * APRILGRID_COLUMNS,
           APRILGRID_ROWS * APRILGRID_COLUMNS - 1)
    centers = [
        [(tag_id % APRILGRID_COLUMNS) * pitch,
         (tag_id // APRILGRID_COLUMNS) * pitch, 0.0]
        for tag_id in ids
    ]
    return vertices, outline, centers, [f"ID{tag_id}" for tag_id in ids]


def _message_pose(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return (
        np.array((position.x, position.y, position.z), dtype=np.float64),
        np.array(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            dtype=np.float64,
        ),
    )


def _message_timestamp_s(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1e9


def _message_image(message) -> np.ndarray:
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding in ("mono8", "8UC1"):
        return np.ascontiguousarray(
            raw.reshape(int(message.height), int(message.step))[:, : int(message.width)]
        )
    if message.encoding in ("bgr8", "rgb8"):
        row = raw.reshape(int(message.height), int(message.step))
        image = row[:, : int(message.width) * 3].reshape(
            int(message.height), int(message.width), 3
        )
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)
    raise ValueError(f"unsupported ROS image encoding: {message.encoding}")


def _rotate_vectors(vectors: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12 or not np.isfinite(norm):
        raise ValueError("invalid quaternion")
    xyz = quaternion[:3] / norm
    w = float(quaternion[3] / norm)
    first = np.cross(xyz, vectors)
    second = np.cross(xyz, first)
    return vectors + 2.0 * (w * first + second)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=RERUN_PORT)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args(argv)

    import rclpy
    import rerun as rr
    import rerun.blueprint as rrb
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image as RosImage, Imu
    from std_msgs.msg import String

    runtime = {"rerun_process": None, "initialized": False}

    def ensure_viewer() -> None:
        if runtime["initialized"]:
            return
        if not args.headless:
            runtime["rerun_process"] = _start_rerun(args.port)
        rr.init("ego_device3_aprilgrid_world", spawn=False)
        if not args.headless:
            connect = getattr(rr, "connect_tcp", None) or rr.connect
            connect(f"127.0.0.1:{args.port}")
        right_panel = rrb.Vertical(
            rrb.Spatial2DView(origin="world/ego/image", name="Ego 左红外"),
            rrb.Spatial2DView(origin="world/device3/image", name="设备3 左红外"),
            rrb.TimeSeriesView(origin="world/ego/imu/**", name="Ego IMU"),
            rrb.TimeSeriesView(origin="world/device3/imu/**", name="设备3 IMU"),
            rrb.TextDocumentView(origin="status", name="运行状态"),
            row_shares=[3.0, 3.0, 1.5, 1.5, 1.2],
        )
        rr.send_blueprint(
            rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(
                        origin="world",
                        name="AprilGrid 固定世界（X列 / Y行 / Z朝相机侧）",
                        background=[40, 40, 40],
                    ),
                    right_panel,
                    column_shares=[0.7, 0.3],
                ),
                collapse_panels=True,
            ),
            make_active=True,
            make_default=True,
        )
        rr.log(
            "world",
            getattr(rr.ViewCoordinates, WORLD_VIEW_COORDINATES),
            static=True,
        )
        vertices, triangles = _grid_planes()
        rr.log(
            "world/grid_planes",
            rr.Mesh3D(
                vertex_positions=vertices,
                triangle_indices=triangles,
                albedo_factor=[210, 210, 210, 110],
            ),
            static=True,
        )
        rr.log(
            "world/grid",
            rr.LineStrips3D(_fixed_grid(), colors=[[80, 80, 80]], radii=0.001),
            static=True,
        )
        rr.log(
            "world/origin_axes",
            rr.Arrows3D(
                origins=[[0.0, 0.0, 0.0]] * 3,
                vectors=[[0.25, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, 0.25]],
                colors=[[255, 50, 50], [50, 255, 50], [50, 100, 255]],
                labels=["+X 标签列", "+Y 标签行", "+Z 朝相机侧"],
            ),
            static=True,
        )
        board_vertices, board_outline, board_centers, board_labels = (
            _aprilgrid_reference()
        )
        rr.log(
            "world/aprilgrid/plane",
            rr.Mesh3D(
                vertex_positions=board_vertices,
                triangle_indices=[[0, 1, 2], [0, 2, 3]],
                albedo_factor=[235, 235, 235, 180],
            ),
            static=True,
        )
        rr.log(
            "world/aprilgrid/outline",
            rr.LineStrips3D(board_outline, colors=[[255, 220, 40]], radii=0.003),
            static=True,
        )
        rr.log(
            "world/aprilgrid/corner_ids",
            rr.Points3D(
                board_centers,
                labels=board_labels,
                colors=[[255, 220, 40]],
                radii=0.006,
                show_labels=True,
            ),
            static=True,
        )
        runtime["initialized"] = True
        print("[3D] 双锚已建立，轻量UMI风格窗口现在启动", flush=True)

    rclpy.init(args=[])
    node = Node("world_pair_rerun_viewer")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )
    tracks = {name: deque(maxlen=3000) for name in COLORS}
    pose_history = {name: deque(maxlen=30) for name in COLORS}
    last_pose_draw = {name: 0.0 for name in COLORS}
    last_trajectory_draw = {name: 0.0 for name in COLORS}
    last_history = {name: 0.0 for name in COLORS}
    last_image_draw = {name: 0.0 for name in COLORS}
    last_imu_draw = {name: 0.0 for name in COLORS}
    counts = {name: 0 for name in COLORS}
    image_counts = {name: 0 for name in COLORS}
    imu_counts = {name: 0 for name in COLORS}
    last_stats_mono = 0.0

    def pose_callback(name: str):
        def receive(message: Odometry) -> None:
            position, quaternion = _message_pose(message)
            if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
                return
            if not runtime["initialized"] and name != "ego":
                return
            ensure_viewer()
            tracks[name].append(position.tolist())
            counts[name] += 1
            now = time.monotonic()
            draw_pose = now - last_pose_draw[name] >= POSE_DRAW_PERIOD_S
            draw_trajectory = (
                now - last_trajectory_draw[name] >= TRAJECTORY_DRAW_PERIOD_S
                and len(tracks[name]) >= 2
            )
            draw_history = now - last_history[name] >= POSE_HISTORY_PERIOD_S
            if not (draw_pose or draw_trajectory or draw_history):
                return
            timestamp = _message_timestamp_s(message)
            set_time = getattr(rr, "set_time", None)
            if set_time is not None:
                set_time("sensor_time", timestamp=timestamp)
            else:
                rr.set_time_seconds("sensor_time", timestamp)
            if draw_pose:
                axes = _rotate_vectors(np.eye(3) * 0.06, quaternion)
                rr.log(
                    f"world/{name}/axes",
                    rr.Arrows3D(
                        origins=[position.tolist()] * 3,
                        vectors=axes.tolist(),
                        colors=[[255, 60, 60], [60, 255, 60], [60, 120, 255]],
                    ),
                )
                rr.log(
                    f"world/{name}/current",
                    rr.Points3D(
                        [position.tolist()],
                        colors=[COLORS[name]],
                        radii=0.012,
                        labels=["Ego" if name == "ego" else "设备3"],
                    ),
                )
                last_pose_draw[name] = now
            if draw_trajectory:
                rr.log(
                    f"world/{name}/trajectory",
                    rr.LineStrips3D(
                        [list(tracks[name])], colors=[COLORS[name]], radii=0.004
                    ),
                )
                last_trajectory_draw[name] = now
            if draw_history:
                pose_history[name].append((position.copy(), quaternion.copy()))
                origins, vectors, colors = [], [], []
                for historical_position, historical_quaternion in pose_history[name]:
                    historical_axes = _rotate_vectors(
                        np.eye(3) * 0.025, historical_quaternion
                    )
                    origins.extend([historical_position.tolist()] * 3)
                    vectors.extend(historical_axes.tolist())
                    colors.extend(
                        [[180, 60, 60], [60, 180, 60], [60, 90, 180]]
                    )
                rr.log(
                    f"world/{name}/pose_axes_history",
                    rr.Arrows3D(origins=origins, vectors=vectors, colors=colors),
                )
                last_history[name] = now

        return receive

    def image_callback(name: str):
        def receive(message: RosImage) -> None:
            if not runtime["initialized"]:
                return
            now = time.monotonic()
            if now - last_image_draw[name] < IMAGE_DRAW_PERIOD_S:
                return
            try:
                image = _message_image(message)
            except (TypeError, ValueError):
                return
            rr.log(f"world/{name}/image", rr.Image(image))
            last_image_draw[name] = now
            image_counts[name] += 1

        return receive

    def imu_callback(name: str):
        def receive(message: Imu) -> None:
            if not runtime["initialized"]:
                return
            now = time.monotonic()
            if now - last_imu_draw[name] < IMU_DRAW_PERIOD_S:
                return
            values = {
                "accel/x": message.linear_acceleration.x,
                "accel/y": message.linear_acceleration.y,
                "accel/z": message.linear_acceleration.z,
                "gyro/x": message.angular_velocity.x,
                "gyro/y": message.angular_velocity.y,
                "gyro/z": message.angular_velocity.z,
            }
            scalar = getattr(rr, "Scalar", None)
            for path, value in values.items():
                archetype = scalar(float(value)) if scalar else rr.Scalars([float(value)])
                rr.log(f"world/{name}/imu/{path}", archetype)
            last_imu_draw[name] = now
            imu_counts[name] += 1

        return receive

    def status_callback(message: String) -> None:
        nonlocal last_stats_mono
        if not runtime["initialized"] or time.monotonic() - last_stats_mono < 0.5:
            return
        last_stats_mono = time.monotonic()
        try:
            payload = json.loads(message.data)
            live = payload.get("live", {})
            sensors = payload.get("sensors", {})
            timing = sensors.get("timestamp_contract", {}).get(
                "cross_camera_nearest", {}
            )
            orb = payload.get("ego_orb_process", {})
            text = (
                f"# Ego + 设备3 世界坐标\n\n"
                f"- 状态：**{payload.get('status')}**\n"
                f"- 运行模式：**{payload.get('runtime_control', {}).get('display')}**\n"
                f"- 世界轴：AprilGrid +X标签列、+Y标签行、+Z朝相机侧（LDB）\n"
                f"- 双锚：**{live.get('anchor_initialized')}**\n"
                f"- 时间同步：**{timing.get('status')}**，"
                f"P95 {timing.get('p95_ms')} ms，最大 {timing.get('max_ms')} ms\n"
                f"- ORB队列：{orb.get('write_queue')}，"
                f"跳帧：{orb.get('stereo_dropped_backpressure')}\n"
                f"- 设备3完整性：{live.get('pose_integrity_failure')}"
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            text = message.data
        rr.log("status", rr.TextDocument(text, media_type="text/markdown"))

    node.create_subscription(Odometry, "/ego/world_odometry", pose_callback("ego"), qos)
    node.create_subscription(
        Odometry, "/device3/world_odometry", pose_callback("device3"), qos
    )
    node.create_subscription(
        RosImage, "/ego/preview/image_raw", image_callback("ego"), qos
    )
    node.create_subscription(
        RosImage, "/device3/preview/image_raw", image_callback("device3"), qos
    )
    node.create_subscription(Imu, "/ego/imu_preview", imu_callback("ego"), qos)
    node.create_subscription(
        Imu, "/device3/imu_preview", imu_callback("device3"), qos
    )
    node.create_subscription(String, "/device3/world_status", status_callback, 10)
    print("[3D] 等待Ego世界位姿；双锚建立后启动轻量UMI界面", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if runtime["initialized"]:
            rr.disconnect()
        _stop_process(runtime["rerun_process"])
    print(
        f"[3D] 停止：位姿Ego={counts['ego']}，设备3={counts['device3']}；"
        f"图像Ego={image_counts['ego']}，设备3={image_counts['device3']}；"
        f"IMU Ego={imu_counts['ego']}，设备3={imu_counts['device3']}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
