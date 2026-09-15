"""Bounded, isolated replay of exported Ego sensors into an existing VINS binary.

No hardware drivers, loop fusion, service restarts or production writes. A healthy
static replay is not spatial calibration or shared-map acceptance.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2
import numpy as np
from rosbags.rosbag2 import Reader

from three_device_slam.spatial.covins_export import sha256
from three_device_slam.devices.d435i_ego.scene_quality import assess_export


def check_replay_liveness(child, began, now, limit_s=85):
    if child.poll() is not None or now - began > limit_s:
        raise RuntimeError(f"VINS exited or {limit_s:g}s bounded replay expired")


def diagnostic_config(config, old_output, new_output, stereo_only_control):
    config = config.replace(old_output, str(new_output))
    if stereo_only_control:
        if config.count("\nimu: 1\n") != 1:
            raise ValueError("cannot unambiguously disable IMU for diagnostic control")
        config = config.replace("\nimu: 1\n", "\nimu: 0\n", 1)
    return config


def runtime_provenance(executable, env):
    paths = {"runner": Path(__file__).resolve(), "executable": executable,
             "solver_library": executable.parent / "vins/libvins_lib.so",
             "introspection": executable.parent / "libvins_fusion_ros2__rosidl_typesupport_introspection_cpp.so",
             "typesupport": executable.parent / "libvins_fusion_ros2__rosidl_typesupport_fastrtps_cpp.so"}
    return {"files": {key: {"path": str(path), "sha256": sha256(path)} for key, path in paths.items()},
            "effective_ld_library_path": env["LD_LIBRARY_PATH"],
            "diagnostic_logging": env.get("EGO_VINS_DIAGNOSTIC") is not None,
            "thread_environment": {key: env.get(key) for key in ("OMP_NUM_THREADS", "OMP_DYNAMIC", "OMP_THREAD_LIMIT", "OPENBLAS_NUM_THREADS")},
            "scope": "runner, executable and primary local VINS libraries; not full system dependency closure"}


def replay_status(checks, scene_supported, stereo_only_control):
    if not checks or not all(checks.values()):
        return "FAIL"
    if not scene_supported:
        return "DIAGNOSTIC_REPLAY_COMPLETE"
    return "VISION_ONLY_CONTROL_PASS" if stereo_only_control else "PROVISIONAL_REPLAY_PASS"


def run(source, output, executable, *, stereo_only_control=False, allow_unsupported_scene=False):
    source, output, executable = source.resolve(), output.resolve(), executable.resolve()
    root = Path(__file__).resolve().parents[1]
    if output.exists() or not output.is_relative_to(root / "artifacts") or output == root / "artifacts":
        raise ValueError("output must be NEW under repository artifacts")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "92":
        raise ValueError("isolated probe requires ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=92")
    report = json.loads((source / "input_report.json").read_text())
    if report["status"] not in {"PROVISIONAL_INPUT_READY", "INPUT_READY"}:
        raise ValueError("input not ready")
    source_mode = report["source_mode"]
    if source_mode not in {
        "bench_no_motion", "shared_world_motion", "device3_anchor_hil"
    }:
        raise ValueError("unsupported physical capture mode")
    agent = report.get("agent", "ego")
    if agent not in {"ego", "left", "right"}:
        raise ValueError("invalid agent identity")
    namespace = f"{agent}_probe"
    for path, expected in report["outputs"].items():
        if sha256(source / path) != expected:
            raise ValueError("input artifact hash mismatch: " + path)
    fs = cv2.FileStorage(str(source / "vins_config.yaml"), cv2.FILE_STORAGE_READ)
    if not fs.isOpened() or fs.getNode("imu").real() != 1:
        raise ValueError("built-in IMU VINS config required")
    old_output = fs.getNode("output_path").string()
    fs.release()
    output.mkdir(parents=True)
    scene = assess_export(source, output / "scene_quality")
    scene_supported = scene["status"] == "STEREO_SUPPORT_PASS"
    if not scene_supported and not allow_unsupported_scene:
        result = {"schema": "three-device-slam.offline-vins-replay.v1", "status": "BLOCKED_SCENE_SUPPORT",
                  "agent": agent,
                  "shared_world_status": "NOT_RUN", "solver_started": False,
                  "scene_support_status": scene["status"], "scene_report": str(output / "scene_quality/report.json"),
                  "failure": "insufficient stereo scene support; not a transport failure"}
        (output / "report.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        print("NEXT: 场景预检未通过，未启动SLAM；相机需看到清晰、有纹理且分布充分的静态场景。", flush=True)
        return 2
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu

    (output / "solver_output").mkdir()
    config = diagnostic_config((source / "vins_config.yaml").read_text(), old_output,
                               output / "solver_output", stereo_only_control)
    for name in ("left.yaml", "right.yaml"):
        (output / name).write_bytes((source / name).read_bytes())
    (output / "vins_config.yaml").write_text(config)
    # The parent rclpy node also logs; set before init, not only for the child.
    os.environ["ROS_LOG_DIR"] = str(output / "ros_logs")
    env = os.environ.copy()
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    # rosidl loads this plugin with dlopen; ldd on the executable alone cannot
    # prove it is discoverable. Use only this binary's own build directory.
    support = executable.parent / "libvins_fusion_ros2__rosidl_typesupport_fastrtps_cpp.so"
    if not support.is_file():
        raise ValueError("matching VINS FastRTPS type-support library missing")
    env["LD_LIBRARY_PATH"] = str(executable.parent) + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    provenance = runtime_provenance(executable, env)
    (output / "runner_snapshot.py").write_bytes(Path(__file__).read_bytes())
    (output / "runtime_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    args = [str(executable), "--ros-args", "-r", f"__ns:=/{namespace}", "-p",
            "config_file:=" + str(output / "vins_config.yaml"), "-p", f"world_frame_id:={agent}/epoch0/world",
            "-p", f"body_frame_id:={agent}/imu_raw", "-p", f"camera_frame_id:={agent}/ir_left"]
    rclpy.init()
    node = rclpy.create_node(f"{agent}_offline_probe_sink", namespace=f"/{namespace}")
    configured_topics = report.get("topics", {})
    image0_topic = configured_topics.get("image0", f"/{namespace}/cam0/image_raw")
    image1_topic = configured_topics.get("image1", f"/{namespace}/cam1/image_raw")
    imu_topic = configured_topics.get("imu", f"/{namespace}/imu0")
    odometry_topic = f"/{namespace}/odometry"
    specs = [
        (image0_topic, Image, 100), (image1_topic, Image, 100),
        (imu_topic, Imu, 2000)]
    if stereo_only_control:
        specs = specs[:2]
    pubs = {topic: node.create_publisher(cls, topic, depth) for topic, cls, depth in specs}
    poses, frames = [], []
    def callback(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses.append([msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        frames.append((msg.header.frame_id, msg.child_frame_id))
    node.create_subscription(Odometry, odometry_topic, callback, 2000)
    child = None
    failure = None
    counts = {topic: 0 for topic in pubs}
    maximum_lag = 0.
    image_stamps = []
    began = time.monotonic()
    replay_limit_s = max(85.0, (report["formal_end_ns"] - report["formal_start_ns"]) / 1e9 + 30.0)
    try:
        # No production publishers or competing replay allowed on these topics.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.05)
        if (any(p.get_subscription_count() for p in pubs.values())
                or any(node.count_publishers(t) != 1 for t in pubs)
                or node.count_publishers(odometry_topic)):
            raise RuntimeError("probe topics already occupied in isolated domain")
        with (output / "vins.log").open("wb") as log:
            child = subprocess.Popen(args, cwd=output, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            print(f"STARTED: VINS pid={child.pid}; domain92 localhost; IMU={'OFF diagnostic control' if stereo_only_control else 'ON'}; no hardware", flush=True)
            ready_deadline = time.monotonic() + 15
            while not all(p.get_subscription_count() == 1 for p in pubs.values()):
                if child.poll() is not None:
                    raise RuntimeError(f"VINS exited during startup: {child.returncode}")
                if time.monotonic() > ready_deadline:
                    raise TimeoutError("sensor subscriptions not ready in15s")
                rclpy.spin_once(node, timeout_sec=.02)
            print("READY: required sensor subscribers matched; replay at original timestamps/rate", flush=True)
            with Reader(source / "sensors") as reader:
                wall0, stamp0, last_report = time.monotonic(), None, time.monotonic()
                for conn, stamp, payload in reader.messages():
                    if stereo_only_control and conn.topic == "/ego_probe/imu0":
                        continue
                    if stamp0 is None:
                        stamp0 = stamp
                    due = wall0 + (stamp - stamp0) / 1e9
                    while time.monotonic() < due:
                        check_replay_liveness(child, began, time.monotonic(), replay_limit_s)
                        rclpy.spin_once(node, timeout_sec=min(.01, max(0., due - time.monotonic())))
                    check_replay_liveness(child, began, time.monotonic(), replay_limit_s)
                    maximum_lag = max(maximum_lag, time.monotonic() - due)
                    pubs[conn.topic].publish(bytes(payload))
                    counts[conn.topic] += 1
                    if conn.topic == image0_topic:
                        image_stamps.append(stamp)
                    rclpy.spin_once(node, timeout_sec=0.)
                    if time.monotonic() - last_report >= 5:
                        print(f"REPLAY: {(stamp-stamp0)/1e9:.1f}s poses={len(poses)} counts={counts}", flush=True)
                        last_report = time.monotonic()
            drain = time.monotonic() + 5
            while time.monotonic() < drain:
                rclpy.spin_once(node, timeout_sec=.02)
                if child.poll() is not None:
                    raise RuntimeError(f"VINS exited during drain: {child.returncode}")
    except (RuntimeError, TimeoutError, KeyboardInterrupt) as exc:
        failure = type(exc).__name__ + ":" + str(exc)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    with (output / "odometry_raw.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "x", "y", "z", "qw", "qx", "qy", "qz"])
        writer.writerows(poses)
    xyz = np.array([p[1:4] for p in poses])
    stamps = np.array([p[0] for p in poses], dtype=np.int64)
    expected = report["counts"]["ir_left"]
    from scipy.spatial.transform import Rotation
    quaternions = np.array([p[4:8] for p in poses])
    valid_quaternions = bool(poses) and bool(np.all(np.isfinite(quaternions))) and bool(
        np.all(np.abs(np.linalg.norm(quaternions, axis=1) - 1) <= 1e-5))
    rotation_excursion = None
    if valid_quaternions:
        rotations = Rotation.from_quat(quaternions[:, [1, 2, 3, 0]])
        rotation_excursion = float(np.max((rotations[0].inv() * rotations).magnitude()) * 180 / np.pi)
    pose_time_matched = bool(poses) and bool(image_stamps) and all(
        min(abs(int(stamp) - image_stamp) for image_stamp in image_stamps) <= 1000 for stamp in stamps)
    max_step = float(np.max(np.linalg.norm(np.diff(xyz, axis=0), axis=1))) if len(xyz) > 1 else None
    max_excursion = float(np.max(np.linalg.norm(xyz - xyz[0], axis=1))) if len(poses) else None
    checks = {"runtime_completed": failure is None,
        "all_images_published": counts[image0_topic] == counts[image1_topic] == expected,
        "imu_policy_satisfied": (counts.get(imu_topic, 0) == (0 if stereo_only_control else report["combined_imu_samples"])),
        "pose_coverage_ge_95pct": len(poses) >= .95 * expected,
        "pose_timestamp_strictly_increasing": len(poses) > 1 and bool(np.all(np.diff(stamps) > 0)),
        "finite_poses": bool(poses) and bool(np.all(np.isfinite(np.array(poses, dtype=float)))),
        "pose_stamps_match_images_within_1us": pose_time_matched,
        "unit_quaternions": valid_quaternions,
        "frames": bool(frames) and all(f == (f"{agent}/epoch0/world", f"{agent}/imu_raw") for f in frames)}
    if source_mode == "bench_no_motion" or (
        source_mode == "device3_anchor_hil" and agent == "ego"
    ):
        checks.update({
            "static_rotation_excursion_le_5deg": rotation_excursion is not None and rotation_excursion <= 5.,
            "static_max_excursion_le_5cm": len(poses) > 1 and max_excursion <= .05,
        })
    else:
        checks.update({
            "dynamic_translation_excursion_ge_20cm": max_excursion is not None and max_excursion >= .20,
            "dynamic_rotation_excursion_ge_20deg": rotation_excursion is not None and rotation_excursion >= 20.,
            "raw_pose_step_le_10cm": max_step is not None and max_step <= .10,
        })
    status = replay_status(checks, scene_supported, stereo_only_control)
    if status == "PROVISIONAL_REPLAY_PASS" and source_mode in {
        "shared_world_motion", "device3_anchor_hil"
    }:
        status = "VIO_REPLAY_PASS"
    result = {"schema": "three-device-slam.offline-vins-replay.v1", "status": status,
        "agent": agent, "source_mode": source_mode,
        "imu_enabled": not stereo_only_control,
        "solver_started": child is not None, "scene_support_status": scene["status"],
        "scene_support_override": not scene_supported and allow_unsupported_scene,
        "scene_report": str(output / "scene_quality/report.json"),
        "shared_world_status": "NOT_RUN", "calibration_status": report["calibration_status"],
        "failure": failure, "checks": checks, "pose_count": len(poses), "published": counts,
        "max_publish_schedule_lag_s": maximum_lag,
        "max_excursion_m": max_excursion,
        "max_pose_step_m": max_step,
        "max_rotation_excursion_deg": rotation_excursion,
        "source_input_report": {"path": str(source / "input_report.json"), "sha256": sha256(source / "input_report.json")},
        "executable": {"path": str(executable), "sha256": sha256(executable)},
        "typesupport": {"path": str(support), "sha256": sha256(support)},
        "runtime_provenance": provenance,
        "runtime_config_sha256": sha256(output / "vins_config.yaml"), "command": args,
        "ros_domain_id": 92, "ros_localhost_only": 1, "duration_wall_s": time.monotonic() - began,
        "limitations": (["factory geometry/reference noise/td0 not calibrated"] if report["calibration_status"].startswith("FACTORY") else [])
                       + ["VIO replay only", "not cross-device map alignment"]}
    (output / "report.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if all(checks.values()) else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--stereo-only-control", action="store_true", help="diagnostic only; never claims IMU replay acceptance")
    parser.add_argument("--allow-unsupported-scene", action="store_true", help="explicit diagnostic override; output never claims replay PASS for an unsupported scene")
    args = parser.parse_args()
    raise SystemExit(run(args.input, args.output, args.executable, stereo_only_control=args.stereo_only_control,
                        allow_unsupported_scene=args.allow_unsupported_scene))
