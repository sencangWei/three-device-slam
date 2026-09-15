"""Offline raw VINS + matching image export for the COVINS-G ROS1 wrapper.

This produces frontend INPUT, not a merged map or a spatial-accuracy acceptance.
No ROS node, camera, solver, installer or external-repository mutation is
performed.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from decimal import Decimal
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import re
import sqlite3

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .se3 import validate_transform

UPSTREAM_REVISION = "c5b180b443b59d2267a14584fa4b090429038698"
IMAGE_TOPIC = "/device_0/sensor_0/Infrared_1/image/data"
MATCH_TOLERANCE_NS = 1000  # Only float/decimal conversion, NOT delay estimation.
ARTIFACT_ROOT = Path(__file__).resolve().parents[2] / "artifacts"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_poses(path: Path) -> tuple[list[int], list[np.ndarray]]:
    stamps, transforms = [], []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ["t_sec", "x", "y", "z", "qw", "qx", "qy", "qz"]:
            raise ValueError("expected test_vins_auto_loop raw CSV schema")
        for row in reader:
            stamp = int(Decimal(row["t_sec"]) * 1_000_000_000)
            if stamp < 0 or (stamps and stamp <= stamps[-1]):
                raise ValueError("nonmonotonic/duplicate pose timestamp: split map epochs")
            q = np.array([float(row[k]) for k in ("qx", "qy", "qz", "qw")])
            if not np.all(np.isfinite(q)) or abs(np.linalg.norm(q) - 1) > 1e-5:
                raise ValueError("invalid/non-unit quaternion")
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_quat(q).as_matrix()
            transform[:3, 3] = [float(row[k]) for k in ("x", "y", "z")]
            validate_transform(transform)
            if transforms and np.linalg.norm(transform[:3, 3] - transforms[-1][:3, 3]) > 0.05:
                raise ValueError("raw pose step exceeds audited 0.05 m gate; possible reset")
            stamps.append(stamp)
            transforms.append(transform)
    if len(stamps) < 2:
        raise ValueError("at least two raw poses required")
    return stamps, transforms


def load_pair_vins_poses(path: Path) -> tuple[list[int], list[np.ndarray]]:
    """Load the bounded replay runner's integer-nanosecond raw odometry."""
    stamps, transforms = [], []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ["timestamp_ns", "x", "y", "z", "qw", "qx", "qy", "qz"]:
            raise ValueError("expected pair VINS raw CSV schema")
        for row in reader:
            stamp = int(row["timestamp_ns"])
            if stamp < 0 or (stamps and stamp <= stamps[-1]):
                raise ValueError("nonmonotonic/duplicate pose timestamp: split map epochs")
            q = np.array([float(row[k]) for k in ("qx", "qy", "qz", "qw")])
            position = np.array([float(row[k]) for k in ("x", "y", "z")])
            if (not np.all(np.isfinite(q)) or not np.all(np.isfinite(position))
                    or abs(np.linalg.norm(q) - 1) > 1e-5):
                raise ValueError("invalid pair VINS pose")
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_quat(q).as_matrix()
            transform[:3, 3] = position
            validate_transform(transform)
            if transforms and np.linalg.norm(position - transforms[-1][:3, 3]) > 0.05:
                raise ValueError("raw pose step exceeds audited 0.05 m gate; possible reset")
            stamps.append(stamp)
            transforms.append(transform)
    if len(stamps) < 2:
        raise ValueError("at least two raw poses required")
    return stamps, transforms


def pair_stamps(poses: list[int], images: list[int]) -> list[int]:
    if not images or any(b <= a for a, b in zip(images, images[1:])):
        raise ValueError("image timestamps must be nonempty and strictly increasing")
    selected = []
    for stamp in poses:
        pos = bisect_left(images, stamp)
        candidates = [i for i in (pos - 1, pos) if 0 <= i < len(images)]
        index = min(candidates, key=lambda i: abs(images[i] - stamp))
        if abs(images[index] - stamp) > MATCH_TOLERANCE_NS:
            raise ValueError("pose/image mismatch >1 us; wrong clock/session, no interpolation allowed")
        if selected and index <= selected[-1]:
            raise ValueError("image reuse or pose/image order mismatch")
        selected.append(index)
    return selected


def load_native_frame_stamps(path: Path) -> tuple[list[int], list[int]]:
    """Read absolute D405 device times that correspond by row to recorder images."""
    stamps, frame_numbers = [], []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"infrared_left_device_ms", "infrared_left_frame_number", "infrared_left_domain"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("native frame index is missing left-IR identity/time fields")
        for row in reader:
            if row["infrared_left_domain"] != "global_time":
                raise ValueError("native left-IR clock must be global_time")
            stamp = int(Decimal(row["infrared_left_device_ms"]) * 1_000_000)
            frame = int(row["infrared_left_frame_number"])
            if stamp < 0 or (stamps and stamp <= stamps[-1]):
                raise ValueError("native left-IR timestamps must be strictly increasing")
            if frame < 0 or (frame_numbers and frame != frame_numbers[-1] + 1):
                raise ValueError("native left-IR frame numbers must be contiguous")
            stamps.append(stamp)
            frame_numbers.append(frame)
    if len(stamps) < 2:
        raise ValueError("native frame index needs at least two rows")
    return stamps, frame_numbers


def native_candidate_sources(run: Path, session: Path, acceptance: dict, serial: str) -> tuple[dict, dict, dict]:
    """Validate a workstation candidate-A/B recording without weakening RK3576 input."""
    capture_path = session / "acceptance.json"
    manifest_path = session / "candidate_ab_capture.yaml"
    frame_index = session / "d405_frames.csv"
    capture = json.loads(capture_path.read_text())
    manifest = yaml.safe_load(manifest_path.read_text())
    if not isinstance(manifest, dict):
        raise ValueError("invalid native capture manifest")
    if capture.get("result") != "PASS" or capture.get("capture_error") is not None:
        raise ValueError("native source capture is not PASS")
    if str(capture.get("camera_serial")) != serial or str(manifest.get("d405_serial")) != serial:
        raise ValueError("wrong physical device serial")
    if Path(str(capture.get("session", ""))).name != session.name:
        raise ValueError("native capture session identity mismatch")
    bag = session / Path(str(capture.get("bag", ""))).name
    if not bag.is_file() or bag.suffix != ".db3":
        raise ValueError("native source DB3 is missing")
    source_files = {
        "raw_poses": run / "vio_raw.csv", "run_acceptance": run / "run_acceptance.json",
        "config": run / "vins_auto_loop_config.yaml", "camera": run / "left.yaml",
        "images": bag, "capture_acceptance": capture_path, "frame_index": frame_index,
        "capture_manifest": manifest_path,
    }
    hashes = {key: sha256(path) for key, path in source_files.items()}
    recorded = acceptance.get("provenance", {}).get("files", {})
    for key, report_key in (("config", "run_config"), ("camera", "left_calibration"),
                            ("capture_acceptance", "capture_acceptance"), ("frame_index", "camera_timestamps")):
        if hashes[key] != recorded.get(report_key, {}).get("sha256"):
            raise ValueError(f"source calibration/capture hash mismatch: {key}")
    replay = manifest.get("replay_inputs", {})
    for key, manifest_key in (("images", bag.name), ("frame_index", "d405_frames.csv")):
        if hashes[key] != replay.get(manifest_key):
            raise ValueError(f"native source hash mismatch: {key}")
    stamps, frame_numbers = load_native_frame_stamps(frame_index)
    return source_files, hashes, {
        "layout": "candidate_ab_native_recording_v1",
        "source_session_id": session.name,
        "image_pair_stamps": stamps,
        "image_frame_numbers": frame_numbers,
        "source_image_semantics": "factory-rectified D405 infrared-left used by accepted raw VINS",
        "clock_scope": "D405 global_time shared by frame index and raw VINS; cross-device synchronization NOT established",
        "timestamp_policy": "map recorder-relative image rows to immutable D405 global_time frame index; <=1us numeric pairing; no td reapplication",
    }


def calibration(run: Path, config_name: str = "vins_auto_loop_config.yaml") -> tuple[np.ndarray, dict]:
    fs = cv2.FileStorage(str(run / config_name), cv2.FILE_STORAGE_READ)
    try:
        if not fs.isOpened():
            raise ValueError("cannot open VINS config")
        if any(fs.getNode(k).empty() for k in ("estimate_extrinsic", "estimate_td", "cam0_calib", "body_T_cam0")):
            raise ValueError("missing required VINS calibration field")
        if fs.getNode("estimate_extrinsic").real() != 0 or fs.getNode("estimate_td").real() != 0:
            raise ValueError("online-calibrated runs need final calibration, unsupported here")
        if fs.getNode("cam0_calib").string() != "left.yaml":
            raise ValueError("unsupported cam0 calibration path")
        body_from_camera = validate_transform(fs.getNode("body_T_cam0").mat())
    finally:
        fs.release()
    fs = cv2.FileStorage(str(run / "left.yaml"), cv2.FILE_STORAGE_READ)
    try:
        if any(fs.getNode(k).empty() for k in ("model_type", "image_width", "image_height", "projection_parameters", "distortion_parameters")):
            raise ValueError("missing required camera calibration field")
        for group, keys in (("projection_parameters", ("fx", "fy", "cx", "cy")),
                            ("distortion_parameters", ("k1", "k2", "p1", "p2"))):
            if any(fs.getNode(group).getNode(k).empty() for k in keys):
                raise ValueError("missing required camera calibration coefficient")
        if fs.getNode("model_type").string() != "PINHOLE":
            raise ValueError("only PINHOLE calibrated images supported")
        values = {k: fs.getNode("projection_parameters").getNode(k).real()
                  for k in ("fx", "fy", "cx", "cy")}
        values.update({k: fs.getNode("distortion_parameters").getNode(k).real()
                       for k in ("k1", "k2", "p1", "p2")})
        values.update({k: int(fs.getNode("image_" + k).real()) for k in ("width", "height")})
        if (not all(np.isfinite(v) for v in values.values())
                or min(values[k] for k in ("fx", "fy", "width", "height")) <= 0):
            raise ValueError("invalid camera calibration")
        # This audited path is factory-rectified IR; never drop a distortion model.
        if any(values[k] != 0 for k in ("k1", "k2", "p1", "p2")):
            raise ValueError("only rectified zero-distortion IR supported by this adapter")
    finally:
        fs.release()
    return body_from_camera, values


def pair_vins_sources(run: Path, session: Path, agent: str, serial: str) -> tuple[dict, dict, dict]:
    """Validate one shared-owner pair VIO result and its immutable ROS2 input."""
    run_report_path = run / "report.json"
    input_report_path = session / "input_report.json"
    run_report = json.loads(run_report_path.read_text())
    input_report = json.loads(input_report_path.read_text())
    if (run_report.get("schema") != "three-device-slam.offline-vins-replay.v1"
            or run_report.get("status") != "VIO_REPLAY_PASS"
            or run_report.get("agent") != agent
            or run_report.get("shared_world_status") != "NOT_RUN"
            or not run_report.get("checks")
            or not all(run_report["checks"].values())):
        raise ValueError("pair VIO run is not an accepted single map epoch")
    if input_report.get("source_mode") != "shared_world_motion":
        raise ValueError("pair VIO input is not shared-world motion")
    input_agent = input_report.get("agent", "ego")
    input_serial = input_report.get("serial") or input_report.get("device", {}).get("serial")
    if input_agent != agent or str(input_serial) != serial:
        raise ValueError("pair VIO agent/physical serial mismatch")
    if (Path(run_report["source_input_report"]["path"]).resolve() != input_report_path.resolve()
            or sha256(input_report_path) != run_report["source_input_report"]["sha256"]):
        raise ValueError("pair VIO input report binding mismatch")
    if sha256(run / "vins_config.yaml") != run_report["runtime_config_sha256"]:
        raise ValueError("pair VIO runtime config hash mismatch")
    for relative, expected in input_report["outputs"].items():
        if sha256(session / relative) != expected:
            raise ValueError(f"pair VIO input artifact hash mismatch: {relative}")
    source_files = {
        "raw_poses": run / "odometry_raw.csv",
        "run_acceptance": run_report_path,
        "config": run / "vins_config.yaml",
        "camera": run / "left.yaml",
        "images": session / "sensors/sensors.db3",
        "image_metadata": session / "sensors/metadata.yaml",
        "input_report": input_report_path,
    }
    hashes = {key: sha256(path) for key, path in source_files.items()}
    image_topic = input_report.get("topics", {}).get("image0", f"/{agent}_probe/cam0/image_raw")
    raw_source = Path(input_report["source"])
    return source_files, hashes, {
        "layout": "shared_owner_pair_vins_replay_v1",
        "source_session_id": raw_source.parent.name,
        "image_pair_stamps": None,
        "image_frame_numbers": None,
        "image_topic": image_topic,
        "source_image_semantics": "factory-rectified stereo infrared-left used by accepted pair VIO",
        "clock_scope": "shared-owner host-monotonic epoch common to Ego and right",
        "timestamp_policy": "preserve shared host-monotonic image/pose nanoseconds; <=1us exact pairing; no td reapplication",
        "pose_count": int(run_report["pose_count"]),
        "image_count": int(input_report["counts"]["ir_left"]),
        "config_name": "vins_config.yaml",
        "pose_schema": "pair_vins",
    }


def wrapper_config(body_from_camera: np.ndarray, camera: dict, fps: float) -> str:
    lines = ["%YAML:1.0", "# Offline candidate input; NOT calibrated shared-world acceptance.",
             'Camera.type: "PinHole"']
    lines += [f"Camera.{k}: {float(camera[k])!r}" for k in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2")]
    lines += [f"Camera.width: {camera['width']}", f"Camera.height: {camera['height']}",
              f"Camera.fps: {float(fps)!r}", "Camera.RGB: 0", "is_fisheye: 0",
              "odom_in_imu_frame: 1", "Tbc: !!opencv-matrix", "   rows: 4", "   cols: 4", "   dt: d",
              "   data: " + json.dumps(body_from_camera.reshape(-1).tolist()),
              "extractor.type: ORB", "extractor.nFeatures: 1000", "t_min: 0.15", "r_min: 0.15",
              "ORBextractor.nFeaturesPR: 1000", "ORBextractor.scaleFactor: 1.2",
              "ORBextractor.nLevels: 8", "ORBextractor.iniThFAST: 20", "ORBextractor.minThFAST: 7"]
    return "\n".join(lines) + "\n"


def export(run: Path, session: Path, output: Path, *, agent: str, map_id: str, serial: str) -> dict:
    from rosbags.rosbag1 import Reader, Writer
    from rosbags.typesys import Stores, get_typestore

    for name in (agent, map_id, serial):
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
            raise ValueError("agent/map/serial must be nonempty identifiers")
    run, session, output = run.resolve(), session.resolve(), output.resolve()
    root = ARTIFACT_ROOT.resolve()
    if not output.is_relative_to(root) or output == root or output.exists():
        raise ValueError("output must be a NEW directory under repository artifacts")
    pair_layout = (run / "report.json").is_file() and (session / "input_report.json").is_file()
    if pair_layout:
        source_files, hashes, source_meta = pair_vins_sources(run, session, agent, serial)
        acceptance = json.loads((run / "report.json").read_text())
    else:
        acceptance = json.loads((run / "run_acceptance.json").read_text())
        if acceptance.get("result") != "PASS" or acceptance.get("runtime_error") is not None:
            raise ValueError("source SLAM run is not PASS")
    if pair_layout:
        pass
    elif (session / "source_provenance.json").is_file():
        if Path(acceptance["session"]).resolve() != session:
            raise ValueError("SLAM run does not belong to requested image session")
        provenance = json.loads((session / "source_provenance.json").read_text())
        if provenance.get("schema") != "three-device-slam.rk3576-umi-slam-export.v1":
            raise ValueError("unsupported source export schema")
        manifest_path = Path(provenance["source_session"]) / "manifest.json"
        if sha256(manifest_path) != provenance["source_manifest_sha256"]:
            raise ValueError("source manifest hash mismatch")
        manifest = json.loads(manifest_path.read_text())
        if manifest["device"]["d405_sdk_serial"] != serial:
            raise ValueError("wrong physical device serial")
        source_files = {"raw_poses": run / "vio_raw.csv", "run_acceptance": run / "run_acceptance.json",
                        "config": run / "vins_auto_loop_config.yaml", "camera": run / "left.yaml",
                        "images": session / "umi_ir.db3", "provenance": session / "source_provenance.json",
                        "source_manifest": manifest_path}
        hashes = {k: sha256(p) for k, p in source_files.items()}
        for key, recorded in (("config", "run_config"), ("camera", "left_calibration")):
            if hashes[key] != acceptance["provenance"]["files"][recorded]["sha256"]:
                raise ValueError(f"source calibration hash mismatch: {key}")
        if hashes["images"] != provenance["files"]["umi_ir.db3"]["sha256"]:
            raise ValueError("source DB3 hash mismatch")
        source_meta = {
            "layout": "rk3576_slam_export_v1", "source_session_id": provenance["source_session_id"],
            "image_pair_stamps": None, "image_frame_numbers": None,
            "source_image_semantics": manifest["profile"]["infrared"]["semantics"],
            "clock_scope": "source device global_time only; cross-device synchronization NOT established",
            "timestamp_policy": "preserve original per-stream stamps; <=1us numeric pairing; no td reapplication",
        }
    else:
        source_files, hashes, source_meta = native_candidate_sources(run, session, acceptance, serial)
    stamps, poses = (
        load_pair_vins_poses(source_files["raw_poses"])
        if source_meta.get("pose_schema") == "pair_vins"
        else load_poses(source_files["raw_poses"])
    )
    expected_pose_count = source_meta.get("pose_count", acceptance.get("raw_odometry_samples"))
    if len(poses) != expected_pose_count:
        raise ValueError("raw pose count changed since source acceptance")
    body_from_camera, camera = calibration(run, source_meta.get("config_name", "vins_auto_loop_config.yaml"))
    ros2 = get_typestore(Stores.ROS2_HUMBLE)
    ros1 = get_typestore(Stores.ROS1_NOETIC)
    types = ros1.types
    world_frame, body_frame = f"{agent}/{map_id}/world", f"{agent}/body"
    camera_frame = f"{agent}/cam0"
    image_topic, odom_topic = f"/{agent}/cam0/image_raw", f"/{agent}/odometry_raw"

    def header(stamp: int, frame: str, seq: int):
        return types["std_msgs/msg/Header"](seq, types["builtin_interfaces/msg/Time"](*divmod(stamp, 10**9)), frame)

    # SQLite read-only: no journal, writes or external source changes.
    with sqlite3.connect(source_files["images"].as_uri() + "?mode=ro", uri=True) as db:
        source_image_topic = source_meta.get("image_topic", IMAGE_TOPIC)
        topics = db.execute("SELECT id,type,serialization_format FROM topics WHERE name=?", (source_image_topic,)).fetchall()
        if len(topics) != 1 or topics[0][1:] != ("sensor_msgs/msg/Image", "cdr"):
            raise ValueError("expected one CDR left IR image topic")
        rows = db.execute("SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY id", (topics[0][0],)).fetchall()
        source_image_stamps = [r[1] for r in rows]
        image_stamps = source_meta["image_pair_stamps"] or source_image_stamps
        image_frame_numbers = source_meta["image_frame_numbers"] or [None] * len(rows)
        if len(image_stamps) != len(rows):
            raise ValueError("native frame index/image count mismatch")
        indices = pair_stamps(stamps, image_stamps)
        expected_image_count = source_meta.get("image_count", acceptance.get("camera_frames"))
        if len(rows) != expected_image_count:
            raise ValueError("image count changed since source acceptance")
        fps = (len(rows) - 1) * 1e9 / (image_stamps[-1] - image_stamps[0])
        output.mkdir(parents=True, exist_ok=False)
        print(f"EXPORT: {len(indices)} raw-pose/image pairs; no backend or hardware start", flush=True)
        audit = []
        with Writer(output / "frontend.bag") as bag:
            ci = bag.add_connection(image_topic, "sensor_msgs/msg/Image", typestore=ros1)
            co = bag.add_connection(odom_topic, "nav_msgs/msg/Odometry", typestore=ros1)
            for seq, (stamp, pose, index) in enumerate(zip(stamps, poses, indices)):
                row_id, source_image_stamp = rows[index]
                image_stamp = image_stamps[index]
                data = db.execute("SELECT data FROM messages WHERE id=?", (row_id,)).fetchone()[0]
                image = ros2.deserialize_cdr(data, "sensor_msgs/msg/Image")
                header_ns = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
                if header_ns != source_image_stamp:
                    raise ValueError("image storage/header clock mismatch")
                if (image.encoding not in ("mono8", "8UC1") or image.width != camera["width"]
                        or image.height != camera["height"] or image.step != image.width
                        or len(image.data) != image.step * image.height):
                    raise ValueError("image format/calibration dimensions mismatch")
                image1 = types["sensor_msgs/msg/Image"](header(image_stamp, camera_frame, seq), image.height,
                    image.width, "mono8", image.is_bigendian, image.step, image.data)
                q = Rotation.from_matrix(pose[:3, :3]).as_quat()
                geom_pose = types["geometry_msgs/msg/Pose"](types["geometry_msgs/msg/Point"](*pose[:3, 3]),
                    types["geometry_msgs/msg/Quaternion"](*q))
                # These covariances/velocities are unavailable, not precision claims.
                covariance = np.zeros(36)
                vector = types["geometry_msgs/msg/Vector3"]
                twist = types["geometry_msgs/msg/Twist"](vector(0., 0., 0.), vector(0., 0., 0.))
                odom = types["nav_msgs/msg/Odometry"](header(stamp, world_frame, seq), body_frame,
                    types["geometry_msgs/msg/PoseWithCovariance"](geom_pose, covariance),
                    types["geometry_msgs/msg/TwistWithCovariance"](twist, covariance))
                messages = [(image_stamp, ci, image1), (stamp, co, odom)]
                for ts, conn, message in sorted(messages, key=lambda v: v[0]):
                    bag.write(conn, ts, ros1.serialize_ros1(message, conn.msgtype))
                audit.append({"pose_timestamp_ns": stamp, "image_timestamp_ns": image_stamp,
                              "source_image_timestamp_ns": source_image_stamp,
                              "source_frame_number": image_frame_numbers[index], "source_encoding": image.encoding,
                              "db3_row_id": row_id, "pixel_sha256": hashlib.sha256(image.data).hexdigest(),
                              "T_local_world_camera": (pose @ body_from_camera).tolist()})
        (output / "pairs.jsonl").write_text("".join(json.dumps(x) + "\n" for x in audit))
    (output / "frontend.yaml").write_text(wrapper_config(body_from_camera, camera, fps))
    with Reader(output / "frontend.bag") as reader:
        count = sum(1 for _ in reader.messages())
        if count != 2 * len(poses):
            raise ValueError("ROS1 bag readback count mismatch")
    report = {
        "schema": "three-device-slam.covins-frontend-export.v1", "status": "INPUT_EXPORT_PASS",
        "shared_world_status": "NOT_RUN", "backend_executed": False,
        "agent": agent, "map_id": map_id, "serial": serial,
        "source_layout": source_meta["layout"],
        "source_session_id": source_meta["source_session_id"], "upstream_revision": UPSTREAM_REVISION,
        "implementation_sha256": sha256(Path(__file__).resolve()),
        "dependencies": {k: version(k) for k in ("numpy", "scipy", "rosbags")},
        "pose_convention": "T_local_world_body; metric metres; quaternion source wxyz / ROS xyzw",
        "camera_pose_convention": "T_local_world_camera = T_local_world_body @ body_T_cam0",
        "T_body_camera": body_from_camera.tolist(), "topics": {"image": image_topic, "odometry": odom_topic},
        "wrapper_remaps": {"/camera/image_raw": image_topic, "/cam_odom": odom_topic},
        "source_image_frames": len(rows), "exported_pairs": len(poses),
        "images_without_raw_pose": len(rows) - len(poses),
        "max_pair_delta_ns": max(abs(s - image_stamps[i]) for s, i in zip(stamps, indices)),
        "timestamp_policy": source_meta["timestamp_policy"],
        "clock_scope": source_meta["clock_scope"],
        "covariance_and_twist": "unknown; ROS zero placeholders, wrapper consumes pose only",
        "reset_policy": "single source run/map epoch; timestamps/steps gated; silent resets cannot be certified from CSV",
        "source_image_semantics": source_meta["source_image_semantics"],
        "sources": {k: {"path": str(p), "sha256": hashes[k]} for k, p in source_files.items()},
        "outputs": {p.name: sha256(p) for p in output.iterdir() if p.is_file()},
        "limitations": ["not a COVINS runtime test", "not a two-device overlap test",
                        "not spatial/temporal or TCP calibration acceptance", "no map merging or Ego-world transform estimated"],
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--agent", choices=("ego", "left", "right"), required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--serial", required=True)
    args = parser.parse_args()
    try:
        report = export(args.run, args.session, args.output, agent=args.agent, map_id=args.map_id, serial=args.serial)
    except (ValueError, OSError, KeyError, sqlite3.Error) as exc:
        print(f"BLOCKED: {exc}; any partial output retained, never reuse as PASS", flush=True)
        return 2
    print(json.dumps(report, indent=2))
    print("NEXT: validate second-agent VIO input and isolated COVINS-G runtime; shared world NOT_RUN.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
