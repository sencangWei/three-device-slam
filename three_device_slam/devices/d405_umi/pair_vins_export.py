"""Export a sealed shared-owner D405 role into a ROS 2 VINS input.

The source pair capture stays immutable.  Camera and external-IMU timestamps
remain on the common host-monotonic clock established by ``rsusb_pair``.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
import json
from pathlib import Path
import struct

import numpy as np
import yaml
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

from three_device_slam.quality.verify_pair_session import _verify_payload_crc
from three_device_slam.spatial.covins_export import sha256


ARTIFACT_ROOT = Path(__file__).resolve().parents[3] / "artifacts"
IMU_RECORD = struct.Struct("<dI7f")
G0 = 9.80665
DEG2RAD = np.pi / 180.0
CLOCK = "realsense_global_time_mapped_to_host_monotonic"
VINS_IMU_ROTATION = np.array(
    [
        [0.99980212, -0.01423891, -0.01389161],
        [-0.01423891, -0.02458715, -0.99959628],
        [0.01389161, 0.99959628, -0.02478503],
    ],
    dtype=np.float64,
)


def imu_to_vins_axes(row: tuple) -> tuple[np.ndarray, np.ndarray]:
    """Apply the frozen product replay axis convention after SI conversion."""
    gyro = VINS_IMU_ROTATION @ (np.asarray(row[2:5]) * DEG2RAD)
    accel = VINS_IMU_ROTATION @ (np.asarray(row[5:8]) * G0)
    return gyro, accel


def _atomic_text(path: Path, text: str) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text)
    temporary.replace(path)


def load_formal_imu(path: Path, start_ns: int, end_ns: int) -> list[tuple]:
    """Load complete packed records inside the pair capture's formal window."""
    payload = path.read_bytes()
    if len(payload) % IMU_RECORD.size:
        raise ValueError("external IMU payload ends with a partial record")
    rows = [IMU_RECORD.unpack_from(payload, offset)
            for offset in range(0, len(payload), IMU_RECORD.size)]
    stamps = [int(round(row[0] * 1e9)) for row in rows]
    if len(stamps) < 2 or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("external IMU timestamps are not strictly increasing")
    return [row for row, stamp in zip(rows, stamps) if start_ns <= stamp <= end_ns]


def validate_imu_index(path: Path, rows: list[tuple]) -> None:
    indexed = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    if len(indexed) != len(rows):
        raise ValueError("external IMU binary/index count mismatch")
    for index, (item, row) in enumerate(zip(indexed, rows)):
        try:
            counter = int(item["counter"])
            stamp_ns = int(round(float(item["ts_mono"]) * 1e9))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid external IMU index row {index}") from exc
        if counter != int(row[1]) or abs(stamp_ns - int(round(row[0] * 1e9))) > 1:
            raise ValueError(f"external IMU binary/index mismatch at row {index}")


def render_runtime_config(source: str, output: Path, namespace: str) -> str:
    replacements = {
        'imu_topic: "/imu0"': f'imu_topic: "/{namespace}/imu0"',
        'image0_topic: "/cam0/image_raw"': f'image0_topic: "/{namespace}/cam0/image_raw"',
        'image1_topic: "/cam1/image_raw"': f'image1_topic: "/{namespace}/cam1/image_raw"',
    }
    for old, new in replacements.items():
        if source.count(old) != 1:
            raise ValueError(f"active VINS config topic anchor changed: {old}")
        source = source.replace(old, new, 1)
    lines = source.splitlines()
    output_rows = [index for index, line in enumerate(lines)
                   if line.strip().startswith("output_path:")]
    if len(output_rows) != 1:
        raise ValueError("active VINS config output_path is ambiguous")
    lines[output_rows[0]] = f'output_path: "{output / "solver_output"}/"'
    return "\n".join(lines) + "\n"


def _read_camera_rows(source: Path, role: str, name: str, manifest: dict,
                      start_ns: int, end_ns: int) -> list[dict]:
    stream = f"{role}.{name}"
    entry = manifest["streams"][stream]
    for suffix, field in (("jsonl", "index_sha256"), ("bin", "payload_sha256")):
        path = source / f"{stream}.{suffix}"
        if sha256(path) != entry[field]:
            raise ValueError(f"source hash mismatch: {path.name}")
    _verify_payload_crc(source.parent, source.name, stream)
    rows = [json.loads(line) for line in
            (source / f"{stream}.jsonl").read_text().splitlines()]
    formal = []
    for row in rows:
        if (row.get("stream_id") != stream or row.get("valid") is not True
                or row.get("clock_domain") != CLOCK
                or row.get("metadata", {}).get("encoding") != "Y8"):
            raise ValueError(f"invalid identity/clock/encoding in {stream}")
        if start_ns <= int(row["acquisition_ns"]) < end_ns and not row["warmup"]:
            formal.append(row)
    return formal


def export(source: Path, output: Path, calibration: Path) -> dict:
    source, output, calibration = source.resolve(), output.resolve(), calibration.resolve()
    if output.exists() or output == ARTIFACT_ROOT.resolve() or not output.is_relative_to(ARTIFACT_ROOT.resolve()):
        raise ValueError("output must be NEW under repository artifacts")
    role = source.name
    if role not in {"left", "right"}:
        raise ValueError("D405 pair source directory must be named left or right")
    pair = json.loads((source.parent / "pair_acceptance.json").read_text())
    acceptance = json.loads((source / "acceptance.json").read_text())
    manifest = json.loads((source / "manifest.json").read_text())
    camera = json.loads((source / "calibration.json").read_text())
    release = yaml.safe_load((calibration / "manifest.yaml").read_text())
    if pair.get("schema") != "ego.rsusb_pair.acceptance.v2" or pair.get("status") != "PASS":
        raise ValueError("pair transport acceptance is not PASS")
    if pair.get("mode") not in {
        "bench_no_motion", "shared_world_motion", "device3_anchor_hil"
    }:
        raise ValueError("pair capture mode is unsupported")
    if acceptance.get("result") != "PASS" or acceptance.get("manifest") != manifest:
        raise ValueError("D405 transport acceptance/manifest mismatch")
    if acceptance.get("calibration_id") != release.get("release_id"):
        raise ValueError("D405 calibration ID does not match active release")
    serial = camera.get("device", {}).get("serial")
    if (camera.get("schema") != "umi.d405.factory_calibration.v1"
            or serial != str(release.get("d405_serial"))
            or pair.get("inventory", {}).get(serial) != "Intel RealSense D405"):
        raise ValueError("D405 serial/release/pair identity mismatch")
    for name, expected in release["files"].items():
        if sha256(calibration / name) != expected:
            raise ValueError(f"active calibration hash mismatch: {name}")
    start, end = int(pair["formal_start_ns"]), int(pair["formal_end_ns"])
    left = _read_camera_rows(source, role, "ir_left", manifest, start, end)
    right = _read_camera_rows(source, role, "ir_right", manifest, start, end)
    expected_pairs = acceptance["camera_streams"]["ir_left"]["samples"]
    if len(left) != len(right) or len(left) != expected_pairs:
        raise ValueError("formal D405 stereo count mismatch")
    if [(r["sequence"], r["acquisition_ns"]) for r in left] != [
            (r["sequence"], r["acquisition_ns"]) for r in right]:
        raise ValueError("formal D405 stereo is not exactly paired")
    imu_path = source / "external_imu/imu.bin"
    all_imu_rows = load_formal_imu(imu_path, -2**63, 2**63 - 1)
    validate_imu_index(source / "external_imu/imu_ts.csv", all_imu_rows)
    imu_rows = [row for row in all_imu_rows
                if start <= int(round(row[0] * 1e9)) <= end]
    if len(imu_rows) != acceptance["imu"]["formal_samples"]:
        raise ValueError("formal external IMU count mismatch")

    namespace = f"{role}_probe"
    output.mkdir(parents=True)
    (output / "solver_output").mkdir()
    for name in ("left.yaml", "right.yaml"):
        (output / name).write_bytes((calibration / name).read_bytes())
    config = render_runtime_config(
        (calibration / "vins_config.yaml").read_text(), output, namespace)
    (output / "vins_config.yaml").write_text(config)

    store = get_typestore(Stores.ROS2_HUMBLE)
    types = store.types
    image_topics = {
        "ir_left": f"/{namespace}/cam0/image_raw",
        "ir_right": f"/{namespace}/cam1/image_raw",
    }
    imu_topic = f"/{namespace}/imu0"
    def header(stamp_ns: int, frame: str):
        return types["std_msgs/msg/Header"](
            types["builtin_interfaces/msg/Time"](*divmod(stamp_ns, 10**9)), frame)
    with Writer(output / "sensors", version=8) as bag, ExitStack() as stack:
        conns = {
            name: bag.add_connection(topic, "sensor_msgs/msg/Image", typestore=store)
            for name, topic in image_topics.items()
        }
        conns["imu"] = bag.add_connection(imu_topic, "sensor_msgs/msg/Imu", typestore=store)
        files = {name: stack.enter_context((source / f"{role}.{name}.bin").open("rb"))
                 for name in image_topics}
        events = [(int(r["acquisition_ns"]), name, index)
                  for name, rows in (("ir_left", left), ("ir_right", right))
                  for index, r in enumerate(rows)]
        events += [(int(round(row[0] * 1e9)), "imu", index)
                   for index, row in enumerate(imu_rows)]
        unknown = np.zeros(9); unknown[0] = -1.
        vector = types["geometry_msgs/msg/Vector3"]
        for stamp, name, index in sorted(events):
            conn = conns[name]
            if name == "imu":
                row = imu_rows[index]
                gyro, accel = imu_to_vins_axes(row)
                msg = types["sensor_msgs/msg/Imu"](
                    header(stamp, f"{role}/imu_raw"),
                    types["geometry_msgs/msg/Quaternion"](0., 0., 0., 1.), unknown,
                    vector(*gyro), np.zeros(9), vector(*accel), np.zeros(9))
            else:
                row = left[index] if name == "ir_left" else right[index]
                width, height = int(row["metadata"]["width"]), int(row["metadata"]["height"])
                if row["size"] != width * height or (width, height) != (1280, 720):
                    raise ValueError("D405 formal image dimensions/size mismatch")
                files[name].seek(row["offset"])
                data = np.frombuffer(files[name].read(row["size"]), dtype=np.uint8)
                if data.size != row["size"]:
                    raise ValueError("D405 image payload truncated during export")
                msg = types["sensor_msgs/msg/Image"](
                    header(stamp, f"{role}/{name}"), height, width, "mono8", 0,
                    width, data)
            bag.write(conn, stamp, store.serialize_cdr(msg, conn.msgtype))

    sources = [source / name for name in
               ("calibration.json", "acceptance.json", "manifest.json")]
    sources += [source.parent / "pair_acceptance.json", imu_path,
                source / "external_imu/imu_ts.csv", calibration / "manifest.yaml"]
    sources += [calibration / name for name in release["files"]]
    outputs = {str(path.relative_to(output)): sha256(path)
               for path in output.rglob("*") if path.is_file()}
    result = {
        "schema": "umi.d405.pair-vins-export.v1",
        "status": "INPUT_READY",
        "agent": role,
        "serial": serial,
        "calibration_status": "PRODUCT_RELEASE_CALIBRATION",
        "calibration_id": release["release_id"],
        "camera_imu_td_s": float(release["camera_imu_td_s"]),
        "source": str(source),
        "source_mode": pair["mode"],
        "formal_start_ns": start,
        "formal_end_ns": end,
        "counts": {"ir_left": len(left), "ir_right": len(right)},
        "combined_imu_samples": len(imu_rows),
        "timestamp_policy": "unaltered shared-owner host-monotonic nanoseconds; calibrated td remains in VINS config",
        "imu_policy": "external STM32/KT-EX9 packed deg/s,g converted to SI and rotated by the frozen product VINS IMU-axis transform",
        "vins_imu_rotation": VINS_IMU_ROTATION.tolist(),
        "topics": {"image0": image_topics["ir_left"], "image1": image_topics["ir_right"], "imu": imu_topic},
        "source_stream_hashes": manifest["streams"],
        "sources": {str(path): sha256(path) for path in sources},
        "outputs": outputs,
        "shared_world_status": "NOT_RUN",
        "limitations": ["VINS input only", "no inter-agent place recognition or map merge executed"],
    }
    _atomic_text(output / "input_report.json", json.dumps(result, indent=2) + "\n")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = export(args.source, args.output, args.calibration)
    except (KeyError, OSError, ValueError) as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}))
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
