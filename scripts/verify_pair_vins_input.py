#!/usr/bin/env python3
"""Independently read back a D405 pair-capture VINS export."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from three_device_slam.devices.d405_umi.pair_vins_export import (
    DEG2RAD,
    G0,
    load_formal_imu,
)
from three_device_slam.spatial.covins_export import sha256


REFERENCE_VINS_IMU_ROTATION = np.array(
    [
        [0.99980212, -0.01423891, -0.01389161],
        [-0.01423891, -0.02458715, -0.99959628],
        [0.01389161, 0.99959628, -0.02478503],
    ],
    dtype=np.float64,
)


def verify(directory: Path) -> dict:
    directory = directory.resolve()
    report = json.loads((directory / "input_report.json").read_text())
    if report.get("schema") != "umi.d405.pair-vins-export.v1" or report.get("status") != "INPUT_READY":
        raise ValueError("D405 pair VINS export report required")
    if not np.allclose(
        np.asarray(report.get("vins_imu_rotation")),
        REFERENCE_VINS_IMU_ROTATION,
        rtol=0,
        atol=0,
    ):
        raise ValueError("VINS IMU-axis transform mismatch")
    for relative, expected in report["outputs"].items():
        if sha256(directory / relative) != expected:
            raise ValueError(f"output hash mismatch: {relative}")
    for source, expected in report["sources"].items():
        if sha256(Path(source)) != expected:
            raise ValueError(f"source hash mismatch: {source}")

    source, role = Path(report["source"]), report["agent"]
    start, end = report["formal_start_ns"], report["formal_end_ns"]
    camera_rows = {}
    camera_files = {}
    for name in ("ir_left", "ir_right"):
        stream = f"{role}.{name}"
        rows = [json.loads(line) for line in (source / f"{stream}.jsonl").read_text().splitlines()]
        camera_rows[name] = {int(row["acquisition_ns"]): row for row in rows
                             if start <= int(row["acquisition_ns"]) < end and not row["warmup"]}
        camera_files[name] = (source / f"{stream}.bin").open("rb")
    all_imu = load_formal_imu(source / "external_imu/imu.bin", -2**63, 2**63 - 1)
    imu_rows = {int(round(row[0] * 1e9)): row for row in all_imu
                if start <= int(round(row[0] * 1e9)) <= end}
    topics = {report["topics"]["image0"]: "ir_left",
              report["topics"]["image1"]: "ir_right",
              report["topics"]["imu"]: "imu"}
    store = get_typestore(Stores.ROS2_HUMBLE)
    counts = {name: 0 for name in ("ir_left", "ir_right", "imu")}
    max_gyro_error = max_accel_error = 0.0
    previous = {name: None for name in counts}
    try:
        with Reader(directory / "sensors") as reader:
            for connection, stamp, payload in reader.messages():
                name = topics.get(connection.topic)
                if name is None:
                    raise ValueError(f"unexpected bag topic: {connection.topic}")
                if previous[name] is not None and stamp <= previous[name]:
                    raise ValueError(f"nonmonotonic {name} bag timestamps")
                previous[name] = stamp
                msg = store.deserialize_cdr(payload, connection.msgtype)
                header_stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if header_stamp != stamp:
                    raise ValueError(f"{name} header/bag timestamp mismatch")
                if name == "imu":
                    row = imu_rows.get(stamp)
                    if row is None:
                        raise ValueError("exported IMU timestamp absent from source")
                    gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                    accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                    expected_gyro = REFERENCE_VINS_IMU_ROTATION @ (
                        np.asarray(row[2:5]) * DEG2RAD
                    )
                    expected_accel = REFERENCE_VINS_IMU_ROTATION @ (
                        np.asarray(row[5:8]) * G0
                    )
                    max_gyro_error = max(
                        max_gyro_error,
                        float(np.max(np.abs(gyro - expected_gyro))),
                    )
                    max_accel_error = max(
                        max_accel_error,
                        float(np.max(np.abs(accel - expected_accel))),
                    )
                else:
                    row = camera_rows[name].get(stamp)
                    if row is None:
                        raise ValueError(f"exported {name} timestamp absent from source")
                    camera_files[name].seek(row["offset"])
                    expected = camera_files[name].read(row["size"])
                    if (msg.encoding != "mono8" or msg.width != 1280 or msg.height != 720
                            or bytes(msg.data) != expected):
                        raise ValueError(f"{name} message is not pixel-exact mono8")
                counts[name] += 1
    finally:
        for stream in camera_files.values():
            stream.close()
    expected_counts = {"ir_left": report["counts"]["ir_left"],
                       "ir_right": report["counts"]["ir_right"],
                       "imu": report["combined_imu_samples"]}
    checks = {
        "counts_exact": counts == expected_counts,
        "images_pixel_exact": counts["ir_left"] == counts["ir_right"] == expected_counts["ir_left"],
        "imu_si_conversion_exact": max_gyro_error <= 1e-12 and max_accel_error <= 1e-12,
        "timestamps_strictly_increasing_per_topic": all(value is not None for value in previous.values()),
    }
    result = {
        "schema": "three-device-slam.d405-pair-vins-readback.v1",
        "status": "READBACK_PASS" if all(checks.values()) else "FAIL",
        "agent": role,
        "counts": counts,
        "max_gyro_conversion_error_rad_s": max_gyro_error,
        "max_accel_conversion_error_m_s2": max_accel_error,
        "checks": checks,
        "input_report_sha256": sha256(directory / "input_report.json"),
        "shared_world_status": "NOT_RUN",
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = verify(args.input)
    except (KeyError, OSError, ValueError) as exc:
        result = {"status": "FAIL", "reason": str(exc)}
    (args.input / "readback.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "READBACK_PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
