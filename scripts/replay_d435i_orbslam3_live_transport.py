#!/usr/bin/env python3
"""Exercise the live TrackStereo transport with an existing D435i export."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from three_device_slam.devices.d435i_ego.orbslam3_live import (  # noqa: E402
    DEFAULT_RUNTIME,
    OrbSlam3LiveProcess,
    sha256,
)
from three_device_slam.spatial.d435i_live_vins import CombinedImu  # noqa: E402


def load_imu(path: Path) -> list[CombinedImu]:
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        next(reader)
        for values in reader:
            stamp = int(values[0])
            gyro = np.asarray([float(value) for value in values[1:4]])
            accel = np.asarray([float(value) for value in values[4:7]])
            rows.append(CombinedImu(stamp, gyro, accel))
    return rows


def run(
    dataset: Path,
    output: Path,
    runtime: Path,
    rate: float,
    max_frames: int | None = None,
) -> dict:
    dataset, output, runtime = dataset.resolve(), output.resolve(), runtime.resolve()
    if output.exists():
        raise FileExistsError(output)
    if rate <= 0:
        raise ValueError("rate must be positive")
    export = json.loads((dataset / "export_report.json").read_text(encoding="utf-8"))
    timestamps = [
        int(line)
        for line in (dataset / "times.txt").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if max_frames is not None:
        if max_frames < 2:
            raise ValueError("max_frames must be at least 2")
        timestamps = timestamps[:max_frames]
    imu = load_imu(dataset / "mav0/imu0/data.csv")
    process = OrbSlam3LiveProcess(
        output,
        runtime=runtime,
        settings=dataset / "orbslam3_d435i.yaml",
    )
    process.start()
    started = time.monotonic()
    first_timestamp = timestamps[0]
    imu_index = 0
    try:
        for index, timestamp_ns in enumerate(timestamps):
            deadline = started + (timestamp_ns - first_timestamp) / 1e9 / rate
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            while imu_index < len(imu) and (
                process._last_imu_ns is None
                or process._last_imu_ns < timestamp_ns + process.imu_lead_ns
            ):
                process.push_imu(imu[imu_index])
                imu_index += 1
            left = cv2.imread(
                str(dataset / f"mav0/cam0/data/{timestamp_ns}.png"),
                cv2.IMREAD_GRAYSCALE,
            )
            right = cv2.imread(
                str(dataset / f"mav0/cam1/data/{timestamp_ns}.png"),
                cv2.IMREAD_GRAYSCALE,
            )
            if left is None or right is None:
                raise RuntimeError(f"missing stereo pair: {timestamp_ns}")
            process.push_stereo(timestamp_ns, left, right)
            process.check_alive()
            while process.poll_pose() is not None:
                pass
            if (index + 1) % 300 == 0:
                print(
                    f"LIVE-REPLAY: {index + 1}/{len(timestamps)} "
                    f"poses={process.poses_received} queue={process._write_queue.qsize()}",
                    flush=True,
                )
        deadline = time.monotonic() + 30.0
        while process.poses_received < process.stereo_sent and time.monotonic() < deadline:
            process.check_alive()
            while process.poll_pose() is not None:
                pass
            time.sleep(0.02)
        before_stop = process.stats()
    finally:
        process.stop()
    after_stop = process.stats()
    log_path = output / "orbslam3.log"
    log = log_path.read_text(encoding="utf-8", errors="replace")
    checks = {
        "source_ready": export.get("status") == "PROVISIONAL_ORB_INPUT_READY",
        "all_stereo_sent": before_stop["stereo_sent"] == len(timestamps),
        "all_poses_returned": before_stop["poses_received"] == len(timestamps),
        "no_transport_failure": before_stop["failure"] is None,
        "child_clean_exit": process._child is not None and process._child.returncode == 0,
        "no_bad_imu_reset": "Reset map because local mapper set the bad imu flag" not in log,
    }
    if max_frames is None:
        checks.update(
            {
                "viba1_completed": "end VIBA 1" in log,
                "viba2_completed": "end VIBA 2" in log,
                "ba2_ready_reported": before_stop["inertial_ba2_ready_poses"] > 0,
            }
        )
    report = {
        "schema": "three-device-slam.d435i-orbslam3-live-transport-replay.v1",
        "status": (
            (
                "LIVE_TRANSPORT_PREFIX_COMPLETE"
                if max_frames is not None
                else "LIVE_TRANSPORT_REPLAY_COMPLETE_UNASSESSED"
            )
            if all(checks.values())
            else "LIVE_TRANSPORT_REPLAY_FAILED"
        ),
        "dataset": str(dataset),
        "runtime": str(runtime),
        "rate": rate,
        "max_frames": max_frames,
        "checks": checks,
        "before_stop": before_stop,
        "after_stop": after_stop,
        "elapsed_s": time.monotonic() - started,
        "orb_log_sha256": sha256(log_path),
        "limitations": [
            "recorded-data transport replay is not live camera HIL acceptance",
            "local ORB pose is not yet aligned to the fixed AprilGrid world",
        ],
    }
    (output / "live_transport_replay_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--runtime", type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int)
    args = parser.parse_args()
    report = run(args.dataset, args.output, args.runtime, args.rate, args.max_frames)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] != "LIVE_TRANSPORT_REPLAY_FAILED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
