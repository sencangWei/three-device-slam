"""Offline-readable gripper stream checks and camera timestamp association."""

from __future__ import annotations

import bisect
import csv
from pathlib import Path

import numpy as np


MAX_ENCODER_PAIR_GAP_US = 250
MAX_CAMERA_GRIPPER_DELTA_MS = 2.0


def analyze_gripper_csv(path: Path) -> dict:
    if not path.exists():
        return {"result": "FAIL", "error": "gripper_encoder.csv missing", "rows": 0}
    with path.open(newline="", encoding="utf-8") as fp:
        rows = list(csv.DictReader(fp))
    encoder_times = [float(row["encoder_ts_mono"]) for row in rows if row["encoder_ts_mono"]]
    pair_gaps = [int(row["sensor_pair_delta_us"]) for row in rows if row["sensor_pair_delta_us"]]
    valid_rows = sum(int(row["encoder_valid"]) for row in rows)
    regressions = sum(right <= left for left, right in zip(encoder_times, encoder_times[1:]))
    result = bool(rows) and valid_rows == len(rows) and len(encoder_times) == len(rows)
    result = result and regressions == 0 and bool(pair_gaps)
    result = result and min(pair_gaps) >= 0 and max(pair_gaps) <= MAX_ENCODER_PAIR_GAP_US
    return {
        "result": "PASS" if result else "FAIL",
        "path": str(path),
        "rows": len(rows),
        "valid_rows": valid_rows,
        "invalid_rows": len(rows) - valid_rows,
        "timestamp_regressions": regressions,
        "sensor_pair_delta_us": {
            "min": min(pair_gaps) if pair_gaps else None,
            "p50": float(np.percentile(pair_gaps, 50)) if pair_gaps else None,
            "p95": float(np.percentile(pair_gaps, 95)) if pair_gaps else None,
            "p99": float(np.percentile(pair_gaps, 99)) if pair_gaps else None,
            "max": max(pair_gaps) if pair_gaps else None,
        },
        "calibration_id": rows[0]["calibration_id"] if rows else None,
    }


def write_gripper_camera_alignment(
    frame_csv: Path,
    gripper_csv: Path,
    output_path: Path,
    *,
    camera_to_imu_td_s: float = 0.0,
) -> dict:
    """Join every RGB/depth frameset to its nearest valid encoder sample.

    VINS defines its IMU integration boundary as ``camera_ts + td``.  The
    encoder shares the IMU's MCU clock, so product capture must apply the same
    calibrated ``td`` before nearest-neighbour association.
    """
    if not frame_csv.exists() or not gripper_csv.exists():
        return {"result": "FAIL", "error": "timestamp input missing", "rows": 0}
    with gripper_csv.open(newline="", encoding="utf-8") as fp:
        gripper_rows = [
            row for row in csv.DictReader(fp)
            if row["encoder_valid"] == "1" and row["encoder_ts_mono"]
        ]
    encoder_times = [float(row["encoder_ts_mono"]) for row in gripper_rows]
    if not encoder_times:
        return {"result": "FAIL", "error": "no valid encoder rows", "rows": 0}
    output_fields = [
        "schema", "camera_set_index", "camera_frame_number", "camera_ts_mono",
        "encoder_query_ts_mono", "camera_imu_td_s", "encoder_ts_mono",
        "camera_encoder_delta_ms", "encoder_sequence",
        "imu_counter", "raw_count", "angle_deg", "direction", "closure_ratio",
        "estimated_no_load_gap_mm", "dual_closing_distance_mm",
        "single_jaw_travel_mm", "loaded_object_size_valid", "calibration_id",
    ]
    deltas = []
    count = 0
    with frame_csv.open(newline="", encoding="utf-8") as src, output_path.open(
        "w", newline="", encoding="utf-8"
    ) as dst:
        writer = csv.DictWriter(dst, fieldnames=output_fields)
        writer.writeheader()
        for camera in csv.DictReader(src):
            stream = "color" if camera.get("color_mono") else "depth"
            camera_ts = float(camera.get(f"{stream}_mono") or camera["arrival_mono"])
            encoder_query_ts = camera_ts + float(camera_to_imu_td_s)
            index = bisect.bisect_left(encoder_times, encoder_query_ts)
            candidates = [value for value in (index - 1, index) if 0 <= value < len(encoder_times)]
            nearest = min(
                candidates,
                key=lambda value: abs(encoder_times[value] - encoder_query_ts),
            )
            gripper = gripper_rows[nearest]
            delta_ms = (encoder_times[nearest] - encoder_query_ts) * 1000.0
            deltas.append(abs(delta_ms))
            writer.writerow({
                "schema": "umi_camera_gripper_alignment_v1",
                "camera_set_index": camera["set_index"],
                "camera_frame_number": camera.get(f"{stream}_frame_number", ""),
                "camera_ts_mono": f"{camera_ts:.9f}",
                "encoder_query_ts_mono": f"{encoder_query_ts:.9f}",
                "camera_imu_td_s": f"{float(camera_to_imu_td_s):.9f}",
                "encoder_ts_mono": gripper["encoder_ts_mono"],
                "camera_encoder_delta_ms": f"{delta_ms:.6f}",
                "encoder_sequence": gripper["sequence"],
                "imu_counter": gripper["imu_counter"],
                "raw_count": gripper["raw_count"],
                "angle_deg": gripper["angle_deg"],
                "direction": gripper["direction"],
                "closure_ratio": gripper["closure_ratio"],
                "estimated_no_load_gap_mm": gripper["estimated_no_load_gap_mm"],
                "dual_closing_distance_mm": gripper["dual_closing_distance_mm"],
                "single_jaw_travel_mm": gripper["single_jaw_travel_mm"],
                "loaded_object_size_valid": gripper["loaded_object_size_valid"],
                "calibration_id": gripper["calibration_id"],
            })
            count += 1
    maximum = max(deltas) if deltas else None
    passed = bool(deltas) and maximum <= MAX_CAMERA_GRIPPER_DELTA_MS
    return {
        "result": "PASS" if passed else "FAIL",
        "path": str(output_path),
        "rows": count,
        "camera_imu_td_s": float(camera_to_imu_td_s),
        "absolute_delta_ms": {
            "p50": float(np.percentile(deltas, 50)) if deltas else None,
            "p95": float(np.percentile(deltas, 95)) if deltas else None,
            "p99": float(np.percentile(deltas, 99)) if deltas else None,
            "max": maximum,
        },
    }
