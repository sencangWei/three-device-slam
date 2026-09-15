"""Offline stereo scene-support gate; never a calibration or SLAM accuracy gate.

Uses the audited Ego VINS export, without starting ROS or hardware. Tests static
stereo support, not temporal motion/observability or cross-agent map overlap.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
from pathlib import Path

import cv2
import numpy as np

from three_device_slam.spatial.covins_export import sha256

THRESHOLDS = {"ratio": .75, "max_vertical_px": 2., "min_disparity_px": 2.,
              "min_unique_matches": 30, "min_grid_cells": 4, "min_vertical_fraction": .25,
              "min_sampled_pairs": 10, "min_supported_fraction": .8}


def aggregate_support(samples):
    fraction = sum(r["supported"] for r in samples) / len(samples) if samples else 0.
    return len(samples) >= THRESHOLDS["min_sampled_pairs"] and fraction >= THRESHOLDS["min_supported_fraction"], fraction


def support_statistics(left, right, width, height):
    left, right = np.asarray(left, dtype=float).reshape(-1, 2), np.asarray(right, dtype=float).reshape(-1, 2)
    if left.shape != right.shape or width <= 0 or height <= 0:
        raise ValueError("invalid stereo dimensions")
    valid = np.all(np.isfinite(left), axis=1) & np.all(np.isfinite(right), axis=1)
    valid &= np.all((left >= 0) & (left < [width, height]), axis=1)
    valid &= np.all((right >= 0) & (right < [width, height]), axis=1)
    valid &= np.abs(left[:, 1] - right[:, 1]) <= THRESHOLDS["max_vertical_px"]
    valid &= left[:, 0] - right[:, 0] > THRESHOLDS["min_disparity_px"]
    accepted = left[valid]
    if len(accepted):
        # SIFT may describe the same location at multiple orientations. Do not
        # count those as independent spatial support.
        _, unique = np.unique(np.round(accepted * 2), axis=0, return_index=True)
        accepted = accepted[unique]
    cells = len(np.unique(np.floor(accepted / [width / 4, height / 3]), axis=0)) if len(accepted) else 0
    span = float(np.ptp(accepted[:, 1]) / height) if len(accepted) else 0.
    checks = {"unique_matches": len(accepted) >= THRESHOLDS["min_unique_matches"],
              "grid_coverage": cells >= THRESHOLDS["min_grid_cells"],
              "vertical_coverage": span >= THRESHOLDS["min_vertical_fraction"]}
    return {"supported": all(checks.values()), "checks": checks, "mutual_ratio_matches": len(left),
            "epipolar_matches": int(valid.sum()), "unique_matches": len(accepted),
            "occupied_4x3_cells": cells, "vertical_fraction": span}


def evaluate_pair(left, right):
    if left.ndim != 2 or left.shape != right.shape or left.dtype != np.uint8 or right.dtype != np.uint8:
        raise ValueError("same-size mono8 stereo required")
    detector = cv2.SIFT_create(nfeatures=2000)
    kl, dl = detector.detectAndCompute(left, None)
    kr, dr = detector.detectAndCompute(right, None)
    if dl is None or dr is None or len(dl) < 2 or len(dr) < 2:
        return support_statistics([], [], left.shape[1], left.shape[0])
    matcher = cv2.BFMatcher()
    forward, reverse = matcher.knnMatch(dl, dr, k=2), matcher.knnMatch(dr, dl, k=2)
    accepted_reverse = {a.queryIdx: a.trainIdx for a, b in reverse if a.distance < THRESHOLDS["ratio"] * b.distance}
    matches = [a for a, b in forward if a.distance < THRESHOLDS["ratio"] * b.distance
               and accepted_reverse.get(a.trainIdx) == a.queryIdx]
    result = support_statistics([kl[m.queryIdx].pt for m in matches], [kr[m.trainIdx].pt for m in matches],
                                left.shape[1], left.shape[0])
    result["detected_features"] = [len(kl), len(kr)]
    return result


def assess_export(input_dir: Path, output: Path):
    input_dir, output = input_dir.resolve(), output.resolve()
    root = Path(__file__).resolve().parents[3] / "artifacts"
    if output.exists() or output == root or not output.is_relative_to(root):
        raise ValueError("output must be NEW under repository artifacts")
    report = json.loads((input_dir / "input_report.json").read_text())
    if report["status"] not in {"PROVISIONAL_INPUT_READY", "INPUT_READY"}:
        raise ValueError("audited stereo VINS export required")
    source = Path(report["source"])
    cal = json.loads((source / "calibration.json").read_text())
    if sha256(source / "calibration.json") != report["sources"][str(source / "calibration.json")]:
        raise ValueError("calibration hash mismatch")
    agent = report.get("agent", "ego")
    if cal.get("schema") == "ego.d435i.factory_calibration.v1":
        intrinsics = cal["intrinsics"]["ir_left"]
        right_intrinsics = cal["intrinsics"]["ir_right"]
        ex = cal["extrinsics"]["ir_left_to_ir_right"]
    elif cal.get("schema") == "umi.d405.factory_calibration.v1":
        intrinsics = cal["streams"]["infrared_left"]
        right_intrinsics = cal["streams"]["infrared_right"]
        ex = cal["left_to_right"]
    else:
        raise ValueError("unsupported factory stereo calibration schema")
    expected_serial = str(report.get("serial", "327122078613"))
    if cal["device"]["serial"] != expected_serial or intrinsics != right_intrinsics:
        raise ValueError("bound device with identical rectified stereo intrinsics required")
    if (any(intrinsics["coefficients"]) or not np.allclose(ex["rotation_row_major"], np.eye(3).ravel(), atol=1e-8)
            or not np.allclose(ex["translation_m"][1:], 0, atol=1e-8) or ex["translation_m"][0] >= 0):
        raise ValueError("rectified horizontal positive-disparity stereo required")
    rows = {}
    for name in ("ir_left", "ir_right"):
        stream = agent + "." + name
        for suffix, key in (("bin", "payload_sha256"), ("jsonl", "index_sha256")):
            if sha256(source / f"{stream}.{suffix}") != report["source_stream_hashes"][stream][key]:
                raise ValueError("source hash mismatch")
        rows[name] = [r for line in (source / f"{stream}.jsonl").read_text().splitlines()
                      if not (r := json.loads(line))["warmup"]
                      and report["formal_start_ns"] <= r["acquisition_ns"] < report["formal_end_ns"]]
    for name, rr in rows.items():
        if len(rr) != report["counts"][name] or any(not r["valid"] or r["metadata"]["encoding"] != "Y8" for r in rr):
            raise ValueError("invalid source count/encoding/validity")
    if [(r["sequence"], r["acquisition_ns"]) for r in rows["ir_left"]] != [(r["sequence"], r["acquisition_ns"]) for r in rows["ir_right"]]:
        raise ValueError("stereo pairing mismatch")
    cv2.setNumThreads(1)
    sampled = sorted(set(range(0, len(rows["ir_left"]), 30)) | {len(rows["ir_left"])-1})
    results = []
    with ExitStack() as stack:
        streams = {n: stack.enter_context((source / f"{agent}.{n}.bin").open("rb")) for n in rows}
        for idx in sampled:
            images = []
            for name in rows:
                r = rows[name][idx]
                streams[name].seek(r["offset"])
                images.append(np.frombuffer(streams[name].read(r["size"]), np.uint8).reshape(intrinsics["height"], intrinsics["width"]))
            result = evaluate_pair(*images)
            result.update(frame=idx, timestamp_ns=rows["ir_left"][idx]["acquisition_ns"])
            results.append(result)
    supported, fraction = aggregate_support(results)
    result = {"schema": "three-device-slam.stereo-scene-support.v1",
        "agent": agent,
        "status": "STEREO_SUPPORT_PASS" if supported else "INSUFFICIENT_STEREO_SUPPORT",
        "thresholds": THRESHOLDS, "sampled_pairs": len(results), "supported_fraction": fraction,
        "unique_matches_min_median_max": [int(min(r["unique_matches"] for r in results)), float(np.median([r["unique_matches"] for r in results])), int(max(r["unique_matches"] for r in results))],
        "samples": results, "opencv_version": cv2.__version__, "source": str(source),
        "input_report_sha256": sha256(input_dir / "input_report.json"), "implementation_sha256": sha256(Path(__file__)),
        "shared_world_status": "NOT_RUN", "limitations": ["candidate stereo matches, not ground truth", "engineering preflight, not calibrated precision", "does not validate dynamic VIO, IMU or cross-agent overlap"]}
    output.mkdir(parents=True)
    (output / "report.json").write_text(json.dumps(result, indent=2)+"\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = assess_export(args.input, args.output)
    print(json.dumps({k:v for k,v in result.items() if k != "samples"}, indent=2))
    raise SystemExit(0 if result["status"] == "STEREO_SUPPORT_PASS" else 2)
