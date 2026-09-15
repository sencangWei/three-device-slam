#!/usr/bin/env python3
"""Replay one exported D435i sequence through the pinned ORB-SLAM3 engine."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from three_device_slam.devices.d435i_ego.orbslam3_export import (  # noqa: E402
    ARTIFACT_ROOT,
    ORB_SLAM3_COMMIT,
)
from three_device_slam.spatial.covins_export import sha256  # noqa: E402


DEFAULT_RUNTIME = ARTIFACT_ROOT / "runtime/orbslam3-4452a3c4-d435i-init"
MIN_DYNAMIC_TRAJECTORY_COVERAGE = 0.85


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _validate_runtime(runtime: Path) -> tuple[dict, Path, Path]:
    manifest_path = runtime / "runtime_manifest.json"
    manifest = _load_json(manifest_path)
    if manifest.get("orbslam3_commit") != ORB_SLAM3_COMMIT:
        raise ValueError("runtime ORB-SLAM3 commit is not pinned")
    for relative, expected in manifest.get("files", {}).items():
        path = runtime / relative
        if not path.is_file() or sha256(path) != expected:
            raise ValueError(f"runtime hash mismatch: {relative}")
    binary = runtime / "bin/stereo_inertial_euroc"
    vocabulary = runtime / "share/ORBvoc.txt"
    return manifest, binary, vocabulary


def _validate_export(dataset: Path) -> dict:
    report = _load_json(dataset / "export_report.json")
    if report.get("status") not in {
        "PROVISIONAL_ORB_INPUT_READY",
        "DIAGNOSTIC_ORB_INPUT_READY_SOURCE_FAILED",
    }:
        raise ValueError("input is not a ready D435i ORB export")
    if report.get("engine", {}).get("commit") != ORB_SLAM3_COMMIT:
        raise ValueError("input and runtime ORB-SLAM3 commits differ")
    outputs = report["outputs"]
    paths = {
        "settings_sha256": dataset / "orbslam3_d435i.yaml",
        "times_sha256": dataset / "times.txt",
        "imu_sha256": dataset / "mav0/imu0/data.csv",
        "image_manifest_sha256": dataset / "image_manifest.jsonl",
    }
    for key, path in paths.items():
        if not path.is_file() or sha256(path) != outputs[key]:
            raise ValueError(f"export hash mismatch: {path.name}")
    times = (dataset / "times.txt").read_text(encoding="utf-8").splitlines()
    expected = int(report["counts"]["stereo_pairs"])
    if len(times) != expected:
        raise ValueError("camera timestamp count differs from export report")
    for camera in ("cam0", "cam1"):
        if sum(1 for _ in (dataset / f"mav0/{camera}/data").glob("*.png")) != expected:
            raise ValueError(f"{camera} PNG count differs from export report")
    return report


def classify_replay(
    *,
    returncode: int,
    source_mode: str,
    stdout: str,
    atlas_exists: bool,
    trajectory_exists: bool,
    frame_trajectory_rows: int = 0,
    frame_trajectory_duration_s: float = 0.0,
    input_stereo_pairs: int = 0,
    input_duration_s: float = 0.0,
) -> tuple[str, list[str]]:
    limitations: list[str] = []
    consumed = "Shutdown" in stdout
    zero_map = "There are 0 maps in the atlas" in stdout
    if source_mode == "bench_no_motion" and consumed and zero_map and atlas_exists:
        limitations.extend(
            [
                "source is a stationary transport capture and cannot initialize inertial SLAM",
                "official example aborts while requesting a trajectory from an empty Atlas",
                "this status proves bounded ingestion only, not pose or map quality",
            ]
        )
        return "DIAGNOSTIC_STATIC_INGEST_COMPLETE_NO_MAP", limitations
    bad_imu_resets = stdout.count("Reset map because local mapper set the bad imu flag")
    row_coverage = (
        frame_trajectory_rows / input_stereo_pairs if input_stereo_pairs else 0.0
    )
    duration_coverage = (
        frame_trajectory_duration_s / input_duration_s if input_duration_s else 0.0
    )
    if (
        returncode == 0
        and consumed
        and trajectory_exists
        and (
            not atlas_exists
            or bad_imu_resets > 0
            or row_coverage < MIN_DYNAMIC_TRAJECTORY_COVERAGE
            or duration_coverage < MIN_DYNAMIC_TRAJECTORY_COVERAGE
        )
    ):
        limitations.extend(
            [
                "inertial map did not remain continuous after the allowed initialization prefix",
                "a final short trajectory after map resets is not a successful replay",
            ]
        )
        return "REPLAY_FAILED_INERTIAL_INITIALIZATION", limitations
    if returncode == 0 and consumed and atlas_exists and trajectory_exists:
        limitations.append("trajectory exists but has not passed motion/world acceptance")
        return "REPLAY_COMPLETE_UNASSESSED", limitations
    limitations.append("ORB-SLAM3 did not complete the expected replay contract")
    return "REPLAY_FAILED", limitations


def _trajectory_metrics(path: Path) -> dict:
    timestamps_ns: list[float] = []
    if path.is_file():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            fields = line.split()
            if fields:
                timestamps_ns.append(float(fields[0]))
    duration_s = (
        (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
        if len(timestamps_ns) > 1
        else 0.0
    )
    return {"rows": len(timestamps_ns), "duration_s": duration_s}


def run(dataset: Path, output: Path, runtime: Path, timeout_s: float) -> dict:
    dataset, output, runtime = dataset.resolve(), output.resolve(), runtime.resolve()
    artifact_root = ARTIFACT_ROOT.resolve()
    if output.exists() or not output.is_relative_to(artifact_root):
        raise ValueError("output must be NEW under repository artifacts")
    export_report = _validate_export(dataset)
    runtime_manifest, binary, vocabulary = _validate_runtime(runtime)
    output.mkdir(parents=True)
    stdout_path, stderr_path = output / "orbslam3.stdout.log", output / "orbslam3.stderr.log"
    command = [
        str(binary),
        str(vocabulary),
        str(dataset / "orbslam3_d435i.yaml"),
        str(dataset),
        str(dataset / "times.txt"),
        "d435i_ego",
    ]
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = str(runtime / "lib") + (
        ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else ""
    )
    started = time.monotonic()
    timed_out = False
    with stdout_path.open("w", encoding="utf-8") as stdout_file, stderr_path.open(
        "w", encoding="utf-8"
    ) as stderr_file:
        process = subprocess.Popen(
            command,
            cwd=output,
            env=env,
            stdout=stdout_file,
            stderr=stderr_file,
            text=True,
        )
        try:
            returncode = process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            process.terminate()
            try:
                returncode = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                returncode = process.wait()
    elapsed_s = time.monotonic() - started
    stdout = stdout_path.read_text(encoding="utf-8", errors="replace")
    atlas = output / "map_atlas.osa"
    trajectories = sorted(output.glob("*d435i_ego*.txt"))
    frame_trajectory = output / "f_d435i_ego.txt"
    frame_metrics = _trajectory_metrics(frame_trajectory)
    times = [
        int(value)
        for value in (dataset / "times.txt").read_text(encoding="utf-8").splitlines()
        if value.strip()
    ]
    input_duration_s = (times[-1] - times[0]) / 1e9 if len(times) > 1 else 0.0
    status, limitations = classify_replay(
        returncode=returncode,
        source_mode=str(export_report["source_mode"]),
        stdout=stdout,
        atlas_exists=atlas.is_file(),
        trajectory_exists=bool(trajectories),
        frame_trajectory_rows=frame_metrics["rows"],
        frame_trajectory_duration_s=frame_metrics["duration_s"],
        input_stereo_pairs=int(export_report["counts"]["stereo_pairs"]),
        input_duration_s=input_duration_s,
    )
    if timed_out:
        status = "REPLAY_FAILED"
        limitations.append("replay exceeded its bounded timeout")
    report = {
        "schema": "three-device-slam.d435i-orbslam3-replay.v1",
        "status": status,
        "dataset": str(dataset),
        "source_mode": export_report["source_mode"],
        "engine": {
            "name": "ORB-SLAM3",
            "commit": ORB_SLAM3_COMMIT,
            "runtime_manifest_sha256": sha256(runtime / "runtime_manifest.json"),
            "patch_sha256": runtime_manifest["patch_sha256"],
        },
        "command": command,
        "returncode": returncode,
        "timed_out": timed_out,
        "elapsed_s": elapsed_s,
        "input_counts": export_report["counts"],
        "evidence": {
            "shutdown_reached": "Shutdown" in stdout,
            "empty_atlas_reported": "There are 0 maps in the atlas" in stdout,
            "insufficient_acceleration_reports": stdout.count("not enough acceleration"),
            "bad_imu_map_resets": stdout.count(
                "Reset map because local mapper set the bad imu flag"
            ),
            "atlas_bytes": atlas.stat().st_size if atlas.is_file() else 0,
            "trajectory_files": [path.name for path in trajectories],
            "frame_trajectory": frame_metrics,
            "input_duration_s": input_duration_s,
            "frame_row_coverage": (
                frame_metrics["rows"] / int(export_report["counts"]["stereo_pairs"])
            ),
            "frame_duration_coverage": (
                frame_metrics["duration_s"] / input_duration_s
                if input_duration_s
                else 0.0
            ),
            "stdout_sha256": sha256(stdout_path),
            "stderr_sha256": sha256(stderr_path),
        },
        "limitations": limitations,
    }
    (output / "replay_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument("--timeout-s", type=float, default=180.0)
    args = parser.parse_args()
    report = run(args.dataset, args.output, args.runtime, args.timeout_s)
    print(json.dumps(report, indent=2))
    if report["status"].startswith("REPLAY_FAILED"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
