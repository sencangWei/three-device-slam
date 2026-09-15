import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest
import numpy as np

from three_device_slam.spatial.apriltag_demo import export_apriltag_demo
from three_device_slam.spatial.se3 import compose, invert, pose_error


def test_demo_export_writes_hash_bound_machine_readable_evidence(tmp_path):
    output = tmp_path / "demo"

    report = export_apriltag_demo(
        output,
        duration_s=1.0,
        fps=10,
        render_video=False,
    )

    assert report["verdict"] == "PASS/simulation"
    assert report["hil_status"] == "NOT_RUN"
    assert report["timeline"]["samples"] == 10
    assert report["video"]["status"] == "SKIPPED/test_or_operator_choice"
    expected = {
        "calibration/tag_mounts.json",
        "derived/tag_observations.jsonl",
        "derived/trajectories_ego_world.jsonl",
        "quality/alignment_report.json",
    }
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["schema"] == "three-device-slam.apriltag-demo.v1"
    assert manifest["mode"] == "deterministic_simulation"
    assert set(manifest["files"]) == expected
    for relative_path, evidence in manifest["files"].items():
        payload = (output / relative_path).read_bytes()
        assert evidence["bytes"] == len(payload)
        assert evidence["sha256"] == hashlib.sha256(payload).hexdigest()

    mounts = json.loads((output / "calibration/tag_mounts.json").read_text())
    assert mounts["scope"] == "simulation_only_not_installation_calibration"
    assert [(item["device_id"], item["tag_id"]) for item in mounts["mounts"]] == [
        ("left", 0),
        ("right", 1),
    ]
    trajectories = [
        json.loads(line)
        for line in (output / "derived/trajectories_ego_world.jsonl")
        .read_text()
        .splitlines()
    ]
    assert len(trajectories) == 10
    assert all(tuple(row["devices"]) == ("ego", "left", "right") for row in trajectories)


def test_full_timeline_contains_independent_occlusion_bridging_and_reacquisition(tmp_path):
    report = export_apriltag_demo(
        tmp_path / "demo",
        duration_s=30.0,
        fps=30,
        render_video=False,
    )

    assert report["timeline"]["samples"] == 900
    assert report["chains"]["left"]["tag_id"] == 0
    assert report["chains"]["right"]["tag_id"] == 1
    for device_id in ("left", "right"):
        chain = report["chains"][device_id]
        assert chain["anchor_translation_error_m"] < 1e-10
        assert chain["anchor_rotation_error_deg"] < 1e-8
        assert chain["initial_consensus_outliers"] == 1
        assert chain["vio_bridge_samples"] > 0
        assert chain["reacquired_after_occlusion"] is True
        assert chain["reacquisition_anchor_verdict"] == "PASS/replay"
        assert chain["reacquisition_anchor_accepted"] is True
        assert chain["reacquisition_candidates"] == 5
        assert chain["reacquisition_inliers"] == 4
        assert chain["reacquisition_consensus_outliers"] == 1
    assert report["chains"]["left"]["vio_bridge_samples"] != report["chains"]["right"]["vio_bridge_samples"]


def test_rejected_demo_rows_preserve_the_actual_flipped_pose_evidence(tmp_path):
    output = tmp_path / "demo"
    export_apriltag_demo(output, duration_s=30.0, fps=30, render_video=False)
    calibration = json.loads((output / "calibration/tag_mounts.json").read_text())
    mounts = {
        item["device_id"]: _matrix(item["T_tag_gripper"])
        for item in calibration["mounts"]
    }
    trajectories = {
        row["timestamp_ns"]: row
        for row in (
            json.loads(line)
            for line in (output / "derived/trajectories_ego_world.jsonl")
            .read_text()
            .splitlines()
        )
    }
    tag_rows = [
        json.loads(line)
        for line in (output / "derived/tag_observations.jsonl")
        .read_text()
        .splitlines()
    ]
    rejected = [
        row
        for row in tag_rows
        if row["quality"] == "REJECTED/anchor_consensus_outlier"
    ]

    assert len(rejected) == 4
    for row in rejected:
        trajectory = trajectories[row["timestamp_ns"]]["poses"]
        world_from_camera = _matrix(trajectory["ego"]["T_world_device"])
        world_from_gripper = _matrix(
            trajectory[row["device_id"]]["T_world_device"]
        )
        expected_clean = compose(
            invert(world_from_camera),
            world_from_gripper,
            invert(mounts[row["device_id"]]),
        )
        preserved_observation = _matrix(row["T_camera_tag"])
        error = pose_error(preserved_observation, expected_clean)
        assert error.translation_m > 0.1
        assert error.rotation_deg > 90.0


def test_demo_export_refuses_to_overwrite_an_existing_session(tmp_path):
    output = tmp_path / "demo"
    output.mkdir()

    with pytest.raises(FileExistsError):
        export_apriltag_demo(output, duration_s=1.0, fps=10, render_video=False)


def test_default_video_path_is_hash_bound_and_fully_decodable(tmp_path):
    output = tmp_path / "demo"
    script = r"""
import json
from pathlib import Path
import sys
import cv2
from three_device_slam.spatial.apriltag_demo import export_apriltag_demo

output = Path(sys.argv[1])
report = export_apriltag_demo(output, duration_s=0.6, fps=10, render_video=True)
capture = cv2.VideoCapture(str(output / 'exports/demo_overlay.mp4'))
assert capture.isOpened()
reported_frames = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
reported_fps = capture.get(cv2.CAP_PROP_FPS)
decoded = 0
while True:
    ok, _frame = capture.read()
    if not ok:
        break
    decoded += 1
capture.release()
print(json.dumps({
    'reported_frames': reported_frames,
    'reported_fps': reported_fps,
    'decoded': decoded,
    'video_status': report['video']['status'],
}))
"""
    completed = subprocess.run(
        [sys.executable, "-c", script, str(output)],
        cwd=Path(__file__).resolve().parents[1],
        check=True,
        capture_output=True,
        text=True,
    )
    child_result = json.loads(completed.stdout)

    video_path = output / "exports/demo_overlay.mp4"
    manifest = json.loads((output / "manifest.json").read_text())
    evidence = manifest["files"]["exports/demo_overlay.mp4"]
    payload = video_path.read_bytes()
    assert evidence == {
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }
    assert child_result == {
        "reported_frames": 6,
        "reported_fps": pytest.approx(10.0),
        "decoded": 6,
        "video_status": "PASS/simulation",
    }


def _matrix(payload: dict) -> np.ndarray:
    return np.asarray(payload["matrix_4x4_row_major"], dtype=np.float64).reshape(4, 4)
