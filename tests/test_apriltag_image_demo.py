import json
from pathlib import Path
import subprocess
import sys


def test_image_level_demo_runs_pixels_through_both_alignment_chains(tmp_path):
    output = tmp_path / "image-demo"
    script = r"""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

from three_device_slam.spatial.apriltag_image_demo import export_apriltag_image_demo

output = Path(sys.argv[1])
report = export_apriltag_image_demo(output, duration_s=6.0, fps=10)
manifest = json.loads((output / 'manifest.json').read_text())
for relative, evidence in manifest['files'].items():
    payload = (output / relative).read_bytes()
    assert evidence == {'bytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest()}
rows = [json.loads(line) for line in (output / 'derived/trajectories_ego_world.jsonl').read_text().splitlines()]
def matrix(payload):
    return np.asarray(payload['matrix_4x4_row_major'], dtype=float).reshape(4, 4)
for device_id in ('left', 'right'):
    chain = report['chains'][device_id]
    switch = chain['reacquisition_switch_timestamp_ns']
    before = max((row for row in rows if row['timestamp_ns'] < switch), key=lambda row: row['timestamp_ns'])
    after = min((row for row in rows if row['timestamp_ns'] >= switch), key=lambda row: row['timestamp_ns'])
    initial_anchor = matrix(chain['initial_T_world_odom'])
    reacquired_anchor = matrix(chain['reacquisition_T_world_odom'])
    before_local = matrix(before['poses'][device_id]['T_odom_device'])
    after_local = matrix(after['poses'][device_id]['T_odom_device'])
    assert np.allclose(matrix(before['poses'][device_id]['T_world_device']), initial_anchor @ before_local, atol=1e-12)
    assert np.allclose(matrix(after['poses'][device_id]['T_world_device']), reacquired_anchor @ after_local, atol=1e-12)
    assert not np.allclose(matrix(after['poses'][device_id]['T_world_device']), initial_anchor @ after_local, atol=1e-6)
print(json.dumps({'report': report, 'manifest': manifest}))
"""
    completed = subprocess.run(
        [sys.executable, "-c", script, str(output)],
        cwd=Path(__file__).resolve().parents[1],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(completed.stdout)
    report = result["report"]
    manifest = result["manifest"]

    assert report["verdict"] == "PASS/simulation_image_replay"
    assert report["hil_status"] == "NOT_RUN"
    assert report["detector_backend"] == "opencv_aruco_reference"
    assert report["timeline"] == {"duration_s": 6.0, "fps": 10, "frames": 60}
    assert report["image_pipeline"]["frames_processed"] == 60
    assert report["image_pipeline"]["false_id_detections"] == 0
    for device_id, tag_id in (("left", 0), ("right", 1)):
        chain = report["chains"][device_id]
        assert chain["tag_id"] == tag_id
        assert chain["initial_anchor_verdict"] == "PASS/replay"
        assert chain["initial_anchor_candidates"] >= 5
        assert chain["initial_anchor_inliers"] >= 5
        assert chain["reacquisition_anchor_verdict"] == "PASS/replay"
        assert chain["reacquisition_accepted"] is True
        assert chain["occluded_frames"] > 0
        assert chain["anchor_translation_error_m"] < 0.04
        assert chain["anchor_rotation_error_deg"] < 4.0

    expected_files = {
        "calibration/camera.json",
        "calibration/tag_mounts.json",
        "derived/image_detections.jsonl",
        "derived/trajectories_ego_world.jsonl",
        "exports/image_detection_overlay.mp4",
        "quality/alignment_report.json",
    }
    assert manifest["schema"] == "three-device-slam.apriltag-image-demo.v1"
    assert manifest["mode"] == "deterministic_image_replay"
    assert manifest["verdict"] == "PASS/simulation_image_replay"
    assert set(manifest["files"]) == expected_files


def test_image_demo_refuses_to_overwrite_existing_output(tmp_path):
    output = tmp_path / "exists"
    output.mkdir()
    script = r"""
from pathlib import Path
import sys
from three_device_slam.spatial.apriltag_image_demo import export_apriltag_image_demo
try:
    export_apriltag_image_demo(Path(sys.argv[1]), duration_s=1.0, fps=10)
except FileExistsError:
    raise SystemExit(0)
raise SystemExit(3)
"""
    completed = subprocess.run(
        [sys.executable, "-c", script, str(output)],
        cwd=Path(__file__).resolve().parents[1],
    )
    assert completed.returncode == 0
