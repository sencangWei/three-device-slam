import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROVENANCE = ROOT / "provenance" / "source_files.json"
D405_MAIN = "a7a143df9a138ada481e6c234b03803ab0cae837"
MIGRATION = "de2b52f40b493d0f5ee63bbeca8d4afcb1a8cb78"
LOCAL_SNAPSHOT_SHA256 = "b4e8d32e5d879be73268439fe52c964df1c43a4f205a1dcf0f70dc9afb4c1d9a"

D405_TARGETS = {
    "three_device_slam/devices/d405_umi/camera/realsense_capture.py": (
        "d405_product_main",
        "ego_vio/camera/realsense_capture.py",
        D405_MAIN,
    ),
    "three_device_slam/devices/d405_umi/imu/calibration.py": (
        "d405_product_main",
        "ego_vio/imu/calibration.py",
        D405_MAIN,
    ),
    "three_device_slam/devices/d405_umi/gripper/manual_gripper.py": (
        "d405_product_main",
        "ego_vio/gripper/manual_gripper.py",
        D405_MAIN,
    ),
    "three_device_slam/devices/d405_umi/gripper/training_sync.py": (
        "d405_product_main",
        "ego_vio/gripper/training_sync.py",
        D405_MAIN,
    ),
    "three_device_slam/devices/d405_umi/timing.py": (
        "d405_product_main",
        "ego_vio/timing.py",
        D405_MAIN,
    ),
    "three_device_slam/devices/d405_umi/gripper/umi_manual_gripper_20260824.yaml": (
        "d405_product_main",
        "config/gripper/umi_manual_gripper_20260824.yaml",
        D405_MAIN,
    ),
    "three_device_slam/devices/d405_umi/imu/imu_reader.py": (
        "three_device_migration",
        "ego_vio/imu/imu_reader.py",
        MIGRATION,
    ),
    "three_device_slam/devices/d405_umi/imu/stream_protocol.py": (
        "three_device_migration",
        "ego_vio/imu/stream_protocol.py",
        MIGRATION,
    ),
    "three_device_slam/devices/d405_umi/recorder/recorder.py": (
        "three_device_migration",
        "ego_vio/recorder/recorder.py",
        MIGRATION,
    ),
    "three_device_slam/devices/d405_umi/worker.py": (
        "three_device_migration",
        "scripts/capture_d405_720p_rgb_stereo_ir.py",
        MIGRATION,
    ),
}


def test_frozen_sources_are_explicit():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    assert payload["schema"] == "three-device-slam.source-provenance.v1"
    assert payload["sources"]["d405_product_main"]["commit"] == D405_MAIN
    assert payload["sources"]["three_device_migration"]["commit"] == MIGRATION
    assert payload["files"]


def test_source_map_has_unique_targets():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    targets = [row["target"] for row in payload["files"]]
    assert len(targets) == 22
    assert len(targets) == len(set(targets))


def test_d405_source_map_matches_extracted_origins():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    source_map = {
        row["target"]: (row["source_name"], row["source_path"], row["source_commit"])
        for row in payload["files"]
    }
    for target, expected in D405_TARGETS.items():
        assert source_map[target] == expected


def test_source_extraction_reachability_is_recorded():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    assert payload["sources"]["d405_product_main"]["remote_commit_reachable_at_extraction"] is True
    migration = payload["sources"]["three_device_migration"]
    assert migration["remote_commit_reachable_at_extraction"] is False
    assert migration["local_snapshot_archive_sha256"] == LOCAL_SNAPSHOT_SHA256


def test_runtime_never_references_sibling_d405_repo():
    forbidden = ("D405-MAXIMU", ".worktrees/three-device-acquisition")
    for path in (ROOT / "three_device_slam").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert not any(value in text for value in forbidden), path
