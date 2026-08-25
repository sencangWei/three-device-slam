# Acquisition and Synchronization Clean Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Build an independently installable Ubuntu phase-one product that records the 2UQ2 Ego device and two D405/external-IMU UMI devices through one automatic warmup gate, seals immutable raw data, builds the 30 Hz common-time index, and emits evidence-backed acquisition/timing status without modifying or depending on D405-MAXIMU.

**Architecture:** Start from the empty three-device-slam repository and cleanly extract only the accepted acquisition path. General three-device primitives move into the three_device_slam package; D405 and 2UQ2 code live behind separate device adapters; a coordinator owns the joint state machine; offline indexing and verification never rewrite raw data. The root launcher reads a provisioned Ubuntu product configuration, automatically records after joint readiness, seals on Ctrl+C or duration, then runs phase-one verification.

**Tech Stack:** Ubuntu 22.04 LTS, ROS 2 Humble, Python 3.10, pytest, pyrealsense2/librealsense, OpenCV, NumPy, pyserial, GStreamer, C/Make, Bash.

## Global Constraints

- Execute in an isolated worktree created with the using-git-worktrees skill on branch feature/acquisition-sync-clean-extraction.
- D405 formal source is exactly GitHub main commit a7a143df9a138ada481e6c234b03803ab0cae837.
- Three-device migration source is exactly de2b52f40b493d0f5ee63bbeca8d4afcb1a8cb78.
- Never modify D405-MAXIMU, its branches, worktrees, or existing untracked files.
- Do not add D405-MAXIMU as a submodule, dependency, PYTHONPATH entry, symlink, relative import, or runtime filesystem reference.
- Preserve estimate_td=0, td=-0.009312 s, and imu_lead_guard_ms=-6.812 unless later hardware evidence authorizes a separate change.
- Product runtime is Ubuntu 22.04 / ROS 2 Humble / Linux x86-64. Windows-only success is not product acceptance.
- Do not commit the 2UQ2 vendor SDK before its redistribution license is confirmed. Build against /home/robot/vendor/2uq2/YLX_XU_API_2026721, commit only the owned bridge, and verify the installed library by hash.
- Raw session files are append-only while recording and immutable after sealing. Derived output retains source hashes.
- USB arrival time is diagnostic only. Synchronization uses verified acquisition time mapped to one PC monotonic domain.
- Joint readiness requires at least 5 s warmup, a continuous healthy 3 s window, and a 30 s maximum wait.
- Recording starts automatically after readiness. Ctrl+C or configured duration must seal safely.
- A phase-one PASS means acquisition/timing only. Overall remains BLOCKED until Ego SLAM, markerless tracking, fusion, and spatial truth pass later plans.
- Missing hardware is BLOCKED/device_hardware_unavailable. Skipped checks never count as PASS.
- Port or add tests first, observe failure, implement the minimum change, then run focused and cumulative tests.
- Commit after each task. Do not push or create a remote repository in this plan.

## Plan Boundary

The approved product is split into four independently testable plans:

1. This plan: repository bootstrap, acquisition, common time, one-click Ubuntu phase-one runtime, and HIL stop gate.
2. Later: 2UQ2 calibration/replay and offline Ego stereo-inertial SLAM.
3. Later: EVT truth capture and markerless left/right UMI 6D tracking.
4. Later: two hand factor graphs, spatial qualification, and full product release.

Do not create empty modules or fake trajectories for later plans. Record those stages as BLOCKED/phase_not_delivered.

## Locked File Structure

~~~text
pyproject.toml
run_three_device_slam.sh
provenance/source_files.json
three_device_slam/
  __init__.py
  config.py
  cli.py
  core/{model,barrier,joint_gate,session_writer}.py
  acquisition/coordinator.py
  devices/two_uq2/{capture,worker}.py
  devices/d405_umi/
    worker.py
    camera/realsense_capture.py
    imu/{calibration,imu_reader,stream_protocol}.py
    gripper/{manual_gripper,training_sync}.py
    recorder/recorder.py
  synchronization/{sync_index,build_index}.py
  quality/verify_session.py
native/2uq2_xu_bridge/{Makefile,bridge.c}
scripts/{install_ubuntu,verify_ubuntu_environment}.sh
tests/
~~~

## Frozen Source Map

- de2b52f: ego_vio/multidevice/model.py, barrier.py, joint_gate.py, session_writer.py, sync_index.py.
- de2b52f: ego_vio/camera/two_uq2_capture.py, scripts/capture_two_uq2.py, and native/2uq2_xu_bridge.
- a7a143d base: ego_vio/camera/realsense_capture.py, ego_vio/imu/calibration.py, ego_vio/gripper/manual_gripper.py, ego_vio/gripper/training_sync.py.
- de2b52f after proving the a7a143d base is unchanged for those paths: ego_vio/imu/imu_reader.py, ego_vio/imu/stream_protocol.py, ego_vio/recorder/recorder.py, scripts/capture_d405_720p_rgb_stereo_ir.py.
- de2b52f: scripts/capture_three_devices.py, scripts/build_three_device_index.py, scripts/verify_three_device_session.py.

---

### Task 1: Bootstrap package and provenance gate

**Files:**

- Create: pyproject.toml
- Create: three_device_slam/__init__.py
- Create: provenance/source_files.json
- Create: tests/test_source_provenance.py

**Interfaces:**

- Consumes: approved design and two frozen source commits.
- Produces: installable package and provenance schema three-device-slam.source-provenance.v1.

- [ ] **Step 1: Create isolated worktree**

Invoke using-git-worktrees from D:\semg.claude\three-device-slam. Create feature/acquisition-sync-clean-extraction from main.

~~~powershell
git status --short --branch
git rev-parse HEAD
~~~

Expected: clean feature branch whose history contains approved design commit 462e4d648c0704831f949e1e4c53a83c94ad65f5 and this implementation plan.

- [ ] **Step 2: Write failing provenance tests**

Create tests/test_source_provenance.py:

~~~python
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROVENANCE = ROOT / "provenance" / "source_files.json"
D405_MAIN = "a7a143df9a138ada481e6c234b03803ab0cae837"
MIGRATION = "de2b52f40b493d0f5ee63bbeca8d4afcb1a8cb78"


def test_frozen_sources_are_explicit():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    assert payload["schema"] == "three-device-slam.source-provenance.v1"
    assert payload["sources"]["d405_product_main"]["commit"] == D405_MAIN
    assert payload["sources"]["three_device_migration"]["commit"] == MIGRATION
    assert payload["files"]


def test_source_map_has_unique_targets():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    targets = [row["target"] for row in payload["files"]]
    assert len(targets) == len(set(targets))


def test_runtime_never_references_sibling_d405_repo():
    forbidden = ("D405-MAXIMU", ".worktrees/three-device-acquisition")
    for path in (ROOT / "three_device_slam").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert not any(value in text for value in forbidden), path
~~~

- [ ] **Step 3: Verify red state**

~~~powershell
python -m pytest tests/test_source_provenance.py -q
~~~

Expected: FAIL because provenance/source_files.json is missing.

- [ ] **Step 4: Add minimal metadata**

Create pyproject.toml:

~~~toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "three-device-slam"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["numpy>=1.21", "pyserial>=3.5"]

[project.optional-dependencies]
test = ["pytest>=7"]

[tool.setuptools.packages.find]
include = ["three_device_slam*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
~~~

Create three_device_slam/__init__.py:

~~~python
"""Three-device Ego and dual-UMI product pipeline."""

__version__ = "0.1.0"
~~~

Create provenance/source_files.json using this exact shape:

~~~json
{
  "schema": "three-device-slam.source-provenance.v1",
  "sources": {
    "d405_product_main": {
      "url": "https://github.com/sencangWei/D405-MAXIMU.git",
      "commit": "a7a143df9a138ada481e6c234b03803ab0cae837"
    },
    "three_device_migration": {
      "url": "https://github.com/sencangWei/D405-MAXIMU.git",
      "commit": "de2b52f40b493d0f5ee63bbeca8d4afcb1a8cb78"
    }
  },
  "files": [
    {
      "source_name": "three_device_migration",
      "source_path": "ego_vio/multidevice/model.py",
      "source_commit": "de2b52f40b493d0f5ee63bbeca8d4afcb1a8cb78",
      "target": "three_device_slam/core/model.py",
      "license_review": "repository_license"
    }
  ]
}
~~~

Repeat the file object for all pairs:

~~~text
ego_vio/multidevice/model.py -> three_device_slam/core/model.py
ego_vio/multidevice/barrier.py -> three_device_slam/core/barrier.py
ego_vio/multidevice/joint_gate.py -> three_device_slam/core/joint_gate.py
ego_vio/multidevice/session_writer.py -> three_device_slam/core/session_writer.py
ego_vio/multidevice/sync_index.py -> three_device_slam/synchronization/sync_index.py
ego_vio/camera/two_uq2_capture.py -> three_device_slam/devices/two_uq2/capture.py
scripts/capture_two_uq2.py -> three_device_slam/devices/two_uq2/worker.py
native/2uq2_xu_bridge/Makefile -> native/2uq2_xu_bridge/Makefile
native/2uq2_xu_bridge/bridge.c -> native/2uq2_xu_bridge/bridge.c
ego_vio/camera/realsense_capture.py -> three_device_slam/devices/d405_umi/camera/realsense_capture.py
ego_vio/imu/calibration.py -> three_device_slam/devices/d405_umi/imu/calibration.py
ego_vio/gripper/manual_gripper.py -> three_device_slam/devices/d405_umi/gripper/manual_gripper.py
ego_vio/gripper/training_sync.py -> three_device_slam/devices/d405_umi/gripper/training_sync.py
ego_vio/imu/imu_reader.py -> three_device_slam/devices/d405_umi/imu/imu_reader.py
ego_vio/imu/stream_protocol.py -> three_device_slam/devices/d405_umi/imu/stream_protocol.py
ego_vio/recorder/recorder.py -> three_device_slam/devices/d405_umi/recorder/recorder.py
scripts/capture_d405_720p_rgb_stereo_ir.py -> three_device_slam/devices/d405_umi/worker.py
scripts/capture_three_devices.py -> three_device_slam/acquisition/coordinator.py
scripts/build_three_device_index.py -> three_device_slam/synchronization/build_index.py
scripts/verify_three_device_session.py -> three_device_slam/quality/verify_session.py
~~~

Use d405_product_main for formal-base rows and three_device_migration for the rest. The vendor SDK is not a file row.

- [ ] **Step 5: Verify green state and commit**

~~~powershell
python -m pytest tests/test_source_provenance.py -q
git diff --check
git add pyproject.toml provenance/source_files.json three_device_slam/__init__.py tests/test_source_provenance.py
git commit -m "build: bootstrap independent three-device package"
~~~

Expected: 3 passed, diff check exit 0, one focused commit.

---

### Task 2: Port general session and time primitives

**Files:**

- Create: three_device_slam/core/__init__.py
- Create: three_device_slam/core/model.py
- Create: three_device_slam/core/barrier.py
- Create: three_device_slam/core/joint_gate.py
- Create: three_device_slam/core/session_writer.py
- Create: three_device_slam/synchronization/__init__.py
- Create: three_device_slam/synchronization/sync_index.py
- Create: tests/test_core_model.py
- Create: tests/test_barrier.py
- Create: tests/test_joint_gate.py
- Create: tests/test_session_writer.py
- Create: tests/test_sync_index.py

**Interfaces:**

- Produces SensorRecord, FrameStamp, DeviceHeartbeat, Triplet, BarrierDirectory, JointWarmupGate, AppendOnlySessionWriter, and build_triplets.
- BarrierDirectory additionally produces request_stop(at_ns, reason) and read_stop().

- [ ] **Step 1: Materialize immutable migration snapshot**

~~~powershell
$SourceRepo='D:\semg.claude\D405-MAXIMU\.worktrees\three-device-acquisition'
$SourceZip='D:\semg.claude\.tmp\three-device-source-de2b52f.zip'
$SourceTree='D:\semg.claude\.tmp\three-device-source-de2b52f'
git -C $SourceRepo archive --format=zip --output=$SourceZip de2b52f40b493d0f5ee63bbeca8d4afcb1a8cb78
Expand-Archive -LiteralPath $SourceZip -DestinationPath $SourceTree
git -C $SourceRepo status --short --branch
~~~

Expected: snapshot exists; source feature worktree remains clean.

- [ ] **Step 2: Port tests before implementation**

~~~powershell
Copy-Item "$SourceTree\tests\test_multidevice_model.py" tests\test_core_model.py
Copy-Item "$SourceTree\tests\test_multidevice_barrier.py" tests\test_barrier.py
Copy-Item "$SourceTree\tests\test_joint_gate.py" tests\test_joint_gate.py
Copy-Item "$SourceTree\tests\test_session_writer.py" tests\test_session_writer.py
Copy-Item "$SourceTree\tests\test_sync_index.py" tests\test_sync_index.py
~~~

Rewrite imports exactly:

~~~text
ego_vio.multidevice.model          -> three_device_slam.core.model
ego_vio.multidevice.barrier        -> three_device_slam.core.barrier
ego_vio.multidevice.joint_gate     -> three_device_slam.core.joint_gate
ego_vio.multidevice.session_writer -> three_device_slam.core.session_writer
ego_vio.multidevice.sync_index     -> three_device_slam.synchronization.sync_index
import ego_vio.multidevice         -> import three_device_slam.core
~~~

Run:

~~~powershell
python -m pytest tests/test_core_model.py tests/test_barrier.py tests/test_joint_gate.py tests/test_session_writer.py tests/test_sync_index.py -q
~~~

Expected: collection FAIL because target modules are missing.

- [ ] **Step 3: Copy implementations**

~~~powershell
New-Item -ItemType Directory -Path three_device_slam\core,three_device_slam\synchronization -Force | Out-Null
Copy-Item "$SourceTree\ego_vio\multidevice\model.py" three_device_slam\core\model.py
Copy-Item "$SourceTree\ego_vio\multidevice\barrier.py" three_device_slam\core\barrier.py
Copy-Item "$SourceTree\ego_vio\multidevice\joint_gate.py" three_device_slam\core\joint_gate.py
Copy-Item "$SourceTree\ego_vio\multidevice\session_writer.py" three_device_slam\core\session_writer.py
Copy-Item "$SourceTree\ego_vio\multidevice\sync_index.py" three_device_slam\synchronization\sync_index.py
~~~

Create three_device_slam/core/__init__.py:

~~~python
from .barrier import BarrierDirectory
from .joint_gate import GateState, JointWarmupGate
from .model import DeviceHeartbeat, FrameStamp, SensorRecord, Triplet
from .session_writer import AppendOnlySessionWriter

__all__ = [
    "AppendOnlySessionWriter", "BarrierDirectory", "DeviceHeartbeat",
    "FrameStamp", "GateState", "JointWarmupGate", "SensorRecord", "Triplet",
]
~~~

Create synchronization/__init__.py:

~~~python
from .sync_index import build_triplets

__all__ = ["build_triplets"]
~~~

- [ ] **Step 4: Add stop-protocol tests**

Append to tests/test_barrier.py:

~~~python
def test_stop_request_is_first_writer_latched(tmp_path):
    barrier = BarrierDirectory(tmp_path)
    barrier.request_stop(100, "operator_interrupt")
    barrier.request_stop(200, "later_reason")
    assert barrier.read_stop() == {"at_ns": 100, "reason": "operator_interrupt"}


def test_invalid_stop_request_writes_nothing(tmp_path):
    barrier = BarrierDirectory(tmp_path)
    with pytest.raises(ValueError):
        barrier.request_stop(-1, "operator_interrupt")
    with pytest.raises(ValueError):
        barrier.request_stop(1, "")
    assert barrier.read_stop() is None
~~~

Run and expect missing-method failure.

- [ ] **Step 5: Implement stop protocol using existing atomic writer**

Add to BarrierDirectory:

~~~python
def request_stop(self, at_ns: int, reason: str) -> None:
    if isinstance(at_ns, bool) or not isinstance(at_ns, int) or at_ns < 0:
        raise ValueError("at_ns must be a nonnegative integer")
    if not isinstance(reason, str) or not reason:
        raise ValueError("reason must be a nonempty string")
    claim = self.path / "stop.lock"
    try:
        descriptor = os.open(claim, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        return
    os.close(descriptor)
    self._write_atomic(self.path / "stop.json", {"at_ns": at_ns, "reason": reason})


def read_stop(self) -> dict | None:
    path = self.path / "stop.json"
    if not path.exists():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    return {"at_ns": int(payload["at_ns"]), "reason": str(payload["reason"])}
~~~

- [ ] **Step 6: Verify and commit**

~~~powershell
python -m pytest tests/test_core_model.py tests/test_barrier.py tests/test_joint_gate.py tests/test_session_writer.py tests/test_sync_index.py -q
python -m pytest tests/test_source_provenance.py -q
git add three_device_slam/core three_device_slam/synchronization tests
git commit -m "feat: add joint session and timing primitives"
~~~

Expected: selected tests PASS.

---

### Task 3: Port 2UQ2 adapter

**Files:**

- Create: three_device_slam/devices/__init__.py
- Create: three_device_slam/devices/two_uq2/__init__.py
- Create: three_device_slam/devices/two_uq2/capture.py
- Create: three_device_slam/devices/two_uq2/worker.py
- Create: native/2uq2_xu_bridge/Makefile
- Create: native/2uq2_xu_bridge/bridge.c
- Create: tests/test_two_uq2_capture.py

**Interfaces:**

- Consumes BarrierDirectory, DeviceHeartbeat, SensorRecord, and AppendOnlySessionWriter.
- Produces TwoUQ2Capture, raw video/XU evidence, heartbeat, and ego acceptance.json.

- [ ] **Step 1: Port test and verify red state**

~~~powershell
Copy-Item "$SourceTree\tests\test_two_uq2_capture.py" tests\test_two_uq2_capture.py
~~~

Rewrite ego_vio.camera.two_uq2_capture to three_device_slam.devices.two_uq2.capture, scripts.capture_two_uq2 to three_device_slam.devices.two_uq2.worker, and all multidevice imports to three_device_slam.core.

~~~powershell
python -m pytest tests/test_two_uq2_capture.py -q
~~~

Expected: collection FAIL for missing target module.

- [ ] **Step 2: Copy implementation and bridge**

~~~powershell
New-Item -ItemType Directory -Path three_device_slam\devices\two_uq2,native\2uq2_xu_bridge -Force | Out-Null
Copy-Item "$SourceTree\ego_vio\camera\two_uq2_capture.py" three_device_slam\devices\two_uq2\capture.py
Copy-Item "$SourceTree\scripts\capture_two_uq2.py" three_device_slam\devices\two_uq2\worker.py
Copy-Item "$SourceTree\native\2uq2_xu_bridge\Makefile" native\2uq2_xu_bridge\Makefile
Copy-Item "$SourceTree\native\2uq2_xu_bridge\bridge.c" native\2uq2_xu_bridge\bridge.c
~~~

Remove worker sys.path manipulation and apply the same import map. devices/__init__.py is docstring-only. Create devices/two_uq2/__init__.py:

~~~python
from .capture import TwoUQ2Capture, TwoUQ2Frame

__all__ = ["TwoUQ2Capture", "TwoUQ2Frame"]
~~~

This import must not open hardware.

- [ ] **Step 3: Add and implement graceful stop**

Add tests for this helper:

~~~python
def formal_window_complete(*, now_ns, start_ns, duration_s, stop_record):
    if stop_record is not None:
        return True
    if duration_s is None:
        return False
    return now_ns >= start_ns + int(duration_s * 1_000_000_000)
~~~

Test duration completion, stop-record completion, and indefinite no-stop. Replace the direct deadline check with this helper plus barrier.read_stop(). All graceful exits must finalize acceptance.

- [ ] **Step 4: Verify and commit**

~~~powershell
python -m pytest tests/test_two_uq2_capture.py tests/test_barrier.py -q
python -m py_compile three_device_slam/devices/two_uq2/capture.py three_device_slam/devices/two_uq2/worker.py
git add three_device_slam/devices native/2uq2_xu_bridge tests/test_two_uq2_capture.py
git commit -m "feat: add 2uq2 raw capture adapter"
~~~

Expected: selected tests PASS and modules compile.

---

### Task 4: Extract isolated D405 UMI adapter

**Files:**

- Create: three_device_slam/devices/d405_umi/worker.py
- Create: three_device_slam/devices/d405_umi/camera/realsense_capture.py
- Create: three_device_slam/devices/d405_umi/imu/calibration.py
- Create: three_device_slam/devices/d405_umi/imu/imu_reader.py
- Create: three_device_slam/devices/d405_umi/imu/stream_protocol.py
- Create: three_device_slam/devices/d405_umi/gripper/manual_gripper.py
- Create: three_device_slam/devices/d405_umi/gripper/training_sync.py
- Create: three_device_slam/devices/d405_umi/recorder/recorder.py
- Create required package init files.
- Create: tests/test_d405_capture_quality.py
- Create: tests/test_d405_imu_calibration.py
- Create: tests/test_d405_imu_parse.py
- Create: tests/test_d405_training_sync.py

**Interfaces:**

- Consumes serial, external IMU by-id path, left/right device_id, session, and barrier.
- Produces one isolated D405/IMU worker, raw camera/IMU/gripper files, heartbeat, and acceptance.

- [ ] **Step 1: Clone and pin disposable formal-main source**

~~~powershell
$MainTree='D:\semg.claude\.tmp\d405-main-a7a143d'
git clone --no-checkout https://github.com/sencangWei/D405-MAXIMU.git $MainTree
git -C $MainTree checkout --detach a7a143df9a138ada481e6c234b03803ab0cae837
git -C $MainTree rev-parse HEAD
~~~

Expected: exact a7a143df9a138ada481e6c234b03803ab0cae837.

- [ ] **Step 2: Prove accepted feature patches sit on unchanged formal files**

~~~powershell
$FeatureBase='685e4b0d70f73fb86810c2796b3190a1a7df4f2b'
git -C $MainTree fetch --depth 1 origin $FeatureBase
git -C $MainTree diff --exit-code $FeatureBase a7a143df9a138ada481e6c234b03803ab0cae837 -- ego_vio/imu/imu_reader.py ego_vio/imu/stream_protocol.py ego_vio/recorder/recorder.py scripts/capture_d405_720p_rgb_stereo_ir.py
~~~

Expected: exit 0. If not, stop and reconcile formal-main changes before copying.

- [ ] **Step 3: Port tests first**

~~~powershell
Copy-Item "$SourceTree\tests\test_capture_quality.py" tests\test_d405_capture_quality.py
Copy-Item "$SourceTree\tests\test_imu_parse.py" tests\test_d405_imu_parse.py
Copy-Item "$SourceTree\tests\test_training_gripper_sync.py" tests\test_d405_training_sync.py
Copy-Item "$MainTree\tests\test_imu_calibration.py" tests\test_d405_imu_calibration.py
~~~

Rewrite ego_vio camera, imu, gripper, and recorder imports under three_device_slam.devices.d405_umi. Rewrite multidevice imports to three_device_slam.core and the old capture script import to three_device_slam.devices.d405_umi.worker.

~~~powershell
python -m pytest tests/test_d405_capture_quality.py tests/test_d405_imu_calibration.py tests/test_d405_imu_parse.py tests/test_d405_training_sync.py -q
~~~

Expected: collection FAIL for missing adapter.

- [ ] **Step 4: Copy formal base and accepted feature result**

~~~powershell
New-Item -ItemType Directory -Path three_device_slam\devices\d405_umi\camera,three_device_slam\devices\d405_umi\imu,three_device_slam\devices\d405_umi\gripper,three_device_slam\devices\d405_umi\recorder -Force | Out-Null
Copy-Item "$MainTree\ego_vio\camera\realsense_capture.py" three_device_slam\devices\d405_umi\camera\realsense_capture.py
Copy-Item "$MainTree\ego_vio\imu\calibration.py" three_device_slam\devices\d405_umi\imu\calibration.py
Copy-Item "$MainTree\ego_vio\gripper\manual_gripper.py" three_device_slam\devices\d405_umi\gripper\manual_gripper.py
Copy-Item "$MainTree\ego_vio\gripper\training_sync.py" three_device_slam\devices\d405_umi\gripper\training_sync.py
Copy-Item "$SourceTree\ego_vio\imu\imu_reader.py" three_device_slam\devices\d405_umi\imu\imu_reader.py
Copy-Item "$SourceTree\ego_vio\imu\stream_protocol.py" three_device_slam\devices\d405_umi\imu\stream_protocol.py
Copy-Item "$SourceTree\ego_vio\recorder\recorder.py" three_device_slam\devices\d405_umi\recorder\recorder.py
Copy-Item "$SourceTree\scripts\capture_d405_720p_rgb_stereo_ir.py" three_device_slam\devices\d405_umi\worker.py
~~~

Create package init files:

~~~python
# camera/__init__.py
from .realsense_capture import CameraFrame, RealSenseCapture
__all__ = ["CameraFrame", "RealSenseCapture"]

# imu/__init__.py
from .calibration import IMUCalibration
from .imu_reader import ImuReader, ImuSample, find_frames, parse_frame, verify_checksum
__all__ = [
    "IMUCalibration", "ImuReader", "ImuSample",
    "find_frames", "parse_frame", "verify_checksum",
]

# gripper/__init__.py
from .manual_gripper import (
    DEFAULT_PROFILE,
    ManualGripperCalibration,
    ManualGripperState,
    ManualGripperTracker,
)
__all__ = [
    "DEFAULT_PROFILE", "ManualGripperCalibration",
    "ManualGripperState", "ManualGripperTracker",
]

# recorder/__init__.py
from .recorder import Recorder, UnitRecorder
__all__ = ["Recorder", "UnitRecorder"]
~~~

devices/d405_umi/__init__.py is docstring-only. Remove worker sys.path manipulation and rewrite imports.

- [ ] **Step 5: Add graceful-stop tests and implementation**

Add tests:

~~~python
def test_stop_record_finishes_joint_formal_window():
    assert formal_window_complete(
        now_ns=100,
        start_ns=0,
        duration_s=None,
        stop_record={"at_ns": 100, "reason": "operator_interrupt"},
    )


def test_indefinite_window_continues_without_stop():
    assert not formal_window_complete(
        now_ns=10_000_000_000,
        start_ns=0,
        duration_s=None,
        stop_record=None,
    )
~~~

Implement the identical helper contract used by the 2UQ2 worker and poll barrier.read_stop(). Flush UnitRecorder and write acceptance before return.

- [ ] **Step 6: Verify isolation and commit**

~~~powershell
python -m pytest tests/test_d405_capture_quality.py tests/test_d405_imu_calibration.py tests/test_d405_imu_parse.py tests/test_d405_training_sync.py -q
rg -n "D405-MAXIMU|ego_vio|three-device-acquisition" three_device_slam tests
git add three_device_slam/devices/d405_umi tests/test_d405_*
git commit -m "feat: extract isolated d405 umi capture adapter"
~~~

Expected: tests PASS; rg finds no runtime import or sibling path.

---

### Task 5: Port and harden joint coordinator

**Files:**

- Create: three_device_slam/acquisition/__init__.py
- Create: three_device_slam/acquisition/coordinator.py
- Create: tests/test_three_device_coordinator.py

**Interfaces:**

- Consumes worker commands, BarrierDirectory, JointWarmupGate, optional duration, and stop_requested callback.
- Produces coordinator.json, recording_start_ns, worker terminal results, phase acquisition status, CaptureResult, and run_product_capture(config, stop_requested).

- [ ] **Step 1: Port test and add red cases**

Copy de2b52f test_three_device_coordinator.py and rewrite imports to the new package. Add:

~~~python
def test_product_run_never_prompts_twice(tmp_path, monkeypatch):
    monkeypatch.setattr(
        coordinator,
        "_input",
        lambda _message: (_ for _ in ()).throw(AssertionError("unexpected prompt")),
    )
    _, _, _, _, report = run_fake_session(tmp_path, monkeypatch, auto_start=True)
    assert report["scheduled_start_ns"] is not None


def test_stop_callback_publishes_operator_stop(tmp_path, monkeypatch):
    session, _, processes, _, report = run_fake_session(
        tmp_path,
        monkeypatch,
        duration_s=1.0,
        stop_requested=lambda: True,
    )
    payload = json.loads((session / "barrier" / "stop.json").read_text())
    assert payload["reason"] == "operator_interrupt"
    assert all(process.finalized_naturally for process in processes.values())
    assert report["reason"] == "operator_stop"
~~~

Run and expect missing coordinator/stop support.

- [ ] **Step 2: Port implementation and module worker commands**

Copy scripts/capture_three_devices.py to acquisition/coordinator.py. Remove sys.path manipulation. Use three_device_slam.core imports. Execute workers using:

~~~python
[sys.executable, "-m", "three_device_slam.devices.d405_umi.worker", ...]
[sys.executable, "-m", "three_device_slam.devices.two_uq2.worker", ...]
~~~

Append --duration only when duration_s is not None.

- [ ] **Step 3: Implement automatic start and stop callback**

Use:

~~~python
def run_coordinator(
    session,
    worker_commands,
    duration_s,
    clock_ns=time.monotonic_ns,
    *,
    auto_start=True,
    stop_requested=lambda: False,
):
~~~

After RECORDING begins, the first true stop_requested call writes:

~~~python
barrier.request_stop(clock_ns(), "operator_interrupt")
~~~

Wait up to FINALIZATION_GRACE_NS. A worker ignoring stop becomes FAIL/worker_stop_timeout and is terminated.

- [ ] **Step 4: Add the public product-capture adapter**

Add:

~~~python
@dataclass(frozen=True)
class CaptureResult:
    session: Path
    report: dict


def run_product_capture(config, stop_requested) -> CaptureResult:
    session = _new_session(Path(config.output_root))
    commands = _build_worker_commands(config, session)
    report = run_coordinator(
        session,
        commands,
        config.duration_s,
        auto_start=True,
        stop_requested=stop_requested,
    )
    return CaptureResult(session=session, report=report)
~~~

Change _build_worker_commands to read config.left, config.right, and config.ego fields defined in Task 7. It remains callable in Task 5 tests with SimpleNamespace values matching that shape.

- [ ] **Step 5: Verify and commit**

~~~powershell
python -m pytest tests/test_three_device_coordinator.py tests/test_joint_gate.py tests/test_barrier.py -q
python -m py_compile three_device_slam/acquisition/coordinator.py
git add three_device_slam/acquisition tests/test_three_device_coordinator.py
git commit -m "feat: coordinate automatic three-device recording"
~~~

Expected: selected tests PASS.

---

### Task 6: Port immutable index and offline verifier

**Files:**

- Create: three_device_slam/synchronization/build_index.py
- Create: three_device_slam/quality/__init__.py
- Create: three_device_slam/quality/verify_session.py
- Create: tests/test_three_device_session_verifier.py

**Interfaces:**

- Consumes sealed coordinator and raw worker output.
- Produces sync/common_30hz.csv, sync/manifest.json, quality/acquisition_timing.json, quality/product_status.json.

- [ ] **Step 1: Port tests and verify red state**

Copy test_three_device_session_verifier.py. Rewrite scripts.build_three_device_index to three_device_slam.synchronization.build_index, scripts.verify_three_device_session to three_device_slam.quality.verify_session, and all multidevice imports.

~~~powershell
python -m pytest tests/test_three_device_session_verifier.py -q
~~~

Expected: collection FAIL for missing modules.

- [ ] **Step 2: Copy offline modules**

~~~powershell
New-Item -ItemType Directory -Path three_device_slam\quality -Force | Out-Null
Copy-Item "$SourceTree\scripts\build_three_device_index.py" three_device_slam\synchronization\build_index.py
Copy-Item "$SourceTree\scripts\verify_three_device_session.py" three_device_slam\quality\verify_session.py
~~~

Remove sys.path manipulation and rewrite imports. Change outputs to sync/common_30hz.csv, sync/manifest.json, and quality/acquisition_timing.json. Preserve atomic publication, exclusive lock, strict parsing, source rehash, and raw immutability.

- [ ] **Step 3: Add truthful overall-status test**

~~~python
def test_phase_one_pass_keeps_product_overall_blocked():
    assert verifier.compose_product_status("PASS") == {
        "acquisition_timing": "PASS",
        "ego_slam": "BLOCKED/phase_not_delivered",
        "markerless_tracking": "BLOCKED/phase_not_delivered",
        "fusion": "BLOCKED/phase_not_delivered",
        "spatial_accuracy": "BLOCKED/phase_not_delivered",
        "overall": "BLOCKED",
    }
~~~

- [ ] **Step 4: Implement status composition**

~~~python
def compose_product_status(acquisition_timing: str) -> dict[str, str]:
    if acquisition_timing not in {"PASS", "FAIL", "BLOCKED"}:
        raise ValueError("invalid acquisition_timing status")
    pending = "BLOCKED/phase_not_delivered"
    return {
        "acquisition_timing": acquisition_timing,
        "ego_slam": pending,
        "markerless_tracking": pending,
        "fusion": pending,
        "spatial_accuracy": pending,
        "overall": "BLOCKED" if acquisition_timing == "PASS" else acquisition_timing,
    }
~~~

Write the result atomically. Do not create trajectory files.

- [ ] **Step 5: Verify and commit**

~~~powershell
python -m pytest tests/test_sync_index.py tests/test_three_device_session_verifier.py -q
python -m pytest tests -q
git diff --check
git add three_device_slam/synchronization three_device_slam/quality tests/test_three_device_session_verifier.py
git commit -m "feat: verify immutable three-device timing sessions"
~~~

Expected: full software suite PASS.

---

### Task 7: Add provisioned Ubuntu config and one-click entry

**Files:**

- Create: three_device_slam/config.py
- Create: three_device_slam/cli.py
- Create: run_three_device_slam.sh
- Create: scripts/install_ubuntu.sh
- Create: scripts/verify_ubuntu_environment.sh
- Create: tests/test_product_config.py
- Create: tests/test_product_cli.py
- Create: tests/test_ubuntu_scripts.py

**Interfaces:**

- Consumes /etc/three-device-slam/product.json by default.
- Produces run_three_device_slam.sh product and automatic capture, seal, index, verify.

Configuration fields are schema; left/right serial, imu_path, calibration_id; ego video_device, xu_library, calibration_id; output_root; duration_s. duration_s is positive finite or null. Do not commit fake serials.

- [ ] **Step 1: Write failing config tests**

~~~python
import json

import pytest

from three_device_slam.config import ConfigError, load_product_config


def write_valid_config(
    tmp_path,
    *,
    duration_s=None,
    left_serial="left-serial",
    right_serial="right-serial",
):
    payload = {
        "schema": "three-device-slam.product-config.v1",
        "left": {
            "serial": left_serial,
            "imu_path": "/dev/serial/by-id/left-imu",
            "calibration_id": "left-cal-v1",
        },
        "right": {
            "serial": right_serial,
            "imu_path": "/dev/serial/by-id/right-imu",
            "calibration_id": "right-cal-v1",
        },
        "ego": {
            "video_device": "/dev/video0",
            "xu_library": "/opt/three-device-slam/lib/libtwo_uq2_xu.so",
            "calibration_id": "ego-cal-v1",
        },
        "output_root": str(tmp_path / "sessions"),
        "duration_s": duration_s,
    }
    path = tmp_path / "product.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_loads_distinct_devices(tmp_path):
    path = write_valid_config(tmp_path, duration_s=None)
    config = load_product_config(path)
    assert config.left.serial == "left-serial"
    assert config.duration_s is None


def test_rejects_duplicate_d405_identity(tmp_path):
    path = write_valid_config(
        tmp_path,
        left_serial="same",
        right_serial="same",
    )
    with pytest.raises(ConfigError, match="distinct"):
        load_product_config(path)
~~~

The fixture writes schema three-device-slam.product-config.v1 and absolute Linux device paths. Run and expect missing config module.

- [ ] **Step 2: Implement frozen typed config**

Create frozen ProductConfig, UmiConfig, EgoConfig dataclasses. load_product_config rejects missing fields, unknown schema, duplicates, non-absolute device paths, empty calibration IDs, and invalid duration. It never enumerates hardware or rewrites config.

- [ ] **Step 3: Write failing CLI ordering tests**

~~~python
from types import SimpleNamespace

from three_device_slam import cli


def fake_config(tmp_path):
    return SimpleNamespace(output_root=tmp_path / "sessions", duration_s=None)


def test_product_runs_capture_index_verify_in_order(tmp_path, monkeypatch):
    events = []
    monkeypatch.setattr(cli, "load_product_config", lambda _path: fake_config(tmp_path))
    capture = SimpleNamespace(
        session=tmp_path / "session",
        report={"status": "PASS"},
    )
    monkeypatch.setattr(cli, "run_capture", lambda _c, _s: events.append("capture") or capture)
    monkeypatch.setattr(cli, "build_index", lambda s: events.append("index") or {})
    monkeypatch.setattr(cli, "verify_session", lambda s: events.append("verify") or {"status": "PASS"})
    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == 3
    assert events == ["capture", "index", "verify"]


def test_missing_config_is_blocked(tmp_path, capsys):
    assert cli.main(["product", "--config", str(tmp_path / "missing.json")]) == 3
    assert "BLOCKED/device_config_missing" in capsys.readouterr().err
~~~

- [ ] **Step 4: Implement CLI signal and stage order**

Load config before opening devices. SIGINT/SIGTERM set a threading.Event. Pass event.is_set and auto_start=True to coordinator. If raw data seals, run index then verify. Print session and statuses. Return 0 only for future overall PASS, 2 for FAIL, 3 for BLOCKED.

Expose these exact names at module scope:

~~~python
run_capture = coordinator.run_product_capture
build_index = index_builder.build_index
verify_session = session_verifier.verify_session
~~~

main uses capture_result.session as the offline session and capture_result.report as acquisition status.

- [ ] **Step 5: Add launcher**

Create run_three_device_slam.sh:

~~~bash
#!/usr/bin/env bash
set -euo pipefail
exec python3 -m three_device_slam.cli "$@"
~~~

Test that it uses strict mode and exec and contains no D405 checkout source.

- [ ] **Step 6: Add Ubuntu install and read-only verifier**

Create scripts/install_ubuntu.sh:

~~~bash
#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "FAIL/root_required" >&2
  exit 2
fi

source /etc/os-release
if [[ ${ID} != ubuntu || ${VERSION_ID} != 22.04 ]]; then
  echo "FAIL/unsupported_ubuntu" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
if [[ ${ROS_DISTRO:-} != humble ]]; then
  echo "FAIL/ros_humble_missing" >&2
  exit 2
fi

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
vendor_sdk=/home/robot/vendor/2uq2/YLX_XU_API_2026721
test -f "$vendor_sdk/include/V4L2/extunit.h" || { echo "BLOCKED/2uq2_vendor_sdk_missing" >&2; exit 3; }
install -d -o robot -g robot /opt/three-device-slam
install -d -o robot -g robot /opt/three-device-slam/lib
install -d -o robot -g robot /etc/three-device-slam
install -d -o robot -g robot /var/lib/three-device-slam/sessions

if [[ ! -x /opt/three-device-slam/venv/bin/python ]]; then
  python3 -m venv --system-site-packages /opt/three-device-slam/venv
fi
/opt/three-device-slam/venv/bin/python -m pip install "$repo_root"
make -C "$repo_root/native/2uq2_xu_bridge" clean all VENDOR_SDK="$vendor_sdk"
install -m 0755 "$repo_root/native/2uq2_xu_bridge/build/libtwo_uq2_xu.so" /opt/three-device-slam/lib/
file /opt/three-device-slam/lib/libtwo_uq2_xu.so | rg -q 'ELF 64-bit.*x86-64'
nm -D --defined-only /opt/three-device-slam/lib/libtwo_uq2_xu.so | rg -q 'ylx_open'
nm -D --defined-only /opt/three-device-slam/lib/libtwo_uq2_xu.so | rg -q 'ylx_read_imu27'
nm -D --defined-only /opt/three-device-slam/lib/libtwo_uq2_xu.so | rg -q 'ylx_close'
~~~

It creates no product.json and cannot overwrite one.

Create scripts/verify_ubuntu_environment.sh:

~~~bash
#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
[[ ${ID} == ubuntu && ${VERSION_ID} == 22.04 ]] || { echo "FAIL/unsupported_ubuntu" >&2; exit 2; }
source /opt/ros/humble/setup.bash
[[ ${ROS_DISTRO:-} == humble ]] || { echo "FAIL/ros_humble_missing" >&2; exit 2; }
test -x /opt/three-device-slam/venv/bin/python || { echo "FAIL/venv_missing" >&2; exit 2; }
test -f /opt/three-device-slam/lib/libtwo_uq2_xu.so || { echo "FAIL/xu_bridge_missing" >&2; exit 2; }
test -d /var/lib/three-device-slam/sessions || { echo "FAIL/session_root_missing" >&2; exit 2; }
/opt/three-device-slam/venv/bin/python -c 'import three_device_slam'
file /opt/three-device-slam/lib/libtwo_uq2_xu.so | rg -q 'ELF 64-bit.*x86-64'
for symbol in ylx_open ylx_read_imu27 ylx_close; do
  nm -D --defined-only /opt/three-device-slam/lib/libtwo_uq2_xu.so | rg -q "$symbol"
done
echo "PASS/ubuntu_environment"
~~~

Create tests/test_ubuntu_scripts.py:

~~~python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_launcher_is_strict_exec_entry():
    text = (ROOT / "run_three_device_slam.sh").read_text()
    assert "set -euo pipefail" in text
    assert "exec python3 -m three_device_slam.cli" in text
    assert "D405-MAXIMU" not in text


def test_installer_does_not_create_or_overwrite_product_config():
    text = (ROOT / "scripts" / "install_ubuntu.sh").read_text()
    assert "product.json" not in text
    assert "--system-site-packages" in text
    assert 'pip install "$repo_root"' in text
~~~

verify_ubuntu_environment.sh checks the same invariants without writes and returns a specific nonzero reason.

- [ ] **Step 7: Verify and commit**

~~~powershell
python -m pytest tests/test_product_config.py tests/test_product_cli.py tests/test_ubuntu_scripts.py -q
python -m pytest tests -q
git diff --check
git add three_device_slam/config.py three_device_slam/cli.py run_three_device_slam.sh scripts tests
git commit -m "feat: add one-click ubuntu phase-one runtime"
~~~

Expected: full suite PASS.

---

### Task 8: Prove clean-room Ubuntu software operation

**Files:**

- Modify: PROJECT_LOG.md
- Create: docs/acceptance/phase1-software-acceptance.md

**Interfaces:**

- Consumes feature branch Git bundle and Ubuntu host 192.168.113.224.
- Produces reproducible software evidence without changing active D405 release.

- [ ] **Step 1: Fresh local verification**

~~~powershell
python -m pytest tests -q
python -m compileall -q three_device_slam
git diff --check
git status --short --branch
~~~

Expected: tests PASS, compile and diff checks exit 0.

- [ ] **Step 2: Transfer by Git bundle**

~~~powershell
git bundle create D:\semg.claude\.tmp\three-device-slam-phase1.bundle feature/acquisition-sync-clean-extraction
git bundle verify D:\semg.claude\.tmp\three-device-slam-phase1.bundle
~~~

Transfer to /home/robot/incoming without embedding credentials.

- [ ] **Step 3: Clone isolated Ubuntu checkout**

~~~bash
test "$(readlink -f /home/robot/ego_vio_humble)" = "/home/robot/releases/ego_vio_humble/product_v1_20260824"
git clone /home/robot/incoming/three-device-slam-phase1.bundle /home/robot/worktrees/three-device-slam-phase1
cd /home/robot/worktrees/three-device-slam-phase1
git switch feature/acquisition-sync-clean-extraction
git status --short --branch
~~~

Expected: active D405 symlink unchanged; new checkout clean.

- [ ] **Step 4: Run Ubuntu software suite**

~~~bash
cd /home/robot/worktrees/three-device-slam-phase1
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest tests -q
python -m compileall -q three_device_slam
rg -n 'D405-MAXIMU|three-device-acquisition|/home/robot/ego_vio_humble' three_device_slam run_three_device_slam.sh scripts
~~~

Expected: tests PASS, compile exit 0, no runtime dependency hits.

- [ ] **Step 5: Build bridge without devices**

~~~bash
make -C native/2uq2_xu_bridge clean all VENDOR_SDK=/home/robot/vendor/2uq2/YLX_XU_API_2026721
file native/2uq2_xu_bridge/build/libtwo_uq2_xu.so
ldd native/2uq2_xu_bridge/build/libtwo_uq2_xu.so
nm -D --defined-only native/2uq2_xu_bridge/build/libtwo_uq2_xu.so | rg 'ylx_open|ylx_read_imu27|ylx_close'
~~~

Expected: x86-64 ELF and all three symbols.

- [ ] **Step 6: Verify safe missing-config BLOCKED**

~~~bash
set +e
./run_three_device_slam.sh product --config /tmp/three-device-slam-missing.json
status=$?
set -e
test "$status" -eq 3
~~~

Expected: no device opens; stderr includes BLOCKED/device_config_missing.

- [ ] **Step 7: Record evidence and commit**

Acceptance document records UTC, hostname, Ubuntu/ROS, commit, dirty state, commands, exact test count, bridge hash, active D405 symlink before/after, and hardware BLOCKED items.

~~~powershell
git add docs/acceptance/phase1-software-acceptance.md PROJECT_LOG.md
git commit -m "test: qualify ubuntu phase-one software"
~~~

---

### Task 9: Run real HIL stop gate when hardware is available

**Files:**

- Modify: PROJECT_LOG.md
- Create: docs/acceptance/phase1-hil-acceptance.md
- Create outside Git: /etc/three-device-slam/product.json

**Interfaces:**

- Consumes two D405 serials, two external IMU by-id paths, one 2UQ2 video node, installed bridge, and real calibration IDs.
- Produces one 60 s immutable session and phase-one PASS, FAIL, or BLOCKED.

- [ ] **Step 1: Enumerate real identities**

~~~bash
rs-enumerate-devices -s
find /dev/serial/by-id -maxdepth 1 -type l -printf '%f -> %l\n'
v4l2-ctl --list-devices
lsusb
~~~

Expected: exactly two intended D405s, two stable distinct IMUs, and intended 2UQ2. Ambiguity is BLOCKED/device_identity.

- [ ] **Step 2: Provision actual config**

Write only observed identities and real calibration IDs to /etc/three-device-slam/product.json. Set duration_s to 60.0.

~~~bash
/opt/three-device-slam/venv/bin/python -c 'from pathlib import Path; from three_device_slam.config import load_product_config; print(load_product_config(Path("/etc/three-device-slam/product.json")))'
~~~

Expected: typed ProductConfig with distinct devices.

- [ ] **Step 3: Run one-click qualification**

~~~bash
cd /home/robot/worktrees/three-device-slam-phase1
./run_three_device_slam.sh product
~~~

Expected: WARMING, JOINT_READY, automatic RECORDING, sealing after 60 s, indexing, verification, and no second prompt.

- [ ] **Step 4: Enforce timing acceptance**

PASS/acquisition_timing requires:

- verified common PC monotonic acquisition domain;
- 2UQ2 at least 59 Hz, valid stereo relation, monotonic 24-bit sequence;
- each D405 at least 29.5 Hz with no timestamp regression;
- each external IMU 399–401 Hz with continuous uint32 counter and no CRC/bad packet;
- minimum 5 s warmup and continuous healthy 3 s gate;
- no queue loss or storage failure;
- raw hashes unchanged across indexing;
- 30 Hz triplet target span at most 10 ms and hard span at most 16.7 ms;
- samples above hard span retained with trainable=false.

- [ ] **Step 5: Qualify Ctrl+C sealing**

Set duration_s to null, run again, wait for RECORDING, and press Ctrl+C once. Verify one operator_interrupt stop record, three worker acceptances, sealed raw manifest, offline verification, no remaining worker, and no forced second interrupt.

- [ ] **Step 6: Record verdict without crossing spatial boundary**

Document IDs, firmware, calibrations, commit, commands, raw hashes, rates, gaps, skew distributions, storage, and reasons. Even after phase-one PASS, overall remains BLOCKED/phase_not_delivered.

- [ ] **Step 7: Commit only reproducible evidence**

~~~bash
git add docs/acceptance/phase1-hil-acceptance.md PROJECT_LOG.md
git commit -m "test: qualify three-device acquisition hardware"
~~~

If hardware remains unavailable, do not fabricate PASS or a completed HIL file. Record BLOCKED/device_hardware_unavailable and stop.

---

## Spec Coverage Matrix

| Approved design area | Implemented or gated by |
|---|---|
| Independent ownership and formal-main provenance | Tasks 1 and 4 |
| Ubuntu-only product runtime and native bridge | Tasks 7 and 8 |
| D405 dual-instance isolation | Tasks 4, 5, and 9 |
| One-click automatic warmup and recording | Tasks 2, 5, 7, and 9 |
| Immutable session and raw-data contract | Tasks 2, 3, 4, and 6 |
| Common PC acquisition time and 30 Hz index | Tasks 2, 6, and 9 |
| Safe Ctrl+C sealing and recovery | Tasks 2, 3, 4, 5, 7, and 9 |
| PASS, FAIL, BLOCKED composition | Tasks 5, 6, 7, and 9 |
| Ego SLAM and gravity-aligned world origin | Explicitly gated to plan 2; BLOCKED/phase_not_delivered |
| Markerless UMI 6D | Explicitly gated to plan 3; BLOCKED/phase_not_delivered |
| Factor graphs and spatial truth | Explicitly gated to plan 4; BLOCKED/phase_not_delivered |
| No calibration board or production-visible tag | Preserved as a global constraint; no phase-one runtime dependency |

## Final Verification Checklist

- [ ] One focused commit per completed task.
- [ ] git status clean.
- [ ] python -m pytest tests -q passes in independent Ubuntu checkout.
- [ ] python -m compileall -q three_device_slam exits 0.
- [ ] git diff main...HEAD --check exits 0.
- [ ] Runtime files contain no D405-MAXIMU, three-device-acquisition, or sibling checkout path.
- [ ] Original D405 worktree statuses are unchanged.
- [ ] Active /home/robot/ego_vio_humble symlink is unchanged.
- [ ] Temporary source clone, archive, bundle, and transfer copy are removed only after absolute target verification.
- [ ] If hardware is unavailable, HIL is BLOCKED rather than complete.
- [ ] If HIL passes, acquisition/timing alone is PASS and overall remains BLOCKED until later plans.

Cleanup uses only these exact temporary paths after Resolve-Path confirms they remain under D:\semg.claude\.tmp:

~~~text
D:\semg.claude\.tmp\three-device-source-de2b52f.zip
D:\semg.claude\.tmp\three-device-source-de2b52f
D:\semg.claude\.tmp\d405-main-a7a143d
D:\semg.claude\.tmp\three-device-slam-phase1.bundle
~~~

On Ubuntu, delete only /home/robot/incoming/three-device-slam-phase1.bundle after the isolated checkout and bundle verification succeed.
