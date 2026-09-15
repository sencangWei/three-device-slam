import json
import hashlib
from pathlib import Path

import numpy as np
import pytest

from three_device_slam.devices.d435i_ego.orbslam3_live import (
    FRAME_PACKET,
    INPUT_MAGIC,
    OrbSlam3LiveProcess,
    _IMU_WIRE,
    _INPUT_HEADER,
    _POSE_WIRE,
    _settings_without_atlas_save,
    validate_runtime,
)
from three_device_slam.spatial.d435i_live_vins import CombinedImu


def sample(timestamp_ns: int) -> CombinedImu:
    return CombinedImu(
        timestamp_ns,
        np.array((0.1, 0.2, 0.3)),
        np.array((1.0, 2.0, 3.0)),
    )


def test_protocol_struct_sizes_match_native_packing():
    assert _INPUT_HEADER.size == 36
    assert _IMU_WIRE.size == 56
    assert _POSE_WIRE.size == 92


def test_stereo_waits_for_imu_lead_then_emits_ordered_bundle(tmp_path):
    process = OrbSlam3LiveProcess(
        tmp_path / "output", image_shape=(2, 3), imu_lead_ns=5, queue_depth=4
    )
    image = np.arange(6, dtype=np.uint8).reshape(2, 3)
    process.push_imu(sample(90))
    process.push_stereo(100, image, image + 1)
    assert process.stereo_sent == 0
    process.push_imu(sample(100))
    assert process.stereo_sent == 0
    process.push_imu(sample(105))
    assert process.stereo_sent == 1

    header, imu, left, right = process._write_queue.get_nowait()
    values = _INPUT_HEADER.unpack(header)
    assert values == (INPUT_MAGIC, FRAME_PACKET, 100, 3, 2, 0, 6, 6)
    assert imu == b""
    assert left == image.tobytes()
    assert right == (image + 1).tobytes()

    process.push_stereo(200, image, image)
    process.push_imu(sample(205))
    header, imu, _left, _right = process._write_queue.get_nowait()
    values = _INPUT_HEADER.unpack(header)
    assert values[5] == 2
    assert len(imu) == 2 * _IMU_WIRE.size


def test_stereo_without_imu_fails_boundedly(tmp_path):
    process = OrbSlam3LiveProcess(
        tmp_path / "output", image_shape=(2, 3), queue_depth=16
    )
    image = np.zeros((2, 3), dtype=np.uint8)
    for timestamp in range(8):
        process.push_stereo(timestamp, image, image)
    with pytest.raises(RuntimeError, match="waiting_for_imu_overflow"):
        process.push_stereo(8, image, image)


def test_live_backpressure_drops_image_before_consuming_imu(tmp_path):
    process = OrbSlam3LiveProcess(
        tmp_path / "output",
        image_shape=(2, 3),
        imu_lead_ns=0,
        queue_depth=4,
        max_write_backlog=1,
    )
    image = np.zeros((2, 3), dtype=np.uint8)
    process.push_imu(sample(100))
    assert process.push_stereo(100, image, image)
    process.push_imu(sample(150))
    assert not process.push_stereo(150, image, image)
    process._write_queue.get_nowait()
    process.push_imu(sample(200))
    assert process.push_stereo(200, image, image)
    header, imu, _left, _right = process._write_queue.get_nowait()

    assert _INPUT_HEADER.unpack(header)[5] == 3
    assert len(imu) == 3 * _IMU_WIRE.size
    assert process.stereo_received == 3
    assert process.stereo_sent == 2
    assert process.stereo_dropped_backpressure == 1


def test_frame_with_one_imu_sample_is_dropped_without_consuming_interval(tmp_path):
    process = OrbSlam3LiveProcess(
        tmp_path / "output", image_shape=(2, 3), imu_lead_ns=0, queue_depth=4
    )
    image = np.zeros((2, 3), dtype=np.uint8)
    process.push_imu(sample(90))
    process.push_imu(sample(105))
    process.push_stereo(100, image, image)
    process.push_imu(sample(195))
    process.push_imu(sample(205))
    process.push_stereo(200, image, image)
    process.push_stereo(205, image, image)

    assert process.stereo_sent == 2
    assert process.stereo_dropped_insufficient_imu == 1
    process._write_queue.get_nowait()
    process._write_queue.get_nowait()

    process.push_imu(sample(300))
    process.push_stereo(300, image, image)
    header, imu, _left, _right = process._write_queue.get_nowait()
    assert _INPUT_HEADER.unpack(header)[5] == 2
    assert len(imu) == 2 * _IMU_WIRE.size
    assert process.stereo_sent == 3


def test_finish_input_releases_final_frame_without_future_imu(tmp_path):
    process = OrbSlam3LiveProcess(
        tmp_path / "output", image_shape=(2, 3), imu_lead_ns=5
    )
    image = np.zeros((2, 3), dtype=np.uint8)
    process.push_imu(sample(100))
    process.push_stereo(100, image, image)
    assert process.stereo_sent == 0
    process.finish_input()
    assert process.stereo_sent == 1


def test_runtime_hash_validation(tmp_path):
    runtime = tmp_path / "runtime"
    (runtime / "bin").mkdir(parents=True)
    (runtime / "share").mkdir()
    binary = runtime / "bin/orbslam3_live_adapter"
    library = runtime / "lib/libORB_SLAM3.so"
    vocabulary = runtime / "share/ORBvoc.txt"
    (runtime / "lib").mkdir()
    binary.write_bytes(b"binary")
    library.write_bytes(b"library")
    vocabulary.write_bytes(b"vocabulary")
    manifest = {
        "schema": "three-device-slam.orbslam3-live-runtime.v2",
        "protocol": "ORBI/ORBP packed little-endian v2",
        "pose_wire_bytes": 92,
        "orbslam3_commit": "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4",
        "patch": "patches/orbslam3-d435i-live-v5.patch",
        "patch_sha256": hashlib.sha256(
            (Path(__file__).parents[1] / "patches/orbslam3-d435i-live-v5.patch").read_bytes()
        ).hexdigest(),
        "adapter_source": "native/orbslam3_live_adapter/main.cc",
        "adapter_source_sha256": hashlib.sha256(
            (Path(__file__).parents[1] / "native/orbslam3_live_adapter/main.cc").read_bytes()
        ).hexdigest(),
        "files": {
            "bin/orbslam3_live_adapter": hashlib.sha256(b"binary").hexdigest(),
            "lib/libORB_SLAM3.so": hashlib.sha256(b"library").hexdigest(),
            "share/ORBvoc.txt": hashlib.sha256(b"vocabulary").hexdigest(),
        },
    }
    (runtime / "runtime_manifest.json").write_text(json.dumps(manifest))
    validate_runtime(runtime)
    binary.write_bytes(b"changed")
    with pytest.raises(ValueError, match="hash mismatch"):
        validate_runtime(runtime)


def test_runtime_rejects_old_pose_protocol(tmp_path):
    runtime = tmp_path / "runtime"
    (runtime / "bin").mkdir(parents=True)
    (runtime / "share").mkdir()
    (runtime / "runtime_manifest.json").write_text(
        json.dumps(
            {
                "schema": "three-device-slam.orbslam3-live-runtime.v1",
                "protocol": "ORBI/ORBP packed little-endian v1",
                "pose_wire_bytes": 84,
                "files": {},
            }
        )
    )
    with pytest.raises(ValueError, match="not a pinned"):
        validate_runtime(runtime)


def test_runtime_rejects_v2_manifest_missing_required_hashes(tmp_path):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    manifest = {
        "schema": "three-device-slam.orbslam3-live-runtime.v2",
        "protocol": "ORBI/ORBP packed little-endian v2",
        "pose_wire_bytes": 92,
        "orbslam3_commit": "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4",
        "patch": "patches/orbslam3-d435i-live-v5.patch",
        "patch_sha256": hashlib.sha256(
            (Path(__file__).parents[1] / "patches/orbslam3-d435i-live-v5.patch").read_bytes()
        ).hexdigest(),
        "adapter_source": "native/orbslam3_live_adapter/main.cc",
        "adapter_source_sha256": hashlib.sha256(
            (Path(__file__).parents[1] / "native/orbslam3_live_adapter/main.cc").read_bytes()
        ).hexdigest(),
        "files": {},
    }
    (runtime / "runtime_manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="omits a required"):
        validate_runtime(runtime)


def test_pose_wire_carries_ba2_completion_flag():
    payload = _POSE_WIRE.pack(
        b"ORBP", 123, 2, 7, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 12.5, 1.0
    )
    values = _POSE_WIRE.unpack(payload)
    assert values[1:4] == (123, 2, 7)
    assert bool(values[-1]) is True


def test_live_settings_disable_unsafe_atlas_serialization(tmp_path):
    settings = tmp_path / "settings.yaml"
    settings.write_text(
        '%YAML:1.0\nSystem.SaveAtlasToFile: "map_atlas"\nCamera.fps: 30.0\n'
    )
    rendered = _settings_without_atlas_save(settings)
    assert 'System.SaveAtlasToFile: ""' in rendered
    assert 'System.SaveAtlasToFile: "map_atlas"' not in rendered
    assert "Camera.fps: 30.0" in rendered


def test_pose_quaternion_conversion():
    rotation = OrbSlam3LiveProcess._quaternion_matrix(0.0, 0.0, 2**-0.5, 2**-0.5)
    assert np.allclose(rotation @ np.array((1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))
