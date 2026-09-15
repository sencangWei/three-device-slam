import json
import sqlite3

import cv2
import numpy as np
import pytest
import yaml
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

from three_device_slam.spatial import covins_export as ce


def test_pairing_numeric_roundoff_only():
    assert ce.pair_stamps([10_000_200, 20_000_300], [0, 10_000_000, 20_000_000]) == [1, 2]
    with pytest.raises(ValueError, match="mismatch"):
        ce.pair_stamps([15_000_000], [10_000_000, 20_000_000])
    with pytest.raises(ValueError, match="reuse"):
        ce.pair_stamps([10, 11], [10, 50_000])
    with pytest.raises(ValueError, match="increasing"):
        ce.pair_stamps([10], [10, 10])


@pytest.mark.parametrize("second", [
    "1,0,0,0,1,0,0,0",  # duplicate timestamp
    "0.9,0,0,0,1,0,0,0",  # regression/reset
    "2,0,0,0,0,0,0,0",  # zero quaternion
    "2,0,0,0,2,0,0,0",  # nonunit quaternion
    "2,nan,0,0,1,0,0,0",  # invalid position
    "2,1,0,0,1,0,0,0",  # unaccepted jump
])
def test_invalid_raw_pose(tmp_path, second):
    p = tmp_path / "raw.csv"
    p.write_text("t_sec,x,y,z,qw,qx,qy,qz\n1,0,0,0,1,0,0,0\n" + second + "\n")
    with pytest.raises(ValueError):
        ce.load_poses(p)


def test_pair_vins_pose_loader_preserves_integer_nanoseconds(tmp_path):
    p = tmp_path / "odometry_raw.csv"
    p.write_text(
        "timestamp_ns,x,y,z,qw,qx,qy,qz\n"
        "1000000001,0,0,0,1,0,0,0\n"
        "1033333334,0.01,0,0,1,0,0,0\n"
    )
    stamps, poses = ce.load_pair_vins_poses(p)
    assert stamps == [1_000_000_001, 1_033_333_334]
    np.testing.assert_allclose(poses[1][:3, 3], [0.01, 0, 0])


@pytest.fixture
def small_recording(tmp_path, monkeypatch):
    # Synthetic transport fixture only; never evidence of shared-world accuracy.
    # Production CLI has no output-root override. Isolate the root in unit tests
    # so normal pytest tmp_path works and failure tests reach the intended gate.
    monkeypatch.setattr(ce, "ARTIFACT_ROOT", tmp_path)
    run, session, source = [tmp_path / n for n in ("run", "session", "source")]
    for p in (run, session, source):
        p.mkdir()
    stamps = [1_000_000_000, 1_033_333_333, 1_066_666_666]
    ros2 = get_typestore(Stores.ROS2_HUMBLE)
    typ = ros2.types
    with sqlite3.connect(session / "umi_ir.db3") as db:
        db.execute("CREATE TABLE topics(id INTEGER, name TEXT, type TEXT, serialization_format TEXT)")
        db.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")
        db.execute("INSERT INTO topics VALUES(1, ?, 'sensor_msgs/msg/Image','cdr')", (ce.IMAGE_TOPIC,))
        for i, stamp in enumerate(stamps):
            msg = typ["sensor_msgs/msg/Image"](typ["std_msgs/msg/Header"](
                typ["builtin_interfaces/msg/Time"](*divmod(stamp, 10**9)), "cam0"),
                2, 3, "mono8", 0, 3, np.arange(6, dtype=np.uint8) + i)
            db.execute("INSERT INTO messages VALUES(?,1,?,?)",
                       (i + 1, stamp, ros2.serialize_cdr(msg, "sensor_msgs/msg/Image")))
    (run / "vio_raw.csv").write_text("t_sec,x,y,z,qw,qx,qy,qz\n"
                                    "1.033333333,0,0,0,0.7071067811865476,0,0,0.7071067811865476\n"
                                    "1.066666666,0.01,0,0,0.7071067811865476,0,0,0.7071067811865476\n")
    # An obviously different corrected stream must never be consumed.
    (run / "vio_corrected_stream.csv").write_text("THIS IS NOT RAW ODOMETRY")
    (run / "vins_auto_loop_config.yaml").write_text(
        '%YAML:1.0\nestimate_extrinsic: 0\nestimate_td: 0\ncam0_calib: "left.yaml"\n'
        'body_T_cam0: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n'
        '   data: [1.,0.,0.,0.1, 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.]\n')
    (run / "left.yaml").write_text(
        '%YAML:1.0\nmodel_type: PINHOLE\nimage_width: 3\nimage_height: 2\n'
        'projection_parameters: {fx: 2., fy: 2., cx: 1., cy: 1.}\n'
        'distortion_parameters: {k1: 0., k2: 0., p1: 0., p2: 0.}\n')
    (source / "manifest.json").write_text(json.dumps({"device": {"d405_sdk_serial": "testserial"},
        "profile": {"infrared": {"semantics": "synthetic_test_only"}}}))
    (session / "source_provenance.json").write_text(json.dumps({
        "schema": "three-device-slam.rk3576-umi-slam-export.v1", "source_session": str(source),
        "source_session_id": "test", "source_manifest_sha256": ce.sha256(source / "manifest.json"),
        "files": {"umi_ir.db3": {"sha256": ce.sha256(session / "umi_ir.db3")}}}))
    acceptance = {"result": "PASS", "runtime_error": None, "session": str(session),
                  "raw_odometry_samples": 2, "camera_frames": 3,
                  "provenance": {"files": {"run_config": {"sha256": ce.sha256(run / "vins_auto_loop_config.yaml")},
                                            "left_calibration": {"sha256": ce.sha256(run / "left.yaml")}}}}
    (run / "run_acceptance.json").write_text(json.dumps(acceptance))
    return run, session, tmp_path / "output"


def test_export_roundtrip_preserves_body_frame_and_pixels(small_recording):
    run, session, output = small_recording
    report = ce.export(run, session, output, agent="left", map_id="epoch0", serial="testserial")
    assert report["shared_world_status"] == "NOT_RUN" and not report["backend_executed"]
    assert report["exported_pairs"] == 2 and report["images_without_raw_pose"] == 1
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(output / "frontend.bag") as bag:
        messages = [(c.topic, t, store.deserialize_ros1(b, c.msgtype)) for c, t, b in bag.messages()]
    images = [m for topic, t, m in messages if topic.endswith("image_raw")]
    odoms = [m for topic, t, m in messages if topic.endswith("odometry_raw")]
    np.testing.assert_array_equal(images[0].data, np.arange(6, dtype=np.uint8) + 1)
    assert odoms[0].header.frame_id == "left/epoch0/world"
    assert odoms[0].child_frame_id == "left/body"
    assert odoms[0].pose.pose.position.x == 0
    pairs = [json.loads(line) for line in (output / "pairs.jsonl").read_text().splitlines()]
    # 90-degree world yaw rotates body->camera +0.1m X to world +0.1m Y.
    np.testing.assert_allclose(np.array(pairs[0]["T_local_world_camera"])[:3, 3], [0, .1, 0], atol=1e-12)
    config = cv2.FileStorage(str(output / "frontend.yaml"), cv2.FILE_STORAGE_READ)
    assert config.getNode("odom_in_imu_frame").real() == 1
    assert config.getNode("Tbc").mat()[0, 3] == .1
    config.release()
    with pytest.raises(ValueError, match="NEW"):
        ce.export(run, session, output, agent="left", map_id="epoch0", serial="testserial")


def test_native_candidate_export_maps_recorder_time_to_device_world_time(small_recording):
    run, session, output = small_recording
    (session / "source_provenance.json").unlink()
    frame_index = session / "d405_frames.csv"
    frame_index.write_text(
        "infrared_left_device_ms,infrared_left_frame_number,infrared_left_domain\n"
        "100000.000000,31,global_time\n"
        "101033.333333,32,global_time\n"
        "102066.666666,33,global_time\n")
    (run / "vio_raw.csv").write_text(
        "t_sec,x,y,z,qw,qx,qy,qz\n"
        "101.033333333,0,0,0,0.7071067811865476,0,0,0.7071067811865476\n"
        "102.066666666,0.01,0,0,0.7071067811865476,0,0,0.7071067811865476\n")
    capture = {"result": "PASS", "capture_error": None, "camera_serial": "testserial",
               "session": "/data/candidate_ab/recordings/session", "bag": "/data/session/umi_ir.db3"}
    (session / "acceptance.json").write_text(json.dumps(capture))
    manifest = {"d405_serial": "testserial", "replay_inputs": {
        "umi_ir.db3": ce.sha256(session / "umi_ir.db3"),
        "d405_frames.csv": ce.sha256(frame_index)}}
    (session / "candidate_ab_capture.yaml").write_text(yaml.safe_dump(manifest))
    acceptance_path = run / "run_acceptance.json"
    acceptance = json.loads(acceptance_path.read_text())
    acceptance["session"] = "/data/candidate_ab/recordings/session"
    acceptance["provenance"]["files"].update({
        "capture_acceptance": {"sha256": ce.sha256(session / "acceptance.json")},
        "camera_timestamps": {"sha256": ce.sha256(frame_index)}})
    acceptance_path.write_text(json.dumps(acceptance))

    report = ce.export(run, session, output, agent="right", map_id="epoch0", serial="testserial")
    assert report["source_layout"] == "candidate_ab_native_recording_v1"
    assert report["agent"] == "right" and report["max_pair_delta_ns"] == 0
    assert report["images_without_raw_pose"] == 1
    pairs = [json.loads(line) for line in (output / "pairs.jsonl").read_text().splitlines()]
    assert pairs[0]["source_image_timestamp_ns"] == 1_033_333_333
    assert pairs[0]["image_timestamp_ns"] == 101_033_333_333
    assert pairs[0]["source_frame_number"] == 32
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(output / "frontend.bag") as bag:
        images = [store.deserialize_ros1(raw, conn.msgtype) for conn, _, raw in bag.messages()
                  if conn.msgtype == "sensor_msgs/msg/Image"]
    assert images[0].header.stamp.sec == 101 and images[0].encoding == "mono8"


@pytest.mark.parametrize("fault", ["serial", "db_hash", "config_hash", "failed_run", "wrong_session"])
def test_source_contract_rejects_before_output(small_recording, fault):
    run, session, output = small_recording
    serial = "wrong" if fault == "serial" else "testserial"
    if fault == "db_hash":
        with (session / "umi_ir.db3").open("ab") as stream:
            stream.write(b"changed")
    if fault == "config_hash":
        with (run / "left.yaml").open("a") as stream:
            stream.write("\n# changed\n")
    if fault in ("failed_run", "wrong_session"):
        path = run / "run_acceptance.json"
        data = json.loads(path.read_text())
        data["result" if fault == "failed_run" else "session"] = "FAIL" if fault == "failed_run" else "/different"
        path.write_text(json.dumps(data))
    expected = {"serial": "wrong physical device", "db_hash": "DB3 hash mismatch",
                "config_hash": "calibration hash mismatch", "failed_run": "not PASS",
                "wrong_session": "does not belong"}[fault]
    with pytest.raises(ValueError, match=expected):
        ce.export(run, session, output, agent="left", map_id="epoch0", serial=serial)
    assert not output.exists()


def test_missing_calibration_is_not_silently_zero(small_recording):
    run, _, _ = small_recording
    path = run / "left.yaml"
    path.write_text(path.read_text().replace("p2: 0.", "unused: 0."))
    with pytest.raises(ValueError, match="missing"):
        ce.calibration(run)


def test_output_root_guard(small_recording):
    run, session, output = small_recording
    with pytest.raises(ValueError, match="NEW"):
        ce.export(run, session, output.parent.parent / "outside", agent="left", map_id="epoch0", serial="testserial")
