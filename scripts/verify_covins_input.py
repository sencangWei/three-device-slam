"""Read-only full decoded comparison of a covins_export artifact to its sources."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def verify(directory):
    report = json.loads((directory / "report.json").read_text())
    for entry in report["sources"].values():
        assert digest(entry["path"]) == entry["sha256"], entry["path"]
    for filename, expected in report["outputs"].items():
        assert digest(directory / filename) == expected, filename
    audit = [json.loads(line) for line in (directory / "pairs.jsonl").read_text().splitlines()]
    with open(report["sources"]["raw_poses"]["path"]) as stream:
        raw = list(csv.DictReader(stream))
    ros1, ros2 = get_typestore(Stores.ROS1_NOETIC), get_typestore(Stores.ROS2_HUMBLE)
    ni = no = 0
    source = Path(report["sources"]["images"]["path"])
    with sqlite3.connect(source.as_uri() + "?mode=ro", uri=True) as db, Reader(directory / "frontend.bag") as reader:
        for conn, timestamp, payload in reader.messages():
            msg = ros1.deserialize_ros1(payload, conn.msgtype)
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            assert timestamp == stamp
            if conn.msgtype == "sensor_msgs/msg/Image":
                item = audit[ni]
                data = db.execute("SELECT data FROM messages WHERE id=?", (item["db3_row_id"],)).fetchone()[0]
                original = ros2.deserialize_cdr(data, "sensor_msgs/msg/Image")
                original_stamp = original.header.stamp.sec * 10**9 + original.header.stamp.nanosec
                assert original_stamp == item.get("source_image_timestamp_ns", item["image_timestamp_ns"])
                np.testing.assert_array_equal(msg.data, original.data)
                assert hashlib.sha256(msg.data).hexdigest() == item["pixel_sha256"]
                assert stamp == item["image_timestamp_ns"]
                assert msg.encoding == "mono8"
                assert conn.topic == report["topics"]["image"]
                ni += 1
            else:
                assert conn.msgtype == "nav_msgs/msg/Odometry"
                item, row = audit[no], raw[no]
                assert stamp == item["pose_timestamp_ns"]
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                np.testing.assert_allclose([p.x, p.y, p.z], [float(row[k]) for k in ("x", "y", "z")], atol=1e-15)
                expected_q = [float(row[k]) for k in ("qx", "qy", "qz", "qw")]
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                np.testing.assert_allclose(rotation, Rotation.from_quat(expected_q).as_matrix(), atol=1e-14)
                body_pose = np.eye(4)
                body_pose[:3, :3], body_pose[:3, 3] = rotation, [p.x, p.y, p.z]
                np.testing.assert_allclose(body_pose @ np.array(report["T_body_camera"]), item["T_local_world_camera"], atol=1e-13)
                assert conn.topic == report["topics"]["odometry"]
                assert msg.header.frame_id == f"{report['agent']}/{report['map_id']}/world"
                assert msg.child_frame_id == f"{report['agent']}/body"
                no += 1
    assert ni == no == len(audit) == len(raw) == report["exported_pairs"]
    assert report["shared_world_status"] == "NOT_RUN" and report["backend_executed"] is False
    return {"status": "READBACK_PASS", "images_pixel_exact": ni, "raw_body_poses_checked": no,
            "source_hashes_checked": len(report["sources"]), "shared_world_status": "NOT_RUN"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    print(json.dumps(verify(parser.parse_args().directory), indent=2))
