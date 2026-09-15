"""Export existing pair-capture Ego data for provisional, offline built-in-IMU VINS.

Factory geometry only; noise defaults and td=0 are uncalibrated hypotheses.
No hardware/ROS processes are started. Source data is never changed.
"""
from __future__ import annotations

import argparse
import csv
from contextlib import ExitStack
import json
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

from three_device_slam.quality.verify_pair_session import _verify_payload_crc
from three_device_slam.spatial.covins_export import sha256
from three_device_slam.spatial.se3 import invert, validate_transform

ARTIFACT_ROOT = Path(__file__).resolve().parents[3] / "artifacts"
STREAMS = ("ir_left", "ir_right", "gyro", "accel")
CLOCK = "realsense_global_time_mapped_to_host_monotonic"
MAX_CAPTURE_DURATION_NS = 120_000_000_000


def validate_capture_window(start_ns: int, end_ns: int) -> int:
    duration_ns = end_ns - start_ns
    if not 0 < duration_ns <= MAX_CAPTURE_DURATION_NS:
        raise ValueError("only bounded <=120s capture supported")
    return duration_ns


def factory_transforms(cal: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    def transform(key):
        entry = cal["extrinsics"][key]
        t = np.eye(4)
        t[:3, :3] = np.array(entry["rotation_row_major"]).reshape(3, 3)
        t[:3, 3] = entry["translation_m"]
        return validate_transform(t)
    left_from_gyro = transform("gyro_to_ir_left")
    right_from_left = transform("ir_left_to_ir_right")
    accel_from_gyro = transform("gyro_to_accel")
    if np.linalg.norm(accel_from_gyro[:3, 3]) > 1e-6:
        raise ValueError("non-collocated accel/gyro requires lever-arm compensation")
    if not np.allclose(transform("accel_to_ir_left") @ accel_from_gyro, left_from_gyro, atol=1e-7):
        raise ValueError("inconsistent factory IMU camera geometry")
    return invert(left_from_gyro), invert(left_from_gyro) @ invert(right_from_left), accel_from_gyro[:3, :3].T


def combine_imu(gyro_t, gyro, accel_t, accel, accel_to_gyro, max_gap_ns=10_000_000):
    """Accel interpolation at original gyro stamps; no extrapolation/retimestamping."""
    gyro_t, accel_t = np.asarray(gyro_t, dtype=np.int64), np.asarray(accel_t, dtype=np.int64)
    gyro, accel = np.asarray(gyro), np.asarray(accel)
    if len(gyro_t) < 2 or len(accel_t) < 2 or any(np.any(np.diff(t) <= 0) for t in (gyro_t, accel_t)):
        raise ValueError("IMU timestamps must be strictly increasing")
    if gyro.shape != (len(gyro_t), 3) or accel.shape != (len(accel_t), 3):
        raise ValueError("IMU dimensions")
    if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel)):
        raise ValueError("nonfinite IMU")
    out, audit = [], []
    for i, stamp in enumerate(gyro_t):
        hi = int(np.searchsorted(accel_t, stamp))
        if hi < len(accel_t) and accel_t[hi] == stamp:
            lo, fraction = hi, 0.0
        else:
            lo = hi - 1
            if lo < 0 or hi >= len(accel_t):
                raise ValueError("accel does not bracket gyro; no extrapolation")
            gap = int(accel_t[hi] - accel_t[lo])
            if gap > max_gap_ns:
                raise ValueError("accel interpolation gap exceeds 10 ms")
            fraction = int(stamp - accel_t[lo]) / gap
        value = accel_to_gyro @ ((1 - fraction) * accel[lo] + fraction * accel[hi])
        out.append(np.r_[gyro[i], value])
        audit.append((int(stamp), int(accel_t[lo]), int(accel_t[hi]), fraction))
    return np.asarray(out), audit


def interpolation_support_rows(rows: list[dict], target_timestamps: list[int]) -> list[dict]:
    """Keep the smallest source interval that brackets every target timestamp."""
    if not rows or not target_timestamps:
        raise ValueError("IMU interpolation support is empty")
    stamps = np.asarray([row["acquisition_ns"] for row in rows], dtype=np.int64)
    targets = np.asarray(target_timestamps, dtype=np.int64)
    if np.any(np.diff(stamps) <= 0) or np.any(np.diff(targets) <= 0):
        raise ValueError("IMU timestamps must be strictly increasing")
    first = int(np.searchsorted(stamps, targets[0], side="right") - 1)
    last = int(np.searchsorted(stamps, targets[-1], side="left"))
    if first < 0 or last >= len(rows):
        raise ValueError("accel does not bracket gyro; no extrapolation")
    return rows[first:last + 1]


def read_stream(source, name, manifest):
    stream = "ego." + name
    for suffix, field in (("jsonl", "index_sha256"), ("bin", "payload_sha256")):
        if sha256(source / f"{stream}.{suffix}") != manifest["streams"][stream][field]:
            raise ValueError(f"source hash mismatch: {stream}.{suffix}")
    _verify_payload_crc(source.parent, source.name, stream)
    rows = [json.loads(s) for s in (source / f"{stream}.jsonl").read_text().splitlines()]
    for row in rows:
        if row["stream_id"] != stream or row["valid"] is not True or row["clock_domain"] != CLOCK:
            raise ValueError("invalid stream identity/validity/clock")
        expected = ("rad/s", "float32_le_xyz", 12) if name == "gyro" else ("m/s^2", "float32_le_xyz", 12)
        if name in ("gyro", "accel") and (row["metadata"]["units"], row["metadata"]["encoding"], row["size"]) != expected:
            raise ValueError("invalid IMU units/encoding/size")
    return rows


def export(source: Path, output: Path) -> dict:
    source, output = source.resolve(), output.resolve()
    if output.exists() or output == ARTIFACT_ROOT.resolve() or not output.is_relative_to(ARTIFACT_ROOT.resolve()):
        raise ValueError("output must be NEW under repository artifacts")
    cal = json.loads((source / "calibration.json").read_text())
    acceptance = json.loads((source / "acceptance.json").read_text())
    pair = json.loads((source.parent / "pair_acceptance.json").read_text())
    manifest = json.loads((source / "manifest.json").read_text())
    if cal["schema"] != "ego.d435i.factory_calibration.v1" or cal["device"]["serial"] != "327122078613":
        raise ValueError("not the bound Ego D435i")
    if acceptance["status"] != "PASS" or pair["status"] != "PASS" or manifest != acceptance["manifest"]:
        raise ValueError("source transport acceptance/manifest mismatch")
    if cal["infrared_emitter_configuration"]["readback"] != 0:
        raise ValueError("emitter was not disabled")
    start, end = pair["formal_start_ns"], pair["formal_end_ns"]
    validate_capture_window(start, end)
    all_rows = {name: read_stream(source, name, manifest) for name in STREAMS}
    rows = {name: [r for r in rr if start <= r["acquisition_ns"] < end and not r["warmup"]]
            for name, rr in all_rows.items()}
    for name, rr in rows.items():
        if len(rr) != acceptance["streams"][name]["samples"] or len(rr) < 2:
            raise ValueError(f"formal stream count mismatch: {name}")
        if any(b["acquisition_ns"] <= a["acquisition_ns"] or b["sequence"] != a["sequence"] + 1 for a, b in zip(rr, rr[1:])):
            raise ValueError("formal timestamp/sequence discontinuity")
    left, right = rows["ir_left"], rows["ir_right"]
    if [(r["sequence"], r["acquisition_ns"]) for r in left] != [(r["sequence"], r["acquisition_ns"]) for r in right]:
        raise ValueError("stereo exact sequence/time pairing required")
    b_c0, b_c1, r_ga = factory_transforms(cal)
    gyro_all = np.fromfile(source / "ego.gyro.bin", dtype="<f4").reshape(-1, 3)
    accel_all = np.fromfile(source / "ego.accel.bin", dtype="<f4").reshape(-1, 3)
    gyro = gyro_all[[r["offset"] // 12 for r in rows["gyro"]]]
    # Valid samples immediately outside the formal window may bracket its
    # first/last gyro. Their original timestamps remain in the audit; this does
    # not extrapolate or publish any pre/post-window IMU message.
    accel_rows = interpolation_support_rows(
        all_rows["accel"], [r["acquisition_ns"] for r in rows["gyro"]])
    accel = accel_all[[r["offset"] // 12 for r in accel_rows]]
    imu, audit = combine_imu([r["acquisition_ns"] for r in rows["gyro"]], gyro,
        [r["acquisition_ns"] for r in accel_rows], accel, r_ga)
    output.mkdir(parents=True)
    (output / "solver_output").mkdir()
    for name, fn in (("ir_left", "left.yaml"), ("ir_right", "right.yaml")):
        c = cal["intrinsics"][name]
        if c["distortion_model"] != "distortion.brown_conrady" or any(c["coefficients"]):
            raise ValueError("only factory rectified IR supported; never discard distortion")
        text = (f'%YAML:1.0\nmodel_type: PINHOLE\ncamera_name: {name}\nimage_width: {c["width"]}\nimage_height: {c["height"]}\n'
                'distortion_parameters: {k1: 0., k2: 0., p1: 0., p2: 0.}\n'
                f'projection_parameters: {{fx: {c["fx"]}, fy: {c["fy"]}, cx: {c["ppx"]}, cy: {c["ppy"]}}}\n')
        (output / fn).write_text(text)
    config = ['%YAML:1.0', '# PROVISIONAL factory geometry / reference noise / uncalibrated td=0.',
        'imu: 1', 'num_of_cam: 2', 'imu_topic: "/ego_probe/imu0"',
        'image0_topic: "/ego_probe/cam0/image_raw"', 'image1_topic: "/ego_probe/cam1/image_raw"',
        f'output_path: "{output / "solver_output"}"', 'cam0_calib: "left.yaml"', 'cam1_calib: "right.yaml"',
        'estimate_extrinsic: 0', 'estimate_td: 0', 'td: 0.', 'multiple_thread: 1',
        'max_cnt: 400', 'min_dist: 20', 'freq: 30', 'F_threshold: 1.', 'show_track: 0', 'flow_back: 1',
        'enable_zupt: 0', 'max_solver_time: 0.04', 'max_num_iterations: 8', 'keyframe_parallax: 10.',
        'acc_n: 0.1', 'gyr_n: 0.01', 'acc_w: 0.001', 'gyr_w: 0.0001', 'g_norm: 9.805']
    for name, t in (("body_T_cam0", b_c0), ("body_T_cam1", b_c1)):
        config += [f'{name}: !!opencv-matrix', '   rows: 4', '   cols: 4', '   dt: d',
                   '   data: ' + json.dumps(t.reshape(-1).tolist())]
    (output / "vins_config.yaml").write_text("\n".join(config) + "\n")
    store = get_typestore(Stores.ROS2_HUMBLE)
    typ = store.types
    def header(ts, frame):
        return typ["std_msgs/msg/Header"](typ["builtin_interfaces/msg/Time"](*divmod(ts, 10**9)), frame)
    with Writer(output / "sensors", version=8) as bag, ExitStack() as stack:
        conns = {name: bag.add_connection(topic, msgtype, typestore=store) for name, topic, msgtype in (
            ("ir_left", "/ego_probe/cam0/image_raw", "sensor_msgs/msg/Image"),
            ("ir_right", "/ego_probe/cam1/image_raw", "sensor_msgs/msg/Image"),
            ("gyro", "/ego_probe/imu0", "sensor_msgs/msg/Imu"))}
        files = {n: stack.enter_context((source / f"ego.{n}.bin").open("rb")) for n in ("ir_left", "ir_right")}
        events = sorted((r["acquisition_ns"], name, i) for name in conns for i, r in enumerate(rows[name]))
        for ts, name, i in events:
            conn = conns[name]
            if name == "gyro":
                vector = typ["geometry_msgs/msg/Vector3"]
                unknown = np.zeros(9); unknown[0] = -1.
                msg = typ["sensor_msgs/msg/Imu"](header(ts, "ego/imu_raw"),
                    typ["geometry_msgs/msg/Quaternion"](0., 0., 0., 1.), unknown,
                    vector(*imu[i, :3]), np.zeros(9), vector(*imu[i, 3:]), np.zeros(9))
            else:
                r, c = rows[name][i], cal["intrinsics"][name]
                if r["size"] != c["width"] * c["height"] or r["metadata"]["encoding"] != "Y8":
                    raise ValueError("IR encoding/dimensions mismatch")
                files[name].seek(r["offset"])
                data = np.frombuffer(files[name].read(r["size"]), dtype=np.uint8)
                msg = typ["sensor_msgs/msg/Image"](header(ts, "ego/" + name), c["height"], c["width"], "mono8", 0, c["width"], data)
            bag.write(conn, ts, store.serialize_cdr(msg, conn.msgtype))
    with (output / "imu_interpolation.csv").open("w", newline="") as f:
        writer = csv.writer(f); writer.writerow(["gyro_ns", "accel_before_ns", "accel_after_ns", "fraction"])
        writer.writerows(audit)
    sources = [source / n for n in ("calibration.json", "acceptance.json", "manifest.json")]
    sources += [source.parent / "pair_acceptance.json"]
    report = {"schema": "ego.d435i.vins-export.v1", "status": "PROVISIONAL_INPUT_READY",
        "calibration_status": "FACTORY_GEOMETRY_ONLY_TD_NOISE_UNVALIDATED", "shared_world_status": "NOT_RUN",
        "source": str(source), "device": cal["device"], "formal_start_ns": start, "formal_end_ns": end,
        "counts": {k: len(v) for k, v in rows.items()}, "combined_imu_samples": len(imu),
        "T_body_cam0": b_c0.tolist(), "T_body_cam1": b_c1.tolist(),
        "timestamp_policy": "unaltered host-monotonic acquisition ns, accel bracket interpolation at gyro stamps",
        "imu_accel_interpolation_support": {
            "samples": len(accel_rows),
            "warmup_samples": sum(bool(r["warmup"]) for r in accel_rows),
            "first_ns": accel_rows[0]["acquisition_ns"],
            "last_ns": accel_rows[-1]["acquisition_ns"],
        },
        "imu_policy": "built-in only; native gyro axes; accel rotated into gyro frame; no STM32 rotation or td",
        "imu_accel_norm_median": float(np.median(np.linalg.norm(imu[:, 3:], axis=1))),
        "gyro_norm_median": float(np.median(np.linalg.norm(imu[:, :3], axis=1))),
        "source_mode": pair["mode"], "source_stream_hashes": manifest["streams"],
        "sources": {str(p): sha256(p) for p in sources},
        "outputs": {str(p.relative_to(output)): sha256(p) for p in output.rglob("*") if p.is_file()},
        "limitations": (
            (["static recording cannot validate dynamic scale/td/extrinsics"]
             if pair["mode"] == "bench_no_motion" else [])
            + ["factory geometry and upstream reference noise are not product-calibrated",
               "no VINS executed by exporter"]
        )}
    (output / "input_report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.source, args.output), indent=2))


if __name__ == "__main__":
    main()
