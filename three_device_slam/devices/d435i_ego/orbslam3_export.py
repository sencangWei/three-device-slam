"""Export an accepted D435i capture to ORB-SLAM3 stereo-inertial EuRoC input.

The camera geometry comes from the connected device's frozen factory snapshot.
The default IMU noise values are the ORB-SLAM3 D435i example values and remain
provisional until this physical D435i has an accepted noise/timing calibration.
"""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

from three_device_slam.devices.d435i_ego.vins_export import (
    ARTIFACT_ROOT,
    STREAMS,
    combine_imu,
    factory_transforms,
    interpolation_support_rows,
    read_stream,
    validate_capture_window,
)
from three_device_slam.spatial.covins_export import sha256


ORB_SLAM3_COMMIT = "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4"
BOUND_SERIAL = "327122078613"
INIT_TRANSLATION_ACCUMULATION_M = 0.02
INIT_TRANSLATION_RESET_M = 0.005


def _matrix_yaml(matrix: np.ndarray) -> list[str]:
    values = ", ".join(f"{float(value):.12g}" for value in matrix.reshape(-1))
    return [
        "   rows: 4",
        "   cols: 4",
        "   dt: f",
        f"   data: [{values}]",
    ]


def validate_rectified_factory_model(calibration: dict) -> dict:
    """Return the frozen rectified model or fail before discarding geometry."""
    if calibration.get("schema") != "ego.d435i.factory_calibration.v1":
        raise ValueError("not a D435i factory calibration snapshot")
    device = calibration.get("device", {})
    if device.get("serial") != BOUND_SERIAL or "D435I" not in str(
        device.get("name", "")
    ).upper():
        raise ValueError("not the bound Ego D435i")

    left = calibration["intrinsics"]["ir_left"]
    right = calibration["intrinsics"]["ir_right"]
    for name, camera in (("left", left), ("right", right)):
        if camera.get("distortion_model") != "distortion.brown_conrady":
            raise ValueError(f"{name} IR is not the accepted factory model")
        coefficients = np.asarray(camera.get("coefficients", []), dtype=float)
        if coefficients.shape != (5,) or not np.allclose(coefficients, 0.0):
            raise ValueError(f"{name} IR is not factory rectified")
        if (camera.get("width"), camera.get("height")) != (1280, 720):
            raise ValueError(f"{name} IR resolution is not 1280x720")

    keys = ("fx", "fy", "ppx", "ppy", "width", "height")
    if any(not np.isclose(float(left[key]), float(right[key])) for key in keys):
        raise ValueError("rectified stereo intrinsics differ")

    stereo = calibration["extrinsics"]["ir_left_to_ir_right"]
    rotation = np.asarray(stereo["rotation_row_major"], dtype=float).reshape(3, 3)
    translation = np.asarray(stereo["translation_m"], dtype=float)
    if not np.allclose(rotation, np.eye(3), atol=1e-7):
        raise ValueError("factory stereo pair is not rectified")
    if not np.allclose(translation[1:], 0.0, atol=1e-7):
        raise ValueError("factory stereo baseline is not horizontal")
    baseline = abs(float(translation[0]))
    if not 0.045 <= baseline <= 0.055:
        raise ValueError("D435i factory baseline outside 45-55 mm")

    body_t_cam0, _, accel_to_gyro = factory_transforms(calibration)
    return {
        "width": int(left["width"]),
        "height": int(left["height"]),
        "fps": 30,
        "fx": float(left["fx"]),
        "fy": float(left["fy"]),
        "cx": float(left["ppx"]),
        "cy": float(left["ppy"]),
        "baseline_m": baseline,
        "T_body_cam0": body_t_cam0,
        "R_accel_to_gyro": accel_to_gyro,
    }


def orbslam3_settings(
    calibration: dict,
    *,
    save_atlas: str | None = "map_atlas",
    load_atlas: str | None = None,
    noise_gyro: float = 1e-3,
    noise_acc: float = 1e-2,
    gyro_walk: float = 1e-6,
    acc_walk: float = 1e-4,
    features: int = 2000,
    init_translation_accumulation_m: float = INIT_TRANSLATION_ACCUMULATION_M,
    init_translation_reset_m: float = INIT_TRANSLATION_RESET_M,
) -> str:
    model = validate_rectified_factory_model(calibration)
    if save_atlas and load_atlas:
        raise ValueError("one run cannot both load and overwrite the same Atlas")
    if any(value <= 0 for value in (noise_gyro, noise_acc, gyro_walk, acc_walk)):
        raise ValueError("ORB-SLAM3 IMU noise parameters must be positive")
    if features < 500:
        raise ValueError("ORB feature budget is too small for 1280x720")
    if not (
        0 <= init_translation_reset_m < init_translation_accumulation_m
    ):
        raise ValueError("invalid ORB IMU initialization translation thresholds")

    lines = [
        "%YAML:1.0",
        "",
        f'# Source: D435i factory snapshot for serial {BOUND_SERIAL}.',
        "# IMU noise: provisional ORB-SLAM3 D435i example values.",
        '# Camera/IMU time offset: implicit 0 s in one RealSense "global_time" domain.',
        'File.version: "1.0"',
    ]
    if save_atlas:
        if save_atlas.endswith(".osa") or Path(save_atlas).is_absolute():
            raise ValueError("Atlas name must be relative and omit .osa")
        lines.append(f'System.SaveAtlasToFile: "{save_atlas}"')
    if load_atlas:
        if load_atlas.endswith(".osa") or Path(load_atlas).is_absolute():
            raise ValueError("Atlas name must be relative and omit .osa")
        lines.append(f'System.LoadAtlasFromFile: "{load_atlas}"')
    lines += [
        'Camera.type: "Rectified"',
        f'Camera1.fx: {model["fx"]:.12g}',
        f'Camera1.fy: {model["fy"]:.12g}',
        f'Camera1.cx: {model["cx"]:.12g}',
        f'Camera1.cy: {model["cy"]:.12g}',
        f'Stereo.b: {model["baseline_m"]:.12g}',
        f'Camera.width: {model["width"]}',
        f'Camera.height: {model["height"]}',
        f'Camera.fps: {model["fps"]}',
        "Camera.RGB: 1",
        "Stereo.ThDepth: 40.0",
        "IMU.T_b_c1: !!opencv-matrix",
        *_matrix_yaml(model["T_body_cam0"]),
        "IMU.InsertKFsWhenLost: 0",
        f"IMU.NoiseGyro: {noise_gyro:.12g}",
        f"IMU.NoiseAcc: {noise_acc:.12g}",
        f"IMU.GyroWalk: {gyro_walk:.12g}",
        f"IMU.AccWalk: {acc_walk:.12g}",
        "IMU.Frequency: 200.0",
        f"IMU.InitTranslationAccumulationThreshold: {init_translation_accumulation_m:.12g}",
        f"IMU.InitTranslationResetThreshold: {init_translation_reset_m:.12g}",
        f"ORBextractor.nFeatures: {features}",
        "ORBextractor.scaleFactor: 1.2",
        "ORBextractor.nLevels: 8",
        "ORBextractor.iniThFAST: 20",
        "ORBextractor.minThFAST: 7",
        "Viewer.KeyFrameSize: 0.05",
        "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08",
        "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0",
        "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -3.5",
        "Viewer.ViewpointF: 500.0",
        "Viewer.imageViewScale: 1.0",
    ]
    return "\n".join(lines) + "\n"


def _formal_rows(
    source: Path,
    *,
    require_acceptance: bool = True,
    diagnostic_end_s: float | None = None,
) -> tuple[dict, dict, dict, dict[str, list[dict]]]:
    calibration = json.loads((source / "calibration.json").read_text())
    acceptance = json.loads((source / "acceptance.json").read_text())
    pair = json.loads((source.parent / "pair_acceptance.json").read_text())
    manifest = json.loads((source / "manifest.json").read_text())
    validate_rectified_factory_model(calibration)
    if require_acceptance and (
        acceptance.get("status") != "PASS" or pair.get("status") != "PASS"
    ):
        raise ValueError("source transport acceptance is not PASS")
    if manifest != acceptance.get("manifest"):
        raise ValueError("source manifest does not match acceptance")
    if calibration["infrared_emitter_configuration"].get("readback") != 0:
        raise ValueError("D435i emitter was not disabled")
    start, end = int(pair["formal_start_ns"]), int(pair["formal_end_ns"])
    validate_capture_window(start, end)
    all_rows = {name: read_stream(source, name, manifest) for name in STREAMS}
    formal_rows = {
        name: [
            row
            for row in values
            if start <= int(row["acquisition_ns"]) < end and not row["warmup"]
        ]
        for name, values in all_rows.items()
    }
    for name, values in formal_rows.items():
        if len(values) != acceptance["streams"][name]["samples"] or len(values) < 2:
            raise ValueError(f"formal stream count mismatch: {name}")
    rows = formal_rows
    if diagnostic_end_s is not None:
        if require_acceptance or not 1.0 <= diagnostic_end_s < (end - start) / 1e9:
            raise ValueError("diagnostic slice must shorten a failed formal capture")
        slice_end = start + int(diagnostic_end_s * 1e9)
        rows = {
            name: [row for row in values if int(row["acquisition_ns"]) < slice_end]
            for name, values in formal_rows.items()
        }
        for name, values in rows.items():
            if len(values) < 2:
                raise ValueError(f"diagnostic slice has too few samples: {name}")
            if any(
                int(current["sequence"]) != int(previous["sequence"]) + 1
                for previous, current in zip(values, values[1:])
            ):
                raise ValueError(f"diagnostic slice is not sequence-continuous: {name}")
    left_key = [(row["sequence"], row["acquisition_ns"]) for row in rows["ir_left"]]
    right_key = [(row["sequence"], row["acquisition_ns"]) for row in rows["ir_right"]]
    if left_key != right_key:
        raise ValueError("stereo frames are not exact sequence/time pairs")
    return calibration, pair, manifest, rows | {"_all_accel": all_rows["accel"]}


def _encode_png(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise RuntimeError("OpenCV failed to encode D435i IR PNG")
    return encoded.tobytes()


def _write_images(
    source: Path,
    output: Path,
    rows: dict[str, list[dict]],
    model: dict,
) -> tuple[list[int], str]:
    cam_dirs = {
        "ir_left": output / "mav0/cam0/data",
        "ir_right": output / "mav0/cam1/data",
    }
    for directory in cam_dirs.values():
        directory.mkdir(parents=True)
    timestamps = [int(row["acquisition_ns"]) for row in rows["ir_left"]]
    manifest_path = output / "image_manifest.jsonl"
    with manifest_path.open("w", encoding="utf-8") as manifest_file:
        handles = {
            name: (source / f"ego.{name}.bin").open("rb") for name in cam_dirs
        }
        try:
            for name, directory in cam_dirs.items():
                index_path = directory.parent / "data.csv"
                with index_path.open("w", newline="") as index_file:
                    writer = csv.writer(index_file)
                    writer.writerow(["#timestamp [ns]", "filename"])
                    for row in rows[name]:
                        timestamp = int(row["acquisition_ns"])
                        expected_size = model["width"] * model["height"]
                        if row["size"] != expected_size:
                            raise ValueError("D435i IR payload size mismatch")
                        handles[name].seek(int(row["offset"]))
                        payload = handles[name].read(expected_size)
                        if len(payload) != expected_size:
                            raise ValueError("truncated D435i IR payload")
                        image = np.frombuffer(payload, dtype=np.uint8).reshape(
                            model["height"], model["width"]
                        )
                        encoded = _encode_png(image)
                        filename = f"{timestamp}.png"
                        (directory / filename).write_bytes(encoded)
                        writer.writerow([timestamp, filename])
                        manifest_file.write(
                            json.dumps(
                                {
                                    "stream": name,
                                    "timestamp_ns": timestamp,
                                    "filename": str((directory / filename).relative_to(output)),
                                    "png_sha256": hashlib.sha256(encoded).hexdigest(),
                                },
                                sort_keys=True,
                            )
                            + "\n"
                        )
        finally:
            for handle in handles.values():
                handle.close()
    return timestamps, sha256(manifest_path)


def _write_imu(
    source: Path,
    output: Path,
    gyro_rows: list[dict],
    all_accel_rows: list[dict],
    accel_to_gyro: np.ndarray,
) -> tuple[int, list[tuple[int, int, int, float]]]:
    accel_rows = interpolation_support_rows(
        all_accel_rows, [int(row["acquisition_ns"]) for row in gyro_rows]
    )
    gyro_all = np.fromfile(source / "ego.gyro.bin", dtype="<f4").reshape(-1, 3)
    accel_all = np.fromfile(source / "ego.accel.bin", dtype="<f4").reshape(-1, 3)
    gyro = gyro_all[[int(row["offset"]) // 12 for row in gyro_rows]]
    accel = accel_all[[int(row["offset"]) // 12 for row in accel_rows]]
    imu, audit = combine_imu(
        [int(row["acquisition_ns"]) for row in gyro_rows],
        gyro,
        [int(row["acquisition_ns"]) for row in accel_rows],
        accel,
        accel_to_gyro,
    )
    imu_dir = output / "mav0/imu0"
    imu_dir.mkdir(parents=True)
    with (imu_dir / "data.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "#timestamp [ns]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
            ]
        )
        for row, values in zip(gyro_rows, imu, strict=True):
            writer.writerow([int(row["acquisition_ns"]), *map(float, values)])
    return len(imu), audit


def export(
    source: Path,
    output: Path,
    *,
    diagnostic_end_s: float | None = None,
) -> dict:
    source, output = source.resolve(), output.resolve()
    if (
        output.exists()
        or output == ARTIFACT_ROOT.resolve()
        or not output.is_relative_to(ARTIFACT_ROOT.resolve())
    ):
        raise ValueError("output must be NEW under repository artifacts")
    diagnostic = diagnostic_end_s is not None
    calibration, pair, manifest, rows = _formal_rows(
        source,
        require_acceptance=not diagnostic,
        diagnostic_end_s=diagnostic_end_s,
    )
    model = validate_rectified_factory_model(calibration)
    output.mkdir(parents=True)
    try:
        timestamps, image_manifest_hash = _write_images(
            source, output, rows, model
        )
        imu_count, audit = _write_imu(
            source,
            output,
            rows["gyro"],
            rows["_all_accel"],
            model["R_accel_to_gyro"],
        )
        imu_timestamps = [int(row["acquisition_ns"]) for row in rows["gyro"]]
        if imu_timestamps[0] > timestamps[0] or imu_timestamps[-1] <= timestamps[-1]:
            raise ValueError("ORB-SLAM3 requires IMU coverage across all camera frames")
        (output / "times.txt").write_text(
            "".join(f"{timestamp}\n" for timestamp in timestamps), encoding="utf-8"
        )
        (output / "orbslam3_d435i.yaml").write_text(
            orbslam3_settings(calibration), encoding="utf-8"
        )
        with (output / "imu_interpolation.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["gyro_ns", "accel_before_ns", "accel_after_ns", "fraction"]
            )
            writer.writerows(audit)
        report = {
            "schema": "three-device-slam.d435i-orbslam3-export.v1",
            "status": (
                "DIAGNOSTIC_ORB_INPUT_READY_SOURCE_FAILED"
                if diagnostic
                else "PROVISIONAL_ORB_INPUT_READY"
            ),
            "engine": {
                "name": "ORB-SLAM3",
                "commit": ORB_SLAM3_COMMIT,
                "mode": "IMU_STEREO",
                "init_translation_thresholds_m": {
                    "accumulation": INIT_TRANSLATION_ACCUMULATION_M,
                    "reset": INIT_TRANSLATION_RESET_M,
                },
            },
            "device": calibration["device"],
            "source": str(source),
            "source_mode": pair["mode"],
            "formal_start_ns": int(pair["formal_start_ns"]),
            "formal_end_ns": int(timestamps[-1]),
            "counts": {
                "stereo_pairs": len(timestamps),
                "imu": imu_count,
            },
            "model": {
                key: value.tolist() if isinstance(value, np.ndarray) else value
                for key, value in model.items()
            },
            "timestamp_policy": (
                "unaltered RealSense global_time mapped to host monotonic ns; "
                "accel bracket-interpolated at gyro timestamps"
            ),
            "calibration_status": (
                "FACTORY_RECTIFIED_GEOMETRY_REFERENCE_IMU_NOISE_TD0_UNVALIDATED"
            ),
            "source_acceptance": {
                "ego": json.loads((source / "acceptance.json").read_text())["status"],
                "pair": pair["status"],
            },
            "source_stream_hashes": manifest["streams"],
            "outputs": {
                "settings_sha256": sha256(output / "orbslam3_d435i.yaml"),
                "times_sha256": sha256(output / "times.txt"),
                "imu_sha256": sha256(output / "mav0/imu0/data.csv"),
                "image_manifest_sha256": image_manifest_hash,
            },
            "limitations": [
                "D435i-specific IMU noise and camera/IMU timing are not accepted",
                "export success is not a trajectory or world-coordinate PASS",
                *(
                    [
                        "diagnostic prefix was cut before a known source transport gap",
                        "failed parent capture cannot be promoted through this export",
                    ]
                    if diagnostic
                    else []
                ),
            ],
        }
        (output / "export_report.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        return report
    except Exception:
        # Preserve partial output for diagnosis. It can never carry a ready report.
        raise
