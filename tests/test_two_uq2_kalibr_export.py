import hashlib
import json
from pathlib import Path
import zlib

import numpy as np
import pytest

from three_device_slam.devices.two_uq2.kalibr_export import (
    export_imu_csv,
    export_stereo_dataset,
    select_camera_rows,
)


def test_export_imu_csv_preserves_si_values_and_acquisition_timestamps(tmp_path):
    session = tmp_path / "session"
    output = session / "kalibr_cam_imu_10hz"
    output.mkdir(parents=True)
    payload = b"raw-packets"
    rows = [
        {
            "stream_id": "ego.imu",
            "sequence": 1,
            "acquisition_ns": 1_000_000_001,
            "valid": True,
            "metadata": {
                "gyro_rad_s": [0.1, 0.2, 0.3],
                "accel_m_s2": [1.0, 2.0, 3.0],
            },
        },
        {
            "stream_id": "ego.imu",
            "sequence": 2,
            "acquisition_ns": 1_005_000_001,
            "valid": True,
            "metadata": {
                "gyro_rad_s": [0.4, 0.5, 0.6],
                "accel_m_s2": [4.0, 5.0, 6.0],
            },
        },
    ]
    index = "".join(json.dumps(row) + "\n" for row in rows)
    (session / "ego.imu.bin").write_bytes(payload)
    (session / "ego.imu.jsonl").write_text(index, encoding="utf-8")
    (session / "manifest.json").write_text(
        json.dumps(
            {
                "streams": {
                    "ego.imu": {
                        "payload_sha256": hashlib.sha256(payload).hexdigest(),
                        "index_sha256": hashlib.sha256(index.encode()).hexdigest(),
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    report = export_imu_csv(session=session, output=output)

    assert report["imu_samples"] == 2
    assert report["timestamp_regressions"] == 0
    assert (output / "imu0.csv").read_text().splitlines() == [
        "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z",
        "1000000001,0.1,0.2,0.3,1.0,2.0,3.0",
        "1005000001,0.4,0.5,0.6,4.0,5.0,6.0",
    ]


def test_select_camera_rows_uses_acquisition_time_without_drift():
    rows = [
        {"acquisition_ns": 1_000_000_000},
        {"acquisition_ns": 1_100_000_000},
        {"acquisition_ns": 1_200_000_000},
        {"acquisition_ns": 1_399_000_000},
        {"acquisition_ns": 1_401_000_000},
    ]

    selected = select_camera_rows(rows, frequency_hz=5.0)

    assert [row["acquisition_ns"] for row in selected] == [
        1_000_000_000,
        1_200_000_000,
        1_401_000_000,
    ]


def test_select_camera_rows_rejects_timestamp_regression():
    with pytest.raises(ValueError, match="timestamp regression"):
        select_camera_rows(
            [{"acquisition_ns": 2}, {"acquisition_ns": 1}],
            frequency_hz=5.0,
        )


def test_export_stereo_dataset_validates_and_splits_sbs(tmp_path):
    import cv2

    session = tmp_path / "session"
    session.mkdir()
    left = np.full((8, 12), 20, dtype=np.uint8)
    right = np.full((8, 12), 220, dtype=np.uint8)
    sbs = np.hstack((left, right))
    ok, encoded = cv2.imencode(".jpg", sbs)
    assert ok
    jpeg = encoded.tobytes()
    rows = []
    payload = bytearray()
    for sequence, timestamp_ns in enumerate(
        (1_000_000_000, 1_100_000_000, 1_250_000_000),
        start=1,
    ):
        offset = len(payload)
        payload.extend(jpeg)
        rows.append(
            {
                "stream_id": "ego.video",
                "sequence": sequence,
                "acquisition_ns": timestamp_ns,
                "offset": offset,
                "size": len(jpeg),
                "crc32": zlib.crc32(jpeg) & 0xFFFFFFFF,
                "valid": True,
                "metadata": {
                    "encoding": "MJPG",
                    "width": 24,
                    "height": 8,
                    "layout": "side_by_side_left_then_right_unverified",
                },
            }
        )
    payload_path = session / "ego.video.bin"
    index_path = session / "ego.video.jsonl"
    payload_path.write_bytes(payload)
    index_path.write_text(
        "".join(json.dumps(row) + "\n" for row in rows),
        encoding="utf-8",
    )
    (session / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "ego.three_device.raw_session.v1",
                "streams": {
                    "ego.video": {
                        "payload_sha256": hashlib.sha256(payload).hexdigest(),
                        "index_sha256": hashlib.sha256(index_path.read_bytes()).hexdigest(),
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    (session / "capture_report.json").write_text(
        json.dumps(
            {
                "stage": "stereo",
                "stage_acceptance": {"status": "PASS"},
            }
        ),
        encoding="utf-8",
    )
    output = session / "kalibr_stereo_5hz"

    report = export_stereo_dataset(
        session=session,
        output=output,
        frequency_hz=5.0,
        expected_shape=(8, 24),
    )

    assert report["status"] == "PASS"
    assert report["selected_pairs"] == 2
    assert sorted(path.name for path in (output / "cam0").iterdir()) == [
        "1000000000.png",
        "1250000000.png",
    ]
    cam0 = cv2.imread(str(output / "cam0" / "1000000000.png"), cv2.IMREAD_GRAYSCALE)
    cam1 = cv2.imread(str(output / "cam1" / "1000000000.png"), cv2.IMREAD_GRAYSCALE)
    assert cam0.shape == (8, 12)
    assert cam1.shape == (8, 12)
    assert float(cam0.mean()) < 50
    assert float(cam1.mean()) > 190
    index_rows = [
        json.loads(line)
        for line in (output / "export_index.jsonl").read_text().splitlines()
    ]
    assert index_rows[0]["cam0"] == "cam0/1000000000.png"
    assert index_rows[0]["cam1"] == "cam1/1000000000.png"


def test_export_stereo_dataset_rejects_crc_mismatch(tmp_path):
    session = tmp_path / "session"
    session.mkdir()
    payload = b"\xff\xd8broken\xff\xd9"
    (session / "ego.video.bin").write_bytes(payload)
    row = {
        "stream_id": "ego.video",
        "sequence": 1,
        "acquisition_ns": 1_000_000_000,
        "offset": 0,
        "size": len(payload),
        "crc32": 0,
        "valid": True,
        "metadata": {
            "encoding": "MJPG",
            "width": 24,
            "height": 8,
            "layout": "side_by_side_left_then_right_unverified",
        },
    }
    index = json.dumps(row) + "\n"
    (session / "ego.video.jsonl").write_text(index, encoding="utf-8")
    (session / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "ego.three_device.raw_session.v1",
                "streams": {
                    "ego.video": {
                        "payload_sha256": hashlib.sha256(payload).hexdigest(),
                        "index_sha256": hashlib.sha256(index.encode()).hexdigest(),
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    (session / "capture_report.json").write_text(
        json.dumps(
            {"stage": "stereo", "stage_acceptance": {"status": "PASS"}}
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="CRC mismatch"):
        export_stereo_dataset(
            session=session,
            output=session / "kalibr_stereo_5hz",
            frequency_hz=5.0,
            expected_shape=(8, 24),
        )
