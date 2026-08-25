import json
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from three_device_slam.config import ConfigError, load_product_config


def valid_payload(tmp_path, *, duration_s=None):
    return {
        "schema": "three-device-slam.product-config.v1",
        "left": {
            "serial": "left-serial",
            "imu_path": "/dev/serial/by-id/left-imu",
            "calibration_id": "left-cal-v1",
        },
        "right": {
            "serial": "right-serial",
            "imu_path": "/dev/serial/by-id/right-imu",
            "calibration_id": "right-cal-v1",
        },
        "ego": {
            "video_device": "/dev/video0",
            "xu_library": "/opt/three-device-slam/lib/libtwo_uq2_xu.so",
            "calibration_id": "ego-cal-v1",
        },
        "output_root": "/var/lib/three-device-slam/sessions",
        "duration_s": duration_s,
    }


def write_payload(tmp_path, payload):
    path = tmp_path / "product.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_loads_frozen_distinct_device_config(tmp_path):
    config = load_product_config(write_payload(tmp_path, valid_payload(tmp_path)))

    assert config.left.serial == "left-serial"
    assert config.left.imu_path == "/dev/serial/by-id/left-imu"
    assert config.right.serial == "right-serial"
    assert config.ego.video_device == "/dev/video0"
    assert config.output_root == Path("/var/lib/three-device-slam/sessions")
    assert config.duration_s is None
    with pytest.raises(FrozenInstanceError):
        config.left.serial = "replacement"


def test_rejects_duplicate_d405_identity(tmp_path):
    payload = valid_payload(tmp_path)
    payload["right"]["serial"] = payload["left"]["serial"]

    with pytest.raises(ConfigError, match="distinct"):
        load_product_config(write_payload(tmp_path, payload))


def test_rejects_duplicate_d405_imu_path(tmp_path):
    payload = valid_payload(tmp_path)
    payload["right"]["imu_path"] = payload["left"]["imu_path"]

    with pytest.raises(ConfigError, match="IMU paths must be distinct"):
        load_product_config(write_payload(tmp_path, payload))


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.pop("ego"), "missing"),
        (lambda value: value.__setitem__("extra", True), "unknown"),
        (lambda value: value["left"].__setitem__("extra", True), "unknown"),
        (lambda value: value.__setitem__("schema", "other"), "schema"),
        (lambda value: value["left"].__setitem__("serial", ""), "serial"),
        (lambda value: value["left"].__setitem__("imu_path", "ttyUSB0"), "absolute Linux"),
        (lambda value: value["ego"].__setitem__("video_device", "C:\\video0"), "absolute Linux"),
        (lambda value: value.__setitem__("output_root", "sessions"), "absolute Linux"),
        (lambda value: value["ego"].__setitem__("calibration_id", "  "), "calibration"),
        (lambda value: value.__setitem__("duration_s", 0), "duration_s"),
        (lambda value: value.__setitem__("duration_s", -1.0), "duration_s"),
        (lambda value: value.__setitem__("duration_s", True), "duration_s"),
        (lambda value: value.__setitem__("duration_s", "60"), "duration_s"),
        (lambda value: value.__setitem__("duration_s", 10**400), "duration_s"),
    ],
)
def test_rejects_invalid_schema_values(tmp_path, mutation, message):
    payload = valid_payload(tmp_path)
    mutation(payload)

    with pytest.raises(ConfigError, match=message):
        load_product_config(write_payload(tmp_path, payload))


@pytest.mark.parametrize("constant", ["NaN", "Infinity", "-Infinity"])
def test_rejects_nonfinite_duration(tmp_path, constant):
    payload = valid_payload(tmp_path)
    text = json.dumps(payload).replace('"duration_s": null', f'"duration_s": {constant}')
    path = tmp_path / "product.json"
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ConfigError, match="finite"):
        load_product_config(path)


def test_rejects_duplicate_json_members(tmp_path):
    payload = valid_payload(tmp_path)
    text = json.dumps(payload).replace(
        '"schema": "three-device-slam.product-config.v1",',
        '"schema": "three-device-slam.product-config.v1", "schema": "other",',
    )
    path = tmp_path / "product.json"
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ConfigError, match="duplicate"):
        load_product_config(path)
