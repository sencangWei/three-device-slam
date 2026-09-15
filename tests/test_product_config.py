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
    assert config.left.imu_protocol == "auto"
    assert config.right.serial == "right-serial"
    assert config.ego.type == "2uq2"
    assert config.ego.video_device == "/dev/video0"
    assert config.output_root == Path("/var/lib/three-device-slam/sessions")
    assert config.duration_s is None
    with pytest.raises(FrozenInstanceError):
        config.left.serial = "replacement"


def test_loads_explicit_d405_imu_protocol(tmp_path):
    payload = valid_payload(tmp_path)
    payload["left"]["imu_protocol"] = "stm32_combined_v1"
    payload["right"]["imu_protocol"] = "kt_ex9_37"

    config = load_product_config(write_payload(tmp_path, payload))

    assert config.left.imu_protocol == "stm32_combined_v1"
    assert config.right.imu_protocol == "kt_ex9_37"


def test_rejects_unknown_d405_imu_protocol(tmp_path):
    payload = valid_payload(tmp_path)
    payload["left"]["imu_protocol"] = "invented"

    with pytest.raises(ConfigError, match="imu_protocol"):
        load_product_config(write_payload(tmp_path, payload))


def test_loads_explicit_d435i_ego_config(tmp_path):
    payload = valid_payload(tmp_path)
    payload["ego"] = {
        "type": "d435i",
        "serial": "327122078613",
        "calibration_id": "d435i-factory-327122078613-v1",
    }

    config = load_product_config(write_payload(tmp_path, payload))

    assert config.ego.type == "d435i"
    assert config.ego.serial == "327122078613"
    assert config.ego.video_device is None
    assert config.ego.xu_library is None


def d435i_single_umi_payload(tmp_path):
    payload = valid_payload(tmp_path)
    payload.pop("right")
    payload["ego"] = {
        "type": "d435i",
        "serial": "327122078613",
        "calibration_id": "d435i-factory-327122078613-v1",
    }
    payload["left"]["imu_protocol"] = "stm32_combined_v1"
    return payload


def test_loads_d435i_single_umi_config_without_right(tmp_path):
    config = load_product_config(
        write_payload(tmp_path, d435i_single_umi_payload(tmp_path))
    )

    assert config.ego.type == "d435i"
    assert config.right is None
    assert config.left.imu_protocol == "stm32_combined_v1"


def test_loads_d435i_single_umi_config_with_null_right(tmp_path):
    payload = d435i_single_umi_payload(tmp_path)
    payload["right"] = None

    config = load_product_config(write_payload(tmp_path, payload))

    assert config.right is None


def test_loads_d435i_right_only_config(tmp_path):
    payload = d435i_single_umi_payload(tmp_path)
    payload["right"] = payload.pop("left")

    config = load_product_config(write_payload(tmp_path, payload))

    assert config.left is None
    assert config.right.serial == "left-serial"


def test_rejects_d435i_without_any_umi(tmp_path):
    payload = d435i_single_umi_payload(tmp_path)
    payload.pop("left")

    with pytest.raises(ConfigError, match="at least one UMI"):
        load_product_config(write_payload(tmp_path, payload))


def test_rejects_missing_right_for_2uq2_ego(tmp_path):
    payload = valid_payload(tmp_path)
    payload.pop("right")

    with pytest.raises(ConfigError, match="right UMI is required"):
        load_product_config(write_payload(tmp_path, payload))


@pytest.mark.parametrize(
    "ego",
    [
        {"type": "d435i", "calibration_id": "cal"},
        {"type": "d435i", "serial": "serial", "calibration_id": "cal", "extra": 1},
        {"type": "other", "serial": "serial", "calibration_id": "cal"},
    ],
)
def test_rejects_invalid_explicit_ego_type_config(tmp_path, ego):
    payload = valid_payload(tmp_path)
    payload["ego"] = ego

    with pytest.raises(ConfigError):
        load_product_config(write_payload(tmp_path, payload))


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
