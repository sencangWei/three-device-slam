"""Strict, side-effect-free loading of provisioned product configuration."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any


SCHEMA = "three-device-slam.product-config.v1"


class ConfigError(ValueError):
    """The product configuration does not match the frozen schema."""


@dataclass(frozen=True)
class UmiConfig:
    serial: str
    imu_path: str
    calibration_id: str
    imu_protocol: str = "auto"


@dataclass(frozen=True)
class EgoConfig:
    type: str
    calibration_id: str
    video_device: str | None = None
    xu_library: str | None = None
    serial: str | None = None


@dataclass(frozen=True)
class ProductConfig:
    left: UmiConfig | None
    right: UmiConfig | None
    ego: EgoConfig
    output_root: Path
    duration_s: float | None


def load_product_config(path: Path | str) -> ProductConfig:
    """Read and validate one config file without probing hardware or changing it."""
    try:
        payload = json.loads(
            Path(path).read_text(encoding="utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_constant,
        )
    except ConfigError:
        raise
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ConfigError("product config must be valid UTF-8 JSON") from exc

    root = _object(payload, "product config")
    _exact_fields(
        root,
        {"schema", "ego", "output_root", "duration_s"},
        "product config",
        optional={"left", "right"},
    )
    if root["schema"] != SCHEMA:
        raise ConfigError("product config schema is unsupported")

    ego_value = _object(root["ego"], "ego")
    ego = _ego(ego_value)
    left_value = root.get("left")
    right_value = root.get("right")
    left = _umi(left_value, "left") if left_value is not None else None
    right = _umi(right_value, "right") if right_value is not None else None
    if ego.type != "d435i" and left is None:
        raise ConfigError("left UMI is required unless ego.type is d435i")
    if ego.type != "d435i" and right is None:
        raise ConfigError("right UMI is required unless ego.type is d435i")
    if ego.type == "d435i" and left is None and right is None:
        raise ConfigError("at least one UMI is required")
    if left is not None and right is not None:
        if left.serial == right.serial:
            raise ConfigError("left and right D405 identities must be distinct")
        if left.imu_path == right.imu_path:
            raise ConfigError("left and right IMU paths must be distinct")
    duration = root["duration_s"]
    if duration is not None:
        if isinstance(duration, bool) or not isinstance(duration, (int, float)):
            raise ConfigError("duration_s must be positive and finite or null")
        try:
            duration = float(duration)
        except OverflowError as exc:
            raise ConfigError(
                "duration_s must be positive and finite or null"
            ) from exc
        if not math.isfinite(duration) or duration <= 0:
            raise ConfigError("duration_s must be positive and finite or null")

    return ProductConfig(
        left=left,
        right=right,
        ego=ego,
        output_root=Path(_linux_path(root["output_root"], "output_root")),
        duration_s=duration,
    )


def _umi(value: Any, name: str) -> UmiConfig:
    item = _object(value, name)
    expected = {"serial", "imu_path", "calibration_id"}
    if "imu_protocol" in item:
        expected.add("imu_protocol")
    _exact_fields(item, expected, name)
    imu_protocol = item.get("imu_protocol", "auto")
    if imu_protocol not in {"auto", "kt_ex9_37", "stm32_combined_v1"}:
        raise ConfigError(
            f"{name}.imu_protocol must be auto, kt_ex9_37, or stm32_combined_v1"
        )
    return UmiConfig(
        serial=_nonempty_string(item["serial"], f"{name}.serial"),
        imu_path=_linux_path(item["imu_path"], f"{name}.imu_path"),
        calibration_id=_nonempty_string(
            item["calibration_id"], f"{name}.calibration_id"
        ),
        imu_protocol=imu_protocol,
    )


def _ego(value: dict[str, Any]) -> EgoConfig:
    ego_type = value.get("type", "2uq2")
    if ego_type == "2uq2":
        expected = {"video_device", "xu_library", "calibration_id"}
        if "type" in value:
            expected.add("type")
        _exact_fields(value, expected, "ego")
        return EgoConfig(
            type="2uq2",
            video_device=_linux_path(value["video_device"], "ego.video_device"),
            xu_library=_linux_path(value["xu_library"], "ego.xu_library"),
            calibration_id=_nonempty_string(
                value["calibration_id"], "ego.calibration_id"
            ),
        )
    if ego_type == "d435i":
        _exact_fields(value, {"type", "serial", "calibration_id"}, "ego")
        return EgoConfig(
            type="d435i",
            serial=_nonempty_string(value["serial"], "ego.serial"),
            calibration_id=_nonempty_string(
                value["calibration_id"], "ego.calibration_id"
            ),
        )
    raise ConfigError("ego.type must be 2uq2 or d435i")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ConfigError(f"duplicate JSON member: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ConfigError(f"duration_s must be finite, not {value}")


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(f"{name} must be an object")
    return value


def _exact_fields(
    value: dict[str, Any],
    expected: set[str],
    name: str,
    optional: set[str] = frozenset(),
) -> None:
    missing = sorted(expected - value.keys())
    unknown = sorted(value.keys() - expected - set(optional))
    if missing:
        raise ConfigError(f"{name} missing fields: {', '.join(missing)}")
    if unknown:
        raise ConfigError(f"{name} has unknown fields: {', '.join(unknown)}")


def _nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{name} must be a non-empty string")
    return value


def _linux_path(value: Any, name: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or "\\" in value
        or "\x00" in value
        or not PurePosixPath(value).is_absolute()
    ):
        raise ConfigError(f"{name} must be an absolute Linux path")
    return value
