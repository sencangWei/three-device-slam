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


@dataclass(frozen=True)
class EgoConfig:
    video_device: str
    xu_library: str
    calibration_id: str


@dataclass(frozen=True)
class ProductConfig:
    left: UmiConfig
    right: UmiConfig
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
        {"schema", "left", "right", "ego", "output_root", "duration_s"},
        "product config",
    )
    if root["schema"] != SCHEMA:
        raise ConfigError("product config schema is unsupported")

    left = _umi(root["left"], "left")
    right = _umi(root["right"], "right")
    if left.serial == right.serial:
        raise ConfigError("left and right D405 identities must be distinct")

    ego_value = _object(root["ego"], "ego")
    _exact_fields(
        ego_value,
        {"video_device", "xu_library", "calibration_id"},
        "ego",
    )
    ego = EgoConfig(
        video_device=_linux_path(ego_value["video_device"], "ego.video_device"),
        xu_library=_linux_path(ego_value["xu_library"], "ego.xu_library"),
        calibration_id=_nonempty_string(
            ego_value["calibration_id"], "ego.calibration_id"
        ),
    )
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
    _exact_fields(item, {"serial", "imu_path", "calibration_id"}, name)
    return UmiConfig(
        serial=_nonempty_string(item["serial"], f"{name}.serial"),
        imu_path=_linux_path(item["imu_path"], f"{name}.imu_path"),
        calibration_id=_nonempty_string(
            item["calibration_id"], f"{name}.calibration_id"
        ),
    )


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


def _exact_fields(value: dict[str, Any], expected: set[str], name: str) -> None:
    missing = sorted(expected - value.keys())
    unknown = sorted(value.keys() - expected)
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
