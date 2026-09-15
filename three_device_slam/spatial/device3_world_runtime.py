"""Quality-gated world-anchor runtime for the accepted device3/right mount.

The live detector/VIO adapter produces one ``T_world_right_odom`` candidate per
synchronized observation.  This module owns the product decision: initialize or
update only from a low-angular-rate multi-frame consensus and otherwise retain
the last accepted anchor.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .apriltag_alignment import AnchorAcceptance, TagMount, reconcile_anchor
from .se3 import compose, invert, pose_error, validate_transform


PROFILE_SCHEMA = "three-device-slam.device3-world-runtime-profile.v1"
CALIBRATION_SCHEMA = "three-device-slam.device3-right-world-mount-calibration.v1"
PACKAGE_SCHEMA = "three-device-slam.device3-right-world-mount-accepted-package.v1"
EXPECTED_EGO_SERIAL = "327122078613"
EXPECTED_RIGHT_SERIAL = "260422274454"
EXPECTED_DEVICE_ID = "right"
EXPECTED_TAG_ID = 1
EXPECTED_TAG_FAMILY = "tag36h11"


class RuntimeProfileError(ValueError):
    """The right-only runtime profile is not the accepted device3 chain."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_object(path: Path, *, label: str) -> dict:
    try:
        payload = json.loads(path.read_text())
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeProfileError(f"cannot read {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise RuntimeProfileError(f"{label} must contain a JSON object")
    return payload


def _string(value, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeProfileError(f"{name} must be a non-empty string")
    return value


def _number(value, *, name: str, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeProfileError(f"{name} must be finite")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        raise RuntimeProfileError(f"{name} must be finite and positive")
    return result


def _integer(value, *, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise RuntimeProfileError(f"{name} must be an integer >= {minimum}")
    return value


def _path(value, *, profile_path: Path, name: str) -> Path:
    raw = Path(_string(value, name=name)).expanduser()
    return (raw if raw.is_absolute() else profile_path.parent / raw).resolve()


@dataclass(frozen=True)
class RuntimeAnchorPolicy:
    stationary_gyro_max_deg_s: float
    minimum_inliers: int
    consensus_translation_threshold_m: float
    consensus_rotation_threshold_deg: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "stationary_gyro_max_deg_s",
            _number(
                self.stationary_gyro_max_deg_s,
                name="stationary_gyro_max_deg_s",
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            "minimum_inliers",
            _integer(self.minimum_inliers, name="minimum_inliers", minimum=2),
        )
        object.__setattr__(
            self,
            "consensus_translation_threshold_m",
            _number(
                self.consensus_translation_threshold_m,
                name="consensus_translation_threshold_m",
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            "consensus_rotation_threshold_deg",
            _number(
                self.consensus_rotation_threshold_deg,
                name="consensus_rotation_threshold_deg",
                positive=True,
            ),
        )


@dataclass(frozen=True)
class Device3RuntimeProfile:
    profile_id: str
    scope: str
    ego_serial: str
    right_serial: str
    right_imu_path: str
    calibration_path: Path
    calibration_sha256: str
    package_manifest_path: Path
    package_manifest_sha256: str
    mount: TagMount
    policy: RuntimeAnchorPolicy
    runtime_calibration_id: str
    three_device_group_ready: bool

    @property
    def stationary_gyro_max_deg_s(self) -> float:
        return self.policy.stationary_gyro_max_deg_s


def load_device3_runtime_profile(path: Path | str) -> Device3RuntimeProfile:
    """Load and verify the accepted right-only profile and all packaged evidence."""
    profile_path = Path(path).resolve()
    profile = _json_object(profile_path, label="runtime profile")
    if profile.get("schema") != PROFILE_SCHEMA:
        raise RuntimeProfileError("unexpected runtime profile schema")
    if profile.get("enabled") is not True:
        raise RuntimeProfileError("runtime profile is not enabled")
    if profile.get("scope") != "ego_plus_right_only":
        raise RuntimeProfileError("runtime profile is not right-only pair scope")
    if profile.get("three_device_group_ready") is not False:
        raise RuntimeProfileError("right-only profile cannot mark the group ready")
    if profile.get("frame_convention") != "T_A_B maps coordinates in frame B into frame A":
        raise RuntimeProfileError("unexpected frame convention")

    identities = profile.get("identities")
    if not isinstance(identities, dict):
        raise RuntimeProfileError("identities must be an object")
    ego_serial = _string(identities.get("ego_d435i_serial"), name="ego serial")
    right_serial = _string(identities.get("right_d405_serial"), name="right serial")
    if ego_serial != EXPECTED_EGO_SERIAL or right_serial != EXPECTED_RIGHT_SERIAL:
        raise RuntimeProfileError("runtime device identity mismatch")
    right_imu_path = _string(identities.get("right_imu_path"), name="right IMU path")

    calibration_ref = profile.get("accepted_calibration")
    package_ref = profile.get("accepted_package_manifest")
    if not isinstance(calibration_ref, dict) or not isinstance(package_ref, dict):
        raise RuntimeProfileError("accepted calibration references must be objects")
    calibration_path = _path(
        calibration_ref.get("path"), profile_path=profile_path, name="calibration path"
    )
    expected_calibration_hash = _string(
        calibration_ref.get("sha256"), name="calibration SHA-256"
    )
    if not calibration_path.is_file() or _sha256(calibration_path) != expected_calibration_hash:
        raise RuntimeProfileError("calibration SHA-256 mismatch")
    package_path = _path(
        package_ref.get("path"), profile_path=profile_path, name="package manifest path"
    )
    expected_package_hash = _string(
        package_ref.get("sha256"), name="package manifest SHA-256"
    )
    if not package_path.is_file() or _sha256(package_path) != expected_package_hash:
        raise RuntimeProfileError("package manifest SHA-256 mismatch")

    package = _json_object(package_path, label="accepted package manifest")
    if package.get("schema") != PACKAGE_SCHEMA:
        raise RuntimeProfileError("unexpected accepted package schema")
    if package.get("calibration_sha256") != expected_calibration_hash:
        raise RuntimeProfileError("package calibration SHA-256 mismatch")
    packaged_calibration = Path(
        _string(package.get("calibration"), name="packaged calibration path")
    ).resolve()
    if packaged_calibration != calibration_path:
        raise RuntimeProfileError("package points to another calibration")
    _verify_package_files(package)

    calibration = _json_object(calibration_path, label="accepted calibration")
    if calibration.get("schema") != CALIBRATION_SCHEMA:
        raise RuntimeProfileError("unexpected accepted calibration schema")
    if calibration.get("status") != package.get("status"):
        raise RuntimeProfileError("calibration/package status mismatch")
    if calibration.get("activation") != (
        "DEVICE3_RIGHT_MOUNT_ACCEPTED_NOT_INSTALLED_AS_THREE_DEVICE_GROUP"
    ):
        raise RuntimeProfileError("device3 mount is not accepted for pair runtime")
    checks = calibration.get("checks")
    required_checks = (
        "aprime_bprime_model_pass",
        "body_camera_transform_valid_se3",
        "camera_tag_transform_valid_se3",
        "cprime_independent_all_gates_pass",
        "dynamic_world_anchor_hil_pass",
        "operator_confirms_installed_tag_matches_printed_asset",
        "print_pipeline_40mm_actual_size_hash_verified",
        "right_role_tag_identity_matches_group_contract",
    )
    if not isinstance(checks, dict) or not all(checks.get(name) is True for name in required_checks):
        raise RuntimeProfileError("accepted calibration checks are incomplete")
    if checks.get("three_device_group_ready") is not False:
        raise RuntimeProfileError("accepted calibration unexpectedly marks group ready")

    device = calibration.get("device")
    tag = calibration.get("tag")
    transforms = calibration.get("transforms")
    raw_policy = calibration.get("runtime_anchor_policy")
    if not all(isinstance(item, dict) for item in (device, tag, transforms, raw_policy)):
        raise RuntimeProfileError("accepted calibration structure is incomplete")
    if device.get("device_id") != EXPECTED_DEVICE_ID or device.get("d405_serial") != right_serial:
        raise RuntimeProfileError("accepted calibration belongs to another device")
    if tag.get("family") != EXPECTED_TAG_FAMILY or tag.get("id") != EXPECTED_TAG_ID:
        raise RuntimeProfileError("accepted calibration has the wrong tag identity")
    if float(tag.get("nominal_black_square_m", -1.0)) != 0.04:
        raise RuntimeProfileError("accepted calibration has the wrong tag size")

    try:
        tag_from_body = validate_transform(
            np.asarray(transforms["T_mount_tag_from_vins_body"], dtype=np.float64),
            name="T_mount_tag_from_vins_body",
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeProfileError("invalid accepted tag/body transform") from exc
    mount = TagMount(
        device_id=EXPECTED_DEVICE_ID,
        family=EXPECTED_TAG_FAMILY,
        tag_id=EXPECTED_TAG_ID,
        tag_from_gripper=tag_from_body,
        calibration_id=_string(calibration.get("calibration_id"), name="calibration ID"),
    )
    policy = RuntimeAnchorPolicy(
        stationary_gyro_max_deg_s=raw_policy.get("stationary_gyro_max_deg_s"),
        minimum_inliers=raw_policy.get("minimum_inliers"),
        consensus_translation_threshold_m=raw_policy.get(
            "consensus_translation_threshold_m"
        ),
        consensus_rotation_threshold_deg=raw_policy.get(
            "consensus_rotation_threshold_deg"
        ),
    )
    return Device3RuntimeProfile(
        profile_id=_string(profile.get("profile_id"), name="profile ID"),
        scope="ego_plus_right_only",
        ego_serial=ego_serial,
        right_serial=right_serial,
        right_imu_path=right_imu_path,
        calibration_path=calibration_path,
        calibration_sha256=expected_calibration_hash,
        package_manifest_path=package_path,
        package_manifest_sha256=expected_package_hash,
        mount=mount,
        policy=policy,
        runtime_calibration_id=_string(
            device.get("runtime_calibration_id"), name="runtime calibration ID"
        ),
        three_device_group_ready=False,
    )


def _verify_package_files(package: dict) -> None:
    files = package.get("files")
    if not isinstance(files, list) or not files:
        raise RuntimeProfileError("accepted package has no evidence files")
    for item in files:
        if not isinstance(item, dict):
            raise RuntimeProfileError("invalid accepted package file entry")
        evidence_path = Path(_string(item.get("path"), name="evidence path")).resolve()
        expected_hash = _string(item.get("sha256"), name="evidence SHA-256")
        if not evidence_path.is_file() or _sha256(evidence_path) != expected_hash:
            raise RuntimeProfileError(f"accepted evidence SHA-256 mismatch: {evidence_path}")
        expected_bytes = _integer(item.get("bytes"), name="evidence bytes")
        if evidence_path.stat().st_size != expected_bytes:
            raise RuntimeProfileError(f"accepted evidence size mismatch: {evidence_path}")


@dataclass(frozen=True)
class AnchorCandidate:
    timestamp_ns: int
    gyro_norm_deg_s: float
    world_from_odom: np.ndarray

    def __post_init__(self) -> None:
        timestamp = _integer(self.timestamp_ns, name="timestamp_ns")
        gyro = _number(self.gyro_norm_deg_s, name="gyro_norm_deg_s")
        transform = np.array(
            validate_transform(self.world_from_odom, name="world_from_odom"),
            dtype=np.float64,
            copy=True,
        )
        transform.setflags(write=False)
        object.__setattr__(self, "timestamp_ns", timestamp)
        object.__setattr__(self, "gyro_norm_deg_s", gyro)
        object.__setattr__(self, "world_from_odom", transform)


def anchor_candidate_from_synced_observation(
    profile: Device3RuntimeProfile,
    *,
    timestamp_ns: int,
    gyro_norm_deg_s: float,
    world_from_ego_camera: np.ndarray,
    ego_camera_from_mount_tag: np.ndarray,
    right_odom_from_body: np.ndarray,
) -> AnchorCandidate:
    """Compose one synchronized candidate using the accepted frame contract."""
    if not isinstance(profile, Device3RuntimeProfile):
        raise TypeError("profile must be a Device3RuntimeProfile")
    world_from_right_body = compose(
        validate_transform(world_from_ego_camera, name="world_from_ego_camera"),
        validate_transform(
            ego_camera_from_mount_tag, name="ego_camera_from_mount_tag"
        ),
        profile.mount.tag_from_gripper,
    )
    world_from_odom = compose(
        world_from_right_body,
        invert(validate_transform(right_odom_from_body, name="right_odom_from_body")),
    )
    return AnchorCandidate(
        timestamp_ns=timestamp_ns,
        gyro_norm_deg_s=gyro_norm_deg_s,
        world_from_odom=world_from_odom,
    )


@dataclass(frozen=True)
class RuntimeDecision:
    status: str
    reason: str
    world_from_odom: np.ndarray | None
    candidate_count: int
    eligible_count: int
    inlier_count: int

    def __post_init__(self) -> None:
        if self.world_from_odom is not None:
            transform = np.array(self.world_from_odom, dtype=np.float64, copy=True)
            transform.setflags(write=False)
            object.__setattr__(self, "world_from_odom", transform)


class Device3WorldAnchorController:
    """Stateful right-only anchor acceptance and hold policy."""

    def __init__(self, profile: Device3RuntimeProfile):
        if not isinstance(profile, Device3RuntimeProfile):
            raise TypeError("profile must be a Device3RuntimeProfile")
        self.profile = profile
        self._current: AnchorAcceptance | None = None

    @property
    def current(self) -> AnchorAcceptance | None:
        return self._current

    def process_window(self, candidates: Iterable[AnchorCandidate]) -> RuntimeDecision:
        values = tuple(candidates)
        if any(not isinstance(item, AnchorCandidate) for item in values):
            raise TypeError("candidates must contain AnchorCandidate values")
        if len({item.timestamp_ns for item in values}) != len(values):
            raise ValueError("candidate timestamps must be unique")
        if not values:
            return self._no_update("tag_unavailable", 0, 0, 0)
        eligible = tuple(
            item
            for item in values
            if item.gyro_norm_deg_s <= self.profile.policy.stationary_gyro_max_deg_s
        )
        if not eligible:
            return self._no_update("fast_motion", len(values), 0, 0)
        if len(eligible) < self.profile.policy.minimum_inliers:
            return self._no_update(
                "insufficient_inliers", len(values), len(eligible), len(eligible)
            )
        medoid, inliers = _candidate_consensus(eligible, self.profile.policy)
        if len(inliers) < self.profile.policy.minimum_inliers:
            return self._no_update(
                "no_consensus", len(values), len(eligible), len(inliers)
            )
        accepted_at_ns = max(eligible[index].timestamp_ns for index in inliers)
        proposed = AnchorAcceptance.initial(
            device_id=EXPECTED_DEVICE_ID,
            world_from_odom=eligible[medoid].world_from_odom,
            accepted_at_ns=accepted_at_ns,
        )
        if self._current is None:
            self._current = proposed
            return self._decision(
                "INITIALIZED", "accepted", len(values), len(eligible), len(inliers)
            )
        reconciliation = reconcile_anchor(
            self._current,
            proposed,
            max_translation_update_m=(
                self.profile.policy.consensus_translation_threshold_m
            ),
            max_rotation_update_deg=self.profile.policy.consensus_rotation_threshold_deg,
        )
        self._current = reconciliation.state
        return self._decision(
            "UPDATED" if reconciliation.accepted else "HELD",
            reconciliation.reason,
            len(values),
            len(eligible),
            len(inliers),
        )

    def world_from_body(self, odom_from_body: np.ndarray) -> np.ndarray:
        if self._current is None:
            raise RuntimeError("world anchor is not initialized")
        return compose(
            self._current.world_from_odom,
            validate_transform(odom_from_body, name="odom_from_body"),
        )

    def _no_update(
        self, reason: str, candidate_count: int, eligible_count: int, inlier_count: int
    ) -> RuntimeDecision:
        return self._decision(
            "HELD" if self._current is not None else "BLOCKED",
            reason,
            candidate_count,
            eligible_count,
            inlier_count,
        )

    def _decision(
        self,
        status: str,
        reason: str,
        candidate_count: int,
        eligible_count: int,
        inlier_count: int,
    ) -> RuntimeDecision:
        return RuntimeDecision(
            status=status,
            reason=reason,
            world_from_odom=(
                None if self._current is None else self._current.world_from_odom
            ),
            candidate_count=candidate_count,
            eligible_count=eligible_count,
            inlier_count=inlier_count,
        )


def evaluate_device3_runtime_preflight(
    profile: Device3RuntimeProfile,
    *,
    inventory: Iterable[dict],
    imu_available: bool,
    free_bytes: int,
    required_free_bytes: int,
    active_holders: Iterable[str],
) -> dict:
    """Evaluate a read-only hardware inventory before bounded live capture."""
    if not isinstance(profile, Device3RuntimeProfile):
        raise TypeError("profile must be a Device3RuntimeProfile")
    devices = tuple(inventory)
    if any(not isinstance(item, dict) for item in devices):
        raise TypeError("inventory must contain dictionaries")
    if isinstance(imu_available, np.bool_):
        imu_available = bool(imu_available)
    if not isinstance(imu_available, bool):
        raise TypeError("imu_available must be boolean")
    available = _integer(free_bytes, name="free_bytes")
    required = _integer(required_free_bytes, name="required_free_bytes", minimum=1)
    holders = tuple(str(item) for item in active_holders)

    ego_matches = [item for item in devices if item.get("serial") == profile.ego_serial]
    right_matches = [
        item for item in devices if item.get("serial") == profile.right_serial
    ]
    ego = ego_matches[0] if len(ego_matches) == 1 else None
    right = right_matches[0] if len(right_matches) == 1 else None
    checks = {
        "ego_identity": bool(
            ego is not None and "D435I" in str(ego.get("name", "")).upper()
        ),
        "right_identity": bool(
            right is not None and "D405" in str(right.get("name", "")).upper()
        ),
        "ego_usb3": bool(ego is not None and _is_usb3(ego.get("usb_type"))),
        "right_usb3": bool(right is not None and _is_usb3(right.get("usb_type"))),
        "right_imu_available": imu_available,
        "storage_headroom": available >= required,
        "no_active_device_holders": not holders,
    }
    return {
        "schema": "three-device-slam.device3-world-runtime-preflight.v1",
        "status": (
            "PASS_READY_FOR_BOUNDED_LIVE_CAPTURE"
            if all(checks.values())
            else "BLOCKED_HARDWARE_PREFLIGHT"
        ),
        "scope": profile.scope,
        "profile_id": profile.profile_id,
        "checks": checks,
        "devices": {
            "ego": ego,
            "right": right,
            "right_imu_path": profile.right_imu_path,
        },
        "storage": {
            "free_bytes": available,
            "required_free_bytes": required,
        },
        "active_holders": list(holders),
        "three_device_group": "DEFERRED",
    }


def _is_usb3(value) -> bool:
    if not isinstance(value, str):
        return False
    try:
        return int(value.split(".", 1)[0]) >= 3
    except ValueError:
        return False


def _candidate_consensus(
    candidates: tuple[AnchorCandidate, ...], policy: RuntimeAnchorPolicy
) -> tuple[int, tuple[int, ...]]:
    best_key: tuple[int, float, int, int] | None = None
    best_index = 0
    best_inliers: tuple[int, ...] = ()
    for index, candidate in enumerate(candidates):
        inliers = []
        cost = 0.0
        for other_index, other in enumerate(candidates):
            error = pose_error(other.world_from_odom, candidate.world_from_odom)
            if (
                error.translation_m <= policy.consensus_translation_threshold_m
                and error.rotation_deg <= policy.consensus_rotation_threshold_deg
            ):
                inliers.append(other_index)
                cost += (
                    error.translation_m / policy.consensus_translation_threshold_m
                    + error.rotation_deg / policy.consensus_rotation_threshold_deg
                )
        key = (-len(inliers), cost, candidate.timestamp_ns, index)
        if best_key is None or key < best_key:
            best_key = key
            best_index = index
            best_inliers = tuple(inliers)
    return best_index, best_inliers


def replay_retained_constraints(
    profile_path: Path | str,
    constraints_path: Path | str,
    output: Path | str,
    *,
    right_odometry_path: Path | str | None = None,
) -> dict:
    """Replay the retained run24 candidates through the product state machine."""
    output_path = Path(output)
    if output_path.exists():
        raise FileExistsError(f"runtime replay output already exists: {output_path}")
    profile_source = Path(profile_path).resolve()
    constraints_source = Path(constraints_path).resolve()
    profile = load_device3_runtime_profile(profile_source)
    rows = []
    try:
        with constraints_source.open() as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                row = json.loads(line)
                rows.append((line_number, row))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read retained constraints: {constraints_source}") from exc
    if not rows:
        raise ValueError("retained constraints are empty")

    by_phase: dict[str, list[AnchorCandidate]] = {}
    for line_number, row in rows:
        try:
            phase = _string(row["phase"], name="constraint phase")
            item = AnchorCandidate(
                timestamp_ns=int(row["ego_stamp_ns"]),
                gyro_norm_deg_s=float(row["gyro_norm_deg_s"]),
                world_from_odom=np.asarray(row["mount_anchor"], dtype=np.float64),
            )
        except (KeyError, TypeError, ValueError, RuntimeProfileError) as exc:
            raise ValueError(f"invalid retained constraint at line {line_number}") from exc
        by_phase.setdefault(phase, []).append(item)

    controller = Device3WorldAnchorController(profile)
    initialization = controller.process_window(by_phase.get("initial_static", ()))
    if initialization.world_from_odom is None:
        initial_anchor = None
    else:
        initial_anchor = initialization.world_from_odom.copy()
    initial_acceptance = controller.current
    moving_candidates = tuple(
        item
        for phase in ("translation", "rotation")
        for item in by_phase.get(phase, ())
        if item.gyro_norm_deg_s > profile.policy.stationary_gyro_max_deg_s
    )
    fast_motion = controller.process_window(moving_candidates)
    occlusion = controller.process_window(())
    reacquisition = controller.process_window(by_phase.get("return_final", ()))
    final_acceptance = controller.current

    if initial_anchor is not None and reacquisition.world_from_odom is not None:
        endpoint = pose_error(reacquisition.world_from_odom, initial_anchor)
        endpoint_payload = {
            "translation_m": endpoint.translation_m,
            "rotation_deg": endpoint.rotation_deg,
        }
    else:
        endpoint_payload = {"translation_m": None, "rotation_deg": None}
    holds_preserve_anchor = bool(
        initial_anchor is not None
        and fast_motion.world_from_odom is not None
        and occlusion.world_from_odom is not None
        and np.array_equal(fast_motion.world_from_odom, initial_anchor)
        and np.array_equal(occlusion.world_from_odom, initial_anchor)
    )
    checks = {
        "initializes_from_stationary_consensus": initialization.status == "INITIALIZED",
        "fast_motion_holds_anchor": fast_motion.status == "HELD",
        "tag_loss_holds_anchor": occlusion.status == "HELD",
        "holds_preserve_exact_anchor": holds_preserve_anchor,
        "reacquisition_updates_from_consensus": reacquisition.status == "UPDATED",
        "endpoint_translation_within_30mm": (
            endpoint_payload["translation_m"] is not None
            and endpoint_payload["translation_m"]
            <= profile.policy.consensus_translation_threshold_m
        ),
        "endpoint_rotation_within_3deg": (
            endpoint_payload["rotation_deg"] is not None
            and endpoint_payload["rotation_deg"]
            <= profile.policy.consensus_rotation_threshold_deg
        ),
        "three_device_group_remains_deferred": not profile.three_device_group_ready,
    }
    trajectory_rows = None
    trajectory_summary = None
    odometry_source = None
    if right_odometry_path is not None:
        odometry_source = Path(right_odometry_path).resolve()
        expected_odometry_hash = _accepted_odometry_hash(
            profile, odometry_source
        )
        if _sha256(odometry_source) != expected_odometry_hash:
            raise ValueError("right odometry SHA-256 mismatch")
        if initial_acceptance is None or final_acceptance is None:
            raise ValueError("cannot export trajectory without accepted anchors")
        trajectory_rows, epoch_counts, trajectory_metrics = _world_trajectory_rows(
            odometry_source,
            initial_anchor=initial_acceptance.world_from_odom,
            reacquired_anchor=final_acceptance.world_from_odom,
            reacquired_at_ns=final_acceptance.accepted_at_ns,
        )
        trajectory_summary = {
            "rows": len(trajectory_rows),
            "anchor_epochs": epoch_counts,
            **trajectory_metrics,
            "reacquired_at_ns": final_acceptance.accepted_at_ns,
            "initial_anchor_backfilled_before_live_acceptance": True,
            "source_sha256": expected_odometry_hash,
        }
        checks["world_trajectory_exported"] = bool(
            trajectory_rows
            and sum(epoch_counts.values()) == len(trajectory_rows)
            and epoch_counts["initial"] > 0
            and epoch_counts["reacquired"] > 0
        )
        checks["world_trajectory_continuous"] = bool(
            trajectory_metrics["anchor_switch_count"] == 1
            and trajectory_metrics["anchor_switch_position_step_m"]
            <= profile.policy.consensus_translation_threshold_m
            and trajectory_metrics["max_position_step_m"] <= 0.1
        )
    status = (
        "PASS_RETAINED_RUNTIME_REPLAY"
        if all(checks.values())
        else "FAIL_RETAINED_RUNTIME_REPLAY"
    )
    sources = {
        str(profile_source): _sha256(profile_source),
        str(constraints_source): _sha256(constraints_source),
        str(profile.calibration_path): profile.calibration_sha256,
        str(profile.package_manifest_path): profile.package_manifest_sha256,
    }
    if odometry_source is not None:
        sources[str(odometry_source)] = _sha256(odometry_source)
    report = {
        "schema": "three-device-slam.device3-world-runtime-replay.v1",
        "status": status,
        "scope": profile.scope,
        "profile_id": profile.profile_id,
        "three_device_group": "DEFERRED",
        "frame_convention": "T_A_B maps coordinates in frame B into frame A",
        "policy": {
            "stationary_gyro_max_deg_s": profile.policy.stationary_gyro_max_deg_s,
            "minimum_inliers": profile.policy.minimum_inliers,
            "consensus_translation_threshold_m": (
                profile.policy.consensus_translation_threshold_m
            ),
            "consensus_rotation_threshold_deg": (
                profile.policy.consensus_rotation_threshold_deg
            ),
        },
        "input": {
            "rows": len(rows),
            "phase_counts": {
                phase: len(values) for phase, values in sorted(by_phase.items())
            },
        },
        "decisions": {
            "initialization": _decision_payload(initialization),
            "fast_motion": _decision_payload(fast_motion),
            "occlusion": _decision_payload(occlusion),
            "reacquisition": _decision_payload(reacquisition),
        },
        "endpoint_update": endpoint_payload,
        "world_trajectory": trajectory_summary,
        "checks": checks,
        "limitations": [
            "Retained-data replay validates the runtime decision path; it is not a new live HIL run.",
            "Only Ego plus device3/right is enabled; device2/left remains unavailable.",
        ],
        "sources": sources,
    }
    output_path.mkdir(parents=True)
    written_files = {}
    if trajectory_rows is not None:
        trajectory_path = output_path / "right_world_trajectory.csv"
        with trajectory_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["timestamp_ns", "x", "y", "z", "qw", "qx", "qy", "qz", "anchor_epoch"]
            )
            writer.writerows(trajectory_rows)
        payload = trajectory_path.read_bytes()
        written_files[trajectory_path.name] = {
            "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    report_path = output_path / "runtime_replay_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    report_bytes = report_path.read_bytes()
    written_files[report_path.name] = {
        "bytes": len(report_bytes),
        "sha256": hashlib.sha256(report_bytes).hexdigest(),
    }
    manifest = {
        "schema": "three-device-slam.device3-world-runtime-replay-package.v1",
        "status": status,
        "files": written_files,
        "sources": report["sources"],
    }
    (output_path / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    return report


def _accepted_odometry_hash(
    profile: Device3RuntimeProfile, odometry_path: Path
) -> str:
    package = _json_object(
        profile.package_manifest_path, label="accepted package manifest"
    )
    reports = [
        Path(item["path"]).resolve()
        for item in package["files"]
        if isinstance(item, dict)
        and str(item.get("path", "")).endswith("/anchor_hil_v2/report.json")
    ]
    if len(reports) != 1:
        raise ValueError("accepted package does not identify one anchor HIL report")
    report = _json_object(reports[0], label="accepted anchor HIL report")
    provenance = report.get("provenance")
    if not isinstance(provenance, dict):
        raise ValueError("accepted anchor HIL report has no provenance")
    expected = provenance.get(str(odometry_path))
    if not isinstance(expected, str):
        raise ValueError("right odometry is not bound by accepted HIL provenance")
    return expected


def _world_trajectory_rows(
    odometry_path: Path,
    *,
    initial_anchor: np.ndarray,
    reacquired_anchor: np.ndarray,
    reacquired_at_ns: int,
) -> tuple[list[list], dict[str, int], dict[str, float | int]]:
    from .covins_export import load_pair_vins_poses

    stamps, local_poses = load_pair_vins_poses(odometry_path)
    rows = []
    counts = {"initial": 0, "reacquired": 0}
    world_positions = []
    epochs = []
    for stamp, local_pose in zip(stamps, local_poses):
        epoch = "reacquired" if stamp >= reacquired_at_ns else "initial"
        anchor = reacquired_anchor if epoch == "reacquired" else initial_anchor
        world_pose = compose(anchor, local_pose)
        world_positions.append(world_pose[:3, 3].copy())
        epochs.append(epoch)
        qw, qx, qy, qz = _quaternion_wxyz(world_pose[:3, :3])
        rows.append([
            stamp,
            *world_pose[:3, 3].tolist(),
            qw,
            qx,
            qy,
            qz,
            epoch,
        ])
        counts[epoch] += 1
    steps = np.linalg.norm(np.diff(np.asarray(world_positions), axis=0), axis=1)
    switches = [
        index for index in range(1, len(epochs)) if epochs[index] != epochs[index - 1]
    ]
    switch_step = (
        float(np.linalg.norm(world_positions[switches[0]] - world_positions[switches[0] - 1]))
        if len(switches) == 1
        else math.inf
    )
    metrics = {
        "max_position_step_m": float(np.max(steps)),
        "anchor_switch_count": len(switches),
        "anchor_switch_position_step_m": switch_step,
    }
    return rows, counts, metrics


def _quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    values = np.asarray((qw, qx, qy, qz), dtype=np.float64)
    values /= np.linalg.norm(values)
    if values[0] < 0.0:
        values *= -1.0
    return tuple(float(value) for value in values)


def _decision_payload(decision: RuntimeDecision) -> dict:
    return {
        "status": decision.status,
        "reason": decision.reason,
        "candidate_count": decision.candidate_count,
        "eligible_count": decision.eligible_count,
        "inlier_count": decision.inlier_count,
        "T_world_right_odom": (
            None
            if decision.world_from_odom is None
            else decision.world_from_odom.tolist()
        ),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Replay retained device3 world-anchor constraints"
    )
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--constraints", required=True, type=Path)
    parser.add_argument("--right-odometry", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = replay_retained_constraints(
            args.profile,
            args.constraints,
            args.output,
            right_odometry_path=args.right_odometry,
        )
    except (FileExistsError, RuntimeProfileError, ValueError) as exc:
        print(f"FAIL/{exc}")
        return 2
    print(json.dumps({
        "status": report["status"],
        "scope": report["scope"],
        "three_device_group": report["three_device_group"],
        "endpoint_update": report["endpoint_update"],
        "checks": report["checks"],
    }, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS_RETAINED_RUNTIME_REPLAY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
