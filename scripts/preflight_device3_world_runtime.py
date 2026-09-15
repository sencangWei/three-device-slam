"""Read-only hardware preflight for the accepted Ego + device3 runtime."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat

from three_device_slam.acquisition.rsusb_pair import required_capture_free_bytes
from three_device_slam.spatial.device3_world_runtime import (
    evaluate_device3_runtime_preflight,
    load_device3_runtime_profile,
)


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def inventory_realsense() -> list[dict]:
    import pyrealsense2 as rs

    rows = []
    for device in rs.context().query_devices():
        def info(key):
            return device.get_info(key) if device.supports(key) else None

        rows.append({
            "name": info(rs.camera_info.name),
            "serial": info(rs.camera_info.serial_number),
            "firmware": info(rs.camera_info.firmware_version),
            "physical_port": info(rs.camera_info.physical_port),
            "usb_type": info(rs.camera_info.usb_type_descriptor),
            "product_id": info(rs.camera_info.product_id),
        })
    return rows


def device_holders(paths: list[Path]) -> list[str]:
    targets = {str(path.resolve()) for path in paths if path.exists()}
    holders = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        fd_root = process / "fd"
        try:
            descriptors = tuple(fd_root.iterdir())
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                target = str(descriptor.resolve())
            except OSError:
                continue
            if target not in targets:
                continue
            try:
                command = (process / "comm").read_text().strip()
            except OSError:
                command = "unknown"
            holders.append(f"pid={process.name} command={command} device={target}")
    return sorted(set(holders))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--storage-root",
        type=Path,
        default=Path("/home/robot/three-device-slam/artifacts/product_sessions"),
    )
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--start-delay", type=float, default=20.0)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(f"preflight output already exists: {args.output}")
    if args.duration <= 0 or args.start_delay <= 0:
        raise ValueError("duration and start delay must be positive")

    profile_path = args.profile.resolve()
    profile = load_device3_runtime_profile(profile_path)
    imu_path = Path(profile.right_imu_path)
    imu_available = bool(
        imu_path.exists() and stat.S_ISCHR(imu_path.resolve().stat().st_mode)
    )
    video_paths = list(Path("/dev").glob("video*"))
    holders = device_holders(video_paths + [imu_path])
    storage_root = args.storage_root.resolve()
    free_bytes = shutil.disk_usage(storage_root).free
    required_bytes = required_capture_free_bytes(
        args.duration, args.start_delay, 1
    )
    report = evaluate_device3_runtime_preflight(
        profile,
        inventory=inventory_realsense(),
        imu_available=imu_available,
        free_bytes=free_bytes,
        required_free_bytes=required_bytes,
        active_holders=holders,
    )
    report.update({
        "observed_at_utc": datetime.now(timezone.utc).isoformat(),
        "capture_contract": {
            "mode": "device3_anchor_hil",
            "formal_duration_s": args.duration,
            "start_delay_s": args.start_delay,
            "preview_hz": 5.0,
        },
        "sources": {
            str(profile_path): digest(profile_path),
            str(profile.calibration_path): profile.calibration_sha256,
            str(profile.package_manifest_path): profile.package_manifest_sha256,
        },
    })
    args.output.mkdir(parents=True)
    report_path = args.output / "preflight_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    report_bytes = report_path.read_bytes()
    manifest = {
        "schema": "three-device-slam.device3-world-runtime-preflight-package.v1",
        "status": report["status"],
        "files": {
            "preflight_report.json": {
                "bytes": len(report_bytes),
                "sha256": hashlib.sha256(report_bytes).hexdigest(),
            }
        },
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps({
        "status": report["status"],
        "checks": report["checks"],
        "devices": report["devices"],
        "storage": report["storage"],
        "active_holders": report["active_holders"],
        "evidence": str(report_path),
    }, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS_READY_FOR_BOUNDED_LIVE_CAPTURE" else 3


if __name__ == "__main__":
    raise SystemExit(main())
