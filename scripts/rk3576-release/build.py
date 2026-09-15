#!/usr/bin/env python3
"""Build a self-contained collector application release from explicit sources."""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--runtime", type=Path, required=True)
parser.add_argument("--license", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--version", default="0.1.0")
args = parser.parse_args()
if not re.fullmatch(r"\d+\.\d+\.\d+", args.version):
    parser.error("version must have three numeric components")
source = Path(__file__).resolve().parents[2]
target = args.output.resolve() / ("rk3576-umi-" + args.version)
target.mkdir(parents=True, exist_ok=False)
files = [
    "three_device_slam/__init__.py", "three_device_slam/devices/__init__.py",
    "three_device_slam/devices/d405_umi/__init__.py",
    "three_device_slam/devices/d405_umi/imu/__init__.py",
    "three_device_slam/devices/d405_umi/imu/stream_protocol.py",
]
files += [str(p.relative_to(source)) for p in sorted((source / "three_device_slam/edge_rk3576").glob("*.py"))]
for relative in files:
    destination = target / "app" / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source / relative, destination)
for name in ("umi-record", "umi-validate"):
    (target / "bin").mkdir(exist_ok=True)
    shutil.copy2(Path(__file__).parent / name, target / "bin" / name)
    (target / "bin" / name).chmod(0o755)
for name in ("install.sh", "check-host.sh", "99-umi-devices.rules", "README.md"):
    shutil.copy2(Path(__file__).parent / name, target / name)
(target / "runtime").mkdir()
for name in ("pyrealsense2.cpython-312-aarch64-linux-gnu.so", "pyrsutils.cpython-312-aarch64-linux-gnu.so"):
    shutil.copy2(args.runtime / name, target / "runtime" / name)
shutil.copy2(args.license, target / "runtime/LICENSE-librealsense")
(target / "tests").mkdir()
for name in ("test_rk3576_device_discovery.py", "test_rk3576_edge_capture.py", "test_rk3576_rsusb_probe.py"):
    shutil.copy2(source / "tests" / name, target / "tests" / name)
entries = []
for path in sorted(target.rglob("*")):
    if path.is_file():
        entries.append({"path": str(path.relative_to(target)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
manifest = target / "release-manifest.json"
manifest.write_text(json.dumps({"version": args.version, "type": "collector-application",
    "architecture": "aarch64", "python": "3.12", "os": "Ubuntu 24.04 with RK3576 MPP",
    "rsusb": {"version": "2.58.2", "FORCE_RSUSB_BACKEND": True},
    "device_selection": "required D405 SDK and USB serial arguments; optional STM32 by-id path",
    "files": entries}, indent=2) + "\n")
entries.append({"path": "release-manifest.json", "sha256": hashlib.sha256(manifest.read_bytes()).hexdigest()})
(target / "SHA256SUMS").write_text("".join(f"{e['sha256']}  {e['path']}\n" for e in entries))
print(target)
