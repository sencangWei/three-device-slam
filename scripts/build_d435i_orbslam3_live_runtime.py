#!/usr/bin/env python3
"""Build a reproducible live stdin adapter around the pinned ORB-SLAM3 library."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_COMMIT = "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4"
DEFAULT_BASE_RUNTIME = ROOT / "artifacts/runtime/orbslam3-4452a3c4-d435i-init"
DEFAULT_OUTPUT = ROOT / "artifacts/runtime/orbslam3-4452a3c4-d435i-live-v5"
DEFAULT_PANGOLIN_PREFIX = Path(
    "/tmp/pangolin_orb_probe_20260915_UUSFiQ/install"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def command_output(command) -> str:
    return subprocess.check_output(command, text=True).strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--orb-repository", type=Path, required=True)
    parser.add_argument("--base-runtime", type=Path, default=DEFAULT_BASE_RUNTIME)
    parser.add_argument("--pangolin-prefix", type=Path, default=DEFAULT_PANGOLIN_PREFIX)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    repository = args.orb_repository.resolve()
    base_runtime = args.base_runtime.resolve()
    output = args.output.resolve()
    pangolin_prefix = args.pangolin_prefix.resolve()
    patch = ROOT / "patches/orbslam3-d435i-live-v5.patch"
    source = ROOT / "native/orbslam3_live_adapter/main.cc"
    if output.exists():
        raise FileExistsError(output)
    if not (pangolin_prefix / "include/pangolin/pangolin.h").is_file():
        raise FileNotFoundError("Pangolin development headers are missing")
    if command_output(["git", "-C", str(repository), "rev-parse", "HEAD"]) != EXPECTED_COMMIT:
        raise RuntimeError("ORB-SLAM3 repository is not at the pinned commit")
    base_manifest_path = base_runtime / "runtime_manifest.json"
    base_manifest = json.loads(base_manifest_path.read_text(encoding="utf-8"))
    for relative, expected in base_manifest["files"].items():
        candidate = base_runtime / relative
        if not candidate.is_file() or sha256(candidate) != expected:
            raise RuntimeError(f"base runtime hash mismatch: {relative}")

    worktree = Path(tempfile.mkdtemp(prefix="orbslam3-live-build-"))
    worktree.rmdir()
    try:
        subprocess.run(
            ["git", "-C", str(repository), "worktree", "add", "--quiet", "--detach", str(worktree), EXPECTED_COMMIT],
            check=True,
        )
        subprocess.run(["git", "-C", str(worktree), "apply", str(patch)], check=True)
        dbow_library_dir = worktree / "Thirdparty/DBoW2/lib"
        dbow_library_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(base_runtime / "lib/libDBoW2.so", dbow_library_dir / "libDBoW2.so")
        g2o_library_dir = worktree / "Thirdparty/g2o/lib"
        g2o_library_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(base_runtime / "lib/libg2o.so", g2o_library_dir / "libg2o.so")
        build_dir = worktree / "build"
        subprocess.run(
            [
                "cmake", "-S", str(worktree), "-B", str(build_dir),
                "-DCMAKE_BUILD_TYPE=Release",
                f"-DCMAKE_PREFIX_PATH={pangolin_prefix}",
                "-DBUILD_EXAMPLES=OFF",
            ],
            check=True,
        )
        subprocess.run(
            ["cmake", "--build", str(build_dir), "--target", "ORB_SLAM3", "-j8"],
            check=True,
        )
        shutil.copytree(base_runtime, output)
        patched_orb_library = worktree / "lib/libORB_SLAM3.so"
        if not patched_orb_library.is_file():
            raise FileNotFoundError("patched ORB-SLAM3 library is missing")
        shutil.copy2(patched_orb_library, output / "lib/libORB_SLAM3.so")
        binary = output / "bin/orbslam3_live_adapter"
        opencv = shlex.split(command_output(["pkg-config", "--cflags", "--libs", "opencv4"]))
        command = [
            "g++", "-std=c++17", "-O3", "-DNDEBUG", "-pthread",
            f"-I{worktree}",
            f"-I{worktree / 'include'}",
            f"-I{worktree / 'include/CameraModels'}",
            f"-I{worktree / 'Thirdparty/Sophus'}",
            f"-I{pangolin_prefix / 'include'}",
            "-I/usr/include/eigen3",
            str(source),
            f"-L{output / 'lib'}",
            "-lORB_SLAM3", "-lDBoW2", "-lg2o", "-lpangolin",
            "-lOpenGL", "-lGLEW",
            "-Wl,-rpath,$ORIGIN/../lib",
            "-o", str(binary),
            *opencv,
        ]
        subprocess.run(command, check=True)
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = str(output / "lib")
        linkage = command_output(["ldd", str(binary)])
        if "not found" in linkage:
            raise RuntimeError(f"unresolved live adapter dependency:\n{linkage}")
        files = dict(base_manifest["files"])
        files["lib/libORB_SLAM3.so"] = sha256(output / "lib/libORB_SLAM3.so")
        files["bin/orbslam3_live_adapter"] = sha256(binary)
        manifest = {
            "schema": "three-device-slam.orbslam3-live-runtime.v2",
            "orbslam3_commit": EXPECTED_COMMIT,
            "base_runtime_manifest_sha256": sha256(base_manifest_path),
            "patch": str(patch.relative_to(ROOT)),
            "patch_sha256": sha256(patch),
            "patch_scope": [
                "fix Rectified settings logger null dereference",
                "make early IMU translation thresholds configurable",
                "wait for ORB worker threads before Atlas and trajectory serialization",
                "guard missing per-frame IMU preintegration and use visual motion fallback",
                "reject non-finite or nonphysical IMU bias corrections before Sophus SO3 exp",
                "retain the last sane frame/keyframe bias when inertial optimization diverges",
                "require sustained low motion before resetting early inertial initialization",
                "expose inertial BA2 readiness to the live adapter",
            ],
            "adapter_source": str(source.relative_to(ROOT)),
            "adapter_source_sha256": sha256(source),
            "protocol": "ORBI/ORBP packed little-endian v2",
            "pose_wire_bytes": 92,
            "camera_owner": "external single RSUSB process",
            "build_host": {"platform": platform.platform(), "opencv": "4.5.4"},
            "files": files,
            "scope": "D435i live stereo-inertial TrackStereo adapter; HIL pending",
        }
        (output / "runtime_manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(manifest, indent=2))
        return 0
    except Exception:
        if output.exists():
            shutil.rmtree(output)
        raise
    finally:
        subprocess.run(
            ["git", "-C", str(repository), "worktree", "remove", "--force", str(worktree)],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


if __name__ == "__main__":
    raise SystemExit(main())
