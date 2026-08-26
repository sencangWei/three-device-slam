import os
import shutil
import stat
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = (
    ROOT / "run_three_device_slam.sh",
    ROOT / "scripts" / "install_ubuntu.sh",
    ROOT / "scripts" / "verify_ubuntu_environment.sh",
)


def test_shell_entries_are_user_executable():
    if os.name != "posix":
        pytest.skip("executable mode is a POSIX deployment invariant")
    for path in SCRIPTS:
        assert path.stat().st_mode & stat.S_IXUSR


def test_shell_entries_use_absolute_bash_interpreter():
    for path in SCRIPTS:
        assert path.read_text(encoding="utf-8").splitlines()[0] == "#!/bin/bash"


def test_launcher_uses_isolated_absolute_installed_python(tmp_path):
    text = (ROOT / "run_three_device_slam.sh").read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    installed_python = "/opt/three-device-slam/venv/bin/python"
    assert f'exec {installed_python} -I -B -m three_device_slam.cli "$@"' in text
    assert "D405-MAXIMU" not in text

    fake_python = tmp_path / "fake-python"
    arguments = tmp_path / "arguments.txt"
    fake_python.write_text(
        '#!/usr/bin/env bash\nprintf \'%s\\n\' "$@" >"$ARGUMENTS_FILE"\n',
        encoding="utf-8",
    )
    fake_python.chmod(0o755)
    launcher = tmp_path / "launcher.sh"
    launcher.write_text(text.replace(installed_python, str(fake_python)), encoding="utf-8")
    launcher.chmod(0o755)

    result = subprocess.run(
        [str(launcher), "product", "--config", "/tmp/config.json"],
        cwd=tmp_path,
        env={**os.environ, "ARGUMENTS_FILE": str(arguments)},
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert arguments.read_text(encoding="utf-8").splitlines() == [
        "-I",
        "-B",
        "-m",
        "three_device_slam.cli",
        "product",
        "--config",
        "/tmp/config.json",
    ]


def test_installer_has_frozen_platform_and_bridge_checks_without_product_config():
    text = (ROOT / "scripts" / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    assert "FAIL/root_required" in text
    assert "FAIL/untrusted_installation" in text
    assert 'require_trusted_tree "$install_root"' in text
    assert "FAIL/tool_missing:" in text
    assert "ubuntu" in text and "22.04" in text and "humble" in text
    assert "BLOCKED/2uq2_vendor_sdk_missing" in text
    assert 'VENDOR_SDK="$vendor_sdk" CC=cc RM=rm' in text
    assert "--system-site-packages" in text
    assert 'pip install "$repo_root"' in text
    assert "libtwo_uq2_xu.so" in text
    for symbol in ("ylx_open", "ylx_read_imu27", "ylx_close"):
        assert symbol in text
    assert "prepared_bridge_hash=$(sha256sum" in text
    assert 'validate_bridge "${temporary_lib}/libtwo_uq2_xu.so"' in text
    assert 'require_trusted_tree "${install_root}/venv"' in text
    assert "umask 0022" in text
    assert 'prepared_bridge_hash=$(sha256sum "${temporary_lib}/libtwo_uq2_xu.so"' in text
    assert "installed_hash=$(sha256sum" in text
    assert "${bridge}.sha256" in text
    assert 'mktemp -d "${install_root}/.lib.' in text
    assert 'mv -T --no-clobber "$temporary_lib" "$lib"' in text
    assert 'ensure_root_directory "$install_root"' in text
    assert "ensure_root_directory /run/three-device-slam" in text
    assert "acquire_install_lock /run/three-device-slam/install.lock" in text
    assert "ensure_root_directory /etc/three-device-slam" in text
    assert "require_trusted_tree /etc/three-device-slam" in text
    assert 'install -d -o root -g root -m 0755 "$target"' in text
    assert "ensure_session_directory /var/lib/three-device-slam/sessions" in text
    assert 'install -d -o robot -g robot -m 0750 "$target"' in text
    assert "product.json" not in text


def test_environment_verifier_is_read_only_and_checks_frozen_invariants():
    text = (ROOT / "scripts" / "verify_ubuntu_environment.sh").read_text(
        encoding="utf-8"
    )
    assert "set -euo pipefail" in text
    assert "ubuntu" in text and "22.04" in text and "humble" in text
    assert "PASS/ubuntu_environment" in text
    assert "libtwo_uq2_xu.so" in text
    for symbol in ("ylx_open", "ylx_read_imu27", "ylx_close"):
        assert symbol in text
    assert '"$python" -I -B -c' in text
    assert 'manifest="${bridge}.sha256"' in text
    assert 'validate_bridge_manifest "$bridge" "$manifest" "$bridge"' in text
    assert "FAIL/untrusted_installation" in text
    assert 'require_trusted_tree "$install_root"' in text
    assert "require_trusted_tree /etc/three-device-slam" in text
    assert "FAIL/tool_missing:" in text
    for write_command in ("install ", "mkdir ", "touch ", "rm ", "mv ", "cp "):
        assert write_command not in text


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
@pytest.mark.parametrize(
    "script", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
def test_ros_setup_tolerates_unset_ament_trace_and_restores_nounset(
    tmp_path, script
):
    setup = tmp_path / "setup.bash"
    setup.write_text(
        ': "${AMENT_TRACE_SETUP_FILES}"\nexport ROS_DISTRO=humble\n',
        encoding="utf-8",
    )
    path = ROOT / "scripts" / script
    shell = f'''
source "{path}"
unset AMENT_TRACE_SETUP_FILES
source_ros_setup "{setup}"
[[ $ROS_DISTRO == humble ]]
case $- in *u*) ;; *) exit 7 ;; esac
'''

    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr


@pytest.mark.skipif(
    shutil.which("make") is None or shutil.which("rm") is None,
    reason="make and rm are required",
)
def test_bridge_clean_is_idempotent_with_frozen_installer_rm(tmp_path):
    bridge = tmp_path / "bridge"
    bridge.mkdir()
    shutil.copy(ROOT / "native" / "2uq2_xu_bridge" / "Makefile", bridge)

    result = subprocess.run(
        ["make", "-C", str(bridge), "clean", "RM=rm"],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_bridge_validation_accepts_portable_sha256sum_output(tmp_path):
    artifact = tmp_path / "bridge.so"
    artifact.write_bytes(b"bridge")
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
file() {{ printf '%s: ELF 64-bit LSB shared object, x86-64\n' "$1"; }}
nm() {{
  printf '0000000000000000 T ylx_open\n'
  printf '0000000000000000 T ylx_read_imu27\n'
  printf '0000000000000000 T ylx_close\n'
}}
sha256sum() {{ printf '%064d  %s\n' 0 "$1"; }}
awk() {{
  [[ $1 != *'{{64}}'* ]] || return 2
  command awk "$@"
}}
validate_bridge "{artifact}"
'''

    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr


def test_package_metadata_builds_with_ubuntu_host_backend(tmp_path):
    source = tmp_path / "source"
    output = tmp_path / "wheelhouse"
    source.mkdir()
    output.mkdir()
    shutil.copy(ROOT / "pyproject.toml", source)
    setup_cfg = ROOT / "setup.cfg"
    if setup_cfg.exists():
        shutil.copy(setup_cfg, source)
    shutil.copytree(ROOT / "three_device_slam", source / "three_device_slam")

    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "pip",
            "wheel",
            "--no-build-isolation",
            "--no-deps",
            str(source),
            "-w",
            str(output),
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert [path.name for path in output.iterdir()] == [
        "three_device_slam-0.1.0-py3-none-any.whl"
    ]


def test_package_metadata_declares_yaml_runtime_dependency():
    text = (ROOT / "setup.cfg").read_text(encoding="utf-8")

    assert "PyYAML>=5.4" in text


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
@pytest.mark.parametrize(
    ("failed_import", "reason"),
    [
        ("yaml", "FAIL/python_dependency_missing:yaml"),
        ("cv2", "FAIL/python_dependency_missing:cv2"),
        ("pyrealsense2", "FAIL/python_dependency_missing:pyrealsense2"),
        ("gi", "FAIL/python_dependency_missing:gstreamer"),
    ],
)
@pytest.mark.parametrize(
    "script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
def test_capture_dependency_failure_has_stable_classification(
    tmp_path, script_name, failed_import, reason
):
    install_root = tmp_path / "install"
    module = (
        install_root
        / "venv"
        / "lib"
        / "python3.10"
        / "site-packages"
        / "pyrealsense2.cpython-310-x86_64-linux-gnu.so"
    )
    module.parent.mkdir(parents=True)
    module.write_bytes(b"module")
    python = tmp_path / "python"
    python.write_text(
        """#!/bin/bash
case "$*" in
  *"import yaml"*) [[ $FAILED_IMPORT != yaml ]] ;;
  *"import cv2"*) [[ $FAILED_IMPORT != cv2 ]] ;;
  *"import pyrealsense2"*)
    [[ $FAILED_IMPORT != pyrealsense2 ]] || exit 1
    printf '%s\n' "$REALSENSE_MODULE"
    ;;
  *"import gi"*) [[ $FAILED_IMPORT != gi ]] ;;
esac
""",
        encoding="utf-8",
    )
    python.chmod(0o755)
    script = ROOT / "scripts" / script_name
    shell = f'''
source "{script}"
require_trusted_file() {{ :; }}
validate_realsense_binary() {{ :; }}
require_capture_dependencies "{python}" "{install_root}"
'''

    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        env={
            **os.environ,
            "FAILED_IMPORT": failed_import,
            "REALSENSE_MODULE": str(module),
        },
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert result.stderr.strip() == reason


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
@pytest.mark.parametrize(
    "script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
def test_gstreamer_dependency_probe_disables_registry_writes(tmp_path, script_name):
    cache = tmp_path / "cache"
    cache.mkdir()
    python = tmp_path / "python"
    python.write_text(
        """#!/bin/bash
if [[ $GST_REGISTRY != /dev/null || $GST_REGISTRY_UPDATE != no ]]; then
  printf registry >"$XDG_CACHE_HOME/registry.bin"
fi
""",
        encoding="utf-8",
    )
    python.chmod(0o755)
    script = ROOT / "scripts" / script_name
    shell = f'''
source "{script}"
require_gstreamer_dependency "{python}"
'''

    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        env={**os.environ, "HOME": str(tmp_path), "XDG_CACHE_HOME": str(cache)},
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert list(cache.iterdir()) == []


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_real_gstreamer_verifier_probe_preserves_isolated_home(tmp_path):
    available = subprocess.run(
        [
            sys.executable,
            "-c",
            'import gi; gi.require_version("Gst", "1.0"); from gi.repository import Gst',
        ],
        capture_output=True,
    )
    if available.returncode != 0:
        pytest.skip("GStreamer introspection is unavailable")
    home = tmp_path / "home"
    cache = tmp_path / "cache"
    home.mkdir()
    cache.mkdir()
    script = ROOT / "scripts" / "verify_ubuntu_environment.sh"
    shell = f'''
source "{script}"
require_gstreamer_dependency "{sys.executable}"
'''

    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        env={**os.environ, "HOME": str(home), "XDG_CACHE_HOME": str(cache)},
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert list(home.iterdir()) == []
    assert list(cache.iterdir()) == []


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_missing_realsense_vendor_artifact_is_blocked_before_install_mutation(tmp_path):
    missing = tmp_path / "missing-pyrealsense2.so"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
require_realsense_vendor_artifact "{missing}"
'''

    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 3
    assert result.stderr.strip() == "BLOCKED/realsense_python_missing"


@pytest.mark.parametrize(
    "script", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
def test_missing_tool_has_stable_classification(script):
    path = ROOT / "scripts" / script
    result = subprocess.run(
        ["bash", "-c", f'source "{path}"; require_tools definitely_missing_tool'],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/tool_missing:definitely_missing_tool"


@pytest.mark.skipif(os.name != "posix", reason="POSIX PATH behavior")
@pytest.mark.parametrize(
    ("script_name", "preflight", "missing_tools"),
    [
        (
            "install_ubuntu.sh",
            "require_installer_tools",
            ("awk", "cc", "mkdir", "rm", "chown"),
        ),
        ("verify_ubuntu_environment.sh", "require_verifier_tools", ("awk",)),
    ],
)
def test_real_script_preflight_classifies_each_missing_tool(
    tmp_path, script_name, preflight, missing_tools
):
    required = {
        "install_ubuntu.sh": (
            "python3", "dirname", "install", "make", "sha256sum", "awk", "file",
            "grep", "nm", "stat", "readlink", "chmod", "chown", "mktemp", "mv", "rm",
            "sync", "find", "sort", "id", "cc", "mkdir", "flock",
        ),
        "verify_ubuntu_environment.sh": (
            "sha256sum", "awk", "file", "grep", "nm", "stat", "readlink", "find",
            "sort", "id",
        ),
    }[script_name]
    script = ROOT / "scripts" / script_name

    for missing in missing_tools:
        tool_dir = tmp_path / missing
        tool_dir.mkdir()
        for tool in required:
            if tool != missing:
                resolved = shutil.which(tool)
                assert resolved, tool
                (tool_dir / tool).symlink_to(resolved)
        result = subprocess.run(
            ["/bin/bash", "-c", f'source "{script}"; {preflight}'],
            env={**os.environ, "PATH": str(tool_dir)},
            capture_output=True,
            text=True,
        )
        assert result.returncode == 2
        assert result.stderr.strip() == f"FAIL/tool_missing:{missing}"


@pytest.mark.skipif(os.name != "posix", reason="POSIX ownership invariant")
def test_trust_check_rejects_non_root_owned_path(tmp_path):
    path = ROOT / "scripts" / "install_ubuntu.sh"
    result = subprocess.run(
        ["bash", "-c", f'source "{path}"; require_trusted_path "{tmp_path}"'],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/untrusted_installation"


@pytest.mark.skipif(os.name != "posix", reason="POSIX tree types")
@pytest.mark.parametrize("script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"])
def test_trusted_tree_rejects_symlink_and_special_file(tmp_path, script_name):
    tree = tmp_path / "tree"
    tree.mkdir()
    (tree / "safe").write_text("safe", encoding="utf-8")
    script = ROOT / "scripts" / script_name
    fake_stat = '''
stat() {
  case "$*" in
    *%u*) echo 0 ;;
    *%g*) echo 0 ;;
    *%a*) echo 755 ;;
    *%d*) echo 1 ;;
    *) command stat "$@" ;;
  esac
}
'''

    safe = subprocess.run(
        ["/bin/bash", "-c", f'source "{script}"; {fake_stat} require_trusted_tree "{tree}"'],
        capture_output=True,
        text=True,
    )
    assert safe.returncode == 0, safe.stderr

    (tree / "link").symlink_to(tree / "safe")
    linked = subprocess.run(
        ["/bin/bash", "-c", f'source "{script}"; {fake_stat} require_trusted_tree "{tree}"'],
        capture_output=True,
        text=True,
    )
    assert linked.returncode == 2
    (tree / "link").unlink()

    os.mkfifo(tree / "special")
    special = subprocess.run(
        ["/bin/bash", "-c", f'source "{script}"; {fake_stat} require_trusted_tree "{tree}"'],
        capture_output=True,
        text=True,
    )
    assert special.returncode == 2


@pytest.mark.skipif(os.name != "posix", reason="POSIX tree metadata")
@pytest.mark.parametrize("unsafe_field", ["mode", "device"])
def test_trusted_tree_rejects_writable_or_cross_filesystem_entry(tmp_path, unsafe_field):
    tree = tmp_path / "tree"
    tree.mkdir()
    unsafe = tree / "unsafe"
    unsafe.write_text("unsafe", encoding="utf-8")
    script = ROOT / "scripts" / "install_ubuntu.sh"
    unsafe_mode = "775" if unsafe_field == "mode" else "755"
    unsafe_device = "2" if unsafe_field == "device" else "1"
    shell = f'''
source "{script}"
stat() {{
  case "$*" in
    *%u*) echo 0 ;;
    *%g*) echo 0 ;;
    *%a*unsafe*) echo {unsafe_mode} ;;
    *%a*) echo 755 ;;
    *%d*unsafe*) echo {unsafe_device} ;;
    *%d*) echo 1 ;;
    *) command stat "$@" ;;
  esac
}}
require_trusted_tree "{tree}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/untrusted_installation"


@pytest.mark.skipif(os.name != "posix", reason="bash coprocess behavior")
@pytest.mark.parametrize("script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"])
def test_trusted_tree_propagates_find_failure_after_partial_nul_output(tmp_path, script_name):
    tree = tmp_path / "tree"
    tree.mkdir()
    safe = tree / "safe"
    safe.write_text("safe", encoding="utf-8")
    script = ROOT / "scripts" / script_name
    shell = f'''
source "{script}"
stat() {{
  case "$*" in
    *%u*|*%g*) echo 0 ;;
    *%a*) echo 755 ;;
    *%d*) echo 1 ;;
    *) command stat "$@" ;;
  esac
}}
find() {{ printf '%s\\0%s\\0' "{tree}" "{safe}"; return 1; }}
require_trusted_tree "{tree}"
'''
    before = set(tmp_path.iterdir())
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/untrusted_installation"
    assert set(tmp_path.iterdir()) == before


@pytest.mark.skipif(os.name != "posix", reason="bash coprocess behavior")
def test_trusted_tree_stops_and_reaps_find_when_entry_validation_fails(tmp_path):
    tree = tmp_path / "tree"
    tree.mkdir()
    special = tree / "special"
    os.mkfifo(special)
    script = ROOT / "scripts" / "verify_ubuntu_environment.sh"
    shell = f'''
source "{script}"
stat() {{
  case "$*" in
    *%u*|*%g*) echo 0 ;;
    *%a*) echo 755 ;;
    *%d*) echo 1 ;;
    *) command stat "$@" ;;
  esac
}}
find() {{ printf '%s\\0%s\\0' "{tree}" "{special}"; while :; do :; done; }}
require_trusted_tree "{tree}"
'''
    result = subprocess.run(
        ["/bin/bash", "-c", shell], capture_output=True, text=True, timeout=3
    )

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/untrusted_installation"


def test_invalid_bridge_artifact_does_not_replace_existing_library(tmp_path):
    install_root = tmp_path / "install"
    lib = install_root / "lib"
    lib.mkdir(parents=True)
    bridge = lib / "libtwo_uq2_xu.so"
    manifest = lib / "libtwo_uq2_xu.so.sha256"
    bridge.write_bytes(b"known-good")
    manifest.write_text("known-good-manifest", encoding="utf-8")
    invalid_artifact = tmp_path / "artifact"
    invalid_artifact.write_bytes(b"not-an-elf")
    script = ROOT / "scripts" / "install_ubuntu.sh"

    result = subprocess.run(
        [
            "bash",
            "-c",
            f'source "{script}"; publish_bridge "{invalid_artifact}" "{install_root}"',
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/xu_bridge_artifact_invalid"
    assert bridge.read_bytes() == b"known-good"
    assert manifest.read_text(encoding="utf-8") == "known-good-manifest"


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
@pytest.mark.parametrize("script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"])
def test_bridge_manifest_must_be_single_record_bound_to_bridge(tmp_path, script_name):
    bridge = tmp_path / "bridge.so"
    bridge.write_bytes(b"bridge")
    digest = subprocess.check_output(["sha256sum", str(bridge)], text=True).split()[0]
    manifest = tmp_path / "bridge.so.sha256"
    script = ROOT / "scripts" / script_name

    for content, expected in (
        (f"{digest}  {bridge}\n", 0),
        (f"{digest}  /tmp/other\n", 2),
        (f"{digest}  {bridge}\n{digest}  {bridge}\n", 2),
        (f"{'0' * 64}  {bridge}\n", 2),
    ):
        manifest.write_text(content, encoding="utf-8")
        result = subprocess.run(
            [
                "bash",
                "-c",
                f'source "{script}"; validate_bridge_manifest "{bridge}" '
                f'"{manifest}" "{bridge}"',
            ],
            capture_output=True,
            text=True,
        )
        assert result.returncode == expected, result.stderr


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_publish_bridge_is_idempotent_and_preserves_first_different_hash(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    first = tmp_path / "first.so"
    first.write_bytes(b"first")
    different = tmp_path / "different.so"
    different.write_bytes(b"different")
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
validate_bridge() {{ :; }}
require_trusted_path() {{ :; }}
require_trusted_file() {{ :; }}
sync() {{ :; }}
install() {{ command cp "$7" "$8"; command chmod "$6" "$8"; }}
publish_bridge "{first}" "{install_root}"
publish_bridge "{first}" "{install_root}"
publish_bridge "{different}" "{install_root}"
'''

    result = subprocess.run(["bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/xu_bridge_conflict"
    assert (install_root / "lib" / "libtwo_uq2_xu.so").read_bytes() == b"first"


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_same_hash_first_publish_race_is_idempotent(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    artifact = tmp_path / "artifact.so"
    artifact.write_bytes(b"same")
    sync_log = tmp_path / "sync.log"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
validate_bridge() {{ :; }}
require_trusted_path() {{ :; }}
require_trusted_file() {{ :; }}
sync() {{ printf '%s\n' "$*" >>"{sync_log}"; }}
install() {{ command cp "$7" "$8"; command chmod "$6" "$8"; }}
mv() {{ command cp -a "$3" "$4"; }}
publish_bridge "{artifact}" "{install_root}"
'''

    result = subprocess.run(["bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert (install_root / "lib" / "libtwo_uq2_xu.so").read_bytes() == b"same"
    assert f"-f {install_root}" in sync_log.read_text(encoding="utf-8").splitlines()


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_python_runtime_validates_full_venv_before_first_execution(tmp_path):
    install_root = tmp_path / "install"
    site_packages = install_root / "venv" / "lib" / "python3" / "site-packages"
    site_packages.mkdir(parents=True)
    python = install_root / "venv" / "bin" / "python"
    python.parent.mkdir()
    trace = tmp_path / "trace"
    python.write_text(f'#!/bin/bash\necho execute >>"{trace}"\n', encoding="utf-8")
    python.chmod(0o755)
    (install_root / "venv" / "pyvenv.cfg").write_text(
        "home = /usr/bin", encoding="utf-8"
    )
    (site_packages / "module.py").write_text("VALUE = 1", encoding="utf-8")
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
require_trusted_tree() {{ echo tree >>"{trace}"; }}
require_trusted_file() {{ echo file >>"{trace}"; }}
require_runtime_parent_access() {{ :; }}
validate_python_runtime "{install_root}" "{python}"
'''

    result = subprocess.run(["bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert trace.read_text(encoding="utf-8").splitlines() == ["tree", "file", "execute"]


@pytest.mark.skipif(os.name != "posix", reason="POSIX venv layout")
def test_fresh_cpython_venv_is_normalized_before_strict_tree_validation(tmp_path):
    install_root = tmp_path / "install"
    venv = install_root / "venv"
    subprocess.run(
        [sys.executable, "-m", "venv", "--copies", "--without-pip", str(venv)],
        check=True,
        capture_output=True,
        text=True,
    )
    lib64 = venv / "lib64"
    if not lib64.is_symlink():
        pytest.skip("interpreter does not generate the 64-bit lib64 venv link")
    assert os.readlink(lib64) == "lib"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    fake_stat = '''
stat() {
  case "$*" in
    *%u*) echo 0 ;;
    *%g*) echo 0 ;;
    *%a*) echo 755 ;;
    *%d*) echo 1 ;;
    *) command stat "$@" ;;
  esac
}
'''
    shell = f'''
source "{script}"
normalize_venv_layout "{install_root}"
{fake_stat}
require_trusted_tree "{venv}"
'''

    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert not lib64.exists() and not lib64.is_symlink()


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_install_lock_contention_has_stable_failure(tmp_path):
    lock = tmp_path / "install.lock"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
require_trusted_file() {{ :; }}
flock() {{ return 1; }}
acquire_install_lock "{lock}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/install_in_progress"


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_bridge_conflict_precedes_pip_and_preserves_venv(tmp_path):
    install_root = tmp_path / "install"
    lib = install_root / "lib"
    python = install_root / "venv" / "bin" / "python"
    lib.mkdir(parents=True)
    python.parent.mkdir(parents=True)
    sentinel = b"venv-sentinel"
    python.write_bytes(sentinel)
    python.chmod(0o755)
    before_mtime = python.stat().st_mtime_ns
    (lib / "libtwo_uq2_xu.so").write_bytes(b"existing")
    artifact = tmp_path / "artifact.so"
    artifact.write_bytes(b"different")
    pip_log = tmp_path / "pip.log"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
validate_bridge() {{ :; }}
validate_existing_library() {{ :; }}
require_trusted_path() {{ :; }}
require_trusted_file() {{ :; }}
require_runtime_parent_access() {{ :; }}
install() {{ command cp "$7" "$8"; command chmod "$6" "$8"; }}
validate_python_runtime() {{ echo pip >>"{pip_log}"; }}
python_venv_capability_checked=true
install_runtime "{artifact}" "{install_root}" "{ROOT}"
'''

    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/xu_bridge_conflict"
    assert not pip_log.exists()
    assert python.read_bytes() == sentinel
    assert python.stat().st_mtime_ns == before_mtime


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_missing_python_venv_capability_precedes_install_tree_mutation(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    artifact = tmp_path / "artifact"
    artifact.write_bytes(b"artifact")
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
python3() {{ return 1; }}
prepare_bridge() {{ mkdir "{install_root}/mutated"; }}
install_runtime "{artifact}" "{install_root}" "{ROOT}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/tool_missing:python3-venv"
    assert list(install_root.iterdir()) == []


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_failed_new_venv_creation_leaves_no_final_or_owned_temp(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
python3() {{ return 1; }}
create_new_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert not (install_root / "venv").exists()
    assert list(install_root.glob(".venv.*.tmp")) == []


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_new_venv_is_first_published_by_rename_after_validation(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    trace = tmp_path / "trace"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
python3() {{
  local target="${{@: -1}}"
  mkdir -p "$target/bin" "$target/lib/python3/site-packages"
  printf '#!/bin/bash\n' >"$target/bin/python"
  printf home >"$target/pyvenv.cfg"
  printf module >"$target/lib/python3/site-packages/module.py"
  chmod 0755 "$target/bin/python"
}}
require_trusted_tree() {{ echo validate >>"{trace}"; }}
require_trusted_file() {{ :; }}
require_runtime_parent_access() {{ :; }}
chown() {{ :; }}
sync() {{ :; }}
create_new_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert (install_root / "venv" / "bin" / "python").is_file()
    assert trace.read_text(encoding="utf-8").splitlines() == [
        "validate",
        "validate",
        "validate",
    ]
    assert list(install_root.glob(".venv.*.tmp")) == []


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_existing_venv_is_not_rebuilt_or_republished(tmp_path):
    install_root = tmp_path / "install"
    python = install_root / "venv" / "bin" / "python"
    python.parent.mkdir(parents=True)
    (install_root / "venv" / "lib").mkdir()
    lib64 = install_root / "venv" / "lib64"
    lib64.symlink_to("lib", target_is_directory=True)
    python.write_bytes(b"sentinel")
    before = (python.read_bytes(), python.stat().st_mtime_ns)
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
python3() {{ echo rebuilt >&2; return 1; }}
ensure_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert (python.read_bytes(), python.stat().st_mtime_ns) == before
    assert lib64.is_symlink()
    assert os.readlink(lib64) == "lib"
    assert list(install_root.glob(".venv.*.tmp")) == []


@pytest.mark.skipif(os.name != "posix", reason="bash inode behavior")
def test_replaced_temporary_venv_is_neither_published_nor_deleted(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    mutation_log = tmp_path / "mutation-log"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
python3() {{
  local target="${{@: -1}}"
  mkdir "${{target}}.replacement"
  printf replacement >"${{target}}.replacement/marker"
  rm -rf -- "$target"
  mv "${{target}}.replacement" "$target"
}}
chown() {{ echo chown >>"{mutation_log}"; }}
chmod() {{ echo chmod >>"{mutation_log}"; }}
require_trusted_tree() {{ :; }}
sync() {{ :; }}
create_new_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    replacements = list(install_root.glob(".venv.*.tmp"))
    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/untrusted_installation"
    assert not (install_root / "venv").exists()
    assert len(replacements) == 1
    assert (replacements[0] / "marker").read_text(encoding="utf-8") == "replacement"
    assert not mutation_log.exists()


@pytest.mark.skipif(os.name != "posix", reason="POSIX Python isolation")
def test_create_new_venv_ignores_hostile_cwd_and_pythonpath(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    hostile = tmp_path / "hostile"
    hostile.mkdir()
    sentinel = tmp_path / "sentinel"
    (hostile / "venv.py").write_text(
        f'from pathlib import Path\nPath({str(sentinel)!r}).write_text("executed")\n',
        encoding="utf-8",
    )
    wrapper_dir = tmp_path / "bin"
    wrapper_dir.mkdir()
    arguments = tmp_path / "python-arguments"
    wrapper = wrapper_dir / "python3"
    wrapper.write_text(
        '#!/bin/bash\nprintf \'%s\\n\' "$@" >"$PYTHON_ARGUMENTS"\n'
        'exec /usr/bin/python3 "$@" --without-pip\n',
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
chown() {{ :; }}
require_trusted_tree() {{ :; }}
require_runtime_parent_access() {{ :; }}
sync() {{ :; }}
create_new_venv "{install_root}"
'''
    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        cwd=hostile,
        env={
            **os.environ,
            "PATH": f"{wrapper_dir}:/usr/sbin:/usr/bin:/sbin:/bin",
            "PYTHONPATH": str(hostile),
            "PYTHON_ARGUMENTS": str(arguments),
        },
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert not sentinel.exists()
    args = arguments.read_text(encoding="utf-8").splitlines()
    assert args[:4] == ["-I", "-B", "-m", "venv"]
    assert "--system-site-packages" in args
    subprocess.run(
        [str(install_root / "venv" / "bin" / "python"), "-I", "-B", "-c", "import json"],
        check=True,
        cwd=hostile,
        env={**os.environ, "PYTHONPATH": str(hostile)},
    )


@pytest.mark.skipif(os.name != "posix", reason="POSIX mode behavior")
def test_published_venv_has_nonowner_runtime_permissions_without_broad_read(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
    chown_log = tmp_path / "chown-log"
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
python3() {{
  local target="${{@: -1}}"
  mkdir -m 0700 "$target/bin" "$target/lib"
  mkdir -p "$target/lib/python3/site-packages"
  printf '#!/bin/bash\nexit 0\n' >"$target/bin/python"
  printf home >"$target/pyvenv.cfg"
  printf private >"$target/private-data"
  chmod 0700 "$target/bin/python"
  chmod 0600 "$target/pyvenv.cfg" "$target/private-data"
}}
chown() {{ printf '%s\n' "$*" >>"{chown_log}"; }}
require_trusted_tree() {{ :; }}
require_runtime_parent_access() {{ :; }}
sync() {{ :; }}
create_new_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    venv = install_root / "venv"
    assert stat.S_IMODE(venv.stat().st_mode) == 0o755
    assert stat.S_IMODE((venv / "bin").stat().st_mode) == 0o755
    assert stat.S_IMODE((venv / "lib").stat().st_mode) == 0o755
    assert stat.S_IMODE((venv / "bin" / "python").stat().st_mode) == 0o755
    assert stat.S_IMODE((venv / "pyvenv.cfg").stat().st_mode) == 0o644
    assert stat.S_IMODE((venv / "private-data").stat().st_mode) == 0o600
    chown_lines = chown_log.read_text(encoding="utf-8").splitlines()
    assert len(chown_lines) == 1
    assert chown_lines[0].startswith("-R root:root ")
    assert "/.venv." in chown_lines[0] and chown_lines[0].endswith(".tmp")
    for path in (venv, venv / "bin", venv / "bin" / "python"):
        assert stat.S_IMODE(path.stat().st_mode) & 0o005 == 0o005
    assert stat.S_IMODE((venv / "pyvenv.cfg").stat().st_mode) & 0o004


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
@pytest.mark.parametrize("failure_command", ["chown", "chmod"])
def test_venv_owner_or_mode_adjustment_failure_cleans_without_publish(
    tmp_path, failure_command
):
    install_root = tmp_path / "install"
    install_root.mkdir()
    script = ROOT / "scripts" / "install_ubuntu.sh"
    chown_function = "chown() { return 1; }" if failure_command == "chown" else "chown() { :; }"
    chmod_function = (
        "chmod() { return 1; }"
        if failure_command == "chmod"
        else "chmod() { command chmod \"$@\"; }"
    )
    shell = f'''
source "{script}"
python3() {{
  local target="${{@: -1}}"
  mkdir "$target/bin" "$target/lib"
  printf '#!/bin/bash\n' >"$target/bin/python"
  printf home >"$target/pyvenv.cfg"
}}
{chown_function}
{chmod_function}
require_trusted_tree() {{ :; }}
create_new_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 2
    assert not (install_root / "venv").exists()
    assert list(install_root.glob(".venv.*.tmp")) == []


def _runtime_access_fixture(tmp_path):
    install_root = tmp_path / "install"
    venv = install_root / "venv"
    site_packages = venv / "lib" / "python3" / "site-packages"
    site_packages.mkdir(parents=True)
    (venv / "bin").mkdir()
    python = venv / "bin" / "python"
    python.write_text(
        '#!/bin/bash\nprintf executed >"$VENV_SENTINEL"\n', encoding="utf-8"
    )
    config = venv / "pyvenv.cfg"
    config.write_text("home = /usr/bin", encoding="utf-8")
    module = site_packages / "module.py"
    module.write_text("VALUE = 1", encoding="utf-8")
    for directory in (
        venv,
        venv / "bin",
        venv / "lib",
        venv / "lib" / "python3",
        site_packages,
    ):
        directory.chmod(0o755)
    python.chmod(0o755)
    config.chmod(0o644)
    module.chmod(0o644)
    return install_root, python, config, module, site_packages


def _tree_snapshot(root):
    return {
        path.relative_to(root): (
            stat.S_IMODE(path.lstat().st_mode),
            path.read_bytes() if path.is_file() else None,
        )
        for path in (root, *root.rglob("*"))
    }


@pytest.mark.skipif(os.name != "posix", reason="POSIX other-bit model")
@pytest.mark.parametrize(
    "script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
@pytest.mark.parametrize(
    "inaccessible",
    ["venv", "bin", "lib", "site_packages", "config", "python", "module"],
)
def test_existing_venv_inaccessible_to_nonowner_fails_before_python(
    tmp_path, script_name, inaccessible
):
    install_root, python, config, module, site_packages = _runtime_access_fixture(tmp_path)
    targets = {
        "venv": install_root / "venv",
        "bin": install_root / "venv" / "bin",
        "lib": install_root / "venv" / "lib",
        "site_packages": site_packages,
        "config": config,
        "python": python,
        "module": module,
    }
    targets[inaccessible].chmod(0o700 if targets[inaccessible].is_dir() else 0o600)
    before = _tree_snapshot(install_root)
    sentinel = tmp_path / "sentinel"
    script = ROOT / "scripts" / script_name
    call = (
        f'ensure_venv "{install_root}"\n'
        f'validate_python_runtime "{install_root}" "{python}"'
        if script_name == "install_ubuntu.sh"
        else f'verify_python_runtime "{install_root}"'
    )
    shell = f'''
source "{script}"
require_trusted_tree() {{ :; }}
require_trusted_file() {{ :; }}
require_runtime_parent_access() {{ :; }}
{call}
'''
    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        env={**os.environ, "VENV_SENTINEL": str(sentinel)},
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/untrusted_installation"
    assert not sentinel.exists()
    assert _tree_snapshot(install_root) == before


@pytest.mark.skipif(os.name != "posix", reason="POSIX other-bit model")
@pytest.mark.parametrize(
    "script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
def test_existing_venv_complete_nonowner_model_reaches_controlled_python(
    tmp_path, script_name
):
    install_root, python, _config, _module, _site_packages = _runtime_access_fixture(tmp_path)
    sentinel = tmp_path / "sentinel"
    script = ROOT / "scripts" / script_name
    call = (
        f'ensure_venv "{install_root}"\n'
        f'validate_python_runtime "{install_root}" "{python}"'
        if script_name == "install_ubuntu.sh"
        else f'verify_python_runtime "{install_root}"'
    )
    shell = f'''
source "{script}"
require_trusted_tree() {{ :; }}
require_trusted_file() {{ :; }}
require_runtime_parent_access() {{ :; }}
{call}
'''
    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        env={**os.environ, "VENV_SENTINEL": str(sentinel)},
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert sentinel.read_text(encoding="utf-8") == "executed"


def _runtime_parent_fixture(tmp_path):
    model_root = tmp_path / "model-root"
    intermediate = model_root / "opt"
    install_root = intermediate / "three-device-slam"
    venv = install_root / "venv"
    site_packages = venv / "lib" / "python3" / "site-packages"
    site_packages.mkdir(parents=True)
    (venv / "bin").mkdir()
    python = venv / "bin" / "python"
    python.write_text(
        '#!/bin/bash\nprintf executed >"$VENV_SENTINEL"\n', encoding="utf-8"
    )
    (venv / "pyvenv.cfg").write_text("home = /usr/bin", encoding="utf-8")
    (site_packages / "module.py").write_text("VALUE = 1", encoding="utf-8")
    for directory in (
        model_root,
        intermediate,
        install_root,
        venv,
        venv / "bin",
        venv / "lib",
        venv / "lib" / "python3",
        site_packages,
    ):
        directory.chmod(0o755)
    python.chmod(0o755)
    (venv / "pyvenv.cfg").chmod(0o644)
    (site_packages / "module.py").chmod(0o644)
    return model_root, intermediate, install_root, python


def _root_metadata_stat_wrapper(model_root):
    return f'''
stat() {{
  local target="${{@: -1}}"
  case "$*" in
    *%u*) printf '0\n' ;;
    *%g*) printf '0\n' ;;
    *%a*)
      case "$target" in
        "{model_root}"|"{model_root}"/*) command stat -c '%a' -- "$target" ;;
        *) printf '755\n' ;;
      esac
      ;;
    *) command stat "$@" ;;
  esac
}}
'''


@pytest.mark.skipif(os.name != "posix", reason="POSIX other-bit model")
@pytest.mark.parametrize(
    "script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
@pytest.mark.parametrize("blocked_directory", ["install_root", "intermediate"])
def test_runtime_parent_without_other_execute_fails_before_python_without_mutation(
    tmp_path, script_name, blocked_directory
):
    model_root, intermediate, install_root, python = _runtime_parent_fixture(tmp_path)
    blocked = install_root if blocked_directory == "install_root" else intermediate
    blocked.chmod(0o700 if blocked_directory == "install_root" else 0o744)
    before = _tree_snapshot(model_root)
    sentinel = tmp_path / "sentinel"
    script = ROOT / "scripts" / script_name
    call = (
        f'ensure_venv "{install_root}"\n'
        f'validate_python_runtime "{install_root}" "{python}"'
        if script_name == "install_ubuntu.sh"
        else f'verify_python_runtime "{install_root}"'
    )
    shell = f'''
source "{script}"
{_root_metadata_stat_wrapper(model_root)}
{call}
'''
    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        env={**os.environ, "VENV_SENTINEL": str(sentinel)},
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert result.stderr.strip() == "FAIL/untrusted_installation"
    assert not sentinel.exists()
    assert _tree_snapshot(model_root) == before


@pytest.mark.skipif(os.name != "posix", reason="POSIX other-bit model")
@pytest.mark.parametrize(
    "script_name", ["install_ubuntu.sh", "verify_ubuntu_environment.sh"]
)
def test_runtime_parent_complete_other_execute_chain_reaches_python(
    tmp_path, script_name
):
    model_root, _intermediate, install_root, python = _runtime_parent_fixture(tmp_path)
    sentinel = tmp_path / "sentinel"
    script = ROOT / "scripts" / script_name
    call = (
        f'ensure_venv "{install_root}"\n'
        f'validate_python_runtime "{install_root}" "{python}"'
        if script_name == "install_ubuntu.sh"
        else f'verify_python_runtime "{install_root}"'
    )
    shell = f'''
source "{script}"
{_root_metadata_stat_wrapper(model_root)}
{call}
'''
    result = subprocess.run(
        ["/bin/bash", "-c", shell],
        env={**os.environ, "VENV_SENTINEL": str(sentinel)},
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert sentinel.read_text(encoding="utf-8") == "executed"
