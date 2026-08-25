import os
import stat
import subprocess
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
    assert "--system-site-packages" in text
    assert 'pip install "$repo_root"' in text
    assert "libtwo_uq2_xu.so" in text
    for symbol in ("ylx_open", "ylx_read_imu27", "ylx_close"):
        assert symbol in text
    assert "built_hash=$(sha256sum" in text
    assert 'validate_bridge "${temporary_lib}/libtwo_uq2_xu.so"' in text
    assert 'require_trusted_tree "${install_root}/venv"' in text
    assert "umask 0022" in text
    assert 'built_hash=$(sha256sum "${temporary_lib}/libtwo_uq2_xu.so"' in text
    assert "installed_hash=$(sha256sum" in text
    assert "${bridge}.sha256" in text
    assert 'mktemp -d "${install_root}/.lib.' in text
    assert 'mv -T --no-clobber "$temporary_lib" "$lib"' in text
    assert 'ensure_root_directory "$install_root"' in text
    assert "ensure_root_directory /etc/three-device-slam" in text
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
    assert "FAIL/tool_missing:" in text
    for write_command in ("install ", "mkdir ", "touch ", "rm ", "mv ", "cp "):
        assert write_command not in text


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
    python = install_root / "venv" / "bin" / "python"
    python.parent.mkdir(parents=True)
    trace = tmp_path / "trace"
    python.write_text(f'#!/bin/bash\necho execute >>"{trace}"\n', encoding="utf-8")
    python.chmod(0o755)
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
require_trusted_tree() {{ echo tree >>"{trace}"; }}
require_trusted_file() {{ echo file >>"{trace}"; }}
validate_python_runtime "{install_root}" "{python}"
'''

    result = subprocess.run(["bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert trace.read_text(encoding="utf-8").splitlines() == ["tree", "file", "execute"]
