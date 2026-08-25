import os
import stat
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


def test_launcher_is_strict_exec_entry():
    text = (ROOT / "run_three_device_slam.sh").read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    assert 'exec python3 -m three_device_slam.cli "$@"' in text
    assert "D405-MAXIMU" not in text


def test_installer_has_frozen_platform_and_bridge_checks_without_product_config():
    text = (ROOT / "scripts" / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    assert "FAIL/root_required" in text
    assert "ubuntu" in text and "22.04" in text and "humble" in text
    assert "BLOCKED/2uq2_vendor_sdk_missing" in text
    assert "--system-site-packages" in text
    assert 'pip install "$repo_root"' in text
    assert "libtwo_uq2_xu.so" in text
    for symbol in ("ylx_open", "ylx_read_imu27", "ylx_close"):
        assert symbol in text
    assert "built_hash=$(sha256sum" in text
    assert "installed_hash=$(sha256sum" in text
    assert "${bridge}.sha256" in text
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
    assert '"$python" -B -c' in text
    assert 'sha256sum --check --status "${bridge}.sha256"' in text
    for write_command in ("install ", "mkdir ", "touch ", "rm ", "mv ", "cp "):
        assert write_command not in text
