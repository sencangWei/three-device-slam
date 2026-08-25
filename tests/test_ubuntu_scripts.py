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
        ("install_ubuntu.sh", "require_installer_tools", ("awk", "cc", "mkdir", "rm")),
        ("verify_ubuntu_environment.sh", "require_verifier_tools", ("awk",)),
    ],
)
def test_real_script_preflight_classifies_each_missing_tool(
    tmp_path, script_name, preflight, missing_tools
):
    required = {
        "install_ubuntu.sh": (
            "python3", "dirname", "install", "make", "sha256sum", "awk", "file",
            "grep", "nm", "stat", "readlink", "chmod", "mktemp", "mv", "rm",
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
  mkdir -p "$target/bin"
  printf '#!/bin/bash\n' >"$target/bin/python"
  chmod 0755 "$target/bin/python"
}}
require_trusted_tree() {{ echo validate >>"{trace}"; }}
require_trusted_file() {{ :; }}
sync() {{ :; }}
create_new_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert (install_root / "venv" / "bin" / "python").is_file()
    assert trace.read_text(encoding="utf-8").splitlines() == ["validate", "validate"]
    assert list(install_root.glob(".venv.*.tmp")) == []


@pytest.mark.skipif(os.name != "posix", reason="bash behavior test")
def test_existing_venv_is_not_rebuilt_or_republished(tmp_path):
    install_root = tmp_path / "install"
    python = install_root / "venv" / "bin" / "python"
    python.parent.mkdir(parents=True)
    python.write_bytes(b"sentinel")
    before = (python.read_bytes(), python.stat().st_mtime_ns)
    script = ROOT / "scripts" / "install_ubuntu.sh"
    shell = f'''
source "{script}"
python3() {{ echo rebuilt >&2; return 1; }}
normalize_venv_layout() {{ :; }}
ensure_venv "{install_root}"
'''
    result = subprocess.run(["/bin/bash", "-c", shell], capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert (python.read_bytes(), python.stat().st_mtime_ns) == before
    assert list(install_root.glob(".venv.*.tmp")) == []


@pytest.mark.skipif(os.name != "posix", reason="bash inode behavior")
def test_replaced_temporary_venv_is_neither_published_nor_deleted(tmp_path):
    install_root = tmp_path / "install"
    install_root.mkdir()
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
