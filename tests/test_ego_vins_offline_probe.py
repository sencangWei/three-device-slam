from pathlib import Path
from types import SimpleNamespace
import json

import pytest

from scripts.ego_vins_offline_probe import (
    check_replay_liveness, diagnostic_config, runtime_provenance,
)
from scripts import ego_vins_offline_probe as probe


def test_control_is_explicit_and_default_preserves_imu():
    config = "%YAML:1.0\nimu: 1\noutput_path: /old\n"
    assert diagnostic_config(config, "/old", Path("/new"), False) == config.replace("/old", "/new")
    assert "\nimu: 0\n" in diagnostic_config(config, "/old", Path("/new"), True)
    assert "imu: 1" in config


@pytest.mark.parametrize("config", ["imu: 1", "\nimu: 1\n\nimu: 1\n", "\nimu: 0\n"])
def test_control_rejects_ambiguous_config(config):
    with pytest.raises(ValueError, match="unambiguously"):
        diagnostic_config(config, "/old", Path("/new"), True)


@pytest.mark.parametrize("returncode,elapsed", [(0, 1), (2, 1), (None, 85.001)])
def test_watchdog_rejects_exited_or_overdue_solver(returncode, elapsed):
    with pytest.raises(RuntimeError, match="expired"):
        check_replay_liveness(SimpleNamespace(poll=lambda: returncode), 10, 10 + elapsed)


def test_watchdog_allows_live_bounded_solver():
    check_replay_liveness(SimpleNamespace(poll=lambda: None), 10, 94.999)


def test_provenance_covers_primary_solver_and_exact_loader_path(tmp_path):
    executable = tmp_path / "node"
    executable.write_bytes(b"executable")
    (tmp_path / "vins").mkdir()
    (tmp_path / "vins/libvins_lib.so").write_bytes(b"solver")
    (tmp_path / "libvins_fusion_ros2__rosidl_typesupport_fastrtps_cpp.so").write_bytes(b"support")
    introspection = tmp_path / "libvins_fusion_ros2__rosidl_typesupport_introspection_cpp.so"
    with pytest.raises(FileNotFoundError):
        runtime_provenance(executable, {"LD_LIBRARY_PATH": "/specific"})
    introspection.write_bytes(b"introspection")
    result = runtime_provenance(executable, {"LD_LIBRARY_PATH": "/specific:/inherited"})
    assert set(result["files"]) == {"runner", "executable", "solver_library", "typesupport", "introspection"}
    assert len(result["files"]["solver_library"]["sha256"]) == 64
    assert result["effective_ld_library_path"] == "/specific:/inherited"
    (tmp_path / "vins/libvins_lib.so").unlink()
    with pytest.raises(FileNotFoundError):
        runtime_provenance(executable, {"LD_LIBRARY_PATH": "/specific"})


def test_scene_failure_stops_before_ros_or_child_start(tmp_path, monkeypatch):
    root = tmp_path / "repo"
    source = root / "input"
    source.mkdir(parents=True)
    (root / "scripts").mkdir()
    (root / "artifacts").mkdir()
    (source / "input_report.json").write_text(json.dumps({"status": "PROVISIONAL_INPUT_READY",
        "source_mode": "bench_no_motion", "outputs": {}}))
    (source / "vins_config.yaml").write_text('%YAML:1.0\nimu: 1\noutput_path: "/old"\n')
    monkeypatch.setattr(probe, "__file__", str(root / "scripts/probe.py"))
    monkeypatch.setenv("ROS_DOMAIN_ID", "92")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setattr(probe, "assess_export", lambda *args: {"status": "INSUFFICIENT_STEREO_SUPPORT"})
    def forbidden(*args, **kwargs):
        raise AssertionError("must not start solver for unsupported scene")
    monkeypatch.setattr(probe.subprocess, "Popen", forbidden)
    out = root / "artifacts/blocked"
    assert probe.run(source, out, root / "missing_binary") == 2
    report = json.loads((out / "report.json").read_text())
    assert report["status"] == "BLOCKED_SCENE_SUPPORT"
    assert report["solver_started"] is False


@pytest.mark.parametrize("scene,control,expected", [
    (False,False,"DIAGNOSTIC_REPLAY_COMPLETE"), (False,True,"DIAGNOSTIC_REPLAY_COMPLETE"),
    (True,False,"PROVISIONAL_REPLAY_PASS"), (True,True,"VISION_ONLY_CONTROL_PASS")])
def test_replay_success_never_promotes_unsupported_scene(scene,control,expected):
    assert probe.replay_status({"runtime":True},scene,control) == expected
    assert probe.replay_status({"runtime":False},scene,control) == "FAIL"
    assert probe.replay_status({},scene,control) == "FAIL"
