import signal
from types import SimpleNamespace

import pytest

from three_device_slam import cli
from three_device_slam.config import ConfigError


def fake_config(tmp_path):
    return SimpleNamespace(output_root=tmp_path / "sessions", duration_s=None)


def test_product_runs_capture_index_verify_in_order(tmp_path, monkeypatch, capsys):
    events = []
    monkeypatch.setattr(cli, "load_product_config", lambda _path: fake_config(tmp_path))
    capture = SimpleNamespace(session=tmp_path / "session", report={"status": "PASS"})
    monkeypatch.setattr(
        cli, "run_capture", lambda _config, _stop: events.append("capture") or capture
    )
    monkeypatch.setattr(cli, "build_index", lambda session: events.append("index") or {})
    monkeypatch.setattr(
        cli,
        "verify_session",
        lambda session: events.append("verify") or {"status": "PASS"},
    )

    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == 3
    assert events == ["capture", "index", "verify"]
    output = capsys.readouterr().out
    assert f"session={capture.session}" in output
    assert "acquisition=PASS" in output
    assert "index=PASS" in output
    assert "verification=PASS" in output
    assert "overall=BLOCKED" in output


def test_default_product_config_path(tmp_path, monkeypatch):
    observed = []
    monkeypatch.setattr(
        cli,
        "load_product_config",
        lambda path: observed.append(path) or fake_config(tmp_path),
    )
    capture = SimpleNamespace(session=tmp_path / "session", report={"status": "FAIL"})
    monkeypatch.setattr(cli, "run_capture", lambda _config, _stop: capture)

    assert cli.main(["product"]) == 2
    assert observed == [cli.DEFAULT_CONFIG_PATH]


def test_missing_config_is_blocked_without_capture(tmp_path, monkeypatch, capsys):
    called = False

    def fail_if_called(*_args):
        nonlocal called
        called = True

    monkeypatch.setattr(cli, "run_capture", fail_if_called)

    assert cli.main(["product", "--config", str(tmp_path / "missing.json")]) == 3
    assert called is False
    assert "BLOCKED/device_config_missing" in capsys.readouterr().err


def test_invalid_config_is_stable_fail_without_leaking_detail(tmp_path, monkeypatch, capsys):
    secret = "private-serial-value"
    monkeypatch.setattr(
        cli,
        "load_product_config",
        lambda _path: (_ for _ in ()).throw(ConfigError(secret)),
    )
    monkeypatch.setattr(cli, "run_capture", lambda *_args: pytest.fail("opened hardware"))

    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == 2
    error = capsys.readouterr().err
    assert "FAIL/device_config_invalid" in error
    assert secret not in error


@pytest.mark.parametrize("capture_status, expected", [("FAIL", 2), ("BLOCKED", 3)])
def test_unsuccessful_capture_never_runs_offline_stages(
    tmp_path, monkeypatch, capture_status, expected
):
    monkeypatch.setattr(cli, "load_product_config", lambda _path: fake_config(tmp_path))
    capture = SimpleNamespace(
        session=tmp_path / "session", report={"status": capture_status}
    )
    monkeypatch.setattr(cli, "run_capture", lambda _config, _stop: capture)
    monkeypatch.setattr(cli, "build_index", lambda _session: pytest.fail("indexed"))
    monkeypatch.setattr(cli, "verify_session", lambda _session: pytest.fail("verified"))

    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == expected


def test_capture_exception_is_stable_fail_and_stops_pipeline(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(cli, "load_product_config", lambda _path: fake_config(tmp_path))
    monkeypatch.setattr(
        cli, "run_capture", lambda *_args: (_ for _ in ()).throw(RuntimeError("secret"))
    )
    monkeypatch.setattr(cli, "build_index", lambda _session: pytest.fail("indexed"))

    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == 2
    assert capsys.readouterr().err.strip() == "FAIL/capture_exception"


def test_index_failure_prevents_verification(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(cli, "load_product_config", lambda _path: fake_config(tmp_path))
    capture = SimpleNamespace(session=tmp_path / "session", report={"status": "PASS"})
    monkeypatch.setattr(cli, "run_capture", lambda _config, _stop: capture)
    monkeypatch.setattr(
        cli, "build_index", lambda _session: (_ for _ in ()).throw(RuntimeError("secret"))
    )
    monkeypatch.setattr(cli, "verify_session", lambda _session: pytest.fail("verified"))

    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == 2
    assert "FAIL/index_exception" in capsys.readouterr().err


def test_signal_handler_only_requests_natural_stop(tmp_path, monkeypatch):
    handlers = {}
    monkeypatch.setattr(cli.signal, "signal", lambda number, handler: handlers.setdefault(number, handler))
    monkeypatch.setattr(cli, "load_product_config", lambda _path: fake_config(tmp_path))

    def capture(_config, stop_requested):
        assert not stop_requested()
        handlers[signal.SIGTERM](signal.SIGTERM, None)
        assert stop_requested()
        return SimpleNamespace(session=tmp_path / "session", report={"status": "BLOCKED"})

    monkeypatch.setattr(cli, "run_capture", capture)

    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == 3
    assert set(handlers) == {signal.SIGINT, signal.SIGTERM}


def test_verification_fail_controls_exit_code(tmp_path, monkeypatch):
    monkeypatch.setattr(cli, "load_product_config", lambda _path: fake_config(tmp_path))
    capture = SimpleNamespace(session=tmp_path / "session", report={"status": "PASS"})
    monkeypatch.setattr(cli, "run_capture", lambda _config, _stop: capture)
    monkeypatch.setattr(cli, "build_index", lambda _session: {})
    monkeypatch.setattr(cli, "verify_session", lambda _session: {"status": "FAIL"})

    assert cli.main(["product", "--config", str(tmp_path / "product.json")]) == 2
