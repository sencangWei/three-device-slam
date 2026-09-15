"""The release requires explicit device IDs for calibration association."""
import pytest

from three_device_slam.edge_rk3576.rsusb_capture import main


@pytest.mark.parametrize("extra", [[], ["--d405-sdk-serial", "sdk"], ["--d405-usb-serial", "usb"]])
def test_cli_rejects_missing_identity(tmp_path, extra):
    with pytest.raises(SystemExit) as exc:
        main(["--output-root", str(tmp_path), *extra])
    assert exc.value.code == 2


def test_cli_preserves_explicit_device_pair(tmp_path, monkeypatch):
    observed = {}
    def capture(**kwargs):
        observed.update(kwargs)
        return tmp_path / "sealed"
    monkeypatch.setattr("three_device_slam.edge_rk3576.rsusb_capture.capture_rsusb_session", capture)
    assert main(["--output-root", str(tmp_path), "--d405-sdk-serial", "sdk-A",
                 "--d405-usb-serial", "usb-A", "--stm32-port", "/dev/serial/by-id/paired-stm32"]) == 0
    assert observed["d405_sdk_serial"] == "sdk-A"
    assert observed["d405_usb_serial"] == "usb-A"
    assert observed["stm32_port"] == "/dev/serial/by-id/paired-stm32"
