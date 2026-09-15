from pathlib import Path
from types import SimpleNamespace

import pytest

from three_device_slam.devices.d435i_ego import worker as d435i_worker

from three_device_slam.devices.d435i_ego.capture import (
    D435iContract,
    StreamSample,
    build_capture_acceptance,
    configure_streams,
    derive_monotonic_acquisition_ns,
    evaluate_stream,
    is_warmup_sample,
)
from three_device_slam.devices.d435i_ego.worker import (
    _disable_infrared_emitter,
    _warmup_timeout_seconds,
    _write_joint_terminal_acceptance,
    parse_args,
)


class RecordingConfig:
    def __init__(self):
        self.device_serial = None
        self.streams = []

    def enable_device(self, serial):
        self.device_serial = serial

    def enable_stream(self, *args):
        self.streams.append(args)


class FakeDepthSensor:
    def __init__(self, *, supported=True, readback=0.0):
        self.supported = supported
        self.readback = readback
        self.set_calls = []

    def supports(self, option):
        return self.supported

    def set_option(self, option, value):
        self.set_calls.append((option, value))

    def get_option(self, option):
        return self.readback

    def get_info(self, key):
        return "Stereo Module"


class FakeDevice:
    def __init__(self, sensor):
        self.sensor = sensor

    def first_depth_sensor(self):
        return self.sensor


def fake_rs():
    return SimpleNamespace(
        stream=SimpleNamespace(infrared="infrared", gyro="gyro", accel="accel"),
        format=SimpleNamespace(y8="y8", motion_xyz32f="motion_xyz32f"),
    )


def test_d435i_emitter_is_explicitly_disabled_and_read_back():
    rs = SimpleNamespace(
        option=SimpleNamespace(emitter_enabled="emitter_enabled"),
        camera_info=SimpleNamespace(name="name"),
    )
    sensor = FakeDepthSensor()

    result = _disable_infrared_emitter(rs, FakeDevice(sensor))

    assert sensor.set_calls == [("emitter_enabled", 0.0)]
    assert result["status"] == "PASS"
    assert result["readback"] == 0.0


def test_d435i_emitter_disable_fails_closed_when_unsupported_or_not_applied():
    rs = SimpleNamespace(
        option=SimpleNamespace(emitter_enabled="emitter_enabled"),
        camera_info=SimpleNamespace(name="name"),
    )
    with pytest.raises(RuntimeError, match="unsupported"):
        _disable_infrared_emitter(rs, FakeDevice(FakeDepthSensor(supported=False)))
    with pytest.raises(RuntimeError, match="readback"):
        _disable_infrared_emitter(rs, FakeDevice(FakeDepthSensor(readback=1.0)))


def samples(count, interval_ns, *, width=None, height=None):
    return [
        StreamSample(
            sequence=index + 1,
            acquisition_ns=1_000_000_000 + index * interval_ns,
            arrival_ns=1_001_000_000 + index * interval_ns,
            timestamp_domain="global_time",
            width=width,
            height=height,
        )
        for index in range(count)
    ]


def test_default_contract_is_720p_stereo_and_200hz_motion():
    contract = D435iContract()

    assert (contract.width, contract.height, contract.camera_hz) == (1280, 720, 30)
    assert contract.gyro_hz == 200
    assert contract.accel_hz == 200
    assert contract.writer_queue_depth == 512


def test_configure_streams_requires_explicit_serial_and_exact_profiles():
    config = RecordingConfig()

    configure_streams(fake_rs(), config, "327122078613", D435iContract())

    assert config.device_serial == "327122078613"
    assert config.streams == [
        ("infrared", 1, 1280, 720, "y8", 30),
        ("infrared", 2, 1280, 720, "y8", 30),
        ("gyro", "motion_xyz32f", 200),
        ("accel", "motion_xyz32f", 200),
    ]

    with pytest.raises(ValueError, match="serial"):
        configure_streams(fake_rs(), RecordingConfig(), "", D435iContract())


def test_global_time_is_explicitly_mapped_to_host_monotonic():
    assert derive_monotonic_acquisition_ns(
        device_timestamp_ms=1_787_900_000_000.125,
        wall_to_monotonic_offset_ns=1_787_600_000_000_000_000,
    ) == 300_000_000_125_000


def test_formal_boundary_uses_acquisition_not_later_usb_arrival_time():
    assert is_warmup_sample(
        formal_start_ns=1_000,
        acquisition_ns=999,
        timestamp_valid=True,
    ) is True
    assert is_warmup_sample(
        formal_start_ns=1_000,
        acquisition_ns=1_000,
        timestamp_valid=True,
    ) is False
    assert is_warmup_sample(
        formal_start_ns=1_000,
        acquisition_ns=2_000,
        timestamp_valid=False,
    ) is True


def test_stream_evaluation_accepts_exact_camera_and_motion_contracts():
    camera = evaluate_stream(
        samples(300, 33_333_333, width=1280, height=720),
        expected_hz=30,
        hz_tolerance=0.05,
        expected_resolution=(1280, 720),
    )
    gyro = evaluate_stream(
        samples(2000, 5_000_000),
        expected_hz=200,
        hz_tolerance=0.05,
    )

    assert camera["status"] == "PASS"
    assert camera["sequence_gaps"] == 0
    assert camera["timestamp_regressions"] == 0
    assert camera["resolution_mismatches"] == 0
    assert gyro["status"] == "PASS"


def test_stream_evaluation_fails_gap_regression_and_wrong_resolution():
    observed = samples(10, 33_333_333, width=640, height=480)
    observed[5] = StreamSample(
        sequence=7,
        acquisition_ns=observed[4].acquisition_ns - 1,
        arrival_ns=observed[5].arrival_ns,
        timestamp_domain="global_time",
        width=640,
        height=480,
    )

    report = evaluate_stream(
        observed,
        expected_hz=30,
        hz_tolerance=0.05,
        expected_resolution=(1280, 720),
    )

    assert report["status"] == "FAIL"
    assert report["sequence_gaps"] >= 1
    assert report["timestamp_regressions"] == 1
    assert report["resolution_mismatches"] == 10


def test_capture_acceptance_never_hides_queue_drop_or_operator_abort():
    passing = {
        "ir_left": evaluate_stream(
            samples(300, 33_333_333, width=1280, height=720),
            expected_hz=30,
            hz_tolerance=0.05,
            expected_resolution=(1280, 720),
        ),
        "ir_right": evaluate_stream(
            samples(300, 33_333_333, width=1280, height=720),
            expected_hz=30,
            hz_tolerance=0.05,
            expected_resolution=(1280, 720),
        ),
        "gyro": evaluate_stream(
            samples(2000, 5_000_000),
            expected_hz=200,
            hz_tolerance=0.05,
        ),
        "accel": evaluate_stream(
            samples(2000, 5_000_000),
            expected_hz=200,
            hz_tolerance=0.05,
        ),
    }

    assert build_capture_acceptance(
        passing, writer_queue_drops=0, operator_aborted=False, capture_error=None
    )["status"] == "PASS"
    assert build_capture_acceptance(
        passing, writer_queue_drops=1, operator_aborted=False, capture_error=None
    )["status"] == "FAIL"
    assert build_capture_acceptance(
        passing, writer_queue_drops=0, operator_aborted=True, capture_error=None
    )["status"] == "FAIL"
    assert build_capture_acceptance(
        passing,
        writer_queue_drops=0,
        operator_aborted=False,
        capture_error=None,
        formal_start_ns=1_000,
        first_acquisition_ns=999,
    )["status"] == "FAIL"


def test_cli_requires_serial_and_output_and_keeps_preview_enabled_by_default():
    args = parse_args(
        [
            "--serial",
            "327122078613",
            "--output",
            "/home/robot/three-device-slam/artifacts/d435i/run1",
            "--duration",
            "10",
        ]
    )

    assert args.serial == "327122078613"
    assert args.output == Path(
        "/home/robot/three-device-slam/artifacts/d435i/run1"
    )
    assert args.duration == 10
    assert args.preview_hz == 5
    assert args.no_preview is False

    with pytest.raises(SystemExit):
        parse_args(["--output", "/tmp/run", "--duration", "10"])


def test_cli_accepts_joint_coordinator_contract_without_standalone_output():
    args = parse_args(
        [
            "--serial",
            "327122078613",
            "--session",
            "/tmp/session",
            "--barrier-dir",
            "/tmp/session/barrier",
            "--device-id",
            "ego",
            "--calibration-id",
            "d435i-cal-v1",
            "--duration",
            "10",
        ]
    )

    assert args.output is None
    assert args.session == Path("/tmp/session")
    assert args.calibration_id == "d435i-cal-v1"


def test_joint_prestart_blocker_writes_coordinator_readable_acceptance(tmp_path):
    args = SimpleNamespace(
        session=tmp_path / "session",
        barrier_dir=tmp_path / "session" / "barrier",
        calibration_id="d435i-cal-v1",
    )
    args.session.mkdir()

    _write_joint_terminal_acceptance(
        args, status="BLOCKED", reason="d435i_hardware_unavailable"
    )

    payload = __import__("json").loads(
        (args.session / "ego" / "acceptance.json").read_text()
    )
    assert payload == {
        "schema": "ego.d435i.acceptance.v1",
        "status": "BLOCKED",
        "reason": "d435i_hardware_unavailable",
        "calibration_id": "d435i-cal-v1",
        "formal_evidence": {},
        "hashes": {},
    }


def test_joint_warmup_timeout_covers_coordinator_launch_window():
    assert _warmup_timeout_seconds(2.0, joint_mode=True) >= 30.0
    assert _warmup_timeout_seconds(2.0, joint_mode=False) == 10.0


def test_d435i_run_forwards_shared_realsense_context(monkeypatch):
    context = object()
    observed = []
    monkeypatch.setattr(
        d435i_worker,
        "capture",
        lambda args, *, realsense_context=None: observed.append(realsense_context)
        or {"status": "PASS"},
    )

    report = d435i_worker.run(SimpleNamespace(session=None), realsense_context=context)

    assert report == {"status": "PASS"}
    assert observed == [context]
