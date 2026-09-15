import csv
import hashlib
import json
import subprocess
import struct
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from three_device_slam.devices.d405_umi import worker as capture_quality_module

rs = capture_quality_module.rs
IDENTITY_ARGS = [
    "--serial",
    "test-d405",
    "--imu-port",
    "/dev/serial/by-id/test-imu",
]


from three_device_slam.devices.d405_umi.worker import (
    CameraClockEvidence,
    JointBarrierController,
    JointCaptureHealth,
    JointWorkerFailure,
    MetadataFrame,
    OutputProgress,
    PRODUCT_CAMERA_IMU_TD_S,
    StreamContinuity,
    analyze_ir_exposure,
    barrier_root,
    build_joint_start_acceptance,
    build_parser,
    capture_streams_for_mode,
    capture_session_directory,
    configure_global_time,
    configure_ir_auto_exposure,
    count_persisted_imu_samples,
    decode_cdr_string,
    drain_frame_queue,
    metadata_frame_from_text,
    pair_metadata_frames,
    parse_args,
    pause_joint_recorder_for_shutdown,
    persisted_imu_covers_formal,
    required_staging_bytes,
    run_prestart_step,
    stats_delta,
    timestamps_aligned,
    live_vins_timestamp_monotonic,
    imu_transport_accepted,
    imu_reader_warmup_frames,
    write_frames_csv,
)


class FakeFrameQueue:
    def __init__(self, frames):
        self.frames = list(frames)
        self.polls = 0

    def poll_for_frame(self):
        self.polls += 1
        return self.frames.pop(0) if self.frames else None


def test_joint_monitor_drains_available_frames_before_barrier_io():
    queue = FakeFrameQueue(["color", "left", "right", "next-color"])

    assert drain_frame_queue(queue, limit=3) == ("color", "left", "right")
    assert queue.frames == ["next-color"]
    assert queue.polls == 3

    assert drain_frame_queue(queue, limit=64) == ("next-color",)
    assert queue.polls == 5


def test_joint_formal_imu_count_uses_persisted_monotonic_window(tmp_path):
    path = tmp_path / "imu_ts.csv"
    path.write_text(
        "counter,ts_mono,rx_mono,ts_wall\n"
        "1,0.9975,0.9976,10.0\n"
        "2,1.0000,1.0001,10.1\n"
        "3,1.0025,1.0026,10.2\n"
        "4,2.0000,2.0001,11.0\n"
        "5,2.0025,2.0026,11.1\n",
        encoding="utf-8",
    )

    assert count_persisted_imu_samples(
        path, start_ns=1_000_000_000, stop_ns=2_000_000_000
    ) == 3


def test_joint_formal_imu_count_rejects_invalid_persisted_timestamp(tmp_path):
    path = tmp_path / "imu_ts.csv"
    path.write_text("counter,ts_mono\n1,not-a-time\n", encoding="utf-8")

    with pytest.raises(ValueError, match="ts_mono"):
        count_persisted_imu_samples(path, start_ns=0, stop_ns=1)


def test_cli_can_pin_the_physical_imu_protocol():
    args = parse_args(
        [*IDENTITY_ARGS, "--imu-protocol", "stm32_combined_v1"]
    )

    assert args.imu_protocol == "stm32_combined_v1"

    with pytest.raises(SystemExit):
        parse_args([*IDENTITY_ARGS, "--imu-protocol", "invented"])


def test_d405_run_forwards_shared_realsense_context(monkeypatch):
    context = object()
    observed = []
    monkeypatch.setattr(
        capture_quality_module,
        "_run_claimed",
        lambda args, *, realsense_context=None: observed.append(realsense_context)
        or 0,
    )

    result = capture_quality_module.run(
        SimpleNamespace(session=None), realsense_context=context
    )

    assert result == 0
    assert observed == [context]


class FakeOptionRange:
    def __init__(self, minimum, maximum):
        self.min = minimum
        self.max = maximum


class FakeSensor:
    def __init__(self):
        self.values = {}
        self.set_calls = []

    def supports(self, _option):
        return True

    def get_option_range(self, option):
        if "gain" in str(option).lower():
            return FakeOptionRange(16.0, 248.0)
        return FakeOptionRange(1.0, 165000.0)

    def set_option(self, option, value):
        self.values[option] = value
        self.set_calls.append((option, value))

    def get_option(self, option):
        return self.values[option]


class MismatchedReadbackSensor(FakeSensor):
    def get_option(self, option):
        value = super().get_option(option)
        if "exposure limit" in str(option).lower():
            return value + 1000.0
        return value


class FakeGlobalTimeSensor:
    def __init__(self, *, supported=True, readback=1.0):
        self.supported = supported
        self.readback = readback
        self.set_calls = []

    def supports(self, option):
        assert option == rs.option.global_time_enabled
        return self.supported

    def set_option(self, option, value):
        self.set_calls.append((option, value))

    def get_option(self, option):
        assert option == rs.option.global_time_enabled
        return self.readback


class FakeBarrier:
    def __init__(self, start_ns=None, stop_record=None):
        self.start_ns = start_ns
        self.stop_record = stop_record
        self.stop_reads = 0
        self.heartbeats = []
        self.failures = []

    def read_start_ns(self):
        return self.start_ns

    def write_heartbeat(self, heartbeat):
        self.heartbeats.append(heartbeat)

    def read_stop(self):
        self.stop_reads += 1
        return self.stop_record

    def latch_failure(self, device_id, reason, at_ns):
        if not self.failures:
            self.failures.append((device_id, reason, at_ns))


def test_stream_continuity_reports_frame_gaps_and_timestamp_regression():
    stats = StreamContinuity()
    stats.add(10, 1000.0)
    stats.add(11, 1033.333)
    stats.add(14, 1133.333)
    stats.add(15, 1100.0)

    report = stats.report()

    assert report["received"] == 4
    assert report["skipped_frames"] == 2
    assert report["gap_events"] == 1
    assert report["timestamp_regressions"] == 1
    assert report["gap_ratio"] == 0.4


def test_global_time_configuration_is_unverified_when_option_is_unsupported():
    sensor = FakeGlobalTimeSensor(supported=False)

    assert configure_global_time(sensor) == {
        "supported": False,
        "requested": 1.0,
        "readback": None,
        "verified": False,
    }
    assert sensor.set_calls == []


def test_global_time_configuration_is_unverified_on_readback_mismatch():
    sensor = FakeGlobalTimeSensor(readback=0.0)

    assert configure_global_time(sensor) == {
        "supported": True,
        "requested": 1.0,
        "readback": 0.0,
        "verified": False,
    }
    assert sensor.set_calls == [(rs.option.global_time_enabled, 1.0)]


def test_global_time_configuration_requires_successful_readback():
    sensor = FakeGlobalTimeSensor(readback=1.0)

    assert configure_global_time(sensor)["verified"] is True


@pytest.mark.parametrize(
    "domains",
    (
        (rs.timestamp_domain.system_time,) * 3,
        (rs.timestamp_domain.hardware_clock,) * 3,
        (
            rs.timestamp_domain.global_time,
            rs.timestamp_domain.system_time,
            rs.timestamp_domain.hardware_clock,
        ),
    ),
)
def test_camera_clock_evidence_rejects_system_hardware_and_mixed_domains(domains):
    evidence = CameraClockEvidence(
        ("color", "infrared_left", "infrared_right"),
        {"supported": True, "requested": 1.0, "readback": 1.0, "verified": True},
    )
    for stream, domain in zip(evidence.stream_keys, domains):
        evidence.observe(stream, domain)

    assert evidence.report()["verified"] is False


def test_camera_clock_evidence_requires_every_stream_observed():
    evidence = CameraClockEvidence(
        ("color", "infrared_left", "infrared_right"),
        {"supported": True, "requested": 1.0, "readback": 1.0, "verified": True},
    )
    evidence.observe("color", rs.timestamp_domain.global_time)
    evidence.observe("infrared_left", rs.timestamp_domain.global_time)

    report = evidence.report()

    assert report["verified"] is False
    assert report["observed_domains"]["infrared_right"] == {}


def test_camera_clock_evidence_verifies_all_required_streams_are_global_time():
    configuration = {
        "supported": True,
        "requested": 1.0,
        "readback": 1.0,
        "verified": True,
    }
    evidence = CameraClockEvidence(
        ("color", "infrared_left", "infrared_right"), configuration
    )
    for stream in evidence.stream_keys:
        evidence.observe(stream, rs.timestamp_domain.global_time)
        evidence.observe(stream, rs.timestamp_domain.global_time)

    assert evidence.report() == {
        "configuration": configuration,
        "required_streams": ["color", "infrared_left", "infrared_right"],
        "observed_domains": {
            "color": {"global_time": 2},
            "infrared_left": {"global_time": 2},
            "infrared_right": {"global_time": 2},
        },
        "verified": True,
    }


def test_depth_stereo_mode_replaces_color_without_dropping_dual_ir():
    names = [item[0] for item in capture_streams_for_mode("depth_stereo_ir")]

    assert names == ["depth", "infrared_left", "infrared_right"]


def test_live_vins_uses_the_same_global_time_mapping_as_recorded_db3():
    """Live VINS must consume the exposure time later reconstructed for replay."""
    assert live_vins_timestamp_monotonic(1_785_952_580_123.0, 1_785_952_000.0) == pytest.approx(580.123)


def test_decode_cdr_string_reads_ros2_std_msgs_string():
    value = "Frame number=42;timestamp=1234.5;"
    encoded = value.encode() + b"\0"
    blob = b"\x00\x01\x00\x00" + struct.pack("<I", len(encoded)) + encoded

    assert decode_cdr_string(blob) == value


def test_metadata_parser_reads_actual_exposure_and_gain():
    frame = metadata_frame_from_text(
        "Frame number=42;timestamp=1234.5;Actual Exposure=7954;Gain Level=209;"
    )

    assert frame == MetadataFrame(42, 1234.5, exposure_us=7954.0, gain=209.0)


def test_ir_exposure_report_rejects_recorded_frame_over_limit():
    records = {
        "color": [MetadataFrame(1, 1000.0)],
        "infrared_left": [MetadataFrame(1, 1000.0, 7954.0, 209.0)],
        "infrared_right": [MetadataFrame(1, 1000.0, 8200.0, 209.0)],
    }

    report = analyze_ir_exposure(records, limit_us=8000.0)

    assert report["result"] == "FAIL"
    assert report["streams"]["infrared_left"]["result"] == "PASS"
    assert report["streams"]["infrared_right"]["result"] == "FAIL"


def test_ir_exposure_report_requires_complete_metadata():
    records = {
        "infrared_left": [MetadataFrame(1, 1000.0, 7954.0, None)],
        "infrared_right": [],
    }

    report = analyze_ir_exposure(records, limit_us=8000.0)

    assert report["result"] == "FAIL"
    assert report["streams"]["infrared_left"]["metadata_complete"] is False
    assert report["streams"]["infrared_right"]["metadata_complete"] is False


def test_ir_exposure_report_accepts_exact_tolerance_boundary():
    records = {
        key: [MetadataFrame(1, 1000.0, 8100.0, 248.0)]
        for key in ("infrared_left", "infrared_right")
    }

    assert analyze_ir_exposure(records, limit_us=8000.0)["result"] == "PASS"


def test_configure_ir_auto_exposure_sets_limits_before_streaming():
    sensor = FakeSensor()

    report = configure_ir_auto_exposure(sensor, exposure_limit_us=8000.0, gain_limit=248.0)

    assert report["result"] == "PASS"
    assert report["applied"]["auto_exposure_limit_us"] == 8000.0
    assert report["applied"]["auto_gain_limit"] == 248.0
    assert len(sensor.set_calls) == 5


def test_configure_ir_auto_exposure_rejects_mismatched_readback():
    sensor = MismatchedReadbackSensor()

    with pytest.raises(RuntimeError, match="读回值不一致"):
        configure_ir_auto_exposure(
            sensor, exposure_limit_us=8000.0, gain_limit=248.0
        )


def test_stats_delta_excludes_warmup_counts():
    start = {"frames_ok": 500, "frames_bad": 3, "dropped_frames": 1}
    end = {"frames_ok": 4500, "frames_bad": 3, "dropped_frames": 1}

    assert stats_delta(start, end) == {
        "frames_ok": 4000,
        "frames_bad": 0,
        "dropped_frames": 0,
    }


def test_stm32_transport_gate_requires_outer_protocol_evidence():
    clean = {
        "frames_bad": 0,
        "resyncs": 0,
        "dropped_frames": 0,
        "counter_resets": 0,
        "counter_stalls": 0,
        "sequence_gaps": 0,
        "invalid_imu_flags": 0,
        "queue_overflow_flags": 0,
        "serial_errors": 0,
        "serial_reconnects": 0,
    }

    assert imu_transport_accepted("stm32_combined_v1", 400.0, clean, 0)
    assert not imu_transport_accepted("mixed", 400.0, clean, 0)
    assert not imu_transport_accepted(
        "stm32_combined_v1", 400.0, {**clean, "sequence_gaps": 1}, 0
    )


def test_legacy_transport_gate_remains_backward_compatible():
    clean = {
        "frames_bad": 0,
        "resyncs": 0,
        "dropped_frames": 0,
        "counter_resets": 0,
        "counter_stalls": 0,
        "serial_errors": 0,
        "serial_reconnects": 0,
    }

    assert imu_transport_accepted("kt_ex9_37", 400.0, clean, 0)


def test_timestamps_aligned_matches_d405_rgb_and_ir_without_equal_frame_numbers():
    assert timestamps_aligned([840.3936, 839.8533, 839.8533], 2.0)
    assert not timestamps_aligned([840.3936, 806.6047, 806.6047], 2.0)


def test_pair_metadata_frames_skips_missing_stream_frame_without_cross_pairing():
    frame = lambda number, timestamp: MetadataFrame(number, timestamp)
    records = {
        "color": [frame(10, 1000.5), frame(11, 1033.8), frame(12, 1067.2)],
        "infrared_left": [frame(20, 1000.0), frame(22, 1066.7)],
        "infrared_right": [frame(20, 1000.0), frame(21, 1033.3), frame(22, 1066.7)],
    }

    pairs = pair_metadata_frames(records, 2.0)

    assert [pair["color"].number for pair in pairs] == [10, 12]
    assert [pair["infrared_left"].number for pair in pairs] == [20, 22]
    assert [pair["infrared_right"].number for pair in pairs] == [20, 22]


def test_required_staging_bytes_includes_headroom_and_disables_unbounded_capture():
    assert required_staging_bytes(10.0) == 1_271_808_000
    assert required_staging_bytes(0.0) == 0


def test_write_frames_csv_uses_global_time_not_relative_bag_timestamp(tmp_path):
    frame = lambda number, device_ms: MetadataFrame(number, device_ms)
    records = {
        "color": [frame(10, 1_786_463_632_107.8)],
        "infrared_left": [frame(20, 1_786_463_632_107.3)],
        "infrared_right": [frame(20, 1_786_463_632_107.3)],
    }
    path = tmp_path / "frames.csv"

    assert write_frames_csv(path, records, 1_785_952_580.3986, 2.0) == 1
    row = next(csv.DictReader(path.open()))

    assert float(row["arrival_wall"]) > 1_700_000_000
    assert 500_000 < float(row["arrival_mono"]) < 600_000
    assert row["warmup"] == "0"
    assert {
        row[f"{stream}_domain"]
        for stream in ("color", "infrared_left", "infrared_right")
    } == {"global_time"}


def test_joint_barrier_arguments_are_optional():
    args = build_parser().parse_args(IDENTITY_ARGS)

    assert args.barrier_dir is None
    assert args.session is None
    assert args.device_id == "d405"


def test_worker_requires_explicit_device_identity():
    with pytest.raises(SystemExit):
        parse_args([])


def test_duration_omission_is_indefinite_until_stop():
    assert build_parser().parse_args(IDENTITY_ARGS).duration is None


@pytest.mark.parametrize("duration", ["nan", "inf", "-inf", "0", "-1"])
def test_parser_rejects_non_finite_and_non_positive_duration(duration):
    with pytest.raises(SystemExit):
        parse_args([*IDENTITY_ARGS, "--duration", duration])


def test_joint_worker_preserves_product_td():
    assert PRODUCT_CAMERA_IMU_TD_S == -0.009312


def test_joint_frames_csv_marks_prestart_rows_as_warmup(tmp_path):
    rows = {
        "infrared_left": [MetadataFrame(1, 1000.0), MetadataFrame(2, 1033.333)],
        "infrared_right": [MetadataFrame(1, 1000.0), MetadataFrame(2, 1033.333)],
        "color": [MetadataFrame(1, 1000.5), MetadataFrame(2, 1033.833)],
    }
    output = tmp_path / "frames.csv"

    write_frames_csv(
        output,
        rows,
        epoch_offset=0.0,
        tolerance_ms=2.0,
        formal_start_device_ms=1033.0,
    )

    written = list(csv.DictReader(output.open()))
    assert [row["warmup"] for row in written] == ["1", "0"]


def test_joint_frames_csv_uses_explicit_unverified_timestamp_domain(tmp_path):
    rows = {
        key: [MetadataFrame(1, 1000.0)]
        for key in ("color", "infrared_left", "infrared_right")
    }
    output = tmp_path / "frames.csv"

    write_frames_csv(
        output,
        rows,
        epoch_offset=0.0,
        tolerance_ms=2.0,
        timestamp_domain="unverified",
    )

    written = next(csv.DictReader(output.open()))
    assert {
        written[f"{stream}_domain"]
        for stream in ("color", "infrared_left", "infrared_right")
    } == {"unverified"}


def test_joint_arguments_require_session_and_hand_device(tmp_path):
    with pytest.raises(SystemExit):
        parse_args([*IDENTITY_ARGS, "--barrier-dir", str(tmp_path)])
    with pytest.raises(SystemExit):
        parse_args(
            [*IDENTITY_ARGS, "--barrier-dir", str(tmp_path), "--device-id", "left"]
        )

    args = parse_args(
        [
            *IDENTITY_ARGS,
            "--barrier-dir",
            str(tmp_path / "barrier"),
            "--session",
            str(tmp_path),
            "--device-id",
            "right",
        ]
    )
    assert args.device_id == "right"


def test_phase_one_parser_rejects_live_vins_in_all_modes(tmp_path):
    with pytest.raises(SystemExit):
        parse_args([*IDENTITY_ARGS, "--publish-vins"])
    with pytest.raises(SystemExit):
        parse_args(
            [
                *IDENTITY_ARGS,
                "--barrier-dir",
                str(tmp_path / "barrier"),
                "--session",
                str(tmp_path),
                "--device-id",
                "left",
                "--publish-vins",
            ]
        )


def test_worker_import_does_not_load_hardware_modules():
    code = """
import sys
import three_device_slam.devices.d405_umi.worker
assert 'cv2' not in sys.modules
assert 'pyrealsense2' not in sys.modules
"""
    subprocess.run([sys.executable, "-c", code], check=True)


def test_barrier_root_normalizes_session_and_barrier_paths(tmp_path):
    assert barrier_root(tmp_path) == tmp_path
    assert barrier_root(tmp_path / "barrier") == tmp_path


def test_joint_start_acceptance_declares_host_monotonic_clock(tmp_path):
    report = build_joint_start_acceptance(
        device_id="left",
        barrier_dir=tmp_path / "barrier",
        formal_start_host_monotonic_ns=123_456_789,
        first_formal_camera_device_ms=9876.5,
        first_formal_imu_counter=42,
        clock_domain="host_monotonic",
    )

    assert report == {
        "device_id": "left",
        "barrier_directory": str(tmp_path / "barrier"),
        "formal_start_host_monotonic_ns": 123_456_789,
        "clock_domain": "host_monotonic",
        "first_formal_camera_device_ms": 9876.5,
        "first_formal_imu_counter": 42,
        "raw_recording_is_append_only": True,
        "raw_warmup_preserved": True,
        "classification_metadata": "d405_frames.csv:warmup",
    }


def test_joint_start_acceptance_preserves_computed_unverified_clock(tmp_path):
    report = build_joint_start_acceptance(
        device_id="right",
        barrier_dir=tmp_path,
        formal_start_host_monotonic_ns=123,
        first_formal_camera_device_ms=None,
        first_formal_imu_counter=None,
        clock_domain="unverified",
    )

    assert report["clock_domain"] == "unverified"


def test_clean_d405_capture_with_unverified_clock_is_blocked():
    classify = capture_quality_module.classify_d405_result
    assert classify(True, True, True, True, False) == "BLOCKED"
    assert classify(False, True, True, True, False) == "FAIL"
    assert classify(True, True, True, True, True) == "PASS"


def test_d405_raw_manifest_hashes_only_existing_regular_files(tmp_path):
    db3 = tmp_path / "camera.db3"
    raw = tmp_path / "external_imu" / "raw_imu_packets.bin"
    db3.write_bytes(b"db3")
    raw.parent.mkdir()
    raw.write_bytes(b"packet")

    manifest = capture_quality_module.build_raw_file_manifest(
        tmp_path, [db3, raw, tmp_path / "missing.csv"]
    )

    assert set(manifest) == {"camera.db3", "external_imu/raw_imu_packets.bin"}
    assert manifest["camera.db3"]["size_bytes"] == 3
    assert len(manifest["external_imu/raw_imu_packets.bin"]["sha256"]) == 64


def test_joint_controller_rejects_stale_start_and_latches_first_failure():
    barrier = FakeBarrier(start_ns=399_999_999)
    controller = JointBarrierController(barrier, "left")

    with pytest.raises(JointWorkerFailure, match="stale_start") as raised:
        controller.poll(300_000_000, True, ())

    assert raised.value.exit_code == 2
    assert barrier.failures == [("left", "stale_start", 300_000_000)]


def test_joint_controller_rejects_start_change_even_after_installation():
    barrier = FakeBarrier(start_ns=500_000_000)
    controller = JointBarrierController(barrier, "right")
    assert controller.poll(300_000_000, True, ()) is None
    barrier.start_ns = 600_000_000

    with pytest.raises(JointWorkerFailure, match="changed_start"):
        controller.poll(350_000_000, True, ())

    assert barrier.failures == [("right", "changed_start", 350_000_000)]


def test_joint_controller_latches_terminal_failure_after_scheduled_start():
    barrier = FakeBarrier(start_ns=500_000_000)
    controller = JointBarrierController(barrier, "left")
    controller.poll(300_000_000, True, ())
    assert controller.poll(500_000_000, True, ()) == 500_000_000

    with pytest.raises(JointWorkerFailure, match="storage_error"):
        controller.poll(
            550_000_000,
            False,
            ("storage_error",),
            terminal_reason="storage_error",
        )

    assert barrier.failures == [("left", "storage_error", 550_000_000)]


def test_joint_controller_heartbeats_at_most_100_ms_apart():
    barrier = FakeBarrier()
    controller = JointBarrierController(barrier, "left")

    controller.poll(1, False, ("camera_disconnect", "imu_disconnect"))
    controller.poll(50_000_000, True, ())
    controller.poll(100_000_001, True, ())

    assert [item.at_ns for item in barrier.heartbeats] == [1, 100_000_001]
    assert barrier.heartbeats[0].reasons == (
        "camera_disconnect",
        "imu_disconnect",
    )


def test_joint_controller_keeps_cross_start_backlog_warmup(tmp_path):
    barrier = FakeBarrier(start_ns=500_000_000)
    controller = JointBarrierController(
        barrier, "left", ("color", "infrared_left", "infrared_right")
    )
    controller.poll(300_000_000, True, ())
    assert controller.poll(500_000_000, True, ()) == 500_000_000

    controller.observe_imu(499_999_999, 41)
    controller.observe_imu(500_000_001, 42)
    controller.observe_imu(502_500_001, 43)

    for key, timestamp in (
        ("infrared_left", 1000.0),
        ("infrared_right", 1000.0),
        ("color", 1000.5),
    ):
        controller.observe_camera(key, timestamp)
    assert controller.first_formal_camera_device_ms is None

    controller.mark_camera_backlog_drained()
    for key, timestamp in (
        ("infrared_left", 1033.333),
        ("infrared_right", 1033.333),
        ("color", 1033.833),
    ):
        controller.observe_camera(key, timestamp)

    rows = {
        "infrared_left": [MetadataFrame(1, 1000.0), MetadataFrame(2, 1033.333)],
        "infrared_right": [MetadataFrame(1, 1000.0), MetadataFrame(2, 1033.333)],
        "color": [MetadataFrame(1, 1000.5), MetadataFrame(2, 1033.833)],
    }
    output = tmp_path / "frames.csv"
    write_frames_csv(
        output,
        rows,
        epoch_offset=0.0,
        tolerance_ms=2.0,
        formal_start_device_ms=controller.first_formal_camera_device_ms,
    )

    assert controller.first_formal_camera_device_ms == 1033.833
    assert controller.first_formal_imu_counter == 42
    assert [row["warmup"] for row in csv.DictReader(output.open())] == ["1", "0"]


def test_capture_session_directory_preserves_standalone_naming(tmp_path):
    standalone = build_parser().parse_args(
        [*IDENTITY_ARGS, "--output-root", str(tmp_path)]
    )
    joint = parse_args(
        [
            *IDENTITY_ARGS,
            "--barrier-dir",
            str(tmp_path / "barrier"),
            "--session",
            str(tmp_path / "joint"),
            "--device-id",
            "left",
        ]
    )

    assert capture_session_directory(standalone, "20260825_120000") == (
        tmp_path / "d405_720p_rgb_stereo_ir_20260825_120000"
    )
    assert capture_session_directory(joint, "ignored") == tmp_path / "joint" / "left"


def test_joint_worker_rejects_sealed_session_before_loading_hardware(
    tmp_path, monkeypatch
):
    session = tmp_path / "session"
    session.mkdir()
    (session / "session.seal.json").write_text("sealed", encoding="utf-8")
    args = parse_args(
        [
            *IDENTITY_ARGS,
            "--barrier-dir",
            str(session),
            "--session",
            str(session),
            "--device-id",
            "left",
        ]
    )
    hardware_loaded = []
    monkeypatch.setattr(capture_quality_module, "parse_args", lambda: args)
    monkeypatch.setattr(
        capture_quality_module,
        "_load_hardware_modules",
        lambda: hardware_loaded.append(True),
    )

    with pytest.raises(RuntimeError, match="sealed"):
        capture_quality_module._main()

    assert hardware_loaded == []
    assert {path.name for path in session.iterdir()} == {"session.seal.json"}


def test_joint_reader_keeps_raw_imu_warmup_while_standalone_still_discards_it():
    assert imu_reader_warmup_frames(joint_mode=False) == 500
    assert imu_reader_warmup_frames(joint_mode=True) == 0


def test_joint_health_reports_only_device_recorder_timestamp_and_storage():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    for key in ("color", "infrared_left", "infrared_right"):
        health.observe_camera(key, 1000.0, 1_000_000_000)
    health.observe_imu(1_000_000_000, 1.0)

    assert health.evaluate(
        1_050_000_000,
        imu_stats={"frames_bad": 0, "serial_errors": 0},
        recorder_healthy=True,
        storage_healthy=True,
        disconnect_is_terminal=True,
    ) == (True, (), None)

    health.observe_camera("color", 999.0, 1_060_000_000)
    assert health.evaluate(
        1_060_000_000,
        imu_stats={"frames_bad": 0, "serial_errors": 0},
        recorder_healthy=True,
        storage_healthy=True,
        disconnect_is_terminal=True,
    ) == (False, ("timestamp_regression",), "timestamp_regression")


def test_d405_joint_ready_requires_clean_recent_three_second_30hz_window():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    for counter in range(1201):
        at_ns = start_ns + counter * 2_500_000
        health.observe_imu(at_ns, at_ns / 1_000_000_000, counter=counter)

    heartbeat_ns = start_ns + 3_010_000_000
    assert health.joint_ready(heartbeat_ns, clock_verified=True)
    assert not health.joint_ready(heartbeat_ns, clock_verified=False)

    health.observe_camera(
        "color", 4_033.333, start_ns + 3_033_333_333, frame_number=93
    )
    assert not health.joint_ready(start_ns + 3_033_333_333, clock_verified=True)


@pytest.mark.parametrize("imu_hz", [11, 30])
def test_d405_joint_ready_rejects_slow_recent_imu_window(imu_hz):
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    period_ns = int(1_000_000_000 / imu_hz)
    for counter in range(imu_hz * 3 + 1):
        at_ns = start_ns + counter * period_ns
        health.observe_imu(at_ns, at_ns / 1_000_000_000, counter=counter)

    assert not health.joint_ready(start_ns + 3_010_000_000, clock_verified=True)


def test_d405_joint_ready_rejects_recent_imu_counter_gap():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    for counter in range(1201):
        at_ns = start_ns + counter * 2_500_000
        observed_counter = counter + 1 if counter >= 600 else counter
        health.observe_imu(
            at_ns, at_ns / 1_000_000_000, counter=observed_counter
        )

    assert not health.joint_ready(start_ns + 3_010_000_000, clock_verified=True)


def test_d405_joint_ready_rejects_recent_imu_timestamp_regression():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    for counter in range(1201):
        at_ns = start_ns + counter * 2_500_000
        acquisition_s = at_ns / 1_000_000_000
        if counter == 900:
            acquisition_s -= 0.01
        health.observe_imu(at_ns, acquisition_s, counter=counter)

    assert not health.joint_ready(start_ns + 3_010_000_000, clock_verified=True)


def test_d405_joint_ready_accepts_uint32_imu_counter_wrap():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    initial_counter = 0xFFFFFFFF - 600
    for index in range(1201):
        at_ns = start_ns + index * 2_500_000
        health.observe_imu(
            at_ns,
            at_ns / 1_000_000_000,
            counter=(initial_counter + index) & 0xFFFFFFFF,
        )

    assert health.joint_ready(start_ns + 3_010_000_000, clock_verified=True)


@pytest.mark.parametrize(
    ("rate_hz", "expected_ready"),
    ((401.0, True), (401.001, False)),
)
def test_d405_joint_ready_imu_upper_rate_boundary_with_ns_quantization(
    rate_hz, expected_ready
):
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    intervals = 1203
    for index in range(intervals + 1):
        host_ns = start_ns + round(index * 3_000_000_000 / intervals)
        acquisition_ns = start_ns + round(index * 1_000_000_000 / rate_hz)
        health.observe_imu(
            host_ns, acquisition_ns / 1_000_000_000, counter=index
        )

    assert health.joint_ready(
        start_ns + 3_010_000_000, clock_verified=True
    ) is expected_ready


def test_d405_joint_ready_rejects_imu_acquisition_clock_one_second_behind():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    for counter in range(1201):
        host_ns = start_ns + counter * 2_500_000
        health.observe_imu(
            host_ns, (host_ns - 1_000_000_000) / 1_000_000_000, counter=counter
        )

    assert not health.joint_ready(start_ns + 3_010_000_000, clock_verified=True)


def test_d405_joint_ready_rejects_only_1_99_seconds_of_imu_acquisition_span():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))
    start_ns = 1_000_000_000
    for index in range(91):
        at_ns = start_ns + index * 33_333_333
        for key in health.stream_keys:
            health.observe_camera(key, at_ns / 1_000_000, at_ns, frame_number=index)
    for counter in range(797):
        host_ns = start_ns + round(counter * 3_000_000_000 / 796)
        acquisition_ns = start_ns + 12_500_000 + counter * 2_500_000
        health.observe_imu(
            host_ns, acquisition_ns / 1_000_000_000, counter=counter
        )

    assert not health.joint_ready(start_ns + 3_010_000_000, clock_verified=True)


def test_joint_health_latches_imu_recorder_and_disconnect_errors():
    health = JointCaptureHealth(("color", "infrared_left", "infrared_right"))

    healthy, reasons, terminal = health.evaluate(
        1_000_000_000,
        imu_stats={"frames_bad": 1},
        recorder_healthy=False,
        storage_healthy=False,
        disconnect_is_terminal=True,
    )

    assert not healthy
    assert reasons == (
        "camera_disconnect",
        "imu_disconnect",
        "imu_error",
        "recorder_error",
        "storage_error",
    )
    assert terminal == "imu_error"


def test_joint_health_latches_imu_timestamp_regression():
    health = JointCaptureHealth(())
    health.observe_imu(1_000_000_000, 10.0)
    health.observe_imu(1_000_000_001, 9.9)

    assert health.evaluate(
        1_000_000_001,
        imu_stats={},
        recorder_healthy=True,
        storage_healthy=True,
        disconnect_is_terminal=True,
    ) == (False, ("timestamp_regression",), "timestamp_regression")


def test_output_progress_requires_real_file_growth_after_grace(tmp_path):
    output = tmp_path / "capture.db3"
    progress = OutputProgress(output, grace_ns=100, stale_ns=50)

    assert progress.healthy(0)
    assert not progress.observed_growth
    assert not progress.healthy(101)
    output.write_bytes(b"first")
    assert progress.healthy(102)
    assert progress.observed_growth
    assert progress.healthy(152)
    assert not progress.healthy(153)
    output.write_bytes(b"first-more")
    assert progress.healthy(154)


def test_prestart_step_latches_specific_joint_failure_and_preserves_standalone():
    barrier = FakeBarrier()
    controller = JointBarrierController(barrier, "left")

    def fail():
        raise OSError("disk unavailable")

    with pytest.raises(JointWorkerFailure, match="storage_error") as raised:
        run_prestart_step(
            controller, "storage_error", fail, clock_ns=lambda: 123
        )
    assert raised.value.exit_code == 2
    assert barrier.failures == [("left", "storage_error", 123)]

    with pytest.raises(OSError, match="disk unavailable"):
        run_prestart_step(None, "storage_error", fail, clock_ns=lambda: 456)


def test_joint_recorder_pause_shutdown_failure_latches_without_raising():
    class FailingRecorder:
        def pause(self):
            raise OSError("native recorder flush failed")

    barrier = FakeBarrier()
    controller = JointBarrierController(barrier, "left")

    error = pause_joint_recorder_for_shutdown(
        FailingRecorder(), controller, clock_ns=lambda: 789
    )

    assert error == "OSError: native recorder flush failed"
    assert barrier.failures == [("left", "recorder_error", 789)]


@pytest.mark.parametrize(
    ("stop_record", "duration_s"),
    [
        ({"at_ns": 100, "reason": "operator_interrupt"}, None),
        (None, 0.0000001),
    ],
)
def test_terminal_failure_precedes_stop_or_duration_completion(
    stop_record, duration_s
):
    barrier = FakeBarrier(start_ns=0, stop_record=stop_record)
    controller = JointBarrierController(barrier, "left")
    controller.start_ns = 0

    with pytest.raises(JointWorkerFailure, match="storage_error"):
        capture_quality_module.poll_joint_formal_window(
            controller,
            now_ns=100,
            healthy=False,
            reasons=("storage_error",),
            terminal_reason="storage_error",
            start_ns=0,
            duration_s=duration_s,
        )

    assert barrier.failures == [("left", "storage_error", 100)]
    assert barrier.stop_reads == 0


def test_joint_finalization_flushes_hashes_then_publishes_acceptance(tmp_path):
    events = []
    bag_path = tmp_path / "camera.db3"

    class CameraRecorder:
        def pause(self):
            events.append("camera_pause")
            bag_path.write_bytes(b"camera")

    class TrackingUnitRecorder(capture_quality_module.UnitRecorder):
        def stop(self, *args, **kwargs):
            events.append("imu_stop")
            return super().stop(*args, **kwargs)

    barrier = FakeBarrier()
    controller = JointBarrierController(barrier, "left")
    recorder = TrackingUnitRecorder("external_imu", tmp_path, max_queue=8)
    recorder.start()
    recorder.put_raw_imu_packet(b"wire-packet")

    shutdown = capture_quality_module.shutdown_capture_outputs(
        pause_recorder=lambda: pause_joint_recorder_for_shutdown(
            CameraRecorder(), controller
        ),
        stop_sensor=lambda: events.append("sensor_stop"),
        close_sensor=lambda: events.append("sensor_close"),
        stop_imu=lambda: events.append("imu_device_stop"),
        imu_recorder=recorder,
    )
    acceptance_path = tmp_path / "acceptance.json"
    assert not acceptance_path.exists()

    report = capture_quality_module.finalize_joint_outputs(
        session=tmp_path,
        report={"result": "PASS"},
        raw_paths=(
            bag_path,
            tmp_path / "external_imu" / "raw_imu_packets.bin",
        ),
        camera_recorder_clean_shutdown=shutdown["camera_clean"],
        imu_recorder_clean_shutdown=shutdown["imu_clean"],
        recorder_write_error=shutdown["recorder_write_error"],
        camera_recorder_shutdown_error=shutdown["camera_error"],
        stage_move_error=None,
        shutdown_errors=shutdown["errors"],
    )

    assert events == [
        "camera_pause",
        "sensor_stop",
        "sensor_close",
        "imu_device_stop",
        "imu_stop",
    ]
    assert acceptance_path.exists()
    assert json.loads(acceptance_path.read_text()) == report
    assert report["raw_files"]["camera.db3"]["sha256"] == hashlib.sha256(
        b"camera"
    ).hexdigest()
    assert report["raw_files"]["external_imu/raw_imu_packets.bin"][
        "sha256"
    ] == hashlib.sha256(b"wire-packet").hexdigest()

    failed_session = tmp_path / "failed"
    failed_session.mkdir()
    failed = capture_quality_module.finalize_joint_outputs(
        session=failed_session,
        report={"result": "PASS"},
        raw_paths=(),
        camera_recorder_clean_shutdown=False,
        imu_recorder_clean_shutdown=False,
        recorder_write_error="TimeoutError: recorder shutdown timed out",
        camera_recorder_shutdown_error="OSError: recorder pause failed",
        stage_move_error=None,
        shutdown_errors=(),
    )

    assert failed["result"] == "FAIL"
    assert failed["reason"] == "output_finalization_failed"
    assert json.loads((failed_session / "acceptance.json").read_text())[
        "result"
    ] == "FAIL"


@pytest.mark.parametrize(
    "failing_action",
    ["camera_pause", "sensor_stop", "sensor_close", "imu_stop", "recorder_stop"],
)
def test_every_shutdown_fault_forces_failed_acceptance(tmp_path, failing_action):
    def action(name):
        def run():
            if name == failing_action:
                raise OSError(f"{name} failed")

        return run

    class Recorder:
        first_write_error = None

        def stop(self):
            action("recorder_stop")()
            return True

    shutdown = capture_quality_module.shutdown_capture_outputs(
        pause_recorder=action("camera_pause"),
        stop_sensor=action("sensor_stop"),
        close_sensor=action("sensor_close"),
        stop_imu=action("imu_stop"),
        imu_recorder=Recorder(),
    )
    report = capture_quality_module.finalize_joint_outputs(
        session=tmp_path,
        report={"result": "PASS"},
        raw_paths=(),
        camera_recorder_clean_shutdown=shutdown["camera_clean"],
        imu_recorder_clean_shutdown=shutdown["imu_clean"],
        recorder_write_error=shutdown["recorder_write_error"],
        camera_recorder_shutdown_error=shutdown["camera_error"],
        stage_move_error=None,
        shutdown_errors=shutdown["errors"],
    )

    assert report["result"] == "FAIL"
    assert report["reason"] == "output_finalization_failed"
    assert any(failing_action in error for error in report["finalization_errors"])


def test_live_writer_timeout_publishes_no_unsealed_raw_hashes(tmp_path):
    class BlockingRawFile:
        def __init__(self, real_file):
            self.real_file = real_file
            self.write_entered = threading.Event()
            self.release_write = threading.Event()

        def write(self, data):
            self.write_entered.set()
            self.release_write.wait(timeout=2.0)
            return self.real_file.write(data)

        def flush(self):
            return self.real_file.flush()

        def close(self):
            return self.real_file.close()

    class ShortTimeoutRecorder(capture_quality_module.UnitRecorder):
        def stop(self):
            return super().stop(timeout_s=0.02)

    recorder = ShortTimeoutRecorder("external_imu", tmp_path, max_queue=8)
    recorder.start()
    blocked_file = BlockingRawFile(recorder._raw_imu_fp)
    recorder._raw_imu_fp = blocked_file
    recorder.put_raw_imu_packet(b"still-writing")
    assert blocked_file.write_entered.wait(timeout=1.0)

    try:
        shutdown = capture_quality_module.shutdown_capture_outputs(
            pause_recorder=None,
            stop_sensor=None,
            close_sensor=None,
            stop_imu=None,
            imu_recorder=recorder,
        )
        report = capture_quality_module.finalize_joint_outputs(
            session=tmp_path,
            report={"result": "PASS"},
            raw_paths=(tmp_path / "external_imu" / "raw_imu_packets.bin",),
            camera_recorder_clean_shutdown=shutdown["camera_clean"],
            imu_recorder_clean_shutdown=shutdown["imu_clean"],
            recorder_write_error=shutdown["recorder_write_error"],
            camera_recorder_shutdown_error=shutdown["camera_error"],
            stage_move_error=None,
            shutdown_errors=shutdown["errors"],
        )

        assert recorder._thread.is_alive()
        assert report["result"] == "FAIL"
        assert report["raw_files"] == {}
    finally:
        blocked_file.release_write.set()
        capture_quality_module.UnitRecorder.stop(recorder, timeout_s=1.0)


def test_joint_imu_acceptance_requires_clean_persisted_formal_lower_bound():
    assert persisted_imu_covers_formal(
        clean_shutdown=True, persisted_samples=500, formal_samples=400
    )
    assert not persisted_imu_covers_formal(
        clean_shutdown=False, persisted_samples=500, formal_samples=400
    )
    assert not persisted_imu_covers_formal(
        clean_shutdown=True, persisted_samples=399, formal_samples=400
    )



def test_stop_record_finishes_joint_formal_window():
    assert capture_quality_module.formal_window_complete(
        now_ns=100,
        start_ns=0,
        duration_s=None,
        stop_record={"at_ns": 100, "reason": "operator_interrupt"},
    )


def test_indefinite_window_continues_without_stop():
    assert not capture_quality_module.formal_window_complete(
        now_ns=10_000_000_000,
        start_ns=0,
        duration_s=None,
        stop_record=None,
    )


def test_positive_duration_window_completes_at_deadline():
    assert not capture_quality_module.formal_window_complete(
        now_ns=99,
        start_ns=0,
        duration_s=0.0000001,
        stop_record=None,
    )
    assert capture_quality_module.formal_window_complete(
        now_ns=100,
        start_ns=0,
        duration_s=0.0000001,
        stop_record=None,
    )


@pytest.mark.parametrize("duration", [float("nan"), float("inf"), float("-inf"), 0, -1])
def test_formal_window_rejects_invalid_explicit_duration(duration):
    with pytest.raises(ValueError, match="finite and positive"):
        capture_quality_module.formal_window_complete(
            now_ns=100,
            start_ns=0,
            duration_s=duration,
            stop_record=None,
        )
