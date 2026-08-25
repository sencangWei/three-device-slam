import json
import os
import struct
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from three_device_slam.core import AppendOnlySessionWriter, SensorRecord
from three_device_slam.core import session_writer as session_writer_module
from three_device_slam.devices.two_uq2 import worker as two_uq2_worker
from three_device_slam.devices.two_uq2.capture import (
    CaptureRuntimeError,
    DeviceUnavailableError,
    TwoUQ2Capture,
    TwoUQ2Packet,
    VendorXuLinkError,
    _CtypesXuBridge,
    _load_gst,
    build_pipeline_description,
)
from three_device_slam.devices.two_uq2.worker import (
    BarrierIoFailure,
    CaptureRuntimeFailure,
    EgoFrameRecorder,
    FORMAL_EDGE_LAG_NS,
    HardwareUnavailableFailure,
    MINIMUM_START_GUARD_NS,
    StaleScheduledStart,
    StorageIoFailure,
    VendorXuLinkFailure,
    _write_acceptance,
    acceptance_exit_code,
    build_acceptance,
    capture_start_failure,
    formal_deadline_ns,
    formal_window_complete,
    parse_args,
    run_formal_window,
    schedule_local_start,
    setup_storage,
    wait_for_scheduled_start,
)


def packet_bytes(sequence, *, accel=4096):
    main = struct.pack(">6h", accel, -4096, 2048, 164, -164, 82)
    backup = struct.pack(">6h", 0, 4096, -2048, 328, 0, -328)
    return sequence.to_bytes(3, "big") + main + backup


def minimal_jpeg(scan_data=b"\x11"):
    return (
        b"\xff\xd8"
        b"\xff\xe0\x00\x04AB"
        b"\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00"
        b"\xff\xda\x00\x08\x01\x01\x00\x00\x3f\x00"
        + scan_data.replace(b"\xff", b"\xff\x00")
        + b"\xff\xd9"
    )


def formal_evidence(
    *,
    first_ns=None,
    last_ns=None,
    total_frames=0,
    valid_frames=0,
    rate_hz=0.0,
    width=3840,
    height=1080,
    sequence_gaps=0,
    duplicate_sequences=0,
    sequence_regressions=0,
    valid_sequences=None,
):
    return {
        "observed_frames": total_frames,
        "total_frames": total_frames,
        "valid_frames": valid_frames,
        "first_acquisition_ns": first_ns,
        "last_acquisition_ns": last_ns,
        "first_arrival_ns": first_ns,
        "last_arrival_ns": last_ns,
        "rate_hz": rate_hz,
        "measured_width": width if total_frames else None,
        "measured_height": height if total_frames else None,
        "resolution_mismatches": int(total_frames > 0 and (width, height) != (3840, 1080)),
        "duplicate_sequences": duplicate_sequences,
        "sequence_gaps": sequence_gaps,
        "sequence_regressions": sequence_regressions,
        "valid_sequences": valid_frames if valid_sequences is None else valid_sequences,
        "xu_failures": 0,
        "bad_jpegs": 0,
        "timestamp_regressions": 0,
        "timestamp_unavailable": 0,
    }


def test_vendor_packet_is_big_endian_and_scaled():
    packet = TwoUQ2Packet.parse(packet_bytes(0x010203))

    assert packet.sequence == 0x010203
    assert packet.main_accel_g == pytest.approx((1.0, -1.0, 0.5))
    assert packet.main_gyro_dps == pytest.approx((10.0, -10.0, 5.0))
    assert packet.backup_accel_g == pytest.approx((0.0, 1.0, -0.5))
    assert packet.backup_gyro_dps == pytest.approx((20.0, 0.0, -20.0))
    assert packet.raw == packet_bytes(0x010203)


@pytest.mark.parametrize("size", [0, 26, 28])
def test_packet_length_is_exactly_27(size):
    with pytest.raises(ValueError, match="27"):
        TwoUQ2Packet.parse(bytes(size))


@pytest.mark.parametrize("raw", [None, "not-bytes", 27, object()])
def test_packet_rejects_non_byte_inputs(raw):
    with pytest.raises((TypeError, ValueError)):
        TwoUQ2Packet.parse(raw)


def test_pipeline_preserves_encoded_mjpeg_and_escapes_device():
    description = build_pipeline_description('/dev/video 0"test')

    assert 'device="/dev/video 0\\"test"' in description
    assert "image/jpeg,width=3840,height=1080,framerate=60/1" in description
    assert "appsink name=sink emit-signals=true sync=false max-buffers=8 drop=false" in description
    assert "decode" not in description.lower()


@pytest.mark.parametrize("device", ["", "\x00/dev/video0", "/dev/video0\nappsink"])
def test_pipeline_rejects_invalid_device_values(device):
    with pytest.raises(ValueError, match="device"):
        build_pipeline_description(device)


class FakeMapInfo:
    def __init__(self, data):
        self.data = data


class FakeBuffer:
    def __init__(self, data, pts):
        self._data = data
        self.pts = pts
        self.unmapped = False

    def map(self, _flags):
        return True, FakeMapInfo(self._data)

    def unmap(self, _info):
        self.unmapped = True


class FakeSample:
    def __init__(self, buffer, width, height):
        self._buffer = buffer
        self._caps = FakeCaps(width, height)

    def get_buffer(self):
        return self._buffer

    def get_caps(self):
        return self._caps


class FakeCapsStructure:
    def __init__(self, width, height):
        self.values = {"width": width, "height": height}

    def get_value(self, name):
        return self.values[name]


class FakeCaps:
    def __init__(self, width, height):
        self.structure = FakeCapsStructure(width, height)

    def get_structure(self, index):
        assert index == 0
        return self.structure


class FakeSink:
    def __init__(self):
        self.callback = None
        self.samples = []

    def connect(self, signal, callback):
        assert signal == "new-sample"
        self.callback = callback
        return 17

    def disconnect(self, handler_id):
        assert handler_id == 17
        self.callback = None

    def emit(self, signal):
        assert signal == "pull-sample"
        return self.samples.pop(0)

    def push(self, data, pts, *, width=3840, height=1080):
        buffer = FakeBuffer(data, pts)
        self.samples.append(FakeSample(buffer, width, height))
        result = self.callback(self)
        return result, buffer


class FakeMessage:
    def __init__(self, message_type):
        self.type = message_type


class FakeBus:
    def __init__(self):
        self.messages = []

    def timed_pop_filtered(self, timeout, message_types):
        assert timeout == 0
        assert message_types
        return self.messages.pop(0) if self.messages else None


class FakePipeline:
    def __init__(self, base_time=10_000):
        self.sink = FakeSink()
        self.base_time = base_time
        self.states = []
        self.state_result = "ok"
        self.bus = FakeBus()

    def get_by_name(self, name):
        return self.sink if name == "sink" else None

    def set_state(self, state):
        self.states.append(state)
        return self.state_result

    def get_base_time(self):
        return self.base_time

    def get_bus(self):
        return self.bus


class FakeGst:
    CLOCK_TIME_NONE = (1 << 64) - 1

    class MapFlags:
        READ = "read"

    class FlowReturn:
        OK = "flow-ok"
        ERROR = "flow-error"

    class State:
        PLAYING = "playing"
        NULL = "null"

    class StateChangeReturn:
        FAILURE = "failure"
        ASYNC = "async"

    class MessageType:
        ERROR = 1
        EOS = 2

    def __init__(self):
        self.pipeline = FakePipeline()
        self.description = None
        self.initialized = False

    def init(self, _args):
        self.initialized = True

    def parse_launch(self, description):
        self.description = description
        return self.pipeline


class FakeXu:
    def __init__(self, packets):
        self.packets = list(packets)
        self.opened = []
        self.closed = []

    def open(self, device):
        self.opened.append(device)
        return 23

    def read_imu27(self, fd):
        assert fd == 23
        value = self.packets.pop(0)
        if isinstance(value, BaseException):
            raise value
        return value

    def close(self, fd):
        self.closed.append(fd)


def make_capture(packets, on_frame):
    gst = FakeGst()
    xu = FakeXu(packets)
    ticks = iter(range(20_000, 30_000))
    capture = TwoUQ2Capture(
        "/dev/video0",
        "/tmp/libtwo_uq2_xu.so",
        on_frame,
        _gst=gst,
        _xu=xu,
        _monotonic_ns=lambda: next(ticks),
    )
    return capture, gst, xu


def test_fake_buffer_and_xu_emit_one_encoded_frame():
    frames = []
    capture, gst, _xu = make_capture([packet_bytes(7)], frames.append)
    capture.start()

    jpeg = minimal_jpeg(b"encoded-mjpeg")
    result, buffer = gst.pipeline.sink.push(jpeg, 250)

    assert result == FakeGst.FlowReturn.OK
    assert buffer.unmapped
    assert len(frames) == 1
    frame = frames[0]
    assert frame.sequence == 7
    assert frame.acquisition_ns == 10_250
    assert frame.arrival_ns == 20_000
    assert frame.jpeg == jpeg
    assert frame.xu_raw == packet_bytes(7)
    assert frame.packet.sequence == 7
    assert frame.valid is True
    assert frame.error_reason == ""
    assert (frame.width, frame.height) == (3840, 1080)


def test_xu_failure_retains_raw_mjpeg_without_fabricated_xu_data():
    frames = []
    capture, gst, _xu = make_capture([OSError("vendor read failed")], frames.append)
    capture.start()

    jpeg = minimal_jpeg(b"still-record-this")
    gst.pipeline.sink.push(jpeg, 250)

    frame = frames[0]
    assert frame.jpeg == jpeg
    assert frame.sequence is None
    assert frame.xu_raw == b""
    assert frame.packet is None
    assert frame.valid is False
    assert frame.error_reason == "xu_read_failed"
    assert capture.stats()["xu_failures"] == 1


@pytest.mark.parametrize("missing", ["pts", "base_time"])
def test_missing_gstreamer_timestamp_retains_raw_and_does_not_poison_regression(
    missing,
):
    frames = []
    capture, gst, _xu = make_capture([packet_bytes(1), packet_bytes(2)], frames.append)
    capture.start()
    if missing == "base_time":
        gst.pipeline.base_time = FakeGst.CLOCK_TIME_NONE
        first_pts = 100
    else:
        first_pts = FakeGst.CLOCK_TIME_NONE

    raw_jpeg = minimal_jpeg(b"raw-timestamp-evidence")
    gst.pipeline.sink.push(raw_jpeg, first_pts)
    gst.pipeline.base_time = 10_000
    gst.pipeline.sink.push(minimal_jpeg(b"next-normal"), 200)

    missing_frame, normal_frame = frames
    assert missing_frame.jpeg == raw_jpeg
    assert missing_frame.acquisition_ns is None
    assert missing_frame.valid is False
    assert missing_frame.error_reason == "timestamp_unavailable"
    assert missing_frame.xu_raw == packet_bytes(1)
    assert normal_frame.acquisition_ns == 10_200
    assert capture.stats()["timestamp_unavailable"] == 1
    assert capture.stats()["timestamp_regressions"] == 0

    writer = MemoryWriter()
    recorder = EgoFrameRecorder(writer)
    recorder.set_start_ns(1)
    recorder.handle(missing_frame)
    video_record = writer.rows[0][0]
    assert video_record.warmup is True
    assert video_record.metadata["task_start_candidate"] is False
    assert video_record.clock_domain == "gstreamer_timestamp_unavailable"
    assert video_record.acquisition_ns == 0
    assert video_record.arrival_ns != video_record.acquisition_ns


def test_negotiated_resolution_is_measured_and_mismatch_invalidates_frame():
    frames = []
    capture, gst, _xu = make_capture([packet_bytes(1)], frames.append)
    capture.start()

    gst.pipeline.sink.push(minimal_jpeg(), 100, width=1920, height=1080)

    assert frames[0].valid is False
    assert frames[0].error_reason == "resolution_mismatch"
    assert (frames[0].width, frames[0].height) == (1920, 1080)
    assert capture.stats()["measured_width"] == 1920
    assert capture.stats()["measured_height"] == 1080
    assert capture.stats()["resolution_mismatches"] == 1
    report = build_acceptance(
        stats=capture.stats(),
        formal_stats=formal_evidence(
            first_ns=10_100,
            last_ns=10_100,
            total_frames=1,
            valid_frames=0,
            width=1920,
            height=1080,
        ),
        counts={"warmup_frames": 0, "formal_frames": 1},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
    )
    assert report["checks"]["resolution"]["status"] == "FAIL"
    assert report["checks"]["resolution"]["measurement"] == [1920, 1080]


def test_soi_arbitrary_interior_and_eoi_is_not_a_structural_jpeg():
    frames = []
    capture, gst, _xu = make_capture([packet_bytes(1)], frames.append)
    capture.start()

    gst.pipeline.sink.push(b"\xff\xd8arbitrary-interior\xff\xd9", 100)

    assert frames[0].valid is False
    assert frames[0].error_reason == "bad_jpeg"
    assert capture.stats()["bad_jpegs"] == 1


def test_stats_count_duplicate_gap_regression_timestamp_bad_jpeg_and_callbacks():
    def rejected(_frame):
        raise RuntimeError("consumer failed")

    capture, gst, _xu = make_capture(
        [packet_bytes(1), packet_bytes(1), packet_bytes(4), packet_bytes(2)],
        rejected,
    )
    capture.start()
    for jpeg, pts in [
        (minimal_jpeg(b"a"), 100),
        (minimal_jpeg(b"b"), 200),
        (b"not-a-jpeg", 150),
        (minimal_jpeg(b"d"), 300),
    ]:
        gst.pipeline.sink.push(jpeg, pts)

    stats = capture.stats()
    assert stats["frames"] == 4
    assert stats["duplicate_sequences"] == 1
    assert stats["sequence_gaps"] == 2
    assert stats["sequence_regressions"] == 1
    assert stats["timestamp_regressions"] == 1
    assert stats["bad_jpegs"] == 1
    assert stats["callback_exceptions"] == 4


def test_sequence_wraps_at_24_bits_without_gap_or_regression():
    capture, gst, _xu = make_capture(
        [packet_bytes(value) for value in (0xFFFFFE, 0xFFFFFF, 0, 1)],
        lambda _frame: None,
    )
    capture.start()
    for index in range(4):
        gst.pipeline.sink.push(minimal_jpeg(bytes([index + 1])), index * 100)

    stats = capture.stats()
    assert stats["sequence_regressions"] == 0
    assert stats["sequence_gaps"] == 0
    assert stats["duplicate_sequences"] == 0


def test_capture_lifecycle_opens_once_disconnects_and_closes_once():
    capture, gst, xu = make_capture([], lambda _frame: None)

    capture.start()
    with pytest.raises(RuntimeError, match="already started"):
        capture.start()
    capture.stop()
    capture.stop()

    assert gst.initialized
    assert gst.pipeline.states == [FakeGst.State.PLAYING, FakeGst.State.NULL]
    assert gst.pipeline.sink.callback is None
    assert xu.opened == ["/dev/video0"]
    assert xu.closed == [23]


def test_failed_pipeline_start_returns_to_null_and_closes_xu():
    capture, gst, xu = make_capture([], lambda _frame: None)
    gst.pipeline.state_result = FakeGst.StateChangeReturn.FAILURE

    with pytest.raises(DeviceUnavailableError, match="2uq2_hardware_unavailable"):
        capture.start()
    capture.stop()

    assert gst.pipeline.states == [FakeGst.State.PLAYING, FakeGst.State.NULL]
    assert gst.pipeline.sink.callback is None
    assert xu.closed == [23]


@pytest.mark.parametrize(
    ("message_type", "expected_reason"),
    [
        (FakeGst.MessageType.ERROR, "gstreamer_error"),
        (FakeGst.MessageType.EOS, "gstreamer_eos"),
    ],
)
def test_async_pipeline_terminal_message_is_stable_counted_once_and_cleaned_up(
    message_type, expected_reason
):
    capture, gst, xu = make_capture([], lambda _frame: None)
    gst.pipeline.state_result = FakeGst.StateChangeReturn.ASYNC
    capture.start()
    gst.pipeline.bus.messages.append(FakeMessage(message_type))

    assert capture.poll_terminal() == expected_reason
    assert capture.poll_terminal() == expected_reason

    assert capture.stats()["capture_errors"] == 1
    assert gst.pipeline.states == [FakeGst.State.PLAYING, FakeGst.State.NULL]
    assert xu.closed == [23]


class NeverStartsBarrier:
    def read_start_ns(self):
        return None


class TerminalCapture:
    def poll_terminal(self):
        return "gstreamer_error"


def test_terminal_capture_breaks_wait_for_barrier_and_formal_loops():
    with pytest.raises(CaptureRuntimeFailure, match="gstreamer_error"):
        wait_for_scheduled_start(
            NeverStartsBarrier(),
            TerminalCapture(),
            clock_ns=lambda: 100,
            sleep=lambda _seconds: None,
        )


class HealthyCapture:
    def poll_terminal(self):
        return None


class StorageFailedRecorder:
    def raise_if_storage_failed(self):
        raise StorageIoFailure()


class StopBarrier:
    def __init__(self, stop_record):
        self.stop_record = stop_record
        self.reads = 0

    def read_stop(self):
        self.reads += 1
        return self.stop_record


class ScheduledBarrier:
    def __init__(self, start_ns=None):
        self.start_ns = start_ns
        self.scheduled = []

    def read_start_ns(self):
        return self.start_ns

    def schedule_start(self, start_ns):
        self.start_ns = start_ns
        self.scheduled.append(start_ns)


def test_future_start_is_installed_before_boundary_frame_is_classified():
    from three_device_slam.devices.two_uq2.capture import TwoUQ2Frame

    start_ns = 1_000_000_000
    barrier = ScheduledBarrier(start_ns)
    writer = MemoryWriter()
    recorder = EgoFrameRecorder(writer)

    accepted = wait_for_scheduled_start(
        barrier,
        HealthyCapture(),
        recorder=recorder,
        clock_ns=lambda: start_ns - MINIMUM_START_GUARD_NS,
        sleep=lambda _seconds: None,
    )
    recorder.handle(
        TwoUQ2Frame(
            None,
            start_ns,
            start_ns + 1,
            b"invalid-raw-evidence",
            b"",
            None,
            False,
            "xu_read_failed",
        )
    )

    assert accepted == start_ns
    assert writer.rows[0][0].warmup is False


def test_stale_external_start_is_blocked_and_cannot_shorten_formal_duration():
    recorder = EgoFrameRecorder(MemoryWriter())
    now_ns = 2_000_000_000

    with pytest.raises(StaleScheduledStart, match="stale_scheduled_start"):
        wait_for_scheduled_start(
            ScheduledBarrier(now_ns + MINIMUM_START_GUARD_NS - 1),
            HealthyCapture(),
            recorder=recorder,
            clock_ns=lambda: now_ns,
            sleep=lambda _seconds: None,
        )

    assert formal_deadline_ns(now_ns + MINIMUM_START_GUARD_NS, 2.5) == (
        now_ns + MINIMUM_START_GUARD_NS + 2_500_000_000
    )


def test_formal_window_completes_at_duration():
    assert formal_window_complete(
        now_ns=3_500_000_000,
        start_ns=1_000_000_000,
        duration_s=2.5,
        stop_record=None,
    )


def test_formal_window_completes_on_stop_record():
    assert formal_window_complete(
        now_ns=1_000_000_000,
        start_ns=1_000_000_000,
        duration_s=None,
        stop_record={"at_ns": 1_000_000_000, "reason": "operator_interrupt"},
    )


def test_indefinite_formal_window_without_stop_does_not_complete():
    assert not formal_window_complete(
        now_ns=10_000_000_000,
        start_ns=1_000_000_000,
        duration_s=None,
        stop_record=None,
    )


def test_formal_window_returns_normally_when_barrier_stop_exists():
    barrier = StopBarrier({"at_ns": 1_000_000_000, "reason": "operator_interrupt"})

    end_ns = run_formal_window(
        HealthyCapture(),
        barrier=barrier,
        start_ns=1_000_000_000,
        duration_s=None,
        clock_ns=lambda: 1_000_000_001,
        sleep=lambda _seconds: None,
    )

    assert end_ns == 1_000_000_001
    assert barrier.reads == 1


def test_formal_window_returns_normally_at_duration_after_reading_stop():
    barrier = StopBarrier(None)

    end_ns = run_formal_window(
        HealthyCapture(),
        barrier=barrier,
        start_ns=1_000_000_000,
        duration_s=2.5,
        clock_ns=lambda: 3_500_000_000,
        sleep=lambda _seconds: None,
    )

    assert end_ns == 3_500_000_000
    assert barrier.reads == 1


def test_barrier_stop_does_not_mask_capture_terminal_failure():
    barrier = StopBarrier(
        {"at_ns": 1_000_000_000, "reason": "operator_interrupt"}
    )
    with pytest.raises(CaptureRuntimeFailure, match="gstreamer_error"):
        run_formal_window(
            TerminalCapture(),
            barrier=barrier,
            start_ns=1_000_000_000,
            duration_s=None,
            clock_ns=lambda: 1_000_000_001,
            sleep=lambda _seconds: None,
        )
    assert barrier.reads == 0


def test_duration_completion_does_not_mask_capture_terminal_failure():
    barrier = StopBarrier(None)
    with pytest.raises(CaptureRuntimeFailure, match="gstreamer_error"):
        run_formal_window(
            TerminalCapture(),
            barrier=barrier,
            start_ns=1_000_000_000,
            duration_s=1.0,
            clock_ns=lambda: 2_000_000_000,
            sleep=lambda _seconds: None,
        )
    assert barrier.reads == 0


def test_barrier_stop_does_not_mask_storage_failure():
    barrier = StopBarrier(
        {"at_ns": 1_000_000_000, "reason": "operator_interrupt"}
    )
    with pytest.raises(StorageIoFailure, match="storage_io"):
        run_formal_window(
            HealthyCapture(),
            barrier=barrier,
            start_ns=1_000_000_000,
            duration_s=None,
            recorder=StorageFailedRecorder(),
            clock_ns=lambda: 1_000_000_001,
            sleep=lambda _seconds: None,
        )
    assert barrier.reads == 0


def test_duration_completion_does_not_mask_storage_failure():
    barrier = StopBarrier(None)
    with pytest.raises(StorageIoFailure, match="storage_io"):
        run_formal_window(
            HealthyCapture(),
            barrier=barrier,
            start_ns=1_000_000_000,
            duration_s=1.0,
            recorder=StorageFailedRecorder(),
            clock_ns=lambda: 2_000_000_000,
            sleep=lambda _seconds: None,
        )
    assert barrier.reads == 0


def test_storage_failure_precedes_capture_terminal_failure():
    barrier = StopBarrier(
        {"at_ns": 1_000_000_000, "reason": "operator_interrupt"}
    )
    with pytest.raises(StorageIoFailure, match="storage_io"):
        run_formal_window(
            TerminalCapture(),
            barrier=barrier,
            start_ns=1_000_000_000,
            duration_s=None,
            recorder=StorageFailedRecorder(),
            clock_ns=lambda: 1_000_000_001,
            sleep=lambda _seconds: None,
        )
    assert barrier.reads == 0


def install_run_fakes(
    tmp_path,
    monkeypatch,
    *,
    completion,
    duration_s,
    seal_failure=False,
    formal_failure=False,
):
    start_ns = 1_000_000_000
    end_ns = 2_000_000_000
    payload = b"encoded-2uq2-raw"
    events = []
    ego_directory = tmp_path / "session" / "ego"

    class EventWriter:
        def __init__(self):
            self.writer = AppendOnlySessionWriter(ego_directory)

        def append(self, record, raw):
            return self.writer.append(record, raw)

        def close(self):
            try:
                manifest = self.writer.close()
            except OSError:
                events.append("writer.seal_failure")
                raise
            events.append("writer.sealed")
            return manifest

    writer = EventWriter()

    class RunRecorder:
        def __init__(self, recorder_writer):
            assert recorder_writer is writer
            recorder_writer.append(
                SensorRecord(
                    "ego.video",
                    0,
                    start_ns,
                    start_ns,
                    "gstreamer_monotonic",
                    False,
                    True,
                    {"task_start_candidate": True},
                ),
                payload,
            )

        def handle(self, _frame):
            raise AssertionError("fake capture must not emit frames")

        def formal_stats(self):
            return formal_evidence(
                first_ns=start_ns,
                last_ns=end_ns,
                total_frames=60,
                valid_frames=60,
                rate_hz=60.0,
            )

        def counts(self):
            return {"warmup_frames": 0, "formal_frames": 60}

    class RunCapture:
        def start(self):
            events.append("capture.start")

        def stop(self):
            events.append("capture.stop")

        def stats(self):
            return {
                "frames": 60,
                "callback_exceptions": 0,
                "capture_errors": 0,
            }

    class RunBarrier:
        def latch_failure(self, _device_id, reason, _at_ns):
            events.append(f"barrier.failure:{reason}")

    capture = RunCapture()
    barrier = RunBarrier()
    original_write_acceptance = two_uq2_worker._write_acceptance
    original_replace = os.replace

    if seal_failure:
        def fail_manifest_replace(source, target):
            target_name = Path(target).name
            if target_name == "manifest.json":
                events.append("manifest.replace_failure")
                raise OSError("manifest replace failed")
            if target_name == "acceptance.json":
                events.append("acceptance.replace")
            return original_replace(source, target)

        monkeypatch.setattr(session_writer_module.os, "replace", fail_manifest_replace)

    def observe_acceptance(path, report):
        manifest_path = ego_directory / "manifest.json"
        assert (ego_directory / "ego.video.bin").read_bytes() == payload
        assert not path.exists()
        if seal_failure:
            assert not manifest_path.exists()
            assert events[-1] == "writer.seal_failure"
            assert report["status"] == "FAIL"
            assert report["reason"] == "storage_io"
        else:
            assert manifest_path.is_file()
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            assert events[-1] == "writer.sealed"
            assert report["hashes"]["streams"] == manifest["streams"]
        events.append("acceptance.publish")
        original_write_acceptance(path, report)

    monkeypatch.setattr(
        two_uq2_worker,
        "setup_storage",
        lambda _session: (ego_directory, writer),
    )
    monkeypatch.setattr(two_uq2_worker, "BarrierDirectory", lambda _path: barrier)
    monkeypatch.setattr(two_uq2_worker, "EgoFrameRecorder", RunRecorder)
    monkeypatch.setattr(
        two_uq2_worker,
        "TwoUQ2Capture",
        lambda _device, _library, _on_frame: capture,
    )
    monkeypatch.setattr(
        two_uq2_worker,
        "wait_for_scheduled_start",
        lambda *_args, **_kwargs: events.append("barrier.start") or start_ns,
    )
    def run_formal_window(*_args, **_kwargs):
        events.append(f"formal.{completion}")
        if formal_failure:
            raise CaptureRuntimeFailure("gstreamer_error")
        return end_ns

    monkeypatch.setattr(two_uq2_worker, "run_formal_window", run_formal_window)
    monkeypatch.setattr(two_uq2_worker, "_evidence", lambda _path: {"python": "3.test"})
    monkeypatch.setattr(two_uq2_worker, "_write_acceptance", observe_acceptance)

    args = SimpleNamespace(
        session=tmp_path / "session",
        barrier_dir=tmp_path / "session" / "barrier",
        device="/dev/video0",
        xu_library="/tmp/libtwo_uq2_xu.so",
        duration=duration_s,
    )
    return args, events, ego_directory


@pytest.mark.parametrize(
    ("completion", "duration_s"),
    [("barrier_stop", None), ("duration", 1.0)],
)
def test_run_graceful_completion_seals_raw_before_publishing_pass_acceptance(
    tmp_path,
    monkeypatch,
    completion,
    duration_s,
):
    args, events, ego_directory = install_run_fakes(
        tmp_path,
        monkeypatch,
        completion=completion,
        duration_s=duration_s,
    )

    assert two_uq2_worker.run(args) == 0

    assert events == [
        "capture.start",
        "barrier.start",
        f"formal.{completion}",
        "capture.stop",
        "writer.sealed",
        "acceptance.publish",
    ]
    report = json.loads(
        (ego_directory / "acceptance.json").read_text(encoding="utf-8")
    )
    assert report["status"] == "PASS"


def test_run_seal_failure_publishes_fail_not_false_pass(tmp_path, monkeypatch):
    args, events, ego_directory = install_run_fakes(
        tmp_path,
        monkeypatch,
        completion="duration",
        duration_s=1.0,
        seal_failure=True,
    )

    assert two_uq2_worker.run(args) == 2

    report = json.loads(
        (ego_directory / "acceptance.json").read_text(encoding="utf-8")
    )
    assert report["status"] == "FAIL"
    assert report["reason"] == "storage_io"
    assert not (ego_directory / "manifest.json").exists()
    assert (ego_directory / "ego.video.jsonl").is_file()
    assert not (ego_directory / ".writer.lock").exists()
    assert events[-4:] == [
        "writer.seal_failure",
        "acceptance.publish",
        "acceptance.replace",
        "barrier.failure:storage_io",
    ]


def test_run_seal_failure_precedes_existing_capture_failure(tmp_path, monkeypatch):
    args, events, ego_directory = install_run_fakes(
        tmp_path,
        monkeypatch,
        completion="capture_failure",
        duration_s=1.0,
        seal_failure=True,
        formal_failure=True,
    )

    assert two_uq2_worker.run(args) == 2

    assert "formal.capture_failure" in events
    assert "manifest.replace_failure" in events
    assert "acceptance.replace" in events
    assert not (ego_directory / "manifest.json").exists()
    report = json.loads(
        (ego_directory / "acceptance.json").read_text(encoding="utf-8")
    )
    assert report["status"] == "FAIL"
    assert report["reason"] == "storage_io"


def test_standalone_start_is_scheduled_in_future_for_raw_warmup():
    barrier = ScheduledBarrier()
    now_ns = 3_000_000_000

    start_ns = schedule_local_start(barrier, clock_ns=lambda: now_ns)

    assert start_ns - now_ns >= MINIMUM_START_GUARD_NS
    assert barrier.scheduled == [start_ns]

    with pytest.raises(CaptureRuntimeFailure, match="gstreamer_error"):
        run_formal_window(
            TerminalCapture(),
            barrier=StopBarrier(None),
            start_ns=0,
            duration_s=1.0,
            clock_ns=lambda: 100,
            sleep=lambda _seconds: None,
        )


class MemoryWriter:
    def __init__(self):
        self.rows = []

    def append(self, record, payload):
        self.rows.append((record, bytes(payload)))
        return f"{record.stream_id}:{len(self.rows)}"


def test_ego_recorder_keeps_warmup_and_marks_only_first_healthy_formal_candidate():
    from three_device_slam.devices.two_uq2.capture import TwoUQ2Frame

    writer = MemoryWriter()
    recorder = EgoFrameRecorder(writer)
    valid = TwoUQ2Packet.parse(packet_bytes(9))
    warmup = TwoUQ2Frame(9, 100, 110, b"\xff\xd8a\xff\xd9", valid.raw, valid, True, "")
    invalid = TwoUQ2Frame(None, 210, 220, b"\xff\xd8b\xff\xd9", b"", None, False, "xu_read_failed")
    formal = TwoUQ2Frame(9, 220, 230, b"\xff\xd8c\xff\xd9", valid.raw, valid, True, "")
    later = TwoUQ2Frame(9, 230, 240, b"\xff\xd8d\xff\xd9", valid.raw, valid, True, "")

    recorder.handle(warmup)
    recorder.set_start_ns(200)
    recorder.handle(invalid)
    recorder.handle(formal)
    recorder.handle(later)

    video_rows = [row for row in writer.rows if row[0].stream_id == "ego.video"]
    xu_rows = [row for row in writer.rows if row[0].stream_id == "ego.xu"]
    assert [row[0].warmup for row in video_rows] == [True, False, False, False]
    assert [row[0].valid for row in video_rows] == [True, False, True, True]
    assert [row[1] for row in xu_rows] == [valid.raw, b"", valid.raw, valid.raw]
    assert [row[0].metadata["task_start_candidate"] for row in video_rows] == [False, False, True, False]
    assert recorder.counts() == {"warmup_frames": 1, "formal_frames": 3}
    formal_stats = recorder.formal_stats()
    assert formal_stats["total_frames"] == 3
    assert formal_stats["valid_frames"] == 2
    assert formal_stats["first_acquisition_ns"] == 210
    assert formal_stats["last_acquisition_ns"] == 230
    assert formal_stats["xu_failures"] == 1
    assert formal_stats["duplicate_sequences"] == 1


def test_ego_joint_ready_requires_clean_recent_three_second_60hz_window():
    from three_device_slam.devices.two_uq2.capture import TwoUQ2Frame

    recorder = EgoFrameRecorder(MemoryWriter())
    start_ns = 1_000_000_000
    for index in range(181):
        at_ns = start_ns + index * 16_666_667
        packet = TwoUQ2Packet.parse(packet_bytes(index))
        recorder.handle(
            TwoUQ2Frame(
                index,
                at_ns,
                at_ns,
                b"jpeg",
                packet.raw,
                packet,
                True,
                "",
                3840,
                1080,
            )
        )

    assert recorder.joint_ready(start_ns + 3_000_000_000)

    bad_ns = start_ns + 3_016_666_667
    recorder.handle(
        TwoUQ2Frame(None, bad_ns, bad_ns, b"jpeg", b"", None, False, "xu_read_failed")
    )
    assert not recorder.joint_ready(bad_ns)


def test_worker_arguments_default_to_ego_and_reject_other_device_ids(tmp_path):
    args = parse_args(
        [
            "--device", "/dev/video0",
            "--xu-library", "/tmp/lib.so",
            "--session", str(tmp_path),
            "--duration", "1.5",
        ]
    )
    assert args.device_id == "ego"
    assert args.barrier_dir is None
    assert args.duration == 1.5

    with pytest.raises(SystemExit):
        parse_args(
            [
                "--device", "/dev/video0",
                "--xu-library", "/tmp/lib.so",
                "--session", str(tmp_path),
                "--duration", "1",
                "--device-id", "left",
            ]
        )


def test_worker_accepts_indefinite_duration(tmp_path):
    args = parse_args(
        [
            "--device", "/dev/video0",
            "--xu-library", "/tmp/lib.so",
            "--session", str(tmp_path),
        ]
    )

    assert args.duration is None


@pytest.mark.parametrize("duration", ["nan", "inf", "-inf", "0", "-1"])
def test_worker_rejects_non_finite_and_non_positive_duration(tmp_path, duration):
    with pytest.raises(SystemExit):
        parse_args(
            [
                "--device", "/dev/video0",
                "--xu-library", "/tmp/lib.so",
                "--session", str(tmp_path),
                "--duration", duration,
            ]
        )


def test_worker_script_help_runs_from_repository_root_without_gi():
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "three_device_slam.devices.two_uq2.worker",
            "--help",
        ],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--device-id" in result.stdout


def test_acceptance_has_measured_checks_hashes_versions_and_no_skipped_status():
    report = build_acceptance(
        stats={
            "frames": 601,
            "rate_hz": 60.0,
            "measured_width": 3840,
            "measured_height": 1080,
            "resolution_mismatches": 0,
            "timestamp_regressions": 0,
            "xu_failures": 0,
            "duplicate_sequences": 0,
            "sequence_gaps": 1,
            "valid_sequences": 2000,
            "bad_jpegs": 0,
            "callback_exceptions": 0,
            "capture_errors": 0,
        },
        counts={"warmup_frames": 10, "formal_frames": 2000},
        manifest={"streams": {"ego.video": {"payload_sha256": "abc"}}},
        evidence={"xu_library_sha256": "def", "python": "3.test", "gstreamer": "1.test"},
        requested_duration_s=10.0,
        measured_formal_duration_s=10.01,
        formal_start_ns=1_000_000_000,
        formal_deadline_ns=11_000_000_000,
        formal_stats=formal_evidence(
            first_ns=1_000_000_000,
            last_ns=11_000_000_000,
            total_frames=601,
            valid_frames=600,
            rate_hz=60.0,
            sequence_gaps=1,
            valid_sequences=2000,
        ),
    )

    assert report["status"] == "PASS"
    assert report["checks"]["rate_hz"] == {
        "status": "PASS",
        "threshold": ">= 59.0",
        "measurement": 60.0,
        "scope": "formal_window",
    }
    assert report["checks"]["gap_ratio"]["measurement"] == pytest.approx(0.0005)
    assert set(check["status"] for check in report["checks"].values()) <= {"PASS", "FAIL", "BLOCKED"}
    assert report["hashes"]["xu_library_sha256"] == "def"
    assert report["tool_versions"]["python"] == "3.test"
    assert report["counts"] == {"warmup_frames": 10, "formal_frames": 2000}
    assert report["formal_window"] == {
        "requested_duration_s": 10.0,
        "measured_duration_s": 10.01,
        "formal_frames": 2000,
        "start_ns": 1_000_000_000,
        "deadline_ns": 11_000_000_000,
        "edge_lag_threshold_ns": FORMAL_EDGE_LAG_NS,
    }
    assert report["checks"]["formal_duration_s"]["status"] == "PASS"


def test_warmup_rate_cannot_qualify_when_no_formal_frame_exists():
    report = build_acceptance(
        stats={"frames": 600, "rate_hz": 60.0, "measured_width": 3840, "measured_height": 1080},
        formal_stats=formal_evidence(),
        counts={"warmup_frames": 600, "formal_frames": 0},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        requested_duration_s=10.0,
        measured_formal_duration_s=10.0,
        formal_start_ns=1_000_000_000,
        formal_deadline_ns=11_000_000_000,
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["valid_formal_frames"]["status"] == "FAIL"
    assert report["checks"]["rate_hz"]["measurement"] is None
    assert report["capture_diagnostics"]["rate_hz"] == 60.0


def test_formal_60hz_burst_that_ends_early_fails_window_coverage():
    start_ns = 1_000_000_000
    deadline_ns = 11_000_000_000
    report = build_acceptance(
        stats={"frames": 61, "rate_hz": 60.0},
        formal_stats=formal_evidence(
            first_ns=start_ns,
            last_ns=start_ns + 1_000_000_000,
            total_frames=61,
            valid_frames=61,
            rate_hz=60.0,
        ),
        counts={"warmup_frames": 0, "formal_frames": 61},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        requested_duration_s=10.0,
        measured_formal_duration_s=10.0,
        formal_start_ns=start_ns,
        formal_deadline_ns=deadline_ns,
    )

    assert report["checks"]["rate_hz"]["status"] == "FAIL"
    assert report["checks"]["formal_end_coverage"]["status"] == "FAIL"
    assert report["status"] == "FAIL"


def test_late_60hz_burst_near_deadline_fails_full_window_rate_and_start_coverage():
    start_ns = 1_000_000_000
    deadline_ns = 11_000_000_000
    report = build_acceptance(
        stats={"frames": 61, "rate_hz": 60.0, "callback_exceptions": 0, "capture_errors": 0},
        formal_stats=formal_evidence(
            first_ns=start_ns + 9_000_000_000,
            last_ns=deadline_ns,
            total_frames=61,
            valid_frames=61,
            rate_hz=60.0,
        ),
        counts={"warmup_frames": 0, "formal_frames": 61},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        requested_duration_s=10.0,
        measured_formal_duration_s=10.0,
        formal_start_ns=start_ns,
        formal_deadline_ns=deadline_ns,
    )

    assert report["checks"]["rate_hz"]["measurement"] == pytest.approx(6.1)
    assert report["checks"]["rate_hz"]["status"] == "FAIL"
    assert report["checks"]["formal_start_coverage"]["status"] == "FAIL"
    assert report["checks"]["formal_end_coverage"]["status"] == "PASS"
    assert report["status"] == "FAIL"


def test_full_window_formal_stream_is_eligible_for_pass():
    start_ns = 1_000_000_000
    deadline_ns = 11_000_000_000
    report = build_acceptance(
        stats={"frames": 700, "rate_hz": 60.0, "callback_exceptions": 0, "capture_errors": 0},
        formal_stats=formal_evidence(
            first_ns=start_ns,
            last_ns=deadline_ns - FORMAL_EDGE_LAG_NS,
            total_frames=598,
            valid_frames=598,
            rate_hz=60.0,
        ),
        counts={"warmup_frames": 102, "formal_frames": 598},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        requested_duration_s=10.0,
        measured_formal_duration_s=10.0,
        formal_start_ns=start_ns,
        formal_deadline_ns=deadline_ns,
    )

    assert report["status"] == "PASS"
    assert report["checks"]["formal_end_coverage"] == {
        "status": "PASS",
        "threshold": f"last comparable frame within {FORMAL_EDGE_LAG_NS} ns of deadline",
        "measurement": deadline_ns - FORMAL_EDGE_LAG_NS,
        "scope": "formal_window",
    }


def test_callback_and_capture_exceptions_use_whole_run_diagnostics():
    start_ns = 1_000_000_000
    deadline_ns = 11_000_000_000
    report = build_acceptance(
        stats={
            "frames": 601,
            "rate_hz": 60.0,
            "callback_exceptions": 1,
            "capture_errors": 1,
        },
        formal_stats=formal_evidence(
            first_ns=start_ns,
            last_ns=deadline_ns,
            total_frames=601,
            valid_frames=601,
            rate_hz=60.0,
        ),
        counts={"warmup_frames": 0, "formal_frames": 601},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        requested_duration_s=10.0,
        measured_formal_duration_s=10.0,
        formal_start_ns=start_ns,
        formal_deadline_ns=deadline_ns,
    )

    assert report["checks"]["callback_exceptions"] == {
        "status": "FAIL",
        "threshold": "== 0",
        "measurement": 1,
        "scope": "whole_run",
    }
    assert report["checks"]["capture_errors"]["measurement"] == 1
    assert report["checks"]["capture_errors"]["scope"] == "whole_run"
    assert report["status"] == "FAIL"


def test_nonzero_runtime_diagnostic_is_observed_before_first_frame():
    report = build_acceptance(
        stats={"frames": 0, "callback_exceptions": 0, "capture_errors": 1},
        formal_stats=formal_evidence(),
        counts={"warmup_frames": 0, "formal_frames": 0},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        failure=CaptureRuntimeFailure("gstreamer_error"),
        requested_duration_s=10.0,
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "2uq2_capture_runtime"
    assert report["checks"]["capture_errors"] == {
        "status": "FAIL",
        "threshold": "== 0",
        "measurement": 1,
        "scope": "whole_run",
    }
    assert report["checks"]["callback_exceptions"]["measurement"] is None
    assert report["checks"]["callback_exceptions"]["status"] == "BLOCKED"


def test_acceptance_write_flushes_file_replaces_and_fsyncs_directory(tmp_path, monkeypatch):
    calls = []
    real_fsync = os.fsync

    def observed_fsync(descriptor):
        calls.append(descriptor)
        return real_fsync(descriptor)

    monkeypatch.setattr("three_device_slam.devices.two_uq2.worker.os.fsync", observed_fsync)
    path = tmp_path / "ego" / "acceptance.json"
    path.parent.mkdir()

    _write_acceptance(path, {"status": "BLOCKED", "reason": "test"})

    assert json.loads(path.read_text()) == {"reason": "test", "status": "BLOCKED"}
    assert len(calls) == 2
    assert not list(path.parent.glob("*.tmp"))


def test_acceptance_records_unavailable_hardware_as_blocked():
    report = build_acceptance(
        stats={},
        counts={"warmup_frames": 0, "formal_frames": 0},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        blocked_reason="2uq2_hardware_unavailable",
    )

    assert report["status"] == "BLOCKED"
    assert report["reason"] == "2uq2_hardware_unavailable"
    assert all(check["status"] == "BLOCKED" for check in report["checks"].values())
    assert "SKIPPED" not in json.dumps(report)


@pytest.mark.parametrize("failure", [HardwareUnavailableFailure(), VendorXuLinkFailure()])
def test_actual_zero_initialized_startup_blocker_has_only_null_measurements(failure):
    capture, _gst, _xu = make_capture([], lambda _frame: None)
    recorder = EgoFrameRecorder(MemoryWriter())

    report = build_acceptance(
        stats=capture.stats(),
        formal_stats=recorder.formal_stats(),
        counts=recorder.counts(),
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        failure=failure,
        requested_duration_s=10.0,
    )

    assert report["status"] == "BLOCKED"
    assert all(
        check["status"] == "BLOCKED" and check["measurement"] is None
        for check in report["checks"].values()
    )
    assert report["rates"]["video_hz"] is None
    assert all(value is None for value in report["sequence_evidence"].values())
    assert report["xu_failures"] is None
    assert report["bad_jpegs"] is None


def test_runtime_failure_after_formal_frames_retains_measured_diagnostics():
    report = build_acceptance(
        stats={"frames": 601, "rate_hz": 60.0},
        formal_stats=formal_evidence(
            first_ns=1_000_000_000,
            last_ns=11_000_000_000,
            total_frames=601,
            valid_frames=600,
            rate_hz=60.0,
        ),
        counts={"warmup_frames": 0, "formal_frames": 601},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        failure=CaptureRuntimeFailure("gstreamer_error"),
        requested_duration_s=10.0,
        measured_formal_duration_s=10.0,
        formal_start_ns=1_000_000_000,
        formal_deadline_ns=11_000_000_000,
    )

    assert report["status"] == "FAIL"
    assert report["reason"] == "2uq2_capture_runtime"
    assert report["checks"]["rate_hz"]["measurement"] == 60.0
    assert report["rates"]["video_hz"] == 60.0


def test_acceptance_exit_codes_distinguish_fail_from_blocked():
    assert acceptance_exit_code("PASS") == 0
    assert acceptance_exit_code("FAIL") == 2
    assert acceptance_exit_code("BLOCKED") == 3


@pytest.mark.parametrize(
    ("failure", "status", "reason", "exit_code"),
    [
        (VendorXuLinkFailure(), "BLOCKED", "vendor_xu_link", 3),
        (HardwareUnavailableFailure(), "BLOCKED", "2uq2_hardware_unavailable", 3),
        (CaptureRuntimeFailure("gstreamer_error"), "FAIL", "2uq2_capture_runtime", 2),
        (BarrierIoFailure(), "FAIL", "barrier_io", 2),
        (StorageIoFailure(), "FAIL", "storage_io", 2),
    ],
)
def test_typed_failures_produce_distinct_acceptance_status_reason_and_exit(
    failure, status, reason, exit_code
):
    report = build_acceptance(
        stats={},
        counts={"warmup_frames": 0, "formal_frames": 0},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
        failure=failure,
    )

    assert report["status"] == status
    assert report["reason"] == reason
    assert acceptance_exit_code(report["status"]) == exit_code


def test_capture_start_boundary_maps_link_and_device_errors_separately():
    assert isinstance(capture_start_failure(VendorXuLinkError()), VendorXuLinkFailure)
    assert isinstance(
        capture_start_failure(DeviceUnavailableError()), HardwareUnavailableFailure
    )
    assert isinstance(capture_start_failure(RuntimeError("generic")), CaptureRuntimeFailure)
    assert isinstance(capture_start_failure(OSError("generic")), CaptureRuntimeFailure)


def test_missing_pygobject_is_typed_as_capture_runtime(monkeypatch):
    monkeypatch.setitem(sys.modules, "gi", None)

    with pytest.raises(CaptureRuntimeError, match="gstreamer_unavailable"):
        _load_gst()


def test_vendor_library_missing_required_symbol_is_link_blocker(monkeypatch):
    class LibraryWithoutSymbols:
        pass

    monkeypatch.setattr(
        "three_device_slam.devices.two_uq2.capture.ctypes.CDLL",
        lambda _path: LibraryWithoutSymbols(),
    )

    with pytest.raises(VendorXuLinkError, match="vendor_xu_link"):
        _CtypesXuBridge("/tmp/fake.so").open("/dev/video0")


def test_storage_setup_failure_is_typed_without_path_details(tmp_path):
    def broken_writer(_root):
        raise OSError("secret customer path")

    with pytest.raises(StorageIoFailure) as caught:
        setup_storage(tmp_path, writer_factory=broken_writer)

    assert str(caught.value) == "storage_io"


class BrokenBarrier:
    def read_start_ns(self):
        raise OSError("private path must not escape")


def test_barrier_io_is_typed_at_wait_boundary():
    with pytest.raises(BarrierIoFailure, match="barrier_io"):
        wait_for_scheduled_start(
            BrokenBarrier(),
            HealthyCapture(),
            clock_ns=lambda: 100,
            sleep=lambda _seconds: None,
        )


class BrokenWriter:
    def append(self, _record, _payload):
        raise OSError("private storage detail")


def test_writer_io_is_latched_as_stable_storage_failure():
    from three_device_slam.devices.two_uq2.capture import TwoUQ2Frame

    recorder = EgoFrameRecorder(BrokenWriter())
    frame = TwoUQ2Frame(
        None,
        None,
        100,
        b"raw",
        b"",
        None,
        False,
        "timestamp_unavailable",
    )

    with pytest.raises(OSError):
        recorder.handle(frame)
    with pytest.raises(StorageIoFailure, match="storage_io"):
        recorder.raise_if_storage_failed()


def test_non_unique_xu_sequence_is_a_frame_relation_blocker():
    report = build_acceptance(
        stats={
            "rate_hz": 60.0,
            "measured_width": 3840,
            "measured_height": 1080,
            "resolution_mismatches": 0,
            "timestamp_regressions": 0,
            "xu_failures": 0,
            "duplicate_sequences": 1,
            "sequence_regressions": 0,
            "sequence_gaps": 0,
            "valid_sequences": 600,
            "bad_jpegs": 0,
        },
        formal_stats=formal_evidence(
            first_ns=1_000_000_000,
            last_ns=2_000_000_000,
            total_frames=600,
            valid_frames=600,
            rate_hz=60.0,
            duplicate_sequences=1,
            valid_sequences=600,
        ),
        counts={"warmup_frames": 0, "formal_frames": 600},
        manifest={"streams": {}},
        evidence={"python": "3.test"},
    )

    assert report["status"] == "BLOCKED"
    assert report["reason"] == "2uq2_xu_frame_relation"
    assert report["checks"]["duplicate_sequences"]["measurement"] == 1
