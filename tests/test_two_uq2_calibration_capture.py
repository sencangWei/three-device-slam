import os
from pathlib import Path
import struct
import threading
from types import SimpleNamespace

import pytest

from three_device_slam.devices.two_uq2.calibration_capture import (
    CalibrationRecorder,
    IsolatedImuReader,
    LatestFrameSlot,
    StereoAprilGridPreview,
    _configure_preview_worker_environment,
    _capture_until_deadline,
    apply_stage_acceptance,
    aprilgrid_geometry_is_consistent,
    build_capture_report,
    build_preview_report,
    capture,
    capture_was_aborted,
    expected_aprilgrid_detections,
    has_complete_jpeg_boundaries,
    period_multiple_jitter_ns,
    _drain_imu_messages,
    resolve_output_directory,
    split_side_by_side,
    format_capture_progress,
)
from three_device_slam.devices.two_uq2.capture import TwoUQ2Packet


def test_capture_report_accepts_native_60_hz_video_and_200_hz_imu():
    report = build_capture_report(
        duration_s=10.0,
        video_frames=600,
        imu_packets=2000,
        imu_samples=2000,
        video_timestamp_regressions=0,
        imu_timestamp_regressions=0,
        bad_jpegs=0,
        xu_failures=0,
    )

    assert report["status"] == "PASS"
    assert report["rates"] == {
        "video_hz": 60.0,
        "imu_packet_hz": 200.0,
        "imu_sample_hz": 200.0,
    }


def test_capture_report_rejects_synthetic_second_imu_sample():
    report = build_capture_report(
        duration_s=10.0,
        video_frames=600,
        imu_packets=1000,
        imu_samples=2000,
        video_timestamp_regressions=0,
        imu_timestamp_regressions=0,
        bad_jpegs=0,
        xu_failures=0,
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["imu_packet_hz"]["status"] == "FAIL"
    assert report["checks"]["one_main_sample_per_packet"]["status"] == "FAIL"


def test_capture_report_rejects_operator_abort():
    report = build_capture_report(
        duration_s=10.0,
        video_frames=600,
        imu_packets=2000,
        imu_samples=2000,
        video_timestamp_regressions=0,
        imu_timestamp_regressions=0,
        bad_jpegs=0,
        xu_failures=0,
        operator_aborted=True,
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["operator_aborted"]["status"] == "FAIL"


def test_capture_report_rejects_more_than_one_percent_estimated_drops():
    report = build_capture_report(
        duration_s=10.0,
        video_frames=594,
        imu_packets=2000,
        imu_samples=1980,
        video_timestamp_regressions=0,
        imu_timestamp_regressions=0,
        bad_jpegs=0,
        xu_failures=0,
        video_estimated_drops=7,
        imu_estimated_drops=21,
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["video_drop_ratio"]["status"] == "FAIL"
    assert report["checks"]["imu_drop_ratio"]["status"] == "FAIL"


def test_stereo_stage_passes_with_good_video_and_grid_despite_strict_imu_failure():
    report = build_capture_report(
        duration_s=10.0,
        video_frames=600,
        imu_packets=1750,
        imu_samples=1750,
        video_timestamp_regressions=0,
        imu_timestamp_regressions=0,
        bad_jpegs=0,
        xu_failures=0,
        imu_estimated_drops=300,
        imu_max_interval_ns=25_000_000,
    )
    preview_report = build_preview_report(
        {
            "processed_frames": 20,
            "left_frames_with_four_tags": 20,
            "right_frames_with_four_tags": 20,
            "both_frames_with_four_tags": 20,
            "left_tag_total": 200,
            "right_tag_total": 200,
            "processing_errors": 0,
        }
    )

    apply_stage_acceptance(report, stage="stereo", preview_report=preview_report)

    assert report["strict_transport_status"] == "FAIL"
    assert report["stage_acceptance"]["status"] == "PASS"
    assert report["status"] == "PASS"


def test_provisional_cam_imu_stage_accepts_bounded_loss_without_hiding_strict_fail():
    report = build_capture_report(
        duration_s=10.0,
        video_frames=600,
        imu_packets=1750,
        imu_samples=1750,
        video_timestamp_regressions=0,
        imu_timestamp_regressions=0,
        bad_jpegs=0,
        xu_failures=0,
        imu_estimated_drops=300,
        imu_max_interval_ns=25_000_000,
    )

    apply_stage_acceptance(
        report,
        stage="cam-imu-provisional",
        preview_report=None,
    )

    assert report["strict_transport_status"] == "FAIL"
    assert report["stage_acceptance"]["checks"]["imu_drop_ratio"]["status"] == "PASS"
    assert report["stage_acceptance"]["checks"]["imu_max_interval_ms"]["status"] == "PASS"
    assert report["stage_acceptance"]["status"] == "PROVISIONAL_PASS"
    assert report["status"] == "PROVISIONAL_PASS"


@pytest.mark.parametrize(
    ("imu_estimated_drops", "imu_max_interval_ns", "failed_check"),
    [
        (400, 25_000_000, "imu_drop_ratio"),
        (300, 31_000_000, "imu_max_interval_ms"),
    ],
)
def test_provisional_cam_imu_stage_rejects_excessive_loss_or_gap(
    imu_estimated_drops,
    imu_max_interval_ns,
    failed_check,
):
    report = build_capture_report(
        duration_s=10.0,
        video_frames=600,
        imu_packets=1750,
        imu_samples=1750,
        video_timestamp_regressions=0,
        imu_timestamp_regressions=0,
        bad_jpegs=0,
        xu_failures=0,
        imu_estimated_drops=imu_estimated_drops,
        imu_max_interval_ns=imu_max_interval_ns,
    )

    apply_stage_acceptance(
        report,
        stage="cam-imu-provisional",
        preview_report=None,
    )

    assert report["stage_acceptance"]["checks"][failed_check]["status"] == "FAIL"
    assert report["status"] == "FAIL"


def test_jitter_does_not_count_an_inferred_drop_twice():
    assert period_multiple_jitter_ns(14_953_257, 5_000_000) == 46_743


def test_preview_worker_limits_openblas_before_imports(monkeypatch):
    monkeypatch.setenv("OPENBLAS_NUM_THREADS", "24")

    _configure_preview_worker_environment()

    assert os.environ["OPENBLAS_NUM_THREADS"] == "1"


def test_split_side_by_side_preserves_left_then_right_order():
    import numpy as np

    image = np.arange(2 * 8, dtype=np.uint8).reshape(2, 8)

    left, right = split_side_by_side(image)

    assert left.tolist() == image[:, :4].tolist()
    assert right.tolist() == image[:, 4:].tolist()


def test_split_side_by_side_rejects_odd_width():
    import numpy as np

    with pytest.raises(ValueError, match="even"):
        split_side_by_side(np.zeros((2, 7), dtype=np.uint8))


def test_latest_preview_slot_replaces_stale_frame_without_blocking():
    slot = LatestFrameSlot()

    assert slot.put((1, b"first")) == 0
    assert slot.put((2, b"second")) == 1
    assert slot.get(timeout=0.01) == (2, b"second")


def test_preview_pump_only_polls_abort_state_in_capture_thread():
    preview = StereoAprilGridPreview()
    with preview._lock:
        preview._stats["operator_aborted"] = True

    assert preview.pump() is True


def test_preview_detects_unexpected_child_exit():
    class DeadProcess:
        exitcode = -11

        @staticmethod
        def is_alive():
            return False

    preview = StereoAprilGridPreview()
    preview._process = DeadProcess()
    preview._process_started = True

    with pytest.raises(RuntimeError, match="preview_processing_failed"):
        preview.raise_if_failed()


def test_preview_stop_does_not_join_process_that_never_started():
    class UnstartedProcess:
        @staticmethod
        def join(*_args, **_kwargs):
            raise AssertionError("join must not be called")

    preview = StereoAprilGridPreview()
    preview._process = UnstartedProcess()
    preview._process_started = False

    preview.stop()


def test_preview_report_rejects_abort_even_when_grid_metrics_pass():
    report = build_preview_report(
        {
            "processed_frames": 20,
            "left_frames_with_four_tags": 20,
            "right_frames_with_four_tags": 20,
            "both_frames_with_four_tags": 20,
            "left_tag_total": 200,
            "right_tag_total": 200,
            "processing_errors": 0,
            "operator_aborted": True,
        }
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["operator_aborted"]["status"] == "FAIL"


def test_late_preview_abort_is_merged_after_capture_loop_exits():
    assert capture_was_aborted(False, {"operator_aborted": True}) is True


def test_expected_aprilgrid_detections_filters_ids_duplicates_and_bad_corners():
    detections = [
        SimpleNamespace(tag_id=0, corners=[[0, 0], [1, 0], [1, 1], [0, 1]]),
        SimpleNamespace(tag_id=0, corners=[[0, 0], [1, 0], [1, 1], [0, 1]]),
        SimpleNamespace(tag_id=35, corners=[[0, 0], [1, 0], [1, 1], [0, 1]]),
        SimpleNamespace(tag_id=36, corners=[[0, 0], [1, 0], [1, 1], [0, 1]]),
        SimpleNamespace(tag_id=2, corners=[[0, 0], [1, 0], [1, 1]]),
    ]

    selected = expected_aprilgrid_detections(detections)

    assert [d.tag_id for d in selected] == [0, 35]


def test_aprilgrid_geometry_requires_one_consistent_grid_homography():
    import cv2
    import numpy as np

    object_to_image = np.array(
        [[90.0, 8.0, 40.0], [4.0, 85.0, 30.0], [0.01, 0.015, 1.0]],
        dtype=np.float64,
    )
    detections = []
    for tag_id in (0, 1, 6, 7, 14, 21):
        row, column = divmod(tag_id, 6)
        x0, y0 = column * 1.3, row * 1.3
        object_corners = np.asarray(
            [[x0, y0], [x0 + 1, y0], [x0 + 1, y0 + 1], [x0, y0 + 1]],
            dtype=np.float64,
        ).reshape(-1, 1, 2)
        image_corners = cv2.perspectiveTransform(
            object_corners, object_to_image
        ).reshape(4, 2)
        detections.append(SimpleNamespace(tag_id=tag_id, corners=image_corners))

    assert aprilgrid_geometry_is_consistent(detections) is True
    detections[-1].corners = detections[-1].corners + np.array([120.0, -80.0])
    assert aprilgrid_geometry_is_consistent(detections) is False


def test_preview_start_failure_does_not_create_output_directory(tmp_path, monkeypatch):
    class FakePipeline:
        def get_by_name(self, _name):
            return object()

    class FakeGst:
        @staticmethod
        def init(_args):
            return None

        @staticmethod
        def parse_launch(_description):
            return FakePipeline()

    def fail_preview(_self):
        raise RuntimeError("preview failed")

    output = tmp_path / "run_01"
    monkeypatch.setattr(
        "three_device_slam.devices.two_uq2.calibration_capture._load_gst",
        lambda: FakeGst(),
    )
    monkeypatch.setattr(StereoAprilGridPreview, "start", fail_preview)

    with pytest.raises(RuntimeError, match="preview failed"):
        capture(
            device="/dev/fake",
            xu_library="/fake.so",
            output=output,
            duration_s=1.0,
        )

    assert not output.exists()


def test_preview_report_requires_repeated_grid_visibility_in_both_halves():
    report = build_preview_report(
        {
            "processed_frames": 20,
            "left_frames_with_four_tags": 16,
            "right_frames_with_four_tags": 15,
            "both_frames_with_four_tags": 14,
            "left_tag_total": 160,
            "right_tag_total": 140,
            "processing_errors": 0,
        }
    )

    assert report["status"] == "PASS"
    assert report["checks"]["simultaneous_grid_detection_rate"]["measurement"] == 0.7


def test_preview_report_rejects_one_sided_grid_visibility():
    report = build_preview_report(
        {
            "processed_frames": 20,
            "left_frames_with_four_tags": 18,
            "right_frames_with_four_tags": 2,
            "both_frames_with_four_tags": 2,
            "left_tag_total": 180,
            "right_tag_total": 20,
            "processing_errors": 0,
        }
    )

    assert report["status"] == "FAIL"
    assert report["checks"]["right_grid_detection_rate"]["status"] == "FAIL"


@pytest.mark.parametrize(
    ("payload", "expected"),
    [
        (b"\xff\xd8payload\xff\xd9", True),
        (b"\xff\xd8truncated", False),
        (b"not-a-jpeg\xff\xd9", False),
        (b"", False),
    ],
)
def test_capture_hot_path_checks_jpeg_boundaries(payload, expected):
    assert has_complete_jpeg_boundaries(payload) is expected


def test_output_directory_must_be_new_direct_child_of_artifact_root(tmp_path):
    root = tmp_path / "artifacts" / "ego_calibration"
    root.mkdir(parents=True)

    output = resolve_output_directory(root / "run_01", root)

    assert output == root / "run_01"
    with pytest.raises(ValueError, match="direct child"):
        resolve_output_directory(root / "nested" / "run_01", root)
    existing = root / "existing"
    existing.mkdir()
    with pytest.raises(FileExistsError, match="already exists"):
        resolve_output_directory(existing, root)


@pytest.mark.parametrize("name", ["", ".", "..", "bad name", "bad/name"])
def test_output_directory_rejects_unsafe_session_name(tmp_path, name):
    root = tmp_path / "artifacts" / "ego_calibration"
    root.mkdir(parents=True)
    candidate = root / name if name else Path("")

    with pytest.raises(ValueError):
        resolve_output_directory(candidate, root)


def test_imu_producer_is_not_blocked_by_slow_storage():
    class SlowWriter:
        def __init__(self):
            self.entered = threading.Event()
            self.release = threading.Event()

        def append(self, _record, _payload):
            self.entered.set()
            assert self.release.wait(timeout=1.0)

    raw = (
        bytes((0, 0, 1))
        + struct.pack(">6h", 1, 2, 3, 4, 5, 6)
        + struct.pack(">6h", 7, 8, 9, 10, 11, 12)
    )
    writer = SlowWriter()
    recorder = CalibrationRecorder(writer, gst=None, pipeline=None)
    recorder.begin()
    producer = threading.Thread(
        target=recorder.record_packet,
        kwargs={
            "packet": TwoUQ2Packet.parse(raw),
            "read_started_ns": 10_000_000,
            "read_ended_ns": 12_000_000,
        },
    )
    try:
        producer.start()
        assert writer.entered.wait(timeout=1.0)
        producer.join(timeout=0.1)
        assert not producer.is_alive()
    finally:
        writer.release.set()
        producer.join(timeout=1.0)
        recorder.finish_writes()


def test_recorder_tracks_maximum_imu_acquisition_interval():
    class Writer:
        @staticmethod
        def append(_record, _payload):
            return None

    raw = (
        bytes((0, 0, 1))
        + struct.pack(">6h", 1, 2, 3, 4, 5, 6)
        + struct.pack(">6h", 7, 8, 9, 10, 11, 12)
    )
    packet = TwoUQ2Packet.parse(raw)
    recorder = CalibrationRecorder(Writer(), gst=None, pipeline=None)
    try:
        recorder.begin()
        recorder.record_packet(
            packet,
            read_started_ns=0,
            read_ended_ns=2_000_000,
        )
        recorder.record_packet(
            packet,
            read_started_ns=25_000_000,
            read_ended_ns=27_000_000,
        )
        recorder.end()

        assert recorder.stats()["imu_max_interval_ns"] == 25_000_000
    finally:
        recorder.finish_writes()


def test_isolated_imu_reader_uses_spawn_context():
    reader = IsolatedImuReader(
        device="/dev/video0",
        xu_library="/tmp/libtwo_uq2_xu.so",
    )

    assert reader.start_method == "spawn"


def test_capture_progress_reports_time_and_stream_counts():
    line = format_capture_progress(
        elapsed_s=15.2,
        duration_s=120.0,
        stats={"video_frames": 904, "imu_samples": 3011},
    )

    assert line == (
        "PROGRESS: elapsed=15.2s remaining=104.8s "
        "video_frames=904 imu_samples=3011"
    )


def test_capture_loop_converts_keyboard_interrupt_to_operator_abort():
    class Recorder:
        @staticmethod
        def raise_if_failed():
            return None

        @staticmethod
        def stats():
            return {"video_frames": 60, "imu_samples": 200}

    class ImuReader:
        @staticmethod
        def drain(_recorder):
            return 0

    messages = []

    def interrupted_sleep(_seconds):
        raise KeyboardInterrupt

    end_ns, operator_aborted = _capture_until_deadline(
        recorder=Recorder(),
        imu_reader=ImuReader(),
        preview_monitor=None,
        window_start_ns=1_000_000_000,
        deadline_ns=121_000_000_000,
        clock_ns=lambda: 2_000_000_000,
        sleep=interrupted_sleep,
        emit=messages.append,
    )

    assert end_ns == 2_000_000_000
    assert operator_aborted is True
    assert messages[-1] == "ABORTED: Ctrl-C received; finalizing data and report."


def test_parent_drains_one_main_sample_and_xu_error_from_imu_process():
    import queue

    class Recorder:
        def __init__(self):
            self.packets = []
            self.xu_failures = 0

        def record_packet(self, packet, *, read_started_ns, read_ended_ns):
            self.packets.append((packet, read_started_ns, read_ended_ns))

        def record_xu_failure(self):
            self.xu_failures += 1

    messages = queue.Queue()
    raw = (
        (42).to_bytes(3, "big")
        + struct.pack(">6h", 1, 2, 3, 4, 5, 6)
        + struct.pack(">6h", 7, 8, 9, 10, 11, 12)
    )
    messages.put(("sample", raw, 10_000_000, 14_000_000))
    messages.put(("xu_error",))
    recorder = Recorder()

    drained = _drain_imu_messages(messages, recorder)

    assert drained == 2
    assert recorder.xu_failures == 1
    assert recorder.packets == [(TwoUQ2Packet.parse(raw), 10_000_000, 14_000_000)]


def test_parent_drains_batched_imu_process_messages():
    import queue

    class Recorder:
        def __init__(self):
            self.packets = []
            self.xu_failures = 0

        def record_packet(self, packet, *, read_started_ns, read_ended_ns):
            self.packets.append((packet.sequence, read_started_ns, read_ended_ns))

        def record_xu_failure(self):
            self.xu_failures += 1

    first = (
        (1).to_bytes(3, "big")
        + struct.pack(">6h", 1, 2, 3, 4, 5, 6)
        + struct.pack(">6h", 7, 8, 9, 10, 11, 12)
    )
    second = bytes((0, 0, 2)) + first[3:]
    messages = queue.Queue()
    messages.put(
        (
            "batch",
            ((first, 10, 14), (second, 15, 19)),
            1,
        )
    )
    recorder = Recorder()

    drained = _drain_imu_messages(messages, recorder)

    assert drained == 3
    assert recorder.xu_failures == 1
    assert recorder.packets == [(1, 10, 14), (2, 15, 19)]
