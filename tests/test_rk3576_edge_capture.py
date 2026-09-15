from __future__ import annotations

import hashlib
import io
import json
import os
import signal
import shutil
import struct
import subprocess
import threading
from pathlib import Path

import pytest

from three_device_slam.devices.d405_umi.imu.stream_protocol import crc16_ccitt_false
from three_device_slam.edge_rk3576.capture import (
    CaptureError,
    FPS,
    HEIGHT,
    IR_FRAME_BYTES,
    RGB_FPS,
    SCHEMA,
    WIDTH,
    _FrameSpooler,
    capture_session,
    parse_gstreamer_rgb_line,
    parse_v4l2_frame_line,
)
from three_device_slam.edge_rk3576.validate import (
    RSUSB_SCHEMA,
    RSUSB_SPLIT_H265_SCHEMA,
    RSUSB_SPLIT_ZSTD_SCHEMA,
    RSUSB_ZSTD_SCHEMA,
    SessionValidationError,
    validate_rk3576_session,
)
from three_device_slam.edge_rk3576.preview import _PreviewState, extract_complete_jpegs
from three_device_slam.edge_rk3576.rsusb_capture import (
    IR_H265_BITRATE_PER_EYE,
    IR_H265_GOP_FRAMES,
    _AsyncFrameWriter,
    _H265FrameWriter,
    _cancel_started_capture_children,
    _RgbEncoder,
    _ZstdFrameWriter,
    _ir_h265_gstreamer_command,
    _publish_session,
    _rgb_gstreamer_command,
    capture_rsusb_session,
    interleave_stereo_y8,
    main as rsusb_main,
)


def _packet(sequence: int) -> bytes:
    imu = bytearray(37)
    imu[:3] = b"\xeb\x90\x22"
    struct.pack_into("<7f", imu, 4, 1, 2, 3, 4, 5, 6, 25)
    struct.pack_into("<I", imu, 32, sequence)
    imu[36] = sum(imu[:36]) & 0xFF
    packet = bytearray(63)
    packet[:4] = b"\xa5\x5a\x01\x3f"
    struct.pack_into("<H", packet, 4, 3)
    struct.pack_into("<IIII", packet, 6, sequence, sequence * 2500, 20, sequence)
    struct.pack_into("<H", packet, 22, sequence & 0xFFFF)
    packet[24:61] = imu
    struct.pack_into("<H", packet, 61, crc16_ccitt_false(packet[:61]))
    return bytes(packet)


def _write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def _refresh_claim(manifest: dict, path: Path) -> None:
    manifest["files"][path.name] = {
        "size": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def _sealed_session(root: Path, *, ir_encoding: str = "y8i_raw") -> Path:
    root.mkdir()
    split_eye = ir_encoding in {"y8_split_zstd", "y8_split_h265"}
    config = {
        "schema": SCHEMA,
        "session_id": root.name,
        "device": {"d405_serial": "test"},
        "profile": {
            "infrared": {
                "encoding": ir_encoding,
                "layout": (
                    "separate_left_right_files"
                    if split_eye
                    else "byte_interleaved_left_right"
                ),
                "width": WIDTH,
                "height": HEIGHT,
                "fps": 30,
                "frame_bytes": IR_FRAME_BYTES,
                "frame_bytes_per_eye": WIDTH * HEIGHT,
                "files": (
                    {
                        "left": (
                            "infrared-left-y8.raw.zst"
                            if ir_encoding == "y8_split_zstd"
                            else "infrared-left-y8.h265"
                        ),
                        "right": (
                            "infrared-right-y8.raw.zst"
                            if ir_encoding == "y8_split_zstd"
                            else "infrared-right-y8.h265"
                        ),
                    }
                    if split_eye
                    else {
                        "stereo": (
                            "infrared-y8i.raw.zst"
                            if ir_encoding == "y8i_zstd"
                            else "infrared-y8i.raw"
                        )
                    }
                ),
                "semantics": (
                    "lossy_stereo_slam_candidate"
                    if ir_encoding == "y8_split_h265"
                    else "lossless_stereo_sensor_sample"
                ),
                "compression": (
                    {
                        "codec": "zstd",
                        "level": 1,
                        "threads_per_stream": 1 if ir_encoding == "y8_split_zstd" else 2,
                        "content": (
                            "concatenated_complete_y8_frames_per_eye"
                            if ir_encoding == "y8_split_zstd"
                            else "concatenated_complete_y8i_frames"
                        ),
                    }
                    if ir_encoding in {"y8_split_zstd", "y8i_zstd"}
                    else (
                        {
                            "codec": "h265",
                            "container": "annex_b",
                            "encoder": "rockchip_mpp",
                            "rate_control": "cbr",
                            "target_bitrate_per_eye": 3_456_000,
                            "minimum_bitrate_per_eye": 3_240_000,
                            "maximum_bitrate_per_eye": 3_672_000,
                            "gop_frames": 30,
                            "pixel_format": "yuv420p_nv12",
                            "source_encoding": "y8",
                            "content": "one_access_unit_per_complete_y8_frame_per_eye",
                        }
                        if ir_encoding == "y8_split_h265"
                        else None
                    )
                ),
            },
            "rgb": {
                "encoding": "h265_annex_b",
                "width": WIDTH,
                "height": HEIGHT,
                "fps": RGB_FPS,
            },
            "stm32": {"encoding": "stm32_combined_v1_raw", "packet_bytes": 63},
        },
        "requested": {"duration_s": 2 / FPS, "frames": 2},
    }
    _write_json(root / "session_config.json", config)
    ir_raw = bytes(IR_FRAME_BYTES * 2)
    if ir_encoding == "y8_split_zstd":
        for side in ("left", "right"):
            with (root / f"infrared-{side}-y8.raw.zst").open("xb") as compressed:
                subprocess.run(
                    ["zstd", "--quiet", "-1", "--stdout"],
                    input=bytes(WIDTH * HEIGHT * 2),
                    stdout=compressed,
                    check=True,
                )
    elif ir_encoding == "y8_split_h265":
        for side in ("left", "right"):
            (root / f"infrared-{side}-y8.h265").write_bytes(
                b"\x00\x00\x00\x01\x40fixture"
            )
            (root / f"ir-{side}-gstreamer.log").write_text(
                "ok\n", encoding="utf-8"
            )
    elif ir_encoding == "y8i_zstd":
        with (root / "infrared-y8i.raw.zst").open("xb") as compressed:
            subprocess.run(
                [
                    "zstd",
                    "--quiet",
                    "-1",
                    "--stdout",
                ],
                input=ir_raw,
                stdout=compressed,
                check=True,
            )
    else:
        (root / "infrared-y8i.raw").write_bytes(ir_raw)
    with (root / "ir_frames.jsonl").open("w", encoding="utf-8") as stream:
        for i in range(2):
            row = {
                "record_index": i,
                "sequence": 10 + i,
                "bytes_used": IR_FRAME_BYTES,
                "source_monotonic_ns": 1_000_000_000 + i * 33_333_333,
            }
            if split_eye:
                row.update(
                    {
                        "left_sequence": 10 + i,
                        "right_sequence": 10 + i,
                        "left_source_timestamp_ms": 1000 + i * 33.333333,
                        "right_source_timestamp_ms": 1000 + i * 33.333333,
                    }
                )
            stream.write(json.dumps(row) + "\n")
    (root / "rgb.h265").write_bytes(b"\x00\x00\x00\x01\x40test")
    with (root / "rgb_frames.jsonl").open("w", encoding="utf-8") as stream:
        for i in range(2):
            stream.write(json.dumps({
                "record_index": i,
                "bytes_used": WIDTH * HEIGHT * 2,
                "pipeline_pts_ns": i * 33_333_333,
            }) + "\n")
    packets = [_packet(40), _packet(41)]
    (root / "stm32.bin").write_bytes(b"".join(packets))
    with (root / "stm32_packets.jsonl").open("w", encoding="utf-8") as stream:
        for i, packet in enumerate(packets):
            stream.write(json.dumps({
                "record_index": i,
                "offset": i * len(packet),
                "size": len(packet),
                "sequence": 40 + i,
                "flags": 3,
            }) + "\n")
    (root / "ir-v4l2.log").write_text("ok\n", encoding="utf-8")
    (root / "rgb-gstreamer.log").write_text("ok\n", encoding="utf-8")
    (root / "kernel-usb.log").write_text("", encoding="utf-8")
    files = {}
    for path in root.iterdir():
        if path.name == "manifest.json":
            continue
        files[path.name] = {
            "size": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    ir_metrics = {"frames": 2}
    metrics = {
        "ir": ir_metrics,
        "rgb": {"input_frames": 2, "pts_regressions": 0},
        "stm32": {
            "packets": 2,
            "observed_rate_hz": 400.0,
            "crc_errors": 0,
            "discarded_bytes": 0,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
            "invalid_imu_flags": 0,
            "invalid_encoder_flags": 0,
        },
    }
    if split_eye:
        eye_bytes = WIDTH * HEIGHT * 2
        suffix = "raw.zst" if ir_encoding == "y8_split_zstd" else "h265"
        left_compressed = (root / f"infrared-left-y8.{suffix}").stat().st_size
        right_compressed = (root / f"infrared-right-y8.{suffix}").stat().st_size
        stream_integrity = {
            "payload_size_errors": 0,
            "repeated_sequences": 0,
            "sequence_gaps": 0,
            "sequence_regressions": 0,
            "timestamp_regressions": 0,
            "timestamp_span_s": 1 / FPS,
            "observed_rate_hz": float(FPS),
            "max_arrival_interval_ms": 1000 / FPS,
        }
        metrics["rsusb_streams"] = {
            name: {"received": 2, **stream_integrity}
            for name in ("color", "infrared_left", "infrared_right")
        }
        streams = {}
        for side, compressed_bytes in (
            ("left", left_compressed),
            ("right", right_compressed),
        ):
            streams[side] = (
                {
                    "encoding": "y8_zstd",
                    "frames": 2,
                    "bytes": eye_bytes,
                    "compressed_bytes": compressed_bytes,
                    "zstd_level": 1,
                    "zstd_threads": 1,
                    "queue_overflows": 0,
                }
                if ir_encoding == "y8_split_zstd"
                else {
                    "encoding": "y8_h265_annex_b",
                    "codec": "h265",
                    "encoder": "rockchip_mpp",
                    "lossless": False,
                    "target_bitrate": IR_H265_BITRATE_PER_EYE,
                    "gop_frames": IR_H265_GOP_FRAMES,
                    "frames": 2,
                    "bytes": eye_bytes,
                    "compressed_bytes": compressed_bytes,
                    "queue_overflows": 0,
                }
            )
        compressed_total = left_compressed + right_compressed
        ir_metrics["writer"] = {
            "frames": 2,
            "input_bytes": eye_bytes * 2,
            "compressed_bytes": compressed_total,
            "compression_ratio": compressed_total / (eye_bytes * 2),
            "queue_overflows": 0,
            "streams": streams,
        }
    manifest = {
        "schema": SCHEMA,
        "status": "SEALED",
        "session_id": root.name,
        "warnings": [],
        "profile": config["profile"],
        "device": config["device"],
        "counts": {"ir_frames": 2, "rgb_input_frames": 2, "stm32_packets": 2},
        "metrics": metrics,
        "files": files,
    }
    _write_json(root / "manifest.json", manifest)
    return root


def test_parse_v4l2_frame_ignores_non_frame_and_exposes_driver_timestamp() -> None:
    assert parse_v4l2_frame_line("VIDIOC_STREAMON returned 0") is None
    row = parse_v4l2_frame_line(
        "cap dqbuf: 2 seq: 42 bytesused: 1843200 ts: 835.553596 "
        "delta: 33.290 ms field: None (ts-monotonic, ts-src-soe)"
    )
    assert row == {
        "sequence": 42,
        "bytes_used": 1_843_200,
        "source_monotonic_ns": 835_553_596_000,
        "flags": "delta: 33.290 ms field: None (ts-monotonic, ts-src-soe)",
    }


def test_parse_gstreamer_rgb_timing() -> None:
    line = (
        "/GstPipeline:pipeline0/GstIdentity:rgbraw: last-message = chain   ******* "
        "(rgbraw:sink) (1843200 bytes, dts: 0:00:00.502800810, "
        "pts: 0:00:00.502800810, duration: 0:00:00.033333333)"
    )
    assert parse_gstreamer_rgb_line(line) == {
        "bytes_used": 1_843_200,
        "pipeline_pts_ns": 502_800_810,
    }


def test_extract_complete_jpegs_handles_chunk_boundaries_and_noise() -> None:
    first = b"\xff\xd8first\xff\xd9"
    second = b"\xff\xd8second\xff\xd9"
    buffer = bytearray(b"noise" + first + second[:-1])
    assert extract_complete_jpegs(buffer) == [first]
    buffer.extend(second[-1:])
    assert extract_complete_jpegs(buffer) == [second]
    assert buffer == bytearray()


def test_preview_queue_preserves_frames_published_in_one_batch() -> None:
    state = _PreviewState()
    state.publish(b"first")
    state.publish(b"second")
    assert state.wait_after(0, 0.01) == (1, b"first")
    assert state.wait_after(1, 0.01) == (2, b"second")


def test_capture_duration_allows_one_hour_soak_but_rejects_longer(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("three_device_slam.edge_rk3576.capture.shutil.which", lambda _: None)
    with pytest.raises(CaptureError, match="required executable"):
        capture_session(
            output_root=tmp_path,
            duration_s=2_898,
            d405_serial="test",
        )
    with pytest.raises(CaptureError, match="between 1 and 3600 seconds"):
        capture_session(
            output_root=tmp_path,
            duration_s=3_601,
            d405_serial="test",
        )


def test_frame_spooler_preserves_complete_frames_and_reports_bounded_queue(
    tmp_path: Path,
) -> None:
    fifo = tmp_path / "frames.pipe"
    output = tmp_path / "frames.raw"
    spooler = _FrameSpooler(fifo, output, frame_bytes=4, queue_frames=2)
    spooler.start()
    descriptor = os.open(fifo, os.O_WRONLY)
    try:
        os.write(descriptor, b"abcdefgh")
    finally:
        os.close(descriptor)
    spooler.finish()

    assert output.read_bytes() == b"abcdefgh"
    assert not fifo.exists()
    metrics = spooler.metrics()
    assert metrics == {
        "frames": 2,
        "bytes": 8,
        "queue_capacity_frames": 2,
        "maximum_queue_depth_frames": metrics["maximum_queue_depth_frames"],
        "direct_io": False,
        "direct_io_rate_bytes_per_second": None,
    }
    assert 1 <= metrics["maximum_queue_depth_frames"] <= 2


def test_frame_spooler_rejects_truncated_frame_and_removes_fifo(
    tmp_path: Path,
) -> None:
    fifo = tmp_path / "frames.pipe"
    spooler = _FrameSpooler(fifo, tmp_path / "frames.raw", frame_bytes=4)
    spooler.start()
    descriptor = os.open(fifo, os.O_WRONLY)
    try:
        os.write(descriptor, b"abcde")
    finally:
        os.close(descriptor)

    with pytest.raises(CaptureError, match="partial frame"):
        spooler.finish()
    assert not fifo.exists()


def test_rsusb_stereo_interleave_preserves_left_right_pixel_order() -> None:
    assert interleave_stereo_y8(b"\x01\x02\x03", b"\x11\x12\x13") == (
        b"\x01\x11\x02\x12\x03\x13"
    )
    with pytest.raises(CaptureError, match="different payload sizes"):
        interleave_stereo_y8(b"\x01", b"\x11\x12")


def test_rsusb_async_writer_preserves_frame_boundaries(tmp_path: Path) -> None:
    output = tmp_path / "rsusb.raw"
    writer = _AsyncFrameWriter(output, frame_bytes=4, queue_frames=2)
    writer.start()
    writer.submit(b"abcd")
    writer.submit(b"efgh")
    writer.finish()

    assert output.read_bytes() == b"abcdefgh"
    metrics = writer.metrics()
    assert metrics["frames"] == 2
    assert metrics["bytes"] == 8
    assert metrics["queue_overflows"] == 0
    assert 1 <= metrics["maximum_queue_depth_frames"] <= 2


def test_rsusb_python_api_defaults_to_slam_verified_split_h265_ir() -> None:
    assert capture_rsusb_session.__kwdefaults__["ir_encoding"] == "y8_split_h265"


@pytest.mark.parametrize("ir_encoding", ["y8i_zstd", "y8i_raw"])
def test_rsusb_signal_control_rejects_legacy_ir_schemas(
    tmp_path: Path, ir_encoding: str
) -> None:
    with pytest.raises(CaptureError, match="requires split-eye schema v3 or v4"):
        capture_rsusb_session(
            output_root=tmp_path,
            duration_s=60,
            d405_sdk_serial="sdk",
            d405_usb_serial="usb",
            ir_encoding=ir_encoding,
            stop_requested=threading.Event(),
        )


def test_rsusb_cli_until_signal_supplies_a_stop_event(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    observed = {}
    original_sigterm = signal.getsignal(signal.SIGTERM)

    def fake_capture(**kwargs):
        observed.update(kwargs)
        assert isinstance(kwargs["stop_requested"], threading.Event)
        os.kill(os.getpid(), signal.SIGTERM)
        assert kwargs["stop_requested"].is_set()
        return tmp_path / "sealed"

    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture.capture_rsusb_session",
        fake_capture,
    )
    assert rsusb_main(
        [
            "--output-root",
            str(tmp_path),
            "--duration",
            "60",
            "--until-signal",
            "--d405-sdk-serial",
            "sdk",
            "--d405-usb-serial",
            "usb",
        ]
    ) == 0
    assert observed["duration_s"] == 60
    assert signal.getsignal(signal.SIGTERM) is original_sigterm


def test_rsusb_cli_second_signal_force_aborts_from_any_main_phase(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    original_sigterm = signal.getsignal(signal.SIGTERM)

    def fake_capture(**kwargs):
        os.kill(os.getpid(), signal.SIGTERM)
        assert kwargs["stop_requested"].is_set()
        os.kill(os.getpid(), signal.SIGTERM)
        raise AssertionError("the second signal must interrupt this phase")

    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture.capture_rsusb_session",
        fake_capture,
    )
    assert rsusb_main(
        [
            "--output-root",
            str(tmp_path),
            "--duration",
            "60",
            "--until-signal",
            "--d405-sdk-serial",
            "sdk",
            "--d405-usb-serial",
            "usb",
        ]
    ) == 130
    assert signal.getsignal(signal.SIGTERM) is original_sigterm


def test_rsusb_async_writer_rejects_wrong_frame_size(tmp_path: Path) -> None:
    writer = _AsyncFrameWriter(tmp_path / "rsusb.raw", frame_bytes=4)
    writer.start()
    with pytest.raises(CaptureError, match="payload size"):
        writer.submit(b"bad")
    writer.finish()


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_rsusb_zstd_writer_is_lossless_and_reports_ratio(tmp_path: Path) -> None:
    output = tmp_path / "rsusb.raw.zst"
    writer = _ZstdFrameWriter(output, frame_bytes=4, queue_frames=2, threads=1)
    writer.start()
    writer.submit(b"abcd")
    writer.submit(b"efgh")
    writer.finish()

    result = subprocess.run(
        ["zstd", "--decompress", "--quiet", "--stdout", str(output)],
        check=True,
        stdout=subprocess.PIPE,
    )
    assert result.stdout == b"abcdefgh"
    metrics = writer.metrics()
    assert metrics["frames"] == 2
    assert metrics["bytes"] == 8
    assert metrics["compressed_bytes"] == output.stat().st_size
    assert metrics["queue_overflows"] == 0


def test_rsusb_zstd_writer_kills_a_stalled_compressor(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class StalledStdin:
        def __init__(self, stopped):
            self.stopped = stopped
            self.closed = False

        def write(self, _payload):
            self.stopped.wait()
            raise BrokenPipeError("compressor stopped")

        def close(self):
            self.closed = True

    class StalledProcess:
        def __init__(self):
            self.stopped = threading.Event()
            self.stdin = StalledStdin(self.stopped)
            self.stderr = io.BytesIO()
            self.killed = False

        def poll(self):
            return -9 if self.stopped.is_set() else None

        def kill(self):
            self.killed = True
            self.stopped.set()

        def wait(self, timeout=None):
            assert self.stopped.wait(timeout)
            return -9

    process = StalledProcess()
    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture.subprocess.Popen",
        lambda *args, **kwargs: process,
    )
    writer = _ZstdFrameWriter(
        tmp_path / "stalled.raw.zst",
        frame_bytes=4,
        queue_frames=1,
        threads=1,
    )
    writer.start()
    writer.submit(b"abcd")

    with pytest.raises(CaptureError, match="did not stop"):
        writer.finish(timeout_s=0.05)

    assert process.killed is True
    assert writer._thread.is_alive() is False


def test_rsusb_rgb_pipeline_never_clock_throttles_recording_or_preview(
    tmp_path: Path,
) -> None:
    command = _rgb_gstreamer_command(tmp_path / "rgb.h265", tmp_path / "preview.pipe")
    assert command.count("sync=false") == 2
    assert "video/x-raw,framerate=15/1" in command
    assert "video/x-raw,framerate=10/1" not in command


def test_ir_h265_pipeline_matches_ego_scaled_profile(tmp_path: Path) -> None:
    command = _ir_h265_gstreamer_command(tmp_path / "ir-left.h265")

    assert "format=gray8" in command
    assert "video/x-raw,format=NV12" in command
    assert "mpph265enc" in command
    assert f"bps={IR_H265_BITRATE_PER_EYE}" in command
    assert "bps-min=3240000" in command
    assert "bps-max=3672000" in command
    assert f"gop={IR_H265_GOP_FRAMES}" in command
    assert "header-mode=each-idr" in command
    assert "max-reenc=0" in command
    assert "sync=false" in command


def test_rsusb_rgb_pipeline_produces_app_ready_h264_rtp(tmp_path: Path) -> None:
    command = _rgb_gstreamer_command(
        tmp_path / "rgb.h265",
        None,
        preview_rtp_host="127.0.0.1",
        preview_rtp_port=5004,
    )

    assert "mpph264enc" in command
    assert "video/x-h264,stream-format=byte-stream,alignment=au" in command
    assert "rtph264pay" in command
    assert "pt=96" in command
    assert "udpsink" in command
    assert "host=127.0.0.1" in command
    assert "port=5004" in command
    assert "jpegenc" not in command
    assert command.count("sync=false") == 2


def test_rsusb_rgb_pipeline_rejects_two_preview_outputs(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="mutually exclusive"):
        _rgb_gstreamer_command(
            tmp_path / "rgb.h265",
            tmp_path / "preview.pipe",
            preview_rtp_host="127.0.0.1",
        )


def test_rgb_encoder_cancel_stops_an_idle_writer_thread(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class FakeProcess:
        def __init__(self) -> None:
            self.stdin = io.BytesIO()
            self.returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, _signal):
            self.returncode = 0

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            return 0 if self.returncode is None else self.returncode

    process = FakeProcess()
    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture.subprocess.Popen",
        lambda *args, **kwargs: process,
    )
    encoder = _RgbEncoder(
        tmp_path / "rgb.h265",
        tmp_path / "rgb.log",
        preview_fifo=None,
    )
    encoder.start()
    encoder.cancel()

    assert encoder._thread.is_alive() is False


def test_rgb_encoder_isolated_from_terminal_signal_session(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class FakeProcess:
        def __init__(self) -> None:
            self.stdin = io.BytesIO()
            self.returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, _signal):
            self.returncode = 0

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            return 0 if self.returncode is None else self.returncode

    popen_options = {}

    def fake_popen(*args, **kwargs):
        popen_options.update(kwargs)
        return FakeProcess()

    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture.subprocess.Popen",
        fake_popen,
    )
    encoder = _RgbEncoder(
        tmp_path / "rgb.h265",
        tmp_path / "rgb.log",
        preview_fifo=None,
    )
    encoder.start()
    encoder.cancel()

    assert popen_options["start_new_session"] is True


@pytest.mark.parametrize("writer_kind", ["zstd", "h265"])
def test_ir_writers_are_isolated_from_terminal_signal_session(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, writer_kind: str
) -> None:
    class FakeProcess:
        def __init__(self) -> None:
            self.stdin = io.BytesIO()
            self.stderr = io.BytesIO()
            self.returncode = None

        def poll(self):
            return self.returncode

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            self.returncode = 0 if self.returncode is None else self.returncode
            return self.returncode

    popen_options = {}
    output = tmp_path / f"ir.{writer_kind}"

    def fake_popen(*args, **kwargs):
        popen_options.update(kwargs)
        if writer_kind == "h265":
            output.write_bytes(b"encoded")
        return FakeProcess()

    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture.shutil.which",
        lambda _name: "/usr/bin/fake",
    )
    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture.subprocess.Popen",
        fake_popen,
    )
    writer = (
        _ZstdFrameWriter(output, frame_bytes=4, queue_frames=1, threads=1)
        if writer_kind == "zstd"
        else _H265FrameWriter(
            output,
            tmp_path / "ir-gstreamer.log",
            frame_bytes=4,
            queue_frames=1,
        )
    )
    writer.start()
    writer.finish()

    assert popen_options["start_new_session"] is True


def test_capture_cleanup_attempts_every_started_encoder_after_one_fails() -> None:
    calls = []

    class FakeWorker:
        def __init__(self, name: str, *, fail: bool = False) -> None:
            self.name = name
            self.fail = fail

        def cancel(self) -> None:
            calls.append(self.name)
            if self.fail:
                raise OSError(f"{self.name} cancel failed")

    failures = _cancel_started_capture_children(
        FakeWorker("rgb", fail=True),
        encoder_started=True,
        started_ir_writers=[
            ("left", FakeWorker("left")),
            ("right", FakeWorker("right")),
        ],
    )

    assert calls == ["rgb", "left", "right"]
    assert failures == ["rgb: OSError: rgb cancel failed"]


def test_capture_cleanup_reraises_force_abort_after_trying_every_encoder() -> None:
    calls = []

    class FakeWorker:
        def __init__(self, name: str, *, force_abort: bool = False) -> None:
            self.name = name
            self.force_abort = force_abort

        def cancel(self) -> None:
            calls.append(self.name)
            if self.force_abort:
                raise KeyboardInterrupt("second signal")

    with pytest.raises(KeyboardInterrupt, match="second signal"):
        _cancel_started_capture_children(
            FakeWorker("rgb", force_abort=True),
            encoder_started=True,
            started_ir_writers=[
                ("left", FakeWorker("left")),
                ("right", FakeWorker("right")),
            ],
        )

    assert calls == ["rgb", "left", "right"]


def test_publish_session_rolls_back_if_parent_fsync_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    partial = tmp_path / ".session.partial"
    final = tmp_path / "session"
    partial.mkdir()
    (partial / ".recording").write_text("unsealed\n", encoding="ascii")
    real_fsync = os.fsync
    failed = False

    def fail_first_parent_fsync(path: Path) -> None:
        nonlocal failed
        if path == tmp_path and final.exists() and not failed:
            failed = True
            raise OSError("injected parent fsync failure")
        descriptor = os.open(path, os.O_RDONLY)
        try:
            real_fsync(descriptor)
        finally:
            os.close(descriptor)

    monkeypatch.setattr(
        "three_device_slam.edge_rk3576.rsusb_capture._fsync_directory",
        fail_first_parent_fsync,
    )
    with pytest.raises(OSError, match="injected parent fsync failure"):
        _publish_session(partial, final, tmp_path)

    assert partial.is_dir()
    assert (partial / ".recording").is_file()
    assert not final.exists()


def test_sealed_session_validates(tmp_path: Path) -> None:
    report = validate_rk3576_session(_sealed_session(tmp_path / "session"))
    assert report["status"] == "PASS"
    assert report["counts"] == {
        "ir_frames": 2,
        "rgb_input_frames": 2,
        "stm32_packets": 2,
    }


def test_rsusb_sealed_session_schema_validates(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SCHEMA
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SCHEMA
    manifest["files"]["session_config.json"] = {
        "size": config_path.stat().st_size,
        "sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
    }
    _write_json(manifest_path, manifest)

    assert validate_rk3576_session(root)["status"] == "PASS"


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_zstd_ir_session_validates_without_expanding_to_disk(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8i_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_ZSTD_SCHEMA
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_ZSTD_SCHEMA
    manifest["files"]["session_config.json"] = {
        "size": config_path.stat().st_size,
        "sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
    }
    _write_json(manifest_path, manifest)

    assert validate_rk3576_session(root)["status"] == "PASS"
    assert not (root / "infrared-y8i.raw").exists()


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_ir_session_validates_without_expanding_to_disk(
    tmp_path: Path,
) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    manifest["files"]["session_config.json"] = {
        "size": config_path.stat().st_size,
        "sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
    }
    _write_json(manifest_path, manifest)

    assert validate_rk3576_session(root)["status"] == "PASS"
    assert not (root / "infrared-y8i.raw.zst").exists()


def test_split_h265_ir_session_validates_as_v4_candidate(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_h265")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_H265_SCHEMA
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_H265_SCHEMA
    _refresh_claim(manifest, config_path)
    _write_json(manifest_path, manifest)

    report = validate_rk3576_session(root)
    assert report["status"] == "PASS"
    assert report["ir_decoded_frames"] is None


def test_split_h265_rejects_wrong_writer_bitrate(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_h265")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_H265_SCHEMA
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_H265_SCHEMA
    manifest["metrics"]["ir"]["writer"]["streams"]["left"][
        "target_bitrate"
    ] = 1
    _refresh_claim(manifest, config_path)
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="writer metric mismatch"):
        validate_rk3576_session(root)


def test_split_h265_rejects_non_annex_b_payload(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_h265")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_H265_SCHEMA
    _write_json(config_path, config)
    ir_path = root / "infrared-left-y8.h265"
    ir_path.write_bytes(b"not-annex-b")
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_H265_SCHEMA
    manifest["metrics"]["ir"]["writer"]["streams"]["left"][
        "compressed_bytes"
    ] = ir_path.stat().st_size
    manifest["metrics"]["ir"]["writer"]["compressed_bytes"] = (
        ir_path.stat().st_size + (root / "infrared-right-y8.h265").stat().st_size
    )
    manifest["metrics"]["ir"]["writer"]["compression_ratio"] = (
        manifest["metrics"]["ir"]["writer"]["compressed_bytes"]
        / manifest["metrics"]["ir"]["writer"]["input_bytes"]
    )
    _refresh_claim(manifest, config_path)
    _refresh_claim(manifest, ir_path)
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="not Annex-B"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_rejects_incomplete_requested_capture(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    config["requested"]["frames"] = 900
    config["requested"]["duration_s"] = 30
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    _refresh_claim(manifest, config_path)
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="requested frame count"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_rejects_fixed_duration_frame_mismatch(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    config["requested"] = {
        "mode": "fixed_frames",
        "duration_s": 1,
        "frames": 2,
    }
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    _refresh_claim(manifest, config_path)
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="duration/frame relationship"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_signal_controlled_session_validates_termination(
    tmp_path: Path,
) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    config["requested"] = {"mode": "until_signal", "maximum_duration_s": 60}
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    manifest["metrics"]["termination"] = {
        "mode": "until_signal",
        "reason": "external_stop",
        "captured_frames": 2,
        "stop_observed_monotonic_ns": 1_033_333_333,
        "storage_guard": None,
    }
    manifest["metrics"]["formal_start_monotonic_ns"] = 1_000_000_000
    manifest["metrics"]["formal_stop_monotonic_ns"] = 1_033_333_333
    manifest["metrics"]["formal_host_span_s"] = 0.033333333
    _refresh_claim(manifest, config_path)
    _write_json(manifest_path, manifest)

    assert validate_rk3576_session(root)["status"] == "PASS"
    manifest["metrics"]["termination"]["captured_frames"] = 1
    _write_json(manifest_path, manifest)
    with pytest.raises(SessionValidationError, match="termination evidence"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("formal_span", "span does not match frame count"),
        ("stop_outside", "timestamp is outside the capture window"),
        ("stream_rate", "observed rate is outside the 30 Hz band"),
        ("stream_span", "timestamp span is inconsistent"),
    ],
)
def test_split_zstd_rejects_impossible_signal_timing(
    tmp_path: Path, mutation: str, message: str
) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    config["requested"] = {"mode": "until_signal", "maximum_duration_s": 60}
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    manifest["metrics"].update(
        {
            "formal_start_monotonic_ns": 1_000_000_000,
            "formal_stop_monotonic_ns": 1_033_333_333,
            "formal_host_span_s": 0.033333333,
            "termination": {
                "mode": "until_signal",
                "reason": "external_stop",
                "captured_frames": 2,
                "stop_observed_monotonic_ns": 1_033_333_333,
                "storage_guard": None,
            },
        }
    )
    if mutation == "formal_span":
        manifest["metrics"]["formal_stop_monotonic_ns"] = 11_000_000_000
        manifest["metrics"]["formal_host_span_s"] = 10.0
        manifest["metrics"]["termination"]["stop_observed_monotonic_ns"] = 2_000_000_000
    elif mutation == "stop_outside":
        manifest["metrics"]["termination"]["stop_observed_monotonic_ns"] = 2_000_000_000
    elif mutation == "stream_rate":
        manifest["metrics"]["rsusb_streams"]["color"]["observed_rate_hz"] = 15.0
    else:
        manifest["metrics"]["rsusb_streams"]["color"]["timestamp_span_s"] = 0.2
    _refresh_claim(manifest, config_path)
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match=message):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_rejects_false_maximum_and_low_storage_termination(
    tmp_path: Path,
) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    config["requested"] = {"mode": "until_signal", "maximum_duration_s": 60}
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    manifest["metrics"]["formal_start_monotonic_ns"] = 1_000_000_000
    manifest["metrics"]["formal_stop_monotonic_ns"] = 1_033_333_333
    manifest["metrics"]["formal_host_span_s"] = 0.033333333
    manifest["metrics"]["termination"] = {
        "mode": "until_signal",
        "reason": "maximum_duration_complete",
        "captured_frames": 2,
        "stop_observed_monotonic_ns": None,
        "storage_guard": None,
    }
    _refresh_claim(manifest, config_path)
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="maximum-duration"):
        validate_rk3576_session(root)

    manifest["metrics"]["termination"].update(
        {
            "reason": "low_storage_guard",
            "stop_observed_monotonic_ns": 1_500_000_000,
        }
    )
    _write_json(manifest_path, manifest)
    with pytest.raises(SessionValidationError, match="storage_guard"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_rejects_writer_overflow_and_stream_count_mismatch(
    tmp_path: Path,
) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    _refresh_claim(manifest, config_path)
    manifest["metrics"]["rsusb_streams"]["infrared_right"]["received"] = 1
    manifest["metrics"]["ir"]["writer"]["queue_overflows"] = 7
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="received count mismatch"):
        validate_rk3576_session(root)

    manifest["metrics"]["rsusb_streams"]["infrared_right"]["received"] = 2
    _write_json(manifest_path, manifest)
    with pytest.raises(SessionValidationError, match="queue_overflows"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_rejects_missing_eye_index_metadata(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    _write_json(config_path, config)
    index_path = root / "ir_frames.jsonl"
    rows = [json.loads(line) for line in index_path.read_text().splitlines()]
    rows[0].pop("left_sequence")
    index_path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_SPLIT_ZSTD_SCHEMA
    _refresh_claim(manifest, config_path)
    _refresh_claim(manifest, index_path)
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="eye sequence mismatch"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_split_zstd_ir_rejects_a_legacy_schema_identity(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8_split_zstd")

    with pytest.raises(SessionValidationError, match="requires the RSUSB v3"):
        validate_rk3576_session(root)


@pytest.mark.skipif(shutil.which("zstd") is None, reason="zstd is unavailable")
def test_zstd_ir_rejects_a_legacy_schema_identity(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session", ir_encoding="y8i_zstd")

    with pytest.raises(SessionValidationError, match="requires the RSUSB v2"):
        validate_rk3576_session(root)


def test_rsusb_v2_rejects_an_uncompressed_ir_payload(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session")
    config_path = root / "session_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["schema"] = RSUSB_ZSTD_SCHEMA
    _write_json(config_path, config)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema"] = RSUSB_ZSTD_SCHEMA
    manifest["files"]["session_config.json"] = {
        "size": config_path.stat().st_size,
        "sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
    }
    _write_json(manifest_path, manifest)

    with pytest.raises(SessionValidationError, match="requires compressed Y8I"):
        validate_rk3576_session(root)


def test_payload_corruption_is_detected(tmp_path: Path) -> None:
    root = _sealed_session(tmp_path / "session")
    with (root / "stm32.bin").open("r+b") as stream:
        stream.seek(10)
        stream.write(b"X")
    with pytest.raises(SessionValidationError, match="SHA-256"):
        validate_rk3576_session(root)
