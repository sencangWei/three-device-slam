"""Single-owner RSUSB D405 + STM32 capture for the RK3576 edge host."""

from __future__ import annotations

import argparse
import datetime as dt
import ipaddress
import json
import mmap
import os
import queue
import shutil
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from three_device_slam.edge_rk3576.capture import (
    CAPTURE_QUEUE_FRAMES,
    DIRECT_IO_CHUNK_BYTES,
    FPS,
    HEIGHT,
    IR_FRAME_BYTES,
    MIN_FREE_RESERVE_BYTES,
    RGB_BITRATE,
    RGB_FPS,
    SCHEMA,
    STM32_PACKET_BYTES,
    WIDTH,
    CaptureError,
    _SerialCollector,
    _device_owners,
    _file_manifest,
    _fsync_directory,
    _json_bytes,
    _kernel_usb_events,
    _thermal_snapshot,
    _write_atomic,
    discover_stm32_port,
    verify_d405_superspeed,
)
from three_device_slam.edge_rk3576.preview import MjpegPreviewServer
from three_device_slam.edge_rk3576.rsusb_probe import (
    PAYLOAD_BYTES,
    StreamContinuity,
    _module_evidence,
    _select_profiles,
    _stream_key,
    evaluate_streams,
)


RSUSB_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v1"
RSUSB_ZSTD_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v2"
RSUSB_SPLIT_ZSTD_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v3"
RSUSB_SPLIT_H265_SCHEMA = "three-device-slam.rk3576-umi-rsusb-session.v4"
WARMUP_SECONDS = 3.0
PAIRING_WINDOW_FRAMES = 16
RSUSB_STORAGE_QUEUE_FRAMES = 256
RSUSB_DIRECT_IO_RATE_BYTES_PER_SECOND = 96 * 1024 * 1024
IR_ENCODINGS = ("y8_split_zstd", "y8_split_h265", "y8i_zstd", "y8i_raw")
ZSTD_LEVEL = 1
ZSTD_THREADS = 2
SPLIT_ZSTD_THREADS_PER_EYE = 1
IR_EYE_FRAME_BYTES = WIDTH * HEIGHT
IR_H265_BITRATE_PER_EYE = 3_456_000
IR_H265_BITRATE_MIN_PER_EYE = 3_240_000
IR_H265_BITRATE_MAX_PER_EYE = 3_672_000
IR_H265_GOP_FRAMES = FPS
STOP_ALIGNMENT_TIMEOUT_SECONDS = 5.0
STORAGE_STOP_HEADROOM_BYTES = (
    RSUSB_STORAGE_QUEUE_FRAMES * IR_FRAME_BYTES
    + CAPTURE_QUEUE_FRAMES * PAYLOAD_BYTES["color"]
    + IR_FRAME_BYTES * FPS * 2
)


def interleave_stereo_y8(left: bytes, right: bytes) -> bytes:
    """Return the existing Y8I layout: left byte then right byte per pixel."""
    if len(left) != len(right):
        raise CaptureError(
            f"stereo IR frames have different payload sizes: {len(left)} != {len(right)}"
        )
    interleaved = bytearray(len(left) * 2)
    interleaved[0::2] = left
    interleaved[1::2] = right
    return bytes(interleaved)


class _AsyncFrameWriter:
    """Bounded non-blocking producer with aligned, paced frame writes."""

    def __init__(
        self,
        output_path: Path,
        *,
        frame_bytes: int,
        queue_frames: int = RSUSB_STORAGE_QUEUE_FRAMES,
    ) -> None:
        if frame_bytes < 1 or queue_frames < 1:
            raise ValueError("frame_bytes and queue_frames must be positive")
        self.output_path = output_path
        self.frame_bytes = frame_bytes
        self.queue_frames = queue_frames
        self._queue: queue.Queue[bytes | None] = queue.Queue(queue_frames)
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._done = threading.Event()
        self._cancel_requested = threading.Event()
        self._queue_depth_lock = threading.Lock()
        self._queued_frames = 0
        self._failure: BaseException | None = None
        self._started = False
        self.frames = 0
        self.bytes = 0
        self.queue_overflows = 0
        self.maximum_queue_depth_frames = 0
        self.maximum_frame_write_ms = 0.0
        self.slow_frame_writes = 0
        self.write_started_ns: int | None = None
        self.write_finished_ns: int | None = None
        self.direct_io = bool(
            getattr(os, "O_DIRECT", 0) and self.frame_bytes % mmap.PAGESIZE == 0
        )

    def start(self) -> None:
        if self._started:
            raise RuntimeError("frame writer is already started")
        self._thread.start()
        self._started = True

    def submit(self, frame: bytes) -> None:
        if len(frame) != self.frame_bytes:
            raise CaptureError(
                f"frame payload size is {len(frame)}, expected {self.frame_bytes}"
            )
        if not self._started or self._done.is_set():
            raise CaptureError("frame writer is not running")
        if self._failure is not None:
            raise CaptureError(f"frame writer failed: {self._failure}")
        try:
            with self._queue_depth_lock:
                self._queue.put_nowait(frame)
                self._queued_frames += 1
                self.maximum_queue_depth_frames = max(
                    self.maximum_queue_depth_frames, self._queued_frames
                )
        except queue.Full as exc:
            self.queue_overflows += 1
            raise CaptureError("frame writer queue overflow") from exc

    def _mark_frame_dequeued(self, frame: bytes | None) -> None:
        if frame is None:
            return
        with self._queue_depth_lock:
            self._queued_frames -= 1

    def _write_loop(self) -> None:
        descriptor: int | None = None
        aligned: mmap.mmap | None = None
        try:
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
            if self.direct_io:
                flags |= os.O_DIRECT
            descriptor = os.open(self.output_path, flags, 0o644)
            if self.direct_io:
                aligned = mmap.mmap(-1, self.frame_bytes)
            while True:
                frame = self._queue.get()
                try:
                    self._mark_frame_dequeued(frame)
                    if frame is None:
                        break
                    if self._cancel_requested.is_set():
                        break
                    frame_write_started_ns = time.monotonic_ns()
                    if self.write_started_ns is None:
                        self.write_started_ns = frame_write_started_ns
                    if aligned is not None:
                        aligned[:] = frame
                        full_view = memoryview(aligned)
                        started = time.monotonic()
                        offset = 0
                        try:
                            while offset < len(full_view):
                                end = min(offset + DIRECT_IO_CHUNK_BYTES, len(full_view))
                                view = full_view[offset:end]
                                try:
                                    while view:
                                        written = os.write(descriptor, view)
                                        if not written:
                                            raise OSError("frame writer made no progress")
                                        offset += written
                                        view = view[written:]
                                finally:
                                    view.release()
                                target = (
                                    started
                                    + offset / RSUSB_DIRECT_IO_RATE_BYTES_PER_SECOND
                                )
                                delay = target - time.monotonic()
                                if delay > 0:
                                    time.sleep(delay)
                        finally:
                            full_view.release()
                    else:
                        view = memoryview(frame)
                        try:
                            while view:
                                written = os.write(descriptor, view)
                                if not written:
                                    raise OSError("frame writer made no progress")
                                view = view[written:]
                        finally:
                            view.release()
                    self.frames += 1
                    self.bytes += len(frame)
                    self.write_finished_ns = time.monotonic_ns()
                    frame_write_ms = (
                        self.write_finished_ns - frame_write_started_ns
                    ) / 1_000_000.0
                    self.maximum_frame_write_ms = max(
                        self.maximum_frame_write_ms, frame_write_ms
                    )
                    if frame_write_ms > 1000.0 / FPS:
                        self.slow_frame_writes += 1
                finally:
                    self._queue.task_done()
            os.fsync(descriptor)
        except BaseException as exc:
            self._failure = exc
        finally:
            if aligned is not None:
                aligned.close()
            if descriptor is not None:
                os.close(descriptor)
            self._done.set()

    def finish(self, *, timeout_s: float = 60.0) -> None:
        if not self._started:
            return
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        deadline = time.monotonic() + timeout_s
        while not self._done.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.cancel()
                raise CaptureError("frame writer stop request timed out")
            try:
                self._queue.put(None, timeout=min(0.1, remaining))
                break
            except queue.Full:
                continue
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if self._thread.is_alive():
            self.cancel()
            raise CaptureError("frame writer did not stop")
        if self._failure is not None:
            raise CaptureError(f"frame writer failed: {self._failure}")

    def cancel(self, *, timeout_s: float = 5.0) -> None:
        """Request bounded best-effort shutdown after a failed capture."""
        if not self._started:
            return
        self._cancel_requested.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=max(0.0, timeout_s))

    def metrics(self) -> dict[str, Any]:
        span_s = (
            (self.write_finished_ns - self.write_started_ns) / 1_000_000_000.0
            if self.write_started_ns is not None
            and self.write_finished_ns is not None
            and self.write_finished_ns > self.write_started_ns
            else 0.0
        )
        return {
            "frames": self.frames,
            "bytes": self.bytes,
            "queue_capacity_frames": self.queue_frames,
            "maximum_queue_depth_frames": self.maximum_queue_depth_frames,
            "queue_overflows": self.queue_overflows,
            "maximum_frame_write_ms": self.maximum_frame_write_ms,
            "slow_frame_writes_over_33ms": self.slow_frame_writes,
            "observed_write_bytes_per_second": self.bytes / span_s if span_s else 0.0,
            "direct_io": self.direct_io,
            "direct_io_rate_bytes_per_second": (
                RSUSB_DIRECT_IO_RATE_BYTES_PER_SECOND if self.direct_io else None
            ),
        }


class _ZstdFrameWriter(_AsyncFrameWriter):
    """Losslessly stream complete Y8I frames into one checksummed zstd frame."""

    def __init__(
        self,
        output_path: Path,
        *,
        frame_bytes: int,
        queue_frames: int = RSUSB_STORAGE_QUEUE_FRAMES,
        threads: int = ZSTD_THREADS,
        encoding: str = "y8i_zstd",
    ) -> None:
        if shutil.which("zstd") is None:
            raise CaptureError("required executable is unavailable: zstd")
        if threads < 1:
            raise ValueError("threads must be positive")
        super().__init__(
            output_path,
            frame_bytes=frame_bytes,
            queue_frames=queue_frames,
        )
        self.direct_io = False
        self.threads = threads
        self.encoding = encoding
        self.command = [
            "zstd",
            "--quiet",
            f"-{ZSTD_LEVEL}",
            f"-T{threads}",
            "--stdout",
        ]
        self.compressed_bytes = 0
        self._process: subprocess.Popen[bytes] | None = None

    def _write_loop(self) -> None:
        process: subprocess.Popen[bytes] | None = None
        output = None
        try:
            output = self.output_path.open("xb", buffering=0)
            process = subprocess.Popen(
                self.command,
                stdin=subprocess.PIPE,
                stdout=output,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
            self._process = process
            if process.stdin is None:
                raise RuntimeError("zstd stdin is unavailable")
            while True:
                frame = self._queue.get()
                try:
                    self._mark_frame_dequeued(frame)
                    if frame is None:
                        break
                    if self._cancel_requested.is_set():
                        break
                    frame_write_started_ns = time.monotonic_ns()
                    if self.write_started_ns is None:
                        self.write_started_ns = frame_write_started_ns
                    view = memoryview(frame)
                    try:
                        while view:
                            written = process.stdin.write(view)
                            if not written:
                                raise OSError("zstd writer made no progress")
                            view = view[written:]
                    finally:
                        view.release()
                    self.frames += 1
                    self.bytes += len(frame)
                    self.write_finished_ns = time.monotonic_ns()
                    frame_write_ms = (
                        self.write_finished_ns - frame_write_started_ns
                    ) / 1_000_000.0
                    self.maximum_frame_write_ms = max(
                        self.maximum_frame_write_ms, frame_write_ms
                    )
                    if frame_write_ms > 1000.0 / FPS:
                        self.slow_frame_writes += 1
                finally:
                    self._queue.task_done()
            process.stdin.close()
            return_code = process.wait(timeout=45.0)
            stderr = process.stderr.read().decode("utf-8", errors="replace")
            if return_code != 0:
                raise OSError(f"zstd exited {return_code}: {stderr.strip()}")
            output.flush()
            os.fsync(output.fileno())
            self.compressed_bytes = self.output_path.stat().st_size
        except BaseException as exc:
            self._failure = exc
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=5.0)
        finally:
            if process is not None:
                if process.stdin is not None and not process.stdin.closed:
                    process.stdin.close()
                if process.stderr is not None:
                    process.stderr.close()
            if output is not None:
                output.close()
            self._process = None
            self._done.set()

    def cancel(self, *, timeout_s: float = 5.0) -> None:
        self._cancel_requested.set()
        process = self._process
        if process is not None and process.poll() is None:
            process.kill()
        super().cancel(timeout_s=timeout_s)

    def metrics(self) -> dict[str, Any]:
        metrics = super().metrics()
        metrics.update(
            {
                "encoding": self.encoding,
                "zstd_level": ZSTD_LEVEL,
                "zstd_threads": self.threads,
                "compressed_bytes": self.compressed_bytes,
                "compression_ratio": (
                    self.compressed_bytes / self.bytes if self.bytes else None
                ),
            }
        )
        return metrics


def _ir_h265_gstreamer_command(output_path: Path) -> list[str]:
    """Return the EGO-equivalent RK MPP HEVC pipeline for one Y8 eye."""
    return [
        "gst-launch-1.0",
        "-q",
        "-e",
        "fdsrc",
        "fd=0",
        "!",
        "rawvideoparse",
        "format=gray8",
        f"width={WIDTH}",
        f"height={HEIGHT}",
        f"framerate={FPS}/1",
        "!",
        "videoconvert",
        "!",
        "video/x-raw,format=NV12",
        "!",
        "mpph265enc",
        f"bps={IR_H265_BITRATE_PER_EYE}",
        f"bps-min={IR_H265_BITRATE_MIN_PER_EYE}",
        f"bps-max={IR_H265_BITRATE_MAX_PER_EYE}",
        f"gop={IR_H265_GOP_FRAMES}",
        "header-mode=each-idr",
        "max-reenc=0",
        "!",
        "filesink",
        "sync=false",
        f"location={output_path}",
    ]


class _H265FrameWriter(_AsyncFrameWriter):
    """Encode one complete Y8 eye stream with the RK hardware HEVC encoder."""

    def __init__(
        self,
        output_path: Path,
        log_path: Path,
        *,
        frame_bytes: int = IR_EYE_FRAME_BYTES,
        queue_frames: int = RSUSB_STORAGE_QUEUE_FRAMES,
    ) -> None:
        if shutil.which("gst-launch-1.0") is None:
            raise CaptureError("required executable is unavailable: gst-launch-1.0")
        super().__init__(
            output_path,
            frame_bytes=frame_bytes,
            queue_frames=queue_frames,
        )
        self.direct_io = False
        self.log_path = log_path
        self.command = _ir_h265_gstreamer_command(output_path)
        self.compressed_bytes = 0
        self._process: subprocess.Popen[bytes] | None = None

    def _write_loop(self) -> None:
        process: subprocess.Popen[bytes] | None = None
        log = None
        try:
            log = self.log_path.open("xb")
            process = subprocess.Popen(
                self.command,
                stdin=subprocess.PIPE,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._process = process
            if process.stdin is None:
                raise RuntimeError("IR H.265 encoder stdin is unavailable")
            while True:
                frame = self._queue.get()
                try:
                    self._mark_frame_dequeued(frame)
                    if frame is None or self._cancel_requested.is_set():
                        break
                    frame_write_started_ns = time.monotonic_ns()
                    if self.write_started_ns is None:
                        self.write_started_ns = frame_write_started_ns
                    view = memoryview(frame)
                    try:
                        while view:
                            written = process.stdin.write(view)
                            if not written:
                                raise OSError("IR H.265 encoder made no progress")
                            view = view[written:]
                    finally:
                        view.release()
                    self.frames += 1
                    self.bytes += len(frame)
                    self.write_finished_ns = time.monotonic_ns()
                    frame_write_ms = (
                        self.write_finished_ns - frame_write_started_ns
                    ) / 1_000_000.0
                    self.maximum_frame_write_ms = max(
                        self.maximum_frame_write_ms, frame_write_ms
                    )
                    if frame_write_ms > 1000.0 / FPS:
                        self.slow_frame_writes += 1
                finally:
                    self._queue.task_done()
            process.stdin.close()
            return_code = process.wait(timeout=45.0)
            if return_code != 0:
                raise OSError(f"IR H.265 encoder exited with status {return_code}")
            log.flush()
            os.fsync(log.fileno())
            with self.output_path.open("r+b") as output:
                os.fsync(output.fileno())
            self.compressed_bytes = self.output_path.stat().st_size
        except BaseException as exc:
            self._failure = exc
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=5.0)
        finally:
            if (
                process is not None
                and process.stdin is not None
                and not process.stdin.closed
            ):
                process.stdin.close()
            if log is not None:
                log.close()
            self._process = None
            self._done.set()

    def cancel(self, *, timeout_s: float = 5.0) -> None:
        self._cancel_requested.set()
        process = self._process
        if process is not None and process.poll() is None:
            process.kill()
        super().cancel(timeout_s=timeout_s)

    def metrics(self) -> dict[str, Any]:
        metrics = super().metrics()
        metrics.update(
            {
                "encoding": "y8_h265_annex_b",
                "codec": "h265",
                "encoder": "rockchip_mpp",
                "lossless": False,
                "target_bitrate": IR_H265_BITRATE_PER_EYE,
                "gop_frames": IR_H265_GOP_FRAMES,
                "compressed_bytes": self.compressed_bytes,
                "compression_ratio": (
                    self.compressed_bytes / self.bytes if self.bytes else None
                ),
            }
        )
        return metrics


def _rgb_gstreamer_command(
    output_path: Path,
    preview_fifo: Path | None,
    *,
    preview_rtp_host: str | None = None,
    preview_rtp_port: int = 5004,
) -> list[str]:
    if preview_fifo is not None and preview_rtp_host is not None:
        raise ValueError("MJPEG and RTP preview outputs are mutually exclusive")
    command = [
        "gst-launch-1.0",
        "-e",
        "fdsrc",
        "fd=0",
        "!",
        "rawvideoparse",
        "format=yuy2",
        f"width={WIDTH}",
        f"height={HEIGHT}",
        f"framerate={RGB_FPS}/1",
        "!",
        "tee",
        "name=rgbtee",
        "rgbtee.",
        "!",
        "queue",
        "max-size-buffers=6",
        "max-size-bytes=0",
        "max-size-time=0",
        "!",
        "videoconvert",
        "!",
        "video/x-raw,format=NV12",
        "!",
        "mpph265enc",
        f"bps={RGB_BITRATE}",
        f"gop={RGB_FPS}",
        "header-mode=each-idr",
        "!",
        "filesink",
        "sync=false",
        f"location={output_path}",
    ]
    if preview_fifo is not None:
        command.extend(
            [
                "rgbtee.",
                "!",
                "queue",
                "leaky=downstream",
                "max-size-buffers=1",
                "!",
                "videorate",
                "drop-only=true",
                "!",
                "video/x-raw,framerate=15/1",
                "!",
                "videoscale",
                "!",
                "video/x-raw,width=640,height=360",
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=I420",
                "!",
                "jpegenc",
                "quality=75",
                "!",
                "filesink",
                "sync=false",
                f"location={preview_fifo}",
            ]
        )
    elif preview_rtp_host is not None:
        command.extend(
            [
                "rgbtee.",
                "!",
                "queue",
                "leaky=downstream",
                "max-size-buffers=2",
                "max-size-bytes=0",
                "max-size-time=0",
                "!",
                "videorate",
                "drop-only=true",
                "!",
                "video/x-raw,framerate=15/1",
                "!",
                "videoscale",
                "!",
                "video/x-raw,width=960,height=540",
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=NV12",
                "!",
                "mpph264enc",
                "bps=2000000",
                "gop=30",
                "header-mode=each-idr",
                "!",
                "video/x-h264,stream-format=byte-stream,alignment=au",
                "!",
                "rtph264pay",
                "pt=96",
                "config-interval=1",
                "mtu=1200",
                "!",
                "udpsink",
                "sync=false",
                "async=false",
                f"host={preview_rtp_host}",
                f"port={preview_rtp_port}",
            ]
        )
    return command


class _RgbEncoder:
    """Feed exact RSUSB YUYV frames to GStreamer without blocking capture."""

    def __init__(
        self,
        output_path: Path,
        log_path: Path,
        *,
        preview_fifo: Path | None,
        preview_rtp_host: str | None = None,
        preview_rtp_port: int = 5004,
        queue_frames: int = CAPTURE_QUEUE_FRAMES,
    ) -> None:
        self.output_path = output_path
        self.log_path = log_path
        self.command = _rgb_gstreamer_command(
            output_path,
            preview_fifo,
            preview_rtp_host=preview_rtp_host,
            preview_rtp_port=preview_rtp_port,
        )
        self.queue_frames = queue_frames
        self._queue: queue.Queue[bytes | None] = queue.Queue(queue_frames)
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._process: subprocess.Popen[bytes] | None = None
        self._log = None
        self._failure: BaseException | None = None
        self._started = False
        self._cancel_requested = threading.Event()
        self._queue_depth_lock = threading.Lock()
        self._queued_frames = 0
        self.frames = 0
        self.bytes = 0
        self.queue_overflows = 0
        self.maximum_queue_depth_frames = 0

    def start(self) -> None:
        if self._started:
            raise RuntimeError("RGB encoder is already started")
        try:
            self._log = self.log_path.open("xb")
            self._process = subprocess.Popen(
                self.command,
                stdin=subprocess.PIPE,
                stdout=self._log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._thread.start()
            self._started = True
        except BaseException:
            if self._process is not None and self._process.poll() is None:
                self._process.kill()
                self._process.wait(timeout=5.0)
            if self._log is not None and not self._log.closed:
                self._log.close()
            raise

    def submit(self, frame: bytes) -> None:
        if len(frame) != PAYLOAD_BYTES["color"]:
            raise CaptureError(
                f"RGB payload size is {len(frame)}, expected {PAYLOAD_BYTES['color']}"
            )
        if not self._started or self._failure is not None:
            raise CaptureError(f"RGB encoder is unavailable: {self._failure}")
        try:
            with self._queue_depth_lock:
                self._queue.put_nowait(frame)
                self._queued_frames += 1
                self.maximum_queue_depth_frames = max(
                    self.maximum_queue_depth_frames, self._queued_frames
                )
        except queue.Full as exc:
            self.queue_overflows += 1
            raise CaptureError("RGB encoder queue overflow") from exc

    def _write_loop(self) -> None:
        try:
            assert self._process is not None and self._process.stdin is not None
            while True:
                frame = self._queue.get()
                try:
                    if frame is not None:
                        with self._queue_depth_lock:
                            self._queued_frames -= 1
                    if frame is None:
                        break
                    self._process.stdin.write(frame)
                    self._process.stdin.flush()
                    self.frames += 1
                    self.bytes += len(frame)
                finally:
                    self._queue.task_done()
            self._process.stdin.close()
        except BaseException as exc:
            self._failure = exc

    def finish(self) -> None:
        if not self._started:
            return
        while self._thread.is_alive():
            try:
                self._queue.put(None, timeout=0.1)
                break
            except queue.Full:
                continue
        self._thread.join(timeout=30.0)
        if self._thread.is_alive():
            self.cancel()
            raise CaptureError("RGB encoder input writer did not stop")
        assert self._process is not None
        try:
            returncode = self._process.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            self.cancel()
            raise CaptureError("RGB encoder did not finish after end of input")
        finally:
            if self._log is not None:
                self._log.flush()
                os.fsync(self._log.fileno())
                self._log.close()
        if self._failure is not None:
            raise CaptureError(f"RGB encoder input failed: {self._failure}")
        if returncode != 0:
            raise CaptureError(f"RGB encoder exited with status {returncode}")

    def cancel(self, *, timeout_s: float = 5.0) -> None:
        self._cancel_requested.set()
        if self._process is not None and self._process.poll() is None:
            self._process.send_signal(signal.SIGINT)
            try:
                self._process.wait(timeout=min(3.0, timeout_s))
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2.0)
        if self._started and self._thread.is_alive():
            deadline = time.monotonic() + timeout_s
            while self._thread.is_alive():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    self._queue.put(None, timeout=min(0.1, remaining))
                    break
                except queue.Full:
                    continue
            self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if self._log is not None and not self._log.closed:
            self._log.close()

    def metrics(self) -> dict[str, int]:
        return {
            "input_frames": self.frames,
            "input_bytes": self.bytes,
            "queue_capacity_frames": self.queue_frames,
            "maximum_queue_depth_frames": self.maximum_queue_depth_frames,
            "queue_overflows": self.queue_overflows,
        }


def _cancel_started_capture_children(
    encoder: _RgbEncoder,
    *,
    encoder_started: bool,
    started_ir_writers: list[tuple[str, _AsyncFrameWriter]],
) -> list[str]:
    """Attempt every child cancellation and return any cleanup failures."""
    workers: list[tuple[str, Any]] = []
    if encoder_started:
        workers.append(("rgb", encoder))
    workers.extend(started_ir_writers)
    failures = []
    deferred_abort: BaseException | None = None
    for name, worker in workers:
        try:
            worker.cancel()
        except Exception as exc:
            failures.append(f"{name}: {type(exc).__name__}: {exc}")
        except BaseException as exc:
            if deferred_abort is None:
                deferred_abort = exc
    if deferred_abort is not None:
        raise deferred_abort
    return failures


def _device_snapshot(rs, device) -> dict[str, str]:
    return {
        "sdk_serial": device.get_info(rs.camera_info.serial_number),
        "name": device.get_info(rs.camera_info.name),
        "firmware": device.get_info(rs.camera_info.firmware_version),
        "usb_type": device.get_info(rs.camera_info.usb_type_descriptor),
    }


def _publish_session(partial: Path, final: Path, output_root: Path) -> None:
    """Durably publish a session, rolling back to partial on a reported failure."""
    marker = partial / ".recording"
    _fsync_directory(partial)
    os.replace(partial, final)
    try:
        _fsync_directory(output_root)
        marker = final / ".recording"
        marker.unlink()
        _fsync_directory(final)
        _fsync_directory(output_root)
    except BaseException:
        try:
            if final.exists() and not partial.exists():
                marker = final / ".recording"
                if not marker.exists():
                    marker.write_text("unsealed\n", encoding="ascii")
                    _fsync_directory(final)
                os.replace(final, partial)
                _fsync_directory(output_root)
        except BaseException:
            pass
        raise


def capture_rsusb_session(
    *,
    output_root: Path,
    duration_s: int,
    d405_sdk_serial: str,
    d405_usb_serial: str,
    stm32_port: str | None = None,
    preview_listen: str | None = None,
    preview_port: int = 8765,
    preview_rtp_host: str | None = None,
    preview_rtp_port: int = 5004,
    ir_encoding: str = "y8_split_h265",
    stop_requested: threading.Event | None = None,
) -> Path:
    if duration_s < 1 or duration_s > 3600:
        raise CaptureError("duration must be between 1 and 3600 seconds")
    if ir_encoding not in IR_ENCODINGS:
        raise CaptureError(f"unsupported IR encoding: {ir_encoding}")
    if stop_requested is not None and ir_encoding not in {
        "y8_split_zstd",
        "y8_split_h265",
    }:
        raise CaptureError(
            "signal-controlled recording requires split-eye schema v3 or v4"
        )
    if preview_listen is not None and preview_rtp_host is not None:
        raise CaptureError("MJPEG and RTP preview outputs are mutually exclusive")
    if preview_rtp_host is not None:
        try:
            preview_rtp_host = str(ipaddress.ip_address(preview_rtp_host))
        except ValueError as exc:
            raise CaptureError("RTP preview host must be an IPv4 or IPv6 address") from exc
        if not 1 <= preview_rtp_port <= 65535:
            raise CaptureError("RTP preview port must be between 1 and 65535")
        if shutil.which("gst-inspect-1.0") is None:
            raise CaptureError("required executable is unavailable: gst-inspect-1.0")
        for element in ("mpph264enc", "rtph264pay", "udpsink"):
            available = subprocess.run(
                ["gst-inspect-1.0", element],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if available.returncode != 0:
                raise CaptureError(f"required GStreamer element is unavailable: {element}")
    if shutil.which("gst-launch-1.0") is None:
        raise CaptureError("required executable is unavailable: gst-launch-1.0")
    if ir_encoding in {"y8_split_zstd", "y8i_zstd"} and shutil.which("zstd") is None:
        raise CaptureError("required executable is unavailable: zstd")
    if ir_encoding == "y8_split_h265":
        if shutil.which("gst-inspect-1.0") is None:
            raise CaptureError("required executable is unavailable: gst-inspect-1.0")
        for element in ("rawvideoparse", "videoconvert", "mpph265enc", "filesink"):
            available = subprocess.run(
                ["gst-inspect-1.0", element],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if available.returncode != 0:
                raise CaptureError(f"required GStreamer element is unavailable: {element}")
    session_schema = {
        "y8_split_zstd": RSUSB_SPLIT_ZSTD_SCHEMA,
        "y8_split_h265": RSUSB_SPLIT_H265_SCHEMA,
        "y8i_zstd": RSUSB_ZSTD_SCHEMA,
        "y8i_raw": RSUSB_SCHEMA,
    }[ir_encoding]

    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise CaptureError("the isolated RSUSB pyrealsense2 module is unavailable") from exc

    usb = verify_d405_superspeed(d405_usb_serial)
    serial_path = discover_stm32_port(stm32_port)
    owners = _device_owners([serial_path])
    if owners:
        raise CaptureError(f"the STM32 serial device is already open: {owners}")
    context = rs.context()
    matches = [
        device
        for device in context.query_devices()
        if device.get_info(rs.camera_info.serial_number) == d405_sdk_serial
    ]
    if len(matches) != 1:
        raise CaptureError(
            f"expected one RSUSB D405 serial {d405_sdk_serial}, found {len(matches)}"
        )
    device = matches[0]
    snapshot = _device_snapshot(rs, device)
    if "D405" not in snapshot["name"].upper():
        raise CaptureError(f"RSUSB device is not a D405: {snapshot}")
    sensor = device.first_depth_sensor()
    profiles = _select_profiles(rs, sensor)

    maximum_frames = duration_s * FPS
    signal_controlled = stop_requested is not None
    preflight_seconds = min(duration_s, 30) if signal_controlled else duration_s
    if ir_encoding == "y8_split_h265":
        encoded_bits_per_second = (
            IR_H265_BITRATE_MAX_PER_EYE * 2 + RGB_BITRATE
        )
        estimated = encoded_bits_per_second // 8 * preflight_seconds * 2
    else:
        estimated = (
            IR_FRAME_BYTES * preflight_seconds * FPS
            + (RGB_BITRATE // 8) * preflight_seconds * 2
        )
    output_root = output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(output_root).free
    if free < estimated + MIN_FREE_RESERVE_BYTES:
        raise CaptureError(
            f"insufficient free space: free={free}, estimated={estimated}, "
            f"reserve={MIN_FREE_RESERVE_BYTES}"
        )
    thermal_before = _thermal_snapshot()
    if (
        thermal_before["maximum_millidegree_c"] is not None
        and thermal_before["maximum_millidegree_c"] >= 85_000
    ):
        raise CaptureError(f"host is too hot to start capture: {thermal_before}")

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_id = f"rk3576-rsusb-{stamp}-{uuid.uuid4().hex[:8]}"
    partial = output_root / f".{session_id}.partial"
    final = output_root / session_id
    partial.mkdir(mode=0o700)
    (partial / ".recording").write_text("unsealed\n", encoding="ascii")
    capture_since = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S")

    if ir_encoding in {"y8_split_zstd", "y8_split_h265"}:
        ir_names = {
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
        ir_partials = {
            side: partial / f"{name}.partial" for side, name in ir_names.items()
        }
        if ir_encoding == "y8_split_zstd":
            ir_writers = {
                side: _ZstdFrameWriter(
                    ir_partials[side],
                    frame_bytes=IR_EYE_FRAME_BYTES,
                    threads=SPLIT_ZSTD_THREADS_PER_EYE,
                    encoding="y8_zstd",
                )
                for side in ("left", "right")
            }
        else:
            ir_writers = {
                side: _H265FrameWriter(
                    ir_partials[side],
                    partial / f"ir-{side}-gstreamer.log",
                )
                for side in ("left", "right")
            }
    else:
        ir_name = (
            "infrared-y8i.raw.zst"
            if ir_encoding == "y8i_zstd"
            else "infrared-y8i.raw"
        )
        ir_names = {"stereo": ir_name}
        ir_partials = {"stereo": partial / f"{ir_name}.partial"}
        ir_writers = {
            "stereo": (
                _ZstdFrameWriter(
                    ir_partials["stereo"], frame_bytes=IR_FRAME_BYTES
                )
                if ir_encoding == "y8i_zstd"
                else _AsyncFrameWriter(
                    ir_partials["stereo"], frame_bytes=IR_FRAME_BYTES
                )
            )
        }
    rgb_partial = partial / "rgb.h265.partial"
    preview_fifo = partial / ".preview.mjpeg.pipe"
    encoder = _RgbEncoder(
        rgb_partial,
        partial / "rgb-gstreamer.log",
        preview_fifo=preview_fifo if preview_listen is not None else None,
        preview_rtp_host=preview_rtp_host,
        preview_rtp_port=preview_rtp_port,
    )
    serial = _SerialCollector(
        serial_path,
        partial / "stm32.bin.partial",
        partial / "stm32_packets.jsonl",
    )
    preview: MjpegPreviewServer | None = None
    ir_index = (partial / "ir_frames.jsonl").open("x", encoding="utf-8")
    rgb_index = (partial / "rgb_frames.jsonl").open("x", encoding="utf-8")
    frame_queue = rs.frame_queue(CAPTURE_QUEUE_FRAMES, keep_frames=False)
    continuity = {
        name: StreamContinuity(expected_payload_bytes=size)
        for name, size in PAYLOAD_BYTES.items()
    }
    pending_ir: dict[int, dict[str, tuple[bytes, float, int]]] = {}
    maximum_pending_ir = 0
    ir_rows: list[dict[str, Any]] = []
    rgb_rows: list[dict[str, Any]] = []
    warmup_counts = {name: 0 for name in PAYLOAD_BYTES}
    sensor_opened = sensor_started = serial_started = False
    started_ir_writers: list[tuple[str, _AsyncFrameWriter]] = []
    encoder_started = False
    formal_start_ns = formal_stop_ns = None

    config = {
        "schema": session_schema,
        "session_id": session_id,
        "capture_host": {
            "hostname": os.uname().nodename,
            "machine": os.uname().machine,
            "thermal_before": thermal_before,
        },
        "device": {
            "d405_serial": d405_sdk_serial,
            "d405_sdk_serial": d405_sdk_serial,
            "d405_usb_descriptor_serial": d405_usb_serial,
            "d405_usb": usb,
            "d405_rsusb": snapshot,
            "rsusb_module": _module_evidence(rs),
            "stm32_port": serial_path,
        },
        "profile": {
            "infrared": {
                "encoding": ir_encoding,
                "layout": (
                    "separate_left_right_files"
                    if ir_encoding in {"y8_split_zstd", "y8_split_h265"}
                    else "byte_interleaved_left_right"
                ),
                "width": WIDTH,
                "height": HEIGHT,
                "fps": FPS,
                "frame_bytes": IR_FRAME_BYTES,
                "frame_bytes_per_eye": IR_EYE_FRAME_BYTES,
                "files": ir_names,
                "semantics": (
                    "lossy_stereo_slam_candidate"
                    if ir_encoding == "y8_split_h265"
                    else "lossless_stereo_sensor_sample"
                ),
                "compression": (
                    {
                        "codec": "zstd",
                        "level": ZSTD_LEVEL,
                        "threads_per_stream": (
                            SPLIT_ZSTD_THREADS_PER_EYE
                            if ir_encoding == "y8_split_zstd"
                            else ZSTD_THREADS
                        ),
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
                            "target_bitrate_per_eye": IR_H265_BITRATE_PER_EYE,
                            "minimum_bitrate_per_eye": IR_H265_BITRATE_MIN_PER_EYE,
                            "maximum_bitrate_per_eye": IR_H265_BITRATE_MAX_PER_EYE,
                            "gop_frames": IR_H265_GOP_FRAMES,
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
                "source_encoding": "yuyv_rsusb",
                "width": WIDTH,
                "height": HEIGHT,
                "fps": RGB_FPS,
                "target_bitrate": RGB_BITRATE,
                "semantics": "lossy_preview_and_training_image",
            },
            "stm32": {
                "encoding": "stm32_combined_v1_raw",
                "packet_bytes": STM32_PACKET_BYTES,
                "baud": 921_600,
                "nominal_rate_hz": 400,
                "semantics": "lossless_wire_packet",
            },
        },
        "requested": (
            {
                "mode": "until_signal",
                "maximum_duration_s": duration_s,
            }
            if signal_controlled
            else {
                "mode": "fixed_frames",
                "duration_s": duration_s,
                "frames": maximum_frames,
            }
        ),
    }
    if preview_listen is not None:
        config["preview"] = {
            "transport": "http_mjpeg",
            "listen": preview_listen,
            "port": preview_port,
            "width": 640,
            "height": 360,
            "fps": 10,
        }
    elif preview_rtp_host is not None:
        config["preview"] = {
            "transport": "rtp_h264",
            "host": preview_rtp_host,
            "port": preview_rtp_port,
            "payload_type": 96,
            "codec": "H264",
            "width": 960,
            "height": 540,
            "fps": 15,
            "bitrate": 2_000_000,
            "semantics": "webrtc_ingress_candidate",
        }
    _write_atomic(partial / "session_config.json", config)

    primary_error: BaseException | None = None
    try:
        if preview_listen is not None:
            os.mkfifo(preview_fifo, mode=0o600)
            preview = MjpegPreviewServer(preview_fifo, preview_listen, preview_port)
            preview.start()
        serial.start()
        serial_started = True
        for side, writer in ir_writers.items():
            writer.start()
            started_ir_writers.append((side, writer))
        encoder.start()
        encoder_started = True
        sensor.open(profiles)
        sensor_opened = True
        sensor.start(frame_queue)
        sensor_started = True

        warmup_deadline = time.monotonic() + WARMUP_SECONDS
        while time.monotonic() < warmup_deadline:
            frame = frame_queue.wait_for_frame(1000)
            key = _stream_key(rs, frame)
            if key is not None:
                bytes(frame.get_data())
                warmup_counts[key] += 1
        while True:
            frame = frame_queue.poll_for_frame()
            if not frame:
                break
            key = _stream_key(rs, frame)
            if key is not None:
                bytes(frame.get_data())
                warmup_counts[key] += 1
        if any(count < FPS for count in warmup_counts.values()):
            raise CaptureError(f"RSUSB warmup was incomplete: {warmup_counts}")

        serial.begin()
        formal_start_ns = time.monotonic_ns()
        rgb_first_timestamp_ms: float | None = None
        bounded_deadline = time.monotonic() + duration_s + 15.0
        capture_target = None if signal_controlled else maximum_frames
        termination_reason = "fixed_duration_complete"
        last_storage_check_ns = formal_start_ns
        stop_observed_monotonic_ns: int | None = None
        stop_alignment_deadline: float | None = None
        storage_guard_evidence: dict[str, int] | None = None
        while True:
            if capture_target is None and stop_requested is not None and stop_requested.is_set():
                stop_observed_monotonic_ns = time.monotonic_ns()
                capture_target = max(
                    2, max(item.received for item in continuity.values())
                )
                termination_reason = "external_stop"
                stop_alignment_deadline = (
                    time.monotonic() + STOP_ALIGNMENT_TIMEOUT_SECONDS
                )
            if capture_target is None and all(
                item.received >= maximum_frames for item in continuity.values()
            ):
                capture_target = maximum_frames
                termination_reason = "maximum_duration_complete"
            if capture_target is not None and all(
                item.received >= capture_target for item in continuity.values()
            ):
                break
            if (
                stop_alignment_deadline is not None
                and time.monotonic() >= stop_alignment_deadline
            ):
                raise CaptureError(
                    "stream alignment did not complete within the stop deadline: "
                    f"target={capture_target}, "
                    f"received={ {name: item.received for name, item in continuity.items()} }"
                )
            if time.monotonic() >= bounded_deadline:
                raise CaptureError(
                    "RSUSB capture exceeded its bounded deadline: "
                    f"{ {name: item.received for name, item in continuity.items()} }"
                )
            frame = frame_queue.wait_for_frame(1000)
            key = _stream_key(rs, frame)
            stream_limit = capture_target if capture_target is not None else maximum_frames
            if key is None or continuity[key].received >= stream_limit:
                continue
            arrival_ns = time.monotonic_ns()
            sequence = int(frame.get_frame_number())
            timestamp_ms = float(frame.get_timestamp())
            payload = bytes(frame.get_data())
            continuity[key].observe(
                sequence=sequence,
                timestamp_ms=timestamp_ms,
                arrival_ns=arrival_ns,
                payload_bytes=len(payload),
            )
            if signal_controlled and arrival_ns - last_storage_check_ns >= 1_000_000_000:
                last_storage_check_ns = arrival_ns
                free_now = shutil.disk_usage(output_root).free
                if free_now <= MIN_FREE_RESERVE_BYTES + STORAGE_STOP_HEADROOM_BYTES:
                    stop_observed_monotonic_ns = arrival_ns
                    capture_target = max(
                        2, max(item.received for item in continuity.values())
                    )
                    termination_reason = "low_storage_guard"
                    stop_alignment_deadline = (
                        time.monotonic() + STOP_ALIGNMENT_TIMEOUT_SECONDS
                    )
                    storage_guard_evidence = {
                        "observed_free_bytes": free_now,
                        "threshold_bytes": (
                            MIN_FREE_RESERVE_BYTES + STORAGE_STOP_HEADROOM_BYTES
                        ),
                        "observed_monotonic_ns": arrival_ns,
                    }
            if key == "color":
                if rgb_first_timestamp_ms is None:
                    rgb_first_timestamp_ms = timestamp_ms
                encoder.submit(payload)
                row = {
                    "record_index": len(rgb_rows),
                    "sequence": sequence,
                    "bytes_used": len(payload),
                    "source_timestamp_ms": timestamp_ms,
                    "observed_host_monotonic_ns": arrival_ns,
                    "pipeline_pts_ns": round(
                        (timestamp_ms - rgb_first_timestamp_ms) * 1_000_000
                    ),
                }
                rgb_rows.append(row)
                rgb_index.write(_json_bytes(row).decode("utf-8"))
                continue

            side = "left" if key == "infrared_left" else "right"
            pair = pending_ir.setdefault(sequence, {})
            pair[side] = (payload, timestamp_ms, arrival_ns)
            if "left" in pair and "right" in pair:
                left, left_timestamp_ms, left_arrival_ns = pair["left"]
                right, right_timestamp_ms, right_arrival_ns = pair["right"]
                if ir_encoding in {"y8_split_zstd", "y8_split_h265"}:
                    ir_writers["left"].submit(left)
                    ir_writers["right"].submit(right)
                else:
                    ir_writers["stereo"].submit(
                        interleave_stereo_y8(left, right)
                    )
                row = {
                    "record_index": len(ir_rows),
                    "sequence": sequence,
                    "left_sequence": sequence,
                    "right_sequence": sequence,
                    "bytes_used": len(left) + len(right),
                    "source_monotonic_ns": round(left_timestamp_ms * 1_000_000),
                    "left_source_timestamp_ms": left_timestamp_ms,
                    "right_source_timestamp_ms": right_timestamp_ms,
                    "observed_host_monotonic_ns": max(
                        left_arrival_ns, right_arrival_ns
                    ),
                }
                ir_rows.append(row)
                ir_index.write(_json_bytes(row).decode("utf-8"))
                del pending_ir[sequence]
            maximum_pending_ir = max(maximum_pending_ir, len(pending_ir))
            if len(pending_ir) > PAIRING_WINDOW_FRAMES:
                raise CaptureError(
                    f"stereo IR pairing window exceeded: {sorted(pending_ir)[:20]}"
                )
        formal_stop_ns = time.monotonic_ns()

        serial.stop()
        serial_started = False
        sensor.stop()
        sensor_started = False
        sensor.close()
        sensor_opened = False
        encoder.finish()
        encoder_started = False
        for writer in ir_writers.values():
            writer.finish()
        started_ir_writers.clear()
        if preview is not None:
            preview.stop()
            preview_fifo.unlink(missing_ok=True)
        ir_index.flush()
        rgb_index.flush()
        os.fsync(ir_index.fileno())
        os.fsync(rgb_index.fileno())
        ir_index.close()
        rgb_index.close()

        stream_reports = {
            name: tracker.report() for name, tracker in continuity.items()
        }
        captured_frames = next(iter(continuity.values())).received
        decision = evaluate_streams(stream_reports, expected_frames=captured_frames)
        if decision["status"] != "PASS":
            raise CaptureError(f"RSUSB continuity failed: {decision['reasons']}")
        if pending_ir or len(ir_rows) != captured_frames:
            raise CaptureError(
                f"stereo IR pairing mismatch: pairs={len(ir_rows)}, pending={sorted(pending_ir)}"
            )
        stream_ir_metrics = {
            side: writer.metrics() for side, writer in ir_writers.items()
        }
        ir_metrics = {
            "frames": len(ir_rows),
            "input_bytes": sum(
                metrics["bytes"] for metrics in stream_ir_metrics.values()
            ),
            "compressed_bytes": sum(
                metrics.get("compressed_bytes", metrics["bytes"])
                for metrics in stream_ir_metrics.values()
            ),
            "queue_overflows": sum(
                metrics["queue_overflows"]
                for metrics in stream_ir_metrics.values()
            ),
            "maximum_queue_depth_frames": max(
                metrics["maximum_queue_depth_frames"]
                for metrics in stream_ir_metrics.values()
            ),
            "streams": stream_ir_metrics,
        }
        ir_metrics["compression_ratio"] = (
            ir_metrics["compressed_bytes"] / ir_metrics["input_bytes"]
            if ir_metrics["input_bytes"]
            else None
        )
        rgb_metrics = encoder.metrics()
        if (
            any(
                metrics["frames"] != captured_frames
                for metrics in stream_ir_metrics.values()
            )
            or ir_metrics["queue_overflows"] != 0
            or (
                ir_encoding == "y8i_raw"
                and ir_partials["stereo"].stat().st_size
                != captured_frames * IR_FRAME_BYTES
            )
            or any(path.stat().st_size == 0 for path in ir_partials.values())
        ):
            raise CaptureError(f"IR writer mismatch: {ir_metrics}")
        if ir_encoding in {"y8_split_zstd", "y8i_zstd"}:
            for side, path in ir_partials.items():
                integrity = subprocess.run(
                    ["zstd", "--test", "--quiet", str(path)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                if integrity.returncode != 0:
                    raise CaptureError(
                        "lossless IR zstd verification failed "
                        f"for {side}: {integrity.stdout.strip()}"
                    )
        elif ir_encoding == "y8_split_h265":
            for side, path in ir_partials.items():
                with path.open("rb") as stream:
                    prefix = stream.read(4)
                if not (
                    prefix.startswith(b"\x00\x00\x01")
                    or prefix == b"\x00\x00\x00\x01"
                ):
                    raise CaptureError(f"IR {side} output is not H.265 Annex-B")
        if (
            rgb_metrics["input_frames"] != captured_frames
            or rgb_metrics["queue_overflows"] != 0
            or rgb_partial.stat().st_size == 0
        ):
            raise CaptureError(f"RGB encoder mismatch: {rgb_metrics}")
        with rgb_partial.open("rb") as stream:
            prefix = stream.read(4)
        if not (prefix.startswith(b"\x00\x00\x01") or prefix == b"\x00\x00\x00\x01"):
            raise CaptureError("RGB output is not H.265 Annex-B")
        serial_metrics = serial.metrics()
        if not 395.0 <= serial_metrics["observed_rate_hz"] <= 405.0:
            raise CaptureError(f"STM32 rate is outside acceptance band: {serial_metrics}")
        for key in (
            "crc_errors",
            "discarded_bytes",
            "sequence_gaps",
            "sequence_regressions",
            "invalid_imu_flags",
            "invalid_encoder_flags",
        ):
            if serial_metrics[key]:
                raise CaptureError(f"STM32 integrity failure: {serial_metrics}")

        finalized_files = [
            (ir_partials[side], partial / name)
            for side, name in ir_names.items()
        ]
        finalized_files.extend(
            [
                (rgb_partial, partial / "rgb.h265"),
                (partial / "stm32.bin.partial", partial / "stm32.bin"),
            ]
        )
        for source, target in finalized_files:
            os.replace(source, target)
        kernel_events = _kernel_usb_events(capture_since)
        (partial / "kernel-usb.log").write_text(
            "\n".join(kernel_events) + ("\n" if kernel_events else ""),
            encoding="utf-8",
        )
        fatal_patterns = (
            "usb disconnect",
            "reset superspeed usb device",
            "device descriptor read",
            "unable to submit",
            "failed to submit",
            "i/o error",
            "aborting journal",
        )
        fatal_events = [
            line
            for line in kernel_events
            if any(pattern in line.lower() for pattern in fatal_patterns)
        ]
        if fatal_events:
            raise CaptureError(f"fatal USB/storage kernel events: {fatal_events}")
        thermal_after = _thermal_snapshot()
        if (
            thermal_after["maximum_millidegree_c"] is not None
            and thermal_after["maximum_millidegree_c"] >= 85_000
        ):
            raise CaptureError(f"host exceeded the thermal limit: {thermal_after}")
        preview_metrics = None
        if preview is not None:
            preview_metrics = preview.metrics()
            if preview_metrics["reader_failure"] is not None:
                raise CaptureError(f"preview reader failed: {preview_metrics}")
            if preview_metrics["published_frames"] == 0:
                raise CaptureError("preview produced no frames")
        metrics = {
            "rsusb_streams": stream_reports,
            "formal_start_monotonic_ns": formal_start_ns,
            "formal_stop_monotonic_ns": formal_stop_ns,
            "formal_host_span_s": (formal_stop_ns - formal_start_ns) / 1e9,
            "termination": {
                "mode": "until_signal" if signal_controlled else "fixed_frames",
                "reason": termination_reason,
                "captured_frames": captured_frames,
                "stop_observed_monotonic_ns": stop_observed_monotonic_ns,
                "storage_guard": storage_guard_evidence,
            },
            "warmup_counts": warmup_counts,
            "ir": {
                "frames": len(ir_rows),
                "maximum_pending_pairs": maximum_pending_ir,
                "writer": ir_metrics,
            },
            "rgb": {
                **rgb_metrics,
                "encoded_bytes": (partial / "rgb.h265").stat().st_size,
                "pts_regressions": 0,
                "timestamp_semantics": "realsense_device_relative",
            },
            "stm32": serial_metrics,
            "thermal_before": thermal_before,
            "thermal_after": thermal_after,
            "kernel_event_count": len(kernel_events),
        }
        if preview_metrics is not None:
            metrics["preview"] = preview_metrics
        manifest = {
            "schema": session_schema,
            "status": "SEALED",
            "session_id": session_id,
            "warnings": [],
            "profile": config["profile"],
            "device": config["device"],
            "counts": {
                "ir_frames": len(ir_rows),
                "rgb_input_frames": len(rgb_rows),
                "stm32_packets": serial.count,
            },
            "metrics": metrics,
            "files": _file_manifest(partial),
        }
        _write_atomic(partial / "manifest.json", manifest)
        _publish_session(partial, final, output_root)
        return final
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        if sensor_started:
            try:
                sensor.stop()
            except BaseException:
                pass
        if sensor_opened:
            try:
                sensor.close()
            except BaseException:
                pass
        cleanup_failures = _cancel_started_capture_children(
            encoder,
            encoder_started=encoder_started,
            started_ir_writers=started_ir_writers,
        )
        if serial_started:
            serial.stop_requested.set()
            serial.thread.join(timeout=3.0)
        if preview is not None:
            preview.stop()
        preview_fifo.unlink(missing_ok=True)
        for stream in (ir_index, rgb_index):
            if not stream.closed:
                stream.close()
        if primary_error is not None and partial.exists():
            try:
                _write_atomic(
                    partial / "failure.json",
                    {
                        "schema": session_schema,
                        "status": "FAILED",
                        "reason": str(primary_error),
                        "diagnostics": {
                            "ir_rows": len(ir_rows),
                            "rgb_rows": len(rgb_rows),
                            "ir_writers": {
                                side: writer.metrics()
                                for side, writer in ir_writers.items()
                            },
                            "rgb_encoder": encoder.metrics(),
                            "cleanup_failures": cleanup_failures,
                            "continuity": {
                                name: tracker.report()
                                for name, tracker in continuity.items()
                            },
                        },
                    },
                )
            except OSError:
                pass


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--duration",
        type=int,
        default=10,
        help="fixed duration, or maximum runtime when --until-signal is used",
    )
    parser.add_argument(
        "--until-signal",
        action="store_true",
        help="record until SIGINT/SIGTERM, then align all streams and seal",
    )
    parser.add_argument("--d405-sdk-serial", required=True)
    parser.add_argument("--d405-usb-serial", required=True)
    parser.add_argument("--stm32-port")
    parser.add_argument("--preview-listen")
    parser.add_argument("--preview-port", type=int, default=8765)
    parser.add_argument("--preview-rtp-host")
    parser.add_argument("--preview-rtp-port", type=int, default=5004)
    parser.add_argument(
        "--ir-encoding",
        choices=IR_ENCODINGS,
        default="y8_split_h265",
        help=(
            "dual-IR storage encoding; y8_split_h265 is the compact EGO-style "
            "default; y8_split_zstd retains the lossless rollback"
        ),
    )
    args = parser.parse_args(argv)
    stop_requested = threading.Event() if args.until_signal else None
    previous_handlers: dict[int, Any] = {}
    if stop_requested is not None:
        def request_stop(signum, _frame) -> None:
            if stop_requested.is_set():
                signal.signal(signum, signal.SIG_DFL)
                raise KeyboardInterrupt(
                    "capture force-aborted after a repeated termination signal"
                )
            stop_requested.set()

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)
    try:
        session = capture_rsusb_session(
            output_root=args.output_root,
            duration_s=args.duration,
            d405_sdk_serial=args.d405_sdk_serial,
            d405_usb_serial=args.d405_usb_serial,
            stm32_port=args.stm32_port,
            preview_listen=args.preview_listen,
            preview_port=args.preview_port,
            preview_rtp_host=args.preview_rtp_host,
            preview_rtp_port=args.preview_rtp_port,
            ir_encoding=args.ir_encoding,
            stop_requested=stop_requested,
        )
    except CaptureError as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    except KeyboardInterrupt as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 130
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    print(
        json.dumps(
            {"status": "SEALED", "session": str(session)}, ensure_ascii=False
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
