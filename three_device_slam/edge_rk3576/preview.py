"""Small dependency-free HTTP MJPEG preview server for the RK3576 collector."""

from __future__ import annotations

import json
import os
import select
import socket
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit


def extract_complete_jpegs(buffer: bytearray) -> list[bytes]:
    """Remove and return complete JPEG images from an arbitrary byte stream."""
    images: list[bytes] = []
    while True:
        start = buffer.find(b"\xff\xd8")
        if start < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            break
        if start:
            del buffer[:start]
        end = buffer.find(b"\xff\xd9", 2)
        if end < 0:
            break
        images.append(bytes(buffer[: end + 2]))
        del buffer[: end + 2]
    return images


class _PreviewState:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.running = True
        self.latest: bytes | None = None
        self.frames: deque[tuple[int, bytes]] = deque(maxlen=16)
        self.sequence = 0
        self.first_publish_ns: int | None = None
        self.last_publish_ns: int | None = None
        self.client_connections = 0
        self.active_clients = 0
        self.delivered_frames = 0
        self.skipped_frames = 0
        self.maximum_write_ms = 0.0

    def publish(self, image: bytes) -> None:
        now = time.monotonic_ns()
        with self.condition:
            self.latest = image
            self.sequence += 1
            self.frames.append((self.sequence, image))
            if self.first_publish_ns is None:
                self.first_publish_ns = now
            self.last_publish_ns = now
            self.condition.notify_all()

    def wait_after(self, sequence: int, timeout: float) -> tuple[int, bytes] | None:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.running and not any(item[0] > sequence for item in self.frames):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
            return next((item for item in self.frames if item[0] > sequence), None)

    def current_sequence(self) -> int:
        with self.condition:
            return self.sequence

    def wait_latest(self, timeout: float) -> bytes | None:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.running and self.latest is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
            return self.latest

    def stop(self) -> None:
        with self.condition:
            self.running = False
            self.condition.notify_all()

    def metrics(self) -> dict[str, Any]:
        with self.condition:
            span_ns = (
                self.last_publish_ns - self.first_publish_ns
                if self.first_publish_ns is not None
                and self.last_publish_ns is not None
                and self.last_publish_ns > self.first_publish_ns
                else 0
            )
            rate = (self.sequence - 1) * 1_000_000_000 / span_ns if span_ns else 0.0
            age_ms = (
                (time.monotonic_ns() - self.last_publish_ns) / 1_000_000
                if self.last_publish_ns is not None
                else None
            )
            return {
                "published_frames": self.sequence,
                "observed_rate_hz": rate,
                "latest_frame_age_ms": age_ms,
                "first_publish_monotonic_ns": self.first_publish_ns,
                "last_publish_monotonic_ns": self.last_publish_ns,
                "client_connections": self.client_connections,
                "active_clients": self.active_clients,
                "delivered_frames": self.delivered_frames,
                "skipped_frames": self.skipped_frames,
                "maximum_write_ms": self.maximum_write_ms,
            }


class _ThreadingServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], state: _PreviewState) -> None:
        self.preview_state = state
        super().__init__(address, _PreviewHandler)


class _PreviewHandler(BaseHTTPRequestHandler):
    server: _ThreadingServer

    def log_message(self, format: str, *args: object) -> None:
        return

    def _json(self, value: dict[str, Any]) -> None:
        payload = (json.dumps(value, sort_keys=True) + "\n").encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
        path = urlsplit(self.path).path
        state = self.server.preview_state
        if path == "/healthz":
            self._json({"status": "RUNNING" if state.running else "STOPPED", **state.metrics()})
            return
        if path == "/snapshot.jpg":
            image = state.wait_latest(3.0)
            if image is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "no preview frame")
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(image)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(image)
            return
        if path != "/stream.mjpg":
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        with state.condition:
            state.client_connections += 1
            state.active_clients += 1
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = state.current_sequence()
            while state.running:
                item = state.wait_after(sequence, 1.0)
                if item is None:
                    continue
                next_sequence, image = item
                write_started = time.monotonic_ns()
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(image)).encode("ascii")
                    + b"\r\n\r\n"
                    + image
                    + b"\r\n"
                )
                self.wfile.flush()
                write_ms = (time.monotonic_ns() - write_started) / 1_000_000
                with state.condition:
                    state.delivered_frames += 1
                    state.skipped_frames += max(0, next_sequence - sequence - 1)
                    state.maximum_write_ms = max(state.maximum_write_ms, write_ms)
                sequence = next_sequence
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            with state.condition:
                state.active_clients -= 1


class MjpegPreviewServer:
    """Drain JPEGs from a FIFO and expose the latest frames over HTTP."""

    def __init__(self, fifo: Path, listen: str, port: int) -> None:
        self.fifo = fifo
        self.state = _PreviewState()
        self.server = _ThreadingServer((listen, port), self.state)
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.http = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.failure: BaseException | None = None
        self._stop_requested = threading.Event()
        self._stop_lock = threading.Lock()
        self._stopped = False

    @property
    def port(self) -> int:
        return int(self.server.server_address[1])

    def start(self) -> None:
        self.reader.start()
        self.http.start()

    def _read_loop(self) -> None:
        descriptor: int | None = None
        buffer = bytearray()
        try:
            descriptor = os.open(self.fifo, os.O_RDONLY | os.O_NONBLOCK)
            while not self._stop_requested.is_set():
                readable, _, _ = select.select([descriptor], [], [], 0.1)
                if not readable:
                    continue
                chunk = os.read(descriptor, 256 * 1024)
                if not chunk:
                    time.sleep(0.01)
                    continue
                buffer.extend(chunk)
                for image in extract_complete_jpegs(buffer):
                    self.state.publish(image)
                if len(buffer) > 4 * 1024 * 1024:
                    raise RuntimeError("preview JPEG stream exceeded parser buffer limit")
        except BaseException as exc:
            self.failure = exc
        finally:
            if descriptor is not None:
                os.close(descriptor)

    def stop(self) -> None:
        with self._stop_lock:
            if self._stopped:
                return
            self._stopped = True
            self._stop_requested.set()
            self.state.stop()
            self.server.shutdown()
            self.server.server_close()
            self.reader.join(timeout=2.0)
            self.http.join(timeout=2.0)

    def metrics(self) -> dict[str, Any]:
        result = self.state.metrics()
        result["reader_failure"] = None if self.failure is None else str(self.failure)
        result["listen_port"] = self.port
        return result
