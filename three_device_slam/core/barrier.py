import json
import os
import uuid
from pathlib import Path
from typing import Dict, Optional

from .model import DeviceHeartbeat


class BarrierDirectory:
    """Filesystem-backed coordination records for multi-device acquisition."""

    def __init__(self, session_directory: Path):
        self.path = Path(session_directory) / "barrier"
        self.path.mkdir(parents=True, exist_ok=True)

    def write_heartbeat(self, heartbeat: DeviceHeartbeat) -> None:
        device_id = self._device_id(heartbeat.device_id)
        payload = self._serialize(
            {
                "device_id": device_id,
                "at_ns": self._timestamp(heartbeat.at_ns),
                "healthy": heartbeat.healthy,
                "reasons": list(heartbeat.reasons),
            }
        )
        self._write_atomic(self.path / f"heartbeat.{device_id}.json", payload)

    def read_heartbeats(self) -> Dict[str, DeviceHeartbeat]:
        heartbeats = {}
        for file_path in self.path.glob("heartbeat.*.json"):
            payload = self._read_json(file_path)
            heartbeats[payload["device_id"]] = DeviceHeartbeat(
                payload["device_id"],
                payload["at_ns"],
                payload["healthy"],
                tuple(payload.get("reasons", ())),
            )
        return heartbeats

    def schedule_start(self, start_ns: int) -> None:
        payload = self._serialize({"start_ns": self._timestamp(start_ns)})
        lock_path = self.path / "start.lock"
        self._create_lock(lock_path)
        self._write_atomic(self.path / "start.json", payload)

    def read_start_ns(self) -> Optional[int]:
        start_path = self.path / "start.json"
        if not start_path.exists():
            return None
        return self._read_json(start_path)["start_ns"]

    def request_stop(self, at_ns: int, reason: str) -> None:
        if isinstance(at_ns, bool) or not isinstance(at_ns, int) or at_ns < 0:
            raise ValueError("at_ns must be a nonnegative integer")
        if not isinstance(reason, str) or not reason:
            raise ValueError("reason must be a nonempty string")
        claim = self.path / "stop.lock"
        try:
            descriptor = os.open(claim, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            return
        os.close(descriptor)
        payload = self._serialize({"at_ns": at_ns, "reason": reason})
        self._write_atomic(self.path / "stop.json", payload)

    def read_stop(self) -> dict | None:
        path = self.path / "stop.json"
        if not path.exists():
            return None
        payload = json.loads(path.read_text(encoding="utf-8"))
        return {"at_ns": int(payload["at_ns"]), "reason": str(payload["reason"])}

    def latch_failure(self, device_id: str, reason: str, at_ns: int) -> None:
        device_id = self._device_id(device_id)
        payload = self._serialize(
            {
                "device_id": device_id,
                "reason": self._reason(reason),
                "at_ns": self._timestamp(at_ns),
            }
        )
        lock_path = self.path / f"failure.{device_id}.lock"
        try:
            self._create_lock(lock_path)
        except FileExistsError:
            return
        try:
            self._write_atomic(self.path / f"failure.{device_id}.json", payload)
        except BaseException:
            lock_path.unlink(missing_ok=True)
            self._fsync_directory()
            raise

    def read_failures(self) -> Dict[str, dict]:
        failures = {}
        for file_path in self.path.glob("failure.*.json"):
            payload = self._read_json(file_path)
            failures[payload["device_id"]] = payload
        return failures

    def _create_lock(self, lock_path: Path) -> None:
        descriptor = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        self._fsync_directory()

    def _write_atomic(self, final_path: Path, payload: str) -> None:
        temporary_path = final_path.with_name(f".{final_path.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary_path.open("x", encoding="utf-8") as file_handle:
                file_handle.write(payload)
                file_handle.flush()
                os.fsync(file_handle.fileno())
            temporary_path.replace(final_path)
            self._fsync_directory()
        finally:
            if temporary_path.exists():
                temporary_path.unlink()

    def _fsync_directory(self) -> None:
        descriptor = os.open(self.path, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @staticmethod
    def _read_json(file_path: Path) -> dict:
        with file_path.open(encoding="utf-8") as file_handle:
            return json.load(file_handle)

    @staticmethod
    def _timestamp(value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("timestamps must be non-negative integer nanoseconds")
        return value

    @staticmethod
    def _device_id(value: str) -> str:
        if value not in {"ego", "left", "right"}:
            raise ValueError("device_id must be ego, left, or right")
        return value

    @staticmethod
    def _reason(value: str) -> str:
        if not isinstance(value, str) or not value:
            raise ValueError("failure reason must be a non-empty string")
        return value

    @staticmethod
    def _serialize(payload: dict) -> str:
        return json.dumps(payload, separators=(",", ":"))
