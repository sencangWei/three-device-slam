import hashlib
import json
import os
import re
from dataclasses import asdict
from pathlib import Path
from typing import BinaryIO
from zlib import crc32

from .model import SensorRecord
from .session_lifecycle import assert_producer_writes_allowed


_SAFE_STREAM_ID = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9._-]*[A-Za-z0-9])?$")
_RESERVED_STREAM_IDS = {"CON", "PRN", "AUX", "NUL"} | {
    f"{prefix}{number}"
    for prefix in ("COM", "LPT")
    for number in range(1, 10)
}


class AppendOnlySessionWriter:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        assert_producer_writes_allowed(self.root)
        self._claim_path = self.root / ".writer.lock"
        self._claim_owned = False
        try:
            descriptor = os.open(
                self._claim_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644
            )
        except FileExistsError as error:
            raise RuntimeError("session already has an active writer") from error
        self._claim_owned = True
        try:
            os.close(descriptor)
            descriptor = None
            assert_producer_writes_allowed(self.root)
            if (self.root / "manifest.json").exists():
                raise RuntimeError("session is sealed")
            self._streams: dict[str, tuple[BinaryIO, BinaryIO]] = {}
            self._manifest: dict | None = None
            self._state = "active"
        except BaseException:
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
            self._release_claim_best_effort()
            raise

    def append(
        self, record: SensorRecord, payload: bytes | bytearray | memoryview
    ) -> str:
        if self._state == "failed":
            raise RuntimeError("writer is terminally failed")
        if self._state != "active":
            raise RuntimeError("writer is closed")

        self._validate_stream_id(record.stream_id)
        payload_bytes = bytes(payload)
        row = asdict(record)
        row.update(
            offset=self._offset(record.stream_id),
            size=len(payload_bytes),
            crc32=crc32(payload_bytes) & 0xFFFFFFFF,
        )
        index_bytes = (json.dumps(row, sort_keys=True) + "\n").encode()
        stream_files = self._streams.get(record.stream_id)
        if stream_files is None:
            stream_files = (
                (self.root / f"{record.stream_id}.bin").open("ab"),
                (self.root / f"{record.stream_id}.jsonl").open("ab"),
            )
            self._streams[record.stream_id] = stream_files
        payload_file, index_file = stream_files
        offset = payload_file.tell()
        payload_file.write(payload_bytes)
        index_file.write(index_bytes)
        return f"{record.stream_id}.bin:{offset}:{len(payload_bytes)}"

    def close(self) -> dict:
        if self._state == "sealed":
            return self._manifest
        if self._state == "failed":
            raise RuntimeError("writer is terminally failed")

        try:
            streams = {}
            for stream_id, (payload_file, index_file) in self._streams.items():
                for file in (payload_file, index_file):
                    file.flush()
                    os.fsync(file.fileno())
                    file.close()
                streams[stream_id] = {
                    "payload_sha256": self._sha256(self.root / f"{stream_id}.bin"),
                    "index_sha256": self._sha256(self.root / f"{stream_id}.jsonl"),
                }
            manifest = {
                "schema": "ego.three_device.raw_session.v1",
                "streams": streams,
            }
            temporary = self.root / "manifest.json.tmp"
            with temporary.open("w", encoding="utf-8") as file:
                json.dump(manifest, file, sort_keys=True)
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary, self.root / "manifest.json")
            directory = os.open(self.root, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
            self._release_claim()
        except BaseException:
            self._state = "failed"
            try:
                self._close_streams_best_effort()
            except BaseException:
                pass
            try:
                self._release_claim_best_effort()
            except BaseException:
                pass
            raise
        self._manifest = manifest
        self._state = "sealed"
        return self._manifest

    def _close_streams_best_effort(self) -> None:
        for stream_files in self._streams.values():
            for file in stream_files:
                if file.closed:
                    continue
                try:
                    file.close()
                except BaseException:
                    pass

    def _release_claim(self) -> None:
        if not self._claim_owned:
            return
        self._claim_path.unlink()
        self._claim_owned = False

    def _release_claim_best_effort(self) -> None:
        try:
            self._release_claim()
        except BaseException:
            pass

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as file:
            for block in iter(lambda: file.read(65536), b""):
                digest.update(block)
        return digest.hexdigest()

    def _offset(self, stream_id: str) -> int:
        stream_files = self._streams.get(stream_id)
        if stream_files is None:
            return (self.root / f"{stream_id}.bin").stat().st_size if (
                self.root / f"{stream_id}.bin"
            ).exists() else 0
        return stream_files[0].tell()

    @staticmethod
    def _validate_stream_id(stream_id: str) -> None:
        if (
            not _SAFE_STREAM_ID.fullmatch(stream_id)
            or ".." in stream_id
            or stream_id.split(".", 1)[0].upper() in _RESERVED_STREAM_IDS
        ):
            raise ValueError("stream_id must be a safe filename token")
