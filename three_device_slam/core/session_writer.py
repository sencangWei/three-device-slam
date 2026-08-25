import hashlib
import json
import os
import re
from dataclasses import asdict
from pathlib import Path
from typing import BinaryIO
from zlib import crc32

from .model import SensorRecord


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
        self._streams: dict[str, tuple[BinaryIO, BinaryIO]] = {}
        self._manifest: dict | None = None

    def append(
        self, record: SensorRecord, payload: bytes | bytearray | memoryview
    ) -> str:
        if self._manifest is not None:
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
        if self._manifest is not None:
            return self._manifest

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
        self._manifest = {
            "schema": "ego.three_device.raw_session.v1",
            "streams": streams,
        }
        temporary = self.root / "manifest.json.tmp"
        with temporary.open("w", encoding="utf-8") as file:
            json.dump(self._manifest, file, sort_keys=True)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, self.root / "manifest.json")
        directory = os.open(self.root, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        return self._manifest

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
