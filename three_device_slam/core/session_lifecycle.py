"""Cross-platform cooperative claims for immutable session finalization."""

from __future__ import annotations

import os
import re
import stat
from contextlib import contextmanager
from pathlib import Path


_SAFE_ID = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9_-]*[A-Za-z0-9])?$")
_REPARSE_POINT = 0x400
OFFLINE_CLAIM_NAME = ".offline.lock"
SEAL_NAME = "session.seal.json"


def canonical_session_root(path: Path, *, create: bool = False) -> Path:
    absolute = Path(os.path.abspath(Path(path)))
    if create and not absolute.exists():
        absolute.mkdir(parents=True)
    info = _safe_lstat(absolute, "session root")
    if not stat.S_ISDIR(info.st_mode):
        raise ValueError("session root must be a directory")
    resolved = absolute.resolve(strict=True)
    if resolved != absolute:
        raise ValueError("session root must be canonical and contain no symlink")
    return resolved


def assert_safe_path(
    path: Path,
    session_root: Path,
    label: str,
    *,
    kind: str | None = None,
) -> os.stat_result | None:
    path = Path(path)
    root = Path(session_root)
    try:
        info = path.lstat()
    except FileNotFoundError:
        return None
    _reject_link_or_reparse(info, label)
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"unsafe {label}: path escapes session root")
    cursor = root
    for part in relative.parts:
        cursor = cursor / part
        _reject_link_or_reparse(cursor.lstat(), label)
    resolved = path.resolve(strict=True)
    if not resolved.is_relative_to(root):
        raise ValueError(f"unsafe {label}: path escapes session root")
    if kind == "file" and not stat.S_ISREG(info.st_mode):
        raise ValueError(f"unsafe {label}: expected regular file")
    if kind == "directory" and not stat.S_ISDIR(info.st_mode):
        raise ValueError(f"unsafe {label}: expected directory")
    return info


def assert_producer_writes_allowed(session_directory: Path) -> None:
    directory = Path(session_directory)
    session_root = directory.parent if directory.name in {"ego", "left", "right"} else directory
    if not session_root.exists():
        return
    root = canonical_session_root(session_root)
    for name, reason in (
        (SEAL_NAME, "session is sealed"),
        (OFFLINE_CLAIM_NAME, "session has an offline operation in progress"),
    ):
        marker = root / name
        info = assert_safe_path(marker, root, name, kind="file")
        if info is not None:
            raise RuntimeError(reason)


@contextmanager
def producer_claim(session: Path, producer_id: str):
    if not isinstance(producer_id, str) or not _SAFE_ID.fullmatch(producer_id):
        raise ValueError("producer_id must be a safe token")
    root = canonical_session_root(session, create=True)
    assert_producer_writes_allowed(root)
    claim_path = root / f".producer.{producer_id}.lock"
    identity = _create_claim(claim_path, root, f"producer {producer_id}")
    try:
        _claim_created_hook("producer", root)
        try:
            assert_producer_writes_allowed(root)
        except BaseException:
            _release_claim(claim_path, root, identity)
            identity = None
            raise
        yield root
    finally:
        if identity is not None:
            _release_claim(claim_path, root, identity)


@contextmanager
def offline_claim(session: Path):
    root = canonical_session_root(session)
    claim_path = root / OFFLINE_CLAIM_NAME
    identity = _create_claim(claim_path, root, "offline publication")
    try:
        _claim_created_hook("offline", root)
        producers = _producer_claims(root)
        if producers:
            raise RuntimeError(
                "session has an active producer: "
                + ", ".join(path.name for path in producers)
            )
        yield root
    except BaseException:
        _release_claim(claim_path, root, identity)
        identity = None
        raise
    finally:
        if identity is not None:
            _release_claim(claim_path, root, identity)


def _create_claim(
    path: Path, root: Path, label: str
) -> tuple[int, int, int, int, int]:
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags, 0o644)
    except FileExistsError as exc:
        info = assert_safe_path(path, root, path.name, kind="file")
        if info is None:
            raise RuntimeError(f"{label} claim raced") from exc
        if path.name == OFFLINE_CLAIM_NAME:
            raise RuntimeError("offline publication already in progress") from exc
        raise RuntimeError(f"{label} already active") from exc
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or _is_reparse(info):
            raise ValueError(f"unsafe {label} claim")
        os.fsync(descriptor)
        identity = _identity(info)
    finally:
        os.close(descriptor)
    _fsync_directory(root)
    return identity


def _release_claim(
    path: Path, root: Path, expected_identity: tuple[int, int, int, int, int]
) -> None:
    info = assert_safe_path(path, root, path.name, kind="file")
    if info is None or _identity(info) != expected_identity:
        raise RuntimeError(f"owned claim changed before release: {path.name}")
    path.unlink()
    _fsync_directory(root)


def _producer_claims(root: Path) -> list[Path]:
    claims = []
    for path in root.iterdir():
        if path.name.startswith(".producer.") and path.name.endswith(".lock"):
            assert_safe_path(path, root, path.name, kind="file")
            claims.append(path)
    return sorted(claims)


def _safe_lstat(path: Path, label: str) -> os.stat_result:
    try:
        info = path.lstat()
    except FileNotFoundError as exc:
        raise ValueError(f"{label} does not exist") from exc
    _reject_link_or_reparse(info, label)
    return info


def _reject_link_or_reparse(info: os.stat_result, label: str) -> None:
    if stat.S_ISLNK(info.st_mode) or _is_reparse(info):
        raise ValueError(f"unsafe {label}: symlink or reparse point")


def _is_reparse(info: os.stat_result) -> bool:
    return bool(getattr(info, "st_file_attributes", 0) & _REPARSE_POINT)


def _identity(info: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        info.st_dev,
        info.st_ino,
        info.st_size,
        info.st_mtime_ns,
        info.st_ctime_ns,
    )


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _claim_created_hook(_kind: str, _root: Path) -> None:
    """Deterministic test hook at the claim race boundary."""
