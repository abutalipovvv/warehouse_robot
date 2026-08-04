"""Small durable file-writing helpers shared by runtime applications."""

from __future__ import annotations

import json
import os
from pathlib import Path
from stat import S_IMODE
from tempfile import NamedTemporaryFile
from typing import Any


def atomic_write_text(
    path: Path,
    content: str,
    *,
    encoding: str = "utf-8",
) -> None:
    """Replace a text file without exposing a partially written version."""

    _atomic_write(path, content, mode="w", encoding=encoding)


def atomic_write_bytes(path: Path, content: bytes) -> None:
    """Replace a binary file without exposing a partially written version."""

    _atomic_write(path, content, mode="wb")


def _atomic_write(
    path: Path,
    content: str | bytes,
    *,
    mode: str,
    encoding: str | None = None,
) -> None:
    target = path.expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    target_mode = _file_mode(target)
    temporary_path: Path | None = None

    try:
        options: dict[str, Any] = {
            "mode": mode,
            "dir": target.parent,
            "prefix": f".{target.name}.",
            "suffix": ".tmp",
            "delete": False,
        }
        if encoding is not None:
            options["encoding"] = encoding
        with NamedTemporaryFile(
            **options,
        ) as temporary:
            temporary_path = Path(temporary.name)
            os.chmod(temporary_path, target_mode)
            temporary.write(content)
            temporary.flush()
            os.fsync(temporary.fileno())

        os.replace(temporary_path, target)
        temporary_path = None
        _sync_directory(target.parent)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def atomic_write_json(path: Path, payload: Any) -> None:
    atomic_write_text(
        path,
        json.dumps(payload, ensure_ascii=False, indent=2),
    )




def _file_mode(path: Path) -> int:
    try:
        return S_IMODE(path.stat().st_mode)
    except OSError:
        return 0o644


def _sync_directory(directory: Path) -> None:
    """Persist the rename when the platform supports directory fsync."""

    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    try:
        descriptor = os.open(directory, flags)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)
