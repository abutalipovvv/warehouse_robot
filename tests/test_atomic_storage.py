from __future__ import annotations

import json
from pathlib import Path
from stat import S_IMODE

import pytest

from fleet_manager import storage


def test_atomic_write_text_creates_parent_and_file(tmp_path: Path) -> None:
    target = tmp_path / "nested" / "value.txt"

    storage.atomic_write_text(target, "new value")

    assert target.read_text(encoding="utf-8") == "new value"
    assert list(target.parent.glob(f".{target.name}.*.tmp")) == []


def test_atomic_write_text_replaces_existing_file(tmp_path: Path) -> None:
    target = tmp_path / "value.txt"
    target.write_text("old value", encoding="utf-8")

    storage.atomic_write_text(target, "new value")

    assert target.read_text(encoding="utf-8") == "new value"


def test_atomic_write_bytes_replaces_existing_file(tmp_path: Path) -> None:
    target = tmp_path / "map.pgm"
    target.write_bytes(b"old")

    storage.atomic_write_bytes(target, b"P5\n1 1\n255\n\xfe")

    assert target.read_bytes() == b"P5\n1 1\n255\n\xfe"
    assert list(tmp_path.glob(f".{target.name}.*.tmp")) == []


def test_atomic_write_text_preserves_existing_permissions(
    tmp_path: Path,
) -> None:
    target = tmp_path / "value.txt"
    target.write_text("old value", encoding="utf-8")
    target.chmod(0o640)

    storage.atomic_write_text(target, "new value")

    assert S_IMODE(target.stat().st_mode) == 0o640


def test_atomic_write_json_preserves_unicode(tmp_path: Path) -> None:
    target = tmp_path / "state.json"

    storage.atomic_write_json(target, {"robot": "Робот-1"})

    assert json.loads(target.read_text(encoding="utf-8")) == {
        "robot": "Робот-1",
    }


def test_failed_replace_keeps_previous_file_and_removes_temporary_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = tmp_path / "value.txt"
    target.write_text("stable value", encoding="utf-8")

    def fail_replace(_source: Path, _target: Path) -> None:
        raise OSError("simulated rename failure")

    monkeypatch.setattr(storage.os, "replace", fail_replace)

    with pytest.raises(OSError, match="simulated rename failure"):
        storage.atomic_write_text(target, "incomplete value")

    assert target.read_text(encoding="utf-8") == "stable value"
    assert list(tmp_path.glob(f".{target.name}.*.tmp")) == []
