from __future__ import annotations

from pathlib import Path
from stat import S_IMODE
import sys

import pytest

from fleet_manager.map_data.pgm import PgmImage as FleetPgmImage


ROBOT_PLANNER_SRC = (
    Path(__file__).resolve().parents[1]
    / "sim_robot"
    / "ws"
    / "src"
    / "robot_planner"
)
if str(ROBOT_PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(ROBOT_PLANNER_SRC))

from robot_planner.route_core import atomic_storage
from robot_planner.route_core.pgm import PgmImage, read_pgm_size


def test_robot_pgm_preserves_whitespace_first_pixel(
    tmp_path: Path,
) -> None:
    path = tmp_path / "map.pgm"
    pixels = b"\x0a\x20\x00\xfe"
    path.write_bytes(b"P5\n# robot map\n2 2\n255\r\n" + pixels)

    image = PgmImage.read(path)

    assert read_pgm_size(path) == (2, 2)
    assert image.pixels == pixels


@pytest.mark.parametrize(
    "raw",
    [
        b"P5\n2 1\n255\n\x00\xfe",
        b"P5\r\n2 1\r\n255\r\n\x20\x0a",
        b"P2\n2 2\n10\n0 5 # gray\n10 1\n",
    ],
)
def test_robot_and_server_pgm_decoders_have_matching_results(
    raw: bytes,
) -> None:
    robot_image = PgmImage.from_bytes(raw)
    fleet_image = FleetPgmImage.from_bytes(raw)

    assert robot_image == PgmImage(
        width=fleet_image.width,
        height=fleet_image.height,
        maximum=fleet_image.maximum,
        pixels=fleet_image.pixels,
        encoding=fleet_image.encoding,
    )


def test_robot_atomic_writer_preserves_mode_and_replaces_contents(
    tmp_path: Path,
) -> None:
    target = tmp_path / "params.yaml"
    target.write_text("old: value\n", encoding="utf-8")
    target.chmod(0o640)

    atomic_storage.atomic_write_text(target, "new: value\n")

    assert target.read_text(encoding="utf-8") == "new: value\n"
    assert S_IMODE(target.stat().st_mode) == 0o640
    assert list(tmp_path.glob(f".{target.name}.*.tmp")) == []


def test_robot_atomic_writer_keeps_old_file_if_replace_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = tmp_path / "map.pgm"
    target.write_bytes(b"stable")

    def fail_replace(_source: Path, _target: Path) -> None:
        raise OSError("simulated failure")

    monkeypatch.setattr(
        atomic_storage.os,
        "replace",
        fail_replace,
    )

    with pytest.raises(OSError, match="simulated failure"):
        atomic_storage.atomic_write_bytes(target, b"incomplete")

    assert target.read_bytes() == b"stable"
    assert list(tmp_path.glob(f".{target.name}.*.tmp")) == []
