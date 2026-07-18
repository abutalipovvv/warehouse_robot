from __future__ import annotations

import base64
from pathlib import Path

import pytest
import yaml

from fleet_manager.core.route_core.map_exchange import build_editable_map_payload
from fleet_manager.core.route_core.map_writer import save_editable_map


def _make_map(tmp_path: Path) -> Path:
    map_dir = tmp_path / "raster_test.smap"
    map_dir.mkdir()
    (map_dir / "map.pgm").write_bytes(b"P5\n4 3\n255\n" + bytes(range(12)))
    (map_dir / "map.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "map.pgm",
                "resolution": 0.05,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "LMs.yaml").write_text(
        yaml.safe_dump(
            {
                "mapName": "raster_test",
                "coordinateFrame": "map_top_left",
                "LMs": [{"name": "LM1", "x": 0.05, "y": 0.05}],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "graphs.yaml").write_text(
        yaml.safe_dump(
            {
                "mapName": "raster_test",
                "coordinateFrame": "map_top_left",
                "primitives": [],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "graph_edges_lengths.yaml").write_text("[]\n", encoding="utf-8")
    return map_dir


def test_save_editable_map_persists_gray8_raster(tmp_path: Path) -> None:
    map_dir = _make_map(tmp_path)
    payload = build_editable_map_payload(map_dir)
    pixels = bytes([254, 254, 0, 0, 205, 205, 0, 0, 254, 254, 254, 254])
    payload["map"]["raster"] = {
        "encoding": "base64",
        "format": "gray8",
        "width": 4,
        "height": 3,
        "pixelsBase64": base64.b64encode(pixels).decode("ascii"),
    }

    loaded = save_editable_map(map_dir, payload)

    assert loaded.map_metadata.width == 4
    assert loaded.map_metadata.height == 3
    assert (map_dir / "map.pgm").read_bytes() == b"P5\n4 3\n255\n" + pixels


def test_save_editable_map_rejects_raster_dimension_mismatch(tmp_path: Path) -> None:
    map_dir = _make_map(tmp_path)
    original = (map_dir / "map.pgm").read_bytes()
    payload = build_editable_map_payload(map_dir)
    payload["map"]["raster"] = {
        "encoding": "base64",
        "format": "gray8",
        "width": 2,
        "height": 2,
        "pixelsBase64": base64.b64encode(b"\x00\x00\x00\x00").decode("ascii"),
    }

    with pytest.raises(ValueError, match="dimensions must match"):
        save_editable_map(map_dir, payload)

    assert (map_dir / "map.pgm").read_bytes() == original
