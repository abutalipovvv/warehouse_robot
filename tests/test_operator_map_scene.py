from __future__ import annotations

from pathlib import Path

from fleet_manager.core.mapping.maps.models import LoadedMapData, MapMetadata
from operator_app.core.map_scene import MapSceneBuilder, horizontal_runs


def _loaded_map(map_dir: Path, *, width: int = 4, height: int = 3) -> LoadedMapData:
    return LoadedMapData(
        map_dir=map_dir,
        map_metadata=MapMetadata(
            map_name="tiny",
            width=width,
            height=height,
            resolution=1.0,
            ros_origin=(0.0, 0.0, 0.0),
            image_data_url="data:image/x-portable-graymap;base64,",
        ),
        landmarks={},
        edges=[],
    )


def test_horizontal_runs_are_explicit_and_deterministic() -> None:
    occupied = {0, 1, 3, 5, 6}

    assert horizontal_runs(8, occupied.__contains__) == [
        (0, 2),
        (3, 1),
        (5, 2),
    ]


def test_wall_builder_merges_equal_runs_across_rows(tmp_path: Path) -> None:
    # Occupancy:
    # XX..
    # XX..
    # ..XX
    pixels = bytes(
        [
            0, 0, 254, 254,
            0, 0, 254, 254,
            254, 254, 0, 0,
        ]
    )
    builder = MapSceneBuilder(_loaded_map(tmp_path))

    rectangles = builder.merge_wall_rectangles(
        4,
        3,
        pixels,
        occupied_threshold=0.65,
        negate=0,
        stride=1,
        wall_height=1.8,
    )

    assert rectangles == [
        {
            "x": 1.0,
            "z": 1.0,
            "width": 2.0,
            "depth": 2.0,
            "height": 1.8,
            "stride": 1,
        },
        {
            "x": 3.0,
            "z": 2.5,
            "width": 2.0,
            "depth": 1.0,
            "height": 1.8,
            "stride": 1,
        },
    ]


def test_scene_payload_is_cached_after_map_raster_load(tmp_path: Path) -> None:
    (tmp_path / "tiny.pgm").write_bytes(b"P5\n2 1\n255\n\x00\xfe")
    (tmp_path / "tiny.yaml").write_text(
        "\n".join(
            [
                "image: tiny.pgm",
                "resolution: 1.0",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        ),
        encoding="utf-8",
    )
    builder = MapSceneBuilder(_loaded_map(tmp_path, width=2, height=1))

    first = builder.build()
    second = builder.build()

    assert second is first
    assert first["mapName"] == tmp_path.stem
    assert first["walls"] == [
        {
            "x": 0.5,
            "z": 0.5,
            "width": 1.0,
            "depth": 1.0,
            "height": 1.8,
            "stride": 1,
        }
    ]


def test_large_raster_uses_adaptive_visual_wall_stride(tmp_path: Path) -> None:
    width = 20
    height = 20
    (tmp_path / "large.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(width * height)
    )
    (tmp_path / "large.yaml").write_text(
        "\n".join(
            [
                "image: large.pgm",
                "resolution: 0.02",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        ),
        encoding="utf-8",
    )
    builder = MapSceneBuilder(
        _loaded_map(tmp_path, width=width, height=height),
        target_wall_grid_cells=100,
    )

    walls = builder.wall_rectangles(wall_height=1.8)

    assert len(walls) == 1
    assert walls[0]["stride"] == 2
