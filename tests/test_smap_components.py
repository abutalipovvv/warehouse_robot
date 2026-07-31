from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from fleet_manager.map_data.smap_bundle import (
    GraphEdgeBuilder,
    SmapBundleWriter,
    SmapDocumentParser,
    parse_properties,
)
from fleet_manager.map_data.smap_deserialize import deserialize_smap
from fleet_manager.map_data.smap_raster import OccupancyRaster, SmapHeader


def _document() -> dict[str, object]:
    return {
        "header": {
            "mapName": "tiny",
            "minPos": {"x": 0.0, "y": 0.0},
            "maxPos": {"x": 2.0, "y": 1.0},
            "resolution": 1.0,
        },
        "normalPosList": [{"x": 0.0, "y": 0.0}, {"broken": True}],
        "advancedPointList": [
            {
                "className": "LocationMark",
                "instanceName": "A",
                "pos": {"x": 0.0, "y": 0.0},
            },
            {
                "className": "LocationMark",
                "instanceName": "B",
                "pos": {"x": 2.0, "y": 0.0},
            },
        ],
        "advancedLineList": [
            {
                "className": "FeatureLine",
                "line": {
                    "startPos": {"x": 0.0, "y": 0.0},
                    "endPos": {"x": 2.0, "y": 0.0},
                },
                "property": [{"key": "speed", "doubleValue": 0.5}],
            }
        ],
        "advancedCurveList": [],
    }


def test_header_uses_inclusive_pixel_centres() -> None:
    document = _document()
    document["header"]["maxPos"]["x"] = 2.0000000000000004

    header = SmapHeader.from_document(document, fallback_name="fallback")

    assert (header.width, header.height) == (3, 2)


def test_raster_bulk_points_and_line_use_same_coordinate_system() -> None:
    header = SmapHeader.from_document(_document(), fallback_name="fallback")
    raster = OccupancyRaster(header)

    used, skipped = raster.mark_raw_points(
        [{"x": 0.0, "y": 1.0}, {"bad": True}]
    )
    raster.draw_line(
        header.minimum,
        type(header.minimum)(2.0, 0.0),
    )

    assert (used, skipped) == (1, 1)
    assert raster.pgm_bytes() == b"P5\n3 2\n255\n\x00\xfe\xfe\x00\x00\x00"


def test_parser_separates_raster_primitives_landmarks_and_edges() -> None:
    bundle = SmapDocumentParser(
        _document(),
        fallback_name="fallback",
    ).parse()

    assert bundle.normal_points.used == 1
    assert bundle.normal_points.skipped == 1
    assert len(bundle.line_primitives) == 1
    assert [item["name"] for item in bundle.landmarks] == ["A", "B"]
    assert bundle.edges == [
        {
            "from": "A",
            "to": "B",
            "length": 2.0,
            "kind": "line",
            "type": "FeatureLine",
            "properties": {"speed": 0.5},
        }
    ]


def test_graph_builder_keeps_shortest_duplicate_direction() -> None:
    landmarks = [
        {"name": "A", "x": 0.0, "y": 0.0},
        {"name": "B", "x": 1.0, "y": 0.0},
    ]
    lines = [
        {
            "start": {"x": 0.0, "y": 0.0},
            "end": {"x": 1.0, "y": 0.0},
            "length_m": length,
            "line_type": "FeatureLine",
            "properties": {},
        }
        for length in (2.0, 1.0)
    ]

    edges = GraphEdgeBuilder(landmarks).build(lines, [])

    assert len(edges) == 1
    assert edges[0]["length"] == 1.0


def test_bundle_writer_produces_loadable_artifacts(tmp_path: Path) -> None:
    bundle = SmapDocumentParser(
        _document(),
        fallback_name="fallback",
    ).parse()

    SmapBundleWriter().write(bundle, tmp_path)

    assert (tmp_path / "tiny.pgm").read_bytes().startswith(b"P5\n3 2\n255\n")
    assert yaml.safe_load((tmp_path / "LMs.yaml").read_text())["mapName"] == "tiny"
    assert json.loads((tmp_path / "smap_summary.json").read_text())[
        "counts"
    ]["edges_total"] == 1
    assert not list(tmp_path.glob(".*.tmp"))


def test_property_parser_prefers_typed_values() -> None:
    assert parse_properties(
        [
            {
                "key": "capacity",
                "int32Value": 2,
                "value": "ignored",
            }
        ]
    ) == {"capacity": 2}


def test_deserializer_rejects_non_object_root(tmp_path: Path) -> None:
    source = tmp_path / "invalid.smap"
    source.write_text("[]", encoding="utf-8")

    with pytest.raises(ValueError, match="root must be a JSON object"):
        deserialize_smap(source, tmp_path / "output")
