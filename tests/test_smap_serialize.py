from __future__ import annotations

import json
from pathlib import Path

import yaml

from fleet_manager.map_data.smap_deserialize import deserialize_smap
from fleet_manager.map_data.smap_serialize import serialize_smap_bundle
from fleet_manager.core.route_core.map_loader import WarehouseMapLoader
from fleet_manager.benchmarking.rds_dynamic_orders import MapGraph


ROOT = Path(__file__).resolve().parents[1]
MAPS_ROOT = ROOT / "fleet_manager" / "map_data" / "maps_out"
SOURCE = MAPS_ROOT / "smart_kiva_large_w_mode.smap"
OPEN_SOURCE = MAPS_ROOT / "benchmark_open_kiva.smap"


def test_unpacked_smart_kiva_round_trips_through_single_smap(
    tmp_path: Path,
) -> None:
    output = tmp_path / "smart_kiva_large_w_mode.smap"

    summary = serialize_smap_bundle(SOURCE, output)
    payload = json.loads(output.read_text(encoding="utf-8"))

    assert summary["mapName"] == "smart_kiva_large_w_mode"
    assert summary["landmarks"] == 576
    assert summary["directedEdges"] == 1240
    assert summary["trafficZones"] == 32
    assert payload["header"]["mapType"] == "2D-Map"
    assert len(payload["normalPosList"]) > 20_000
    assert payload["advancedLineList"] == []
    assert len(payload["advancedPointList"]) == 576
    assert len(payload["advancedCurveList"]) == 1240
    assert len(payload["advancedAreaList"]) == 32

    bundle_graph = MapGraph(SOURCE)
    serialized_graph = MapGraph(output)
    assert serialized_graph.points == bundle_graph.points
    assert serialized_graph.point_properties == bundle_graph.point_properties
    assert serialized_graph.adjacency == bundle_graph.adjacency
    assert serialized_graph.corridors == bundle_graph.corridors

    unpacked = tmp_path / "roundtrip"
    deserialize_smap(output, unpacked)
    source_map = WarehouseMapLoader(SOURCE).load()
    roundtrip_map = WarehouseMapLoader(unpacked).load()

    assert roundtrip_map.map_metadata.width == source_map.map_metadata.width
    assert roundtrip_map.map_metadata.height == source_map.map_metadata.height
    assert roundtrip_map.map_metadata.resolution == source_map.map_metadata.resolution
    assert {
        name: (round(landmark.x, 9), round(landmark.y, 9))
        for name, landmark in roundtrip_map.landmarks.items()
    } == {
        name: (round(landmark.x, 9), round(landmark.y, 9))
        for name, landmark in source_map.landmarks.items()
    }
    assert {
        (
            edge.from_name,
            edge.to_name,
            round(edge.length, 6),
            edge.motion_direction_code(),
        )
        for edge in roundtrip_map.edges
    } == {
        (
            edge.from_name,
            edge.to_name,
            round(edge.length, 6),
            edge.motion_direction_code(),
        )
        for edge in source_map.edges
    }

    source_ros = yaml.safe_load(
        (SOURCE / "smart_kiva_large_w_mode.yaml").read_text(encoding="utf-8")
    )
    roundtrip_ros = yaml.safe_load(
        (unpacked / "smart_kiva_large_w_mode.yaml").read_text(encoding="utf-8")
    )
    source_pixels = WarehouseMapLoader(SOURCE)._load_pgm(
        SOURCE / source_ros["image"]
    )[2]
    roundtrip_pixels = WarehouseMapLoader(unpacked)._load_pgm(
        unpacked / roundtrip_ros["image"]
    )[2]
    assert roundtrip_pixels == source_pixels


def test_open_map_export_keeps_rds_canvas_extent(tmp_path: Path) -> None:
    output = tmp_path / "benchmark_open_kiva.smap"

    summary = serialize_smap_bundle(OPEN_SOURCE, output)
    payload = json.loads(output.read_text(encoding="utf-8"))
    normal = payload["normalPosList"]
    points = payload["advancedPointList"]
    x_values = sorted({round(float(item["pos"]["x"]), 6) for item in points})
    y_values = sorted({round(float(item["pos"]["y"]), 6) for item in points})

    assert summary["occupiedPoints"] == (2 * 440) + (2 * 404) - 4
    assert min(item["x"] for item in normal) == 0.0
    assert max(item["x"] for item in normal) == 43.9
    assert min(item["y"] for item in normal) == 0.0
    assert max(item["y"] for item in normal) == 40.3
    assert {round(b - a, 6) for a, b in zip(x_values, x_values[1:])} == {1.2}
    assert {round(b - a, 6) for a, b in zip(y_values, y_values[1:])} == {1.2}
