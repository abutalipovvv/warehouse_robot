from __future__ import annotations

import base64
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from fleet_manager.core.mapping.maps.map_exchange import (
    build_editable_map_bundle_payload,
    build_editable_map_payload,
)
from operator_app.core.map_cache import MapCache
from operator_app.core.fleet_map_service import FleetMapService
from test_map_writer_raster import _make_map


ROBOT_PLANNER_SRC = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "robot_driver"
    / "src"
    / "robot_planner"
)
if str(ROBOT_PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(ROBOT_PLANNER_SRC))

from robot_planner import build_editable_map_payload as build_robot_map_payload


def test_server_and_robot_use_the_same_full_map_signature(tmp_path: Path) -> None:
    map_dir = _make_map(tmp_path)
    (map_dir / "traffic_zones.yaml").write_text(
        (
            "mapName: raster_test\n"
            "coordinateFrame: map_top_left\n"
            "zones:\n"
            "  - id: corridor:test\n"
            "    kind: controlled_corridor\n"
            "    shape: rectangle\n"
            "    bounds: {minX: 0.0, minY: 0.0, maxX: 0.1, maxY: 0.1}\n"
            "    capacity: 1\n"
        ),
        encoding="utf-8",
    )

    server = build_editable_map_payload(map_dir)
    robot = build_robot_map_payload(map_dir)

    assert server["signatureVersion"] == robot["signatureVersion"] == 2
    assert server["contentManifest"] == robot["contentManifest"]
    assert server["trafficZones"] == robot["trafficZones"]
    assert server["signature"] == robot["signature"]


def test_operator_cache_rejects_corrupted_pull_without_losing_current_map(
    tmp_path: Path,
) -> None:
    source = _make_map(tmp_path)
    bundle = build_editable_map_bundle_payload(source)
    cache = MapCache(tmp_path / "cache")
    cache.save_pulled_map("robot-1", bundle, activate=True)
    original = cache.load_active_map("robot-1")
    assert original is not None
    original_signature = original["signature"]

    corrupted = {**bundle, "files": [dict(item) for item in bundle["files"]]}
    pgm = next(item for item in corrupted["files"] if item["path"] == "map.pgm")
    raw = base64.b64decode(pgm["content"])
    pgm["content"] = base64.b64encode(raw[:-1] + b"\xff").decode("ascii")

    with pytest.raises(ValueError, match="signature mismatch"):
        cache.save_pulled_map("robot-1", corrupted, activate=True)

    current = cache.load_active_map("robot-1")
    assert current is not None
    assert current["signature"] == original_signature
    robot_cache = tmp_path / "cache" / "robot-1"
    assert not list(robot_cache.glob(".*.incoming-*"))
    assert not list(robot_cache.glob(".*.backup"))


def test_local_only_bundle_is_never_reported_as_robot_synced(tmp_path: Path) -> None:
    source = _make_map(tmp_path)
    bundle = build_editable_map_bundle_payload(source)
    cache = MapCache(tmp_path / "cache")

    cache.save_local_bundle("robot-1", bundle, activate=True)
    local = cache.load_active_map("robot-1")

    assert local is not None
    assert local["hasLocalChanges"] is True
    assert local["isStoredOnRobot"] is False
    assert local["robotSignature"] == ""


def test_fleet_push_updates_storage_but_requires_explicit_load(
    tmp_path: Path,
) -> None:
    map_dir = _make_map(tmp_path)
    initial = build_editable_map_payload(map_dir)
    owner = SimpleNamespace(
        map_dir=map_dir,
        maps_root=tmp_path,
        params_path=tmp_path / "params.yaml",
        mode="robots",
        _active_map_signature=initial["signature"],
    )
    owner._resolve_map_dir_by_name = lambda map_name: map_dir
    service = FleetMapService(owner)
    edited = build_editable_map_payload(map_dir)
    edited["lms"][0]["x"] = 0.10

    pushed = service.push_map_payload(
        {
            "mapName": "raster_test",
            "sourceMapName": "raster_test",
            "map": edited,
        }
    )

    assert pushed["signature"] != initial["signature"]
    assert service.maps_active_payload()["signature"] == initial["signature"]
    stored = service.maps_list_payload()["maps"][0]
    assert stored["active"] is True
    assert stored["signature"] == pushed["signature"]

    def load_context(target: Path) -> None:
        owner.map_dir = target
        owner._active_map_signature = build_editable_map_payload(target)["signature"]

    owner._load_context = load_context
    loaded = service.load_map_payload({"mapName": "raster_test"})
    assert loaded["signature"] == pushed["signature"]
