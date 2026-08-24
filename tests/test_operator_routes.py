from __future__ import annotations

from urllib.parse import urlparse

import pytest

from operator_app.core.fleet_manager import (
    FLEET_MANAGER_ID,
    FLEET_MANAGER_SIM_ID,
)
from operator_app.web.routes import (
    FLEET_ACTIONS,
    ROBOT_MAP_ACTIONS,
    FleetRoute,
    RobotMapRoute,
    SceneAssetRoute,
    parse_fleet_route,
    parse_robot_map_route,
    parse_robot_params_route,
    parse_robot_proxy_route,
    parse_robot_slam_route,
    parse_scene_asset_route,
)


@pytest.mark.parametrize(
    ("parts", "action"),
    list(FLEET_ACTIONS.items()),
)
def test_all_declarative_fleet_routes(
    parts: tuple[str, ...],
    action: str,
) -> None:
    suffix = "/".join(parts)
    path = "/api/fleet-manager" + (f"/{suffix}" if suffix else "")

    assert parse_fleet_route(urlparse(path)) == FleetRoute(
        FLEET_MANAGER_ID,
        action,
        "",
    )


def test_simulation_and_dynamic_fleet_map_routes() -> None:
    assert parse_fleet_route(
        urlparse("/api/fleet-manager-sim/maps/pull?name=Main%20Map")
    ) == FleetRoute(FLEET_MANAGER_SIM_ID, "maps_pull", "Main Map")
    assert parse_fleet_route(
        urlparse("/api/fleet-manager/maps/local/My%20Map")
    ) == FleetRoute(FLEET_MANAGER_ID, "maps_local_get", "My Map")


def test_scene_asset_routes_are_versioned_per_manager() -> None:
    assert parse_scene_asset_route(
        urlparse(
            "/api/fleet-manager/scene3d/assets/abc123/walls.f32"
        )
    ) == SceneAssetRoute(FLEET_MANAGER_ID, "abc123", "walls.f32")
    assert parse_scene_asset_route(
        urlparse(
            "/api/fleet-manager-sim/scene3d/assets/def456/floor.png"
        )
    ) == SceneAssetRoute(
        FLEET_MANAGER_SIM_ID,
        "def456",
        "floor.png",
    )


@pytest.mark.parametrize(
    ("parts", "action"),
    list(ROBOT_MAP_ACTIONS.items()),
)
def test_all_declarative_robot_map_routes(
    parts: tuple[str, ...],
    action: str,
) -> None:
    path = f"/api/robots/robot%201/{'/'.join(parts)}"

    assert parse_robot_map_route(urlparse(path)) == RobotMapRoute(
        "robot 1",
        action,
        "",
    )


def test_dynamic_robot_routes_preserve_names_and_proxy_query() -> None:
    assert parse_robot_map_route(
        urlparse("/api/robots/r1/maps/pull?name=Map%20A")
    ) == RobotMapRoute("r1", "pull", "Map A")
    assert parse_robot_map_route(
        urlparse("/api/robots/r1/maps/local/Map%20B")
    ) == RobotMapRoute("r1", "local_get", "Map B")
    assert parse_robot_proxy_route(
        urlparse("/robots/r%201/api/status?full=1")
    ) == ("r 1", "/api/status?full=1")


def test_params_slam_and_unknown_routes() -> None:
    assert parse_robot_params_route(
        urlparse("/api/robots/r1/params")
    ) == "r1"
    assert parse_robot_slam_route(
        urlparse("/api/robots/r1/slam/start")
    ) == ("r1", "start")
    assert parse_robot_slam_route(
        urlparse("/api/robots/r1/slam/unknown")
    ) is None
    assert parse_fleet_route(urlparse("/api/unknown")) is None
    assert parse_robot_map_route(urlparse("/api/robots/r1/orders")) is None
