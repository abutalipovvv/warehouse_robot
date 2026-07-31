"""Pure URL parsing for the operator HTTP API."""

from __future__ import annotations

from typing import NamedTuple
from urllib.parse import ParseResult, parse_qs, unquote

from ..core.fleet_manager import FLEET_MANAGER_ID, FLEET_MANAGER_SIM_ID


class FleetRoute(NamedTuple):
    manager_id: str
    action: str
    argument: str = ""


class RobotMapRoute(NamedTuple):
    robot_id: str
    action: str
    argument: str = ""


FLEET_ACTIONS: dict[tuple[str, ...], str] = {
    (): "identity",
    ("identity",): "identity",
    ("status",): "status",
    ("state",): "state",
    ("mode",): "mode",
    ("map",): "map",
    ("scene3d",): "scene3d",
    ("params",): "params",
    ("orders",): "orders",
    ("setOrder",): "set_order",
    ("orders", "set"): "set_order",
    ("orders", "dispatch"): "orders_dispatch",
    ("orders", "cancel"): "orders_cancel",
    ("orders", "pause"): "orders_pause",
    ("orders", "resume"): "orders_resume",
    ("orders", "clear"): "orders_clear",
    ("plan",): "plan",
    ("benchmark",): "benchmark",
    ("tick",): "tick",
    ("world",): "world",
    ("check",): "check",
    ("manual-step",): "manual_step",
    ("manual-stop",): "manual_stop",
    ("maps", "list"): "maps_list",
    ("maps", "active"): "maps_active",
    ("maps", "local"): "maps_local_list",
    ("maps", "local", "active"): "maps_local_active",
    ("maps", "local", "save"): "maps_local_save",
    ("maps", "local", "activate"): "maps_local_activate",
    ("maps", "pull-sync"): "maps_pull_sync",
    ("maps", "push"): "maps_push",
    ("maps", "push-sync"): "maps_push_sync",
    ("maps", "load"): "maps_load",
    ("maps", "save"): "maps_save",
    ("robots",): "robots_add",
    ("robots", "remove"): "robots_remove",
    ("robots", "update"): "robots_update",
    ("robots", "stop"): "robots_stop",
    ("robots", "reset"): "robots_reset",
}

ROBOT_MAP_ACTIONS: dict[tuple[str, ...], str] = {
    ("maps", "list"): "robot_list",
    ("maps", "active"): "robot_active",
    ("maps", "local"): "local_list",
    ("maps", "local", "active"): "local_active",
    ("maps", "local", "save"): "local_save",
    ("maps", "local", "activate"): "local_activate",
    ("maps", "pull-sync"): "pull_sync",
    ("maps", "push"): "push",
    ("maps", "push-sync"): "push_sync",
    ("maps", "load"): "load",
}


def parse_fleet_route(parsed: ParseResult) -> FleetRoute | None:
    base_and_manager = _fleet_base(parsed.path)
    if base_and_manager is None:
        return None
    base, manager_id = base_and_manager
    parts = _parts(parsed.path.removeprefix(base))

    action = FLEET_ACTIONS.get(parts)
    if action is not None:
        return FleetRoute(manager_id, action)
    if parts == ("maps", "pull"):
        return FleetRoute(
            manager_id,
            "maps_pull",
            _query_value(parsed, "name"),
        )
    if (
        len(parts) == 3
        and parts[:2] == ("maps", "local")
    ):
        return FleetRoute(
            manager_id,
            "maps_local_get",
            unquote(parts[2]).strip(),
        )
    return None


def parse_robot_map_route(parsed: ParseResult) -> RobotMapRoute | None:
    robot_tail = _robot_api_tail(parsed.path)
    if robot_tail is None:
        return None
    robot_id, tail = robot_tail
    parts = _parts(tail)
    action = ROBOT_MAP_ACTIONS.get(parts)
    if action is not None:
        return RobotMapRoute(robot_id, action)
    if parts == ("maps", "pull"):
        return RobotMapRoute(
            robot_id,
            "pull",
            _query_value(parsed, "name"),
        )
    if (
        len(parts) == 3
        and parts[:2] == ("maps", "local")
    ):
        return RobotMapRoute(
            robot_id,
            "local_get",
            unquote(parts[2]).strip(),
        )
    return None


def parse_robot_params_route(parsed: ParseResult) -> str | None:
    robot_tail = _robot_api_tail(parsed.path)
    if robot_tail is None:
        return None
    robot_id, tail = robot_tail
    return robot_id if _parts(tail) == ("params",) else None


def parse_robot_slam_route(
    parsed: ParseResult,
) -> tuple[str, str] | None:
    robot_tail = _robot_api_tail(parsed.path)
    if robot_tail is None:
        return None
    robot_id, tail = robot_tail
    parts = _parts(tail)
    if (
        len(parts) == 2
        and parts[0] == "slam"
        and parts[1] in {"defaults", "state", "start", "finish", "cancel"}
    ):
        return robot_id, parts[1]
    return None


def parse_robot_proxy_route(
    parsed: ParseResult,
) -> tuple[str, str] | None:
    if not parsed.path.startswith("/robots/"):
        return None
    remainder = parsed.path.removeprefix("/robots/")
    if not remainder:
        return None
    robot_part, separator, tail = remainder.partition("/")
    robot_id = unquote(robot_part).strip()
    if not robot_id:
        return None
    robot_path = "/" if not separator else f"/{tail}"
    if parsed.query:
        robot_path = f"{robot_path}?{parsed.query}"
    return robot_id, robot_path


def _fleet_base(path: str) -> tuple[str, str] | None:
    simulation_base = "/api/fleet-manager-sim"
    real_base = "/api/fleet-manager"
    if path == simulation_base or path.startswith(f"{simulation_base}/"):
        return simulation_base, FLEET_MANAGER_SIM_ID
    if path == real_base or path.startswith(f"{real_base}/"):
        return real_base, FLEET_MANAGER_ID
    return None


def _robot_api_tail(path: str) -> tuple[str, str] | None:
    prefix = "/api/robots/"
    if not path.startswith(prefix):
        return None
    robot_part, separator, tail = path.removeprefix(prefix).partition("/")
    robot_id = unquote(robot_part).strip()
    if not separator or not robot_id:
        return None
    return robot_id, tail


def _parts(path: str) -> tuple[str, ...]:
    return tuple(item for item in path.strip("/").split("/") if item)


def _query_value(parsed: ParseResult, name: str) -> str:
    return str(
        parse_qs(parsed.query).get(name, [""])[0] or ""
    ).strip()


__all__ = [
    "FLEET_ACTIONS",
    "ROBOT_MAP_ACTIONS",
    "FleetRoute",
    "RobotMapRoute",
    "parse_fleet_route",
    "parse_robot_map_route",
    "parse_robot_params_route",
    "parse_robot_proxy_route",
    "parse_robot_slam_route",
]
