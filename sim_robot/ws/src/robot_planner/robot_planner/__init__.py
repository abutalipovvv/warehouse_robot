from importlib import import_module

from .route_core import (
    DEFAULT_PARAMS_PATH,
    DEFAULT_ROUTE_PARAMS,
    LmRoutePlanner,
    WarehouseMapLoader,
    load_route_params,
    save_route_params,
)


_LAZY_EXPORTS = {
    "PlannedRobotRoute": ("runtime", "PlannedRobotRoute"),
    "Pose2D": ("runtime", "Pose2D"),
    "RobotRuntime": ("runtime", "RobotRuntime"),
    "RoutePoint": ("runtime", "RoutePoint"),
    "route_update_is_stale": ("runtime", "route_update_is_stale"),
    "RobotTrajectoryPlanner": ("route_planner", "RobotTrajectoryPlanner"),
    "RouteExecutor": ("executor", "RouteExecutor"),
    "RobotRouteNode": ("route_node", "RobotRouteNode"),
}


def __getattr__(name: str):
    if name not in _LAZY_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute_name = _LAZY_EXPORTS[name]
    value = getattr(import_module(f"{__name__}.{module_name}"), attribute_name)
    globals()[name] = value
    return value


__all__ = [
    "DEFAULT_PARAMS_PATH",
    "DEFAULT_ROUTE_PARAMS",
    "LmRoutePlanner",
    "PlannedRobotRoute",
    "Pose2D",
    "RobotRuntime",
    "RobotTrajectoryPlanner",
    "RouteExecutor",
    "RobotRouteNode",
    "RoutePoint",
    "WarehouseMapLoader",
    "load_route_params",
    "route_update_is_stale",
    "save_route_params",
]
