from importlib import import_module

from .route_core import (
    DEFAULT_PARAMS_PATH,
    DEFAULT_ROUTE_PARAMS,
    LmRoutePlanner,
    WarehouseMapLoader,
    build_editable_map_bundle_payload,
    build_editable_map_payload,
    editable_map_signature,
    list_editable_maps,
    load_route_params,
    restore_editable_map_bundle,
    save_route_params,
)


_LAZY_EXPORTS = {
    "PlannedRobotRoute": ("runtime", "PlannedRobotRoute"),
    "Pose2D": ("runtime", "Pose2D"),
    "RobotRuntime": ("runtime", "RobotRuntime"),
    "RoutePoint": ("runtime", "RoutePoint"),
    "route_update_is_stale": ("runtime", "route_update_is_stale"),
    "RobotTrajectoryPlanner": ("planning", "RobotTrajectoryPlanner"),
    "RouteExecutor": ("execution", "RouteExecutor"),
    "RobotRouteNode": ("route_node", "RobotRouteNode"),
    "PidController": ("control", "PidController"),
    "PidParameters": ("control", "PidParameters"),
    "PathProjection": ("math", "PathProjection"),
    "TrajectoryArray": ("math", "TrajectoryArray"),
    "TrajectoryMath": ("math", "TrajectoryMath"),
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
    "PidController",
    "PidParameters",
    "PathProjection",
    "PlannedRobotRoute",
    "Pose2D",
    "RobotRuntime",
    "RobotTrajectoryPlanner",
    "RouteExecutor",
    "RobotRouteNode",
    "RoutePoint",
    "TrajectoryArray",
    "TrajectoryMath",
    "WarehouseMapLoader",
    "build_editable_map_bundle_payload",
    "build_editable_map_payload",
    "editable_map_signature",
    "list_editable_maps",
    "load_route_params",
    "restore_editable_map_bundle",
    "route_update_is_stale",
    "save_route_params",
]
