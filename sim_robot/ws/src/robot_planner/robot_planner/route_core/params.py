from copy import deepcopy
from pathlib import Path
from typing import Any

from warehouse_maps.params import (
    DEFAULT_ROUTE_PARAMS as _SHARED_DEFAULT_ROUTE_PARAMS,
    load_route_params as _load_route_params,
    save_route_params as _save_route_params,
)


def _discover_default_params_path() -> Path:
    module_path = Path(__file__).resolve()
    for parent in module_path.parents:
        candidate = parent / "params.yaml"
        if (parent / "robot_planner").exists() and (parent / "robot_map_manager").exists():
            return candidate
        if candidate.exists() and (parent / "robot_map_manager").exists():
            return candidate
    for parent in module_path.parents:
        candidate = parent / "params.yaml"
        if candidate.exists() and (parent / "map_data").exists():
            return candidate
    cwd_candidate = Path.cwd() / "params.yaml"
    if cwd_candidate.exists():
        return cwd_candidate.resolve()
    if len(module_path.parents) > 3:
        return module_path.parents[3] / "params.yaml"
    return module_path.parents[1] / "params.yaml"


DEFAULT_PARAMS_PATH = _discover_default_params_path()
DEFAULT_ROUTE_PARAMS = deepcopy(_SHARED_DEFAULT_ROUTE_PARAMS)
DEFAULT_ROUTE_PARAMS.pop("fleet", None)


def load_route_params(path: Path | None = None, create: bool = False) -> dict[str, Any]:
    return _load_route_params(
        path or DEFAULT_PARAMS_PATH,
        create=create,
        defaults=DEFAULT_ROUTE_PARAMS,
    )


def save_route_params(params: dict[str, Any], path: Path | None = None) -> Path:
    return _save_route_params(params, path or DEFAULT_PARAMS_PATH, defaults=DEFAULT_ROUTE_PARAMS)


__all__ = [
    "DEFAULT_PARAMS_PATH",
    "DEFAULT_ROUTE_PARAMS",
    "load_route_params",
    "save_route_params",
]
