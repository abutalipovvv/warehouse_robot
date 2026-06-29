from pathlib import Path
from typing import Any

from warehouse_maps.params import (
    DEFAULT_ROUTE_PARAMS,
    load_route_params as _load_route_params,
    save_route_params as _save_route_params,
)


DEFAULT_PARAMS_PATH = Path(__file__).resolve().parents[1] / "params.yaml"


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
