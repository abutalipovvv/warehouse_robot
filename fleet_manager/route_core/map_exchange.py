from __future__ import annotations

from pathlib import Path
from typing import Any

from warehouse_maps.map_exchange import (
    build_editable_map_bundle_payload as _build_editable_map_bundle_payload,
    build_editable_map_payload as _build_editable_map_payload,
    editable_map_signature,
    find_ros_map_yaml,
    list_editable_maps,
    restore_editable_map_bundle,
)

from .params import load_route_params


def build_editable_map_payload(
    map_dir: Path,
    *,
    params: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return _build_editable_map_payload(map_dir, params=params or load_route_params())


def build_editable_map_bundle_payload(
    map_dir: Path,
    *,
    params: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return _build_editable_map_bundle_payload(map_dir, params=params or load_route_params())


__all__ = [
    "build_editable_map_bundle_payload",
    "build_editable_map_payload",
    "editable_map_signature",
    "find_ros_map_yaml",
    "list_editable_maps",
    "restore_editable_map_bundle",
]
