from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path
from typing import Any

from .map_loader import WarehouseMapLoader
from .planner import LmRoutePlanner


def find_ros_map_yaml(map_dir: Path) -> Path:
    directory = Path(map_dir).resolve()
    candidates = sorted(
        path
        for path in directory.glob("*.yaml")
        if path.name not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml"}
    )
    if not candidates:
        raise FileNotFoundError(f"No ROS map yaml found in {directory}")
    return candidates[0]


def build_editable_map_payload(
    map_dir: Path,
    *,
    params: dict[str, Any] | None = None,
) -> dict[str, Any]:
    loaded_map = WarehouseMapLoader(map_dir).load()
    planner = LmRoutePlanner(loaded_map.landmarks, loaded_map.edges, params=params)
    landmarks = [loaded_map.landmarks[name] for name in sorted(loaded_map.landmarks)]
    payload = {
        "ok": True,
        "mapName": loaded_map.map_dir.stem.replace(".smap", ""),
        "coordinateFrame": "map_top_left",
        "mapDir": str(loaded_map.map_dir),
        "map": loaded_map.map_metadata.to_dict(),
        "lms": [item.to_dict() for item in landmarks],
        "edges": [edge.to_dict() for edge in loaded_map.edges],
        "routes": planner.build_route_catalog(),
        "defaultGoal": landmarks[-1].name if landmarks else "",
    }
    payload["signature"] = editable_map_signature(payload)
    return payload


def build_editable_map_bundle_payload(
    map_dir: Path,
    *,
    params: dict[str, Any] | None = None,
) -> dict[str, Any]:
    payload = build_editable_map_payload(map_dir, params=params)
    root = Path(map_dir).resolve()
    files: list[dict[str, str]] = []
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        files.append(
            {
                "path": str(path.relative_to(root)).replace("\\", "/"),
                "encoding": "base64",
                "content": base64.b64encode(path.read_bytes()).decode("ascii"),
            }
        )
    payload["files"] = files
    return payload


def restore_editable_map_bundle(map_dir: Path, payload: dict[str, Any]) -> Path:
    root = Path(map_dir).resolve()
    files = payload.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("map bundle does not contain files")
    root.mkdir(parents=True, exist_ok=True)
    for item in files:
        if not isinstance(item, dict):
            continue
        relative = str(item.get("path") or "").strip().replace("\\", "/")
        if not relative:
            continue
        target = (root / relative).resolve()
        if root not in target.parents and target != root:
            raise ValueError("bundle file must stay inside map directory")
        target.parent.mkdir(parents=True, exist_ok=True)
        encoding = str(item.get("encoding") or "base64").strip()
        content = item.get("content")
        if encoding != "base64" or not isinstance(content, str):
            raise ValueError(f"unsupported bundle entry for {relative}")
        target.write_bytes(base64.b64decode(content.encode("ascii")))
    return root


def editable_map_signature(payload: dict[str, Any]) -> str:
    normalized = {
        "mapName": str(payload.get("mapName") or ""),
        "map": payload.get("map") or {},
        "lms": payload.get("lms") or [],
        "edges": payload.get("edges") or [],
    }
    encoded = json.dumps(normalized, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def list_editable_maps(
    maps_root: Path,
    *,
    active_map_dir: Path | None = None,
) -> dict[str, Any]:
    root = Path(maps_root).resolve()
    active_dir = active_map_dir.resolve() if active_map_dir is not None else None
    maps: list[dict[str, Any]] = []
    for item in sorted(root.glob("*.smap")):
        if not item.is_dir():
            continue
        if not (item / "LMs.yaml").exists():
            continue
        maps.append(
            {
                "name": item.stem.replace(".smap", ""),
                "folder": item.name,
                "mapDir": str(item.resolve()),
                "active": active_dir is not None and item.resolve() == active_dir,
            }
        )
    return {
        "ok": True,
        "active": active_dir.stem.replace(".smap", "") if active_dir is not None else "",
        "maps": maps,
    }
