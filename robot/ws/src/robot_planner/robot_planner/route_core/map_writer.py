from __future__ import annotations

import math
import re
import shutil
from pathlib import Path
from typing import Any

import yaml

from .map_loader import WarehouseMapLoader
from .models import LoadedMapData


def save_editable_map(
    map_dir: Path,
    payload: dict[str, Any],
    *,
    output_name: str = "",
    overwrite_output: bool = False,
) -> LoadedMapData:
    source_dir = Path(map_dir).resolve()
    target_dir = _target_map_dir(source_dir, output_name, overwrite_output)
    if target_dir != source_dir:
        shutil.copytree(source_dir, target_dir, dirs_exist_ok=overwrite_output)

    map_name = str(payload.get("mapName") or target_dir.stem.replace(".smap", "")).strip()
    landmarks = _normalize_landmarks(payload.get("lms") or payload.get("landmarks") or [])
    edges = _normalize_edges(payload.get("edges") or [], landmarks)

    _write_yaml(
        target_dir / "LMs.yaml",
        {
            "mapName": map_name,
            "LMs": landmarks,
        },
    )
    _write_yaml(
        target_dir / "graphs.yaml",
        {
            "mapName": map_name,
            "primitives": [_edge_to_primitive(edge, landmarks) for edge in edges],
        },
    )
    _write_yaml(
        target_dir / "graph_edges_lengths.yaml",
        [_edge_to_length_item(edge) for edge in edges],
    )
    return WarehouseMapLoader(target_dir).load()


def _target_map_dir(source_dir: Path, output_name: str, overwrite_output: bool) -> Path:
    name = str(output_name or "").strip()
    if not name:
        return source_dir
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._")
    if not safe_name:
        raise ValueError("save-as name is empty")
    if not safe_name.endswith(".smap"):
        safe_name = f"{safe_name}.smap"
    target = (source_dir.parent / safe_name).resolve()
    if source_dir.parent not in target.parents and target != source_dir.parent:
        raise ValueError("save-as target must stay inside maps_out")
    if target == source_dir and not overwrite_output:
        raise ValueError("save-as target is the current map; use overwrite current")
    if target.exists() and target != source_dir and not overwrite_output:
        raise ValueError(f"map already exists: {target.name}")
    return target


def _normalize_landmarks(raw_items: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_items, list):
        raise ValueError("lms must be a list")
    landmarks: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name") or "").strip()
        if not name:
            raise ValueError("LM name is required")
        if name in seen:
            raise ValueError(f"duplicate LM name: {name}")
        seen.add(name)
        landmarks.append(
            {
                "name": name,
                "x": _round_m(item.get("x")),
                "y": _round_m(item.get("y")),
                "ignoreDir": item.get("ignoreDir"),
                "properties": dict(item.get("properties") or {}),
            }
        )
    if not landmarks:
        raise ValueError("map must contain at least one LM")
    return landmarks


def _normalize_edges(raw_items: Any, landmarks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not isinstance(raw_items, list):
        raise ValueError("edges must be a list")
    lm_names = {str(item["name"]) for item in landmarks}
    result: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        start = str(item.get("from") or item.get("from_name") or "").strip()
        goal = str(item.get("to") or item.get("to_name") or "").strip()
        if not start or not goal or start == goal:
            continue
        if start not in lm_names or goal not in lm_names:
            continue
        key = (start, goal)
        if key in seen:
            continue
        seen.add(key)

        properties = dict(item.get("properties") or {})
        if "direction" not in properties:
            properties["direction"] = int(item.get("motionDirectionCode", 2) or 2)
        kind = str(item.get("kind") or ("curve" if item.get("geometry") == "bezier" else "line"))
        edge_type = str(item.get("type") or item.get("curve_type") or item.get("line_type") or "FeatureLine")
        geometry = _normalize_geometry(item, start, goal, landmarks)
        length = _edge_length(item, geometry, start, goal, landmarks)
        result.append(
            {
                "from": start,
                "to": goal,
                "length": length,
                "kind": kind,
                "type": edge_type,
                "properties": properties,
                "geometry": geometry,
            }
        )
    return result


def _normalize_geometry(
    item: dict[str, Any],
    start: str,
    goal: str,
    landmarks: list[dict[str, Any]],
) -> list[dict[str, float]] | None:
    raw_points = item.get("control_points")
    if not isinstance(raw_points, list) or len(raw_points) != 4:
        return None
    points: list[dict[str, float]] = []
    for point in raw_points:
        if not isinstance(point, dict):
            return None
        points.append({"x": _round_m(point.get("x")), "y": _round_m(point.get("y"))})
    start_lm = _landmark_by_name(landmarks, start)
    goal_lm = _landmark_by_name(landmarks, goal)
    if start_lm and goal_lm:
        points[0] = {"x": start_lm["x"], "y": start_lm["y"]}
        points[3] = {"x": goal_lm["x"], "y": goal_lm["y"]}
    return points


def _edge_length(
    item: dict[str, Any],
    geometry: list[dict[str, float]] | None,
    start: str,
    goal: str,
    landmarks: list[dict[str, Any]],
) -> float:
    if geometry:
        return _round_m(_bezier_length(geometry))
    start_lm = _landmark_by_name(landmarks, start)
    goal_lm = _landmark_by_name(landmarks, goal)
    if start_lm and goal_lm:
        return _round_m(math.hypot(goal_lm["x"] - start_lm["x"], goal_lm["y"] - start_lm["y"]))
    return _round_m(item.get("length", 0.0))


def _edge_to_primitive(edge: dict[str, Any], landmarks: list[dict[str, Any]]) -> dict[str, Any]:
    start = _landmark_by_name(landmarks, edge["from"])
    goal = _landmark_by_name(landmarks, edge["to"])
    if start is None or goal is None:
        raise ValueError(f"edge references unknown LM: {edge['from']}->{edge['to']}")
    geometry = edge.get("geometry")
    if isinstance(geometry, list) and len(geometry) == 4:
        return {
            "kind": "curve",
            "curve_type": str(edge.get("type") or "DegenerateBezier"),
            "curve": {
                "start": {"x": start["x"], "y": start["y"]},
                "end": {"x": goal["x"], "y": goal["y"]},
                "control1": geometry[1],
                "control2": geometry[2],
                "start_name": edge["from"],
                "end_name": edge["to"],
            },
            "properties": dict(edge.get("properties") or {}),
            "length_m": edge["length"],
        }
    return {
        "kind": "line",
        "line_type": str(edge.get("type") or "FeatureLine"),
        "start": {"x": start["x"], "y": start["y"]},
        "end": {"x": goal["x"], "y": goal["y"]},
        "start_name": edge["from"],
        "end_name": edge["to"],
        "properties": dict(edge.get("properties") or {}),
        "length_m": edge["length"],
    }


def _edge_to_length_item(edge: dict[str, Any]) -> dict[str, Any]:
    return {
        "from": edge["from"],
        "to": edge["to"],
        "length": edge["length"],
        "kind": edge["kind"],
        "type": edge["type"],
        "properties": dict(edge.get("properties") or {}),
    }


def _landmark_by_name(landmarks: list[dict[str, Any]], name: str) -> dict[str, Any] | None:
    for landmark in landmarks:
        if landmark["name"] == name:
            return landmark
    return None


def _bezier_length(points: list[dict[str, float]], steps: int = 120) -> float:
    previous = _bezier_point(points, 0.0)
    total = 0.0
    for index in range(1, steps + 1):
        current = _bezier_point(points, index / steps)
        total += math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
        previous = current
    return total


def _bezier_point(points: list[dict[str, float]], t: float) -> dict[str, float]:
    p0, p1, p2, p3 = points
    u = 1.0 - t
    return {
        "x": (u**3 * p0["x"]) + (3 * u * u * t * p1["x"]) + (3 * u * t * t * p2["x"]) + (t**3 * p3["x"]),
        "y": (u**3 * p0["y"]) + (3 * u * u * t * p1["y"]) + (3 * u * t * t * p2["y"]) + (t**3 * p3["y"]),
    }


def _round_m(value: Any) -> float:
    return round(float(value or 0.0), 6)


def _write_yaml(path: Path, payload: Any) -> None:
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
