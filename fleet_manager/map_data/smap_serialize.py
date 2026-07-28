#!/usr/bin/env python3
"""Serialize an unpacked operator map directory into one RoboShop/RDS .smap.

The repository also contains the inverse ``smap_deserialize.py`` utility.
Keeping this exporter independent of the web application makes Push/Pull/Load
map exchange usable from tests, maintenance scripts and the operator UI.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
from typing import Any

import yaml


def _read_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _find_ros_yaml(map_dir: Path) -> Path:
    candidates = sorted(
        path
        for path in map_dir.glob("*.yaml")
        if path.name not in {
            "LMs.yaml",
            "graphs.yaml",
            "graph_edges_lengths.yaml",
            "traffic_zones.yaml",
        }
    )
    for path in candidates:
        payload = _read_yaml(path)
        if isinstance(payload, dict) and payload.get("image"):
            return path
    raise ValueError(f"ROS map yaml was not found in {map_dir}")


def _read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    raw = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    while index < len(raw) and len(tokens) < 4:
        while index < len(raw) and chr(raw[index]).isspace():
            index += 1
        if index < len(raw) and raw[index] == ord("#"):
            while index < len(raw) and raw[index] not in {10, 13}:
                index += 1
            continue
        start = index
        while index < len(raw) and not chr(raw[index]).isspace():
            index += 1
        if start < index:
            tokens.append(raw[start:index])

    if len(tokens) != 4 or tokens[0] != b"P5":
        raise ValueError(f"only binary P5 PGM is supported: {path}")
    width = int(tokens[1])
    height = int(tokens[2])
    maximum = int(tokens[3])
    pixel_count = width * height
    # Binary pixel values may themselves be ASCII whitespace. Taking the
    # declared number of bytes from the end avoids consuming valid pixels
    # while still accepting LF and CRLF PGM headers.
    pixels = raw[-pixel_count:]
    if len(pixels) != width * height:
        raise ValueError(
            f"invalid PGM payload in {path}: "
            f"{len(pixels)} != {width}x{height}"
        )
    return width, height, maximum, pixels


def _coordinate_frame_is_map_top_left(payload: Any) -> bool:
    return (
        isinstance(payload, dict)
        and str(
            payload.get("coordinateFrame")
            or payload.get("coordinate_frame")
            or ""
        ).strip()
        == "map_top_left"
    )


def _map_to_ros_point(
    x: float,
    y: float,
    *,
    map_top_left: bool,
    height_m: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> dict[str, float]:
    if not map_top_left:
        return {"x": float(x), "y": float(y)}
    local_x = float(x)
    local_y = height_m - float(y)
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    return {
        "x": origin_x + (cos_yaw * local_x) - (sin_yaw * local_y),
        "y": origin_y + (sin_yaw * local_x) + (cos_yaw * local_y),
    }


def _base64_value(value: object) -> str:
    if isinstance(value, bool):
        text = "true" if value else "false"
    else:
        text = str(value)
    return base64.b64encode(text.encode("utf-8")).decode("ascii")


def _property(key: str, value: object) -> dict[str, object]:
    payload: dict[str, object] = {
        "key": str(key),
        "value": _base64_value(value),
    }
    if isinstance(value, bool):
        payload.update({"type": "bool", "boolValue": value})
    elif isinstance(value, int):
        payload.update({"type": "int", "int32Value": value})
    elif isinstance(value, float) and math.isfinite(value):
        payload.update({"type": "double", "doubleValue": value})
    else:
        payload.update({"type": "string", "stringValue": str(value)})
    return payload


def _properties(values: Any) -> list[dict[str, object]]:
    if not isinstance(values, dict):
        return []
    return [
        _property(str(key), value)
        for key, value in values.items()
        if value is not None
        and isinstance(value, (str, int, float, bool))
        and not (isinstance(value, float) and not math.isfinite(value))
    ]


def _occupied_points(
    pixels: bytes,
    *,
    width: int,
    height: int,
    maximum: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    occupied_thresh: float,
    negate: bool,
) -> list[dict[str, float]]:
    threshold = occupied_thresh * maximum
    decimals = max(3, int(math.ceil(-math.log10(resolution))) + 2)
    points: list[dict[str, float]] = []
    for pixel_y in range(height):
        map_y = origin_y + ((height - 1 - pixel_y) * resolution)
        row = pixel_y * width
        for pixel_x in range(width):
            value = pixels[row + pixel_x]
            occupied = value >= threshold if negate else value <= maximum - threshold
            if not occupied:
                continue
            points.append(
                {
                    "x": round(origin_x + (pixel_x * resolution), decimals),
                    "y": round(map_y, decimals),
                }
            )
    return points


def _map_boundary_points(
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> list[dict[str, float]]:
    """Return a one-cell map frame for a completely empty occupancy grid.

    RDS/RoboShop derives part of the canvas extent from ``normalPosList``.
    An empty list makes some RDS versions collapse or distort one display
    axis even when ``header.maxPos`` is correct. A physical perimeter is also
    a safer representation of an open benchmark field: the graph remains
    unchanged and its outer LMs are one metre inside the frame.
    """
    decimals = max(3, int(math.ceil(-math.log10(resolution))) + 2)
    min_x = origin_x
    min_y = origin_y
    max_x = origin_x + ((width - 1) * resolution)
    max_y = origin_y + ((height - 1) * resolution)
    points: list[dict[str, float]] = []
    for pixel_x in range(width):
        x = round(origin_x + (pixel_x * resolution), decimals)
        points.append({"x": x, "y": round(min_y, decimals)})
        if height > 1:
            points.append({"x": x, "y": round(max_y, decimals)})
    for pixel_y in range(1, max(1, height - 1)):
        y = round(origin_y + (pixel_y * resolution), decimals)
        points.append({"x": round(min_x, decimals), "y": y})
        if width > 1:
            points.append({"x": round(max_x, decimals), "y": y})
    return points


def _advanced_points(
    lms_payload: Any,
    *,
    height_m: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> list[dict[str, object]]:
    raw_lms = lms_payload.get("LMs", []) if isinstance(lms_payload, dict) else []
    map_top_left = _coordinate_frame_is_map_top_left(lms_payload)
    points: list[dict[str, object]] = []
    for lm in raw_lms:
        if not isinstance(lm, dict):
            continue
        name = str(lm.get("name") or "").strip()
        if not name:
            continue
        pos = _map_to_ros_point(
            float(lm.get("x", 0.0) or 0.0),
            float(lm.get("y", 0.0) or 0.0),
            map_top_left=map_top_left,
            height_m=height_m,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        )
        point: dict[str, object] = {
            "className": "LocationMark",
            "instanceName": name,
            "pos": pos,
            "dir": float(lm.get("dir", 0.0) or 0.0),
            "property": _properties(lm.get("properties")),
        }
        if lm.get("ignoreDir") is not None:
            point["ignoreDir"] = lm["ignoreDir"]
        points.append(point)
    return points


def _advanced_curves(
    edges_payload: Any,
    lms_payload: Any,
    *,
    height_m: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> list[dict[str, object]]:
    raw_lms = lms_payload.get("LMs", []) if isinstance(lms_payload, dict) else []
    map_top_left = _coordinate_frame_is_map_top_left(lms_payload)
    landmarks = {
        str(lm.get("name") or ""): _map_to_ros_point(
            float(lm.get("x", 0.0) or 0.0),
            float(lm.get("y", 0.0) or 0.0),
            map_top_left=map_top_left,
            height_m=height_m,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        )
        for lm in raw_lms
        if isinstance(lm, dict) and str(lm.get("name") or "")
    }
    curves: list[dict[str, object]] = []
    for index, edge in enumerate(edges_payload if isinstance(edges_payload, list) else []):
        if not isinstance(edge, dict):
            continue
        start_name = str(edge.get("from") or "").strip()
        end_name = str(edge.get("to") or "").strip()
        start = landmarks.get(start_name)
        end = landmarks.get(end_name)
        if start is None or end is None or start_name == end_name:
            continue
        dx = end["x"] - start["x"]
        dy = end["y"] - start["y"]
        curves.append(
            {
                "className": "DegenerateBezier",
                "instanceName": (
                    f"{start_name}-{end_name}"
                    if start_name and end_name
                    else str(index)
                ),
                "startPos": {
                    "instanceName": start_name,
                    "pos": dict(start),
                },
                "controlPos1": {
                    "x": start["x"] + (dx / 3.0),
                    "y": start["y"] + (dy / 3.0),
                },
                "controlPos2": {
                    "x": start["x"] + ((2.0 * dx) / 3.0),
                    "y": start["y"] + ((2.0 * dy) / 3.0),
                },
                "endPos": {
                    "instanceName": end_name,
                    "pos": dict(end),
                },
                "property": _properties(edge.get("properties")),
            }
        )
    return curves


def _advanced_areas(
    zones_payload: Any,
    *,
    height_m: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> list[dict[str, object]]:
    zones = zones_payload.get("zones", []) if isinstance(zones_payload, dict) else []
    map_top_left = _coordinate_frame_is_map_top_left(zones_payload)
    areas: list[dict[str, object]] = []
    for zone in zones:
        if not isinstance(zone, dict):
            continue
        bounds = zone.get("bounds")
        if not isinstance(bounds, dict):
            continue
        try:
            min_x = float(bounds["minX"])
            min_y = float(bounds["minY"])
            max_x = float(bounds["maxX"])
            max_y = float(bounds["maxY"])
        except (KeyError, TypeError, ValueError):
            continue
        values: dict[str, object] = {
            "kind": str(zone.get("kind") or "traffic_zone"),
            "shape": str(zone.get("shape") or "rectangle"),
            "capacity": int(zone.get("capacity", 1) or 1),
            **(
                dict(zone.get("properties"))
                if isinstance(zone.get("properties"), dict)
                else {}
            ),
        }
        corners = [
            _map_to_ros_point(
                x,
                y,
                map_top_left=map_top_left,
                height_m=height_m,
                origin_x=origin_x,
                origin_y=origin_y,
                origin_yaw=origin_yaw,
            )
            for x, y in (
                (min_x, min_y),
                (max_x, min_y),
                (max_x, max_y),
                (min_x, max_y),
            )
        ]
        areas.append(
            {
                "className": "AdvancedArea",
                "instanceName": str(zone.get("id") or f"area-{len(areas) + 1}"),
                "dir": 0.0,
                "attribute": {
                    "colorBrush": 352299605,
                    "colorPen": 4294901845,
                },
                "posGroup": corners,
                "property": _properties(values),
            }
        )
    return areas


def serialize_smap_bundle(map_dir: Path, output_path: Path) -> dict[str, Any]:
    map_dir = Path(map_dir).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    if not map_dir.is_dir():
        raise ValueError(f"map bundle directory does not exist: {map_dir}")
    if output_path.exists() and output_path.is_dir():
        raise ValueError(f"output path is an existing directory: {output_path}")

    ros_yaml_path = _find_ros_yaml(map_dir)
    ros_map = _read_yaml(ros_yaml_path)
    if not isinstance(ros_map, dict):
        raise ValueError(f"invalid ROS map yaml: {ros_yaml_path}")
    resolution = float(ros_map.get("resolution", 0.05) or 0.05)
    origin = list(ros_map.get("origin") or [0.0, 0.0, 0.0])
    origin_x = float(origin[0] if len(origin) > 0 else 0.0)
    origin_y = float(origin[1] if len(origin) > 1 else 0.0)
    origin_yaw = float(origin[2] if len(origin) > 2 else 0.0)
    pgm_path = map_dir / str(ros_map.get("image") or "")
    width, height, maximum, pixels = _read_pgm(pgm_path)

    lms_payload = _read_yaml(map_dir / "LMs.yaml")
    edges_payload = _read_yaml(map_dir / "graph_edges_lengths.yaml")
    traffic_zones_path = map_dir / "traffic_zones.yaml"
    zones_payload = (
        _read_yaml(traffic_zones_path)
        if traffic_zones_path.exists()
        else {}
    )
    map_name = str(
        (
            lms_payload.get("mapName")
            if isinstance(lms_payload, dict)
            else ""
        )
        or ros_yaml_path.stem
    )
    occupied_points = _occupied_points(
        pixels,
        width=width,
        height=height,
        maximum=maximum,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        occupied_thresh=float(ros_map.get("occupied_thresh", 0.65) or 0.65),
        negate=bool(int(ros_map.get("negate", 0) or 0)),
    )
    if not occupied_points:
        occupied_points = _map_boundary_points(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )
    payload = {
        "header": {
            "mapType": "2D-Map",
            "mapName": map_name,
            "minPos": {"x": origin_x, "y": origin_y},
            # The inverse deserializer adds one raster cell to inclusive
            # bounds, so maxPos denotes the centre of the final PGM pixel.
            "maxPos": {
                "x": round(origin_x + ((width - 1) * resolution), 12),
                "y": round(origin_y + ((height - 1) * resolution), 12),
            },
            "resolution": resolution,
            "version": "1.0.6",
        },
        "normalPosList": occupied_points,
        "advancedPointList": _advanced_points(
            lms_payload,
            height_m=height * resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        ),
        # FeatureLine is rasterized as physical occupancy by the legacy
        # importer. Graph lanes are therefore encoded as straight degenerate
        # Beziers, matching real RoboShop route exports.
        "advancedLineList": [],
        "advancedCurveList": _advanced_curves(
            edges_payload,
            lms_payload,
            height_m=height * resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        ),
        "advancedAreaList": _advanced_areas(
            zones_payload,
            height_m=height * resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        ),
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    return {
        "mapName": map_name,
        "output": str(output_path),
        "width": width,
        "height": height,
        "resolution": resolution,
        "occupiedPoints": len(payload["normalPosList"]),
        "landmarks": len(payload["advancedPointList"]),
        "directedEdges": len(payload["advancedCurveList"]),
        "trafficZones": len(payload["advancedAreaList"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Serialize an unpacked operator map directory to JSON .smap",
    )
    parser.add_argument("map_dir", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    summary = serialize_smap_bundle(args.map_dir, args.output)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
