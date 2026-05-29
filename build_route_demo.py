#!/usr/bin/env python3
"""
Build a self-contained HTML demo for LM graph visualization and A* routing.

Example:
  python build_route_demo.py --map-dir maps_out/22.05.26_smap.smap --start LM91 --goal LM323
"""

from __future__ import annotations

import argparse
import base64
import heapq
import json
import math
import os
import struct
import zlib
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build an interactive route demo for a warehouse map."
    )
    parser.add_argument(
        "--map-dir",
        required=True,
        type=Path,
        help="Directory with map yaml/pgm, LMs.yaml and graph_edges_lengths.yaml.",
    )
    parser.add_argument(
        "--start",
        default=None,
        help="Default start LM shown when the demo opens.",
    )
    parser.add_argument(
        "--goal",
        default=None,
        help="Default target LM shown when the demo opens.",
    )
    parser.add_argument(
        "--output",
        default=None,
        type=Path,
        help="Output HTML path. Default: <map-dir>/route_demo.html",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the generated demo in the default browser after build.",
    )
    return parser.parse_args()


def read_yaml(path: Path) -> object:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def find_ros_map_yaml(map_dir: Path) -> Path:
    candidates = sorted(
        path
        for path in map_dir.glob("*.yaml")
        if path.name not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml"}
    )
    if not candidates:
        raise FileNotFoundError(f"No ROS map yaml found in {map_dir}")
    return candidates[0]


def _read_pgm_token(data: bytes, index: int) -> Tuple[bytes, int]:
    length = len(data)
    while index < length:
        byte = data[index]
        if byte == 35:  # '#'
            while index < length and data[index] not in (10, 13):
                index += 1
        elif chr(byte).isspace():
            index += 1
        else:
            break

    start = index
    while index < length and not chr(data[index]).isspace():
        index += 1

    return data[start:index], index


def load_pgm(path: Path) -> Tuple[int, int, bytes]:
    raw = path.read_bytes()
    magic, index = _read_pgm_token(raw, 0)
    if magic not in {b"P5", b"P2"}:
        raise ValueError(f"Unsupported PGM format in {path}: {magic!r}")

    width_token, index = _read_pgm_token(raw, index)
    height_token, index = _read_pgm_token(raw, index)
    max_value_token, index = _read_pgm_token(raw, index)
    width = int(width_token)
    height = int(height_token)
    max_value = int(max_value_token)

    while index < len(raw) and chr(raw[index]).isspace():
        index += 1

    if magic == b"P5":
        if max_value > 255:
            raise ValueError("Only 8-bit binary PGM files are supported.")
        pixels = raw[index : index + (width * height)]
        if len(pixels) != width * height:
            raise ValueError("PGM pixel data is shorter than expected.")
        return width, height, pixels

    text_values = raw[index:].split()
    if len(text_values) < width * height:
        raise ValueError("PGM ascii pixel data is shorter than expected.")

    scale = 255 / max_value if max_value else 1.0
    pixels = bytes(int(round(int(token) * scale)) for token in text_values[: width * height])
    return width, height, pixels


def build_grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    if len(pixels) != width * height:
        raise ValueError("Pixel buffer size does not match image dimensions.")

    def chunk(kind: bytes, payload: bytes) -> bytes:
        crc = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)

    rows = []
    row_size = width
    for y in range(height):
        start = y * row_size
        rows.append(b"\x00" + pixels[start : start + row_size])

    compressed = zlib.compress(b"".join(rows), level=9)
    png = bytearray()
    png.extend(b"\x89PNG\r\n\x1a\n")
    png.extend(
        chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0),
        )
    )
    png.extend(chunk(b"IDAT", compressed))
    png.extend(chunk(b"IEND", b""))
    return bytes(png)


def load_lms(path: Path) -> Dict[str, Dict[str, float]]:
    payload = read_yaml(path)
    if not isinstance(payload, dict) or "LMs" not in payload:
        raise ValueError(f"Unexpected LM file format: {path}")

    lms: Dict[str, Dict[str, float]] = {}
    for item in payload["LMs"]:
        name = str(item["name"])
        lms[name] = {
            "x": float(item["x"]),
            "y": float(item["y"]),
        }
    return lms


def load_edges(path: Path, lms: Dict[str, Dict[str, float]]) -> List[Dict[str, object]]:
    payload = read_yaml(path)
    if not isinstance(payload, list):
        raise ValueError(f"Unexpected edge file format: {path}")

    geometries = load_graph_geometries(path.parent / "graphs.yaml")
    edges: List[Dict[str, object]] = []
    for item in payload:
        start = str(item["from"])
        goal = str(item["to"])
        if start not in lms or goal not in lms:
            continue

        start_xy = lms[start]
        goal_xy = lms[goal]
        edges.append(
            {
                "from": start,
                "to": goal,
                "length": float(item["length"]),
                "kind": str(item.get("kind", "unknown")),
                "type": str(item.get("type", "unknown")),
                "world_points": [
                    {"x": start_xy["x"], "y": start_xy["y"]},
                    {"x": goal_xy["x"], "y": goal_xy["y"]},
                ],
            }
        )
        geometry = geometries.get((start, goal))
        if geometry:
            edges[-1].update(geometry)
    return edges


def load_graph_geometries(path: Path) -> Dict[Tuple[str, str], Dict[str, object]]:
    if not path.exists():
        return {}

    payload = read_yaml(path)
    if not isinstance(payload, dict):
        return {}

    primitives = payload.get("primitives", [])
    if not isinstance(primitives, list):
        return {}

    geometries: Dict[Tuple[str, str], Dict[str, object]] = {}
    for primitive in primitives:
        if not isinstance(primitive, dict) or primitive.get("kind") != "curve":
            continue

        curve = primitive.get("curve")
        if not isinstance(curve, dict):
            continue

        start_name = curve.get("start_name")
        end_name = curve.get("end_name")
        if not start_name or not end_name:
            continue

        point_keys = ("start", "control1", "control2", "end")
        try:
            control_points = [
                {
                    "x": float(curve[key]["x"]),
                    "y": float(curve[key]["y"]),
                }
                for key in point_keys
            ]
        except (KeyError, TypeError, ValueError):
            continue

        geometry = {
            "geometry": "bezier",
            "control_points": control_points,
            "curve_type": str(primitive.get("curve_type", "Bezier")),
        }
        geometries[(str(start_name), str(end_name))] = geometry
        geometries[(str(end_name), str(start_name))] = {
            **geometry,
            "control_points": list(reversed(control_points)),
        }

    return geometries


def world_distance(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


def astar(
    start: str,
    goal: str,
    lms: Dict[str, Dict[str, float]],
    edges: Sequence[Dict[str, object]],
) -> Tuple[List[str], float]:
    adjacency: Dict[str, List[Tuple[str, float]]] = {}
    for edge in edges:
        adjacency.setdefault(str(edge["from"]), []).append(
            (str(edge["to"]), float(edge["length"]))
        )

    if start not in lms:
        raise ValueError(f"Unknown start LM: {start}")
    if goal not in lms:
        raise ValueError(f"Unknown goal LM: {goal}")

    open_heap: List[Tuple[float, str]] = [(0.0, start)]
    came_from: Dict[str, str] = {}
    g_score: Dict[str, float] = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            route = [current]
            while current in came_from:
                current = came_from[current]
                route.append(current)
            route.reverse()
            return route, g_score[goal]

        for neighbor, cost in adjacency.get(current, []):
            tentative = g_score[current] + cost
            if tentative >= g_score.get(neighbor, math.inf):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative
            heuristic = world_distance(lms[neighbor], lms[goal])
            heapq.heappush(open_heap, (tentative + heuristic, neighbor))

    raise ValueError(f"No route found from {start} to {goal}")


def pick_defaults(
    lms: Dict[str, Dict[str, float]],
    requested_start: str | None,
    requested_goal: str | None,
) -> Tuple[str, str]:
    names = sorted(lms)
    if not names:
        raise ValueError("No LMs were found.")

    start = requested_start or names[0]
    goal = requested_goal or names[-1]

    if start not in lms:
        raise ValueError(f"Default start LM does not exist: {start}")
    if goal not in lms:
        raise ValueError(f"Default goal LM does not exist: {goal}")

    return start, goal


def unique_edges(edges: Iterable[Dict[str, object]]) -> List[Dict[str, object]]:
    seen = set()
    unique: List[Dict[str, object]] = []
    for edge in edges:
        key = tuple(sorted((str(edge["from"]), str(edge["to"]))))
        if key in seen:
            continue
        seen.add(key)
        unique.append(edge)
    return unique


def build_demo_html(
    *,
    map_name: str,
    image_png_base64: str,
    map_width: int,
    map_height: int,
    resolution: float,
    origin: Sequence[float],
    lms: Dict[str, Dict[str, float]],
    edges: Sequence[Dict[str, object]],
    default_start: str,
    default_goal: str,
) -> str:
    view_padding = 36
    view_width = map_width + (view_padding * 2)
    view_height = map_height + (view_padding * 2)
    payload = {
        "mapName": map_name,
        "map": {
            "width": map_width,
            "height": map_height,
            "viewPadding": view_padding,
            "viewWidth": view_width,
            "viewHeight": view_height,
            "resolution": resolution,
            "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
            "imageDataUrl": f"data:image/png;base64,{image_png_base64}",
        },
        "lms": [{"name": name, **coords} for name, coords in sorted(lms.items())],
        "edges": list(edges),
        "defaultStart": default_start,
        "defaultGoal": default_goal,
    }

    payload_json = json.dumps(payload, ensure_ascii=False).replace("</script>", "<\\/script>")
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>{map_name} Route Demo</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f1efe6;
      --panel: rgba(255, 252, 245, 0.94);
      --ink: #20242c;
      --muted: #586174;
      --edge: rgba(73, 82, 95, 0.42);
      --point: #9b2c2c;
      --route: #1664d9;
      --start: #1e8e3e;
      --goal: #d9480f;
      --robot: #0b7285;
      --footprint: rgba(11, 114, 133, 0.24);
      --blocked: #c92a2a;
      --obstacle: #5f3dc4;
      --lookahead: rgba(201, 42, 42, 0.18);
      --shadow: 0 18px 40px rgba(48, 54, 61, 0.12);
    }}

    * {{
      box-sizing: border-box;
    }}

    body {{
      margin: 0;
      font-family: "Segoe UI", Tahoma, sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(255, 255, 255, 0.9), rgba(255, 255, 255, 0) 32%),
        linear-gradient(135deg, #e9e3d4 0%, #f7f4ed 52%, #ebe7db 100%);
      min-height: 100vh;
    }}

    .shell {{
      display: grid;
      grid-template-columns: 360px minmax(0, 1fr);
      gap: 20px;
      padding: 20px;
      min-height: 100vh;
    }}

    .panel {{
      background: var(--panel);
      border: 1px solid rgba(32, 36, 44, 0.08);
      border-radius: 22px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(10px);
      padding: 20px;
    }}

    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
      line-height: 1.1;
    }}

    .subtitle {{
      margin: 0 0 18px;
      color: var(--muted);
      line-height: 1.45;
    }}

    .controls {{
      display: grid;
      gap: 14px;
    }}

    label {{
      display: grid;
      gap: 6px;
      font-size: 14px;
      color: var(--muted);
    }}

    select,
    input,
    button {{
      width: 100%;
      border-radius: 14px;
      border: 1px solid rgba(32, 36, 44, 0.14);
      padding: 12px 14px;
      font: inherit;
      background: #fffdf9;
      color: var(--ink);
    }}

    input[type="number"] {{
      appearance: textfield;
    }}

    .control-grid {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }}

    button {{
      cursor: pointer;
      transition: transform 140ms ease, box-shadow 140ms ease, background 140ms ease;
      font-weight: 600;
    }}

    button:hover {{
      transform: translateY(-1px);
      box-shadow: 0 10px 18px rgba(22, 100, 217, 0.12);
    }}

    button.primary {{
      background: linear-gradient(135deg, #2b6de0, #1755b7);
      color: white;
      border-color: transparent;
    }}

    button.secondary {{
      background: linear-gradient(135deg, #f7f2e7, #efe8d8);
    }}

    button.warning {{
      background: #fff5f5;
      color: var(--blocked);
      border-color: rgba(201, 42, 42, 0.22);
    }}

    .stats {{
      display: grid;
      gap: 10px;
      margin-top: 18px;
    }}

    .card {{
      border-radius: 16px;
      background: rgba(255, 255, 255, 0.72);
      border: 1px solid rgba(32, 36, 44, 0.07);
      padding: 14px;
    }}

    .card-title {{
      font-size: 12px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--muted);
      margin-bottom: 8px;
    }}

    .metric {{
      font-size: 24px;
      font-weight: 700;
    }}

    .route-list {{
      margin: 0;
      padding: 0;
      list-style: none;
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }}

    .route-list li {{
      border-radius: 999px;
      padding: 6px 10px;
      background: rgba(22, 100, 217, 0.1);
      color: #0d4ba8;
      font-size: 13px;
    }}

    .viewer {{
      position: relative;
      background: rgba(255, 255, 255, 0.55);
      border-radius: 26px;
      box-shadow: var(--shadow);
      overflow: hidden;
      min-height: 72vh;
      border: 1px solid rgba(32, 36, 44, 0.08);
    }}

    .viewer-toolbar {{
      position: absolute;
      top: 16px;
      right: 16px;
      z-index: 3;
      display: flex;
      gap: 8px;
    }}

    .viewer-toolbar button {{
      width: auto;
      min-width: 88px;
      padding: 10px 12px;
      background: rgba(255, 255, 255, 0.9);
    }}

    svg {{
      display: block;
      width: 100%;
      height: 100%;
      background: linear-gradient(180deg, rgba(255,255,255,0.25), rgba(255,255,255,0.04));
      user-select: none;
    }}

    .legend {{
      margin-top: 18px;
      display: grid;
      gap: 10px;
      color: var(--muted);
      font-size: 14px;
    }}

    .lm-label {{
      font-size: 5.5px;
      fill: #243041;
      text-anchor: middle;
      paint-order: stroke;
      stroke: rgba(255, 255, 255, 0.92);
      stroke-width: 3px;
      stroke-linejoin: round;
      font-weight: 600;
      pointer-events: none;
    }}

    .legend-row {{
      display: flex;
      align-items: center;
      gap: 10px;
    }}

    .swatch {{
      width: 18px;
      height: 18px;
      border-radius: 999px;
      border: 1px solid rgba(0, 0, 0, 0.14);
      flex: 0 0 auto;
    }}

    @media (max-width: 1100px) {{
      .shell {{
        grid-template-columns: 1fr;
      }}

      .viewer {{
        min-height: 62vh;
      }}
    }}
  </style>
</head>
<body>
  <div class="shell">
    <aside class="panel">
      <h1>{map_name}</h1>
      <p class="subtitle">
        Демо визуализация LM-графа поверх карты. Маршрут строится через A* и
        подсвечивается синим, а робот едет строго от LM к LM.
      </p>

      <div class="controls">
        <label>
          Start LM
          <select id="startSelect"></select>
        </label>
        <label>
          Goal LM
          <select id="goalSelect"></select>
        </label>
        <button id="planButton" class="primary" type="button">Plan A* Route</button>
        <button id="driveButton" class="secondary" type="button">Drive Along Path</button>
        <button id="stopButton" class="warning" type="button">Stop</button>
        <button id="obstacleModeButton" class="secondary" type="button">Add Obstacles: Off</button>
        <button id="clearObstaclesButton" class="secondary" type="button">Clear Obstacles</button>
        <div class="control-grid">
          <label>
            Robot W, m
            <input id="robotWidthInput" type="number" value="0.55" min="0.10" max="2.00" step="0.01" />
          </label>
          <label>
            Robot L, m
            <input id="robotLengthInput" type="number" value="0.70" min="0.10" max="2.50" step="0.01" />
          </label>
          <label>
            Lookahead, m
            <input id="lookaheadInput" type="number" value="0.80" min="0.10" max="3.00" step="0.05" />
          </label>
          <label>
            Speed, m/s
            <input id="speedInput" type="number" value="0.35" min="0.02" max="1.50" step="0.01" />
          </label>
        </div>
      </div>

      <div class="stats">
        <div class="card">
          <div class="card-title">Route Length</div>
          <div class="metric" id="routeLength">0.00 m</div>
        </div>
        <div class="card">
          <div class="card-title">Visited LM</div>
          <ul id="routeList" class="route-list"></ul>
        </div>
        <div class="card">
          <div class="card-title">Status</div>
          <div id="statusText">Ready.</div>
        </div>
      </div>

      <div class="legend">
        <div class="legend-row"><span class="swatch" style="background: var(--edge);"></span> All graph edges</div>
        <div class="legend-row"><span class="swatch" style="background: var(--route);"></span> A* shortest route</div>
        <div class="legend-row"><span class="swatch" style="background: var(--start);"></span> Start LM</div>
        <div class="legend-row"><span class="swatch" style="background: var(--goal);"></span> Goal LM</div>
        <div class="legend-row"><span class="swatch" style="background: var(--robot);"></span> Robot position</div>
        <div class="legend-row"><span class="swatch" style="background: var(--obstacle);"></span> Lidar obstacle point</div>
        <div class="legend-row"><span class="swatch" style="background: var(--lookahead);"></span> Swept footprint check</div>
      </div>
    </aside>

    <main class="viewer">
      <div class="viewer-toolbar">
        <button id="zoomInButton" type="button">Zoom In</button>
        <button id="zoomOutButton" type="button">Zoom Out</button>
        <button id="resetViewButton" type="button">Reset</button>
      </div>
      <svg id="mapSvg" viewBox="0 0 {view_width} {view_height}">
        <g id="viewport">
          <image id="mapImage" x="{view_padding}" y="{view_padding}" width="{map_width}" height="{map_height}" href="" preserveAspectRatio="none"></image>
          <g id="graphLayer"></g>
          <g id="pathLayer"></g>
          <g id="lookaheadLayer"></g>
          <g id="obstacleLayer"></g>
          <g id="pointLayer"></g>
          <g id="robotLayer"></g>
        </g>
      </svg>
    </main>
  </div>

  <script>
    const DEMO_DATA = {payload_json};

    const svg = document.getElementById("mapSvg");
    const viewport = document.getElementById("viewport");
    const graphLayer = document.getElementById("graphLayer");
    const pathLayer = document.getElementById("pathLayer");
    const lookaheadLayer = document.getElementById("lookaheadLayer");
    const obstacleLayer = document.getElementById("obstacleLayer");
    const pointLayer = document.getElementById("pointLayer");
    const robotLayer = document.getElementById("robotLayer");
    const routeLength = document.getElementById("routeLength");
    const routeList = document.getElementById("routeList");
    const statusText = document.getElementById("statusText");
    const startSelect = document.getElementById("startSelect");
    const goalSelect = document.getElementById("goalSelect");
    const planButton = document.getElementById("planButton");
    const driveButton = document.getElementById("driveButton");
    const stopButton = document.getElementById("stopButton");
    const obstacleModeButton = document.getElementById("obstacleModeButton");
    const clearObstaclesButton = document.getElementById("clearObstaclesButton");
    const robotWidthInput = document.getElementById("robotWidthInput");
    const robotLengthInput = document.getElementById("robotLengthInput");
    const lookaheadInput = document.getElementById("lookaheadInput");
    const speedInput = document.getElementById("speedInput");
    const zoomInButton = document.getElementById("zoomInButton");
    const zoomOutButton = document.getElementById("zoomOutButton");
    const resetViewButton = document.getElementById("resetViewButton");
    const mapImage = document.getElementById("mapImage");

    const nodeByName = new Map();
    const adjacency = new Map();
    const edgeByKey = new Map();
    const scaleState = {{ zoom: 1, panX: 0, panY: 0 }};
    const baseView = {{
      width: DEMO_DATA.map.viewWidth,
      height: DEMO_DATA.map.viewHeight,
    }};

    let currentPath = [];
    let currentTrajectory = [];
    let animationFrame = null;
    let robotShape = null;
    let obstacleMode = false;
    let obstacles = [];
    let simulation = null;

    mapImage.setAttribute("href", DEMO_DATA.map.imageDataUrl);

    function worldToPixel(point) {{
      const px = DEMO_DATA.map.viewPadding + ((point.x - DEMO_DATA.map.origin[0]) / DEMO_DATA.map.resolution);
      const py = DEMO_DATA.map.viewPadding + (DEMO_DATA.map.height - 1) - ((point.y - DEMO_DATA.map.origin[1]) / DEMO_DATA.map.resolution);
      return {{ x: px, y: py }};
    }}

    function pixelToWorld(point) {{
      const x = ((point.x - DEMO_DATA.map.viewPadding) * DEMO_DATA.map.resolution) + DEMO_DATA.map.origin[0];
      const y = ((DEMO_DATA.map.height - 1) - (point.y - DEMO_DATA.map.viewPadding)) * DEMO_DATA.map.resolution + DEMO_DATA.map.origin[1];
      return {{ x, y }};
    }}

    function eventToWorld(event) {{
      const ctm = viewport.getScreenCTM();
      if (!ctm) {{
        return null;
      }}
      const point = new DOMPoint(event.clientX, event.clientY).matrixTransform(ctm.inverse());
      return pixelToWorld(point);
    }}

    function distWorld(a, b) {{
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      return Math.hypot(dx, dy);
    }}

    function ensureAdjacency(name) {{
      if (!adjacency.has(name)) {{
        adjacency.set(name, []);
      }}
      return adjacency.get(name);
    }}

    function buildData() {{
      for (const lm of DEMO_DATA.lms) {{
        nodeByName.set(lm.name, lm);
        ensureAdjacency(lm.name);
      }}

      for (const edge of DEMO_DATA.edges) {{
        edgeByKey.set(`${{edge.from}}|${{edge.to}}`, edge);
        ensureAdjacency(edge.from).push({{
          to: edge.to,
          length: edge.length,
          edge,
        }});
      }}
    }}

    function populateSelectors() {{
      for (const lm of DEMO_DATA.lms) {{
        const startOption = document.createElement("option");
        startOption.value = lm.name;
        startOption.textContent = lm.name;
        startSelect.appendChild(startOption);

        const goalOption = document.createElement("option");
        goalOption.value = lm.name;
        goalOption.textContent = lm.name;
        goalSelect.appendChild(goalOption);
      }}

      startSelect.value = DEMO_DATA.defaultStart;
      goalSelect.value = DEMO_DATA.defaultGoal;
    }}

    function createSvgElement(tag, attrs) {{
      const element = document.createElementNS("http://www.w3.org/2000/svg", tag);
      for (const [key, value] of Object.entries(attrs)) {{
        element.setAttribute(key, String(value));
      }}
      return element;
    }}

    function drawGraph() {{
      graphLayer.innerHTML = "";
      const seen = new Set();

      for (const edge of DEMO_DATA.edges) {{
        const key = [edge.from, edge.to].sort().join("|");
        if (seen.has(key)) {{
          continue;
        }}
        seen.add(key);

        const start = worldToPixel(nodeByName.get(edge.from));
        const goal = worldToPixel(nodeByName.get(edge.to));
        if (edge.geometry === "bezier" && edge.control_points && edge.control_points.length === 4) {{
          const cp = edge.control_points.map(worldToPixel);
          graphLayer.appendChild(
            createSvgElement("path", {{
              d: `M ${{cp[0].x}} ${{cp[0].y}} C ${{cp[1].x}} ${{cp[1].y}}, ${{cp[2].x}} ${{cp[2].y}}, ${{cp[3].x}} ${{cp[3].y}}`,
              fill: "none",
              stroke: "var(--edge)",
              "stroke-width": 2,
              "stroke-linecap": "round",
            }})
          );
        }} else {{
          graphLayer.appendChild(
            createSvgElement("line", {{
              x1: start.x,
              y1: start.y,
              x2: goal.x,
              y2: goal.y,
              stroke: "var(--edge)",
              "stroke-width": 2,
              "stroke-linecap": "round",
            }})
          );
        }}
      }}
    }}

    function drawPoints() {{
      pointLayer.innerHTML = "";
      const startName = startSelect.value;
      const goalName = goalSelect.value;

      for (const lm of DEMO_DATA.lms) {{
        const pos = worldToPixel(lm);
        let fill = "var(--point)";
        let radius = 4;

        if (lm.name === startName) {{
          fill = "var(--start)";
          radius = 7;
        }} else if (lm.name === goalName) {{
          fill = "var(--goal)";
          radius = 7;
        }}

        pointLayer.appendChild(
          createSvgElement("circle", {{
            cx: pos.x,
            cy: pos.y,
            r: radius,
            fill,
            opacity: 0.95,
          }})
        );

        const label = createSvgElement("text", {{
          x: pos.x,
          y: pos.y + radius + 12,
          class: "lm-label",
        }});
        label.textContent = lm.name;
        pointLayer.appendChild(label);
      }}
    }}

    function heuristic(nameA, nameB) {{
      return distWorld(nodeByName.get(nameA), nodeByName.get(nameB));
    }}

    function findRoute(startName, goalName) {{
      const open = [{{ name: startName, f: 0 }}];
      const cameFrom = new Map();
      const gScore = new Map([[startName, 0]]);

      while (open.length > 0) {{
        open.sort((a, b) => a.f - b.f);
        const current = open.shift().name;

        if (current === goalName) {{
          const route = [current];
          let cursor = current;
          while (cameFrom.has(cursor)) {{
            cursor = cameFrom.get(cursor);
            route.push(cursor);
          }}
          route.reverse();
          return {{
            nodes: route,
            length: gScore.get(goalName),
          }};
        }}

        for (const edge of adjacency.get(current) || []) {{
          const tentative = gScore.get(current) + edge.length;
          const known = gScore.has(edge.to) ? gScore.get(edge.to) : Number.POSITIVE_INFINITY;
          if (tentative >= known) {{
            continue;
          }}

          cameFrom.set(edge.to, current);
          gScore.set(edge.to, tentative);
          open.push({{
            name: edge.to,
            f: tentative + heuristic(edge.to, goalName),
          }});
        }}
      }}

      return null;
    }}

    function updateRouteInfo(route) {{
      routeLength.textContent = route ? `${{route.length.toFixed(2)}} m` : "No route";
      routeList.innerHTML = "";
      if (!route) {{
        return;
      }}

      for (const name of route.nodes) {{
        const item = document.createElement("li");
        item.textContent = name;
        routeList.appendChild(item);
      }}
    }}

    function getEdge(fromName, toName) {{
      return edgeByKey.get(`${{fromName}}|${{toName}}`) || null;
    }}

    function cubicBezier(points, t) {{
      const u = 1 - t;
      const tt = t * t;
      const uu = u * u;
      const uuu = uu * u;
      const ttt = tt * t;
      return {{
        x: (uuu * points[0].x) + (3 * uu * t * points[1].x) + (3 * u * tt * points[2].x) + (ttt * points[3].x),
        y: (uuu * points[0].y) + (3 * uu * t * points[1].y) + (3 * u * tt * points[2].y) + (ttt * points[3].y),
      }};
    }}

    function cubicBezierDerivative(points, t) {{
      const u = 1 - t;
      return {{
        x: (3 * u * u * (points[1].x - points[0].x)) + (6 * u * t * (points[2].x - points[1].x)) + (3 * t * t * (points[3].x - points[2].x)),
        y: (3 * u * u * (points[1].y - points[0].y)) + (6 * u * t * (points[2].y - points[1].y)) + (3 * t * t * (points[3].y - points[2].y)),
      }};
    }}

    function sampleLine(start, goal, edgeId, spacing) {{
      const length = Math.max(spacing, distWorld(start, goal));
      const steps = Math.max(2, Math.ceil(length / spacing));
      const yaw = Math.atan2(goal.y - start.y, goal.x - start.x);
      const samples = [];
      for (let i = 0; i <= steps; i += 1) {{
        const t = i / steps;
        samples.push({{
          x: start.x + ((goal.x - start.x) * t),
          y: start.y + ((goal.y - start.y) * t),
          yaw,
          edgeId,
        }});
      }}
      return samples;
    }}

    function sampleBezier(points, edgeId, spacing) {{
      const roughLength = points.reduce((total, point, index) => {{
        if (index === 0) {{
          return 0;
        }}
        return total + distWorld(points[index - 1], point);
      }}, 0);
      const steps = Math.max(12, Math.ceil(roughLength / spacing));
      const samples = [];
      for (let i = 0; i <= steps; i += 1) {{
        const t = i / steps;
        const point = cubicBezier(points, t);
        const tangent = cubicBezierDerivative(points, t);
        samples.push({{
          x: point.x,
          y: point.y,
          yaw: Math.atan2(tangent.y, tangent.x),
          edgeId,
        }});
      }}
      return samples;
    }}

    function buildTrajectory(route) {{
      if (!route || route.nodes.length < 2) {{
        return [];
      }}

      const spacing = 0.05;
      const trajectory = [];
      for (let i = 0; i < route.nodes.length - 1; i += 1) {{
        const fromName = route.nodes[i];
        const toName = route.nodes[i + 1];
        const edge = getEdge(fromName, toName);
        const edgeId = `${{fromName}}->${{toName}}`;
        let samples;
        if (edge && edge.geometry === "bezier" && edge.control_points && edge.control_points.length === 4) {{
          samples = sampleBezier(edge.control_points, edgeId, spacing);
        }} else {{
          samples = sampleLine(nodeByName.get(fromName), nodeByName.get(toName), edgeId, spacing);
        }}

        if (trajectory.length > 0) {{
          samples = samples.slice(1);
        }}
        trajectory.push(...samples);
      }}

      let distance = 0;
      for (let i = 0; i < trajectory.length; i += 1) {{
        if (i > 0) {{
          distance += distWorld(trajectory[i - 1], trajectory[i]);
        }}
        trajectory[i].s = distance;
        trajectory[i].targetSpeed = Number(speedInput.value) || 0.35;
      }}
      return trajectory;
    }}

    function drawRoute(route) {{
      pathLayer.innerHTML = "";
      currentTrajectory = buildTrajectory(route);
      if (!route || route.nodes.length < 2 || currentTrajectory.length < 2) {{
        return;
      }}

      const points = currentTrajectory.map((pose) => {{
        const pos = worldToPixel(pose);
        return `${{pos.x}},${{pos.y}}`;
      }});

      pathLayer.appendChild(
        createSvgElement("polyline", {{
          points: points.join(" "),
          fill: "none",
          stroke: "var(--route)",
          "stroke-width": 6,
          "stroke-linecap": "round",
          "stroke-linejoin": "round",
          opacity: 0.92,
        }})
      );
    }}

    function stopAnimation() {{
      if (animationFrame !== null) {{
        cancelAnimationFrame(animationFrame);
        animationFrame = null;
      }}
      simulation = null;
      lookaheadLayer.innerHTML = "";
    }}

    function robotConfig() {{
      return {{
        width: Math.max(0.1, Number(robotWidthInput.value) || 0.55),
        length: Math.max(0.1, Number(robotLengthInput.value) || 0.70),
        lookahead: Math.max(0.1, Number(lookaheadInput.value) || 0.8),
      }};
    }}

    function footprintCorners(pose) {{
      const cfg = robotConfig();
      const halfLength = cfg.length / 2;
      const halfWidth = cfg.width / 2;
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);
      return [
        {{ x: halfLength, y: halfWidth }},
        {{ x: halfLength, y: -halfWidth }},
        {{ x: -halfLength, y: -halfWidth }},
        {{ x: -halfLength, y: halfWidth }},
      ].map((corner) => ({{
        x: pose.x + (corner.x * cos) - (corner.y * sin),
        y: pose.y + (corner.x * sin) + (corner.y * cos),
      }}));
    }}

    function drawFootprint(pose, attrs = {{}}) {{
      const points = footprintCorners(pose)
        .map(worldToPixel)
        .map((point) => `${{point.x}},${{point.y}}`)
        .join(" ");
      return createSvgElement("polygon", {{
        points,
        fill: attrs.fill || "var(--footprint)",
        stroke: attrs.stroke || "var(--robot)",
        "stroke-width": attrs.strokeWidth || 2,
        "stroke-linejoin": "round",
        opacity: attrs.opacity || 1,
      }});
    }}

    function drawRobotPose(pose, blocked = false) {{
      robotLayer.innerHTML = "";
      if (!pose) {{
        robotShape = null;
        return;
      }}

      robotShape = drawFootprint(pose, {{
        fill: blocked ? "rgba(201, 42, 42, 0.25)" : "var(--footprint)",
        stroke: blocked ? "var(--blocked)" : "var(--robot)",
        strokeWidth: 2.5,
      }});
      robotLayer.appendChild(robotShape);

      const nose = worldToPixel({{
        x: pose.x + Math.cos(pose.yaw) * (robotConfig().length / 2),
        y: pose.y + Math.sin(pose.yaw) * (robotConfig().length / 2),
      }});
      const center = worldToPixel(pose);
      robotLayer.appendChild(createSvgElement("line", {{
        x1: center.x,
        y1: center.y,
        x2: nose.x,
        y2: nose.y,
        stroke: blocked ? "var(--blocked)" : "var(--robot)",
        "stroke-width": 3,
        "stroke-linecap": "round",
      }}));
      robotLayer.appendChild(createSvgElement("circle", {{
        cx: center.x,
        cy: center.y,
        r: 3.5,
        fill: blocked ? "var(--blocked)" : "var(--robot)",
      }}));
    }}

    function drawRobotAt(name) {{
      const lm = nodeByName.get(name);
      if (!lm) {{
        drawRobotPose(null);
        return;
      }}
      const nextLm = currentPath.length > 1 ? nodeByName.get(currentPath[1]) : null;
      const yaw = nextLm ? Math.atan2(nextLm.y - lm.y, nextLm.x - lm.x) : 0;
      drawRobotPose({{ x: lm.x, y: lm.y, yaw }});
    }}

    function drawObstacles() {{
      obstacleLayer.innerHTML = "";
      obstacles.forEach((obstacle, index) => {{
        const pos = worldToPixel(obstacle);
        obstacleLayer.appendChild(createSvgElement("circle", {{
          cx: pos.x,
          cy: pos.y,
          r: Math.max(4, obstacle.radius / DEMO_DATA.map.resolution),
          fill: "var(--obstacle)",
          stroke: "white",
          "stroke-width": 2,
          opacity: 0.88,
          "data-index": index,
        }}));
      }});
    }}

    function obstacleHitsPose(obstacle, pose) {{
      const cfg = robotConfig();
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);
      const dx = obstacle.x - pose.x;
      const dy = obstacle.y - pose.y;
      const localX = (dx * cos) + (dy * sin);
      const localY = (-dx * sin) + (dy * cos);
      const radius = obstacle.radius || 0.08;
      return (
        Math.abs(localX) <= (cfg.length / 2) + radius &&
        Math.abs(localY) <= (cfg.width / 2) + radius
      );
    }}

    function drawLookahead(poses, blocked) {{
      lookaheadLayer.innerHTML = "";
      for (let i = 0; i < poses.length; i += Math.max(1, Math.floor(poses.length / 8))) {{
        lookaheadLayer.appendChild(drawFootprint(poses[i], {{
          fill: blocked ? "var(--lookahead)" : "rgba(11, 114, 133, 0.08)",
          stroke: blocked ? "var(--blocked)" : "rgba(11, 114, 133, 0.22)",
          strokeWidth: 1,
          opacity: 0.8,
        }}));
      }}
    }}

    function collisionAhead(trajectory, index) {{
      const cfg = robotConfig();
      const startDistance = trajectory[index].s;
      const poses = [];
      for (let i = index; i < trajectory.length; i += 1) {{
        const pose = trajectory[i];
        if (pose.s - startDistance > cfg.lookahead) {{
          break;
        }}
        poses.push(pose);
        for (const obstacle of obstacles) {{
          if (obstacleHitsPose(obstacle, pose)) {{
            drawLookahead(poses, true);
            return {{ blocked: true, obstacle, poses }};
          }}
        }}
      }}
      drawLookahead(poses, false);
      return {{ blocked: false, obstacle: null, poses }};
    }}

    function poseAtDistance(trajectory, distance) {{
      if (distance <= 0) {{
        return trajectory[0];
      }}
      const last = trajectory[trajectory.length - 1];
      if (distance >= last.s) {{
        return last;
      }}

      let index = 0;
      while (index < trajectory.length - 2 && trajectory[index + 1].s < distance) {{
        index += 1;
      }}

      const start = trajectory[index];
      const goal = trajectory[index + 1];
      const span = Math.max(0.0001, goal.s - start.s);
      const t = (distance - start.s) / span;
      return {{
        x: start.x + ((goal.x - start.x) * t),
        y: start.y + ((goal.y - start.y) * t),
        yaw: start.yaw + ((goal.yaw - start.yaw) * t),
        s: distance,
        edgeId: start.edgeId,
      }};
    }}

    function animateRoute(route) {{
      const trajectory = currentTrajectory.length ? currentTrajectory : buildTrajectory(route);
      if (!route || route.nodes.length < 2 || trajectory.length < 2) {{
        statusText.textContent = "Route is empty.";
        return;
      }}

      stopAnimation();
      const speedMetersPerSec = Math.max(0.02, Number(speedInput.value) || 0.35);
      simulation = {{
        trajectory,
        index: 0,
        s: 0,
        lastTs: null,
        paused: false,
      }};
      statusText.textContent = `Driving from ${{route.nodes[0]}} to ${{route.nodes[route.nodes.length - 1]}}`;

      function step(ts) {{
        if (!simulation) {{
          return;
        }}

        if (simulation.s >= trajectory[trajectory.length - 1].s) {{
          statusText.textContent = `Arrived at ${{route.nodes[route.nodes.length - 1]}}`;
          drawRobotPose(trajectory[trajectory.length - 1]);
          animationFrame = null;
          simulation = null;
          lookaheadLayer.innerHTML = "";
          return;
        }}

        const check = collisionAhead(trajectory, simulation.index);
        if (check.blocked) {{
          statusText.textContent = "WAIT_BLOCKED: obstacle intersects swept footprint.";
          drawRobotPose(poseAtDistance(trajectory, simulation.s), true);
          simulation.lastTs = ts;
          animationFrame = requestAnimationFrame(step);
          return;
        }}

        if (simulation.lastTs === null) {{
          simulation.lastTs = ts;
        }}

        const dt = Math.min(0.08, Math.max(0, (ts - simulation.lastTs) / 1000));
        simulation.s = Math.min(trajectory[trajectory.length - 1].s, simulation.s + (speedMetersPerSec * dt));
        while (simulation.index < trajectory.length - 2 && trajectory[simulation.index + 1].s < simulation.s) {{
          simulation.index += 1;
        }}

        simulation.lastTs = ts;
        const pose = poseAtDistance(trajectory, simulation.s);
        drawRobotPose(pose);
        statusText.textContent = `Driving ${{trajectory[simulation.index].edgeId}}`;
        animationFrame = requestAnimationFrame(step);
      }}

      animationFrame = requestAnimationFrame(step);
    }}

    function planRoute() {{
      stopAnimation();
      drawPoints();
      const startName = startSelect.value;
      const goalName = goalSelect.value;
      const route = findRoute(startName, goalName);
      currentPath = route ? route.nodes : [];
      updateRouteInfo(route);
      drawRoute(route);
      drawRobotAt(startName);

      if (route) {{
        statusText.textContent = `Route planned: ${{startName}} -> ${{goalName}}`;
      }} else {{
        statusText.textContent = `No route found from ${{startName}} to ${{goalName}}`;
      }}
    }}

    function applyTransform() {{
      viewport.setAttribute(
        "transform",
        `translate(${{scaleState.panX}} ${{scaleState.panY}}) scale(${{scaleState.zoom}})`
      );
    }}

    function zoom(multiplier) {{
      scaleState.zoom = Math.min(6, Math.max(0.5, scaleState.zoom * multiplier));
      applyTransform();
    }}

    function resetView() {{
      scaleState.zoom = 1;
      scaleState.panX = 0;
      scaleState.panY = 0;
      applyTransform();
    }}

    function enableDrag() {{
      let active = false;
      let lastX = 0;
      let lastY = 0;
      let downX = 0;
      let downY = 0;

      svg.addEventListener("pointerdown", (event) => {{
        downX = event.clientX;
        downY = event.clientY;
        if (obstacleMode) {{
          return;
        }}
        active = true;
        lastX = event.clientX;
        lastY = event.clientY;
        svg.setPointerCapture(event.pointerId);
      }});

      svg.addEventListener("pointermove", (event) => {{
        if (!active) {{
          return;
        }}
        scaleState.panX += event.clientX - lastX;
        scaleState.panY += event.clientY - lastY;
        lastX = event.clientX;
        lastY = event.clientY;
        applyTransform();
      }});

      function stop(event) {{
        if (obstacleMode && event) {{
          const moved = Math.hypot(event.clientX - downX, event.clientY - downY);
          if (moved < 6) {{
            const world = eventToWorld(event);
            if (world) {{
              obstacles.push({{ x: world.x, y: world.y, radius: 0.08 }});
              drawObstacles();
              statusText.textContent = "Obstacle point added. Drive will stop if footprint hits it.";
            }}
          }}
          return;
        }}
        active = false;
        if (event && svg.hasPointerCapture(event.pointerId)) {{
          svg.releasePointerCapture(event.pointerId);
        }}
      }}

      svg.addEventListener("pointerup", stop);
      svg.addEventListener("pointercancel", stop);
      svg.addEventListener("wheel", (event) => {{
        event.preventDefault();
        zoom(event.deltaY < 0 ? 1.1 : 0.9);
      }}, {{ passive: false }});
    }}

    buildData();
    populateSelectors();
    drawGraph();
    enableDrag();
    planRoute();

    planButton.addEventListener("click", planRoute);
    driveButton.addEventListener("click", () => {{
      const route = currentPath.length ? {{
        nodes: currentPath.slice(),
        length: 0,
      }} : null;
      if (!route) {{
        statusText.textContent = "Plan a route first.";
        return;
      }}

      for (let i = 0; i < route.nodes.length - 1; i += 1) {{
        route.length += heuristic(route.nodes[i], route.nodes[i + 1]);
      }}
      animateRoute(route);
    }});
    stopButton.addEventListener("click", () => {{
      stopAnimation();
      statusText.textContent = "Stopped.";
      if (currentTrajectory.length) {{
        drawRobotPose(currentTrajectory[0]);
      }}
    }});
    obstacleModeButton.addEventListener("click", () => {{
      obstacleMode = !obstacleMode;
      obstacleModeButton.textContent = `Add Obstacles: ${{obstacleMode ? "On" : "Off"}}`;
      statusText.textContent = obstacleMode ? "Click the map to add lidar obstacle points." : "Obstacle edit mode off.";
    }});
    clearObstaclesButton.addEventListener("click", () => {{
      obstacles = [];
      drawObstacles();
      lookaheadLayer.innerHTML = "";
      statusText.textContent = "Obstacles cleared.";
    }});
    robotWidthInput.addEventListener("change", () => drawRobotAt(startSelect.value));
    robotLengthInput.addEventListener("change", () => drawRobotAt(startSelect.value));
    lookaheadInput.addEventListener("change", () => lookaheadLayer.innerHTML = "");
    speedInput.addEventListener("change", () => {{
      const route = currentPath.length ? {{ nodes: currentPath.slice(), length: 0 }} : null;
      currentTrajectory = buildTrajectory(route);
    }});
    startSelect.addEventListener("change", planRoute);
    goalSelect.addEventListener("change", planRoute);
    zoomInButton.addEventListener("click", () => zoom(1.2));
    zoomOutButton.addEventListener("click", () => zoom(0.85));
    resetViewButton.addEventListener("click", resetView);
  </script>
</body>
</html>
"""


def main() -> None:
    args = parse_args()
    map_dir = args.map_dir.resolve()
    if not map_dir.is_dir():
        raise FileNotFoundError(f"Map directory does not exist: {map_dir}")

    ros_map_yaml = find_ros_map_yaml(map_dir)
    ros_map = read_yaml(ros_map_yaml)
    if not isinstance(ros_map, dict):
        raise ValueError(f"Unexpected ROS map file format: {ros_map_yaml}")

    image_path = (map_dir / str(ros_map["image"])).resolve()
    width, height, pixels = load_pgm(image_path)
    png_bytes = build_grayscale_png(width, height, pixels)
    image_png_base64 = base64.b64encode(png_bytes).decode("ascii")

    lms = load_lms(map_dir / "LMs.yaml")
    edges = load_edges(map_dir / "graph_edges_lengths.yaml", lms)
    start, goal = pick_defaults(lms, args.start, args.goal)
    astar(start, goal, lms, edges)

    html = build_demo_html(
        map_name=str(ros_map.get("image", image_path.stem)).replace(".pgm", ""),
        image_png_base64=image_png_base64,
        map_width=width,
        map_height=height,
        resolution=float(ros_map["resolution"]),
        origin=ros_map["origin"],
        lms=lms,
        edges=edges,
        default_start=start,
        default_goal=goal,
    )

    output_path = args.output.resolve() if args.output else map_dir / "route_demo.html"
    output_path.write_text(html, encoding="utf-8")
    print(f"Built route demo: {output_path}")
    print(f"Default route: {start} -> {goal}")
    print(f"Open in browser: {output_path}")

    if args.open:
        os.startfile(str(output_path))


if __name__ == "__main__":
    main()
