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
    return edges


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
    button {{
      width: 100%;
      border-radius: 14px;
      border: 1px solid rgba(32, 36, 44, 0.14);
      padding: 12px 14px;
      font: inherit;
      background: #fffdf9;
      color: var(--ink);
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
    const pointLayer = document.getElementById("pointLayer");
    const robotLayer = document.getElementById("robotLayer");
    const routeLength = document.getElementById("routeLength");
    const routeList = document.getElementById("routeList");
    const statusText = document.getElementById("statusText");
    const startSelect = document.getElementById("startSelect");
    const goalSelect = document.getElementById("goalSelect");
    const planButton = document.getElementById("planButton");
    const driveButton = document.getElementById("driveButton");
    const zoomInButton = document.getElementById("zoomInButton");
    const zoomOutButton = document.getElementById("zoomOutButton");
    const resetViewButton = document.getElementById("resetViewButton");
    const mapImage = document.getElementById("mapImage");

    const nodeByName = new Map();
    const adjacency = new Map();
    const scaleState = {{ zoom: 1, panX: 0, panY: 0 }};
    const baseView = {{
      width: DEMO_DATA.map.viewWidth,
      height: DEMO_DATA.map.viewHeight,
    }};

    let currentPath = [];
    let animationFrame = null;
    let robotDot = null;

    mapImage.setAttribute("href", DEMO_DATA.map.imageDataUrl);

    function worldToPixel(point) {{
      const px = DEMO_DATA.map.viewPadding + ((point.x - DEMO_DATA.map.origin[0]) / DEMO_DATA.map.resolution);
      const py = DEMO_DATA.map.viewPadding + (DEMO_DATA.map.height - 1) - ((point.y - DEMO_DATA.map.origin[1]) / DEMO_DATA.map.resolution);
      return {{ x: px, y: py }};
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
        ensureAdjacency(edge.from).push({{
          to: edge.to,
          length: edge.length,
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

    function drawRoute(route) {{
      pathLayer.innerHTML = "";
      if (!route || route.nodes.length < 2) {{
        return;
      }}

      const points = route.nodes.map((name) => {{
        const lm = nodeByName.get(name);
        const pos = worldToPixel(lm);
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
    }}

    function drawRobotAt(name) {{
      robotLayer.innerHTML = "";
      const lm = nodeByName.get(name);
      if (!lm) {{
        robotDot = null;
        return;
      }}

      const pos = worldToPixel(lm);
      robotDot = createSvgElement("circle", {{
        cx: pos.x,
        cy: pos.y,
        r: 9,
        fill: "var(--robot)",
        stroke: "white",
        "stroke-width": 3,
      }});
      robotLayer.appendChild(robotDot);
    }}

    function setRobotPixel(x, y) {{
      if (!robotDot) {{
        robotDot = createSvgElement("circle", {{
          cx: x,
          cy: y,
          r: 9,
          fill: "var(--robot)",
          stroke: "white",
          "stroke-width": 3,
        }});
        robotLayer.appendChild(robotDot);
      }}

      robotDot.setAttribute("cx", String(x));
      robotDot.setAttribute("cy", String(y));
    }}

    function animateRoute(route) {{
      if (!route || route.nodes.length < 2) {{
        statusText.textContent = "Route is empty.";
        return;
      }}

      stopAnimation();
      const speedMetersPerSec = 0.7;
      const segments = [];

      for (let i = 0; i < route.nodes.length - 1; i += 1) {{
        const start = nodeByName.get(route.nodes[i]);
        const goal = nodeByName.get(route.nodes[i + 1]);
        const worldLen = distWorld(start, goal);
        segments.push({{
          startPx: worldToPixel(start),
          goalPx: worldToPixel(goal),
          durationMs: Math.max(150, (worldLen / speedMetersPerSec) * 1000),
          startName: route.nodes[i],
          goalName: route.nodes[i + 1],
        }});
      }}

      let segmentIndex = 0;
      let segmentStartTs = null;
      statusText.textContent = `Driving from ${{route.nodes[0]}} to ${{route.nodes[route.nodes.length - 1]}}`;

      function step(ts) {{
        if (segmentIndex >= segments.length) {{
          statusText.textContent = `Arrived at ${{route.nodes[route.nodes.length - 1]}}`;
          drawRobotAt(route.nodes[route.nodes.length - 1]);
          animationFrame = null;
          return;
        }}

        const segment = segments[segmentIndex];
        if (segmentStartTs === null) {{
          segmentStartTs = ts;
        }}

        const progress = Math.min(1, (ts - segmentStartTs) / segment.durationMs);
        const x = segment.startPx.x + ((segment.goalPx.x - segment.startPx.x) * progress);
        const y = segment.startPx.y + ((segment.goalPx.y - segment.startPx.y) * progress);
        setRobotPixel(x, y);

        if (progress >= 1) {{
          segmentIndex += 1;
          segmentStartTs = ts;
        }}

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

      svg.addEventListener("pointerdown", (event) => {{
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
        edges=unique_edges(edges),
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
