#!/usr/bin/env python3
# smap_deserialize.py
#
# Usage:
#   python smap_deserialize.py ./maps/15.05.26_map.smap --out ./maps_out/15.05.26_map
#
# Produces:
#   - <mapName>.pgm
#   - <mapName>.yaml   (ROS map_server compatible)
#   - LMs.yaml         (from advancedPointList LocationMark)
#   - graphs.yaml      (lines + curves, with type tags + length_m)
#   - graph_edges_lengths.yaml (LM->LM graph edges with length, direction if detectable)
#   - primitives_lengths.csv  (length of every primitive)
#   - smap_summary.json
#
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional, List

import yaml

FREE = 254  # white
OCC = 0     # black


# ----------------------------
# Helpers
# ----------------------------
def _prop_to_value(pr: Dict[str, Any]) -> Any:
    """
    RoboShop/RDS often stores ints in int32Value. Some fields may appear as base64 in `value`.
    We prefer numeric typed fields when present.
    """
    if pr.get("int32Value") is not None:
        return pr["int32Value"]
    if pr.get("doubleValue") is not None:
        return pr["doubleValue"]
    if pr.get("boolValue") is not None:
        return pr["boolValue"]
    if pr.get("value") is not None:
        return pr["value"]
    return None


def _parse_properties(prop_list: Any) -> Dict[str, Any]:
    props: Dict[str, Any] = {}
    if not isinstance(prop_list, list):
        return props
    for pr in prop_list:
        if isinstance(pr, dict) and pr.get("key") is not None:
            props[str(pr["key"])] = _prop_to_value(pr)
    return props


def _pos_xy(obj: Any) -> Optional[Dict[str, float]]:
    """
    Handles both:
      - {"x":..., "y":...}
      - {"pos":{"x":...,"y":...}, ...}
    """
    if not isinstance(obj, dict):
        return None

    if "x" in obj and "y" in obj:
        try:
            return {"x": float(obj["x"]), "y": float(obj["y"])}
        except Exception:
            return None

    pos = obj.get("pos")
    if isinstance(pos, dict) and "x" in pos and "y" in pos:
        try:
            return {"x": float(pos["x"]), "y": float(pos["y"])}
        except Exception:
            return None

    return None


def _dist(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


def _bezier_point(p0, p1, p2, p3, t: float) -> Dict[str, float]:
    u = 1.0 - t
    return {
        "x": (u*u*u)*p0["x"] + 3*(u*u)*t*p1["x"] + 3*u*(t*t)*p2["x"] + (t*t*t)*p3["x"],
        "y": (u*u*u)*p0["y"] + 3*(u*u)*t*p1["y"] + 3*u*(t*t)*p2["y"] + (t*t*t)*p3["y"],
    }


def _bezier_length(p0, p1, p2, p3, steps: int = 200) -> float:
    prev = _bezier_point(p0, p1, p2, p3, 0.0)
    total = 0.0
    for i in range(1, steps + 1):
        t = i / steps
        pt = _bezier_point(p0, p1, p2, p3, t)
        total += _dist(prev, pt)
        prev = pt
    return total


def _nearest_lm_name(
    x: float,
    y: float,
    lm_index: Dict[str, Dict[str, float]],
    max_radius_m: float = 0.75
) -> Optional[str]:
    best_name = None
    best_d = 1e9
    for name, p in lm_index.items():
        d = math.hypot(p["x"] - x, p["y"] - y)
        if d < best_d:
            best_d = d
            best_name = name
    if best_name is None:
        return None
    return best_name if best_d <= max_radius_m else None


def _is_one_way(props: Dict[str, Any]) -> bool:
    """
    Heuristic one-way detection from properties.
    Adjust to your real .smap property keys later if needed.
    """
    for k, v in props.items():
        kk = str(k).lower()
        if kk in {"oneway", "one_way", "isoneway", "one-way"}:
            return bool(v)
        if kk in {"direction", "dir"} and str(v).lower() in {"forward", "fwd", "oneway"}:
            return True
    return False


def _reverse_one_way(props: Dict[str, Any]) -> bool:
    """
    Heuristic reverse-direction detection from properties.
    """
    for k, v in props.items():
        kk = str(k).lower()
        if kk in {"reverse", "reversed", "backward", "rev"}:
            return bool(v)
        if kk in {"direction", "dir"} and str(v).lower() in {"backward", "bwd", "reverse"}:
            return True
    return False


# ----------------------------
# Main deserialize
# ----------------------------
def deserialize_smap(smap_path: Path, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    data = json.loads(smap_path.read_text(encoding="utf-8"))
    hdr = data.get("header", {}) or {}

    # Header fields
    minx = float((hdr.get("minPos") or {}).get("x", 0.0))
    miny = float((hdr.get("minPos") or {}).get("y", 0.0))
    maxx = float((hdr.get("maxPos") or {}).get("x", 0.0))
    maxy = float((hdr.get("maxPos") or {}).get("y", 0.0))
    res = float(hdr.get("resolution", 0.05))
    map_name = hdr.get("mapName", smap_path.stem)

    # Grid size
    if not (maxx > minx and maxy > miny and res > 0):
        raise ValueError("Bad header bounds/resolution; can't build grid.")

    width = int(math.ceil((maxx - minx) / res)) + 1
    height = int(math.ceil((maxy - miny) / res)) + 1

    # Image buffer (top-left origin)
    grid = [[FREE for _ in range(width)] for _ in range(height)]

    def world_to_grid(x: float, y: float) -> tuple[int, int]:
        gx = int((x - minx) / res)
        gy = int((y - miny) / res)
        iy = (height - 1) - gy  # flip Y for image coordinates
        ix = gx
        return ix, iy

    def mark_point(x: float, y: float, val: int = OCC, r_px: int = 0) -> None:
        ix, iy = world_to_grid(x, y)
        for dy in range(-r_px, r_px + 1):
            for dx in range(-r_px, r_px + 1):
                x2, y2 = ix + dx, iy + dy
                if 0 <= x2 < width and 0 <= y2 < height:
                    grid[y2][x2] = val

    def draw_line(x0: float, y0: float, x1: float, y1: float, val: int = OCC) -> None:
        # Bresenham in pixel space
        x0i, y0i = world_to_grid(x0, y0)
        x1i, y1i = world_to_grid(x1, y1)

        dx = abs(x1i - x0i)
        dy = abs(y1i - y0i)
        sx = 1 if x0i < x1i else -1
        sy = 1 if y0i < y1i else -1
        err = dx - dy

        x, y = x0i, y0i
        while True:
            if 0 <= x < width and 0 <= y < height:
                grid[y][x] = val
            if x == x1i and y == y1i:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # For CSV export of lengths
    lengths_rows: List[Dict[str, Any]] = []

    # ----------------------------
    # 1) normalPosList -> occupied pixels
    # ----------------------------
    pts = data.get("normalPosList", [])
    used_pts, skipped_pts = 0, 0
    if isinstance(pts, list):
        for p in pts:
            if isinstance(p, dict) and "x" in p and "y" in p:
                mark_point(float(p["x"]), float(p["y"]), OCC, r_px=0)
                used_pts += 1
            else:
                skipped_pts += 1

    # ----------------------------
    # 2) advancedLineList -> "line" primitives + rasterize + length
    # ----------------------------
    advanced_lines = data.get("advancedLineList", [])
    line_primitives = []
    used_lines, skipped_lines = 0, 0

    if isinstance(advanced_lines, list):
        for idx, item in enumerate(advanced_lines):
            if not isinstance(item, dict):
                skipped_lines += 1
                continue

            line = item.get("line") or {}
            sp = (line.get("startPos") or {})
            ep = (line.get("endPos") or {})

            if "x" in sp and "y" in sp and "x" in ep and "y" in ep:
                x0, y0 = float(sp["x"]), float(sp["y"])
                x1, y1 = float(ep["x"]), float(ep["y"])

                draw_line(x0, y0, x1, y1, OCC)

                props = _parse_properties(item.get("property") or [])
                length_m = math.hypot(x1 - x0, y1 - y0)

                prim = {
                    "kind": "line",
                    "line_type": item.get("className"),  # FeatureLine/ForbiddenLine/...
                    "start": {"x": x0, "y": y0},
                    "end": {"x": x1, "y": y1},
                    "properties": props,
                    "length_m": float(length_m),
                }
                line_primitives.append(prim)

                lengths_rows.append({
                    "idx": idx,
                    "kind": "line",
                    "type": str(item.get("className")),
                    "start_x": x0, "start_y": y0,
                    "end_x": x1, "end_y": y1,
                    "length_m": float(length_m),
                })

                used_lines += 1
            else:
                skipped_lines += 1

    # ----------------------------
    # 2b) advancedCurveList -> "curve" primitives + length (Bezier)
    # ----------------------------
    advanced_curves = data.get("advancedCurveList", [])
    curve_primitives = []
    used_curves, skipped_curves = 0, 0

    if isinstance(advanced_curves, list):
        for idx, item in enumerate(advanced_curves):
            if not isinstance(item, dict):
                skipped_curves += 1
                continue

            props = _parse_properties(item.get("property") or [])

            start = _pos_xy(item.get("startPos"))
            end = _pos_xy(item.get("endPos"))
            c1 = _pos_xy(item.get("controlPos1"))
            c2 = _pos_xy(item.get("controlPos2"))

            curve_payload = {
                "start": start,
                "end": end,
                "control1": c1,
                "control2": c2,
                "start_name": (item.get("startPos") or {}).get("instanceName"),
                "end_name": (item.get("endPos") or {}).get("instanceName"),
            }

            length_m = None
            if start and end and c1 and c2:
                length_m = _bezier_length(start, c1, c2, end, steps=200)

            prim = {
                "kind": "curve",
                "curve_type": item.get("className"),     # usually DegenerateBezier
                "curve": curve_payload,
                "properties": props,
                "length_m": (float(length_m) if length_m is not None else None),
            }
            curve_primitives.append(prim)

            lengths_rows.append({
                "idx": idx,
                "kind": "curve",
                "type": str(item.get("className")),
                "start_x": (start["x"] if start else None),
                "start_y": (start["y"] if start else None),
                "end_x": (end["x"] if end else None),
                "end_y": (end["y"] if end else None),
                "length_m": (float(length_m) if length_m is not None else None),
            })

            used_curves += 1

    # ----------------------------
    # 3) advancedPointList -> LMs.yaml
    # ----------------------------
    advanced_points = data.get("advancedPointList", [])
    lms = []
    if isinstance(advanced_points, list):
        for p in advanced_points:
            if not isinstance(p, dict):
                continue
            if p.get("className") != "LocationMark":
                continue

            name = p.get("instanceName") or p.get("name") or p.get("id")
            pos = p.get("pos") or {}
            if name is None or not isinstance(pos, dict) or "x" not in pos or "y" not in pos:
                continue

            props = _parse_properties(p.get("property") or [])
            lms.append(
                {
                    "name": str(name),
                    "x": float(pos["x"]),
                    "y": float(pos["y"]),
                    "ignoreDir": p.get("ignoreDir"),
                    "properties": props,
                }
            )

    # ----------------------------
    # 3c) Build graph edges from primitives (snap endpoints to nearest LM)
    # Output: graph_edges_lengths.yaml
    # ----------------------------
    lm_index: Dict[str, Dict[str, float]] = {lm["name"]: {"x": lm["x"], "y": lm["y"]} for lm in lms}
    edges: List[Dict[str, Any]] = []

    def _add_edge(u: Optional[str], v: Optional[str], length_m: Optional[float], kind: str, typ: str, props: Dict[str, Any]) -> None:
        if not u or not v:
            return
        if u == v:
            return
        if length_m is None:
            return
        try:
            L = float(length_m)
        except Exception:
            return
        if not math.isfinite(L) or L <= 0:
            return
        edges.append({
            "from": u,
            "to": v,
            "length": L,
            "kind": kind,
            "type": typ,
            "properties": props,
        })

    # --- lines ---
    for p in line_primitives:
        sp = p.get("start") or {}
        ep = p.get("end") or {}
        props = p.get("properties") or {}
        kind = "line"
        typ = str(p.get("line_type"))

        sx, sy = sp.get("x"), sp.get("y")
        ex, ey = ep.get("x"), ep.get("y")
        if sx is None or sy is None or ex is None or ey is None:
            continue

        u = _nearest_lm_name(float(sx), float(sy), lm_index, max_radius_m=0.75)
        v = _nearest_lm_name(float(ex), float(ey), lm_index, max_radius_m=0.75)

        _add_edge(u, v, p.get("length_m"), kind, typ, props)

    # --- curves ---
    for p in curve_primitives:
        curve = (p.get("curve") or {})
        sp = curve.get("start") or {}
        ep = curve.get("end") or {}
        props = p.get("properties") or {}
        kind = "curve"
        typ = str(p.get("curve_type"))

        sx, sy = sp.get("x"), sp.get("y")
        ex, ey = ep.get("x"), ep.get("y")
        if sx is None or sy is None or ex is None or ey is None:
            continue

        sname = curve.get("start_name")
        ename = curve.get("end_name")
        u = (str(sname) if sname in lm_index else None)
        v = (str(ename) if ename in lm_index else None)

        if u is None:
            u = _nearest_lm_name(float(sx), float(sy), lm_index, max_radius_m=0.75)
        if v is None:
            v = _nearest_lm_name(float(ex), float(ey), lm_index, max_radius_m=0.75)

        _add_edge(u, v, p.get("length_m"), kind, typ, props)

    # dedupe by (from,to) keeping minimal length
    dedup: Dict[tuple, Dict[str, Any]] = {}
    for e in edges:
        key = (e["from"], e["to"])
        if key not in dedup or e["length"] < dedup[key]["length"]:
            dedup[key] = e
    edges = list(dedup.values())

    (out_dir / "graph_edges_lengths.yaml").write_text(
        yaml.safe_dump(edges, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    # ----------------------------
    # Write PGM
    # ----------------------------
    pgm_path = out_dir / f"{map_name}.pgm"
    with pgm_path.open("wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for row in grid:
            f.write(bytes(row))

    # ----------------------------
    # Write ROS map YAML
    # ----------------------------
    map_yaml = {
        "image": pgm_path.name,
        "resolution": res,
        "origin": [minx, miny, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }
    (out_dir / f"{map_name}.yaml").write_text(
        yaml.safe_dump(map_yaml, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    # ----------------------------
    # graphs.yaml (lines + curves) + length_m already inside each primitive
    # ----------------------------
    graphs_obj = {
        "mapName": map_name,
        "primitives": line_primitives + curve_primitives,
    }
    (out_dir / "graphs.yaml").write_text(
        yaml.safe_dump(graphs_obj, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    # ----------------------------
    # LMs.yaml
    # ----------------------------
    (out_dir / "LMs.yaml").write_text(
        yaml.safe_dump({"mapName": map_name, "LMs": lms}, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    # ----------------------------
    # primitives_lengths.csv (each primitive length)
    # ----------------------------
    csv_path = out_dir / "primitives_lengths.csv"
    fieldnames = ["idx", "kind", "type", "start_x", "start_y", "end_x", "end_y", "length_m"]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in lengths_rows:
            w.writerow(r)

    # ----------------------------
    # Length summary stats
    # ----------------------------
    def _stats(vals):
        vals = [v for v in vals if isinstance(v, (int, float)) and math.isfinite(v)]
        if not vals:
            return {"count": 0, "sum": 0.0, "min": None, "max": None, "mean": None}
        s = float(sum(vals))
        return {
            "count": len(vals),
            "sum": s,
            "min": float(min(vals)),
            "max": float(max(vals)),
            "mean": float(s / len(vals)),
        }

    line_lengths = [p.get("length_m") for p in line_primitives]
    curve_lengths = [p.get("length_m") for p in curve_primitives]
    all_lengths = [v for v in (line_lengths + curve_lengths)]

    # breakdown by line_type
    line_type_sums: Dict[str, float] = {}
    for p in line_primitives:
        t = str(p.get("line_type"))
        L = p.get("length_m")
        if isinstance(L, (int, float)):
            line_type_sums[t] = line_type_sums.get(t, 0.0) + float(L)

    # ----------------------------
    # Summary
    # ----------------------------
    summary = {
        "header": hdr,
        "counts": {
            "normalPosList_total": len(pts) if isinstance(pts, list) else None,
            "normalPosList_used": used_pts,
            "normalPosList_skipped": skipped_pts,

            "advancedLineList_total": len(advanced_lines) if isinstance(advanced_lines, list) else None,
            "lines_used": used_lines,
            "lines_skipped": skipped_lines,

            "advancedCurveList_total": len(advanced_curves) if isinstance(advanced_curves, list) else None,
            "curves_used": used_curves,
            "curves_skipped": skipped_curves,

            "advancedPointList_total": len(advanced_points) if isinstance(advanced_points, list) else None,
            "LMs_found": len(lms),

            "edges_total": len(edges),

            "grid": {"width": width, "height": height},
        },
        "lengths_m": {
            "lines": _stats(line_lengths),
            "curves": _stats(curve_lengths),
            "all": _stats(all_lengths),
            "line_type_sum": line_type_sums,
        },
        "top_level_keys": sorted(list(data.keys())),
        "outputs": {
            "pgm": str(pgm_path.name),
            "ros_map_yaml": str(f"{map_name}.yaml"),
            "LMs_yaml": "LMs.yaml",
            "graphs_yaml": "graphs.yaml",
            "graph_edges_lengths_yaml": "graph_edges_lengths.yaml",
            "primitives_lengths_csv": "primitives_lengths.csv",
            "summary_json": "smap_summary.json",
        }
    }
    (out_dir / "smap_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description="Deserialize .smap (JSON) -> pgm/yaml/LMs/graphs + primitive lengths + graph edges"
    )
    ap.add_argument("smap", type=Path, help="Path to .smap file")
    ap.add_argument("--out", type=Path, default=None, help="Output directory")
    args = ap.parse_args()

    out_dir = args.out or Path(f"smap_deserialized_{args.smap.stem}")
    deserialize_smap(args.smap, out_dir)

    print("Done.")
    print("Output dir:", out_dir.resolve())


if __name__ == "__main__":
    main()
