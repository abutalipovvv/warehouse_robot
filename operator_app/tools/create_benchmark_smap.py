from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SMART_KIVA_MAP = PROJECT_ROOT / "lifelong-smart" / "maps" / "kiva_large_w_mode.json"
DEFAULT_MAP_NAME = "benchmark_open_kiva"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "fleet_manager" / "map_data" / "maps_out"


def _load_smart_dimensions(path: Path) -> tuple[int, int]:
    if not path.is_file():
        return 33, 36
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = int(payload.get("n_row") or 33)
    cols = int(payload.get("n_col") or 36)
    return rows, cols


def _lm_name(row: int, col: int) -> str:
    return f"B{row + 1:02d}{col + 1:02d}"


def _point(row: int, col: int, spacing: float, margin: float) -> dict[str, float]:
    return {
        "x": round(margin + (col * spacing), 4),
        "y": round(margin + (row * spacing), 4),
    }


def _line_primitive(
    start_name: str,
    end_name: str,
    start: dict[str, float],
    end: dict[str, float],
    length: float,
) -> dict[str, Any]:
    return {
        "kind": "line",
        "line_type": "FeatureLine",
        "start": dict(start),
        "end": dict(end),
        "start_name": start_name,
        "end_name": end_name,
        "properties": {
            "direction": 2,
            "capacity": 1,
            "benchmark": True,
        },
        "length_m": round(length, 6),
    }


def _edge(start_name: str, end_name: str, length: float) -> dict[str, Any]:
    return {
        "from": start_name,
        "to": end_name,
        "length": round(length, 6),
        "kind": "line",
        "type": "FeatureLine",
        "properties": {
            "direction": 2,
            "capacity": 1,
            "benchmark": True,
        },
    }


def _write_free_pgm(path: Path, width_px: int, height_px: int) -> None:
    header = f"P5\n{width_px} {height_px}\n255\n".encode("ascii")
    path.write_bytes(header + (bytes([254]) * width_px * height_px))


def create_benchmark_smap(
    output_root: Path,
    map_name: str,
    rows: int,
    cols: int,
    spacing: float,
    margin: float,
    resolution: float,
) -> Path:
    map_dir = output_root / f"{map_name}.smap"
    map_dir.mkdir(parents=True, exist_ok=True)

    width_m = ((cols - 1) * spacing) + (margin * 2.0)
    height_m = ((rows - 1) * spacing) + (margin * 2.0)
    width_px = int(math.ceil(width_m / resolution))
    height_px = int(math.ceil(height_m / resolution))

    landmarks: list[dict[str, Any]] = []
    points: dict[tuple[int, int], dict[str, float]] = {}
    for row in range(rows):
        for col in range(cols):
            name = _lm_name(row, col)
            point = _point(row, col, spacing, margin)
            points[(row, col)] = point
            landmarks.append(
                {
                    "name": name,
                    "x": point["x"],
                    "y": point["y"],
                    "ignoreDir": None,
                    "properties": {
                        "spin": False,
                        "benchmark": True,
                        "waitAllowed": True,
                    },
                }
            )

    primitives: list[dict[str, Any]] = []
    edges: list[dict[str, Any]] = []

    def add_directed(row_a: int, col_a: int, row_b: int, col_b: int) -> None:
        start_name = _lm_name(row_a, col_a)
        end_name = _lm_name(row_b, col_b)
        start = points[(row_a, col_a)]
        end = points[(row_b, col_b)]
        length = math.dist((start["x"], start["y"]), (end["x"], end["y"]))
        primitives.append(_line_primitive(start_name, end_name, start, end, length))
        edges.append(_edge(start_name, end_name, length))

    for row in range(rows):
        for col in range(cols):
            if col + 1 < cols:
                add_directed(row, col, row, col + 1)
                add_directed(row, col + 1, row, col)
            if row + 1 < rows:
                add_directed(row, col, row + 1, col)
                add_directed(row + 1, col, row, col)

    _write_free_pgm(map_dir / f"{map_name}.pgm", width_px, height_px)
    (map_dir / f"{map_name}.yaml").write_text(
        yaml.safe_dump(
            {
                "image": f"{map_name}.pgm",
                "mode": "trinary",
                "resolution": resolution,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "LMs.yaml").write_text(
        yaml.safe_dump(
            {
                "mapName": map_name,
                "coordinateFrame": "map_top_left",
                "LMs": landmarks,
            },
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "graphs.yaml").write_text(
        yaml.safe_dump(
            {
                "mapName": map_name,
                "coordinateFrame": "map_top_left",
                "primitives": primitives,
            },
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "graph_edges_lengths.yaml").write_text(
        yaml.safe_dump(edges, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    (map_dir / ".operator_meta.json").write_text(
        json.dumps(
            {
                "schemaVersion": 1,
                "mapName": map_name,
                "kind": "fleet_sim_benchmark",
                "source": str(SMART_KIVA_MAP.relative_to(PROJECT_ROOT))
                if SMART_KIVA_MAP.is_file()
                else "generated",
                "walls": False,
                "coordinateFrame": "map_top_left",
                "rows": rows,
                "cols": cols,
                "landmarks": len(landmarks),
                "directedEdges": len(edges),
                "spacingM": spacing,
                "marginM": margin,
                "resolutionM": resolution,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return map_dir


def main() -> None:
    default_rows, default_cols = _load_smart_dimensions(SMART_KIVA_MAP)
    parser = argparse.ArgumentParser(description="Create an open-grid benchmark .smap for Fleet Manager Sim.")
    parser.add_argument("--map-name", default=DEFAULT_MAP_NAME)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--rows", type=int, default=default_rows)
    parser.add_argument("--cols", type=int, default=default_cols)
    parser.add_argument("--spacing", type=float, default=1.2)
    parser.add_argument("--margin", type=float, default=1.0)
    parser.add_argument("--resolution", type=float, default=0.1)
    args = parser.parse_args()

    map_dir = create_benchmark_smap(
        output_root=args.output_root,
        map_name=args.map_name,
        rows=args.rows,
        cols=args.cols,
        spacing=args.spacing,
        margin=args.margin,
        resolution=args.resolution,
    )
    meta = json.loads((map_dir / ".operator_meta.json").read_text(encoding="utf-8"))
    print(
        json.dumps(
            {
                "ok": True,
                "mapDir": str(map_dir),
                "mapName": args.map_name,
                "rows": meta["rows"],
                "cols": meta["cols"],
                "landmarks": meta["landmarks"],
                "directedEdges": meta["directedEdges"],
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
