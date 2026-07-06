from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "fleet_manager" / "map_data" / "maps_out"
DEFAULT_SOURCES = [
    PROJECT_ROOT / "lifelong-smart" / "maps" / "kiva_large_w_mode.json",
    PROJECT_ROOT / "lifelong-smart" / "maps" / "random-32-32-20.json",
    PROJECT_ROOT / "lifelong-smart" / "maps" / "symbotic_33-39.json",
    PROJECT_ROOT / "lifelong-smart" / "maps" / "front_fig_5x5.json",
]


def _safe_map_name(path: Path) -> str:
    stem = path.stem.lower().replace("-", "_").replace(".", "_")
    return f"smart_{stem}"


def _load_layout(path: Path) -> tuple[str, list[str]]:
    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        layout = payload.get("layout")
        if not isinstance(layout, list) or not layout:
            raise ValueError(f"JSON map has no layout: {path}")
        return str(payload.get("name") or path.stem), [str(row) for row in layout]

    lines = path.read_text(encoding="utf-8").splitlines()
    try:
        map_index = next(index for index, line in enumerate(lines) if line.strip().lower() == "map")
    except StopIteration as exc:
        raise ValueError(f"MovingAI map has no map section: {path}") from exc
    layout = [line.rstrip("\n") for line in lines[map_index + 1 :] if line.strip()]
    if not layout:
        raise ValueError(f"MovingAI map has no grid rows: {path}")
    return path.stem, layout


def _is_blocked(char: str) -> bool:
    if char in {"@", "T", "O"}:
        return True
    return False


def _lm_name(prefix: str, row: int, col: int) -> str:
    return f"{prefix}{row + 1:03d}{col + 1:03d}"


def _point(row: int, col: int, cell_size: float) -> dict[str, float]:
    return {
        "x": round((col + 0.5) * cell_size, 4),
        "y": round((row + 0.5) * cell_size, 4),
    }


def _write_layout_pgm(
    path: Path,
    layout: list[str],
    *,
    cell_px: int,
) -> tuple[int, int]:
    rows = len(layout)
    cols = max(len(row) for row in layout)
    width = cols * cell_px
    height = rows * cell_px
    pixels = bytearray([254] * width * height)
    for row_index, row in enumerate(layout):
        padded = row.ljust(cols, "@")
        for col_index, char in enumerate(padded):
            if not _is_blocked(char):
                continue
            for py in range(row_index * cell_px, (row_index + 1) * cell_px):
                start = (py * width) + (col_index * cell_px)
                pixels[start : start + cell_px] = bytes([0]) * cell_px
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels))
    return width, height


def create_smart_smap(
    source: Path,
    output_root: Path,
    *,
    map_name: str | None = None,
    cell_size: float = 1.0,
    resolution: float = 0.1,
) -> Path:
    source_name, layout = _load_layout(source)
    name = map_name or _safe_map_name(source)
    map_dir = output_root / f"{name}.smap"
    map_dir.mkdir(parents=True, exist_ok=True)

    rows = len(layout)
    cols = max(len(row) for row in layout)
    cell_px = max(1, int(round(cell_size / resolution)))
    width_px, height_px = _write_layout_pgm(map_dir / f"{name}.pgm", layout, cell_px=cell_px)

    prefix = "".join(char for char in name.upper() if char.isalnum())[:1] or "S"
    points: dict[tuple[int, int], dict[str, float]] = {}
    landmarks: list[dict[str, Any]] = []
    for row in range(rows):
        padded = layout[row].ljust(cols, "@")
        for col, char in enumerate(padded):
            if _is_blocked(char):
                continue
            lm_name = _lm_name(prefix, row, col)
            point = _point(row, col, cell_size)
            points[(row, col)] = point
            landmarks.append(
                {
                    "name": lm_name,
                    "x": point["x"],
                    "y": point["y"],
                    "ignoreDir": None,
                    "properties": {
                        "spin": False,
                        "smartCell": char,
                        "waitAllowed": True,
                    },
                }
            )

    primitives: list[dict[str, Any]] = []
    edges: list[dict[str, Any]] = []

    def add_directed(row_a: int, col_a: int, row_b: int, col_b: int) -> None:
        start_name = _lm_name(prefix, row_a, col_a)
        end_name = _lm_name(prefix, row_b, col_b)
        start = points[(row_a, col_a)]
        end = points[(row_b, col_b)]
        length = round(math.dist((start["x"], start["y"]), (end["x"], end["y"])), 6)
        props = {"direction": 2, "capacity": 1, "smart": True}
        primitives.append(
            {
                "kind": "line",
                "line_type": "FeatureLine",
                "start": dict(start),
                "end": dict(end),
                "start_name": start_name,
                "end_name": end_name,
                "properties": props,
                "length_m": length,
            }
        )
        edges.append(
            {
                "from": start_name,
                "to": end_name,
                "length": length,
                "kind": "line",
                "type": "FeatureLine",
                "properties": props,
            }
        )

    for row, col in sorted(points):
        for drow, dcol in ((0, 1), (1, 0)):
            neighbor = (row + drow, col + dcol)
            if neighbor not in points:
                continue
            add_directed(row, col, neighbor[0], neighbor[1])
            add_directed(neighbor[0], neighbor[1], row, col)

    (map_dir / f"{name}.yaml").write_text(
        yaml.safe_dump(
            {
                "image": f"{name}.pgm",
                "mode": "trinary",
                "resolution": resolution,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "LMs.yaml").write_text(
        yaml.safe_dump(
            {"mapName": name, "coordinateFrame": "map_top_left", "LMs": landmarks},
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (map_dir / "graphs.yaml").write_text(
        yaml.safe_dump(
            {"mapName": name, "coordinateFrame": "map_top_left", "primitives": primitives},
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
                "mapName": name,
                "kind": "smart_layout_benchmark",
                "source": str(source.relative_to(PROJECT_ROOT)) if source.is_relative_to(PROJECT_ROOT) else str(source),
                "sourceName": source_name,
                "rows": rows,
                "cols": cols,
                "cellSizeM": cell_size,
                "resolutionM": resolution,
                "imageWidthPx": width_px,
                "imageHeightPx": height_px,
                "landmarks": len(landmarks),
                "directedEdges": len(edges),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return map_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert SMART/MovingAI layouts to Fleet Manager .smap benchmark maps.")
    parser.add_argument("sources", nargs="*", type=Path, default=DEFAULT_SOURCES)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--cell-size", type=float, default=1.0)
    parser.add_argument("--resolution", type=float, default=0.1)
    args = parser.parse_args()

    summaries = []
    for source in args.sources:
        source = source.expanduser().resolve()
        if not source.is_file():
            continue
        map_dir = create_smart_smap(
            source,
            args.output_root,
            cell_size=args.cell_size,
            resolution=args.resolution,
        )
        meta = json.loads((map_dir / ".operator_meta.json").read_text(encoding="utf-8"))
        summaries.append(
            {
                "mapName": meta["mapName"],
                "mapDir": str(map_dir),
                "landmarks": meta["landmarks"],
                "directedEdges": meta["directedEdges"],
                "source": meta["source"],
            }
        )
    print(json.dumps({"ok": True, "maps": summaries}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
