from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import yaml


MAP_NAME = "smart_kiva_large_w_mode"
MAP_DIR = Path(__file__).resolve().parent / "maps_out" / f"{MAP_NAME}.smap"
MIDDLE_ROWS = frozenset(range(5, 30, 4))
LOWER_LANE_ROWS = frozenset(range(4, 29, 4))
UPPER_LANE_ROWS = frozenset(range(6, 31, 4))
REMOVED_PERIMETER_ROWS = frozenset({1, 33})
REMOVED_PERIMETER_COLUMNS = frozenset({1, 36})
AISLE_CONNECTOR_COLUMNS = frozenset({2, 13, 24, 35})
PGM_WIDTH = 360
PGM_HEIGHT = 330
SHELF_PIXEL_ROW_STARTS = tuple(range(20, 301, 40))
SHELF_END_TRIM_RANGES = (
    (118, 120),
    (130, 132),
    (228, 230),
    (240, 242),
)
LM_NAME_PATTERN = re.compile(r"^S(?P<row>\d{3})(?P<column>\d{3})$")
EDGE_PROPERTIES = {"direction": 2, "capacity": 1, "smart": True}


def _grid_position(name: str) -> tuple[int, int]:
    match = LM_NAME_PATTERN.fullmatch(name)
    if match is None:
        raise ValueError(f"unexpected SMART landmark name: {name}")
    return int(match["row"]), int(match["column"])


def _lane_y(row: int, current_y: float) -> float:
    grid_y = float(row) - 0.5
    if row == 2:
        return 1.0
    if row == 32:
        return 32.0
    if row in LOWER_LANE_ROWS:
        return round(grid_y + 0.3, 6)
    if row in UPPER_LANE_ROWS:
        return round(grid_y - 0.3, 6)
    return current_y


def _lane_x(column: int, current_x: float) -> float:
    if column == 2:
        return 1.0
    if column == 35:
        return 35.0
    return current_x


def _edge(start: dict[str, Any], goal: dict[str, Any]) -> dict[str, Any]:
    length = round(
        math.hypot(
            float(goal["x"]) - float(start["x"]),
            float(goal["y"]) - float(start["y"]),
        ),
        6,
    )
    return {
        "from": start["name"],
        "to": goal["name"],
        "length": length,
        "kind": "line",
        "type": "FeatureLine",
        "properties": dict(EDGE_PROPERTIES),
    }


def _primitive(
    edge: dict[str, Any],
    landmarks: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    start = landmarks[edge["from"]]
    goal = landmarks[edge["to"]]
    return {
        "kind": "line",
        "line_type": "FeatureLine",
        "start": {"x": start["x"], "y": start["y"]},
        "end": {"x": goal["x"], "y": goal["y"]},
        "start_name": start["name"],
        "end_name": goal["name"],
        "properties": dict(EDGE_PROPERTIES),
        "length_m": edge["length"],
    }


def _write_yaml(path: Path, payload: Any) -> None:
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _widen_internal_cross_aisles(map_dir: Path) -> None:
    map_yaml = yaml.safe_load(
        (map_dir / f"{MAP_NAME}.yaml").read_text(encoding="utf-8")
    )
    pgm_path = map_dir / str(map_yaml["image"])
    raw = pgm_path.read_bytes()
    pixel_count = PGM_WIDTH * PGM_HEIGHT
    if not raw.startswith(b"P5") or len(raw) < pixel_count:
        raise ValueError(f"unexpected PGM format: {pgm_path}")

    header = raw[:-pixel_count]
    pixels = bytearray(raw[-pixel_count:])
    for row_start in SHELF_PIXEL_ROW_STARTS:
        for y in range(row_start, row_start + 10):
            row_offset = y * PGM_WIDTH
            for column_start, column_end in SHELF_END_TRIM_RANGES:
                pixels[
                    row_offset + column_start : row_offset + column_end
                ] = b"\xfe" * (column_end - column_start)
    pgm_path.write_bytes(header + pixels)


def rebuild(map_dir: Path = MAP_DIR) -> None:
    lm_path = map_dir / "LMs.yaml"
    lm_payload = yaml.safe_load(lm_path.read_text(encoding="utf-8"))
    source_landmarks = lm_payload["LMs"]

    landmarks_by_grid: dict[tuple[int, int], dict[str, Any]] = {}
    for source in source_landmarks:
        row, column = _grid_position(str(source["name"]))
        if (
            row in MIDDLE_ROWS
            or row in REMOVED_PERIMETER_ROWS
            or column in REMOVED_PERIMETER_COLUMNS
        ):
            continue
        landmark = {
            **source,
            "x": _lane_x(column, float(source["x"])),
            "y": _lane_y(row, float(source["y"])),
        }
        landmarks_by_grid[(row, column)] = landmark

    row_numbers = sorted({row for row, _column in landmarks_by_grid})
    edges: list[dict[str, Any]] = []

    def connect(start: dict[str, Any], goal: dict[str, Any]) -> None:
        edges.append(_edge(start, goal))
        edges.append(_edge(goal, start))

    for row in row_numbers:
        columns = sorted(column for item_row, column in landmarks_by_grid if item_row == row)
        for left_column, right_column in zip(columns, columns[1:], strict=False):
            if right_column != left_column + 1:
                continue
            connect(
                landmarks_by_grid[(row, left_column)],
                landmarks_by_grid[(row, right_column)],
            )

    for upper_row, lower_row in zip(row_numbers, row_numbers[1:], strict=False):
        upper_columns = {
            column for item_row, column in landmarks_by_grid if item_row == upper_row
        }
        lower_columns = {
            column for item_row, column in landmarks_by_grid if item_row == lower_row
        }
        common_columns = upper_columns & lower_columns
        if lower_row - upper_row > 1:
            common_columns &= AISLE_CONNECTOR_COLUMNS
        for column in sorted(common_columns):
            connect(
                landmarks_by_grid[(upper_row, column)],
                landmarks_by_grid[(lower_row, column)],
            )

    landmarks = [
        landmarks_by_grid[key]
        for key in sorted(landmarks_by_grid)
    ]
    landmarks_by_name = {
        str(landmark["name"]): landmark
        for landmark in landmarks
    }
    if len(landmarks) != 576 or len(edges) != 1240:
        raise ValueError(
            f"unexpected two-lane graph size: {len(landmarks)} LMs, {len(edges)} edges"
        )

    _write_yaml(
        lm_path,
        {
            "mapName": MAP_NAME,
            "coordinateFrame": "map_top_left",
            "LMs": landmarks,
        },
    )
    _write_yaml(
        map_dir / "graph_edges_lengths.yaml",
        edges,
    )
    _write_yaml(
        map_dir / "graphs.yaml",
        {
            "mapName": MAP_NAME,
            "coordinateFrame": "map_top_left",
            "primitives": [
                _primitive(edge, landmarks_by_name)
                for edge in edges
            ],
        },
    )
    _widen_internal_cross_aisles(map_dir)

    metadata_path = map_dir / ".operator_meta.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata.update(
        {
            "directedEdges": len(edges),
            "landmarks": len(landmarks),
            "layoutVariant": "two_lane_clearance",
            "aisleLaneClearanceM": 0.8,
            "aisleLaneSpacingM": 1.4,
            "aisleConnectorColumns": sorted(AISLE_CONNECTOR_COLUMNS),
            "crossLaneConnectionsPerAisle": len(AISLE_CONNECTOR_COLUMNS),
            "internalCrossAisleWidthM": 1.4,
            "perimeterLaneClearanceM": 1.0,
            "perimeterLaneCount": 1,
            "removedMiddleRows": len(MIDDLE_ROWS),
            "shelfEndTrimM": 0.2,
        }
    )
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    rebuild()
