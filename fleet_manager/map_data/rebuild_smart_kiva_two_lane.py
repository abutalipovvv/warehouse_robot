from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import yaml


MAP_NAME = "smart_kiva_large_w_mode"
MAP_DIR = Path(__file__).resolve().parent / "maps_out" / f"{MAP_NAME}.smap"
# The original SMART grid uses a 1.0 m pitch while the Ecom robot has a
# 1.302 m safe circumscribed diameter (body plus collision margin).  Adjacent
# robots therefore overlap during an in-place turn even though both centres
# are on valid LMs.  Expand the complete physical map instead of weakening the
# collision model.  4/3 keeps the raster dimensions integral and gives every
# pair of adjacent LMs enough room for arbitrary simultaneous headings.
MAP_SCALE = 4.0 / 3.0
MIDDLE_ROWS = frozenset(range(5, 30, 4))
LOWER_LANE_ROWS = frozenset(range(4, 29, 4))
UPPER_LANE_ROWS = frozenset(range(6, 31, 4))
REMOVED_PERIMETER_ROWS = frozenset({1, 33})
REMOVED_PERIMETER_COLUMNS = frozenset({1, 36})
AISLE_CONNECTOR_COLUMNS = frozenset({2, 13, 24, 35})
# Shelf crossings are the genuinely single-lane parts of this map.  Each
# crossing is a small local controlled corridor with one internal LM; the
# open horizontal aisles retain their two independent passing lanes.
CONTROLLED_CORRIDOR_COLUMNS = AISLE_CONNECTOR_COLUMNS
CONTROLLED_CORRIDOR_ROW_PAIRS = tuple(
    (row, row + 2)
    for row in range(2, 31, 4)
)
BASE_PGM_WIDTH = 360
BASE_PGM_HEIGHT = 330
PGM_WIDTH = round(BASE_PGM_WIDTH * MAP_SCALE)
PGM_HEIGHT = round(BASE_PGM_HEIGHT * MAP_SCALE)
SHELF_PIXEL_ROW_STARTS = tuple(range(20, 301, 40))
SHELF_END_TRIM_RANGES = (
    (118, 120),
    (130, 132),
    (228, 230),
    (240, 242),
)
LM_NAME_PATTERN = re.compile(r"^S(?P<row>\d{3})(?P<column>\d{3})$")
EDGE_PROPERTIES = {"capacity": 1, "smart": True}


def _grid_position(name: str) -> tuple[int, int]:
    match = LM_NAME_PATTERN.fullmatch(name)
    if match is None:
        raise ValueError(f"unexpected SMART landmark name: {name}")
    return int(match["row"]), int(match["column"])


def _scaled(value: float) -> float:
    return round(value * MAP_SCALE, 6)


def _lane_y(row: int) -> float:
    grid_y = float(row) - 0.5
    if row == 2:
        return _scaled(1.0)
    if row == 32:
        return _scaled(32.0)
    if row in LOWER_LANE_ROWS:
        return _scaled(grid_y + 0.3)
    if row in UPPER_LANE_ROWS:
        return _scaled(grid_y - 0.3)
    return _scaled(grid_y)


def _lane_x(column: int) -> float:
    if column == 2:
        return _scaled(1.0)
    if column == 35:
        return _scaled(35.0)
    return _scaled(float(column) - 0.5)


def _edge(
    start: dict[str, Any],
    goal: dict[str, Any],
    *,
    motion_direction: int,
) -> dict[str, Any]:
    length = round(
        math.hypot(
            float(goal["x"]) - float(start["x"]),
            float(goal["y"]) - float(start["y"]),
        ),
        6,
    )
    properties = {
        **EDGE_PROPERTIES,
        # Traffic direction is represented by the presence of each directed
        # graph edge.  This field is the robot body motion rule: canonical
        # increasing grid direction is forward, its reverse is backward.
        "direction": motion_direction,
    }
    return {
        "from": start["name"],
        "to": goal["name"],
        "length": length,
        "kind": "line",
        "type": "FeatureLine",
        "properties": properties,
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
        "properties": dict(edge.get("properties") or EDGE_PROPERTIES),
        "length_m": edge["length"],
    }


def _write_yaml(path: Path, payload: Any) -> None:
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _parse_pgm(raw: bytes, pgm_path: Path) -> tuple[int, int, bytearray]:
    match = re.match(rb"^P5\s+(\d+)\s+(\d+)\s+255\s", raw)
    if match is None:
        raise ValueError(f"unexpected PGM format: {pgm_path}")
    width = int(match[1])
    height = int(match[2])
    pixels = bytearray(raw[match.end():])
    if len(pixels) != width * height:
        raise ValueError(
            f"unexpected PGM payload: {pgm_path} "
            f"({len(pixels)} != {width}x{height})"
        )
    return width, height, pixels


def _widen_and_expand_map(map_dir: Path) -> None:
    map_yaml = yaml.safe_load(
        (map_dir / f"{MAP_NAME}.yaml").read_text(encoding="utf-8")
    )
    pgm_path = map_dir / str(map_yaml["image"])
    width, height, pixels = _parse_pgm(pgm_path.read_bytes(), pgm_path)
    if (width, height) == (PGM_WIDTH, PGM_HEIGHT):
        return
    if (width, height) != (BASE_PGM_WIDTH, BASE_PGM_HEIGHT):
        raise ValueError(
            f"unexpected PGM dimensions: {pgm_path} "
            f"({width}x{height})"
        )

    # Open the shelf ends before scaling.  Scaling the complete occupancy
    # raster keeps PGM obstacles, graph coordinates and traffic zones in the
    # same frame while increasing both corridor width and LM separation.
    for row_start in SHELF_PIXEL_ROW_STARTS:
        for y in range(row_start, row_start + 10):
            row_offset = y * BASE_PGM_WIDTH
            for column_start, column_end in SHELF_END_TRIM_RANGES:
                pixels[
                    row_offset + column_start : row_offset + column_end
                ] = b"\xfe" * (column_end - column_start)

    expanded = bytearray(PGM_WIDTH * PGM_HEIGHT)
    for target_y in range(PGM_HEIGHT):
        source_y = min(
            BASE_PGM_HEIGHT - 1,
            (target_y * BASE_PGM_HEIGHT) // PGM_HEIGHT,
        )
        source_row = source_y * BASE_PGM_WIDTH
        target_row = target_y * PGM_WIDTH
        for target_x in range(PGM_WIDTH):
            source_x = min(
                BASE_PGM_WIDTH - 1,
                (target_x * BASE_PGM_WIDTH) // PGM_WIDTH,
            )
            expanded[target_row + target_x] = pixels[source_row + source_x]

    pgm_path.write_bytes(
        f"P5\n{PGM_WIDTH} {PGM_HEIGHT}\n255\n".encode("ascii") + expanded
    )


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
            "x": _lane_x(column),
            "y": _lane_y(row),
        }
        properties = dict(landmark.get("properties") or {})
        for legacy_key in (
            "controlled_region",
            "controlled_region_capacity",
            "holding_point",
            "can_wait",
        ):
            properties.pop(legacy_key, None)
        landmark["properties"] = properties
        landmarks_by_grid[(row, column)] = landmark

    row_numbers = sorted({row for row, _column in landmarks_by_grid})
    edges: list[dict[str, Any]] = []

    def connect(start: dict[str, Any], goal: dict[str, Any]) -> None:
        edges.append(_edge(start, goal, motion_direction=0))
        edges.append(_edge(goal, start, motion_direction=1))

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
    _write_yaml(
        map_dir / "traffic_zones.yaml",
        {
            "mapName": MAP_NAME,
            "coordinateFrame": "map_top_left",
            "zones": [
                {
                    "id": (
                        "corridor:smart-kiva:"
                        f"c{column:03d}:r{top:03d}-r{bottom:03d}"
                    ),
                    "kind": "controlled_corridor",
                    "shape": "rectangle",
                    "bounds": {
                        "minX": round(
                            float(landmarks_by_grid[(top, column)]["x"])
                            - _scaled(0.18),
                            6,
                        ),
                        "minY": round(
                            min(
                                float(landmarks_by_grid[(top, column)]["y"]),
                                float(landmarks_by_grid[(bottom, column)]["y"]),
                            ) - _scaled(0.08),
                            6,
                        ),
                        "maxX": round(
                            float(landmarks_by_grid[(top, column)]["x"])
                            + _scaled(0.18),
                            6,
                        ),
                        "maxY": round(
                            max(
                                float(landmarks_by_grid[(top, column)]["y"]),
                                float(landmarks_by_grid[(bottom, column)]["y"]),
                            ) + _scaled(0.08),
                            6,
                        ),
                    },
                    "capacity": 1,
                    "properties": {
                        "policy": "traffic_light",
                    },
                }
                for column in sorted(CONTROLLED_CORRIDOR_COLUMNS)
                for top, bottom in CONTROLLED_CORRIDOR_ROW_PAIRS
            ],
        },
    )
    _widen_and_expand_map(map_dir)

    metadata_path = map_dir / ".operator_meta.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata.update(
        {
            "directedEdges": len(edges),
            "landmarks": len(landmarks),
            "layoutVariant": "two_lane_expanded_clearance",
            "mapScale": round(MAP_SCALE, 6),
            "cellSizeM": _scaled(1.0),
            "imageWidthPx": PGM_WIDTH,
            "imageHeightPx": PGM_HEIGHT,
            "aisleLaneClearanceM": _scaled(0.8),
            "aisleLaneSpacingM": _scaled(1.4),
            "aisleConnectorColumns": sorted(AISLE_CONNECTOR_COLUMNS),
            "crossLaneConnectionsPerAisle": len(AISLE_CONNECTOR_COLUMNS),
            "controlledCorridorColumns": sorted(CONTROLLED_CORRIDOR_COLUMNS),
            "controlledCorridorRowPairs": [
                list(pair)
                for pair in CONTROLLED_CORRIDOR_ROW_PAIRS
            ],
            "controlledCorridorCount": (
                len(CONTROLLED_CORRIDOR_COLUMNS)
                * len(CONTROLLED_CORRIDOR_ROW_PAIRS)
            ),
            "controlledCorridorMode": "explicit_shelf_crossings",
            "internalCrossAisleWidthM": _scaled(1.4),
            "perimeterLaneClearanceM": _scaled(1.0),
            "perimeterLaneCount": 1,
            "removedMiddleRows": len(MIDDLE_ROWS),
            "shelfEndTrimM": _scaled(0.2),
        }
    )
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    rebuild()
