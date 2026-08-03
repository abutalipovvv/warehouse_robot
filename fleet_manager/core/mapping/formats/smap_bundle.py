"""Parse an SMAP JSON document into a typed, writable map bundle."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from io import StringIO
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

from fleet_manager.core.io.atomic_files import atomic_write_bytes, atomic_write_text
from fleet_manager.core.math.curves import (
    cubic_bezier_length,
    cubic_bezier_point,
)
from fleet_manager.core.math.geometry import Vector2

from .smap_raster import (
    OCCUPIED_CELL,
    OccupancyRaster,
    SmapHeader,
    point_payload,
    vector_from_payload,
)


Primitive = dict[str, Any]
LandmarkPayload = dict[str, Any]
EdgePayload = dict[str, Any]
LengthRow = dict[str, Any]


@dataclass(frozen=True, slots=True)
class ParsedItems:
    """Source count and accepted/skipped counts for one SMAP collection."""

    total: int | None
    used: int
    skipped: int


@dataclass(slots=True)
class SmapBundle:
    """All artifacts derived from one SMAP document before persistence."""

    header: SmapHeader
    raster: OccupancyRaster
    line_primitives: list[Primitive]
    curve_primitives: list[Primitive]
    landmarks: list[LandmarkPayload]
    edges: list[EdgePayload]
    length_rows: list[LengthRow]
    normal_points: ParsedItems
    lines: ParsedItems
    curves: ParsedItems
    advanced_points_total: int | None
    top_level_keys: list[str]

    @property
    def primitives(self) -> list[Primitive]:
        return [*self.line_primitives, *self.curve_primitives]

    def summary(self) -> dict[str, Any]:
        line_lengths = [
            primitive.get("length_m")
            for primitive in self.line_primitives
        ]
        curve_lengths = [
            primitive.get("length_m")
            for primitive in self.curve_primitives
        ]
        line_type_sums: dict[str, float] = {}
        for primitive in self.line_primitives:
            length = primitive.get("length_m")
            if isinstance(length, (int, float)):
                line_type = str(primitive.get("line_type"))
                line_type_sums[line_type] = (
                    line_type_sums.get(line_type, 0.0) + float(length)
                )

        map_name = self.header.map_name
        return {
            "header": self.header.raw,
            "counts": {
                "normalPosList_total": self.normal_points.total,
                "normalPosList_used": self.normal_points.used,
                "normalPosList_skipped": self.normal_points.skipped,
                "advancedLineList_total": self.lines.total,
                "lines_used": self.lines.used,
                "lines_skipped": self.lines.skipped,
                "advancedCurveList_total": self.curves.total,
                "curves_used": self.curves.used,
                "curves_skipped": self.curves.skipped,
                "advancedPointList_total": self.advanced_points_total,
                "LMs_found": len(self.landmarks),
                "edges_total": len(self.edges),
                "grid": {
                    "width": self.header.width,
                    "height": self.header.height,
                },
            },
            "lengths_m": {
                "lines": length_statistics(line_lengths),
                "curves": length_statistics(curve_lengths),
                "all": length_statistics([*line_lengths, *curve_lengths]),
                "line_type_sum": line_type_sums,
            },
            "top_level_keys": self.top_level_keys,
            "outputs": {
                "pgm": f"{map_name}.pgm",
                "ros_map_yaml": f"{map_name}.yaml",
                "LMs_yaml": "LMs.yaml",
                "graphs_yaml": "graphs.yaml",
                "graph_edges_lengths_yaml": "graph_edges_lengths.yaml",
                "primitives_lengths_csv": "primitives_lengths.csv",
                "summary_json": "smap_summary.json",
            },
        }


class SmapDocumentParser:
    """Convert one decoded SMAP mapping into independent bundle artifacts."""

    def __init__(
        self,
        document: Mapping[str, Any],
        *,
        fallback_name: str,
    ) -> None:
        self.document = document
        self.header = SmapHeader.from_document(
            document,
            fallback_name=fallback_name,
        )
        self.raster = OccupancyRaster(self.header)

    def parse(self) -> SmapBundle:
        normal_points = self._draw_normal_points()
        line_primitives, line_rows, lines = self._parse_lines()
        curve_primitives, curve_rows, curves = self._parse_curves()
        landmarks, advanced_points_total = self._parse_landmarks()
        edges = GraphEdgeBuilder(landmarks).build(
            line_primitives,
            curve_primitives,
        )
        return SmapBundle(
            header=self.header,
            raster=self.raster,
            line_primitives=line_primitives,
            curve_primitives=curve_primitives,
            landmarks=landmarks,
            edges=edges,
            length_rows=[*line_rows, *curve_rows],
            normal_points=normal_points,
            lines=lines,
            curves=curves,
            advanced_points_total=advanced_points_total,
            top_level_keys=sorted(self.document),
        )

    def _draw_normal_points(self) -> ParsedItems:
        source = self.document.get("normalPosList")
        if not isinstance(source, list):
            return ParsedItems(total=None, used=0, skipped=0)
        used, skipped = self.raster.mark_raw_points(
            source,
            value=OCCUPIED_CELL,
        )
        return ParsedItems(total=len(source), used=used, skipped=skipped)

    def _parse_lines(
        self,
    ) -> tuple[list[Primitive], list[LengthRow], ParsedItems]:
        source = self.document.get("advancedLineList")
        if not isinstance(source, list):
            return [], [], ParsedItems(total=None, used=0, skipped=0)

        primitives: list[Primitive] = []
        rows: list[LengthRow] = []
        skipped = 0
        for index, item in enumerate(source):
            if not isinstance(item, dict):
                skipped += 1
                continue
            line = item.get("line")
            line = line if isinstance(line, dict) else {}
            start = vector_from_payload(line.get("startPos"))
            end = vector_from_payload(line.get("endPos"))
            if start is None or end is None:
                skipped += 1
                continue

            self.raster.draw_line(start, end, value=OCCUPIED_CELL)
            length = start.distance_to(end)
            properties = parse_properties(item.get("property"))
            line_type = item.get("className")
            primitives.append(
                {
                    "kind": "line",
                    "line_type": line_type,
                    "start": point_payload(start),
                    "end": point_payload(end),
                    "properties": properties,
                    "length_m": float(length),
                }
            )
            rows.append(
                length_row(
                    index=index,
                    kind="line",
                    primitive_type=line_type,
                    start=start,
                    end=end,
                    length=length,
                )
            )

        return (
            primitives,
            rows,
            ParsedItems(
                total=len(source),
                used=len(primitives),
                skipped=skipped,
            ),
        )

    def _parse_curves(
        self,
    ) -> tuple[list[Primitive], list[LengthRow], ParsedItems]:
        source = self.document.get("advancedCurveList")
        if not isinstance(source, list):
            return [], [], ParsedItems(total=None, used=0, skipped=0)

        primitives: list[Primitive] = []
        rows: list[LengthRow] = []
        skipped = 0
        for index, item in enumerate(source):
            if not isinstance(item, dict):
                skipped += 1
                continue

            start = vector_from_payload(item.get("startPos"))
            end = vector_from_payload(item.get("endPos"))
            control1 = vector_from_payload(item.get("controlPos1"))
            control2 = vector_from_payload(item.get("controlPos2"))
            length = (
                cubic_bezier_length(start, control1, control2, end)
                if all(
                    point is not None
                    for point in (start, control1, control2, end)
                )
                else None
            )
            start_source = item.get("startPos")
            end_source = item.get("endPos")
            curve = {
                "start": point_payload(start),
                "end": point_payload(end),
                "control1": point_payload(control1),
                "control2": point_payload(control2),
                "start_name": (
                    start_source.get("instanceName")
                    if isinstance(start_source, dict)
                    else None
                ),
                "end_name": (
                    end_source.get("instanceName")
                    if isinstance(end_source, dict)
                    else None
                ),
            }
            curve_type = item.get("className")
            primitives.append(
                {
                    "kind": "curve",
                    "curve_type": curve_type,
                    "curve": curve,
                    "properties": parse_properties(item.get("property")),
                    "length_m": float(length) if length is not None else None,
                }
            )
            rows.append(
                length_row(
                    index=index,
                    kind="curve",
                    primitive_type=curve_type,
                    start=start,
                    end=end,
                    length=length,
                )
            )

        return (
            primitives,
            rows,
            ParsedItems(
                total=len(source),
                used=len(primitives),
                skipped=skipped,
            ),
        )

    def _parse_landmarks(
        self,
    ) -> tuple[list[LandmarkPayload], int | None]:
        source = self.document.get("advancedPointList")
        if not isinstance(source, list):
            return [], None

        landmarks: list[LandmarkPayload] = []
        for item in source:
            if (
                not isinstance(item, dict)
                or item.get("className") != "LocationMark"
            ):
                continue
            name = (
                item.get("instanceName")
                or item.get("name")
                or item.get("id")
            )
            position = vector_from_payload(item.get("pos"))
            if name is None or position is None:
                continue
            landmarks.append(
                {
                    "name": str(name),
                    "x": position.x,
                    "y": position.y,
                    "ignoreDir": item.get("ignoreDir"),
                    "properties": parse_properties(item.get("property")),
                }
            )
        return landmarks, len(source)


class LandmarkIndex:
    """Nearest-landmark lookup used while reconstructing graph endpoints."""

    __slots__ = ("_positions",)

    def __init__(self, landmarks: Iterable[LandmarkPayload]) -> None:
        self._positions = {
            str(item["name"]): (float(item["x"]), float(item["y"]))
            for item in landmarks
        }

    def contains(self, name: Any) -> bool:
        return name in self._positions

    def nearest(
        self,
        point: Vector2,
        *,
        max_radius: float = 0.75,
    ) -> str | None:
        best_name: str | None = None
        best_distance = math.inf
        point_x = point.x
        point_y = point.y
        for name, (candidate_x, candidate_y) in self._positions.items():
            distance = math.hypot(
                candidate_x - point_x,
                candidate_y - point_y,
            )
            if distance < best_distance:
                best_name = name
                best_distance = distance
        if best_name is None or best_distance > max_radius:
            return None
        return best_name


class GraphEdgeBuilder:
    """Snap primitive endpoints to landmarks and deduplicate graph edges."""

    def __init__(self, landmarks: Iterable[LandmarkPayload]) -> None:
        self.landmarks = LandmarkIndex(landmarks)

    def build(
        self,
        lines: Iterable[Primitive],
        curves: Iterable[Primitive],
    ) -> list[EdgePayload]:
        edges: list[EdgePayload] = []
        for primitive in lines:
            start = vector_from_payload(primitive.get("start"))
            end = vector_from_payload(primitive.get("end"))
            if start is None or end is None:
                continue
            self._append(
                edges,
                source=self.landmarks.nearest(start),
                target=self.landmarks.nearest(end),
                length=primitive.get("length_m"),
                kind="line",
                primitive_type=str(primitive.get("line_type")),
                properties=_mapping(primitive.get("properties")),
            )

        for primitive in curves:
            curve = _mapping(primitive.get("curve"))
            start = vector_from_payload(curve.get("start"))
            end = vector_from_payload(curve.get("end"))
            if start is None or end is None:
                continue
            start_name = curve.get("start_name")
            end_name = curve.get("end_name")
            self._append(
                edges,
                source=(
                    str(start_name)
                    if self.landmarks.contains(start_name)
                    else self.landmarks.nearest(start)
                ),
                target=(
                    str(end_name)
                    if self.landmarks.contains(end_name)
                    else self.landmarks.nearest(end)
                ),
                length=primitive.get("length_m"),
                kind="curve",
                primitive_type=str(primitive.get("curve_type")),
                properties=_mapping(primitive.get("properties")),
            )

        shortest_by_direction: dict[tuple[str, str], EdgePayload] = {}
        for edge in edges:
            key = (edge["from"], edge["to"])
            current = shortest_by_direction.get(key)
            if current is None or edge["length"] < current["length"]:
                shortest_by_direction[key] = edge
        return list(shortest_by_direction.values())

    @staticmethod
    def _append(
        edges: list[EdgePayload],
        *,
        source: str | None,
        target: str | None,
        length: Any,
        kind: str,
        primitive_type: str,
        properties: Mapping[str, Any],
    ) -> None:
        if not source or not target or source == target or length is None:
            return
        try:
            numeric_length = float(length)
        except (TypeError, ValueError, OverflowError):
            return
        if not math.isfinite(numeric_length) or numeric_length <= 0.0:
            return
        edges.append(
            {
                "from": source,
                "to": target,
                "length": numeric_length,
                "kind": kind,
                "type": primitive_type,
                "properties": dict(properties),
            }
        )


class SmapBundleWriter:
    """Persist a parsed bundle with atomic replacement per output file."""

    CSV_FIELDS = (
        "idx",
        "kind",
        "type",
        "start_x",
        "start_y",
        "end_x",
        "end_y",
        "length_m",
    )

    def write(self, bundle: SmapBundle, output_dir: Path) -> None:
        output_dir.mkdir(parents=True, exist_ok=True)
        map_name = bundle.header.map_name
        pgm_path = output_dir / f"{map_name}.pgm"

        atomic_write_bytes(pgm_path, bundle.raster.pgm_bytes())
        self._write_yaml(
            output_dir / f"{map_name}.yaml",
            {
                "image": pgm_path.name,
                "resolution": bundle.header.resolution,
                "origin": [
                    bundle.header.minimum.x,
                    bundle.header.minimum.y,
                    0.0,
                ],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
                "mode": "trinary",
            },
        )
        self._write_yaml(
            output_dir / "graphs.yaml",
            {"mapName": map_name, "primitives": bundle.primitives},
        )
        self._write_yaml(
            output_dir / "LMs.yaml",
            {"mapName": map_name, "LMs": bundle.landmarks},
        )
        self._write_yaml(
            output_dir / "graph_edges_lengths.yaml",
            bundle.edges,
        )
        atomic_write_text(
            output_dir / "primitives_lengths.csv",
            self._length_csv(bundle.length_rows),
        )
        atomic_write_text(
            output_dir / "smap_summary.json",
            json.dumps(bundle.summary(), ensure_ascii=False, indent=2),
        )

    @staticmethod
    def _write_yaml(path: Path, payload: Any) -> None:
        atomic_write_text(
            path,
            yaml.safe_dump(
                payload,
                sort_keys=False,
                allow_unicode=True,
            ),
        )

    @classmethod
    def _length_csv(cls, rows: Iterable[LengthRow]) -> str:
        output = StringIO(newline="")
        writer = csv.DictWriter(output, fieldnames=cls.CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
        return output.getvalue()


def parse_properties(value: Any) -> dict[str, Any]:
    if not isinstance(value, list):
        return {}
    properties: dict[str, Any] = {}
    for item in value:
        if not isinstance(item, dict) or item.get("key") is None:
            continue
        properties[str(item["key"])] = property_value(item)
    return properties


def property_value(item: Mapping[str, Any]) -> Any:
    for key in ("int32Value", "doubleValue", "boolValue", "value"):
        if item.get(key) is not None:
            return item[key]
    return None


def length_statistics(values: Iterable[Any]) -> dict[str, Any]:
    finite_values = [
        value
        for value in values
        if isinstance(value, (int, float)) and math.isfinite(value)
    ]
    if not finite_values:
        return {
            "count": 0,
            "sum": 0.0,
            "min": None,
            "max": None,
            "mean": None,
        }
    total = float(sum(finite_values))
    return {
        "count": len(finite_values),
        "sum": total,
        "min": float(min(finite_values)),
        "max": float(max(finite_values)),
        "mean": float(total / len(finite_values)),
    }


def length_row(
    *,
    index: int,
    kind: str,
    primitive_type: Any,
    start: Vector2 | None,
    end: Vector2 | None,
    length: float | None,
) -> LengthRow:
    return {
        "idx": index,
        "kind": kind,
        "type": str(primitive_type),
        "start_x": start.x if start is not None else None,
        "start_y": start.y if start is not None else None,
        "end_x": end.x if end is not None else None,
        "end_y": end.y if end is not None else None,
        "length_m": float(length) if length is not None else None,
    }


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, dict) else {}


__all__ = [
    "GraphEdgeBuilder",
    "LandmarkIndex",
    "ParsedItems",
    "SmapBundle",
    "SmapBundleWriter",
    "SmapDocumentParser",
    "cubic_bezier_length",
    "cubic_bezier_point",
    "length_statistics",
    "parse_properties",
    "property_value",
]
