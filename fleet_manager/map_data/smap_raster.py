"""Raster primitives used while converting an RDS ``.smap`` document."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Iterable, Mapping

from fleet_manager.math import Vector2


FREE_CELL = 254
OCCUPIED_CELL = 0


@dataclass(frozen=True, slots=True)
class SmapHeader:
    """Validated map coordinate system and raster dimensions."""

    map_name: str
    minimum: Vector2
    maximum: Vector2
    resolution: float
    raw: Mapping[str, Any]
    width: int = field(init=False)
    height: int = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "width",
            int(
                round(
                    (self.maximum.x - self.minimum.x) / self.resolution
                )
            )
            + 1,
        )
        object.__setattr__(
            self,
            "height",
            int(
                round(
                    (self.maximum.y - self.minimum.y) / self.resolution
                )
            )
            + 1,
        )

    @classmethod
    def from_document(
        cls,
        document: Mapping[str, Any],
        *,
        fallback_name: str,
    ) -> "SmapHeader":
        raw_header = document.get("header")
        raw = raw_header if isinstance(raw_header, dict) else {}
        minimum = _vector(raw.get("minPos"), default=Vector2(0.0, 0.0))
        maximum = _vector(raw.get("maxPos"), default=Vector2(0.0, 0.0))
        resolution = _number(raw.get("resolution"), default=0.05)

        if (
            not isfinite(resolution)
            or resolution <= 0.0
            or maximum.x <= minimum.x
            or maximum.y <= minimum.y
        ):
            raise ValueError("Bad header bounds/resolution; can't build grid.")

        return cls(
            map_name=str(raw.get("mapName", fallback_name)),
            minimum=minimum,
            maximum=maximum,
            resolution=resolution,
            raw=raw,
        )

    def world_to_grid(self, point: Vector2) -> tuple[int, int]:
        """Map an SMAP cell centre to top-left-origin image coordinates."""

        return self.world_to_grid_xy(point.x, point.y)

    def world_to_grid_xy(self, x: float, y: float) -> tuple[int, int]:
        grid_x = int(round((x - self.minimum.x) / self.resolution))
        grid_y = int(round((y - self.minimum.y) / self.resolution))
        return grid_x, (self.height - 1) - grid_y


class OccupancyRaster:
    """Mutable 8-bit occupancy raster with bounded drawing operations."""

    __slots__ = ("header", "_rows")

    def __init__(self, header: SmapHeader) -> None:
        self.header = header
        self._rows = [
            bytearray([FREE_CELL]) * header.width
            for _ in range(header.height)
        ]

    def mark(
        self,
        point: Vector2,
        *,
        value: int = OCCUPIED_CELL,
        radius_pixels: int = 0,
    ) -> None:
        self.mark_xy(
            point.x,
            point.y,
            value=value,
            radius_pixels=radius_pixels,
        )

    def mark_xy(
        self,
        x: float,
        y: float,
        *,
        value: int = OCCUPIED_CELL,
        radius_pixels: int = 0,
    ) -> None:
        """Fast coordinate overload for large raw occupancy point clouds."""

        centre_x, centre_y = self.header.world_to_grid_xy(x, y)
        radius = max(0, int(radius_pixels))
        for y in range(centre_y - radius, centre_y + radius + 1):
            if not 0 <= y < self.header.height:
                continue
            row = self._rows[y]
            for x in range(centre_x - radius, centre_x + radius + 1):
                if 0 <= x < self.header.width:
                    row[x] = value

    def mark_raw_points(
        self,
        points: Iterable[Any],
        *,
        value: int = OCCUPIED_CELL,
    ) -> tuple[int, int]:
        """Rasterize a large raw ``{x, y}`` collection in one tight pass."""

        minimum_x = self.header.minimum.x
        minimum_y = self.header.minimum.y
        resolution = self.header.resolution
        width = self.header.width
        height = self.header.height
        rows = self._rows
        used = 0
        skipped = 0

        for item in points:
            if (
                not isinstance(item, dict)
                or "x" not in item
                or "y" not in item
            ):
                skipped += 1
                continue
            try:
                x = float(item["x"])
                y = float(item["y"])
            except (TypeError, ValueError, OverflowError):
                skipped += 1
                continue
            grid_x = int(round((x - minimum_x) / resolution))
            grid_y = int(round((y - minimum_y) / resolution))
            image_y = (height - 1) - grid_y
            if 0 <= grid_x < width and 0 <= image_y < height:
                rows[image_y][grid_x] = value
            used += 1
        return used, skipped

    def draw_line(
        self,
        start: Vector2,
        end: Vector2,
        *,
        value: int = OCCUPIED_CELL,
    ) -> None:
        """Rasterize one segment with integer Bresenham stepping."""

        x0, y0 = self.header.world_to_grid(start)
        x1, y1 = self.header.world_to_grid(end)
        delta_x = abs(x1 - x0)
        delta_y = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = delta_x - delta_y

        x, y = x0, y0
        while True:
            if 0 <= x < self.header.width and 0 <= y < self.header.height:
                self._rows[y][x] = value
            if x == x1 and y == y1:
                return
            doubled_error = 2 * error
            if doubled_error > -delta_y:
                error -= delta_y
                x += step_x
            if doubled_error < delta_x:
                error += delta_x
                y += step_y

    def pgm_bytes(self) -> bytes:
        header = (
            f"P5\n{self.header.width} {self.header.height}\n255\n"
        ).encode("ascii")
        return header + b"".join(self._rows)


def vector_from_payload(value: Any) -> Vector2 | None:
    """Read ``{x, y}`` or ``{pos: {x, y}}`` coordinates."""

    if not isinstance(value, dict):
        return None
    coordinates = value
    if "x" not in coordinates or "y" not in coordinates:
        nested = value.get("pos")
        if not isinstance(nested, dict):
            return None
        coordinates = nested
    if "x" not in coordinates or "y" not in coordinates:
        return None
    try:
        return Vector2(float(coordinates["x"]), float(coordinates["y"]))
    except (TypeError, ValueError, OverflowError):
        return None


def point_payload(point: Vector2 | None) -> dict[str, float] | None:
    if point is None:
        return None
    return {"x": point.x, "y": point.y}


def _vector(value: Any, *, default: Vector2) -> Vector2:
    return vector_from_payload(value) or default


def _number(value: Any, *, default: float) -> float:
    try:
        return float(default if value is None else value)
    except (TypeError, ValueError, OverflowError):
        return default


__all__ = [
    "FREE_CELL",
    "OCCUPIED_CELL",
    "OccupancyRaster",
    "SmapHeader",
    "point_payload",
    "vector_from_payload",
]
