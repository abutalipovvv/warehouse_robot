"""Dependency-free polygon geometry for footprint collision checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .geometry import Pose2D, Vector2


@dataclass(frozen=True, slots=True)
class Projection:
    minimum: float
    maximum: float

    def is_separated_from(
        self,
        other: "Projection",
        *,
        margin: float = 0.0,
    ) -> bool:
        padding = max(0.0, float(margin))
        return (
            self.maximum + padding < other.minimum
            or other.maximum + padding < self.minimum
        )


@dataclass(frozen=True, slots=True)
class Polygon2D:
    """An immutable polygon with SAT and distance operations."""

    points: tuple[Vector2, ...]

    def __init__(self, points: Iterable[Vector2]) -> None:
        values = tuple(points)
        if len(values) < 3:
            raise ValueError("a polygon needs at least three points")
        if not all(isinstance(point, Vector2) for point in values):
            raise TypeError("polygon points must be Vector2 values")
        object.__setattr__(self, "points", values)

    def transformed(self, pose: Pose2D) -> "Polygon2D":
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        return Polygon2D(
            Vector2(
                pose.x + (point.x * cosine) - (point.y * sine),
                pose.y + (point.x * sine) + (point.y * cosine),
            )
            for point in self.points
        )

    def axes(self) -> tuple[Vector2, ...]:
        axes: list[Vector2] = []
        for start, end in self.edges():
            edge = end - start
            if edge.length <= 0.000001:
                continue
            axes.append(Vector2(-edge.y, edge.x) / edge.length)
        return tuple(axes)

    def project(self, axis: Vector2) -> Projection:
        values = tuple(point.dot(axis) for point in self.points)
        return Projection(min(values), max(values))

    def overlaps(self, other: "Polygon2D", *, margin: float = 0.0) -> bool:
        return _coordinates_overlap(
            tuple((point.x, point.y) for point in self.points),
            tuple((point.x, point.y) for point in other.points),
            margin=margin,
        )

    def overlaps_transformed(
        self,
        pose: Pose2D,
        other: "Polygon2D",
        other_pose: Pose2D,
        *,
        margin: float = 0.0,
    ) -> bool:
        """Test two positioned polygons without allocating transformed models."""

        return _coordinates_overlap(
            _transformed_coordinates(self.points, pose),
            _transformed_coordinates(other.points, other_pose),
            margin=margin,
        )

    def overlaps_positioned(
        self,
        x: float,
        y: float,
        yaw: float,
        other: "Polygon2D",
        other_x: float,
        other_y: float,
        other_yaw: float,
        *,
        margin: float = 0.0,
    ) -> bool:
        """Test positioned polygons from scalar coordinates.

        Runtime collision loops already normalize their pose payloads to
        floats.  This form keeps the SAT implementation in the math layer but
        avoids allocating two short-lived :class:`Pose2D` objects per pair.
        """

        return _coordinates_overlap(
            _positioned_coordinates(self.points, x, y, yaw),
            _positioned_coordinates(
                other.points,
                other_x,
                other_y,
                other_yaw,
            ),
            margin=margin,
        )

    def contains(self, point: Vector2) -> bool:
        inside = False
        previous = self.points[-1]
        for current in self.points:
            crosses = (
                (current.y > point.y) != (previous.y > point.y)
                and point.x
                < (
                    ((previous.x - current.x) * (point.y - current.y))
                    / ((previous.y - current.y) or 0.000001)
                )
                + current.x
            )
            if crosses:
                inside = not inside
            previous = current
        return inside

    def distance_to(self, point: Vector2) -> float:
        return min(
            distance_to_segment(point, start, end)
            for start, end in self.edges()
        )

    def edges(self) -> tuple[tuple[Vector2, Vector2], ...]:
        return tuple(
            (point, self.points[(index + 1) % len(self.points)])
            for index, point in enumerate(self.points)
        )


def _transformed_coordinates(
    points: tuple[Vector2, ...],
    pose: Pose2D,
) -> tuple[tuple[float, float], ...]:
    return _positioned_coordinates(
        points,
        pose.x,
        pose.y,
        pose.yaw,
    )


def _positioned_coordinates(
    points: tuple[Vector2, ...],
    x: float,
    y: float,
    yaw: float,
) -> tuple[tuple[float, float], ...]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return tuple(
        (
            x + (point.x * cosine) - (point.y * sine),
            y + (point.x * sine) + (point.y * cosine),
        )
        for point in points
    )


def _coordinates_overlap(
    first: tuple[tuple[float, float], ...],
    second: tuple[tuple[float, float], ...],
    *,
    margin: float,
) -> bool:
    padding = max(0.0, float(margin))
    for points in (first, second):
        for index, start in enumerate(points):
            end = points[(index + 1) % len(points)]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length <= 0.000001:
                continue
            axis_x = -dy / length
            axis_y = dx / length
            first_minimum, first_maximum = _projection_extents(
                first,
                axis_x,
                axis_y,
            )
            second_minimum, second_maximum = _projection_extents(
                second,
                axis_x,
                axis_y,
            )
            if (
                first_maximum + padding < second_minimum
                or second_maximum + padding < first_minimum
            ):
                return False
    return True


def _projection_extents(
    points: tuple[tuple[float, float], ...],
    axis_x: float,
    axis_y: float,
) -> tuple[float, float]:
    first_x, first_y = points[0]
    minimum = maximum = (first_x * axis_x) + (first_y * axis_y)
    for point_x, point_y in points[1:]:
        value = (point_x * axis_x) + (point_y * axis_y)
        if value < minimum:
            minimum = value
        elif value > maximum:
            maximum = value
    return minimum, maximum


def distance_to_segment(
    point: Vector2,
    start: Vector2,
    end: Vector2,
) -> float:
    segment = end - start
    if segment.length_squared <= 0.000001:
        return point.distance_to(start)
    fraction = max(
        0.0,
        min(1.0, (point - start).dot(segment) / segment.length_squared),
    )
    return point.distance_to(start + (segment * fraction))
