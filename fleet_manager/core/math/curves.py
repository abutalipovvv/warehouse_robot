"""Reusable Bézier-curve calculations for maps and route planning."""

from __future__ import annotations

import math
from numbers import Integral, Real
from typing import Sequence

from .geometry import Vector2


def bezier_point(
    control_points: Sequence[Vector2],
    fraction: Real,
) -> Vector2:
    """Evaluate a Bézier curve of any degree with De Casteljau's method."""

    if not control_points:
        raise ValueError("a Bézier curve needs at least one control point")
    points = list(control_points)
    while len(points) > 1:
        points = [
            start.lerp(end, fraction)
            for start, end in zip(points, points[1:])
        ]
    return points[0]


def cubic_bezier_point(
    start: Vector2,
    control1: Vector2,
    control2: Vector2,
    end: Vector2,
    fraction: Real,
) -> Vector2:
    """Evaluate a cubic Bézier curve at ``fraction``."""

    value = float(fraction)
    inverse = 1.0 - value
    return Vector2(
        (inverse * inverse * inverse) * start.x
        + 3 * (inverse * inverse) * value * control1.x
        + 3 * inverse * (value * value) * control2.x
        + (value * value * value) * end.x,
        (inverse * inverse * inverse) * start.y
        + 3 * (inverse * inverse) * value * control1.y
        + 3 * inverse * (value * value) * control2.y
        + (value * value * value) * end.y,
    )


def cubic_bezier_derivative(
    start: Vector2,
    control1: Vector2,
    control2: Vector2,
    end: Vector2,
    fraction: Real,
) -> Vector2:
    """Return the tangent vector of a cubic Bézier curve."""

    value = float(fraction)
    inverse = 1.0 - value
    return Vector2(
        (3.0 * inverse * inverse * (control1.x - start.x))
        + (6.0 * inverse * value * (control2.x - control1.x))
        + (3.0 * value * value * (end.x - control2.x)),
        (3.0 * inverse * inverse * (control1.y - start.y))
        + (6.0 * inverse * value * (control2.y - control1.y))
        + (3.0 * value * value * (end.y - control2.y)),
    )


def cubic_bezier_length(
    start: Vector2,
    control1: Vector2,
    control2: Vector2,
    end: Vector2,
    *,
    steps: int = 200,
) -> float:
    """Approximate cubic Bézier length with ordered line segments."""

    if isinstance(steps, bool) or not isinstance(steps, Integral):
        raise TypeError("steps must be an integer")
    if steps <= 0:
        raise ValueError("steps must be positive")

    previous_x = start.x
    previous_y = start.y
    total = 0.0
    for index in range(1, steps + 1):
        fraction = index / steps
        inverse = 1.0 - fraction
        point_x = (
            (inverse * inverse * inverse) * start.x
            + 3 * (inverse * inverse) * fraction * control1.x
            + 3 * inverse * (fraction * fraction) * control2.x
            + (fraction * fraction * fraction) * end.x
        )
        point_y = (
            (inverse * inverse * inverse) * start.y
            + 3 * (inverse * inverse) * fraction * control1.y
            + 3 * inverse * (fraction * fraction) * control2.y
            + (fraction * fraction * fraction) * end.y
        )
        total += math.hypot(point_x - previous_x, point_y - previous_y)
        previous_x = point_x
        previous_y = point_y
    return total
