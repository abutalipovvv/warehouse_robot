"""Two-dimensional geometry primitives used by planning code."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real


def _finite_number(value: Real, *, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real number")
    if isinstance(value, (int, float)):
        result = float(value)
    elif isinstance(value, Real):
        result = float(value)
    else:
        raise TypeError(f"{name} must be a real number")
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def normalize_angle(angle: Real) -> float:
    """Return an angle in the half-open range ``[-pi, pi)``."""

    angle_value = _finite_number(angle, name="angle")
    return (angle_value + math.pi) % math.tau - math.pi


def normalize_angle_rounded(angle: Real, *, digits: int = 9) -> float:
    """Normalize an angle with stable rounding for discrete planners."""

    angle_value = _finite_number(angle, name="angle")
    return round(
        math.atan2(math.sin(angle_value), math.cos(angle_value)),
        digits,
    )


def shortest_angular_distance(from_angle: Real, to_angle: Real) -> float:
    """Return the signed shortest rotation from one angle to another."""

    start = _finite_number(from_angle, name="from_angle")
    target = _finite_number(to_angle, name="to_angle")
    return normalize_angle(target - start)


@dataclass(frozen=True, slots=True)
class Vector2:
    """An immutable vector in a two-dimensional Cartesian coordinate system."""

    x: float
    y: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _finite_number(self.x, name="x"))
        object.__setattr__(self, "y", _finite_number(self.y, name="y"))

    @property
    def length_squared(self) -> float:
        return (self.x * self.x) + (self.y * self.y)

    @property
    def length(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def angle(self) -> float:
        """Return the vector direction in radians.

        The zero vector has angle ``0.0``, matching :func:`math.atan2`.
        """

        return math.atan2(self.y, self.x)

    def distance_to(self, other: Vector2) -> float:
        return (other - self).length

    def dot(self, other: Vector2) -> float:
        return (self.x * other.x) + (self.y * other.y)

    def cross(self, other: Vector2) -> float:
        """Return the scalar z component of the 2D cross product."""

        return (self.x * other.y) - (self.y * other.x)

    def normalized(self) -> Vector2:
        if self.length == 0.0:
            raise ValueError("cannot normalize a zero-length vector")
        return self / self.length

    def rotated(self, angle: Real) -> Vector2:
        angle_value = _finite_number(angle, name="angle")
        cosine = math.cos(angle_value)
        sine = math.sin(angle_value)
        return Vector2(
            x=(self.x * cosine) - (self.y * sine),
            y=(self.x * sine) + (self.y * cosine),
        )

    def lerp(self, other: Vector2, fraction: Real) -> Vector2:
        """Linearly interpolate between this vector and ``other``.

        Values outside ``[0, 1]`` deliberately remain valid and extrapolate.
        """

        amount = _finite_number(fraction, name="fraction")
        return self + ((other - self) * amount)

    def __add__(self, other: Vector2) -> Vector2:
        if not isinstance(other, Vector2):
            return NotImplemented
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2) -> Vector2:
        if not isinstance(other, Vector2):
            return NotImplemented
        return Vector2(self.x - other.x, self.y - other.y)

    def __neg__(self) -> Vector2:
        return Vector2(-self.x, -self.y)

    def __mul__(self, scalar: Real) -> Vector2:
        amount = _finite_number(scalar, name="scalar")
        return Vector2(self.x * amount, self.y * amount)

    def __rmul__(self, scalar: Real) -> Vector2:
        return self * scalar

    def __truediv__(self, scalar: Real) -> Vector2:
        divisor = _finite_number(scalar, name="scalar")
        if divisor == 0.0:
            raise ZeroDivisionError("cannot divide a vector by zero")
        return Vector2(self.x / divisor, self.y / divisor)


@dataclass(frozen=True, slots=True)
class Pose2D:
    """An immutable position and heading in a two-dimensional world."""

    position: Vector2
    yaw: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.position, Vector2):
            raise TypeError("position must be a Vector2")
        object.__setattr__(self, "yaw", _finite_number(self.yaw, name="yaw"))

    @classmethod
    def from_xy(cls, x: Real, y: Real, yaw: Real = 0.0) -> Pose2D:
        return cls(position=Vector2(x, y), yaw=yaw)

    @property
    def x(self) -> float:
        return self.position.x

    @property
    def y(self) -> float:
        return self.position.y

    @property
    def normalized_yaw(self) -> float:
        return normalize_angle(self.yaw)

    def distance_to(self, other: Pose2D) -> float:
        return self.position.distance_to(other.position)

    def translated(self, offset: Vector2) -> Pose2D:
        """Translate in world coordinates without changing the heading."""

        return Pose2D(position=self.position + offset, yaw=self.yaw)

    def transform_local_vector(self, vector: Vector2) -> Vector2:
        """Transform a robot-local vector into a world position."""

        return self.position + vector.rotated(self.yaw)

    def relative_vector_to(self, world_point: Vector2) -> Vector2:
        """Transform a world position into this pose's local frame."""

        return (world_point - self.position).rotated(-self.yaw)

    def with_yaw(self, yaw: Real) -> Pose2D:
        return Pose2D(position=self.position, yaw=_finite_number(yaw, name="yaw"))
