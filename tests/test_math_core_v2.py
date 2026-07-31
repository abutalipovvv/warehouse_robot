from __future__ import annotations

from dataclasses import FrozenInstanceError
import math

import pytest

from fleet_manager.math import (
    Pose2D,
    Polygon2D,
    TimeInterval,
    Vector2,
    distance_to_segment,
    merge_intervals,
    normalize_angle,
    shortest_angular_distance,
    subtract_intervals,
)


def test_vector_arithmetic_and_measurements() -> None:
    first = Vector2(3, 4)
    second = Vector2(-1, 2)

    assert first.length == pytest.approx(5.0)
    assert first.length_squared == pytest.approx(25.0)
    assert first + second == Vector2(2, 6)
    assert first - second == Vector2(4, 2)
    assert -second == Vector2(1, -2)
    assert first * 2 == Vector2(6, 8)
    assert 0.5 * first == Vector2(1.5, 2)
    assert first / 2 == Vector2(1.5, 2)
    assert first.dot(second) == pytest.approx(5.0)
    assert first.cross(second) == pytest.approx(10.0)
    assert first.distance_to(second) == pytest.approx(math.sqrt(20))


def test_vector_normalization_rotation_and_interpolation() -> None:
    vector = Vector2(0, 2)

    assert vector.normalized() == Vector2(0, 1)
    assert vector.rotated(-math.pi / 2).x == pytest.approx(2.0)
    assert vector.rotated(-math.pi / 2).y == pytest.approx(0.0)
    assert Vector2(0, 0).angle == pytest.approx(0.0)
    assert Vector2(2, 4).lerp(Vector2(6, 8), 0.25) == Vector2(3, 5)
    assert Vector2(2, 4).lerp(Vector2(6, 8), 1.5) == Vector2(8, 10)


def test_zero_vector_cannot_be_normalized_or_divided_by_zero() -> None:
    with pytest.raises(ValueError, match="zero-length"):
        Vector2(0, 0).normalized()
    with pytest.raises(ZeroDivisionError, match="zero"):
        Vector2(1, 2) / 0


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_vector_rejects_non_finite_coordinates(value: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        Vector2(value, 0)


def test_vector_rejects_non_numeric_and_boolean_values() -> None:
    with pytest.raises(TypeError, match="real number"):
        Vector2("1", 2)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="real number"):
        Vector2(True, 2)


def test_vector_and_pose_are_immutable() -> None:
    vector = Vector2(1, 2)
    pose = Pose2D(vector, yaw=0.5)

    with pytest.raises(FrozenInstanceError):
        vector.x = 10  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        pose.yaw = 1.0  # type: ignore[misc]


def test_pose_composes_position_and_heading_math() -> None:
    pose = Pose2D.from_xy(10, 5, math.pi / 2)
    target = Pose2D.from_xy(13, 9)

    assert pose.position == Vector2(10, 5)
    assert pose.x == pytest.approx(10)
    assert pose.y == pytest.approx(5)
    assert pose.distance_to(target) == pytest.approx(5)
    assert pose.translated(Vector2(2, -1)) == Pose2D.from_xy(
        12,
        4,
        math.pi / 2,
    )

    world_point = pose.transform_local_vector(Vector2(2, 0))
    assert world_point.x == pytest.approx(10)
    assert world_point.y == pytest.approx(7)
    local_point = pose.relative_vector_to(world_point)
    assert local_point.x == pytest.approx(2)
    assert local_point.y == pytest.approx(0)
    assert pose.with_yaw(3 * math.pi).normalized_yaw == pytest.approx(-math.pi)


def test_pose_validates_position_and_yaw() -> None:
    with pytest.raises(TypeError, match="Vector2"):
        Pose2D((1, 2), 0)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="finite"):
        Pose2D(Vector2(1, 2), math.nan)


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (3 * math.pi, -math.pi),
        (-3 * math.pi, -math.pi),
    ],
)
def test_angle_normalization(angle: float, expected: float) -> None:
    assert normalize_angle(angle) == pytest.approx(expected)


def test_shortest_angular_distance_is_signed() -> None:
    assert shortest_angular_distance(
        math.radians(170),
        math.radians(-170),
    ) == pytest.approx(math.radians(20))
    assert shortest_angular_distance(
        math.radians(-170),
        math.radians(170),
    ) == pytest.approx(math.radians(-20))


def test_interval_is_half_open() -> None:
    interval = TimeInterval(2, 5)

    assert interval.duration == pytest.approx(3)
    assert interval.contains(2)
    assert interval.contains(4.999)
    assert not interval.contains(5)
    assert 2 in interval
    assert 5 not in interval
    assert "2" not in interval


def test_interval_overlap_touch_and_intersection_boundaries() -> None:
    first = TimeInterval(0, 5)
    overlapping = TimeInterval(3, 7)
    adjacent = TimeInterval(5, 8)

    assert first.overlaps(overlapping)
    assert first.intersection(overlapping) == TimeInterval(3, 5)
    assert not first.overlaps(adjacent)
    assert first.touches(adjacent)
    assert first.intersection(adjacent) is None
    assert first.merged_with(adjacent) == TimeInterval(0, 8)


def test_interval_rejects_a_gap_when_merging() -> None:
    with pytest.raises(ValueError, match="gap"):
        TimeInterval(0, 2).merged_with(TimeInterval(3, 4))


def test_interval_subtraction_returns_ordered_pieces() -> None:
    interval = TimeInterval(0, 10)

    assert interval.subtract(TimeInterval(3, 7)) == (
        TimeInterval(0, 3),
        TimeInterval(7, 10),
    )
    assert interval.subtract(TimeInterval(-5, 4)) == (TimeInterval(4, 10),)
    assert interval.subtract(TimeInterval(6, 20)) == (TimeInterval(0, 6),)
    assert interval.subtract(TimeInterval(-1, 20)) == ()
    assert interval.subtract(TimeInterval(10, 12)) == (interval,)


def test_empty_interval_has_no_occupied_points() -> None:
    empty = TimeInterval(4, 4)

    assert empty.is_empty
    assert empty.duration == pytest.approx(0)
    assert 4 not in empty
    assert not empty.overlaps(TimeInterval(3, 5))
    assert empty.intersection(TimeInterval(3, 5)) is None
    assert empty.subtract(TimeInterval(0, 10)) == ()


@pytest.mark.parametrize(
    ("start", "end", "error"),
    [
        (2, 1, ValueError),
        (math.nan, 1, ValueError),
        (0, math.inf, ValueError),
        ("0", 1, TypeError),
        (False, 1, TypeError),
    ],
)
def test_interval_validates_bounds(
    start: object,
    end: object,
    error: type[Exception],
) -> None:
    with pytest.raises(error):
        TimeInterval(start, end)  # type: ignore[arg-type]


def test_merge_intervals_sorts_and_combines_overlap_and_adjacency() -> None:
    assert merge_intervals(
        [
            TimeInterval(8, 10),
            TimeInterval(1, 3),
            TimeInterval(3, 5),
            TimeInterval(4, 7),
            TimeInterval(20, 20),
        ]
    ) == (
        TimeInterval(1, 7),
        TimeInterval(8, 10),
    )


def test_subtract_intervals_builds_safe_windows() -> None:
    safe = subtract_intervals(
        TimeInterval(0, 20),
        [
            TimeInterval(-5, 3),
            TimeInterval(6, 9),
            TimeInterval(8, 12),
            TimeInterval(18, 30),
        ],
    )

    assert safe == (
        TimeInterval(3, 6),
        TimeInterval(12, 18),
    )
    assert TimeInterval(1, 1).shifted(5) == TimeInterval(6, 6)


def test_polygon_overlap_uses_both_shapes_axes_and_margin() -> None:
    square = Polygon2D(
        (
            Vector2(-1, -1),
            Vector2(1, -1),
            Vector2(1, 1),
            Vector2(-1, 1),
        )
    )
    touching = square.transformed(Pose2D.from_xy(2, 0))
    separated = square.transformed(Pose2D.from_xy(2.2, 0))
    rotated = square.transformed(Pose2D.from_xy(1.5, 0, math.pi / 4))

    assert square.overlaps(touching)
    assert not square.overlaps(separated)
    assert square.overlaps(separated, margin=0.201)
    assert square.overlaps(rotated)


def test_positioned_polygon_overlap_matches_pose_based_transform() -> None:
    footprint = Polygon2D(
        (
            Vector2(-0.3, -0.2),
            Vector2(0.3, -0.2),
            Vector2(0.3, 0.2),
            Vector2(-0.3, 0.2),
        )
    )
    first_pose = Pose2D.from_xy(1.2, -0.4, 0.7)
    second_pose = Pose2D.from_xy(1.6, -0.2, -0.3)

    expected = footprint.overlaps_transformed(
        first_pose,
        footprint,
        second_pose,
        margin=0.04,
    )

    assert footprint.overlaps_positioned(
        first_pose.x,
        first_pose.y,
        first_pose.yaw,
        footprint,
        second_pose.x,
        second_pose.y,
        second_pose.yaw,
        margin=0.04,
    ) is expected


def test_polygon_contains_and_distance_follow_boundary_geometry() -> None:
    triangle = Polygon2D(
        (
            Vector2(0, 0),
            Vector2(4, 0),
            Vector2(0, 3),
        )
    )

    assert triangle.contains(Vector2(0.5, 0.5))
    assert not triangle.contains(Vector2(4, 3))
    assert triangle.distance_to(Vector2(2, 2)) == pytest.approx(0.4)
    assert distance_to_segment(
        Vector2(3, 4),
        Vector2(0, 0),
        Vector2(0, 0),
    ) == pytest.approx(5)


def test_polygon_requires_three_typed_points() -> None:
    with pytest.raises(ValueError, match="three"):
        Polygon2D((Vector2(0, 0), Vector2(1, 0)))
    with pytest.raises(TypeError, match="Vector2"):
        Polygon2D(
            (
                Vector2(0, 0),
                Vector2(1, 0),
                (0, 1),  # type: ignore[arg-type]
            )
        )
