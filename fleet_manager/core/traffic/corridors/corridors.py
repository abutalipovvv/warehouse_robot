"""Compile geometric controlled-corridor zones into MAPF graph resources."""

from __future__ import annotations

from dataclasses import replace
from typing import Iterable

from fleet_manager.core.math.curves import bezier_point
from fleet_manager.core.math.geometry import Vector2
from fleet_manager.core.mapping.maps.models import (
    GraphEdge,
    Landmark,
    TrafficZone,
    WorldPoint,
)


DERIVED_PROPERTY = "_traffic_zone_derived"
DERIVED_TRAFFIC_KEYS = (
    "controlled_region",
    "controlled_region_capacity",
    "can_wait",
    "waitAllowed",
    "holding_point",
)
CONTROLLED_CORRIDOR_KINDS = frozenset(
    {"controlled_corridor", "controlled-corridor", "corridor"}
)


def compile_controlled_corridor_zones(
    landmarks: dict[str, Landmark],
    edges: list[GraphEdge],
    zones: Iterable[TrafficZone],
) -> tuple[dict[str, Landmark], list[GraphEdge]]:
    """Return graph objects annotated from map-authored rectangle zones.

    A landmark inside a corridor is a mutex resource and cannot be used as a
    waiting point.  An outside endpoint of an edge crossing the rectangle is
    a legal stop line.  The crossing edge itself belongs to the corridor, so
    admission is acquired before any part of the robot enters the rectangle.
    """

    compiled_landmarks = dict(landmarks)
    compiled_edges = list(edges)
    corridor_zones = [
        zone
        for zone in zones
        if zone.kind.strip().lower() in CONTROLLED_CORRIDOR_KINDS
    ]
    inside_by_zone = {
        zone.zone_id: {
            name
            for name, landmark in compiled_landmarks.items()
            if _point_in_zone(landmark.x, landmark.y, zone)
        }
        for zone in corridor_zones
    }
    inside_any_zone = set().union(*inside_by_zone.values()) if inside_by_zone else set()
    for zone in corridor_zones:
        inside = inside_by_zone[zone.zone_id]
        controlled_edge_indexes = {
            index
            for index, edge in enumerate(compiled_edges)
            if (
                edge.from_name in inside
                or edge.to_name in inside
                or _edge_intersects_zone(edge, zone)
            )
        }
        if not inside and not controlled_edge_indexes:
            continue

        outside_holding_lms: set[str] = set()
        for index in controlled_edge_indexes:
            edge = compiled_edges[index]
            start_inside = edge.from_name in inside
            goal_inside = edge.to_name in inside
            if start_inside != goal_inside:
                outside_holding_lms.add(
                    edge.to_name if start_inside else edge.from_name
                )
            properties = _with_region(
                _mark_derived(edge.properties),
                zone.zone_id,
            )
            properties["controlled_region_capacity"] = max(1, zone.capacity)
            compiled_edges[index] = replace(edge, properties=properties)

        for name in inside:
            landmark = compiled_landmarks[name]
            properties = _with_region(
                _mark_derived(landmark.properties),
                zone.zone_id,
            )
            properties.update(
                {
                    "can_wait": False,
                    "waitAllowed": False,
                }
            )
            properties.pop("holding_point", None)
            compiled_landmarks[name] = replace(
                landmark,
                properties=properties,
            )

        for name in outside_holding_lms - inside_any_zone:
            landmark = compiled_landmarks.get(name)
            if landmark is None:
                continue
            properties = _mark_derived(landmark.properties)
            # A stop line may border two corridor rectangles.  It remains
            # outside both and therefore must not inherit either mutex.
            properties.update(
                {
                    "can_wait": True,
                    "waitAllowed": True,
                    "holding_point": True,
                }
            )
            compiled_landmarks[name] = replace(
                landmark,
                properties=properties,
            )

    return compiled_landmarks, compiled_edges


def strip_derived_traffic_properties(properties: object) -> dict[str, object]:
    """Remove runtime annotations before persisting editor source data."""

    result = dict(properties) if isinstance(properties, dict) else {}
    derived = result.pop(DERIVED_PROPERTY, None)
    if not isinstance(derived, dict):
        return result
    for key in DERIVED_TRAFFIC_KEYS:
        result.pop(key, None)
    original = derived.get("original")
    if isinstance(original, dict):
        result.update(original)
    return result


def _mark_derived(properties: object) -> dict[str, object]:
    result = dict(properties) if isinstance(properties, dict) else {}
    if isinstance(result.get(DERIVED_PROPERTY), dict):
        return result
    result[DERIVED_PROPERTY] = {
        "original": {
            key: result[key]
            for key in DERIVED_TRAFFIC_KEYS
            if key in result
        },
    }
    return result


def _with_region(properties: object, region_id: str) -> dict[str, object]:
    result = dict(properties) if isinstance(properties, dict) else {}
    existing = [
        item.strip()
        for item in str(result.get("controlled_region") or "").split(",")
        if item.strip()
    ]
    if region_id not in existing:
        existing.append(region_id)
    result["controlled_region"] = ",".join(existing)
    return result


def _point_in_zone(x: float, y: float, zone: TrafficZone) -> bool:
    epsilon = 1e-9
    return (
        zone.min_x - epsilon <= float(x) <= zone.max_x + epsilon
        and zone.min_y - epsilon <= float(y) <= zone.max_y + epsilon
    )


def _edge_intersects_zone(edge: GraphEdge, zone: TrafficZone) -> bool:
    points = _edge_points(edge)
    return any(
        _segment_intersects_rectangle(first, second, zone)
        for first, second in zip(points, points[1:])
    )


def _edge_points(edge: GraphEdge) -> tuple[WorldPoint, ...]:
    if edge.geometry is not None and len(edge.geometry.control_points) == 4:
        controls = tuple(edge.geometry.control_points)
        return tuple(
            _bezier_point(controls, index / 16.0)
            for index in range(17)
        )
    points = tuple(edge.world_points)
    return points if len(points) >= 2 else ()


def _bezier_point(
    controls: tuple[WorldPoint, ...],
    t: float,
) -> WorldPoint:
    point = bezier_point(
        tuple(Vector2(item.x, item.y) for item in controls),
        t,
    )
    return WorldPoint(point.x, point.y)


def _segment_intersects_rectangle(
    first: WorldPoint,
    second: WorldPoint,
    zone: TrafficZone,
) -> bool:
    if _point_in_zone(first.x, first.y, zone) or _point_in_zone(
        second.x,
        second.y,
        zone,
    ):
        return True
    corners = (
        WorldPoint(zone.min_x, zone.min_y),
        WorldPoint(zone.max_x, zone.min_y),
        WorldPoint(zone.max_x, zone.max_y),
        WorldPoint(zone.min_x, zone.max_y),
    )
    return any(
        _segments_intersect(first, second, start, goal)
        for start, goal in zip(corners, (*corners[1:], corners[0]))
    )


def _segments_intersect(
    a: WorldPoint,
    b: WorldPoint,
    c: WorldPoint,
    d: WorldPoint,
) -> bool:
    epsilon = 1e-9
    if (
        max(a.x, b.x) + epsilon < min(c.x, d.x)
        or max(c.x, d.x) + epsilon < min(a.x, b.x)
        or max(a.y, b.y) + epsilon < min(c.y, d.y)
        or max(c.y, d.y) + epsilon < min(a.y, b.y)
    ):
        return False

    def orientation(
        first: WorldPoint,
        second: WorldPoint,
        third: WorldPoint,
    ) -> float:
        return (
            (second.x - first.x) * (third.y - first.y)
            - (second.y - first.y) * (third.x - first.x)
        )

    ab_c = orientation(a, b, c)
    ab_d = orientation(a, b, d)
    cd_a = orientation(c, d, a)
    cd_b = orientation(c, d, b)
    return (
        ((ab_c <= epsilon and ab_d >= -epsilon) or (ab_d <= epsilon and ab_c >= -epsilon))
        and ((cd_a <= epsilon and cd_b >= -epsilon) or (cd_b <= epsilon and cd_a >= -epsilon))
    )
