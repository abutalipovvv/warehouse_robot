from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from fleet_manager.route_core import GraphEdge, Landmark

from .reservations import ResourceId


@dataclass(frozen=True, slots=True)
class TrafficVertex:
    id: str
    x: float
    y: float
    can_wait: bool = True
    is_parking: bool = False
    is_charger: bool = False
    mutex_zone_ids: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class TrafficLane:
    id: str
    from_lm: str
    to_lm: str
    length_m: float
    max_speed_mps: float
    lane_group_id: str
    capacity: int = 1
    mutex_zone_ids: tuple[str, ...] = ()


@dataclass(slots=True)
class TrafficGraph:
    vertices: dict[str, TrafficVertex]
    lanes: dict[str, TrafficLane]
    outgoing: dict[str, list[str]]

    @classmethod
    def from_route_core(
        cls,
        landmarks: Mapping[str, Landmark],
        edges: list[GraphEdge],
        *,
        default_speed_mps: float,
    ) -> "TrafficGraph":
        edge_keys = {(edge.from_name, edge.to_name) for edge in edges}
        vertices = {
            name: _traffic_vertex_from_landmark(landmark)
            for name, landmark in landmarks.items()
        }
        lanes: dict[str, TrafficLane] = {}
        outgoing: dict[str, list[str]] = {name: [] for name in landmarks}
        for edge in edges:
            if edge.from_name not in landmarks or edge.to_name not in landmarks:
                continue
            lane = _traffic_lane_from_edge(
                edge,
                default_speed_mps=default_speed_mps,
                has_reverse=(edge.to_name, edge.from_name) in edge_keys,
            )
            lanes[lane.id] = lane
            outgoing.setdefault(lane.from_lm, []).append(lane.id)
        for lane_ids in outgoing.values():
            lane_ids.sort()
        return cls(vertices=vertices, lanes=lanes, outgoing=outgoing)

    def neighbors(self, lm_id: str) -> list[TrafficLane]:
        return [
            self.lanes[lane_id]
            for lane_id in self.outgoing.get(lm_id, [])
            if lane_id in self.lanes
        ]

    def lane_for(self, from_lm: str, to_lm: str) -> TrafficLane | None:
        return self.lanes.get(lane_id(from_lm, to_lm))

    def lane_resources(self, lane: TrafficLane) -> tuple[ResourceId, ...]:
        resources = [
            ResourceId("lane", lane.id),
            ResourceId("lane_group", lane.lane_group_id),
        ]
        resources.extend(ResourceId("mutex_zone", zone_id) for zone_id in lane.mutex_zone_ids)
        return tuple(resources)

    def vertex_resources(self, lm_id: str) -> tuple[ResourceId, ...]:
        vertex = self.vertices.get(lm_id)
        if vertex is None:
            return (ResourceId("vertex", lm_id),)
        resources = [ResourceId("vertex", lm_id)]
        resources.extend(ResourceId("mutex_zone", zone_id) for zone_id in vertex.mutex_zone_ids)
        return tuple(resources)

    def reservation_capacities(self) -> dict[ResourceId, int]:
        capacities: dict[ResourceId, int] = {}
        for lane in self.lanes.values():
            capacities[ResourceId("lane", lane.id)] = max(1, int(lane.capacity))
            group_resource = ResourceId("lane_group", lane.lane_group_id)
            capacities[group_resource] = max(
                capacities.get(group_resource, 1),
                max(1, int(lane.capacity)),
            )
        return capacities


def lane_id(from_lm: str, to_lm: str) -> str:
    return f"{from_lm}->{to_lm}"


def _traffic_vertex_from_landmark(landmark: Landmark) -> TrafficVertex:
    properties = landmark.properties if isinstance(landmark.properties, Mapping) else {}
    name = str(landmark.name)
    return TrafficVertex(
        id=name,
        x=float(landmark.x),
        y=float(landmark.y),
        can_wait=_bool_property(properties, ("can_wait", "canWait", "allow_wait", "allowWait"), True),
        is_parking=_bool_property(properties, ("is_parking", "isParking", "parking", "parkPoint"), name.upper().startswith("PP")),
        is_charger=_bool_property(properties, ("is_charger", "isCharger", "charger", "chargePoint"), name.upper().startswith("CP")),
        mutex_zone_ids=_string_tuple_property(properties, ("mutex_zone", "mutexZone", "mutex_group", "mutexGroup")),
    )


def _traffic_lane_from_edge(
    edge: GraphEdge,
    *,
    default_speed_mps: float,
    has_reverse: bool,
) -> TrafficLane:
    properties = edge.properties if isinstance(edge.properties, Mapping) else {}
    group_id = _string_property(properties, ("lane_group", "laneGroup", "lane_group_id", "laneGroupId", "resource_id", "resourceId"))
    if not group_id:
        group_id = _canonical_lane_group(edge.from_name, edge.to_name) if has_reverse else lane_id(edge.from_name, edge.to_name)
    return TrafficLane(
        id=lane_id(edge.from_name, edge.to_name),
        from_lm=edge.from_name,
        to_lm=edge.to_name,
        length_m=max(0.0, float(edge.length)),
        max_speed_mps=_float_property(
            properties,
            ("max_speed", "maxSpeed", "maxspeed", "speed", "speedLimit"),
            max(0.02, float(default_speed_mps)),
        ),
        lane_group_id=group_id,
        capacity=_int_property(properties, ("capacity", "trafficCapacity", "laneCapacity"), 1),
        mutex_zone_ids=_string_tuple_property(properties, ("mutex_zone", "mutexZone", "mutex_group", "mutexGroup")),
    )


def _canonical_lane_group(first: str, second: str) -> str:
    src, dst = sorted((str(first), str(second)))
    return f"{src}<->{dst}"


def _bool_property(properties: Mapping[str, object], keys: tuple[str, ...], default: bool) -> bool:
    for key in keys:
        if key not in properties:
            continue
        value = properties.get(key)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
    return default


def _float_property(properties: Mapping[str, object], keys: tuple[str, ...], default: float) -> float:
    for key in keys:
        if key not in properties:
            continue
        try:
            return max(0.02, float(properties[key]))
        except (TypeError, ValueError):
            continue
    return default


def _int_property(properties: Mapping[str, object], keys: tuple[str, ...], default: int) -> int:
    for key in keys:
        if key not in properties:
            continue
        try:
            return max(1, int(properties[key]))
        except (TypeError, ValueError):
            continue
    return default


def _string_property(properties: Mapping[str, object], keys: tuple[str, ...]) -> str:
    for key in keys:
        value = str(properties.get(key) or "").strip()
        if value:
            return value
    return ""


def _string_tuple_property(properties: Mapping[str, object], keys: tuple[str, ...]) -> tuple[str, ...]:
    value = _string_property(properties, keys)
    if not value:
        return ()
    return tuple(item.strip() for item in value.split(",") if item.strip())
