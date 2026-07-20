from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from fleet_manager.core.route_core.models import GraphEdge, Landmark

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
    controlled_region_ids: tuple[str, ...] = ()
    clearance_zone_ids: tuple[str, ...] = ()
    rotation_conflict_lms: tuple[str, ...] = ()


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
    controlled_region_ids: tuple[str, ...] = ()
    clearance_zone_ids: tuple[str, ...] = ()
    centerline: tuple[tuple[float, float], ...] = ()


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
        min_robot_center_distance_m: float = 0.0,
        rotation_min_robot_center_distance_m: float = 0.0,
        explicit_controlled_regions_enabled: bool = True,
        controlled_corridors_enabled: bool = False,
        controlled_corridor_min_edges: int = 2,
    ) -> "TrafficGraph":
        edge_keys = {(edge.from_name, edge.to_name) for edge in edges}
        vertices = {
            name: _traffic_vertex_from_landmark(
                landmark,
                controlled_regions_enabled=explicit_controlled_regions_enabled,
            )
            for name, landmark in landmarks.items()
        }
        vertices = _with_clearance_zones(
            vertices,
            max(0.0, float(min_robot_center_distance_m)),
        )
        vertices = _with_rotation_conflict_lms(
            vertices,
            max(0.0, float(rotation_min_robot_center_distance_m)),
        )
        lanes: dict[str, TrafficLane] = {}
        outgoing: dict[str, list[str]] = {name: [] for name in landmarks}
        for edge in edges:
            if edge.from_name not in landmarks or edge.to_name not in landmarks:
                continue
            lane = _traffic_lane_from_edge(
                edge,
                default_speed_mps=default_speed_mps,
                has_reverse=(edge.to_name, edge.from_name) in edge_keys,
                controlled_regions_enabled=explicit_controlled_regions_enabled,
            )
            lanes[lane.id] = lane
            outgoing.setdefault(lane.from_lm, []).append(lane.id)
        if explicit_controlled_regions_enabled:
            vertices = _with_explicit_controlled_region_vertices(
                vertices,
                lanes,
                landmarks,
            )
        if controlled_corridors_enabled:
            vertices, lanes = _with_controlled_corridors(
                vertices,
                lanes,
                landmarks,
                minimum_edges=max(1, int(controlled_corridor_min_edges)),
            )
        vertices, lanes = _with_lane_vertex_clearance_zones(
            vertices,
            lanes,
            max(0.0, float(min_robot_center_distance_m)),
        )
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
        resources.extend(
            ResourceId("controlled_region", region_id)
            for region_id in lane.controlled_region_ids
        )
        resources.extend(ResourceId("clearance", zone_id) for zone_id in lane.clearance_zone_ids)
        # A robot occupies the swept corridor, including both endpoint
        # clearances, for the complete traversal.  Reserving only the abstract
        # edge lets another robot sit at (or too close to) an endpoint while a
        # continuous footprint is still passing through it.
        resources.extend(self.vertex_resources(lane.from_lm))
        resources.extend(self.vertex_resources(lane.to_lm))
        return tuple(dict.fromkeys(resources))

    def vertex_resources(self, lm_id: str) -> tuple[ResourceId, ...]:
        vertex = self.vertices.get(lm_id)
        if vertex is None:
            return (ResourceId("vertex", lm_id),)
        resources = [ResourceId("vertex", lm_id)]
        resources.extend(ResourceId("mutex_zone", zone_id) for zone_id in vertex.mutex_zone_ids)
        resources.extend(
            ResourceId("controlled_region", region_id)
            for region_id in vertex.controlled_region_ids
        )
        resources.extend(ResourceId("clearance", zone_id) for zone_id in vertex.clearance_zone_ids)
        return tuple(resources)

    def rotation_resources(self, lm_id: str) -> tuple[ResourceId, ...]:
        """Resources swept by an in-place rotation at ``lm_id``.

        Normal vertex occupancy remains compact. Nearby turns share a
        turn-only pair resource, so their swept bodies are serialized without
        making an ordinary parked or passing robot occupy the entire
        circumscribed rotation circle. Exact footprint geometry remains the
        authority for a turn against non-rotating traffic.
        """
        vertex = self.vertices.get(lm_id)
        if vertex is None:
            return self.vertex_resources(lm_id)
        resources = list(self.vertex_resources(lm_id))
        resources.extend(
            ResourceId(
                "rotation_clearance",
                "<->".join(sorted((lm_id, other_lm))),
            )
            for other_lm in vertex.rotation_conflict_lms
        )
        return tuple(dict.fromkeys(resources))

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

    def controlled_region_ids(self) -> tuple[str, ...]:
        return tuple(sorted({
            region_id
            for lane in self.lanes.values()
            for region_id in lane.controlled_region_ids
        }))

    def extend_route_index_to_controlled_exit(
        self,
        route_nodes: list[str] | tuple[str, ...],
        index: int,
    ) -> int:
        """Do not end a rolling chunk at an internal controlled-corridor LM."""
        if not route_nodes:
            return 0
        cursor = max(0, min(int(index), len(route_nodes) - 1))
        vertex = self.vertices.get(str(route_nodes[cursor]))
        regions = set(vertex.controlled_region_ids if vertex is not None else ())
        if not regions:
            return cursor
        region_id = sorted(regions)[0]
        previous = str(route_nodes[cursor])
        for next_index in range(cursor + 1, len(route_nodes)):
            node = str(route_nodes[next_index])
            if node == previous:
                cursor = next_index
                continue
            lane = self.lane_for(previous, node)
            if lane is None or region_id not in lane.controlled_region_ids:
                break
            cursor = next_index
            previous = node
            next_vertex = self.vertices.get(node)
            if next_vertex is None or region_id not in next_vertex.controlled_region_ids:
                break
        return cursor


def lane_id(from_lm: str, to_lm: str) -> str:
    return f"{from_lm}->{to_lm}"


def _traffic_vertex_from_landmark(
    landmark: Landmark,
    *,
    controlled_regions_enabled: bool = True,
) -> TrafficVertex:
    properties = landmark.properties if isinstance(landmark.properties, Mapping) else {}
    name = str(landmark.name)
    return TrafficVertex(
        id=name,
        x=float(landmark.x),
        y=float(landmark.y),
        can_wait=_bool_property(
            properties,
            (
                "can_wait",
                "canWait",
                "allow_wait",
                "allowWait",
                "wait_allowed",
                "waitAllowed",
            ),
            True,
        ),
        is_parking=_bool_property(properties, ("is_parking", "isParking", "parking", "parkPoint"), name.upper().startswith("PP")),
        is_charger=_bool_property(properties, ("is_charger", "isCharger", "charger", "chargePoint"), name.upper().startswith("CP")),
        mutex_zone_ids=_string_tuple_property(properties, ("mutex_zone", "mutexZone", "mutex_group", "mutexGroup")),
        controlled_region_ids=(
            _string_tuple_property(
                properties,
                (
                    "controlled_region",
                    "controlledRegion",
                    "corridor_region",
                    "corridorRegion",
                ),
            )
            if controlled_regions_enabled
            else ()
        ),
    )


def _with_clearance_zones(
    vertices: dict[str, TrafficVertex],
    min_center_distance: float,
) -> dict[str, TrafficVertex]:
    if min_center_distance <= 0.0:
        return vertices

    zones: dict[str, list[str]] = {name: [] for name in vertices}
    ordered = sorted(vertices.values(), key=lambda item: item.id)
    threshold_sq = min_center_distance * min_center_distance
    for index, first in enumerate(ordered):
        for second in ordered[index + 1:]:
            distance_sq = ((first.x - second.x) ** 2) + ((first.y - second.y) ** 2)
            if distance_sq >= threshold_sq:
                continue
            zone_id = f"{first.id}<->{second.id}"
            zones[first.id].append(zone_id)
            zones[second.id].append(zone_id)

    return {
        name: TrafficVertex(
            id=vertex.id,
            x=vertex.x,
            y=vertex.y,
            can_wait=vertex.can_wait,
            is_parking=vertex.is_parking,
            is_charger=vertex.is_charger,
            mutex_zone_ids=vertex.mutex_zone_ids,
            controlled_region_ids=vertex.controlled_region_ids,
            clearance_zone_ids=tuple(zones.get(name, ())),
            rotation_conflict_lms=vertex.rotation_conflict_lms,
        )
        for name, vertex in vertices.items()
    }


def _with_rotation_conflict_lms(
    vertices: dict[str, TrafficVertex],
    min_center_distance: float,
) -> dict[str, TrafficVertex]:
    if min_center_distance <= 0.0:
        return vertices

    conflicts: dict[str, list[str]] = {name: [] for name in vertices}
    ordered = sorted(vertices.values(), key=lambda item: item.id)
    threshold_sq = min_center_distance * min_center_distance
    for index, first in enumerate(ordered):
        for second in ordered[index + 1:]:
            distance_sq = ((first.x - second.x) ** 2) + ((first.y - second.y) ** 2)
            if distance_sq >= threshold_sq:
                continue
            conflicts[first.id].append(second.id)
            conflicts[second.id].append(first.id)

    return {
        name: TrafficVertex(
            id=vertex.id,
            x=vertex.x,
            y=vertex.y,
            can_wait=vertex.can_wait,
            is_parking=vertex.is_parking,
            is_charger=vertex.is_charger,
            mutex_zone_ids=vertex.mutex_zone_ids,
            controlled_region_ids=vertex.controlled_region_ids,
            clearance_zone_ids=vertex.clearance_zone_ids,
            rotation_conflict_lms=tuple(conflicts.get(name, ())),
        )
        for name, vertex in vertices.items()
    }


def _with_lane_vertex_clearance_zones(
    vertices: dict[str, TrafficVertex],
    lanes: dict[str, TrafficLane],
    min_center_distance: float,
) -> tuple[dict[str, TrafficVertex], dict[str, TrafficLane]]:
    if min_center_distance <= 0.0:
        return vertices, lanes

    vertex_zones = {
        name: list(vertex.clearance_zone_ids)
        for name, vertex in vertices.items()
    }
    lane_zones: dict[str, list[str]] = {lane_id: [] for lane_id in lanes}
    threshold_sq = min_center_distance * min_center_distance
    cell_size = min_center_distance
    vertex_order = {name: index for index, name in enumerate(vertices)}
    vertex_grid: dict[tuple[int, int], list[str]] = {}
    for vertex in vertices.values():
        cell = (
            math.floor(vertex.x / cell_size),
            math.floor(vertex.y / cell_size),
        )
        vertex_grid.setdefault(cell, []).append(vertex.id)

    for lane in lanes.values():
        start = vertices.get(lane.from_lm)
        end = vertices.get(lane.to_lm)
        if start is None or end is None:
            continue
        centerline = lane.centerline or (
            (start.x, start.y),
            (end.x, end.y),
        )
        min_x = min(point[0] for point in centerline) - min_center_distance
        max_x = max(point[0] for point in centerline) + min_center_distance
        min_y = min(point[1] for point in centerline) - min_center_distance
        max_y = max(point[1] for point in centerline) + min_center_distance
        candidate_names: set[str] = set()
        for cell_x in range(
            math.floor(min_x / cell_size),
            math.floor(max_x / cell_size) + 1,
        ):
            for cell_y in range(
                math.floor(min_y / cell_size),
                math.floor(max_y / cell_size) + 1,
            ):
                candidate_names.update(vertex_grid.get((cell_x, cell_y), ()))

        for vertex_name in sorted(
            candidate_names,
            key=lambda name: vertex_order[name],
        ):
            vertex = vertices[vertex_name]
            if vertex.id in {lane.from_lm, lane.to_lm}:
                continue
            if _point_polyline_distance_sq(vertex, centerline) >= threshold_sq:
                continue
            zone_id = f"{lane.id}<->{vertex.id}"
            lane_zones[lane.id].append(zone_id)
            vertex_zones[vertex.id].append(zone_id)

    updated_vertices = {
        name: TrafficVertex(
            id=vertex.id,
            x=vertex.x,
            y=vertex.y,
            can_wait=vertex.can_wait,
            is_parking=vertex.is_parking,
            is_charger=vertex.is_charger,
            mutex_zone_ids=vertex.mutex_zone_ids,
            controlled_region_ids=vertex.controlled_region_ids,
            clearance_zone_ids=tuple(dict.fromkeys(vertex_zones.get(name, ()))),
            rotation_conflict_lms=vertex.rotation_conflict_lms,
        )
        for name, vertex in vertices.items()
    }
    updated_lanes = {
        lane_id: TrafficLane(
            id=lane.id,
            from_lm=lane.from_lm,
            to_lm=lane.to_lm,
            length_m=lane.length_m,
            max_speed_mps=lane.max_speed_mps,
            lane_group_id=lane.lane_group_id,
            capacity=lane.capacity,
            mutex_zone_ids=lane.mutex_zone_ids,
            controlled_region_ids=lane.controlled_region_ids,
            clearance_zone_ids=tuple(dict.fromkeys(lane_zones.get(lane_id, ()))),
            centerline=lane.centerline,
        )
        for lane_id, lane in lanes.items()
    }
    return updated_vertices, updated_lanes


def _point_polyline_distance_sq(
    point: TrafficVertex,
    centerline: tuple[tuple[float, float], ...],
) -> float:
    if len(centerline) < 2:
        return 10**18
    return min(
        _point_segment_distance_sq(point, start, end)
        for start, end in zip(centerline, centerline[1:])
    )


def _point_segment_distance_sq(
    point: TrafficVertex,
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    start_x, start_y = start
    end_x, end_y = end
    dx = end_x - start_x
    dy = end_y - start_y
    length_sq = (dx * dx) + (dy * dy)
    if length_sq <= 1e-12:
        return ((point.x - start_x) ** 2) + ((point.y - start_y) ** 2)
    ratio = (
        ((point.x - start_x) * dx) + ((point.y - start_y) * dy)
    ) / length_sq
    ratio = max(0.0, min(1.0, ratio))
    nearest_x = start_x + (dx * ratio)
    nearest_y = start_y + (dy * ratio)
    return ((point.x - nearest_x) ** 2) + ((point.y - nearest_y) ** 2)


def _traffic_lane_from_edge(
    edge: GraphEdge,
    *,
    default_speed_mps: float,
    has_reverse: bool,
    controlled_regions_enabled: bool = True,
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
        controlled_region_ids=(
            _string_tuple_property(
                properties,
                (
                    "controlled_region",
                    "controlledRegion",
                    "corridor_region",
                    "corridorRegion",
                ),
            )
            if controlled_regions_enabled
            else ()
        ),
        centerline=_edge_centerline(edge),
    )


def _with_explicit_controlled_region_vertices(
    vertices: dict[str, TrafficVertex],
    lanes: dict[str, TrafficLane],
    landmarks: Mapping[str, Landmark],
) -> dict[str, TrafficVertex]:
    """Complete edge-authored corridor regions at internal graph vertices.

    A Traffic Editor rectangle primarily selects graph edges.  The edge
    resource is enough while a robot is moving, but a robot exactly on an
    untagged intermediate LM would otherwise appear to have left the region.
    That releases the admission token and also lets SIPP schedule a wait in
    the middle of a narrow passage.

    Infer membership only where the same explicit region continues through at
    least two distinct neighbouring LMs.  Single-edge endpoints stay outside
    as legal stop lines.  An explicit holding-point/corridor-boundary property
    always wins.  This preserves a deliberately authored stop line (normally
    paired with separate region IDs on its two sides).
    """
    regional_neighbors: dict[str, dict[str, set[str]]] = {
        name: {}
        for name in vertices
    }
    for lane in lanes.values():
        for region_id in lane.controlled_region_ids:
            regional_neighbors.setdefault(lane.from_lm, {}).setdefault(
                region_id,
                set(),
            ).add(lane.to_lm)
            regional_neighbors.setdefault(lane.to_lm, {}).setdefault(
                region_id,
                set(),
            ).add(lane.from_lm)

    updated: dict[str, TrafficVertex] = {}
    for name, vertex in vertices.items():
        landmark = landmarks.get(name)
        properties = (
            landmark.properties
            if landmark is not None and isinstance(landmark.properties, Mapping)
            else {}
        )
        is_boundary = _bool_property(
            properties,
            (
                "holding_point",
                "holdingPoint",
                "safe_holding_point",
                "safeHoldingPoint",
                "corridor_boundary",
                "corridorBoundary",
            ),
            False,
        )
        inferred_regions = (
            ()
            if is_boundary
            else tuple(sorted(
                region_id
                for region_id, neighbors in regional_neighbors.get(
                    name,
                    {},
                ).items()
                if len(neighbors) >= 2
            ))
        )
        region_ids = tuple(dict.fromkeys(
            (*vertex.controlled_region_ids, *inferred_regions)
        ))
        has_explicit_wait_policy = any(
            key in properties
            for key in (
                "can_wait",
                "canWait",
                "allow_wait",
                "allowWait",
                "wait_allowed",
                "waitAllowed",
            )
        )
        can_wait = vertex.can_wait
        if region_ids and not is_boundary and not has_explicit_wait_policy:
            can_wait = False
        updated[name] = TrafficVertex(
            id=vertex.id,
            x=vertex.x,
            y=vertex.y,
            can_wait=can_wait,
            is_parking=vertex.is_parking,
            is_charger=vertex.is_charger,
            mutex_zone_ids=vertex.mutex_zone_ids,
            controlled_region_ids=region_ids,
            clearance_zone_ids=vertex.clearance_zone_ids,
            rotation_conflict_lms=vertex.rotation_conflict_lms,
        )
    return updated


def _with_controlled_corridors(
    vertices: dict[str, TrafficVertex],
    lanes: dict[str, TrafficLane],
    landmarks: Mapping[str, Landmark],
    *,
    minimum_edges: int,
) -> tuple[dict[str, TrafficVertex], dict[str, TrafficLane]]:
    """Group maximal degree-two chains into whole-corridor mutex resources.

    Junctions, dead ends, parking points, chargers and explicitly marked
    holding points are corridor boundaries. A robot may queue there, but may
    not wait at an internal LM after entering the controlled region.
    """
    adjacency: dict[str, set[str]] = {name: set() for name in vertices}
    for lane in lanes.values():
        adjacency.setdefault(lane.from_lm, set()).add(lane.to_lm)
        adjacency.setdefault(lane.to_lm, set()).add(lane.from_lm)

    def is_explicit_boundary(name: str) -> bool:
        landmark = landmarks.get(name)
        properties = (
            landmark.properties
            if landmark is not None and isinstance(landmark.properties, Mapping)
            else {}
        )
        return _bool_property(
            properties,
            (
                "corridor_boundary",
                "corridorBoundary",
                "holding_point",
                "holdingPoint",
            ),
            False,
        )

    def is_explicit_holding_point(name: str) -> bool:
        landmark = landmarks.get(name)
        properties = (
            landmark.properties
            if landmark is not None and isinstance(landmark.properties, Mapping)
            else {}
        )
        return _bool_property(
            properties,
            (
                "holding_point",
                "holdingPoint",
                "safe_holding_point",
                "safeHoldingPoint",
            ),
            False,
        )

    boundaries = {
        name
        for name, vertex in vertices.items()
        if len(adjacency.get(name, ())) != 2
        or vertex.is_parking
        or vertex.is_charger
        or is_explicit_boundary(name)
    }
    visited_edges: set[tuple[str, str]] = set()
    chains: list[list[str]] = []
    for start in sorted(boundaries):
        for neighbor in sorted(adjacency.get(start, ())):
            edge_key = tuple(sorted((start, neighbor)))
            if edge_key in visited_edges:
                continue
            visited_edges.add(edge_key)
            chain = [start, neighbor]
            previous = start
            current = neighbor
            while current not in boundaries:
                onward = sorted(adjacency.get(current, set()) - {previous})
                if len(onward) != 1:
                    break
                next_node = onward[0]
                next_key = tuple(sorted((current, next_node)))
                if next_key in visited_edges:
                    break
                visited_edges.add(next_key)
                chain.append(next_node)
                previous, current = current, next_node
            if (
                len(chain) - 1 >= minimum_edges
                and chain[-1] in boundaries
                and chain[0] != chain[-1]
            ):
                chains.append(chain)

    vertex_regions: dict[str, list[str]] = {
        name: list(vertex.controlled_region_ids)
        for name, vertex in vertices.items()
    }
    lane_regions: dict[str, list[str]] = {
        name: list(lane.controlled_region_ids)
        for name, lane in lanes.items()
    }
    internal_nodes: set[str] = set()
    stopline_nodes: set[str] = set()
    for chain in chains:
        region_id = f"{chain[0]}<=>{chain[-1]}"
        for node in chain[1:-1]:
            vertex_regions[node].append(region_id)
            internal_nodes.add(node)
        # Keep one legal stop line on each approach while the controlled
        # region remains capacity-one. This gives a delayed rolling handoff a
        # place to pause without parking inside the multi-exit junction.
        if len(chain) > 2:
            stopline_nodes.add(chain[1])
            stopline_nodes.add(chain[-2])
        for start, end in zip(chain, chain[1:]):
            for lane_name in (lane_id(start, end), lane_id(end, start)):
                if lane_name in lane_regions:
                    lane_regions[lane_name].append(region_id)

    # A graph intersection is a transfer box, not a queueing pocket. Allowing
    # SIPP or a rolling chunk to wait there lets one stopped robot occupy the
    # exits of several otherwise independent corridor regions. Real maps can
    # opt a widened intersection back in with an explicit holding-point flag.
    transit_junctions = {
        name
        for name, vertex in vertices.items()
        if len(adjacency.get(name, ())) >= 3
        and not vertex.is_parking
        and not vertex.is_charger
        and not is_explicit_holding_point(name)
    }

    updated_vertices = {
        name: TrafficVertex(
            id=vertex.id,
            x=vertex.x,
            y=vertex.y,
            can_wait=(
                vertex.can_wait
                and name not in transit_junctions
                and (
                    name not in internal_nodes
                    or name in stopline_nodes
                )
            ),
            is_parking=vertex.is_parking,
            is_charger=vertex.is_charger,
            mutex_zone_ids=vertex.mutex_zone_ids,
            controlled_region_ids=tuple(dict.fromkeys(vertex_regions[name])),
            clearance_zone_ids=vertex.clearance_zone_ids,
            rotation_conflict_lms=vertex.rotation_conflict_lms,
        )
        for name, vertex in vertices.items()
    }
    updated_lanes = {
        name: TrafficLane(
            id=lane.id,
            from_lm=lane.from_lm,
            to_lm=lane.to_lm,
            length_m=lane.length_m,
            max_speed_mps=lane.max_speed_mps,
            lane_group_id=lane.lane_group_id,
            capacity=lane.capacity,
            mutex_zone_ids=lane.mutex_zone_ids,
            controlled_region_ids=tuple(dict.fromkeys(lane_regions[name])),
            clearance_zone_ids=lane.clearance_zone_ids,
            centerline=lane.centerline,
        )
        for name, lane in lanes.items()
    }
    return updated_vertices, updated_lanes


def _edge_centerline(edge: GraphEdge) -> tuple[tuple[float, float], ...]:
    geometry = edge.geometry
    if geometry is not None and str(geometry.geometry).lower() == "bezier":
        controls = [
            (float(point.x), float(point.y))
            for point in geometry.control_points
        ]
        if len(controls) >= 2:
            return tuple(
                _bezier_xy(controls, step / 20.0)
                for step in range(21)
            )
    points = tuple(
        (float(point.x), float(point.y))
        for point in edge.world_points
    )
    return points if len(points) >= 2 else ()


def _bezier_xy(
    controls: list[tuple[float, float]],
    t: float,
) -> tuple[float, float]:
    points = list(controls)
    while len(points) > 1:
        points = [
            (
                start[0] + ((end[0] - start[0]) * t),
                start[1] + ((end[1] - start[1]) * t),
            )
            for start, end in zip(points, points[1:])
        ]
    return points[0]


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
