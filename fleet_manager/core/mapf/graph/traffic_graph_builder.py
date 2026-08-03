"""Build :class:`TrafficGraph` instances from route-core map objects."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

from fleet_manager.core.math.curves import bezier_point
from fleet_manager.core.math.geometry import Vector2
from fleet_manager.core.route_core.models import GraphEdge, Landmark

from .traffic_graph_models import (
    TrafficGraph,
    TrafficLane,
    TrafficVertex,
    lane_id,
)
from .traffic_graph_properties import (
    has_any,
    property_map,
    read_bool,
    read_float,
    read_int,
    read_text,
    read_text_tuple,
)


WAIT_KEYS = (
    "can_wait",
    "canWait",
    "allow_wait",
    "allowWait",
    "wait_allowed",
    "waitAllowed",
)
PARKING_KEYS = ("is_parking", "isParking", "parking", "parkPoint")
CHARGER_KEYS = ("is_charger", "isCharger", "charger", "chargePoint")
MUTEX_KEYS = ("mutex_zone", "mutexZone", "mutex_group", "mutexGroup")
CONTROLLED_REGION_KEYS = (
    "controlled_region",
    "controlledRegion",
    "corridor_region",
    "corridorRegion",
)
HOLDING_POINT_KEYS = (
    "holding_point",
    "holdingPoint",
    "safe_holding_point",
    "safeHoldingPoint",
)
CORRIDOR_BOUNDARY_KEYS = ("corridor_boundary", "corridorBoundary")


@dataclass(slots=True)
class TrafficGraphBuilder:
    """Compose the map policies that turn route geometry into a traffic graph."""

    landmarks: Mapping[str, Landmark]
    edges: list[GraphEdge]
    default_speed_mps: float
    min_robot_center_distance_m: float = 0.0
    rotation_min_robot_center_distance_m: float = 0.0
    explicit_controlled_regions_enabled: bool = True
    controlled_corridors_enabled: bool = False
    controlled_corridor_min_edges: int = 2

    def build(
        self,
        *,
        graph_type: type[TrafficGraph] = TrafficGraph,
    ) -> TrafficGraph:
        min_distance = max(0.0, float(self.min_robot_center_distance_m))
        rotation_distance = max(
            0.0,
            float(self.rotation_min_robot_center_distance_m),
        )

        vertices = self._build_vertices()
        positions = (
            {
                vertex.id: Vector2(vertex.x, vertex.y)
                for vertex in vertices.values()
            }
            if min_distance > 0.0 or rotation_distance > 0.0
            else {}
        )
        vertices = self._add_vertex_proximity(
            vertices,
            positions,
            clearance_distance=min_distance,
            rotation_distance=rotation_distance,
        )
        lanes, outgoing = self._build_lanes()

        if self.explicit_controlled_regions_enabled:
            vertices = self._infer_explicit_region_vertices(vertices, lanes)
        if self.controlled_corridors_enabled:
            vertices, lanes = self._add_controlled_corridors(
                vertices,
                lanes,
                minimum_edges=max(1, int(self.controlled_corridor_min_edges)),
            )

        vertices, lanes = self._add_lane_clearances(
            vertices,
            lanes,
            positions,
            min_distance,
        )
        for lane_names in outgoing.values():
            lane_names.sort()
        return graph_type(vertices=vertices, lanes=lanes, outgoing=outgoing)

    def _build_vertices(self) -> dict[str, TrafficVertex]:
        return {
            name: self._vertex_from_landmark(landmark)
            for name, landmark in self.landmarks.items()
        }

    def _vertex_from_landmark(self, landmark: Landmark) -> TrafficVertex:
        properties = property_map(landmark.properties)
        name = str(landmark.name)
        controlled_regions = (
            read_text_tuple(properties, CONTROLLED_REGION_KEYS)
            if self.explicit_controlled_regions_enabled
            else ()
        )
        return TrafficVertex(
            id=name,
            x=float(landmark.x),
            y=float(landmark.y),
            can_wait=read_bool(properties, WAIT_KEYS, True),
            is_parking=read_bool(
                properties,
                PARKING_KEYS,
                name.upper().startswith("PP"),
            ),
            is_charger=read_bool(
                properties,
                CHARGER_KEYS,
                name.upper().startswith("CP"),
            ),
            mutex_zone_ids=read_text_tuple(properties, MUTEX_KEYS),
            controlled_region_ids=controlled_regions,
        )

    def _build_lanes(
        self,
    ) -> tuple[dict[str, TrafficLane], dict[str, list[str]]]:
        edge_keys = {
            (edge.from_name, edge.to_name)
            for edge in self.edges
        }
        lanes: dict[str, TrafficLane] = {}
        outgoing: dict[str, list[str]] = {
            name: []
            for name in self.landmarks
        }
        for edge in self.edges:
            if (
                edge.from_name not in self.landmarks
                or edge.to_name not in self.landmarks
            ):
                continue
            lane = self._lane_from_edge(
                edge,
                has_reverse=(edge.to_name, edge.from_name) in edge_keys,
            )
            lanes[lane.id] = lane
            outgoing.setdefault(lane.from_lm, []).append(lane.id)
        return lanes, outgoing

    def _lane_from_edge(
        self,
        edge: GraphEdge,
        *,
        has_reverse: bool,
    ) -> TrafficLane:
        properties = property_map(edge.properties)
        group_id = read_text(
            properties,
            (
                "lane_group",
                "laneGroup",
                "lane_group_id",
                "laneGroupId",
                "resource_id",
                "resourceId",
            )
        )
        if not group_id:
            group_id = (
                self._canonical_lane_group(edge.from_name, edge.to_name)
                if has_reverse
                else lane_id(edge.from_name, edge.to_name)
            )
        controlled_regions = (
            read_text_tuple(properties, CONTROLLED_REGION_KEYS)
            if self.explicit_controlled_regions_enabled
            else ()
        )
        return TrafficLane(
            id=lane_id(edge.from_name, edge.to_name),
            from_lm=edge.from_name,
            to_lm=edge.to_name,
            length_m=max(0.0, float(edge.length)),
            max_speed_mps=read_float(
                properties,
                ("max_speed", "maxSpeed", "maxspeed", "speed", "speedLimit"),
                max(0.02, float(self.default_speed_mps)),
            ),
            lane_group_id=group_id,
            capacity=read_int(
                properties,
                ("capacity", "trafficCapacity", "laneCapacity"),
                1,
            ),
            mutex_zone_ids=read_text_tuple(properties, MUTEX_KEYS),
            controlled_region_ids=controlled_regions,
            centerline=self._edge_centerline(edge),
        )

    def _add_vertex_proximity(
        self,
        vertices: dict[str, TrafficVertex],
        positions: dict[str, Vector2],
        *,
        clearance_distance: float,
        rotation_distance: float,
    ) -> dict[str, TrafficVertex]:
        if clearance_distance <= 0.0 and rotation_distance <= 0.0:
            return vertices

        zones: dict[str, list[str]] = {name: [] for name in vertices}
        conflicts: dict[str, list[str]] = {name: [] for name in vertices}
        ordered = sorted(vertices.values(), key=lambda item: item.id)
        ordered_positions = [
            (vertex, positions[vertex.id])
            for vertex in ordered
        ]
        clearance_squared = clearance_distance * clearance_distance
        rotation_squared = rotation_distance * rotation_distance
        if clearance_distance > 0.0 and rotation_distance > 0.0:
            self._collect_combined_proximity(
                ordered_positions,
                clearance_squared=clearance_squared,
                rotation_squared=rotation_squared,
                zones=zones,
                conflicts=conflicts,
            )
        elif clearance_distance > 0.0:
            self._collect_clearance_zones(
                ordered_positions,
                threshold_squared=clearance_squared,
                zones=zones,
            )
        else:
            self._collect_rotation_conflicts(
                ordered_positions,
                threshold_squared=rotation_squared,
                conflicts=conflicts,
            )

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
                clearance_zone_ids=(
                    tuple(zones.get(name, ()))
                    if clearance_distance > 0.0
                    else vertex.clearance_zone_ids
                ),
                rotation_conflict_lms=(
                    tuple(conflicts.get(name, ()))
                    if rotation_distance > 0.0
                    else vertex.rotation_conflict_lms
                ),
            )
            for name, vertex in vertices.items()
        }

    @staticmethod
    def _collect_combined_proximity(
        ordered: list[tuple[TrafficVertex, Vector2]],
        *,
        clearance_squared: float,
        rotation_squared: float,
        zones: dict[str, list[str]],
        conflicts: dict[str, list[str]],
    ) -> None:
        for index, (first, first_position) in enumerate(ordered):
            for second, second_position in ordered[index + 1 :]:
                distance_squared = (
                    (first_position.x - second_position.x) ** 2
                    + (first_position.y - second_position.y) ** 2
                )
                if distance_squared < clearance_squared:
                    zone_id = f"{first.id}<->{second.id}"
                    zones[first.id].append(zone_id)
                    zones[second.id].append(zone_id)
                if distance_squared < rotation_squared:
                    conflicts[first.id].append(second.id)
                    conflicts[second.id].append(first.id)

    @staticmethod
    def _collect_clearance_zones(
        ordered: list[tuple[TrafficVertex, Vector2]],
        *,
        threshold_squared: float,
        zones: dict[str, list[str]],
    ) -> None:
        for index, (first, first_position) in enumerate(ordered):
            for second, second_position in ordered[index + 1 :]:
                distance_squared = (
                    (first_position.x - second_position.x) ** 2
                    + (first_position.y - second_position.y) ** 2
                )
                if distance_squared >= threshold_squared:
                    continue
                zone_id = f"{first.id}<->{second.id}"
                zones[first.id].append(zone_id)
                zones[second.id].append(zone_id)

    @staticmethod
    def _collect_rotation_conflicts(
        ordered: list[tuple[TrafficVertex, Vector2]],
        *,
        threshold_squared: float,
        conflicts: dict[str, list[str]],
    ) -> None:
        for index, (first, first_position) in enumerate(ordered):
            for second, second_position in ordered[index + 1 :]:
                distance_squared = (
                    (first_position.x - second_position.x) ** 2
                    + (first_position.y - second_position.y) ** 2
                )
                if distance_squared >= threshold_squared:
                    continue
                conflicts[first.id].append(second.id)
                conflicts[second.id].append(first.id)

    def _add_lane_clearances(
        self,
        vertices: dict[str, TrafficVertex],
        lanes: dict[str, TrafficLane],
        positions: dict[str, Vector2],
        min_distance: float,
    ) -> tuple[dict[str, TrafficVertex], dict[str, TrafficLane]]:
        if min_distance <= 0.0:
            return vertices, lanes

        vertex_zones = {
            name: list(vertex.clearance_zone_ids)
            for name, vertex in vertices.items()
        }
        lane_zones: dict[str, list[str]] = {
            lane_name: []
            for lane_name in lanes
        }
        threshold_squared = min_distance * min_distance
        cell_size = min_distance
        vertex_order = {
            name: index
            for index, name in enumerate(vertices)
        }
        vector_cache = {
            (position.x, position.y): position
            for position in positions.values()
        }
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
            min_x = min(point[0] for point in centerline) - min_distance
            max_x = max(point[0] for point in centerline) + min_distance
            min_y = min(point[1] for point in centerline) - min_distance
            max_y = max(point[1] for point in centerline) + min_distance
            candidate_names: set[str] = set()
            for cell_x in range(
                math.floor(min_x / cell_size),
                math.floor(max_x / cell_size) + 1,
            ):
                for cell_y in range(
                    math.floor(min_y / cell_size),
                    math.floor(max_y / cell_size) + 1,
                ):
                    candidate_names.update(
                        vertex_grid.get((cell_x, cell_y), ())
                    )
            centerline_vectors = self._vectors_for_lane(
                centerline=centerline,
                start=start,
                end=end,
                positions=positions,
                cache=vector_cache,
            )
            for vertex_name in sorted(
                candidate_names,
                key=lambda name: vertex_order[name],
            ):
                vertex = vertices[vertex_name]
                if vertex.id in {lane.from_lm, lane.to_lm}:
                    continue
                if (
                    self._polyline_distance_squared(
                        positions[vertex.id],
                        centerline_vectors,
                    )
                    >= threshold_squared
                ):
                    continue
                zone_id = f"{lane.id}<->{vertex.id}"
                lane_zones[lane.id].append(zone_id)
                vertex_zones[vertex.id].append(zone_id)

        return self._apply_lane_clearance_zones(
            vertices,
            lanes,
            vertex_zones=vertex_zones,
            lane_zones=lane_zones,
        )

    @staticmethod
    def _apply_lane_clearance_zones(
        vertices: dict[str, TrafficVertex],
        lanes: dict[str, TrafficLane],
        *,
        vertex_zones: dict[str, list[str]],
        lane_zones: dict[str, list[str]],
    ) -> tuple[dict[str, TrafficVertex], dict[str, TrafficLane]]:
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
                clearance_zone_ids=tuple(
                    dict.fromkeys(vertex_zones.get(name, ()))
                ),
                rotation_conflict_lms=vertex.rotation_conflict_lms,
            )
            for name, vertex in vertices.items()
        }
        updated_lanes = {
            lane_name: TrafficLane(
                id=lane.id,
                from_lm=lane.from_lm,
                to_lm=lane.to_lm,
                length_m=lane.length_m,
                max_speed_mps=lane.max_speed_mps,
                lane_group_id=lane.lane_group_id,
                capacity=lane.capacity,
                mutex_zone_ids=lane.mutex_zone_ids,
                controlled_region_ids=lane.controlled_region_ids,
                clearance_zone_ids=tuple(
                    dict.fromkeys(lane_zones.get(lane_name, ()))
                ),
                centerline=lane.centerline,
            )
            for lane_name, lane in lanes.items()
        }
        return updated_vertices, updated_lanes

    @staticmethod
    def _vectors_for_lane(
        *,
        centerline: tuple[tuple[float, float], ...],
        start: TrafficVertex,
        end: TrafficVertex,
        positions: dict[str, Vector2],
        cache: dict[tuple[float, float], Vector2],
    ) -> tuple[Vector2, ...]:
        if (
            len(centerline) == 2
            and centerline[0] == (start.x, start.y)
            and centerline[1] == (end.x, end.y)
        ):
            return positions[start.id], positions[end.id]

        vectors: list[Vector2] = []
        for x, y in centerline:
            key = (x, y)
            point = cache.get(key)
            if point is None:
                point = Vector2(x, y)
                cache[key] = point
            vectors.append(point)
        return tuple(vectors)

    @classmethod
    def _polyline_distance_squared(
        cls,
        point: Vector2,
        centerline: tuple[Vector2, ...],
    ) -> float:
        if len(centerline) < 2:
            return 10**18
        return min(
            cls._segment_distance_squared(point, start, end)
            for start, end in zip(centerline, centerline[1:])
        )

    @staticmethod
    def _segment_distance_squared(
        point: Vector2,
        start: Vector2,
        end: Vector2,
    ) -> float:
        delta_x = end.x - start.x
        delta_y = end.y - start.y
        length_squared = (delta_x * delta_x) + (delta_y * delta_y)
        if length_squared <= 1e-12:
            return (
                ((point.x - start.x) ** 2)
                + ((point.y - start.y) ** 2)
            )

        ratio = (
            ((point.x - start.x) * delta_x)
            + ((point.y - start.y) * delta_y)
        ) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        nearest_x = start.x + (delta_x * ratio)
        nearest_y = start.y + (delta_y * ratio)
        return (
            ((point.x - nearest_x) ** 2)
            + ((point.y - nearest_y) ** 2)
        )

    def _infer_explicit_region_vertices(
        self,
        vertices: dict[str, TrafficVertex],
        lanes: dict[str, TrafficLane],
    ) -> dict[str, TrafficVertex]:
        """Fill explicit edge regions at their internal graph vertices."""

        regional_neighbors = self._regional_neighbors(vertices, lanes)
        updated: dict[str, TrafficVertex] = {}
        for name, vertex in vertices.items():
            properties = self._landmark_properties(name)
            is_boundary = read_bool(
                properties,
                (*HOLDING_POINT_KEYS, *CORRIDOR_BOUNDARY_KEYS),
                False,
            )
            inferred_regions = (
                ()
                if is_boundary
                else tuple(
                    sorted(
                        region_id
                        for region_id, neighbors in regional_neighbors.get(
                            name,
                            {},
                        ).items()
                        if len(neighbors) >= 2
                    )
                )
            )
            region_ids = tuple(
                dict.fromkeys(
                    (*vertex.controlled_region_ids, *inferred_regions)
                )
            )
            can_wait = vertex.can_wait
            if (
                region_ids
                and not is_boundary
                and not has_any(properties, WAIT_KEYS)
            ):
                can_wait = False
            updated[name] = vertex.with_traffic_policy(
                can_wait=can_wait,
                controlled_region_ids=region_ids,
            )
        return updated

    @staticmethod
    def _regional_neighbors(
        vertices: dict[str, TrafficVertex],
        lanes: dict[str, TrafficLane],
    ) -> dict[str, dict[str, set[str]]]:
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
        return regional_neighbors

    def _add_controlled_corridors(
        self,
        vertices: dict[str, TrafficVertex],
        lanes: dict[str, TrafficLane],
        *,
        minimum_edges: int,
    ) -> tuple[dict[str, TrafficVertex], dict[str, TrafficLane]]:
        """Group maximal degree-two chains into whole-corridor resources."""

        adjacency = self._undirected_adjacency(vertices, lanes)
        boundaries = {
            name
            for name, vertex in vertices.items()
            if len(adjacency.get(name, ())) != 2
            or vertex.is_parking
            or vertex.is_charger
            or self._is_corridor_boundary(name)
        }
        chains = self._corridor_chains(
            adjacency,
            boundaries,
            minimum_edges=minimum_edges,
        )
        return self._apply_corridor_chains(
            vertices,
            lanes,
            adjacency,
            chains,
        )

    @staticmethod
    def _undirected_adjacency(
        vertices: dict[str, TrafficVertex],
        lanes: dict[str, TrafficLane],
    ) -> dict[str, set[str]]:
        adjacency: dict[str, set[str]] = {
            name: set()
            for name in vertices
        }
        for lane in lanes.values():
            adjacency.setdefault(lane.from_lm, set()).add(lane.to_lm)
            adjacency.setdefault(lane.to_lm, set()).add(lane.from_lm)
        return adjacency

    @staticmethod
    def _corridor_chains(
        adjacency: dict[str, set[str]],
        boundaries: set[str],
        *,
        minimum_edges: int,
    ) -> list[list[str]]:
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
                    onward = sorted(
                        adjacency.get(current, set()) - {previous}
                    )
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
        return chains

    def _apply_corridor_chains(
        self,
        vertices: dict[str, TrafficVertex],
        lanes: dict[str, TrafficLane],
        adjacency: dict[str, set[str]],
        chains: list[list[str]],
    ) -> tuple[dict[str, TrafficVertex], dict[str, TrafficLane]]:
        vertex_regions = {
            name: list(vertex.controlled_region_ids)
            for name, vertex in vertices.items()
        }
        lane_regions = {
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
            if len(chain) > 2:
                stopline_nodes.add(chain[1])
                stopline_nodes.add(chain[-2])
            for start, end in zip(chain, chain[1:]):
                for lane_name in (
                    lane_id(start, end),
                    lane_id(end, start),
                ):
                    if lane_name in lane_regions:
                        lane_regions[lane_name].append(region_id)

        transit_junctions = {
            name
            for name, vertex in vertices.items()
            if len(adjacency.get(name, ())) >= 3
            and not vertex.is_parking
            and not vertex.is_charger
            and not self._is_holding_point(name)
        }
        updated_vertices = {
            name: vertex.with_traffic_policy(
                can_wait=(
                    vertex.can_wait
                    and name not in transit_junctions
                    and (
                        name not in internal_nodes
                        or name in stopline_nodes
                    )
                ),
                controlled_region_ids=tuple(
                    dict.fromkeys(vertex_regions[name])
                ),
            )
            for name, vertex in vertices.items()
        }
        updated_lanes = {
            name: lane.with_controlled_regions(
                tuple(
                    dict.fromkeys(lane_regions[name])
                )
            )
            for name, lane in lanes.items()
        }
        return updated_vertices, updated_lanes

    def _landmark_properties(
        self,
        name: str,
    ) -> Mapping[str, object]:
        landmark = self.landmarks.get(name)
        return property_map(
            landmark.properties if landmark is not None else {}
        )

    def _is_corridor_boundary(self, name: str) -> bool:
        return read_bool(
            self._landmark_properties(name),
            (*CORRIDOR_BOUNDARY_KEYS, "holding_point", "holdingPoint"),
            False,
        )

    def _is_holding_point(self, name: str) -> bool:
        return read_bool(
            self._landmark_properties(name),
            HOLDING_POINT_KEYS,
            False,
        )

    @classmethod
    def _edge_centerline(
        cls,
        edge: GraphEdge,
    ) -> tuple[tuple[float, float], ...]:
        geometry = edge.geometry
        if (
            geometry is not None
            and str(geometry.geometry).lower() == "bezier"
        ):
            controls = [
                Vector2(point.x, point.y)
                for point in geometry.control_points
            ]
            if len(controls) >= 2:
                return tuple(
                    cls._as_xy(
                        cls._bezier_point(controls, step / 20.0)
                    )
                    for step in range(21)
                )

        points = tuple(
            (float(point.x), float(point.y))
            for point in edge.world_points
        )
        return points if len(points) >= 2 else ()

    @staticmethod
    def _bezier_point(
        controls: list[Vector2],
        fraction: float,
    ) -> Vector2:
        return bezier_point(controls, fraction)

    @staticmethod
    def _as_xy(point: Vector2) -> tuple[float, float]:
        return point.x, point.y

    @staticmethod
    def _canonical_lane_group(first: str, second: str) -> str:
        source, destination = sorted((str(first), str(second)))
        return f"{source}<->{destination}"
