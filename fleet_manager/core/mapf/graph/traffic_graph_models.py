"""Stable traffic-graph data types and graph queries."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .reservations import ResourceId

if TYPE_CHECKING:
    from fleet_manager.core.route_core.models import GraphEdge, Landmark


def lane_id(from_lm: str, to_lm: str) -> str:
    return f"{from_lm}->{to_lm}"


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

    def with_traffic_policy(
        self,
        *,
        can_wait: bool,
        controlled_region_ids: tuple[str, ...],
    ) -> "TrafficVertex":
        return TrafficVertex(
            id=self.id,
            x=self.x,
            y=self.y,
            can_wait=can_wait,
            is_parking=self.is_parking,
            is_charger=self.is_charger,
            mutex_zone_ids=self.mutex_zone_ids,
            controlled_region_ids=controlled_region_ids,
            clearance_zone_ids=self.clearance_zone_ids,
            rotation_conflict_lms=self.rotation_conflict_lms,
        )


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

    def with_controlled_regions(
        self,
        region_ids: tuple[str, ...],
    ) -> "TrafficLane":
        return TrafficLane(
            id=self.id,
            from_lm=self.from_lm,
            to_lm=self.to_lm,
            length_m=self.length_m,
            max_speed_mps=self.max_speed_mps,
            lane_group_id=self.lane_group_id,
            capacity=self.capacity,
            mutex_zone_ids=self.mutex_zone_ids,
            controlled_region_ids=region_ids,
            clearance_zone_ids=self.clearance_zone_ids,
            centerline=self.centerline,
        )


@dataclass(slots=True)
class TrafficGraph:
    vertices: dict[str, TrafficVertex]
    lanes: dict[str, TrafficLane]
    outgoing: dict[str, list[str]]

    @classmethod
    def from_route_core(
        cls,
        landmarks: Mapping[str, "Landmark"],
        edges: list["GraphEdge"],
        *,
        default_speed_mps: float,
        min_robot_center_distance_m: float = 0.0,
        rotation_min_robot_center_distance_m: float = 0.0,
        explicit_controlled_regions_enabled: bool = True,
        controlled_corridors_enabled: bool = False,
        controlled_corridor_min_edges: int = 2,
    ) -> "TrafficGraph":
        """Build a traffic graph while keeping the historic entry point."""

        from .traffic_graph_builder import TrafficGraphBuilder

        return TrafficGraphBuilder(
            landmarks=landmarks,
            edges=edges,
            default_speed_mps=default_speed_mps,
            min_robot_center_distance_m=min_robot_center_distance_m,
            rotation_min_robot_center_distance_m=(
                rotation_min_robot_center_distance_m
            ),
            explicit_controlled_regions_enabled=(
                explicit_controlled_regions_enabled
            ),
            controlled_corridors_enabled=controlled_corridors_enabled,
            controlled_corridor_min_edges=controlled_corridor_min_edges,
        ).build(graph_type=cls)

    def neighbors(self, lm_id: str) -> list[TrafficLane]:
        return [
            self.lanes[lane_name]
            for lane_name in self.outgoing.get(lm_id, [])
            if lane_name in self.lanes
        ]

    def lane_for(self, from_lm: str, to_lm: str) -> TrafficLane | None:
        return self.lanes.get(lane_id(from_lm, to_lm))

    def lane_resources(self, lane: TrafficLane) -> tuple[ResourceId, ...]:
        resources = [
            ResourceId("lane", lane.id),
            ResourceId("lane_group", lane.lane_group_id),
        ]
        resources.extend(
            ResourceId("mutex_zone", zone_id)
            for zone_id in lane.mutex_zone_ids
        )
        resources.extend(
            ResourceId("controlled_region", region_id)
            for region_id in lane.controlled_region_ids
        )
        resources.extend(
            ResourceId("clearance", zone_id)
            for zone_id in lane.clearance_zone_ids
        )

        # The swept lane corridor includes both endpoint clearances for the
        # complete traversal.
        resources.extend(self.vertex_resources(lane.from_lm))
        resources.extend(self.vertex_resources(lane.to_lm))
        return tuple(dict.fromkeys(resources))

    def vertex_resources(self, lm_id: str) -> tuple[ResourceId, ...]:
        vertex = self.vertices.get(lm_id)
        if vertex is None:
            return (ResourceId("vertex", lm_id),)

        resources = [ResourceId("vertex", lm_id)]
        resources.extend(
            ResourceId("mutex_zone", zone_id)
            for zone_id in vertex.mutex_zone_ids
        )
        resources.extend(
            ResourceId("controlled_region", region_id)
            for region_id in vertex.controlled_region_ids
        )
        resources.extend(
            ResourceId("clearance", zone_id)
            for zone_id in vertex.clearance_zone_ids
        )
        return tuple(resources)

    def rotation_resources(self, lm_id: str) -> tuple[ResourceId, ...]:
        """Return resources swept by an in-place rotation at ``lm_id``."""

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
            lane_capacity = max(1, int(lane.capacity))
            capacities[ResourceId("lane", lane.id)] = lane_capacity
            group_resource = ResourceId("lane_group", lane.lane_group_id)
            capacities[group_resource] = max(
                capacities.get(group_resource, 1),
                lane_capacity,
            )
        return capacities

    def controlled_region_ids(self) -> tuple[str, ...]:
        return tuple(
            sorted(
                {
                    region_id
                    for lane in self.lanes.values()
                    for region_id in lane.controlled_region_ids
                }
            )
        )

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
            if (
                next_vertex is None
                or region_id not in next_vertex.controlled_region_ids
            ):
                break
        return cursor
