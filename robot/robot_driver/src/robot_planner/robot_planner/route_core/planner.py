from __future__ import annotations

import heapq
import math

import numpy as np

from ..math import TrajectoryMath
from .models import GraphEdge, Landmark, PlannedRoute, WorldPoint
from .params import load_route_params


class LmRoutePlanner:
    def __init__(
        self,
        landmarks: dict[str, Landmark],
        edges: list[GraphEdge],
        params: dict[str, object] | None = None,
    ) -> None:
        self.landmarks = landmarks
        self.edges = edges
        self.params = params or load_route_params()
        planner_params = self.params.get("planner", {})
        if not isinstance(planner_params, dict):
            planner_params = {}
        self.default_sample_distance = float(
            planner_params.get("trajectory_sample_distance", 0.01)
        )
        self._adjacency = self._build_adjacency()
        self._edge_by_key = {
            (edge.from_name, edge.to_name): edge
            for edge in edges
        }

    def nearest_landmark(self, x: float, y: float) -> tuple[Landmark, float]:
        if not self.landmarks:
            raise ValueError("map contains no landmarks")
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(landmark.x - x, landmark.y - y),
        )
        distance = math.hypot(nearest.x - x, nearest.y - y)
        return nearest, distance

    def find_route(self, start: str, goal: str) -> PlannedRoute:
        if start not in self.landmarks:
            raise ValueError(f"Unknown start LM: {start}")
        if goal not in self.landmarks:
            raise ValueError(f"Unknown goal LM: {goal}")
        if start == goal:
            return PlannedRoute(nodes=[start], edges=[], length=0.0)

        open_heap: list[tuple[float, str]] = [(0.0, start)]
        came_from: dict[str, tuple[str, GraphEdge]] = {}
        g_score: dict[str, float] = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                ordered_edges: list[GraphEdge] = []
                while current in came_from:
                    previous, edge = came_from[current]
                    ordered_edges.append(edge)
                    current = previous
                ordered_edges.reverse()
                nodes = [start] + [edge.to_name for edge in ordered_edges]
                return PlannedRoute(nodes=nodes, edges=ordered_edges, length=g_score[goal])

            for edge in self._adjacency.get(current, []):
                tentative = g_score[current] + edge.length
                if tentative >= g_score.get(edge.to_name, math.inf):
                    continue

                came_from[edge.to_name] = (current, edge)
                g_score[edge.to_name] = tentative
                heuristic = self._world_distance(self.landmarks[edge.to_name], self.landmarks[goal])
                heapq.heappush(open_heap, (tentative + heuristic, edge.to_name))

        raise ValueError(f"No route found from {start} to {goal}")

    def build_route_catalog(self) -> dict[str, dict[str, object]]:
        names = sorted(self.landmarks)
        catalog: dict[str, dict[str, object]] = {}
        for start in names:
            for goal in names:
                try:
                    route = self.find_route(start, goal)
                except ValueError:
                    continue
                catalog[self.route_key(start, goal)] = route.to_dict()
        return catalog

    def get_edge(self, from_name: str, to_name: str) -> GraphEdge | None:
        return self._edge_by_key.get((from_name, to_name))

    def sample_route(
        self,
        route: PlannedRoute,
        sample_distance: float | None = None,
    ) -> list[dict[str, float | str]]:
        sample_distance = sample_distance or self.default_sample_distance
        if not route.nodes:
            return []
        if not route.edges:
            goal = self.landmarks[route.nodes[0]]
            return [{"x": goal.x, "y": goal.y, "yaw": 0.0, "edgeId": f"{goal.name}->{goal.name}"}]

        sampled_points: list[dict[str, float | str]] = []
        for edge in route.edges:
            edge_points = self._sample_edge(edge, sample_distance)
            if sampled_points and edge_points:
                edge_points = edge_points[1:]
            sampled_points.extend(edge_points)
        return sampled_points

    def route_key(self, start: str, goal: str) -> str:
        return f"{start}|{goal}"

    def _sample_edge(self, edge: GraphEdge, sample_distance: float) -> list[dict[str, float | str]]:
        start = self.landmarks[edge.from_name]
        end = self.landmarks[edge.to_name]
        edge_id = f"{edge.from_name}->{edge.to_name}"
        if edge.geometry is None or edge.geometry.geometry != "bezier":
            return self._sample_line(start.to_point(), end.to_point(), sample_distance, edge_id)

        points = edge.geometry.control_points
        control_matrix = np.asarray(
            [(point.x, point.y) for point in points],
            dtype=np.float64,
        )
        sampled_xy, derivatives = (
            TrajectoryMath.sample_cubic_bezier_by_arc_length(
                control_matrix,
                sample_distance,
            )
        )
        sampled_yaw = np.arctan2(derivatives[:, 1], derivatives[:, 0])
        if edge.motion_direction_code() == 1:
            sampled_yaw = TrajectoryMath.normalize_angles(sampled_yaw + math.pi)
        direction = edge.motion_direction_label(edge.motion_direction_code())
        return [
            {
                "x": float(point[0]),
                "y": float(point[1]),
                "yaw": float(yaw),
                "edgeId": edge_id,
                "motionDirection": direction,
            }
            for point, yaw in zip(sampled_xy, sampled_yaw)
        ]

    def _sample_line(
        self,
        start: WorldPoint,
        end: WorldPoint,
        sample_distance: float,
        edge_id: str,
    ) -> list[dict[str, float | str]]:
        dx = end.x - start.x
        dy = end.y - start.y
        length = math.hypot(dx, dy)
        if length == 0.0:
            return [{"x": start.x, "y": start.y, "yaw": 0.0, "edgeId": edge_id}]

        steps = max(1, math.ceil(length / sample_distance))
        yaw = math.atan2(dy, dx)
        edge = self._edge_by_key.get(tuple(edge_id.split("->", 1))) if "->" in edge_id else None
        if edge is not None:
            yaw = self._motion_yaw(edge, yaw)
        ratios = np.linspace(0.0, 1.0, steps + 1, dtype=np.float64)
        start_vector = np.asarray((start.x, start.y), dtype=np.float64)
        displacement = np.asarray((dx, dy), dtype=np.float64)
        sampled_xy = start_vector[None, :] + ratios[:, None] * displacement[None, :]
        return [
            {
                "x": float(point[0]),
                "y": float(point[1]),
                "yaw": yaw,
                "edgeId": edge_id,
                "motionDirection": (
                    edge.motion_direction_label(edge.motion_direction_code())
                    if edge is not None
                    else "not_specified"
                ),
            }
            for point in sampled_xy
        ]

    def _build_adjacency(self) -> dict[str, list[GraphEdge]]:
        adjacency: dict[str, list[GraphEdge]] = {}
        for edge in self.edges:
            adjacency.setdefault(edge.from_name, []).append(edge)
        return adjacency

    def _motion_yaw(self, edge: GraphEdge, tangent_yaw: float) -> float:
        if edge.motion_direction_code() == 1:
            return TrajectoryMath.normalize_angle(tangent_yaw + math.pi)
        return tangent_yaw

    def _world_distance(self, first: Landmark, second: Landmark) -> float:
        return math.hypot(second.x - first.x, second.y - first.y)
