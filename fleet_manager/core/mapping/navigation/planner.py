from __future__ import annotations

from collections.abc import Callable, Iterable
import math

from fleet_manager.core.algorithms.math.curves import cubic_bezier_derivative
from fleet_manager.core.algorithms.math.geometry import Vector2
from fleet_manager.core.algorithms.math.search.astar import AStarSolver

from ..maps.models import GraphEdge, Landmark, PlannedRoute, WorldPoint
from .params import load_route_params


class _LandmarkRouteProblem:
    """Adapt landmark routing policies to the shared A* search loop."""

    def __init__(
        self,
        planner: LmRoutePlanner,
        start: str,
        goal: str,
        blocked_edges: set[tuple[str, str]],
        edge_penalties: dict[tuple[str, str], float],
    ) -> None:
        self._planner = planner
        self.start_state = start
        self._goal = goal
        self._blocked_edges = blocked_edges
        self._edge_penalties = edge_penalties

    def is_goal(self, state: str) -> bool:
        return state == self._goal

    def neighbors(self, state: str) -> Iterable[tuple[str, float]]:
        for edge in self._planner._adjacency.get(state, []):
            key = (edge.from_name, edge.to_name)
            if key in self._blocked_edges:
                continue
            penalty = max(0.0, float(self._edge_penalties.get(key, 0.0)))
            yield edge.to_name, float(edge.length) + penalty

    def heuristic(self, state: str) -> float:
        return self._planner._world_distance(
            self._planner.landmarks[state],
            self._planner.landmarks[self._goal],
        )

    @staticmethod
    def tie_breaker(state: str) -> str:
        # The previous spatial heap used ``(f_score, landmark_name)``.
        return state


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
            planner_params.get("trajectory_sample_distance", 0.05)
        )
        self._adjacency = self._build_adjacency()
        self._edge_by_key = {
            (edge.from_name, edge.to_name): edge
            for edge in edges
        }

    def nearest_landmark(self, x: float, y: float) -> tuple[Landmark, float]:
        nearest = min(
            self.landmarks.values(),
            key=lambda landmark: math.hypot(landmark.x - x, landmark.y - y),
        )
        distance = math.hypot(nearest.x - x, nearest.y - y)
        return nearest, distance

    def find_route(
        self,
        start: str,
        goal: str,
        blocked_edges: set[tuple[str, str]] | None = None,
        edge_penalties: dict[tuple[str, str], float] | None = None,
        *,
        should_cancel: Callable[[], bool] | None = None,
        max_expansions: int | None = None,
    ) -> PlannedRoute:
        if start not in self.landmarks:
            raise ValueError(f"Unknown start LM: {start}")
        if goal not in self.landmarks:
            raise ValueError(f"Unknown goal LM: {goal}")
        if start == goal:
            return PlannedRoute(nodes=[start], edges=[], length=0.0)

        blocked_edges = blocked_edges or set()
        edge_penalties = edge_penalties or {}
        problem = _LandmarkRouteProblem(
            self,
            start,
            goal,
            blocked_edges,
            edge_penalties,
        )
        result = AStarSolver[str](max_expansions=max_expansions).solve(
            problem,
            should_cancel=should_cancel,
            cancellation_reason="cancelled",
        )
        if not result.found:
            if result.failure_reason == "cancelled":
                raise ValueError(f"Route search cancelled from {start} to {goal}")
            if result.failure_reason == "expansion_limit":
                raise ValueError(
                    f"Route search expansion limit reached from {start} to {goal}"
                )
            raise ValueError(f"No route found from {start} to {goal}")

        nodes = list(result.path)
        ordered_edges = [
            self._route_edge(src, dst, edge_penalties)
            for src, dst in zip(nodes, nodes[1:])
        ]
        return PlannedRoute(
            nodes=nodes,
            edges=ordered_edges,
            length=sum(float(edge.length) for edge in ordered_edges),
        )

    def _route_edge(
        self,
        source: str,
        target: str,
        edge_penalties: dict[tuple[str, str], float],
    ) -> GraphEdge:
        candidates = [
            edge
            for edge in self._adjacency.get(source, [])
            if edge.to_name == target
        ]
        if not candidates:
            raise RuntimeError(f"route edge disappeared: {source}->{target}")
        penalty = max(
            0.0,
            float(edge_penalties.get((source, target), 0.0)),
        )
        return min(candidates, key=lambda edge: float(edge.length) + penalty)

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
        approx_length = max(edge.length, sample_distance)
        steps = max(2, math.ceil(approx_length / sample_distance))
        samples: list[dict[str, float | str]] = []
        for step in range(steps + 1):
            t = step / steps
            point = self._bezier_point(points, t)
            tangent = self._bezier_derivative(points, t)
            samples.append(
                {
                    "x": point.x,
                    "y": point.y,
                    "yaw": self._motion_yaw(edge, math.atan2(tangent.y, tangent.x)),
                    "edgeId": edge_id,
                    "motionDirection": edge.motion_direction_label(edge.motion_direction_code()),
                }
            )
        return samples

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
        return [
            {
                "x": start.x + dx * (step / steps),
                "y": start.y + dy * (step / steps),
                "yaw": yaw,
                "edgeId": edge_id,
                "motionDirection": edge.motion_direction_label(edge.motion_direction_code()) if edge is not None else "not_specified",
            }
            for step in range(steps + 1)
        ]

    def _bezier_point(self, control_points, t: float) -> WorldPoint:
        p0, p1, p2, p3 = control_points
        omt = 1.0 - t
        x = (
            (omt ** 3) * p0.x
            + 3.0 * (omt ** 2) * t * p1.x
            + 3.0 * omt * (t ** 2) * p2.x
            + (t ** 3) * p3.x
        )
        y = (
            (omt ** 3) * p0.y
            + 3.0 * (omt ** 2) * t * p1.y
            + 3.0 * omt * (t ** 2) * p2.y
            + (t ** 3) * p3.y
        )
        return WorldPoint(x=x, y=y)

    def _bezier_derivative(self, control_points, t: float) -> WorldPoint:
        derivative = cubic_bezier_derivative(
            *(Vector2(point.x, point.y) for point in control_points),
            t,
        )
        return WorldPoint(x=derivative.x, y=derivative.y)

    def _build_adjacency(self) -> dict[str, list[GraphEdge]]:
        adjacency: dict[str, list[GraphEdge]] = {}
        for edge in self.edges:
            adjacency.setdefault(edge.from_name, []).append(edge)
        return adjacency

    def _motion_yaw(self, edge: GraphEdge, tangent_yaw: float) -> float:
        if edge.motion_direction_code() == 1:
            return self._normalize_angle(tangent_yaw + math.pi)
        return tangent_yaw

    def _normalize_angle(self, angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _world_distance(self, first: Landmark, second: Landmark) -> float:
        return math.hypot(second.x - first.x, second.y - first.y)
