from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from .route_core import LmRoutePlanner, PlannedRoute, WarehouseMapLoader, load_route_params

from .runtime import PlannedRobotRoute, Pose2D, RoutePoint


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RobotTrajectoryPlanner:
    def __init__(
        self,
        map_dir: Path,
        params_path: Path,
    ) -> None:
        self.map_dir = Path(map_dir).resolve()
        self.loaded_map = WarehouseMapLoader(self.map_dir).load()
        self.params_path = params_path
        self._params_mtime_ns: int | None = None
        self.params = load_route_params(params_path, create=True)
        self.planner = LmRoutePlanner(
            self.loaded_map.landmarks,
            self.loaded_map.edges,
            params=self.params,
        )
        self._refresh_params_mtime()

    @property
    def map_id(self) -> str:
        return self.loaded_map.map_metadata.map_name

    def update_params(self, params: dict[str, Any]) -> None:
        self.params = params
        self.planner = LmRoutePlanner(
            self.loaded_map.landmarks,
            self.loaded_map.edges,
            params=self.params,
        )

    def reload_map(self, map_dir: Path) -> None:
        self.map_dir = Path(map_dir).resolve()
        self.loaded_map = WarehouseMapLoader(self.map_dir).load()
        self.planner = LmRoutePlanner(
            self.loaded_map.landmarks,
            self.loaded_map.edges,
            params=self.params,
        )

    def site_payload(self, robot_id: str) -> dict[str, Any]:
        landmarks = [self.loaded_map.landmarks[name] for name in sorted(self.loaded_map.landmarks)]
        return {
            "title": "Warehouse Robot Control",
            "robotId": robot_id,
            "mapName": self.loaded_map.map_metadata.map_name,
            "map": self.loaded_map.map_metadata.to_dict(),
            "lms": [item.to_dict() for item in landmarks],
            "edges": [edge.to_dict() for edge in self.loaded_map.edges],
            "params": self.params,
            "defaultGoal": landmarks[-1].name if landmarks else "",
        }

    def current_params(self) -> dict[str, Any]:
        return self.params

    def reload_params_from_disk(self) -> dict[str, Any]:
        try:
            mtime_ns = self.params_path.stat().st_mtime_ns
        except FileNotFoundError:
            mtime_ns = None
        if mtime_ns is not None and mtime_ns == self._params_mtime_ns:
            return self.params
        self.params = load_route_params(self.params_path, create=True)
        self.planner = LmRoutePlanner(
            self.loaded_map.landmarks,
            self.loaded_map.edges,
            params=self.params,
        )
        self._params_mtime_ns = mtime_ns
        return self.params

    def _refresh_params_mtime(self) -> None:
        try:
            self._params_mtime_ns = self.params_path.stat().st_mtime_ns
        except FileNotFoundError:
            self._params_mtime_ns = None

    def plan_from_pose(self, pose: Pose2D, goal_lm: str, start_lm: str | None = None) -> PlannedRobotRoute:
        self.reload_params_from_disk()
        goal = str(goal_lm).strip()
        if goal not in self.loaded_map.landmarks:
            raise ValueError(f"unknown goal LM: {goal}")

        planner_params = self.params.get("planner", {})
        if not isinstance(planner_params, dict):
            planner_params = {}
        sample_distance = max(0.02, float(planner_params.get("trajectory_sample_distance", 0.05) or 0.05))
        tolerance = max(0.01, float(planner_params.get("nearest_lm_tolerance", 0.05) or 0.05))
        on_route_tolerance = max(0.02, float(planner_params.get("on_route_tolerance", 0.12) or 0.12))

        nearest_name = str(start_lm or "").strip()
        connector_length = 0.0
        connector_points: list[RoutePoint] = []
        nodes: list[str] = []

        if nearest_name:
            if nearest_name not in self.loaded_map.landmarks:
                raise ValueError(f"unknown start LM: {nearest_name}")
            nearest = self.loaded_map.landmarks[nearest_name]
            distance = math.hypot(nearest.x - pose.x, nearest.y - pose.y)
        else:
            nearest, distance = self.planner.nearest_landmark(pose.x, pose.y)
            nearest_name = nearest.name

        if not start_lm and distance > tolerance:
            route = self._plan_from_current_edge(
                pose=pose,
                goal_lm=goal,
                sample_distance=sample_distance,
                on_route_tolerance=on_route_tolerance,
            )
            if route is not None:
                return route

        if distance > tolerance:
            connector_points = self._sample_line(
                pose,
                Pose2D(x=nearest.x, y=nearest.y, yaw=pose.yaw),
                sample_distance,
                edge_id=f"CURRENT_POSE->{nearest_name}",
            )
            connector_length = distance
            nodes.extend(["CURRENT_POSE", nearest_name])
        else:
            nodes.append(nearest_name)

        route = self.planner.find_route(nearest_name, goal)
        route_points = self._route_points_from_graph_route(route, sample_distance)
        if connector_points and route_points:
            route_points = route_points[1:]
        trajectory = connector_points + route_points

        if len(nodes) == 1:
            nodes = list(route.nodes)
        elif route.nodes:
            nodes.extend(route.nodes[1:])

        if not trajectory:
            trajectory = [RoutePoint(x=pose.x, y=pose.y, yaw=pose.yaw, edge_id=f"{goal}->{goal}")]

        return PlannedRobotRoute.create(
            start_lm=nearest_name,
            goal_lm=goal,
            nodes=nodes,
            trajectory=trajectory,
            length=connector_length + route.length,
        )

    def plan_from_lm_route(self, pose: Pose2D, route_payload: dict[str, Any]) -> PlannedRobotRoute:
        self.reload_params_from_disk()
        nodes = self._lm_route_nodes(route_payload)
        if not nodes:
            goal_lm = str(route_payload.get("goalLm") or route_payload.get("goal_lm") or route_payload.get("targetLm") or "").strip()
            start_lm = str(route_payload.get("startLm") or route_payload.get("start_lm") or "").strip() or None
            route = self.plan_from_pose(pose=pose, goal_lm=goal_lm, start_lm=start_lm)
            return self._apply_route_metadata(route, route_payload)

        start_lm = str(route_payload.get("startLm") or route_payload.get("start_lm") or "").strip()
        goal_lm = str(route_payload.get("goalLm") or route_payload.get("goal_lm") or route_payload.get("targetLm") or "").strip()
        if start_lm and nodes[0] != start_lm:
            nodes.insert(0, start_lm)
        if goal_lm and nodes[-1] != goal_lm:
            nodes.append(goal_lm)
        goal_lm = goal_lm or nodes[-1]

        for node in nodes:
            if node not in self.loaded_map.landmarks:
                raise ValueError(f"unknown route LM: {node}")

        planner_params = self.params.get("planner", {})
        if not isinstance(planner_params, dict):
            planner_params = {}
        sample_distance = max(0.02, float(planner_params.get("trajectory_sample_distance", 0.05) or 0.05))
        tolerance = max(0.01, float(planner_params.get("nearest_lm_tolerance", 0.05) or 0.05))

        route = self._route_from_nodes(nodes)
        route_points = self._route_points_from_graph_route(route, sample_distance)
        connector_points: list[RoutePoint] = []
        connector_length = 0.0
        start_landmark = self.loaded_map.landmarks[nodes[0]]
        distance_to_start = math.hypot(start_landmark.x - pose.x, start_landmark.y - pose.y)
        if distance_to_start > tolerance:
            connector_points = self._connector_to_route_start(
                pose=pose,
                start_lm=nodes[0],
                sample_distance=sample_distance,
            )
            connector_length = self._path_length(connector_points)
            if connector_points and route_points:
                route_points = route_points[1:]

        trajectory = connector_points + route_points
        if not trajectory:
            trajectory = [
                RoutePoint(
                    x=start_landmark.x,
                    y=start_landmark.y,
                    yaw=pose.yaw,
                    edge_id=f"{nodes[0]}->{nodes[0]}",
                )
            ]

        planned = PlannedRobotRoute.create(
            start_lm=nodes[0],
            goal_lm=goal_lm,
            nodes=nodes,
            trajectory=trajectory,
            length=connector_length + route.length,
        )
        return self._apply_route_metadata(planned, route_payload)

    def _plan_from_current_edge(
        self,
        pose: Pose2D,
        goal_lm: str,
        sample_distance: float,
        on_route_tolerance: float,
    ) -> PlannedRobotRoute | None:
        best: dict[str, Any] | None = None
        nearest_name, _ = self.planner.nearest_landmark(pose.x, pose.y)

        for edge in self.loaded_map.edges:
            sampled = self._sample_edge(edge, sample_distance)
            if len(sampled) < 2:
                continue

            projection = self._project_pose_to_samples(pose, sampled)
            if projection is None or float(projection["distance"]) > on_route_tolerance:
                continue

            if edge.to_name == goal_lm:
                route_nodes = [edge.to_name]
                route_length = 0.0
                route_points: list[RoutePoint] = []
            else:
                try:
                    graph_route = self.planner.find_route(edge.to_name, goal_lm)
                except ValueError:
                    continue
                route_nodes = list(graph_route.nodes)
                route_length = float(graph_route.length)
                route_points = self._route_points_from_graph_route(graph_route, sample_distance)

            remaining_path = self._remaining_edge_path(sampled, projection)
            remaining_length = self._path_length(remaining_path)
            total_length = remaining_length + route_length
            candidate = {
                "edge_id": f"{edge.from_name}->{edge.to_name}",
                "edge_to": edge.to_name,
                "nearest_name": nearest_name.name,
                "remaining_path": remaining_path,
                "remaining_length": remaining_length,
                "route_nodes": route_nodes,
                "route_points": route_points,
                "total_length": total_length,
            }

            if best is None or float(candidate["total_length"]) < float(best["total_length"]):
                best = candidate

        if best is None:
            return None

        trajectory = list(best["remaining_path"])
        route_points = list(best["route_points"])
        if trajectory and route_points:
            route_points = route_points[1:]
        trajectory.extend(route_points)
        if not trajectory:
            return None

        edge_to = str(best["edge_to"])
        edge_id = str(best["edge_id"])
        nodes = [f"CURRENT_EDGE {edge_id}", edge_to]
        route_nodes = [str(item) for item in best["route_nodes"] if str(item)]
        if route_nodes:
            if route_nodes[0] == edge_to:
                nodes.extend(route_nodes[1:])
            else:
                nodes.extend(route_nodes)

        return PlannedRobotRoute.create(
            start_lm=str(best["nearest_name"]),
            goal_lm=goal_lm,
            nodes=nodes,
            trajectory=trajectory,
            length=float(best["total_length"]),
        )

    def _lm_route_nodes(self, route_payload: dict[str, Any]) -> list[str]:
        raw_nodes = route_payload.get("nodes")
        if raw_nodes is None:
            raw_nodes = route_payload.get("routeNodes") or route_payload.get("route_nodes") or []
        if not isinstance(raw_nodes, list):
            return []
        nodes: list[str] = []
        for item in raw_nodes:
            node = str(item).strip()
            if not node or node.startswith("CURRENT_"):
                continue
            if nodes and nodes[-1] == node:
                continue
            nodes.append(node)
        return nodes

    def _route_from_nodes(self, nodes: list[str]) -> PlannedRoute:
        if not nodes:
            raise ValueError("LM route nodes are required")
        if len(nodes) == 1:
            return PlannedRoute(nodes=list(nodes), edges=[], length=0.0)
        edges = []
        total_length = 0.0
        for start_lm, goal_lm in zip(nodes, nodes[1:]):
            if start_lm == goal_lm:
                continue
            edge = self.planner.get_edge(start_lm, goal_lm)
            if edge is None:
                raise ValueError(f"LM route edge is missing: {start_lm}->{goal_lm}")
            edges.append(edge)
            total_length += float(edge.length)
        return PlannedRoute(nodes=list(nodes), edges=edges, length=total_length)

    def _connector_to_route_start(
        self,
        *,
        pose: Pose2D,
        start_lm: str,
        sample_distance: float,
    ) -> list[RoutePoint]:
        current_edge_route = self._plan_from_current_edge(
            pose=pose,
            goal_lm=start_lm,
            sample_distance=sample_distance,
            on_route_tolerance=max(sample_distance * 2.0, 0.12),
        )
        if current_edge_route is not None and current_edge_route.trajectory:
            return current_edge_route.trajectory

        landmark = self.loaded_map.landmarks[start_lm]
        return self._sample_line(
            pose,
            Pose2D(x=landmark.x, y=landmark.y, yaw=pose.yaw),
            sample_distance,
            edge_id=f"CURRENT_POSE->{start_lm}",
        )

    def _apply_route_metadata(
        self,
        route: PlannedRobotRoute,
        route_payload: dict[str, Any],
    ) -> PlannedRobotRoute:
        route_id = str(route_payload.get("routeId") or route_payload.get("route_id") or "").strip()
        if route_id:
            route.route_id = route_id
        route.protocol = str(route_payload.get("protocol") or route_payload.get("routeProtocol") or "lm_route")
        route.order_id = str(route_payload.get("orderId") or route_payload.get("order_id") or "")
        try:
            route.revision = int(route_payload.get("revision", 0) or 0)
        except (TypeError, ValueError):
            route.revision = 0
        chunk = route_payload.get("chunk")
        if not isinstance(chunk, dict):
            chunk = {}
        final_goal_lm = str(
            route_payload.get("finalGoalLm")
            or route_payload.get("final_goal_lm")
            or chunk.get("finalGoalLm")
            or route.goal_lm
        ).strip()
        route.final_goal_lm = final_goal_lm or route.goal_lm
        raw_full_nodes = route_payload.get("fullNodes") or route_payload.get("full_nodes") or chunk.get("fullNodes")
        if isinstance(raw_full_nodes, list):
            route.full_nodes = [str(item) for item in raw_full_nodes if str(item)]
        elif not route.full_nodes:
            route.full_nodes = list(route.nodes)
        try:
            route.chunk_index = int(chunk.get("index", 0) or 0)
        except (TypeError, ValueError):
            route.chunk_index = 0
        try:
            route.chunk_offset = int(chunk.get("offset", 0) or 0)
        except (TypeError, ValueError):
            route.chunk_offset = 0
        route.chunk_is_final = bool(chunk.get("isFinal", route.goal_lm == route.final_goal_lm))
        route.replace_mode = str(route_payload.get("replaceMode") or route_payload.get("replace_mode") or "immediate")
        return route

    def _sample_line(
        self,
        start: Pose2D,
        goal: Pose2D,
        sample_distance: float,
        edge_id: str,
    ) -> list[RoutePoint]:
        dx = goal.x - start.x
        dy = goal.y - start.y
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            return [RoutePoint(x=start.x, y=start.y, yaw=start.yaw, edge_id=edge_id)]
        steps = max(1, math.ceil(length / sample_distance))
        yaw = math.atan2(dy, dx)
        return [
            RoutePoint(
                x=start.x + (dx * (step / steps)),
                y=start.y + (dy * (step / steps)),
                yaw=yaw,
                edge_id=edge_id,
                motion_direction="forward",
            )
            for step in range(steps + 1)
        ]

    def _route_points_from_graph_route(
        self,
        route,
        sample_distance: float,
    ) -> list[RoutePoint]:
        return [
            RoutePoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point.get("yaw", 0.0) or 0.0),
                edge_id=str(point.get("edgeId") or ""),
                motion_direction=str(point.get("motionDirection") or "forward"),
            )
            for point in self.planner.sample_route(route, sample_distance=sample_distance)
        ]

    def _sample_edge(self, edge, sample_distance: float) -> list[RoutePoint]:
        return [
            RoutePoint(
                x=float(point["x"]),
                y=float(point["y"]),
                yaw=float(point.get("yaw", 0.0) or 0.0),
                edge_id=str(point.get("edgeId") or ""),
                motion_direction=str(point.get("motionDirection") or "forward"),
            )
            for point in self.planner._sample_edge(edge, sample_distance)
        ]

    def _project_pose_to_samples(
        self,
        pose: Pose2D,
        samples: list[RoutePoint],
    ) -> dict[str, Any] | None:
        best: dict[str, Any] | None = None
        for index in range(0, len(samples) - 1):
            first = samples[index]
            second = samples[index + 1]
            dx = second.x - first.x
            dy = second.y - first.y
            length_sq = (dx * dx) + (dy * dy)
            if length_sq <= 1e-9:
                continue
            ratio = clamp((((pose.x - first.x) * dx) + ((pose.y - first.y) * dy)) / length_sq, 0.0, 1.0)
            projected_x = first.x + (dx * ratio)
            projected_y = first.y + (dy * ratio)
            projected_yaw = normalize_angle(first.yaw + (normalize_angle(second.yaw - first.yaw) * ratio))
            distance = math.hypot(pose.x - projected_x, pose.y - projected_y)
            if best is None or distance < float(best["distance"]):
                best = {
                    "x": projected_x,
                    "y": projected_y,
                    "yaw": projected_yaw,
                    "distance": distance,
                    "segment_index": index,
                    "edge_id": first.edge_id,
                    "motion_direction": first.motion_direction,
                }
        return best

    def _remaining_edge_path(
        self,
        samples: list[RoutePoint],
        projection: dict[str, Any],
    ) -> list[RoutePoint]:
        remaining = [
            RoutePoint(
                x=float(projection["x"]),
                y=float(projection["y"]),
                yaw=float(projection["yaw"]),
                edge_id=str(projection["edge_id"]),
                motion_direction=str(projection.get("motion_direction") or "forward"),
            )
        ]
        segment_index = int(projection["segment_index"])
        for index in range(segment_index + 1, len(samples)):
            remaining.append(samples[index])
        return remaining

    def _path_length(self, points: list[RoutePoint]) -> float:
        length = 0.0
        for index in range(1, len(points)):
            previous = points[index - 1]
            current = points[index]
            length += math.hypot(current.x - previous.x, current.y - previous.y)
        return length
