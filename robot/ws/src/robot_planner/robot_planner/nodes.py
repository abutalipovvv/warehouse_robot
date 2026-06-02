from __future__ import annotations

import json

from rclpy.node import Node

from robot_msgs.srv import PlanRoute

from .route_planner import RobotTrajectoryPlanner
from .runtime import Pose2D


class RoutePlannerNode(Node):
    def __init__(
        self,
        route_planner: RobotTrajectoryPlanner,
        service_name: str,
    ) -> None:
        super().__init__("route_planner")
        self.route_planner = route_planner
        self.create_service(PlanRoute, service_name, self._handle_plan_route)

    def _handle_plan_route(self, request, response):
        try:
            if not bool(request.use_start_pose):
                raise ValueError("start pose is required")
            pose = Pose2D(
                x=float(request.start_x),
                y=float(request.start_y),
                yaw=float(request.start_yaw),
            )
            start_lm = str(request.start_lm or "").strip() or None
            route = self.route_planner.plan_from_pose(
                pose=pose,
                goal_lm=str(request.goal_lm or "").strip(),
                start_lm=start_lm,
            )
            response.ok = True
            response.error = ""
            response.route_json = json.dumps(route.to_dict(), ensure_ascii=False)
        except Exception as exc:  # pragma: no cover - ROS service boundary
            response.ok = False
            response.error = str(exc)
            response.route_json = ""
        return response
