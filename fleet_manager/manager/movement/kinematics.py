"""Pose interpolation and graph/trajectory kinematic helpers."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.robot.model import FleetRobot


class FleetMotionKinematicsMixin:
    """Interpolate poses and relate trajectory progress to graph landmarks."""

    def _interpolate_pose(
        self,
        start: dict[str, float],
        end: dict[str, float],
        ratio: float,
    ) -> dict[str, float]:
        return {
            "x": float(start.get("x", 0.0) or 0.0)
            + ((float(end.get("x", 0.0) or 0.0) - float(start.get("x", 0.0) or 0.0)) * ratio),
            "y": float(start.get("y", 0.0) or 0.0)
            + ((float(end.get("y", 0.0) or 0.0) - float(start.get("y", 0.0) or 0.0)) * ratio),
            "yaw": self._interpolate_angle(
                float(start.get("yaw", 0.0) or 0.0),
                float(end.get("yaw", 0.0) or 0.0),
                ratio,
            ),
        }

    def _runtime_motion_step(self) -> float:
        return max(0.02, min(0.05, self._continuous_collision_step() / 2.0))


    def _safe_replan_start_lm(self, robot: FleetRobot) -> str:
        if robot.pose is None:
            return robot.current_lm if robot.current_lm in self.landmarks else ""
        # ``current_lm`` is updated from tagged trajectory samples and is the
        # overwhelmingly common replan boundary. Check it before scanning the
        # complete map; the configured tolerance is much smaller than graph
        # spacing, so at most one landmark can satisfy this fast path.
        current = self.landmarks.get(robot.current_lm)
        if current is not None and math.hypot(
            current.x - float(robot.pose.get("x", 0.0) or 0.0),
            current.y - float(robot.pose.get("y", 0.0) or 0.0),
        ) <= self._runtime_replan_lm_tolerance():
            return robot.current_lm
        nearest_lm = self._nearest_lm_for_robot(robot)
        landmark = self.landmarks.get(nearest_lm)
        if landmark is None:
            return ""
        distance = math.hypot(
            landmark.x - float(robot.pose.get("x", 0.0) or 0.0),
            landmark.y - float(robot.pose.get("y", 0.0) or 0.0),
        )
        if distance > self._runtime_replan_lm_tolerance():
            return ""
        return nearest_lm

    def _pose_is_at_lm(self, pose: dict[str, Any], lm_name: str) -> bool:
        landmark = self.landmarks.get(lm_name)
        if landmark is None:
            return False
        return math.hypot(
            landmark.x - float(pose.get("x", 0.0) or 0.0),
            landmark.y - float(pose.get("y", 0.0) or 0.0),
        ) <= self._runtime_replan_lm_tolerance()

    def _update_current_lm_from_trajectory(self, robot: FleetRobot) -> None:
        index = self._trajectory_sample_index_at_or_before(
            robot.trajectory,
            robot.route_clock + 0.000001,
        )
        while index >= 0:
            lm_name = str(robot.trajectory[index].get("lm") or "").strip()
            if lm_name in self.landmarks:
                robot.current_lm = lm_name
                return
            index -= 1

    def _planned_wait_lm_at_trajectory(
        self,
        trajectory: list[dict[str, Any]],
        elapsed: float,
    ) -> str:
        if len(trajectory) < 2:
            return ""
        index = self._trajectory_segment_index(trajectory, elapsed)
        start = trajectory[index]
        end = trajectory[index + 1]
        start_time = float(start.get("t", 0.0) or 0.0)
        end_time = float(end.get("t", start_time) or start_time)
        if end_time <= start_time or not (start_time <= elapsed < end_time):
            return ""
        edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
        if edge_id.startswith("WAIT@ROTATE:"):
            return ""
        if edge_id.startswith("WAIT@"):
            return self._lm_from_wait_segment(start, end)
        if "->" not in edge_id:
            return ""
        source, target = (value.strip() for value in edge_id.split("->", 1))
        if source == target and source in self.landmarks:
            return source
        return ""

    def _runtime_replan_lm_tolerance(self) -> float:
        return self.settings.fleet.number(
            "runtime_replan_lm_tolerance_m",
            0.10,
            minimum=0.03,
            default_if_falsy=True,
        )
