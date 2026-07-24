"""Real-robot state synchronization and gRPC route execution."""

from __future__ import annotations

import math
from typing import Any

from fleet_manager.core.constants import (
    EXTERNAL_CONTROL_PAUSE_PREFIX,
    FLEET_CONTROL_OWNER_ID,
    FLEET_CONTROL_OWNER_NAME,
    TERMINAL_ORDER_STATUSES,
)
from fleet_manager.core.models import FleetOrder, FleetRobot


class GrpcRobotRuntimeMixin:
    """Execute shared fleet decisions through robot gRPC tunnels."""

    def _remote_poll_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.25
        try:
            return max(0.05, float(fleet.get("remote_poll_interval_sec", 0.25) or 0.25))
        except (TypeError, ValueError):
            return 0.25

    def _sync_remote_robot(self, robot: FleetRobot, now: float, force: bool = False) -> None:
        if not robot.is_remote() or not robot.base_url:
            return
        if (
            not force
            and robot.remote_last_poll_at is not None
            and now - robot.remote_last_poll_at < self._remote_poll_interval()
        ):
            return
        robot.remote_last_poll_at = now
        try:
            payload = self.remote_adapter.status(robot.base_url)
        except Exception as exc:
            robot.remote_online = False
            robot.remote_error = str(exc)
            robot.status = "OFFLINE"
            robot.last_reason = f"remote status failed: {exc}"
            robot.updated_at = now
            return
        self._apply_remote_status(robot, payload, now)

    def _apply_remote_status(self, robot: FleetRobot, payload: dict[str, Any], now: float) -> None:
        status_payload = self._remote_status_robot(payload)
        robot.remote_status = dict(status_payload)
        robot.remote_online = bool(status_payload.get("connected", True))
        robot.remote_error = ""
        robot.updated_at = now

        state = self._normalize_remote_state(str(status_payload.get("state") or "IDLE"))
        robot.status = state
        message = str(status_payload.get("message") or status_payload.get("reason") or state).strip()
        if message:
            robot.last_reason = message

        target_lm = str(status_payload.get("targetLm") or status_payload.get("target_lm") or "").strip()
        if target_lm or not robot.active_order_id:
            robot.target_lm = target_lm

        current_lm = str(
            status_payload.get("currentLm")
            or status_payload.get("currentLM")
            or status_payload.get("nearestLm")
            or status_payload.get("nearest_lm")
            or status_payload.get("currentStation")
            or status_payload.get("current_station")
            or ""
        ).strip()
        if current_lm in self.landmarks:
            robot.current_lm = current_lm

        pose = self._remote_pose_from_status(status_payload)
        if pose is not None:
            robot.pose = pose
            if current_lm not in self.landmarks:
                nearest_lm = self._nearest_lm_for_pose(pose)
                if nearest_lm:
                    robot.current_lm = nearest_lm
        elif robot.current_lm in self.landmarks and robot.pose is None:
            robot.pose = self._pose_at_landmark(robot.current_lm)

        progress = status_payload.get("routeProgress")
        if progress is not None and robot.trajectory:
            try:
                final_time = float(robot.trajectory[-1].get("t", 0.0) or 0.0)
                robot.route_clock = max(0.0, min(final_time, final_time * float(progress)))
            except (TypeError, ValueError):
                pass
        elif robot.pose is not None and robot.trajectory:
            robot.route_clock = self._nearest_trajectory_clock(robot.trajectory, robot.pose)

    def _remote_control_owner(self, robot: FleetRobot) -> tuple[str, str]:
        status = robot.remote_status if isinstance(robot.remote_status, dict) else {}
        control = status.get("control")
        if not isinstance(control, dict):
            control = {}
        owner_id = str(
            status.get("controlOwner")
            or status.get("control_owner")
            or control.get("ownerId")
            or control.get("owner_id")
            or ""
        ).strip()
        owner_name = str(
            status.get("controlOwnerName")
            or status.get("control_owner_name")
            or control.get("ownerName")
            or control.get("owner_name")
            or owner_id
        ).strip()
        return owner_id, owner_name

    def _pause_order_for_external_control(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        now: float,
        owner_name: str,
    ) -> None:
        reason = f"{EXTERNAL_CONTROL_PAUSE_PREFIX} {owner_name or 'another client'}"
        first_pause = not str(order.error or "").startswith(EXTERNAL_CONTROL_PAUSE_PREFIX)
        order.status = "PAUSED"
        order.error = reason
        order.updated_at = now
        robot.status = "MANUAL"
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_clock = 0.0
        robot.route_started_at = None
        robot.blocked_since = None
        robot.last_reason = reason
        self._clear_remote_route_metadata(robot)
        robot.updated_at = now
        if first_pause:
            self._event("warn", f"order paused: {order.order_id}; {reason}")

    def _resume_order_after_external_control(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        now: float,
    ) -> None:
        order.status = "QUEUED"
        order.error = ""
        order.start_lm = self._nearest_lm_for_robot(robot)
        order.updated_at = now
        robot.active_order_id = ""
        robot.target_lm = ""
        robot.trajectory = []
        robot.plan_nodes = []
        robot.trajectory_dirty = True
        robot.route_clock = 0.0
        robot.route_started_at = None
        robot.blocked_since = None
        robot.last_reason = "external control released; replanning fleet order"
        self._clear_remote_route_metadata(robot)
        robot.updated_at = now
        self._event("info", f"order resumed after external control: {order.order_id}")
        self._dispatch_order(order, force=True)

    def _normalize_remote_state(self, state: str) -> str:
        value = state.strip().upper()
        if value in {"EXECUTING_ROUTE", "FOLLOWING_ROUTE", "RUNNING"}:
            return "MOVING"
        if value in {"WAITING_TRAFFIC", "WAITING_OBSTACLE", "WAITING"}:
            return "WAITING"
        if value in {"ARRIVED", "IDLE", "STOPPED", "LOCALIZING", "ERROR", "MANUAL"}:
            return value
        if value in {"BLOCKED", "PAUSED"}:
            return "BLOCKED"
        return value or "IDLE"

    def _advance_remote_robot_order(self, robot: FleetRobot, now: float) -> None:
        self._sync_remote_robot(robot, now, force=False)
        if not robot.active_order_id:
            robot.last_tick_at = now
            return
        order = self.orders.get(robot.active_order_id)
        if order is None or order.status in TERMINAL_ORDER_STATUSES:
            robot.active_order_id = ""
            robot.last_tick_at = now
            return

        owner_id, owner_name = self._remote_control_owner(robot)
        if owner_id and owner_id != FLEET_CONTROL_OWNER_ID:
            self._pause_order_for_external_control(robot, order, now, owner_name or owner_id)
            robot.last_tick_at = now
            return
        if (
            not owner_id
            and str(order.error or "").startswith(EXTERNAL_CONTROL_PAUSE_PREFIX)
            and robot.status in {"ARRIVED", "IDLE", "STOPPED"}
        ):
            self._resume_order_after_external_control(robot, order, now)
            robot.last_tick_at = now
            return

        target_lm = self._active_order_target(order)
        if robot.status in {"ARRIVED", "IDLE"} and target_lm and robot.current_lm == target_lm:
            order.route_nodes = list(robot.plan_nodes)
            completed = self._advance_or_complete_order(order, robot, now)
            robot.target_lm = ""
            robot.trajectory = []
            robot.plan_nodes = []
            robot.trajectory_dirty = True
            robot.route_clock = 0.0
            robot.route_started_at = None
            robot.blocked_since = None
            robot.last_reason = "arrived"
            if completed:
                robot.status = "ARRIVED"
                self._clear_remote_route_metadata(robot)
                self._event("info", f"order completed: {order.order_id} {robot.name}@{target_lm}")
            else:
                robot.status = "IDLE"
            robot.updated_at = now
        elif (
            robot.status in {"ARRIVED", "IDLE"}
            and target_lm
            and robot.route_chunk_goal_lm
            and robot.current_lm == robot.route_chunk_goal_lm
            and robot.current_lm != target_lm
        ):
            order.start_lm = robot.current_lm
            order.status = "QUEUED"
            order.error = ""
            order.updated_at = now
            robot.target_lm = ""
            robot.trajectory = []
            robot.plan_nodes = []
            robot.trajectory_dirty = True
            robot.route_clock = 0.0
            robot.route_started_at = None
            robot.blocked_since = None
            robot.last_reason = f"chunk arrived at {robot.current_lm}; planning next chunk"
            self._event(
                "info",
                f"route chunk completed: {order.order_id} {robot.name}@{robot.current_lm}; next {target_lm}",
            )
            robot.active_order_id = ""
            if not self._dispatch_order(order, force=True):
                robot.active_order_id = order.order_id
                order.status = "QUEUED"
            robot.updated_at = now
        elif robot.status in {"ERROR", "BLOCKED", "OFFLINE"}:
            order.status = "PAUSED" if robot.status != "OFFLINE" else "QUEUED"
            order.error = robot.last_reason or robot.remote_error or robot.status.lower()
            order.updated_at = now
        elif robot.status in {"MOVING", "WAITING"} and robot.trajectory:
            blocked_reason = self._blocked_ahead(robot, robot.route_clock)
            if blocked_reason and self._should_replan_for_blocked_reason(blocked_reason):
                self._maybe_replan_remote_robot_order(robot, order, now, blocked_reason)
            self._update_active_order_from_robot(robot)
        else:
            self._update_active_order_from_robot(robot)
        robot.last_tick_at = now

    def _execute_remote_plan(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
        result: dict[str, Any],
    ) -> dict[str, Any]:
        route = self._remote_route_payload(robot, order, plan)
        payload = {
            "route": route,
            "order": order.to_dict(),
            "ownerId": FLEET_CONTROL_OWNER_ID,
            "fleet": {
                "manager": "fleet_manager",
                "routeProtocol": "lm_route",
                "planNote": self._plan_note(result),
                "debug": result.get("debug", {}),
            },
        }
        self._ensure_remote_control(robot, "execute route")
        response = self.remote_adapter.execute_route(robot.base_url, payload)
        robot.remote_online = True
        robot.remote_error = ""
        status = response.get("status")
        if isinstance(status, dict):
            self._apply_remote_status(robot, status, self._now())
        return route

    def _remote_route_payload(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        plan: dict[str, Any],
    ) -> dict[str, Any]:
        full_nodes = [str(item) for item in plan.get("nodes", []) if str(item)]
        start_lm = str(plan.get("startLm") or order.start_lm or (full_nodes[0] if full_nodes else ""))
        final_goal_lm = str(plan.get("goalLm") or order.target_lm or (full_nodes[-1] if full_nodes else ""))
        if not full_nodes and start_lm and final_goal_lm:
            full_nodes = [start_lm, final_goal_lm] if start_lm != final_goal_lm else [start_lm]
        chunk_nodes = self._remote_route_chunk_nodes(full_nodes)
        chunk_start_lm = chunk_nodes[0] if chunk_nodes else start_lm
        chunk_goal_lm = chunk_nodes[-1] if chunk_nodes else final_goal_lm
        chunk_length = self._lm_path_length(chunk_nodes)
        full_length = float(plan.get("length", 0.0) or 0.0)
        revision = self._next_route_revision()
        chunk_index = self._next_remote_chunk_index(robot, order, chunk_start_lm)
        is_final = bool(chunk_nodes and full_nodes and chunk_nodes[-1] == full_nodes[-1])
        dispatch_epoch = self._now() + self._remote_dispatch_lead_time()
        timed_segments = self._timed_segments_from_trajectory(plan.get("trajectory", []))
        if not timed_segments:
            timed_segments = [
                dict(item)
                for item in plan.get("timedSegments", [])
                if isinstance(item, dict)
            ]
        return {
            "routeId": f"{order.order_id}:{order.step_index}",
            "protocol": "lm_route",
            "protocolVersion": 2,
            "revision": revision,
            "orderId": order.order_id,
            "startLm": chunk_start_lm,
            "goalLm": chunk_goal_lm,
            "finalGoalLm": final_goal_lm,
            "nodes": chunk_nodes,
            "fullNodes": full_nodes,
            "length": chunk_length if chunk_length > 0.0 else full_length,
            "fullLength": full_length,
            "replaceMode": "immediate",
            "dispatchEpochSec": dispatch_epoch,
            "timedSegments": timed_segments,
            "chunk": {
                "index": chunk_index,
                "stepIndex": order.step_index,
                "offset": 0,
                "startLm": chunk_start_lm,
                "goalLm": chunk_goal_lm,
                "finalGoalLm": final_goal_lm,
                "nodes": chunk_nodes,
                "fullNodes": full_nodes,
                "isFinal": is_final,
            },
        }

    def _remote_dispatch_lead_time(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.75
        try:
            return max(0.25, float(fleet.get("remote_dispatch_lead_sec", 0.75) or 0.75))
        except (TypeError, ValueError):
            return 0.75

    def _timed_segments_from_trajectory(self, raw_trajectory: Any) -> list[dict[str, Any]]:
        if not isinstance(raw_trajectory, list):
            return []
        trajectory = [item for item in raw_trajectory if isinstance(item, dict)]
        segments: list[dict[str, Any]] = []
        for index in range(len(trajectory) - 1):
            start = trajectory[index]
            end = trajectory[index + 1]
            edge_id = str(end.get("edgeId") or start.get("edgeId") or "")
            start_time = float(start.get("t", 0.0) or 0.0)
            end_time = float(end.get("t", start_time) or start_time)
            if edge_id.startswith("WAIT@ROTATE:"):
                node = str(end.get("lm") or start.get("lm") or "").strip()
                segments.append(
                    {
                        "kind": "rotate",
                        "node": node,
                        "fromYaw": float(start.get("yaw", 0.0) or 0.0),
                        "toYaw": float(end.get("yaw", 0.0) or 0.0),
                        "notBeforeSec": start_time,
                        "plannedArrivalSec": end_time,
                    }
                )
                continue
            parsed = self._parse_edge_id(edge_id)
            if parsed is None or edge_id.startswith("WAIT@") or parsed[0] == parsed[1]:
                continue
            src, dst = parsed
            motion_direction = str(
                end.get("motionDirection")
                or start.get("motionDirection")
                or "not_specified"
            )
            if (
                segments
                and segments[-1].get("from") == src
                and segments[-1].get("to") == dst
                and segments[-1].get("motionDirection") == motion_direction
                and abs(float(segments[-1].get("plannedArrivalSec", 0.0)) - start_time) < 1e-6
            ):
                segments[-1]["plannedArrivalSec"] = end_time
                continue
            segments.append(
                {
                    "kind": "move",
                    "from": src,
                    "to": dst,
                    "motionDirection": motion_direction,
                    "notBeforeSec": start_time,
                    "plannedArrivalSec": end_time,
                }
            )
        return segments

    def _remote_route_chunk_nodes(self, nodes: list[str]) -> list[str]:
        if len(nodes) <= 2:
            return list(nodes)
        limit = self._remote_route_chunk_lms()
        if limit <= 0 or limit >= len(nodes):
            return list(nodes)
        return list(nodes[:max(2, limit)])

    def _remote_route_chunk_lms(self) -> int:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 5
        try:
            return max(0, int(fleet.get("remote_route_chunk_lms", 5) or 0))
        except (TypeError, ValueError):
            return 5

    def _next_remote_chunk_index(self, robot: FleetRobot, order: FleetOrder, chunk_start_lm: str) -> int:
        if robot.route_final_lm == order.target_lm:
            if robot.route_chunk_goal_lm and robot.current_lm == robot.route_chunk_goal_lm:
                return max(0, robot.route_chunk_index + 1)
            if robot.active_order_id != order.order_id:
                return 0
            if chunk_start_lm == robot.current_lm:
                return max(0, robot.route_chunk_index)
        return 0

    def _apply_remote_route_metadata(self, robot: FleetRobot, route: dict[str, Any], now: float) -> None:
        chunk = route.get("chunk")
        if not isinstance(chunk, dict):
            chunk = {}
        robot.route_revision = int(route.get("revision", 0) or 0)
        robot.route_chunk_index = int(chunk.get("index", 0) or 0)
        robot.route_chunk_goal_lm = str(chunk.get("goalLm") or route.get("goalLm") or "").strip()
        robot.route_final_lm = str(
            chunk.get("finalGoalLm")
            or route.get("finalGoalLm")
            or route.get("goalLm")
            or ""
        ).strip()
        self._update_route_preview(
            robot,
            robot.current_lm,
            robot.route_final_lm,
            committed_trajectory=robot.trajectory,
            committed_nodes=route.get("nodes"),
        )
        if robot.route_chunk_goal_lm:
            robot.target_lm = robot.route_chunk_goal_lm
        robot.updated_at = now

    def _lm_path_length(self, nodes: list[str]) -> float:
        if len(nodes) < 2:
            return 0.0
        length = 0.0
        edge_lengths = {
            (edge.from_name, edge.to_name): float(edge.length)
            for edge in self.edges
        }
        for start_lm, goal_lm in zip(nodes, nodes[1:]):
            if start_lm == goal_lm:
                continue
            edge_length = edge_lengths.get((start_lm, goal_lm))
            if edge_length is None:
                return 0.0
            length += edge_length
        return length

    def _cancel_remote_route(self, robot: FleetRobot, reason: str) -> bool:
        if not robot.is_remote() or not robot.base_url:
            return True
        try:
            self._ensure_remote_control(robot, "cancel route")
            self.remote_adapter.cancel_route(robot.base_url, owner_id=FLEET_CONTROL_OWNER_ID)
            robot.remote_online = True
            robot.remote_error = ""
        except Exception as exc:
            robot.remote_online = self._is_remote_control_conflict(exc)
            robot.remote_error = str(exc)
            self._event("warn", f"{robot.name} remote cancel failed: {exc}")
            return False
        robot.last_reason = reason
        return True

    def _stop_remote_robot(self, robot: FleetRobot) -> None:
        if not robot.is_remote() or not robot.base_url:
            return
        try:
            self._ensure_remote_control(robot, "stop")
            self.remote_adapter.stop(robot.base_url, owner_id=FLEET_CONTROL_OWNER_ID)
            robot.remote_online = True
            robot.remote_error = ""
        except Exception as exc:
            robot.remote_online = self._is_remote_control_conflict(exc)
            robot.remote_error = str(exc)
            self._event("warn", f"{robot.name} remote stop failed: {exc}")

    def _ensure_remote_control(self, robot: FleetRobot, action: str) -> None:
        if not robot.is_remote() or not robot.base_url:
            return
        try:
            self.remote_adapter.acquire_control(
                robot.base_url,
                owner_id=FLEET_CONTROL_OWNER_ID,
                owner_name=FLEET_CONTROL_OWNER_NAME,
                # A Fleet Manager background tick must never steal a robot
                # from an operator who explicitly took manual control.
                force=False,
                lease_ms=0,
            )
            robot.remote_online = True
            robot.remote_error = ""
        except Exception as exc:
            robot.remote_online = self._is_remote_control_conflict(exc)
            robot.remote_error = str(exc)
            raise ValueError(f"remote control acquire failed before {action}: {exc}") from exc

    @staticmethod
    def _is_remote_control_conflict(error: Exception | str) -> bool:
        text = str(error or "").strip().lower()
        return "control is owned by" in text or "control owner" in text

    def _nearest_trajectory_clock(self, trajectory: list[dict[str, Any]], pose: dict[str, float]) -> float:
        best_time = float(trajectory[0].get("t", 0.0) or 0.0) if trajectory else 0.0
        best_dist = float("inf")
        px = float(pose.get("x", 0.0) or 0.0)
        py = float(pose.get("y", 0.0) or 0.0)
        for sample in trajectory:
            dist = math.hypot(
                px - float(sample.get("x", 0.0) or 0.0),
                py - float(sample.get("y", 0.0) or 0.0),
            )
            if dist < best_dist:
                best_dist = dist
                best_time = float(sample.get("t", 0.0) or 0.0)
        return best_time
