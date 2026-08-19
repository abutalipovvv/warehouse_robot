"""Pause and restore Nav2 lifecycle nodes around SLAM and odometry reset."""

from __future__ import annotations

from time import monotonic, sleep
from typing import Any

NAV2_LIFECYCLE_MANAGER_SERVICES: tuple[tuple[str, str], ...] = (
    ("/lifecycle_manager_navigation/manage_nodes", "navigation"),
    ("/lifecycle_manager_localization/manage_nodes", "localization"),
    ("/lifecycle_manager_map_server/manage_nodes", "map_server"),
)

NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES: tuple[tuple[str, str], ...] = (
    ("/lifecycle_manager_map_server/manage_nodes", "map_server"),
    ("/lifecycle_manager_localization/manage_nodes", "localization"),
    ("/lifecycle_manager_navigation/manage_nodes", "navigation"),
)

NAV2_LIFECYCLE_NODES: tuple[str, ...] = (
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "smoother_server",
    "bt_navigator",
)

NAV2_LIFECYCLE_RESUME_NODES: tuple[str, ...] = (
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "smoother_server",
    "bt_navigator",
)

class RosRuntimeNav2LifecycleMixin:
    """Pause and restore Nav2 lifecycle nodes around SLAM and odometry reset."""

    def _reset_odom_for_slam(self) -> None:
        if not self.reset_odom_service_name:
            print("[robot_api_server] Odom reset before 2D SLAM is disabled.", flush=True)
            return
        if self._reset_odom_client is None:
            raise ValueError(f"{self.reset_odom_service_name} client is not configured")

        reset_started_at = monotonic()
        with self._lock:
            self._latest_odom_pose = None
            self._latest_odom_at = None
            self._latest_slam_pose = None
            self._latest_slam_pose_at = None
            self._slam_trail = []

        request = self._reset_odom_client.srv_type.Request()
        self._call_service(self._reset_odom_client, request, "reset odom", timeout_sec=3.0)
        self._wait_for_zero_odom_after_reset(reset_started_at)
        print(f"[robot_api_server] Odom reset for 2D SLAM via {self.reset_odom_service_name}", flush=True)

    def _wait_for_zero_odom_after_reset(self, reset_started_at: float) -> None:
        deadline = monotonic() + 2.5
        last_pose: dict[str, float] | None = None
        while monotonic() < deadline:
            with self._lock:
                pose = dict(self._latest_odom_pose) if isinstance(self._latest_odom_pose, dict) else None
                received_at = self._latest_odom_at
            if pose is not None:
                last_pose = pose
            if pose is not None and received_at is not None and received_at >= reset_started_at:
                x = float(pose.get("x", 0.0) or 0.0)
                y = float(pose.get("y", 0.0) or 0.0)
                yaw = self._normalize_angle(float(pose.get("yaw", 0.0) or 0.0))
                if abs(x) <= 0.08 and abs(y) <= 0.08 and abs(yaw) <= 0.15:
                    return
            sleep(0.02)
        if last_pose is None:
            raise ValueError(f"no fresh odom received on {self.odom_topic} after reset")
        raise ValueError(
            "odom did not settle near zero after reset: "
            f"x={float(last_pose.get('x', 0.0) or 0.0):.3f}, "
            f"y={float(last_pose.get('y', 0.0) or 0.0):.3f}, "
            f"yaw={self._normalize_angle(float(last_pose.get('yaw', 0.0) or 0.0)):.3f}"
        )

    def _pause_nav2_for_slam(self) -> dict[str, Any]:
        details: dict[str, Any] = {"changed": False, "managers": [], "nodes": [], "errors": []}
        self._stop_robot_motion_for_slam(details)

        paused_labels: set[str] = set()
        manager_type = self._manage_lifecycle_nodes_type
        if manager_type is not None:
            pause_command = int(manager_type.Request.PAUSE)
            for service_name, label in NAV2_LIFECYCLE_MANAGER_SERVICES:
                if self._call_nav2_lifecycle_manager(service_name, label, pause_command, "pause", details):
                    paused_labels.add(label)
                    details["changed"] = True

        covered_nodes: set[str] = set()
        if "navigation" in paused_labels:
            covered_nodes.update({"controller_server", "planner_server", "behavior_server", "smoother_server", "bt_navigator"})
        if "localization" in paused_labels:
            covered_nodes.add("amcl")
        if "map_server" in paused_labels:
            covered_nodes.add("map_server")

        transition_type = self._transition_type
        if transition_type is not None:
            for node_name in NAV2_LIFECYCLE_NODES:
                if node_name in covered_nodes:
                    continue
                if self._call_nav2_node_transition(
                    node_name,
                    int(transition_type.TRANSITION_DEACTIVATE),
                    "deactivate",
                    details,
                ):
                    details["changed"] = True

        if bool(details.get("changed")):
            print(
                "[robot_api_server] Nav2 paused for 2D SLAM: "
                f"managers={details.get('managers') or []}, nodes={details.get('nodes') or []}",
                flush=True,
            )
        else:
            print("[robot_api_server] Nav2 lifecycle pause did not find active Nav2 services before 2D SLAM.", flush=True)
        return details

    def _resume_nav2_after_slam(self) -> dict[str, Any]:
        with self._lock:
            should_resume = bool(self._nav2_paused_for_slam)
            self._nav2_paused_for_slam = False
        details: dict[str, Any] = {"changed": False, "managers": [], "nodes": [], "errors": []}
        if not should_resume:
            return details

        resumed_labels: set[str] = set()
        manager_type = self._manage_lifecycle_nodes_type
        if manager_type is not None:
            resume_command = int(manager_type.Request.RESUME)
            for service_name, label in NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES:
                if self._call_nav2_lifecycle_manager(service_name, label, resume_command, "resume", details):
                    resumed_labels.add(label)
                    details["changed"] = True

        covered_nodes: set[str] = set()
        if "map_server" in resumed_labels:
            covered_nodes.add("map_server")
        if "localization" in resumed_labels:
            covered_nodes.add("amcl")
        if "navigation" in resumed_labels:
            covered_nodes.update({"controller_server", "planner_server", "behavior_server", "smoother_server", "bt_navigator"})

        transition_type = self._transition_type
        if transition_type is not None:
            for node_name in NAV2_LIFECYCLE_RESUME_NODES:
                if node_name in covered_nodes:
                    continue
                if self._call_nav2_node_transition(
                    node_name,
                    int(transition_type.TRANSITION_ACTIVATE),
                    "activate",
                    details,
                ):
                    details["changed"] = True

        print(
            "[robot_api_server] Nav2 resume after 2D SLAM: "
            f"managers={details.get('managers') or []}, nodes={details.get('nodes') or []}",
            flush=True,
        )
        return details

    def _ensure_slam_toolbox_active(self, timeout_sec: float = 4.0) -> None:
        transition_type = self._transition_type
        if transition_type is None:
            raise ValueError("SLAM lifecycle transition type is unavailable")

        deadline = monotonic() + max(0.5, float(timeout_sec))
        errors: list[str] = []
        while monotonic() < deadline:
            with self._lock:
                if self._latest_map is not None:
                    return
            state_client = self._slam_lifecycle_state_client
            if state_client is None or not self._service_available(state_client, 0.2):
                sleep(0.1)
                continue
            try:
                response = self._call_service(
                    state_client,
                    state_client.srv_type.Request(),
                    "slam_toolbox lifecycle state",
                    timeout_sec=1.0,
                )
                current_state = getattr(response, "current_state", None)
                state_id = int(getattr(current_state, "id", 0) or 0)
                state_label = str(getattr(current_state, "label", "") or "").lower()
            except Exception as exc:
                errors.append(str(exc))
                sleep(0.1)
                continue
            if state_id == 3 or state_label == "active":
                return
            if state_id != 2 and state_label != "inactive":
                sleep(0.1)
                continue
            details: dict[str, Any] = {
                "changed": False,
                "managers": [],
                "nodes": [],
                "errors": [],
            }
            if self._call_nav2_node_transition(
                "slam_toolbox",
                int(transition_type.TRANSITION_ACTIVATE),
                "activate",
                details,
            ):
                print("[robot_api_server] slam_toolbox lifecycle is active.", flush=True)
                return
            errors.extend(str(item) for item in details["errors"])
            sleep(0.1)

        with self._lock:
            if self._latest_map is not None:
                return
        detail = errors[-1] if errors else "change_state service is unavailable"
        raise ValueError(f"slam_toolbox did not become active: {detail}")

    def _stop_robot_motion_for_slam(self, details: dict[str, Any]) -> None:
        try:
            self._publish_twist(0.0, 0.0)
        except Exception as exc:
            details["errors"].append(f"cmd_vel stop failed: {exc}")

        try:
            if self._cancel_route_client is not None and self._service_available(self._cancel_route_client, 0.05):
                request = self._cancel_route_client.srv_type.Request()
                request.message = "Route canceled before 2D SLAM."
                response = self._call_service(self._cancel_route_client, request, "route cancel", timeout_sec=1.5)
                if not bool(response.ok):
                    details["errors"].append(str(response.error or "route cancel failed"))
            else:
                self._publish_go_to_lm("cancel")
        except Exception as exc:
            details["errors"].append(f"route cancel failed: {exc}")

    def _call_nav2_lifecycle_manager(
        self,
        service_name: str,
        label: str,
        command: int,
        action: str,
        details: dict[str, Any],
    ) -> bool:
        manager_type = self._manage_lifecycle_nodes_type
        if manager_type is None:
            return False
        client = self._nav2_lifecycle_clients.get(self._topic(service_name))
        if client is None or not self._service_available(client, 0.2):
            return False
        request = manager_type.Request()
        request.command = int(command)
        try:
            response = self._call_service(client, request, f"Nav2 {label} lifecycle {action}", timeout_sec=5.0)
        except Exception as exc:
            details["errors"].append(f"Nav2 {label} lifecycle {action} failed: {exc}")
            return False
        if not bool(getattr(response, "success", False)):
            details["errors"].append(f"Nav2 {label} lifecycle {action} returned success=false")
            return False
        details["managers"].append(label)
        return True

    def _call_nav2_node_transition(
        self,
        node_name: str,
        transition_id: int,
        transition_label: str,
        details: dict[str, Any],
    ) -> bool:
        if self._change_state_type is None:
            return False
        service_name = self._topic(f"/{node_name}/change_state")
        client = self._nav2_change_state_clients.get(service_name)
        if client is None or not self._service_available(client, 0.2):
            return False
        request = self._change_state_type.Request()
        request.transition.id = int(transition_id)
        request.transition.label = str(transition_label)
        try:
            response = self._call_service(client, request, f"Nav2 {node_name} {transition_label}", timeout_sec=3.0)
        except Exception as exc:
            details["errors"].append(f"Nav2 {node_name} {transition_label} failed: {exc}")
            return False
        if not bool(getattr(response, "success", False)):
            details["errors"].append(f"Nav2 {node_name} {transition_label} returned success=false")
            return False
        details["nodes"].append(node_name)
        return True

__all__ = ["RosRuntimeNav2LifecycleMixin"]
