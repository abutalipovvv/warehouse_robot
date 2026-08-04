"""Build and validate future controlled-corridor intents."""

from __future__ import annotations

from typing import Any

from fleet_manager.manager.tasks.statuses import TERMINAL_ORDER_STATUSES
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.robot.model import FleetRobot
from fleet_manager.core.traffic.corridors.scheduling.corridor_models import (
    CorridorRequest,
    CorridorResourceWindow,
)

from .controlled_corridor_prefetch_models import (
    _CorridorIntentDraft,
    _CorridorRouteDraft,
)


class ControlledCorridorPrefetchIntentMixin:
    """Build and validate future controlled-corridor intents."""

    def _controlled_corridor_prefetch_intent(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        *,
        prediction_offset: float,
        now: float,
    ) -> dict[str, Any] | None:
        """Register the first authored-corridor passage before MAPF.

        A normal committed trajectory is already visible to the central
        calendar. A rolling continuation is not: without this intent SIPP can
        discover a downstream reservation only after entering a no-wait
        chain, reject the plan, and repeat forever. A nominal kinematic
        timeline is sufficient for admission; the finished MAPF trajectory is
        rechecked against the live slot before it is appended.
        """
        scheduler = self._controlled_corridor_scheduler
        if scheduler is None:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        draft = self._controlled_corridor_prefetch_draft(
            order,
            robot,
            request,
            prediction_offset=prediction_offset,
            now=now,
            scheduler=scheduler,
        )
        if draft is None:
            self._controlled_corridor_prefetch_intents.pop(
                robot.name,
                None,
            )
            return None
        existing = self._controlled_corridor_prefetch_intents.get(
            robot.name
        )
        if (
            isinstance(existing, dict)
            and existing.get("signature") == draft.signature
        ):
            # The executable passage is unchanged; only refresh the parent
            # spatial-route generation without moving its schedule epoch.
            existing["spatial_route_revision"] = int(
                order.spatial_route_revision or 0
            )
            return existing
        intent = self._controlled_corridor_prefetch_intent_payload(
            order,
            robot,
            draft,
            now=now,
        )
        self._controlled_corridor_prefetch_intents[robot.name] = intent
        return intent

    def _controlled_corridor_prefetch_route_draft(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
    ) -> _CorridorRouteDraft | None:
        """Build the nominal trajectory used only for passage discovery."""
        route_nodes = [
            str(node)
            for node in request.get("routeNodes", ())
            if str(node) in self.landmarks
        ]
        if len(route_nodes) < 2:
            return None
        start_lm = str(request.get("startLm") or "")
        if route_nodes[0] != start_lm:
            return None
        route_payload: dict[str, Any] = {}
        if order.speed > 0.0:
            route_payload["speed"] = order.speed
        if order.acceleration > 0.0:
            route_payload["acceleration"] = order.acceleration
        speed = self.planner._route_speed(route_payload)
        acceleration = self.planner._route_acceleration(route_payload)
        start_pose = request.get("startPose")
        start_yaw = (
            float(start_pose.get("yaw", 0.0) or 0.0)
            if isinstance(start_pose, dict)
            else 0.0
        )
        trajectory = self.planner._trajectory_for_nodes(
            route_nodes,
            speed,
            acceleration=acceleration,
            rotate_enabled=bool(order.rotate),
            turn_speed=(
                order.turn_speed
                if order.turn_speed > 0.0
                else self.planner._turn_speed({})
            ),
            stretch_motion_to_reservation_ticks=(
                order.stretch_motion_to_reservation_ticks
            ),
            start_yaw=start_yaw,
        )
        if len(trajectory) < 2:
            return None
        pose = (
            {
                "x": float(start_pose.get("x", 0.0) or 0.0),
                "y": float(start_pose.get("y", 0.0) or 0.0),
                "yaw": start_yaw,
            }
            if isinstance(start_pose, dict)
            else self._pose_at_landmark(start_lm)
        )
        return _CorridorRouteDraft(
            route_nodes=route_nodes,
            start_lm=start_lm,
            trajectory=trajectory,
            pose=pose,
        )

    def _controlled_corridor_prefetch_draft(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        request: dict[str, Any],
        *,
        prediction_offset: float,
        now: float,
        scheduler: Any,
    ) -> _CorridorIntentDraft | None:
        """Discover the first controlled passage and its nominal timing."""
        route = self._controlled_corridor_prefetch_route_draft(
            order,
            robot,
            request,
        )
        if route is None:
            return None
        synthetic = FleetRobot(
            name=robot.name,
            current_lm=route.start_lm,
            target_lm=str(
                request.get("goalLm") or route.route_nodes[-1]
            ),
            status="MOVING",
            active_order_id=robot.active_order_id,
            pose=route.pose,
            trajectory=route.trajectory,
            route_clock=0.0,
            route_revision=int(robot.route_revision),
        )
        entry = self._next_controlled_corridor_entry(synthetic)
        regions = self._controlled_corridor_entry_regions(entry)
        if (
            entry is None
            or not regions
            or not set(regions).issubset(scheduler.controlled_regions)
        ):
            return None
        exit_lm = str(entry.get("exit_lm") or "")
        staging_lm = str(
            entry.get("holding_lm")
            or entry.get("src")
            or ""
        )
        if exit_lm not in self.landmarks or staging_lm not in self.landmarks:
            return None
        direction = self._controlled_corridor_flow_direction(entry)
        staging_clock = max(
            0.0,
            float(entry.get("staging_clock", 0.0) or 0.0),
        )
        exit_clock = max(
            staging_clock + self._runtime_motion_step(),
            float(
                entry.get("exit_clock", staging_clock)
                or staging_clock
            ),
        )
        intent_kind = (
            "rolling"
            if (
                robot.active_order_id == order.order_id
                and bool(robot.trajectory)
            )
            else "dispatch"
        )
        signature = (
            int(robot.route_revision),
            order.order_id,
            intent_kind,
            route.start_lm,
            tuple(route.route_nodes),
            regions,
            direction,
            staging_lm,
            exit_lm,
        )
        handoff_at = now + max(0.0, float(prediction_offset))
        return _CorridorIntentDraft(
            route=route,
            entry=entry,
            regions=regions,
            direction=direction,
            staging_lm=staging_lm,
            exit_lm=exit_lm,
            staging_clock=staging_clock,
            exit_clock=exit_clock,
            kind=intent_kind,
            signature=signature,
            handoff_at=handoff_at,
            earliest_entry=handoff_at + staging_clock,
        )

    def _controlled_corridor_prefetch_intent_payload(
        self,
        order: FleetOrder,
        robot: FleetRobot,
        draft: _CorridorIntentDraft,
        *,
        now: float,
    ) -> dict[str, Any]:
        """Publish a draft in the legacy dictionary contract."""
        corridor_request = CorridorRequest(
            robot_id=robot.name,
            regions=draft.regions,
            direction=draft.direction,
            earliest_entry=draft.earliest_entry,
            duration_sec=max(
                self._runtime_motion_step(),
                draft.exit_clock - draft.staging_clock,
            ),
            staging_lm=draft.staging_lm,
            exit_lm=draft.exit_lm,
            route_revision=int(robot.route_revision),
            priority=float(order.priority or 0),
            wait_age_sec=0.0,
            deadline=None,
            downstream_available=True,
            entered=False,
            past_commit_point=False,
            # This is only a future route proposal. It becomes immutable in
            # ``commit_slot`` after the exact SIPP trajectory has been
            # revalidated; wall-clock proximity alone must never create a
            # green command for an idle robot.
            requires_explicit_commit=True,
            resource_windows=tuple(
                window
                for window in draft.entry.get(
                    "resource_windows",
                    (),
                )
                if isinstance(window, CorridorResourceWindow)
            ),
        )
        return {
            "signature": draft.signature,
            "kind": draft.kind,
            "order_id": order.order_id,
            "start_lm": draft.route.start_lm,
            "route_revision": int(robot.route_revision),
            "spatial_route_revision": int(
                order.spatial_route_revision or 0
            ),
            "trajectory_route_nodes": tuple(draft.route.route_nodes),
            "request": corridor_request,
            "entry": dict(draft.entry),
            "trajectory": draft.route.trajectory,
            "start_pose": draft.route.pose,
            "registered_at": now,
            "handoff_at": draft.handoff_at,
            "last_schedule_epoch": None,
        }

    def _controlled_corridor_intent_is_current(
        self,
        robot: FleetRobot,
        order: FleetOrder,
        intent: dict[str, Any],
    ) -> bool:
        """Validate a future passage without relying on positional tuples."""
        signature = intent.get("signature")
        raw_request = intent.get("request")
        kind = str(intent.get("kind") or "")
        start_lm = str(intent.get("start_lm") or "")
        if (
            not isinstance(signature, tuple)
            or len(signature) != 9
            or not isinstance(raw_request, CorridorRequest)
            or kind not in {"dispatch", "rolling"}
            or self.orders.get(order.order_id) is not order
            or self.robots.get(robot.name) is not robot
            or order.status in TERMINAL_ORDER_STATUSES
            or str(intent.get("order_id") or "") != order.order_id
            or int(intent.get("route_revision", -1))
            != int(robot.route_revision)
            or int(intent.get("spatial_route_revision", -1))
            != int(order.spatial_route_revision or 0)
            or signature
            != (
                int(robot.route_revision),
                order.order_id,
                kind,
                start_lm,
                tuple(
                    str(node)
                    for node in intent.get("trajectory_route_nodes", ())
                ),
                raw_request.regions,
                raw_request.direction,
                raw_request.staging_lm,
                raw_request.exit_lm,
            )
        ):
            return False
        if kind == "rolling":
            return bool(
                robot.active_order_id == order.order_id
                and robot.trajectory
                and str(robot.route_chunk_goal_lm or "") == start_lm
            )
        owner = str(order.vehicle or order.assigned_robot or "")
        return bool(
            owner == robot.name
            and not robot.active_order_id
            and not robot.trajectory
            and order.status in {"QUEUED", "PLANNING"}
            and self._safe_replan_start_lm(robot) == start_lm
        )
