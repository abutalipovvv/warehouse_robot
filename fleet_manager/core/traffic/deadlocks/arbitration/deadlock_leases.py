"""Deadlock recovery cooldowns and active priority leases."""

from __future__ import annotations

from fleet_manager.core.fleet.domain.models import FleetRobot


class DeadlockLeaseMixin:
    """Maintain one deterministic recovery attempt and its bounded lease."""

    def _wait_cycle_recovery_signature(
        self,
        action: str,
        selected: FleetRobot,
        robots: list[FleetRobot],
    ) -> tuple[str, str, tuple[tuple[str, str, str, str], ...]]:
        """Describe the graph-stable state for one spatial recovery action."""
        members = tuple(
            sorted(
                (
                    robot.name,
                    self._traffic_lm_for_robot(robot),
                    str(robot.active_order_id or ""),
                    str(robot.route_final_lm or robot.target_lm or ""),
                )
                for robot in robots
            )
        )
        return str(action), selected.name, members

    def _wait_cycle_recovery_cooldown(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            stationary_retry = float(
                fleet.get("stationary_recovery_retry_sec", 4.0) or 4.0
            )
        except (TypeError, ValueError):
            stationary_retry = 4.0
        default = max(
            self._deadlock_priority_lease(),
            stationary_retry,
        )
        try:
            configured = float(
                fleet.get("deadlock_recovery_cooldown_sec", default) or default
            )
        except (TypeError, ValueError):
            configured = default
        return max(self._deadlock_priority_lease(), min(30.0, configured))

    def _wait_cycle_recovery_ready(
        self,
        signature: tuple[
            str,
            str,
            tuple[tuple[str, str, str, str], ...],
        ],
        now: float,
    ) -> bool:
        """Gate identical recovery attempts across transient wait states."""
        cooldown = self._wait_cycle_recovery_cooldown()
        # Order ids are intentionally part of the signature so a new task gets
        # an independent recovery episode. Bound old signatures nevertheless:
        # lifelong package benchmarks must not accumulate one key per order.
        retention = max(60.0, cooldown * 8.0)
        for old_signature, attempted_at in list(
            self._wait_cycle_recovery_attempts.items()
        ):
            if now - attempted_at >= retention:
                self._wait_cycle_recovery_attempts.pop(old_signature, None)
        attempted_at = self._wait_cycle_recovery_attempts.get(signature)
        return attempted_at is None or now - attempted_at >= cooldown

    def _record_wait_cycle_recovery_attempt(
        self,
        signature: tuple[
            str,
            str,
            tuple[tuple[str, str, str, str], ...],
        ],
        now: float,
    ) -> None:
        self._wait_cycle_recovery_attempts[signature] = now

    def _clear_wait_cycle_recovery_attempts(
        self,
        robot_names: tuple[str, ...] | list[str] | set[str],
    ) -> None:
        names = set(robot_names)
        if not names:
            return
        for signature in list(self._wait_cycle_recovery_attempts):
            if any(member[0] in names for member in signature[2]):
                self._wait_cycle_recovery_attempts.pop(signature, None)

    def _maintain_runtime_wait_cycle_lease(
        self,
        cycle: list[str],
        waiting: dict[str, FleetRobot],
        now: float,
    ) -> bool:
        robots = [waiting[name] for name in cycle if name in waiting]
        leased = [robot for robot in robots if robot.traffic_priority_until > now]
        if not leased:
            return False
        cycle_stall_started = min(
            (robot.traffic_stall_since or robot.blocked_since or now for robot in robots),
            default=now,
        )
        if now - cycle_stall_started >= self._deadlock_coupled_replan_after():
            # The leased winner failed to produce even one route-clock advance.
            # Stop renewing it as soon as the local-CBS deadline is reached;
            # otherwise the short priority lease can be refreshed forever and
            # a visibly stopped group never reaches coupled replanning.
            for robot in leased:
                robot.traffic_priority_until = 0.0
            return False
        winner = max(
            leased,
            key=lambda robot: (robot.traffic_priority_until, robot.name),
        )
        # The runtime collision check may mark the winner WAITING again during
        # the same geometric encounter.  Reassert the existing lease without
        # counting/resolving the identical cycle at every 10 Hz physics tick.
        winner.status = "MOVING" if winner.trajectory else "WAITING"
        winner.last_reason = "deadlock priority active"
        self._clear_wait_dependency(winner)
        winner.updated_at = now
        for robot in robots:
            if robot.name == winner.name:
                continue
            robot.status = "WAITING"
            robot.last_reason = f"yield to {winner.name}"
            robot.wait_for_robot = winner.name
            robot.wait_resource = self._edge_id_at_trajectory(
                robot.trajectory,
                robot.route_clock,
            )
            robot.wait_release_at = winner.traffic_priority_until
            robot.blocked_since = robot.blocked_since or now
            robot.updated_at = now
        return True
