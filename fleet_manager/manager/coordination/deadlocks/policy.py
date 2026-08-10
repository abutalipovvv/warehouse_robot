"""Validated timing policy accessors for deadlock recovery."""

from __future__ import annotations


class DeadlockPolicyMixin:
    """Read and clamp traffic recovery timing policy from fleet parameters."""

    def _deadlock_priority_lease(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.5
        try:
            return max(0.5, float(fleet.get("deadlock_priority_lease_sec", 1.5) or 1.5))
        except (TypeError, ValueError):
            return 1.5

    def _deadlock_wait_timeout(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(
                0.5,
                float(fleet.get("deadlock_wait_timeout_sec", 1.0) or 1.0),
            )
        except (TypeError, ValueError):
            return 1.0

    def _traffic_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 3.0
        try:
            return max(
                self._deadlock_wait_timeout(),
                float(fleet.get("traffic_replan_after_sec", 3.0) or 3.0),
            )
        except (TypeError, ValueError):
            return 3.0

    def _blocked_replan_after(self, reason: str) -> float:
        if not self._is_parked_robot_conflict(reason):
            return self._traffic_replan_after()
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(
                0.25,
                float(
                    fleet.get("parked_robot_replan_after_sec", 1.0)
                    or 1.0
                ),
            )
        except (TypeError, ValueError):
            return 1.0

    def _traffic_zone_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 6.0
        try:
            # Give the starvation-aware phase selector one normal opportunity
            # before switching corridors, but still impose a hard upper bound.
            return max(
                self._traffic_zone_param("traffic_zone_starvation_sec", 5.0),
                float(fleet.get("traffic_zone_replan_after_sec", 6.0) or 6.0),
            )
        except (TypeError, ValueError):
            return 6.0

    def _controlled_corridor_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 8.0
        try:
            return max(
                self._controlled_corridor_param(
                    "controlled_corridor_starvation_sec",
                    8.0,
                ),
                float(
                    fleet.get(
                        "controlled_corridor_replan_after_sec",
                        8.0,
                    )
                    or 8.0
                ),
            )
        except (TypeError, ValueError):
            return 8.0

    def _deadlock_coupled_replan_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.5
        try:
            return max(
                self._deadlock_wait_timeout(),
                float(fleet.get("deadlock_coupled_replan_after_sec", 1.5) or 1.5),
            )
        except (TypeError, ValueError):
            return 1.5

    def _deadlock_coupled_replan_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.5
        try:
            return max(
                0.5,
                float(fleet.get("deadlock_coupled_replan_interval_sec", 1.5) or 1.5),
            )
        except (TypeError, ValueError):
            return 1.5

    def _deadlock_retreat_after(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 4.5
        try:
            return max(
                self._deadlock_coupled_replan_after(),
                float(fleet.get("deadlock_retreat_after_sec", 4.5) or 4.5),
            )
        except (TypeError, ValueError):
            return 4.5

    def _replan_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 1.0
        try:
            return max(0.25, float(fleet.get("replan_interval_sec", 1.0) or 1.0))
        except (TypeError, ValueError):
            return 1.0

    def _runtime_collision_preflight_interval(self) -> float:
        fleet = self.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 0.50
        try:
            return max(
                0.10,
                min(
                    0.50,
                    float(
                        fleet.get(
                            "runtime_collision_preflight_interval_sec",
                            0.50,
                        )
                        or 0.50
                    ),
                ),
            )
        except (TypeError, ValueError):
            return 0.50
