"""Snapshot and runtime-read payloads for the operator fleet facade."""

from __future__ import annotations

from typing import Any


class FleetSnapshotService:
    """Build contextual fleet snapshots without owning fleet state."""

    def __init__(self, owner: Any) -> None:
        self.owner = owner

    def sidebar_payload(
        self,
        include_runtime: bool = True,
    ) -> dict[str, Any]:
        owner = self.owner
        robots = []
        if include_runtime:
            state = owner.state_payload(
                include_trajectories=False,
                advance_runtime=False,
            )
            robots = state.get("robots", [])
        return {
            "id": owner.manager_id,
            "name": owner.display_name,
            "type": "fleet_manager",
            "online": True,
            "host": "local",
            "port": 0,
            "baseUrl": "",
            "identity": {
                "robotId": (
                    "fleet-manager-sim"
                    if owner.mode == "simulation"
                    else "fleet-manager"
                ),
                "mapId": owner.map_dir.stem.replace(
                    ".smap",
                    "",
                ),
                "type": "fleet_manager",
                "mode": owner.mode,
            },
            "status": {
                "state": owner.mode.upper(),
                "robots": (
                    len(robots)
                    if isinstance(robots, list)
                    else 0
                ),
            },
            "runtimeFresh": include_runtime,
            "error": "",
        }

    def state_payload(
        self,
        include_trajectories: bool = True,
        *,
        advance_runtime: bool = False,
    ) -> dict[str, Any]:
        owner = self.owner
        owner._sync_manager_mode()
        if advance_runtime:
            owner._pump_dynamic_benchmark()
            state = owner.manager.state(
                include_trajectories=include_trajectories
            )
        else:
            state = owner.manager.snapshot(
                include_trajectories=include_trajectories
            )
        return owner._state_with_context(state)

    def runtime_step(self) -> None:
        owner = self.owner
        owner._sync_manager_mode()
        if owner.mode == "simulation":
            owner._pump_dynamic_benchmark()
        owner.manager.advance_runtime()

    def tick_payload(
        self,
        payload: dict[str, Any] | None = None,
        *,
        advance_runtime: bool = True,
        route_revisions: dict[str, int] | None = None,
        include_runtime_details: bool = True,
    ) -> dict[str, Any]:
        owner = self.owner
        owner._sync_manager_mode()
        if advance_runtime:
            owner._pump_dynamic_benchmark()
            state = owner.manager.tick(payload or {})
        else:
            state = owner.manager.stream_tick(
                route_revisions=route_revisions,
                include_runtime_details=include_runtime_details,
            )
        return owner._state_with_context(state)

    def _state_with_context(
        self,
        state: Any,
    ) -> dict[str, Any]:
        owner = self.owner
        if not isinstance(state, dict):
            state = owner.manager.snapshot()
        state["mode"] = owner.mode
        state["mapName"] = owner.map_dir.stem.replace(
            ".smap",
            "",
        )
        state["managerId"] = owner.manager_id
        state["managerName"] = owner.display_name
        # High-rate websocket ticks intentionally omit slow collections.
        if owner.mode == "simulation" and (
            "orders" in state
            or "events" in state
        ):
            state["dynamicBenchmark"] = (
                owner._dynamic_benchmark_payload()
            )
        return state

    def _result_with_context(
        self,
        result: dict[str, Any],
    ) -> dict[str, Any]:
        owner = self.owner
        state = result.get("state")
        if isinstance(state, dict):
            result["state"] = owner._state_with_context(state)
        result["mode"] = owner.mode
        result["mapName"] = owner.map_dir.stem.replace(
            ".smap",
            "",
        )
        result["managerId"] = owner.manager_id
        result["managerName"] = owner.display_name
        return result
