"""Route fleet manager commands and stream snapshots."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .fleet_manager import FLEET_MANAGER_ID, FLEET_MANAGER_SIM_ID
from .state_common import utc_now


class FleetApiRoutingMixin:
    """Route fleet manager commands and stream snapshots."""

    def fleet_params_payload(self) -> dict[str, Any]:
        return self._execute_fleet_command(
            FLEET_MANAGER_ID,
            self.fleet_manager.params_payload,
        )

    def fleet_scene3d_asset_path(
        self,
        manager_id: str,
        source_digest: str,
        relative_path: str,
    ) -> Path:
        manager = self._fleet_manager_for_id(manager_id)
        return self._execute_fleet_command(
            manager_id,
            lambda: manager.scene3d_asset_path(
                source_digest,
                relative_path,
            ),
        )

    def save_fleet_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._execute_fleet_command(
            FLEET_MANAGER_ID,
            lambda: self.fleet_manager.save_params_payload(payload),
        )

    def fleet_manager_get_payload(
        self,
        action: str,
        arg: str = "",
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)

        def build_payload() -> dict[str, Any]:
            if action == "identity":
                return manager.sidebar_payload()
            if action == "status":
                return manager.state_payload(
                    advance_runtime=False,
                )
            if action == "state":
                return manager.state_payload(
                    advance_runtime=False,
                )
            if action == "mode":
                return manager.mode_payload()
            if action == "map":
                return manager.map_payload()
            if action == "scene3d":
                return manager.scene3d_payload()
            if action == "maps_list":
                return manager.maps_list_payload()
            if action == "maps_active":
                return manager.maps_active_payload()
            if action == "maps_pull":
                return self.fleet_pull_map_payload({"mapName": arg}, manager_id=manager_id)
            if action == "maps_local_list":
                return self.fleet_local_maps_payload(manager_id=manager_id)
            if action == "maps_local_active":
                return self.fleet_local_active_map_payload(manager_id=manager_id)
            if action == "maps_local_get":
                return self.fleet_local_map_payload(arg, manager_id=manager_id)
            if action == "params":
                return manager.params_payload()
            if action == "orders":
                return manager.orders_payload()
            raise ValueError(f"unknown fleet manager action: {action}")

        return self._execute_fleet_command(
            manager_id,
            build_payload,
        )

    def fleet_manager_post_payload(
        self,
        action: str,
        payload: dict[str, Any],
        manager_id: str = FLEET_MANAGER_ID,
    ) -> dict[str, Any]:
        manager = self._fleet_manager_for_id(manager_id)

        def apply_command() -> dict[str, Any]:
            if action == "mode":
                return manager.set_mode_payload(payload)
            if action == "params":
                return manager.save_params_payload(payload)
            if action == "plan":
                return manager.plan_payload(payload)
            if action == "benchmark":
                return manager.benchmark_payload(payload)
            if action == "set_order":
                return manager.set_order_payload(payload)
            if action == "orders_dispatch":
                return manager.dispatch_orders_payload(payload)
            if action == "orders_cancel":
                return manager.cancel_order_payload(payload)
            if action == "orders_pause":
                return manager.pause_order_payload(payload)
            if action == "orders_resume":
                return manager.resume_order_payload(payload)
            if action == "orders_clear":
                return manager.clear_orders_payload(payload)
            if action == "tick":
                return manager.tick_payload(
                    payload,
                    advance_runtime=manager_id != FLEET_MANAGER_SIM_ID,
                )
            if action == "world":
                return manager.world_payload(payload)
            if action == "check":
                return manager.check_payload(payload)
            if action == "manual_step":
                return manager.manual_step_payload(payload)
            if action == "manual_stop":
                return manager.manual_stop_payload(payload)
            if action == "robots_control_acquire":
                return manager.acquire_control_payload(payload)
            if action == "robots_control_release":
                return manager.release_control_payload(payload)
            if action == "maps_load":
                return self.fleet_load_map_payload(payload, manager_id=manager_id)
            if action == "maps_save":
                return manager.save_map_payload(payload)
            if action == "maps_local_save":
                return self.fleet_save_local_map_payload(payload, manager_id=manager_id)
            if action == "maps_local_activate":
                return self.fleet_activate_local_map_payload(payload, manager_id=manager_id)
            if action == "maps_pull":
                return self.fleet_pull_map_payload(payload, manager_id=manager_id)
            if action == "maps_pull_sync":
                return self.fleet_pull_sync_payload(manager_id=manager_id)
            if action == "maps_push":
                return self.fleet_push_map_payload(payload, manager_id=manager_id)
            if action == "maps_push_sync":
                return self.fleet_push_sync_payload(manager_id=manager_id)
            if action == "robots_add":
                return manager.add_robot_payload(payload)
            if action == "robots_remove":
                return manager.remove_robot_payload(payload)
            if action == "robots_update":
                return manager.update_robot_payload(payload)
            if action == "robots_stop":
                return manager.stop_robot_payload(payload)
            if action == "robots_reset":
                return manager.reset_robot_payload(payload)
            raise ValueError(f"unknown fleet manager action: {action}")

        return self._execute_fleet_command(
            manager_id,
            apply_command,
        )

    def fleet_manager_stream_payload(
        self,
        initial: bool = False,
        manager_id: str = FLEET_MANAGER_ID,
        route_revisions: dict[str, int] | None = None,
        include_runtime_details: bool = True,
    ) -> dict[str, Any] | None:
        def build_snapshot() -> dict[str, Any]:
            manager = self._fleet_manager_for_id(manager_id)
            state = (
                manager.state_payload(
                    include_trajectories=True,
                    advance_runtime=False,
                )
                if initial
                else manager.tick_payload(
                    {},
                    advance_runtime=False,
                    route_revisions=route_revisions,
                    include_runtime_details=include_runtime_details,
                )
            )
            return {
                "ok": True,
                "type": "state" if initial else "tick",
                "state": state,
                "sentAt": utc_now(),
            }

        return self._execute_fleet_command(
            manager_id,
            build_snapshot,
        )
