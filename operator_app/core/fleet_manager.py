from __future__ import annotations

import math
import random
from pathlib import Path
from time import perf_counter, time
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FLEET_ROOT = PROJECT_ROOT / "fleet_manager"
FLEET_MAP_DATA_ROOT = FLEET_ROOT / "map_data"
FLEET_MAPS_OUT_ROOT = FLEET_MAP_DATA_ROOT / "maps_out"
DEFAULT_FLEET_MAP_DIR = FLEET_MAPS_OUT_ROOT / "22.05.26_smap.smap"
DEFAULT_FLEET_SIM_MAP_DIR = FLEET_MAPS_OUT_ROOT / "benchmark_open_kiva.smap"
FLEET_MANAGER_ID = "__fleet_manager__"
FLEET_MANAGER_SIM_ID = "__fleet_manager_sim__"
FLEET_MANAGER_IDS = {FLEET_MANAGER_ID, FLEET_MANAGER_SIM_ID}

from fleet_manager.runtime.grpc.manager import FleetManagerROS
from fleet_manager.runtime.simulation.manager import FleetManagerSim
from fleet_manager.core.route_core.map_exchange import (
    build_editable_map_bundle_payload,
    build_editable_map_payload,
    list_editable_maps,
)
from fleet_manager.core.route_core.map_loader import WarehouseMapLoader
from fleet_manager.core.route_core.map_writer import save_editable_map
from fleet_manager.core.route_core.params import load_route_params, save_route_params


class OperatorFleetManager:
    def __init__(
        self,
        map_dir: Path,
        params_path: Path,
        remote_adapter: Any | None = None,
        *,
        manager_id: str = FLEET_MANAGER_ID,
        display_name: str = "Fleet Manager",
        mode: str = "robots",
    ) -> None:
        self.params_path = Path(params_path).expanduser().resolve()
        self.remote_adapter = remote_adapter
        mode = str(mode or "").strip().lower()
        if mode not in {"robots", "simulation"}:
            raise ValueError("mode must be robots or simulation")
        self.manager_id = manager_id
        self.display_name = display_name
        self.mode = mode
        self.map_dir = self.resolve_map_dir(map_dir)
        self.maps_root = self.map_dir.parent
        self._scene3d_cache: dict[str, Any] | None = None
        self._load_context(self.map_dir)

    def sidebar_payload(self, include_runtime: bool = True) -> dict[str, Any]:
        robots = []
        if include_runtime:
            state = self.state_payload(
                include_trajectories=False,
                advance_runtime=self.mode != "simulation",
            )
            robots = state.get("robots", [])
        return {
            "id": self.manager_id,
            "name": self.display_name,
            "type": "fleet_manager",
            "online": True,
            "host": "local",
            "port": 0,
            "baseUrl": "",
            "identity": {
                "robotId": "fleet-manager-sim" if self.mode == "simulation" else "fleet-manager",
                "mapId": self.map_dir.stem.replace(".smap", ""),
                "type": "fleet_manager",
                "mode": self.mode,
            },
            "status": {
                "state": self.mode.upper(),
                "robots": len(robots) if isinstance(robots, list) else 0,
            },
            "runtimeFresh": include_runtime,
            "error": "",
        }

    def mode_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "id": self.manager_id,
            "mode": self.mode,
            "mapName": self.map_dir.stem.replace(".smap", ""),
        }

    def set_mode_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode") or "").strip().lower()
        if mode not in {"simulation", "robots"}:
            raise ValueError("mode must be simulation or robots")
        if mode != self.mode:
            raise ValueError(f"{self.display_name} mode is fixed to {self.mode}")
        return self.mode_payload()

    def params_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "path": str(self.params_path),
            "params": load_route_params(self.params_path, create=True),
        }

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        params_payload = payload.get("params")
        if not isinstance(params_payload, dict):
            params_payload = payload
        save_route_params(params_payload, self.params_path)
        self._load_context(self.map_dir)
        return self.params_payload()

    def map_payload(self) -> dict[str, Any]:
        params = load_route_params(self.params_path, create=True)
        return build_editable_map_payload(self.map_dir, params=params)

    def scene3d_payload(self) -> dict[str, Any]:
        self._sync_manager_mode()
        static_scene = self._static_scene3d_payload()
        state = (
            self.manager.snapshot(include_trajectories=True)
            if self.mode == "simulation"
            else self.manager.state(include_trajectories=True)
        )
        robots = state.get("robots", []) if isinstance(state, dict) else []
        return {
            **static_scene,
            "robots": robots if isinstance(robots, list) else [],
            "mode": self.mode,
            "managerId": self.manager_id,
            "managerName": self.display_name,
        }

    def maps_active_payload(self) -> dict[str, Any]:
        payload = self.map_payload()
        return {
            "ok": True,
            "mapName": str(payload.get("mapName") or self.map_dir.stem.replace(".smap", "")),
            "mapDir": str(self.map_dir),
            "signature": str(payload.get("signature") or ""),
        }

    def maps_list_payload(self) -> dict[str, Any]:
        return list_editable_maps(self.maps_root, active_map_dir=self.map_dir)

    def pull_map_payload(self, map_name: str = "") -> dict[str, Any]:
        target = self._resolve_map_dir_by_name(map_name)
        params = load_route_params(self.params_path, create=True)
        return build_editable_map_bundle_payload(target, params=params)

    def push_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        target_name = str(payload.get("mapName") or "").strip()
        source_name = str(payload.get("sourceMapName") or target_name).strip()
        source_dir = self._resolve_map_dir_by_name(source_name)
        output_name = str(payload.get("outputName") or "").strip()
        if not output_name and target_name and source_name and target_name != source_name:
            output_name = target_name
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        loaded_map = save_editable_map(
            source_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=bool(payload.get("overwriteOutput", False)),
        )
        params = load_route_params(self.params_path, create=True)
        return {
            **build_editable_map_payload(loaded_map.map_dir, params=params),
            "savedAs": bool(output_name),
        }

    def load_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        safe_name = Path(map_name).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.maps_root / safe_name).resolve()
        if self.maps_root.resolve() not in target.parents and target != self.maps_root.resolve():
            raise ValueError("map must stay inside maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        self._load_context(target)
        return {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "mode": self.mode,
        }

    def save_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            editable_map = payload
        output_name = str(payload.get("outputName") or "").strip()
        loaded_map = save_editable_map(
            self.map_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=bool(payload.get("overwriteOutput", False)),
        )
        self._load_context(loaded_map.map_dir)
        return {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "map": self.map_payload(),
            "maps": self.maps_list_payload(),
        }

    def state_payload(
        self,
        include_trajectories: bool = True,
        *,
        advance_runtime: bool = True,
    ) -> dict[str, Any]:
        self._sync_manager_mode()
        if advance_runtime:
            self._pump_dynamic_benchmark()
            state = self.manager.state(include_trajectories=include_trajectories)
        else:
            state = self.manager.snapshot(include_trajectories=include_trajectories)
        return self._state_with_context(state)

    def runtime_step(self) -> None:
        if self.mode != "simulation":
            return
        self._sync_manager_mode()
        self._pump_dynamic_benchmark()
        self.manager.advance_runtime()

    def plan_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        result = self.manager.plan(payload)
        if isinstance(result.get("fleetState"), dict):
            result["state"] = result["fleetState"]
        return self._result_with_context(result)

    def benchmark_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        if self.mode != "simulation":
            raise ValueError("benchmark is only available in Fleet Manager Sim")
        raw_count = payload.get("count", 20)
        count = max(0, min(300, int(20 if raw_count is None else raw_count)))
        seed = int(payload.get("seed", 42) or 42)
        speed = float(payload.get("speed") or payload.get("routeSpeed") or 0.0)
        acceleration = float(payload.get("acceleration") or payload.get("routeAcceleration") or 0.0)
        rotate = bool(payload.get("rotate") or payload.get("simulateRotation") or payload.get("simulate_rotation"))
        turn_speed = float(payload.get("turnSpeed") or payload.get("turn_speed") or payload.get("rotationSpeed") or 0.0)
        if speed <= 0.0:
            params = load_route_params(self.params_path, create=True)
            navigation = params.get("navigation", {}) if isinstance(params, dict) else {}
            speed = float(navigation.get("route_speed", 1.0) or 1.0)
            acceleration = float(navigation.get("route_acceleration", acceleration) or acceleration or 0.0)
            rotate = bool(navigation.get("simulate_rotation", rotate))
            turn_speed = float(navigation.get("turn_speed", turn_speed) or turn_speed or 0.0)
        action = str(payload.get("action") or "").strip().lower()
        add_only = action in {"add", "add_robots", "ensure"} or bool(payload.get("addOnly", False))
        plan_existing = action in {"plan", "plan_existing"} or bool(payload.get("planExisting", False))
        package_existing = action in {
            "package",
            "package_orders",
            "package_waves",
        }
        stop_dynamic = action in {"stop", "stop_dynamic", "pause_dynamic"}
        set_time_scale = action in {"time_scale", "set_time_scale"}
        reset_default = not (
            add_only
            or plan_existing
            or package_existing
            or stop_dynamic
            or set_time_scale
        )
        if payload.get("reset", reset_default):
            self._clear_simulation_runtime()
        if set_time_scale:
            # Finish the elapsed slice at the old rate first. This keeps a
            # live robot continuous when the operator changes, for example,
            # from 8x back to 1x between two physics ticks.
            self._pump_dynamic_benchmark()
            self.manager.advance_runtime()
            scale = self.manager.set_simulation_time_scale(
                payload.get("timeScale", payload.get("scale", 1.0))
            )
            return self._state_with_context({
                **self.manager.snapshot(include_trajectories=True),
                "benchmark": {
                    "action": "time_scale",
                    "timeScale": scale,
                },
            })
        if count <= 0:
            return self._state_with_context({
                **self.manager.state(include_trajectories=True),
                "benchmark": {
                    "count": 0,
                    "planned": 0,
                    "elapsedMs": 0.0,
                    "seed": seed,
                    "cleared": True,
                },
            })

        if add_only:
            return self._ensure_benchmark_robots_payload(count=count, seed=seed)

        if stop_dynamic:
            return self._stop_dynamic_benchmark_payload()

        if package_existing:
            return self._start_dynamic_benchmark_payload(
                count=count,
                seed=seed,
                horizon_sec=float(payload.get("horizonSec", 10.0) or 10.0),
                order_interval_sec=0.0,
                queue_depth=1,
                speed=speed,
                acceleration=acceleration,
                rotate=rotate,
                turn_speed=turn_speed,
                generation_mode="package_waves",
            )

        if plan_existing:
            return self._start_dynamic_benchmark_payload(
                count=count,
                seed=seed,
                horizon_sec=float(payload.get("horizonSec", 10.0) or 10.0),
                order_interval_sec=float(payload.get("orderIntervalSec", 3.0) or 3.0),
                queue_depth=int(payload.get("queueDepth", 2) or 2),
                speed=speed,
                acceleration=acceleration,
                rotate=rotate,
                turn_speed=turn_speed,
            )

        # Explicit Plan is a traffic demonstration: routes are deliberately
        # longer and overlap at shared corridors. The plain benchmark endpoint
        # remains balanced unless stress is requested explicitly.
        stress = bool(payload.get("stress", plan_existing))
        stress_profile = 0
        used_traffic_stress = stress
        requests = (
            self._benchmark_requests_for_existing(
                count=count,
                seed=seed,
                stress=stress,
                stress_profile=stress_profile,
            )
            if plan_existing
            else self._benchmark_requests(
                count=count,
                seed=seed,
                stress=stress,
                stress_profile=stress_profile,
            )
        )
        if plan_existing:
            self.manager.orders.clear()
        fast = bool(payload.get("fast", False))
        restore_fleet_params = self._apply_fast_benchmark_params(count)
        started = perf_counter()
        allocation_attempts = 1
        try:
            result = self.manager.plan({
                "robots": requests,
                "speed": speed,
                "acceleration": acceleration,
                "rotate": rotate,
                "turnSpeed": turn_speed,
                "stretchMotionToReservationTicks": True,
            })
            # A challenge profile can still encode an impossible terminal
            # order in a one-way aisle. Try deterministic alternatives, then
            # fall back to the balanced allocator instead of accepting a
            # permanent deadlock.
            while (
                not result.get("ok")
                and allocation_attempts < (4 if stress else 6)
            ):
                retry_seed = seed + allocation_attempts
                stress_profile = allocation_attempts
                use_traffic_stress = stress and stress_profile < 3
                requests = (
                    self._benchmark_requests_for_existing(
                        count=count,
                        seed=seed if use_traffic_stress else retry_seed,
                        stress=use_traffic_stress,
                        stress_profile=stress_profile,
                    )
                    if plan_existing
                    else self._benchmark_requests(
                        count=count,
                        seed=seed if use_traffic_stress else retry_seed,
                        stress=use_traffic_stress,
                        stress_profile=stress_profile,
                    )
                )
                used_traffic_stress = use_traffic_stress
                allocation_attempts += 1
                result = self.manager.plan({
                    "robots": requests,
                    "speed": speed,
                    "acceleration": acceleration,
                    "rotate": rotate,
                    "turnSpeed": turn_speed,
                    "stretchMotionToReservationTicks": True,
                })
        finally:
            if restore_fleet_params is not None:
                self._restore_fleet_params(restore_fleet_params)
        elapsed_ms = (perf_counter() - started) * 1000.0
        plans = result.get("plans", [])
        if not isinstance(plans, list):
            plans = []
        debug = result.get("debug", {})
        if not isinstance(debug, dict):
            debug = {}
        plan_stats = self._benchmark_plan_stats(
            plans,
            float(result.get("timeStepSec", 1.0) or 1.0),
        )
        result["benchmark"] = {
            "count": count,
            "planned": len(plans),
            "elapsedMs": round(elapsed_ms, 3),
            "seed": seed,
            "speed": speed,
            "acceleration": acceleration,
            "rotate": rotate,
            "turnSpeed": turn_speed,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "plannerBackend": debug.get("plannerBackend", ""),
            "reason": debug.get("reason", ""),
            "expandedNodes": debug.get("expandedNodes", 0),
            "highLevelNodes": debug.get("highLevelNodes", 0),
            "fast": fast,
            "allocationAttempts": allocation_attempts,
            "action": "plan" if plan_existing else "benchmark",
            "scenario": (
                "traffic_stress"
                if used_traffic_stress
                else "balanced_fallback"
                if stress
                else "balanced"
            ),
            "stressProfile": stress_profile if used_traffic_stress else None,
            "resolvedPriorityConflicts": int(debug.get("conflictsResolved", 0) or 0),
            **plan_stats,
        }
        if isinstance(result.get("fleetState"), dict):
            result["state"] = result["fleetState"]
        return self._result_with_context(result)

    def orders_payload(self) -> dict[str, Any]:
        self._sync_manager_mode()
        self._pump_dynamic_benchmark()
        result = self.manager.orders_payload()
        return self._result_with_context(result)

    def set_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.set_order(payload))

    def dispatch_orders_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.dispatch_orders(payload or {}))

    def cancel_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.cancel_order(payload))

    def pause_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.pause_order(payload))

    def resume_order_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.resume_order(payload))

    def clear_orders_payload(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.clear_orders(payload or {}))

    def tick_payload(
        self,
        payload: dict[str, Any] | None = None,
        *,
        advance_runtime: bool = True,
        route_revisions: dict[str, int] | None = None,
        include_runtime_details: bool = True,
    ) -> dict[str, Any]:
        self._sync_manager_mode()
        if advance_runtime:
            self._pump_dynamic_benchmark()
            state = self.manager.tick(payload or {})
        else:
            state = self.manager.stream_tick(
                route_revisions=route_revisions,
                include_runtime_details=include_runtime_details,
            )
        return self._state_with_context(state)

    def world_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_world(payload))

    def check_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self.manager.check_path(payload)

    def manual_step_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if self.mode == "robots":
            result = self.manager.teleop_robot(
                {
                    "name": name,
                    "linear": float(payload.get("linear", 0.0) or 0.0),
                    "angular": float(payload.get("angular", 0.0) or 0.0),
                    "timeoutMs": int(payload.get("timeoutMs", 350) or 350),
                },
                include_state=False,
            )
            return {
                "ok": True,
                "blocked": False,
                "reason": "",
                "robot": result.get("robot"),
                "state": None,
            }
        poses = payload.get("poses", [])
        check = self.manager.check_path({"name": name, "poses": poses})
        update_payload = {
            "name": name,
            "status": "MANUAL_BLOCKED" if check.get("blocked") else "MANUAL",
            "targetLm": "",
            "currentLm": str(
                payload.get("blockedCurrentLm" if check.get("blocked") else "currentLm")
                or payload.get("currentLm")
                or ""
            ),
        }
        pose_key = "blockedPose" if check.get("blocked") else "nextPose"
        pose = payload.get(pose_key)
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        # Manual commands arrive at 30 Hz while rendering runs at 60 FPS. Do
        # not serialize the complete 50-robot fleet (including routes/orders)
        # into every command response; the fleet websocket remains the source
        # of full snapshots and this response carries only the changed robot.
        result = self.manager.update_robot(update_payload, include_state=False)
        return {
            "ok": True,
            "blocked": bool(check.get("blocked")),
            "reason": str(check.get("reason") or ""),
            "index": check.get("index"),
            "pose": check.get("pose"),
            "robot": result.get("robot"),
            "state": None,
        }

    def manual_stop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        name = str(payload.get("name") or "").strip()
        if not name:
            raise ValueError("robot name is required")
        if self.mode == "robots":
            result = self.manager.teleop_stop_robot({"name": name})
            return {
                "ok": True,
                "robot": result.get("robot"),
                "state": self._state_with_context(result.get("state")),
            }

        update_payload = {
            "name": name,
            "status": "IDLE",
            "targetLm": "",
            "currentLm": str(payload.get("currentLm") or ""),
        }
        pose = payload.get("pose")
        if isinstance(pose, dict):
            update_payload["pose"] = pose
        result = self.manager.update_robot(update_payload)
        return {
            "ok": True,
            "robot": result.get("robot"),
            "state": self._state_with_context(result.get("state")),
        }

    def note_external_control_takeover(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
    ) -> bool:
        if self.mode != "robots":
            return False
        self._sync_manager_mode()
        return self.manager.note_external_control_takeover(
            endpoint,
            owner_id=owner_id,
            owner_name=owner_name,
        )

    def add_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        payload = dict(payload)
        payload["mode"] = "remote" if self.mode == "robots" else "simulated"
        return self._result_with_context(self.manager.add_robot(payload))

    def remove_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.remove_robot(payload))

    def update_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.update_robot(payload))

    def stop_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        include_state = bool(payload.get("includeState", True))
        return self._result_with_context(
            self.manager.stop_robot(payload, include_state=include_state)
        )

    def reset_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._sync_manager_mode()
        return self._result_with_context(self.manager.reset_robot(payload))

    def _clear_simulation_runtime(self) -> None:
        if self.mode != "simulation":
            return
        self.manager.robots.clear()
        self.manager.orders.clear()
        self.manager.events.clear()
        self.manager.reset_planning_runtime_state()
        for key in getattr(self.manager, "traffic_metrics", {}):
            self.manager.traffic_metrics[key] = 0
        self._reset_dynamic_benchmark()

    def _benchmark_sim_robots(self) -> list[Any]:
        return sorted(
            [
                robot
                for robot in self.manager.robots.values()
                if not robot.is_remote()
            ],
            key=lambda robot: str(robot.name),
        )

    def _ensure_benchmark_robots_payload(self, *, count: int, seed: int) -> dict[str, Any]:
        existing = self._benchmark_sim_robots()
        target_count = max(0, count)
        if len(existing) >= target_count:
            state = self.manager.state(include_trajectories=True)
            return self._state_with_context({
                **state,
                "benchmark": {
                    "action": "add",
                    "target": target_count,
                    "robots": len(existing),
                    "added": 0,
                    "seed": seed,
                },
            })

        spawn_lms = self._benchmark_spawn_lms(target_count, seed)
        used_lms = {
            str(robot.current_lm)
            for robot in existing
            if str(robot.current_lm)
        }
        existing_names = set(self.manager.robots)
        next_index = self._next_benchmark_robot_index()
        added = 0
        for lm_name in spawn_lms:
            if len(existing) + added >= target_count:
                break
            if lm_name in used_lms:
                continue
            while f"bench_{next_index:03d}" in existing_names:
                next_index += 1
            name = f"bench_{next_index:03d}"
            self.manager.add_robot({
                "name": name,
                "spawnLm": lm_name,
                "mode": "simulated",
            })
            existing_names.add(name)
            used_lms.add(lm_name)
            next_index += 1
            added += 1

        robots = self._benchmark_sim_robots()
        state = self.manager.state(include_trajectories=True)
        return self._state_with_context({
            **state,
            "benchmark": {
                "action": "add",
                "target": target_count,
                "robots": len(robots),
                "added": added,
                "seed": seed,
            },
        })

    def _start_dynamic_benchmark_payload(
        self,
        *,
        count: int,
        seed: int,
        horizon_sec: float,
        order_interval_sec: float,
        queue_depth: int,
        speed: float,
        acceleration: float,
        rotate: bool,
        turn_speed: float,
        generation_mode: str = "continuous",
    ) -> dict[str, Any]:
        robots = self._benchmark_sim_robots()
        if count > 0:
            robots = robots[:count]
        if not robots:
            raise ValueError("add robots before starting dynamic orders")
        pending_benchmark_orders = [
            order
            for order in self.manager.orders.values()
            if order.order_id.startswith("dynamic-")
            and order.status not in {"COMPLETED", "FAILED", "CANCELED"}
        ]
        if pending_benchmark_orders:
            raise ValueError(
                f"wait for {len(pending_benchmark_orders)} active benchmark order(s) "
                "to finish before starting a new generator"
            )
        # A benchmark session owns its counters and bounded queue history.
        # Leaving terminal dynamic orders from the previous session made the
        # first pump count them as completions of the new run.
        for order_id, order in list(self.manager.orders.items()):
            if (
                order_id.startswith("dynamic-")
                and order.status in {"COMPLETED", "FAILED", "CANCELED"}
            ):
                self.manager.orders.pop(order_id, None)

        requested_horizon_sec = max(1.0, min(120.0, float(horizon_sec)))
        horizon_sec = self._safe_dynamic_rolling_horizon(
            requested_horizon_sec,
            robot_count=len(robots),
        )
        generation_mode = (
            "package_waves"
            if str(generation_mode).strip().lower() == "package_waves"
            else "continuous"
        )
        order_interval_sec = (
            0.0
            if generation_mode == "package_waves"
            else max(0.25, min(120.0, float(order_interval_sec)))
        )
        queue_depth = max(1, min(5, int(queue_depth)))
        fleet = self.manager.params.setdefault("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
            self.manager.params["fleet"] = fleet
        fleet["rolling_horizon_sec"] = horizon_sec
        # ``horizonSec`` controls the committed rolling window only. Keep the
        # independently configured reservation horizon; the fleet runtime
        # already raises it to the longest controlled-corridor traversal plus
        # its safety margin when that topology needs more look-ahead.
        self.manager.reset_planning_runtime_state()
        for key in getattr(self.manager, "traffic_metrics", {}):
            self.manager.traffic_metrics[key] = 0

        now = self._runtime_now()
        self._dynamic_rng = random.Random(seed)
        self._dynamic_benchmark = {
            "active": True,
            "generationMode": generation_mode,
            "scenario": (
                "package_order_waves"
                if generation_mode == "package_waves"
                else "continuous_random_orders"
            ),
            "seed": seed,
            "startedAt": now,
            "stoppedAt": 0.0,
            "horizonSec": horizon_sec,
            "horizonRequestedSec": requested_horizon_sec,
            "orderIntervalSec": order_interval_sec,
            "queueDepth": queue_depth,
            "ordersGenerated": 0,
            "ordersCompleted": 0,
            "ordersTerminated": 0,
            "generationFailures": 0,
            "generatedDistanceMTotal": 0.0,
            "lastOrderDistanceM": 0.0,
            "dispatchElapsedMs": 0.0,
            "lastOrderAt": 0.0,
            "lastPumpAt": now,
            "nextOrderAt": {},
            "orderSequence": 0,
            "sessionId": int(now * 1000),
            "countedTerminalOrders": set(),
            "completedDurationsSec": [],
            "completedDurationTotalSec": 0.0,
            "lastTerminalAt": 0.0,
            "measurementFinishedAt": 0.0,
            "speed": speed,
            "acceleration": acceleration,
            "rotate": rotate,
            "turnSpeed": turn_speed,
            "initialWaveQueued": False,
            "waveOrderIds": set(),
            "waveIndex": 0,
            "wavesStarted": 0,
            "wavesCompleted": 0,
            "waveStartedAt": 0.0,
            "lastWaveDurationSec": 0.0,
            "waveDurationTotalSec": 0.0,
            "packageWaveOrderIds": {},
            "packageWaveRobots": {},
            "packageWaveStartedAt": {},
            "packageCompletedWaves": set(),
            "packageRobotRounds": {},
        }
        # Queue the first order for every robot atomically.  Dispatch is
        # intentionally deferred so MAPF can see small coupled groups instead
        # of treating the rest of the fleet as permanently parked obstacles.
        if generation_mode == "continuous":
            for robot in robots:
                self._dynamic_benchmark["nextOrderAt"][robot.name] = now

        started = perf_counter()
        generated = self._pump_dynamic_benchmark(now=now)
        elapsed_ms = (perf_counter() - started) * 1000.0
        benchmark = self._dynamic_benchmark_payload()
        benchmark.update({
            "action": (
                "start_package_waves"
                if generation_mode == "package_waves"
                else "start_dynamic"
            ),
            "scenario": self._dynamic_benchmark["scenario"],
            "count": len(robots),
            "planned": len(robots),
            "generatedNow": generated,
            "elapsedMs": round(elapsed_ms, 3),
        })
        state = self._state_with_context(self.manager.state(include_trajectories=True))
        return self._result_with_context({
            "ok": True,
            "benchmark": benchmark,
            "state": state,
        })

    def _safe_dynamic_rolling_horizon(
        self,
        requested: float,
        *,
        robot_count: int | None = None,
    ) -> float:
        """Keep a benchmark window schedulable through explicit corridors.

        The topology floor protects one corridor transfer.  A dense fleet also
        needs enough committed time for the serialized planner to prepare all
        future windows before their endpoints synchronize.  This second floor
        is deliberately limited to maps that actually contain authored
        controlled corridors; open maps keep the operator's requested value.
        """
        requested = max(1.0, min(120.0, float(requested)))
        corridor_ticks = self.manager.planner.controlled_corridor_max_ticks()
        if corridor_ticks <= 0:
            return requested
        corridor_sec = (
            corridor_ticks
            * self.manager._reservation_time_step()
        )
        topology_minimum = (
            corridor_sec
            + self.manager._controlled_corridor_entry_lookahead()
            + (2.0 * self.manager._reservation_safety_time())
        )
        # Starting a benchmark writes the effective value back to
        # ``rolling_horizon_sec``.  Reusing that mutable value here made the
        # horizon sticky: after one 30 s run, a later 10 s request still ran at
        # 30 s and paid the larger SIPP search cost forever.  The API request
        # is authoritative; only the map's current corridor safety floor may
        # raise it.
        fleet = self.manager.params.get("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
        try:
            dense_threshold = max(
                2,
                int(
                    fleet.get(
                        "dense_controlled_corridor_robot_threshold",
                        32,
                    )
                    or 32
                ),
            )
        except (TypeError, ValueError):
            dense_threshold = 32
        try:
            dense_minimum = max(
                topology_minimum,
                float(
                    fleet.get(
                        "dense_controlled_corridor_horizon_sec",
                        30.0,
                    )
                    or 30.0
                ),
            )
        except (TypeError, ValueError):
            dense_minimum = max(topology_minimum, 30.0)
        active_robot_count = (
            len(self._benchmark_sim_robots())
            if robot_count is None
            else max(0, int(robot_count))
        )
        if active_robot_count >= dense_threshold:
            topology_minimum = max(topology_minimum, dense_minimum)
        return min(120.0, max(requested, topology_minimum))

    def _stop_dynamic_benchmark_payload(self) -> dict[str, Any]:
        if self._dynamic_benchmark.get("active"):
            self._dynamic_benchmark["active"] = False
            self._dynamic_benchmark["stoppedAt"] = self._runtime_now()
        benchmark = self._dynamic_benchmark_payload()
        benchmark["action"] = "stop_dynamic"
        state = self._state_with_context(self.manager.state(include_trajectories=True))
        return self._result_with_context({
            "ok": True,
            "benchmark": benchmark,
            "state": state,
        })

    def _pump_dynamic_benchmark(self, now: float | None = None) -> int:
        if self.mode != "simulation" or not hasattr(self, "manager"):
            return 0
        now = self._runtime_now() if now is None else float(now)
        config = getattr(self, "_dynamic_benchmark", {})
        self._prune_dynamic_order_history()
        if str(config.get("generationMode") or "continuous") == "package_waves":
            self._finish_terminal_package_waves(now)
            if not config.get("active"):
                config["lastPumpAt"] = now
                return 0
            generated = self._top_up_package_orders(now)
            config["lastPumpAt"] = now
            return generated
        if not config.get("active"):
            config["lastPumpAt"] = now
            return 0
        robots = self._benchmark_sim_robots()
        next_order_at = config.setdefault("nextOrderAt", {})
        interval = float(config.get("orderIntervalSec", 3.0) or 3.0)
        queue_depth = int(config.get("queueDepth", 2) or 2)
        depths = {
            robot.name: self._dynamic_order_depth(robot.name)
            for robot in robots
        }
        due = [
            robot for robot in robots
            if depths[robot.name] < queue_depth
            and (
                depths[robot.name] == 0
                or now >= float(next_order_at.get(robot.name, now))
            )
        ]
        # Coverage is the hard invariant. Queue-depth prefill may stay
        # throttled, but every robot with zero orders is replenished in this
        # same pump instead of waiting behind a two-robot batch limit.
        due.sort(
            key=lambda robot: (
                depths[robot.name] != 0,
                float(next_order_at.get(robot.name, now)),
                robot.name,
            )
        )
        generated = 0
        initial_wave = not bool(config.get("initialWaveQueued", False))
        uncovered_count = sum(depths[robot.name] == 0 for robot in due)
        batch_limit = (
            len(due)
            if initial_wave
            else min(
                len(due),
                uncovered_count + self._dynamic_generation_batch_size(),
            )
        )
        for robot in due[:batch_limit]:
            started = perf_counter()
            try:
                order_payload = self._next_dynamic_order_payload(robot, now)
                if order_payload is None:
                    config["generationFailures"] = int(config.get("generationFailures", 0) or 0) + 1
                    self.manager._event(
                        "warn",
                        f"dynamic order pending for {robot.name}: no free reachable LM",
                    )
                else:
                    self.manager.set_order(order_payload, dispatch=False)
                    generated += 1
                    self._record_generated_dynamic_order(order_payload, now)
            except (RuntimeError, ValueError) as exc:
                config["generationFailures"] = int(config.get("generationFailures", 0) or 0) + 1
                self.manager._event(
                    "warn",
                    f"dynamic order pending for {robot.name}: {exc}",
                )
            finally:
                config["dispatchElapsedMs"] = float(config.get("dispatchElapsedMs", 0.0) or 0.0) + (
                    (perf_counter() - started) * 1000.0
                )
            jitter = self._dynamic_rng.uniform(0.70, 1.30)
            next_order_at[robot.name] = now + (interval * jitter)
        if initial_wave:
            config["initialWaveQueued"] = True
        config["lastPumpAt"] = now
        self._prune_dynamic_order_history()
        return generated

    def _finish_terminal_package_waves(self, now: float) -> int:
        config = self._dynamic_benchmark
        wave_orders = config.setdefault("packageWaveOrderIds", {})
        wave_robots = config.setdefault("packageWaveRobots", {})
        wave_started = config.setdefault("packageWaveStartedAt", {})
        completed_waves = config.setdefault("packageCompletedWaves", set())
        completed_wave_indices: set[int] = set()
        for raw_index in completed_waves:
            try:
                completed_wave_indices.add(int(raw_index))
            except (TypeError, ValueError):
                continue
        completed_total = int(config.get("wavesCompleted", 0) or 0)
        expected_robots = len(self._benchmark_sim_robots())
        completed_now = 0
        finished_indices: set[int] = set()
        for raw_index in sorted(wave_orders, key=int):
            wave_index = int(raw_index)
            if wave_index in completed_wave_indices:
                finished_indices.add(wave_index)
                continue
            order_ids = set(wave_orders.get(raw_index, set()))
            robot_names = set(wave_robots.get(raw_index, set()))
            if (
                expected_robots <= 0
                or len(order_ids) != expected_robots
                or len(robot_names) != expected_robots
            ):
                continue
            if any(
                order_id in self.manager.orders
                and self.manager.orders[order_id].status
                not in {"COMPLETED", "FAILED", "CANCELED"}
                for order_id in order_ids
            ):
                continue
            started_at = float(wave_started.get(raw_index, now) or now)
            duration = max(0.0, now - started_at)
            completed_waves.add(wave_index)
            completed_wave_indices.add(wave_index)
            finished_indices.add(wave_index)
            completed_now += 1
            config["lastWaveDurationSec"] = duration
            config["waveDurationTotalSec"] = float(
                config.get("waveDurationTotalSec", 0.0) or 0.0
            ) + duration
            self.manager._event(
                "info",
                f"package wave {wave_index} completed: "
                f"{len(order_ids)} orders in {duration:.1f} simulated seconds",
            )
        config["wavesCompleted"] = completed_total + completed_now
        # The aggregate counters above are the benchmark record. Keeping all
        # per-wave order-id/name sets forever made this 10 Hz method scan the
        # complete run history after hours of operation.
        for wave_index in finished_indices:
            for mapping in (wave_orders, wave_robots, wave_started):
                mapping.pop(wave_index, None)
                mapping.pop(str(wave_index), None)
            completed_waves.discard(wave_index)
            completed_waves.discard(str(wave_index))
        return completed_now

    def _top_up_package_orders(self, now: float) -> int:
        config = self._dynamic_benchmark
        robots = [
            robot
            for robot in self._benchmark_sim_robots()
            if self._dynamic_order_depth(robot.name) == 0
        ]
        if not robots:
            return 0
        robot_rounds = config.setdefault("packageRobotRounds", {})
        by_wave: dict[int, list[Any]] = {}
        for robot in robots:
            wave_index = int(robot_rounds.get(robot.name, 0) or 0) + 1
            by_wave.setdefault(wave_index, []).append(robot)
        return sum(
            self._generate_package_orders_for_wave(group, wave_index, now)
            for wave_index, group in sorted(by_wave.items())
        )

    def _generate_package_order_wave(self, now: float) -> int:
        """Backward-compatible entry point for package coverage generation."""
        return self._top_up_package_orders(now)

    def _generate_package_orders_for_wave(
        self,
        robots: list[Any],
        wave_index: int,
        now: float,
    ) -> int:
        config = self._dynamic_benchmark
        assignments = self._package_wave_assignments(robots, wave_index)
        if len(assignments) != len(robots):
            last_failure = float(config.get("lastWaveFailureAt", 0.0) or 0.0)
            if now - last_failure >= 5.0:
                self.manager._event(
                    "warn",
                    f"package wave {wave_index} coverage pending: assigned "
                    f"{len(assignments)}/{len(robots)} peripheral goals",
                )
                config["lastWaveFailureAt"] = now
            config["generationFailures"] = int(
                config.get("generationFailures", 0) or 0
            ) + max(1, len(robots) - len(assignments))
            return 0

        wave_orders = config.setdefault("packageWaveOrderIds", {})
        wave_robots = config.setdefault("packageWaveRobots", {})
        wave_started = config.setdefault("packageWaveStartedAt", {})
        robot_rounds = config.setdefault("packageRobotRounds", {})
        order_ids = wave_orders.setdefault(wave_index, set())
        robot_names = wave_robots.setdefault(wave_index, set())
        new_wave = not order_ids
        generated = 0
        started = perf_counter()
        for index, (robot, target_lm) in enumerate(assignments):
            priority = (wave_index + index) % 3
            order_payload = self._dynamic_order_payload(
                robot,
                target_lm,
                now,
                priority=priority,
                external_prefix=f"package-wave-{wave_index}",
            )
            try:
                self.manager.set_order(order_payload, dispatch=False)
            except (RuntimeError, ValueError) as exc:
                config["generationFailures"] = int(
                    config.get("generationFailures", 0) or 0
                ) + 1
                self.manager._event(
                    "warn",
                    f"package order pending for {robot.name}: {exc}",
                )
                continue
            order_ids.add(str(order_payload["id"]))
            robot_names.add(str(robot.name))
            robot_rounds[str(robot.name)] = wave_index
            generated += 1
            self._record_generated_dynamic_order(order_payload, now)
        config["dispatchElapsedMs"] = float(
            config.get("dispatchElapsedMs", 0.0) or 0.0
        ) + ((perf_counter() - started) * 1000.0)
        if generated != len(robots):
            # Keep the partial wave authoritative. It must drain before a new
            # wave is attempted; duplicate orders are never generated for the
            # robots that were successfully queued.
            config["generationFailures"] = int(
                config.get("generationFailures", 0) or 0
            ) + (len(robots) - generated)
        if generated and new_wave:
            wave_started[wave_index] = now
            config["wavesStarted"] = int(config.get("wavesStarted", 0) or 0) + 1
        config["waveIndex"] = max(
            int(config.get("waveIndex", 0) or 0),
            wave_index,
        )
        config["waveStartedAt"] = float(wave_started.get(wave_index, now) or now)
        config["waveOrderIds"] = set(order_ids)
        config["initialWaveQueued"] = True
        if generated:
            self.manager._event(
                "info",
                f"package wave {wave_index} coverage queued: "
                f"{generated}/{len(robots)} orders to map perimeter",
            )
        return generated

    def _next_dynamic_order_payload(self, robot: Any, now: float) -> dict[str, Any] | None:
        config = self._dynamic_benchmark
        origin = self._dynamic_order_origin(robot.name) or str(robot.current_lm)
        if origin not in self.loaded_map.landmarks:
            return None
        used_goals = {
            str(order.target_lm)
            for order in self.manager.orders.values()
            if order.status not in {"COMPLETED", "FAILED", "CANCELED"}
            and str(order.target_lm) in self.loaded_map.landmarks
        }
        occupied_lms = {
            str(item.current_lm)
            for item in self._benchmark_sim_robots()
            if item.name != robot.name and str(item.current_lm) in self.loaded_map.landmarks
        }
        min_hops, max_hops = self._dynamic_goal_hop_window()
        candidates = self._forward_benchmark_goals(
            origin,
            used_goals,
            occupied_lms,
            self._dynamic_rng,
            min_hops=min_hops,
            max_hops=max_hops,
        )
        if not candidates:
            candidates = self._forward_benchmark_goals(
                origin,
                used_goals,
                occupied_lms,
                self._dynamic_rng,
                min_hops=max(2, min_hops // 3),
                max_hops=min(200, max(max_hops, len(self.loaded_map.landmarks))),
            )
        if not candidates:
            return None
        target_lm = self._far_dynamic_goal(origin, candidates)
        sequence = int(config.get("orderSequence", 0) or 0) + 1
        priority = self._dynamic_rng.choice((0, 0, 1, 1, 2, 3))
        if sequence % 10 == 0:
            priority = 5
        return self._dynamic_order_payload(
            robot,
            target_lm,
            now,
            priority=priority,
            external_prefix="benchmark",
        )

    def _dynamic_order_payload(
        self,
        robot: Any,
        target_lm: str,
        now: float,
        *,
        priority: int,
        external_prefix: str,
    ) -> dict[str, Any]:
        config = self._dynamic_benchmark
        origin = self._dynamic_order_origin(robot.name) or str(robot.current_lm)
        origin_lm = self.loaded_map.landmarks[origin]
        target = self.loaded_map.landmarks[target_lm]
        distance_m = math.hypot(target.x - origin_lm.x, target.y - origin_lm.y)
        sequence = int(config.get("orderSequence", 0) or 0) + 1
        config["orderSequence"] = sequence
        return {
            "id": f"dynamic-{int(config.get('sessionId', 0) or 0)}-{sequence:07d}-{robot.name}",
            "vehicle": robot.name,
            "targetLm": target_lm,
            "priority": priority,
            "speed": float(config.get("speed", 0.0) or 0.0),
            "acceleration": float(config.get("acceleration", 0.0) or 0.0),
            "rotate": bool(config.get("rotate", False)),
            "turnSpeed": float(config.get("turnSpeed", 0.0) or 0.0),
            "stretchMotionToReservationTicks": True,
            "externalId": f"{external_prefix}-{int(now * 1000)}-{sequence}",
            "benchmarkDistanceM": round(distance_m, 3),
        }

    def _record_generated_dynamic_order(
        self,
        order_payload: dict[str, Any],
        now: float,
    ) -> None:
        config = self._dynamic_benchmark
        config["ordersGenerated"] = int(config.get("ordersGenerated", 0) or 0) + 1
        distance_m = float(order_payload.get("benchmarkDistanceM", 0.0) or 0.0)
        config["generatedDistanceMTotal"] = float(
            config.get("generatedDistanceMTotal", 0.0) or 0.0
        ) + distance_m
        config["lastOrderDistanceM"] = distance_m
        config["lastOrderAt"] = now
        config["measurementFinishedAt"] = 0.0

    def _dynamic_goal_hop_window(self) -> tuple[int, int]:
        fleet = self.manager.params.get("fleet", {})
        if not isinstance(fleet, dict):
            return 30, 160
        try:
            minimum = max(2, int(fleet.get("dynamic_order_min_hops", 30) or 30))
        except (TypeError, ValueError):
            minimum = 30
        try:
            maximum = max(minimum, int(fleet.get("dynamic_order_max_hops", 160) or 160))
        except (TypeError, ValueError):
            maximum = 160
        return minimum, maximum

    def _far_dynamic_goal(self, origin: str, candidates: list[str]) -> str:
        start = self.loaded_map.landmarks[origin]
        ranked = sorted(
            candidates,
            key=lambda name: math.hypot(
                self.loaded_map.landmarks[name].x - start.x,
                self.loaded_map.landmarks[name].y - start.y,
            ),
        )
        fleet = self.manager.params.get("fleet", {})
        try:
            fraction = float(fleet.get("dynamic_order_far_fraction", 0.08) or 0.08)
        except (AttributeError, TypeError, ValueError):
            fraction = 0.08
        fraction = max(0.05, min(1.0, fraction))
        pool_size = max(1, int(math.ceil(len(ranked) * fraction)))
        return self._dynamic_rng.choice(ranked[-pool_size:])

    def _package_wave_assignments(
        self,
        robots: list[Any],
        wave_index: int,
    ) -> list[tuple[Any, str]]:
        peripheral = self._benchmark_peripheral_lms(len(robots))
        if not peripheral:
            return []
        perimeter_rank = {name: index for index, name in enumerate(peripheral)}
        occupied_lms = {
            str(robot.current_lm)
            for robot in self._benchmark_sim_robots()
            if str(robot.current_lm) in self.loaded_map.landmarks
        }
        used_goals = {
            str(order.target_lm)
            for order in self.manager.orders.values()
            if order.status not in {"COMPLETED", "FAILED", "CANCELED"}
            and str(order.target_lm) in self.loaded_map.landmarks
        }
        assignments: list[tuple[Any, str]] = []
        min_hops, max_hops = self._dynamic_goal_hop_window()
        robot_count = max(1, len(robots))
        departure_names = {
            str(robot.name)
            for robot in robots
        }
        coordinated_departure_lms = {
            str(robot.current_lm)
            for robot in self._benchmark_sim_robots()
            if (
                str(robot.name) in departure_names
                and str(robot.current_lm) in self.loaded_map.landmarks
            )
        }
        # A target occupied by a robot outside this exact departure cohort
        # cannot become free as a consequence of the generated batch. Never
        # drop all occupied-LM exclusions merely because the perimeter is
        # full: that creates an impossible order behind a parked robot and a
        # permanent plan/replan loop. Cohort members may still exchange their
        # current portals as one coordinated permutation.
        excluded_goal_lms = occupied_lms - coordinated_departure_lms
        # A perimeter-only Kiva layout may have fewer than two parking portals
        # per robot. In that case the wave is a coordinated permutation:
        # targets may be another wave robot's current portal because that
        # owner receives its departure in the same atomic batch. Unique goals
        # and the collision planner still prevent two robots sharing a portal.
        # Half-slot rotation prevents every wave from assigning the same edge
        # cells to the same robot while keeping targets uniformly distributed.
        wave_phase = (max(0, wave_index - 1) * 0.5) % robot_count
        for index, robot in enumerate(sorted(robots, key=lambda item: str(item.name))):
            origin = self._dynamic_order_origin(robot.name) or str(robot.current_lm)
            if origin not in self.loaded_map.landmarks:
                continue
            reachable = self._forward_benchmark_goals(
                origin,
                used_goals,
                excluded_goal_lms,
                self._dynamic_rng,
                min_hops=min_hops,
                max_hops=max_hops,
            )
            candidates = [name for name in reachable if name in perimeter_rank]
            if not candidates:
                reachable = self._forward_benchmark_goals(
                    origin,
                    used_goals,
                    excluded_goal_lms,
                    self._dynamic_rng,
                    min_hops=2,
                    max_hops=min(300, max(max_hops, len(self.loaded_map.landmarks))),
                )
                candidates = [name for name in reachable if name in perimeter_rank]
            if not candidates:
                continue

            desired_fraction = ((index + wave_phase) % robot_count) / robot_count
            desired_rank = desired_fraction * len(peripheral)
            origin_lm = self.loaded_map.landmarks[origin]

            def candidate_key(name: str) -> tuple[float, float, str]:
                rank = float(perimeter_rank[name])
                rank_distance = abs(rank - desired_rank)
                circular_distance = min(
                    rank_distance,
                    len(peripheral) - rank_distance,
                )
                target = self.loaded_map.landmarks[name]
                distance = math.hypot(target.x - origin_lm.x, target.y - origin_lm.y)
                return circular_distance, -distance, name

            target_lm = min(candidates, key=candidate_key)
            assignments.append((robot, target_lm))
            used_goals.add(target_lm)
        return assignments

    def _benchmark_peripheral_lms(self, robot_count: int) -> list[str]:
        names = [
            name
            for name in self._largest_benchmark_component()
            if (
                name in self.loaded_map.landmarks
                and self._benchmark_goal_lm_is_safe(name)
            )
        ]
        if not names:
            return []
        landmarks = self.loaded_map.landmarks
        min_x = min(landmarks[name].x for name in names)
        max_x = max(landmarks[name].x for name in names)
        min_y = min(landmarks[name].y for name in names)
        max_y = max(landmarks[name].y for name in names)
        width = max(0.001, max_x - min_x)
        height = max(0.001, max_y - min_y)

        def edge_distance(name: str) -> float:
            lm = landmarks[name]
            return min(
                lm.x - min_x,
                max_x - lm.x,
                lm.y - min_y,
                max_y - lm.y,
            )

        # Use a broad outer ring so 100-robot waves still have enough unique,
        # footprint-separated destinations after excluding occupied cells.
        pool_size = min(len(names), max(64, int(robot_count) * 8))
        by_edge_distance = sorted(
            names,
            key=lambda name: (edge_distance(name), name),
        )
        distance_limit = edge_distance(by_edge_distance[pool_size - 1])
        # Include the complete distance band at the cutoff. Without this, a
        # lexicographic tie on the outermost row could select the top edge but
        # accidentally omit the bottom edge from small waves.
        outer_ring = [
            name
            for name in by_edge_distance
            if edge_distance(name) <= distance_limit + 0.000001
        ]

        def perimeter_position(name: str) -> float:
            lm = landmarks[name]
            distances = (
                (abs(lm.y - min_y), lm.x - min_x),
                (abs(lm.x - max_x), width + (lm.y - min_y)),
                (abs(lm.y - max_y), width + height + (max_x - lm.x)),
                (abs(lm.x - min_x), (2.0 * width) + height + (max_y - lm.y)),
            )
            return min(distances, key=lambda item: item[0])[1]

        return sorted(outer_ring, key=lambda name: (perimeter_position(name), name))

    def _dynamic_order_origin(self, robot_name: str) -> str:
        orders = [
            order for order in self.manager.orders.values()
            if (order.vehicle == robot_name or order.assigned_robot == robot_name)
            and order.status not in {"COMPLETED", "FAILED", "CANCELED"}
        ]
        if not orders:
            robot = self.manager.robots.get(robot_name)
            return str(robot.route_final_lm or robot.current_lm) if robot is not None else ""
        orders.sort(key=lambda order: (order.created_at, order.order_id))
        return str(orders[-1].target_lm)

    def _dynamic_order_depth(self, robot_name: str) -> int:
        return sum(
            1 for order in self.manager.orders.values()
            if (order.vehicle == robot_name or order.assigned_robot == robot_name)
            and order.status not in {"COMPLETED", "FAILED", "CANCELED"}
        )

    def _dynamic_generation_batch_size(self) -> int:
        # Bound synchronous MAPF work per web tick. Orders still arrive for
        # every robot, but the status stream never blocks on a fleet-wide burst.
        return 2

    def _prune_dynamic_order_history(self) -> None:
        config = self._dynamic_benchmark
        counted = config.setdefault("countedTerminalOrders", set())
        session_prefix = (
            f"dynamic-{int(config.get('sessionId', 0) or 0)}-"
        )
        terminal = [
            order for order in self.manager.orders.values()
            if order.order_id.startswith(session_prefix)
            and order.status in {"COMPLETED", "FAILED", "CANCELED"}
        ]
        terminal.sort(key=lambda order: (order.updated_at, order.order_id), reverse=True)
        for order in terminal:
            if order.order_id in counted:
                continue
            counted.add(order.order_id)
            if order.status == "COMPLETED":
                config["ordersCompleted"] = int(config.get("ordersCompleted", 0) or 0) + 1
                duration = max(0.0, float(order.updated_at) - float(order.created_at))
                durations = config.setdefault("completedDurationsSec", [])
                if isinstance(durations, list):
                    durations.append(duration)
                    del durations[:-1000]
                config["completedDurationTotalSec"] = float(
                    config.get("completedDurationTotalSec", 0.0) or 0.0
                ) + duration
            else:
                config["ordersTerminated"] = int(config.get("ordersTerminated", 0) or 0) + 1
            config["lastTerminalAt"] = max(
                float(config.get("lastTerminalAt", 0.0) or 0.0),
                float(order.updated_at),
            )
        # The operator queue exposes at most 120 records. Retaining additional
        # terminal benchmark orders only increases every lifecycle scan.
        for order in terminal[120:]:
            self.manager.orders.pop(order.order_id, None)
            counted.discard(order.order_id)

    def _dynamic_benchmark_payload(self) -> dict[str, Any]:
        config = getattr(self, "_dynamic_benchmark", {})
        generated = int(config.get("ordersGenerated", 0) or 0)
        session_prefix = (
            f"dynamic-{int(config.get('sessionId', 0) or 0)}-"
        )
        dynamic_orders = [
            order for order in self.manager.orders.values()
            if order.order_id.startswith(session_prefix)
        ] if hasattr(self, "manager") else []
        self._prune_dynamic_order_history()
        completed = int(config.get("ordersCompleted", 0) or 0)
        queued = sum(order.status == "QUEUED" for order in dynamic_orders)
        executing = sum(order.status not in {"QUEUED", "COMPLETED", "FAILED", "CANCELED"} for order in dynamic_orders)
        benchmark_robots = self._benchmark_sim_robots() if hasattr(self, "manager") else []
        waiting = sum(robot.status == "WAITING" for robot in benchmark_robots)
        robots_with_orders = {
            str(order.vehicle or order.assigned_robot)
            for order in dynamic_orders
            if order.status not in {"COMPLETED", "FAILED", "CANCELED"}
            and str(order.vehicle or order.assigned_robot)
        }
        robots_with_orders &= {str(robot.name) for robot in benchmark_robots}
        traffic = dict(getattr(self.manager, "traffic_metrics", {})) if hasattr(self, "manager") else {}
        dispatch_ms = float(config.get("dispatchElapsedMs", 0.0) or 0.0)
        distance_total = float(config.get("generatedDistanceMTotal", 0.0) or 0.0)
        terminated = int(config.get("ordersTerminated", 0) or 0)
        outstanding = max(0, generated - completed - terminated)
        started_at = float(config.get("startedAt", 0.0) or 0.0)
        now = self._runtime_now() if started_at > 0.0 else 0.0
        if not config.get("active") and outstanding <= 0 and generated > 0:
            finished_at = float(config.get("measurementFinishedAt", 0.0) or 0.0)
            if finished_at <= 0.0:
                finished_at = max(
                    started_at,
                    float(config.get("lastTerminalAt", 0.0) or 0.0),
                    float(config.get("stoppedAt", 0.0) or 0.0),
                )
                config["measurementFinishedAt"] = finished_at
        else:
            finished_at = 0.0
        elapsed_sim_sec = max(
            0.0,
            (finished_at or now or started_at) - started_at,
        ) if started_at > 0.0 else 0.0
        throughput = (
            (completed * 60.0) / elapsed_sim_sec
            if completed > 0 and elapsed_sim_sec > 0.000001
            else 0.0
        )
        completed_durations = [
            float(value)
            for value in config.get("completedDurationsSec", [])
            if isinstance(value, (int, float))
        ]
        completed_durations.sort()
        p95_duration = (
            completed_durations[
                min(
                    len(completed_durations) - 1,
                    max(0, int(math.ceil(len(completed_durations) * 0.95)) - 1),
                )
            ]
            if completed_durations
            else 0.0
        )
        waves_completed = int(config.get("wavesCompleted", 0) or 0)
        return {
            "active": bool(config.get("active", False)),
            "generationMode": str(config.get("generationMode") or "continuous"),
            "scenario": str(config.get("scenario") or "continuous_random_orders"),
            "seed": int(config.get("seed", 42) or 42),
            "horizonSec": float(config.get("horizonSec", 10.0) or 10.0),
            "effectiveHorizonSec": round(
                float(self.manager._rolling_horizon()),
                3,
            ),
            "horizonRequestedSec": float(
                config.get(
                    "horizonRequestedSec",
                    config.get("horizonSec", 10.0),
                )
                or 10.0
            ),
            "orderIntervalSec": float(config.get("orderIntervalSec", 3.0) or 3.0),
            "queueDepth": int(config.get("queueDepth", 2) or 2),
            "ordersGenerated": generated,
            "ordersCompleted": completed,
            "ordersQueued": queued,
            "ordersExecuting": executing,
            "ordersOutstanding": outstanding,
            "robotsWithOrders": len(robots_with_orders),
            "robotsWithoutOrders": max(
                0,
                len(benchmark_robots) - len(robots_with_orders),
            ),
            "waitingRobots": waiting,
            "generationFailures": int(config.get("generationFailures", 0) or 0),
            "averageDispatchMs": round(dispatch_ms / generated, 3) if generated else 0.0,
            "averageOrderDistanceM": round(distance_total / generated, 2) if generated else 0.0,
            "lastOrderDistanceM": round(float(config.get("lastOrderDistanceM", 0.0) or 0.0), 2),
            "elapsedSimSec": round(elapsed_sim_sec, 3),
            "throughputOrdersPerMin": round(throughput, 3),
            "averageOrderDurationSec": round(
                float(config.get("completedDurationTotalSec", 0.0) or 0.0) / completed,
                3,
            ) if completed else 0.0,
            "p95OrderDurationSec": round(p95_duration, 3),
            "waveIndex": int(config.get("waveIndex", 0) or 0),
            "wavesStarted": int(config.get("wavesStarted", 0) or 0),
            "wavesCompleted": waves_completed,
            "waveOrders": len(set(config.get("waveOrderIds", set()))),
            "lastWaveDurationSec": round(
                float(config.get("lastWaveDurationSec", 0.0) or 0.0),
                3,
            ),
            "averageWaveDurationSec": round(
                float(config.get("waveDurationTotalSec", 0.0) or 0.0)
                / waves_completed,
                3,
            ) if waves_completed else 0.0,
            "timeScale": self.manager.simulation_time_scale() if hasattr(self, "manager") else 1.0,
            **traffic,
        }

    def _runtime_now(self) -> float:
        if self.mode == "simulation" and hasattr(self, "manager"):
            return self.manager.simulation_time()
        return time()

    def _reset_dynamic_benchmark(self) -> None:
        self._dynamic_rng = random.Random(42)
        self._dynamic_benchmark = {
            "active": False,
            "generationMode": "continuous",
            "scenario": "continuous_random_orders",
            "seed": 42,
            "horizonSec": 10.0,
            "orderIntervalSec": 3.0,
            "queueDepth": 2,
            "ordersGenerated": 0,
            "ordersCompleted": 0,
            "ordersTerminated": 0,
            "generationFailures": 0,
            "generatedDistanceMTotal": 0.0,
            "lastOrderDistanceM": 0.0,
            "dispatchElapsedMs": 0.0,
            "nextOrderAt": {},
            "orderSequence": 0,
            "sessionId": 0,
            "countedTerminalOrders": set(),
            "completedDurationsSec": [],
            "completedDurationTotalSec": 0.0,
            "lastTerminalAt": 0.0,
            "measurementFinishedAt": 0.0,
            "initialWaveQueued": False,
            "waveOrderIds": set(),
            "waveIndex": 0,
            "wavesStarted": 0,
            "wavesCompleted": 0,
            "waveStartedAt": 0.0,
            "lastWaveDurationSec": 0.0,
            "waveDurationTotalSec": 0.0,
            "packageWaveOrderIds": {},
            "packageWaveRobots": {},
            "packageWaveStartedAt": {},
            "packageCompletedWaves": set(),
            "packageRobotRounds": {},
        }

    def _benchmark_spawn_lms(self, count: int, seed: int) -> list[str]:
        names = [
            name
            for name in self._largest_benchmark_component()
            if (
                self._benchmark_spawn_lm_is_safe(name)
                and self._benchmark_wait_lm_is_safe(name)
            )
        ]
        if len(names) < count:
            raise ValueError(
                f"add robots needs at least {count} collision-safe connected LMs; "
                f"largest component has {len(names)} safe spawn positions"
            )
        rng = random.Random(seed + 7919)
        shuffled = self._corridor_safe_benchmark_lms(names, rng)
        spaced = self._spatially_separated_lms(shuffled, count)
        if len(spaced) < count:
            raise ValueError(
                f"map can safely place only {len(spaced)} of {count} robots "
                f"with {self._benchmark_min_separation():.2f} m center spacing"
            )
        return spaced

    def _benchmark_spawn_lm_is_safe(self, name: str) -> bool:
        landmark = self.loaded_map.landmarks.get(name)
        if landmark is None:
            return False
        return not self.manager.collision.blocked_reason(
            {
                "x": float(landmark.x),
                "y": float(landmark.y),
                "yaw": 0.0,
            },
            self.manager.obstacles,
            self.manager.obstacle_areas,
        )

    def _benchmark_corridor_region(self, name: str) -> str:
        graph = getattr(self.manager, "_controlled_corridor_graph", None)
        vertex = graph.vertices.get(name) if graph is not None else None
        if vertex is None or not vertex.controlled_region_ids:
            return ""
        return sorted(vertex.controlled_region_ids)[0]

    def _benchmark_goal_lm_is_safe(self, name: str) -> bool:
        """Keep benchmark parking destinations out of traffic bottlenecks."""
        if not self._benchmark_wait_lm_is_safe(name):
            return False
        graph = getattr(self.manager, "_controlled_corridor_graph", None)
        vertex = graph.vertices.get(name) if graph is not None else None
        if vertex is None:
            return True
        # On Kiva maps, no-wait aisle chains terminate at shared junctions.
        # Parking a completed wave robot on an internal four-way junction
        # removes a transit vertex and can disconnect every remaining route.
        # Degree-three perimeter portals are still graph-safe wait points and
        # leave the inner cross-aisles available to unfinished orders.
        neighbours = self.manager.planner.graph.get(name, {})
        return len(neighbours) <= 3

    def _benchmark_wait_lm_is_safe(self, name: str) -> bool:
        graph = getattr(self.manager, "_controlled_corridor_graph", None)
        vertex = graph.vertices.get(name) if graph is not None else None
        if vertex is None:
            return True
        # Auto-controlled corridors expose capacity-one stop-line LMs just
        # before each junction. They are legal holding/spawn points because a
        # robot there owns the whole corridor mutex; the benchmark's
        # corridor-safe selector below still places at most one robot in each
        # such region.
        return bool(vertex.can_wait)

    def _corridor_safe_benchmark_lms(
        self,
        names: list[str],
        rng: random.Random,
    ) -> list[str]:
        """Prefer holding points and place at most one robot inside a corridor."""
        if getattr(self.manager, "_controlled_corridor_graph", None) is None:
            shuffled = list(names)
            rng.shuffle(shuffled)
            return shuffled
        holding: list[str] = []
        inside_by_region: dict[str, list[str]] = {}
        for name in names:
            region_id = self._benchmark_corridor_region(name)
            if region_id:
                inside_by_region.setdefault(region_id, []).append(name)
            else:
                holding.append(name)
        rng.shuffle(holding)
        region_ids = list(inside_by_region)
        rng.shuffle(region_ids)
        representatives: list[str] = []
        for region_id in region_ids:
            candidates = inside_by_region[region_id]
            rng.shuffle(candidates)
            representatives.append(candidates[0])
        return holding + representatives

    def _next_benchmark_robot_index(self) -> int:
        max_index = 0
        for name in self.manager.robots:
            value = str(name)
            if not value.startswith("bench_"):
                continue
            try:
                max_index = max(max_index, int(value.split("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        return max_index + 1

    def _apply_fast_benchmark_params(self, count: int = 0) -> dict[str, Any] | None:
        params = getattr(self.manager, "params", None)
        if not isinstance(params, dict):
            return None
        fleet = params.setdefault("fleet", {})
        if not isinstance(fleet, dict):
            fleet = {}
            params["fleet"] = fleet
        previous = {
            "batch_collision_horizon_sec": fleet.get("batch_collision_horizon_sec"),
            "batch_wait_max_iterations": fleet.get("batch_wait_max_iterations"),
            "continuous_collision_step_sec": fleet.get("continuous_collision_step_sec"),
            "stretch_motion_to_reservation_ticks": fleet.get("stretch_motion_to_reservation_ticks"),
            "_planner_backend": getattr(getattr(self.manager, "planner", None), "planner_backend", None),
            "_planner_stretch_motion_to_reservation_ticks": getattr(
                getattr(self.manager, "planner", None),
                "stretch_motion_to_reservation_ticks",
                None,
            ),
        }
        if count >= 100:
            horizon = 8.0
            iterations = 8
        elif count >= 50:
            horizon = 10.0
            iterations = 10
        else:
            # The interactive 20-robot traffic scenario uses long routes.
            # Validate their complete time horizon so late corridor conflicts
            # cannot hide behind the benchmark speed optimization.
            horizon = None
            iterations = 60
        fleet["batch_collision_horizon_sec"] = horizon
        fleet["batch_wait_max_iterations"] = iterations
        fleet["continuous_collision_step_sec"] = 0.10
        fleet["stretch_motion_to_reservation_ticks"] = True
        planner = getattr(self.manager, "planner", None)
        if planner is not None:
            planner.planner_backend = "hybrid"
            planner.stretch_motion_to_reservation_ticks = True
        return previous

    def _restore_fleet_params(self, previous: dict[str, Any]) -> None:
        params = getattr(self.manager, "params", None)
        if not isinstance(params, dict):
            return
        fleet = params.setdefault("fleet", {})
        if not isinstance(fleet, dict):
            return
        for key, value in previous.items():
            if key == "_planner_backend":
                planner = getattr(self.manager, "planner", None)
                if planner is not None and value is not None:
                    planner.planner_backend = value
                continue
            if key == "_planner_stretch_motion_to_reservation_ticks":
                planner = getattr(self.manager, "planner", None)
                if planner is not None and value is not None:
                    planner.stretch_motion_to_reservation_ticks = value
                continue
            if value is None:
                fleet.pop(key, None)
            else:
                fleet[key] = value

    def _benchmark_requests(
        self,
        *,
        count: int,
        seed: int,
        stress: bool = False,
        stress_profile: int = 0,
    ) -> list[dict[str, Any]]:
        names = self._largest_benchmark_component()
        if len(names) < count * 2:
            raise ValueError(
                f"benchmark needs at least {count * 2} connected LMs; "
                f"largest component has {len(names)}"
            )
        rng = random.Random(seed + count)
        shuffled = self._corridor_safe_benchmark_lms(names, rng)
        starts = self._spatially_separated_lms(shuffled, count)
        if len(starts) < count:
            raise ValueError(
                f"benchmark can safely place only {len(starts)} of {count} robots"
            )
        used_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        start_set = set(starts)
        for index, start_name in enumerate(starts, start=1):
            candidates = self._forward_benchmark_goals(
                start_name,
                used_goals,
                start_set,
                random.Random(seed + (index * 1009)),
                **self._traffic_goal_window(stress, stress_profile, count),
            )
            if not candidates:
                raise ValueError("not enough physically separated benchmark goals")
            if stress:
                goal_name = self._traffic_goal_from_candidates(
                    candidates,
                    stress_profile,
                    count,
                )
            else:
                goal_name = candidates[0]
            used_goals.add(goal_name)
            requests.append({
                "name": f"bench_{index:03d}",
                "startLm": start_name,
                "goalLm": goal_name,
            })
        return requests

    def _benchmark_requests_for_existing(
        self,
        *,
        count: int,
        seed: int,
        stress: bool = False,
        stress_profile: int = 0,
    ) -> list[dict[str, Any]]:
        robots = self._benchmark_sim_robots()
        if count > 0:
            robots = robots[:count]
        if not robots:
            raise ValueError("add robots before planning")

        names = self._largest_benchmark_component()
        if len(names) < len(robots):
            raise ValueError(
                f"planning needs at least {len(robots)} connected LMs; "
                f"largest component has {len(names)}"
            )
        name_set = set(names)
        planning_starts: dict[str, str] = {}
        for robot in robots:
            start_lm = self.manager._safe_replan_start_lm(robot)
            if start_lm not in name_set:
                raise ValueError(
                    f"{robot.name} is between graph landmarks; "
                    "wait until it reaches the next LM before planning"
                )
            planning_starts[str(robot.name)] = start_lm
        start_lms = set(planning_starts.values())
        avoid_start_goals = len(names) >= len(robots) * 2
        used_goals: set[str] = set()
        requests: list[dict[str, Any]] = []
        for index, robot in enumerate(robots, start=1):
            start_lm = planning_starts[str(robot.name)]
            candidates = self._forward_benchmark_goals(
                start_lm,
                used_goals,
                start_lms if avoid_start_goals else set(),
                random.Random(seed + (index * 1009)),
                **self._traffic_goal_window(
                    stress,
                    stress_profile,
                    len(robots),
                ),
            )
            if not candidates:
                candidates = [
                    name
                    for name in names
                    if name != start_lm
                    and name not in used_goals
                    and self._lm_is_separated_from(name, used_goals)
                    and (
                        not self._benchmark_corridor_region(name)
                        or self._benchmark_corridor_region(name)
                        not in {
                            self._benchmark_corridor_region(goal)
                            for goal in used_goals
                            if self._benchmark_corridor_region(goal)
                        }
                    )
                ]
            if not candidates:
                raise ValueError(f"no target LM available for {robot.name}")
            if stress:
                goal_lm = self._traffic_goal_from_candidates(
                    candidates,
                    stress_profile,
                    len(robots),
                )
            else:
                goal_lm = candidates[0]
            used_goals.add(goal_lm)
            request: dict[str, Any] = {
                "name": str(robot.name),
                "startLm": start_lm,
                "goalLm": goal_lm,
            }
            if isinstance(robot.pose, dict):
                request["startPose"] = dict(robot.pose)
            requests.append(request)
        return requests

    def _traffic_goal_window(
        self,
        stress: bool,
        profile: int,
        count: int,
    ) -> dict[str, int]:
        if not stress:
            return {"min_hops": 3, "max_hops": 15}
        profiles = (
            ((5, 20), (6, 20), (4, 18))
            if count <= 30
            else ((4, 16), (5, 18), (3, 15))
        )
        minimum, maximum = profiles[min(max(0, profile), len(profiles) - 1)]
        return {"min_hops": minimum, "max_hops": maximum}

    def _traffic_goal_from_candidates(
        self,
        candidates: list[str],
        profile: int,
        count: int,
    ) -> str:
        fractions = (0.50, 0.60, 0.40) if count <= 30 else (0.35, 0.45, 0.30)
        fraction = fractions[min(max(0, profile), len(fractions) - 1)]
        index = min(len(candidates) - 1, max(0, int(len(candidates) * fraction)))
        return candidates[index]

    def _benchmark_plan_stats(
        self,
        plans: list[Any],
        time_step_sec: float,
    ) -> dict[str, Any]:
        waiting_robots = 0
        total_wait_ticks = 0
        max_wait_ticks = 0
        route_steps: list[int] = []
        for plan in plans:
            if not isinstance(plan, dict):
                continue
            nodes = [str(node) for node in plan.get("nodes", [])]
            times = [int(value) for value in plan.get("times", [])]
            robot_wait_ticks = 0
            move_steps = 0
            for index in range(1, min(len(nodes), len(times))):
                duration = max(0, times[index] - times[index - 1])
                if nodes[index] == nodes[index - 1]:
                    robot_wait_ticks += duration
                else:
                    move_steps += 1
            if robot_wait_ticks > 0:
                waiting_robots += 1
            total_wait_ticks += robot_wait_ticks
            max_wait_ticks = max(max_wait_ticks, robot_wait_ticks)
            route_steps.append(move_steps)
        return {
            "plannedWaitingRobots": waiting_robots,
            "plannedWaitTicks": total_wait_ticks,
            "plannedWaitSec": round(
                total_wait_ticks * max(0.0, time_step_sec),
                3,
            ),
            "maxPlannedWaitTicks": max_wait_ticks,
            "averageRouteSteps": (
                round(sum(route_steps) / len(route_steps), 2)
                if route_steps
                else 0.0
            ),
            "maxRouteSteps": max(route_steps, default=0),
        }

    def _benchmark_min_separation(self) -> float:
        fleet = self.manager.params.get("fleet", {})
        if isinstance(fleet, dict):
            configured = fleet.get("mapf_min_robot_center_distance_m")
            if configured is not None:
                try:
                    return max(0.0, float(configured))
                except (TypeError, ValueError):
                    pass
        return self.manager.collision.robot_broadphase_distance()

    def _forward_benchmark_goals(
        self,
        start_lm: str,
        used_goals: set[str],
        excluded_goals: set[str],
        rng: random.Random,
        *,
        min_hops: int = 3,
        max_hops: int = 15,
    ) -> list[str]:
        adjacency: dict[str, list[str]] = {
            name: [] for name in self.loaded_map.landmarks
        }
        for edge in self.loaded_map.edges:
            if edge.from_name in adjacency and edge.to_name in adjacency:
                adjacency[edge.from_name].append(edge.to_name)
        for neighbors in adjacency.values():
            neighbors.sort()

        queue: list[tuple[str, int, int]] = [(start_lm, 0, 0)]
        best_path: dict[str, tuple[int, int]] = {start_lm: (0, 0)}
        candidates: list[tuple[int, int, float, int, str]] = []
        used_goal_regions = {
            region_id
            for region_id in (
                self._benchmark_corridor_region(name)
                for name in used_goals
            )
            if region_id
        }
        sequence = 0
        start = self.loaded_map.landmarks[start_lm]
        while queue:
            node, hops, occupied_starts = queue.pop(0)
            if best_path.get(node) != (hops, occupied_starts):
                continue
            if (
                hops >= min_hops
                and node not in used_goals
                and node not in excluded_goals
                and self._benchmark_goal_lm_is_safe(node)
                and self._lm_is_separated_from(node, used_goals)
                and (
                    not self._benchmark_corridor_region(node)
                    or self._benchmark_corridor_region(node)
                    not in used_goal_regions
                )
            ):
                landmark = self.loaded_map.landmarks[node]
                distance_sq = ((landmark.x - start.x) ** 2) + ((landmark.y - start.y) ** 2)
                candidates.append((occupied_starts, hops, distance_sq, sequence, node))
                sequence += 1
            if hops >= max_hops:
                continue
            neighbors = list(adjacency.get(node, ()))
            rng.shuffle(neighbors)
            for neighbor in neighbors:
                next_hops = hops + 1
                next_occupied = occupied_starts + int(
                    neighbor in excluded_goals and neighbor != start_lm
                )
                previous = best_path.get(neighbor)
                if previous is not None and previous <= (next_hops, next_occupied):
                    continue
                best_path[neighbor] = (next_hops, next_occupied)
                queue.append((neighbor, next_hops, next_occupied))
        candidates.sort()
        return [item[4] for item in candidates]

    def _lm_is_separated_from(self, candidate: str, selected: set[str] | list[str]) -> bool:
        landmark = self.loaded_map.landmarks[candidate]
        minimum = self._benchmark_min_separation()
        candidate_pose = {"x": landmark.x, "y": landmark.y, "yaw": 0.0}
        for name in selected:
            if name not in self.loaded_map.landmarks:
                continue
            other = self.loaded_map.landmarks[name]
            if math.hypot(
                landmark.x - self.loaded_map.landmarks[name].x,
                landmark.y - self.loaded_map.landmarks[name].y,
            ) < minimum:
                return False
            if self.manager.collision.robot_footprints_conflict(
                candidate_pose,
                {"x": other.x, "y": other.y, "yaw": 0.0},
            ):
                return False
        return True

    def _spatially_separated_lms(self, candidates: list[str], count: int) -> list[str]:
        selected: list[str] = []
        for name in candidates:
            if name not in self.loaded_map.landmarks:
                continue
            if not self._lm_is_separated_from(name, selected):
                continue
            selected.append(name)
            if len(selected) >= count:
                break
        return selected

    def _largest_benchmark_component(self) -> list[str]:
        adjacency: dict[str, set[str]] = {name: set() for name in self.loaded_map.landmarks}
        for edge in self.loaded_map.edges:
            if edge.from_name in adjacency and edge.to_name in adjacency:
                adjacency[edge.from_name].add(edge.to_name)
                adjacency[edge.to_name].add(edge.from_name)
        visited: set[str] = set()
        components: list[list[str]] = []
        for name in sorted(adjacency):
            if name in visited:
                continue
            stack = [name]
            visited.add(name)
            component: list[str] = []
            while stack:
                current = stack.pop()
                component.append(current)
                for neighbor in adjacency.get(current, set()):
                    if neighbor in visited:
                        continue
                    visited.add(neighbor)
                    stack.append(neighbor)
            components.append(sorted(component))
        return max(components, key=len, default=[])

    def resolve_map_dir(self, map_dir: Path) -> Path:
        candidate = Path(map_dir).expanduser()
        safe_name = Path(candidate).name
        maps_root = FLEET_MAPS_OUT_ROOT.resolve()
        if candidate.is_absolute():
            resolved = candidate.resolve()
            if resolved.exists() and (resolved == maps_root or maps_root in resolved.parents):
                return resolved
            candidate = Path(safe_name)

        candidates = [
            FLEET_MAPS_OUT_ROOT / candidate,
            FLEET_MAP_DATA_ROOT / candidate,
            FLEET_ROOT / candidate,
            FLEET_MAPS_OUT_ROOT / safe_name,
        ]
        if not safe_name.endswith(".smap"):
            candidates.append(FLEET_MAPS_OUT_ROOT / f"{safe_name}.smap")
        for item in candidates:
            resolved = item.resolve()
            if resolved.exists() and (resolved == maps_root or maps_root in resolved.parents):
                return resolved
        return DEFAULT_FLEET_MAP_DIR.resolve()

    def _resolve_map_dir_by_name(self, map_name: str) -> Path:
        if not map_name:
            return self.map_dir
        safe_name = Path(str(map_name)).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        target = (self.maps_root / safe_name).resolve()
        root = self.maps_root.resolve()
        if root not in target.parents and target != root:
            raise ValueError("map must stay inside fleet maps_out")
        if not target.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        return target

    def _load_context(self, map_dir: Path) -> None:
        loaded_map = WarehouseMapLoader(map_dir).load()
        params = load_route_params(self.params_path, create=True)
        self.loaded_map = loaded_map
        self.map_dir = loaded_map.map_dir.resolve()
        self.maps_root = self.map_dir.parent
        self._scene3d_cache = None
        manager_class = (
            FleetManagerROS
            if self.mode == "robots"
            else FleetManagerSim
        )
        self.manager = manager_class(
            loaded_map.landmarks,
            loaded_map.edges,
            params=params,
            map_dir=loaded_map.map_dir,
            map_metadata=loaded_map.map_metadata,
            remote_adapter=self.remote_adapter,
        )
        self._reset_dynamic_benchmark()
        self._sync_manager_mode()

    def _sync_manager_mode(self) -> None:
        if not hasattr(self, "manager"):
            return
        self.manager.set_active_robot_modes(self._active_robot_modes())

    def _active_robot_modes(self) -> set[str]:
        if self.mode == "robots":
            return {"remote"}
        return {"simulated"}

    def _static_scene3d_payload(self) -> dict[str, Any]:
        if isinstance(self._scene3d_cache, dict):
            return self._scene3d_cache

        metadata = self.loaded_map.map_metadata
        wall_height = 1.8
        walls = self._wall_rectangles_from_pgm(wall_height=wall_height)
        payload = {
            "ok": True,
            "mapName": self.map_dir.stem.replace(".smap", ""),
            "mapDir": str(self.map_dir),
            "coordinateFrame": "map_top_left",
            "floor": {
                "width": metadata.width * metadata.resolution,
                "depth": metadata.height * metadata.resolution,
                "resolution": metadata.resolution,
                "imageDataUrl": metadata.image_data_url,
            },
            "bounds": {
                "minX": 0.0,
                "minZ": 0.0,
                "maxX": metadata.width * metadata.resolution,
                "maxZ": metadata.height * metadata.resolution,
            },
            "walls": walls,
            "wallHeight": wall_height,
            "lms": [self.loaded_map.landmarks[name].to_dict() for name in sorted(self.loaded_map.landmarks)],
            "edges": [edge.to_dict() for edge in self.loaded_map.edges],
        }
        self._scene3d_cache = payload
        return payload

    def _wall_rectangles_from_pgm(self, *, wall_height: float) -> list[dict[str, Any]]:
        ros_map_yaml = self._find_ros_map_yaml()
        ros_map = yaml.safe_load(ros_map_yaml.read_text(encoding="utf-8"))
        if not isinstance(ros_map, dict):
            raise ValueError(f"Unexpected ROS map file format: {ros_map_yaml}")
        image_path = (self.map_dir / str(ros_map["image"])).resolve()
        width, height, pixels = WarehouseMapLoader(self.map_dir)._load_pgm(image_path)
        occupied_thresh = float(ros_map.get("occupied_thresh", 0.65) or 0.65)
        negate = int(ros_map.get("negate", 0) or 0)
        max_rectangles = 6000
        for stride in (1, 2, 4, 8):
            rectangles = self._build_wall_rectangles(
                width,
                height,
                pixels,
                occupied_thresh=occupied_thresh,
                negate=negate,
                stride=stride,
                wall_height=wall_height,
            )
            if len(rectangles) <= max_rectangles or stride == 8:
                return rectangles
        return []

    def _build_wall_rectangles(
        self,
        width: int,
        height: int,
        pixels: bytes,
        *,
        occupied_thresh: float,
        negate: int,
        stride: int,
        wall_height: float,
    ) -> list[dict[str, Any]]:
        resolution = float(self.loaded_map.map_metadata.resolution)
        grid_width = math.ceil(width / stride)
        grid_height = math.ceil(height / stride)
        active: dict[tuple[int, int], dict[str, int]] = {}
        rectangles_px: list[dict[str, int]] = []

        def block_occupied(cell_x: int, cell_y: int) -> bool:
            start_x = cell_x * stride
            start_y = cell_y * stride
            end_x = min(width, start_x + stride)
            end_y = min(height, start_y + stride)
            for py in range(start_y, end_y):
                row_offset = py * width
                for px in range(start_x, end_x):
                    value = pixels[row_offset + px]
                    occupancy = (value / 255.0) if negate else ((255 - value) / 255.0)
                    if occupancy > occupied_thresh:
                        return True
            return False

        for cell_y in range(grid_height):
            runs: list[tuple[int, int]] = []
            run_start: int | None = None
            for cell_x in range(grid_width):
                if block_occupied(cell_x, cell_y):
                    if run_start is None:
                        run_start = cell_x
                elif run_start is not None:
                    runs.append((run_start, cell_x - run_start))
                    run_start = None
            if run_start is not None:
                runs.append((run_start, grid_width - run_start))

            next_active: dict[tuple[int, int], dict[str, int]] = {}
            for x, w in runs:
                key = (x, w)
                rect = active.pop(key, None)
                if rect is None:
                    rect = {"x": x, "y": cell_y, "w": w, "h": 1}
                else:
                    rect["h"] += 1
                next_active[key] = rect
            rectangles_px.extend(active.values())
            active = next_active

        rectangles_px.extend(active.values())
        rectangles: list[dict[str, Any]] = []
        for rect in rectangles_px:
            px = rect["x"] * stride
            py = rect["y"] * stride
            pw = min(width - px, rect["w"] * stride)
            ph = min(height - py, rect["h"] * stride)
            rectangles.append(
                {
                    "x": round((px + (pw / 2.0)) * resolution, 4),
                    "z": round((py + (ph / 2.0)) * resolution, 4),
                    "width": round(pw * resolution, 4),
                    "depth": round(ph * resolution, 4),
                    "height": wall_height,
                    "stride": stride,
                }
            )
        return rectangles

    def _find_ros_map_yaml(self) -> Path:
        candidates = sorted(
            path
            for path in self.map_dir.glob("*.yaml")
            if path.name not in {"LMs.yaml", "graphs.yaml", "graph_edges_lengths.yaml"}
        )
        if not candidates:
            raise FileNotFoundError(f"No ROS map yaml found in {self.map_dir}")
        return candidates[0]

    def _state_with_context(self, state: Any) -> dict[str, Any]:
        if not isinstance(state, dict):
            state = self.manager.state()
        state["mode"] = self.mode
        state["mapName"] = self.map_dir.stem.replace(".smap", "")
        state["managerId"] = self.manager_id
        state["managerName"] = self.display_name
        # High-rate websocket ticks intentionally omit slow collections. Keep
        # benchmark aggregation on the same 5 Hz control-plane cadence; the
        # browser retains the previous value while consuming pose deltas.
        if self.mode == "simulation" and (
            "orders" in state or "events" in state
        ):
            state["dynamicBenchmark"] = self._dynamic_benchmark_payload()
        return state

    def _result_with_context(self, result: dict[str, Any]) -> dict[str, Any]:
        state = result.get("state")
        if isinstance(state, dict):
            result["state"] = self._state_with_context(state)
        result["mode"] = self.mode
        result["mapName"] = self.map_dir.stem.replace(".smap", "")
        result["managerId"] = self.manager_id
        result["managerName"] = self.display_name
        return result
