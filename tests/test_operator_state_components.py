from __future__ import annotations

from threading import RLock
from types import SimpleNamespace
from typing import Any

import pytest

from operator_app.core.fleet_manager import (
    FLEET_MANAGER_ID,
    FLEET_MANAGER_SIM_ID,
)
from operator_app.core.state import OperatorAppState
from operator_app.core.state_fleet_api import FleetApiRoutingMixin
from operator_app.core.state_fleet_maps import FleetMapSyncMixin
from operator_app.core.state_robot_control import RobotControlProxyMixin
from operator_app.core.state_robot_maps import RobotMapSyncMixin
from operator_app.core.state_robot_registry import RobotRegistryProbeMixin
from operator_app.core.state_runtime import RuntimeOwnershipMixin


def test_operator_state_facade_composes_focused_capabilities() -> None:
    assert OperatorAppState.__bases__ == (
        RuntimeOwnershipMixin,
        RobotRegistryProbeMixin,
        FleetApiRoutingMixin,
        FleetMapSyncMixin,
        RobotMapSyncMixin,
        RobotControlProxyMixin,
    )
    assert OperatorAppState.__init__ is RuntimeOwnershipMixin.__init__
    assert (
        OperatorAppState.list_robots_payload
        is RobotRegistryProbeMixin.list_robots_payload
    )
    assert (
        OperatorAppState.fleet_manager_stream_payload
        is FleetApiRoutingMixin.fleet_manager_stream_payload
    )
    assert (
        OperatorAppState.fleet_local_active_map_payload
        is FleetMapSyncMixin.fleet_local_active_map_payload
    )
    assert (
        OperatorAppState.pull_sync_payload
        is RobotMapSyncMixin.pull_sync_payload
    )
    assert (
        OperatorAppState.watch_robot_laser_scan
        is RobotControlProxyMixin.watch_robot_laser_scan
    )


def test_fleet_lock_selector_is_per_manager_and_keeps_legacy_fallback() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    real_lock = RLock()
    simulation_lock = RLock()
    state._fleet_manager_lock = real_lock
    state._fleet_manager_sim_lock = simulation_lock
    state._fleet_lock = real_lock

    assert state._fleet_lock_for_id(FLEET_MANAGER_ID) is real_lock
    assert state._fleet_lock_for_id(FLEET_MANAGER_SIM_ID) is simulation_lock

    legacy_state = OperatorAppState.__new__(OperatorAppState)
    legacy_lock = RLock()
    legacy_state._fleet_lock = legacy_lock

    assert legacy_state._fleet_lock_for_id(FLEET_MANAGER_ID) is legacy_lock
    assert (
        legacy_state._fleet_lock_for_id(FLEET_MANAGER_SIM_ID)
        is legacy_lock
    )


def test_sidebar_snapshots_have_fixed_order_and_nonblocking_fallback() -> None:
    events: list[tuple[str, Any]] = []

    class RecordingLock:
        def __init__(self, name: str, *, available: bool) -> None:
            self.name = name
            self.available = available

        def acquire(self, blocking: bool = True) -> bool:
            events.append((f"{self.name}.acquire", blocking))
            return self.available

        def release(self) -> None:
            events.append((f"{self.name}.release", None))

    class Manager:
        def __init__(self, name: str) -> None:
            self.name = name

        def sidebar_payload(
            self,
            include_runtime: bool = True,
        ) -> dict[str, Any]:
            events.append((f"{self.name}.sidebar", include_runtime))
            return {"manager": self.name}

    state = OperatorAppState.__new__(OperatorAppState)
    state.fleet_manager = Manager("real")
    state.fleet_manager_sim = Manager("simulation")
    state._fleet_manager_lock = RecordingLock("real", available=True)
    state._fleet_manager_sim_lock = RecordingLock(
        "simulation",
        available=False,
    )

    assert state._fleet_sidebar_payloads(include_runtime=False) == [
        {"manager": "real"},
        {"manager": "simulation"},
    ]
    assert events == [
        ("real.acquire", False),
        ("real.sidebar", False),
        ("real.release", None),
        ("simulation.acquire", False),
        ("simulation.sidebar", False),
    ]


def test_stream_snapshot_uses_only_selected_manager_lock() -> None:
    events: list[tuple[str, Any]] = []

    class RecordingLock:
        def __init__(self, name: str) -> None:
            self.name = name

        def acquire(self, blocking: bool = True) -> bool:
            events.append((f"{self.name}.acquire", blocking))
            return True

        def release(self) -> None:
            events.append((f"{self.name}.release", None))

    class Manager:
        def __init__(self, name: str) -> None:
            self.name = name

        def tick_payload(
            self,
            _payload: dict[str, Any],
            *,
            advance_runtime: bool,
            route_revisions: dict[str, int] | None,
            include_runtime_details: bool,
        ) -> dict[str, Any]:
            events.append((f"{self.name}.tick", advance_runtime))
            assert route_revisions == {"robot": 3}
            assert not include_runtime_details
            return {"manager": self.name}

    state = OperatorAppState.__new__(OperatorAppState)
    state.fleet_manager = Manager("real")
    state.fleet_manager_sim = Manager("simulation")
    state._fleet_manager_lock = RecordingLock("real")
    state._fleet_manager_sim_lock = RecordingLock("simulation")

    payload = state.fleet_manager_stream_payload(
        manager_id=FLEET_MANAGER_SIM_ID,
        route_revisions={"robot": 3},
        include_runtime_details=False,
    )

    assert payload is not None
    assert payload["state"] == {"manager": "simulation"}
    assert events == [
        ("simulation.acquire", True),
        ("simulation.tick", False),
        ("simulation.release", None),
    ]


def test_robot_endpoint_validation_contract() -> None:
    state = OperatorAppState.__new__(OperatorAppState)

    assert state._grpc_robot_key(" Robot LAN/1 ", 50051) == (
        "grpc_robot_lan_1_50051"
    )
    assert state._require_host({"host": " robot-1.local "}) == (
        "robot-1.local"
    )
    assert state._require_port({"port": "50052"}) == 50052
    assert state._payload_robot_type({"mode": " GRPC "}) == "grpc"


def test_saved_slam_map_can_remain_inactive() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    robot = SimpleNamespace(id="robot-1", is_grpc=True)
    state.get_robot = lambda robot_id: robot
    state._grpc_endpoint = lambda selected: "127.0.0.1:50051"
    calls: list[tuple[str, Any]] = []

    class Adapter:
        def finish_slam(
            self,
            endpoint: str,
            *,
            map_name: str,
            activate: bool,
        ) -> dict[str, Any]:
            calls.append(("finish", (endpoint, map_name, activate)))
            return {
                "ok": True,
                "mapName": map_name,
                "bundle": {"mapName": map_name},
            }

        def list_maps(self, endpoint: str) -> dict[str, Any]:
            calls.append(("list", endpoint))
            return {"ok": True, "maps": []}

    class Cache:
        def save_local_bundle(
            self,
            robot_id: str,
            bundle: dict[str, Any],
            *,
            activate: bool,
        ) -> dict[str, Any]:
            calls.append(("cache", (robot_id, bundle, activate)))
            return {"mapName": bundle["mapName"]}

    class Workspace:
        def save_active_map_meta(self, *args: Any) -> None:
            calls.append(("active", args))

        def save_map_index(self, selected: Any, payload: dict[str, Any]) -> None:
            calls.append(("index", (selected.id, payload)))

    state.grpc_adapter = Adapter()
    state.map_cache = Cache()
    state.workspace = Workspace()

    result = state.finish_robot_slam_payload(
        "robot-1",
        {"mapName": "new-slam", "activate": False},
    )

    assert result["ok"] is True
    assert ("finish", ("127.0.0.1:50051", "new-slam", False)) in calls
    assert (
        "cache",
        ("robot-1", {"mapName": "new-slam"}, False),
    ) in calls
    assert not any(kind == "active" for kind, _payload in calls)


def test_robot_push_sync_uploads_without_loading_runtime_map() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    robot = SimpleNamespace(id="robot-1", is_grpc=True)
    state.get_robot = lambda robot_id: robot
    state._grpc_endpoint = lambda selected: "robot:50051"
    calls: list[tuple[str, Any]] = []

    class Adapter:
        def list_maps(self, endpoint: str) -> dict[str, Any]:
            calls.append(("list", endpoint))
            return {
                "active": "map-a",
                "activeSignature": "old-runtime",
                "maps": [{"name": "map-a", "signature": "old-storage"}],
            }

    class Cache:
        def load_active_map(self, robot_id: str) -> dict[str, Any]:
            return {
                "mapName": "map-a",
                "sourceMapName": "map-a",
                "map": {"signature": "new-local"},
            }

    class Workspace:
        def save_map_index(self, selected: Any, payload: dict[str, Any]) -> None:
            calls.append(("index", selected.id))

    state.grpc_adapter = Adapter()
    state.map_cache = Cache()
    state.workspace = Workspace()
    state.push_robot_map_payload = lambda robot_id, payload: calls.append(
        ("push", (robot_id, payload))
    ) or {
        "pushed": {"mapName": "map-a"},
        "verified": {"signature": "new-local"},
        "local": {"mapName": "map-a"},
    }

    result = state.push_sync_payload("robot-1")

    assert result["changed"] is True
    assert result["loadRequired"] is True
    assert [kind for kind, _payload in calls].count("push") == 1
    assert not any(kind == "load" for kind, _payload in calls)


def test_robot_load_rolls_back_previous_map_when_verification_fails() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    robot = SimpleNamespace(id="robot-1", is_grpc=True)
    state.get_robot = lambda robot_id: robot
    state._is_grpc_robot_id = lambda robot_id: True
    state._grpc_endpoint = lambda selected: "robot:50051"
    load_calls: list[str] = []

    class Adapter:
        def active_map(self, endpoint: str) -> dict[str, Any]:
            if not load_calls:
                return {"mapName": "old-map", "signature": "old-signature"}
            return {"mapName": "wrong-map", "signature": "wrong-signature"}

        def load_map(self, endpoint: str, map_name: str) -> dict[str, Any]:
            load_calls.append(map_name)
            return {"mapName": map_name}

    state.grpc_adapter = Adapter()

    with pytest.raises(ValueError, match="Load verification failed"):
        state.load_robot_map_payload("robot-1", {"mapName": "new-map"})

    assert load_calls == ["new-map", "old-map"]


def test_robot_map_state_distinguishes_storage_from_same_name_runtime() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    robot = SimpleNamespace(id="robot-1", is_grpc=True)
    state.get_robot = lambda robot_id: robot
    state._is_grpc_robot_id = lambda robot_id: True
    state._grpc_endpoint = lambda selected: "robot:50051"

    class Adapter:
        def list_maps(self, endpoint: str) -> dict[str, Any]:
            return {
                "active": "map-a",
                "activeSignature": "old-runtime-signature",
                "maps": [{"name": "map-a", "signature": "new-signature"}],
            }

    class Cache:
        def active_map_name(self, robot_id: str) -> str:
            return "map-a"

        def load_active_map(self, robot_id: str) -> dict[str, Any]:
            return {
                "mapName": "map-a",
                "sourceMapName": "map-a",
                "signature": "new-signature",
                "map": {"signature": "new-signature"},
                "hasLocalChanges": False,
            }

    state.grpc_adapter = Adapter()
    state.map_cache = Cache()

    result = state.local_active_map_payload("robot-1")

    assert result["hasLocalChanges"] is False
    assert result["robotSignature"] == "new-signature"
    assert result["robotActiveSignature"] == "old-runtime-signature"
    assert result["activationRequired"] is True
    assert result["syncState"] == "load_required"


def test_robot_connection_bootstrap_downloads_entire_map_library() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    robot = SimpleNamespace(id="robot-1")
    cached: list[tuple[str, bool]] = []
    saved: list[tuple[str, Any]] = []
    state._ensure_robot_workspace = lambda selected: {"robotId": selected.id}

    class Adapter:
        def list_maps(self, endpoint: str) -> dict[str, Any]:
            return {
                "active": "map-b",
                "activeSignature": "sig-b",
                "maps": [
                    {"name": "map-a", "signature": "sig-a"},
                    {"name": "map-b", "signature": "sig-b"},
                ],
            }

        def active_map(self, endpoint: str) -> dict[str, Any]:
            raise RuntimeError("second list call unavailable")

        def get_map_bundle(self, endpoint: str, map_name: str) -> dict[str, Any]:
            return {"mapName": map_name, "signature": f"sig-{map_name[-1]}"}

        def get_params(self, endpoint: str) -> dict[str, Any]:
            return {"params": {"robot": {"name": "robot-1"}}}

    class Cache:
        def save_pulled_map(
            self,
            robot_id: str,
            bundle: dict[str, Any],
            *,
            activate: bool,
        ) -> None:
            cached.append((bundle["mapName"], activate))

    class Workspace:
        def save_map_index(self, selected: Any, payload: dict[str, Any]) -> None:
            saved.append(("index", payload["active"]))

        def save_active_map_meta(self, selected: Any, payload: dict[str, Any]) -> None:
            saved.append(("active", payload["mapName"]))

    state.grpc_adapter = Adapter()
    state.map_cache = Cache()
    state.workspace = Workspace()
    state._cache_robot_params = lambda selected, params, source: saved.append(
        ("params", source)
    )

    result = state._bootstrap_robot_workspace(robot, "robot:50051")

    assert result["cachedMaps"] == ["map-a", "map-b"]
    assert result["activeMapName"] == "map-b"
    assert cached == [("map-a", False), ("map-b", True)]
    assert ("active", "map-b") in saved
    assert ("params", "robot") in saved


def test_robot_params_save_is_read_back_before_local_cache_is_updated() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    robot = SimpleNamespace(id="robot-1", is_grpc=True)
    requested = {"navigation": {"route_speed": 1.37}}
    calls: list[tuple[str, Any]] = []
    state.get_robot = lambda robot_id: robot
    state._is_grpc_robot_id = lambda robot_id: True
    state._grpc_endpoint = lambda selected: "robot:50051"

    class Adapter:
        def put_params(self, endpoint: str, params: dict[str, Any]) -> dict[str, Any]:
            calls.append(("put", params))
            return {"ok": True, "params": params, "reloaded": True}

        def get_params(self, endpoint: str) -> dict[str, Any]:
            calls.append(("get", endpoint))
            return {"ok": True, "params": requested}

    state.grpc_adapter = Adapter()
    state._cache_robot_params = lambda selected, params, source: calls.append(
        ("cache", (params, source))
    )

    result = state.save_robot_params_payload("robot-1", {"params": requested})

    assert result["synced"] is True
    assert result["verified"] is True
    assert result["params"] == requested
    assert calls == [
        ("put", requested),
        ("get", "robot:50051"),
        ("cache", (requested, "robot-verified")),
    ]


def test_robot_params_save_rejects_read_back_mismatch() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    robot = SimpleNamespace(id="robot-1", is_grpc=True)
    cached: list[dict[str, Any]] = []
    state.get_robot = lambda robot_id: robot
    state._is_grpc_robot_id = lambda robot_id: True
    state._grpc_endpoint = lambda selected: "robot:50051"
    state.grpc_adapter = SimpleNamespace(
        put_params=lambda endpoint, params: {"ok": True, "params": params},
        get_params=lambda endpoint: {"ok": True, "params": {"navigation": {"route_speed": 0.2}}},
    )
    state._cache_robot_params = lambda selected, params, source: cached.append(params)

    with pytest.raises(ValueError, match="differ"):
        state.save_robot_params_payload(
            "robot-1",
            {"params": {"navigation": {"route_speed": 1.37}}},
        )

    assert cached == []
