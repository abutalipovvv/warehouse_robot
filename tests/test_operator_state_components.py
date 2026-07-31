from __future__ import annotations

from threading import RLock
from typing import Any

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


def test_fleet_local_response_detects_local_map_differences() -> None:
    state = OperatorAppState.__new__(OperatorAppState)

    assert (
        state._fleet_local_response(
            None,
            active_name="local",
            robot_active_name="robot",
            robot_signature="remote-signature",
        )
        is None
    )
    assert state._fleet_local_response(
        {
            "mapName": "local",
            "map": {"lms": []},
            "signature": "local-signature",
            "sourceMapName": "robot",
        },
        active_name="fallback",
        robot_active_name="robot",
        robot_signature="remote-signature",
    ) == {
        "activeMapName": "local",
        "mapName": "local",
        "map": {"lms": []},
        "sourceMapName": "robot",
        "signature": "local-signature",
        "robotSignature": "remote-signature",
        "robotMapName": "robot",
        "hasLocalChanges": True,
    }


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
