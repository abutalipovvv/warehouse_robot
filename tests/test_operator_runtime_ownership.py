from __future__ import annotations

from pathlib import Path
from threading import Event, Lock, RLock, Thread, current_thread, get_ident
from time import monotonic, sleep

import operator_app.core.state_runtime as state_runtime_module
from fleet_manager.runtime.loop import RuntimeLoop, RuntimeLoopState
from operator_app.core.fleet_manager import (
    FLEET_MANAGER_ID,
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)
from operator_app.core.state import OperatorAppState


def _wait_until(predicate, *, timeout: float = 1.0) -> bool:
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return True
        sleep(0.002)
    return predicate()


def test_operator_state_owns_real_and_simulation_runtime_loops(
    monkeypatch,
    tmp_path: Path,
) -> None:
    tracker_lock = Lock()
    calls: dict[str, list[tuple[float, int, bool]]] = {
        "robots": [],
        "simulation": [],
    }
    active_steps = 0
    maximum_active_steps = 0
    fail_real_once = True
    instances: dict[str, FakeOperatorFleetManager] = {}

    class FakeCore:
        def __init__(self, mode: str) -> None:
            self.params = {
                "fleet": {
                    "simulation_tick_interval_sec": 0.15,
                }
            }
            self.events: list[tuple[str, str]] = []
            self.mode = mode

        def _event(self, level: str, message: str) -> None:
            self.events.append((level, message))

    class FakeOperatorFleetManager:
        def __init__(
            self,
            _map_dir,
            _params_path,
            remote_adapter=None,
            *,
            manager_id: str,
            display_name: str,
            mode: str,
        ) -> None:
            del display_name, remote_adapter
            self.manager_id = manager_id
            self.mode = mode
            self.manager = FakeCore(mode)
            self.closed = False
            instances[mode] = self

        def runtime_step(self) -> None:
            nonlocal active_steps, maximum_active_steps, fail_real_once
            with tracker_lock:
                active_steps += 1
                maximum_active_steps = max(
                    maximum_active_steps,
                    active_steps,
                )
                calls[self.mode].append(
                    (
                        monotonic(),
                        get_ident(),
                        current_thread().daemon,
                    )
                )
            try:
                sleep(0.005)
                if self.mode == "robots" and fail_real_once:
                    fail_real_once = False
                    raise ValueError("expected real runtime failure")
            finally:
                with tracker_lock:
                    active_steps -= 1

        def close(self) -> None:
            self.closed = True

    class FakeGrpcAdapter:
        def __init__(self, timeout: float) -> None:
            self.timeout = timeout

    monkeypatch.setattr(
        state_runtime_module,
        "OperatorFleetManager",
        FakeOperatorFleetManager,
    )
    monkeypatch.setattr(
        state_runtime_module,
        "GrpcRobotAdapter",
        FakeGrpcAdapter,
    )

    state = OperatorAppState(
        registry_path=tmp_path / "robots.json",
        probe_timeout=0.1,
        fleet_params_path=tmp_path / "params.yaml",
        fleet_map_dir=tmp_path / "map.smap",
    )
    try:
        assert _wait_until(
            lambda: (
                len(calls["robots"]) >= 2
                and len(calls["simulation"]) >= 1
            ),
        )
        assert _wait_until(
            lambda: bool(instances["robots"].manager.events),
        )

        with state._fleet_lock_for_id(FLEET_MANAGER_SIM_ID):
            instances["simulation"].manager.params["fleet"][
                "simulation_tick_interval_sec"
            ] = 0.05

        assert _wait_until(
            lambda: len(calls["simulation"]) >= 3,
            timeout=0.8,
        )
        assert _wait_until(
            lambda: state._fleet_runtime_loops[1].interval_seconds == 0.05,
        )

        with tracker_lock:
            assert maximum_active_steps == 2
            for mode_calls in calls.values():
                assert len({thread_id for _, thread_id, _ in mode_calls}) == 1
                assert {daemon for _, _, daemon in mode_calls} == {False}
            simulation_times = [
                timestamp
                for timestamp, _thread_id, _daemon in calls["simulation"]
            ]
        assert simulation_times[-1] - simulation_times[-2] < 0.10
        assert "expected real runtime failure" in (
            instances["robots"].manager.events[0][1]
        )
    finally:
        state.close()

    assert all(
        loop.state is RuntimeLoopState.CLOSED
        for loop in state._fleet_runtime_loops
    )
    assert instances["robots"].closed
    assert instances["simulation"].closed

    calls_after_close = {
        mode: len(mode_calls)
        for mode, mode_calls in calls.items()
    }
    sleep(0.06)
    assert {
        mode: len(mode_calls)
        for mode, mode_calls in calls.items()
    } == calls_after_close


def test_slow_real_tick_does_not_delay_simulation_tick() -> None:
    real_entered = Event()
    release_real = Event()
    simulation_finished = Event()

    class SlowRealManager:
        def runtime_step(self) -> None:
            real_entered.set()
            assert release_real.wait(timeout=1.0)

    class SimulationManager:
        def runtime_step(self) -> None:
            simulation_finished.set()

    state = OperatorAppState.__new__(OperatorAppState)
    state.fleet_manager = SlowRealManager()
    state.fleet_manager_sim = SimulationManager()
    state._fleet_manager_lock = RLock()
    state._fleet_manager_sim_lock = RLock()
    state._fleet_lock = state._fleet_manager_lock

    real_thread = Thread(
        target=state._fleet_runtime_step,
        args=(state.fleet_manager,),
    )
    simulation_thread = Thread(
        target=state._fleet_runtime_step,
        args=(state.fleet_manager_sim,),
    )
    real_thread.start()
    assert real_entered.wait(timeout=0.5)

    simulation_thread.start()
    try:
        assert simulation_finished.wait(timeout=0.2)
        assert real_thread.is_alive()
    finally:
        release_real.set()
        real_thread.join(timeout=1.0)
        simulation_thread.join(timeout=1.0)

    assert not real_thread.is_alive()
    assert not simulation_thread.is_alive()


def test_api_command_does_not_hold_fleet_lock_while_waiting_for_owner() -> None:
    owner_thread_ids: list[int] = []

    class RoutedManager:
        def __init__(self) -> None:
            self.executor = None

        def sidebar_payload(self) -> dict[str, object]:
            assert self.executor is not None
            return self.executor(
                lambda: owner_thread_ids.append(get_ident()) or {"ok": True}
            )

    manager = RoutedManager()
    state = OperatorAppState.__new__(OperatorAppState)
    state.fleet_manager = manager
    state.fleet_manager_sim = manager
    state._fleet_manager_lock = RLock()
    state._fleet_manager_sim_lock = RLock()
    state._fleet_lock = state._fleet_manager_lock

    runtime_loop = RuntimeLoop(
        lambda: None,
        interval_seconds=1.0,
        name="operator-api-owner-test",
    )
    state._fleet_runtime_loops = (runtime_loop, runtime_loop)
    manager.executor = runtime_loop.execute
    caller_thread_id = get_ident()
    runtime_loop.start()
    try:
        assert state.fleet_manager_get_payload(
            "identity",
            manager_id=FLEET_MANAGER_SIM_ID,
        ) == {"ok": True}
    finally:
        runtime_loop.close()

    assert owner_thread_ids
    assert owner_thread_ids[0] != caller_thread_id


def test_operator_fleet_runtime_step_advances_both_modes() -> None:
    class Core:
        def __init__(self) -> None:
            self.advance_calls = 0

        def advance_runtime(self) -> None:
            self.advance_calls += 1

    def service(mode: str) -> tuple[OperatorFleetManager, Core, list[str]]:
        result = OperatorFleetManager.__new__(OperatorFleetManager)
        result.mode = mode
        core = Core()
        result.manager = core
        pump_calls: list[str] = []
        result._sync_manager_mode = lambda: None
        result._pump_dynamic_benchmark = lambda: pump_calls.append(mode)
        return result, core, pump_calls

    real, real_core, real_pumps = service("robots")
    simulation, simulation_core, simulation_pumps = service("simulation")

    real.runtime_step()
    simulation.runtime_step()

    assert real_core.advance_calls == 1
    assert simulation_core.advance_calls == 1
    assert real_pumps == []
    assert simulation_pumps == ["simulation"]


def test_state_sidebar_and_scene_use_snapshots_only() -> None:
    class SnapshotCore:
        def __init__(self) -> None:
            self.snapshot_calls = 0
            self.state_calls = 0

        def set_active_robot_modes(self, _modes) -> None:
            pass

        def snapshot(self, include_trajectories: bool = True):
            del include_trajectories
            self.snapshot_calls += 1
            return {"ok": True, "robots": []}

        def state(self, include_trajectories: bool = True):
            del include_trajectories
            self.state_calls += 1
            raise AssertionError("a read payload advanced runtime")

    service = OperatorFleetManager.__new__(OperatorFleetManager)
    service.mode = "robots"
    service.manager = SnapshotCore()
    service.manager_id = FLEET_MANAGER_ID
    service.display_name = "Fleet Manager"
    service.map_dir = Path("/tmp/read-only-map.smap")
    service._static_scene3d_payload = lambda: {"ok": True, "lms": []}

    assert service.state_payload()["ok"]
    assert service.sidebar_payload()["runtimeFresh"]
    assert service.scene3d_payload()["ok"]
    assert service.manager.snapshot_calls == 3
    assert service.manager.state_calls == 0


def test_get_and_websocket_paths_never_request_runtime_advance() -> None:
    class PayloadManager:
        def __init__(self) -> None:
            self.state_advance_values: list[bool] = []
            self.tick_advance_values: list[bool] = []

        def state_payload(
            self,
            include_trajectories: bool = True,
            *,
            advance_runtime: bool,
        ):
            del include_trajectories
            self.state_advance_values.append(advance_runtime)
            return {"ok": True, "robots": []}

        def tick_payload(
            self,
            _payload,
            *,
            advance_runtime: bool,
            route_revisions,
            include_runtime_details: bool,
        ):
            del route_revisions, include_runtime_details
            self.tick_advance_values.append(advance_runtime)
            return {"ok": True, "robots": []}

    state = OperatorAppState.__new__(OperatorAppState)
    state._fleet_lock = RLock()
    state.fleet_manager = PayloadManager()
    state.fleet_manager_sim = PayloadManager()

    state.fleet_manager_get_payload("state", manager_id=FLEET_MANAGER_ID)
    state.fleet_manager_get_payload("status", manager_id=FLEET_MANAGER_SIM_ID)
    state.fleet_manager_stream_payload(
        initial=True,
        manager_id=FLEET_MANAGER_ID,
    )
    state.fleet_manager_stream_payload(
        initial=False,
        manager_id=FLEET_MANAGER_ID,
    )
    state.fleet_manager_stream_payload(
        initial=False,
        manager_id=FLEET_MANAGER_SIM_ID,
    )

    assert state.fleet_manager.state_advance_values == [False, False]
    assert state.fleet_manager.tick_advance_values == [False]
    assert state.fleet_manager_sim.state_advance_values == [False]
    assert state.fleet_manager_sim.tick_advance_values == [False]
