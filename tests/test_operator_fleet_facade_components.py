from __future__ import annotations

import inspect
from pathlib import Path

from operator_app.core.fleet_context import FleetContextService
from operator_app.core.fleet_manager import OperatorFleetManager
from operator_app.core.fleet_manual_control import (
    FleetManualControlService,
)
from operator_app.core.fleet_map_service import FleetMapService
from operator_app.core.fleet_snapshot_service import (
    FleetSnapshotService,
)


CONTEXT_HOOKS = {
    "close",
    "mode_payload",
    "set_mode_payload",
    "params_payload",
    "save_params_payload",
    "resolve_map_dir",
    "_resolve_map_dir_by_name",
    "_load_context",
    "_sync_manager_mode",
    "_active_robot_modes",
}
MAP_HOOKS = {
    "map_payload",
    "scene3d_payload",
    "maps_active_payload",
    "maps_list_payload",
    "pull_map_payload",
    "push_map_payload",
    "load_map_payload",
    "save_map_payload",
    "_static_scene3d_payload",
    "_wall_rectangles_from_pgm",
    "_build_wall_rectangles",
    "_find_ros_map_yaml",
}
MANUAL_HOOKS = {
    "manual_step_payload",
    "manual_stop_payload",
    "note_external_control_takeover",
}
SNAPSHOT_HOOKS = {
    "sidebar_payload",
    "state_payload",
    "runtime_step",
    "tick_payload",
    "_state_with_context",
    "_result_with_context",
}


def test_component_signatures_match_fleet_facade_hooks() -> None:
    groups = (
        (FleetContextService, CONTEXT_HOOKS),
        (FleetMapService, MAP_HOOKS),
        (FleetManualControlService, MANUAL_HOOKS),
        (FleetSnapshotService, SNAPSHOT_HOOKS),
    )

    for component, hooks in groups:
        for hook in hooks:
            assert inspect.signature(
                getattr(OperatorFleetManager, hook)
            ) == inspect.signature(getattr(component, hook))


def test_facade_components_are_lazy_for_lightweight_test_doubles() -> None:
    manager = OperatorFleetManager.__new__(
        OperatorFleetManager
    )

    context = manager._context_service
    snapshot = manager._snapshot_service

    assert isinstance(context, FleetContextService)
    assert isinstance(snapshot, FleetSnapshotService)
    assert manager._context_service is context
    assert manager._snapshot_service is snapshot


def test_refactor_guard_lives_outside_production_core() -> None:
    root = Path(__file__).resolve().parents[1]

    assert not (
        root
        / "operator_app"
        / "core"
        / "benchmark_service_refactor.py"
    ).exists()
    assert (
        root
        / "operator_app"
        / "benchmarking"
        / "operator_fleet_refactor_guard.py"
    ).is_file()
