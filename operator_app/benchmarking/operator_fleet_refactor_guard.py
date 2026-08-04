#!/usr/bin/env python3
"""Differential and latency guard for OperatorFleetManager services.

Use ``--legacy-source`` while the refactor is uncommitted. If it is committed
separately, ``--legacy-ref HEAD^`` compares it with the previous revision.
"""

from __future__ import annotations

import argparse
import gc
import importlib.util
from pathlib import Path
import random
import statistics
import subprocess
import sys
from time import perf_counter_ns
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import fleet_manager.manager.manager as manager_module
import fleet_manager.robot.model as models_module
from fleet_manager.manager.tasks.models import FleetOrder
import operator_app.core.fleet_benchmark_commands as command_service_module
import operator_app.core.fleet_dynamic_benchmark as dynamic_service_module
from operator_app.core.fleet_manager import (
    FLEET_MANAGER_SIM_ID,
    OperatorFleetManager,
)
from operator_app.core.fleet_context import (
    DEFAULT_FLEET_MAP_DIR,
    DEFAULT_FLEET_SIM_MAP_DIR,
)


FLEET_MANAGER_SOURCE = "operator_app/core/fleet_manager.py"


def load_legacy_class(
    *,
    git_ref: str,
    source_path: Path | None,
) -> type:
    if source_path is not None:
        source = source_path.read_text(encoding="utf-8")
        source_label = str(source_path)
    else:
        source = subprocess.run(
            ["git", "show", f"{git_ref}:{FLEET_MANAGER_SOURCE}"],
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        source_label = f"{git_ref}:{FLEET_MANAGER_SOURCE}"
    module_name = "operator_app.core._fleet_manager_refactor_legacy"
    spec = importlib.util.spec_from_loader(module_name, loader=None)
    if spec is None:
        raise RuntimeError("could not create a legacy module")
    module = importlib.util.module_from_spec(spec)
    module.__package__ = "operator_app.core"
    module.__file__ = str(
        PROJECT_ROOT / FLEET_MANAGER_SOURCE
    )
    sys.modules[module_name] = module
    exec(
        compile(source, f"<{source_label}>", "exec"),
        module.__dict__,
    )
    return module.OperatorFleetManager


def build_services(
    legacy_class: type,
) -> tuple[Any, OperatorFleetManager]:
    params_path = (
        DEFAULT_FLEET_MAP_DIR.parents[2]
        / "config"
        / "params.yaml"
    )
    arguments = {
        "manager_id": FLEET_MANAGER_SIM_ID,
        "mode": "simulation",
    }
    return (
        legacy_class(
            DEFAULT_FLEET_SIM_MAP_DIR,
            params_path,
            **arguments,
        ),
        OperatorFleetManager(
            DEFAULT_FLEET_SIM_MAP_DIR,
            params_path,
            **arguments,
        ),
    )


def run_facade_component_differential(
    legacy_class: type,
) -> int:
    """Compare representative context, map, snapshot and manual commands."""
    legacy, refactored = build_services(legacy_class)
    matched = 0

    def assert_same(label: str, old: Any, new: Any) -> None:
        nonlocal matched
        if old != new:
            raise AssertionError(
                f"{label} differs:\nlegacy={old!r}\n"
                f"refactored={new!r}"
            )
        matched += 1

    try:
        assert_same(
            "mode-payload",
            legacy.mode_payload(),
            refactored.mode_payload(),
        )
        assert_same(
            "params-payload",
            legacy.params_payload(),
            refactored.params_payload(),
        )
        assert_same(
            "map-payload",
            legacy.map_payload(),
            refactored.map_payload(),
        )
        assert_same(
            "active-map-payload",
            legacy.maps_active_payload(),
            refactored.maps_active_payload(),
        )
        assert_same(
            "map-list-payload",
            legacy.maps_list_payload(),
            refactored.maps_list_payload(),
        )
        assert_same(
            "pull-map-payload",
            legacy.pull_map_payload(),
            refactored.pull_map_payload(),
        )
        assert_same(
            "scene-payload",
            legacy.scene3d_payload(),
            refactored.scene3d_payload(),
        )
        assert_same(
            "sidebar-payload",
            legacy.sidebar_payload(include_runtime=False),
            refactored.sidebar_payload(
                include_runtime=False
            ),
        )
        assert_same(
            "state-payload",
            legacy.state_payload(),
            refactored.state_payload(),
        )
        assert_same(
            "active-robot-modes",
            legacy._active_robot_modes(),
            refactored._active_robot_modes(),
        )
        assert_same(
            "empty-map-resolution",
            str(legacy._resolve_map_dir_by_name("")),
            str(refactored._resolve_map_dir_by_name("")),
        )
        add_payload = {
            "action": "add",
            "count": 1,
            "seed": 42,
            "reset": False,
        }
        assert_same(
            "manual-add-robot",
            legacy.benchmark_payload(dict(add_payload)),
            refactored.benchmark_payload(dict(add_payload)),
        )
        old_robot = legacy._benchmark_sim_robots()[0]
        new_robot = refactored._benchmark_sim_robots()[0]
        assert_same(
            "manual-robot-name",
            old_robot.name,
            new_robot.name,
        )
        old_lm = legacy.loaded_map.landmarks[
            old_robot.current_lm
        ]
        new_lm = refactored.loaded_map.landmarks[
            new_robot.current_lm
        ]
        assert_same(
            "manual-current-landmark",
            (old_lm.name, old_lm.x, old_lm.y),
            (new_lm.name, new_lm.x, new_lm.y),
        )
        old_pose = {
            "x": float(old_lm.x),
            "y": float(old_lm.y),
            "yaw": 0.0,
        }
        new_pose = {
            "x": float(new_lm.x),
            "y": float(new_lm.y),
            "yaw": 0.0,
        }
        assert_same(
            "manual-step",
            legacy.manual_step_payload(
                {
                    "name": old_robot.name,
                    "poses": [old_pose],
                    "currentLm": old_lm.name,
                    "nextPose": old_pose,
                }
            ),
            refactored.manual_step_payload(
                {
                    "name": new_robot.name,
                    "poses": [new_pose],
                    "currentLm": new_lm.name,
                    "nextPose": new_pose,
                }
            ),
        )
        assert_same(
            "manual-stop",
            legacy.manual_stop_payload(
                {
                    "name": old_robot.name,
                    "currentLm": old_lm.name,
                    "pose": old_pose,
                }
            ),
            refactored.manual_stop_payload(
                {
                    "name": new_robot.name,
                    "currentLm": new_lm.name,
                    "pose": new_pose,
                }
            ),
        )
        map_name = legacy.map_dir.name
        assert_same(
            "reload-map",
            legacy.load_map_payload(
                {"mapName": map_name}
            ),
            refactored.load_map_payload(
                {"mapName": map_name}
            ),
        )
    finally:
        legacy.close()
        refactored.close()
    return matched


def run_seeded_differential(
    legacy: Any,
    refactored: OperatorFleetManager,
) -> int:
    matched = 0

    def assert_same(label: str, old: Any, new: Any) -> None:
        nonlocal matched
        if old != new:
            raise AssertionError(
                f"{label} differs:\nlegacy={old!r}\n"
                f"refactored={new!r}"
            )
        matched += 1

    for seed in (7, 42, 99):
        assert_same(
            f"spawn-{seed}",
            legacy._benchmark_spawn_lms(6, seed),
            refactored._benchmark_spawn_lms(6, seed),
        )
        for stress in (False, True):
            for profile in (0, 2):
                arguments = {
                    "count": 4,
                    "seed": seed,
                    "stress": stress,
                    "stress_profile": profile,
                }
                assert_same(
                    f"requests-{seed}-{stress}-{profile}",
                    legacy._benchmark_requests(**arguments),
                    refactored._benchmark_requests(**arguments),
                )

    plans = [
        {"nodes": ["A", "A", "B"], "times": [0, 2, 5]},
        {"nodes": ["C", "D"], "times": [0, 4]},
    ]
    assert_same(
        "plan-stats",
        legacy._benchmark_plan_stats(plans, 0.5),
        refactored._benchmark_plan_stats(plans, 0.5),
    )
    add_payload = {
        "action": "add",
        "count": 6,
        "seed": 42,
        "reset": False,
    }
    assert_same(
        "add-command",
        legacy.benchmark_payload(dict(add_payload)),
        refactored.benchmark_payload(dict(add_payload)),
    )

    for seed in (7, 42, 99):
        arguments = {
            "count": 4,
            "seed": seed,
            "stress": True,
            "stress_profile": 1,
        }
        assert_same(
            f"existing-requests-{seed}",
            legacy._benchmark_requests_for_existing(**arguments),
            refactored._benchmark_requests_for_existing(**arguments),
        )
        legacy._dynamic_rng = random.Random(seed)
        refactored._dynamic_rng = random.Random(seed)
        old_robots = legacy._benchmark_sim_robots()[:4]
        new_robots = refactored._benchmark_sim_robots()[:4]
        assert_same(
            f"package-wave-{seed}",
            [
                (robot.name, goal)
                for robot, goal
                in legacy._package_wave_assignments(
                    old_robots,
                    2,
                )
            ],
            [
                (robot.name, goal)
                for robot, goal
                in refactored._package_wave_assignments(
                    new_robots,
                    2,
                )
            ],
        )

    old_names = [
        robot.name
        for robot in legacy._benchmark_sim_robots()[:2]
    ]
    new_names = [
        robot.name
        for robot in refactored._benchmark_sim_robots()[:2]
    ]
    assert_same("active-order-robot-names", old_names, new_names)
    active_order_id = "refactor-differential-active"
    terminal_order_id = "refactor-differential-terminal"
    for service, names in (
        (legacy, old_names),
        (refactored, new_names),
    ):
        service.manager.orders[active_order_id] = FleetOrder(
            order_id=active_order_id,
            target_lm="",
            vehicle=names[0],
            assigned_robot=names[1],
            status="EXECUTING",
            created_at=1000.0,
            updated_at=1000.0,
        )
        service.manager.orders[terminal_order_id] = FleetOrder(
            order_id=terminal_order_id,
            target_lm="",
            vehicle=names[0],
            assigned_robot=names[0],
            status="COMPLETED",
            created_at=1000.0,
            updated_at=1000.0,
        )
        service._dynamic_benchmark.update(
            {
                "active": True,
                "generationMode": "continuous",
                "queueDepth": -1,
                "initialWaveQueued": True,
                "nextOrderAt": {},
            }
        )
    for index in range(2):
        assert_same(
            f"active-order-depth-{index}",
            legacy._dynamic_order_depth(old_names[index]),
            refactored._dynamic_order_depth(new_names[index]),
        )
    assert_same(
        "active-pump",
        legacy._pump_dynamic_benchmark(now=1000.5),
        refactored._pump_dynamic_benchmark(now=1000.5),
    )
    assert_same(
        "active-pump-state",
        legacy._dynamic_benchmark,
        refactored._dynamic_benchmark,
    )
    for service in (legacy, refactored):
        service.manager.orders.pop(active_order_id)
        service.manager.orders.pop(terminal_order_id)

    for service in (legacy, refactored):
        service._dynamic_benchmark.update(
            {
                "active": False,
                "startedAt": 1000.0,
                "lastPumpAt": 999.0,
                "ordersGenerated": 5,
                "generatedDistanceMTotal": 25.0,
            }
        )
    assert_same(
        "inactive-pump",
        legacy._pump_dynamic_benchmark(now=1001.0),
        refactored._pump_dynamic_benchmark(now=1001.0),
    )
    assert_same(
        "pump-state",
        legacy._dynamic_benchmark,
        refactored._dynamic_benchmark,
    )
    assert_same(
        "dynamic-payload",
        legacy._dynamic_benchmark_payload(),
        refactored._dynamic_benchmark_payload(),
    )
    return matched


def run_start_command_differential(legacy_class: type) -> int:
    """Compare both continuous and package-wave startup transactions."""
    matched = 0
    for action in ("plan", "package_waves"):
        legacy, refactored = build_services(legacy_class)
        try:
            add_payload = {
                "action": "add",
                "count": 6,
                "seed": 42,
                "reset": False,
            }
            legacy.benchmark_payload(dict(add_payload))
            refactored.benchmark_payload(dict(add_payload))
            start_payload = {
                "action": action,
                "count": 6,
                "seed": 42,
                "reset": False,
                "horizonSec": 5,
                "orderIntervalSec": 1,
                "queueDepth": 1,
            }
            old_result = legacy.benchmark_payload(
                dict(start_payload)
            )
            new_result = refactored.benchmark_payload(
                dict(start_payload)
            )
            if old_result != new_result:
                raise AssertionError(
                    f"{action} start result differs:\n"
                    f"legacy={old_result!r}\n"
                    f"refactored={new_result!r}"
                )
            matched += 1
            if legacy._dynamic_benchmark != (
                refactored._dynamic_benchmark
            ):
                raise AssertionError(
                    f"{action} start state differs:\n"
                    f"legacy={legacy._dynamic_benchmark!r}\n"
                    "refactored="
                    f"{refactored._dynamic_benchmark!r}"
                )
            matched += 1
        finally:
            legacy.close()
            refactored.close()
    return matched


def run_hot_benchmarks(
    legacy: Any,
    refactored: OperatorFleetManager,
    *,
    robot_count: int,
    threshold: float,
) -> dict[str, dict[str, float]]:
    add_payload = {
        "action": "add",
        "count": robot_count,
        "seed": 42,
        "reset": False,
    }
    legacy.benchmark_payload(dict(add_payload))
    refactored.benchmark_payload(dict(add_payload))
    for service in (legacy, refactored):
        service._dynamic_benchmark.update(
            {
                "active": True,
                "generationMode": "continuous",
                "startedAt": 1000.0,
                "lastPumpAt": 1000.0,
                "queueDepth": -1,
                "initialWaveQueued": True,
                "nextOrderAt": {},
            }
        )

    paths = {
        "runtime_pump": (
            lambda: legacy._pump_dynamic_benchmark(now=1001.0),
            lambda: refactored._pump_dynamic_benchmark(now=1001.0),
            500,
            40,
        ),
        "benchmark_payload": (
            legacy._dynamic_benchmark_payload,
            refactored._dynamic_benchmark_payload,
            500,
            40,
        ),
        "request_generation": (
            lambda: legacy._benchmark_requests_for_existing(
                count=robot_count,
                seed=42,
                stress=True,
                stress_profile=1,
            ),
            lambda: refactored._benchmark_requests_for_existing(
                count=robot_count,
                seed=42,
                stress=True,
                stress_profile=1,
            ),
            1,
            20,
        ),
    }
    results: dict[str, dict[str, float]] = {}
    for name, (
        old_call,
        new_call,
        batch_size,
        samples,
    ) in paths.items():
        result = _measure_alternating(
            old_call,
            new_call,
            batch_size=batch_size,
            samples=samples,
        )
        if result["ratio"] > threshold:
            raise AssertionError(
                f"{name} slowdown {result['ratio']:.4f} exceeds "
                f"{threshold:.4f}"
            )
        results[name] = result
    return results


def _measure_alternating(
    legacy: Callable[[], Any],
    refactored: Callable[[], Any],
    *,
    batch_size: int,
    samples: int,
) -> dict[str, float]:
    for _ in range(5):
        legacy()
        refactored()
    old_samples: list[float] = []
    new_samples: list[float] = []
    paired_ratios: list[float] = []
    gc.disable()
    try:
        for sample_index in range(samples):
            calls = (
                (
                    ("legacy", legacy, old_samples),
                    ("refactored", refactored, new_samples),
                )
                if sample_index % 2 == 0
                else (
                    ("refactored", refactored, new_samples),
                    ("legacy", legacy, old_samples),
                )
            )
            pair: dict[str, float] = {}
            for label, callback, measurements in calls:
                started = perf_counter_ns()
                for _ in range(batch_size):
                    callback()
                elapsed = (perf_counter_ns() - started) / batch_size
                measurements.append(elapsed)
                pair[label] = elapsed
            paired_ratios.append(
                pair["refactored"] / pair["legacy"]
            )
    finally:
        gc.enable()
    legacy_ns = statistics.median(old_samples)
    refactored_ns = statistics.median(new_samples)
    return {
        "legacy_ns": legacy_ns,
        "refactored_ns": refactored_ns,
        "ratio": statistics.median(paired_ratios),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy-ref", default="HEAD^")
    parser.add_argument("--legacy-source", type=Path)
    parser.add_argument("--robots", type=int, default=20)
    parser.add_argument("--max-slowdown", type=float, default=1.05)
    args = parser.parse_args()

    fixed_time = lambda: 1000.0
    manager_module.time = fixed_time
    models_module.time = fixed_time
    legacy_class = load_legacy_class(
        git_ref=args.legacy_ref,
        source_path=args.legacy_source,
    )
    fixed_counter = lambda: 2000.0
    legacy_module = sys.modules[legacy_class.__module__]
    legacy_module.perf_counter = fixed_counter
    command_service_module.perf_counter = fixed_counter
    dynamic_service_module.perf_counter = fixed_counter
    legacy, refactored = build_services(legacy_class)
    try:
        matches = run_facade_component_differential(
            legacy_class
        )
        matches += run_seeded_differential(
            legacy,
            refactored,
        )
        matches += run_start_command_differential(legacy_class)
        benchmarks = run_hot_benchmarks(
            legacy,
            refactored,
            robot_count=max(1, args.robots),
            threshold=max(1.0, args.max_slowdown),
        )
    finally:
        legacy.close()
        refactored.close()

    print(f"differential: {matches} exact representative matches")
    for name, result in benchmarks.items():
        print(
            f"{name}: legacy={result['legacy_ns']:.1f} ns, "
            f"refactored={result['refactored_ns']:.1f} ns, "
            f"ratio={result['ratio']:.4f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
