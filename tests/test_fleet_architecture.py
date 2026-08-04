from __future__ import annotations

import ast
from pathlib import Path

from fleet_manager.manager.manager import FleetManagerCore
from fleet_manager.manager.tasks.models import FleetOrder
from fleet_manager.manager.scheduler import PlanningWorker
from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.grpc.manager import FleetManagerGrpc, FleetManagerROS
from fleet_manager.runtime.simulation.manager import FleetManagerSim


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = PROJECT_ROOT / "fleet_manager"


def _absolute_imports(path: Path) -> list[tuple[int, str]]:
    module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports: list[tuple[int, str]] = []
    for node in ast.walk(module):
        if isinstance(node, ast.Import):
            imports.extend((node.lineno, alias.name) for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            imports.append((node.lineno, node.module))
    return imports


class _FakeGrpcGateway:
    transport = "grpc"


def _graph() -> tuple[dict[str, Landmark], list[GraphEdge]]:
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    edges = [
        GraphEdge(
            from_name="A",
            to_name="B",
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
            properties={"direction": 1},
        )
    ]
    return landmarks, edges


def test_sim_and_ros_managers_share_one_transport_neutral_core() -> None:
    landmarks, edges = _graph()
    sim = FleetManagerSim(landmarks, edges, params={"fleet": {}})
    gateway = _FakeGrpcGateway()
    real = FleetManagerROS(
        landmarks,
        edges,
        params={"fleet": {}},
        remote_adapter=gateway,
    )

    assert issubclass(FleetManagerSim, FleetManagerCore)
    assert issubclass(FleetManagerROS, FleetManagerCore)
    assert FleetManagerGrpc is FleetManagerROS
    assert sim.runtime_kind == "simulation"
    assert sim.robot_gateway.transport == "simulation"
    assert sim.active_robot_modes == {"simulated"}
    assert real.runtime_kind == "grpc"
    assert real.robot_gateway is gateway
    assert real.active_robot_modes == {"remote"}


def test_task_manager_owns_the_core_order_storage() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerCore(landmarks, edges, params={"fleet": {}})
    order = FleetOrder(order_id="o1", target_lm="B", vehicle="r1")

    manager.orders[order.order_id] = order

    assert manager.task_manager.orders is manager.orders
    assert manager.task_manager.active_for_robot("r1") is order

    replacement = {"o2": FleetOrder(order_id="o2", target_lm="A")}
    manager.orders = replacement
    assert manager.task_manager.orders is replacement


def test_packages_do_not_hide_runtime_classes_behind_reexports() -> None:
    import fleet_manager
    import fleet_manager.core
    import fleet_manager.runtime

    assert vars(fleet_manager).keys().isdisjoint({"FleetManager", "FleetManagerCore"})
    assert "FleetManagerCore" not in vars(fleet_manager.core)
    assert "FleetManagerSim" not in vars(fleet_manager.runtime)


def test_fleet_manager_package_initializers_do_not_reexport_names() -> None:
    violations: list[tuple[Path, int, str]] = []
    for path in PACKAGE_ROOT.rglob("__init__.py"):
        module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for statement in module.body:
            is_docstring = (
                isinstance(statement, ast.Expr)
                and isinstance(statement.value, ast.Constant)
                and isinstance(statement.value.value, str)
            )
            if not is_docstring:
                violations.append(
                    (
                        path.relative_to(PACKAGE_ROOT),
                        statement.lineno,
                        type(statement).__name__,
                    )
                )

    assert violations == []


def test_package_dependencies_follow_core_robot_manager_runtime_order() -> None:
    forbidden_by_package = {
        "core": (
            "fleet_manager.manager",
            "fleet_manager.robot",
            "fleet_manager.runtime",
            "operator_app",
        ),
        "robot": (
            "fleet_manager.manager",
            "fleet_manager.runtime",
            "operator_app",
        ),
        "manager": (
            "fleet_manager.runtime",
            "operator_app",
        ),
    }
    violations: list[tuple[Path, int, str]] = []
    for package_name, forbidden_prefixes in forbidden_by_package.items():
        package_root = PACKAGE_ROOT / package_name
        for path in package_root.rglob("*.py"):
            for line, imported_module in _absolute_imports(path):
                if imported_module.startswith(forbidden_prefixes):
                    violations.append(
                        (
                            path.relative_to(PACKAGE_ROOT),
                            line,
                            imported_module,
                        )
                    )

    assert violations == []


def test_source_has_no_wildcard_imports_or_all_declarations() -> None:
    violations: list[tuple[Path, int, str]] = []
    for path in PACKAGE_ROOT.rglob("*.py"):
        module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(module):
            if isinstance(node, ast.ImportFrom) and any(
                alias.name == "*" for alias in node.names
            ):
                violations.append((path.relative_to(PACKAGE_ROOT), node.lineno, "*"))
            if isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                if any(
                    isinstance(target, ast.Name) and target.id == "__all__"
                    for target in targets
                ):
                    violations.append(
                        (path.relative_to(PACKAGE_ROOT), node.lineno, "__all__")
                    )

    assert violations == []


def test_removed_architecture_paths_do_not_return() -> None:
    removed_paths = (
        "core/algorithms",
        "core/io",
        "core/fleet",
        "core/tasks",
        "core/transport",
        "core/manager_state.py",
        "core/planning_models.py",
        "core/planning_scheduler.py",
        "core/mapf/cbs/lm_cbs.py",
        "core/mapf/graph/traffic_graph.py",
    )
    assert [path for path in removed_paths if (PACKAGE_ROOT / path).exists()] == []


def test_planning_worker_has_one_submission_api() -> None:
    assert hasattr(PlanningWorker, "submit_job")
    assert not hasattr(PlanningWorker, "submit")
    assert not hasattr(PlanningWorker, "publish_result")

    source = (PACKAGE_ROOT / "manager/tasks/planning_jobs.py").read_text(
        encoding="utf-8"
    )
    assert "_submit_legacy_planning_hook" not in source
    assert "_LegacyPlanningResult" not in source


def test_simulation_params_refresh_does_not_install_a_grpc_gateway() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerSim(landmarks, edges, params={"fleet": {}})

    manager.update_world({"params": {"fleet": {}}})

    assert manager.robot_gateway.transport == "simulation"


def test_simulation_uses_transport_neutral_stop_and_order_replacement_hooks() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerSim(landmarks, edges, params={"fleet": {}})
    manager.add_robot({"name": "r1", "spawnLm": "A", "mode": "simulated"})

    first = manager.set_order({"id": "o1", "vehicle": "r1", "targetLm": "B"})
    replacement = manager.set_order(
        {
            "id": "o2",
            "vehicle": "r1",
            "targetLm": "B",
            "replaceActive": True,
        }
    )
    stopped = manager.stop_robot({"name": "r1"})

    assert first["ok"] is True
    assert replacement["ok"] is True
    assert manager.orders["o1"].status == "CANCELED"
    assert stopped["robot"]["status"] == "STOPPED"


def test_manual_off_graph_pose_reconnects_before_processing_order() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerSim(landmarks, edges, params={"fleet": {}})
    manager.add_robot({"name": "r1", "spawnLm": "A", "mode": "simulated"})
    manager.update_robot(
        {
            "name": "r1",
            "status": "IDLE",
            "targetLm": "",
            "pose": {"x": 0.35, "y": 0.0, "yaw": 0.0},
        }
    )

    result = manager.set_order(
        {"id": "manual-goal", "vehicle": "r1", "targetLm": "B", "replaceActive": True}
    )
    robot = manager.robots["r1"]

    assert result["ok"] is True
    assert robot.status == "MOVING"
    assert robot.route_note == "manual graph reconnect"
    assert robot.pose is not None
    assert robot.pose["x"] > 0.34
    assert abs(robot.pose["y"]) < 0.001
    assert robot.trajectory[0]["x"] == 0.35
    assert manager.orders["manual-goal"].status == "EXECUTING"

    robot.pose = dict(robot.trajectory[-1])
    robot.current_lm = robot.route_chunk_goal_lm
    assert manager._complete_simulated_route_chunk(robot, manager.simulation_time()) is True
    manager._dispatch_orders()

    assert robot.status == "MOVING"
    assert robot.target_lm == "B"
    assert robot.route_note != "manual graph reconnect"
    assert manager.orders["manual-goal"].status == "EXECUTING"


def test_async_dispatch_completes_order_already_at_target() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerSim(landmarks, edges, params={"fleet": {}})
    manager.add_robot({"name": "r1", "spawnLm": "A", "mode": "simulated"})
    manager.set_order(
        {"id": "already-there", "vehicle": "r1", "targetLm": "A"},
        dispatch=False,
    )

    manager._dispatch_orders(async_simulated=True)

    assert manager.orders["already-there"].status == "COMPLETED"
    assert manager.robots["r1"].status == "ARRIVED"
