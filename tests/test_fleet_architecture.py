from __future__ import annotations

import ast
from pathlib import Path

from fleet_manager.core.manager import FleetManagerCore
from fleet_manager.core.models import FleetOrder
from fleet_manager.core.models import FleetRobot
from fleet_manager.core.route_core.models import GraphEdge, Landmark, WorldPoint
from fleet_manager.runtime.grpc.manager import FleetManagerGrpc, FleetManagerROS
from fleet_manager.runtime.simulation.manager import FleetManagerSim


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


def test_fleet_manager_package_initializers_are_declarative_only() -> None:
    package_root = Path(__file__).resolve().parents[1] / "fleet_manager"

    violations: list[tuple[Path, int, str]] = []
    for path in package_root.rglob("__init__.py"):
        module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for statement in module.body:
            is_docstring = (
                isinstance(statement, ast.Expr)
                and isinstance(statement.value, ast.Constant)
                and isinstance(statement.value.value, str)
            )
            is_local_export = (
                isinstance(statement, ast.ImportFrom)
                and statement.level == 1
            )
            is_all_declaration = (
                isinstance(statement, ast.Assign)
                and len(statement.targets) == 1
                and isinstance(statement.targets[0], ast.Name)
                and statement.targets[0].id == "__all__"
            )
            if not (is_docstring or is_local_export or is_all_declaration):
                violations.append(
                    (
                        path.relative_to(package_root),
                        statement.lineno,
                        type(statement).__name__,
                    )
                )

    assert violations == []


def test_transport_neutral_core_does_not_import_runtime_packages() -> None:
    core_root = (
        Path(__file__).resolve().parents[1]
        / "fleet_manager"
        / "core"
    )
    violations: list[tuple[Path, int, str]] = []

    for path in core_root.rglob("*.py"):
        module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(module):
            imported_modules: list[str] = []
            if isinstance(node, ast.Import):
                imported_modules = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules = [node.module]
            for imported_module in imported_modules:
                if imported_module.startswith("fleet_manager.runtime"):
                    violations.append(
                        (
                            path.relative_to(core_root),
                            node.lineno,
                            imported_module,
                        )
                    )

    assert violations == []


def test_foundation_packages_do_not_import_policy_or_runtime_layers() -> None:
    project_root = Path(__file__).resolve().parents[1]
    forbidden_prefixes = (
        "fleet_manager.core",
        "fleet_manager.runtime",
        "operator_app",
    )
    violations: list[tuple[Path, int, str]] = []

    for package_name in ("math", "search", "map_data"):
        package_root = project_root / "fleet_manager" / package_name
        for path in package_root.rglob("*.py"):
            module = ast.parse(
                path.read_text(encoding="utf-8"),
                filename=str(path),
            )
            for node in ast.walk(module):
                modules: list[str] = []
                if isinstance(node, ast.Import):
                    modules = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module:
                    modules = [node.module]
                for imported in modules:
                    if imported.startswith(forbidden_prefixes):
                        violations.append(
                            (
                                path.relative_to(project_root),
                                node.lineno,
                                imported,
                            )
                        )

    assert violations == []


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
