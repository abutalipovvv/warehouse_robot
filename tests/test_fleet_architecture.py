from __future__ import annotations

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


def test_fleet_manager_package_initializers_stay_empty() -> None:
    package_root = Path(__file__).resolve().parents[1] / "fleet_manager"

    non_empty = [
        path.relative_to(package_root)
        for path in package_root.rglob("__init__.py")
        if path.read_text(encoding="utf-8")
    ]

    assert non_empty == []


def test_simulation_params_refresh_does_not_install_a_grpc_gateway() -> None:
    landmarks, edges = _graph()
    manager = FleetManagerSim(landmarks, edges, params={"fleet": {}})

    manager.update_world({"params": {"fleet": {}}})

    assert manager.robot_gateway.transport == "simulation"
