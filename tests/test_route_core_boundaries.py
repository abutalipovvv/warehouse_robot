from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
ROBOT_PLANNER_SRC = ROOT / "sim_robot" / "ws" / "src" / "robot_planner"
if str(ROBOT_PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(ROBOT_PLANNER_SRC))


def test_route_core_packages_are_owned_by_their_applications() -> None:
    from fleet_manager.route_core import Landmark as FleetLandmark
    from fleet_manager.route_core import WarehouseMapLoader as FleetLoader
    from robot_planner.route_core import Landmark as RobotLandmark
    from robot_planner.route_core import WarehouseMapLoader as RobotLoader

    assert FleetLoader.__module__.startswith("fleet_manager.route_core")
    assert RobotLoader.__module__.startswith("robot_planner.route_core")
    assert FleetLandmark.__module__.startswith("fleet_manager.route_core")
    assert RobotLandmark.__module__.startswith("robot_planner.route_core")
    assert FleetLoader is not RobotLoader
    assert FleetLandmark is not RobotLandmark


def test_contextual_default_params_paths_are_separate() -> None:
    from fleet_manager.route_core import DEFAULT_PARAMS_PATH as fleet_params_path
    from robot_planner.route_core import DEFAULT_PARAMS_PATH as robot_params_path

    assert fleet_params_path == ROOT / "fleet_manager" / "params.yaml"
    assert robot_params_path == ROOT / "sim_robot" / "ws" / "src" / "params.yaml"


def test_fleet_and_robot_params_keep_separate_defaults() -> None:
    from fleet_manager.route_core import load_route_params as load_fleet_params
    from robot_planner.route_core import load_route_params as load_robot_params

    fleet_params = load_fleet_params()
    robot_params = load_robot_params()

    assert fleet_params["navigation"]["route_speed"] == 1.37
    assert "fleet" in fleet_params
    assert robot_params["navigation"]["route_speed"] == 1
    assert "fleet" not in robot_params


def test_contextual_route_planners_load_local_params() -> None:
    from fleet_manager.route_core import GraphEdge, Landmark, LmRoutePlanner, WorldPoint
    from robot_planner.route_core import GraphEdge as RobotGraphEdge
    from robot_planner.route_core import Landmark as RobotLandmark
    from robot_planner.route_core import LmRoutePlanner as RobotLmRoutePlanner
    from robot_planner.route_core import WorldPoint as RobotWorldPoint

    start = Landmark(name="A", x=0.0, y=0.0)
    goal = Landmark(name="B", x=1.0, y=0.0)
    landmarks = {"A": start, "B": goal}
    edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
    )

    fleet_planner = LmRoutePlanner(landmarks, [edge])
    robot_planner = RobotLmRoutePlanner(
        {"A": RobotLandmark(name="A", x=0.0, y=0.0), "B": RobotLandmark(name="B", x=1.0, y=0.0)},
        [
            RobotGraphEdge(
                from_name="A",
                to_name="B",
                length=1.0,
                kind="line",
                edge_type="FeatureLine",
                world_points=(RobotWorldPoint(0.0, 0.0), RobotWorldPoint(1.0, 0.0)),
            )
        ],
    )

    assert fleet_planner.params["navigation"]["route_speed"] == 1.37
    assert "fleet" in fleet_planner.params
    assert robot_planner.params["navigation"]["route_speed"] == 1
    assert "fleet" not in robot_planner.params
