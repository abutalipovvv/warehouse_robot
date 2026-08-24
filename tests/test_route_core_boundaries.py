from pathlib import Path
import math
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
ROBOT_PLANNER_SRC = (
    ROOT / "robot" / "robot_driver" / "src" / "robot_planner"
)
if str(ROBOT_PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(ROBOT_PLANNER_SRC))


def test_route_core_packages_are_owned_by_their_applications() -> None:
    from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader as FleetLoader
    from fleet_manager.core.mapping.maps.models import Landmark as FleetLandmark
    from robot_planner.route_core import Landmark as RobotLandmark
    from robot_planner.route_core import WarehouseMapLoader as RobotLoader

    assert FleetLoader.__module__.startswith("fleet_manager.core.mapping.maps")
    assert RobotLoader.__module__.startswith("robot_planner.route_core")
    assert FleetLandmark.__module__.startswith("fleet_manager.core.mapping.maps")
    assert RobotLandmark.__module__.startswith("robot_planner.route_core")
    assert FleetLoader is not RobotLoader
    assert FleetLandmark is not RobotLandmark


def test_contextual_default_params_paths_are_separate() -> None:
    from fleet_manager.core.mapping.navigation.params import DEFAULT_PARAMS_PATH as fleet_params_path
    from robot_planner.route_core import DEFAULT_PARAMS_PATH as robot_params_path

    assert fleet_params_path == ROOT / "fleet_manager" / "config" / "params.yaml"
    assert robot_params_path == (
        ROOT / "robot" / "robot_driver" / "src" / "params" / "params.yaml"
    )


def test_fleet_and_robot_params_keep_separate_defaults() -> None:
    from fleet_manager.core.mapping.navigation.params import load_route_params as load_fleet_params
    from robot_planner.route_core import load_route_params as load_robot_params

    fleet_params = load_fleet_params()
    robot_params = load_robot_params()

    assert fleet_params["navigation"]["route_speed"] == 1.37
    assert "fleet" in fleet_params
    assert robot_params["navigation"]["route_speed"] == 1
    assert "fleet" not in robot_params


def test_contextual_route_planners_load_local_params() -> None:
    from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, WorldPoint
    from fleet_manager.core.mapping.navigation.planner import LmRoutePlanner
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


def test_edge_direction_is_strict_and_backward_heading_matches_in_both_runtimes() -> None:
    from fleet_manager.core.mapping.maps.models import GraphEdge, Landmark, PlannedRoute, WorldPoint
    from fleet_manager.core.mapping.navigation.planner import LmRoutePlanner
    from robot_planner.route_core import GraphEdge as RobotGraphEdge
    from robot_planner.route_core import Landmark as RobotLandmark
    from robot_planner.route_core import LmRoutePlanner as RobotLmRoutePlanner
    from robot_planner.route_core import PlannedRoute as RobotPlannedRoute
    from robot_planner.route_core import WorldPoint as RobotWorldPoint

    fleet_landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    fleet_edge = GraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
        properties={"direction": 1},
    )
    fleet_planner = LmRoutePlanner(fleet_landmarks, [fleet_edge])
    fleet_samples = fleet_planner.sample_route(
        PlannedRoute(nodes=["A", "B"], edges=[fleet_edge], length=1.0)
    )

    robot_landmarks = {
        "A": RobotLandmark(name="A", x=0.0, y=0.0),
        "B": RobotLandmark(name="B", x=1.0, y=0.0),
    }
    robot_edge = RobotGraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(RobotWorldPoint(0.0, 0.0), RobotWorldPoint(1.0, 0.0)),
        properties={"direction": 1},
    )
    robot_planner = RobotLmRoutePlanner(robot_landmarks, [robot_edge])
    robot_samples = robot_planner.sample_route(
        RobotPlannedRoute(nodes=["A", "B"], edges=[robot_edge], length=1.0)
    )

    assert fleet_samples[0]["motionDirection"] == "backward"
    assert robot_samples[0]["motionDirection"] == "backward"
    assert abs(abs(float(fleet_samples[0]["yaw"])) - math.pi) < 1e-6
    assert abs(abs(float(robot_samples[0]["yaw"])) - math.pi) < 1e-6
    with pytest.raises(ValueError, match="No route"):
        fleet_planner.find_route("B", "A")
    with pytest.raises(ValueError, match="No route"):
        robot_planner.find_route("B", "A")


def test_not_specified_motion_codes_are_normalized_without_losing_forward_zero() -> None:
    from fleet_manager.core.mapping.maps.map_writer import _normalize_edges
    from fleet_manager.core.mapping.maps.models import GraphEdge, WorldPoint

    forward = _normalize_edges(
        [{"from": "A", "to": "B", "motionDirectionCode": 0, "length": 1.0}],
        [{"name": "A", "x": 0.0, "y": 0.0}, {"name": "B", "x": 1.0, "y": 0.0}],
    )
    unspecified = GraphEdge(
        from_name="A",
        to_name="B",
        length=1.0,
        kind="line",
        edge_type="FeatureLine",
        world_points=(WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
        properties={"direction": 2},
    )

    assert forward[0]["properties"]["direction"] == 0
    assert unspecified.motion_direction_code() == -1
    assert unspecified.motion_direction_label(unspecified.motion_direction_code()) == "not_specified"


def test_map_loader_merges_routing_and_geometry_edge_properties(
    tmp_path: Path,
) -> None:
    from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader
    from fleet_manager.core.mapping.maps.models import Landmark, MapMetadata

    (tmp_path / "graphs.yaml").write_text(
        """
mapName: test
coordinateFrame: map_top_left
primitives:
  - kind: line
    start: {x: 0.0, y: 0.0}
    end: {x: 1.0, y: 0.0}
    start_name: A
    end_name: B
    properties:
      direction: 1
      smart: true
""".lstrip(),
        encoding="utf-8",
    )
    (tmp_path / "graph_edges_lengths.yaml").write_text(
        """
- from: A
  to: B
  length: 1.0
  kind: line
  type: FeatureLine
  properties:
    direction: 0
    controlled_region: corridor:test
""".lstrip(),
        encoding="utf-8",
    )
    landmarks = {
        "A": Landmark(name="A", x=0.0, y=0.0),
        "B": Landmark(name="B", x=1.0, y=0.0),
    }
    metadata = MapMetadata(
        map_name="test",
        width=10,
        height=10,
        resolution=1.0,
        ros_origin=(0.0, 0.0, 0.0),
        image_data_url="",
    )

    edges = WarehouseMapLoader(tmp_path)._load_edges(
        tmp_path / "graph_edges_lengths.yaml",
        landmarks,
        metadata,
    )

    assert len(edges) == 1
    assert edges[0].properties == {
        "direction": 1,
        "controlled_region": "corridor:test",
        "smart": True,
    }


def test_geometric_corridor_is_compiled_by_core_not_the_map_generator() -> None:
    from fleet_manager.core.mapping.maps.models import (
        GraphEdge,
        Landmark,
        TrafficZone,
    )
    from fleet_manager.core.traffic.corridors.corridors import (
        compile_controlled_corridor_zones,
    )

    landmarks = {
        name: Landmark(name=name, x=x, y=0.0)
        for name, x in (("HOLD_LEFT", 0.0), ("INNER_A", 1.0), ("INNER_B", 2.0), ("HOLD_RIGHT", 3.0))
    }
    edges = [
        GraphEdge(
            from_name=src,
            to_name=dst,
            length=1.0,
            kind="line",
            edge_type="FeatureLine",
            world_points=(
                landmarks[src].to_point(),
                landmarks[dst].to_point(),
            ),
        )
        for first, second in (
            ("HOLD_LEFT", "INNER_A"),
            ("INNER_A", "INNER_B"),
            ("INNER_B", "HOLD_RIGHT"),
        )
        for src, dst in ((first, second), (second, first))
    ]
    zone = TrafficZone(
        zone_id="corridor:any-map",
        kind="controlled_corridor",
        min_x=0.9,
        min_y=-0.2,
        max_x=2.1,
        max_y=0.2,
    )

    compiled_lms, compiled_edges = compile_controlled_corridor_zones(
        landmarks,
        edges,
        [zone],
    )

    assert compiled_lms["INNER_A"].properties["can_wait"] is False
    assert compiled_lms["INNER_B"].properties["can_wait"] is False
    assert compiled_lms["HOLD_LEFT"].properties["holding_point"] is True
    assert compiled_lms["HOLD_RIGHT"].properties["holding_point"] is True
    assert all(
        edge.properties["controlled_region"] == zone.zone_id
        for edge in compiled_edges
    )
