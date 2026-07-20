from __future__ import annotations

import math
from collections import Counter, defaultdict, deque
from pathlib import Path

import yaml

from fleet_manager.core.geometry.collision import FleetCollisionChecker
from fleet_manager.core.route_core.map_loader import WarehouseMapLoader


ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "fleet_manager" / "map_data" / "maps_out" / "smart_kiva_large_w_mode.smap"
PARAMS_PATH = ROOT / "fleet_manager" / "config" / "params.yaml"
LOWER_LANE_Y = tuple(3.8 + (4.0 * index) for index in range(7))
UPPER_LANE_Y = tuple(5.2 + (4.0 * index) for index in range(7))
REMOVED_MIDDLE_Y = tuple(4.5 + (4.0 * index) for index in range(7))
AISLE_CONNECTOR_COLUMNS = frozenset({2, 13, 24, 35})
CONTROLLED_CORRIDOR_ROW_PAIRS = tuple(
    (row, row + 2)
    for row in range(2, 31, 4)
)


def test_smart_kiva_uses_two_clearance_lanes_per_internal_aisle() -> None:
    loaded = WarehouseMapLoader(MAP_DIR).load()
    landmarks = loaded.landmarks

    assert len(landmarks) == 576
    assert all(
        not math.isclose(landmark.y, middle_y, abs_tol=1e-9)
        for landmark in landmarks.values()
        for middle_y in REMOVED_MIDDLE_Y
    )

    for aisle_index, (lower_y, upper_y) in enumerate(
        zip(LOWER_LANE_Y, UPPER_LANE_Y, strict=True)
    ):
        lower_lane = [
            landmark
            for landmark in landmarks.values()
            if math.isclose(landmark.y, lower_y)
        ]
        upper_lane = [
            landmark
            for landmark in landmarks.values()
            if math.isclose(landmark.y, upper_y)
        ]
        assert len(lower_lane) == 34
        assert len(upper_lane) == 34
        assert math.isclose(upper_y - lower_y, 1.4, abs_tol=1e-9)

        upper_shelf_edge = 3.0 + (4.0 * aisle_index)
        lower_shelf_edge = 6.0 + (4.0 * aisle_index)
        assert math.isclose(lower_y - upper_shelf_edge, 0.8, abs_tol=1e-9)
        assert math.isclose(lower_shelf_edge - upper_y, 0.8, abs_tol=1e-9)


def test_smart_kiva_two_lane_graph_is_bidirectional_and_connected() -> None:
    loaded = WarehouseMapLoader(MAP_DIR).load()
    landmarks = loaded.landmarks
    edges = loaded.edges
    edge_pairs = {(edge.from_name, edge.to_name) for edge in edges}

    assert len(edges) == 1240
    assert len(edge_pairs) == len(edges)
    assert all((goal, start) in edge_pairs for start, goal in edge_pairs)
    assert all(
        math.isclose(
            edge.length,
            math.hypot(
                landmarks[edge.to_name].x - landmarks[edge.from_name].x,
                landmarks[edge.to_name].y - landmarks[edge.from_name].y,
            ),
            abs_tol=1e-9,
        )
        for edge in edges
    )

    adjacency: dict[str, list[str]] = defaultdict(list)
    for edge in edges:
        adjacency[edge.from_name].append(edge.to_name)
    start = next(iter(landmarks))
    reached = {start}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        for neighbor in adjacency[current]:
            if neighbor in reached:
                continue
            reached.add(neighbor)
            queue.append(neighbor)
    assert reached == set(landmarks)

    for aisle_index in range(7):
        lower_row = 4 + (4 * aisle_index)
        upper_row = 6 + (4 * aisle_index)
        for column in range(2, 36):
            lower_name = f"S{lower_row:03d}{column:03d}"
            upper_name = f"S{upper_row:03d}{column:03d}"
            should_connect = column in AISLE_CONNECTOR_COLUMNS
            assert ((lower_name, upper_name) in edge_pairs) is should_connect
            assert ((upper_name, lower_name) in edge_pairs) is should_connect

    assert ("S024014", "S026014") not in edge_pairs
    assert ("S026014", "S024014") not in edge_pairs
    assert ("S024014", "S024013") in edge_pairs
    assert ("S024014", "S024015") in edge_pairs
    assert ("S024013", "S026013") in edge_pairs


def test_smart_kiva_compiles_geometric_corridors_with_external_stop_lines() -> None:
    loaded = WarehouseMapLoader(MAP_DIR).load()
    regions: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for edge in loaded.edges:
        for region_id in str(
            edge.properties.get("controlled_region") or ""
        ).split(","):
            if region_id:
                regions[region_id].append((edge.from_name, edge.to_name))

    assert len(regions) == (
        len(AISLE_CONNECTOR_COLUMNS)
        * len(CONTROLLED_CORRIDOR_ROW_PAIRS)
    )
    assert len(loaded.traffic_zones) == len(regions)

    for column in AISLE_CONNECTOR_COLUMNS:
        for top_row, bottom_row in CONTROLLED_CORRIDOR_ROW_PAIRS:
            region_id = (
                "corridor:smart-kiva:"
                f"c{column:03d}:r{top_row:03d}-r{bottom_row:03d}"
            )
            middle_row = top_row + 1
            top = f"S{top_row:03d}{column:03d}"
            middle = f"S{middle_row:03d}{column:03d}"
            bottom = f"S{bottom_row:03d}{column:03d}"
            assert (top, middle) in regions[region_id]
            assert (middle, top) in regions[region_id]
            assert (middle, bottom) in regions[region_id]
            assert (bottom, middle) in regions[region_id]
            assert loaded.landmarks[top].properties["can_wait"] is False
            assert loaded.landmarks[bottom].properties["can_wait"] is False
            assert loaded.landmarks[middle].properties["can_wait"] is False
            for name in (top, middle, bottom):
                assert region_id in str(
                    loaded.landmarks[name].properties["controlled_region"]
                ).split(",")

            # A rectangle encloses the narrow centre line.  The immediate
            # lateral LMs remain outside and act as the traffic lights.
            for row in (top_row, bottom_row):
                for side_column in (column - 1, column + 1):
                    holding = f"S{row:03d}{side_column:03d}"
                    if holding not in loaded.landmarks:
                        continue
                    assert loaded.landmarks[holding].properties["holding_point"] is True
                    assert loaded.landmarks[holding].properties["can_wait"] is True

    # The exact live trouble spots are never legal wait vertices.
    assert loaded.landmarks["S014013"].properties["can_wait"] is False
    assert loaded.landmarks["S016013"].properties["can_wait"] is False
    for name in ("S012012", "S014012", "S012014", "S014014"):
        assert loaded.landmarks[name].properties["holding_point"] is True
        assert loaded.landmarks[name].properties["can_wait"] is True


def test_smart_kiva_perimeter_uses_one_centered_clearance_lane() -> None:
    loaded = WarehouseMapLoader(MAP_DIR).load()
    landmarks = loaded.landmarks

    assert all(
        not landmark.name.startswith(("S001", "S033"))
        for landmark in landmarks.values()
    )
    assert all(
        not landmark.name.endswith(("001", "036"))
        for landmark in landmarks.values()
    )
    assert {
        landmark.x
        for landmark in landmarks.values()
        if landmark.name.endswith("002")
    } == {1.0}
    assert {
        landmark.x
        for landmark in landmarks.values()
        if landmark.name.endswith("035")
    } == {35.0}
    assert {
        landmark.y
        for landmark in landmarks.values()
        if landmark.name.startswith("S002")
    } == {1.0}
    assert {
        landmark.y
        for landmark in landmarks.values()
        if landmark.name.startswith("S032")
    } == {32.0}


def test_smart_kiva_every_turn_capable_lm_clears_the_robot_footprint() -> None:
    loaded = WarehouseMapLoader(MAP_DIR).load()
    params = yaml.safe_load(PARAMS_PATH.read_text(encoding="utf-8"))
    collision = FleetCollisionChecker(params, MAP_DIR, loaded.map_metadata)
    adjacency: dict[str, set[str]] = defaultdict(set)
    for edge in loaded.edges:
        adjacency[edge.from_name].add(edge.to_name)

    checked = 0
    for name, landmark in loaded.landmarks.items():
        horizontal = any(
            not math.isclose(loaded.landmarks[neighbor].x, landmark.x)
            for neighbor in adjacency[name]
        )
        vertical = any(
            not math.isclose(loaded.landmarks[neighbor].y, landmark.y)
            for neighbor in adjacency[name]
        )
        if not horizontal or not vertical:
            continue
        checked += 1
        for yaw in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
            assert not collision.blocked_reason(
                {"x": landmark.x, "y": landmark.y, "yaw": yaw},
                [],
                [],
            ), f"{name} cannot safely turn at yaw={yaw}"
    assert checked > 0


def test_smart_kiva_keeps_24_physical_obstacles() -> None:
    ros_map = yaml.safe_load((MAP_DIR / "smart_kiva_large_w_mode.yaml").read_text(encoding="utf-8"))
    width, height, pixels = WarehouseMapLoader(MAP_DIR)._load_pgm(MAP_DIR / ros_map["image"])
    occupied = {
        (x, y)
        for y in range(height)
        for x in range(width)
        if pixels[(y * width) + x] < 128
    }
    components: list[set[tuple[int, int]]] = []
    while occupied:
        start = occupied.pop()
        component = {start}
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for neighbor in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if neighbor not in occupied:
                    continue
                occupied.remove(neighbor)
                component.add(neighbor)
                queue.append(neighbor)
        components.append(component)

    assert len(components) == 24
    assert Counter(len(component) for component in components) == Counter(
        {960: 8, 980: 16}
    )

    bounds = sorted(
        (
            min(x for x, _y in component),
            max(x for x, _y in component),
            min(y for _x, y in component),
            max(y for _x, y in component),
        )
        for component in components
    )
    for row_start in range(20, 301, 40):
        row_bounds = [bound for bound in bounds if bound[2] == row_start]
        assert row_bounds == [
            (20, 117, row_start, row_start + 9),
            (132, 227, row_start, row_start + 9),
            (242, 339, row_start, row_start + 9),
        ]
