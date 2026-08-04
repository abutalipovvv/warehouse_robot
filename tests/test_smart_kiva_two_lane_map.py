from __future__ import annotations

import math
from collections import Counter, defaultdict, deque
from pathlib import Path

import yaml

from fleet_manager.core.fleet.safety.collision import FleetCollisionChecker
from fleet_manager.core.mapping.maps.map_loader import WarehouseMapLoader


ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "fleet_manager" / "map_data" / "maps_out" / "smart_kiva_large_w_mode.smap"
PARAMS_PATH = ROOT / "fleet_manager" / "config" / "params.yaml"
MAP_SCALE = 4.0 / 3.0
LOWER_LANE_Y = tuple(
    round((3.8 + (4.0 * index)) * MAP_SCALE, 6)
    for index in range(7)
)
UPPER_LANE_Y = tuple(
    round((5.2 + (4.0 * index)) * MAP_SCALE, 6)
    for index in range(7)
)
REMOVED_MIDDLE_Y = tuple(
    round((4.5 + (4.0 * index)) * MAP_SCALE, 6)
    for index in range(7)
)
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
        assert math.isclose(upper_y - lower_y, 1.4 * MAP_SCALE, abs_tol=1e-6)

        upper_shelf_edge = 3.0 + (4.0 * aisle_index)
        lower_shelf_edge = 6.0 + (4.0 * aisle_index)
        assert math.isclose(
            lower_y - (upper_shelf_edge * MAP_SCALE),
            0.8 * MAP_SCALE,
            abs_tol=1e-6,
        )
        assert math.isclose(
            (lower_shelf_edge * MAP_SCALE) - upper_y,
            0.8 * MAP_SCALE,
            abs_tol=1e-6,
        )


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


def test_smart_kiva_reverse_edges_keep_one_stable_body_heading() -> None:
    loaded = WarehouseMapLoader(MAP_DIR).load()
    edges = {
        (edge.from_name, edge.to_name): edge
        for edge in loaded.edges
    }

    for start, goal in edges:
        reverse = edges[(goal, start)]
        assert edges[(start, goal)].motion_direction_code() in {0, 1}
        assert {
            edges[(start, goal)].motion_direction_code(),
            reverse.motion_direction_code(),
        } == {0, 1}
        start_lm = loaded.landmarks[start]
        goal_lm = loaded.landmarks[goal]
        body_heading = math.atan2(
            goal_lm.y - start_lm.y,
            goal_lm.x - start_lm.x,
        )
        if edges[(start, goal)].motion_direction_code() == 1:
            body_heading += math.pi
        reverse_heading = math.atan2(
            start_lm.y - goal_lm.y,
            start_lm.x - goal_lm.x,
        )
        if reverse.motion_direction_code() == 1:
            reverse_heading += math.pi
        heading_delta = math.atan2(
            math.sin(body_heading - reverse_heading),
            math.cos(body_heading - reverse_heading),
        )
        assert math.isclose(heading_delta, 0.0, abs_tol=1e-9)

    # The generator's canonical increasing grid direction is forward. The
    # reverse edge is driven backward, so both traversals keep the same body
    # orientation instead of adding a 180 degree turn at every reversal.
    assert edges[("S002002", "S002003")].motion_direction_code() == 0
    assert edges[("S002003", "S002002")].motion_direction_code() == 1
    assert edges[("S002002", "S003002")].motion_direction_code() == 0
    assert edges[("S003002", "S002002")].motion_direction_code() == 1


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
    } == {round(1.0 * MAP_SCALE, 6)}
    assert {
        landmark.x
        for landmark in landmarks.values()
        if landmark.name.endswith("035")
    } == {round(35.0 * MAP_SCALE, 6)}
    assert {
        landmark.y
        for landmark in landmarks.values()
        if landmark.name.startswith("S002")
    } == {round(1.0 * MAP_SCALE, 6)}
    assert {
        landmark.y
        for landmark in landmarks.values()
        if landmark.name.startswith("S032")
    } == {round(32.0 * MAP_SCALE, 6)}


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
        for yaw_step in range(72):
            yaw = yaw_step * math.pi / 36.0
            assert not collision.blocked_reason(
                {"x": landmark.x, "y": landmark.y, "yaw": yaw},
                [],
                [],
            ), f"{name} cannot safely turn at yaw={yaw}"
    assert checked > 0


def test_smart_kiva_adjacent_lms_clear_two_turning_robot_footprints() -> None:
    loaded = WarehouseMapLoader(MAP_DIR).load()
    params = yaml.safe_load(PARAMS_PATH.read_text(encoding="utf-8"))
    collision = FleetCollisionChecker(params, MAP_DIR, loaded.map_metadata)
    minimum_edge = min(edge.length for edge in loaded.edges)

    # This is a deliberately stronger invariant than checking a few known
    # corners: centres on any two adjacent graph LMs remain farther apart than
    # the complete safe turning diameter, regardless of both robot headings.
    assert minimum_edge > collision.robot_broadphase_distance()


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
        {1664: 5, 1703: 10, 1792: 3, 1834: 6}
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
    expected_row_bounds = (
        (27, 39),
        (80, 93),
        (134, 146),
        (187, 199),
        (240, 253),
        (294, 306),
        (347, 359),
        (400, 413),
    )
    for row_start, row_end in expected_row_bounds:
        row_bounds = [bound for bound in bounds if bound[2] == row_start]
        assert row_bounds == [
            (27, 157, row_start, row_end),
            (176, 303, row_start, row_end),
            (323, 453, row_start, row_end),
        ]
