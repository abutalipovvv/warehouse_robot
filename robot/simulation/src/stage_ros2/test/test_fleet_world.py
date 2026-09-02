from dataclasses import replace
from pathlib import Path

import pytest

from stage_ros2.fake_bms_publisher import fake_battery_state
from stage_ros2.fleet_world import (
    apply_domain_override,
    load_fleet_config,
    load_fleet_definition,
    render_fleet_world,
    robot_domain_map,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / 'config' / 'fleet.yaml'


def test_fake_battery_payload_is_complete():
    message = fake_battery_state()

    assert message.present is True
    assert message.percentage == pytest.approx(0.75)
    assert message.temperature == pytest.approx(30.0)
    assert len(message.cell_voltage) == 8


def test_default_fleet_loads_four_independent_robots():
    fleet = load_fleet_definition(DEFAULT_CONFIG)
    robots = fleet.robots

    assert fleet.world == '22.05.26_smap'
    assert fleet.smap.smap_path.name == '22.05.26_smap.smap'
    assert fleet.smap.map_yaml_path.name == '22.05.26_smap.yaml'
    assert (fleet.smap.width_pixels, fleet.smap.height_pixels) == (1692, 650)
    assert [robot.robot_id for robot in robots] == [
        'robot11',
        'robot12',
        'robot13',
        'robot14',
    ]
    assert [robot.ip for robot in robots] == [
        '127.0.0.11',
        '127.0.0.12',
        '127.0.0.13',
        '127.0.0.14',
    ]
    assert [robot.lm for robot in robots] == [
        'LM91',
        'LM101',
        'LM111',
        'LM121',
    ]
    assert (robots[0].x, robots[0].y) == pytest.approx((-4.902, 1.362))
    assert robot_domain_map(robots) == (
        'robot11=11,robot12=12,robot13=13,robot14=14'
    )


def test_domain_override_preserves_robot_configuration_except_domain():
    robots = load_fleet_config(DEFAULT_CONFIG)

    overridden = apply_domain_override(
        robots, 'robot11=71,robot12=72,robot13=73,robot14=74'
    )

    assert robot_domain_map(overridden) == (
        'robot11=71,robot12=72,robot13=73,robot14=74'
    )
    assert [replace(robot, ros_domain_id=0) for robot in overridden] == [
        replace(robot, ros_domain_id=0) for robot in robots
    ]


def test_rendered_world_uses_exact_smap_geometry_and_robot_blocks(tmp_path):
    fleet = load_fleet_definition(DEFAULT_CONFIG)
    fleet = replace(fleet, robots=fleet.robots[:2])
    include = tmp_path / 'trp1.inc'
    template = (PACKAGE_ROOT / 'world' / 'fleet_generated.world.in').read_text(
        encoding='utf-8'
    )

    world = render_fleet_world(template, fleet, trp1_include=include)

    assert f'include "{include}"' in world
    assert f'bitmap "{fleet.smap.bitmap_path}"' in world
    assert 'name "22.05.26_smap"' in world
    assert 'size [ 33.840000 13.000000 2.000000 ]' in world
    assert 'pose [ -6.016000 1.811000 0.000000 0.000000 ]' in world
    assert 'boundary 0' in world
    assert 'gui_outline 1' in world
    assert 'size [ 1400.000000 600.000000 ]' in world
    assert 'show_data 0' in world
    assert 'name "robot11"' in world
    assert 'name "robot12"' in world
    assert 'name "robot13"' not in world
    assert world.count('trp1_with_laser') == 2


def _fleet_yaml(smap: Path, robots: str) -> str:
    return f'world: test\nsmap: {smap}\nrobots:\n{robots}'


def test_config_rejects_duplicate_domains(tmp_path):
    smap = load_fleet_definition(DEFAULT_CONFIG).smap.smap_path
    config = tmp_path / 'invalid_fleet.yaml'
    config.write_text(
        _fleet_yaml(
            smap,
            """  - robot_id: robot11
    ros_domain_id: 11
    ip: 127.0.0.11
    lm: LM91
  - robot_id: robot12
    ros_domain_id: 11
    ip: 127.0.0.12
    lm: LM101
""",
        ),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='duplicate ros_domain_id'):
        load_fleet_config(config)


def test_config_rejects_unknown_landmark_and_duplicate_ip(tmp_path):
    smap = load_fleet_definition(DEFAULT_CONFIG).smap.smap_path
    config = tmp_path / 'invalid_fleet.yaml'
    config.write_text(
        _fleet_yaml(
            smap,
            """  - robot_id: robot11
    ros_domain_id: 11
    ip: 127.0.0.11
    lm: DOES_NOT_EXIST
""",
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='does not exist in SMAP'):
        load_fleet_definition(config)

    config.write_text(
        _fleet_yaml(
            smap,
            """  - robot_id: robot11
    ros_domain_id: 11
    ip: 127.0.0.11
    lm: LM91
  - robot_id: robot12
    ros_domain_id: 12
    ip: 127.0.0.11
    lm: LM101
""",
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='duplicate robot ip'):
        load_fleet_definition(config)


def test_override_requires_exact_robot_set():
    robots = load_fleet_config(DEFAULT_CONFIG)

    with pytest.raises(ValueError, match='does not match fleet config'):
        apply_domain_override(robots, 'robot11=71,robot12=72')
