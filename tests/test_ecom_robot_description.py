from __future__ import annotations

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION_ROOT = (
    PROJECT_ROOT / "sim_robot" / "ws" / "src" / "ecom_mobile_robot_description"
)
DESCRIPTION = DESCRIPTION_ROOT / "urdf" / "ecom_stage.urdf.xacro"
STAGE_ROOT = PROJECT_ROOT / "sim_robot" / "ws" / "src" / "stage_ros2"


def test_ecom_description_preserves_the_trp1_tf_contract() -> None:
    root = ET.parse(DESCRIPTION).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    assert links == {
        "base_link",
        "imu_link",
        "camera_mount_link",
        "camera_optical_depth_link",
        "left_wheel",
        "right_wheel",
    }
    assert set(joints) == {
        "imu_joint",
        "camera_joint",
        "camera_mount_joint",
        "left_wheel_hinge",
        "right_wheel_hinge",
    }
    assert _joint_frames(joints["imu_joint"]) == ("base_link", "imu_link")
    assert _joint_frames(joints["camera_joint"]) == ("base_link", "camera_mount_link")
    assert _joint_frames(joints["camera_mount_joint"]) == (
        "camera_mount_link",
        "camera_optical_depth_link",
    )
    assert _joint_frames(joints["left_wheel_hinge"]) == ("base_link", "left_wheel")
    assert _joint_frames(joints["right_wheel_hinge"]) == ("base_link", "right_wheel")

    meshes = [mesh.attrib["filename"] for mesh in root.findall(".//mesh")]
    assert len(meshes) == 9
    assert all(
        filename.startswith("package://ecom_mobile_robot_description/meshes/")
        for filename in meshes
    )
    assert all((DESCRIPTION_ROOT / "meshes" / Path(filename).name).is_file() for filename in meshes)


def test_ecom_dimensions_are_used_by_urdf_stage_nav2_and_planner() -> None:
    root = ET.parse(DESCRIPTION).getroot()
    collision_box = root.find("./link[@name='base_link']/collision/geometry/box")
    assert collision_box is not None
    assert [float(value) for value in collision_box.attrib["size"].split()] == [
        1.0,
        0.7,
        0.1923,
    ]

    stage_model = (STAGE_ROOT / "world" / "include" / "trp1.inc").read_text(
        encoding="utf-8"
    )
    assert "size [1.00 0.70 0.17]" in stage_model
    assert "origin [0.0 0.0 0.0 0.0]" in stage_model
    assert "points 18" in stage_model
    assert "point[0]  [-0.5230 -0.1840]" in stage_model
    assert "point[13] [-0.0479  0.3468]" in stage_model
    assert 'color "LightSteelBlue"' in stage_model
    assert "pose [0 0 -0.01 0]" in stage_model
    assert "z [0 0.045]" in stage_model
    assert "hokuyolaser(pose [ 0.32487 0.24906 0.048 0 ])" in stage_model

    params = yaml.safe_load(
        (PROJECT_ROOT / "sim_robot" / "ws" / "src" / "params.yaml").read_text(
            encoding="utf-8"
        )
    )
    # The generated operator summary intentionally rounds display geometry to
    # millimetres, while Nav2 keeps the measured footprint below.
    expected_operator_footprint = [
        {"x": -0.523, "y": -0.353},
        {"x": 0.477, "y": -0.353},
        {"x": 0.477, "y": 0.347},
        {"x": -0.523, "y": 0.347},
    ]
    expected_nav_footprint = [
        {"x": -0.523, "y": -0.3532},
        {"x": 0.477, "y": -0.3532},
        {"x": 0.477, "y": 0.3468},
        {"x": -0.523, "y": 0.3468},
    ]
    assert params["robot_model"]["footprint"] == expected_operator_footprint
    assert params["nav2"]["local_costmap"]["footprint"] == expected_nav_footprint
    assert params["nav2"]["global_costmap"]["footprint"] == expected_nav_footprint

    nav2 = yaml.safe_load(
        (
            PROJECT_ROOT
            / "sim_robot"
            / "ws"
            / "src"
            / "nav2"
            / "config"
            / "nav2_params.yaml"
        ).read_text(encoding="utf-8")
    )
    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = nav2[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["robot_radius"] == 0.632
        assert ast.literal_eval(costmap["footprint"]) == [
            [-0.523, -0.3532],
            [0.477, -0.3532],
            [0.477, 0.3468],
            [-0.523, 0.3468],
        ]


def test_stage_launch_uses_ecom_description_and_keeps_stage_hardware_contract() -> None:
    launch = (STAGE_ROOT / "launch" / "stage.launch.py").read_text(encoding="utf-8")
    assert "FindPackageShare('ecom_mobile_robot_description')" in launch
    assert "'ecom_stage.urdf.xacro'" in launch
    assert "FindPackageShare('trp1_description')" not in launch

    vehicle = (STAGE_ROOT / "src" / "vehicle.cpp").read_text(encoding="utf-8")
    assert '#define TOPIC_CMD_VEL "cmd_vel"' in vehicle
    assert '#define TOPIC_ODOM "odom"' in vehicle
    assert "frame_id_base_link_" in vehicle
    assert "relative_imu_yaw" in vehicle
    assert "delta_world_x" in vehicle
    assert "body_motion.a * body_motion.x" in vehicle
    assert "9.80665" in vehicle
    assert "msg_odom_.twist.twist.linear.x = body_motion.x" in vehicle

    nav2 = yaml.safe_load(
        (
            PROJECT_ROOT
            / "sim_robot"
            / "ws"
            / "src"
            / "nav2"
            / "config"
            / "nav2_params.yaml"
        ).read_text(encoding="utf-8")
    )
    amcl = nav2["amcl"]["ros__parameters"]
    assert amcl["max_beams"] == 120
    assert amcl["min_particles"] == 500
    assert amcl["max_particles"] == 2000
    assert amcl["sigma_hit"] == 0.15
    assert amcl["transform_tolerance"] == 0.5
    assert amcl["z_hit"] + amcl["z_rand"] == 1.0
    assert (
        nav2["controller_server"]["ros__parameters"]["FollowPath"]["CostCritic"]
        ["consider_footprint"]
        is True
    )

    stage_main = (STAGE_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
    assert "node->world->IsGUI()" in stage_main
    assert "std::this_thread::sleep_until(next_update)" in stage_main

    web_scene = (PROJECT_ROOT / "operator_app" / "static" / "scene3d.js").read_text(
        encoding="utf-8"
    )
    assert "this.addEcomModel(group, robot, active)" in web_scene
    assert "ecomBody: 0xffffff" in web_scene
    assert "color: COLORS.ecomBody" in web_scene
    assert "group.userData.underglowMaterial = underglow" in web_scene
    assert "group.userData.underglowMesh = underglowMesh" in web_scene
    assert "new THREE.CircleGeometry(0.64, 56)" in web_scene
    assert "new THREE.RingGeometry(0.61, 0.68, 56)" in web_scene
    assert "group.userData.selectionHaloMesh = selectionHaloMesh" in web_scene
    assert "updateSelectionAnimation(timestamp)" in web_scene
    assert "this.routeRibbonGeometry(points, 0.105)" in web_scene
    assert "addExtrudedPolygon(bodyOutline, 0.170, 0.0, body)" in web_scene
    assert "addExtrudedPolygon(deckOutline, 0.045, 0.160, deck)" in web_scene
    assert "y: 0.300, z: 0.060" in web_scene
    assert "new THREE.TorusGeometry" not in web_scene
    assert "const bodyOutline = [" in web_scene
    assert "this.addTrp1Model(group, active)" not in web_scene

    web_app = (PROJECT_ROOT / "operator_app" / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert "this.fleetSelectionCleared = true" in web_app
    assert "this.clearFleetRobotSelection();" in web_app
    assert '"fleet-route-preview active"' in web_app
    assert "this.fleetRobotColor(robot.name)" in web_app

    web_styles = (
        PROJECT_ROOT / "operator_app" / "static" / "styles.css"
    ).read_text(encoding="utf-8")
    assert ".robot-selection-halo" in web_styles
    assert "@keyframes fleet-robot-selection-pulse" in web_styles

    legacy_launch = (
        PROJECT_ROOT
        / "sim_robot"
        / "ws"
        / "src"
        / "trp1_description"
        / "launch"
        / "launch.py"
    ).read_text(encoding="utf-8")
    assert "FindPackageShare('ecom_mobile_robot_description')" in legacy_launch
    assert "'ecom_stage.urdf.xacro'" in legacy_launch


def _joint_frames(joint: ET.Element) -> tuple[str, str]:
    parent = joint.find("parent")
    child = joint.find("child")
    assert parent is not None and child is not None
    return parent.attrib["link"], child.attrib["link"]
