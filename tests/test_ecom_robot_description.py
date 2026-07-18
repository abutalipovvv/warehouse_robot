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
OPERATOR_STATIC_ROOT = PROJECT_ROOT / "operator_app" / "web" / "static"


def _operator_app_source() -> str:
    module_root = OPERATOR_STATIC_ROOT / "js" / "app"
    paths = [OPERATOR_STATIC_ROOT / "app.js", *sorted(module_root.glob("*.js"))]
    return "\n".join(path.read_text(encoding="utf-8") for path in paths)


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

    web_scene = (OPERATOR_STATIC_ROOT / "scene3d.js").read_text(encoding="utf-8")
    assert 'const B = globalThis.BABYLON || await loadBabylon();' in web_scene
    assert '"./vendor/babylon-9.16.2.js"' in web_scene
    assert "new B.WebGPUEngine" in web_scene
    assert 'return this.engine.webGLVersion >= 2 ? "WEBGL 2" : "WEBGL";' in web_scene
    assert "this.scene.useRightHandedSystem = true;" in web_scene
    assert "texture.update(true);" in web_scene
    assert "mesh.rotation.x = -Math.PI / 2;" in web_scene
    assert "beginCameraInteraction()" in web_scene
    assert "optimizeStaticScene()" in web_scene
    assert "const compactMode = robotList.length >= 40;" in web_scene
    assert "this.addEcomModel(group, robot, active)" in web_scene
    assert "ecomBody: 0xe8ecef" in web_scene
    assert "COLORS.ecomBody, 0.12, 0.48" in web_scene
    assert "addFloorGrid" not in web_scene
    assert "B.Texture.NEAREST_SAMPLINGMODE" in web_scene
    assert "createRibbonBatch(" in web_scene
    assert 'addCylinder("lidar-front"' in web_scene
    assert 'addCylinder("lidar-rear"' in web_scene
    assert 'addBox("front-panel"' in web_scene
    assert 'this.viewMode = "3d";' in web_scene
    assert 'setViewMode(mode)' in web_scene
    assert 'if (!active) {' in web_scene
    assert "group.metadata.underglowMaterial = underglow" in web_scene
    assert "group.metadata.underglowMesh = underglowMesh" in web_scene
    assert 'B.MeshBuilder.CreateDisc(`${group.name}-selection-halo`' in web_scene
    assert 'this.createRing(`${group.name}-selection-ring`, 0.61, 0.68, 56' in web_scene
    assert "group.metadata.selectionHaloMesh = selectionHaloMesh" in web_scene
    assert "updateSelectionAnimation(timestamp)" in web_scene
    assert "updateRobotMotion(timestamp)" in web_scene
    assert "const alpha = 1 - Math.exp(-32 * dt);" in web_scene
    assert "const robotAnimating = this.updateRobotMotion(timestamp);" in web_scene
    assert "const cap = Number(robotCount || 0) >= 40 ? 1.0 : 1.35;" in web_scene
    assert 'updateRobots(robots, selectedName = "", waitBlockerName = "")' in web_scene
    assert "updateRobotPoses(robots)" in web_scene
    assert "robotAlertLabel(robot)" in web_scene
    assert 'const needsLabel = showLabel || active || Boolean(alertText);' in web_scene
    assert "label.isVisible = needsLabel;" in web_scene
    assert "waitBlocker ? 0xff7a00" in web_scene
    assert 'this.viewMode === "2d" ? 0.13 : 0.105' in web_scene
    assert "futureRobotTrajectory(robot, active)" in web_scene
    assert 'addExtrudedPolygon("body", bodyOutline, 0.170, 0.0, body)' in web_scene
    assert 'addExtrudedPolygon("deck", deckOutline, 0.045, 0.160, deck)' in web_scene
    assert "y: 0.300, z: 0.060" in web_scene
    assert "THREE" not in web_scene
    assert "const bodyOutline = [" in web_scene
    assert "this.addTrp1Model(group, active)" not in web_scene

    web_app = _operator_app_source()
    assert "this.fleetSelectionCleared = true" in web_app
    assert "this.clearFleetRobotSelection();" in web_app
    assert '"fleet-route-preview active"' in web_app
    assert "this.fleetRobotColor(robot.name)" in web_app
    assert "const frameIntervalMs = 1000 / 60;" not in web_app
    assert "this.drawFleetAnimationFrame(now);" in web_app
    assert (
        "const simulatedMapTurn = this.isFleetManager() && "
        "!this.isFleetRobotsMode();"
    ) in web_app
    assert (
        "angular: (simulatedMapTurn ? right - left : left - right) * "
        "angularSpeed"
    ) in web_app
    assert "startFleetSimManualCommandLoop()" in web_app
    assert "now - this.fleetSimManualLastAt >= 15" in web_app
    assert "this.fleetSimManualFrame = window.requestAnimationFrame(publish);" in web_app
    assert "const visualPoseAtAck = this.animatedFleetManualPose(robot) || nextPose;" in web_app
    assert "generation !== this.fleetSimManualGeneration" in web_app
    assert "this.fleetStreamIntervalMs = 50;" in web_app
    assert "this.fleetStatusFreshTimeoutMs = 1500;" in web_app
    assert "this.fleetNavigationPredictionMaxSec = 0.4;" in web_app
    assert "drawFleetRobotMotionLayer(robotStyle)" in web_app
    assert "while (low + 1 < high)" in web_app
    assert "baseClock + visualLeadSec" in web_app
    assert "catchUpRate" not in web_app
    assert "const elapsed = Math.min(0.75" in web_app
    assert "this.setFleetManualAnimation(robot.name, pose, twist);" in web_app
    assert "drawFleetRobotLayer(robotStyle)" in web_app
    assert "this.fleetRobotSvgEntries = new Map();" in web_app
    assert "fleetRobotWaitBlockerName(robot)" in web_app
    assert "fleetRobotAlertLabel(robot)" in web_app
    assert "`robot-alert-label ${this.fleetRobotAlertSeverity(robot)}`" in web_app
    assert '"robot-wait-dependency-link"' in web_app
    assert '"robot-wait-blocker-halo"' in web_app
    assert "scene.updateRobots(robots, selectedName, waitBlockerName);" in web_app
    assert "scene.updateRobotPoses(robots)" in web_app

    web_styles = (OPERATOR_STATIC_ROOT / "styles.css").read_text(encoding="utf-8")
    assert ".robot-selection-halo" in web_styles
    assert ".robot-wait-blocker-halo" in web_styles
    assert ".robot-wait-dependency-link" in web_styles
    assert ".fleet-robot .robot-alert-label.warning" in web_styles
    assert ".fleet-robot .robot-alert-label.error" in web_styles
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
