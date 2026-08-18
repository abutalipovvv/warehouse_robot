from __future__ import annotations

import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from typing import Any

import pytest
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROBOT_API_ROOT = (
    PROJECT_ROOT / "sim_robot" / "ws" / "src" / "robot_grpc_api"
)
ROBOT_PLANNER_ROOT = (
    PROJECT_ROOT / "sim_robot" / "ws" / "src" / "robot_planner"
)
for source_root in (ROBOT_API_ROOT, ROBOT_PLANNER_ROOT):
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))

from robot_grpc_api import ros_runtime_control  # noqa: E402
from robot_grpc_api import ros_runtime_lifecycle  # noqa: E402
from robot_grpc_api import ros_runtime_nav2_lifecycle  # noqa: E402
from robot_grpc_api import ros_runtime_params  # noqa: E402
from robot_grpc_api import ros_runtime_slam  # noqa: E402
from robot_grpc_api.ros_runtime import RosRobotRuntime  # noqa: E402
from robot_grpc_api.ros_runtime_control import (  # noqa: E402
    RosRuntimeControlMixin,
)
from robot_grpc_api.ros_runtime_lifecycle import (  # noqa: E402
    RosRuntimeLifecycleMixin,
)
from robot_grpc_api.ros_runtime_maps import (  # noqa: E402
    RosRuntimeMapTransferMixin,
)
from robot_grpc_api.ros_runtime_nav2_lifecycle import (  # noqa: E402
    NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES,
    NAV2_LIFECYCLE_MANAGER_SERVICES,
    RosRuntimeNav2LifecycleMixin,
)
from robot_grpc_api.ros_runtime_params import (  # noqa: E402
    RosRuntimeParametersMixin,
)
from robot_grpc_api.ros_runtime_ros_helpers import (  # noqa: E402
    RosRuntimeMessageServiceMixin,
)
from robot_grpc_api.ros_runtime_slam import (  # noqa: E402
    RosRuntimeSlamMixin,
)
from robot_planner.route_core.atomic_storage import (  # noqa: E402
    atomic_write_bytes as real_atomic_write_bytes,
)
from robot_planner.route_core.atomic_storage import (  # noqa: E402
    atomic_write_text as real_atomic_write_text,
)
from robot_planner.route_core.pgm import PgmImage  # noqa: E402


EXPECTED_COMPONENT_METHODS = {
    RosRuntimeLifecycleMixin: {
        "__init__",
        "start",
        "available",
        "error",
        "_topic",
        "_default_slam_params_file",
        "_default_slam_launch_file",
        "close",
        "_start",
        "wait_for_status",
        "_spin_executor",
        "_on_status",
        "_on_odom",
        "_on_scan",
        "_on_map",
        "_odom_pose_payload",
        "_slam_tf_pose_payload",
        "_append_slam_trail_pose_locked",
        "_latest_message",
        "_status_age_sec",
    },
    RosRuntimeControlMixin: {
        "identity_payload",
        "sidebar_payload",
        "status_payload",
        "status_robot_payload",
        "teleop",
        "teleop_stop",
        "laser_scan_payload",
        "stop",
        "cancel_route",
        "execute_route",
        "acquire_control",
        "release_control",
        "relocate",
        "_map_pose_to_ros_pose",
        "confirm_localization",
        "pause_route",
        "resume_route",
        "_ensure_navigation_mode",
        "_ensure_manual_control_allowed",
        "_execute_pose_route",
        "_straight_pose_route_payload",
        "_route_payload_from_request",
        "_goal_pose_payload",
        "_current_pose_payload",
        "_current_slam_pose_payload",
        "_yaw_from_quaternion",
        "_stamp_sec",
        "_normalize_angle",
        "_message_state",
        "_set_route_paused",
        "_control_payload",
        "_control_state_payload",
        "_ensure_control_owner",
        "_expire_control_owner_locked",
        "_clean_owner_id",
        "_covariance_from_json",
        "_route_payload",
        "_persist_event",
        "_append_runtime_event",
    },
    RosRuntimeMapTransferMixin: {
        "active_map_payload",
        "list_maps_payload",
        "pull_map_bundle_payload",
        "push_map_bundle_payload",
        "load_map",
        "_stop_navigation_before_map_change",
    },
    RosRuntimeSlamMixin: {
        "slam_defaults_payload",
        "start_slam",
        "slam_state_payload",
        "slam_map_frame_payload",
        "finish_slam",
        "cancel_slam",
        "_set_slam_progress",
        "_slam_cells_to_bytes",
        "_safe_map_name",
        "_coerce_slam_params",
        "_normalize_slam_params",
        "_slam_maps_root",
        "_save_slam_map_files",
        "_write_current_map_files",
        "_write_empty_smap_sidecars",
        "_pgm_size",
        "_stop_slam_process",
    },
    RosRuntimeNav2LifecycleMixin: {
        "_reset_odom_for_slam",
        "_wait_for_zero_odom_after_reset",
        "_pause_nav2_for_slam",
        "_resume_nav2_after_slam",
        "_stop_robot_motion_for_slam",
        "_call_nav2_lifecycle_manager",
        "_call_nav2_node_transition",
    },
    RosRuntimeParametersMixin: {
        "params_payload",
        "save_params_payload",
        "_apply_saved_nav2_params_when_ready",
        "_apply_nav2_runtime_params",
        "_filter_declared_parameters",
        "_parameter_message",
        "_deep_get",
        "_values_equal",
        "_reload_route_status_params",
    },
    RosRuntimeMessageServiceMixin: {
        "_publish_twist",
        "_publish_go_to_lm",
        "_service_available",
        "_call_service",
        "_message_to_robot_payload",
    },
}

EXPECTED_STATE_KEYS = [
    "robot_id",
    "robot_name",
    "host",
    "domain_id",
    "namespace",
    "status_topic",
    "cmd_vel_topic",
    "odom_topic",
    "initial_pose_topic",
    "scan_topic",
    "map_frame",
    "base_frame",
    "go_to_lm_topic",
    "plan_service_name",
    "execute_service_name",
    "cancel_service_name",
    "route_pause_service_name",
    "route_load_map_service_name",
    "status_load_map_service_name",
    "map_state_service_name",
    "map_load_service_name",
    "map_list_service_name",
    "map_get_bundle_service_name",
    "map_put_bundle_service_name",
    "map_topic",
    "slam_save_map_service_name",
    "reset_odom_service_name",
    "slam_params_file",
    "slam_launch_file",
    "params_path",
    "_lock",
    "_latest_status",
    "_latest_status_at",
    "_latest_scan",
    "_latest_scan_at",
    "_latest_odom_pose",
    "_latest_odom_at",
    "_latest_slam_pose",
    "_latest_slam_pose_at",
    "_latest_map",
    "_latest_map_at",
    "_control_owner_id",
    "_control_owner_name",
    "_control_acquired_at",
    "_control_lease_ms",
    "_navigation_paused",
    "_localization_confirmed",
    "_relocation_requested_at",
    "_events",
    "_available",
    "_error",
    "_node",
    "_rclpy",
    "_context",
    "_time_type",
    "_tf_buffer",
    "_tf_listener",
    "_twist_type",
    "_odom_type",
    "_pose_with_covariance_type",
    "_laser_scan_type",
    "_string_type",
    "_set_parameters_type",
    "_list_parameters_type",
    "_manage_lifecycle_nodes_type",
    "_change_state_type",
    "_transition_type",
    "_parameter_type",
    "_parameter_value_type",
    "_parameter_type_enum",
    "_executor",
    "_thread",
    "_cmd_vel_pub",
    "_initial_pose_pub",
    "_go_to_lm_pub",
    "_plan_route_client",
    "_execute_route_client",
    "_cancel_route_client",
    "_route_pause_client",
    "_route_load_map_client",
    "_status_load_map_client",
    "_map_state_client",
    "_map_load_client",
    "_map_list_client",
    "_map_get_bundle_client",
    "_map_put_bundle_client",
    "_slam_save_map_client",
    "_reset_odom_client",
    "_nav2_lifecycle_clients",
    "_nav2_change_state_clients",
    "_save_map_type",
    "_std_empty_type",
    "_occupancy_grid_type",
    "_slam_process",
    "_slam_temp_dir",
    "_nav2_paused_for_slam",
    "_slam_ignore_maps_until",
    "_slam_state",
    "_slam_trail",
    "_nav2_param_clients",
    "_nav2_list_param_clients",
    "_startup_attempted",
]


@pytest.fixture
def runtime(tmp_path: Path) -> RosRobotRuntime:
    instance = RosRobotRuntime(
        robot_id="standalone-01",
        robot_name="Standalone 01",
        host="192.0.2.10",
        domain_id=21,
        namespace="warehouse/s",
        map_frame="map_local",
        base_frame="base_footprint",
        slam_params_file=str(tmp_path / "slam-params.yaml"),
        slam_launch_file=str(tmp_path / "slam-launch.py"),
        params_path=str(tmp_path / "runtime-params.yaml"),
        autostart=False,
    )
    yield instance
    instance.close()


def _defined_methods(component: type[Any]) -> set[str]:
    return {
        name
        for name, value in vars(component).items()
        if inspect.isfunction(value)
        or isinstance(value, (staticmethod, classmethod, property))
    }


def _status_message(**overrides: Any) -> SimpleNamespace:
    values = {
        "robot_id": "standalone-r1",
        "map_id": "map-s",
        "connected": True,
        "localization_ok": True,
        "localization_age_sec": 0.1,
        "state": "IDLE",
        "message": "ready",
        "target_lm": "",
        "nearest_lm": "LM-1",
        "current_edge_id": "",
        "route_id": "",
        "route_progress": 0.0,
        "pose_x": 2.0,
        "pose_y": -3.0,
        "pose_yaw": 0.5,
        "linear_velocity": 0.0,
        "angular_velocity": 0.0,
        "battery_level": 91.0,
        "battery_voltage": 25.0,
        "battery_current": 0.5,
        "battery_temperature": 30.0,
        "battery_charging": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_explicit_mapf_route_is_forwarded_without_replanning(runtime: RosRobotRuntime) -> None:
    payload = {
        "protocol": "lm_route",
        "protocolVersion": 2,
        "routeId": "mapf-route-17",
        "revision": 4,
        "startLm": "LM280",
        "goalLm": "LM131",
        "nodes": ["LM280", "LM131"],
        "replaceMode": "immediate",
        "ownerId": "fleet-coordinator",
        "commandId": "dispatch-17",
    }

    route = runtime._route_payload_from_request(payload)

    assert route == {
        "protocol": "lm_route",
        "protocolVersion": 2,
        "routeId": "mapf-route-17",
        "revision": 4,
        "startLm": "LM280",
        "goalLm": "LM131",
        "nodes": ["LM280", "LM131"],
        "replaceMode": "immediate",
    }


def test_facade_composes_seven_disjoint_local_capabilities() -> None:
    assert RosRobotRuntime.__bases__ == tuple(EXPECTED_COMPONENT_METHODS)
    owned: set[str] = set()
    for component, expected in EXPECTED_COMPONENT_METHODS.items():
        actual = _defined_methods(component)
        assert actual == expected
        assert owned.isdisjoint(actual)
        owned.update(actual)
    assert len(owned) == 103
    assert _defined_methods(RosRobotRuntime) == set()
    assert not any(
        "fleet_manager" in value.__module__
        for value in EXPECTED_COMPONENT_METHODS
    )


def test_initialization_order_and_explicit_start_are_preserved(
    runtime: RosRobotRuntime,
) -> None:
    assert list(vars(runtime)) == EXPECTED_STATE_KEYS
    assert runtime._startup_attempted is False
    starts: list[str] = []
    runtime._start = lambda: starts.append("start")

    runtime.start()
    runtime.start()

    assert starts == ["start"]
    assert runtime._startup_attempted is True
    assert runtime.status_topic == "/warehouse/s/robot_status"
    assert runtime.odom_topic == "/warehouse/s/odom"
    assert runtime.map_frame == "map_local"
    assert runtime.base_frame == "base_footprint"
    assert runtime._lock.acquire(blocking=False)
    runtime._lock.release()


def test_close_keeps_process_and_ros_shutdown_order(
    runtime: RosRobotRuntime,
) -> None:
    calls: list[tuple[str, Any]] = []
    runtime._stop_slam_process = lambda: calls.append(("slam", None))
    runtime._executor = SimpleNamespace(
        shutdown=lambda: calls.append(("executor", None))
    )
    runtime._thread = SimpleNamespace(
        is_alive=lambda: True,
        join=lambda timeout: calls.append(("thread", timeout)),
    )
    runtime._node = SimpleNamespace(
        destroy_node=lambda: calls.append(("node", None))
    )
    runtime._context = SimpleNamespace(
        try_shutdown=lambda: calls.append(("context", None))
    )

    runtime.close()

    assert calls == [
        ("slam", None),
        ("executor", None),
        ("thread", 1.0),
        ("node", None),
        ("context", None),
    ]
    runtime._executor = None
    runtime._thread = None
    runtime._node = None
    runtime._context = None


def test_status_staleness_and_inferred_navigation_state(
    runtime: RosRobotRuntime,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = [100.0]
    monkeypatch.setattr(
        ros_runtime_lifecycle,
        "monotonic",
        lambda: now[0],
    )
    monkeypatch.setattr(
        ros_runtime_control,
        "monotonic",
        lambda: now[0],
    )
    message = _status_message(target_lm="LM-9")
    runtime._latest_status = message
    runtime._latest_status_at = 99.5

    robot = runtime.status_robot_payload()
    assert robot["connected"] is True
    assert robot["state"] == "EXECUTING_ROUTE"
    assert robot["statusAgeSec"] == 0.5

    now[0] = 103.1
    stale = runtime.status_robot_payload()
    assert stale["connected"] is False
    assert stale["state"] == "DISCONNECTED"
    assert stale["statusAgeSec"] == pytest.approx(3.6)
    assert "stale" in stale["message"]


def test_odom_callback_tracks_slam_pose_and_trail(
    runtime: RosRobotRuntime,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        ros_runtime_lifecycle,
        "monotonic",
        lambda: 50.0,
    )
    runtime._slam_state = {
        "active": True,
        "state": "mapping",
    }
    odom = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=250_000_000)
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.5, y=-0.25),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0,
                ),
            )
        ),
    )

    runtime._on_odom(odom)

    assert runtime._latest_odom_pose == {
        "x": 1.5,
        "y": -0.25,
        "yaw": 0.0,
        "stampSec": 12.25,
    }
    assert runtime._latest_slam_pose == runtime._latest_odom_pose
    assert runtime._latest_odom_at == 50.0
    assert runtime._latest_slam_pose_at == 50.0
    assert runtime._slam_trail == [runtime._latest_odom_pose]


def test_reset_odom_waits_for_fresh_zero_pose(
    runtime: RosRobotRuntime,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        ros_runtime_nav2_lifecycle,
        "monotonic",
        lambda: 200.0,
    )

    class Empty:
        class Request:
            pass

    client = SimpleNamespace(srv_type=Empty)
    runtime._reset_odom_client = client
    runtime._slam_trail = [{"x": 9.0, "y": 9.0}]
    calls: list[tuple[str, float]] = []

    def call_service(
        used_client: Any,
        request: Any,
        label: str,
        *,
        timeout_sec: float,
    ) -> SimpleNamespace:
        assert used_client is client
        assert isinstance(request, Empty.Request)
        calls.append((label, timeout_sec))
        runtime._latest_odom_pose = {
            "x": 0.01,
            "y": -0.02,
            "yaw": 0.03,
        }
        runtime._latest_odom_at = 200.0
        return SimpleNamespace()

    runtime._call_service = call_service
    runtime._reset_odom_for_slam()

    assert calls == [("reset odom", 3.0)]
    assert runtime._slam_trail == []


def test_nav2_pause_and_resume_keep_manager_order(
    runtime: RosRobotRuntime,
) -> None:
    class ManageLifecycle:
        class Request:
            PAUSE = 10
            RESUME = 20

            def __init__(self) -> None:
                self.command = 0

    runtime._manage_lifecycle_nodes_type = ManageLifecycle
    runtime._stop_robot_motion_for_slam = lambda details: None
    runtime._service_available = lambda client, timeout_sec=0.05: True
    client = SimpleNamespace()
    runtime._nav2_lifecycle_clients = {
        runtime._topic(service_name): client
        for service_name, _ in NAV2_LIFECYCLE_MANAGER_SERVICES
    }
    calls: list[tuple[str, int]] = []

    def call_service(
        used_client: Any,
        request: Any,
        label: str,
        *,
        timeout_sec: float,
    ) -> SimpleNamespace:
        assert used_client is client
        assert timeout_sec == 5.0
        calls.append((label, request.command))
        return SimpleNamespace(success=True)

    runtime._call_service = call_service
    paused = runtime._pause_nav2_for_slam()
    assert paused["changed"] is True
    assert paused["managers"] == [
        label for _, label in NAV2_LIFECYCLE_MANAGER_SERVICES
    ]

    runtime._nav2_paused_for_slam = True
    resumed = runtime._resume_nav2_after_slam()
    assert resumed["changed"] is True
    assert resumed["managers"] == [
        label for _, label in NAV2_LIFECYCLE_MANAGER_RESUME_SERVICES
    ]
    assert [command for _, command in calls] == [
        10,
        10,
        10,
        20,
        20,
        20,
    ]


def test_navigation_mode_guards_map_changes(runtime: RosRobotRuntime) -> None:
    runtime._slam_state = {
        "active": True,
        "state": "mapping",
    }
    with pytest.raises(
        ValueError,
        match="Cannot load map while robot is in mapping mode",
    ):
        runtime.load_map("alpha")

    runtime._ensure_manual_control_allowed("teleop")
    runtime._slam_state["state"] = "saving"
    with pytest.raises(ValueError, match="switching modes"):
        runtime._ensure_manual_control_allowed("teleop")


def test_slam_writes_are_atomic_and_use_local_shared_pgm(
    runtime: RosRobotRuntime,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    writes: list[tuple[str, str]] = []

    def write_bytes(path: Path, content: bytes) -> None:
        writes.append(("bytes", Path(path).name))
        real_atomic_write_bytes(Path(path), content)

    def write_text(path: Path, content: str) -> None:
        writes.append(("text", Path(path).name))
        real_atomic_write_text(Path(path), content)

    monkeypatch.setattr(ros_runtime_slam, "atomic_write_bytes", write_bytes)
    monkeypatch.setattr(ros_runtime_slam, "atomic_write_text", write_text)
    runtime._latest_map = SimpleNamespace(
        info=SimpleNamespace(
            width=2,
            height=2,
            resolution=0.1,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=2.0),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0,
                ),
            ),
        ),
        data=[0, 100, -1, 50],
    )
    target = tmp_path / "standalone.smap"
    target.mkdir()

    runtime._write_current_map_files(target, "standalone")
    runtime._write_empty_smap_sidecars(target, "standalone")

    image = PgmImage.read(target / "standalone.pgm")
    assert (image.width, image.height) == (2, 2)
    assert image.pixels == bytes([205, 205, 254, 0])
    assert runtime._pgm_size(target / "standalone.pgm") == (2, 2)
    summary = json.loads(
        (target / "smap_summary.json").read_text(encoding="utf-8")
    )
    assert summary["counts"]["grid"] == {"width": 2, "height": 2}
    assert writes == [
        ("bytes", "standalone.pgm"),
        ("text", "standalone.yaml"),
        ("text", "LMs.yaml"),
        ("text", "graphs.yaml"),
        ("text", "graph_edges_lengths.yaml"),
        ("text", "primitives_lengths.csv"),
        ("text", ".operator_meta.json"),
        ("text", "smap_summary.json"),
    ]


def test_params_and_temporary_slam_params_use_local_atomic_writer(
    runtime: RosRobotRuntime,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    writes: list[Path] = []

    def write_text(path: Path, content: str) -> None:
        writes.append(Path(path))
        real_atomic_write_text(Path(path), content)

    monkeypatch.setattr(ros_runtime_params, "atomic_write_text", write_text)
    payload = {
        "nav2": {
            "amcl": {"update_min_d": 0.25},
        }
    }
    result = runtime.save_params_payload(payload, reload_runtime=False)
    assert result["ok"] is True
    assert writes == [runtime.params_path]
    assert yaml.safe_load(runtime.params_path.read_text(encoding="utf-8")) == payload

    template = {
        "slam_toolbox": {
            "ros__parameters": {
                "base_frame": "base_link",
                "resolution": 0.05,
            }
        }
    }
    incoming = {
        "slam_toolbox": {
            "ros__parameters": {
                "base_frame": "base_footprint",
                "resolution": 1,
            }
        }
    }
    normalized = runtime._normalize_slam_params(
        runtime._coerce_slam_params(incoming, template),
        template,
    )
    assert normalized["slam_toolbox"]["ros__parameters"] == {
        "base_frame": "base_link",
        "resolution": 1.0,
    }


def test_source_path_import_is_standalone(
    tmp_path: Path,
) -> None:
    environment = {
        "PATH": os.environ.get("PATH", ""),
        "PYTHONPATH": os.pathsep.join(
            [str(ROBOT_API_ROOT), str(ROBOT_PLANNER_ROOT)]
        ),
    }
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; "
                "from robot_grpc_api.ros_runtime import RosRobotRuntime; "
                "runtime=RosRobotRuntime(robot_id='r',robot_name='R',autostart=False); "
                "print(runtime.__class__.__module__); "
                "print(len(runtime.__class__.__bases__)); "
                "print('fleet_manager' in sys.modules); "
                "runtime.close()"
            ),
        ],
        cwd=tmp_path,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.splitlines() == [
        "robot_grpc_api.ros_runtime",
        "7",
        "False",
    ]


def test_pure_helpers_keep_standalone_edge_cases(
    runtime: RosRobotRuntime,
) -> None:
    assert runtime._clean_owner_id(" owner\n one ") == "owner one"
    assert runtime._safe_map_name("../../Map name.smap") == "Map_name"
    assert runtime._slam_cells_to_bytes(
        [-100, -1, 0, 50, 100, 999, "bad"]
    ) == bytes([0, 0, 1, 51, 101, 101, 0])
    assert runtime._normalize_angle(4.0) == pytest.approx(-2.283185307)
    assert runtime._stamp_sec(
        SimpleNamespace(sec=12, nanosec=500_000_000)
    ) == 12.5
