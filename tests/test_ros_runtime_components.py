from __future__ import annotations

import inspect
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
import yaml

from fleet_manager.map_data.pgm import PgmImage
from fleet_manager.runtime.grpc.api import ros_runtime_control
from fleet_manager.runtime.grpc.api import ros_runtime_params
from fleet_manager.runtime.grpc.api import ros_runtime_slam
from fleet_manager.runtime.grpc.api.ros_runtime import RosRobotRuntime
from fleet_manager.runtime.grpc.api.ros_runtime_control import (
    RosRuntimeControlMixin,
)
from fleet_manager.runtime.grpc.api.ros_runtime_lifecycle import (
    RosRuntimeLifecycleMixin,
)
from fleet_manager.runtime.grpc.api.ros_runtime_maps import (
    RosRuntimeMapTransferMixin,
)
from fleet_manager.runtime.grpc.api.ros_runtime_params import (
    RosRuntimeParametersMixin,
)
from fleet_manager.runtime.grpc.api.ros_runtime_ros_helpers import (
    RosRuntimeMessageServiceMixin,
)
from fleet_manager.runtime.grpc.api.ros_runtime_slam import (
    RosRuntimeSlamMixin,
)
from fleet_manager.storage import (
    atomic_write_bytes as real_atomic_write_bytes,
)
from fleet_manager.storage import (
    atomic_write_text as real_atomic_write_text,
)


EXPECTED_COMPONENT_METHODS = {
    RosRuntimeLifecycleMixin: {
        "__init__",
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
        "_on_scan",
        "_on_map",
        "_append_slam_trail_locked",
        "_latest_message",
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
        "_execute_pose_route",
        "_route_payload_from_request",
        "_goal_pose_payload",
        "_current_pose_payload",
        "_yaw_from_quaternion",
        "_set_route_paused",
        "_control_payload",
        "_control_state_payload",
        "_ensure_control_owner",
        "_expire_control_owner_locked",
        "_clean_owner_id",
        "_covariance_from_json",
        "_route_payload",
        "_persist_event",
    },
    RosRuntimeMapTransferMixin: {
        "active_map_payload",
        "list_maps_payload",
        "pull_map_bundle_payload",
        "push_map_bundle_payload",
        "load_map",
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
        "_slam_maps_root",
        "_save_slam_map_files",
        "_write_current_map_files",
        "_write_empty_smap_sidecars",
        "_pgm_size",
        "_stop_slam_process",
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
    "initial_pose_topic",
    "scan_topic",
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
    "slam_params_file",
    "slam_launch_file",
    "params_path",
    "_lock",
    "_latest_status",
    "_latest_status_at",
    "_latest_scan",
    "_latest_scan_at",
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
    "_twist_type",
    "_pose_with_covariance_type",
    "_laser_scan_type",
    "_string_type",
    "_set_parameters_type",
    "_list_parameters_type",
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
    "_save_map_type",
    "_occupancy_grid_type",
    "_slam_process",
    "_slam_temp_dir",
    "_slam_state",
    "_slam_trail",
    "_nav2_param_clients",
    "_nav2_list_param_clients",
]


@pytest.fixture
def runtime(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> RosRobotRuntime:
    monkeypatch.setattr(
        RosRuntimeLifecycleMixin,
        "_start",
        lambda self: None,
    )
    instance = RosRobotRuntime(
        robot_id="robot-01",
        robot_name="Robot 01",
        host="127.0.0.1",
        domain_id=7,
        namespace="warehouse/a",
        slam_params_file=str(tmp_path / "slam-params.yaml"),
        slam_launch_file=str(tmp_path / "slam-launch.py"),
        params_path=str(tmp_path / "runtime-params.yaml"),
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
        "robot_id": "ros-r1",
        "map_id": "map-a",
        "connected": True,
        "localization_ok": True,
        "localization_age_sec": 0.25,
        "state": "MOVING",
        "message": "Following route",
        "target_lm": "LM-9",
        "nearest_lm": "LM-2",
        "current_edge_id": "LM-2->LM-3",
        "route_id": "route-17",
        "route_progress": 0.375,
        "pose_x": 1.25,
        "pose_y": -2.5,
        "pose_yaw": 0.75,
        "linear_velocity": 0.4,
        "angular_velocity": -0.2,
        "battery_level": 82.0,
        "battery_voltage": 25.1,
        "battery_current": 1.2,
        "battery_temperature": 31.5,
        "battery_charging": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_facade_composes_six_disjoint_capabilities() -> None:
    assert RosRobotRuntime.__bases__ == tuple(EXPECTED_COMPONENT_METHODS)
    owned: set[str] = set()
    for component, expected in EXPECTED_COMPONENT_METHODS.items():
        actual = _defined_methods(component)
        assert actual == expected
        assert owned.isdisjoint(actual)
        owned.update(actual)
    assert len(owned) == 80
    assert _defined_methods(RosRobotRuntime) == set()


def test_initialization_preserves_attribute_order_and_locks(
    runtime: RosRobotRuntime,
) -> None:
    assert list(vars(runtime)) == EXPECTED_STATE_KEYS
    assert runtime.namespace == "warehouse/a"
    assert runtime.status_topic == "/warehouse/a/robot_status"
    assert runtime.map_topic == "/warehouse/a/map"
    assert runtime.available is False
    assert runtime.error == ""
    assert runtime._lock.acquire(blocking=False)
    runtime._lock.release()


def test_close_preserves_ros_shutdown_order(runtime: RosRobotRuntime) -> None:
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


def test_status_identity_and_sidebar_payload_contract(
    runtime: RosRobotRuntime,
) -> None:
    runtime._latest_status = _status_message()
    runtime._available = True
    runtime._events = [
        {"stamp": 1.0, "level": "info", "message": "ready"}
    ]

    status = runtime.status_payload()
    assert list(status) == ["ok", "robot", "events", "route"]
    assert status["robot"]["robotId"] == "ros-r1"
    assert status["robot"]["pose"] == {"x": 1.25, "y": -2.5, "yaw": 0.75}
    assert status["robot"]["velocity"] == {
        "linear": 0.4,
        "angular": -0.2,
    }
    assert status["route"] == {
        "routeId": "route-17",
        "goalLm": "LM-9",
        "nodes": ["LM-9"],
        "trajectory": [],
    }

    identity = runtime.identity_payload()
    assert identity["robotId"] == "ros-r1"
    assert identity["namespace"] == "warehouse/a"
    assert identity["available"] is True

    sidebar = runtime.sidebar_payload()
    assert sidebar["id"] == "robot-01"
    assert sidebar["online"] is True
    assert sidebar["status"]["state"] == "MOVING"
    assert sidebar["identity"] == sidebar["lastIdentity"]


def test_control_lease_and_teleop_are_lock_safe(
    runtime: RosRobotRuntime,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = [100.0]
    monkeypatch.setattr(ros_runtime_control, "monotonic", lambda: now[0])

    published: list[Any] = []

    class Twist:
        def __init__(self) -> None:
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    runtime._twist_type = Twist
    runtime._cmd_vel_pub = SimpleNamespace(publish=published.append)

    acquired = runtime.acquire_control(
        owner_id=" operator  one ",
        owner_name="Operator One",
        lease_ms=100,
    )
    assert acquired["control"]["ownerId"] == "operator one"
    assert runtime.teleop(
        linear=0.5,
        angular=-0.25,
        owner_id="operator one",
    ) == {"ok": True, "linear": 0.5, "angular": -0.25}
    assert published[-1].linear.x == 0.5
    assert published[-1].angular.z == -0.25

    now[0] = 100.101
    assert runtime._control_state_payload() == {
        "state": "FREE",
        "ownerId": "",
        "ownerName": "",
        "leaseMs": 0,
        "acquiredAgeSec": 0.0,
    }


def test_scan_and_message_conversion_do_not_need_ros(
    runtime: RosRobotRuntime,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        ros_runtime_control,
        "monotonic",
        lambda: 52.0,
    )
    runtime._latest_scan_at = 50.5
    runtime._latest_scan = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="laser",
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000),
        ),
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=0.1,
        time_increment=0.01,
        scan_time=0.2,
        range_min=0.05,
        range_max=20.0,
        ranges=[1, 2.5],
        intensities=[10, 20],
    )
    payload = runtime.laser_scan_payload(include_intensities=True)
    assert payload["stampSec"] == 12.5
    assert payload["receivedAgeSec"] == 1.5
    assert payload["ranges"] == [1.0, 2.5]
    assert payload["intensities"] == [10.0, 20.0]

    disconnected = runtime._message_to_robot_payload(None)
    assert disconnected["state"] == "DISCONNECTED"
    assert disconnected["connected"] is False
    assert disconnected["pose"] is None


def test_map_transfer_payloads_keep_service_json_contract(
    runtime: RosRobotRuntime,
) -> None:
    class Request:
        pass

    client = SimpleNamespace(
        srv_type=SimpleNamespace(Request=Request),
    )
    runtime._map_state_client = client
    runtime._map_list_client = client
    runtime._map_get_bundle_client = client
    runtime._map_put_bundle_client = client
    runtime._map_load_client = client
    requests: list[tuple[str, Any, float]] = []

    def call_service(
        _client: Any,
        request: Any,
        label: str,
        *,
        timeout_sec: float,
    ) -> Any:
        requests.append((label, request, timeout_sec))
        if label == "map state":
            return SimpleNamespace(
                ok=True,
                error="",
                map_name="alpha",
                map_dir="/maps/alpha.smap",
                map_id="alpha-id",
            )
        if label == "map list":
            return SimpleNamespace(
                ok=True,
                error="",
                active_map_name="alpha",
                active_map_dir="/maps/alpha.smap",
                active_map_id="alpha-id",
                map_names=["alpha", "beta.smap"],
                map_dirs=["/maps/alpha.smap", "/maps/beta.smap"],
                map_ids=["alpha-id", "beta-id"],
            )
        if label == "map bundle pull":
            return SimpleNamespace(
                ok=True,
                error="",
                bundle_json='{"files": {"LMs.yaml": "abc"}}',
                map_name="alpha",
                map_dir="/maps/alpha.smap",
                signature="sig-a",
            )
        return SimpleNamespace(
            ok=True,
            error="",
            map_name="alpha",
            map_dir="/maps/alpha.smap",
            map_id="alpha-id",
            signature="sig-a",
        )

    runtime._call_service = call_service
    assert runtime.active_map_payload()["mapId"] == "alpha-id"
    assert runtime.list_maps_payload()["maps"][1]["folder"] == "beta.smap"
    assert runtime.pull_map_bundle_payload("alpha")["signature"] == "sig-a"
    assert runtime.push_map_bundle_payload(
        {"mapName": "alpha", "files": {}},
        activate=True,
    )["mapName"] == "alpha"
    assert runtime.load_map("alpha")["mapDir"] == "/maps/alpha.smap"
    assert [label for label, _, _ in requests] == [
        "map state",
        "map list",
        "map bundle pull",
        "map bundle push",
        "map load",
    ]
    pushed_request = requests[3][1]
    assert json.loads(pushed_request.bundle_json)["mapName"] == "alpha"
    assert pushed_request.activate is True


def test_slam_map_persistence_uses_atomic_writes_and_shared_pgm(
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
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0, 100, -1, 50],
    )
    target = tmp_path / "capture.smap"
    target.mkdir()

    runtime._write_current_map_files(target, "capture")
    runtime._write_empty_smap_sidecars(target, "capture")

    image = PgmImage.read(target / "capture.pgm")
    assert (image.width, image.height) == (2, 2)
    assert image.pixels == bytes([205, 205, 254, 0])
    assert runtime._pgm_size(target / "capture.pgm") == (2, 2)
    summary = json.loads(
        (target / "smap_summary.json").read_text(encoding="utf-8")
    )
    assert summary["counts"]["grid"] == {"width": 2, "height": 2}
    assert writes == [
        ("bytes", "capture.pgm"),
        ("text", "capture.yaml"),
        ("text", "LMs.yaml"),
        ("text", "graphs.yaml"),
        ("text", "graph_edges_lengths.yaml"),
        ("text", "primitives_lengths.csv"),
        ("text", ".operator_meta.json"),
        ("text", "smap_summary.json"),
    ]


def test_parameter_persistence_is_atomic_and_helpers_stay_typed(
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
            "amcl": {"update_min_d": 0.2},
            "velocity_smoother": {"max_velocity_x": 0.5},
        }
    }
    result = runtime.save_params_payload(payload, reload_runtime=False)
    assert result["ok"] is True
    assert result["reloaded"] is False
    assert writes == [runtime.params_path]
    assert yaml.safe_load(runtime.params_path.read_text(encoding="utf-8")) == payload
    assert runtime._deep_get(payload, "nav2.amcl.update_min_d") == 0.2
    assert runtime._deep_get(payload, "nav2.missing") is None
    assert runtime._values_equal(0.2, 0.2000001) is True
    assert runtime._values_equal("0.2", "0.3") is False


def test_publisher_and_service_helpers_have_small_fakeable_boundaries(
    runtime: RosRobotRuntime,
) -> None:
    published: list[Any] = []

    class String:
        data = ""

    runtime._string_type = String
    runtime._go_to_lm_pub = SimpleNamespace(publish=published.append)
    runtime._publish_go_to_lm('{"id":"LM-5"}')
    assert published[-1].data == '{"id":"LM-5"}'

    response = SimpleNamespace(ok=True)
    future = SimpleNamespace(
        done=lambda: True,
        exception=lambda: None,
        result=lambda: response,
    )
    client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: timeout_sec > 0,
        call_async=lambda request: future,
    )
    assert runtime._service_available(client) is True
    assert runtime._call_service(
        client,
        SimpleNamespace(),
        "fake",
    ) is response


def test_pure_normalizers_and_slam_cells_keep_edge_cases(
    runtime: RosRobotRuntime,
) -> None:
    assert runtime._clean_owner_id("  operator\n  one  ") == "operator one"
    assert runtime._safe_map_name("../../Map name.smap") == "Map_name"
    assert runtime._covariance_from_json("[1, 2]") == []
    covariance = runtime._covariance_from_json(
        json.dumps(list(range(36)))
    )
    assert covariance == [float(value) for value in range(36)]
    assert runtime._slam_cells_to_bytes(
        [-100, -1, 0, 50, 100, 999, "bad"]
    ) == bytes([0, 0, 1, 51, 101, 101, 0])
