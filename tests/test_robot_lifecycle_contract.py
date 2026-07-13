import json
from pathlib import Path

from operator_app.core.models import KnownRobot
from operator_app.core.state import OperatorAppState
from operator_app.robot_grpc_api.contracts import robot_status_from_json, robot_status_to_json
from operator_app.robot_grpc_api.proto import robot_api_pb2, robot_api_pb2_grpc


def test_lifecycle_rpc_methods_are_in_client_contract() -> None:
    assert hasattr(robot_api_pb2, "ControlRequest")
    assert hasattr(robot_api_pb2, "RelocateRequest")
    assert hasattr(robot_api_pb2, "PauseRouteRequest")
    assert hasattr(robot_api_pb2, "ResumeRouteRequest")
    assert "AcquireControl" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert "Relocate" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert "PauseRoute" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert "ResumeRoute" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert hasattr(robot_api_pb2_grpc.RobotApiServicer, "AcquireControl")
    assert hasattr(robot_api_pb2_grpc.RobotApiServicer, "ReleaseControl")
    assert hasattr(robot_api_pb2_grpc.RobotApiServicer, "ConfirmLocalization")


def test_status_roundtrip_preserves_lifecycle_fields_in_raw_json() -> None:
    payload = {
        "robotId": "robot1",
        "connected": True,
        "state": "PAUSED",
        "controlState": "OWNED",
        "controlOwner": "operator-app",
        "control": {"state": "OWNED", "ownerId": "operator-app", "ownerName": "Operator App"},
        "navigationPaused": True,
        "localizationConfirmed": False,
    }
    status = robot_status_from_json(payload)
    decoded = robot_status_to_json(status)
    assert decoded["controlOwner"] == "operator-app"
    assert decoded["control"]["ownerName"] == "Operator App"
    assert decoded["navigationPaused"] is True
    assert decoded["localizationConfirmed"] is False


def test_operator_takeover_stops_previous_autonomous_route() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    adapter = _TakeoverAdapter()
    state.grpc_adapter = adapter
    robot = KnownRobot(id="robot1", name="robot1", host="127.0.0.1", port=50051)
    state.get_robot = lambda robot_id: robot

    status, _, body = state._proxy_grpc_robot_request(
        "robot1",
        "POST",
        "/api/robot/control/acquire",
        body=json.dumps({"force": True, "stopNavigation": True}).encode("utf-8"),
    )
    payload = json.loads(body.decode("utf-8"))

    assert status == 200
    assert payload["navigationStopped"] is True
    assert payload["status"]["robot"]["state"] == "IDLE"
    assert adapter.calls == [
        ("acquire", "grpc://127.0.0.1:50051", "operator-app", True),
        ("stop", "grpc://127.0.0.1:50051", "operator-app"),
    ]


def test_operator_ui_requests_force_takeover_and_graph_safe_fleet_pose() -> None:
    app_js = (Path(__file__).resolve().parents[1] / "operator_app" / "static" / "app.js").read_text(
        encoding="utf-8"
    )

    assert 'acquireRobotControl(true, true)' in app_js
    assert 'stopNavigation: force' in app_js
    assert 'async startFleetPoseNavigation(world)' in app_js
    assert 'await this.startFleetNavigation(nearest.landmark.name' in app_js


class _TakeoverAdapter:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def acquire_control(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
        force: bool,
        lease_ms: int,
    ) -> dict:
        del owner_name, lease_ms
        self.calls.append(("acquire", endpoint, owner_id, force))
        return {
            "ok": True,
            "status": {"ok": True, "robot": {"state": "EXECUTING_ROUTE"}},
        }

    def stop(self, endpoint: str, *, owner_id: str) -> dict:
        self.calls.append(("stop", endpoint, owner_id))
        return {
            "ok": True,
            "status": {"ok": True, "robot": {"state": "IDLE"}},
        }
