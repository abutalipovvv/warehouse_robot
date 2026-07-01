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
