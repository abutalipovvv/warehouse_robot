from __future__ import annotations

import json
from typing import Any

from .contracts import (
    API_VERSION,
    DEFAULT_GRPC_PORT,
    RobotApiError,
    json_dumps,
    json_loads_object,
    normalize_grpc_endpoint,
    parse_grpc_endpoint,
    robot_status_to_json,
)
from .proto import robot_api_pb2


class GrpcRobotError(RobotApiError):
    pass


def _load_grpc_modules():
    try:
        import grpc  # type: ignore
    except ModuleNotFoundError as exc:
        raise GrpcRobotError(
            "grpcio is not installed. Install it on operator/server and robot: python3 -m pip install grpcio"
        ) from exc
    from .proto import robot_api_pb2_grpc

    return grpc, robot_api_pb2_grpc


class GrpcRobotClient:
    def __init__(self, *, timeout: float = 1.5, default_port: int = DEFAULT_GRPC_PORT) -> None:
        self.timeout = max(0.2, float(timeout))
        self.default_port = int(default_port)
        self._channels: dict[str, Any] = {}
        self._stubs: dict[str, Any] = {}

    def close(self) -> None:
        for channel in list(self._channels.values()):
            close = getattr(channel, "close", None)
            if callable(close):
                close()
        self._channels.clear()
        self._stubs.clear()

    def health(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.Health(robot_api_pb2.HealthRequest(), timeout=self.timeout)
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "robot API health failed"))
        return {"ok": True, "service": str(response.service or API_VERSION)}

    def identity(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.Identity(robot_api_pb2.IdentityRequest(), timeout=self.timeout)
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "robot identity failed"))
        return {
            "ok": True,
            "robotId": str(response.robot_id or ""),
            "name": str(response.robot_name or response.robot_id or ""),
            "mapId": str(response.map_id or ""),
            "apiVersion": str(response.api_version or API_VERSION),
            "driver": str(response.driver or "grpc"),
            "type": "grpc",
        }

    def status(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.GetStatus(robot_api_pb2.StatusRequest(), timeout=self.timeout)
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "robot status failed"))
        robot = robot_status_to_json(response.status)
        return {
            "ok": True,
            "robot": robot,
            "events": [],
            "route": self._route_payload(robot),
        }

    def execute_route(self, endpoint: str, payload: dict[str, Any]) -> dict[str, Any]:
        stub = self._stub(endpoint)
        route_payload = payload.get("route") if isinstance(payload.get("route"), dict) else payload
        route_goal_lm = route_payload.get("goalLm") if isinstance(route_payload, dict) else ""
        route_start_lm = route_payload.get("startLm") if isinstance(route_payload, dict) else ""
        request = robot_api_pb2.ExecuteRouteRequest(
            command_id=str(payload.get("commandId") or payload.get("command_id") or ""),
            route_json=json_dumps(route_payload if isinstance(route_payload, dict) else {}),
            goal_lm=str(payload.get("goalLm") or payload.get("goal_lm") or route_goal_lm or ""),
            start_lm=str(payload.get("startLm") or payload.get("start_lm") or route_start_lm or ""),
            order_json=json_dumps(payload.get("order") if isinstance(payload.get("order"), dict) else {}),
        )
        response = stub.ExecuteRoute(request, timeout=max(self.timeout, 5.0))
        return self._command_response(response)

    def cancel_route(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.CancelRoute(robot_api_pb2.CancelRouteRequest(message="Route canceled."), timeout=self.timeout)
        return self._command_response(response)

    def teleop(self, endpoint: str, *, linear: float, angular: float, timeout_ms: int = 350) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.Teleop(
            robot_api_pb2.TeleopRequest(linear=float(linear), angular=float(angular), timeout_ms=max(80, int(timeout_ms))),
            timeout=self.timeout,
        )
        return self._command_response(response)

    def teleop_stop(self, endpoint: str) -> dict[str, Any]:
        return self.teleop(endpoint, linear=0.0, angular=0.0, timeout_ms=80)

    def stop(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.Stop(robot_api_pb2.StopRequest(), timeout=self.timeout)
        return self._command_response(response)

    def list_maps(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.ListMaps(robot_api_pb2.ListMapsRequest(), timeout=max(self.timeout, 5.0))
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "robot map list failed"))
        maps = [
            {
                "name": str(item.name or ""),
                "folder": str(item.folder or item.name or ""),
                "mapDir": str(item.map_dir or ""),
                "mapId": str(item.map_id or ""),
                "signature": str(item.signature or ""),
                "active": bool(item.active),
            }
            for item in response.maps
        ]
        return {
            "ok": True,
            "active": str(response.active_map_name or ""),
            "activeMapDir": str(response.active_map_dir or ""),
            "activeMapId": str(response.active_map_id or ""),
            "maps": maps,
        }

    def active_map(self, endpoint: str) -> dict[str, Any]:
        maps = self.list_maps(endpoint)
        active = str(maps.get("active") or "")
        active_item = next((item for item in maps.get("maps", []) if item.get("active")), {})
        return {
            "ok": True,
            "mapName": active,
            "mapDir": str(active_item.get("mapDir") or maps.get("activeMapDir") or ""),
            "mapId": str(active_item.get("mapId") or maps.get("activeMapId") or ""),
            "signature": str(active_item.get("signature") or ""),
        }

    def get_map_bundle(self, endpoint: str, map_name: str = "") -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.GetMapBundle(
            robot_api_pb2.GetMapBundleRequest(map_name=str(map_name or "")),
            timeout=max(self.timeout, 20.0),
        )
        return self._map_bundle_response(response, include_bundle=True)

    def put_map_bundle(
        self,
        endpoint: str,
        bundle_payload: dict[str, Any],
        *,
        map_name: str = "",
        activate: bool = False,
    ) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.PutMapBundle(
            robot_api_pb2.PutMapBundleRequest(
                map_name=str(map_name or bundle_payload.get("mapName") or ""),
                bundle_json=json_dumps(bundle_payload),
                activate=bool(activate),
            ),
            timeout=max(self.timeout, 30.0),
        )
        return self._map_bundle_response(response, include_bundle=False)

    def load_map(self, endpoint: str, map_name: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.LoadMap(
            robot_api_pb2.LoadMapRequest(map_name=str(map_name or "")),
            timeout=max(self.timeout, 20.0),
        )
        return self._map_bundle_response(response, include_bundle=False)

    def normalize_endpoint(self, endpoint: str) -> str:
        return normalize_grpc_endpoint(endpoint, default_port=self.default_port)

    def _stub(self, endpoint: str):
        parsed = parse_grpc_endpoint(endpoint, default_port=self.default_port)
        key = parsed.url
        cached = self._stubs.get(key)
        if cached is not None:
            return cached
        grpc, robot_api_pb2_grpc = _load_grpc_modules()
        if parsed.secure:
            channel = grpc.secure_channel(parsed.target, grpc.ssl_channel_credentials())
        else:
            channel = grpc.insecure_channel(parsed.target)
        stub = robot_api_pb2_grpc.RobotApiStub(channel)
        self._channels[key] = channel
        self._stubs[key] = stub
        return stub

    def _command_response(self, response: robot_api_pb2.CommandResponse) -> dict[str, Any]:
        robot = robot_status_to_json(response.status)
        payload: dict[str, Any] = {
            "ok": bool(response.ok),
            "commandId": str(response.command_id or ""),
            "status": {"ok": True, "robot": robot, "route": self._route_payload(robot), "events": []},
        }
        if response.error:
            payload["error"] = str(response.error)
        if response.route_json:
            try:
                payload["route"] = json_loads_object(response.route_json)
            except RobotApiError:
                payload["routeJson"] = str(response.route_json)
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "robot command failed"))
        return payload

    def _map_bundle_response(self, response: robot_api_pb2.MapBundleResponse, *, include_bundle: bool) -> dict[str, Any]:
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "robot map RPC failed"))
        payload: dict[str, Any] = {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
            "signature": str(response.signature or ""),
        }
        if include_bundle:
            try:
                bundle = json.loads(str(response.bundle_json or "{}"))
            except json.JSONDecodeError as exc:
                raise GrpcRobotError("robot returned invalid map bundle JSON") from exc
            if not isinstance(bundle, dict):
                raise GrpcRobotError("robot returned invalid map bundle payload")
            bundle.setdefault("ok", True)
            bundle.setdefault("mapName", payload["mapName"])
            bundle.setdefault("mapDir", payload["mapDir"])
            bundle.setdefault("signature", payload["signature"])
            return bundle
        return payload

    def _route_payload(self, robot: dict[str, Any]) -> dict[str, Any]:
        return {
            "active": str(robot.get("state") or "").upper() in {"EXECUTING_ROUTE", "MOVING", "WAITING"},
            "routeId": str(robot.get("routeId") or ""),
            "targetLm": str(robot.get("targetLm") or ""),
            "progress": float(robot.get("routeProgress") or 0.0),
        }


class GrpcRobotAdapter:
    transport = "grpc"

    def __init__(self, *, timeout: float = 1.5, default_port: int = DEFAULT_GRPC_PORT) -> None:
        self.client = GrpcRobotClient(timeout=timeout, default_port=default_port)

    def identity(self, endpoint: str) -> dict[str, Any]:
        return self.client.identity(endpoint)

    def status(self, endpoint: str) -> dict[str, Any]:
        return self.client.status(endpoint)

    def execute_route(self, endpoint: str, payload: dict[str, Any]) -> dict[str, Any]:
        return self.client.execute_route(endpoint, payload)

    def cancel_route(self, endpoint: str) -> dict[str, Any]:
        return self.client.cancel_route(endpoint)

    def teleop(self, endpoint: str, *, linear: float, angular: float, timeout_ms: int = 350) -> dict[str, Any]:
        return self.client.teleop(endpoint, linear=linear, angular=angular, timeout_ms=timeout_ms)

    def teleop_stop(self, endpoint: str) -> dict[str, Any]:
        return self.client.teleop_stop(endpoint)

    def stop(self, endpoint: str) -> dict[str, Any]:
        return self.client.stop(endpoint)

    def list_maps(self, endpoint: str) -> dict[str, Any]:
        return self.client.list_maps(endpoint)

    def active_map(self, endpoint: str) -> dict[str, Any]:
        return self.client.active_map(endpoint)

    def get_map_bundle(self, endpoint: str, map_name: str = "") -> dict[str, Any]:
        return self.client.get_map_bundle(endpoint, map_name)

    def put_map_bundle(
        self,
        endpoint: str,
        bundle_payload: dict[str, Any],
        *,
        map_name: str = "",
        activate: bool = False,
    ) -> dict[str, Any]:
        return self.client.put_map_bundle(endpoint, bundle_payload, map_name=map_name, activate=activate)

    def load_map(self, endpoint: str, map_name: str) -> dict[str, Any]:
        return self.client.load_map(endpoint, map_name)

    def normalize_endpoint(self, endpoint: str) -> str:
        return self.client.normalize_endpoint(endpoint)
