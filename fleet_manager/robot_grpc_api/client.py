from __future__ import annotations

import base64
import json
import math
from typing import Any

from .contracts import (
    API_VERSION,
    DEFAULT_GRPC_PORT,
    GRPC_CHANNEL_OPTIONS,
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


def _finite_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _finite_or_default(value: Any, default: float = 0.0) -> float:
    number = _finite_or_none(value)
    return float(default) if number is None else number


def _load_grpc_modules():
    try:
        import grpc  # type: ignore
    except ModuleNotFoundError as exc:
        raise GrpcRobotError(
            "grpcio is not installed. Install it on operator/server and robot: sudo apt install python3-grpcio"
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
            owner_id=str(payload.get("ownerId") or payload.get("owner_id") or ""),
        )
        response = stub.ExecuteRoute(request, timeout=max(self.timeout, 5.0))
        return self._command_response(response)

    def cancel_route(self, endpoint: str, *, owner_id: str = "") -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.CancelRoute(
            robot_api_pb2.CancelRouteRequest(message="Route canceled.", owner_id=str(owner_id or "")),
            timeout=self.timeout,
        )
        return self._command_response(response)

    def teleop(
        self,
        endpoint: str,
        *,
        linear: float,
        angular: float,
        timeout_ms: int = 350,
        owner_id: str = "",
    ) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.Teleop(
            robot_api_pb2.TeleopRequest(
                linear=float(linear),
                angular=float(angular),
                timeout_ms=max(80, int(timeout_ms)),
                owner_id=str(owner_id or ""),
            ),
            timeout=self.timeout,
        )
        return self._command_response(response)

    def teleop_stop(self, endpoint: str, *, owner_id: str = "") -> dict[str, Any]:
        return self.teleop(endpoint, linear=0.0, angular=0.0, timeout_ms=80, owner_id=owner_id)

    def teleop_stream(self, endpoint: str, commands, *, owner_id: str = "") -> Any:
        stub = self._stub(endpoint)

        def _requests():
            for command in commands:
                if not isinstance(command, dict):
                    continue
                yield robot_api_pb2.TeleopRequest(
                    linear=float(command.get("linear", 0.0) or 0.0),
                    angular=float(command.get("angular", 0.0) or 0.0),
                    timeout_ms=max(80, int(command.get("timeoutMs", command.get("timeout_ms", 350)) or 350)),
                    owner_id=str(command.get("ownerId") or command.get("owner_id") or owner_id or ""),
                )

        call = stub.TeleopStream(_requests())
        try:
            for response in call:
                yield self._command_response(response)
        finally:
            cancel = getattr(call, "cancel", None)
            if callable(cancel):
                cancel()

    def watch_laser_scan(
        self,
        endpoint: str,
        *,
        topic: str = "/scan",
        hz: float = 1.0,
        include_intensities: bool = False,
    ) -> Any:
        stub = self._stub(endpoint)
        request = robot_api_pb2.WatchLaserScanRequest(
            topic=str(topic or "/scan"),
            hz=max(0.1, min(10.0, float(hz or 1.0))),
            include_intensities=bool(include_intensities),
        )
        call = stub.WatchLaserScan(request)
        try:
            for response in call:
                yield self._laser_scan_frame(response)
        finally:
            cancel = getattr(call, "cancel", None)
            if callable(cancel):
                cancel()

    def get_slam_defaults(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.GetSlamDefaults(robot_api_pb2.SlamDefaultsRequest(), timeout=max(self.timeout, 5.0))
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "SLAM defaults failed"))
        params = json_loads_object(response.params_json) if response.params_json else {}
        return {"ok": True, "params": params, "paramsPath": str(response.params_path or "")}

    def start_slam(self, endpoint: str, params_payload: dict[str, Any], *, use_sim_time: bool = True) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.StartSlam(
            robot_api_pb2.StartSlamRequest(
                params_json=json_dumps(params_payload if isinstance(params_payload, dict) else {}),
                use_sim_time=bool(use_sim_time),
            ),
            timeout=max(self.timeout, 10.0),
        )
        return self._slam_state_response(response)

    def get_slam_state(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.GetSlamState(robot_api_pb2.SlamStateRequest(), timeout=max(self.timeout, 5.0))
        return self._slam_state_response(response)

    def watch_slam_map(self, endpoint: str, *, hz: float = 1.0, include_cells: bool = True) -> Any:
        stub = self._stub(endpoint)
        request = robot_api_pb2.WatchSlamMapRequest(
            hz=max(0.2, min(5.0, float(hz or 1.0))),
            include_cells=bool(include_cells),
        )
        call = stub.WatchSlamMap(request)
        try:
            for response in call:
                yield self._slam_map_frame(response)
        finally:
            cancel = getattr(call, "cancel", None)
            if callable(cancel):
                cancel()

    def finish_slam(self, endpoint: str, *, map_name: str, activate: bool = True) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.FinishSlam(
            robot_api_pb2.FinishSlamRequest(map_name=str(map_name or ""), activate=bool(activate)),
            timeout=max(self.timeout, 60.0),
        )
        return self._slam_finish_response(response)

    def cancel_slam(self, endpoint: str, *, reason: str = "") -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.CancelSlam(
            robot_api_pb2.CancelSlamRequest(reason=str(reason or "")),
            timeout=max(self.timeout, 10.0),
        )
        return self._slam_state_response(response)

    def stop(self, endpoint: str, *, owner_id: str = "") -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.Stop(robot_api_pb2.StopRequest(owner_id=str(owner_id or "")), timeout=self.timeout)
        return self._command_response(response)

    def acquire_control(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str = "",
        force: bool = False,
        lease_ms: int = 0,
    ) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.AcquireControl(
            robot_api_pb2.ControlRequest(
                owner_id=str(owner_id or ""),
                owner_name=str(owner_name or ""),
                force=bool(force),
                lease_ms=max(0, int(lease_ms or 0)),
            ),
            timeout=self.timeout,
        )
        return self._command_response(response)

    def release_control(self, endpoint: str, *, owner_id: str, force: bool = False) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.ReleaseControl(
            robot_api_pb2.ControlRequest(owner_id=str(owner_id or ""), force=bool(force)),
            timeout=self.timeout,
        )
        return self._command_response(response)

    def relocate(
        self,
        endpoint: str,
        *,
        x: float,
        y: float,
        yaw: float,
        owner_id: str,
        frame_id: str = "map",
        covariance: list[float] | None = None,
        confirm: bool = False,
    ) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.Relocate(
            robot_api_pb2.RelocateRequest(
                owner_id=str(owner_id or ""),
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                frame_id=str(frame_id or "map"),
                covariance_json=json.dumps(covariance or [], ensure_ascii=False),
                confirm=bool(confirm),
            ),
            timeout=max(self.timeout, 5.0),
        )
        return self._command_response(response)

    def confirm_localization(
        self,
        endpoint: str,
        *,
        owner_id: str,
        accepted: bool = True,
        message: str = "",
    ) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.ConfirmLocalization(
            robot_api_pb2.ConfirmLocalizationRequest(
                owner_id=str(owner_id or ""),
                accepted=bool(accepted),
                message=str(message or ""),
            ),
            timeout=self.timeout,
        )
        return self._command_response(response)

    def pause_route(self, endpoint: str, *, owner_id: str, message: str = "") -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.PauseRoute(
            robot_api_pb2.PauseRouteRequest(owner_id=str(owner_id or ""), message=str(message or "")),
            timeout=self.timeout,
        )
        return self._command_response(response)

    def resume_route(self, endpoint: str, *, owner_id: str, message: str = "") -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.ResumeRoute(
            robot_api_pb2.ResumeRouteRequest(owner_id=str(owner_id or ""), message=str(message or "")),
            timeout=self.timeout,
        )
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

    def get_params(self, endpoint: str) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.GetParams(robot_api_pb2.ParamsRequest(), timeout=max(self.timeout, 5.0))
        return self._params_response(response)

    def put_params(
        self,
        endpoint: str,
        params_payload: dict[str, Any],
        *,
        reload_runtime: bool = True,
    ) -> dict[str, Any]:
        stub = self._stub(endpoint)
        response = stub.PutParams(
            robot_api_pb2.PutParamsRequest(
                params_json=json_dumps(params_payload),
                reload_runtime=bool(reload_runtime),
            ),
            timeout=max(self.timeout, 15.0),
        )
        return self._params_response(response)

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
            channel = grpc.secure_channel(parsed.target, grpc.ssl_channel_credentials(), options=GRPC_CHANNEL_OPTIONS)
        else:
            channel = grpc.insecure_channel(parsed.target, options=GRPC_CHANNEL_OPTIONS)
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

    def _params_response(self, response: robot_api_pb2.ParamsResponse) -> dict[str, Any]:
        if not bool(response.ok):
            raise GrpcRobotError(str(response.error or "robot params RPC failed"))
        try:
            params = json.loads(str(response.params_json or "{}"))
        except json.JSONDecodeError as exc:
            raise GrpcRobotError("robot returned invalid params JSON") from exc
        if not isinstance(params, dict):
            raise GrpcRobotError("robot returned invalid params payload")
        payload: dict[str, Any] = {
            "ok": True,
            "params": params,
            "path": str(response.params_path or ""),
            "reloaded": bool(response.reloaded),
        }
        if response.error:
            payload["warning"] = str(response.error)
        return payload

    def _laser_scan_frame(self, response: robot_api_pb2.LaserScanFrame) -> dict[str, Any]:
        return {
            "ok": bool(response.ok),
            "error": str(response.error or ""),
            "robotId": str(response.robot_id or ""),
            "topic": str(response.topic or ""),
            "frameId": str(response.frame_id or ""),
            "stampSec": _finite_or_default(response.stamp_sec),
            "angleMin": _finite_or_default(response.angle_min),
            "angleMax": _finite_or_default(response.angle_max),
            "angleIncrement": _finite_or_default(response.angle_increment),
            "timeIncrement": _finite_or_default(response.time_increment),
            "scanTime": _finite_or_default(response.scan_time),
            "rangeMin": _finite_or_default(response.range_min),
            "rangeMax": _finite_or_default(response.range_max),
            "ranges": [_finite_or_none(item) for item in response.ranges],
            "intensities": [_finite_or_none(item) for item in response.intensities],
        }

    def _slam_state_payload(self, state: Any) -> dict[str, Any]:
        return {
            "active": bool(getattr(state, "active", False)),
            "state": str(getattr(state, "state", "") or "idle"),
            "message": str(getattr(state, "message", "") or ""),
            "sessionId": str(getattr(state, "session_id", "") or ""),
            "startedAtSec": _finite_or_default(getattr(state, "started_at_sec", 0.0)),
            "progress": int(getattr(state, "progress", 0) or 0),
            "savedMapName": str(getattr(state, "saved_map_name", "") or ""),
            "mapDir": str(getattr(state, "map_dir", "") or ""),
            "mapWidth": int(getattr(state, "map_width", 0) or 0),
            "mapHeight": int(getattr(state, "map_height", 0) or 0),
            "resolution": _finite_or_default(getattr(state, "resolution", 0.0)),
            "frameId": str(getattr(state, "frame_id", "") or ""),
            "trailPoints": int(getattr(state, "trail_points", 0) or 0),
        }

    def _slam_state_response(self, response: Any) -> dict[str, Any]:
        payload = {
            "ok": bool(getattr(response, "ok", False)),
            "error": str(getattr(response, "error", "") or ""),
            "state": self._slam_state_payload(getattr(response, "state", None)),
        }
        if not payload["ok"]:
            raise GrpcRobotError(payload["error"] or "SLAM request failed")
        return payload

    def _pose2d_payload(self, pose: Any) -> dict[str, Any]:
        return {
            "x": _finite_or_default(getattr(pose, "x", 0.0)),
            "y": _finite_or_default(getattr(pose, "y", 0.0)),
            "yaw": _finite_or_default(getattr(pose, "yaw", 0.0)),
            "stampSec": _finite_or_default(getattr(pose, "stamp_sec", 0.0)),
        }

    def _slam_map_frame(self, response: Any) -> dict[str, Any]:
        return {
            "ok": bool(getattr(response, "ok", False)),
            "error": str(getattr(response, "error", "") or ""),
            "robotId": str(getattr(response, "robot_id", "") or ""),
            "sessionId": str(getattr(response, "session_id", "") or ""),
            "frameId": str(getattr(response, "frame_id", "") or ""),
            "stampSec": _finite_or_default(getattr(response, "stamp_sec", 0.0)),
            "width": int(getattr(response, "width", 0) or 0),
            "height": int(getattr(response, "height", 0) or 0),
            "resolution": _finite_or_default(getattr(response, "resolution", 0.0)),
            "originX": _finite_or_default(getattr(response, "origin_x", 0.0)),
            "originY": _finite_or_default(getattr(response, "origin_y", 0.0)),
            "originYaw": _finite_or_default(getattr(response, "origin_yaw", 0.0)),
            "cellsBase64": base64.b64encode(bytes(getattr(response, "cells", b"") or b"")).decode("ascii"),
            "pose": self._pose2d_payload(getattr(response, "pose", None)),
            "trail": [self._pose2d_payload(item) for item in getattr(response, "trail", [])],
            "state": self._slam_state_payload(getattr(response, "state", None)),
        }

    def _slam_finish_response(self, response: Any) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "ok": bool(getattr(response, "ok", False)),
            "error": str(getattr(response, "error", "") or ""),
            "state": self._slam_state_payload(getattr(response, "state", None)),
            "mapName": str(getattr(response, "map_name", "") or ""),
            "mapDir": str(getattr(response, "map_dir", "") or ""),
            "mapId": str(getattr(response, "map_id", "") or ""),
            "signature": str(getattr(response, "signature", "") or ""),
        }
        if not payload["ok"]:
            raise GrpcRobotError(payload["error"] or "SLAM finish failed")
        bundle_json = str(getattr(response, "bundle_json", "") or "")
        if bundle_json:
            try:
                bundle = json.loads(bundle_json)
            except json.JSONDecodeError:
                bundle = {}
            if isinstance(bundle, dict):
                payload["bundle"] = bundle
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

    def cancel_route(self, endpoint: str, *, owner_id: str = "") -> dict[str, Any]:
        return self.client.cancel_route(endpoint, owner_id=owner_id)

    def teleop(
        self,
        endpoint: str,
        *,
        linear: float,
        angular: float,
        timeout_ms: int = 350,
        owner_id: str = "",
    ) -> dict[str, Any]:
        return self.client.teleop(endpoint, linear=linear, angular=angular, timeout_ms=timeout_ms, owner_id=owner_id)

    def teleop_stop(self, endpoint: str, *, owner_id: str = "") -> dict[str, Any]:
        return self.client.teleop_stop(endpoint, owner_id=owner_id)

    def teleop_stream(self, endpoint: str, commands, *, owner_id: str = "") -> Any:
        return self.client.teleop_stream(endpoint, commands, owner_id=owner_id)

    def watch_laser_scan(
        self,
        endpoint: str,
        *,
        topic: str = "/scan",
        hz: float = 1.0,
        include_intensities: bool = False,
    ) -> Any:
        return self.client.watch_laser_scan(
            endpoint,
            topic=topic,
            hz=hz,
            include_intensities=include_intensities,
        )

    def get_slam_defaults(self, endpoint: str) -> dict[str, Any]:
        return self.client.get_slam_defaults(endpoint)

    def start_slam(self, endpoint: str, params_payload: dict[str, Any], *, use_sim_time: bool = True) -> dict[str, Any]:
        return self.client.start_slam(endpoint, params_payload, use_sim_time=use_sim_time)

    def get_slam_state(self, endpoint: str) -> dict[str, Any]:
        return self.client.get_slam_state(endpoint)

    def watch_slam_map(self, endpoint: str, *, hz: float = 1.0, include_cells: bool = True) -> Any:
        return self.client.watch_slam_map(endpoint, hz=hz, include_cells=include_cells)

    def finish_slam(self, endpoint: str, *, map_name: str, activate: bool = True) -> dict[str, Any]:
        return self.client.finish_slam(endpoint, map_name=map_name, activate=activate)

    def cancel_slam(self, endpoint: str, *, reason: str = "") -> dict[str, Any]:
        return self.client.cancel_slam(endpoint, reason=reason)

    def stop(self, endpoint: str, *, owner_id: str = "") -> dict[str, Any]:
        return self.client.stop(endpoint, owner_id=owner_id)

    def acquire_control(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str = "",
        force: bool = False,
        lease_ms: int = 0,
    ) -> dict[str, Any]:
        return self.client.acquire_control(
            endpoint,
            owner_id=owner_id,
            owner_name=owner_name,
            force=force,
            lease_ms=lease_ms,
        )

    def release_control(self, endpoint: str, *, owner_id: str, force: bool = False) -> dict[str, Any]:
        return self.client.release_control(endpoint, owner_id=owner_id, force=force)

    def relocate(
        self,
        endpoint: str,
        *,
        x: float,
        y: float,
        yaw: float,
        owner_id: str,
        frame_id: str = "map",
        covariance: list[float] | None = None,
        confirm: bool = False,
    ) -> dict[str, Any]:
        return self.client.relocate(
            endpoint,
            x=x,
            y=y,
            yaw=yaw,
            owner_id=owner_id,
            frame_id=frame_id,
            covariance=covariance,
            confirm=confirm,
        )

    def confirm_localization(
        self,
        endpoint: str,
        *,
        owner_id: str,
        accepted: bool = True,
        message: str = "",
    ) -> dict[str, Any]:
        return self.client.confirm_localization(endpoint, owner_id=owner_id, accepted=accepted, message=message)

    def pause_route(self, endpoint: str, *, owner_id: str, message: str = "") -> dict[str, Any]:
        return self.client.pause_route(endpoint, owner_id=owner_id, message=message)

    def resume_route(self, endpoint: str, *, owner_id: str, message: str = "") -> dict[str, Any]:
        return self.client.resume_route(endpoint, owner_id=owner_id, message=message)

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

    def get_params(self, endpoint: str) -> dict[str, Any]:
        return self.client.get_params(endpoint)

    def put_params(
        self,
        endpoint: str,
        params_payload: dict[str, Any],
        *,
        reload_runtime: bool = True,
    ) -> dict[str, Any]:
        return self.client.put_params(endpoint, params_payload, reload_runtime=reload_runtime)

    def normalize_endpoint(self, endpoint: str) -> str:
        return self.client.normalize_endpoint(endpoint)
