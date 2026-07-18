from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any
from urllib.parse import parse_qs, urlencode, urlparse, urlunparse

from .proto import robot_api_pb2

DEFAULT_GRPC_PORT = 50051
DEFAULT_GRPC_MAX_MESSAGE_BYTES = 128 * 1024 * 1024
DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC = 60.0
DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC = 300.0
DEFAULT_GRPC_MAP_LOAD_TIMEOUT_SEC = 120.0
GRPC_CHANNEL_OPTIONS: tuple[tuple[str, int], ...] = (
    ("grpc.max_send_message_length", DEFAULT_GRPC_MAX_MESSAGE_BYTES),
    ("grpc.max_receive_message_length", DEFAULT_GRPC_MAX_MESSAGE_BYTES),
)
API_VERSION = "robot.grpc.v1"


class RobotApiError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class RobotEndpoint:
    host: str
    port: int = DEFAULT_GRPC_PORT
    robot_id: str = ""
    name: str = ""
    secure: bool = False

    @property
    def target(self) -> str:
        host = f"[{self.host}]" if ":" in self.host and not self.host.startswith("[") else self.host
        return f"{host}:{self.port}"

    @property
    def url(self) -> str:
        scheme = "grpcs" if self.secure else "grpc"
        query: dict[str, str] = {}
        if self.robot_id:
            query["robotId"] = self.robot_id
        if self.name:
            query["name"] = self.name
        return build_grpc_endpoint(self.host, self.port, secure=self.secure, query=query)


def build_grpc_endpoint(
    host: str,
    port: int = DEFAULT_GRPC_PORT,
    *,
    secure: bool = False,
    query: dict[str, str] | None = None,
) -> str:
    clean_host = str(host or "").strip()
    if not clean_host:
        raise RobotApiError("robot host is required")
    try:
        clean_port = int(port)
    except (TypeError, ValueError) as exc:
        raise RobotApiError("robot gRPC port must be an integer") from exc
    if clean_port < 1 or clean_port > 65535:
        raise RobotApiError("robot gRPC port must be in range 1..65535")
    netloc = f"[{clean_host}]" if ":" in clean_host and not clean_host.startswith("[") else clean_host
    netloc = f"{netloc}:{clean_port}"
    return urlunparse(("grpcs" if secure else "grpc", netloc, "", "", urlencode(query or {}), ""))


def parse_grpc_endpoint(value: str, *, default_port: int = DEFAULT_GRPC_PORT) -> RobotEndpoint:
    raw = str(value or "").strip().rstrip("/")
    if not raw:
        raise RobotApiError("robot gRPC endpoint is required")
    if "://" not in raw:
        raw = f"grpc://{raw}"
    parsed = urlparse(raw)
    if parsed.scheme not in {"grpc", "grpcs"} or not parsed.hostname:
        raise RobotApiError(f"invalid robot gRPC endpoint: {value}")
    try:
        port = int(parsed.port or default_port)
    except ValueError as exc:
        raise RobotApiError(f"invalid robot gRPC port: {value}") from exc
    query = parse_qs(parsed.query)
    return RobotEndpoint(
        host=parsed.hostname,
        port=port,
        robot_id=_first(query, "robotId") or _first(query, "robot_id"),
        name=_first(query, "name"),
        secure=parsed.scheme == "grpcs",
    )


def normalize_grpc_endpoint(value: str, *, default_port: int = DEFAULT_GRPC_PORT) -> str:
    return parse_grpc_endpoint(value, default_port=default_port).url


def _first(query: dict[str, list[str]], key: str) -> str:
    values = query.get(key)
    return str(values[0]) if values else ""


def robot_status_from_json(payload: dict[str, Any] | None) -> robot_api_pb2.RobotStatus:
    source = payload if isinstance(payload, dict) else {}
    status = robot_api_pb2.RobotStatus()
    status.robot_id = str(source.get("robotId") or source.get("robot_id") or "")
    status.map_id = str(source.get("mapId") or source.get("map_id") or "")
    status.connected = bool(source.get("connected", True))
    status.localization_ok = bool(source.get("localizationOk") or source.get("localization_ok") or False)
    status.localization_age_sec = _float(source.get("localizationAgeSec") or source.get("localization_age_sec"), 0.0)
    status.state = str(source.get("state") or "")
    status.message = str(source.get("message") or source.get("reason") or "")
    status.target_lm = str(source.get("targetLm") or source.get("target_lm") or "")
    status.nearest_lm = str(
        source.get("nearestLm")
        or source.get("nearest_lm")
        or source.get("currentLm")
        or source.get("currentLM")
        or source.get("currentStation")
        or source.get("current_station")
        or ""
    )
    status.current_edge_id = str(source.get("currentEdgeId") or source.get("current_edge_id") or "")
    status.route_id = str(source.get("routeId") or source.get("route_id") or "")
    status.route_progress = _float(source.get("routeProgress") or source.get("route_progress"), 0.0)

    pose = source.get("pose") if isinstance(source.get("pose"), dict) else source
    if isinstance(pose, dict):
        status.pose_x = _float(pose.get("x") or pose.get("poseX") or source.get("poseX"), 0.0)
        status.pose_y = _float(pose.get("y") or pose.get("poseY") or source.get("poseY"), 0.0)
        status.pose_yaw = _float(pose.get("yaw") or pose.get("theta") or pose.get("angle") or source.get("poseYaw"), 0.0)

    velocity = source.get("velocity") if isinstance(source.get("velocity"), dict) else source
    if isinstance(velocity, dict):
        status.linear_velocity = _float(velocity.get("linear") or velocity.get("linearVelocity") or source.get("linearVelocity"), 0.0)
        status.angular_velocity = _float(velocity.get("angular") or velocity.get("angularVelocity") or source.get("angularVelocity"), 0.0)

    battery = source.get("battery") if isinstance(source.get("battery"), dict) else source
    if isinstance(battery, dict):
        status.battery_level = _float(battery.get("level") or battery.get("batteryLevel") or source.get("batteryLevel"), 0.0)
        status.battery_voltage = _float(battery.get("voltage") or battery.get("batteryVoltage") or source.get("batteryVoltage"), 0.0)
        status.battery_current = _float(battery.get("current") or battery.get("batteryCurrent") or source.get("batteryCurrent"), 0.0)
        status.battery_temperature = _float(
            battery.get("temperature") or battery.get("batteryTemperature") or source.get("batteryTemperature"),
            0.0,
        )
        status.battery_charging = bool(battery.get("charging") or source.get("batteryCharging") or False)

    try:
        status.raw_json = json.dumps(source, ensure_ascii=False, sort_keys=True)
    except (TypeError, ValueError):
        status.raw_json = "{}"
    return status


def robot_status_to_json(status: robot_api_pb2.RobotStatus | None) -> dict[str, Any]:
    if status is None:
        return {}
    raw = {}
    if getattr(status, "raw_json", ""):
        try:
            decoded = json.loads(status.raw_json)
            if isinstance(decoded, dict):
                raw = decoded
        except json.JSONDecodeError:
            raw = {}
    connected = bool(status.connected)
    localization_ok = bool(status.localization_ok)
    pose = {
        "x": float(status.pose_x),
        "y": float(status.pose_y),
        "yaw": float(status.pose_yaw),
    } if connected and localization_ok else None
    payload: dict[str, Any] = {
        **raw,
        "robotId": status.robot_id,
        "mapId": status.map_id,
        "connected": connected,
        "localizationOk": localization_ok,
        "localizationAgeSec": float(status.localization_age_sec),
        "state": status.state,
        "message": status.message,
        "targetLm": status.target_lm,
        "nearestLm": status.nearest_lm,
        "currentEdgeId": status.current_edge_id,
        "routeId": status.route_id,
        "routeProgress": float(status.route_progress),
        "pose": pose,
        "velocity": {
            "linear": float(status.linear_velocity),
            "angular": float(status.angular_velocity),
        },
        "battery": {
            "level": float(status.battery_level),
            "voltage": float(status.battery_voltage),
            "current": float(status.battery_current),
            "temperature": float(status.battery_temperature),
            "charging": bool(status.battery_charging),
        },
    }
    if status.nearest_lm:
        payload.setdefault("currentLm", status.nearest_lm)
    return payload


def json_dumps(payload: dict[str, Any] | None) -> str:
    return json.dumps(payload if isinstance(payload, dict) else {}, ensure_ascii=False, separators=(",", ":"))


def json_loads_object(value: str) -> dict[str, Any]:
    try:
        payload = json.loads(str(value or "{}"))
    except json.JSONDecodeError as exc:
        raise RobotApiError("invalid JSON payload") from exc
    if not isinstance(payload, dict):
        raise RobotApiError("JSON payload must be an object")
    return payload


def _float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
