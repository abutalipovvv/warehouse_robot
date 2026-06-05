from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlparse
import webbrowser

if TYPE_CHECKING:
    from .client import RobotRosClient


def _project_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "commands.txt").exists() and (parent / "robot").exists():
            return parent
        if (parent / "fleet_manager").exists() and (parent / "robot").exists():
            return parent
    return Path(__file__).resolve().parents[1]


def _maps_root(project_root: Path) -> Path:
    candidates = [
        project_root / "map_data" / "maps_out",
        project_root / "fleet_manager" / "map_data" / "maps_out",
        project_root / "robot" / "ws" / "src" / "robot_map_manager" / "maps_out",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()
    return candidates[0].resolve()


def _params_path(project_root: Path) -> Path:
    candidates = [
        project_root / "params.yaml",
        project_root / "fleet_manager" / "params.yaml",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0].resolve()


PROJECT_ROOT = _project_root()
MAPS_ROOT = _maps_root(PROJECT_ROOT)
from robot_planner.route_core import (
    build_editable_map_bundle_payload,
    build_editable_map_payload,
    list_editable_maps,
    load_route_params,
    save_editable_map,
)

DEFAULT_ROBOT_MAP_DIR = MAPS_ROOT / "22.05.26_smap.smap"
DEFAULT_PARAMS_PATH = _params_path(PROJECT_ROOT)


class RobotHttpApiBridge:
    def __init__(
        self,
        robot_id: str,
        ros_client: RobotRosClient,
    ) -> None:
        self.robot_id = robot_id
        self.ros_client = ros_client

    def identity_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "robotId": self.robot_id,
            "mapId": self.ros_client.map_id,
            "api": "robot_http_api",
            "version": 1,
        }

    def params_payload(self) -> dict[str, Any]:
        return self.ros_client.params_payload()

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.ros_client.save_params_payload(payload)

    def status_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "robot": self.ros_client.latest_status_payload(),
            "route": self.ros_client.active_route_payload(),
            "events": self.ros_client.events_payload(),
        }

    def plan_route_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        route = self._route_from_request(payload)
        self.ros_client.add_event("info", f"planned route to {route.get('goalLm')}")
        return {"ok": True, "route": route}

    def execute_route_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        route = self._route_from_request(payload)
        executed = self.ros_client.execute_route_payload(route)
        return {"ok": True, "route": executed, "status": self.status_payload()}

    def cancel_route_payload(self) -> dict[str, Any]:
        self.ros_client.cancel_route("Route canceled.")
        return {"ok": True, "status": self.status_payload()}

    def teleop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        linear = float(payload.get("linear", 0.0) or 0.0)
        angular = float(payload.get("angular", 0.0) or 0.0)
        timeout_ms = max(80, int(payload.get("timeoutMs", 300) or 300))
        current_status = self.ros_client.latest_status_payload()
        self.ros_client.teleop(linear=linear, angular=angular, timeout_ms=timeout_ms)
        if str(current_status.get("state") or "") != "MANUAL":
            self.ros_client.add_event("info", "manual control engaged")
        return {"ok": True, "status": self.status_payload()}

    def teleop_stop_payload(self) -> dict[str, Any]:
        self.ros_client.teleop_stop()
        self.ros_client.add_event("info", "manual control released")
        return {"ok": True, "status": self.status_payload()}

    def stop_payload(self) -> dict[str, Any]:
        self.ros_client.stop()
        self.ros_client.add_event("warn", "robot stopped")
        return {"ok": True, "status": self.status_payload()}

    def maps_list_payload(self) -> dict[str, Any]:
        active = self.ros_client.active_map_payload()
        active_map_dir = Path(str(active.get("mapDir") or "")).resolve() if active.get("mapDir") else None
        maps_root = active_map_dir.parent if active_map_dir is not None else self.ros_client.route_planner.map_dir.parent
        return list_editable_maps(maps_root, active_map_dir=active_map_dir)

    def maps_active_payload(self) -> dict[str, Any]:
        return self.ros_client.active_map_payload()

    def pull_map_payload(self, map_name: str) -> dict[str, Any]:
        active = self.ros_client.active_map_payload()
        active_map_dir = Path(str(active.get("mapDir") or "")).resolve()
        target = self._resolve_map_dir(active_map_dir, map_name)
        params = load_route_params(self.ros_client.params_path, create=True)
        return build_editable_map_bundle_payload(target, params=params)

    def push_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        active = self.ros_client.active_map_payload()
        active_map_dir = Path(str(active.get("mapDir") or "")).resolve()
        target_name = str(payload.get("mapName") or "").strip()
        source_name = str(payload.get("sourceMapName") or target_name).strip()
        source_dir = self._resolve_map_dir(active_map_dir, source_name)
        output_name = str(payload.get("outputName") or "").strip()
        if not output_name and target_name and source_name and target_name != source_name:
            output_name = target_name
        overwrite_output = bool(payload.get("overwriteOutput", False))
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        loaded_map = save_editable_map(
            source_dir,
            editable_map,
            output_name=output_name,
            overwrite_output=overwrite_output,
        )
        params = load_route_params(self.ros_client.params_path, create=True)
        return {
            **build_editable_map_payload(loaded_map.map_dir, params=params),
            "savedAs": bool(output_name),
        }

    def load_map_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        return self.ros_client.load_map(map_name)

    def _resolve_map_dir(self, active_map_dir: Path, map_name: str) -> Path:
        if not map_name:
            return active_map_dir
        safe_name = Path(map_name).name
        if not safe_name.endswith(".smap"):
            safe_name = f"{safe_name}.smap"
        candidate = (active_map_dir.parent / safe_name).resolve()
        if active_map_dir.parent not in candidate.parents and candidate != active_map_dir.parent:
            raise ValueError("map must stay inside maps_root")
        if not candidate.is_dir():
            raise ValueError(f"map not found: {safe_name}")
        return candidate

    def _route_from_request(self, payload: dict[str, Any]) -> dict[str, Any]:
        route_payload = payload.get("route")
        if isinstance(route_payload, dict):
            goal_lm = str(route_payload.get("goalLm") or route_payload.get("goal_lm") or "").strip()
            if not goal_lm:
                raise ValueError("route.goalLm is required")
            return route_payload

        goal_lm = str(payload.get("goalLm") or payload.get("targetLm") or "").strip()
        if not goal_lm:
            raise ValueError("goalLm is required")

        pose_payload = payload.get("startPose")
        pose = None
        if isinstance(pose_payload, dict):
            pose = {
                "x": float(pose_payload.get("x", 0.0) or 0.0),
                "y": float(pose_payload.get("y", 0.0) or 0.0),
                "yaw": float(pose_payload.get("yaw", 0.0) or 0.0),
            }
        if pose is None:
            pose = self._best_available_pose()
        if pose is None:
            raise ValueError("robot pose is not available yet")
        start_lm = str(payload.get("startLm") or "").strip() or None
        return self.ros_client.plan_route_payload(pose=pose, goal_lm=goal_lm, start_lm=start_lm)

    def _best_available_pose(self) -> dict[str, float] | None:
        pose = self.ros_client.latest_pose()
        if pose is None:
            return None
        return {"x": float(pose.x), "y": float(pose.y), "yaw": float(pose.yaw)}


class RobotApiRequestHandler(BaseHTTPRequestHandler):
    bridge: RobotHttpApiBridge | None = None

    def do_GET(self) -> None:
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/":
                self._send_json(self._root_payload())
                return
            if path == "/health":
                self._send_json({"ok": True})
                return
            if path == "/api/robot/identity":
                self._send_json(self._require_bridge().identity_payload())
                return
            if path == "/api/robot/status":
                self._send_json(self._require_bridge().status_payload())
                return
            if path == "/api/params":
                self._send_json(self._require_bridge().params_payload())
                return
            if path == "/api/maps/list":
                self._send_json(self._require_bridge().maps_list_payload())
                return
            if path == "/api/maps/active":
                self._send_json(self._require_bridge().maps_active_payload())
                return
            if path == "/api/maps/pull":
                map_name = str(parse_qs(parsed.query).get("name", [""])[0] or "").strip()
                self._send_json(self._require_bridge().pull_map_payload(map_name))
                return
            self._send_error_json(404, "Not found. Robot UI is served by operator_app; this service exposes API only.")
        except BrokenPipeError:
            return
        except ConnectionResetError:
            return
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def do_POST(self) -> None:
        try:
            path = urlparse(self.path).path
            if path == "/api/params":
                self._handle_json(self._handle_save_params)
                return
            if path == "/api/robot/teleop":
                self._handle_json(self._handle_teleop)
                return
            if path == "/api/robot/teleop/stop":
                self._handle_json(self._handle_teleop_stop)
                return
            if path == "/api/robot/route/plan":
                self._handle_json(self._handle_plan_route)
                return
            if path == "/api/robot/route/execute":
                self._handle_json(self._handle_execute_route)
                return
            if path == "/api/robot/route/cancel":
                self._handle_json(self._handle_cancel_route)
                return
            if path == "/api/robot/stop":
                self._handle_json(self._handle_stop)
                return
            if path == "/api/maps/push":
                self._handle_json(self._handle_push_map)
                return
            if path == "/api/maps/load":
                self._handle_json(self._handle_load_map)
                return
            self._send_error_json(404, "Not found")
        except BrokenPipeError:
            return
        except ConnectionResetError:
            return
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_save_params(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().save_params_payload(payload)

    def _handle_teleop(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().teleop_payload(payload)

    def _handle_teleop_stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        del payload
        return self._require_bridge().teleop_stop_payload()

    def _handle_plan_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().plan_route_payload(payload)

    def _handle_execute_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().execute_route_payload(payload)

    def _handle_cancel_route(self, payload: dict[str, Any]) -> dict[str, Any]:
        del payload
        return self._require_bridge().cancel_route_payload()

    def _handle_stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        del payload
        return self._require_bridge().stop_payload()

    def _handle_push_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().push_map_payload(payload)

    def _handle_load_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._require_bridge().load_map_payload(payload)

    def _root_payload(self) -> dict[str, Any]:
        identity = self._require_bridge().identity_payload()
        return {
            "ok": True,
            "service": "robot_http_api",
            "ui": "operator_app",
            "message": "This robot exposes API only. Open the operator app and connect to this robot by IP.",
            "identity": identity,
            "endpoints": [
                "/health",
                "/api/robot/identity",
                "/api/robot/status",
                "/api/robot/route/plan",
                "/api/robot/route/execute",
                "/api/robot/route/cancel",
                "/api/robot/teleop",
                "/api/robot/teleop/stop",
                "/api/robot/stop",
                "/api/maps/list",
                "/api/maps/active",
                "/api/maps/pull",
                "/api/maps/push",
                "/api/maps/load",
                "/api/params",
            ],
        }

    def _handle_json(self, callback) -> None:
        try:
            payload = self._read_json_payload()
            if payload is None:
                return
            if not isinstance(payload, dict):
                self._send_error_json(400, "Expected object")
                return
            self._send_json(callback(payload))
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover
            self._send_error_json(500, str(exc))

    def _read_json_payload(self) -> object | None:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        if not raw_body:
            return {}
        try:
            return json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self._send_error_json(400, "Invalid JSON")
            return None

    def _send_json(self, payload: object, status: int = 200) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        try:
            self.wfile.write(encoded)
        except (BrokenPipeError, ConnectionResetError):
            return

    def _send_error_json(self, status: int, message: str) -> None:
        self._send_json({"ok": False, "error": str(message)}, status=status)

    def _require_bridge(self) -> RobotHttpApiBridge:
        if self.bridge is None:
            raise RuntimeError("robot_http_api bridge is not ready")
        return self.bridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve single robot HTTP API for the operator app.")
    parser.add_argument("--map-dir", default=DEFAULT_ROBOT_MAP_DIR, type=Path)
    parser.add_argument("--params", default=DEFAULT_PARAMS_PATH, type=Path)
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8790, type=int)
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--status-topic", default="/robot_status")
    parser.add_argument("--plan-service", default="/route/plan")
    parser.add_argument("--execute-service", default="/route/execute")
    parser.add_argument("--cancel-service", default="/route/cancel")
    parser.add_argument("--map-state-service", default="/robot/maps/state")
    parser.add_argument("--map-load-service", default="/robot/maps/load")
    args, _unknown = parser.parse_known_args()
    return args


def resolve_map_dir(map_dir: Path) -> Path:
    if map_dir.exists():
        return map_dir
    if map_dir.is_absolute():
        return map_dir
    safe_name = Path(map_dir).name
    candidates = [
        PROJECT_ROOT / map_dir,
        MAPS_ROOT / safe_name,
        PROJECT_ROOT / "map_data" / map_dir,
        PROJECT_ROOT / "fleet_manager" / "map_data" / map_dir,
        PROJECT_ROOT / "robot" / "ws" / "src" / "robot_map_manager" / map_dir,
    ]
    if not safe_name.endswith(".smap"):
        candidates.append(MAPS_ROOT / f"{safe_name}.smap")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return map_dir


def serve_http_server(
    *,
    bridge: RobotHttpApiBridge,
    robot_id: str,
    host: str,
    port: int,
    open_browser: bool,
) -> None:
    RobotApiRequestHandler.bridge = bridge
    server = ThreadingHTTPServer((host, port), RobotApiRequestHandler)
    url = f"http://{host}:{port}/"
    browser_url = f"http://127.0.0.1:{port}/" if host in {"0.0.0.0", "::"} else url
    print(f"Serving single robot API: {url}")
    if browser_url != url:
        print(f"Open locally: {browser_url}")
    print("UI: use serve_operator.py and connect to this robot by IP.")
    print(f"Robot id: {robot_id}")
    print(f"Map dir: {bridge.ros_client.route_planner.loaded_map.map_dir}")
    print(f"Params path: {bridge.ros_client.params_path}")
    if open_browser:
        webbrowser.open(browser_url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()
