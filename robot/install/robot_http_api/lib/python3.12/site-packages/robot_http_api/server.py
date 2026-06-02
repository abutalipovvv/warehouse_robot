from __future__ import annotations

import argparse
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import urlparse
import webbrowser

from route_core import DEFAULT_PARAMS_PATH, load_route_params, save_route_params

from robot_planner import PlannedRobotRoute, Pose2D, RobotRuntime, RobotTrajectoryPlanner

if TYPE_CHECKING:
    from robot_gateway import RobotRosBridgeNode


PROJECT_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_ROBOT_MAP_DIR = PROJECT_ROOT / "map_data" / "maps_out" / "22.05.26_smap.smap"
DEFAULT_STATIC_DIR = Path(__file__).resolve().parent / "static"


class RobotHttpApiBridge:
    def __init__(
        self,
        runtime: RobotRuntime,
        route_planner: RobotTrajectoryPlanner,
        params_path: Path,
        ros_bridge: RobotRosBridgeNode,
    ) -> None:
        self.runtime = runtime
        self.route_planner = route_planner
        self.params_path = params_path
        self.ros_bridge = ros_bridge

    def identity_payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "robotId": self.runtime.robot_id,
            "mapId": self.runtime.map_id,
            "api": "robot_http_api",
            "version": 1,
        }

    def params_payload(self) -> dict[str, Any]:
        return load_route_params(self.params_path, create=True)

    def save_params_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        saved_path = save_route_params(payload, self.params_path)
        params = load_route_params(self.params_path, create=True)
        self.route_planner.update_params(params)
        self.runtime.add_event("info", f"params saved: {saved_path}")
        return {
            "ok": True,
            "path": str(saved_path),
            "params": params,
        }

    def status_payload(self) -> dict[str, Any]:
        snapshot = self.runtime.snapshot()
        return {
            "ok": True,
            "robot": self.ros_bridge.latest_status_payload(),
            "route": snapshot.get("route"),
            "events": snapshot.get("events", []),
        }

    def plan_route_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        route = self._route_from_request(payload)
        self.runtime.add_event("info", f"planned route to {route.goal_lm}")
        return {
            "ok": True,
            "route": route.to_dict(),
        }

    def execute_route_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        route = self._route_from_request(payload)
        self.ros_bridge.execute_route(route)
        return {
            "ok": True,
            "route": route.to_dict(),
            "status": self.status_payload(),
        }

    def cancel_route_payload(self) -> dict[str, Any]:
        self.ros_bridge.cancel_route("Route canceled.")
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def teleop_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        linear = float(payload.get("linear", 0.0) or 0.0)
        angular = float(payload.get("angular", 0.0) or 0.0)
        timeout_ms = max(80, int(payload.get("timeoutMs", 300) or 300))
        self.ros_bridge.set_teleop(linear=linear, angular=angular, timeout_ms=timeout_ms)
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def teleop_stop_payload(self) -> dict[str, Any]:
        self.ros_bridge.release_manual()
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def stop_payload(self) -> dict[str, Any]:
        self.ros_bridge.stop_robot("Stopped.")
        return {
            "ok": True,
            "status": self.status_payload(),
        }

    def _route_from_request(self, payload: dict[str, Any]) -> PlannedRobotRoute:
        route_payload = payload.get("route")
        if isinstance(route_payload, dict):
            route = PlannedRobotRoute.from_dict(route_payload)
            if not route.goal_lm:
                raise ValueError("route.goalLm is required")
            return route

        goal_lm = str(payload.get("goalLm") or payload.get("targetLm") or "").strip()
        if not goal_lm:
            raise ValueError("goalLm is required")

        pose_payload = payload.get("startPose")
        pose = None
        if isinstance(pose_payload, dict):
            pose = Pose2D(
                x=float(pose_payload.get("x", 0.0) or 0.0),
                y=float(pose_payload.get("y", 0.0) or 0.0),
                yaw=float(pose_payload.get("yaw", 0.0) or 0.0),
            )
        if pose is None:
            pose = self._best_available_pose()
        if pose is None:
            raise ValueError("robot pose is not available yet")

        start_lm = str(payload.get("startLm") or "").strip() or None
        return self.ros_bridge.plan_route_via_service(
            pose=pose,
            goal_lm=goal_lm,
            start_lm=start_lm,
        )

    def _best_available_pose(self) -> Pose2D | None:
        pose = self.runtime.latest_pose()
        if pose is not None:
            return pose

        snapshot = self.runtime.snapshot()
        pose_payload = snapshot.get("pose")
        if isinstance(pose_payload, dict):
            return Pose2D(
                x=float(pose_payload.get("x", 0.0) or 0.0),
                y=float(pose_payload.get("y", 0.0) or 0.0),
                yaw=float(pose_payload.get("yaw", 0.0) or 0.0),
            )

        return self.ros_bridge.latest_pose()


class RobotWebRequestHandler(SimpleHTTPRequestHandler):
    bridge: RobotHttpApiBridge | None = None
    site_data_script: str = ""

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/health":
            self._send_json({"ok": True})
            return
        if path == "/demo-data.js":
            self._send_script(self.site_data_script)
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
        super().do_GET()

    def do_POST(self) -> None:
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
        self.send_error(404, "Not found")

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

    def _handle_json(self, callback) -> None:
        try:
            payload = self._read_json_payload()
            if payload is None:
                return
            if not isinstance(payload, dict):
                self.send_error(400, "Expected object")
                return
            self._send_json(callback(payload))
        except ValueError as exc:
            self.send_error(400, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self.send_error(500, str(exc))

    def _read_json_payload(self) -> object | None:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        if not raw_body:
            return {}
        try:
            return json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
            return None

    def _send_json(self, payload: object) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_script(self, content: str) -> None:
        encoded = content.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/javascript; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _require_bridge(self) -> RobotHttpApiBridge:
        if self.bridge is None:
            raise RuntimeError("robot_http_api bridge is not ready")
        return self.bridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve single robot HTTP API and web UI.")
    parser.add_argument("--map-dir", default=DEFAULT_ROBOT_MAP_DIR, type=Path)
    parser.add_argument("--params", default=DEFAULT_PARAMS_PATH, type=Path)
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8790, type=int)
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--amcl-topic", default="/amcl_pose")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--status-topic", default="/robot_status")
    parser.add_argument("--plan-service", default="/route/plan")
    parser.add_argument("--execute-service", default="/route/execute")
    parser.add_argument("--cancel-service", default="/route/cancel")
    parser.add_argument("--teleop-service", default="/robot/teleop/set")
    parser.add_argument("--release-manual-service", default="/robot/teleop/release")
    parser.add_argument("--stop-service", default="/robot/stop")
    return parser.parse_args()


def resolve_map_dir(map_dir: Path) -> Path:
    if map_dir.exists():
        return map_dir
    if map_dir.is_absolute():
        return map_dir

    relocated = PROJECT_ROOT / "map_data" / map_dir
    if relocated.exists():
        return relocated
    return map_dir


def serve_http_server(
    *,
    bridge: RobotHttpApiBridge,
    route_planner: RobotTrajectoryPlanner,
    robot_id: str,
    host: str,
    port: int,
    open_browser: bool,
) -> None:
    site_payload = json.dumps(route_planner.site_payload(robot_id), ensure_ascii=False).replace(
        "</script>",
        "<\\/script>",
    )
    RobotWebRequestHandler.bridge = bridge
    RobotWebRequestHandler.site_data_script = f"window.ROBOT_WEB_DATA = {site_payload};\n"
    handler = partial(RobotWebRequestHandler, directory=str(DEFAULT_STATIC_DIR.resolve()))
    server = ThreadingHTTPServer((host, port), handler)
    url = f"http://{host}:{port}/"
    browser_url = f"http://127.0.0.1:{port}/" if host in {"0.0.0.0", "::"} else url
    print(f"Serving single robot UI: {url}")
    if browser_url != url:
        print(f"Open locally: {browser_url}")
    print(f"Robot id: {robot_id}")
    print(f"Map dir: {route_planner.loaded_map.map_dir}")
    print(f"Params path: {bridge.params_path}")
    if open_browser:
        webbrowser.open(browser_url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()
