from __future__ import annotations

import json
from http.server import SimpleHTTPRequestHandler
from urllib.parse import parse_qs, unquote, urlparse

from ..core.config import APP_ROUTES
from ..core.state import OperatorAppState
from ..core.state_common import RobotProbeError
from ..core.fleet_manager import FLEET_MANAGER_ID, FLEET_MANAGER_SIM_ID
from fleet_manager.runtime.grpc.api.client import GrpcRobotError
from .routes import (
    parse_fleet_route,
    parse_robot_map_route,
    parse_robot_params_route,
    parse_robot_proxy_route,
    parse_robot_slam_route,
)
from .socket_handlers import OperatorWebSocketHandlerMixin


class OperatorRequestHandler(
    OperatorWebSocketHandlerMixin,
    SimpleHTTPRequestHandler,
):
    protocol_version = "HTTP/1.1"
    app_state: OperatorAppState | None = None

    def end_headers(self) -> None:
        static_path = urlparse(self.path).path
        no_store_paths = {
            "/",
            "/index.html",
            "/app.js",
            "/scene3d.js",
            "/occupancy-walls.js",
            "/occupancy-wall-worker.js",
            "/styles.css",
            "/map-editor.html",
            "/map-editor.js",
            "/map-editor.css",
        }
        if (
            static_path in no_store_paths
            or static_path in APP_ROUTES
            or static_path.startswith("/js/")
        ):
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.send_header("Pragma", "no-cache")
        elif static_path.startswith("/vendor/"):
            self.send_header("Cache-Control", "public, max-age=31536000, immutable")
        super().end_headers()

    def do_GET(self) -> None:
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/ws/fleet-manager":
                self._handle_fleet_manager_ws(parsed, FLEET_MANAGER_ID)
                return
            if path == "/ws/fleet-manager-sim":
                self._handle_fleet_manager_ws(parsed, FLEET_MANAGER_SIM_ID)
                return
            if path == "/ws/robot/status":
                self._handle_robot_status_ws(parsed)
                return
            if path == "/ws/robot/scan":
                self._handle_robot_scan_ws(parsed)
                return
            if path == "/ws/robot/slam":
                self._handle_robot_slam_ws(parsed)
                return
            if path == "/ws/robot/teleop":
                self._handle_robot_teleop_ws(parsed)
                return
            if path == "/health":
                self._send_json({"ok": True})
                return
            if path == "/api/robots/ping":
                self._send_json(self._require_state().robot_pings_payload())
                return
            if path == "/api/robots":
                self._send_json(
                    self._require_state().list_robots_payload(
                        probe_robots=self._should_probe_robots(parsed),
                    )
                )
                return
            if path == "/api/fleet/params":
                self._send_json(self._require_state().fleet_params_payload())
                return
            if path == "/api/fleet/orders" or path == "/orders":
                self._handle_fleet_manager_get("orders", "")
                return
            fleet_target = self._parse_fleet_manager_api(parsed)
            if fleet_target is not None:
                manager_id, action, arg = fleet_target
                self._handle_fleet_manager_get(action, arg, manager_id)
                return
            params_target = self._parse_robot_params_api(parsed)
            if params_target is not None:
                self._send_json(self._require_state().robot_params_payload(params_target))
                return
            slam_target = self._parse_robot_slam_api(parsed)
            if slam_target is not None:
                robot_id, action = slam_target
                self._handle_robot_slam_get(robot_id, action)
                return
            map_target = self._parse_robot_maps_api(parsed)
            if map_target is not None:
                robot_id, action, arg = map_target
                self._handle_robot_maps_get(robot_id, action, arg)
                return
            proxy_target = self._parse_robot_proxy(parsed)
            if proxy_target is not None:
                self._proxy_robot_request("GET", *proxy_target)
                return
            if path in APP_ROUTES:
                self.path = "/index.html"
            super().do_GET()
        except BrokenPipeError:
            return
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _should_probe_robots(self, parsed) -> bool:
        raw = str(parse_qs(parsed.query).get("probe", ["1"])[0] or "1").strip().lower()
        return raw not in {"0", "false", "no", "off"}


    def do_POST(self) -> None:
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/api/robots/probe":
                self._handle_json(self._require_state().probe_payload)
                return
            if path == "/api/robots":
                self._handle_json(self._require_state().add_robot_payload)
                return
            if path == "/api/fleet/params":
                self._handle_json(self._require_state().save_fleet_params_payload)
                return
            if path == "/setOrder" or path == "/api/fleet/setOrder":
                self._handle_fleet_manager_post("set_order")
                return
            if path == "/api/fleet/orders/dispatch":
                self._handle_fleet_manager_post("orders_dispatch")
                return
            if path == "/api/fleet/orders/clear":
                self._handle_fleet_manager_post("orders_clear")
                return
            if path == "/api/fleet/orders/cancel":
                self._handle_fleet_manager_post("orders_cancel")
                return
            if path == "/api/fleet/orders/pause":
                self._handle_fleet_manager_post("orders_pause")
                return
            if path == "/api/fleet/orders/resume":
                self._handle_fleet_manager_post("orders_resume")
                return
            fleet_target = self._parse_fleet_manager_api(parsed)
            if fleet_target is not None:
                manager_id, action, arg = fleet_target
                del arg
                self._handle_fleet_manager_post(action, manager_id)
                return
            params_target = self._parse_robot_params_api(parsed)
            if params_target is not None:
                self._handle_robot_params_post(params_target)
                return
            slam_target = self._parse_robot_slam_api(parsed)
            if slam_target is not None:
                robot_id, action = slam_target
                self._handle_robot_slam_post(robot_id, action)
                return
            map_target = self._parse_robot_maps_api(parsed)
            if map_target is not None:
                robot_id, action, arg = map_target
                self._handle_robot_maps_post(robot_id, action, arg)
                return
            proxy_target = self._parse_robot_proxy(parsed)
            if proxy_target is not None:
                self._proxy_robot_request("POST", *proxy_target)
                return
            self._send_error_json(404, "not found")
        except BrokenPipeError:
            return
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def do_DELETE(self) -> None:
        path = urlparse(self.path).path
        if path.startswith("/api/robots/"):
            robot_id = unquote(path.removeprefix("/api/robots/")).strip()
            if not robot_id:
                self._send_error_json(400, "robot id is required")
                return
            try:
                self._send_json(self._require_state().delete_robot_payload(robot_id))
            except ValueError as exc:
                self._send_error_json(404, str(exc))
            return
        self._send_error_json(404, "not found")

    def _parse_fleet_manager_api(self, parsed) -> tuple[str, str, str] | None:
        route = parse_fleet_route(parsed)
        return tuple(route) if route is not None else None

    def _handle_fleet_manager_get(
        self,
        action: str,
        arg: str,
        manager_id: str = FLEET_MANAGER_ID,
    ) -> None:
        try:
            self._send_json(
                self._require_state().fleet_manager_get_payload(
                    action,
                    arg,
                    manager_id=manager_id,
                )
            )
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_fleet_manager_post(self, action: str, manager_id: str = FLEET_MANAGER_ID) -> None:
        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self._send_error_json(400, "expected object payload")
            return
        try:
            self._send_json(
                self._require_state().fleet_manager_post_payload(
                    action,
                    payload,
                    manager_id=manager_id,
                )
            )
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _proxy_robot_request(self, method: str, robot_id: str, robot_path: str) -> None:
        request_headers = {}
        content_type = self.headers.get("Content-Type", "").strip()
        if content_type:
            request_headers["Content-Type"] = content_type
        accept = self.headers.get("Accept", "").strip()
        if accept:
            request_headers["Accept"] = accept
        content_length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(content_length) if content_length > 0 else None
        try:
            status, response_headers, response_body = self._require_state().proxy_request(
                robot_id,
                method,
                robot_path,
                headers=request_headers,
                body=body,
            )
        except ValueError as exc:
            self._send_error_json(404, str(exc))
            return
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
            return

        content_type = str(response_headers.get("Content-Type") or "application/octet-stream")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(response_body)))
        self.end_headers()
        self.wfile.write(response_body)

    def _parse_robot_maps_api(self, parsed) -> tuple[str, str, str] | None:
        route = parse_robot_map_route(parsed)
        return tuple(route) if route is not None else None

    def _parse_robot_params_api(self, parsed) -> str | None:
        return parse_robot_params_route(parsed)

    def _parse_robot_slam_api(self, parsed) -> tuple[str, str] | None:
        return parse_robot_slam_route(parsed)

    def _handle_robot_params_post(self, robot_id: str) -> None:
        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self._send_error_json(400, "expected object payload")
            return
        try:
            self._send_json(self._require_state().save_robot_params_payload(robot_id, payload))
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except GrpcRobotError as exc:
            self._send_error_json(502, str(exc))
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_robot_slam_get(self, robot_id: str, action: str) -> None:
        try:
            state = self._require_state()
            if action == "defaults":
                self._send_json(state.robot_slam_defaults_payload(robot_id))
                return
            if action == "state":
                self._send_json(state.robot_slam_state_payload(robot_id))
                return
            self._send_error_json(404, "not found")
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except GrpcRobotError as exc:
            self._send_error_json(502, str(exc))
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_robot_slam_post(self, robot_id: str, action: str) -> None:
        if action not in {"start", "finish", "cancel"}:
            self._send_error_json(404, "not found")
            return
        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self._send_error_json(400, "expected object payload")
            return
        try:
            state = self._require_state()
            if action == "start":
                self._send_json(state.start_robot_slam_payload(robot_id, payload))
                return
            if action == "finish":
                self._send_json(state.finish_robot_slam_payload(robot_id, payload))
                return
            if action == "cancel":
                self._send_json(state.cancel_robot_slam_payload(robot_id, payload))
                return
            self._send_error_json(404, "not found")
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except GrpcRobotError as exc:
            self._send_error_json(502, str(exc))
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_robot_maps_get(self, robot_id: str, action: str, arg: str) -> None:
        try:
            state = self._require_state()
            if action == "robot_list":
                self._send_json(state.robot_maps_list_payload(robot_id))
                return
            if action == "robot_active":
                self._send_json(state.robot_maps_active_payload(robot_id))
                return
            if action == "pull":
                self._send_json(state.pull_robot_map_payload(robot_id, {"mapName": arg}))
                return
            if action == "local_list":
                self._send_json(state.local_maps_payload(robot_id))
                return
            if action == "local_active":
                self._send_json(state.local_active_map_payload(robot_id))
                return
            if action == "local_get":
                self._send_json(state.local_map_payload(robot_id, arg))
                return
            self._send_error_json(404, "not found")
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except GrpcRobotError as exc:
            self._send_error_json(502, str(exc))
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_robot_maps_post(self, robot_id: str, action: str, _arg: str) -> None:
        if action not in {
            "local_save",
            "local_activate",
            "push",
            "load",
            "pull",
            "pull_sync",
            "push_sync",
        }:
            self._send_error_json(404, "not found")
            return
        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self._send_error_json(400, "expected object payload")
            return
        try:
            state = self._require_state()
            if action == "local_save":
                self._send_json(state.save_local_map_payload(robot_id, payload))
                return
            if action == "local_activate":
                self._send_json(state.activate_local_map_payload(robot_id, payload))
                return
            if action == "push":
                self._send_json(state.push_robot_map_payload(robot_id, payload))
                return
            if action == "pull_sync":
                self._send_json(state.pull_sync_payload(robot_id))
                return
            if action == "push_sync":
                self._send_json(state.push_sync_payload(robot_id))
                return
            if action == "load":
                self._send_json(state.load_robot_map_payload(robot_id, payload))
                return
            if action == "pull":
                self._send_json(state.pull_robot_map_payload(robot_id, payload))
                return
            self._send_error_json(404, "not found")
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except GrpcRobotError as exc:
            self._send_error_json(502, str(exc))
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _parse_robot_proxy(self, parsed) -> tuple[str, str] | None:
        return parse_robot_proxy_route(parsed)

    def _handle_json(self, callback) -> None:
        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self._send_error_json(400, "expected object payload")
            return
        try:
            self._send_json(callback(payload))
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _read_json_payload(self) -> object | None:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        if not raw_body:
            return {}
        try:
            return json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self._send_error_json(400, "invalid JSON")
            return None

    def _send_json(self, payload: object, status: int = 200) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_error_json(self, status: int, message: str) -> None:
        self._send_json({"ok": False, "error": message}, status=status)

    def _require_state(self) -> OperatorAppState:
        if self.app_state is None:
            raise RuntimeError("operator app is not ready")
        return self.app_state
