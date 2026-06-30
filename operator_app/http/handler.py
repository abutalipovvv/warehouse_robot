from __future__ import annotations

import base64
import hashlib
import json
import select
import struct
import time
from http.server import SimpleHTTPRequestHandler
from urllib.parse import parse_qs, unquote, urlparse

from ..config import (
    APP_ROUTES,
    DEFAULT_FLEET_WS_INTERVAL_MS,
    MAX_FLEET_WS_INTERVAL_MS,
    MIN_FLEET_WS_INTERVAL_MS,
    WEBSOCKET_GUID,
)
from ..core.state import OperatorAppState, RobotProbeError, utc_now


class OperatorRequestHandler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    app_state: OperatorAppState | None = None

    def do_GET(self) -> None:
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/ws/fleet-manager":
                self._handle_fleet_manager_ws(parsed)
                return
            if path == "/ws/robot/status":
                self._handle_robot_status_ws(parsed)
                return
            if path == "/health":
                self._send_json({"ok": True})
                return
            if path == "/api/robots":
                self._send_json(self._require_state().list_robots_payload(probe_robots=self._should_probe_robots(parsed)))
                return
            if path == "/api/fleet/params":
                self._send_json(self._require_state().fleet_params_payload())
                return
            if path == "/api/fleet/orders" or path == "/orders":
                self._handle_fleet_manager_get("orders", "")
                return
            fleet_target = self._parse_fleet_manager_api(parsed)
            if fleet_target is not None:
                action, arg = fleet_target
                self._handle_fleet_manager_get(action, arg)
                return
            params_target = self._parse_robot_params_api(parsed)
            if params_target is not None:
                self._send_json(self._require_state().robot_params_payload(params_target))
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

    def _handle_fleet_manager_ws(self, parsed) -> None:
        if not self._is_websocket_upgrade():
            self._send_error_json(400, "expected websocket upgrade")
            return
        key = self.headers.get("Sec-WebSocket-Key", "").strip()
        if not key:
            self._send_error_json(400, "missing Sec-WebSocket-Key")
            return

        accept = base64.b64encode(
            hashlib.sha1(f"{key}{WEBSOCKET_GUID}".encode("ascii")).digest()
        ).decode("ascii")
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True

        state = self._require_state()
        interval_sec = self._fleet_ws_interval_sec(parsed)
        try:
            initial_payload = state.fleet_manager_stream_payload(initial=True)
            if initial_payload is not None:
                self._send_ws_json(initial_payload)
            while True:
                if self._ws_client_closed():
                    return
                time.sleep(interval_sec)
                if self._ws_client_closed():
                    return
                payload = state.fleet_manager_stream_payload(initial=False)
                if payload is not None:
                    self._send_ws_json(payload)
        except (BrokenPipeError, ConnectionError, OSError):
            return
        except Exception as exc:  # pragma: no cover - defensive websocket path
            try:
                self._send_ws_json({"ok": False, "type": "error", "error": str(exc)})
            except (BrokenPipeError, ConnectionError, OSError):
                return

    def _handle_robot_status_ws(self, parsed) -> None:
        if not self._is_websocket_upgrade():
            self._send_error_json(400, "expected websocket upgrade")
            return
        robot_id = str(parse_qs(parsed.query).get("robotId", [""])[0] or "").strip()
        if not robot_id:
            self._send_error_json(400, "robotId is required")
            return
        state = self._require_state()
        try:
            robot = state.get_robot(robot_id)
            if not robot.is_grpc:
                raise ValueError("unsupported robot transport; use grpc")
        except Exception as exc:
            self._send_error_json(404, str(exc))
            return
        key = self.headers.get("Sec-WebSocket-Key", "").strip()
        if not key:
            self._send_error_json(400, "missing Sec-WebSocket-Key")
            return

        accept = base64.b64encode(
            hashlib.sha1(f"{key}{WEBSOCKET_GUID}".encode("ascii")).digest()
        ).decode("ascii")
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True

        interval_sec = self._fleet_ws_interval_sec(parsed)
        try:
            while True:
                if self._ws_client_closed():
                    return
                self._send_ws_json(
                    {
                        "ok": True,
                        "type": "state",
                        "state": state.grpc_adapter.status(state._grpc_endpoint(robot)),
                        "sentAt": utc_now(),
                    }
                )
                time.sleep(interval_sec)
        except (BrokenPipeError, ConnectionError, OSError):
            return
        except Exception as exc:  # pragma: no cover - defensive websocket path
            try:
                self._send_ws_json({"ok": False, "type": "error", "error": str(exc)})
            except (BrokenPipeError, ConnectionError, OSError):
                return

    def _is_websocket_upgrade(self) -> bool:
        upgrade = self.headers.get("Upgrade", "").strip().lower()
        connection = self.headers.get("Connection", "").strip().lower()
        return upgrade == "websocket" and "upgrade" in connection

    def _fleet_ws_interval_sec(self, parsed) -> float:
        raw_interval = parse_qs(parsed.query).get("intervalMs", [str(DEFAULT_FLEET_WS_INTERVAL_MS)])[0]
        try:
            interval_ms = int(raw_interval)
        except (TypeError, ValueError):
            interval_ms = DEFAULT_FLEET_WS_INTERVAL_MS
        interval_ms = max(MIN_FLEET_WS_INTERVAL_MS, min(MAX_FLEET_WS_INTERVAL_MS, interval_ms))
        return interval_ms / 1000.0

    def _send_ws_json(self, payload: object) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.connection.sendall(self._ws_frame(encoded, opcode=0x1))

    @staticmethod
    def _ws_frame(payload: bytes, opcode: int) -> bytes:
        length = len(payload)
        first_byte = 0x80 | opcode
        if length < 126:
            return bytes([first_byte, length]) + payload
        if length <= 0xFFFF:
            return bytes([first_byte, 126]) + struct.pack("!H", length) + payload
        return bytes([first_byte, 127]) + struct.pack("!Q", length) + payload

    def _ws_client_closed(self) -> bool:
        readable, _, _ = select.select([self.connection], [], [], 0)
        if not readable:
            return False
        previous_timeout = self.connection.gettimeout()
        self.connection.settimeout(0.05)
        try:
            header = self._recv_exact(2)
            if not header:
                return True
            opcode = header[0] & 0x0F
            masked = bool(header[1] & 0x80)
            length = header[1] & 0x7F
            if length == 126:
                extended = self._recv_exact(2)
                if not extended:
                    return True
                length = struct.unpack("!H", extended)[0]
            elif length == 127:
                extended = self._recv_exact(8)
                if not extended:
                    return True
                length = struct.unpack("!Q", extended)[0]
            mask = self._recv_exact(4) if masked else b""
            payload = self._recv_exact(length) if length else b""
            if length and not payload:
                return True
            if masked and mask:
                payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
            if opcode == 0x8:
                return True
            if opcode == 0x9:
                self.connection.sendall(self._ws_frame(payload, opcode=0xA))
            return False
        except TimeoutError:
            return False
        except (ConnectionError, OSError):
            return True
        finally:
            self.connection.settimeout(previous_timeout)

    def _recv_exact(self, size: int) -> bytes:
        chunks = []
        remaining = size
        while remaining > 0:
            chunk = self.connection.recv(remaining)
            if not chunk:
                return b""
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

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
                action, arg = fleet_target
                del arg
                self._handle_fleet_manager_post(action)
                return
            params_target = self._parse_robot_params_api(parsed)
            if params_target is not None:
                self._handle_robot_params_post(params_target)
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

    def _parse_fleet_manager_api(self, parsed) -> tuple[str, str] | None:
        path = parsed.path
        if not path.startswith("/api/fleet-manager"):
            return None
        tail = path.removeprefix("/api/fleet-manager").strip("/")
        parts = [item for item in tail.split("/") if item]
        if not parts:
            return "identity", ""
        if parts == ["identity"]:
            return "identity", ""
        if parts == ["status"]:
            return "status", ""
        if parts == ["state"]:
            return "state", ""
        if parts == ["mode"]:
            return "mode", ""
        if parts == ["map"]:
            return "map", ""
        if parts == ["params"]:
            return "params", ""
        if parts == ["orders"]:
            return "orders", ""
        if parts == ["setOrder"] or parts == ["orders", "set"]:
            return "set_order", ""
        if parts == ["orders", "dispatch"]:
            return "orders_dispatch", ""
        if parts == ["orders", "cancel"]:
            return "orders_cancel", ""
        if parts == ["orders", "pause"]:
            return "orders_pause", ""
        if parts == ["orders", "resume"]:
            return "orders_resume", ""
        if parts == ["orders", "clear"]:
            return "orders_clear", ""
        if parts == ["plan"]:
            return "plan", ""
        if parts == ["tick"]:
            return "tick", ""
        if parts == ["world"]:
            return "world", ""
        if parts == ["check"]:
            return "check", ""
        if parts == ["manual-step"]:
            return "manual_step", ""
        if parts == ["manual-stop"]:
            return "manual_stop", ""
        if parts == ["maps", "list"]:
            return "maps_list", ""
        if parts == ["maps", "active"]:
            return "maps_active", ""
        if parts == ["maps", "pull"]:
            map_name = str(parse_qs(parsed.query).get("name", [""])[0] or "").strip()
            return "maps_pull", map_name
        if parts == ["maps", "local"]:
            return "maps_local_list", ""
        if parts == ["maps", "local", "active"]:
            return "maps_local_active", ""
        if parts == ["maps", "local", "save"]:
            return "maps_local_save", ""
        if parts == ["maps", "local", "activate"]:
            return "maps_local_activate", ""
        if len(parts) == 3 and parts[1] == "local":
            return "maps_local_get", unquote(parts[2]).strip()
        if parts == ["maps", "pull-sync"]:
            return "maps_pull_sync", ""
        if parts == ["maps", "push"]:
            return "maps_push", ""
        if parts == ["maps", "push-sync"]:
            return "maps_push_sync", ""
        if parts == ["maps", "load"]:
            return "maps_load", ""
        if parts == ["maps", "save"]:
            return "maps_save", ""
        if parts == ["robots"]:
            return "robots_add", ""
        if parts == ["robots", "remove"]:
            return "robots_remove", ""
        if parts == ["robots", "update"]:
            return "robots_update", ""
        if parts == ["robots", "stop"]:
            return "robots_stop", ""
        if parts == ["robots", "reset"]:
            return "robots_reset", ""
        return None

    def _handle_fleet_manager_get(self, action: str, arg: str) -> None:
        try:
            self._send_json(self._require_state().fleet_manager_get_payload(action, arg))
        except ValueError as exc:
            self._send_error_json(400, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_fleet_manager_post(self, action: str) -> None:
        payload = self._read_json_payload()
        if payload is None:
            return
        if not isinstance(payload, dict):
            self._send_error_json(400, "expected object payload")
            return
        try:
            self._send_json(self._require_state().fleet_manager_post_payload(action, payload))
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
        path = parsed.path
        if not path.startswith("/api/robots/"):
            return None
        remainder = path.removeprefix("/api/robots/")
        robot_part, sep, tail = remainder.partition("/")
        if not sep:
            return None
        robot_id = unquote(robot_part).strip()
        if not robot_id or not tail.startswith("maps"):
            return None
        parts = [item for item in tail.split("/") if item]
        if not parts:
            return None
        if parts == ["maps", "list"]:
            return robot_id, "robot_list", ""
        if parts == ["maps", "active"]:
            return robot_id, "robot_active", ""
        if parts == ["maps", "pull"]:
            map_name = str(parse_qs(parsed.query).get("name", [""])[0] or "").strip()
            return robot_id, "pull", map_name
        if parts == ["maps", "local"]:
            return robot_id, "local_list", ""
        if parts == ["maps", "local", "active"]:
            return robot_id, "local_active", ""
        if parts == ["maps", "local", "save"]:
            return robot_id, "local_save", ""
        if parts == ["maps", "local", "activate"]:
            return robot_id, "local_activate", ""
        if len(parts) == 3 and parts[1] == "local":
            return robot_id, "local_get", unquote(parts[2]).strip()
        if parts == ["maps", "pull-sync"]:
            return robot_id, "pull_sync", ""
        if parts == ["maps", "push"]:
            return robot_id, "push", ""
        if parts == ["maps", "push-sync"]:
            return robot_id, "push_sync", ""
        if parts == ["maps", "load"]:
            return robot_id, "load", ""
        return None

    def _parse_robot_params_api(self, parsed) -> str | None:
        path = parsed.path
        if not path.startswith("/api/robots/"):
            return None
        remainder = path.removeprefix("/api/robots/")
        robot_part, sep, tail = remainder.partition("/")
        if not sep:
            return None
        robot_id = unquote(robot_part).strip()
        if not robot_id or tail.strip("/") != "params":
            return None
        return robot_id

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
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _handle_robot_maps_post(self, robot_id: str, action: str, _arg: str) -> None:
        if action not in {"local_save", "local_activate", "push", "load", "pull", "pull_sync", "push_sync"}:
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
        except RobotProbeError as exc:
            self._send_error_json(502, str(exc))
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

    def _parse_robot_proxy(self, parsed) -> tuple[str, str] | None:
        path = parsed.path
        if not path.startswith("/robots/"):
            return None
        remainder = path.removeprefix("/robots/")
        if not remainder:
            return None
        robot_part, sep, tail = remainder.partition("/")
        robot_id = unquote(robot_part).strip()
        if not robot_id:
            return None
        robot_path = "/" if not sep else f"/{tail}"
        if parsed.query:
            robot_path = f"{robot_path}?{parsed.query}"
        return robot_id, robot_path

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
