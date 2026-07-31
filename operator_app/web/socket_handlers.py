"""WebSocket sessions exposed by the Operator HTTP server."""

from __future__ import annotations

import queue
import threading
import time
from urllib.parse import parse_qs

from ..core.config import (
    DEFAULT_FLEET_WS_INTERVAL_MS,
    MAX_FLEET_WS_INTERVAL_MS,
    MIN_FLEET_WS_INTERVAL_MS,
)
from ..core.fleet_manager import FLEET_MANAGER_ID
from ..core.state import utc_now
from .websocket import (
    WebSocketConnection,
    encode_frame,
    is_websocket_upgrade,
    teleop_command,
    websocket_accept_value,
)


class OperatorWebSocketHandlerMixin:
    """Own complete WebSocket sessions while the HTTP handler owns routing.

    The host class supplies ``headers``, ``connection`` and the normal
    ``BaseHTTPRequestHandler`` response methods.  Keeping the session loops in
    one capability makes their lifecycle and error handling visible without
    burying the REST API dispatcher.
    """

    def _handle_fleet_manager_ws(
        self,
        parsed,
        manager_id: str = FLEET_MANAGER_ID,
    ) -> None:
        if not self._is_websocket_upgrade():
            self._send_error_json(400, "expected websocket upgrade")
            return
        key = self.headers.get("Sec-WebSocket-Key", "").strip()
        if not key:
            self._send_error_json(400, "missing Sec-WebSocket-Key")
            return

        accept = websocket_accept_value(key)
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True

        state = self._require_state()
        interval_sec = self._fleet_ws_interval_sec(parsed)
        route_revisions: dict[str, int] = {}
        runtime_details_interval_sec = 0.20
        next_runtime_details_at = (
            time.monotonic() + runtime_details_interval_sec
        )
        try:
            initial_payload = state.fleet_manager_stream_payload(
                initial=True,
                manager_id=manager_id,
            )
            if initial_payload is not None:
                self._send_ws_json(initial_payload)
                self._update_stream_route_revisions(
                    route_revisions,
                    initial_payload,
                )
            next_tick_at = time.monotonic() + interval_sec
            while True:
                if self._ws_client_closed():
                    return
                delay = next_tick_at - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                if self._ws_client_closed():
                    return
                payload = state.fleet_manager_stream_payload(
                    initial=False,
                    manager_id=manager_id,
                    route_revisions=route_revisions,
                    include_runtime_details=(
                        time.monotonic() >= next_runtime_details_at
                    ),
                )
                if payload is not None:
                    self._send_ws_json(payload)
                    self._update_stream_route_revisions(
                        route_revisions,
                        payload,
                    )
                    if "orders" in payload.get("state", {}):
                        next_runtime_details_at = (
                            time.monotonic()
                            + runtime_details_interval_sec
                        )
                next_tick_at += interval_sec
                now = time.monotonic()
                while next_tick_at <= now:
                    next_tick_at += interval_sec
        except (BrokenPipeError, ConnectionError, OSError):
            return
        except Exception as exc:  # pragma: no cover - defensive websocket path
            try:
                self._send_ws_json(
                    {
                        "ok": False,
                        "type": "error",
                        "error": str(exc),
                    }
                )
            except (BrokenPipeError, ConnectionError, OSError):
                return

    @staticmethod
    def _update_stream_route_revisions(
        route_revisions: dict[str, int],
        payload: dict[str, object],
    ) -> None:
        state = payload.get("state")
        if not isinstance(state, dict):
            return
        robots = state.get("robots")
        if not isinstance(robots, list):
            return
        current_names: set[str] = set()
        for robot in robots:
            if not isinstance(robot, dict):
                continue
            name = str(robot.get("name") or "")
            if not name:
                continue
            current_names.add(name)
            try:
                route_revisions[name] = int(
                    robot.get("routeRevision", 0) or 0
                )
            except (TypeError, ValueError):
                route_revisions[name] = 0
        for name in list(route_revisions):
            if name not in current_names:
                route_revisions.pop(name, None)

    def _handle_robot_status_ws(self, parsed) -> None:
        if not self._is_websocket_upgrade():
            self._send_error_json(400, "expected websocket upgrade")
            return
        robot_id = self._robot_id_from_query(parsed)
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

        accept = websocket_accept_value(key)
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
                        "state": state.grpc_adapter.status(
                            state._grpc_endpoint(robot)
                        ),
                        "sentAt": utc_now(),
                    }
                )
                time.sleep(interval_sec)
        except (BrokenPipeError, ConnectionError, OSError):
            return
        except Exception as exc:  # pragma: no cover - defensive websocket path
            try:
                self._send_ws_json(
                    {
                        "ok": False,
                        "type": "error",
                        "error": str(exc),
                    }
                )
            except (BrokenPipeError, ConnectionError, OSError):
                return

    def _handle_robot_scan_ws(self, parsed) -> None:
        robot_id = self._robot_id_from_query(parsed)
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
        if not self._accept_websocket():
            return

        query = parse_qs(parsed.query)
        topic = str(query.get("topic", ["/scan"])[0] or "/scan")
        hz = self._float_query_value(query, "hz", 1.0)
        include_intensities = self._boolean_query_value(
            query,
            "includeIntensities",
            default=False,
        )

        try:
            for frame in state.watch_robot_laser_scan(
                robot_id,
                topic=topic,
                hz=hz,
                include_intensities=include_intensities,
            ):
                if self._ws_client_closed():
                    return
                payload = dict(frame)
                payload["type"] = "scan"
                payload["robotId"] = robot_id
                payload["sentAt"] = utc_now()
                self._send_ws_json(payload)
        except (BrokenPipeError, ConnectionError, OSError):
            return
        except Exception as exc:  # pragma: no cover - defensive websocket path
            try:
                self._send_ws_json(
                    {
                        "ok": False,
                        "type": "error",
                        "error": str(exc),
                    }
                )
            except (BrokenPipeError, ConnectionError, OSError):
                return

    def _handle_robot_slam_ws(self, parsed) -> None:
        robot_id = self._robot_id_from_query(parsed)
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
        if not self._accept_websocket():
            return

        query = parse_qs(parsed.query)
        hz = self._float_query_value(query, "hz", 1.0)
        include_cells = self._boolean_query_value(
            query,
            "includeCells",
            default=True,
        )

        try:
            for frame in state.watch_robot_slam_map(
                robot_id,
                hz=hz,
                include_cells=include_cells,
            ):
                if self._ws_client_closed():
                    return
                payload = dict(frame)
                payload["type"] = "slamMap"
                payload["robotId"] = robot_id
                payload["sentAt"] = utc_now()
                self._send_ws_json(payload)
        except (BrokenPipeError, ConnectionError, OSError):
            return
        except Exception as exc:  # pragma: no cover - defensive websocket path
            try:
                self._send_ws_json(
                    {
                        "ok": False,
                        "type": "error",
                        "error": str(exc),
                    }
                )
            except (BrokenPipeError, ConnectionError, OSError):
                return

    def _handle_robot_teleop_ws(self, parsed) -> None:
        robot_id = self._robot_id_from_query(parsed)
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
        if not self._accept_websocket():
            return

        commands: queue.Queue[object] = queue.Queue(maxsize=4)
        responses: queue.Queue[dict[str, object]] = queue.Queue()
        stop_marker = object()

        def put_latest(item: object) -> None:
            while True:
                try:
                    commands.put_nowait(item)
                    return
                except queue.Full:
                    try:
                        commands.get_nowait()
                    except queue.Empty:
                        return

        def command_iter():
            while True:
                item = commands.get()
                if item is stop_marker:
                    return
                yield item

        def grpc_worker() -> None:
            try:
                for response in state.robot_teleop_stream(
                    robot_id,
                    command_iter(),
                ):
                    responses.put(
                        {
                            "ok": True,
                            "type": "teleopAck",
                            "response": response,
                            "sentAt": utc_now(),
                        }
                    )
            except Exception as exc:
                responses.put(
                    {
                        "ok": False,
                        "type": "error",
                        "error": str(exc),
                        "sentAt": utc_now(),
                    }
                )

        worker = threading.Thread(
            target=grpc_worker,
            name=f"operator-robot-teleop-{robot_id}",
            daemon=True,
        )
        worker.start()

        try:
            while True:
                message = self._read_ws_json(timeout_sec=0.05)
                if isinstance(message, dict):
                    if message.get("__closed"):
                        break
                    command = self._teleop_command_from_ws(message)
                    if command is not None:
                        put_latest(command)
                while True:
                    try:
                        response = responses.get_nowait()
                    except queue.Empty:
                        break
                    self._send_ws_json(response)
                    if (
                        response.get("ok") is False
                        and response.get("type") == "error"
                    ):
                        return
        except (BrokenPipeError, ConnectionError, OSError):
            return
        finally:
            while True:
                try:
                    commands.get_nowait()
                except queue.Empty:
                    break
            put_latest(
                {
                    "linear": 0.0,
                    "angular": 0.0,
                    "timeoutMs": 80,
                }
            )
            put_latest(stop_marker)
            worker.join(timeout=0.5)

    def _accept_websocket(self) -> bool:
        if not self._is_websocket_upgrade():
            self._send_error_json(400, "expected websocket upgrade")
            return False
        key = self.headers.get("Sec-WebSocket-Key", "").strip()
        if not key:
            self._send_error_json(400, "missing Sec-WebSocket-Key")
            return False
        accept = websocket_accept_value(key)
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True
        return True

    def _is_websocket_upgrade(self) -> bool:
        return is_websocket_upgrade(self.headers)

    def _fleet_ws_interval_sec(self, parsed) -> float:
        raw_interval = parse_qs(parsed.query).get(
            "intervalMs",
            [str(DEFAULT_FLEET_WS_INTERVAL_MS)],
        )[0]
        try:
            interval_ms = int(raw_interval)
        except (TypeError, ValueError):
            interval_ms = DEFAULT_FLEET_WS_INTERVAL_MS
        interval_ms = max(
            MIN_FLEET_WS_INTERVAL_MS,
            min(MAX_FLEET_WS_INTERVAL_MS, interval_ms),
        )
        return interval_ms / 1000.0

    def _send_ws_json(self, payload: object) -> None:
        WebSocketConnection(self.connection).send_json(payload)

    @staticmethod
    def _ws_frame(payload: bytes, opcode: int) -> bytes:
        return encode_frame(payload, opcode)

    def _ws_client_closed(self) -> bool:
        return WebSocketConnection(self.connection).client_closed()

    def _read_ws_json(
        self,
        *,
        timeout_sec: float = 0.0,
    ) -> dict[str, object] | None:
        return WebSocketConnection(self.connection).read_json(
            timeout_sec=timeout_sec,
        )

    def _read_ws_frame(
        self,
        *,
        timeout_sec: float = 0.0,
    ) -> tuple[int, bytes] | None:
        return WebSocketConnection(self.connection).read_frame(
            timeout_sec=timeout_sec,
        )

    @staticmethod
    def _teleop_command_from_ws(
        message: dict[str, object],
    ) -> dict[str, object] | None:
        return teleop_command(message)

    def _recv_exact(self, size: int) -> bytes:
        return WebSocketConnection(self.connection).recv_exact(size)

    @staticmethod
    def _robot_id_from_query(parsed) -> str:
        return str(
            parse_qs(parsed.query).get("robotId", [""])[0] or ""
        ).strip()

    @staticmethod
    def _float_query_value(
        query: dict[str, list[str]],
        name: str,
        default: float,
    ) -> float:
        try:
            return float(query.get(name, [str(default)])[0] or default)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _boolean_query_value(
        query: dict[str, list[str]],
        name: str,
        *,
        default: bool,
    ) -> bool:
        fallback = "1" if default else "0"
        raw_value = str(query.get(name, [fallback])[0] or fallback)
        return raw_value.strip().lower() in {"1", "true", "yes", "on"}
