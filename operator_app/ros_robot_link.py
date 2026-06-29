from __future__ import annotations

import argparse
import json
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

from .ros_robot_bridge import RosRobotBridge


class RosRobotLinkState:
    def __init__(self, bridge: RosRobotBridge) -> None:
        self.bridge = bridge


class RosRobotLinkHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    state: RosRobotLinkState | None = None

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:
        try:
            route = urlparse(self.path).path.rstrip("/") or "/"
            bridge = self._bridge()
            if route == "/health":
                self._send_json(
                    {
                        "ok": bool(bridge.available),
                        "available": bool(bridge.available),
                        "error": bridge.error,
                    },
                    status=200 if bridge.available else 503,
                )
                return
            if route == "/api/robot/identity":
                self._send_json(bridge.identity_payload())
                return
            if route == "/api/robot/status":
                self._send_json(bridge.status_payload())
                return
            if route == "/api/robot/sidebar":
                self._send_json({"ok": True, **bridge.sidebar_payload()})
                return
            self._send_json({"ok": False, "error": f"not found: {route}"}, status=404)
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, status=500)

    def do_POST(self) -> None:
        try:
            route = urlparse(self.path).path.rstrip("/") or "/"
            payload = self._read_json_payload()
            if payload is None:
                return
            bridge = self._bridge()
            if route == "/api/robot/teleop":
                self._send_json(
                    bridge.teleop(
                        linear=float(payload.get("linear", 0.0) or 0.0),
                        angular=float(payload.get("angular", 0.0) or 0.0),
                        timeout_ms=int(payload.get("timeoutMs", 350) or 350),
                    )
                )
                return
            if route == "/api/robot/teleop/stop":
                self._send_json(bridge.teleop_stop())
                return
            if route == "/api/robot/stop":
                self._send_json(bridge.stop())
                return
            if route == "/api/robot/route/cancel":
                self._send_json(bridge.cancel_route())
                return
            if route == "/api/robot/route/execute":
                self._send_json(bridge.execute_route(payload))
                return
            self._send_json({"ok": False, "error": f"not found: {route}"}, status=404)
        except ValueError as exc:
            self._send_json({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, status=500)

    def _bridge(self) -> RosRobotBridge:
        if self.state is None:
            raise RuntimeError("ROS robot link is not ready")
        return self.state.bridge

    def _read_json_payload(self) -> dict[str, Any] | None:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length) if content_length > 0 else b""
        if not raw_body:
            return {}
        try:
            payload = json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self._send_json({"ok": False, "error": "invalid JSON"}, status=400)
            return None
        if not isinstance(payload, dict):
            self._send_json({"ok": False, "error": "expected object payload"}, status=400)
            return None
        return payload

    def _send_json(self, payload: object, *, status: int = 200) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Local HTTP bridge for one ROS2 robot.")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--domain-id", required=True, type=int)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--status-topic", default="/robot_status")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--go-to-lm-topic", default="/go_to_lm")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bridge = RosRobotBridge(
        robot_id=args.robot_id,
        robot_name=args.robot_name,
        host=args.host,
        domain_id=args.domain_id,
        namespace=args.namespace,
        status_topic=args.status_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        go_to_lm_topic=args.go_to_lm_topic,
    )
    RosRobotLinkHandler.state = RosRobotLinkState(bridge)
    handler = partial(RosRobotLinkHandler)
    server = ThreadingHTTPServer((args.bind_host, args.port), handler)
    server.daemon_threads = True
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        server.server_close()


if __name__ == "__main__":
    main()
