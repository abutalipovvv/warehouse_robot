from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock
from typing import Any
from urllib.parse import unquote, urlparse
import webbrowser

from .models import KnownRobot
from .registry import RobotRegistry, default_registry_path
from .robot_client import RobotClient, RobotProbeError

DEFAULT_STATIC_DIR = Path(__file__).resolve().parent / "static"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class OperatorAppState:
    def __init__(self, registry_path: Path, probe_timeout: float) -> None:
        self.registry = RobotRegistry(registry_path)
        self.client = RobotClient(timeout=probe_timeout)
        self._lock = Lock()

    def list_robots_payload(self) -> dict[str, Any]:
        with self._lock:
            robots = self.registry.load()
        items = [self._robot_payload(robot) for robot in robots]
        return {"ok": True, "robots": items}

    def probe_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        host = self._require_host(payload)
        port = self._require_port(payload)
        result = self._probe_robot(host, port)
        return {"ok": True, "probe": result}

    def add_robot_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        host = self._require_host(payload)
        port = self._require_port(payload)
        probe = self._probe_robot(host, port)
        identity = probe.get("identity", {})
        robot_id = str(identity.get("robotId") or f"{host}:{port}").strip()
        name = str(payload.get("name") or "").strip() or robot_id
        robot = KnownRobot(
            id=f"{robot_id}@{host}:{port}",
            name=name,
            host=host,
            port=port,
            last_seen=utc_now(),
            last_identity=identity if isinstance(identity, dict) else None,
        )
        with self._lock:
            self.registry.upsert(robot)
        return {
            "ok": True,
            "robot": self._robot_payload(robot, probe=probe),
        }

    def delete_robot_payload(self, robot_id: str) -> dict[str, Any]:
        with self._lock:
            deleted = self.registry.remove(robot_id)
        if not deleted:
            raise ValueError(f"unknown robot: {robot_id}")
        return {"ok": True, "deleted": robot_id}

    def get_robot(self, robot_id: str) -> KnownRobot:
        with self._lock:
            robots = self.registry.load()
        for robot in robots:
            if robot.id == robot_id:
                return robot
        raise ValueError(f"unknown robot: {robot_id}")

    def _robot_payload(self, robot: KnownRobot, probe: dict[str, Any] | None = None) -> dict[str, Any]:
        if probe is None:
            try:
                probe = self._probe_robot(robot.host, robot.port)
            except RobotProbeError as exc:
                probe = {
                    "ok": False,
                    "baseUrl": robot.base_url,
                    "online": False,
                    "error": str(exc),
                    "identity": robot.last_identity or None,
                    "status": None,
                }
        payload = robot.to_dict()
        payload.update(
            {
                "online": bool(probe.get("ok", False)),
                "baseUrl": str(probe.get("baseUrl") or robot.base_url),
                "identity": probe.get("identity") or robot.last_identity or None,
                "status": probe.get("status") if isinstance(probe.get("status"), dict) else None,
                "error": str(probe.get("error") or "").strip(),
            }
        )
        return payload

    def _probe_robot(self, host: str, port: int) -> dict[str, Any]:
        try:
            result = self.client.probe(host, port)
        except RobotProbeError as exc:
            raise RobotProbeError(f"{host}:{port} is not reachable: {exc}") from exc
        result["online"] = True
        return result

    def proxy_request(
        self,
        robot_id: str,
        method: str,
        path: str,
        *,
        headers: dict[str, str] | None = None,
        body: bytes | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        robot = self.get_robot(robot_id)
        return self.client.request(robot.base_url, path, method=method, headers=headers, body=body)

    @staticmethod
    def _require_host(payload: dict[str, Any]) -> str:
        host = str(payload.get("host") or "").strip()
        if not host:
            raise ValueError("host is required")
        return host

    @staticmethod
    def _require_port(payload: dict[str, Any]) -> int:
        raw = payload.get("port", 8790)
        try:
            port = int(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("port must be an integer") from exc
        if port < 1 or port > 65535:
            raise ValueError("port must be in range 1..65535")
        return port


class OperatorRequestHandler(SimpleHTTPRequestHandler):
    app_state: OperatorAppState | None = None

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/health":
            self._send_json({"ok": True})
            return
        if path == "/api/robots":
            self._send_json(self._require_state().list_robots_payload())
            return
        proxy_target = self._parse_robot_proxy(parsed)
        if proxy_target is not None:
            self._proxy_robot_request("GET", *proxy_target)
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/api/robots/probe":
            self._handle_json(self._require_state().probe_payload)
            return
        if path == "/api/robots":
            self._handle_json(self._require_state().add_robot_payload)
            return
        proxy_target = self._parse_robot_proxy(parsed)
        if proxy_target is not None:
            self._proxy_robot_request("POST", *proxy_target)
            return
        self._send_error_json(404, "not found")

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

        response_body, content_type = self._rewrite_proxy_content(robot_path, response_headers, response_body)
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(response_body)))
        self.end_headers()
        self.wfile.write(response_body)

    def _rewrite_proxy_content(
        self,
        robot_path: str,
        response_headers: dict[str, str],
        response_body: bytes,
    ) -> tuple[bytes, str]:
        content_type = str(response_headers.get("Content-Type") or "application/octet-stream")
        if "text/html" in content_type:
            text = response_body.decode("utf-8")
            text = text.replace('src="/demo-data.js"', 'src="demo-data.js"')
            return text.encode("utf-8"), "text/html; charset=utf-8"
        if robot_path.endswith("/app.js") and "javascript" in content_type:
            text = response_body.decode("utf-8")
            text = text.replace('"/api/', '"api/')
            text = text.replace("'/api/", "'api/")
            return text.encode("utf-8"), "application/javascript; charset=utf-8"
        return response_body, content_type

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve operator app for managing robots by IP.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8780, type=int)
    parser.add_argument("--registry", default=default_registry_path(), type=Path)
    parser.add_argument("--probe-timeout", default=1.0, type=float)
    parser.add_argument("--open", action="store_true")
    return parser.parse_args()


def browser_url(host: str, port: int) -> str:
    if host in {"0.0.0.0", "::"}:
        return f"http://127.0.0.1:{port}/"
    return f"http://{host}:{port}/"


def main() -> None:
    args = parse_args()
    state = OperatorAppState(registry_path=args.registry, probe_timeout=args.probe_timeout)
    OperatorRequestHandler.app_state = state
    handler = partial(OperatorRequestHandler, directory=str(DEFAULT_STATIC_DIR.resolve()))
    server = ThreadingHTTPServer((args.host, args.port), handler)
    url = browser_url(args.host, args.port)
    print(f"Serving operator app: {url}")
    print(f"Registry path: {args.registry.expanduser()}")
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
