from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class RobotProbeError(RuntimeError):
    pass


class RobotClient:
    def __init__(self, timeout: float = 1.5) -> None:
        self.timeout = max(0.2, float(timeout))

    def probe(self, host: str, port: int) -> dict[str, Any]:
        base_url = f"http://{host}:{port}"
        self._get_json(f"{base_url}/health")
        identity = self._get_json(f"{base_url}/api/robot/identity")
        status_payload = self._get_json(f"{base_url}/api/robot/status")
        robot_status = status_payload.get("robot", {})
        if not isinstance(robot_status, dict):
            robot_status = {}
        return {
            "ok": True,
            "baseUrl": base_url,
            "identity": identity,
            "status": robot_status,
        }

    def request(
        self,
        base_url: str,
        path: str,
        *,
        method: str = "GET",
        headers: dict[str, str] | None = None,
        body: bytes | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        url = f"{base_url.rstrip('/')}{path}"
        request = Request(url, data=body, method=method.upper())
        for key, value in (headers or {}).items():
            request.add_header(key, value)
        try:
            with urlopen(request, timeout=self.timeout) as response:
                raw = response.read()
                response_headers = {key: value for key, value in response.headers.items()}
                return int(response.status), response_headers, raw
        except HTTPError as exc:
            raw = exc.read()
            response_headers = {key: value for key, value in exc.headers.items()}
            return int(exc.code), response_headers, raw
        except URLError as exc:
            reason = getattr(exc, "reason", exc)
            raise RobotProbeError(f"Network error: {reason}") from exc

    def _get_json(self, url: str) -> dict[str, Any]:
        request = Request(url, headers={"Accept": "application/json"})
        try:
            with urlopen(request, timeout=self.timeout) as response:
                raw = response.read()
        except HTTPError as exc:
            raise RobotProbeError(f"HTTP {exc.code} from {url}") from exc
        except URLError as exc:
            reason = getattr(exc, "reason", exc)
            raise RobotProbeError(f"Network error: {reason}") from exc
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise RobotProbeError(f"Invalid JSON from {url}") from exc
        if not isinstance(payload, dict):
            raise RobotProbeError(f"Unexpected payload from {url}")
        return payload
