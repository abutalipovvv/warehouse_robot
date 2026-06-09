from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen


class RemoteRobotError(RuntimeError):
    pass


def normalize_robot_base_url(value: str) -> str:
    raw = str(value or "").strip().rstrip("/")
    if not raw:
        return ""
    if "://" not in raw:
        raw = f"http://{raw}"
    parsed = urlparse(raw)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise RemoteRobotError(f"invalid robot API URL: {value}")
    return raw


class RemoteRobotAdapter:
    """Small HTTP adapter for robot_http_api-compatible robots."""

    def __init__(self, timeout: float = 0.8) -> None:
        self.timeout = max(0.2, float(timeout))

    def identity(self, base_url: str) -> dict[str, Any]:
        return self.get_json(base_url, "/api/robot/identity")

    def status(self, base_url: str) -> dict[str, Any]:
        return self.get_json(base_url, "/api/robot/status")

    def execute_route(self, base_url: str, payload: dict[str, Any]) -> dict[str, Any]:
        return self.post_json(base_url, "/api/robot/route/execute", payload)

    def cancel_route(self, base_url: str) -> dict[str, Any]:
        return self.post_json(base_url, "/api/robot/route/cancel", {})

    def stop(self, base_url: str) -> dict[str, Any]:
        return self.post_json(base_url, "/api/robot/stop", {})

    def get_json(self, base_url: str, path: str, timeout: float | None = None) -> dict[str, Any]:
        return self.request_json(base_url, path, method="GET", timeout=timeout)

    def post_json(
        self,
        base_url: str,
        path: str,
        payload: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        return self.request_json(base_url, path, method="POST", payload=payload or {}, timeout=timeout)

    def request_json(
        self,
        base_url: str,
        path: str,
        *,
        method: str = "GET",
        payload: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        base = normalize_robot_base_url(base_url)
        url = f"{base}{path}"
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            headers["Content-Type"] = "application/json"
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = Request(url, data=data, headers=headers, method=method.upper())
        try:
            with urlopen(request, timeout=self.timeout if timeout is None else max(0.2, float(timeout))) as response:
                raw = response.read()
                status = int(response.status)
        except HTTPError as exc:
            raw = exc.read()
            status = int(exc.code)
        except URLError as exc:
            reason = getattr(exc, "reason", exc)
            raise RemoteRobotError(f"network error: {reason}") from exc
        except TimeoutError as exc:
            raise RemoteRobotError("request timed out") from exc

        try:
            decoded = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise RemoteRobotError(f"invalid JSON from {url}") from exc
        if not isinstance(decoded, dict):
            raise RemoteRobotError(f"unexpected payload from {url}")
        if status >= 400 or decoded.get("ok") is False:
            raise RemoteRobotError(str(decoded.get("error") or f"HTTP {status}"))
        return decoded
