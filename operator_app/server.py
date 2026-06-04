from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse
import webbrowser

from .map_cache import MapCache
from .models import KnownRobot
from .registry import RobotRegistry, default_registry_path
from .robot_client import RobotClient, RobotProbeError

DEFAULT_STATIC_DIR = Path(__file__).resolve().parent / "static"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class OperatorAppState:
    def __init__(self, registry_path: Path, probe_timeout: float) -> None:
        self.registry = RobotRegistry(registry_path)
        self.map_cache = MapCache()
        self.client = RobotClient(timeout=probe_timeout)
        self.map_timeout = max(10.0, float(probe_timeout) * 10.0)
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
        pulled = None
        try:
            active = self.robot_maps_active_payload(robot.id)
            active_name = str(active.get("mapName") or "").strip()
            if active_name:
                pulled = self.pull_robot_map_payload(robot.id, {"mapName": active_name})
        except Exception:
            pulled = None
        return {
            "ok": True,
            "robot": self._robot_payload(robot, probe=probe),
            "pulled": pulled,
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
            except Exception as exc:
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

    def robot_maps_list_payload(self, robot_id: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        return self.client.request_json(robot.base_url, "/api/maps/list", timeout=self.map_timeout)

    def robot_maps_active_payload(self, robot_id: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        return self.client.request_json(robot.base_url, "/api/maps/active", timeout=self.map_timeout)

    def pull_robot_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        map_name = str(payload.get("mapName") or "").strip()
        path = "/api/maps/pull"
        if map_name:
            path = f"{path}?name={map_name}"
        result = self.client.request_json(robot.base_url, path, timeout=self.map_timeout)
        local_name = str(result.get("mapName") or map_name or "active").strip() or "active"
        cached = self.map_cache.save_pulled_map(robot_id, result, activate=True)
        return {
            "ok": True,
            "pulled": result,
            "local": {
                "mapName": str(cached.get("mapName") or local_name),
                "savedAt": str(cached.get("savedAt") or ""),
            },
        }

    def local_maps_payload(self, robot_id: str) -> dict[str, Any]:
        self.get_robot(robot_id)
        return {
            "ok": True,
            "activeMapName": self.map_cache.active_map_name(robot_id),
            "maps": self.map_cache.list_maps(robot_id),
        }

    def local_active_map_payload(self, robot_id: str) -> dict[str, Any]:
        self.get_robot(robot_id)
        active_name = self.map_cache.active_map_name(robot_id)
        active_payload = self.map_cache.load_active_map(robot_id)
        if isinstance(active_payload, dict):
            robot_signature = str(active_payload.get("robotSignature") or "").strip()
            if not robot_signature:
                try:
                    robot_active = self.robot_maps_active_payload(robot_id)
                    robot_active_name = str(robot_active.get("mapName") or "").strip()
                    robot_current = self._fetch_robot_map_payload(robot_id, robot_active_name)
                    local_signature = str(active_payload.get("signature") or "").strip()
                    fresh_robot_signature = str(robot_current.get("signature") or "").strip()
                    active_payload["robotSignature"] = fresh_robot_signature
                    active_payload["robotMapName"] = robot_active_name
                    active_payload["hasLocalChanges"] = bool(
                        (local_signature and fresh_robot_signature and local_signature != fresh_robot_signature)
                        or (robot_active_name and str(active_payload.get("mapName") or "").strip() != robot_active_name)
                    )
                except Exception:
                    pass
        return {
            "ok": True,
            "activeMapName": active_name,
            "map": active_payload.get("map") if isinstance(active_payload, dict) else None,
            "sourceMapName": str(active_payload.get("sourceMapName") or "") if isinstance(active_payload, dict) else "",
            "signature": str(active_payload.get("signature") or "") if isinstance(active_payload, dict) else "",
            "robotSignature": str(active_payload.get("robotSignature") or "") if isinstance(active_payload, dict) else "",
            "robotMapName": str(active_payload.get("robotMapName") or "") if isinstance(active_payload, dict) else "",
            "hasLocalChanges": bool(active_payload.get("hasLocalChanges")) if isinstance(active_payload, dict) else False,
        }

    def local_map_payload(self, robot_id: str, map_name: str) -> dict[str, Any]:
        self.get_robot(robot_id)
        payload = self.map_cache.load_map(robot_id, map_name)
        return {"ok": True, **payload}

    def save_local_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        self.get_robot(robot_id)
        map_name = str(payload.get("mapName") or "").strip()
        editable_map = payload.get("map")
        if not map_name:
            raise ValueError("mapName is required")
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        saved = self.map_cache.save_map(
            robot_id,
            map_name,
            editable_map,
            source_map_name=str(payload.get("sourceMapName") or map_name),
            activate=bool(payload.get("activate", True)),
        )
        return {"ok": True, "local": saved}

    def activate_local_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        self.get_robot(robot_id)
        map_name = str(payload.get("mapName") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        self.map_cache.load_map(robot_id, map_name)
        self.map_cache.set_active_map(robot_id, map_name)
        return {"ok": True, "activeMapName": map_name}

    def push_robot_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        editable_map = payload.get("map")
        if not isinstance(editable_map, dict):
            local_name = str(payload.get("localMapName") or payload.get("mapName") or "").strip()
            cached = self.map_cache.load_map(robot_id, local_name)
            editable_map = cached.get("map")
            payload = {
                **payload,
                "sourceMapName": str(payload.get("sourceMapName") or cached.get("sourceMapName") or local_name),
            }
        if not isinstance(editable_map, dict):
            raise ValueError("map payload is required")
        request_payload = {
            "map": editable_map,
            "mapName": str(payload.get("mapName") or editable_map.get("mapName") or "").strip(),
            "sourceMapName": str(payload.get("sourceMapName") or payload.get("mapName") or "").strip(),
            "outputName": str(payload.get("outputName") or "").strip(),
            "overwriteOutput": bool(payload.get("overwriteOutput", False)),
        }
        result = self.client.request_json(
            robot.base_url,
            "/api/maps/push",
            method="POST",
            payload=request_payload,
            timeout=self.map_timeout,
        )
        local_name = str(result.get("mapName") or request_payload.get("outputName") or request_payload.get("mapName") or "map").strip()
        cached = self.map_cache.save_map(
            robot_id,
            local_name,
            result,
            source_map_name=str(result.get("mapName") or local_name),
        )
        return {"ok": True, "pushed": result, "local": cached}

    def pull_sync_payload(self, robot_id: str) -> dict[str, Any]:
        robot_active = self.robot_maps_active_payload(robot_id)
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(robot_id)
        robot_current = self._fetch_robot_map_payload(robot_id, robot_active_name)
        local_signature = ""
        if isinstance(local_active, dict):
            local_map = local_active.get("map")
            if isinstance(local_map, dict):
                local_signature = str(local_map.get("signature") or "").strip()
        robot_signature = str(robot_current.get("signature") or "").strip()
        local_active_name = str(local_active.get("mapName") or "") if isinstance(local_active, dict) else ""
        if robot_active_name and robot_active_name == local_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "message": f"Operator already has active robot map {robot_active_name}.",
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_active_name,
            }
        cached = self.map_cache.save_pulled_map(robot_id, robot_current, activate=True)
        pulled = {
            "ok": True,
            "pulled": robot_current,
            "local": {
                "mapName": str(cached.get("mapName") or robot_active_name),
                "savedAt": str(cached.get("savedAt") or ""),
            },
        }
        return {
            "ok": True,
            "changed": True,
            "message": f"Pulled active robot map {robot_active_name}.",
            "robotActiveMapName": robot_active_name,
            "localActiveMapName": str(pulled.get("local", {}).get("mapName") or robot_active_name),
            **pulled,
        }

    def load_robot_map_payload(self, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        map_name = str(payload.get("mapName") or payload.get("folder") or "").strip()
        if not map_name:
            raise ValueError("mapName is required")
        return self.client.request_json(
            robot.base_url,
            "/api/maps/load",
            method="POST",
            payload={"mapName": map_name},
            timeout=self.map_timeout,
        )

    def push_sync_payload(self, robot_id: str) -> dict[str, Any]:
        robot_active = self.robot_maps_active_payload(robot_id)
        robot_active_name = str(robot_active.get("mapName") or "").strip()
        local_active = self.map_cache.load_active_map(robot_id)
        if not isinstance(local_active, dict):
            raise ValueError("operator has no active local map")
        local_map_name = str(local_active.get("mapName") or "").strip()
        if not local_map_name:
            raise ValueError("operator active local map is invalid")
        local_map_payload = local_active.get("map")
        local_signature = str(local_map_payload.get("signature") or "").strip() if isinstance(local_map_payload, dict) else ""
        has_local_changes = bool(local_active.get("hasLocalChanges"))
        robot_current = self._fetch_robot_map_payload(robot_id, robot_active_name)
        robot_signature = str(robot_current.get("signature") or "").strip()
        if (not has_local_changes) and local_map_name == robot_active_name and local_signature and local_signature == robot_signature:
            return {
                "ok": True,
                "changed": False,
                "message": f"Robot already uses {robot_active_name}.",
                "robotActiveMapName": robot_active_name,
                "localActiveMapName": local_map_name,
            }
        pushed = self.push_robot_map_payload(
            robot_id,
            {
                "localMapName": local_map_name,
                "mapName": local_map_name,
                "sourceMapName": str(local_active.get("sourceMapName") or local_map_name),
            },
        )
        loaded = self.load_robot_map_payload(robot_id, {"mapName": local_map_name})
        local_signature_after_push = str((pushed.get("pushed") or {}).get("signature") or local_signature).strip()
        synced_local = self.map_cache.mark_synced(
            robot_id,
            local_map_name,
            robot_signature=local_signature_after_push,
            robot_map_name=str(loaded.get("mapName") or local_map_name),
            activate=True,
        )
        return {
            "ok": True,
            "changed": True,
            "message": f"Pushed and activated {local_map_name} on robot.",
            "robotActiveMapName": str(loaded.get("mapName") or local_map_name),
            "localActiveMapName": local_map_name,
            "pushed": pushed.get("pushed"),
            "local": synced_local,
            "loaded": loaded,
        }

    def _fetch_robot_map_payload(self, robot_id: str, map_name: str) -> dict[str, Any]:
        robot = self.get_robot(robot_id)
        path = "/api/maps/pull"
        if map_name:
            path = f"{path}?name={map_name}"
        return self.client.request_json(robot.base_url, path, timeout=self.map_timeout)

    @staticmethod
    def _require_host(payload: dict[str, Any]) -> str:
        host = str(payload.get("host") or "").strip()
        if not host:
            raise ValueError("host is required")
        if host in {"0.0.0.0", "::"}:
            raise ValueError(
                f"{host} is a listen/bind address, not a robot address. "
                "Use 127.0.0.1 on the same PC or the robot LAN IP like 192.168.x.x."
            )
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
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/health":
                self._send_json({"ok": True})
                return
            if path == "/api/robots":
                self._send_json(self._require_state().list_robots_payload())
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
            super().do_GET()
        except BrokenPipeError:
            return
        except Exception as exc:  # pragma: no cover - defensive server path
            self._send_error_json(500, str(exc))

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
