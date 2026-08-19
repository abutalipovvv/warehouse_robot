"""Transfer, enumerate and activate robot map bundles."""

from __future__ import annotations

import json
from typing import Any

from .contracts import (
    DEFAULT_GRPC_MAP_LOAD_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC,
    DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC,
)

class RosRuntimeMapTransferMixin:
    """Transfer, enumerate and activate robot map bundles."""

    def active_map_payload(self) -> dict[str, Any]:
        if self._map_state_client is None:
            raise ValueError("map state service is not configured")
        request = self._map_state_client.srv_type.Request()
        response = self._call_service(
            self._map_state_client,
            request,
            "map state",
            timeout_sec=DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC,
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "map state failed"))
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
            "signature": str(response.signature or ""),
        }

    def list_maps_payload(self) -> dict[str, Any]:
        if self._map_list_client is None:
            raise ValueError("map list service is not configured")
        response = self._call_service(
            self._map_list_client,
            self._map_list_client.srv_type.Request(),
            "map list",
            timeout_sec=DEFAULT_GRPC_MAP_QUERY_TIMEOUT_SEC,
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "map list failed"))
        maps = []
        signatures = list(response.map_signatures)
        for index, (name, map_dir, map_id) in enumerate(
            zip(response.map_names, response.map_dirs, response.map_ids)
        ):
            maps.append(
                {
                    "name": str(name),
                    "folder": f"{name}.smap" if str(name) and not str(name).endswith(".smap") else str(name),
                    "mapDir": str(map_dir),
                    "mapId": str(map_id),
                    "signature": str(signatures[index]) if index < len(signatures) else "",
                    "active": str(name) == str(response.active_map_name),
                }
            )
        return {
            "ok": True,
            "active": str(response.active_map_name or ""),
            "activeMapDir": str(response.active_map_dir or ""),
            "activeMapId": str(response.active_map_id or ""),
            "activeSignature": str(response.active_map_signature or ""),
            "maps": maps,
        }

    def pull_map_bundle_payload(self, map_name: str = "") -> dict[str, Any]:
        if self._map_get_bundle_client is None:
            raise ValueError("map bundle service is not configured")
        request = self._map_get_bundle_client.srv_type.Request()
        request.map_name = str(map_name or "")
        response = self._call_service(
            self._map_get_bundle_client,
            request,
            "map bundle pull",
            timeout_sec=DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC,
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "map bundle pull failed"))
        try:
            payload = json.loads(str(response.bundle_json or "{}"))
        except json.JSONDecodeError as exc:
            raise ValueError("map bundle service returned invalid JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("map bundle service returned invalid payload")
        payload.setdefault("ok", True)
        payload.setdefault("mapName", str(response.map_name or ""))
        payload.setdefault("mapDir", str(response.map_dir or ""))
        payload.setdefault("signature", str(response.signature or ""))
        return payload

    def push_map_bundle_payload(
        self,
        bundle_payload: dict[str, Any],
        *,
        map_name: str = "",
        activate: bool = False,
    ) -> dict[str, Any]:
        if self._map_put_bundle_client is None:
            raise ValueError("map bundle push service is not configured")
        request = self._map_put_bundle_client.srv_type.Request()
        request.map_name = str(map_name or bundle_payload.get("mapName") or "")
        request.bundle_json = json.dumps(bundle_payload, ensure_ascii=False)
        request.activate = bool(activate)
        response = self._call_service(
            self._map_put_bundle_client,
            request,
            "map bundle push",
            timeout_sec=DEFAULT_GRPC_MAP_TRANSFER_TIMEOUT_SEC,
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "map bundle push failed"))
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
            "signature": str(response.signature or ""),
        }

    def load_map(self, map_name: str) -> dict[str, Any]:
        if self._map_load_client is None:
            raise ValueError("map load service is not configured")
        request = self._map_load_client.srv_type.Request()
        request.map_name = str(map_name)
        request.map_dir = ""
        response = self._call_service(
            self._map_load_client,
            request,
            "map load",
            timeout_sec=DEFAULT_GRPC_MAP_LOAD_TIMEOUT_SEC,
        )
        if not bool(response.ok):
            raise ValueError(str(response.error or "map load failed"))
        return {
            "ok": True,
            "mapName": str(response.map_name or ""),
            "mapDir": str(response.map_dir or ""),
            "mapId": str(response.map_id or ""),
            "signature": str(response.signature or ""),
        }
