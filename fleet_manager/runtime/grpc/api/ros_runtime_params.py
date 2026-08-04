"""Persist runtime parameters and apply declared Nav2 values."""

from __future__ import annotations

from time import sleep
from typing import Any

import yaml

from fleet_manager.core.io.atomic_files import atomic_write_text

NAV2_RUNTIME_PARAMETERS: tuple[tuple[str, str, str], ...] = (
    ("nav2.amcl.update_min_d", "/amcl/set_parameters", "update_min_d"),
    ("nav2.amcl.update_min_a", "/amcl/set_parameters", "update_min_a"),
    ("nav2.amcl.transform_tolerance", "/amcl/set_parameters", "transform_tolerance"),
    ("nav2.amcl.min_particles", "/amcl/set_parameters", "min_particles"),
    ("nav2.amcl.max_particles", "/amcl/set_parameters", "max_particles"),
    ("nav2.controller_server.controller_frequency", "/controller_server/set_parameters", "controller_frequency"),
    ("nav2.controller_server.follow_path.vx_max", "/controller_server/set_parameters", "FollowPath.vx_max"),
    ("nav2.controller_server.follow_path.vx_min", "/controller_server/set_parameters", "FollowPath.vx_min"),
    ("nav2.controller_server.follow_path.wz_max", "/controller_server/set_parameters", "FollowPath.wz_max"),
    ("nav2.local_costmap.robot_radius", "/local_costmap/local_costmap/set_parameters", "robot_radius"),
    (
        "nav2.local_costmap.inflation_radius",
        "/local_costmap/local_costmap/set_parameters",
        "inflation_layer.inflation_radius",
    ),
    (
        "nav2.local_costmap.cost_scaling_factor",
        "/local_costmap/local_costmap/set_parameters",
        "inflation_layer.cost_scaling_factor",
    ),
    ("nav2.global_costmap.robot_radius", "/global_costmap/global_costmap/set_parameters", "robot_radius"),
    (
        "nav2.global_costmap.inflation_radius",
        "/global_costmap/global_costmap/set_parameters",
        "inflation_layer.inflation_radius",
    ),
)

class RosRuntimeParametersMixin:
    """Persist runtime parameters and apply declared Nav2 values."""

    def params_payload(self) -> dict[str, Any]:
        if self.params_path is None:
            raise ValueError("params path is not configured")
        try:
            payload = yaml.safe_load(self.params_path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(f"params file does not exist: {self.params_path}") from exc
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            raise ValueError(f"params file must contain a YAML object: {self.params_path}")
        return {"ok": True, "params": payload, "path": str(self.params_path)}

    def save_params_payload(self, params_payload: dict[str, Any], *, reload_runtime: bool = True) -> dict[str, Any]:
        if self.params_path is None:
            raise ValueError("params path is not configured")
        if not isinstance(params_payload, dict):
            raise ValueError("params payload must be an object")
        previous_payload: dict[str, Any] | None = None
        try:
            loaded = yaml.safe_load(self.params_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                previous_payload = loaded
        except Exception:
            previous_payload = None
        atomic_write_text(
            self.params_path,
            yaml.safe_dump(params_payload, allow_unicode=True, sort_keys=False),
        )
        warnings = []
        reloaded = False
        if reload_runtime:
            try:
                reloaded = self._apply_nav2_runtime_params(params_payload, previous_payload=previous_payload) or reloaded
            except Exception as exc:
                warnings.append(f"Nav2 runtime apply failed: {exc}")
            try:
                reloaded = self._reload_route_status_params() or reloaded
            except Exception as exc:
                warnings.append(f"route/status reload failed: {exc}")
        return {
            "ok": True,
            "params": params_payload,
            "path": str(self.params_path),
            "reloaded": reloaded,
            "warning": "; ".join(warnings),
        }

    def _apply_saved_nav2_params_when_ready(self) -> None:
        if self.params_path is None:
            return
        for _attempt in range(20):
            sleep(1.0)
            try:
                payload = yaml.safe_load(self.params_path.read_text(encoding="utf-8"))
            except Exception:
                return
            if not isinstance(payload, dict) or "nav2" not in payload:
                return
            try:
                if self._apply_nav2_runtime_params(payload, require_available=False):
                    return
            except Exception:
                continue

    def _apply_nav2_runtime_params(
        self,
        params_payload: dict[str, Any],
        *,
        require_available: bool = True,
        previous_payload: dict[str, Any] | None = None,
    ) -> bool:
        if not isinstance(params_payload.get("nav2"), dict):
            return False
        if self._node is None or self._set_parameters_type is None:
            if require_available:
                raise ValueError("ROS2 parameter runtime is not available")
            return False

        grouped: dict[str, dict[str, Any]] = {}
        for source_path, service_name, param_name in NAV2_RUNTIME_PARAMETERS:
            value = self._deep_get(params_payload, source_path)
            if value is None:
                continue
            if previous_payload is not None and self._values_equal(value, self._deep_get(previous_payload, source_path)):
                continue
            grouped.setdefault(service_name, {})[param_name] = value

        velocity = params_payload.get("nav2", {}).get("velocity_smoother")
        if isinstance(velocity, dict):
            max_x = velocity.get("max_velocity_x")
            max_theta = velocity.get("max_velocity_theta")
            previous_velocity = previous_payload.get("nav2", {}).get("velocity_smoother") if isinstance(previous_payload, dict) else None
            velocity_changed = max_x is not None or max_theta is not None
            if isinstance(previous_velocity, dict):
                velocity_changed = (
                    not self._values_equal(max_x, previous_velocity.get("max_velocity_x"))
                    or not self._values_equal(max_theta, previous_velocity.get("max_velocity_theta"))
                )
            if velocity_changed:
                previous_max_x = previous_velocity.get("max_velocity_x") if isinstance(previous_velocity, dict) else None
                previous_max_theta = previous_velocity.get("max_velocity_theta") if isinstance(previous_velocity, dict) else None
                grouped.setdefault("/velocity_smoother/set_parameters", {})["max_velocity"] = [
                    float(max_x if max_x is not None else (previous_max_x if previous_max_x is not None else 0.5)),
                    0.0,
                    float(max_theta if max_theta is not None else (previous_max_theta if previous_max_theta is not None else 2.0)),
                ]

        applied = False
        missing: list[str] = []
        for service_name, values in grouped.items():
            client = self._nav2_param_clients.get(service_name)
            if client is None:
                client = self._node.create_client(self._set_parameters_type, service_name)
                self._nav2_param_clients[service_name] = client
            if not self._service_available(client, 0.25):
                missing.append(service_name)
                continue
            values = self._filter_declared_parameters(service_name, values, require_available=require_available)
            if not values:
                continue
            request = self._set_parameters_type.Request()
            request.parameters = [
                self._parameter_message(param_name, value)
                for param_name, value in values.items()
            ]
            response = self._call_service(client, request, f"{service_name} set_parameters", timeout_sec=3.0)
            failed = [
                result.reason or "parameter rejected"
                for result in getattr(response, "results", [])
                if not bool(getattr(result, "successful", False))
            ]
            if failed:
                raise ValueError(f"{service_name}: {'; '.join(failed)}")
            applied = True
        if missing and require_available:
            raise ValueError(f"Nav2 parameter services unavailable: {', '.join(missing)}")
        return applied

    def _filter_declared_parameters(
        self,
        set_service_name: str,
        values: dict[str, Any],
        *,
        require_available: bool,
    ) -> dict[str, Any]:
        if self._node is None or self._list_parameters_type is None:
            return values
        list_service_name = set_service_name.rsplit("/", 1)[0] + "/list_parameters"
        client = self._nav2_list_param_clients.get(list_service_name)
        if client is None:
            client = self._node.create_client(self._list_parameters_type, list_service_name)
            self._nav2_list_param_clients[list_service_name] = client
        if not self._service_available(client, 0.15):
            return values if require_available else {}
        request = self._list_parameters_type.Request()
        request.prefixes = []
        request.depth = 0
        response = self._call_service(client, request, f"{list_service_name} list_parameters", timeout_sec=2.0)
        declared = set(str(name) for name in getattr(getattr(response, "result", None), "names", []))
        if not declared:
            return values
        return {name: value for name, value in values.items() if name in declared}

    def _parameter_message(self, name: str, value: Any) -> Any:
        if self._parameter_type is None or self._parameter_value_type is None or self._parameter_type_enum is None:
            raise ValueError("ROS2 parameter message types are not available")
        parameter = self._parameter_type()
        parameter.name = str(name)
        parameter_value = self._parameter_value_type()
        if isinstance(value, bool):
            parameter_value.type = self._parameter_type_enum.PARAMETER_BOOL
            parameter_value.bool_value = bool(value)
        elif isinstance(value, int) and not isinstance(value, bool):
            parameter_value.type = self._parameter_type_enum.PARAMETER_INTEGER
            parameter_value.integer_value = int(value)
        elif isinstance(value, float):
            parameter_value.type = self._parameter_type_enum.PARAMETER_DOUBLE
            parameter_value.double_value = float(value)
        elif isinstance(value, list):
            parameter_value.type = self._parameter_type_enum.PARAMETER_DOUBLE_ARRAY
            parameter_value.double_array_value = [float(item) for item in value]
        else:
            parameter_value.type = self._parameter_type_enum.PARAMETER_STRING
            parameter_value.string_value = str(value)
        parameter.value = parameter_value
        return parameter

    def _deep_get(self, source: dict[str, Any], path: str) -> Any:
        current: Any = source
        for part in str(path).split("."):
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]
        return current

    def _values_equal(self, left: Any, right: Any) -> bool:
        if isinstance(left, (int, float)) or isinstance(right, (int, float)):
            try:
                return abs(float(left) - float(right)) < 0.000001
            except (TypeError, ValueError):
                return False
        return left == right

    def _reload_route_status_params(self) -> bool:
        active = self.active_map_payload()
        map_dir = str(active.get("mapDir") or "").strip()
        map_name = str(active.get("mapName") or active.get("mapId") or "").strip()
        if not map_dir:
            raise ValueError("active map directory is not available")

        reloaded = False
        for client, label in (
            (self._route_load_map_client, "route/load_map"),
            (self._status_load_map_client, "status/load_map"),
        ):
            if client is None:
                continue
            if not self._service_available(client, 0.2):
                continue
            request = client.srv_type.Request()
            request.map_name = map_name
            request.map_dir = map_dir
            response = self._call_service(client, request, label, timeout_sec=5.0)
            if not bool(response.ok):
                raise ValueError(str(response.error or f"{label} failed"))
            reloaded = True
        if not reloaded:
            raise ValueError("route/status load-map services are not available")
        return reloaded
