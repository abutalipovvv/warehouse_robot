from __future__ import annotations

import json
import time
from concurrent import futures
from typing import Any

from .contracts import API_VERSION, GRPC_CHANNEL_OPTIONS, json_loads_object, robot_status_from_json
from .proto import robot_api_pb2


def _load_grpc_modules():
    try:
        import grpc  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "grpcio is not installed. Install it on the robot: sudo apt install python3-grpcio"
        ) from exc
    from .proto import robot_api_pb2_grpc

    return grpc, robot_api_pb2_grpc


class RobotApiService:
    def __init__(self, runtime: Any) -> None:
        self.runtime = runtime

    def Health(self, request, context) -> robot_api_pb2.HealthResponse:
        del request, context
        error = str(getattr(self.runtime, "error", "") or "")
        return robot_api_pb2.HealthResponse(ok=not error, error=error, service=API_VERSION)

    def Identity(self, request, context) -> robot_api_pb2.IdentityResponse:
        del request, context
        try:
            payload = self.runtime.identity_payload()
            return robot_api_pb2.IdentityResponse(
                ok=bool(payload.get("ok", True)),
                error=str(payload.get("error") or ""),
                robot_id=str(payload.get("robotId") or payload.get("robot_id") or ""),
                robot_name=str(payload.get("name") or payload.get("robotName") or payload.get("robotId") or ""),
                map_id=str(payload.get("mapId") or payload.get("map_id") or ""),
                api_version=API_VERSION,
                driver=str(payload.get("driver") or payload.get("type") or "ros2"),
            )
        except Exception as exc:
            return robot_api_pb2.IdentityResponse(ok=False, error=str(exc), api_version=API_VERSION)

    def GetStatus(self, request, context) -> robot_api_pb2.StatusResponse:
        del request, context
        return self._status_response()

    def WatchStatus(self, request, context):
        interval_sec = max(0.05, min(5.0, float(request.interval_ms or 200) / 1000.0))
        while context.is_active():
            yield self._status_response()
            time.sleep(interval_sec)

    def ExecuteRoute(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            payload = json_loads_object(request.route_json)
            if request.goal_lm:
                payload.setdefault("goalLm", str(request.goal_lm))
            if request.start_lm:
                payload.setdefault("startLm", str(request.start_lm))
            if request.command_id:
                payload.setdefault("commandId", str(request.command_id))
            if request.owner_id:
                payload.setdefault("ownerId", str(request.owner_id))
            order = json_loads_object(request.order_json) if request.order_json else {}
            if order:
                payload.setdefault("order", order)
            result = self.runtime.execute_route(payload)
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def CancelRoute(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.cancel_route(owner_id=str(request.owner_id or ""))
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def Teleop(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.teleop(
                linear=float(request.linear),
                angular=float(request.angular),
                timeout_ms=max(80, int(request.timeout_ms or 350)),
                owner_id=str(request.owner_id or ""),
            )
            return self._command_response(result)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc))

    def TeleopStream(self, request_iterator, context):
        for request in request_iterator:
            if not context.is_active():
                return
            yield self.Teleop(request, context)

    def Stop(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.stop(owner_id=str(request.owner_id or ""))
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def AcquireControl(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.acquire_control(
                owner_id=str(request.owner_id or ""),
                owner_name=str(request.owner_name or ""),
                force=bool(request.force),
                lease_ms=int(request.lease_ms or 0),
            )
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def ReleaseControl(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.release_control(
                owner_id=str(request.owner_id or ""),
                force=bool(request.force),
            )
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def Relocate(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.relocate(
                x=float(request.x),
                y=float(request.y),
                yaw=float(request.yaw),
                frame_id=str(request.frame_id or "map"),
                covariance_json=str(request.covariance_json or ""),
                confirm=bool(request.confirm),
                owner_id=str(request.owner_id or ""),
            )
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def ConfirmLocalization(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.confirm_localization(
                accepted=bool(request.accepted),
                message=str(request.message or ""),
                owner_id=str(request.owner_id or ""),
            )
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def PauseRoute(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.pause_route(
                owner_id=str(request.owner_id or ""),
                message=str(request.message or ""),
            )
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def ResumeRoute(self, request, context) -> robot_api_pb2.CommandResponse:
        del context
        try:
            result = self.runtime.resume_route(
                owner_id=str(request.owner_id or ""),
                message=str(request.message or ""),
            )
            return self._command_response(result, command_id=request.command_id)
        except Exception as exc:
            return robot_api_pb2.CommandResponse(ok=False, error=str(exc), command_id=str(request.command_id or ""))

    def WatchLaserScan(self, request, context):
        hz = max(0.1, min(10.0, float(request.hz or 1.0)))
        interval_sec = 1.0 / hz
        topic = str(request.topic or "/scan")
        include_intensities = bool(request.include_intensities)
        while context.is_active():
            try:
                payload = self.runtime.laser_scan_payload(
                    topic=topic,
                    include_intensities=include_intensities,
                )
                yield self._laser_scan_response(payload, topic=topic)
            except Exception as exc:
                yield robot_api_pb2.LaserScanFrame(
                    ok=False,
                    error=str(exc),
                    robot_id=str(getattr(self.runtime, "robot_id", "") or ""),
                    topic=topic,
                )
            time.sleep(interval_sec)

    def GetSlamDefaults(self, request, context) -> robot_api_pb2.SlamDefaultsResponse:
        del request, context
        try:
            payload = self.runtime.slam_defaults_payload()
            return robot_api_pb2.SlamDefaultsResponse(
                ok=bool(payload.get("ok", True)),
                error=str(payload.get("error") or ""),
                params_json=json.dumps(payload.get("params") if isinstance(payload.get("params"), dict) else {}, ensure_ascii=False),
                params_path=str(payload.get("paramsPath") or payload.get("path") or ""),
            )
        except Exception as exc:
            return robot_api_pb2.SlamDefaultsResponse(ok=False, error=str(exc))

    def StartSlam(self, request, context) -> robot_api_pb2.SlamStateResponse:
        del context
        try:
            params = json_loads_object(request.params_json) if request.params_json else {}
            payload = self.runtime.start_slam(
                params,
                use_sim_time=bool(request.use_sim_time),
                command_id=str(request.command_id or ""),
            )
            return self._slam_state_response(payload)
        except Exception as exc:
            return robot_api_pb2.SlamStateResponse(ok=False, error=str(exc), state=self._slam_state_message({}))

    def GetSlamState(self, request, context) -> robot_api_pb2.SlamStateResponse:
        del request, context
        try:
            return self._slam_state_response(self.runtime.slam_state_payload())
        except Exception as exc:
            return robot_api_pb2.SlamStateResponse(ok=False, error=str(exc), state=self._slam_state_message({}))

    def WatchSlamMap(self, request, context):
        hz = max(0.2, min(5.0, float(request.hz or 1.0)))
        interval_sec = 1.0 / hz
        include_cells = bool(request.include_cells)
        while context.is_active():
            try:
                payload = self.runtime.slam_map_frame_payload(include_cells=include_cells)
                yield self._slam_map_frame_response(payload)
            except Exception as exc:
                yield robot_api_pb2.SlamMapFrame(
                    ok=False,
                    error=str(exc),
                    robot_id=str(getattr(self.runtime, "robot_id", "") or ""),
                    state=self._slam_state_message({}),
                )
            time.sleep(interval_sec)

    def FinishSlam(self, request, context) -> robot_api_pb2.SlamFinishResponse:
        del context
        try:
            payload = self.runtime.finish_slam(
                map_name=str(request.map_name or ""),
                activate=bool(request.activate),
                command_id=str(request.command_id or ""),
            )
            return self._slam_finish_response(payload)
        except Exception as exc:
            return robot_api_pb2.SlamFinishResponse(ok=False, error=str(exc), state=self._slam_state_message({}))

    def CancelSlam(self, request, context) -> robot_api_pb2.SlamStateResponse:
        del context
        try:
            payload = self.runtime.cancel_slam(
                reason=str(request.reason or ""),
                command_id=str(request.command_id or ""),
            )
            return self._slam_state_response(payload)
        except Exception as exc:
            return robot_api_pb2.SlamStateResponse(ok=False, error=str(exc), state=self._slam_state_message({}))

    def ListMaps(self, request, context) -> robot_api_pb2.ListMapsResponse:
        del request, context
        try:
            payload = self.runtime.list_maps_payload()
            response = robot_api_pb2.ListMapsResponse(
                ok=bool(payload.get("ok", True)),
                error=str(payload.get("error") or ""),
                active_map_name=str(payload.get("active") or payload.get("activeMapName") or ""),
                active_map_dir=str(payload.get("activeMapDir") or ""),
                active_map_id=str(payload.get("activeMapId") or ""),
            )
            for item in payload.get("maps", []) if isinstance(payload.get("maps"), list) else []:
                if not isinstance(item, dict):
                    continue
                response.maps.append(
                    robot_api_pb2.RobotMapInfo(
                        name=str(item.get("name") or ""),
                        folder=str(item.get("folder") or item.get("name") or ""),
                        map_dir=str(item.get("mapDir") or item.get("map_dir") or ""),
                        map_id=str(item.get("mapId") or item.get("map_id") or ""),
                        signature=str(item.get("signature") or ""),
                        active=bool(item.get("active")),
                    )
                )
            return response
        except Exception as exc:
            return robot_api_pb2.ListMapsResponse(ok=False, error=str(exc))

    def GetMapBundle(self, request, context) -> robot_api_pb2.MapBundleResponse:
        del context
        try:
            payload = self.runtime.pull_map_bundle_payload(str(request.map_name or ""))
            return robot_api_pb2.MapBundleResponse(
                ok=bool(payload.get("ok", True)),
                error=str(payload.get("error") or ""),
                map_name=str(payload.get("mapName") or request.map_name or ""),
                map_dir=str(payload.get("mapDir") or ""),
                map_id=str(payload.get("mapId") or ""),
                signature=str(payload.get("signature") or ""),
                bundle_json=json.dumps(payload, ensure_ascii=False),
            )
        except Exception as exc:
            return robot_api_pb2.MapBundleResponse(ok=False, error=str(exc), map_name=str(request.map_name or ""))

    def PutMapBundle(self, request, context) -> robot_api_pb2.MapBundleResponse:
        del context
        try:
            bundle = json_loads_object(request.bundle_json)
            result = self.runtime.push_map_bundle_payload(
                bundle,
                map_name=str(request.map_name or bundle.get("mapName") or ""),
                activate=bool(request.activate),
            )
            return self._map_response(result)
        except Exception as exc:
            return robot_api_pb2.MapBundleResponse(ok=False, error=str(exc), map_name=str(request.map_name or ""))

    def LoadMap(self, request, context) -> robot_api_pb2.MapBundleResponse:
        del context
        try:
            result = self.runtime.load_map(str(request.map_name or ""))
            return self._map_response(result)
        except Exception as exc:
            return robot_api_pb2.MapBundleResponse(ok=False, error=str(exc), map_name=str(request.map_name or ""))

    def GetParams(self, request, context) -> robot_api_pb2.ParamsResponse:
        del request, context
        try:
            payload = self.runtime.params_payload()
            return self._params_response(payload)
        except Exception as exc:
            return robot_api_pb2.ParamsResponse(ok=False, error=str(exc))

    def PutParams(self, request, context) -> robot_api_pb2.ParamsResponse:
        del context
        try:
            params = json_loads_object(request.params_json)
            payload = self.runtime.save_params_payload(params, reload_runtime=bool(request.reload_runtime))
            return self._params_response(payload)
        except Exception as exc:
            return robot_api_pb2.ParamsResponse(ok=False, error=str(exc))

    def _status_response(self) -> robot_api_pb2.StatusResponse:
        try:
            payload = self.runtime.status_payload()
            robot = payload.get("robot") if isinstance(payload.get("robot"), dict) else payload
            return robot_api_pb2.StatusResponse(ok=True, status=robot_status_from_json(robot))
        except Exception as exc:
            return robot_api_pb2.StatusResponse(ok=False, error=str(exc))

    def _command_response(self, result: dict[str, Any], *, command_id: str = "") -> robot_api_pb2.CommandResponse:
        status_payload = self.runtime.status_payload()
        robot = status_payload.get("robot") if isinstance(status_payload.get("robot"), dict) else status_payload
        route_payload = result.get("route") if isinstance(result.get("route"), dict) else {}
        return robot_api_pb2.CommandResponse(
            ok=bool(result.get("ok", True)),
            error=str(result.get("error") or ""),
            command_id=str(result.get("commandId") or result.get("command_id") or command_id or ""),
            route_json=json.dumps(route_payload, ensure_ascii=False) if route_payload else "",
            status=robot_status_from_json(robot),
        )

    def _map_response(self, result: dict[str, Any]) -> robot_api_pb2.MapBundleResponse:
        return robot_api_pb2.MapBundleResponse(
            ok=bool(result.get("ok", True)),
            error=str(result.get("error") or ""),
            map_name=str(result.get("mapName") or ""),
            map_dir=str(result.get("mapDir") or ""),
            map_id=str(result.get("mapId") or ""),
            signature=str(result.get("signature") or ""),
            bundle_json=str(result.get("bundleJson") or ""),
        )

    def _params_response(self, result: dict[str, Any]) -> robot_api_pb2.ParamsResponse:
        params = result.get("params") if isinstance(result.get("params"), dict) else {}
        return robot_api_pb2.ParamsResponse(
            ok=bool(result.get("ok", True)),
            error=str(result.get("error") or result.get("warning") or ""),
            params_json=json.dumps(params, ensure_ascii=False),
            params_path=str(result.get("path") or result.get("paramsPath") or ""),
            reloaded=bool(result.get("reloaded")),
        )

    def _laser_scan_response(self, result: dict[str, Any], *, topic: str) -> robot_api_pb2.LaserScanFrame:
        def float_list(items: Any) -> list[float]:
            if not isinstance(items, list):
                return []
            values: list[float] = []
            for item in items:
                try:
                    values.append(float(item))
                except (TypeError, ValueError):
                    values.append(float("nan"))
            return values

        return robot_api_pb2.LaserScanFrame(
            ok=bool(result.get("ok", True)),
            error=str(result.get("error") or ""),
            robot_id=str(result.get("robotId") or getattr(self.runtime, "robot_id", "") or ""),
            topic=str(result.get("topic") or topic),
            frame_id=str(result.get("frameId") or ""),
            stamp_sec=float(result.get("stampSec", 0.0) or 0.0),
            angle_min=float(result.get("angleMin", 0.0) or 0.0),
            angle_max=float(result.get("angleMax", 0.0) or 0.0),
            angle_increment=float(result.get("angleIncrement", 0.0) or 0.0),
            time_increment=float(result.get("timeIncrement", 0.0) or 0.0),
            scan_time=float(result.get("scanTime", 0.0) or 0.0),
            range_min=float(result.get("rangeMin", 0.0) or 0.0),
            range_max=float(result.get("rangeMax", 0.0) or 0.0),
            ranges=float_list(result.get("ranges")),
            intensities=float_list(result.get("intensities")),
        )

    def _slam_state_response(self, result: dict[str, Any]) -> robot_api_pb2.SlamStateResponse:
        return robot_api_pb2.SlamStateResponse(
            ok=bool(result.get("ok", True)),
            error=str(result.get("error") or ""),
            state=self._slam_state_message(result.get("state") if isinstance(result.get("state"), dict) else result),
        )

    def _slam_finish_response(self, result: dict[str, Any]) -> robot_api_pb2.SlamFinishResponse:
        return robot_api_pb2.SlamFinishResponse(
            ok=bool(result.get("ok", True)),
            error=str(result.get("error") or ""),
            state=self._slam_state_message(result.get("state") if isinstance(result.get("state"), dict) else result),
            map_name=str(result.get("mapName") or ""),
            map_dir=str(result.get("mapDir") or ""),
            map_id=str(result.get("mapId") or ""),
            signature=str(result.get("signature") or ""),
            bundle_json=str(result.get("bundleJson") or ""),
        )

    def _slam_state_message(self, state: dict[str, Any] | None) -> robot_api_pb2.SlamState:
        source = state if isinstance(state, dict) else {}
        return robot_api_pb2.SlamState(
            active=bool(source.get("active")),
            state=str(source.get("state") or "idle"),
            message=str(source.get("message") or ""),
            session_id=str(source.get("sessionId") or source.get("session_id") or ""),
            started_at_sec=float(source.get("startedAtSec") or source.get("started_at_sec") or 0.0),
            progress=int(source.get("progress") or 0),
            saved_map_name=str(source.get("savedMapName") or source.get("saved_map_name") or ""),
            map_dir=str(source.get("mapDir") or source.get("map_dir") or ""),
            map_width=int(source.get("mapWidth") or source.get("map_width") or 0),
            map_height=int(source.get("mapHeight") or source.get("map_height") or 0),
            resolution=float(source.get("resolution") or 0.0),
            frame_id=str(source.get("frameId") or source.get("frame_id") or ""),
            trail_points=int(source.get("trailPoints") or source.get("trail_points") or 0),
        )

    def _pose2d_message(self, pose: dict[str, Any] | None) -> robot_api_pb2.Pose2D:
        source = pose if isinstance(pose, dict) else {}
        return robot_api_pb2.Pose2D(
            x=float(source.get("x") or 0.0),
            y=float(source.get("y") or 0.0),
            yaw=float(source.get("yaw") or source.get("theta") or 0.0),
            stamp_sec=float(source.get("stampSec") or source.get("stamp_sec") or 0.0),
        )

    def _slam_map_frame_response(self, result: dict[str, Any]) -> robot_api_pb2.SlamMapFrame:
        frame = robot_api_pb2.SlamMapFrame(
            ok=bool(result.get("ok", True)),
            error=str(result.get("error") or ""),
            robot_id=str(result.get("robotId") or getattr(self.runtime, "robot_id", "") or ""),
            session_id=str(result.get("sessionId") or ""),
            frame_id=str(result.get("frameId") or ""),
            stamp_sec=float(result.get("stampSec") or 0.0),
            width=int(result.get("width") or 0),
            height=int(result.get("height") or 0),
            resolution=float(result.get("resolution") or 0.0),
            origin_x=float(result.get("originX") or 0.0),
            origin_y=float(result.get("originY") or 0.0),
            origin_yaw=float(result.get("originYaw") or 0.0),
            cells=bytes(result.get("cells") or b""),
            pose=self._pose2d_message(result.get("pose") if isinstance(result.get("pose"), dict) else {}),
            state=self._slam_state_message(result.get("state") if isinstance(result.get("state"), dict) else {}),
        )
        for pose in result.get("trail", []) if isinstance(result.get("trail"), list) else []:
            if isinstance(pose, dict):
                frame.trail.append(self._pose2d_message(pose))
        return frame


def serve_robot_api(runtime: Any, *, host: str = "0.0.0.0", port: int = 50051, max_workers: int = 8):
    grpc, robot_api_pb2_grpc = _load_grpc_modules()
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=max(2, int(max_workers))),
        options=GRPC_CHANNEL_OPTIONS,
    )
    robot_api_pb2_grpc.add_RobotApiServicer_to_server(RobotApiService(runtime), server)
    bind = f"{host}:{int(port)}"
    actual_port = server.add_insecure_port(bind)
    if actual_port == 0:
        raise RuntimeError(f"failed to bind robot gRPC API on {bind}")
    server.start()
    return server
