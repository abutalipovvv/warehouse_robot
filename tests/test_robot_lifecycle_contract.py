import json
from pathlib import Path

from operator_app.core.models import KnownRobot
from operator_app.core.state import OperatorAppState
from fleet_manager.runtime.grpc.api.contracts import robot_status_from_json
from operator_app.core.grpc.contracts import robot_status_to_json
from fleet_manager.runtime.grpc.api.proto import robot_api_pb2, robot_api_pb2_grpc
from fleet_manager.runtime.grpc.api.server import GRPC_SERVER_OPTIONS, RobotApiService


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OPERATOR_STATIC_ROOT = PROJECT_ROOT / "operator_app" / "web" / "static"


def _operator_app_source() -> str:
    module_root = OPERATOR_STATIC_ROOT / "js" / "app"
    paths = [OPERATOR_STATIC_ROOT / "app.js", *sorted(module_root.glob("*.js"))]
    return "\n".join(path.read_text(encoding="utf-8") for path in paths)


def test_lifecycle_rpc_methods_are_in_client_contract() -> None:
    assert hasattr(robot_api_pb2, "ControlRequest")
    assert hasattr(robot_api_pb2, "RelocateRequest")
    assert hasattr(robot_api_pb2, "PauseRouteRequest")
    assert hasattr(robot_api_pb2, "ResumeRouteRequest")
    assert "AcquireControl" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert "Relocate" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert "PauseRoute" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert "ResumeRoute" in robot_api_pb2.DESCRIPTOR.services_by_name["RobotApi"].methods_by_name
    assert hasattr(robot_api_pb2_grpc.RobotApiServicer, "AcquireControl")
    assert hasattr(robot_api_pb2_grpc.RobotApiServicer, "ReleaseControl")
    assert hasattr(robot_api_pb2_grpc.RobotApiServicer, "ConfirmLocalization")


def test_grpc_health_is_ready_while_ros_runtime_initializes() -> None:
    class StartingRuntime:
        available = False
        error = ""

    response = RobotApiService(StartingRuntime()).Health(None, None)

    assert response.ok is True
    assert response.error == ""


def test_grpc_health_reports_ros_runtime_initialization_failure() -> None:
    class FailedRuntime:
        available = False
        error = "ROS2 runtime failed"

    response = RobotApiService(FailedRuntime()).Health(None, None)

    assert response.ok is False
    assert response.error == "ROS2 runtime failed"


def test_robot_api_disables_grpc_port_reuse_for_single_control_owner() -> None:
    assert ("grpc.so_reuseport", 0) in GRPC_SERVER_OPTIONS

    standalone_server = (
        PROJECT_ROOT
        / "robot/robot_driver/src/robot_grpc_api/robot_grpc_api/server.py"
    ).read_text(encoding="utf-8")
    assert '("grpc.so_reuseport", 0)' in standalone_server


def test_status_roundtrip_preserves_lifecycle_fields_in_raw_json() -> None:
    payload = {
        "robotId": "robot1",
        "connected": True,
        "state": "PAUSED",
        "controlState": "OWNED",
        "controlOwner": "operator-app",
        "control": {"state": "OWNED", "ownerId": "operator-app", "ownerName": "Operator App"},
        "navigationPaused": True,
        "localizationConfirmed": False,
    }
    status = robot_status_from_json(payload)
    decoded = robot_status_to_json(status)
    assert decoded["controlOwner"] == "operator-app"
    assert decoded["control"]["ownerName"] == "Operator App"
    assert decoded["navigationPaused"] is True
    assert decoded["localizationConfirmed"] is False


def test_operator_seize_forces_takeover_and_stops_previous_route() -> None:
    state = OperatorAppState.__new__(OperatorAppState)
    adapter = _TakeoverAdapter()
    state.grpc_adapter = adapter
    robot = KnownRobot(id="robot1", name="robot1", host="127.0.0.1", port=50051)
    state.get_robot = lambda robot_id: robot

    status, _, body = state._proxy_grpc_robot_request(
        "robot1",
        "POST",
        "/api/robot/control/acquire",
        body=json.dumps({"force": True, "stopNavigation": True}).encode("utf-8"),
    )
    payload = json.loads(body.decode("utf-8"))

    assert status == 200
    assert payload["navigationStopped"] is True
    assert payload["status"]["robot"]["state"] == "IDLE"
    assert adapter.calls == [
        ("acquire", "grpc://127.0.0.1:50051", "operator-app", True),
        ("stop", "grpc://127.0.0.1:50051", "operator-app"),
    ]


def test_operator_ui_uses_exclusive_control_and_graph_safe_fleet_pose() -> None:
    app_js = _operator_app_source()
    styles = (OPERATOR_STATIC_ROOT / "styles.css").read_text(encoding="utf-8")
    index_html = (OPERATOR_STATIC_ROOT / "index.html").read_text(encoding="utf-8")

    assert 'acquireRobotControl(true, true)' in app_js
    assert 'stopNavigation: true' in app_js
    assert '? this.releaseFleetRobotControl()' in app_js
    assert ': this.acquireFleetRobotControl()' in app_js
    assert '? this.releaseRobotControl()' in app_js
    assert ': this.acquireRobotControl(true, true)' in app_js
    assert 'this.fleetApiPath("/robots/control/acquire")' in app_js
    assert 'this.fleetApiPath("/robots/control/release")' in app_js
    assert 'remoteStatus === "object"' in app_js
    assert 'id="controlToggleButton"' in index_html
    assert 'id="takeControlButton"' not in index_html
    assert 'id="releaseControlButton"' not in index_html
    assert 'addEventListener("click", () => this.toggleControl())' in app_js
    assert 'classList.toggle("control-state-owned", ownsControl)' in app_js
    assert 'classList.toggle("control-state-free", !control.ownerId)' in app_js
    assert "this.controlToggleButton.disabled = mappingActive;" in app_js
    assert "Click to take control." in app_js
    assert "#controlToggleButton.control-state-owned" in styles
    assert "#controlToggleButton.control-state-free" in styles
    assert 'Navigation blocked. Press Seize Control first.' in app_js
    assert 'body[data-fleet-page="fleet"] #driveActionGroup' in styles
    assert 'async startFleetPoseNavigation(world)' in app_js
    assert 'await this.startFleetNavigation(nearest.landmark.name' in app_js
    assert 'return Boolean(this.targetFleetRobot());' in app_js
    assert 'incomingUpdatedAt + 0.000001 < priorUpdatedAt' in app_js
    assert 'this.scheduleAdaptiveMapLayers();' in app_js


def test_all_static_map_views_and_editors_use_babylon_2d() -> None:
    app_js = _operator_app_source()
    editor_js = (OPERATOR_STATIC_ROOT / "map-editor.js").read_text(encoding="utf-8")
    editor_html = (OPERATOR_STATIC_ROOT / "map-editor.html").read_text(encoding="utf-8")
    index_html = (OPERATOR_STATIC_ROOT / "index.html").read_text(encoding="utf-8")
    scene_js = (OPERATOR_STATIC_ROOT / "scene3d.js").read_text(encoding="utf-8")

    assert "renderOperatorBabylonMap()" in app_js
    assert "Promise.allSettled([" in app_js
    assert 'this.scene3d?.setViewMode(show3d ? "3d" : "2d");' in app_js
    assert "this.operatorMapSvg?.classList.toggle(\"hidden\", useBabylon);" in app_js
    assert 'id="editorBabylon"' in editor_html
    assert 'import("./scene3d.js")' in editor_js
    assert 'scene.setViewMode("2d");' in editor_js
    assert "renderBabylonCanvas(options = {})" in editor_js
    assert 'this.editorSvg.classList.add("hidden");' in editor_js
    assert 'id="operatorLmNamesButton"' in index_html
    assert 'id="operatorEdgeDirectionsButton"' in index_html
    assert 'id="lmNamesButton"' in editor_html
    assert 'id="edgeDirectionsButton"' in editor_html
    assert "setLandmarkLabelsVisible(visible)" in scene_js
    assert "setEdgeDirectionsVisible(visible)" in scene_js
    assert "this.addEdgeDirections(edges);" in scene_js
    assert "const MAP_TEXTURE_INVERT_Y = false;" in scene_js
    assert "candidates.length >= 320" not in scene_js
    assert 'preferences.setBoolean("lmNamesVisible"' in app_js
    assert 'preferences.setBoolean("lmNamesVisible"' in editor_js
    assert 'preferences.setBoolean("edgeDirectionsVisible"' in app_js
    assert 'preferences.setBoolean("edgeDirectionsVisible"' in editor_js
    assert 'type="module" src="app.js"' in index_html
    assert 'type="module" src="map-editor.js"' in editor_html
    assert 'data-tool="brush"' in editor_html
    assert 'data-tool="eraser"' in editor_html
    assert 'data-tool="fill"' in editor_html
    assert 'data-tool="corridor"' in editor_html
    assert 'data-fleet-map-tool="brush"' in index_html
    assert 'data-fleet-map-tool="corridor"' in index_html
    assert "syncFleetRasterPayload()" in app_js
    assert "markControlledCorridor" in app_js
    assert "OccupancyGrid" in editor_js
    assert "setFloorCanvas(canvas)" in scene_js


def test_operator_map_context_and_edge_controls_are_explicit() -> None:
    app_js = _operator_app_source()
    editor_js = (OPERATOR_STATIC_ROOT / "map-editor.js").read_text(encoding="utf-8")
    editor_html = (OPERATOR_STATIC_ROOT / "map-editor.html").read_text(encoding="utf-8")
    index_html = (OPERATOR_STATIC_ROOT / "index.html").read_text(encoding="utf-8")

    for html in (index_html, editor_html):
        assert 'value="one_way"' in html
        assert 'value="reverse"' in html
        assert 'value="bidirectional"' in html
        assert 'value="-1"' in html
        assert 'value="0"' in html
        assert 'value="1"' in html

    assert "selectionGeneration" in app_js
    assert "selectionIsCurrent(context)" in app_js
    assert "pendingRobotMapsRobotId" in app_js
    assert "refreshInitialWorkspaceInBackground()" in app_js
    assert "workspaceTransitionUntil" in app_js
    assert "createFleetRobotListRow(robotName)" in app_js
    assert 'dataset.managerId = String(payload.managerId || "")' in app_js
    assert "renderInteraction(options = {})" in editor_js
    assert "normalizeEdgeMotionCode" in app_js
    assert "normalizeEdgeMotionCode" in editor_js
    assert 'id="fleetEditorApplyLmButton"' not in index_html
    assert 'id="fleetEditorApplyEdgeButton"' not in index_html
    assert 'input?.addEventListener("change", () => this.applyFleetEditorLmFields())' in app_js
    assert '() => this.applyFleetEditorEdgeFields()' in app_js


def test_map_context_switch_aborts_stale_load_and_bounds_retry() -> None:
    app_js = _operator_app_source()

    assert "mapContextRequestIsCurrent(context)" in app_js
    assert "fetchMapContextWithRetry" in app_js
    assert "this.mapContextAbortController?.abort();" in app_js
    assert "context.requestId === this.mapContextActiveRequestId" in app_js
    assert "Map context request timed out after" in app_js
    assert "this.getJson(url, { signal: controller.signal })" in app_js


def test_map_editor_raster_history_is_atomic_and_has_consistent_shortcuts() -> None:
    app_js = _operator_app_source()
    editor_js = (OPERATOR_STATIC_ROOT / "map-editor.js").read_text(encoding="utf-8")
    index_html = (OPERATOR_STATIC_ROOT / "index.html").read_text(encoding="utf-8")
    editor_html = (OPERATOR_STATIC_ROOT / "map-editor.html").read_text(encoding="utf-8")
    command_stack_js = (
        OPERATOR_STATIC_ROOT / "js" / "editor" / "command-stack.js"
    ).read_text(encoding="utf-8")

    undo_body = command_stack_js[
        command_stack_js.index("  undo() {"):
        command_stack_js.index("  redo() {")
    ]
    redo_body = command_stack_js[
        command_stack_js.index("  redo() {"):
        command_stack_js.index("  notify() {")
    ]
    assert undo_body.index("command.undo();") < undo_body.index("this.undoCommands.pop();")
    assert redo_body.index("command.redo();") < redo_body.index("this.redoCommands.pop();")

    assert "this.fleetMapEditorActive" in app_js
    assert '&& (key === "z" || key === "y")' in app_js
    assert "if (this.fleetRasterDrag) {" in app_js
    assert 'if (key === "y" || event.shiftKey)' in app_js
    assert '["raster_stroke", "raster_rectangle"].includes(this.dragState?.type)' in editor_js
    assert 'title="Undo (Ctrl+Z)"' in index_html
    assert 'title="Redo (Ctrl+Shift+Z or Ctrl+Y)"' in index_html
    assert 'title="Redo (Ctrl+Shift+Z or Ctrl+Y)"' in editor_html


def test_operator_workspace_navigation_and_fleet_sidebar_contract() -> None:
    app_js = _operator_app_source()
    styles_css = (OPERATOR_STATIC_ROOT / "styles.css").read_text(encoding="utf-8")

    assert 'button.addEventListener("dblclick"' in app_js
    assert "selectRobot({ enterWorkspace: false })" in app_js
    card_selection = app_js[
        app_js.index("const selectRobot = async")
        : app_js.index('button.addEventListener("click"', app_js.index("const selectRobot = async"))
    ]
    assert card_selection.index("if (options.home && !enterWorkspace)") < (
        card_selection.index("this.renderSelectedRobot();")
    )
    assert "this.syncRobotCardSelection();" in card_selection
    assert "this.workspaceLoadingMinimumMs = 800;" in app_js
    assert "this.workspaceLoadingMinimumMs - (performance.now() - workspaceLoadingStartedAt)" in app_js
    assert "await this.navigateHomePage({ force: selectionChanged });" in app_js
    assert 'this.fleetActiveTab === "fleet"' in app_js
    assert "returningFromMapEditor" in app_js
    assert "if (this.fleetMapEditorActive) {\n      return [];" in app_js
    assert "if (!this.isFleetManager() || this.fleetMapEditorActive)" in app_js
    assert "fleetBenchmarkMetricModel(result, robotCount)" in app_js
    assert "renderFleetBenchmarkSummary(result, robotCount)" in app_js
    assert ".fleet-benchmark-metrics" in styles_css
    assert ".operator-console.fleet-page-fleet #fleetTabFleet .fleet-queue-block" in styles_css
    assert "Selected · double-click to open" in styles_css


def test_babylon_2d_uses_robot_model_footprint_and_future_route() -> None:
    app_js = _operator_app_source()
    scene_js = (OPERATOR_STATIC_ROOT / "scene3d.js").read_text(encoding="utf-8")

    assert "robotModelFootprint()" in app_js
    assert "footprint: this.robotModelFootprint()" in app_js
    assert 'if (this.viewMode === "2d") {' in scene_js
    assert "this.addFootprintModel(group, robot, active);" in scene_js
    assert "addFootprintModel(group, robot, active)" in scene_js
    assert "this.addEcomModel(group, robot, active);" in scene_js
    assert "futureRobotTrajectory(robot, active)" in scene_js
    assert 'this.viewMode === "2d" ? 0.94 : 0.82' in scene_js
    assert "occupancyWallRectanglesFromImageData(" in scene_js
    assert "this.ensureOccupancyWalls();" in scene_js
    assert "await this.buildOccupancyWalls(" in scene_js
    assert "new Worker(OCCUPANCY_WALL_WORKER_URL" in scene_js
    assert "mesh.thinInstanceSetBuffer(\"matrix\", matrices, 16, true);" in scene_js
    assert "matrices[offset + 12] = Number(wall.x || 0);" in scene_js
    assert 'new B.StandardMaterial("warehouse-wall-material"' in scene_js


class _TakeoverAdapter:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def acquire_control(
        self,
        endpoint: str,
        *,
        owner_id: str,
        owner_name: str,
        force: bool,
        lease_ms: int,
    ) -> dict:
        del owner_name, lease_ms
        self.calls.append(("acquire", endpoint, owner_id, force))
        return {
            "ok": True,
            "status": {"ok": True, "robot": {"state": "EXECUTING_ROUTE"}},
        }

    def stop(self, endpoint: str, *, owner_id: str) -> dict:
        self.calls.append(("stop", endpoint, owner_id))
        return {
            "ok": True,
            "status": {"ok": True, "robot": {"state": "IDLE"}},
        }
