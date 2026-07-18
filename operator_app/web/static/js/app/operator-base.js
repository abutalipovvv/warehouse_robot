import { CommandStack } from "../editor/command-stack.js";
import { preferences } from "../state/preferences.js";
import { FLEET_RASTER_TOOLS } from "./constants.js";
import { FleetRobotModelEditor } from "./robot-model-editor.js";


export class OperatorAppBase {
  constructor() {
    this.fleetManagerId = "__fleet_manager__";
    this.fleetManagerSimId = "__fleet_manager_sim__";
    this.robots = [];
    this.selectedRobotId = preferences.getString("selectedRobotId");
    this.lastFleetManagerId = preferences.getString("lastFleetManagerId");
    this.selectionGeneration = 0;
    this.appBooting = true;
    this.workspaceLoadingRobotId = "";
    this.workspaceLoadingMinimumMs = 800;
    this.workspaceTransitionRobotId = "";
    this.workspaceTransitionUntil = 0;
    this.mapContentLoadingRobotId = "";
    this.selectedFleetRobotName = preferences.getString("selectedFleetRobotName");
    this.fleetSelectionCleared = false;
    this.lastProbe = null;
    this.sidebarOpen = false;
    this.pendingRobotMaps = [];
    this.pendingRobotMapsRobotId = "";
    this.operatorMapPayload = null;
    this.operatorMapSignature = "";
    this.currentStatus = null;
    this.currentRoute = null;
    this.statusRequestPending = false;
    this.robotPingRefreshPending = false;
    this.navigateMode = false;
    this.relocateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.fleetQueue = [];
    this.fleetQueueSequence = 0;
    this.fleetBenchmarkRunId = 0;
    this.lastFleetPlanDebug = null;
    this.selectedFleetOrderId = "";
    this.fleetParams = null;
    this.fleetParamsLoaded = false;
    this.fleetParamsManagerId = "";
    this.robotParams = null;
    this.robotParamsRobotId = "";
    this.robotParamsLoaded = false;
    this.fleetNameEdited = false;
    this.fleetTickPending = false;
    this.mapViewMode = preferences.getString("mapViewMode", "2d");
    this.lmNamesVisible = preferences.getBoolean("lmNamesVisible", false);
    this.edgeDirectionsVisible = preferences.getBoolean("edgeDirectionsVisible", true);
    this.scene3dModulePromise = null;
    this.scene3d = null;
    this.scene3dStaticKey = "";
    this.scene3dPayload = null;
    this.scene3dHoverLmName = "";
    this.scene3dLoadPending = false;
    this.scene3dRenderQueued = false;
    this.babylonMapFailed = false;
    this.babylonMapRevision = 0;
    this.fleetStatusSocket = null;
    this.fleetStatusManagerId = "";
    this.fleetStatusStreamShouldRun = false;
    this.fleetStatusReconnectTimer = null;
    this.fleetStatusReconnectMs = 500;
    this.fleetStatusStreamAttemptedAt = 0;
    this.fleetStatusStreamFallback = false;
    this.fleetHttpFallbackLastAt = 0;
    // Compact pose/status deltas arrive at 20 Hz. Physics is intentionally
    // slower; requestAnimationFrame advances a short, bounded portion of the
    // already committed trajectory between confirmed server clocks.
    this.fleetStreamIntervalMs = 50;
    this.fleetRuntimeUiLastAt = 0;
    this.robotStatusSocket = null;
    this.robotStatusStreamShouldRun = false;
    this.robotStatusReconnectTimer = null;
    this.robotStatusReconnectMs = 500;
    this.robotStatusStreamAttemptedAt = 0;
    this.robotStatusStreamFallback = false;
    this.robotStreamIntervalMs = 180;
    this.robotStatusReceivedAt = 0;
    this.robotStatusFreshTimeoutMs = 2200;
    this.robotStatusRobotId = "";
    this.scanSocket = null;
    this.scanRobotId = "";
    this.scanEnabled = false;
    this.latestScanFrame = null;
    this.slamSocket = null;
    this.slamRobotId = "";
    this.slamActive = false;
    this.slamState = null;
    this.slamMapPayload = null;
    this.slamMapFrame = null;
    this.slamDefaults = null;
    this.teleopSocket = null;
    this.teleopRobotId = "";
    this.fleetStatusReceivedAt = 0;
    this.fleetStatusFreshTimeoutMs = 1500;
    this.fleetStatusObjectRef = null;
    this.fleetAnimationFrame = null;
    this.fleetAnimationLastAt = 0;
    this.fleetVisualControlLastAt = 0;
    this.fleetRouteRenderLastAt = 0;
    this.fleet2dMotionLastAt = 0;
    this.fleetRuntimeLandmarkKey = "";
    this.fleetVisualClocks = new Map();
    this.fleetNavigationPredictionMaxSec = 0.4;
    this.fleetRobotSvgEntries = new Map();
    this.fleetWaitDependencyLine = null;
    this.fleetManualRobotName = "";
    this.fleetManualLastAt = 0;
    this.fleetManualLookahead = null;
    this.fleetManualAnimation = null;
    this.mapSyncDecisionResolve = null;
    this.mapTransferCloseTimer = null;
    this.fleetActiveTab = this.pageForPath(window.location.pathname);
    this.initialFleetRouteSelectionPending = true;
    this.fleetMapEditorActive = false;
    this.fleetMapTool = "select";
    this.fleetMapDraft = null;
    this.fleetMapDirty = false;
    this.fleetMapExitResolve = null;
    this.fleetMapSaveAsResolve = null;
    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.fleetEditorEdgeDrag = null;
    this.fleetEditorLmDrag = null;
    this.fleetEditorBezierDrag = null;
    this.fleetEditorPreview = null;
    this.fleetEditorGuideWorld = null;
    this.fleetCorridorStartLm = "";
    this.fleetRasterGrid = null;
    this.fleetRasterDraftRef = null;
    this.fleetRasterLoadPromise = null;
    this.fleetRasterDrag = null;
    this.fleetRasterPreviewTimer = 0;
    this.fleetRasterHistory = new CommandStack(100);
    this.fleetRasterHistory.onChange = () => this.syncFleetRasterControls();
    this.fleetEditorFieldSyncing = false;
    this.fleetModelEditor = null;
    this.mapDrag = null;
    this.mapAdaptiveLayerTimer = null;
    this.relocationDrag = null;
    this.babylonRelocationDrag = null;
    this.mapClickConsumed = false;
    this.manualKeys = new Set();
    this.teleopPending = false;
    this.fleetSimManualFrame = null;
    this.fleetSimManualLastAt = 0;
    this.fleetSimManualGeneration = 0;
    this.mapView = {
      scale: 1,
      tx: 0,
      ty: 0,
      follow: true,
    };
    this.robotMapState = this.emptyMapState();

    this.robotsList = document.getElementById("robotsList");
    this.robotCountText = document.getElementById("robotCountText");
    this.homePage = document.getElementById("homePage");
    this.homeRobotGrid = document.getElementById("homeRobotGrid");
    this.homeRobotCountText = document.getElementById("homeRobotCountText");
    this.homeRefreshButton = document.getElementById("homeRefreshButton");
    this.homeAddRobotButton = document.getElementById("homeAddRobotButton");
    this.sidebarDrawer = document.getElementById("sidebarDrawer");
    this.sidebarBackdrop = document.getElementById("sidebarBackdrop");
    this.globalHomeButton = document.getElementById("globalHomeButton");
    this.homeButton = document.getElementById("homeButton");
    this.paramsNavButton = document.getElementById("paramsNavButton");
    this.mapEditorNavButton = document.getElementById("mapEditorNavButton");
    this.robotModelNavButton = document.getElementById("robotModelNavButton");
    this.openSidebarButton = document.getElementById("openSidebarButton");
    this.closeSidebarButton = document.getElementById("closeSidebarButton");
    this.emptyState = document.getElementById("emptyState");
    this.workspaceLoadingState = document.getElementById("workspaceLoadingState");
    this.workspaceLoadingTitle = document.getElementById("workspaceLoadingTitle");
    this.workspaceLoadingText = document.getElementById("workspaceLoadingText");
    this.robotView = document.getElementById("robotView");
    this.robotWorkspaceTitle = document.getElementById("robotWorkspaceTitle");
    this.robotActiveMapText = document.getElementById("robotActiveMapText");
    this.operatorActiveMapText = document.getElementById("operatorActiveMapText");
    this.robotStateText = document.getElementById("robotStateText");
    this.robotControlText = document.getElementById("robotControlText");
    this.nearestLmText = document.getElementById("nearestLmText");
    this.driveActionGroup = document.getElementById("driveActionGroup");
    this.localizationActionGroup = document.getElementById("localizationActionGroup");
    this.navigationActionGroup = document.getElementById("navigationActionGroup");
    this.visualActionGroup = document.getElementById("visualActionGroup");
    this.mapActionGroup = document.getElementById("mapActionGroup");
    this.slamActionGroup = document.getElementById("slamActionGroup");
    this.navigateRobotButton = document.getElementById("navigateRobotButton");
    this.takeControlButton = document.getElementById("takeControlButton");
    this.releaseControlButton = document.getElementById("releaseControlButton");
    this.relocateRobotButton = document.getElementById("relocateRobotButton");
    this.pauseRouteButton = document.getElementById("pauseRouteButton");
    this.resumeRouteButton = document.getElementById("resumeRouteButton");
    this.cancelRouteButton = document.getElementById("cancelRouteButton");
    this.stopRobotButton = document.getElementById("stopRobotButton");
    this.scanToggleButton = document.getElementById("scanToggleButton");
    this.controlPullMapButton = document.getElementById("controlPullMapButton");
    this.controlPushMapButton = document.getElementById("controlPushMapButton");
    this.controlLoadMapButton = document.getElementById("controlLoadMapButton");
    this.startSlamButton = document.getElementById("startSlamButton");
    this.doneSlamButton = document.getElementById("doneSlamButton");
    this.cancelSlamButton = document.getElementById("cancelSlamButton");
    this.mapSyncStatus = document.getElementById("mapSyncStatus");
    this.operatorConsole = document.getElementById("operatorConsole");
    this.fleetControlPanel = document.getElementById("fleetControlPanel");
    this.robotParamsPanel = document.getElementById("robotParamsPanel");
    this.robotParamsSummary = document.getElementById("robotParamsSummary");
    this.robotParamsTable = document.getElementById("robotParamsTable");
    this.robotParamsJsonInput = document.getElementById("robotParamsJsonInput");
    this.robotReloadParamsButton = document.getElementById("robotReloadParamsButton");
    this.robotFormatParamsButton = document.getElementById("robotFormatParamsButton");
    this.robotDefaultsParamsButton = document.getElementById("robotDefaultsParamsButton");
    this.robotSaveParamsButton = document.getElementById("robotSaveParamsButton");
    this.robotModelPanel = document.getElementById("robotModelPanel");
    this.fleetRobotNameLabel = document.getElementById("fleetRobotNameLabel");
    this.fleetRobotNameInput = document.getElementById("fleetRobotNameInput");
    this.fleetSpawnLmLabel = document.getElementById("fleetSpawnLmLabel");
    this.fleetSpawnLmLabelText = document.getElementById("fleetSpawnLmLabelText");
    this.fleetSpawnLmSelect = document.getElementById("fleetSpawnLmSelect");
    this.fleetRobotApiLabel = document.getElementById("fleetRobotApiLabel");
    this.fleetRobotApiInput = document.getElementById("fleetRobotApiInput");
    this.fleetAddRobotButton = document.getElementById("fleetAddRobotButton");
    this.fleetPlaceRobotButton = document.getElementById("fleetPlaceRobotButton");
    this.fleetSimTools = document.getElementById("fleetSimTools");
    this.fleetBenchmarkButtons = Array.from(document.querySelectorAll("[data-fleet-benchmark-count]"));
    this.fleetBenchmarkPlanButton = document.getElementById("fleetBenchmarkPlanButton");
    this.fleetBenchmarkPackageButton = document.getElementById("fleetBenchmarkPackageButton");
    this.fleetBenchmarkHorizonInput = document.getElementById("fleetBenchmarkHorizonInput");
    this.fleetBenchmarkIntervalInput = document.getElementById("fleetBenchmarkIntervalInput");
    this.fleetSimulationTimeScaleSelect = document.getElementById("fleetSimulationTimeScaleSelect");
    this.fleetBenchmarkClearButton = document.getElementById("fleetBenchmarkClearButton");
    this.fleetBenchmarkStatus = document.getElementById("fleetBenchmarkStatus");
    this.fleetRobotList = document.getElementById("fleetRobotList");
    this.fleetQueueGoalButton = document.getElementById("fleetQueueGoalButton");
    this.fleetStartQueueButton = document.getElementById("fleetStartQueueButton");
    this.fleetClearQueueButton = document.getElementById("fleetClearQueueButton");
    this.fleetQueueList = document.getElementById("fleetQueueList");
    this.fleetOrderDetails = document.getElementById("fleetOrderDetails");
    this.fleetPauseOrderButton = document.getElementById("fleetPauseOrderButton");
    this.fleetResumeOrderButton = document.getElementById("fleetResumeOrderButton");
    this.fleetCancelOrderButton = document.getElementById("fleetCancelOrderButton");
    this.fleetPlanDebug = document.getElementById("fleetPlanDebug");
    this.fleetRouteSpeedInput = document.getElementById("fleetRouteSpeedInput");
    this.fleetRouteAccelerationInput = document.getElementById("fleetRouteAccelerationInput");
    this.fleetRotateInput = document.getElementById("fleetRotateInput");
    this.fleetTurnSpeedInput = document.getElementById("fleetTurnSpeedInput");
    this.fleetRobotClearanceInput = document.getElementById("fleetRobotClearanceInput");
    this.fleetManualLinearInput = document.getElementById("fleetManualLinearInput");
    this.fleetManualAngularInput = document.getElementById("fleetManualAngularInput");
    this.fleetManualLookaheadInput = document.getElementById("fleetManualLookaheadInput");
    this.fleetManualStepInput = document.getElementById("fleetManualStepInput");
    this.fleetSaveParamsButton = document.getElementById("fleetSaveParamsButton");
    this.fleetParamsJsonInput = document.getElementById("fleetParamsJsonInput");
    this.fleetReloadParamsButton = document.getElementById("fleetReloadParamsButton");
    this.fleetFormatParamsButton = document.getElementById("fleetFormatParamsButton");
    this.fleetSaveJsonParamsButton = document.getElementById("fleetSaveJsonParamsButton");
    this.fleetTabButtons = Array.from(document.querySelectorAll("[data-fleet-tab]"));
    this.fleetTabFleet = document.getElementById("fleetTabFleet");
    this.fleetTabParams = document.getElementById("fleetTabParams");
    this.fleetTabModel = document.getElementById("robotModelPanel");
    this.fleetTabMap = document.getElementById("fleetTabMap");
    this.fleetRobotModelSvg = document.getElementById("fleetRobotModelSvg");
    this.fleetFootprintFields = document.getElementById("fleetFootprintFields");
    this.fleetTfFields = document.getElementById("fleetTfFields");
    this.fleetModelZoomInButton = document.getElementById("fleetModelZoomInButton");
    this.fleetModelZoomOutButton = document.getElementById("fleetModelZoomOutButton");
    this.fleetModelResetViewButton = document.getElementById("fleetModelResetViewButton");
    this.fleetModelResetButton = document.getElementById("fleetModelResetButton");
    this.fleetModelSaveButton = document.getElementById("fleetModelSaveButton");
    this.fleetMapToolButtons = Array.from(document.querySelectorAll("[data-fleet-map-tool]"));
    this.fleetMapEditorHelp = document.getElementById("fleetMapEditorHelp");
    this.fleetEditorLmNameInput = document.getElementById("fleetEditorLmNameInput");
    this.fleetEditorLmXInput = document.getElementById("fleetEditorLmXInput");
    this.fleetEditorLmYInput = document.getElementById("fleetEditorLmYInput");
    this.fleetEditorEdgeFromInput = document.getElementById("fleetEditorEdgeFromInput");
    this.fleetEditorEdgeToInput = document.getElementById("fleetEditorEdgeToInput");
    this.fleetEditorEdgeTrafficSelect = document.getElementById("fleetEditorEdgeTrafficSelect");
    this.fleetEditorEdgeMotionSelect = document.getElementById("fleetEditorEdgeMotionSelect");
    this.fleetRasterBrushSizeInput = document.getElementById("fleetRasterBrushSizeInput");
    this.fleetRasterBrushSizeOutput = document.getElementById("fleetRasterBrushSizeOutput");
    this.fleetRasterUndoButton = document.getElementById("fleetRasterUndoButton");
    this.fleetRasterRedoButton = document.getElementById("fleetRasterRedoButton");
    this.fleetMapSaveButton = document.getElementById("fleetMapSaveButton");
    this.fleetMapSaveAsButton = document.getElementById("fleetMapSaveAsButton");
    this.fleetMapReloadButton = document.getElementById("fleetMapReloadButton");
    this.fleetMapCloseButton = document.getElementById("fleetMapCloseButton");
    this.fleetMapDirtyState = document.getElementById("fleetMapDirtyState");
    this.refreshButton = document.getElementById("refreshButton");
    this.addRobotButton = document.getElementById("addRobotButton");

    this.operatorMapSvg = document.getElementById("operatorMapSvg");
    this.operatorScene3d = document.getElementById("operatorScene3d");
    this.operatorMapLoading = document.getElementById("operatorMapLoading");
    this.operatorMapLoadingTitle = document.getElementById("operatorMapLoadingTitle");
    this.operatorMapLoadingText = document.getElementById("operatorMapLoadingText");
    this.operatorMap2dButton = document.getElementById("operatorMap2dButton");
    this.operatorMap3dButton = document.getElementById("operatorMap3dButton");
    this.operatorViewport = document.getElementById("operatorViewport");
    this.operatorMapImage = document.getElementById("operatorMapImage");
    this.operatorObstacleLayer = document.getElementById("operatorObstacleLayer");
    this.operatorGraphLayer = document.getElementById("operatorGraphLayer");
    this.operatorRouteLayer = document.getElementById("operatorRouteLayer");
    this.operatorLookaheadLayer = document.getElementById("operatorLookaheadLayer");
    this.operatorLandmarkLayer = document.getElementById("operatorLandmarkLayer");
    this.operatorEditorLayer = document.getElementById("operatorEditorLayer");
    this.operatorScanLayer = document.getElementById("operatorScanLayer");
    this.operatorRobotLayer = document.getElementById("operatorRobotLayer");
    this.operatorRelocateLayer = document.getElementById("operatorRelocateLayer");
    this.operatorZoomInButton = document.getElementById("operatorZoomInButton");
    this.operatorZoomOutButton = document.getElementById("operatorZoomOutButton");
    this.operatorResetViewButton = document.getElementById("operatorResetViewButton");
    this.operatorLmNamesButton = document.getElementById("operatorLmNamesButton");
    this.operatorEdgeDirectionsButton = document.getElementById("operatorEdgeDirectionsButton");
    this.operatorFollowRobotButton = document.getElementById("operatorFollowRobotButton");
    this.manualPad = document.getElementById("manualPad");

    this.inspectorRobotText = document.getElementById("inspectorRobotText");
    this.inspectorModeText = document.getElementById("inspectorModeText");
    this.connectionText = document.getElementById("connectionText");
    this.inspectorMapText = document.getElementById("inspectorMapText");
    this.inspectorCurrentLmText = document.getElementById("inspectorCurrentLmText");
    this.localizationText = document.getElementById("localizationText");
    this.targetLmText = document.getElementById("targetLmText");
    this.currentEdgeText = document.getElementById("currentEdgeText");
    this.routeProgressText = document.getElementById("routeProgressText");
    this.inspectorBatteryText = document.getElementById("inspectorBatteryText");
    this.inspectorConfidenceText = document.getElementById("inspectorConfidenceText");
    this.poseText = document.getElementById("poseText");
    this.velocityText = document.getElementById("velocityText");
    this.inspectorApiText = document.getElementById("inspectorApiText");
    this.inspectorReasonText = document.getElementById("inspectorReasonText");
    this.robotMessageText = document.getElementById("robotMessageText");
    this.routeNodesText = document.getElementById("routeNodesText");
    this.robotEventsLog = document.getElementById("robotEventsLog");

    this.addRobotDialog = document.getElementById("addRobotDialog");
    this.closeDialogButton = document.getElementById("closeDialogButton");
    this.robotNameInput = document.getElementById("robotNameInput");
    this.robotHostInput = document.getElementById("robotHostInput");
    this.robotDomainInput = document.getElementById("robotDomainInput");
    this.robotPortInput = document.getElementById("robotPortInput");
    this.probeResult = document.getElementById("probeResult");
    this.probeRobotButton = document.getElementById("probeRobotButton");
    this.saveRobotButton = document.getElementById("saveRobotButton");
    this.loadMapDialog = document.getElementById("loadMapDialog");
    this.loadMapSelect = document.getElementById("loadMapSelect");
    this.loadMapHint = document.getElementById("loadMapHint");
    this.closeLoadMapDialogButton = document.getElementById("closeLoadMapDialogButton");
    this.cancelLoadMapButton = document.getElementById("cancelLoadMapButton");
    this.confirmLoadMapButton = document.getElementById("confirmLoadMapButton");
    this.mapSyncDecisionDialog = document.getElementById("mapSyncDecisionDialog");
    this.mapSyncDecisionTitle = document.getElementById("mapSyncDecisionTitle");
    this.mapSyncDecisionText = document.getElementById("mapSyncDecisionText");
    this.mapSyncDecisionDetail = document.getElementById("mapSyncDecisionDetail");
    this.mapSyncPullButton = document.getElementById("mapSyncPullButton");
    this.mapSyncCancelButton = document.getElementById("mapSyncCancelButton");
    this.mapSyncPushButton = document.getElementById("mapSyncPushButton");
    this.fleetMapExitDialog = document.getElementById("fleetMapExitDialog");
    this.fleetMapExitDiscardButton = document.getElementById("fleetMapExitDiscardButton");
    this.fleetMapExitSaveButton = document.getElementById("fleetMapExitSaveButton");
    this.fleetMapExitPushButton = document.getElementById("fleetMapExitPushButton");
    this.fleetMapExitCancelButton = document.getElementById("fleetMapExitCancelButton");
    this.fleetMapSaveAsDialog = document.getElementById("fleetMapSaveAsDialog");
    this.fleetMapSaveAsForm = document.getElementById("fleetMapSaveAsForm");
    this.fleetMapSaveAsNameInput = document.getElementById("fleetMapSaveAsNameInput");
    this.fleetMapSaveAsCancelButton = document.getElementById("fleetMapSaveAsCancelButton");
    this.mapTransferDialog = document.getElementById("mapTransferDialog");
    this.mapTransferTitle = document.getElementById("mapTransferTitle");
    this.mapTransferPercent = document.getElementById("mapTransferPercent");
    this.mapTransferBar = document.getElementById("mapTransferBar");
    this.mapTransferStatus = document.getElementById("mapTransferStatus");
    this.mapTransferCloseButton = document.getElementById("mapTransferCloseButton");
    this.slamDialog = document.getElementById("slamDialog");
    this.closeSlamDialogButton = document.getElementById("closeSlamDialogButton");
    this.cancelSlamDialogButton = document.getElementById("cancelSlamDialogButton");
    this.confirmStartSlamButton = document.getElementById("confirmStartSlamButton");
    this.slamParamsInput = document.getElementById("slamParamsInput");
    this.slamDialogStatus = document.getElementById("slamDialogStatus");
  }

  async init() {
    this.bindEvents();
    this.initFleetModelEditor();
    this.renderSelectedRobot();
    window.addEventListener("popstate", () => {
      this.applyRoute().catch((error) => {
        this.robotMessageText.textContent = error.message || String(error);
      });
    });
    try {
      await this.applyRoute({ replace: window.location.pathname === "/" });
      // Boot from saved metadata first. Probing every offline gRPC endpoint
      // serially here made the whole UI look frozen for several seconds.
      // The lightweight ping/status loops start immediately afterwards.
      await this.refreshRobots({ probe: false, lightweight: true });
    } finally {
      this.appBooting = false;
      this.workspaceLoadingRobotId = "";
      this.render();
    }
    this.refreshInitialWorkspaceInBackground();
    this.applyDeferredUiActions();
    this.syncFleetStatusStream();
    window.setInterval(() => {
      this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    }, 12000);
    window.setInterval(() => {
      this.refreshRobotPings().catch(() => {});
    }, 2000);
    window.setInterval(() => {
      this.fetchSelectedRobotStatus(true).catch(() => {});
    }, 800);
    window.setInterval(() => {
      this.expireRobotStatusIfStale();
    }, 500);
    window.setInterval(() => {
      this.tickFleetIfSelected().catch(() => {});
    }, 80);
    window.setInterval(() => {
      if (!this.isFleetManagerSim()) {
        this.sendTeleopIfNeeded().catch(() => {});
      }
    }, 33);
  }

  refreshInitialWorkspaceInBackground() {
    if (this.isGlobalHomePage() || !this.selectedRobot()) {
      return;
    }
    const context = this.selectionContext();
    if (!this.activeOperatorMapPayload()?.map) {
      this.mapContentLoadingRobotId = context.robotId;
      this.renderSelectedRobot();
    }
    const requests = [
      this.refreshRobotMapState({ quiet: true }),
      this.fetchSelectedRobotStatus(true),
      this.refreshSelectedSlamState({ quiet: true }),
    ];
    if (!this.isFleetManager() && !this.isRobotModelPage() && !this.isParamsPage()) {
      requests.push(this.ensureRobotParamsLoaded());
    }
    if (this.isRobotModelPage()) {
      requests.push(this.ensureRobotParamsLoaded());
    }
    if (this.isParamsPage()) {
      requests.push(this.ensureCurrentParamsLoaded());
    }
    Promise.allSettled(requests).then(() => {
      if (!this.selectionIsCurrent(context)) {
        return;
      }
      if (this.fleetActiveTab === "map") {
        this.ensureFleetMapDraft();
      }
      this.renderSelectedRobot();
      this.syncFleetStatusStream();
    });
  }

  bindEvents() {
    this.globalHomeButton?.addEventListener("click", async () => this.navigateGlobalHomePage());
    this.homeButton?.addEventListener("click", async () => this.navigateHomePage());
    this.paramsNavButton?.addEventListener("click", async () => this.navigateParamsPage());
    this.mapEditorNavButton?.addEventListener("click", async () => this.navigateMapEditorPage());
    this.robotModelNavButton?.addEventListener("click", async () => this.navigateRobotModelPage());
    this.openSidebarButton?.addEventListener("click", () => this.openSidebar());
    this.closeSidebarButton?.addEventListener("click", () => this.closeSidebar());
    this.sidebarBackdrop?.addEventListener("click", () => this.closeSidebar());
    this.refreshButton?.addEventListener("click", () => this.refreshRobots());
    this.addRobotButton?.addEventListener("click", () => this.openAddRobotDialog());
    this.homeRefreshButton?.addEventListener("click", () => this.refreshRobots());
    this.homeAddRobotButton?.addEventListener("click", () => this.openAddRobotDialog());
    this.navigateRobotButton.addEventListener("click", () => this.toggleNavigateMode());
    this.takeControlButton?.addEventListener("click", () => this.acquireRobotControl(true, true));
    this.releaseControlButton?.addEventListener("click", () => this.releaseRobotControl());
    this.relocateRobotButton?.addEventListener("click", () => this.toggleRelocateMode());
    this.pauseRouteButton?.addEventListener("click", () => this.pauseRobotRoute());
    this.resumeRouteButton?.addEventListener("click", () => this.resumeRobotRoute());
    this.cancelRouteButton.addEventListener("click", () => this.cancelRoute());
    this.stopRobotButton.addEventListener("click", () => this.stopRobot());
    this.scanToggleButton?.addEventListener("click", () => {
      this.toggleScanStream().catch((error) => {
        if (this.robotMessageText) {
          this.robotMessageText.textContent = `Scan failed: ${error.message || error}`;
        }
        this.closeScanStream();
      });
    });
    this.controlPullMapButton.addEventListener("click", () => this.handlePullMap());
    this.controlPushMapButton.addEventListener("click", () => this.handlePushMap());
    this.controlLoadMapButton.addEventListener("click", () => this.handleLoadMap());
    this.operatorMap2dButton?.addEventListener("click", () => this.setMapViewMode("2d"));
    this.operatorMap3dButton?.addEventListener("click", () => this.setMapViewMode("3d"));
    this.operatorLmNamesButton?.addEventListener("click", () => this.toggleLmNames());
    this.operatorEdgeDirectionsButton?.addEventListener("click", () => this.toggleEdgeDirections());
    this.startSlamButton?.addEventListener("click", () => this.openSlamDialog());
    this.doneSlamButton?.addEventListener("click", () => this.finishSlam());
    this.cancelSlamButton?.addEventListener("click", () => this.cancelSlam());
    this.fleetRobotNameInput.addEventListener("input", () => {
      this.fleetNameEdited = true;
    });
    this.fleetAddRobotButton.addEventListener("click", () => this.handleFleetAddRobot());
    this.fleetPlaceRobotButton?.addEventListener("click", () => this.toggleFleetSpawnMode());
    this.fleetBenchmarkButtons.forEach((button) => {
      button.addEventListener("click", () => this.runFleetBenchmark(Number(button.dataset.fleetBenchmarkCount || 20)));
    });
    this.fleetBenchmarkPlanButton?.addEventListener("click", () => this.planFleetBenchmarkRobots());
    this.fleetBenchmarkPackageButton?.addEventListener("click", () => this.planFleetPackageOrders());
    this.fleetSimulationTimeScaleSelect?.addEventListener("change", () => this.setFleetSimulationTimeScale());
    this.fleetBenchmarkClearButton?.addEventListener("click", () => this.runFleetBenchmark(0));
    this.fleetQueueGoalButton.addEventListener("click", () => this.toggleFleetQueueMode());
    this.fleetStartQueueButton.addEventListener("click", () => this.startQueuedFleetPlan());
    this.fleetClearQueueButton.addEventListener("click", () => this.clearFleetQueue());
    this.fleetPauseOrderButton.addEventListener("click", () => this.pauseSelectedFleetOrder());
    this.fleetResumeOrderButton.addEventListener("click", () => this.resumeSelectedFleetOrder());
    this.fleetCancelOrderButton.addEventListener("click", () => this.cancelSelectedFleetOrder());
    this.fleetSaveParamsButton.addEventListener("click", () => this.saveFleetParams());
    this.fleetReloadParamsButton.addEventListener("click", async () => {
      await this.ensureFleetParamsLoaded(true);
      this.renderSelectedRobot();
    });
    this.fleetFormatParamsButton.addEventListener("click", () => this.formatParamsJson(this.fleetParamsJsonInput, this.fleetParams));
    this.fleetSaveJsonParamsButton.addEventListener("click", () => this.saveFleetJsonParams());
    this.robotReloadParamsButton.addEventListener("click", async () => {
      await this.ensureRobotParamsLoaded(true);
      this.renderSelectedRobot();
    });
    this.robotFormatParamsButton.addEventListener("click", () => this.formatParamsJson(this.robotParamsJsonInput, this.robotParams));
    this.robotDefaultsParamsButton.addEventListener("click", () => this.resetRobotParamsToDefaults());
    this.robotSaveParamsButton.addEventListener("click", () => this.saveRobotParams());
    this.fleetModelSaveButton.addEventListener("click", () => this.saveRobotModelParams());
    this.fleetTabButtons.forEach((button) => {
      button.addEventListener("click", async () => {
        await this.navigateFleetPage(button.dataset.fleetTab || "fleet");
      });
    });
    this.fleetMapToolButtons.forEach((button) => {
      button.addEventListener("click", () => this.setFleetMapTool(button.dataset.fleetMapTool || "select"));
    });
    for (const input of [
      this.fleetEditorLmNameInput,
      this.fleetEditorLmXInput,
      this.fleetEditorLmYInput,
    ]) {
      input?.addEventListener("change", () => this.applyFleetEditorLmFields());
    }
    this.fleetEditorEdgeTrafficSelect?.addEventListener(
      "change",
      () => this.applyFleetEditorEdgeFields(),
    );
    this.fleetEditorEdgeMotionSelect?.addEventListener(
      "change",
      () => this.applyFleetEditorEdgeFields(),
    );
    this.fleetRasterBrushSizeInput?.addEventListener("input", () => this.syncFleetRasterControls());
    this.fleetRasterUndoButton?.addEventListener("click", () => this.undoFleetRaster());
    this.fleetRasterRedoButton?.addEventListener("click", () => this.redoFleetRaster());
    this.fleetMapSaveButton.addEventListener("click", () => this.saveFleetMap(false));
    this.fleetMapSaveAsButton.addEventListener("click", () => this.saveFleetMap(true));
    this.fleetMapReloadButton.addEventListener("click", () => this.reloadFleetMapDraft());
    this.fleetMapCloseButton?.addEventListener("click", () => this.navigateFleetPage("fleet"));
    this.fleetMapExitDiscardButton?.addEventListener("click", () => this.resolveFleetMapExit("discard"));
    this.fleetMapExitSaveButton?.addEventListener("click", () => this.resolveFleetMapExit("save"));
    this.fleetMapExitPushButton?.addEventListener("click", () => this.resolveFleetMapExit("save-push"));
    this.fleetMapExitCancelButton?.addEventListener("click", () => this.resolveFleetMapExit("cancel"));
    this.fleetMapExitDialog?.addEventListener("cancel", (event) => {
      event.preventDefault();
      this.resolveFleetMapExit("cancel");
    });
    this.fleetMapSaveAsForm?.addEventListener("submit", (event) => {
      event.preventDefault();
      this.resolveFleetMapSaveAs(this.fleetMapSaveAsNameInput?.value || "");
    });
    this.fleetMapSaveAsCancelButton?.addEventListener("click", () => this.resolveFleetMapSaveAs(""));
    this.fleetMapSaveAsDialog?.addEventListener("cancel", (event) => {
      event.preventDefault();
      this.resolveFleetMapSaveAs("");
    });
    this.closeDialogButton.addEventListener("click", () => this.addRobotDialog.close());
    this.probeRobotButton.addEventListener("click", () => this.handleProbe());
    this.saveRobotButton.addEventListener("click", async (event) => {
      event.preventDefault();
      await this.handleSaveRobot();
    });
    this.closeLoadMapDialogButton.addEventListener("click", () => this.loadMapDialog.close());
    this.cancelLoadMapButton.addEventListener("click", () => this.loadMapDialog.close());
    this.confirmLoadMapButton.addEventListener("click", () => this.confirmLoadMap());
    this.mapSyncPushButton.addEventListener("click", () => this.resolveMapSyncDecision("push"));
    this.mapSyncPullButton.addEventListener("click", () => this.resolveMapSyncDecision("pull"));
    this.mapSyncCancelButton.addEventListener("click", () => this.resolveMapSyncDecision("cancel"));
    this.mapSyncDecisionDialog.addEventListener("cancel", (event) => {
      event.preventDefault();
      this.resolveMapSyncDecision("cancel");
    });
    this.mapTransferCloseButton.addEventListener("click", () => this.mapTransferDialog.close());
    this.closeSlamDialogButton?.addEventListener("click", () => this.slamDialog.close());
    this.cancelSlamDialogButton?.addEventListener("click", () => this.slamDialog.close());
    this.confirmStartSlamButton?.addEventListener("click", () => this.startSlamFromDialog());

    this.operatorZoomInButton.addEventListener("click", () => this.zoomMap(1.16));
    this.operatorZoomOutButton.addEventListener("click", () => this.zoomMap(0.86));
    this.operatorResetViewButton.addEventListener("click", () => this.resetMapView(false));
    this.operatorFollowRobotButton.addEventListener("click", () => {
      this.mapView.follow = !this.mapView.follow;
      this.syncMapControls();
      this.renderOperatorMap();
    });
    this.operatorMapSvg.addEventListener("pointerdown", (event) => this.handleMapPointerDown(event));
    this.operatorMapSvg.addEventListener("pointermove", (event) => this.handleMapPointerMove(event));
    this.operatorMapSvg.addEventListener("pointerup", (event) => this.handleMapPointerUp(event));
    this.operatorMapSvg.addEventListener("pointercancel", (event) => this.handleMapPointerUp(event));
    this.operatorMapSvg.addEventListener("wheel", (event) => this.handleMapWheel(event), { passive: false });
    this.operatorMapSvg.addEventListener("click", (event) => this.handleMapClick(event));
    this.operatorMapSvg.addEventListener("contextmenu", (event) => this.handleMapContextMenu(event));

    document.querySelectorAll("[data-manual-key]").forEach((button) => {
      button.addEventListener("pointerdown", () => this.setManualKey(button.dataset.manualKey, true));
      button.addEventListener("pointerup", () => this.setManualKey(button.dataset.manualKey, false));
      button.addEventListener("pointerleave", () => this.setManualKey(button.dataset.manualKey, false));
    });
    window.addEventListener("keydown", (event) => {
      if (this.isTypingTarget(event.target)) {
        return;
      }
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) {
        return;
      }
      event.preventDefault();
      this.setManualKey(key, true);
    });
    window.addEventListener("keyup", (event) => {
      if (this.isTypingTarget(event.target)) {
        return;
      }
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) {
        return;
      }
      this.setManualKey(key, false);
    });
    window.addEventListener("beforeunload", (event) => {
      if (this.fleetMapEditorActive && this.fleetMapDirty) {
        event.preventDefault();
        event.returnValue = "";
      }
      this.closeScanStream();
      this.closeSlamStream();
      this.closeTeleopSocket(true);
    });
  }

  applyDeferredUiActions() {
    if (window.sessionStorage.getItem("operator:openSidebar") === "1") {
      window.sessionStorage.removeItem("operator:openSidebar");
      this.openSidebar();
    }
    if (window.sessionStorage.getItem("operator:openAddRobot") === "1") {
      window.sessionStorage.removeItem("operator:openAddRobot");
      this.openAddRobotDialog();
    }
  }

  isTypingTarget(target) {
    return Boolean(target && ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName));
  }

  initFleetModelEditor() {
    if (!this.fleetRobotModelSvg) {
      return;
    }
    this.fleetModelEditor = new FleetRobotModelEditor(
      {
        svg: this.fleetRobotModelSvg,
        footprintFields: this.fleetFootprintFields,
        tfFields: this.fleetTfFields,
        zoomIn: this.fleetModelZoomInButton,
        zoomOut: this.fleetModelZoomOutButton,
        resetView: this.fleetModelResetViewButton,
        resetModel: this.fleetModelResetButton,
      },
      (model) => {
        this.robotParams = this.robotParams || {};
        this.robotParams.robot_model = model;
        if (this.isParamsPage() && !this.isFleetManager()) {
          this.renderRobotParamsTable();
          this.syncRobotParamsJson(true);
        }
      }
    );
    this.fleetModelEditor.init();
  }

  pageForPath(pathname) {
    const path = String(pathname || "/").replace(/\/+$/, "") || "/";
    if (path === "/" || path === "/home") {
      return "robots";
    }
    if (path === "/robot" || path === "/robot-home") {
      return "fleet";
    }
    if (path === "/params") {
      return "params";
    }
    if (path === "/robot_model" || path === "/robot-model") {
      return "model";
    }
    if (path === "/map_editor" || path === "/map-editor") {
      return "map";
    }
    return "robots";
  }

  pathForFleetPage(tabName) {
    const tab = ["robots", "fleet", "params", "model", "map"].includes(tabName) ? tabName : "robots";
    return {
      robots: "/home",
      fleet: "/robot",
      params: "/params",
      model: "/robot_model",
      map: "/map_editor",
    }[tab];
  }

  async navigateGlobalHomePage(options = {}) {
    if (!options.skipMapExitGuard && !await this.confirmFleetMapExit()) {
      return;
    }
    const path = this.pathForFleetPage("robots");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "robots" }, "", path);
    }
    this.setFleetTab("robots");
    this.closeRobotStatusStream();
    this.closeFleetStatusStream();
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
    this.stopFleetAnimationLoop();
    this.renderSelectedRobot();
  }

  async navigateHomePage(options = {}) {
    if (!options.skipMapExitGuard && !await this.confirmFleetMapExit()) {
      return;
    }
    const path = this.pathForFleetPage("fleet");
    if (
      !options.force
      && window.location.pathname === path
      && this.fleetActiveTab === "fleet"
      && this.selectedRobot()
    ) {
      return;
    }
    const returningFromMapEditor = this.fleetActiveTab === "map";
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "fleet" }, "", path);
    }
    this.setFleetTab("fleet");
    if (!this.selectedRobot() && this.robots.length) {
      const robot = this.robots.find((item) => !item.system) || this.robots[0];
      this.setSelectedRobotId(robot.id);
    }
    if (returningFromMapEditor && Array.isArray(this.currentStatus?.robots)) {
      // The status stream remains live while editing. Mark its latest cached
      // snapshot as immediately renderable so returning Home never waits for
      // another websocket packet before robot interpolation resumes.
      this.fleetStatusReceivedAt = performance.now();
    }
    this.syncFleetStatusStream();
    this.renderSelectedRobot();
    this.ensureFleetAnimationLoop();
    const context = this.selectionContext();
    const backgroundRequests = [
      this.refreshRobotMapState({ quiet: true }),
      this.fetchSelectedRobotStatus(true),
      this.refreshSelectedSlamState({ quiet: true }),
    ];
    if (!this.isFleetManager()) {
      backgroundRequests.push(this.ensureRobotParamsLoaded());
    }
    Promise.allSettled(backgroundRequests).then(() => {
      if (!this.selectionIsCurrent(context)) {
        return;
      }
      this.renderSelectedRobot();
      this.syncFleetStatusStream();
    });
    await new Promise((resolve) => window.requestAnimationFrame(resolve));
  }

  async navigateParamsPage(options = {}) {
    if (!options.skipMapExitGuard && !await this.confirmFleetMapExit()) {
      return;
    }
    const path = this.pathForFleetPage("params");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "params" }, "", path);
    }
    this.setFleetTab("params");
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
    await this.ensureCurrentParamsLoaded();
    this.renderSelectedRobot();
  }

  async navigateMapEditorPage(options = {}) {
    const selected = this.selectedRobot();
    if (selected && !this.isFleetManager(selected)) {
      this.openMapEditor();
      return;
    }
    await this.navigateFleetPage("map", options);
  }

  async navigateFleetPage(tabName, options = {}) {
    if (tabName === "model") {
      await this.navigateRobotModelPage(options);
      return;
    }
    if (tabName === "params") {
      await this.navigateParamsPage(options);
      return;
    }
    const tab = ["fleet", "params", "map"].includes(tabName) ? tabName : "fleet";
    if (tab !== "map" && !options.skipMapExitGuard && !await this.confirmFleetMapExit()) {
      return;
    }
    const path = this.pathForFleetPage(tab);
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: tab }, "", path);
    }
    this.setFleetTab(tab);
    const selectedChanged = this.ensureFleetManagerSelected();
    if (tab === "map" || tab === "params") {
      await this.ensureFleetPageReady(selectedChanged);
    }
    this.renderSelectedRobot();
  }

  async navigateRobotModelPage(options = {}) {
    if (!options.skipMapExitGuard && !await this.confirmFleetMapExit()) {
      return;
    }
    const path = this.pathForFleetPage("model");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "model" }, "", path);
    }
    this.setFleetTab("model");
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
    const selectedChanged = this.ensureRobotSelectedForModel();
    if (selectedChanged) {
      await this.refreshRobotMapState({ quiet: true });
      await this.fetchSelectedRobotStatus(true);
    }
    await this.ensureRobotParamsLoaded(selectedChanged);
    this.renderSelectedRobot();
  }

  async ensureFleetPageReady(selectedChanged = false) {
    if (selectedChanged) {
      await this.refreshRobotMapState({ quiet: true });
      await this.fetchSelectedRobotStatus(true);
    }
    if (this.fleetActiveTab === "params") {
      await this.ensureFleetParamsLoaded();
    }
    if (this.fleetActiveTab === "map") {
      this.ensureFleetMapDraft();
    }
  }

  async applyRoute(options = {}) {
    const tab = this.pageForPath(window.location.pathname);
    if (
      tab !== "map"
      && this.fleetMapEditorActive
      && !options.skipMapExitGuard
      && !await this.confirmFleetMapExit()
    ) {
      const mapPath = this.pathForFleetPage("map");
      window.history.pushState({ fleetPage: "map" }, "", mapPath);
      return;
    }
    const canonical = this.pathForFleetPage(tab);
    if (options.replace && window.location.pathname !== canonical) {
      window.history.replaceState({ fleetPage: tab }, "", canonical);
    }
    this.setFleetTab(tab);
    if (tab === "robots") {
      this.renderSelectedRobot();
      return;
    }
    if (tab === "model") {
      this.ensureRobotSelectedForModel();
      await this.ensureRobotParamsLoaded();
    } else if (tab === "map") {
      this.ensureFleetManagerSelected();
      await this.ensureFleetPageReady();
    } else if (tab === "params") {
      await this.ensureCurrentParamsLoaded();
    }
    this.renderSelectedRobot();
  }

  ensureFleetManagerSelected() {
    const selected = this.selectedRobot();
    if (selected && this.isFleetManager(selected)) {
      return false;
    }
    const fleet = this.robots.find((robot) => robot.id === this.lastFleetManagerId && this.isFleetManager(robot))
      || this.robots.find((robot) => robot.id === this.fleetManagerId)
      || this.robots.find((robot) => robot.id === this.fleetManagerSimId)
      || this.robots.find((robot) => this.isFleetManager(robot));
    if (!fleet) {
      return false;
    }
    this.setSelectedRobotId(fleet.id);
    this.currentStatus = null;
    this.currentRoute = null;
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
    this.syncFleetStatusStream();
    return true;
  }

  ensureRobotSelectedForModel() {
    const selected = this.selectedRobot();
    if (selected && !this.isFleetManager(selected)) {
      return false;
    }
    const robot = this.robots.find((item) => !this.isFleetManager(item));
    if (!robot) {
      this.setSelectedRobotId("");
      this.currentStatus = null;
      this.currentRoute = null;
      this.closeScanStream();
      this.closeSlamStream();
      this.closeTeleopSocket(true);
      this.syncFleetStatusStream();
      return false;
    }
    this.setSelectedRobotId(robot.id);
    this.currentStatus = null;
    this.currentRoute = null;
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
    this.syncFleetStatusStream();
    return true;
  }

  setFleetTab(tabName) {
    const tab = ["robots", "fleet", "params", "model", "map"].includes(tabName) ? tabName : "robots";
    this.fleetActiveTab = tab;
    window.localStorage.setItem("operator:fleetActiveTab", tab);
    this.fleetTabButtons.forEach((button) => button.classList.toggle("active", button.dataset.fleetTab === tab));
    this.fleetTabFleet.classList.toggle("active", tab === "fleet");
    this.fleetTabParams.classList.toggle("active", tab === "params");
    if (this.fleetTabModel) {
      this.fleetTabModel.classList.toggle("active", tab === "model");
    }
    this.fleetTabMap.classList.toggle("active", tab === "map");
    this.fleetMapEditorActive = tab === "map";
    if (this.fleetMapEditorActive) {
      this.mapViewMode = "2d";
      window.localStorage.setItem("operator:mapViewMode", "2d");
    }
    this.syncFleetPageClass(this.isFleetManager());
    this.operatorMapSvg.classList.toggle("fleet-map-editor-active", this.fleetMapEditorActive);
    if (tab === "map") {
      this.navigateMode = false;
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.syncModeButtons();
      this.ensureFleetMapDraft();
      this.ensureFleetRasterGrid().then(() => this.renderOperatorMap());
      this.syncFleetMapEditorState();
      this.robotMessageText.textContent = "Fleet map editor active.";
      this.stopFleetAnimationLoop();
    }
    this.renderOperatorMap();
    if (tab === "fleet") {
      this.ensureFleetAnimationLoop();
    }
  }

  syncFleetPageClass(isFleet = this.isFleetManager()) {
    const isRobotsHome = this.fleetActiveTab === "robots";
    const isRobotModel = this.fleetActiveTab === "model";
    const isRobotParams = !isFleet && this.fleetActiveTab === "params";
    const pageKey = isRobotsHome ? "robots-home" : (isRobotModel ? "robot-model" : (isFleet ? (this.fleetActiveTab || "fleet") : (isRobotParams ? "robot-params" : "robot")));
    document.body.dataset.fleetPage = pageKey;
    if (this.globalHomeButton) {
      this.globalHomeButton.classList.toggle("primary", isRobotsHome);
    }
    if (this.homeButton) {
      this.homeButton.classList.toggle("primary", this.fleetActiveTab === "fleet");
    }
    if (this.paramsNavButton) {
      this.paramsNavButton.classList.toggle("primary", this.fleetActiveTab === "params");
    }
    if (this.mapEditorNavButton) {
      this.mapEditorNavButton.classList.toggle("primary", isFleet && this.fleetActiveTab === "map");
    }
    if (this.robotModelNavButton) {
      this.robotModelNavButton.classList.toggle("hidden", Boolean(isFleet));
      this.robotModelNavButton.classList.toggle("primary", isRobotModel);
    }
    if (!this.operatorConsole) {
      return;
    }
    this.operatorConsole.classList.toggle("fleet-console", Boolean(isFleet));
    this.operatorConsole.classList.toggle("robot-page-model", isRobotModel);
    this.operatorConsole.classList.toggle("robot-page-params", isRobotParams);
    for (const page of ["fleet", "params", "model", "map"]) {
      this.operatorConsole.classList.remove(`fleet-page-${page}`);
    }
    if (isFleet && !isRobotModel && !isRobotsHome) {
      this.operatorConsole.classList.add(`fleet-page-${this.fleetActiveTab || "fleet"}`);
    }
  }

  setFleetMapTool(tool) {
    const allowed = ["select", "lm", "edge", "corridor", ...FLEET_RASTER_TOOLS];
    this.fleetMapTool = allowed.includes(tool) ? tool : "select";
    if (FLEET_RASTER_TOOLS.has(this.fleetMapTool) && !this.fleetRasterGrid) {
      this.fleetMapTool = "select";
      this.robotMessageText.textContent = "Raster tools are loading for this fleet map.";
      this.ensureFleetRasterGrid();
    }
    if (this.fleetMapTool !== "corridor") {
      this.fleetCorridorStartLm = "";
    }
    this.fleetMapToolButtons.forEach((button) => button.classList.toggle("active", button.dataset.fleetMapTool === this.fleetMapTool));
    const hints = {
      select: "Select LM/edge. Drag LM. Drag Bezier handles. Right-click LM/edge deletes.",
      lm: "Click empty map space to add an LM.",
      edge: "Hold an LM and drag through other LMs to create edges.",
      corridor: "Select two holding LMs to mark the complete single-lane corridor.",
      brush: "Draw occupied map cells on the Babylon floor.",
      eraser: "Draw free map cells on the Babylon floor.",
      unknown: "Mark cells as unknown.",
      fill: "Fill one connected occupancy area.",
      rectangle: "Draw an occupied rectangle.",
    };
    this.fleetMapEditorHelp.textContent = hints[this.fleetMapTool] || hints.select;
    this.syncFleetRasterControls();
    this.renderOperatorMap();
  }

  emptyMapState() {
    return {
      robotActiveMapName: "",
      operatorActiveMapName: "",
      robotSignature: "",
      operatorSignature: "",
      sourceRobotMapName: "",
      hasLocalChanges: false,
    };
  }

  setSelectedRobotId(robotId, options = {}) {
    const nextId = String(robotId || "");
    const changed = nextId !== this.selectedRobotId;
    if (changed) {
      this.selectedRobotId = nextId;
      this.selectionGeneration += 1;
      this.statusRequestPending = false;
      this.workspaceLoadingRobotId = "";
      this.mapContentLoadingRobotId = nextId;
      this.robotMapState = this.emptyMapState();
      this.operatorMapPayload = null;
      this.operatorMapSignature = "";
      this.fleetMapDraft = null;
      this.scene3dStaticKey = "";
      this.scene3dPayload = null;
      if (this.operatorScene3d) {
        this.operatorScene3d.dataset.managerId = "";
        this.operatorScene3d.dataset.mapName = "";
      }
    }
    if (options.remember !== false) {
      if (nextId) {
        window.localStorage.setItem("operator:selectedRobotId", nextId);
      } else {
        window.localStorage.removeItem("operator:selectedRobotId");
      }
    }
    const robot = this.robots.find((item) => item.id === nextId);
    if (robot && this.isFleetManager(robot)) {
      this.lastFleetManagerId = robot.id;
      window.localStorage.setItem("operator:lastFleetManagerId", robot.id);
    }
    return changed;
  }

  selectionContext(robot = this.selectedRobot()) {
    return {
      robotId: String(robot?.id || ""),
      generation: this.selectionGeneration,
    };
  }

  selectionIsCurrent(context) {
    return Boolean(
      context
      && context.robotId
      && context.robotId === this.selectedRobotId
      && context.generation === this.selectionGeneration
    );
  }

  selectedRobot() {
    return this.robots.find((robot) => robot.id === this.selectedRobotId) || null;
  }

  isFleetManager(robot = this.selectedRobot()) {
    return Boolean(robot && (
      robot.id === this.fleetManagerId
      || robot.id === this.fleetManagerSimId
      || robot.type === "fleet_manager"
    ));
  }

  isFleetManagerSim(robot = this.selectedRobot()) {
    if (!this.isFleetManager(robot)) {
      return false;
    }
    const identityMode = String(robot?.identity?.mode || "").toLowerCase();
    const statusState = String(robot?.status?.state || "").toLowerCase();
    return robot.id === this.fleetManagerSimId
      || identityMode === "simulation"
      || statusState === "simulation";
  }

  fleetApiBase(robot = this.selectedRobot()) {
    return this.isFleetManagerSim(robot) ? "/api/fleet-manager-sim" : "/api/fleet-manager";
  }

  fleetApiPath(path = "", robot = this.selectedRobot()) {
    const suffix = String(path || "");
    return `${this.fleetApiBase(robot)}${suffix.startsWith("/") ? suffix : `/${suffix}`}`;
  }

  fleetWsPath(robot = this.selectedRobot()) {
    return this.isFleetManagerSim(robot) ? "/ws/fleet-manager-sim" : "/ws/fleet-manager";
  }

  isRos2Robot(robot = this.selectedRobot()) {
    const type = String(robot?.type || robot?.mode || "").toLowerCase();
    return Boolean(robot && ["grpc", "aivison_grpc", "real_grpc"].includes(type));
  }

  robotBaseName(robot) {
    const identity = robot?.identity || robot?.lastIdentity || {};
    return String(robot?.name || identity.robotId || robot?.id || "-");
  }

  robotEndpointLabel(robot) {
    if (!robot || this.isFleetManager(robot)) {
      return "";
    }
    if (this.isRos2Robot(robot) && String(robot.type || "").toLowerCase().includes("grpc")) {
      return `${robot.host || "-"}:${robot.port || 50051}`;
    }
    return robot.host ? `${robot.host}:${robot.port || ""}`.replace(/:$/, "") : "";
  }

  robotDisplayName(robot) {
    const baseName = this.robotBaseName(robot);
    if (!robot || this.isFleetManager(robot)) {
      return baseName;
    }
    const duplicateCount = this.robots.filter((item) => {
      return !this.isFleetManager(item) && this.robotBaseName(item) === baseName;
    }).length;
    const endpoint = this.robotEndpointLabel(robot);
    return duplicateCount > 1 && endpoint ? `${baseName} (${endpoint})` : baseName;
  }

  robotPingLabel(robot) {
    if (!robot || this.isFleetManager(robot)) {
      return "-";
    }
    const pingMs = Number(robot.pingMs);
    if (Number.isFinite(pingMs) && pingMs >= 0) {
      return `${Math.max(1, Math.round(pingMs))} ms`;
    }
    if (robot.probed && !robot.pingOk) {
      const pingError = String(robot.pingError || "").toLowerCase();
      if (pingError.includes("operation not permitted") || pingError.includes("not installed")) {
        return "unavailable";
      }
      return "timeout";
    }
    return "-";
  }

  fleetRuntimeMode(status = this.currentStatus, robot = this.selectedRobot()) {
    const statusManagerId = String(status?.managerId || "");
    if (!statusManagerId || !robot || statusManagerId === robot.id) {
      const mode = String(status?.mode || "").toLowerCase();
      if (mode) {
        return mode;
      }
    }
    return this.isFleetManagerSim(robot) ? "simulation" : "robots";
  }

  isFleetRobotsMode() {
    return this.isFleetManager() && this.fleetRuntimeMode() === "robots";
  }

  isFleetRemoteRobot(robot) {
    const mode = String(robot?.mode || robot?.type || "").toLowerCase();
    return ["remote", "robot", "real", "grpc", "aivison_grpc", "real_grpc"].includes(mode);
  }

  shouldAnimateFleetRobot(robot) {
    return this.fleetRuntimeMode() !== "robots" && !this.isFleetRemoteRobot(robot);
  }
}
