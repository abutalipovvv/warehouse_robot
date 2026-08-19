import { httpClient } from "./js/api/http-client.js";
import { CommandStack } from "./js/editor/command-stack.js";
import { markControlledCorridorArea } from "./js/editor/graph-tools.js";
import { OccupancyGrid, OCCUPANCY_VALUES } from "./js/editor/occupancy-grid.js";
import { preferences } from "./js/state/preferences.js";
import { cloneJson, escapeHtml } from "./js/shared/json.js";

const RASTER_TOOLS = new Set(["brush", "eraser", "unknown", "fill", "rectangle"]);

function normalizeEdgeMotionCode(value) {
  const numeric = Number(value);
  return numeric === 0 || numeric === 1 ? numeric : -1;
}

class RobotMapEditorApp {
  constructor() {
    const params = new URLSearchParams(window.location.search);
    this.robotId = params.get("robot_id") || "";
    this.robotName = params.get("robot_name") || this.robotId;

    this.robot = null;
    this.robotMaps = [];
    this.localMaps = [];
    this.currentMap = null;
    this.currentLocalMapName = "";
    this.currentSourceMapName = "";
    this.selectedLocalMapName = "";
    this.currentHasLocalChanges = false;
    this.selectedTool = "select";
    this.selection = { type: "none", key: "" };
    this.previewWorld = null;
    this.previewSnapName = "";
    this.dragState = null;
    this.dirty = false;
    this.logs = [];
    this.view = { x: 0, y: 0, width: 100, height: 100 };
    this.babylonScene = null;
    this.babylonRevision = 0;
    this.babylonRenderedRevision = -1;
    this.babylonFailed = false;
    this.lmNamesVisible = preferences.getBoolean("lmNamesVisible", false);
    this.edgeDirectionsVisible = preferences.getBoolean("edgeDirectionsVisible", true);
    this.rasterGrid = null;
    this.rasterHistory = new CommandStack(100);
    this.rasterHistory.onChange = () => this.syncRasterControls();
    this.rasterPreviewTimer = 0;
    this.allowUnload = false;
    this.exitDialogResolve = null;
    this.saveAsDialogResolve = null;

    this.editorRobotTitle = document.getElementById("editorRobotTitle");
    this.editorStatusText = document.getElementById("editorStatusText");
    this.editorWorkflowHeadline = document.getElementById("editorWorkflowHeadline");
    this.workflowStatusTitle = document.getElementById("workflowStatusTitle");
    this.workflowStateChip = document.getElementById("workflowStateChip");
    this.workflowStatusText = document.getElementById("workflowStatusText");
    this.workflowLmCountText = document.getElementById("workflowLmCountText");
    this.workflowEdgeCountText = document.getElementById("workflowEdgeCountText");
    this.editorLog = document.getElementById("editorLog");

    this.editorGlobalHomeButton = document.getElementById("editorGlobalHomeButton");
    this.editorGlobalRefreshButton = document.getElementById("editorGlobalRefreshButton");
    this.editorGlobalAddRobotButton = document.getElementById("editorGlobalAddRobotButton");
    this.editorHomeButton = document.getElementById("editorHomeButton");
    this.editorParamsButton = document.getElementById("editorParamsButton");
    this.editorMapEditorButton = document.getElementById("editorMapEditorButton");
    this.editorRobotModelButton = document.getElementById("editorRobotModelButton");
    this.refreshMapsButton = document.getElementById("refreshMapsButton");
    this.saveLocalButton = document.getElementById("saveLocalButton");
    this.saveAsButton = document.getElementById("saveAsButton");
    this.pushRobotButton = document.getElementById("pushRobotButton");
    this.cancelMapChangesButton = document.getElementById("cancelMapChangesButton");
    this.closeEditorButton = document.getElementById("closeEditorButton");
    this.editorExitDialog = document.getElementById("editorExitDialog");
    this.cancelExitButton = document.getElementById("cancelExitButton");
    this.saveAsDialog = document.getElementById("saveAsDialog");
    this.saveAsForm = document.getElementById("saveAsForm");
    this.saveAsNameInput = document.getElementById("saveAsNameInput");
    this.cancelSaveAsButton = document.getElementById("cancelSaveAsButton");

    this.toolButtons = Array.from(document.querySelectorAll("[data-tool]"));
    this.zoomInButton = document.getElementById("zoomInButton");
    this.zoomOutButton = document.getElementById("zoomOutButton");
    this.resetViewButton = document.getElementById("resetViewButton");
    this.lmNamesButton = document.getElementById("lmNamesButton");
    this.edgeDirectionsButton = document.getElementById("edgeDirectionsButton");
    this.deleteSelectionButton = document.getElementById("deleteSelectionButton");
    this.brushSizeInput = document.getElementById("brushSizeInput");
    this.brushSizeOutput = document.getElementById("brushSizeOutput");
    this.undoRasterButton = document.getElementById("undoRasterButton");
    this.redoRasterButton = document.getElementById("redoRasterButton");

    this.editorSvg = document.getElementById("editorSvg");
    this.editorBabylon = document.getElementById("editorBabylon");
    this.editorMapImage = document.getElementById("editorMapImage");
    this.editorEdgeLayer = document.getElementById("editorEdgeLayer");
    this.editorPreviewLayer = document.getElementById("editorPreviewLayer");
    this.editorLmLayer = document.getElementById("editorLmLayer");
    this.editorHandleLayer = document.getElementById("editorHandleLayer");

    this.selectionTitleText = document.getElementById("selectionTitleText");
    this.selectionEmptyText = document.getElementById("selectionEmptyText");
    this.landmarkInspector = document.getElementById("landmarkInspector");
    this.edgeInspector = document.getElementById("edgeInspector");
    this.landmarkNameInput = document.getElementById("landmarkNameInput");
    this.landmarkXInput = document.getElementById("landmarkXInput");
    this.landmarkYInput = document.getElementById("landmarkYInput");
    this.landmarkIgnoreDirInput = document.getElementById("landmarkIgnoreDirInput");
    this.landmarkCanWaitInput = document.getElementById("landmarkCanWaitInput");
    this.edgeFromText = document.getElementById("edgeFromText");
    this.edgeToText = document.getElementById("edgeToText");
    this.edgeKindSelect = document.getElementById("edgeKindSelect");
    this.edgeTypeInput = document.getElementById("edgeTypeInput");
    this.edgeTrafficSelect = document.getElementById("edgeTrafficSelect");
    this.edgeDirectionSelect = document.getElementById("edgeDirectionSelect");
    this.edgeControlledRegionInput = document.getElementById("edgeControlledRegionInput");
    this.edgeLengthText = document.getElementById("edgeLengthText");
    this.edgeCurveHint = document.getElementById("edgeCurveHint");
  }

  async init() {
    if (!this.robotId) {
      this.setStatus("No robot_id was provided to the map editor page.");
      this.log("error", "Missing robot_id query parameter.");
      return;
    }
    this.bindEvents();
    const engineReady = this.ensureBabylonEditor().catch((error) => {
      this.babylonFailed = true;
      this.editorBabylon?.classList.add("hidden");
      this.editorSvg?.classList.remove("hidden");
      this.log("warn", `Babylon renderer unavailable; SVG fallback enabled: ${error.message || error}`);
    });
    await this.refreshAll({ autoOpenLocal: true });
    await engineReady;
  }

  bindEvents() {
    this.editorGlobalHomeButton.addEventListener("click", () => this.goOperatorPage("/home"));
    this.editorGlobalRefreshButton.addEventListener("click", () => this.refreshAll());
    this.editorGlobalAddRobotButton.addEventListener("click", () => this.goOperatorPage("/home", { openAddRobot: true }));
    this.editorHomeButton.addEventListener("click", () => this.goOperatorPage("/robot"));
    this.editorParamsButton.addEventListener("click", () => this.goOperatorPage("/params"));
    this.editorMapEditorButton.addEventListener("click", () => this.refreshAll());
    this.editorRobotModelButton.addEventListener("click", () => this.goOperatorPage("/robot_model"));
    this.refreshMapsButton.addEventListener("click", () => this.refreshAll());
    this.saveLocalButton.addEventListener("click", () => this.saveLocalDraft());
    this.saveAsButton.addEventListener("click", () => this.saveLocalDraftAs());
    this.pushRobotButton.addEventListener("click", () => this.pushToRobot());
    this.cancelMapChangesButton.addEventListener("click", () => this.cancelMapChanges());
    this.closeEditorButton.addEventListener("click", () => this.handleCloseEditor());
    this.editorExitDialog?.querySelectorAll("[data-exit-choice]").forEach((button) => {
      button.addEventListener("click", () => this.resolveExitDialog(button.dataset.exitChoice || "cancel"));
    });
    this.cancelExitButton?.addEventListener("click", () => this.resolveExitDialog("cancel"));
    this.editorExitDialog?.addEventListener("cancel", (event) => {
      event.preventDefault();
      this.resolveExitDialog("cancel");
    });
    this.saveAsForm?.addEventListener("submit", (event) => {
      event.preventDefault();
      this.resolveSaveAsDialog(this.saveAsNameInput?.value || "");
    });
    this.cancelSaveAsButton?.addEventListener("click", () => this.resolveSaveAsDialog(""));
    this.saveAsDialog?.addEventListener("cancel", (event) => {
      event.preventDefault();
      this.resolveSaveAsDialog("");
    });
    this.zoomInButton.addEventListener("click", () => this.zoomView(0.88));
    this.zoomOutButton.addEventListener("click", () => this.zoomView(1.14));
    this.resetViewButton.addEventListener("click", () => this.resetView());
    this.lmNamesButton?.addEventListener("click", () => this.toggleLmNames());
    this.edgeDirectionsButton?.addEventListener("click", () => this.toggleEdgeDirections());
    this.deleteSelectionButton.addEventListener("click", () => this.deleteSelection());
    this.brushSizeInput?.addEventListener("input", () => this.syncRasterControls());
    this.undoRasterButton?.addEventListener("click", () => this.undoRaster());
    this.redoRasterButton?.addEventListener("click", () => this.redoRaster());
    this.toolButtons.forEach((button) => {
      button.addEventListener("click", () => this.setTool(button.dataset.tool || "select"));
    });

    this.landmarkNameInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.landmarkXInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.landmarkYInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.landmarkIgnoreDirInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.landmarkCanWaitInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.edgeKindSelect.addEventListener("change", () => this.applyEdgeInspector());
    this.edgeTypeInput.addEventListener("change", () => this.applyEdgeInspector());
    this.edgeTrafficSelect.addEventListener("change", () => this.applyEdgeInspector());
    this.edgeDirectionSelect.addEventListener("change", () => this.applyEdgeInspector());
    this.edgeControlledRegionInput.addEventListener("change", () => this.applyEdgeInspector());

    this.editorSvg.addEventListener("pointerdown", (event) => this.onPointerDown(event));
    this.editorSvg.addEventListener("pointermove", (event) => this.onPointerMove(event));
    this.editorSvg.addEventListener("pointerup", (event) => this.onPointerUp(event));
    this.editorSvg.addEventListener("pointerleave", (event) => this.onPointerUp(event));
    this.editorSvg.addEventListener("wheel", (event) => this.onWheel(event), { passive: false });
    window.addEventListener("keydown", (event) => this.onKeyDown(event));
    window.addEventListener("beforeunload", (event) => {
      if (!this.dirty || this.allowUnload) {
        return;
      }
      event.preventDefault();
      event.returnValue = "";
    });
  }

  async ensureBabylonEditor() {
    if (this.babylonScene) {
      await this.babylonScene.readyPromise;
      if (this.babylonScene.initError || !this.babylonScene.scene) {
        throw this.babylonScene.initError || new Error("Babylon.js did not initialize a scene.");
      }
      return this.babylonScene;
    }
    const module = await import("./scene3d.js");
    const scene = new module.OperatorScene3D(this.editorBabylon);
    scene.setHandlers({
      onPointerDown: (hit) => this.onBabylonPointerDown(hit),
      onPointerMove: (hit) => this.onBabylonPointerMove(hit),
      onPointerUp: (hit) => this.onBabylonPointerUp(hit),
      onContextMenu: (hit) => this.onBabylonContextMenu(hit),
    });
    scene.setViewMode("2d");
    scene.setLandmarkLabelsVisible(this.lmNamesVisible);
    scene.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
    this.babylonScene = scene;
    await scene.readyPromise;
    if (scene.initError || !scene.scene) {
      throw scene.initError || new Error("Babylon.js did not initialize a scene.");
    }
    this.editorSvg.classList.add("hidden");
    this.editorBabylon.classList.remove("hidden");
    scene.resize();
    this.renderCanvas({ force: true });
    window.requestAnimationFrame(() => {
      scene.resize();
      window.requestAnimationFrame(() => {
        scene.updateCamera();
        scene.scene?.render();
      });
    });
    return scene;
  }

  async goOperatorPage(path, options = {}) {
    if (!await this.confirmDirtyExit()) {
      return;
    }
    this.allowUnload = true;
    if (this.robotId) {
      window.localStorage.setItem("operator:selectedRobotId", this.robotId);
    }
    if (options.openRobots) {
      window.sessionStorage.setItem("operator:openSidebar", "1");
    }
    if (options.openAddRobot) {
      window.sessionStorage.setItem("operator:openAddRobot", "1");
    }
    window.location.assign(path);
  }

  async refreshAll(options = {}) {
    this.robot = {
      id: this.robotId,
      name: this.robotName || this.robotId,
    };

    try {
      const localPayload = await this.getJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/local`);
      this.localMaps = Array.isArray(localPayload.maps) ? localPayload.maps : [];
      const localActiveMapName = String(localPayload.activeMapName || "").trim();
      if (!this.currentMap && options.autoOpenLocal && localActiveMapName) {
        await this.openLocalDraft(localActiveMapName, { silent: true, activate: false });
        return;
      }
      this.render();
      if (!this.currentMap) {
        this.setStatus("No map is ready for editing. Pull or load a robot map from Control first.");
        return;
      }
      if (!options.silent) {
        this.setStatus("Map refreshed.");
      }
    } catch (error) {
      this.handleError(error);
      this.render();
    }
  }

  async refreshLocalMaps(options = {}) {
    try {
      const localPayload = await this.getJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/local`);
      this.localMaps = Array.isArray(localPayload.maps) ? localPayload.maps : [];
      if (!options.silent) {
        this.render();
      }
    } catch (error) {
      this.handleError(error);
    }
  }

  async openLocalDraft(mapName, options = {}) {
    if (this.dirty && !options.skipDirtyConfirm && !window.confirm("Discard unsaved map edits and reload the map?")) {
      return;
    }
    try {
      if (options.activate !== false) {
        await this.postJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/local/activate`, { mapName });
      }
      const payload = await this.getJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/local/${encodeURIComponent(mapName)}`);
      if (!payload.map || typeof payload.map !== "object") {
        throw new Error("Map payload is invalid.");
      }
      this.currentLocalMapName = String(payload.mapName || mapName || "");
      this.currentSourceMapName = String(payload.robotMapName || payload.sourceMapName || payload.map.mapName || "");
      this.selectedLocalMapName = this.currentLocalMapName;
      this.currentHasLocalChanges = Boolean(payload.hasLocalChanges);
      await this.loadEditableMap(payload.map);
      this.dirty = false;
      if (!options.silent) {
        this.setStatus("Map loaded.");
        this.log("info", "Map loaded.");
      }
      this.render();
    } catch (error) {
      this.handleError(error);
    }
  }

  async saveLocalDraft(options = {}) {
    if (!this.currentMap) {
      this.handleError(new Error("No map is loaded."));
      return null;
    }
    const defaultName = this.currentLocalMapName || this.currentMap.mapName || this.currentSourceMapName || "draft_map";
    const mapName = String(options.mapName || defaultName || "").trim();
    if (!mapName) {
      this.handleError(new Error("Map name is required."));
      return null;
    }
    try {
      this.syncRasterPayload();
      const editableMap = this.clone(this.currentMap);
      if (options.mapName) {
        editableMap.mapName = mapName.replace(/\.smap$/i, "");
      }
      const payload = await this.postJson(
        `/api/robots/${encodeURIComponent(this.robotId)}/maps/local/save`,
        {
          mapName,
          sourceMapName: this.currentSourceMapName || this.currentMap.mapName || mapName,
          map: editableMap,
        },
      );
      const local = payload.local || {};
      this.currentLocalMapName = String(local.mapName || mapName);
      this.selectedLocalMapName = this.currentLocalMapName;
      this.currentMap.mapName = String(editableMap.mapName || this.currentMap.mapName || mapName);
      this.dirty = false;
      this.rasterHistory.clear();
      this.currentHasLocalChanges = Boolean(local.hasLocalChanges);
      if (this.currentHasLocalChanges) {
        this.markPendingPush();
      } else {
        this.clearPendingPush();
      }
      await this.refreshLocalMaps({ silent: true });
      if (!options.silent) {
        this.setStatus("Map saved locally. Push uploads and verifies it; Load activates it on the robot.");
        this.log("info", "Map saved locally.");
      }
      this.render();
      return payload;
    } catch (error) {
      this.handleError(error);
      if (options.throwOnError) {
        throw error;
      }
      return null;
    }
  }

  async saveLocalDraftAs() {
    if (!this.currentMap) {
      this.handleError(new Error("No map is loaded."));
      return null;
    }
    const currentName = this.currentLocalMapName || this.currentMap.mapName || "draft_map";
    const suggestedName = `${String(currentName).replace(/\.smap$/i, "")}_copy`;
    const mapName = await this.promptSaveAsName(suggestedName);
    if (!mapName) {
      return null;
    }
    return this.saveLocalDraft({ mapName });
  }

  async pushToRobot(options = {}) {
    if (!this.currentMap) {
      this.handleError(new Error("No map is loaded."));
      return null;
    }
    if (!options.skipConfirm) {
      const confirmed = window.confirm(
        this.dirty
          ? "Save the current map and push it to the robot?"
          : "Push the saved map to the robot?",
      );
      if (!confirmed) {
        return null;
      }
    }
    try {
      if (this.dirty) {
        await this.saveLocalDraft({ silent: true, throwOnError: true });
      }
      const payload = await this.postJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/push-sync`, {});
      this.clearPendingPush();
      this.currentHasLocalChanges = false;
      await this.refreshLocalMaps({ silent: true });
      this.setStatus(payload.message || "Map uploaded and verified in robot storage. Use Load in Control to activate it.");
      this.log("info", payload.loadRequired ? "Map pushed; robot Load is required." : "Map pushed and verified.");
      this.render();
      return payload;
    } catch (error) {
      this.handleError(error);
      if (options.throwOnError) {
        throw error;
      }
      return null;
    }
  }

  async cancelMapChanges() {
    if (!this.currentMap) {
      this.handleError(new Error("No map is loaded."));
      return;
    }
    const hasSavedLocalChanges = this.hasSavedLocalChanges();
    if (!this.dirty && !hasSavedLocalChanges) {
      this.setStatus("No map changes to cancel.");
      return;
    }
    const confirmed = window.confirm(
      hasSavedLocalChanges
        ? "Cancel all local map changes and restore the current robot map?"
        : "Cancel unsaved map edits?",
    );
    if (!confirmed) {
      return;
    }
    try {
      let restoredMapName = this.currentLocalMapName || this.selectedLocalMapName;
      if (hasSavedLocalChanges) {
        const payload = await this.postJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/pull-sync`, {});
        restoredMapName = String(payload.localActiveMapName || payload.robotActiveMapName || restoredMapName || "").trim();
      }
      if (restoredMapName) {
        await this.openLocalDraft(restoredMapName, { silent: true, activate: false, skipDirtyConfirm: true });
      } else {
        await this.refreshAll({ autoOpenLocal: true, silent: true });
      }
      this.dirty = false;
      this.currentHasLocalChanges = false;
      this.clearPendingPush();
      await this.refreshLocalMaps({ silent: true });
      this.setStatus("Map changes canceled. Operator and robot are synced.");
      this.log("info", "Map changes canceled.");
      this.render();
    } catch (error) {
      this.handleError(error);
    }
  }

  async handleCloseEditor() {
    if (await this.confirmDirtyExit()) {
      this.closeWindow();
    }
  }

  async confirmDirtyExit() {
    if (!this.dirty) {
      return true;
    }
    const decision = await this.promptExitDecision();
    if (decision === "discard") {
      return true;
    }
    if (decision === "save") {
      const saved = await this.saveLocalDraft({ silent: true, throwOnError: false });
      if (saved) {
        this.setStatus("Map saved locally.");
        this.log("info", "Map saved locally before closing.");
      }
      return Boolean(saved);
    }
    if (decision === "save-push") {
      const saved = await this.saveLocalDraft({ silent: true, throwOnError: false });
      if (!saved) {
        return false;
      }
      const pushed = await this.pushToRobot({ skipConfirm: true });
      return Boolean(pushed);
    }
    return false;
  }

  promptExitDecision() {
    if (!this.editorExitDialog || typeof this.editorExitDialog.showModal !== "function") {
      const shouldSave = window.confirm("Save map changes before closing?");
      return Promise.resolve(shouldSave ? "save" : "discard");
    }
    return new Promise((resolve) => {
      this.exitDialogResolve = resolve;
      this.editorExitDialog.showModal();
    });
  }

  resolveExitDialog(decision) {
    if (!this.exitDialogResolve) {
      return;
    }
    const resolve = this.exitDialogResolve;
    this.exitDialogResolve = null;
    if (this.editorExitDialog?.open) {
      this.editorExitDialog.close();
    }
    resolve(decision);
  }

  promptSaveAsName(suggestedName) {
    if (!this.saveAsDialog || typeof this.saveAsDialog.showModal !== "function") {
      return Promise.resolve((window.prompt("Save local map as", suggestedName) || "").trim());
    }
    this.saveAsNameInput.value = suggestedName;
    return new Promise((resolve) => {
      this.saveAsDialogResolve = resolve;
      this.saveAsDialog.showModal();
      window.requestAnimationFrame(() => {
        this.saveAsNameInput.focus();
        this.saveAsNameInput.select();
      });
    });
  }

  resolveSaveAsDialog(mapName) {
    if (!this.saveAsDialogResolve) {
      return;
    }
    const resolve = this.saveAsDialogResolve;
    this.saveAsDialogResolve = null;
    if (this.saveAsDialog?.open) {
      this.saveAsDialog.close();
    }
    resolve(String(mapName || "").trim());
  }

  async loadEditableMap(payload) {
    this.currentMap = this.clone(payload);
    this.currentMap.mapName = String(this.currentMap.mapName || this.currentSourceMapName || "map").trim() || "map";
    this.currentMap.lms = Array.isArray(this.currentMap.lms) ? this.currentMap.lms : [];
    this.currentMap.edges = Array.isArray(this.currentMap.edges) ? this.currentMap.edges : [];
    this.currentMap.map = this.currentMap.map || {};
    this.selection = { type: "none", key: "" };
    this.previewWorld = null;
    this.previewSnapName = "";
    this.rasterGrid = null;
    this.rasterHistory.clear();
    const map = this.currentMap.map;
    if (map.imageDataUrl && Number(map.width) > 0 && Number(map.height) > 0) {
      try {
        this.rasterGrid = await OccupancyGrid.fromImageDataUrl(
          map.imageDataUrl,
          Number(map.width),
          Number(map.height),
        );
      } catch (error) {
        this.log("warn", `Raster tools are unavailable: ${error.message || error}`);
      }
    }
    this.babylonRevision += 1;
    this.babylonRenderedRevision = -1;
    this.recomputeAllEdgeLengths();
    this.resetView();
  }

  setTool(tool) {
    const allowed = ["select", "lm", "edge", "corridor", ...RASTER_TOOLS];
    this.selectedTool = allowed.includes(tool) ? tool : "select";
    if (RASTER_TOOLS.has(this.selectedTool) && !this.rasterGrid) {
      this.selectedTool = "select";
      this.setStatus("Raster tools require a loaded occupancy map.");
    }
    if (this.selectedTool !== "edge") {
      this.previewWorld = null;
      this.previewSnapName = "";
      if (this.dragState?.type === "edge_chain") {
        this.dragState = null;
      }
    }
    this.toolButtons.forEach((button) => {
      button.classList.toggle("active", button.dataset.tool === this.selectedTool);
    });
    this.syncRasterControls();
    this.renderInteraction();
  }

  syncRasterControls() {
    const brushSize = Math.max(1, Math.floor(Number(this.brushSizeInput?.value) || 1));
    if (this.brushSizeOutput) {
      this.brushSizeOutput.textContent = `${brushSize} px`;
    }
    if (this.undoRasterButton) {
      this.undoRasterButton.disabled = !this.rasterHistory.canUndo;
    }
    if (this.redoRasterButton) {
      this.redoRasterButton.disabled = !this.rasterHistory.canRedo;
    }
    for (const button of this.toolButtons) {
      if (RASTER_TOOLS.has(button.dataset.tool || "")) {
        button.disabled = !this.rasterGrid;
      }
    }
  }

  onKeyDown(event) {
    const target = event.target;
    if (target instanceof HTMLInputElement || target instanceof HTMLSelectElement || target instanceof HTMLTextAreaElement) {
      return;
    }
    const key = event.key.toLowerCase();
    if ((event.ctrlKey || event.metaKey) && (key === "z" || key === "y")) {
      event.preventDefault();
      if (this.historyGestureActive()) {
        return;
      }
      if (key === "y" || event.shiftKey) {
        this.redoRaster();
      } else {
        this.undoRaster();
      }
    }
  }

  historyGestureActive() {
    return [
      "raster_stroke",
      "raster_rectangle",
      "corridor_rectangle",
      "landmark",
      "handle",
      "edge_chain",
    ].includes(this.dragState?.type);
  }

  undoRaster() {
    if (this.historyGestureActive()) {
      return;
    }
    if (this.rasterHistory.undo()) {
      this.afterHistoryMutation("Map edit undone.");
    }
  }

  redoRaster() {
    if (this.historyGestureActive()) {
      return;
    }
    if (this.rasterHistory.redo()) {
      this.afterHistoryMutation("Map edit restored.");
    }
  }

  graphSnapshot() {
    return {
      lms: cloneJson(Array.isArray(this.currentMap?.lms) ? this.currentMap.lms : []),
      edges: cloneJson(Array.isArray(this.currentMap?.edges) ? this.currentMap.edges : []),
      trafficZones: cloneJson(Array.isArray(this.currentMap?.trafficZones) ? this.currentMap.trafficZones : []),
      selection: cloneJson(this.selection || { type: "none", key: "" }),
    };
  }

  restoreGraphSnapshot(snapshot) {
    if (!this.currentMap || !snapshot) {
      return;
    }
    this.currentMap.lms = cloneJson(snapshot.lms || []);
    this.currentMap.edges = cloneJson(snapshot.edges || []);
    this.currentMap.trafficZones = cloneJson(snapshot.trafficZones || []);
    this.selection = cloneJson(snapshot.selection || { type: "none", key: "" });
  }

  commitGraphHistory(before, label = "Graph edit") {
    if (!before) {
      return false;
    }
    const after = this.graphSnapshot();
    if (JSON.stringify(before) === JSON.stringify(after)) {
      this.dirty = this.rasterHistory.canUndo;
      this.renderWorkflowSummary();
      return false;
    }
    this.rasterHistory.push({
      label,
      undo: () => this.restoreGraphSnapshot(before),
      redo: () => this.restoreGraphSnapshot(after),
    });
    this.afterHistoryMutation(label);
    return true;
  }

  afterHistoryMutation(message = "") {
    this.dirty = this.rasterHistory.canUndo;
    this.babylonRevision += 1;
    if (this.rasterGrid) {
      this.scheduleRasterPreview(true);
      if (!this.babylonScene || this.babylonFailed) {
        this.currentMap.map.imageDataUrl = this.rasterGrid.toDataUrl();
      }
    }
    this.render();
    if (message) {
      this.log("info", message);
    }
  }

  syncRasterPayload() {
    if (!this.currentMap?.map || !this.rasterGrid) {
      return;
    }
    this.currentMap.map.imageDataUrl = this.rasterGrid.toDataUrl();
    this.currentMap.map.raster = this.rasterGrid.toPayload();
  }

  rasterPoint(world) {
    const resolution = Math.max(0.000001, Number(this.currentMap?.map?.resolution || 1));
    return this.rasterGrid?.normalizePoint({
      x: Number(world?.x || 0) / resolution,
      y: Number(world?.y || 0) / resolution,
    }) || null;
  }

  rasterValueForTool(tool = this.selectedTool) {
    if (tool === "eraser") {
      return OCCUPANCY_VALUES.free;
    }
    if (tool === "unknown") {
      return OCCUPANCY_VALUES.unknown;
    }
    return OCCUPANCY_VALUES.occupied;
  }

  commitRasterPatch(patch, message) {
    const command = this.rasterGrid?.commandForPatch(patch);
    if (!command) {
      return false;
    }
    this.rasterHistory.push(command);
    this.afterRasterMutation(message || `${command.label} changed ${command.pixelCount} cells.`);
    return true;
  }

  afterRasterMutation(message = "") {
    this.dirty = true;
    this.scheduleRasterPreview(true);
    if (!this.babylonScene || this.babylonFailed) {
      this.currentMap.map.imageDataUrl = this.rasterGrid.toDataUrl();
      this.editorMapImage.setAttribute("href", this.currentMap.map.imageDataUrl);
    }
    this.renderWorkflowSummary();
    this.syncRasterControls();
    this.renderCanvas();
    if (message) {
      this.log("info", message);
    }
  }

  scheduleRasterPreview(immediate = false) {
    if (!this.babylonScene || this.babylonFailed) {
      return;
    }
    if (immediate) {
      if (this.rasterPreviewTimer) {
        window.clearTimeout(this.rasterPreviewTimer);
        this.rasterPreviewTimer = 0;
      }
      this.babylonScene.updateFloorCanvas();
      return;
    }
    if (this.rasterPreviewTimer) {
      return;
    }
    this.rasterPreviewTimer = window.setTimeout(() => {
      this.rasterPreviewTimer = 0;
      this.babylonScene?.updateFloorCanvas();
    }, 50);
  }

  resetView() {
    if (!this.currentMap || !this.currentMap.map) {
      return;
    }
    this.view = {
      x: 0,
      y: 0,
      width: Number(this.currentMap.map.viewWidth || 100),
      height: Number(this.currentMap.map.viewHeight || 100),
    };
    this.applyViewBox();
    this.babylonScene?.resetView();
  }

  zoomView(scale, anchor = null) {
    if (!this.currentMap) {
      return;
    }
    if (this.babylonScene && !this.babylonFailed) {
      this.babylonScene.zoomBy(1 / Math.max(0.01, Number(scale || 1)));
      return;
    }
    const mapMeta = this.currentMap.map || {};
    const maxWidth = Number(mapMeta.viewWidth || 100);
    const maxHeight = Number(mapMeta.viewHeight || 100);
    const minWidth = Math.max(80, maxWidth * 0.08);
    const minHeight = Math.max(80, maxHeight * 0.08);
    const nextWidth = this.clamp(this.view.width * scale, minWidth, maxWidth * 3);
    const nextHeight = this.clamp(this.view.height * scale, minHeight, maxHeight * 3);
    const focus = anchor || {
      x: this.view.x + (this.view.width / 2),
      y: this.view.y + (this.view.height / 2),
    };
    const ratioX = (focus.x - this.view.x) / this.view.width;
    const ratioY = (focus.y - this.view.y) / this.view.height;
    this.view = {
      x: focus.x - (ratioX * nextWidth),
      y: focus.y - (ratioY * nextHeight),
      width: nextWidth,
      height: nextHeight,
    };
    this.applyViewBox();
  }

  toggleLmNames() {
    this.lmNamesVisible = !this.lmNamesVisible;
    preferences.setBoolean("lmNamesVisible", this.lmNamesVisible);
    this.babylonScene?.setLandmarkLabelsVisible(this.lmNamesVisible);
    this.syncLmNamesButton();
    if (!this.babylonScene || this.babylonFailed) {
      this.renderLandmarks();
    }
  }

  syncLmNamesButton() {
    if (!this.lmNamesButton) {
      return;
    }
    this.lmNamesButton.classList.toggle("active", this.lmNamesVisible);
    this.lmNamesButton.textContent = `LM names: ${this.lmNamesVisible ? "On" : "Off"}`;
    this.lmNamesButton.setAttribute("aria-pressed", String(this.lmNamesVisible));
  }

  toggleEdgeDirections() {
    this.edgeDirectionsVisible = !this.edgeDirectionsVisible;
    preferences.setBoolean("edgeDirectionsVisible", this.edgeDirectionsVisible);
    this.babylonScene?.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
    this.syncEdgeDirectionsButton();
    if (!this.babylonScene || this.babylonFailed) {
      this.renderEdges();
    }
  }

  syncEdgeDirectionsButton() {
    if (!this.edgeDirectionsButton) {
      return;
    }
    this.edgeDirectionsButton.classList.toggle("active", this.edgeDirectionsVisible);
    this.edgeDirectionsButton.textContent = `Edge directions: ${this.edgeDirectionsVisible ? "On" : "Off"}`;
    this.edgeDirectionsButton.setAttribute("aria-pressed", String(this.edgeDirectionsVisible));
  }

  applyViewBox() {
    this.editorSvg.setAttribute("viewBox", `${this.view.x} ${this.view.y} ${this.view.width} ${this.view.height}`);
  }

  onWheel(event) {
    if (!this.currentMap) {
      return;
    }
    event.preventDefault();
    const point = this.eventToSvgPoint(event);
    this.zoomView(event.deltaY < 0 ? 0.92 : 1.08, point);
  }

  onPointerDown(event) {
    if (!this.currentMap || event.button !== 0) {
      return;
    }
    const svgPoint = this.eventToSvgPoint(event);
    const world = this.svgToWorld(svgPoint);
    if (RASTER_TOOLS.has(this.selectedTool)) {
      if (this.beginRasterPointer({
        pointerId: event.pointerId,
        button: event.button,
        world,
      })) {
        this.editorSvg.setPointerCapture(event.pointerId);
      }
      return;
    }
    if (this.selectedTool === "corridor") {
      if (this.beginCorridorPointer({
        pointerId: event.pointerId,
        button: event.button,
        world,
      })) {
        this.editorSvg.setPointerCapture(event.pointerId);
      }
      return;
    }
    const handle = event.target.closest("[data-handle-index]");
    if (handle && this.selectedTool === "select") {
      this.dragState = {
        type: "handle",
        edgeKey: handle.dataset.edgeKey || "",
        handleIndex: Number(handle.dataset.handleIndex || "0"),
        pointerId: event.pointerId,
        before: this.graphSnapshot(),
        moved: false,
      };
      this.editorSvg.setPointerCapture(event.pointerId);
      return;
    }

    const landmarkNode = event.target.closest("[data-lm-name]");
    if (landmarkNode) {
      const lmName = landmarkNode.dataset.lmName || "";
      if (this.selectedTool === "edge") {
        this.selection = { type: "lm", key: lmName };
        this.dragState = {
          type: "edge_chain",
          currentLm: lmName,
          lastCreated: "",
          pointerId: event.pointerId,
          before: this.graphSnapshot(),
        };
        this.previewWorld = world;
        this.previewSnapName = lmName;
        this.editorSvg.setPointerCapture(event.pointerId);
        this.renderInteraction();
        return;
      }
      this.selection = { type: "lm", key: lmName };
      if (this.selectedTool === "select") {
        this.dragState = {
          type: "landmark",
          name: lmName,
          pointerId: event.pointerId,
          before: this.graphSnapshot(),
          moved: false,
        };
        this.editorSvg.setPointerCapture(event.pointerId);
      }
      this.renderInteraction();
      return;
    }

    const edgeNode = event.target.closest("[data-edge-key]");
    if (edgeNode && this.selectedTool === "select") {
      this.selection = { type: "edge", key: edgeNode.dataset.edgeKey || "" };
      this.renderInteraction();
      return;
    }

    if (this.selectedTool === "lm") {
      const before = this.graphSnapshot();
      const name = this.addLandmark(world);
      this.commitGraphHistory(before, `Added landmark ${name}.`);
      return;
    }

    if (this.selectedTool === "edge") {
      const nearest = this.nearestLandmark(world, 0.28);
      if (nearest) {
        this.selection = { type: "lm", key: nearest.name };
        this.dragState = {
          type: "edge_chain",
          currentLm: nearest.name,
          lastCreated: "",
          pointerId: event.pointerId,
          before: this.graphSnapshot(),
        };
        this.previewWorld = world;
        this.previewSnapName = nearest.name;
        this.editorSvg.setPointerCapture(event.pointerId);
        this.renderInteraction();
        return;
      }
    }

    this.selection = { type: "none", key: "" };
    this.dragState = {
      type: "pan",
      start: svgPoint,
      origin: { ...this.view },
    };
    this.editorSvg.setPointerCapture(event.pointerId);
    this.renderInteraction();
  }

  onPointerMove(event) {
    if (!this.currentMap) {
      return;
    }
    const svgPoint = this.eventToSvgPoint(event);
    if (["raster_stroke", "raster_rectangle"].includes(this.dragState?.type)) {
      this.moveRasterPointer({
        pointerId: event.pointerId,
        world: this.svgToWorld(svgPoint),
      });
      return;
    }
    if (this.dragState?.type === "corridor_rectangle") {
      this.moveCorridorPointer({
        pointerId: event.pointerId,
        world: this.svgToWorld(svgPoint),
      });
      return;
    }
    if (this.dragState?.type === "pan") {
      const dx = svgPoint.x - this.dragState.start.x;
      const dy = svgPoint.y - this.dragState.start.y;
      this.view = {
        ...this.dragState.origin,
        x: this.dragState.origin.x - dx,
        y: this.dragState.origin.y - dy,
      };
      this.applyViewBox();
      return;
    }
    if (this.dragState?.type === "landmark") {
      const world = this.svgToWorld(svgPoint);
      this.moveLandmark(this.dragState.name, world);
      this.dragState.moved = true;
      return;
    }
    if (this.dragState?.type === "handle") {
      const world = this.svgToWorld(svgPoint);
      this.moveCurveHandle(this.dragState.edgeKey, this.dragState.handleIndex, world);
      this.dragState.moved = true;
      return;
    }
    if (this.dragState?.type === "edge_chain") {
      const world = this.svgToWorld(svgPoint);
      const nearest = this.nearestLandmark(world, 0.28);
      if (
        nearest &&
        nearest.name !== this.dragState.currentLm &&
        nearest.name !== this.dragState.lastCreated
      ) {
        const previous = this.dragState.currentLm;
        this.createEdge(previous, nearest.name);
        this.dragState.lastCreated = previous;
        this.dragState.currentLm = nearest.name;
      }
      this.previewWorld = world;
      this.previewSnapName = nearest ? nearest.name : "";
      this.renderPreview();
    }
  }

  onPointerUp(event = null) {
    if (["raster_stroke", "raster_rectangle"].includes(this.dragState?.type)) {
      const world = event ? this.svgToWorld(this.eventToSvgPoint(event)) : null;
      this.endRasterPointer({
        pointerId: event?.pointerId ?? this.dragState.pointerId,
        world,
      });
      return;
    }
    if (this.dragState?.type === "corridor_rectangle") {
      const world = event ? this.svgToWorld(this.eventToSvgPoint(event)) : null;
      this.endCorridorPointer({
        pointerId: event?.pointerId ?? this.dragState.pointerId,
        world,
      });
      return;
    }
    const drag = this.dragState;
    this.dragState = null;
    this.previewWorld = null;
    this.previewSnapName = "";
    if (drag?.type === "landmark" && drag.moved) {
      this.commitGraphHistory(drag.before, `Moved landmark ${drag.name}.`);
      return;
    }
    if (drag?.type === "handle" && drag.moved) {
      this.commitGraphHistory(drag.before, `Updated curve ${drag.edgeKey}.`);
      return;
    }
    if (drag?.type === "edge_chain") {
      if (this.commitGraphHistory(drag.before, "Added graph edge chain.")) {
        return;
      }
    }
    this.renderInteraction();
  }

  onBabylonPointerDown(hit) {
    if (!this.currentMap || Number(hit?.button || 0) !== 0 || !hit?.world) {
      return false;
    }
    if (RASTER_TOOLS.has(this.selectedTool)) {
      return this.beginRasterPointer(hit);
    }
    if (this.selectedTool === "corridor") {
      return this.beginCorridorPointer(hit);
    }
    if (hit.bezierIndex && hit.edgeKey && this.selectedTool === "select") {
      this.selection = { type: "edge", key: hit.edgeKey };
      this.dragState = {
        type: "handle",
        edgeKey: hit.edgeKey,
        handleIndex: Number(hit.bezierIndex),
        pointerId: hit.pointerId,
        before: this.graphSnapshot(),
        moved: false,
      };
      this.renderInteraction();
      return true;
    }
    if (hit.lmName) {
      this.selection = { type: "lm", key: hit.lmName };
      if (this.selectedTool === "edge") {
        this.dragState = {
          type: "edge_chain",
          currentLm: hit.lmName,
          lastCreated: "",
          pointerId: hit.pointerId,
          before: this.graphSnapshot(),
        };
        this.previewWorld = { ...hit.world };
        this.previewSnapName = hit.lmName;
      } else if (this.selectedTool === "select") {
        this.dragState = {
          type: "landmark",
          name: hit.lmName,
          pointerId: hit.pointerId,
          before: this.graphSnapshot(),
          moved: false,
        };
      }
      this.renderInteraction();
      return true;
    }
    if (hit.edgeKey && this.selectedTool === "select") {
      this.selection = { type: "edge", key: hit.edgeKey };
      this.renderInteraction();
      return true;
    }
    if (this.selectedTool === "lm") {
      const before = this.graphSnapshot();
      const name = this.addLandmark(hit.world);
      this.commitGraphHistory(before, `Added landmark ${name}.`);
      return true;
    }
    this.selection = { type: "none", key: "" };
    this.renderInteraction();
    return false;
  }

  onBabylonPointerMove(hit) {
    if (!this.currentMap || !hit?.world || this.dragState?.pointerId !== hit.pointerId) {
      return;
    }
    if (this.dragState.type === "raster_stroke" || this.dragState.type === "raster_rectangle") {
      this.moveRasterPointer(hit);
      return;
    }
    if (this.dragState.type === "corridor_rectangle") {
      this.moveCorridorPointer(hit);
      return;
    }
    if (this.dragState.type === "landmark") {
      this.moveLandmark(this.dragState.name, hit.world);
      this.dragState.moved = true;
      return;
    }
    if (this.dragState.type === "handle") {
      this.moveCurveHandle(this.dragState.edgeKey, this.dragState.handleIndex, hit.world);
      this.dragState.moved = true;
      return;
    }
    if (this.dragState.type === "edge_chain") {
      const nearest = this.nearestLandmark(hit.world, 0.35);
      if (
        nearest
        && nearest.name !== this.dragState.currentLm
        && nearest.name !== this.dragState.lastCreated
      ) {
        const previous = this.dragState.currentLm;
        this.createEdge(previous, nearest.name);
        this.dragState.lastCreated = previous;
        this.dragState.currentLm = nearest.name;
      }
      this.previewWorld = { ...hit.world };
      this.previewSnapName = nearest?.name || "";
      this.renderBabylonCanvas();
    }
  }

  onBabylonPointerUp(hit) {
    if (this.dragState?.pointerId !== hit?.pointerId) {
      return;
    }
    if (this.dragState.type === "raster_stroke" || this.dragState.type === "raster_rectangle") {
      this.endRasterPointer(hit);
      return;
    }
    if (this.dragState.type === "corridor_rectangle") {
      this.endCorridorPointer(hit);
      return;
    }
    const drag = this.dragState;
    this.dragState = null;
    this.previewWorld = null;
    this.previewSnapName = "";
    if (drag?.type === "landmark" && drag.moved) {
      this.commitGraphHistory(drag.before, `Moved landmark ${drag.name}.`);
      return;
    }
    if (drag?.type === "handle" && drag.moved) {
      this.commitGraphHistory(drag.before, `Updated curve ${drag.edgeKey}.`);
      return;
    }
    if (drag?.type === "edge_chain") {
      if (this.commitGraphHistory(drag.before, "Added graph edge chain.")) {
        return;
      }
    }
    this.renderBabylonCanvas();
  }

  beginRasterPointer(hit) {
    if (!this.rasterGrid || !hit?.world) {
      return false;
    }
    const point = this.rasterPoint(hit.world);
    const value = this.rasterValueForTool();
    if (this.selectedTool === "fill") {
      const patch = this.rasterGrid.beginPatch("Fill");
      this.rasterGrid.floodFill(patch, point, value);
      this.commitRasterPatch(patch, "Connected area filled.");
      return true;
    }
    if (this.selectedTool === "rectangle") {
      this.dragState = {
        type: "raster_rectangle",
        pointerId: hit.pointerId,
        start: point,
        current: point,
        startWorld: { ...hit.world },
        currentWorld: { ...hit.world },
        value,
      };
      this.renderCanvas();
      return true;
    }
    const patch = this.rasterGrid.beginPatch(
      this.selectedTool === "eraser"
        ? "Erase"
        : this.selectedTool === "unknown"
          ? "Unknown brush"
          : "Pencil",
    );
    const size = Math.max(1, Math.floor(Number(this.brushSizeInput?.value) || 1));
    this.rasterGrid.paintSquareLine(patch, point, point, size, value);
    this.dragState = {
      type: "raster_stroke",
      pointerId: hit.pointerId,
      patch,
      last: point,
      size,
      value,
    };
    this.scheduleRasterPreview();
    return true;
  }

  moveRasterPointer(hit) {
    const point = this.rasterPoint(hit?.world);
    if (!point || !this.dragState) {
      return;
    }
    if (this.dragState.type === "raster_rectangle") {
      this.dragState.current = point;
      this.dragState.currentWorld = { ...hit.world };
      this.renderCanvas();
      return;
    }
    if (this.dragState.type !== "raster_stroke") {
      return;
    }
    this.rasterGrid.paintSquareLine(
      this.dragState.patch,
      this.dragState.last,
      point,
      this.dragState.size,
      this.dragState.value,
    );
    this.dragState.last = point;
    this.scheduleRasterPreview();
  }

  endRasterPointer(hit) {
    const drag = this.dragState;
    this.dragState = null;
    if (!drag || !this.rasterGrid) {
      return;
    }
    if (drag.type === "raster_rectangle") {
      const patch = this.rasterGrid.beginPatch("Rectangle");
      this.rasterGrid.paintRectangle(
        patch,
        drag.start,
        hit?.world ? this.rasterPoint(hit.world) : drag.current,
        drag.value,
      );
      if (!this.commitRasterPatch(patch, "Occupied rectangle added.")) {
        this.renderCanvas();
      }
      return;
    }
    if (drag.type === "raster_stroke") {
      if (!this.commitRasterPatch(drag.patch)) {
        this.renderCanvas();
      }
    }
  }

  onBabylonContextMenu(hit) {
    if (hit?.lmName && window.confirm(`Delete ${hit.lmName}?`)) {
      this.selection = { type: "lm", key: hit.lmName };
      this.deleteSelection();
      return;
    }
    if (hit?.edgeKey && window.confirm(`Delete edge ${hit.edgeKey}?`)) {
      this.selection = { type: "edge", key: hit.edgeKey };
      this.deleteSelection();
    }
  }

  beginCorridorPointer(hit) {
    if (!hit?.world) {
      return false;
    }
    this.dragState = {
      type: "corridor_rectangle",
      pointerId: hit.pointerId,
      startWorld: { ...hit.world },
      currentWorld: { ...hit.world },
      before: this.graphSnapshot(),
    };
    this.setStatus("Drag around the narrow graph aisle, then release.");
    this.renderCanvas();
    return true;
  }

  moveCorridorPointer(hit) {
    if (this.dragState?.type !== "corridor_rectangle" || !hit?.world) {
      return;
    }
    this.dragState.currentWorld = { ...hit.world };
    this.renderCanvas();
  }

  endCorridorPointer(hit) {
    const drag = this.dragState;
    this.dragState = null;
    if (!drag || drag.type !== "corridor_rectangle") {
      return;
    }
    const result = markControlledCorridorArea(
      this.currentMap,
      drag.startWorld,
      hit?.world || drag.currentWorld,
    );
    if (!result) {
      this.setStatus("The corridor rectangle does not cross any graph edge.");
      this.renderCanvas();
      return;
    }
    const regionCount = result.regions.length;
    this.commitGraphHistory(
      drag.before,
      `Added ${regionCount} controlled corridor rectangle; Core will compile ${result.edgeCount} intersecting directed edges and external stop lines.`,
    );
  }

  addLandmark(world) {
    if (!this.currentMap) {
      return;
    }
    const name = this.uniqueLandmarkName();
    this.currentMap.lms.push({
      name,
      x: this.round(world.x),
      y: this.round(world.y),
      properties: {},
      ignoreDir: "",
    });
    this.selection = { type: "lm", key: name };
    this.markDirty(`Added landmark ${name}.`);
    return name;
  }

  moveLandmark(name, world) {
    const landmark = this.landmarkByName(name);
    if (!landmark) {
      return;
    }
    landmark.x = this.round(world.x);
    landmark.y = this.round(world.y);
    this.refreshConnectedEdges(name);
    this.markDirty(`Moved landmark ${name}.`, { quietLog: true });
  }

  moveCurveHandle(edgeKey, handleIndex, world) {
    const edge = this.edgeByKey(edgeKey);
    if (!edge || !Array.isArray(edge.control_points) || edge.control_points.length !== 4) {
      return;
    }
    if (handleIndex !== 1 && handleIndex !== 2) {
      return;
    }
    edge.control_points[handleIndex] = {
      x: this.round(world.x),
      y: this.round(world.y),
    };
    edge.world_points = this.edgeWorldPoints(edge);
    edge.length = this.round(this.edgeLength(edge));
    this.markDirty(`Updated curve handle for ${edge.from} -> ${edge.to}.`, { quietLog: true });
  }

  createEdge(fromName, toName) {
    if (!this.currentMap || !fromName || !toName || fromName === toName) {
      return;
    }
    if (this.edgeByNames(fromName, toName)) {
      this.log("warn", `Edge ${fromName} -> ${toName} already exists.`);
      return;
    }
    const start = this.landmarkByName(fromName);
    const end = this.landmarkByName(toName);
    if (!start || !end) {
      return;
    }
    const edge = {
      from: fromName,
      to: toName,
      kind: "line",
      type: "FeatureLine",
      properties: { direction: -1 },
      length: this.distance(start, end),
      world_points: [
        { x: start.x, y: start.y },
        { x: end.x, y: end.y },
      ],
    };
    this.currentMap.edges.push(edge);
    this.selection = { type: "edge", key: this.edgeKey(edge) };
    this.markDirty(`Added edge ${fromName} -> ${toName}.`);
  }

  deleteSelection() {
    if (!this.currentMap) {
      return;
    }
    const before = this.graphSnapshot();
    if (this.selection.type === "lm") {
      const name = this.selection.key;
      this.currentMap.lms = this.currentMap.lms.filter((item) => item.name !== name);
      this.currentMap.edges = this.currentMap.edges.filter((edge) => edge.from !== name && edge.to !== name);
      this.selection = { type: "none", key: "" };
      this.commitGraphHistory(before, `Removed landmark ${name}.`);
      return;
    }
    if (this.selection.type === "edge") {
      const key = this.selection.key;
      this.currentMap.edges = this.currentMap.edges.filter((edge) => this.edgeKey(edge) !== key);
      this.selection = { type: "none", key: "" };
      this.commitGraphHistory(before, `Removed edge ${key}.`);
    }
  }

  applyLandmarkInspector() {
    if (this.selection.type !== "lm") {
      return;
    }
    const current = this.landmarkByName(this.selection.key);
    if (!current) {
      return;
    }
    const nextName = String(this.landmarkNameInput.value || "").trim();
    const nextX = Number(this.landmarkXInput.value);
    const nextY = Number(this.landmarkYInput.value);
    const nextIgnoreDir = this.landmarkIgnoreDirInput.value.trim();
    const canWait = Boolean(this.landmarkCanWaitInput.checked);

    if (!nextName) {
      this.handleError(new Error("LM name is required."));
      this.renderInspector();
      return;
    }
    if (nextName !== current.name && this.landmarkByName(nextName)) {
      this.handleError(new Error(`LM ${nextName} already exists.`));
      this.renderInspector();
      return;
    }
    const before = this.graphSnapshot();
    const previousName = current.name;
    current.name = nextName;
    current.x = this.round(Number.isFinite(nextX) ? nextX : current.x);
    current.y = this.round(Number.isFinite(nextY) ? nextY : current.y);
    current.ignoreDir = nextIgnoreDir;
    current.properties = current.properties && typeof current.properties === "object"
      ? current.properties
      : {};
    current.properties.can_wait = canWait;
    if (previousName !== nextName) {
      for (const edge of this.currentMap.edges) {
        if (edge.from === previousName) {
          edge.from = nextName;
        }
        if (edge.to === previousName) {
          edge.to = nextName;
        }
      }
      this.selection = { type: "lm", key: nextName };
    }
    this.refreshConnectedEdges(nextName);
    this.commitGraphHistory(before, `Updated landmark ${nextName}.`);
  }

  applyEdgeInspector() {
    if (this.selection.type !== "edge") {
      return;
    }
    const edge = this.edgeByKey(this.selection.key);
    if (!edge) {
      return;
    }
    const before = this.graphSnapshot();
    const nextKind = this.edgeKindSelect.value === "curve" ? "curve" : "line";
    const nextType = String(this.edgeTypeInput.value || "").trim() || (nextKind === "curve" ? "DegenerateBezier" : "FeatureLine");
    const direction = normalizeEdgeMotionCode(this.edgeDirectionSelect.value);
    const controlledRegion = String(this.edgeControlledRegionInput.value || "").trim();
    edge.kind = nextKind;
    edge.type = nextType;
    edge.properties = typeof edge.properties === "object" && edge.properties ? edge.properties : {};
    edge.properties.direction = direction;
    if (controlledRegion) {
      edge.properties.controlled_region = controlledRegion;
    } else {
      delete edge.properties.controlled_region;
    }
    if (nextKind === "curve") {
      this.ensureCurveGeometry(edge);
    } else {
      delete edge.geometry;
      delete edge.control_points;
      delete edge.curve_type;
    }
    edge.world_points = this.edgeWorldPoints(edge);
    edge.length = this.round(this.edgeLength(edge));
    let reverse = this.edgeByNames(edge.to, edge.from);
    const traffic = this.edgeTrafficSelect.value;
    if (traffic === "bidirectional" || traffic === "reverse") {
      const reverseEdge = reverse || {
        from: edge.to,
        to: edge.from,
        kind: edge.kind,
        type: edge.type,
        properties: {},
      };
      reverseEdge.kind = edge.kind;
      reverseEdge.type = edge.type;
      reverseEdge.properties = {
        ...(reverseEdge.properties || {}),
        ...edge.properties,
      };
      if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
        reverseEdge.geometry = "bezier";
        reverseEdge.curve_type = edge.curve_type;
        reverseEdge.control_points = [...edge.control_points]
          .reverse()
          .map((point) => ({ x: Number(point.x), y: Number(point.y) }));
      } else {
        delete reverseEdge.geometry;
        delete reverseEdge.control_points;
        delete reverseEdge.curve_type;
      }
      reverseEdge.world_points = this.edgeWorldPoints(reverseEdge);
      reverseEdge.length = this.round(this.edgeLength(reverseEdge));
      if (!reverse) {
        this.currentMap.edges.push(reverseEdge);
        reverse = reverseEdge;
      }
    }
    if (traffic === "one_way" && reverse) {
      this.currentMap.edges = this.currentMap.edges.filter((item) => item !== reverse);
    }
    if (traffic === "reverse" && reverse) {
      this.currentMap.edges = this.currentMap.edges.filter((item) => item !== edge);
      this.selection = { type: "edge", key: this.edgeKey(reverse) };
    }
    this.commitGraphHistory(before, `Updated edge ${edge.from} -> ${edge.to}.`);
  }

  ensureCurveGeometry(edge) {
    const start = this.landmarkByName(edge.from);
    const end = this.landmarkByName(edge.to);
    if (!start || !end) {
      return;
    }
    if (!Array.isArray(edge.control_points) || edge.control_points.length !== 4) {
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      edge.control_points = [
        { x: start.x, y: start.y },
        { x: this.round(start.x + (dx / 3)), y: this.round(start.y + (dy / 3)) },
        { x: this.round(start.x + ((dx * 2) / 3)), y: this.round(start.y + ((dy * 2) / 3)) },
        { x: end.x, y: end.y },
      ];
    }
    edge.control_points[0] = { x: start.x, y: start.y };
    edge.control_points[3] = { x: end.x, y: end.y };
    edge.geometry = "bezier";
    edge.curve_type = edge.curve_type || edge.type || "DegenerateBezier";
  }

  refreshConnectedEdges(lmName) {
    if (!this.currentMap) {
      return;
    }
    const start = this.landmarkByName(lmName);
    if (!start) {
      return;
    }
    for (const edge of this.currentMap.edges) {
      if (edge.from !== lmName && edge.to !== lmName) {
        continue;
      }
      if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
        const source = this.landmarkByName(edge.from);
        const target = this.landmarkByName(edge.to);
        if (source) {
          edge.control_points[0] = { x: source.x, y: source.y };
        }
        if (target) {
          edge.control_points[3] = { x: target.x, y: target.y };
        }
      }
      edge.world_points = this.edgeWorldPoints(edge);
      edge.length = this.round(this.edgeLength(edge));
    }
  }

  recomputeAllEdgeLengths() {
    if (!this.currentMap) {
      return;
    }
    for (const edge of this.currentMap.edges) {
      edge.world_points = this.edgeWorldPoints(edge);
      edge.length = this.round(this.edgeLength(edge));
    }
  }

  render() {
    this.syncLmNamesButton();
    this.syncEdgeDirectionsButton();
    this.syncRasterControls();
    this.renderHeader();
    this.renderWorkflowSummary();
    this.renderCanvas();
    this.renderInspector();
    this.renderLogs();
  }

  renderInteraction(options = {}) {
    this.syncLmNamesButton();
    this.syncEdgeDirectionsButton();
    this.renderCanvas();
    this.renderInspector();
    if (options.logs) {
      this.renderLogs();
    }
  }

  renderHeader() {
    const robotName = this.robot?.name || this.robotName || this.robotId;
    this.editorRobotTitle.textContent = robotName;
    if (!this.currentMap) {
      this.editorStatusText.textContent = "No map loaded.";
      return;
    }
    const state = this.mapWorkflowState();
    this.editorStatusText.textContent = state.message;
  }

  renderWorkflowSummary() {
    const state = this.mapWorkflowState();
    this.editorWorkflowHeadline.textContent = state.headline;
    this.workflowStatusTitle.textContent = state.title;
    this.workflowStateChip.textContent = state.chip;
    this.workflowStateChip.classList.toggle("dirty", state.kind !== "synced");
    this.workflowStateChip.classList.toggle("clean", state.kind === "synced");
    this.workflowStatusText.textContent = state.detail;
    this.workflowLmCountText.textContent = String(this.currentMap?.lms?.length || 0);
    this.workflowEdgeCountText.textContent = String(this.currentMap?.edges?.length || 0);
    const hasMap = Boolean(this.currentMap);
    const hasSavedLocalChanges = this.hasSavedLocalChanges();
    this.saveLocalButton.disabled = !hasMap || !this.dirty;
    this.saveAsButton.disabled = !hasMap;
    this.pushRobotButton.disabled = !hasMap || (!this.dirty && !hasSavedLocalChanges);
    this.cancelMapChangesButton.disabled = !hasMap || (!this.dirty && !hasSavedLocalChanges);
    if (!this.currentMap) {
      return;
    }
  }

  mapWorkflowState() {
    if (!this.currentMap) {
      return {
        kind: "empty",
        chip: "not ready",
        title: "No map loaded",
        headline: "No map loaded.",
        message: "No map loaded.",
        detail: "Pull or load a robot map from Control, then open the editor.",
      };
    }
    if (this.dirty) {
      return {
        kind: "unsaved",
        chip: "unsaved",
        title: "Unsaved edits",
        headline: "Unsaved edits in this editor.",
        message: "Unsaved edits. Save or push to apply them.",
        detail: "Save keeps changes on this PC. Push uploads and verifies them in the robot map library.",
      };
    }
    if (this.hasSavedLocalChanges()) {
      return {
        kind: "not_synced",
        chip: "not synced",
        title: "Not synced",
        headline: "Saved locally, not pushed to robot.",
        message: "Operator map is not synced with the robot.",
        detail: "Push uploads this map to robot storage. Use Load in Control to activate it.",
      };
    }
    return {
      kind: "synced",
      chip: "synced",
      title: "Synced",
      headline: "Operator map matches the robot storage copy.",
      message: "Operator and robot storage maps are synced.",
      detail: "Load is a separate Control action that activates a stored robot map.",
    };
  }

  hasSavedLocalChanges() {
    if (this.currentHasLocalChanges) {
      return true;
    }
    const activeName = this.currentLocalMapName || this.selectedLocalMapName;
    const active = this.localMaps.find((item) => item.mapName === activeName || item.active);
    return Boolean(active?.hasLocalChanges);
  }

  babylonScenePayload() {
    const map = this.currentMap?.map || {};
    const resolution = Math.max(0.000001, Number(map.resolution || 1));
    const width = Math.max(1, Number(map.width || 0) * resolution);
    const depth = Math.max(1, Number(map.height || 0) * resolution);
    return {
      ok: true,
      mapName: this.currentLocalMapName || this.currentMap?.mapName || "robot-map-editor",
      coordinateFrame: "map_top_left",
      floor: {
        width,
        depth,
        resolution,
        imageDataUrl: String(map.imageDataUrl || ""),
      },
      bounds: { minX: 0, minZ: 0, maxX: width, maxZ: depth },
      walls: [],
      lms: this.currentMap?.lms || [],
      edges: this.currentMap?.edges || [],
      trafficZones: this.currentMap?.trafficZones || [],
    };
  }

  babylonEditorState() {
    const selectedLmName = this.selection.type === "lm" ? this.selection.key : "";
    const selectedEdgeKey = this.selection.type === "edge" ? this.selection.key : "";
    const dragging = [
      "landmark",
      "handle",
      "edge_chain",
      "raster_rectangle",
      "corridor_rectangle",
    ].includes(this.dragState?.type);
    return {
      active: true,
      revision: this.babylonRevision,
      dragging,
      tool: this.selectedTool,
      selectedLmName,
      selectedEdgeKey,
      preview: this.dragState?.type === "edge_chain" && this.previewWorld
        ? { fromName: this.dragState.currentLm, world: this.previewWorld }
        : null,
      areaPreview: ["raster_rectangle", "corridor_rectangle"].includes(this.dragState?.type)
        ? {
            kind: this.dragState.type === "corridor_rectangle" ? "corridor" : "rectangle",
            start: this.dragState.startWorld,
            current: this.dragState.currentWorld,
          }
        : null,
      lms: this.currentMap?.lms || [],
      edges: this.currentMap?.edges || [],
    };
  }

  renderBabylonCanvas(options = {}) {
    if (!this.babylonScene || !this.currentMap || this.babylonFailed) {
      return false;
    }
    const dragging = [
      "landmark",
      "handle",
      "edge_chain",
      "raster_rectangle",
      "corridor_rectangle",
    ].includes(this.dragState?.type);
    if ((options.force || this.babylonRenderedRevision !== this.babylonRevision) && !dragging) {
      this.babylonScene.setScene(this.babylonScenePayload(), {
        preserveView: this.babylonRenderedRevision >= 0,
      });
      this.babylonRenderedRevision = this.babylonRevision;
    }
    if (this.rasterGrid) {
      this.babylonScene.setFloorCanvas(this.rasterGrid.canvas);
    }
    this.babylonScene.setViewMode("2d");
    this.babylonScene.setLandmarkLabelsVisible(this.lmNamesVisible);
    this.babylonScene.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
    this.babylonScene.setEditorState(this.babylonEditorState());
    this.babylonScene.updateRobots([], "", "");
    this.editorBabylon.dataset.mapName = String(this.currentLocalMapName || this.currentMap?.mapName || "");
    this.editorBabylon.dataset.landmarkCount = String(this.currentMap?.lms?.length || 0);
    this.editorBabylon.dataset.edgeCount = String(this.currentMap?.edges?.length || 0);
    this.editorBabylon.dataset.revision = String(this.babylonRenderedRevision);
    this.editorBabylon.dataset.meshCount = String(this.babylonScene.scene?.meshes?.length || 0);
    this.editorBabylon.dataset.bounds = `${this.babylonScene.bounds.width.toFixed(2)}x${this.babylonScene.bounds.depth.toFixed(2)}`;
    this.editorBabylon.dataset.distance = Number(this.babylonScene.distance || 0).toFixed(2);
    this.editorBabylon.dataset.target = `${this.babylonScene.target.x.toFixed(2)},${this.babylonScene.target.z.toFixed(2)}`;
    this.editorBabylon.dataset.camera = String(this.babylonScene.scene?.activeCamera?.name || "");
    return true;
  }

  renderCanvas(options = {}) {
    if (!this.currentMap) {
      this.editorMapImage.setAttribute("href", "");
      this.editorEdgeLayer.innerHTML = "";
      this.editorLmLayer.innerHTML = "";
      this.editorHandleLayer.innerHTML = "";
      this.editorPreviewLayer.innerHTML = "";
      return;
    }
    if (this.renderBabylonCanvas(options)) {
      return;
    }
    const mapMeta = this.currentMap.map || {};
    const padding = Number(mapMeta.viewPadding || 0);
    const width = Number(mapMeta.width || 0);
    const height = Number(mapMeta.height || 0);
    this.editorMapImage.setAttribute("x", String(padding));
    this.editorMapImage.setAttribute("y", String(padding));
    this.editorMapImage.setAttribute("width", String(width));
    this.editorMapImage.setAttribute("height", String(height));
    this.editorMapImage.setAttribute("href", String(mapMeta.imageDataUrl || ""));
    this.applyViewBox();
    this.renderEdges();
    this.renderLandmarks();
    this.renderHandles();
    this.renderPreview();
  }

  renderEdges() {
    this.editorEdgeLayer.innerHTML = "";
    const directedKeys = new Set(
      this.currentMap.edges.map((edge) => this.edgeKey(edge)),
    );
    for (const edge of this.currentMap.edges) {
      const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
      const selected = this.selection.type === "edge" && this.selection.key === this.edgeKey(edge);
      const controlled = Boolean(edge.properties?.controlled_region);
      path.setAttribute("fill", "none");
      path.setAttribute("stroke-width", selected ? "3.4" : (controlled ? "3" : "2.25"));
      path.setAttribute("stroke", selected ? "var(--edge-selected)" : (controlled ? "#d97706" : "var(--edge)"));
      path.setAttribute("stroke-linecap", "round");
      path.dataset.edgeKey = this.edgeKey(edge);
      path.setAttribute("d", this.edgePath(edge));
      this.editorEdgeLayer.append(path);
      const hasReverse = directedKeys.has(`${edge.to}->${edge.from}`);
      const arrow = this.edgeDirectionsVisible
        ? this.drawEdgeDirectionArrow(edge, hasReverse ? 0.56 : 0.5)
        : null;
      if (arrow) {
        this.editorEdgeLayer.append(arrow);
      }
    }
  }

  renderLandmarks() {
    this.editorLmLayer.innerHTML = "";
    this.syncLmNamesButton();
    const corridorHoldingLms = new Set(
      this.currentMap.edges
        .filter((edge) => edge.properties?.controlled_region)
        .flatMap((edge) => [edge.from, edge.to]),
    );
    for (const landmark of this.currentMap.lms) {
      const svgPoint = this.worldToSvg(landmark);
      const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
      group.dataset.lmName = landmark.name;

      const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      const controlled = Boolean(landmark.properties?.controlled_region);
      const holding = Boolean(
        landmark.properties?.holding_point
        && corridorHoldingLms.has(landmark.name)
      );
      const selected = this.selection.type === "lm" && this.selection.key === landmark.name;
      circle.setAttribute("cx", String(svgPoint.x));
      circle.setAttribute("cy", String(svgPoint.y));
      circle.setAttribute("r", selected ? "7.5" : "6");
      circle.setAttribute("fill", selected ? "var(--lm-selected)" : (controlled ? "#d97706" : "var(--lm)"));
      circle.setAttribute("stroke", holding ? "#d97706" : "#ffffff");
      circle.setAttribute("stroke-width", holding ? "3.5" : "2");
      group.append(circle);

      if (this.lmNamesVisible) {
        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("x", String(svgPoint.x));
        label.setAttribute("y", String(svgPoint.y + 17));
        label.setAttribute("text-anchor", "middle");
        label.setAttribute("font-size", "11");
        label.setAttribute("font-weight", "700");
        label.setAttribute("fill", "var(--text)");
        label.textContent = landmark.name;
        group.append(label);
      }

      this.editorLmLayer.append(group);
    }
  }

  renderHandles() {
    this.editorHandleLayer.innerHTML = "";
    if (this.selection.type !== "edge") {
      return;
    }
    const edge = this.edgeByKey(this.selection.key);
    if (!edge || !Array.isArray(edge.control_points) || edge.control_points.length !== 4) {
      return;
    }
    const points = edge.control_points.map((point) => this.worldToSvg(point));
    const guide1 = document.createElementNS("http://www.w3.org/2000/svg", "line");
    guide1.setAttribute("x1", String(points[0].x));
    guide1.setAttribute("y1", String(points[0].y));
    guide1.setAttribute("x2", String(points[1].x));
    guide1.setAttribute("y2", String(points[1].y));
    guide1.setAttribute("stroke", "rgba(63, 140, 255, 0.42)");
    guide1.setAttribute("stroke-width", "1.8");
    this.editorHandleLayer.append(guide1);

    const guide2 = document.createElementNS("http://www.w3.org/2000/svg", "line");
    guide2.setAttribute("x1", String(points[2].x));
    guide2.setAttribute("y1", String(points[2].y));
    guide2.setAttribute("x2", String(points[3].x));
    guide2.setAttribute("y2", String(points[3].y));
    guide2.setAttribute("stroke", "rgba(63, 140, 255, 0.42)");
    guide2.setAttribute("stroke-width", "1.8");
    this.editorHandleLayer.append(guide2);

    for (const handleIndex of [1, 2]) {
      const handlePoint = points[handleIndex];
      const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      circle.dataset.edgeKey = this.edgeKey(edge);
      circle.dataset.handleIndex = String(handleIndex);
      circle.setAttribute("cx", String(handlePoint.x));
      circle.setAttribute("cy", String(handlePoint.y));
      circle.setAttribute("r", "5.5");
      circle.setAttribute("fill", "var(--handle)");
      circle.setAttribute("stroke", "#ffffff");
      circle.setAttribute("stroke-width", "2");
      this.editorHandleLayer.append(circle);
    }
  }

  renderPreview() {
    this.editorPreviewLayer.innerHTML = "";
    for (const zone of this.currentMap?.trafficZones || []) {
      if (
        String(zone?.kind || "") !== "controlled_corridor"
        || String(zone?.shape || "rectangle") !== "rectangle"
      ) {
        continue;
      }
      const bounds = zone?.bounds || {};
      const start = this.worldToSvg({
        x: Number(bounds.minX),
        y: Number(bounds.minY),
      });
      const goal = this.worldToSvg({
        x: Number(bounds.maxX),
        y: Number(bounds.maxY),
      });
      if (![start.x, start.y, goal.x, goal.y].every(Number.isFinite)) {
        continue;
      }
      const rectangle = document.createElementNS("http://www.w3.org/2000/svg", "rect");
      rectangle.setAttribute("class", "editor-area-preview corridor saved");
      rectangle.dataset.trafficZoneId = String(zone.id || "");
      rectangle.setAttribute("x", String(Math.min(start.x, goal.x)));
      rectangle.setAttribute("y", String(Math.min(start.y, goal.y)));
      rectangle.setAttribute("width", String(Math.abs(goal.x - start.x)));
      rectangle.setAttribute("height", String(Math.abs(goal.y - start.y)));
      this.editorPreviewLayer.append(rectangle);
    }
    const guidePoint = this.currentGuidePoint();
    if (guidePoint) {
      this.drawGuideAtPoint(guidePoint);
    }
    if (
      ["raster_rectangle", "corridor_rectangle"].includes(this.dragState?.type)
      && this.dragState.startWorld
      && this.dragState.currentWorld
    ) {
      const start = this.worldToSvg(this.dragState.startWorld);
      const current = this.worldToSvg(this.dragState.currentWorld);
      const rectangle = document.createElementNS("http://www.w3.org/2000/svg", "rect");
      rectangle.setAttribute(
        "class",
        `editor-area-preview ${this.dragState.type === "corridor_rectangle" ? "corridor" : "raster"}`,
      );
      rectangle.setAttribute("x", String(Math.min(start.x, current.x)));
      rectangle.setAttribute("y", String(Math.min(start.y, current.y)));
      rectangle.setAttribute("width", String(Math.abs(current.x - start.x)));
      rectangle.setAttribute("height", String(Math.abs(current.y - start.y)));
      this.editorPreviewLayer.append(rectangle);
    }
    if (this.selectedTool !== "edge" || this.dragState?.type !== "edge_chain" || !this.dragState.currentLm || !this.previewWorld) {
      return;
    }
    const start = this.landmarkByName(this.dragState.currentLm);
    if (!start) {
      return;
    }
    const source = this.worldToSvg(start);
    const snapTarget = this.previewSnapName ? this.landmarkByName(this.previewSnapName) : null;
    const target = this.worldToSvg(snapTarget || this.previewWorld);
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttribute("x1", String(source.x));
    line.setAttribute("y1", String(source.y));
    line.setAttribute("x2", String(target.x));
    line.setAttribute("y2", String(target.y));
    line.setAttribute("stroke", "rgba(36, 105, 255, 0.7)");
    line.setAttribute("stroke-dasharray", "8 6");
    line.setAttribute("stroke-width", "2");
    this.editorPreviewLayer.append(line);
    if (snapTarget) {
      const ring = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      ring.setAttribute("cx", String(target.x));
      ring.setAttribute("cy", String(target.y));
      ring.setAttribute("r", "11");
      ring.setAttribute("fill", "none");
      ring.setAttribute("stroke", "rgba(36, 105, 255, 0.48)");
      ring.setAttribute("stroke-width", "2");
      this.editorPreviewLayer.append(ring);
    }
  }

  currentGuidePoint() {
    if (this.dragState?.type === "landmark") {
      return this.landmarkByName(this.dragState.name);
    }
    if (this.dragState?.type === "handle") {
      const edge = this.edgeByKey(this.dragState.edgeKey);
      if (!edge || !Array.isArray(edge.control_points) || edge.control_points.length !== 4) {
        return null;
      }
      return edge.control_points[this.dragState.handleIndex] || null;
    }
    if (this.dragState?.type === "edge_chain" && this.previewWorld) {
      return this.previewSnapName ? this.landmarkByName(this.previewSnapName) : this.previewWorld;
    }
    return null;
  }

  drawGuideAtPoint(point) {
    const svgPoint = this.worldToSvg(point);
    const mapMeta = this.currentMap?.map || {};
    const width = Number(mapMeta.viewWidth || 0);
    const height = Number(mapMeta.viewHeight || 0);

    const horizontal = document.createElementNS("http://www.w3.org/2000/svg", "line");
    horizontal.setAttribute("class", "editor-guide-line");
    horizontal.setAttribute("x1", "0");
    horizontal.setAttribute("y1", String(svgPoint.y));
    horizontal.setAttribute("x2", String(width));
    horizontal.setAttribute("y2", String(svgPoint.y));
    this.editorPreviewLayer.append(horizontal);

    const vertical = document.createElementNS("http://www.w3.org/2000/svg", "line");
    vertical.setAttribute("class", "editor-guide-line");
    vertical.setAttribute("x1", String(svgPoint.x));
    vertical.setAttribute("y1", "0");
    vertical.setAttribute("x2", String(svgPoint.x));
    vertical.setAttribute("y2", String(height));
    this.editorPreviewLayer.append(vertical);

    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("class", "editor-guide-label");
    label.setAttribute("x", String(svgPoint.x + 8));
    label.setAttribute("y", String(svgPoint.y - 8));
    label.textContent = `x ${Number(point.x || 0).toFixed(3)}, y ${Number(point.y || 0).toFixed(3)}`;
    this.editorPreviewLayer.append(label);
  }

  drawEdgeDirectionArrow(edge, fraction = 0.5) {
    const segment = this.graphDirectionSegment(edge, fraction);
    if (!segment) {
      return null;
    }
    const arrow = document.createElementNS("http://www.w3.org/2000/svg", "line");
    arrow.setAttribute("x1", String(segment.start.x));
    arrow.setAttribute("y1", String(segment.start.y));
    arrow.setAttribute("x2", String(segment.end.x));
    arrow.setAttribute("y2", String(segment.end.y));
    arrow.setAttribute("stroke", "#56616f");
    arrow.setAttribute("stroke-width", "1.35");
    arrow.setAttribute("stroke-linecap", "round");
    arrow.setAttribute("marker-end", "url(#edgeArrow)");
    return arrow;
  }

  graphDirectionSegment(edge, fraction = 0.5) {
    let mid;
    let tangent;
    if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      mid = this.bezierPoint(edge.control_points, fraction);
      tangent = this.bezierDerivative(edge.control_points, fraction);
    } else {
      const start = this.landmarkByName(edge.from);
      const goal = this.landmarkByName(edge.to);
      if (!start || !goal) {
        return null;
      }
      mid = {
        x: start.x + ((goal.x - start.x) * fraction),
        y: start.y + ((goal.y - start.y) * fraction),
      };
      tangent = {
        x: goal.x - start.x,
        y: goal.y - start.y,
      };
    }
    const length = Math.max(0.0001, Math.hypot(tangent.x, tangent.y));
    const ux = tangent.x / length;
    const uy = tangent.y / length;
    const half = 0.055;
    return {
      start: this.worldToSvg({
        x: mid.x - (ux * half),
        y: mid.y - (uy * half),
      }),
      end: this.worldToSvg({
        x: mid.x + (ux * half),
        y: mid.y + (uy * half),
      }),
    };
  }

  renderInspector() {
    this.selectionEmptyText.classList.toggle("hidden", this.selection.type !== "none");
    this.landmarkInspector.classList.toggle("hidden", this.selection.type !== "lm");
    this.edgeInspector.classList.toggle("hidden", this.selection.type !== "edge");

    if (this.selection.type === "lm") {
      const landmark = this.landmarkByName(this.selection.key);
      if (!landmark) {
        this.selection = { type: "none", key: "" };
        this.renderInspector();
        return;
      }
      this.selectionTitleText.textContent = `LM ${landmark.name}`;
      this.landmarkNameInput.value = landmark.name;
      this.landmarkXInput.value = String(landmark.x);
      this.landmarkYInput.value = String(landmark.y);
      this.landmarkIgnoreDirInput.value = String(landmark.ignoreDir || "");
      this.landmarkCanWaitInput.checked = (
        landmark.properties?.can_wait
        ?? landmark.properties?.canWait
        ?? true
      ) !== false;
      return;
    }

    if (this.selection.type === "edge") {
      const edge = this.edgeByKey(this.selection.key);
      if (!edge) {
        this.selection = { type: "none", key: "" };
        this.renderInspector();
        return;
      }
      this.selectionTitleText.textContent = `${edge.from} -> ${edge.to}`;
      this.edgeFromText.textContent = edge.from;
      this.edgeToText.textContent = edge.to;
      this.edgeKindSelect.value = edge.kind === "curve" ? "curve" : "line";
      this.edgeTypeInput.value = String(edge.type || "");
      this.edgeTrafficSelect.value = this.edgeByNames(edge.to, edge.from)
        ? "bidirectional"
        : "one_way";
      this.edgeDirectionSelect.value = String(
        normalizeEdgeMotionCode(edge.properties && edge.properties.direction),
      );
      this.edgeControlledRegionInput.value = String(
        edge.properties?.controlled_region
        || edge.properties?.controlledRegion
        || "",
      );
      this.edgeLengthText.textContent = `${Number(edge.length || 0).toFixed(2)} m`;
      this.edgeCurveHint.classList.toggle("hidden", !(Array.isArray(edge.control_points) && edge.control_points.length === 4));
      return;
    }

    this.selectionTitleText.textContent = "Nothing selected";
  }

  renderLogs() {
    this.editorLog.innerHTML = "";
    if (!this.logs.length) {
      this.editorLog.append(this.infoCard("No editor events yet."));
      return;
    }
    for (const item of this.logs) {
      const div = document.createElement("div");
      div.className = `log-item ${item.level}`;
      div.textContent = `${item.time} ${item.message}`;
      this.editorLog.append(div);
    }
  }

  edgePath(edge) {
    if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      const [p0, p1, p2, p3] = edge.control_points.map((point) => this.worldToSvg(point));
      return `M ${p0.x} ${p0.y} C ${p1.x} ${p1.y}, ${p2.x} ${p2.y}, ${p3.x} ${p3.y}`;
    }
    const [start, end] = this.edgeWorldPoints(edge).map((point) => this.worldToSvg(point));
    return `M ${start.x} ${start.y} L ${end.x} ${end.y}`;
  }

  edgeWorldPoints(edge) {
    const start = this.landmarkByName(edge.from);
    const end = this.landmarkByName(edge.to);
    if (!start || !end) {
      return [{ x: 0, y: 0 }, { x: 0, y: 0 }];
    }
    return [
      { x: start.x, y: start.y },
      { x: end.x, y: end.y },
    ];
  }

  edgeLength(edge) {
    if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      return this.bezierLength(edge.control_points);
    }
    const [start, end] = this.edgeWorldPoints(edge);
    return this.distance(start, end);
  }

  bezierLength(points) {
    let total = 0;
    let previous = this.bezierPoint(points, 0);
    const steps = 120;
    for (let index = 1; index <= steps; index += 1) {
      const current = this.bezierPoint(points, index / steps);
      total += this.distance(previous, current);
      previous = current;
    }
    return total;
  }

  bezierPoint(points, t) {
    const [p0, p1, p2, p3] = points;
    const u = 1 - t;
    return {
      x: (u ** 3 * p0.x) + (3 * u * u * t * p1.x) + (3 * u * t * t * p2.x) + (t ** 3 * p3.x),
      y: (u ** 3 * p0.y) + (3 * u * u * t * p1.y) + (3 * u * t * t * p2.y) + (t ** 3 * p3.y),
    };
  }

  bezierDerivative(points, t) {
    const [p0, p1, p2, p3] = points;
    const u = 1 - t;
    return {
      x: (3 * u * u * (p1.x - p0.x)) + (6 * u * t * (p2.x - p1.x)) + (3 * t * t * (p3.x - p2.x)),
      y: (3 * u * u * (p1.y - p0.y)) + (6 * u * t * (p2.y - p1.y)) + (3 * t * t * (p3.y - p2.y)),
    };
  }

  uniqueLandmarkName() {
    const existing = new Set((this.currentMap?.lms || []).map((item) => String(item.name || "")));
    let index = 1;
    while (existing.has(`LM${index}`)) {
      index += 1;
    }
    return `LM${index}`;
  }

  landmarkByName(name) {
    return (this.currentMap?.lms || []).find((item) => item.name === name) || null;
  }

  nearestLandmark(point, maxDistance = 0.28) {
    let best = null;
    let bestDistance = Number.POSITIVE_INFINITY;
    for (const lm of this.currentMap?.lms || []) {
      const distance = this.distance(point, lm);
      if (distance < bestDistance) {
        best = lm;
        bestDistance = distance;
      }
    }
    return best && bestDistance <= maxDistance ? best : null;
  }

  edgeKey(edge) {
    return `${edge.from}->${edge.to}`;
  }

  edgeByKey(key) {
    return (this.currentMap?.edges || []).find((edge) => this.edgeKey(edge) === key) || null;
  }

  edgeByNames(fromName, toName) {
    return (this.currentMap?.edges || []).find((edge) => edge.from === fromName && edge.to === toName) || null;
  }

  worldToSvg(point) {
    const mapMeta = this.currentMap?.map || {};
    const resolution = Number(mapMeta.resolution || 1);
    const padding = Number(mapMeta.viewPadding || 0);
    return {
      x: padding + (Number(point.x) / resolution),
      y: padding + (Number(point.y) / resolution),
    };
  }

  svgToWorld(point) {
    const mapMeta = this.currentMap?.map || {};
    const resolution = Number(mapMeta.resolution || 1);
    const padding = Number(mapMeta.viewPadding || 0);
    return {
      x: this.round((point.x - padding) * resolution),
      y: this.round((point.y - padding) * resolution),
    };
  }

  eventToSvgPoint(event) {
    const svgPoint = this.editorSvg.createSVGPoint();
    svgPoint.x = event.clientX;
    svgPoint.y = event.clientY;
    return svgPoint.matrixTransform(this.editorSvg.getScreenCTM().inverse());
  }

  distance(a, b) {
    return Math.hypot(Number(b.x) - Number(a.x), Number(b.y) - Number(a.y));
  }

  round(value) {
    return Math.round(Number(value || 0) * 1000) / 1000;
  }

  clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  markDirty(message, options = {}) {
    this.dirty = true;
    this.babylonRevision += 1;
    const lightweightDrag = Boolean(
      options.quietLog
      && this.babylonScene
      && ["landmark", "handle"].includes(this.dragState?.type)
    );
    if (lightweightDrag) {
      this.renderBabylonCanvas();
      this.renderInspector();
    } else {
      this.render();
    }
    if (!options.quietLog) {
      this.log("info", message);
    }
  }

  setStatus(message) {
    this.editorStatusText.textContent = message;
  }

  log(level, message) {
    const stamp = new Date();
    const hh = String(stamp.getHours()).padStart(2, "0");
    const mm = String(stamp.getMinutes()).padStart(2, "0");
    const ss = String(stamp.getSeconds()).padStart(2, "0");
    this.logs.unshift({
      level: ["warn", "error"].includes(level) ? level : "info",
      message,
      time: `${hh}:${mm}:${ss}`,
    });
    this.logs = this.logs.slice(0, 80);
    this.renderLogs();
  }

  markPendingPush() {
    window.sessionStorage.setItem("operator:pendingPushRobotId", this.robotId);
  }

  clearPendingPush() {
    const pendingRobotId = window.sessionStorage.getItem("operator:pendingPushRobotId") || "";
    if (pendingRobotId === this.robotId) {
      window.sessionStorage.removeItem("operator:pendingPushRobotId");
    }
  }

  closeWindow() {
    this.allowUnload = true;
    window.close();
    window.setTimeout(() => {
      if (!window.closed) {
        if (window.history.length > 1) {
          window.history.back();
          return;
        }
        window.location.assign("/robot");
      }
    }, 120);
  }

  handleError(error) {
    const message = error instanceof Error ? error.message : String(error);
    this.setStatus(message);
    this.log("error", message);
  }

  infoCard(message) {
    const div = document.createElement("div");
    div.className = "log-item";
    div.textContent = message;
    return div;
  }

  clone(value) {
    return cloneJson(value);
  }

  async getJson(url) {
    return httpClient.get(url);
  }

  async postJson(url, payload) {
    return httpClient.post(url, payload);
  }

  escapeHtml(value) {
    return escapeHtml(value);
  }
}

window.addEventListener("DOMContentLoaded", () => {
  const app = new RobotMapEditorApp();
  app.init().catch((error) => {
    window.alert(error instanceof Error ? error.message : String(error));
  });
});
