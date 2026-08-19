import { markControlledCorridorArea } from "../editor/graph-tools.js";
import { OccupancyGrid, OCCUPANCY_VALUES } from "../editor/occupancy-grid.js";
import { cloneJson } from "../shared/json.js";
import {
  FLEET_RASTER_TOOLS,
  normalizeEdgeMotionCode,
} from "./constants.js";


export const withMapEditor = (Base) => class OperatorAppMapEditor extends Base {
  ensureFleetMapDraft() {
    if (!this.fleetMapDraft && this.operatorMapPayload) {
      this.fleetMapDraft = this.cloneJson(this.operatorMapPayload);
      this.fleetMapDirty = false;
    }
    return this.fleetMapDraft;
  }

  async ensureFleetRasterGrid() {
    const draft = this.ensureFleetMapDraft();
    if (!draft?.map?.imageDataUrl) {
      this.fleetRasterGrid = null;
      this.fleetRasterDraftRef = draft;
      this.syncFleetRasterControls();
      return null;
    }
    if (this.fleetRasterGrid && this.fleetRasterDraftRef === draft) {
      return this.fleetRasterGrid;
    }
    if (this.fleetRasterLoadPromise) {
      return this.fleetRasterLoadPromise;
    }
    const targetDraft = draft;
    this.fleetRasterLoadPromise = OccupancyGrid.fromImageDataUrl(
      targetDraft.map.imageDataUrl,
      Number(targetDraft.map.width),
      Number(targetDraft.map.height),
    ).then((grid) => {
      if (this.fleetMapDraft !== targetDraft) {
        return null;
      }
      this.fleetRasterGrid = grid;
      this.fleetRasterDraftRef = targetDraft;
      this.scene3d?.setFloorCanvas(grid.canvas);
      this.syncFleetRasterControls();
      return grid;
    }).catch((error) => {
      this.robotMessageText.textContent = `Fleet raster tools unavailable: ${error.message || error}`;
      return null;
    }).finally(() => {
      this.fleetRasterLoadPromise = null;
    });
    return this.fleetRasterLoadPromise;
  }

  resetFleetRasterGrid() {
    if (this.fleetRasterPreviewTimer) {
      window.clearTimeout(this.fleetRasterPreviewTimer);
      this.fleetRasterPreviewTimer = 0;
    }
    this.fleetRasterGrid = null;
    this.fleetRasterDraftRef = null;
    this.fleetRasterDrag = null;
    this.fleetCorridorDrag = null;
    this.fleetEditorAreaPreview = null;
    this.fleetRasterHistory.clear();
    this.syncFleetRasterControls();
  }

  syncFleetRasterControls() {
    const size = Math.max(1, Math.floor(Number(this.fleetRasterBrushSizeInput?.value) || 1));
    if (this.fleetRasterBrushSizeOutput) {
      this.fleetRasterBrushSizeOutput.textContent = `${size} px`;
    }
    if (this.fleetRasterUndoButton) {
      this.fleetRasterUndoButton.disabled = !this.fleetRasterHistory.canUndo;
    }
    if (this.fleetRasterRedoButton) {
      this.fleetRasterRedoButton.disabled = !this.fleetRasterHistory.canRedo;
    }
    for (const button of this.fleetMapToolButtons) {
      if (FLEET_RASTER_TOOLS.has(button.dataset.fleetMapTool || "")) {
        button.disabled = !this.fleetRasterGrid;
      }
    }
  }

  fleetRasterPoint(world) {
    const resolution = Math.max(0.000001, Number(this.fleetMapDraft?.map?.resolution || 1));
    return this.fleetRasterGrid?.normalizePoint({
      x: Number(world?.x || 0) / resolution,
      y: Number(world?.y || 0) / resolution,
    }) || null;
  }

  fleetRasterValue(tool = this.fleetMapTool) {
    if (tool === "eraser") {
      return OCCUPANCY_VALUES.free;
    }
    if (tool === "unknown") {
      return OCCUPANCY_VALUES.unknown;
    }
    return OCCUPANCY_VALUES.occupied;
  }

  beginFleetRasterPointer(hit) {
    if (!this.fleetRasterGrid || !hit?.world) {
      return false;
    }
    const point = this.fleetRasterPoint(hit.world);
    const value = this.fleetRasterValue();
    if (this.fleetMapTool === "fill") {
      const patch = this.fleetRasterGrid.beginPatch("Fill");
      this.fleetRasterGrid.floodFill(patch, point, value);
      this.commitFleetRasterPatch(patch, "Fleet occupancy area filled.");
      return true;
    }
    if (this.fleetMapTool === "rectangle") {
      this.fleetRasterDrag = {
        type: "rectangle",
        pointerId: hit.pointerId,
        start: point,
        current: point,
        startWorld: { ...hit.world },
        currentWorld: { ...hit.world },
        value,
      };
      this.setFleetEditorAreaPreview("rectangle", hit.world, hit.world);
      return true;
    }
    const patch = this.fleetRasterGrid.beginPatch(
      this.fleetMapTool === "eraser"
        ? "Erase"
        : this.fleetMapTool === "unknown"
          ? "Unknown brush"
          : "Pencil",
    );
    const size = Math.max(1, Math.floor(Number(this.fleetRasterBrushSizeInput?.value) || 1));
    this.fleetRasterGrid.paintSquareLine(patch, point, point, size, value);
    this.fleetRasterDrag = {
      type: "stroke",
      pointerId: hit.pointerId,
      patch,
      last: point,
      size,
      value,
    };
    this.scheduleFleetRasterPreview();
    return true;
  }

  moveFleetRasterPointer(hit) {
    if (!this.fleetRasterDrag || !hit?.world) {
      return;
    }
    const point = this.fleetRasterPoint(hit.world);
    if (this.fleetRasterDrag.type === "rectangle") {
      this.fleetRasterDrag.current = point;
      this.fleetRasterDrag.currentWorld = { ...hit.world };
      this.setFleetEditorAreaPreview(
        "rectangle",
        this.fleetRasterDrag.startWorld,
        this.fleetRasterDrag.currentWorld,
      );
      return;
    }
    this.fleetRasterGrid.paintSquareLine(
      this.fleetRasterDrag.patch,
      this.fleetRasterDrag.last,
      point,
      this.fleetRasterDrag.size,
      this.fleetRasterDrag.value,
    );
    this.fleetRasterDrag.last = point;
    this.scheduleFleetRasterPreview();
  }

  endFleetRasterPointer(hit) {
    const drag = this.fleetRasterDrag;
    this.fleetRasterDrag = null;
    this.setFleetEditorAreaPreview("", null, null);
    if (!drag || !this.fleetRasterGrid) {
      return;
    }
    if (drag.type === "rectangle") {
      const patch = this.fleetRasterGrid.beginPatch("Rectangle");
      this.fleetRasterGrid.paintRectangle(
        patch,
        drag.start,
        hit?.world ? this.fleetRasterPoint(hit.world) : drag.current,
        drag.value,
      );
      this.commitFleetRasterPatch(patch, "Fleet occupied rectangle added.");
      return;
    }
    this.commitFleetRasterPatch(drag.patch);
  }

  commitFleetRasterPatch(patch, message = "") {
    const command = this.fleetRasterGrid?.commandForPatch(patch);
    if (!command) {
      return false;
    }
    this.fleetRasterHistory.push(command);
    this.afterFleetRasterMutation(
      message || `${command.label} changed ${command.pixelCount} fleet map cells.`,
    );
    return true;
  }

  undoFleetRaster() {
    if (this.fleetRasterDrag) {
      return;
    }
    if (this.fleetCorridorDrag) {
      return;
    }
    if (this.fleetRasterHistory.undo()) {
      this.afterFleetMapHistoryMutation("Fleet map edit undone.");
    }
  }

  redoFleetRaster() {
    if (this.fleetRasterDrag) {
      return;
    }
    if (this.fleetCorridorDrag) {
      return;
    }
    if (this.fleetRasterHistory.redo()) {
      this.afterFleetMapHistoryMutation("Fleet map edit restored.");
    }
  }

  fleetGraphSnapshot() {
    const draft = this.ensureFleetMapDraft();
    return {
      lms: cloneJson(Array.isArray(draft?.lms) ? draft.lms : []),
      edges: cloneJson(Array.isArray(draft?.edges) ? draft.edges : []),
      trafficZones: cloneJson(Array.isArray(draft?.trafficZones) ? draft.trafficZones : []),
      selectedLmName: String(this.fleetSelectedLmName || ""),
      selectedEdgeKey: String(this.fleetSelectedEdgeKey || ""),
    };
  }

  restoreFleetGraphSnapshot(snapshot) {
    const draft = this.ensureFleetMapDraft();
    if (!draft || !snapshot) {
      return;
    }
    draft.lms = cloneJson(snapshot.lms || []);
    draft.edges = cloneJson(snapshot.edges || []);
    draft.trafficZones = cloneJson(snapshot.trafficZones || []);
    this.fleetSelectedLmName = String(snapshot.selectedLmName || "");
    this.fleetSelectedEdgeKey = String(snapshot.selectedEdgeKey || "");
  }

  commitFleetGraphHistory(before, label = "Fleet graph edit") {
    if (!before) {
      return false;
    }
    const after = this.fleetGraphSnapshot();
    if (JSON.stringify(before) === JSON.stringify(after)) {
      this.fleetMapDirty = this.fleetRasterHistory.canUndo;
      this.syncFleetMapEditorState();
      return false;
    }
    this.fleetRasterHistory.push({
      label,
      undo: () => this.restoreFleetGraphSnapshot(before),
      redo: () => this.restoreFleetGraphSnapshot(after),
    });
    this.afterFleetMapHistoryMutation(label);
    return true;
  }

  afterFleetMapHistoryMutation(message = "") {
    this.fleetMapDirty = this.fleetRasterHistory.canUndo;
    this.markBabylonMapGeometryDirty();
    if (this.fleetRasterGrid) {
      this.scheduleFleetRasterPreview(true);
    }
    this.syncFleetEditorFields();
    this.syncFleetRasterControls();
    this.syncFleetMapEditorState();
    this.renderOperatorMap();
    if (message) {
      this.robotMessageText.textContent = message;
    }
  }

  afterFleetRasterMutation(message) {
    this.fleetMapDirty = true;
    this.scheduleFleetRasterPreview(true);
    this.syncFleetRasterControls();
    this.syncFleetMapEditorState();
    this.robotMessageText.textContent = message;
    if (this.babylonMapFailed && this.fleetMapDraft?.map) {
      this.fleetMapDraft.map.imageDataUrl = this.fleetRasterGrid.toDataUrl();
      this.renderOperatorMap();
    }
  }

  scheduleFleetRasterPreview(immediate = false) {
    if (!this.scene3d || this.babylonMapFailed) {
      return;
    }
    if (immediate) {
      if (this.fleetRasterPreviewTimer) {
        window.clearTimeout(this.fleetRasterPreviewTimer);
        this.fleetRasterPreviewTimer = 0;
      }
      this.scene3d.updateFloorCanvas();
      return;
    }
    if (this.fleetRasterPreviewTimer) {
      return;
    }
    this.fleetRasterPreviewTimer = window.setTimeout(() => {
      this.fleetRasterPreviewTimer = 0;
      this.scene3d?.updateFloorCanvas();
    }, 50);
  }

  syncFleetRasterPayload() {
    if (!this.fleetMapDraft?.map || !this.fleetRasterGrid) {
      return;
    }
    this.fleetMapDraft.map.imageDataUrl = this.fleetRasterGrid.toDataUrl();
    this.fleetMapDraft.map.raster = this.fleetRasterGrid.toPayload();
  }

  setFleetEditorAreaPreview(kind, start, current) {
    this.fleetEditorAreaPreview = kind && start && current
      ? {
          kind,
          start: { x: Number(start.x || 0), y: Number(start.y || 0) },
          current: { x: Number(current.x || 0), y: Number(current.y || 0) },
        }
      : null;
    if (this.scene3d && !this.babylonMapFailed) {
      this.refreshBabylonEditorState();
    } else {
      this.drawFleetEditorOverlay();
    }
  }

  beginFleetCorridorPointer(hit) {
    if (!hit?.world) {
      return false;
    }
    this.fleetCorridorDrag = {
      pointerId: hit.pointerId,
      start: { ...hit.world },
      current: { ...hit.world },
      before: this.fleetGraphSnapshot(),
    };
    this.setFleetEditorAreaPreview("corridor", hit.world, hit.world);
    return true;
  }

  moveFleetCorridorPointer(hit) {
    if (!this.fleetCorridorDrag || !hit?.world) {
      return;
    }
    this.fleetCorridorDrag.current = { ...hit.world };
    this.setFleetEditorAreaPreview(
      "corridor",
      this.fleetCorridorDrag.start,
      this.fleetCorridorDrag.current,
    );
  }

  endFleetCorridorPointer(hit) {
    const drag = this.fleetCorridorDrag;
    this.fleetCorridorDrag = null;
    this.setFleetEditorAreaPreview("", null, null);
    if (!drag) {
      return;
    }
    const result = markControlledCorridorArea(
      this.fleetMapDraft,
      drag.start,
      hit?.world || drag.current,
    );
    if (!result) {
      this.robotMessageText.textContent = "The corridor rectangle does not cross any graph edge.";
      return;
    }
    const regionCount = result.regions.length;
    this.commitFleetGraphHistory(
      drag.before,
      `Added ${regionCount} controlled corridor rectangle; Core will compile ${result.edgeCount} intersecting directed edges and external stop lines.`,
    );
  }

  reloadFleetMapDraft() {
    if (this.fleetMapDirty && !window.confirm("Discard unsaved fleet map changes?")) {
      return;
    }
    this.discardFleetMapDraft();
    this.robotMessageText.textContent = "Fleet map draft reloaded.";
  }

  discardFleetMapDraft() {
    this.fleetMapDraft = this.cloneJson(this.operatorMapPayload);
    this.resetFleetRasterGrid();
    this.ensureFleetRasterGrid();
    this.fleetMapDirty = false;
    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.markBabylonMapGeometryDirty();
    this.syncFleetEditorFields();
    this.renderOperatorMap();
  }

  cloneJson(value) {
    return cloneJson(value || {});
  }

  handleFleetEditorPointerDown(event) {
    if (!this.fleetMapEditorActive || event.button !== 0) {
      return;
    }
    const draft = this.ensureFleetMapDraft();
    if (!draft) {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    const world = this.eventToMapWorld(event);
    const bezierHandle = event.target.closest("[data-bezier-index]");
    const lmName = event.target.closest(".landmark")?.dataset?.lmName || "";
    const edgeKey = event.target.closest(".graph-edge")?.dataset?.edgeKey || "";

    if (FLEET_RASTER_TOOLS.has(this.fleetMapTool) && world) {
      if (this.beginFleetRasterPointer({
        pointerId: event.pointerId,
        button: event.button,
        world,
      })) {
        this.operatorMapSvg.setPointerCapture(event.pointerId);
      }
      return;
    }

    if (this.fleetMapTool === "corridor" && world) {
      if (this.beginFleetCorridorPointer({
        pointerId: event.pointerId,
        button: event.button,
        world,
      })) {
        this.operatorMapSvg.setPointerCapture(event.pointerId);
      }
      return;
    }

    if (bezierHandle && this.fleetMapTool === "select") {
      const handleEdgeKey = bezierHandle.dataset.edgeKey || this.fleetSelectedEdgeKey;
      this.selectFleetEditorEdge(handleEdgeKey);
      this.fleetEditorBezierDrag = {
        pointerId: event.pointerId,
        edgeKey: handleEdgeKey,
        index: Number(bezierHandle.dataset.bezierIndex || 1),
        before: this.fleetGraphSnapshot(),
      };
      this.operatorMapSvg.setPointerCapture(event.pointerId);
      return;
    }

    if (lmName) {
      this.selectFleetEditorLm(lmName);
      if (this.fleetMapTool === "edge") {
        this.fleetEditorEdgeDrag = {
          pointerId: event.pointerId,
          currentLm: lmName,
          lastCreated: "",
          before: this.fleetGraphSnapshot(),
        };
      } else if (this.fleetMapTool === "select") {
        this.fleetEditorLmDrag = {
          pointerId: event.pointerId,
          name: lmName,
          start: world,
          moved: false,
          before: this.fleetGraphSnapshot(),
        };
      }
      this.operatorMapSvg.setPointerCapture(event.pointerId);
      return;
    }

    if (edgeKey && this.fleetMapTool === "select") {
      this.selectFleetEditorEdge(edgeKey);
      this.operatorMapSvg.setPointerCapture(event.pointerId);
      return;
    }

    if (this.fleetMapTool === "lm" && world) {
      const before = this.fleetGraphSnapshot();
      const added = this.addFleetEditorLm(world);
      this.selectFleetEditorLm(added.name);
      this.commitFleetGraphHistory(before, `Added landmark ${added.name}.`);
      return;
    }

    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.fleetEditorGuideWorld = null;
    this.syncFleetEditorFields();
    this.renderOperatorMap();
    this.mapDrag = { pointerId: event.pointerId, last: this.screenToSvg(event.clientX, event.clientY) };
    this.operatorMapSvg.classList.add("dragging");
    this.operatorMapSvg.setPointerCapture(event.pointerId);
  }

  handleFleetEditorPointerMove(event) {
    if (!this.fleetMapEditorActive) {
      return;
    }
    const world = this.eventToMapWorld(event);
    if (this.fleetRasterDrag?.pointerId === event.pointerId && world) {
      event.preventDefault();
      this.moveFleetRasterPointer({ pointerId: event.pointerId, world });
      return;
    }
    if (this.fleetCorridorDrag?.pointerId === event.pointerId && world) {
      event.preventDefault();
      this.moveFleetCorridorPointer({ pointerId: event.pointerId, world });
      return;
    }
    if (this.fleetEditorBezierDrag && this.fleetEditorBezierDrag.pointerId === event.pointerId && world) {
      event.preventDefault();
      const snapped = this.snapMapPoint(world);
      this.moveFleetEditorBezierHandle(this.fleetEditorBezierDrag.edgeKey, this.fleetEditorBezierDrag.index, snapped);
      this.fleetEditorGuideWorld = snapped;
      this.fleetMapDirty = true;
      this.syncFleetEditorFields();
      this.renderOperatorMap();
      return;
    }
    if (this.fleetEditorLmDrag && this.fleetEditorLmDrag.pointerId === event.pointerId && world) {
      event.preventDefault();
      const snapped = this.snapMapPoint(world);
      this.moveFleetEditorLm(this.fleetEditorLmDrag.name, snapped);
      this.fleetEditorGuideWorld = snapped;
      this.fleetEditorLmDrag.moved = true;
      this.fleetMapDirty = true;
      this.syncFleetEditorFields();
      this.renderOperatorMap();
      return;
    }
    if (this.fleetEditorEdgeDrag && this.fleetEditorEdgeDrag.pointerId === event.pointerId && world) {
      event.preventDefault();
      this.fleetEditorGuideWorld = world;
      const nearest = this.nearestLandmark(world);
      if (
        nearest &&
        nearest.distance <= 0.35 &&
        nearest.landmark.name !== this.fleetEditorEdgeDrag.currentLm &&
        nearest.landmark.name !== this.fleetEditorEdgeDrag.lastCreated
      ) {
        const previous = this.fleetEditorEdgeDrag.currentLm;
        this.addFleetEditorEdge(previous, nearest.landmark.name);
        this.fleetEditorEdgeDrag.lastCreated = previous;
        this.fleetEditorEdgeDrag.currentLm = nearest.landmark.name;
        this.fleetMapDirty = true;
        this.renderOperatorMap();
      }
      this.drawFleetEditorPreview(this.fleetEditorEdgeDrag.currentLm, world);
      return;
    }
    if (this.mapDrag && this.mapDrag.pointerId === event.pointerId) {
      this.applyMapDragMove(event);
    }
  }

  handleFleetEditorPointerUp(event) {
    if (this.fleetRasterDrag?.pointerId === event.pointerId) {
      this.endFleetRasterPointer({
        pointerId: event.pointerId,
        world: this.eventToMapWorld(event),
      });
    }
    if (this.fleetCorridorDrag?.pointerId === event.pointerId) {
      this.endFleetCorridorPointer({
        pointerId: event.pointerId,
        world: this.eventToMapWorld(event),
      });
    }
    if (this.fleetEditorLmDrag && this.fleetEditorLmDrag.pointerId === event.pointerId) {
      const drag = this.fleetEditorLmDrag;
      this.fleetEditorLmDrag = null;
      this.fleetEditorGuideWorld = null;
      if (drag.moved) {
        this.commitFleetGraphHistory(drag.before, `Moved landmark ${drag.name}.`);
      } else {
        this.drawFleetEditorOverlay();
      }
    }
    if (this.fleetEditorEdgeDrag && this.fleetEditorEdgeDrag.pointerId === event.pointerId) {
      const drag = this.fleetEditorEdgeDrag;
      this.fleetEditorEdgeDrag = null;
      this.fleetEditorPreview = null;
      this.fleetEditorGuideWorld = null;
      if (!this.commitFleetGraphHistory(drag.before, "Added graph edge chain.")) {
        this.drawFleetEditorOverlay();
      }
    }
    if (this.fleetEditorBezierDrag && this.fleetEditorBezierDrag.pointerId === event.pointerId) {
      const drag = this.fleetEditorBezierDrag;
      this.fleetEditorBezierDrag = null;
      this.fleetEditorGuideWorld = null;
      if (!this.commitFleetGraphHistory(drag.before, `Updated curve ${drag.edgeKey}.`)) {
        this.drawFleetEditorOverlay();
      }
    }
    if (this.mapDrag && this.mapDrag.pointerId === event.pointerId) {
      if (this.operatorMapSvg.hasPointerCapture(event.pointerId)) {
        this.operatorMapSvg.releasePointerCapture(event.pointerId);
      }
      this.mapDrag = null;
      this.operatorMapSvg.classList.remove("dragging");
    }
    if (this.operatorMapSvg.hasPointerCapture(event.pointerId)) {
      this.operatorMapSvg.releasePointerCapture(event.pointerId);
    }
  }

  eventToMapWorld(event) {
    const point = this.screenToSvg(event.clientX, event.clientY);
    if (!point) {
      return null;
    }
    return this.pixelToWorld({
      x: (point.x - this.mapView.tx) / this.mapView.scale,
      y: (point.y - this.mapView.ty) / this.mapView.scale,
    });
  }

  snapMapPoint(point) {
    const snapped = { x: Number(point.x || 0), y: Number(point.y || 0) };
    const tolerance = 0.035;
    for (const lm of this.fleetMapDraft?.lms || []) {
      if (lm.name === this.fleetSelectedLmName) {
        continue;
      }
      if (Math.abs(snapped.x - Number(lm.x || 0)) <= tolerance) {
        snapped.x = Number(lm.x || 0);
      }
      if (Math.abs(snapped.y - Number(lm.y || 0)) <= tolerance) {
        snapped.y = Number(lm.y || 0);
      }
    }
    return {
      x: Math.round(snapped.x * 1000) / 1000,
      y: Math.round(snapped.y * 1000) / 1000,
    };
  }

  addFleetEditorLm(world) {
    const draft = this.ensureFleetMapDraft();
    const lm = {
      name: this.nextFleetLmName(),
      x: Math.round(Number(world.x || 0) * 1000) / 1000,
      y: Math.round(Number(world.y || 0) * 1000) / 1000,
      ignoreDir: null,
      properties: {},
    };
    draft.lms.push(lm);
    this.fleetMapDirty = true;
    this.markBabylonMapGeometryDirty();
    return lm;
  }

  nextFleetLmName() {
    const names = new Set((this.fleetMapDraft?.lms || []).map((lm) => lm.name));
    let index = 1;
    while (names.has(`LM_NEW_${index}`)) {
      index += 1;
    }
    return `LM_NEW_${index}`;
  }

  moveFleetEditorLm(name, point) {
    const lm = (this.fleetMapDraft?.lms || []).find((item) => item.name === name);
    if (!lm) {
      return;
    }
    const dx = point.x - Number(lm.x || 0);
    const dy = point.y - Number(lm.y || 0);
    lm.x = point.x;
    lm.y = point.y;
    this.markBabylonMapGeometryDirty();
    for (const edge of this.fleetMapDraft.edges || []) {
      if (!Array.isArray(edge.control_points) || edge.control_points.length !== 4) {
        continue;
      }
      if (edge.from === name) {
        edge.control_points[0] = { x: point.x, y: point.y };
        edge.control_points[1] = { x: Number(edge.control_points[1].x || 0) + dx, y: Number(edge.control_points[1].y || 0) + dy };
      }
      if (edge.to === name) {
        edge.control_points[3] = { x: point.x, y: point.y };
        edge.control_points[2] = { x: Number(edge.control_points[2].x || 0) + dx, y: Number(edge.control_points[2].y || 0) + dy };
      }
      edge.length = this.edgeLength(edge);
    }
  }

  addFleetEditorEdge(from, to) {
    const draft = this.ensureFleetMapDraft();
    if (!from || !to || from === to || this.edgeFromKey(this.edgeKey(from, to))) {
      return;
    }
    const start = this.lmByName(from);
    const goal = this.lmByName(to);
    if (!start || !goal) {
      return;
    }
    const c1 = { x: (Number(start.x) * 2 + Number(goal.x)) / 3, y: (Number(start.y) * 2 + Number(goal.y)) / 3 };
    const c2 = { x: (Number(start.x) + Number(goal.x) * 2) / 3, y: (Number(start.y) + Number(goal.y) * 2) / 3 };
    const edge = {
      from,
      to,
      kind: "curve",
      type: "DegenerateBezier",
      geometry: "bezier",
      control_points: [
        { x: Number(start.x), y: Number(start.y) },
        c1,
        c2,
        { x: Number(goal.x), y: Number(goal.y) },
      ],
      properties: { direction: -1, movestyle: 0 },
      length: 0,
    };
    edge.length = this.edgeLength(edge);
    draft.edges.push(edge);
    this.markBabylonMapGeometryDirty();
    this.selectFleetEditorEdge(this.edgeKey(from, to));
  }

  reversedFleetEditorEdge(edge) {
    const reversed = this.cloneJson(edge);
    reversed.from = edge.to;
    reversed.to = edge.from;
    if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      reversed.control_points = [...edge.control_points]
        .reverse()
        .map((point) => ({ x: Number(point.x), y: Number(point.y) }));
    }
    if (Array.isArray(edge.world_points)) {
      reversed.world_points = [...edge.world_points]
        .reverse()
        .map((point) => ({ x: Number(point.x), y: Number(point.y) }));
    }
    reversed.length = Number(edge.length || this.edgeLength(edge));
    return reversed;
  }

  deleteFleetEditorLm(name) {
    const before = this.fleetGraphSnapshot();
    const draft = this.ensureFleetMapDraft();
    draft.lms = (draft.lms || []).filter((lm) => lm.name !== name);
    draft.edges = (draft.edges || []).filter((edge) => edge.from !== name && edge.to !== name);
    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.commitFleetGraphHistory(before, `Deleted landmark ${name}.`);
  }

  deleteFleetEditorEdge(edgeKey) {
    const before = this.fleetGraphSnapshot();
    const draft = this.ensureFleetMapDraft();
    const [from, to] = edgeKey.split("->");
    draft.edges = (draft.edges || []).filter((edge) => !(edge.from === from && edge.to === to));
    this.fleetSelectedEdgeKey = "";
    this.commitFleetGraphHistory(before, `Deleted edge ${edgeKey}.`);
  }

  lmByName(name) {
    return (this.fleetMapDraft?.lms || this.activeOperatorMapPayload()?.lms || []).find((lm) => lm.name === name);
  }

  edgeKey(from, to) {
    return `${from}->${to}`;
  }

  edgeFromKey(edgeKey) {
    const [from, to] = String(edgeKey || "").split("->");
    return (this.fleetMapDraft?.edges || this.activeOperatorMapPayload()?.edges || []).find((edge) => edge.from === from && edge.to === to) || null;
  }

  selectFleetEditorLm(name) {
    this.fleetSelectedLmName = name;
    this.fleetSelectedEdgeKey = "";
    this.syncFleetEditorFields();
    if (this.scene3d && !this.babylonMapFailed) {
      this.refreshBabylonEditorState();
    } else {
      this.renderOperatorMap();
    }
  }

  selectFleetEditorEdge(edgeKey) {
    this.fleetSelectedEdgeKey = edgeKey;
    this.fleetSelectedLmName = "";
    this.syncFleetEditorFields();
    if (this.scene3d && !this.babylonMapFailed) {
      this.refreshBabylonEditorState();
    } else {
      this.renderOperatorMap();
    }
  }

  syncFleetEditorFields() {
    this.fleetEditorFieldSyncing = true;
    const lm = this.fleetSelectedLmName ? this.lmByName(this.fleetSelectedLmName) : null;
    this.fleetEditorLmNameInput.value = lm ? lm.name : "";
    this.fleetEditorLmXInput.value = lm ? Number(lm.x || 0).toFixed(3) : "";
    this.fleetEditorLmYInput.value = lm ? Number(lm.y || 0).toFixed(3) : "";
    const edge = this.fleetSelectedEdgeKey ? this.edgeFromKey(this.fleetSelectedEdgeKey) : null;
    this.fleetEditorEdgeFromInput.value = edge ? edge.from : "";
    this.fleetEditorEdgeToInput.value = edge ? edge.to : "";
    if (edge) {
      this.fleetEditorEdgeTrafficSelect.value = this.edgeFromKey(this.edgeKey(edge.to, edge.from)) ? "bidirectional" : "one_way";
      this.fleetEditorEdgeMotionSelect.value = String(
        normalizeEdgeMotionCode((edge.properties || {}).direction),
      );
    }
    this.fleetEditorFieldSyncing = false;
    this.syncFleetMapEditorState();
  }

  syncFleetMapEditorState() {
    if (this.fleetMapDirtyState) {
      this.fleetMapDirtyState.textContent = this.fleetMapDirty ? "Unsaved" : "Saved";
      this.fleetMapDirtyState.classList.toggle("dirty", this.fleetMapDirty);
    }
    if (this.fleetMapSaveButton) {
      this.fleetMapSaveButton.disabled = !this.fleetMapDraft || !this.fleetMapDirty;
    }
    if (this.fleetMapSaveAsButton) {
      this.fleetMapSaveAsButton.disabled = !this.fleetMapDraft;
    }
    if (this.fleetMapReloadButton) {
      this.fleetMapReloadButton.disabled = !this.fleetMapDraft || !this.fleetMapDirty;
    }
  }

  applyFleetEditorLmFields() {
    if (this.fleetEditorFieldSyncing || !this.fleetSelectedLmName) {
      return;
    }
    const draft = this.ensureFleetMapDraft();
    const lm = this.lmByName(this.fleetSelectedLmName);
    if (!lm) {
      return;
    }
    const nextName = this.fleetEditorLmNameInput.value.trim();
    if (!nextName) {
      return;
    }
    if (nextName !== lm.name && (draft.lms || []).some((item) => item.name === nextName)) {
      window.alert(`LM already exists: ${nextName}`);
      return;
    }
    const before = this.fleetGraphSnapshot();
    const oldName = lm.name;
    const nextPoint = this.snapMapPoint({
      x: Number(this.fleetEditorLmXInput.value || lm.x),
      y: Number(this.fleetEditorLmYInput.value || lm.y),
    });
    this.moveFleetEditorLm(oldName, nextPoint);
    lm.name = nextName;
    for (const edge of draft.edges || []) {
      if (edge.from === oldName) {
        edge.from = nextName;
      }
      if (edge.to === oldName) {
        edge.to = nextName;
      }
    }
    this.fleetSelectedLmName = nextName;
    this.commitFleetGraphHistory(before, `Updated landmark ${nextName}.`);
  }

  applyFleetEditorEdgeFields() {
    if (this.fleetEditorFieldSyncing || !this.fleetSelectedEdgeKey) {
      return;
    }
    const edge = this.edgeFromKey(this.fleetSelectedEdgeKey);
    if (!edge) {
      return;
    }
    const before = this.fleetGraphSnapshot();
    edge.properties = {
      ...(edge.properties || {}),
      direction: normalizeEdgeMotionCode(this.fleetEditorEdgeMotionSelect.value),
      movestyle: Number((edge.properties || {}).movestyle || 0),
    };
    const reverseKey = this.edgeKey(edge.to, edge.from);
    let reverse = this.edgeFromKey(reverseKey);
    const traffic = this.fleetEditorEdgeTrafficSelect.value;
    if ((traffic === "bidirectional" || traffic === "reverse") && !reverse) {
      reverse = this.reversedFleetEditorEdge(edge);
      this.ensureFleetMapDraft().edges.push(reverse);
    }
    if (reverse && (traffic === "bidirectional" || traffic === "reverse")) {
      reverse.properties = { ...(edge.properties || {}) };
    }
    if (traffic === "bidirectional") {
      this.fleetSelectedEdgeKey = this.edgeKey(edge.from, edge.to);
    }
    if (traffic === "one_way" && reverse) {
      this.ensureFleetMapDraft().edges = this.ensureFleetMapDraft().edges.filter(
        (item) => item !== reverse,
      );
      this.fleetSelectedEdgeKey = this.edgeKey(edge.from, edge.to);
    }
    if (traffic === "reverse" && reverse) {
      this.ensureFleetMapDraft().edges = this.ensureFleetMapDraft().edges.filter(
        (item) => item !== edge,
      );
      this.fleetSelectedEdgeKey = reverseKey;
    }
    this.commitFleetGraphHistory(before, `Updated edge ${this.fleetSelectedEdgeKey}.`);
  }

  drawFleetEditorPreview(fromName = "", world = null) {
    this.fleetEditorPreview = fromName && world ? { fromName, world: { x: Number(world.x || 0), y: Number(world.y || 0) } } : null;
    this.drawFleetEditorOverlay();
  }

  drawFleetEditorOverlay() {
    if (!this.operatorEditorLayer) {
      return;
    }
    this.operatorEditorLayer.innerHTML = "";
    if (!this.fleetMapEditorActive) {
      return;
    }
    if (this.fleetEditorGuideWorld) {
      this.drawFleetEditorGuide(this.fleetEditorGuideWorld);
    }
    const areaPreview = this.fleetEditorAreaPreview;
    if (areaPreview?.start && areaPreview?.current) {
      const start = this.worldToPixel(areaPreview.start);
      const current = this.worldToPixel(areaPreview.current);
      const rectangle = document.createElementNS("http://www.w3.org/2000/svg", "rect");
      rectangle.setAttribute(
        "class",
        `editor-area-preview ${areaPreview.kind === "corridor" ? "corridor" : "raster"}`,
      );
      rectangle.setAttribute("x", String(Math.min(start.x, current.x)));
      rectangle.setAttribute("y", String(Math.min(start.y, current.y)));
      rectangle.setAttribute("width", String(Math.abs(current.x - start.x)));
      rectangle.setAttribute("height", String(Math.abs(current.y - start.y)));
      this.operatorEditorLayer.append(rectangle);
    }
    const preview = this.fleetEditorPreview || {};
    const fromName = preview.fromName || "";
    const world = preview.world || null;
    if (fromName && world) {
      const from = this.lmByName(fromName);
      if (from) {
        const a = this.worldToPixel(from);
        const b = this.worldToPixel(world);
        const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
        line.setAttribute("class", "editor-preview");
        line.setAttribute("x1", String(a.x));
        line.setAttribute("y1", String(a.y));
        line.setAttribute("x2", String(b.x));
        line.setAttribute("y2", String(b.y));
        this.operatorEditorLayer.append(line);
      }
    }
    const edge = this.fleetSelectedEdgeKey ? this.edgeFromKey(this.fleetSelectedEdgeKey) : null;
    if (!edge || !Array.isArray(edge.control_points) || edge.control_points.length !== 4) {
      return;
    }
    const points = edge.control_points.map((point) => this.worldToPixel(point));
    const handleGroup = document.createElementNS("http://www.w3.org/2000/svg", "g");
    handleGroup.setAttribute("class", "editor-bezier");
    const firstHandle = document.createElementNS("http://www.w3.org/2000/svg", "line");
    firstHandle.setAttribute("class", "editor-bezier-arm");
    firstHandle.setAttribute("x1", String(points[0].x));
    firstHandle.setAttribute("y1", String(points[0].y));
    firstHandle.setAttribute("x2", String(points[1].x));
    firstHandle.setAttribute("y2", String(points[1].y));
    handleGroup.append(firstHandle);
    const secondHandle = document.createElementNS("http://www.w3.org/2000/svg", "line");
    secondHandle.setAttribute("class", "editor-bezier-arm");
    secondHandle.setAttribute("x1", String(points[3].x));
    secondHandle.setAttribute("y1", String(points[3].y));
    secondHandle.setAttribute("x2", String(points[2].x));
    secondHandle.setAttribute("y2", String(points[2].y));
    handleGroup.append(secondHandle);
    [1, 2].forEach((index) => {
      const handle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      handle.setAttribute("class", "editor-bezier-handle");
      handle.setAttribute("cx", String(points[index].x));
      handle.setAttribute("cy", String(points[index].y));
      handle.setAttribute("r", "5");
      handle.dataset.edgeKey = this.fleetSelectedEdgeKey;
      handle.dataset.bezierIndex = String(index);
      handleGroup.append(handle);
    });
    [0, 3].forEach((index) => {
      const endpoint = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      endpoint.setAttribute("class", "editor-bezier-endpoint");
      endpoint.setAttribute("cx", String(points[index].x));
      endpoint.setAttribute("cy", String(points[index].y));
      endpoint.setAttribute("r", "3");
      handleGroup.append(endpoint);
    });
    this.operatorEditorLayer.append(handleGroup);
  }

  drawFleetEditorGuide(world) {
    const payload = this.activeOperatorMapPayload();
    if (!payload || !payload.map) {
      return;
    }
    const px = this.worldToPixel(world);
    const map = payload.map;
    const width = Number(map.viewWidth || 100);
    const height = Number(map.viewHeight || 100);
    const vertical = document.createElementNS("http://www.w3.org/2000/svg", "line");
    vertical.setAttribute("class", "editor-guide-line");
    vertical.setAttribute("x1", String(px.x));
    vertical.setAttribute("y1", "0");
    vertical.setAttribute("x2", String(px.x));
    vertical.setAttribute("y2", String(height));
    this.operatorEditorLayer.append(vertical);
    const horizontal = document.createElementNS("http://www.w3.org/2000/svg", "line");
    horizontal.setAttribute("class", "editor-guide-line");
    horizontal.setAttribute("x1", "0");
    horizontal.setAttribute("y1", String(px.y));
    horizontal.setAttribute("x2", String(width));
    horizontal.setAttribute("y2", String(px.y));
    this.operatorEditorLayer.append(horizontal);
    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("class", "editor-guide-label");
    label.setAttribute("x", String(px.x + 8));
    label.setAttribute("y", String(px.y - 8));
    label.textContent = `x ${Number(world.x || 0).toFixed(3)} / y ${Number(world.y || 0).toFixed(3)}`;
    this.operatorEditorLayer.append(label);
  }

  moveFleetEditorBezierHandle(edgeKey, index, point) {
    const edge = this.edgeFromKey(edgeKey);
    if (!edge || !Array.isArray(edge.control_points) || edge.control_points.length !== 4 || ![1, 2].includes(index)) {
      return;
    }
    edge.control_points[index] = {
      x: Math.round(Number(point.x || 0) * 1000) / 1000,
      y: Math.round(Number(point.y || 0) * 1000) / 1000,
    };
    edge.length = this.edgeLength(edge);
    this.markBabylonMapGeometryDirty();
  }

  edgeLength(edge) {
    if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      let total = 0;
      let previous = this.bezierPoint(edge.control_points, 0);
      for (let i = 1; i <= 60; i += 1) {
        const current = this.bezierPoint(edge.control_points, i / 60);
        total += Math.hypot(current.x - previous.x, current.y - previous.y);
        previous = current;
      }
      return Math.round(total * 1000000) / 1000000;
    }
    const start = this.lmByName(edge.from);
    const goal = this.lmByName(edge.to);
    if (!start || !goal) {
      return Number(edge.length || 0);
    }
    return Math.round(Math.hypot(Number(goal.x) - Number(start.x), Number(goal.y) - Number(start.y)) * 1000000) / 1000000;
  }

  async saveFleetMap(saveAs, options = {}) {
    const draft = this.ensureFleetMapDraft();
    if (!draft) {
      return null;
    }
    let mapName = this.robotMapState.operatorActiveMapName || draft.mapName || "";
    if (saveAs) {
      mapName = await this.promptFleetMapSaveAs(`${draft.mapName || "fleet_map"}_copy`);
      if (!mapName) {
        return null;
      }
    }
    try {
      this.syncFleetRasterPayload();
      const mapPayload = this.cloneJson(draft);
      if (saveAs) {
        mapPayload.mapName = mapName.replace(/\.smap$/i, "");
      }
      const payload = await this.postJson(this.fleetApiPath("/maps/local/save"), {
        mapName,
        map: mapPayload,
        sourceMapName: this.robotMapState.sourceRobotMapName || draft.mapName || mapName,
        activate: true,
      });
      await this.refreshRobotMapState({ quiet: true });
      this.fleetMapDraft = this.cloneJson(this.operatorMapPayload);
      this.resetFleetRasterGrid();
      await this.ensureFleetRasterGrid();
      this.fleetMapDirty = false;
      this.syncFleetMapEditorState();
      this.robotMessageText.textContent = `Local fleet map saved: ${this.robotMapState.operatorActiveMapName || mapName}. Push uploads it; Load activates it.`;
      this.renderSelectedRobot();
      if (options.offerPush) {
        await this.offerMapSyncDecisionAfterLocalSave({
          message: "Local Fleet Manager map was saved and differs from the active map.",
        });
      }
      return payload;
    } catch (error) {
      this.robotMessageText.textContent = `Save local fleet map failed: ${error.message || error}`;
      return null;
    }
  }

  async confirmFleetMapExit() {
    if (!this.fleetMapEditorActive || !this.fleetMapDirty) {
      return true;
    }
    const decision = await this.promptFleetMapExit();
    if (decision === "discard") {
      this.discardFleetMapDraft();
      return true;
    }
    if (decision === "save") {
      return Boolean(await this.saveFleetMap(false, { skipConfirm: true }));
    }
    if (decision === "save-push") {
      const saved = await this.saveFleetMap(false, { skipConfirm: true });
      if (!saved) {
        return false;
      }
      return Boolean(await this.handlePushMap({ skipConfirm: true }));
    }
    return false;
  }

  promptFleetMapExit() {
    if (!this.fleetMapExitDialog || typeof this.fleetMapExitDialog.showModal !== "function") {
      const shouldSave = window.confirm("Save fleet map changes before closing?");
      return Promise.resolve(shouldSave ? "save" : "discard");
    }
    return new Promise((resolve) => {
      this.fleetMapExitResolve = resolve;
      this.fleetMapExitDialog.showModal();
    });
  }

  resolveFleetMapExit(decision) {
    if (!this.fleetMapExitResolve) {
      return;
    }
    const resolve = this.fleetMapExitResolve;
    this.fleetMapExitResolve = null;
    if (this.fleetMapExitDialog?.open) {
      this.fleetMapExitDialog.close();
    }
    resolve(decision);
  }

  promptFleetMapSaveAs(suggestedName) {
    if (!this.fleetMapSaveAsDialog || typeof this.fleetMapSaveAsDialog.showModal !== "function") {
      return Promise.resolve((window.prompt("Save local fleet map as", suggestedName) || "").trim());
    }
    this.fleetMapSaveAsNameInput.value = suggestedName;
    return new Promise((resolve) => {
      this.fleetMapSaveAsResolve = resolve;
      this.fleetMapSaveAsDialog.showModal();
      window.requestAnimationFrame(() => {
        this.fleetMapSaveAsNameInput.focus();
        this.fleetMapSaveAsNameInput.select();
      });
    });
  }

  resolveFleetMapSaveAs(mapName) {
    if (!this.fleetMapSaveAsResolve) {
      return;
    }
    const resolve = this.fleetMapSaveAsResolve;
    this.fleetMapSaveAsResolve = null;
    if (this.fleetMapSaveAsDialog?.open) {
      this.fleetMapSaveAsDialog.close();
    }
    resolve(String(mapName || "").trim());
  }
};
