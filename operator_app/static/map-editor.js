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
    this.selectedTool = "select";
    this.selection = { type: "none", key: "" };
    this.previewWorld = null;
    this.previewSnapName = "";
    this.dragState = null;
    this.dirty = false;
    this.logs = [];
    this.view = { x: 0, y: 0, width: 100, height: 100 };

    this.editorRobotTitle = document.getElementById("editorRobotTitle");
    this.editorStatusText = document.getElementById("editorStatusText");
    this.activeRobotMapText = document.getElementById("activeRobotMapText");
    this.localDraftCountText = document.getElementById("localDraftCountText");
    this.currentDraftText = document.getElementById("currentDraftText");
    this.draftStateChip = document.getElementById("draftStateChip");
    this.draftMetaText = document.getElementById("draftMetaText");
    this.robotMapsList = document.getElementById("robotMapsList");
    this.localMapsList = document.getElementById("localMapsList");
    this.editorLog = document.getElementById("editorLog");

    this.refreshMapsButton = document.getElementById("refreshMapsButton");
    this.saveLocalButton = document.getElementById("saveLocalButton");
    this.saveAsButton = document.getElementById("saveAsButton");
    this.closeEditorButton = document.getElementById("closeEditorButton");

    this.toolButtons = Array.from(document.querySelectorAll("[data-tool]"));
    this.zoomInButton = document.getElementById("zoomInButton");
    this.zoomOutButton = document.getElementById("zoomOutButton");
    this.resetViewButton = document.getElementById("resetViewButton");
    this.deleteSelectionButton = document.getElementById("deleteSelectionButton");

    this.editorSvg = document.getElementById("editorSvg");
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
    this.edgeFromText = document.getElementById("edgeFromText");
    this.edgeToText = document.getElementById("edgeToText");
    this.edgeKindSelect = document.getElementById("edgeKindSelect");
    this.edgeTypeInput = document.getElementById("edgeTypeInput");
    this.edgeDirectionSelect = document.getElementById("edgeDirectionSelect");
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
    await this.refreshAll({ autoOpenLocal: true });
  }

  bindEvents() {
    this.refreshMapsButton.addEventListener("click", () => this.refreshAll());
    this.saveLocalButton.addEventListener("click", () => this.saveLocalDraft());
    this.saveAsButton.addEventListener("click", () => this.handleSaveAsAndClose());
    this.closeEditorButton.addEventListener("click", () => this.handleCloseEditor());
    this.zoomInButton.addEventListener("click", () => this.zoomView(0.88));
    this.zoomOutButton.addEventListener("click", () => this.zoomView(1.14));
    this.resetViewButton.addEventListener("click", () => this.resetView());
    this.deleteSelectionButton.addEventListener("click", () => this.deleteSelection());
    this.toolButtons.forEach((button) => {
      button.addEventListener("click", () => this.setTool(button.dataset.tool || "select"));
    });

    this.landmarkNameInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.landmarkXInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.landmarkYInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.landmarkIgnoreDirInput.addEventListener("change", () => this.applyLandmarkInspector());
    this.edgeKindSelect.addEventListener("change", () => this.applyEdgeInspector());
    this.edgeTypeInput.addEventListener("change", () => this.applyEdgeInspector());
    this.edgeDirectionSelect.addEventListener("change", () => this.applyEdgeInspector());

    this.editorSvg.addEventListener("pointerdown", (event) => this.onPointerDown(event));
    this.editorSvg.addEventListener("pointermove", (event) => this.onPointerMove(event));
    this.editorSvg.addEventListener("pointerup", () => this.onPointerUp());
    this.editorSvg.addEventListener("pointerleave", () => this.onPointerUp());
    this.editorSvg.addEventListener("wheel", (event) => this.onWheel(event), { passive: false });
    window.addEventListener("beforeunload", (event) => {
      if (!this.dirty) {
        return;
      }
      event.preventDefault();
      event.returnValue = "";
    });
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
        this.setStatus("No local map draft is open. Use Pull Map in Control first, then open the editor.");
        return;
      }
      if (!options.silent) {
        this.setStatus(`Local drafts refreshed. Current draft: ${this.currentMap.mapName || "-"}.`);
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
    if (this.dirty && !window.confirm("Current draft has unsaved changes. Open another local draft?")) {
      return;
    }
    try {
      if (options.activate !== false) {
        await this.postJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/local/activate`, { mapName });
      }
      const payload = await this.getJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/local/${encodeURIComponent(mapName)}`);
      if (!payload.map || typeof payload.map !== "object") {
        throw new Error("Local draft payload is invalid.");
      }
      this.currentLocalMapName = String(payload.mapName || mapName || "");
      this.currentSourceMapName = String(payload.robotMapName || payload.sourceMapName || payload.map.mapName || "");
      this.selectedLocalMapName = this.currentLocalMapName;
      this.loadEditableMap(payload.map);
      this.dirty = false;
      if (!options.silent) {
        this.setStatus(`Loaded local draft ${this.currentLocalMapName}.`);
        this.log("info", `Opened local draft ${this.currentLocalMapName}.`);
      }
      this.render();
    } catch (error) {
      this.handleError(error);
    }
  }

  async saveLocalDraft() {
    if (!this.currentMap) {
      this.handleError(new Error("No map draft is loaded."));
      return;
    }
    const defaultName = this.currentLocalMapName || this.currentMap.mapName || this.currentSourceMapName || "draft_map";
    const mapName = String(defaultName || "").trim();
    try {
      const payload = await this.postJson(
        `/api/robots/${encodeURIComponent(this.robotId)}/maps/local/save`,
        {
          mapName,
          sourceMapName: this.currentSourceMapName || this.currentMap.mapName || mapName,
          map: this.currentMap,
        },
      );
      const local = payload.local || {};
      this.currentLocalMapName = String(local.mapName || mapName);
      this.selectedLocalMapName = this.currentLocalMapName;
      this.dirty = false;
      this.markPendingPush();
      await this.refreshLocalMaps({ silent: true });
      this.setStatus(`Saved local draft ${this.currentLocalMapName}.`);
      this.log("info", `Saved local draft ${this.currentLocalMapName}.`);
      this.render();
    } catch (error) {
      this.handleError(error);
    }
  }

  async handleSaveAsAndClose() {
    if (!this.currentMap) {
      this.handleError(new Error("No map draft is loaded."));
      return;
    }
    const outputName = window.prompt("Save edited map as", this.currentMap.mapName || this.currentLocalMapName || "") || "";
    if (!outputName.trim()) {
      return;
    }
    try {
      await this.saveLocalDraftAs(outputName.trim(), { activate: false });
      this.closeWindow();
    } catch (error) {
      this.handleError(error);
    }
  }

  async handleCloseEditor() {
    if (!this.dirty) {
      this.closeWindow();
      return;
    }
    const shouldSave = window.confirm("Save changes to the current map and push them to the robot before closing?");
    if (!shouldSave) {
      this.closeWindow();
      return;
    }
    try {
      await this.saveAndPushCurrentMapThenClose();
    } catch (error) {
      this.handleError(error);
    }
  }

  async saveLocalDraftAs(mapName, options = {}) {
    if (!this.currentMap) {
      this.handleError(new Error("No map draft is loaded."));
      return;
    }
    const safeName = String(mapName || "").trim();
    if (!safeName) {
      throw new Error("Map name is required.");
    }
    const mapPayload = this.clone(this.currentMap);
    mapPayload.mapName = safeName;
    const payload = await this.postJson(
      `/api/robots/${encodeURIComponent(this.robotId)}/maps/local/save`,
      {
        mapName: safeName,
        sourceMapName: this.currentSourceMapName || this.currentMap.mapName || safeName,
        activate: Boolean(options.activate),
        map: mapPayload,
      },
    );
    const local = payload.local || {};
    if (options.activate !== false) {
      this.currentLocalMapName = String(local.mapName || safeName);
      this.selectedLocalMapName = this.currentLocalMapName;
      this.loadEditableMap(mapPayload);
      this.dirty = false;
    }
    await this.refreshLocalMaps({ silent: true });
    this.setStatus(`Saved local draft ${safeName}.`);
    this.log("info", `Saved local draft ${safeName}.`);
    this.render();
  }

  async saveAndPushCurrentMapThenClose() {
    if (!this.currentMap) {
      throw new Error("No map draft is loaded.");
    }
    const localName = String(this.currentLocalMapName || this.currentMap.mapName || this.currentSourceMapName || "").trim();
    if (!localName) {
      throw new Error("Current local draft name is empty.");
    }
    await this.saveLocalDraftAs(localName, { activate: true });
    await this.postJson(`/api/robots/${encodeURIComponent(this.robotId)}/maps/push-sync`, {});
    this.clearPendingPush();
    this.setStatus(`Saved local draft and pushed ${localName} to robot.`);
    this.log("info", `Pushed ${localName} to robot.`);
    this.closeWindow();
  }

  loadEditableMap(payload) {
    this.currentMap = this.clone(payload);
    this.currentMap.mapName = String(this.currentMap.mapName || this.currentSourceMapName || "map").trim() || "map";
    this.currentMap.lms = Array.isArray(this.currentMap.lms) ? this.currentMap.lms : [];
    this.currentMap.edges = Array.isArray(this.currentMap.edges) ? this.currentMap.edges : [];
    this.currentMap.map = this.currentMap.map || {};
    this.selection = { type: "none", key: "" };
    this.previewWorld = null;
    this.previewSnapName = "";
    this.recomputeAllEdgeLengths();
    this.resetView();
  }

  setTool(tool) {
    this.selectedTool = ["select", "lm", "edge"].includes(tool) ? tool : "select";
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
    this.render();
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
  }

  zoomView(scale, anchor = null) {
    if (!this.currentMap) {
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
    if (!this.currentMap) {
      return;
    }
    const svgPoint = this.eventToSvgPoint(event);
    const world = this.svgToWorld(svgPoint);
    const handle = event.target.closest("[data-handle-index]");
    if (handle && this.selectedTool === "select") {
      this.dragState = {
        type: "handle",
        edgeKey: handle.dataset.edgeKey || "",
        handleIndex: Number(handle.dataset.handleIndex || "0"),
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
        };
        this.previewWorld = world;
        this.previewSnapName = lmName;
        this.editorSvg.setPointerCapture(event.pointerId);
        this.render();
        return;
      }
      this.selection = { type: "lm", key: lmName };
      if (this.selectedTool === "select") {
        this.dragState = {
          type: "landmark",
          name: lmName,
        };
        this.editorSvg.setPointerCapture(event.pointerId);
      }
      this.render();
      return;
    }

    const edgeNode = event.target.closest("[data-edge-key]");
    if (edgeNode && this.selectedTool === "select") {
      this.selection = { type: "edge", key: edgeNode.dataset.edgeKey || "" };
      this.render();
      return;
    }

    if (this.selectedTool === "lm") {
      this.addLandmark(world);
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
        };
        this.previewWorld = world;
        this.previewSnapName = nearest.name;
        this.editorSvg.setPointerCapture(event.pointerId);
        this.render();
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
    this.render();
  }

  onPointerMove(event) {
    if (!this.currentMap) {
      return;
    }
    const svgPoint = this.eventToSvgPoint(event);
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
      return;
    }
    if (this.dragState?.type === "handle") {
      const world = this.svgToWorld(svgPoint);
      this.moveCurveHandle(this.dragState.edgeKey, this.dragState.handleIndex, world);
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

  onPointerUp() {
    if (this.dragState?.type === "edge_chain") {
      this.previewWorld = null;
      this.previewSnapName = "";
      this.render();
    }
    this.dragState = null;
  }

  handleEdgeToolClick(lmName) {
    if (!lmName) {
      return;
    }
    if (!this.edgeStartLm) {
      this.edgeStartLm = lmName;
      this.selection = { type: "lm", key: lmName };
      this.log("info", `Edge start selected: ${lmName}. Click the destination LM.`);
      this.render();
      return;
    }
    if (this.edgeStartLm === lmName) {
      this.edgeStartLm = "";
      this.previewWorld = null;
      this.render();
      return;
    }
    this.createEdge(this.edgeStartLm, lmName);
    this.edgeStartLm = "";
    this.previewWorld = null;
    this.render();
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
  }

  moveLandmark(name, world) {
    const landmark = this.landmarkByName(name);
    if (!landmark) {
      return;
    }
    landmark.x = this.round(world.x);
    landmark.y = this.round(world.y);
    this.refreshConnectedEdges(name);
    this.recomputeAllEdgeLengths();
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
    this.recomputeAllEdgeLengths();
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
      properties: { direction: 2 },
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
    if (this.selection.type === "lm") {
      const name = this.selection.key;
      this.currentMap.lms = this.currentMap.lms.filter((item) => item.name !== name);
      this.currentMap.edges = this.currentMap.edges.filter((edge) => edge.from !== name && edge.to !== name);
      this.selection = { type: "none", key: "" };
      this.markDirty(`Removed landmark ${name}.`);
      return;
    }
    if (this.selection.type === "edge") {
      const key = this.selection.key;
      this.currentMap.edges = this.currentMap.edges.filter((edge) => this.edgeKey(edge) !== key);
      this.selection = { type: "none", key: "" };
      this.markDirty(`Removed edge ${key}.`);
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
    const previousName = current.name;
    current.name = nextName;
    current.x = this.round(Number.isFinite(nextX) ? nextX : current.x);
    current.y = this.round(Number.isFinite(nextY) ? nextY : current.y);
    current.ignoreDir = nextIgnoreDir;
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
    this.recomputeAllEdgeLengths();
    this.markDirty(`Updated landmark ${nextName}.`, { quietLog: true });
  }

  applyEdgeInspector() {
    if (this.selection.type !== "edge") {
      return;
    }
    const edge = this.edgeByKey(this.selection.key);
    if (!edge) {
      return;
    }
    const nextKind = this.edgeKindSelect.value === "curve" ? "curve" : "line";
    const nextType = String(this.edgeTypeInput.value || "").trim() || (nextKind === "curve" ? "DegenerateBezier" : "FeatureLine");
    const direction = Number(this.edgeDirectionSelect.value || "2");
    edge.kind = nextKind;
    edge.type = nextType;
    edge.properties = typeof edge.properties === "object" && edge.properties ? edge.properties : {};
    edge.properties.direction = Number.isFinite(direction) ? direction : 2;
    if (nextKind === "curve") {
      this.ensureCurveGeometry(edge);
    } else {
      delete edge.geometry;
      delete edge.control_points;
      delete edge.curve_type;
    }
    this.recomputeAllEdgeLengths();
    this.markDirty(`Updated edge ${edge.from} -> ${edge.to}.`, { quietLog: true });
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
    this.renderHeader();
    this.renderRobotMaps();
    this.renderLocalMaps();
    this.renderDraftSummary();
    this.renderCanvas();
    this.renderInspector();
    this.renderLogs();
  }

  renderHeader() {
    const robotName = this.robot?.name || this.robotName || this.robotId;
    this.editorRobotTitle.textContent = robotName;
    const currentName = this.currentMap?.mapName || "-";
    const dirtyText = this.dirty ? "Draft has local edits." : "Draft is clean.";
    this.editorStatusText.textContent = `Current local draft: ${currentName}. ${dirtyText}`;
  }

  renderRobotMaps() {
    this.activeRobotMapText.textContent = this.currentSourceMapName || "-";
    this.robotMapsList.innerHTML = "";
    this.robotMapsList.append(this.infoCard("Robot map sync is managed from Control. This editor works only with local drafts."));
  }

  renderLocalMaps() {
    this.localDraftCountText.textContent = `${this.localMaps.length} saved`;
    this.localMapsList.innerHTML = "";
    if (!this.localMaps.length) {
      this.localMapsList.append(this.infoCard("No local drafts yet. Pull a robot map or save the current draft."));
      return;
    }
    for (const item of this.localMaps) {
      const card = document.createElement("div");
      card.className = "map-card";
      if (item.mapName === this.currentLocalMapName || item.mapName === this.selectedLocalMapName) {
        card.classList.add("active");
      }
      card.innerHTML = `
        <div class="map-card-head">
          <div>
            <strong>${this.escapeHtml(item.mapName || "-")}</strong>
            <p>source: ${this.escapeHtml(item.sourceMapName || "-")}</p>
          </div>
          <span class="state-chip ${item.hasLocalChanges ? "dirty" : "clean"}">${item.hasLocalChanges ? "not pushed" : "synced"}</span>
        </div>
        <div class="map-card-actions">
          <button type="button" data-action="open">Open</button>
        </div>
      `;
      card.querySelector('[data-action="open"]').addEventListener("click", () => this.openLocalDraft(item.mapName));
      this.localMapsList.append(card);
    }
  }

  renderDraftSummary() {
    this.currentDraftText.textContent = this.currentMap?.mapName || "-";
    this.draftStateChip.textContent = this.dirty ? "dirty" : "clean";
    this.draftStateChip.classList.toggle("dirty", this.dirty);
    this.draftStateChip.classList.toggle("clean", !this.dirty);
    if (!this.currentMap) {
      this.draftMetaText.textContent = "No local draft loaded.";
      return;
    }
    const parts = [];
    if (this.currentLocalMapName) {
      parts.push(`local: ${this.currentLocalMapName}`);
    }
    if (this.currentSourceMapName) {
      parts.push(`robot source: ${this.currentSourceMapName}`);
    }
    parts.push(`${this.currentMap.lms.length} LM`);
    parts.push(`${this.currentMap.edges.length} edges`);
    this.draftMetaText.textContent = parts.join(" • ");
  }

  renderCanvas() {
    if (!this.currentMap) {
      this.editorMapImage.setAttribute("href", "");
      this.editorEdgeLayer.innerHTML = "";
      this.editorLmLayer.innerHTML = "";
      this.editorHandleLayer.innerHTML = "";
      this.editorPreviewLayer.innerHTML = "";
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
    for (const edge of this.currentMap.edges) {
      const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
      path.setAttribute("fill", "none");
      path.setAttribute("stroke-width", this.selection.type === "edge" && this.selection.key === this.edgeKey(edge) ? "3.4" : "2.25");
      path.setAttribute("stroke", this.selection.type === "edge" && this.selection.key === this.edgeKey(edge) ? "var(--edge-selected)" : "var(--edge)");
      path.setAttribute("stroke-linecap", "round");
      path.dataset.edgeKey = this.edgeKey(edge);
      path.setAttribute("d", this.edgePath(edge));
      this.editorEdgeLayer.append(path);
      const arrow = this.drawEdgeDirectionArrow(edge);
      if (arrow) {
        this.editorEdgeLayer.append(arrow);
      }
    }
  }

  renderLandmarks() {
    this.editorLmLayer.innerHTML = "";
    for (const landmark of this.currentMap.lms) {
      const svgPoint = this.worldToSvg(landmark);
      const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
      group.dataset.lmName = landmark.name;

      const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      circle.setAttribute("cx", String(svgPoint.x));
      circle.setAttribute("cy", String(svgPoint.y));
      circle.setAttribute("r", this.selection.type === "lm" && this.selection.key === landmark.name ? "7.5" : "6");
      circle.setAttribute("fill", this.selection.type === "lm" && this.selection.key === landmark.name ? "var(--lm-selected)" : "var(--lm)");
      circle.setAttribute("stroke", "#ffffff");
      circle.setAttribute("stroke-width", "2");
      group.append(circle);

      const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
      label.setAttribute("x", String(svgPoint.x));
      label.setAttribute("y", String(svgPoint.y + 17));
      label.setAttribute("text-anchor", "middle");
      label.setAttribute("font-size", "11");
      label.setAttribute("font-weight", "700");
      label.setAttribute("fill", "var(--text)");
      label.textContent = landmark.name;
      group.append(label);

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
    const guidePoint = this.currentGuidePoint();
    if (guidePoint) {
      this.drawGuideAtPoint(guidePoint);
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

  drawEdgeDirectionArrow(edge) {
    const segment = this.graphDirectionSegment(edge);
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

  graphDirectionSegment(edge) {
    let mid;
    let tangent;
    if (Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      mid = this.bezierPoint(edge.control_points, 0.5);
      tangent = this.bezierDerivative(edge.control_points, 0.5);
    } else {
      const start = this.landmarkByName(edge.from);
      const goal = this.landmarkByName(edge.to);
      if (!start || !goal) {
        return null;
      }
      mid = {
        x: start.x + ((goal.x - start.x) * 0.5),
        y: start.y + ((goal.y - start.y) * 0.5),
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
      this.edgeDirectionSelect.value = String((edge.properties && edge.properties.direction) ?? 2);
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
    const origin = Array.isArray(mapMeta.origin) ? mapMeta.origin : [0, 0, 0];
    const resolution = Number(mapMeta.resolution || 1);
    const padding = Number(mapMeta.viewPadding || 0);
    const height = Number(mapMeta.height || 0);
    return {
      x: padding + ((Number(point.x) - Number(origin[0] || 0)) / resolution),
      y: padding + height - ((Number(point.y) - Number(origin[1] || 0)) / resolution),
    };
  }

  svgToWorld(point) {
    const mapMeta = this.currentMap?.map || {};
    const origin = Array.isArray(mapMeta.origin) ? mapMeta.origin : [0, 0, 0];
    const resolution = Number(mapMeta.resolution || 1);
    const padding = Number(mapMeta.viewPadding || 0);
    const height = Number(mapMeta.height || 0);
    return {
      x: this.round(Number(origin[0] || 0) + ((point.x - padding) * resolution)),
      y: this.round(Number(origin[1] || 0) + ((height - (point.y - padding)) * resolution)),
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
    this.recomputeAllEdgeLengths();
    this.render();
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
    window.close();
    window.setTimeout(() => {
      if (!window.closed) {
        if (window.history.length > 1) {
          window.history.back();
          return;
        }
        window.location.assign("/");
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
    return JSON.parse(JSON.stringify(value));
  }

  async getJson(url) {
    const response = await fetch(url, { headers: { Accept: "application/json" } });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || payload.ok === false) {
      throw new Error(payload.error || `Request failed: ${response.status}`);
    }
    return payload;
  }

  async postJson(url, payload) {
    const response = await fetch(url, {
      method: "POST",
      headers: {
        "Accept": "application/json",
        "Content-Type": "application/json",
      },
      body: JSON.stringify(payload),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok || data.ok === false) {
      throw new Error(data.error || `Request failed: ${response.status}`);
    }
    return data;
  }

  escapeHtml(value) {
    return String(value)
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#39;");
  }
}

window.addEventListener("DOMContentLoaded", () => {
  const app = new RobotMapEditorApp();
  app.init().catch((error) => {
    window.alert(error instanceof Error ? error.message : String(error));
  });
});
