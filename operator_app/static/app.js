class FleetRobotModelEditor {
  constructor(dom, onChange) {
    this.dom = dom;
    this.onChange = onChange;
    this.center = { x: 260, y: 230 };
    this.scale = 330;
    this.view = { zoom: 1, panX: 0, panY: 0 };
    this.drag = null;
    this.panDrag = null;
    this.snapTolerance = 0.025;
    this.frameOrder = [
      ["lidar", "LiDAR"],
      ["imu", "IMU"],
      ["wheel_left", "Wheel L"],
      ["wheel_right", "Wheel R"],
    ];
    this.model = this.defaultModel();
  }

  defaultModel() {
    return {
      footprint: [
        { x: 0.220000, y: 0.000000 },
        { x: 0.203253, y: 0.084190 },
        { x: 0.155563, y: 0.155563 },
        { x: 0.084190, y: 0.203253 },
        { x: 0.000000, y: 0.220000 },
        { x: -0.084190, y: 0.203253 },
        { x: -0.155563, y: 0.155563 },
        { x: -0.203253, y: 0.084190 },
        { x: -0.220000, y: 0.000000 },
        { x: -0.203253, y: -0.084190 },
        { x: -0.155563, y: -0.155563 },
        { x: -0.084190, y: -0.203253 },
        { x: 0.000000, y: -0.220000 },
        { x: 0.084190, y: -0.203253 },
        { x: 0.155563, y: -0.155563 },
        { x: 0.203253, y: -0.084190 },
      ],
      frames: {
        lidar: { x: 0.28, y: 0, label: "LiDAR", color: "#1f6feb" },
        imu: { x: 0, y: 0, label: "IMU", color: "#d95521" },
        wheel_left: { x: 0, y: 0.225, label: "WL", color: "#2f3a4a" },
        wheel_right: { x: 0, y: -0.225, label: "WR", color: "#2f3a4a" },
      },
    };
  }

  init() {
    this.dom.zoomIn.addEventListener("click", () => this.zoom(1.18));
    this.dom.zoomOut.addEventListener("click", () => this.zoom(0.85));
    this.dom.resetView.addEventListener("click", () => this.resetView());
    this.dom.resetModel.addEventListener("click", () => {
      this.model = this.defaultModel();
      this.render();
      this.emitChange();
    });
    this.attachPointerEvents();
    this.render();
  }

  setModel(model) {
    if (!model || !Array.isArray(model.footprint) || !model.frames) {
      return;
    }
    const defaults = this.defaultModel();
    this.model = {
      footprint: model.footprint.map((point) => ({
        x: this.round(Number(point.x || 0)),
        y: this.round(Number(point.y || 0)),
      })),
      frames: { ...defaults.frames },
    };
    for (const [name, frame] of Object.entries(model.frames || {})) {
      const fallback = defaults.frames[name] || { x: 0, y: 0, label: name, color: "#2f3a4a" };
      this.model.frames[name] = {
        ...fallback,
        ...frame,
        x: this.round(Number(frame.x ?? fallback.x)),
        y: this.round(Number(frame.y ?? fallback.y)),
      };
    }
    this.constrainAllFrames();
    this.render();
  }

  getModel() {
    return {
      footprint: this.model.footprint.map((point) => ({ ...point })),
      frames: Object.fromEntries(Object.entries(this.model.frames).map(([name, frame]) => [name, { ...frame }])),
    };
  }

  attachPointerEvents() {
    const svg = this.dom.svg;
    svg.addEventListener("pointerdown", (event) => {
      if (event.button !== 0) {
        return;
      }
      event.preventDefault();
      const target = event.target.closest("[data-model-drag]");
      if (target) {
        this.drag = {
          kind: target.dataset.modelDrag,
          index: target.dataset.index ? Number(target.dataset.index) : null,
          frame: target.dataset.frame || "",
        };
        svg.setPointerCapture(event.pointerId);
        this.applyDrag(event);
        return;
      }
      this.panDrag = { x: event.clientX, y: event.clientY };
      svg.setPointerCapture(event.pointerId);
    });
    svg.addEventListener("pointermove", (event) => {
      if (this.drag) {
        event.preventDefault();
        this.applyDrag(event);
        return;
      }
      if (this.panDrag) {
        event.preventDefault();
        const prev = this.eventToSvg({ clientX: this.panDrag.x, clientY: this.panDrag.y });
        const curr = this.eventToSvg(event);
        if (prev && curr) {
          this.view.panX += curr.x - prev.x;
          this.view.panY += curr.y - prev.y;
          this.panDrag = { x: event.clientX, y: event.clientY };
          this.renderSvg();
        }
      }
    });
    const stop = (event) => {
      this.drag = null;
      this.panDrag = null;
      if (svg.hasPointerCapture(event.pointerId)) {
        svg.releasePointerCapture(event.pointerId);
      }
    };
    svg.addEventListener("pointerup", stop);
    svg.addEventListener("pointercancel", stop);
    svg.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.zoom(event.deltaY < 0 ? 1.12 : 0.9);
    }, { passive: false });
  }

  createSvg(tag, attrs) {
    const element = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [key, value] of Object.entries(attrs)) {
      element.setAttribute(key, String(value));
    }
    return element;
  }

  toSvg(point) {
    const scale = this.scale * this.view.zoom;
    return {
      x: this.center.x + this.view.panX + (point.x * scale),
      y: this.center.y + this.view.panY - (point.y * scale),
    };
  }

  eventToSvg(event) {
    const ctm = this.dom.svg.getScreenCTM();
    if (!ctm) {
      return null;
    }
    return new DOMPoint(event.clientX, event.clientY).matrixTransform(ctm.inverse());
  }

  eventToLocal(event) {
    const point = this.eventToSvg(event);
    if (!point) {
      return null;
    }
    const scale = this.scale * this.view.zoom;
    return {
      x: this.clamp((point.x - this.center.x - this.view.panX) / scale, -1.4, 1.4),
      y: this.clamp((this.center.y + this.view.panY - point.y) / scale, -1.1, 1.1),
    };
  }

  applyDrag(event) {
    const point = this.eventToLocal(event);
    if (!point || !this.drag) {
      return;
    }
    if (this.drag.kind === "footprint") {
      const snapped = this.snapPoint(point, this.drag.index);
      this.model.footprint[this.drag.index] = { x: this.round(snapped.x), y: this.round(snapped.y) };
      this.constrainAllFrames();
    }
    if (this.drag.kind === "frame") {
      const snapped = this.snapPoint(point);
      const kept = this.keepInsideFootprint(snapped);
      this.model.frames[this.drag.frame].x = this.round(kept.x);
      this.model.frames[this.drag.frame].y = this.round(kept.y);
    }
    this.render();
    this.emitChange();
  }

  snapPoint(point, index = null) {
    const snapped = { ...point };
    if (Math.abs(snapped.x) <= this.snapTolerance) {
      snapped.x = 0;
    }
    if (Math.abs(snapped.y) <= this.snapTolerance) {
      snapped.y = 0;
    }
    for (let i = 0; i < this.model.footprint.length; i += 1) {
      if (i === index) {
        continue;
      }
      const other = this.model.footprint[i];
      if (Math.abs(snapped.x - other.x) <= this.snapTolerance) {
        snapped.x = other.x;
      }
      if (Math.abs(snapped.y - other.y) <= this.snapTolerance) {
        snapped.y = other.y;
      }
    }
    for (const frame of Object.values(this.model.frames)) {
      if (Math.abs(snapped.x - frame.x) <= this.snapTolerance) {
        snapped.x = frame.x;
      }
      if (Math.abs(snapped.y - frame.y) <= this.snapTolerance) {
        snapped.y = frame.y;
      }
    }
    return snapped;
  }

  keepInsideFootprint(point) {
    if (this.pointInPolygon(point, this.model.footprint)) {
      return point;
    }
    const boundary = this.nearestPointOnPolygon(point, this.model.footprint);
    const center = this.footprintCentroid();
    return {
      x: boundary.x + ((center.x - boundary.x) * 0.01),
      y: boundary.y + ((center.y - boundary.y) * 0.01),
    };
  }

  constrainAllFrames() {
    for (const frame of Object.values(this.model.frames)) {
      const kept = this.keepInsideFootprint(frame);
      frame.x = this.round(kept.x);
      frame.y = this.round(kept.y);
    }
  }

  render() {
    this.renderSvg();
    this.renderFields();
  }

  renderSvg() {
    const svg = this.dom.svg;
    svg.innerHTML = "";
    const bounds = { left: 18, right: 502, top: 18, bottom: 442 };
    svg.append(this.createSvg("rect", { x: 0, y: 0, width: 520, height: 460, class: "model-pan-surface" }));
    for (let value = -1.2; value <= 1.2001; value += 0.1) {
      const rounded = Math.round(value * 10) / 10;
      const vertical = this.toSvg({ x: rounded, y: 0 });
      const horizontal = this.toSvg({ x: 0, y: rounded });
      const major = Math.abs((rounded * 10) % 2) < 0.0001;
      if (vertical.x >= bounds.left && vertical.x <= bounds.right) {
        svg.append(this.createSvg("line", { x1: vertical.x, y1: bounds.top, x2: vertical.x, y2: bounds.bottom, class: major ? "model-grid-line model-grid-major" : "model-grid-line" }));
      }
      if (horizontal.y >= bounds.top && horizontal.y <= bounds.bottom) {
        svg.append(this.createSvg("line", { x1: bounds.left, y1: horizontal.y, x2: bounds.right, y2: horizontal.y, class: major ? "model-grid-line model-grid-major" : "model-grid-line" }));
      }
    }
    const origin = this.toSvg({ x: 0, y: 0 });
    svg.append(this.createSvg("line", { x1: bounds.left, y1: origin.y, x2: bounds.right, y2: origin.y, class: "model-axis" }));
    svg.append(this.createSvg("line", { x1: origin.x, y1: bounds.top, x2: origin.x, y2: bounds.bottom, class: "model-axis" }));
    for (let value = -1.0; value <= 1.0001; value += 0.2) {
      const rounded = Math.round(value * 10) / 10;
      if (Math.abs(rounded) < 0.0001) {
        continue;
      }
      const xPos = this.toSvg({ x: rounded, y: 0 });
      const yPos = this.toSvg({ x: 0, y: rounded });
      if (xPos.x >= bounds.left && xPos.x <= bounds.right) {
        const label = this.createSvg("text", { x: xPos.x, y: origin.y + 18, class: "model-axis-number" });
        label.textContent = rounded.toFixed(1);
        svg.append(label);
      }
      if (yPos.y >= bounds.top && yPos.y <= bounds.bottom) {
        const label = this.createSvg("text", { x: origin.x - 22, y: yPos.y + 4, class: "model-axis-number" });
        label.textContent = rounded.toFixed(1);
        svg.append(label);
      }
    }
    const polygon = this.model.footprint.map((point) => this.toSvg(point)).map((point) => `${point.x},${point.y}`).join(" ");
    svg.append(this.createSvg("polygon", { points: polygon, class: "model-footprint" }));
    this.model.footprint.forEach((point, index) => {
      const pos = this.toSvg(point);
      svg.append(this.createSvg("circle", { cx: pos.x, cy: pos.y, r: 7, class: "model-handle", "data-model-drag": "footprint", "data-index": index }));
      const label = this.createSvg("text", { x: pos.x, y: pos.y - 10, class: "model-label" });
      label.textContent = `V${index + 1}`;
      svg.append(label);
    });
    svg.append(this.createSvg("circle", { cx: origin.x, cy: origin.y, r: 4, fill: "#111827" }));
    for (const [name] of this.frameOrder) {
      const frame = this.model.frames[name];
      const pos = this.toSvg(frame);
      svg.append(this.createSvg("circle", { cx: pos.x, cy: pos.y, r: 8, fill: frame.color, class: "model-marker", "data-model-drag": "frame", "data-frame": name }));
      const label = this.createSvg("text", { x: pos.x, y: pos.y + 22, class: "model-label" });
      label.textContent = frame.label;
      svg.append(label);
    }
  }

  renderFields() {
    this.dom.footprintFields.innerHTML = "";
    this.model.footprint.forEach((point, index) => {
      this.dom.footprintFields.append(this.pointRow(`V${index + 1}`, point, (axis, value) => {
        this.model.footprint[index][axis] = value;
        this.model.footprint[index] = this.snapPoint(this.model.footprint[index], index);
        this.model.footprint[index].x = this.round(this.model.footprint[index].x);
        this.model.footprint[index].y = this.round(this.model.footprint[index].y);
        this.constrainAllFrames();
      }));
    });
    this.dom.tfFields.innerHTML = "";
    for (const [name, label] of this.frameOrder) {
      this.dom.tfFields.append(this.pointRow(label, this.model.frames[name], (axis, value) => {
        this.model.frames[name][axis] = value;
        const kept = this.keepInsideFootprint(this.snapPoint(this.model.frames[name]));
        this.model.frames[name].x = this.round(kept.x);
        this.model.frames[name].y = this.round(kept.y);
      }));
    }
  }

  pointRow(name, point, setter) {
    const row = document.createElement("div");
    row.className = "model-field-row";
    const title = document.createElement("strong");
    title.textContent = name;
    row.append(title, this.numberInput(point.x, (value) => setter("x", value)), this.numberInput(point.y, (value) => setter("y", value)));
    return row;
  }

  numberInput(value, onChange) {
    const input = document.createElement("input");
    input.type = "number";
    input.step = "0.001";
    input.value = Number(value || 0).toFixed(3);
    input.addEventListener("change", () => {
      onChange(this.round(Number(input.value || 0)));
      this.render();
      this.emitChange();
    });
    return input;
  }

  emitChange() {
    if (this.onChange) {
      this.onChange(this.getModel());
    }
  }

  zoom(multiplier) {
    this.view.zoom = this.clamp(this.view.zoom * multiplier, 0.45, 4);
    this.renderSvg();
  }

  resetView() {
    this.view = { zoom: 1, panX: 0, panY: 0 };
    this.renderSvg();
  }

  pointInPolygon(point, polygon) {
    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i, i += 1) {
      const a = polygon[i];
      const b = polygon[j];
      const crosses = ((a.y > point.y) !== (b.y > point.y)) &&
        (point.x < ((b.x - a.x) * (point.y - a.y)) / ((b.y - a.y) || 0.000001) + a.x);
      if (crosses) {
        inside = !inside;
      }
    }
    return inside;
  }

  nearestPointOnPolygon(point, polygon) {
    let best = polygon[0];
    let bestDistance = Number.POSITIVE_INFINITY;
    for (let i = 0; i < polygon.length; i += 1) {
      const candidate = this.nearestPointOnSegment(point, polygon[i], polygon[(i + 1) % polygon.length]);
      const distance = Math.hypot(point.x - candidate.x, point.y - candidate.y);
      if (distance < bestDistance) {
        best = candidate;
        bestDistance = distance;
      }
    }
    return best;
  }

  nearestPointOnSegment(point, start, end) {
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const lengthSq = (dx * dx) + (dy * dy);
    if (lengthSq <= 0.000001) {
      return start;
    }
    const t = Math.max(0, Math.min(1, (((point.x - start.x) * dx) + ((point.y - start.y) * dy)) / lengthSq));
    return { x: start.x + (dx * t), y: start.y + (dy * t) };
  }

  footprintCentroid() {
    const total = this.model.footprint.reduce((acc, point) => ({ x: acc.x + point.x, y: acc.y + point.y }), { x: 0, y: 0 });
    return { x: total.x / this.model.footprint.length, y: total.y / this.model.footprint.length };
  }

  clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  round(value) {
    return Math.round(Number(value || 0) * 1000) / 1000;
  }
}

class OperatorApp {
  constructor() {
    this.fleetManagerId = "__fleet_manager__";
    this.robots = [];
    this.selectedRobotId = window.localStorage.getItem("operator:selectedRobotId") || "";
    this.selectedFleetRobotName = window.localStorage.getItem("operator:selectedFleetRobotName") || "";
    this.lastProbe = null;
    this.sidebarOpen = false;
    this.pendingRobotMaps = [];
    this.operatorMapPayload = null;
    this.operatorMapSignature = "";
    this.currentStatus = null;
    this.currentRoute = null;
    this.statusRequestPending = false;
    this.navigateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.fleetQueue = [];
    this.fleetQueueSequence = 0;
    this.selectedFleetOrderId = "";
    this.fleetParams = null;
    this.fleetParamsLoaded = false;
    this.robotParams = null;
    this.robotParamsRobotId = "";
    this.robotParamsLoaded = false;
    this.fleetNameEdited = false;
    this.fleetTickPending = false;
    this.fleetStatusSocket = null;
    this.fleetStatusStreamShouldRun = false;
    this.fleetStatusReconnectTimer = null;
    this.fleetStatusReconnectMs = 500;
    this.fleetStatusStreamAttemptedAt = 0;
    this.fleetStatusStreamFallback = false;
    this.fleetHttpFallbackLastAt = 0;
    this.fleetStreamIntervalMs = 180;
    this.robotStatusSocket = null;
    this.robotStatusStreamShouldRun = false;
    this.robotStatusReconnectTimer = null;
    this.robotStatusReconnectMs = 500;
    this.robotStatusStreamAttemptedAt = 0;
    this.robotStatusStreamFallback = false;
    this.robotStreamIntervalMs = 180;
    this.robotStatusRobotId = "";
    this.fleetStatusReceivedAt = 0;
    this.fleetStatusObjectRef = null;
    this.fleetAnimationFrame = null;
    this.fleetAnimationLastAt = 0;
    this.fleetRouteRenderLastAt = 0;
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
    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.fleetEditorEdgeDrag = null;
    this.fleetEditorLmDrag = null;
    this.fleetEditorBezierDrag = null;
    this.fleetEditorPreview = null;
    this.fleetEditorGuideWorld = null;
    this.fleetEditorFieldSyncing = false;
    this.fleetModelEditor = null;
    this.mapDrag = null;
    this.mapClickConsumed = false;
    this.manualKeys = new Set();
    this.teleopPending = false;
    this.mapView = {
      scale: 1,
      tx: 0,
      ty: 0,
      follow: true,
    };
    this.robotMapState = this.emptyMapState();

    this.robotsList = document.getElementById("robotsList");
    this.robotCountText = document.getElementById("robotCountText");
    this.sidebarDrawer = document.getElementById("sidebarDrawer");
    this.sidebarBackdrop = document.getElementById("sidebarBackdrop");
    this.homeButton = document.getElementById("homeButton");
    this.paramsNavButton = document.getElementById("paramsNavButton");
    this.mapEditorNavButton = document.getElementById("mapEditorNavButton");
    this.robotModelNavButton = document.getElementById("robotModelNavButton");
    this.openSidebarButton = document.getElementById("openSidebarButton");
    this.closeSidebarButton = document.getElementById("closeSidebarButton");
    this.emptyState = document.getElementById("emptyState");
    this.robotView = document.getElementById("robotView");
    this.robotActiveMapText = document.getElementById("robotActiveMapText");
    this.operatorActiveMapText = document.getElementById("operatorActiveMapText");
    this.robotStateText = document.getElementById("robotStateText");
    this.nearestLmText = document.getElementById("nearestLmText");
    this.navigateRobotButton = document.getElementById("navigateRobotButton");
    this.cancelRouteButton = document.getElementById("cancelRouteButton");
    this.stopRobotButton = document.getElementById("stopRobotButton");
    this.refreshRobotStatusButton = document.getElementById("refreshRobotStatusButton");
    this.controlPullMapButton = document.getElementById("controlPullMapButton");
    this.controlPushMapButton = document.getElementById("controlPushMapButton");
    this.controlLoadMapButton = document.getElementById("controlLoadMapButton");
    this.mapSyncStatus = document.getElementById("mapSyncStatus");
    this.operatorConsole = document.getElementById("operatorConsole");
    this.fleetControlPanel = document.getElementById("fleetControlPanel");
    this.robotParamsPanel = document.getElementById("robotParamsPanel");
    this.robotParamsJsonInput = document.getElementById("robotParamsJsonInput");
    this.robotReloadParamsButton = document.getElementById("robotReloadParamsButton");
    this.robotFormatParamsButton = document.getElementById("robotFormatParamsButton");
    this.robotSaveParamsButton = document.getElementById("robotSaveParamsButton");
    this.robotModelPanel = document.getElementById("robotModelPanel");
    this.fleetModeSelect = document.getElementById("fleetModeSelect");
    this.fleetRobotNameLabel = document.getElementById("fleetRobotNameLabel");
    this.fleetRobotNameInput = document.getElementById("fleetRobotNameInput");
    this.fleetSpawnLmLabel = document.getElementById("fleetSpawnLmLabel");
    this.fleetSpawnLmLabelText = document.getElementById("fleetSpawnLmLabelText");
    this.fleetSpawnLmSelect = document.getElementById("fleetSpawnLmSelect");
    this.fleetRobotApiLabel = document.getElementById("fleetRobotApiLabel");
    this.fleetRobotApiInput = document.getElementById("fleetRobotApiInput");
    this.fleetAddRobotButton = document.getElementById("fleetAddRobotButton");
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
    this.fleetEditorApplyLmButton = document.getElementById("fleetEditorApplyLmButton");
    this.fleetEditorEdgeFromInput = document.getElementById("fleetEditorEdgeFromInput");
    this.fleetEditorEdgeToInput = document.getElementById("fleetEditorEdgeToInput");
    this.fleetEditorEdgeTrafficSelect = document.getElementById("fleetEditorEdgeTrafficSelect");
    this.fleetEditorEdgeMotionSelect = document.getElementById("fleetEditorEdgeMotionSelect");
    this.fleetEditorApplyEdgeButton = document.getElementById("fleetEditorApplyEdgeButton");
    this.fleetMapSaveButton = document.getElementById("fleetMapSaveButton");
    this.fleetMapSaveAsButton = document.getElementById("fleetMapSaveAsButton");
    this.fleetMapReloadButton = document.getElementById("fleetMapReloadButton");
    this.refreshButton = document.getElementById("refreshButton");
    this.addRobotButton = document.getElementById("addRobotButton");

    this.operatorMapSvg = document.getElementById("operatorMapSvg");
    this.operatorViewport = document.getElementById("operatorViewport");
    this.operatorMapImage = document.getElementById("operatorMapImage");
    this.operatorGraphLayer = document.getElementById("operatorGraphLayer");
    this.operatorRouteLayer = document.getElementById("operatorRouteLayer");
    this.operatorLookaheadLayer = document.getElementById("operatorLookaheadLayer");
    this.operatorLandmarkLayer = document.getElementById("operatorLandmarkLayer");
    this.operatorEditorLayer = document.getElementById("operatorEditorLayer");
    this.operatorRobotLayer = document.getElementById("operatorRobotLayer");
    this.operatorZoomInButton = document.getElementById("operatorZoomInButton");
    this.operatorZoomOutButton = document.getElementById("operatorZoomOutButton");
    this.operatorResetViewButton = document.getElementById("operatorResetViewButton");
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
    this.mapTransferDialog = document.getElementById("mapTransferDialog");
    this.mapTransferTitle = document.getElementById("mapTransferTitle");
    this.mapTransferPercent = document.getElementById("mapTransferPercent");
    this.mapTransferBar = document.getElementById("mapTransferBar");
    this.mapTransferStatus = document.getElementById("mapTransferStatus");
    this.mapTransferCloseButton = document.getElementById("mapTransferCloseButton");
  }

  async init() {
    this.bindEvents();
    this.initFleetModelEditor();
    await this.applyRoute({ replace: window.location.pathname === "/" });
    window.addEventListener("popstate", () => {
      this.applyRoute().catch((error) => {
        this.robotMessageText.textContent = error.message || String(error);
      });
    });
    await this.refreshRobots();
    this.applyDeferredUiActions();
    this.syncFleetStatusStream();
    window.setInterval(() => {
      this.refreshRobots({ quiet: true, lightweight: true }).catch(() => {});
    }, 12000);
    window.setInterval(() => {
      this.fetchSelectedRobotStatus(true).catch(() => {});
    }, 800);
    window.setInterval(() => {
      this.tickFleetIfSelected().catch(() => {});
    }, 80);
    window.setInterval(() => {
      this.sendTeleopIfNeeded().catch(() => {});
    }, 120);
  }

  bindEvents() {
    this.homeButton.addEventListener("click", async () => this.navigateHomePage());
    this.paramsNavButton.addEventListener("click", async () => this.navigateParamsPage());
    this.mapEditorNavButton.addEventListener("click", async () => this.navigateMapEditorPage());
    this.robotModelNavButton.addEventListener("click", async () => this.navigateRobotModelPage());
    this.openSidebarButton.addEventListener("click", () => this.openSidebar());
    this.closeSidebarButton.addEventListener("click", () => this.closeSidebar());
    this.sidebarBackdrop.addEventListener("click", () => this.closeSidebar());
    this.refreshButton.addEventListener("click", () => this.refreshRobots());
    this.addRobotButton.addEventListener("click", () => this.openAddRobotDialog());
    this.navigateRobotButton.addEventListener("click", () => this.toggleNavigateMode());
    this.cancelRouteButton.addEventListener("click", () => this.cancelRoute());
    this.stopRobotButton.addEventListener("click", () => this.stopRobot());
    this.refreshRobotStatusButton.addEventListener("click", () => this.fetchSelectedRobotStatus(false));
    this.controlPullMapButton.addEventListener("click", () => this.handlePullMap());
    this.controlPushMapButton.addEventListener("click", () => this.handlePushMap());
    this.controlLoadMapButton.addEventListener("click", () => this.handleLoadMap());
    this.fleetModeSelect.addEventListener("change", () => {
      this.syncFleetRemoteFields();
      this.handleFleetModeChange();
    });
    this.fleetRobotNameInput.addEventListener("input", () => {
      this.fleetNameEdited = true;
    });
    this.fleetAddRobotButton.addEventListener("click", () => this.handleFleetAddRobot());
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
    this.fleetEditorApplyLmButton.addEventListener("click", () => this.applyFleetEditorLmFields());
    this.fleetEditorApplyEdgeButton.addEventListener("click", () => this.applyFleetEditorEdgeFields());
    this.fleetMapSaveButton.addEventListener("click", () => this.saveFleetMap(false));
    this.fleetMapSaveAsButton.addEventListener("click", () => this.saveFleetMap(true));
    this.fleetMapReloadButton.addEventListener("click", () => this.reloadFleetMapDraft());
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
      }
    );
    this.fleetModelEditor.init();
  }

  pageForPath(pathname) {
    const path = String(pathname || "/").replace(/\/+$/, "") || "/";
    if (path === "/params") {
      return "params";
    }
    if (path === "/robot_model" || path === "/robot-model") {
      return "model";
    }
    if (path === "/map_editor" || path === "/map-editor") {
      return "map";
    }
    return "fleet";
  }

  pathForFleetPage(tabName) {
    const tab = ["fleet", "params", "model", "map"].includes(tabName) ? tabName : "fleet";
    return {
      fleet: "/home",
      params: "/params",
      model: "/robot_model",
      map: "/map_editor",
    }[tab];
  }

  async navigateHomePage(options = {}) {
    const path = this.pathForFleetPage("fleet");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "fleet" }, "", path);
    }
    this.setFleetTab("fleet");
    this.renderSelectedRobot();
  }

  async navigateParamsPage(options = {}) {
    const path = this.pathForFleetPage("params");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "params" }, "", path);
    }
    this.setFleetTab("params");
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
    const path = this.pathForFleetPage("model");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "model" }, "", path);
    }
    this.setFleetTab("model");
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
    const canonical = this.pathForFleetPage(tab);
    if (options.replace && window.location.pathname !== canonical) {
      window.history.replaceState({ fleetPage: tab }, "", canonical);
    }
    this.setFleetTab(tab);
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
    const fleet = this.robots.find((robot) => this.isFleetManager(robot));
    if (!fleet || this.selectedRobotId === fleet.id) {
      return false;
    }
    this.selectedRobotId = fleet.id;
    window.localStorage.setItem("operator:selectedRobotId", fleet.id);
    this.currentStatus = null;
    this.currentRoute = null;
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
      this.selectedRobotId = "";
      window.localStorage.removeItem("operator:selectedRobotId");
      this.currentStatus = null;
      this.currentRoute = null;
      this.syncFleetStatusStream();
      return false;
    }
    this.selectedRobotId = robot.id;
    window.localStorage.setItem("operator:selectedRobotId", robot.id);
    this.currentStatus = null;
    this.currentRoute = null;
    this.syncFleetStatusStream();
    return true;
  }

  setFleetTab(tabName) {
    const tab = ["fleet", "params", "model", "map"].includes(tabName) ? tabName : "fleet";
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
    this.syncFleetPageClass(this.isFleetManager());
    this.operatorMapSvg.classList.toggle("fleet-map-editor-active", this.fleetMapEditorActive);
    if (tab === "map") {
      this.navigateMode = false;
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.syncModeButtons();
      this.ensureFleetMapDraft();
      this.robotMessageText.textContent = "Fleet map editor active.";
    }
    this.renderOperatorMap();
  }

  syncFleetPageClass(isFleet = this.isFleetManager()) {
    const isRobotModel = this.fleetActiveTab === "model";
    const isRobotParams = !isFleet && this.fleetActiveTab === "params";
    const pageKey = isRobotModel ? "robot-model" : (isFleet ? (this.fleetActiveTab || "fleet") : (isRobotParams ? "robot-params" : "robot"));
    document.body.dataset.fleetPage = pageKey;
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
    if (isFleet && !isRobotModel) {
      this.operatorConsole.classList.add(`fleet-page-${this.fleetActiveTab || "fleet"}`);
    }
  }

  setFleetMapTool(tool) {
    this.fleetMapTool = ["select", "lm", "edge"].includes(tool) ? tool : "select";
    this.fleetMapToolButtons.forEach((button) => button.classList.toggle("active", button.dataset.fleetMapTool === this.fleetMapTool));
    const hints = {
      select: "Select LM/edge. Drag LM. Drag Bezier handles. Right-click LM/edge deletes.",
      lm: "Click empty map space to add an LM.",
      edge: "Hold an LM and drag through other LMs to create edges.",
    };
    this.fleetMapEditorHelp.textContent = hints[this.fleetMapTool] || hints.select;
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

  selectedRobot() {
    return this.robots.find((robot) => robot.id === this.selectedRobotId) || null;
  }

  isFleetManager(robot = this.selectedRobot()) {
    return Boolean(robot && (robot.id === this.fleetManagerId || robot.type === "fleet_manager"));
  }

  isRos2Robot(robot = this.selectedRobot()) {
    const type = String(robot?.type || robot?.mode || "").toLowerCase();
    return Boolean(robot && type === "ros2");
  }

  fleetRuntimeMode(status = this.currentStatus) {
    return String(status?.mode || this.fleetModeSelect?.value || "simulation");
  }

  isFleetRobotsMode() {
    return this.isFleetManager() && this.fleetRuntimeMode() === "robots";
  }

  isFleetRemoteRobot(robot) {
    const mode = String(robot?.mode || robot?.type || "").toLowerCase();
    return ["remote", "remote_ros", "ros", "robot", "real"].includes(mode);
  }

  shouldAnimateFleetRobot(robot) {
    return this.fleetRuntimeMode() !== "robots" && !this.isFleetRemoteRobot(robot);
  }

  robotApiPath(path) {
    const robot = this.selectedRobot();
    if (!robot) {
      throw new Error("No robot selected.");
    }
    if (this.isFleetManager(robot)) {
      throw new Error("Fleet Manager uses fleet-manager API.");
    }
    return `/robots/${encodeURIComponent(robot.id)}${path}`;
  }

  syncFleetStatusStream() {
    if (this.isFleetManager()) {
      this.closeRobotStatusStream();
      this.openFleetStatusStream();
      return;
    }
    this.closeFleetStatusStream();
    this.stopFleetAnimationLoop();
    if (this.selectedRobot()) {
      this.openRobotStatusStream();
    } else {
      this.closeRobotStatusStream();
    }
  }

  fleetStatusStreamOpen() {
    return typeof WebSocket !== "undefined"
      && this.fleetStatusSocket
      && this.fleetStatusSocket.readyState === WebSocket.OPEN;
  }

  fleetStatusStreamConnecting() {
    return typeof WebSocket !== "undefined"
      && this.fleetStatusSocket
      && this.fleetStatusSocket.readyState === WebSocket.CONNECTING;
  }

  fleetStatusStreamConnectingFresh() {
    return this.fleetStatusStreamConnecting()
      && performance.now() - this.fleetStatusStreamAttemptedAt < 1200;
  }

  openFleetStatusStream() {
    this.fleetStatusStreamShouldRun = true;
    if (typeof WebSocket === "undefined") {
      this.fleetStatusStreamFallback = true;
      return;
    }
    if (this.fleetStatusStreamOpen() || this.fleetStatusStreamConnecting()) {
      return;
    }
    if (this.fleetStatusReconnectTimer) {
      window.clearTimeout(this.fleetStatusReconnectTimer);
      this.fleetStatusReconnectTimer = null;
    }

    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${scheme}://${window.location.host}/ws/fleet-manager?intervalMs=${this.fleetStreamIntervalMs}`);
    this.fleetStatusSocket = socket;
    this.fleetStatusStreamAttemptedAt = performance.now();

    socket.addEventListener("open", () => {
      if (this.fleetStatusSocket !== socket) {
        return;
      }
      this.fleetStatusStreamFallback = false;
      this.fleetStatusReconnectMs = 500;
    });
    socket.addEventListener("message", (event) => {
      if (this.fleetStatusSocket !== socket) {
        return;
      }
      this.handleFleetStatusStreamMessage(event);
    });
    socket.addEventListener("error", () => {
      if (this.fleetStatusSocket === socket) {
        this.fleetStatusStreamFallback = true;
      }
    });
    socket.addEventListener("close", () => {
      if (this.fleetStatusSocket !== socket) {
        return;
      }
      this.fleetStatusSocket = null;
      this.fleetStatusStreamFallback = true;
      this.scheduleFleetStatusReconnect();
    });
  }

  closeFleetStatusStream() {
    this.fleetStatusStreamShouldRun = false;
    if (this.fleetStatusReconnectTimer) {
      window.clearTimeout(this.fleetStatusReconnectTimer);
      this.fleetStatusReconnectTimer = null;
    }
    if (!this.fleetStatusSocket) {
      this.fleetStatusStreamAttemptedAt = 0;
      return;
    }
    const socket = this.fleetStatusSocket;
    this.fleetStatusSocket = null;
    this.fleetStatusStreamAttemptedAt = 0;
    try {
      socket.close(1000, "operator target changed");
    } catch (_) {
      // Some browsers throw if the socket is already closing.
    }
  }

  scheduleFleetStatusReconnect() {
    if (!this.fleetStatusStreamShouldRun || !this.isFleetManager() || this.fleetStatusReconnectTimer) {
      return;
    }
    const delay = this.fleetStatusReconnectMs;
    this.fleetStatusReconnectMs = Math.min(5000, Math.round(this.fleetStatusReconnectMs * 1.6));
    this.fleetStatusReconnectTimer = window.setTimeout(() => {
      this.fleetStatusReconnectTimer = null;
      if (this.fleetStatusStreamShouldRun && this.isFleetManager()) {
        this.openFleetStatusStream();
      }
    }, delay);
  }

  robotStatusStreamOpen() {
    return typeof WebSocket !== "undefined"
      && this.robotStatusSocket
      && this.robotStatusSocket.readyState === WebSocket.OPEN;
  }

  robotStatusStreamConnecting() {
    return typeof WebSocket !== "undefined"
      && this.robotStatusSocket
      && this.robotStatusSocket.readyState === WebSocket.CONNECTING;
  }

  robotStatusStreamConnectingFresh() {
    return this.robotStatusStreamConnecting()
      && performance.now() - this.robotStatusStreamAttemptedAt < 1200;
  }

  openRobotStatusStream() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      this.closeRobotStatusStream();
      return;
    }
    this.robotStatusStreamShouldRun = true;
    if (typeof WebSocket === "undefined") {
      this.robotStatusStreamFallback = true;
      return;
    }
    if (this.robotStatusRobotId && this.robotStatusRobotId !== robot.id) {
      this.closeRobotStatusStream();
      this.robotStatusStreamShouldRun = true;
    }
    if (this.robotStatusStreamOpen() || this.robotStatusStreamConnecting()) {
      return;
    }
    if (this.robotStatusReconnectTimer) {
      window.clearTimeout(this.robotStatusReconnectTimer);
      this.robotStatusReconnectTimer = null;
    }

    const url = this.robotStatusWsUrl(robot);
    if (!url) {
      this.robotStatusStreamFallback = true;
      return;
    }
    const socket = new WebSocket(url);
    this.robotStatusSocket = socket;
    this.robotStatusRobotId = robot.id;
    this.robotStatusStreamAttemptedAt = performance.now();

    socket.addEventListener("open", () => {
      if (this.robotStatusSocket !== socket) {
        return;
      }
      this.robotStatusStreamFallback = false;
      this.robotStatusReconnectMs = 500;
    });
    socket.addEventListener("message", (event) => {
      if (this.robotStatusSocket !== socket) {
        return;
      }
      this.handleRobotStatusStreamMessage(event);
    });
    socket.addEventListener("error", () => {
      if (this.robotStatusSocket === socket) {
        this.robotStatusStreamFallback = true;
      }
    });
    socket.addEventListener("close", () => {
      if (this.robotStatusSocket !== socket) {
        return;
      }
      this.robotStatusSocket = null;
      this.robotStatusStreamFallback = true;
      this.scheduleRobotStatusReconnect();
    });
  }

  robotStatusWsUrl(robot) {
    if (this.isRos2Robot(robot)) {
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      return `${protocol}//${window.location.host}/ws/robot/status?robotId=${encodeURIComponent(robot.id)}&intervalMs=${this.robotStreamIntervalMs}`;
    }
    const baseUrl = String(robot?.baseUrl || "").trim();
    if (!baseUrl) {
      return "";
    }
    try {
      const url = new URL(baseUrl);
      url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
      url.pathname = "/ws/robot/status";
      url.search = `?intervalMs=${this.robotStreamIntervalMs}`;
      url.hash = "";
      return url.toString();
    } catch (_) {
      return "";
    }
  }

  closeRobotStatusStream() {
    this.robotStatusStreamShouldRun = false;
    if (this.robotStatusReconnectTimer) {
      window.clearTimeout(this.robotStatusReconnectTimer);
      this.robotStatusReconnectTimer = null;
    }
    if (!this.robotStatusSocket) {
      this.robotStatusRobotId = "";
      this.robotStatusStreamAttemptedAt = 0;
      return;
    }
    const socket = this.robotStatusSocket;
    this.robotStatusSocket = null;
    this.robotStatusRobotId = "";
    this.robotStatusStreamAttemptedAt = 0;
    try {
      socket.close(1000, "operator target changed");
    } catch (_) {
      // Some browsers throw if the socket is already closing.
    }
  }

  scheduleRobotStatusReconnect() {
    if (!this.robotStatusStreamShouldRun || this.isFleetManager() || this.robotStatusReconnectTimer) {
      return;
    }
    const delay = this.robotStatusReconnectMs;
    this.robotStatusReconnectMs = Math.min(5000, Math.round(this.robotStatusReconnectMs * 1.6));
    this.robotStatusReconnectTimer = window.setTimeout(() => {
      this.robotStatusReconnectTimer = null;
      if (this.robotStatusStreamShouldRun && !this.isFleetManager()) {
        this.openRobotStatusStream();
      }
    }, delay);
  }

  handleRobotStatusStreamMessage(event) {
    if (this.isFleetManager()) {
      return;
    }
    let payload = null;
    try {
      payload = JSON.parse(event.data);
    } catch (_) {
      return;
    }
    if (!payload || payload.ok === false) {
      const message = payload && payload.error ? payload.error : "unknown websocket error";
      this.robotMessageText.textContent = `Robot stream error: ${message}`;
      return;
    }
    const state = payload.state && typeof payload.state === "object" ? payload.state : payload;
    if (!state || state.ok === false) {
      return;
    }
    this.currentStatus = state;
    if (state.route) {
      this.currentRoute = state.route;
    }
    this.renderRobotRuntimeTick();
  }

  handleFleetStatusStreamMessage(event) {
    if (!this.isFleetManager()) {
      return;
    }
    let payload = null;
    try {
      payload = JSON.parse(event.data);
    } catch (_) {
      return;
    }
    if (!payload || payload.ok === false) {
      const message = payload && payload.error ? payload.error : "unknown websocket error";
      this.robotMessageText.textContent = `Fleet stream error: ${message}`;
      return;
    }

    const state = payload.state && typeof payload.state === "object" ? payload.state : payload;
    if (!state || state.ok === false) {
      return;
    }
    this.fleetStatusReceivedAt = performance.now();
    if (payload.type === "state") {
      this.currentStatus = state;
      this.renderSelectedRobot();
      this.ensureFleetAnimationLoop();
      return;
    }
    this.currentStatus = this.mergeFleetTickState(state);
    this.renderFleetRuntimeTick();
  }

  ensureFleetAnimationLoop() {
    if (typeof window.requestAnimationFrame !== "function" || this.fleetAnimationFrame) {
      return;
    }
    const animate = (now) => {
      this.fleetAnimationFrame = null;
      if (!this.isFleetManager()) {
        return;
      }
      const shouldContinue = this.fleetNeedsAnimation();
      if (now - this.fleetAnimationLastAt >= 33) {
        this.fleetAnimationLastAt = now;
        this.drawFleetAnimationFrame(now);
      }
      if (!shouldContinue) {
        return;
      }
      this.fleetAnimationFrame = window.requestAnimationFrame(animate);
    };
    this.fleetAnimationFrame = window.requestAnimationFrame(animate);
  }

  stopFleetAnimationLoop() {
    if (!this.fleetAnimationFrame || typeof window.cancelAnimationFrame !== "function") {
      this.fleetAnimationFrame = null;
      return;
    }
    window.cancelAnimationFrame(this.fleetAnimationFrame);
    this.fleetAnimationFrame = null;
  }

  drawFleetAnimationFrame(now = performance.now()) {
    if (!this.isFleetManager() || !this.activeOperatorMapPayload()) {
      return;
    }
    this.drawRobot();
    if (now - this.fleetRouteRenderLastAt >= 180) {
      this.fleetRouteRenderLastAt = now;
      this.drawRoute();
      this.drawLookahead();
      this.syncMapControls();
    }
  }

  fleetNeedsAnimation() {
    if (!this.isFleetManager()) {
      return false;
    }
    if (this.fleetRuntimeMode() === "robots") {
      return false;
    }
    if (this.manualKeys.size && this.fleetManualAnimation) {
      return true;
    }
    const robots = Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : [];
    return robots.some((robot) => this.shouldAnimateFleetRobot(robot) && ["MOVING", "MANUAL"].includes(String(robot.status || "")));
  }

  fleetRenderRobots() {
    const robots = Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : [];
    if (!this.isFleetManager()) {
      return robots;
    }
    if (this.currentStatus && this.currentStatus !== this.fleetStatusObjectRef) {
      this.fleetStatusObjectRef = this.currentStatus;
      if (!this.fleetStatusReceivedAt || performance.now() - this.fleetStatusReceivedAt > this.fleetStreamIntervalMs * 2) {
        this.fleetStatusReceivedAt = performance.now();
      }
    }
    return robots.map((robot) => this.fleetRenderRobot(robot));
  }

  fleetRenderRobot(robot) {
    const animate = this.shouldAnimateFleetRobot(robot);
    const routeClock = animate
      ? this.animatedFleetRouteClock(robot)
      : Math.max(0, Number(robot?.routeClock || 0));
    const pose = animate ? this.animatedFleetRobotPose(robot, routeClock) : (robot?.pose || null);
    return {
      ...robot,
      routeClock,
      pose: pose || robot.pose || null,
    };
  }

  animatedFleetRouteClock(robot) {
    const baseClock = Math.max(0, Number(robot?.routeClock || 0));
    const trajectory = Array.isArray(robot?.trajectory) ? robot.trajectory : [];
    if (String(robot?.status || "") !== "MOVING" || trajectory.length < 2 || !this.fleetStatusReceivedAt) {
      return baseClock;
    }
    const last = trajectory[trajectory.length - 1];
    const finalTime = Math.max(0, Number(last.t ?? trajectory.length - 1));
    const elapsed = Math.max(0, (performance.now() - this.fleetStatusReceivedAt) / 1000);
    const maxPrediction = Math.max(0.12, (this.fleetStreamIntervalMs / 1000) * 1.8);
    return Math.min(finalTime, baseClock + Math.min(elapsed, maxPrediction));
  }

  animatedFleetRobotPose(robot, routeClock) {
    const trajectory = Array.isArray(robot?.trajectory) ? robot.trajectory : [];
    if (String(robot?.status || "") === "MOVING" && trajectory.length >= 2) {
      return this.interpolateTrajectory(trajectory, routeClock);
    }
    const manualPose = this.animatedFleetManualPose(robot);
    if (manualPose) {
      return manualPose;
    }
    return robot?.pose || null;
  }

  animatedFleetManualPose(robot) {
    const animation = this.fleetManualAnimation;
    if (!animation || !robot || robot.name !== animation.robotName) {
      return null;
    }
    if (String(robot.status || "") !== "MANUAL" && !this.manualKeys.size) {
      return null;
    }
    const elapsed = Math.min(0.28, Math.max(0, (performance.now() - animation.startedAt) / 1000));
    return this.integratePose(animation.pose, animation.linear, animation.angular, elapsed);
  }

  setFleetManualAnimation(robotName, pose, twist) {
    this.fleetManualAnimation = {
      robotName,
      pose: { ...pose },
      linear: Number(twist.linear || 0),
      angular: Number(twist.angular || 0),
      startedAt: performance.now(),
    };
    this.ensureFleetAnimationLoop();
  }

  async refreshRobots(options = {}) {
    const result = await this.getJson(options.quiet ? "/api/robots?probe=0" : "/api/robots");
    const nextRobots = Array.isArray(result.robots) ? result.robots : [];
    this.robots = options.quiet ? this.mergeQuietRobotPayloads(nextRobots) : nextRobots;
    if (this.selectedRobotId && !this.selectedRobot()) {
      this.selectedRobotId = "";
      window.localStorage.removeItem("operator:selectedRobotId");
    }
    if (!this.selectedRobotId && this.robots.length) {
      this.selectedRobotId = this.robots[0].id;
      window.localStorage.setItem("operator:selectedRobotId", this.selectedRobotId);
    }
    if (this.fleetActiveTab === "model") {
      this.ensureRobotSelectedForModel();
    } else if (this.fleetActiveTab === "map") {
      this.ensureFleetManagerSelected();
    }
    if (!options.quiet) {
      this.initialFleetRouteSelectionPending = false;
    }
    this.syncFleetStatusStream();
    if (options.lightweight) {
      this.renderRobotList();
      return;
    }
    await this.refreshRobotMapState({ quiet: true });
    await this.fetchSelectedRobotStatus(true);
    if (this.isRobotModelPage()) {
      await this.ensureRobotParamsLoaded();
    }
    if (this.isParamsPage()) {
      await this.ensureCurrentParamsLoaded();
    }
    this.render();
    await this.maybePromptPendingPush();
    if (!options.quiet) {
      this.showProbeResult("neutral", "Robot list refreshed.");
    }
  }

  mergeQuietRobotPayloads(nextRobots) {
    const previousById = new Map(this.robots.map((robot) => [robot.id, robot]));
    return nextRobots.map((robot) => {
      const previous = previousById.get(robot.id);
      if (!previous) {
        return robot;
      }
      if (this.isFleetManager(robot) && robot.runtimeFresh === false) {
        return {
          ...previous,
          ...robot,
          status: previous.status,
        };
      }
      if (robot.probed !== false || this.isFleetManager(robot)) {
        return robot;
      }
      return {
        ...previous,
        ...robot,
        online: previous.online,
        status: previous.status,
        error: previous.error,
      };
    });
  }

  async refreshRobotMapState(options = {}) {
    const robot = this.selectedRobot();
    if (!robot) {
      this.robotMapState = this.emptyMapState();
      this.operatorMapPayload = null;
      this.operatorMapSignature = "";
      return;
    }
    if (this.isFleetManager(robot)) {
      try {
        const robotActive = await this.getJson("/api/fleet-manager/maps/active");
        let localActive = await this.getJson("/api/fleet-manager/maps/local/active");
        const nextSignature = String(localActive.signature || "").trim();
        if (nextSignature && nextSignature !== this.operatorMapSignature) {
          this.resetMapView(true);
        }
        this.operatorMapPayload = localActive.map && typeof localActive.map === "object" ? localActive.map : null;
        this.operatorMapSignature = nextSignature;
        const robotActiveName = String(robotActive.mapName || localActive.robotMapName || "").trim();
        const localActiveName = String(localActive.activeMapName || "").trim();
        this.robotMapState = {
          robotActiveMapName: robotActiveName,
          operatorActiveMapName: localActiveName,
          robotSignature: String(robotActive.signature || localActive.robotSignature || "").trim(),
          operatorSignature: nextSignature,
          sourceRobotMapName: String(localActive.robotMapName || localActive.sourceMapName || robotActiveName).trim(),
          hasLocalChanges: Boolean(
            localActiveName &&
            (
              Boolean(localActive.hasLocalChanges) ||
              (nextSignature &&
               String(robotActive.signature || localActive.robotSignature || "").trim() &&
               nextSignature !== String(robotActive.signature || localActive.robotSignature || "").trim()) ||
              (localActiveName && robotActiveName && localActiveName !== robotActiveName)
            )
          ),
        };
      } catch (error) {
        this.robotMapState = this.emptyMapState();
        this.operatorMapPayload = null;
        this.operatorMapSignature = "";
        if (!options.quiet) {
          window.alert(error.message || String(error));
        }
      }
      return;
    }
    try {
      const [robotActive, localActive] = await Promise.all([
        this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/active`),
        this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/local/active`),
      ]);
      const nextSignature = String(localActive.signature || "").trim();
      if (nextSignature && nextSignature !== this.operatorMapSignature) {
        this.resetMapView(true);
      }
      this.operatorMapPayload = localActive.map && typeof localActive.map === "object" ? localActive.map : null;
      this.operatorMapSignature = nextSignature;
      this.robotMapState = {
        robotActiveMapName: String(robotActive.mapName || "").trim(),
        operatorActiveMapName: String(localActive.activeMapName || "").trim(),
        robotSignature: String(robotActive.signature || localActive.robotSignature || "").trim(),
        operatorSignature: nextSignature,
        sourceRobotMapName: String(localActive.robotMapName || localActive.sourceMapName || "").trim(),
        hasLocalChanges: Boolean(
          localActive.activeMapName &&
          (
            Boolean(localActive.hasLocalChanges) ||
            (nextSignature &&
             String(robotActive.signature || localActive.robotSignature || "").trim() &&
             nextSignature !== String(robotActive.signature || localActive.robotSignature || "").trim()) ||
            (String(localActive.activeMapName || "").trim() &&
             String(robotActive.mapName || "").trim() &&
             String(localActive.activeMapName || "").trim() !== String(robotActive.mapName || "").trim())
          )
        ),
      };
    } catch (error) {
      this.robotMapState = this.emptyMapState();
      this.operatorMapPayload = null;
      this.operatorMapSignature = "";
      if (!options.quiet) {
        window.alert(error.message || String(error));
      }
    }
  }

  async fetchSelectedRobotStatus(silent = false) {
    const robot = this.selectedRobot();
    if (!robot || this.statusRequestPending) {
      return;
    }
    this.statusRequestPending = true;
    try {
      if (this.isFleetManager(robot)) {
        await this.ensureFleetParamsLoaded();
        this.syncFleetStatusStream();
        if (silent && (this.fleetStatusStreamOpen() || this.fleetStatusStreamConnectingFresh())) {
          return;
        }
      } else {
        this.syncFleetStatusStream();
        if (silent && (this.robotStatusStreamOpen() || this.robotStatusStreamConnectingFresh())) {
          return;
        }
      }
      const result = this.isFleetManager(robot)
        ? await this.getJson("/api/fleet-manager/state")
        : await this.getJson(this.robotApiPath("/api/robot/status"));
      if (this.isFleetManager(robot)) {
        this.fleetStatusReceivedAt = performance.now();
      }
      this.currentStatus = result;
      if (result && result.route) {
        this.currentRoute = result.route;
      }
      this.renderSelectedRobot();
    } catch (error) {
      if (!silent) {
        window.alert(error.message || String(error));
      }
      this.currentStatus = {
        robot: {
          connected: false,
          state: "OFFLINE",
          message: error.message || String(error),
        },
        events: [],
        route: null,
      };
      this.renderSelectedRobot();
    } finally {
      this.statusRequestPending = false;
    }
  }

  async tickFleetIfSelected() {
    if (!this.selectedRobot() || !this.isFleetManager() || this.fleetTickPending || this.manualKeys.size) {
      if (!this.isFleetManager()) {
        this.closeFleetStatusStream();
      }
      return;
    }
    this.syncFleetStatusStream();
    if (this.fleetStatusStreamOpen() || this.fleetStatusStreamConnectingFresh()) {
      return;
    }
    const now = performance.now();
    if (now - this.fleetHttpFallbackLastAt < 250) {
      return;
    }
    this.fleetHttpFallbackLastAt = now;
    this.fleetTickPending = true;
    try {
      const result = await this.postJson("/api/fleet-manager/tick", {});
      this.fleetStatusReceivedAt = performance.now();
      this.currentStatus = this.mergeFleetTickState(result);
      this.renderFleetRuntimeTick();
    } finally {
      this.fleetTickPending = false;
    }
  }

  mergeFleetTickState(tickState) {
    const previous = this.currentStatus || {};
    const previousRobots = new Map((Array.isArray(previous.robots) ? previous.robots : []).map((robot) => [robot.name, robot]));
    const nextRobots = (Array.isArray(tickState.robots) ? tickState.robots : []).map((robot) => {
      const prior = previousRobots.get(robot.name) || {};
      const incomingTrajectory = Array.isArray(robot.trajectory) ? robot.trajectory : [];
      const incomingPlanNodes = Array.isArray(robot.planNodes) ? robot.planNodes : [];
      const status = String(robot.status || prior.status || "");
      const hasTarget = Boolean(robot.targetLm || prior.targetLm || robot.targetName || prior.targetName);
      const canReuseRoute = hasTarget && ["MOVING", "WAITING", "BLOCKED", "PLANNING"].includes(status);
      return {
        ...prior,
        ...robot,
        trajectory: incomingTrajectory.length
          ? incomingTrajectory
          : (canReuseRoute && Array.isArray(prior.trajectory) ? prior.trajectory : []),
        planNodes: incomingPlanNodes.length
          ? incomingPlanNodes
          : (canReuseRoute && Array.isArray(prior.planNodes) ? prior.planNodes : []),
      };
    });
    return {
      ...previous,
      ...tickState,
      robots: nextRobots,
      events: Array.isArray(tickState.events) && tickState.events.length
        ? tickState.events
        : (Array.isArray(previous.events) ? previous.events : []),
    };
  }

  render() {
    const savedCount = this.robots.filter((robot) => !this.isFleetManager(robot) && !robot.system).length;
    this.robotCountText.textContent = `${savedCount} saved`;
    this.renderRobotList();
    this.renderSelectedRobot();
  }

  renderRobotList() {
    this.robotsList.innerHTML = "";
    if (!this.robots.length) {
      const empty = document.createElement("div");
      empty.className = "probe-result neutral";
      empty.textContent = "No robots added yet. Use Add Robot + to connect by IP.";
      this.robotsList.append(empty);
      return;
    }

    for (const robot of this.robots) {
      const isFleet = this.isFleetManager(robot);
      const isRos2 = this.isRos2Robot(robot);
      const button = document.createElement("div");
      button.className = "robot-card";
      button.tabIndex = 0;
      button.setAttribute("role", "button");
      if (robot.id === this.selectedRobotId) {
        button.classList.add("active");
      }
      const selectRobot = async () => {
        this.selectedRobotId = robot.id;
        window.localStorage.setItem("operator:selectedRobotId", robot.id);
        this.currentStatus = null;
        this.currentRoute = null;
        this.syncFleetStatusStream();
        this.closeSidebar();
        if (this.isRobotModelPage() && this.isFleetManager(robot)) {
          await this.navigateFleetPage("fleet", { replace: true });
          return;
        }
        if (this.fleetActiveTab === "map" && !this.isFleetManager(robot)) {
          await this.navigateHomePage({ replace: true });
          return;
        }
        await this.refreshRobotMapState({ quiet: true });
        await this.fetchSelectedRobotStatus(true);
        if (this.isRobotModelPage() && !this.isFleetManager(robot)) {
          await this.ensureRobotParamsLoaded(true);
        }
        if (this.isParamsPage()) {
          await this.ensureCurrentParamsLoaded(true);
        }
        this.render();
      };
      button.addEventListener("click", selectRobot);
      button.addEventListener("keydown", async (event) => {
        if (event.key !== "Enter" && event.key !== " ") {
          return;
        }
        event.preventDefault();
        await selectRobot();
      });

      const identity = robot.identity || robot.lastIdentity || {};
      const status = robot.status || {};
      const chipClass = robot.online ? "robot-chip online" : "robot-chip offline";
      const systemRobot = isFleet || robot.system;
      const chipText = systemRobot ? "system" : (robot.online ? "online" : "offline");
      const connectionLabel = isFleet
        ? "local fleet controller"
        : (isRos2
          ? `ROS2 ${this.escapeHtml(robot.host || "DDS")} domain ${this.escapeHtml(String(robot.domainId ?? 0))}`
          : `${this.escapeHtml(robot.host)}:${this.escapeHtml(String(robot.port))}`);
      button.innerHTML = `
        <div class="robot-card-header">
          <div>
            <strong>${this.escapeHtml(robot.name || identity.robotId || robot.id)}</strong>
            <p>${connectionLabel}</p>
          </div>
          <div class="robot-card-actions">
            <span class="${chipClass}">${chipText}</span>
            ${systemRobot ? "" : '<button class="robot-card-remove" type="button" aria-label="Remove robot">Delete</button>'}
          </div>
        </div>
        <div class="robot-card-meta">
          <div>Robot ID: ${this.escapeHtml(identity.robotId || "-")}</div>
          <div>Map: ${this.escapeHtml(identity.mapId || "-")}</div>
          <div>State: ${this.escapeHtml(status.state || status.stateText || "-")}</div>
          ${isFleet ? `<div>Fleet robots: ${this.escapeHtml(String(status.robots || 0))}</div>` : ""}
        </div>
      `;
      const removeButton = button.querySelector(".robot-card-remove");
      if (removeButton) {
        removeButton.addEventListener("click", async (event) => {
          event.stopPropagation();
          await this.handleRemoveRobot(robot);
        });
      }
      this.robotsList.append(button);
    }
  }

  renderSelectedRobot() {
    const robot = this.selectedRobot();
    this.renderSidebar();
    if (!robot) {
      this.emptyState.classList.remove("hidden");
      this.robotView.classList.add("hidden");
      this.fleetControlPanel.classList.add("hidden");
      this.robotParamsPanel.classList.add("hidden");
      this.robotModelPanel.classList.add("hidden");
      this.robotActiveMapText.textContent = "-";
      this.operatorActiveMapText.textContent = "-";
      this.robotStateText.textContent = "-";
      this.nearestLmText.textContent = "-";
      this.mapSyncStatus.className = "probe-result neutral";
      this.mapSyncStatus.textContent = "Select a robot to see map sync state.";
      return;
    }

    this.emptyState.classList.add("hidden");
    this.robotView.classList.remove("hidden");
    this.robotActiveMapText.textContent = this.robotMapState.robotActiveMapName || "-";
    this.operatorActiveMapText.textContent = this.robotMapState.operatorActiveMapName || "-";
    this.renderMapSyncStatus();
    this.renderRobotConsole();
  }

  setText(element, value) {
    if (!element) {
      return;
    }
    const text = value === null || value === undefined || value === "" ? "-" : String(value);
    element.textContent = text;
  }

  renderInspectorDetails(details) {
    this.setText(this.inspectorRobotText, details.robot);
    this.setText(this.inspectorModeText, details.mode);
    this.setText(this.connectionText, details.connection);
    this.setText(this.inspectorMapText, details.map);
    this.setText(this.inspectorCurrentLmText, details.currentLm);
    this.setText(this.localizationText, details.localization);
    this.setText(this.targetLmText, details.targetLm);
    this.setText(this.currentEdgeText, details.currentEdge);
    this.setText(this.routeProgressText, details.progress);
    this.setText(this.inspectorBatteryText, details.battery);
    this.setText(this.inspectorConfidenceText, details.confidence);
    this.setText(this.poseText, details.pose);
    this.setText(this.velocityText, details.velocity);
    this.setText(this.inspectorApiText, details.api);
    this.setText(this.inspectorReasonText, details.reason);
  }

  formatPose(pose) {
    return pose
      ? `x: ${Number(pose.x).toFixed(3)}, y: ${Number(pose.y).toFixed(3)}, yaw: ${Number(pose.yaw || 0).toFixed(3)}`
      : "x: -, y: -, yaw: -";
  }

  formatVelocity(velocity) {
    return velocity
      ? `v: ${Number(velocity.linear || 0).toFixed(3)}, w: ${Number(velocity.angular || 0).toFixed(3)}`
      : "v: -, w: -";
  }

  formatProgress(value, fallback = "-") {
    if (value === null || value === undefined || value === "") {
      return fallback;
    }
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) {
      return String(value);
    }
    const ratio = numeric <= 1 ? numeric : numeric / 100;
    return `${Math.round(Math.max(0, Math.min(1, ratio)) * 100)}%`;
  }

  nestedStatusValue(payload, keys) {
    if (!payload || typeof payload !== "object") {
      return undefined;
    }
    for (const key of keys) {
      if (payload[key] !== undefined && payload[key] !== null && payload[key] !== "") {
        return payload[key];
      }
    }
    const rbk = payload.rbk_report;
    if (rbk && typeof rbk === "object") {
      for (const key of keys) {
        if (rbk[key] !== undefined && rbk[key] !== null && rbk[key] !== "") {
          return rbk[key];
        }
      }
    }
    return undefined;
  }

  formatPercentMetric(value) {
    if (value === undefined || value === null || value === "") {
      return "";
    }
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) {
      return String(value);
    }
    if (numeric >= 0 && numeric <= 1) {
      return `${Math.round(numeric * 100)}%`;
    }
    if (numeric >= 0 && numeric <= 100) {
      return `${Number.isInteger(numeric) ? numeric : numeric.toFixed(1)}%`;
    }
    return String(numeric);
  }

  formatBattery(payload) {
    const rawBattery = this.nestedStatusValue(payload, ["batteryLevel", "battery_level", "battery", "power"]);
    const battery = rawBattery && typeof rawBattery === "object" && !Array.isArray(rawBattery) ? rawBattery : {};
    const value = Object.keys(battery).length
      ? (battery.level ?? battery.percent ?? battery.percentage ?? battery.value)
      : rawBattery;
    const charging = Object.keys(battery).length
      ? (battery.charging ?? battery.isCharging)
      : this.nestedStatusValue(payload, ["charging", "isCharging"]);
    const chargingText = charging === undefined ? "" : (charging ? " charging" : " not charging");
    const parts = [];
    const metric = this.formatPercentMetric(value);
    if (metric) {
      parts.push(metric);
    }
    const voltage = Number(battery.voltage);
    if (Number.isFinite(voltage) && voltage > 0) {
      parts.push(`${voltage.toFixed(1)} V`);
    }
    const current = Number(battery.current);
    if (Number.isFinite(current) && Math.abs(current) > 0.001) {
      parts.push(`${current.toFixed(1)} A`);
    }
    const temperature = Number(battery.temperature ?? battery.temp);
    if (Number.isFinite(temperature) && Math.abs(temperature) > 0.001) {
      parts.push(`${temperature.toFixed(1)} C`);
    }
    if (chargingText) {
      parts.push(chargingText.trim());
    }
    return parts.length ? parts.join(" | ") : "-";
  }

  formatConfidence(payload) {
    const value = this.nestedStatusValue(payload, ["confidence", "localizationConfidence"]);
    return this.formatPercentMetric(value) || "-";
  }

  remoteStatusForFleetRobot(robot) {
    const status = robot?.remoteStatus || robot?.statusPayload || {};
    return status && typeof status === "object" ? status : {};
  }

  fleetRobotProgress(robot, remoteStatus) {
    const remoteProgress = this.nestedStatusValue(remoteStatus, ["routeProgress", "progress"]);
    if (remoteProgress !== undefined) {
      return this.formatProgress(remoteProgress);
    }
    const trajectory = Array.isArray(robot?.trajectory) ? robot.trajectory : [];
    if (trajectory.length && Number(robot?.routeClock || 0) > 0) {
      const finalTime = Number(trajectory[trajectory.length - 1]?.t || 0);
      if (finalTime > 0) {
        return this.formatProgress(Number(robot.routeClock || 0) / finalTime);
      }
    }
    return String(robot?.status || "-");
  }

  fleetRobotMapLabel(robot, remoteStatus) {
    return this.nestedStatusValue(remoteStatus, ["mapId", "currentMap", "current_map", "map"])
      || robot?.mapId
      || this.robotMapState.robotActiveMapName
      || "-";
  }

  fleetRobotConnectionText(robot) {
    if (!robot) {
      return this.fleetStatusStreamOpen() ? "local websocket" : "local http";
    }
    if (!this.isFleetRemoteRobot(robot)) {
      return "simulated";
    }
    if (robot.remoteError) {
      return `offline: ${robot.remoteError}`;
    }
    return robot.online === false ? "offline" : "online";
  }

  isRobotModelPage() {
    return this.fleetActiveTab === "model";
  }

  isParamsPage() {
    return this.fleetActiveTab === "params";
  }

  async ensureCurrentParamsLoaded(force = false) {
    if (this.isFleetManager()) {
      await this.ensureFleetParamsLoaded(force);
      return;
    }
    await this.ensureRobotParamsLoaded(force);
  }

  renderMapSyncStatus() {
    const selected = this.selectedRobot();
    const isFleet = this.isFleetManager(selected);
    const isRobotModel = this.isRobotModelPage();
    const isParams = this.isParamsPage();
    const showRobotParams = !isFleet && isParams;
    this.fleetControlPanel.classList.toggle("hidden", !isFleet);
    this.robotParamsPanel.classList.toggle("hidden", !showRobotParams);
    this.robotModelPanel.classList.toggle("hidden", !isRobotModel);
    this.manualPad.classList.toggle("hidden", isRobotModel || isParams);
    this.controlPullMapButton.classList.toggle("hidden", false);
    this.controlPushMapButton.classList.toggle("hidden", false);
    this.cancelRouteButton.textContent = isFleet ? "Stop Active" : "Cancel Route";
    this.stopRobotButton.textContent = isFleet ? "Stop Fleet" : "Stop";
    this.navigateRobotButton.textContent = this.navigateMode ? "Cancel Navigate" : this.navigateButtonIdleText();
    if (isFleet) {
      this.setFleetTab(this.fleetActiveTab);
    } else {
      this.syncFleetPageClass(false);
      this.operatorMapSvg.classList.remove("fleet-map-editor-active");
      if (showRobotParams) {
        this.syncRobotParamsJson();
      }
    }
    const hasLocal = Boolean(this.robotMapState.operatorActiveMapName);
    const hasChanges = Boolean(this.robotMapState.hasLocalChanges);
    if (!hasLocal) {
      this.mapSyncStatus.className = "probe-result neutral";
      this.mapSyncStatus.textContent = `${isFleet ? "Operator has no local Fleet Manager map yet" : "Operator has no local active map yet"}. Use Pull Map first.`;
      this.controlPushMapButton.classList.remove("primary");
      return;
    }
    if (hasChanges) {
      const source = this.robotMapState.sourceRobotMapName || this.robotMapState.robotActiveMapName || "-";
      this.mapSyncStatus.className = "probe-result warning";
      this.mapSyncStatus.textContent = `Local map differs from ${isFleet ? "Fleet Manager" : "robot"} map ${source}. Use Push Map to apply local changes.`;
      this.controlPushMapButton.classList.add("primary");
      return;
    }
    this.mapSyncStatus.className = "probe-result success";
    this.mapSyncStatus.textContent = `Operator local map matches the current ${isFleet ? "Fleet Manager" : "robot"} map.`;
    this.controlPushMapButton.classList.remove("primary");
  }

  renderRobotConsole() {
    if (this.isFleetManager()) {
      this.renderFleetConsole();
      return;
    }
    const selected = this.selectedRobot();
    const status = this.currentStatus || {};
    const robot = status.robot || {};
    const route = status.route || this.currentRoute || null;
    const pose = robot.pose || null;
    const connected = Boolean(robot.connected);
    const state = String(robot.state || (selected && selected.online ? "ONLINE" : "OFFLINE") || "-");

    this.robotStateText.textContent = state;
    this.nearestLmText.textContent = robot.nearestLm || "-";
    this.renderInspectorDetails({
      robot: robot.robotId || selected?.name || selected?.id || "-",
      mode: "robot",
      connection: this.robotStatusStreamOpen() ? "robot websocket" : (connected ? "online" : "offline"),
      map: robot.mapId || selected?.identity?.mapId || this.robotMapState.robotActiveMapName || "-",
      currentLm: robot.nearestLm || "-",
      localization: robot.localizationOk ? `ok (${Number(robot.localizationAgeSec || 0).toFixed(2)} s)` : "waiting",
      targetLm: robot.targetLm || "-",
      currentEdge: robot.currentEdgeId || "-",
      progress: this.formatProgress(robot.routeProgress, "0%"),
      battery: this.formatBattery(robot),
      confidence: this.formatConfidence(robot),
      pose: this.formatPose(pose),
      velocity: this.formatVelocity(robot.velocity),
      api: this.isRos2Robot(selected)
        ? `ROS2 ${selected?.host || "DDS"} domain ${selected?.domainId ?? 0}`
        : (selected?.baseUrl || "-"),
      reason: robot.message || "-",
    });
    this.robotMessageText.textContent = robot.message || (this.operatorMapPayload ? "Robot status ready." : "Pull the active robot map to display Map & Control.");
    this.routeNodesText.textContent = route && Array.isArray(route.nodes) && route.nodes.length
      ? route.nodes.join(" -> ")
      : "No route planned.";
    this.renderEvents(Array.isArray(status.events) ? status.events : []);
    this.syncModeButtons();
    this.syncManualButtons();
    this.renderOperatorMap();
  }

  renderRobotRuntimeTick() {
    if (this.isFleetManager()) {
      return;
    }
    const selected = this.selectedRobot();
    const status = this.currentStatus || {};
    const robot = status.robot || {};
    const route = status.route || this.currentRoute || null;
    const pose = robot.pose || null;
    const connected = Boolean(robot.connected);
    const state = String(robot.state || (selected && selected.online ? "ONLINE" : "OFFLINE") || "-");

    this.robotStateText.textContent = state;
    this.nearestLmText.textContent = robot.nearestLm || "-";
    this.renderInspectorDetails({
      robot: robot.robotId || selected?.name || selected?.id || "-",
      mode: "robot",
      connection: this.robotStatusStreamOpen() ? "robot websocket" : (connected ? "online" : "offline"),
      map: robot.mapId || selected?.identity?.mapId || this.robotMapState.robotActiveMapName || "-",
      currentLm: robot.nearestLm || "-",
      localization: robot.localizationOk ? `ok (${Number(robot.localizationAgeSec || 0).toFixed(2)} s)` : "waiting",
      targetLm: robot.targetLm || "-",
      currentEdge: robot.currentEdgeId || "-",
      progress: this.formatProgress(robot.routeProgress, "0%"),
      battery: this.formatBattery(robot),
      confidence: this.formatConfidence(robot),
      pose: this.formatPose(pose),
      velocity: this.formatVelocity(robot.velocity),
      api: this.isRos2Robot(selected)
        ? `ROS2 ${selected?.host || "DDS"} domain ${selected?.domainId ?? 0}`
        : (selected?.baseUrl || "-"),
      reason: robot.message || "-",
    });
    this.robotMessageText.textContent = robot.message || (this.operatorMapPayload ? "Robot status ready." : "Pull the active robot map to display Map & Control.");
    this.routeNodesText.textContent = route && Array.isArray(route.nodes) && route.nodes.length
      ? route.nodes.join(" -> ")
      : "No route planned.";
    this.syncModeButtons();
    this.syncManualButtons();
    this.drawRoute();
    this.drawRobot();
    this.syncMapControls();
  }

  renderFleetConsole() {
    const status = this.currentStatus || {};
    const robots = this.fleetRenderRobots();
    const selectedFleetRobot = this.selectedFleetRobot(robots);
    const mode = this.fleetRuntimeMode(status);
    const remoteStatus = this.remoteStatusForFleetRobot(selectedFleetRobot);
    const robotMode = selectedFleetRobot
      ? String(selectedFleetRobot.mode || selectedFleetRobot.type || "simulated")
      : mode;
    const routeMeta = selectedFleetRobot
      ? [
          selectedFleetRobot.baseUrl || (this.isFleetRemoteRobot(selectedFleetRobot) ? "remote API" : "simulation"),
          selectedFleetRobot.routeRevision ? `rev ${selectedFleetRobot.routeRevision}` : "",
          selectedFleetRobot.routeChunkGoalLm ? `chunk ${selectedFleetRobot.routeChunkIndex || 0} -> ${selectedFleetRobot.routeChunkGoalLm}` : "",
        ].filter(Boolean).join(" | ")
      : "local Fleet Manager";
    const localization = selectedFleetRobot
      ? (
          remoteStatus.localizationOk !== undefined
            ? (remoteStatus.localizationOk ? `ok (${Number(remoteStatus.localizationAgeSec || 0).toFixed(2)} s)` : "waiting")
            : (selectedFleetRobot.pose ? "pose available" : "waiting")
        )
      : mode;

    this.fleetModeSelect.value = mode;
    this.robotStateText.textContent = selectedFleetRobot ? String(selectedFleetRobot.status || "-") : mode.toUpperCase();
    this.nearestLmText.textContent = selectedFleetRobot ? (selectedFleetRobot.currentLm || "-") : `${robots.length} robots`;
    this.renderInspectorDetails({
      robot: selectedFleetRobot ? selectedFleetRobot.name : "Fleet Manager",
      mode: selectedFleetRobot ? `${mode} / ${robotMode}` : mode,
      connection: this.fleetRobotConnectionText(selectedFleetRobot),
      map: this.fleetRobotMapLabel(selectedFleetRobot, remoteStatus),
      currentLm: selectedFleetRobot ? (selectedFleetRobot.currentLm || "-") : "-",
      localization,
      targetLm: selectedFleetRobot ? (selectedFleetRobot.targetLm || "-") : "-",
      currentEdge: this.nestedStatusValue(remoteStatus, ["currentEdgeId", "current_edge_id"]) || (selectedFleetRobot?.routeChunkGoalLm ? `chunk -> ${selectedFleetRobot.routeChunkGoalLm}` : "-"),
      progress: selectedFleetRobot ? this.fleetRobotProgress(selectedFleetRobot, remoteStatus) : "-",
      battery: this.formatBattery(remoteStatus),
      confidence: this.formatConfidence(remoteStatus),
      pose: this.formatPose(selectedFleetRobot?.pose),
      velocity: this.formatVelocity(remoteStatus.velocity),
      api: routeMeta,
      reason: selectedFleetRobot
        ? (selectedFleetRobot.remoteError || remoteStatus.message || selectedFleetRobot.reason || selectedFleetRobot.routeNote || "-")
        : `mode: ${mode}`,
    });
    this.robotMessageText.textContent = robots.length
      ? `Fleet Manager is supervising ${robots.length} robot(s).`
      : (mode === "robots" ? "Add a robot IP. LM is read from robot status." : "Add a simulation robot from a start LM.");
    this.routeNodesText.textContent = selectedFleetRobot && Array.isArray(selectedFleetRobot.planNodes) && selectedFleetRobot.planNodes.length
      ? selectedFleetRobot.planNodes.join(" -> ")
      : "No active fleet route.";

    this.renderFleetControls(robots);
    this.renderFleetQueue();
    this.renderFleetPlanDebug();
    this.renderEvents(Array.isArray(status.events) ? status.events : []);
    this.syncModeButtons();
    this.syncManualButtons();
    this.renderOperatorMap();
    this.ensureFleetAnimationLoop();
  }

  renderFleetRuntimeTick() {
    if (!this.isFleetManager()) {
      this.renderSelectedRobot();
      return;
    }
    const status = this.currentStatus || {};
    const robots = this.fleetRenderRobots();
    const selectedFleetRobot = this.selectedFleetRobot(robots);
    const mode = this.fleetRuntimeMode(status);
    const remoteStatus = this.remoteStatusForFleetRobot(selectedFleetRobot);
    const robotMode = selectedFleetRobot
      ? String(selectedFleetRobot.mode || selectedFleetRobot.type || "simulated")
      : mode;
    const routeMeta = selectedFleetRobot
      ? [
          selectedFleetRobot.baseUrl || (this.isFleetRemoteRobot(selectedFleetRobot) ? "remote API" : "simulation"),
          selectedFleetRobot.routeRevision ? `rev ${selectedFleetRobot.routeRevision}` : "",
          selectedFleetRobot.routeChunkGoalLm ? `chunk ${selectedFleetRobot.routeChunkIndex || 0} -> ${selectedFleetRobot.routeChunkGoalLm}` : "",
        ].filter(Boolean).join(" | ")
      : "local Fleet Manager";
    const localization = selectedFleetRobot
      ? (
          remoteStatus.localizationOk !== undefined
            ? (remoteStatus.localizationOk ? `ok (${Number(remoteStatus.localizationAgeSec || 0).toFixed(2)} s)` : "waiting")
            : (selectedFleetRobot.pose ? "pose available" : "waiting")
        )
      : mode;

    this.robotStateText.textContent = selectedFleetRobot ? String(selectedFleetRobot.status || "-") : mode.toUpperCase();
    this.nearestLmText.textContent = selectedFleetRobot ? (selectedFleetRobot.currentLm || "-") : `${robots.length} robots`;
    this.renderInspectorDetails({
      robot: selectedFleetRobot ? selectedFleetRobot.name : "Fleet Manager",
      mode: selectedFleetRobot ? `${mode} / ${robotMode}` : mode,
      connection: this.fleetRobotConnectionText(selectedFleetRobot),
      map: this.fleetRobotMapLabel(selectedFleetRobot, remoteStatus),
      currentLm: selectedFleetRobot ? (selectedFleetRobot.currentLm || "-") : "-",
      localization,
      targetLm: selectedFleetRobot ? (selectedFleetRobot.targetLm || "-") : "-",
      currentEdge: this.nestedStatusValue(remoteStatus, ["currentEdgeId", "current_edge_id"]) || (selectedFleetRobot?.routeChunkGoalLm ? `chunk -> ${selectedFleetRobot.routeChunkGoalLm}` : "-"),
      progress: selectedFleetRobot ? this.fleetRobotProgress(selectedFleetRobot, remoteStatus) : "-",
      battery: this.formatBattery(remoteStatus),
      confidence: this.formatConfidence(remoteStatus),
      pose: this.formatPose(selectedFleetRobot?.pose),
      velocity: this.formatVelocity(remoteStatus.velocity),
      api: routeMeta,
      reason: selectedFleetRobot
        ? (selectedFleetRobot.remoteError || remoteStatus.message || selectedFleetRobot.reason || selectedFleetRobot.routeNote || "-")
        : `mode: ${mode}`,
    });
    this.routeNodesText.textContent = selectedFleetRobot && Array.isArray(selectedFleetRobot.planNodes) && selectedFleetRobot.planNodes.length
      ? selectedFleetRobot.planNodes.join(" -> ")
      : "No active fleet route.";
    this.renderFleetRobotList(robots);
    this.renderFleetQueue();
    this.renderFleetPlanDebug();
    this.drawRoute();
    this.drawLookahead();
    this.syncModeButtons();
    this.syncManualButtons();
    this.ensureFleetAnimationLoop();
  }

  selectedFleetRobot(robots = null) {
    const items = robots || (Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []);
    if (!items.length) {
      return null;
    }
    const selected = items.find((robot) => robot.name === this.selectedFleetRobotName);
    return selected || items[0];
  }

  renderFleetControls(robots) {
    const lms = Array.isArray(this.operatorMapPayload?.lms) ? this.operatorMapPayload.lms : [];
    const previousSpawn = this.fleetSpawnLmSelect.value;
    const selectedFleetRobot = this.selectedFleetRobot(robots);

    this.syncFleetRemoteFields();
    this.fillFleetSpawnSelect(lms.map((lm) => lm.name), previousSpawn);
    if (selectedFleetRobot) {
      this.selectedFleetRobotName = selectedFleetRobot.name;
      window.localStorage.setItem("operator:selectedFleetRobotName", this.selectedFleetRobotName);
    }
    const isRemoteMode = String(this.fleetModeSelect?.value || "simulation") === "robots";
    if (!isRemoteMode && (!this.fleetNameEdited || this.robotNameExists(this.fleetRobotNameInput.value, robots))) {
      this.fleetRobotNameInput.value = this.nextFleetRobotName(robots);
      this.fleetNameEdited = false;
    }
    this.renderFleetRobotList(robots);
  }

  syncFleetRemoteFields() {
    const isRemoteMode = String(this.fleetModeSelect?.value || "simulation") === "robots";
    if (this.fleetRobotNameLabel) {
      this.fleetRobotNameLabel.classList.toggle("hidden", isRemoteMode);
    }
    if (this.fleetSpawnLmLabel) {
      this.fleetSpawnLmLabel.classList.toggle("hidden", isRemoteMode);
    }
    if (this.fleetRobotApiLabel) {
      this.fleetRobotApiLabel.classList.toggle("hidden", !isRemoteMode);
    }
    if (this.fleetRobotApiInput) {
      this.fleetRobotApiInput.placeholder = isRemoteMode ? "192.168.0.10" : "";
    }
    if (this.fleetSpawnLmLabelText) {
      this.fleetSpawnLmLabelText.textContent = "Start LM";
    }
  }

  robotNameExists(name, robots = null) {
    const value = String(name || "").trim();
    if (!value) {
      return false;
    }
    const items = robots || (Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []);
    return items.some((robot) => robot.name === value);
  }

  nextFleetRobotName(robots = null) {
    const items = robots || (Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []);
    let maxIndex = 0;
    for (const robot of items) {
      const match = String(robot.name || "").match(/^robot(\d+)$/i);
      if (match) {
        maxIndex = Math.max(maxIndex, Number(match[1] || 0));
      }
    }
    return `robot${maxIndex + 1}`;
  }

  fillSelect(select, values, selectedValue) {
    const current = String(selectedValue || "");
    select.innerHTML = "";
    for (const value of values) {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = value;
      option.selected = value === current;
      select.append(option);
    }
  }

  fillFleetSpawnSelect(values, selectedValue) {
    const isRemoteMode = String(this.fleetModeSelect?.value || "simulation") === "robots";
    const current = String(selectedValue || "");
    this.fleetSpawnLmSelect.innerHTML = "";
    if (isRemoteMode) {
      return;
    }
    for (const value of values) {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = value;
      option.selected = value === current || (!isRemoteMode && !current && value === values[0]);
      this.fleetSpawnLmSelect.append(option);
    }
  }

  renderFleetRobotList(robots) {
    if (!this.fleetRobotList) {
      return;
    }
    this.fleetRobotList.innerHTML = "";
    if (!robots.length) {
      const empty = document.createElement("div");
      empty.className = "probe-result neutral compact";
      empty.textContent = this.fleetRuntimeMode() === "robots"
        ? "No robots yet. Add a robot IP; LM is read from robot status."
        : "No robots yet. Add a simulation robot from a start LM.";
      this.fleetRobotList.append(empty);
      return;
    }
    for (const robot of robots) {
      const row = document.createElement("div");
      row.className = robot.name === this.selectedFleetRobotName ? "fleet-list-item active" : "fleet-list-item";

      const button = document.createElement("button");
      button.type = "button";
      button.className = "fleet-list-main";
      const selectFleetRobot = () => {
        this.selectedFleetRobotName = robot.name;
        if (this.navigateMode && this.pendingFleetAction) {
          this.pendingFleetRobotName = robot.name;
        }
        window.localStorage.setItem("operator:selectedFleetRobotName", robot.name);
        this.renderSelectedRobot();
      };
      button.addEventListener("pointerdown", (event) => {
        if (event.button !== 0) {
          return;
        }
        event.preventDefault();
        event.stopPropagation();
        selectFleetRobot();
      });
      button.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        selectFleetRobot();
      });

      const color = document.createElement("span");
      color.className = "fleet-list-color";
      color.style.background = robot.name === this.selectedFleetRobotName ? "#2368ff" : "#d37a22";
      button.append(color);

      const info = document.createElement("span");
      info.className = "fleet-list-name";
      const title = document.createElement("strong");
      title.textContent = robot.name || "-";
      const subtitle = document.createElement("span");
      const robotMode = String(robot.mode || robot.type || "simulated");
      const remoteStatus = this.remoteStatusForFleetRobot(robot);
      const mapLabel = this.fleetRobotMapLabel(robot, remoteStatus);
      const meta = [
        `${robot.currentLm || "-"} -> ${robot.targetLm || "-"}`,
        robotMode !== "simulated" ? robotMode : "",
        robotMode !== "simulated" ? (robot.online === false ? "offline" : "online") : "",
        mapLabel && mapLabel !== "-" ? `map ${mapLabel}` : "",
      ].filter(Boolean);
      subtitle.textContent = meta.join(" | ");
      if (this.queuedGoalFor(robot.name)) {
        subtitle.textContent = `${subtitle.textContent} | queued ${this.queuedGoalFor(robot.name)}`;
      }
      info.append(title, subtitle);
      button.append(info);

      const state = document.createElement("span");
      state.className = "fleet-list-state";
      state.textContent = robot.status || "IDLE";
      button.append(state);

      const removeButton = document.createElement("button");
      removeButton.type = "button";
      removeButton.className = "fleet-list-remove";
      removeButton.textContent = "-";
      removeButton.title = `Remove ${robot.name}`;
      removeButton.addEventListener("pointerdown", (event) => {
        event.preventDefault();
        event.stopPropagation();
      });
      removeButton.addEventListener("click", (event) => {
        event.stopPropagation();
        this.handleFleetRemoveRobot(robot.name);
      });

      row.append(button, removeButton);
      this.fleetRobotList.append(row);
    }
  }

  queuedGoalFor(robotName) {
    const draftGoals = this.fleetDraftGoalsFor(robotName);
    if (draftGoals.length) {
      return `${draftGoals.length} draft`;
    }
    const item = this.fleetOrders().find((entry) => {
      const status = String(entry.status || "").toUpperCase();
      if (this.isOrderTerminal(status)) {
        return false;
      }
      return entry.vehicle === robotName || entry.assignedRobot === robotName;
    });
    if (!item) {
      return "";
    }
    const totalSteps = Number(item.totalSteps || (Array.isArray(item.targets) ? item.targets.length : 1) || 1);
    const currentStep = Math.min(totalSteps, Number(item.currentStep || 0) + 1);
    return `${currentStep}/${totalSteps} ${item.targetLm || "-"} ${String(item.status || "").toLowerCase()}`;
  }

  fleetDraftGoalsFor(robotName) {
    return this.fleetQueue
      .filter((entry) => entry.robotName === robotName)
      .sort((a, b) => Number(a.seq || 0) - Number(b.seq || 0));
  }

  fleetDraftGroups() {
    const groups = new Map();
    for (const item of this.fleetQueue.slice().sort((a, b) => Number(a.seq || 0) - Number(b.seq || 0))) {
      if (!groups.has(item.robotName)) {
        groups.set(item.robotName, []);
      }
      groups.get(item.robotName).push(item);
    }
    return Array.from(groups.entries()).map(([robotName, goals]) => ({ robotName, goals }));
  }

  fleetOrders() {
    return Array.isArray(this.currentStatus?.orders) ? this.currentStatus.orders : [];
  }

  isOrderTerminal(status) {
    return ["COMPLETED", "FAILED", "CANCELED"].includes(String(status || "").toUpperCase());
  }

  selectedFleetOrder() {
    const orders = this.fleetOrders();
    if (!orders.length) {
      this.selectedFleetOrderId = "";
      return null;
    }
    const selected = orders.find((order) => (order.id || order.orderId) === this.selectedFleetOrderId);
    if (selected) {
      return selected;
    }
    const active = orders.find((order) => !this.isOrderTerminal(order.status));
    const fallback = active || orders[0];
    this.selectedFleetOrderId = fallback.id || fallback.orderId || "";
    return fallback;
  }

  orderTargetsLabel(order) {
    const targets = Array.isArray(order?.targets) && order.targets.length
      ? order.targets
      : [order?.targetLm || "-"];
    return targets.join(" -> ");
  }

  renderFleetQueue() {
    if (!this.fleetQueueList) {
      return;
    }
    this.fleetQueueList.innerHTML = "";
    const draftGroups = this.fleetDraftGroups();
    const orders = this.fleetOrders();
    if (orders.length) {
      this.selectedFleetOrder();
    }
    if (!draftGroups.length && !orders.length) {
      this.fleetQueueList.textContent = "No orders yet.";
      this.renderFleetOrderDetails();
      return;
    }
    for (const group of draftGroups) {
      const row = document.createElement("div");
      row.className = "fleet-queue-item draft";
      const text = document.createElement("span");
      const targets = group.goals.map((item) => item.targetLm || item.goalLm || "-");
      text.textContent = `${group.robotName} -> ${targets.join(" -> ")} | DRAFT`;
      const remove = document.createElement("button");
      remove.type = "button";
      remove.textContent = "x";
      remove.title = "Remove draft queue";
      remove.addEventListener("click", () => {
        this.fleetQueue = this.fleetQueue.filter((entry) => entry.robotName !== group.robotName);
        this.renderSelectedRobot();
      });
      row.append(text, remove);
      this.fleetQueueList.append(row);
    }
    for (const item of orders.slice(0, 80)) {
      const status = String(item.status || "QUEUED").toUpperCase();
      const orderId = item.id || item.orderId || "-";
      const row = document.createElement("div");
      row.className = [
        "fleet-queue-item",
        status.toLowerCase(),
        orderId === this.selectedFleetOrderId ? "selected" : "",
      ].filter(Boolean).join(" ");
      row.setAttribute("role", "button");
      row.tabIndex = 0;
      const selectOrder = () => {
        this.selectedFleetOrderId = orderId;
        this.renderFleetQueue();
      };
      row.addEventListener("pointerdown", (event) => {
        if (event.button !== 0) {
          return;
        }
        event.preventDefault();
        selectOrder();
      });
      row.addEventListener("keydown", (event) => {
        if (event.key === "Enter" || event.key === " ") {
          event.preventDefault();
          selectOrder();
        }
      });
      const text = document.createElement("span");
      const robotName = item.assignedRobot || item.vehicle || "auto";
      const totalSteps = Number(item.totalSteps || (Array.isArray(item.targets) ? item.targets.length : 1) || 1);
      const currentStep = Math.min(totalSteps, Number(item.currentStep || 0) + 1);
      const stepText = totalSteps > 1 ? ` ${currentStep}/${totalSteps}` : "";
      text.textContent = `${robotName} -> ${this.orderTargetsLabel(item)} | ${status}${stepText} | ${orderId}`;
      const remove = document.createElement("button");
      remove.type = "button";
      remove.textContent = this.isOrderTerminal(status) ? "." : "x";
      remove.disabled = this.isOrderTerminal(status);
      remove.title = this.isOrderTerminal(status) ? "Order finished" : "Cancel order";
      remove.addEventListener("pointerdown", (event) => {
        event.preventDefault();
        event.stopPropagation();
      });
      remove.addEventListener("click", (event) => {
        event.stopPropagation();
        this.cancelFleetOrder(orderId);
      });
      row.append(text, remove);
      this.fleetQueueList.append(row);
    }
    this.renderFleetOrderDetails();
  }

  renderFleetOrderDetails() {
    if (!this.fleetOrderDetails) {
      return;
    }
    const order = this.selectedFleetOrder();
    if (!order) {
      this.fleetOrderDetails.textContent = "Select an order to inspect.";
      this.syncFleetOrderActionButtons(null);
      return;
    }
    const status = String(order.status || "QUEUED").toUpperCase();
    const totalSteps = Number(order.totalSteps || (Array.isArray(order.targets) ? order.targets.length : 1) || 1);
    const currentStep = Math.min(totalSteps, Number(order.currentStep || 0) + 1);
    const details = [
      ["ID", order.id || order.orderId || "-"],
      ["Robot", order.assignedRobot || order.vehicle || "auto"],
      ["Status", status],
      ["Step", `${currentStep}/${totalSteps}`],
      ["Target", order.targetLm || "-"],
      ["Targets", this.orderTargetsLabel(order)],
      ["Route", Array.isArray(order.routeNodes) && order.routeNodes.length ? order.routeNodes.join(" -> ") : "-"],
      ["Reason", order.error || "-"],
    ];
    this.fleetOrderDetails.innerHTML = "";
    for (const [label, value] of details) {
      const row = document.createElement("div");
      row.className = "fleet-order-detail-row";
      const name = document.createElement("span");
      name.textContent = label;
      const text = document.createElement("strong");
      text.textContent = String(value);
      row.append(name, text);
      this.fleetOrderDetails.append(row);
    }
    this.syncFleetOrderActionButtons(order);
  }

  syncFleetOrderActionButtons(order) {
    const status = String(order?.status || "").toUpperCase();
    const hasOrder = Boolean(order);
    const terminal = this.isOrderTerminal(status);
    this.fleetPauseOrderButton.disabled = !hasOrder || terminal || status === "PAUSED";
    this.fleetResumeOrderButton.disabled = !hasOrder || terminal || status !== "PAUSED";
    this.fleetCancelOrderButton.disabled = !hasOrder || terminal;
  }

  renderFleetPlanDebug() {
    if (!this.fleetPlanDebug) {
      return;
    }
    const status = this.currentStatus || {};
    const robots = Array.isArray(status.robots) ? status.robots : [];
    const selected = this.selectedFleetRobot(robots);
    if (!selected) {
      this.fleetPlanDebug.textContent = "Planner idle.";
      return;
    }
    const reason = selected.reason || selected.routeNote || "ready";
    const nodes = Array.isArray(selected.planNodes) && selected.planNodes.length
      ? selected.planNodes.join(" -> ")
      : "no route";
    this.fleetPlanDebug.textContent = `${selected.name}: ${selected.status || "IDLE"} | ${reason} | ${nodes}`;
  }

  targetFleetRobot() {
    const robots = Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : [];
    if (this.pendingFleetRobotName) {
      const pending = robots.find((robot) => robot.name === this.pendingFleetRobotName);
      if (pending) {
        return pending;
      }
    }
    return this.selectedFleetRobot(robots);
  }

  async queueFleetGoal(goalLm) {
    const robot = this.targetFleetRobot();
    if (!robot) {
      this.robotMessageText.textContent = "Select a fleet robot first.";
      return;
    }
    this.fleetQueue.push({
      robotName: robot.name,
      targetLm: goalLm,
      seq: ++this.fleetQueueSequence,
    });
    this.navigateMode = true;
    this.pendingFleetAction = "queue";
    this.pendingFleetRobotName = robot.name;
    this.renderSelectedRobot();
    this.syncModeButtons();
    this.drawLandmarks();
    const count = this.fleetDraftGoalsFor(robot.name).length;
    this.robotMessageText.textContent = `Draft queue ${robot.name}: ${count} LM goal(s). Press Dispatch to send.`;
  }

  async clearFleetQueue() {
    if (this.pendingFleetAction === "queue") {
      this.navigateMode = false;
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
    }
    const draftCount = this.fleetQueue.length;
    this.fleetQueue = [];
    try {
      const result = await this.postJson("/api/fleet-manager/orders/clear", { includeActive: false });
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      const canceled = Number(result.canceled || 0);
      this.robotMessageText.textContent = (draftCount || canceled)
        ? `Queue cleared: draft=${draftCount}, backend=${canceled}.`
        : "Queue is empty.";
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Clear queue failed: ${error.message || error}`;
    }
  }

  async startQueuedFleetPlan() {
    await this.releaseFleetManualControl();
    if (this.fleetQueue.length) {
      await this.dispatchDraftFleetQueue();
      return;
    }
    try {
      const result = await this.postJson("/api/fleet-manager/orders/dispatch", {});
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      const dispatched = Number(result.dispatched || 0);
      this.robotMessageText.textContent = dispatched ? `Orders dispatched: ${dispatched}.` : "No dispatchable orders right now.";
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Dispatch failed: ${error.message || error}`;
    }
  }

  async dispatchDraftFleetQueue() {
    const groups = new Map();
    for (const item of this.fleetQueue.slice().sort((a, b) => Number(a.seq || 0) - Number(b.seq || 0))) {
      if (!groups.has(item.robotName)) {
        groups.set(item.robotName, []);
      }
      groups.get(item.robotName).push(item.targetLm || item.goalLm);
    }
    let sent = 0;
    let lastState = null;
    const sentRobots = new Set();
    try {
      for (const [robotName, targets] of groups.entries()) {
        if (!targets.length) {
          continue;
        }
        const result = await this.postJson("/api/fleet-manager/setOrder", {
          id: this.nextFleetOrderId(robotName),
          vehicle: robotName,
          priority: 10,
          targets,
          speed: this.fleetRouteSpeed(),
        });
        sent += targets.length;
        sentRobots.add(robotName);
        lastState = result.state || lastState;
      }
      this.fleetQueue = [];
      this.navigateMode = false;
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.currentStatus = lastState || await this.getJson("/api/fleet-manager/state");
      this.robotMessageText.textContent = `Dispatched draft queue: ${sent} LM goal(s).`;
      this.renderSelectedRobot();
    } catch (error) {
      if (sentRobots.size) {
        this.fleetQueue = this.fleetQueue.filter((entry) => !sentRobots.has(entry.robotName));
      }
      if (lastState) {
        this.currentStatus = lastState;
      }
      this.robotMessageText.textContent = `Dispatch failed: ${error.message || error}`;
      this.renderSelectedRobot();
    }
  }

  async cancelFleetOrder(orderId) {
    if (!orderId || orderId === "-") {
      return;
    }
    try {
      const result = await this.postJson("/api/fleet-manager/orders/cancel", { id: orderId });
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      this.robotMessageText.textContent = `Order canceled: ${orderId}.`;
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Cancel order failed: ${error.message || error}`;
    }
  }

  selectedFleetOrderIdOrMessage() {
    const order = this.selectedFleetOrder();
    const orderId = order?.id || order?.orderId || "";
    if (!orderId) {
      this.robotMessageText.textContent = "Select an order first.";
      return "";
    }
    return orderId;
  }

  async pauseSelectedFleetOrder() {
    const orderId = this.selectedFleetOrderIdOrMessage();
    if (!orderId) {
      return;
    }
    try {
      const result = await this.postJson("/api/fleet-manager/orders/pause", { id: orderId });
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      this.robotMessageText.textContent = `Order paused: ${orderId}.`;
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Pause order failed: ${error.message || error}`;
    }
  }

  async resumeSelectedFleetOrder() {
    const orderId = this.selectedFleetOrderIdOrMessage();
    if (!orderId) {
      return;
    }
    try {
      const result = await this.postJson("/api/fleet-manager/orders/resume", { id: orderId });
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      this.robotMessageText.textContent = `Order resumed: ${orderId}.`;
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Resume order failed: ${error.message || error}`;
    }
  }

  async cancelSelectedFleetOrder() {
    const orderId = this.selectedFleetOrderIdOrMessage();
    if (!orderId) {
      return;
    }
    await this.cancelFleetOrder(orderId);
  }

  nextFleetOrderId(robotName) {
    const safeRobot = String(robotName || "robot").replace(/[^a-z0-9_-]+/gi, "-").replace(/^-+|-+$/g, "") || "robot";
    this.fleetQueueSequence += 1;
    return `${safeRobot}-${Date.now()}-${this.fleetQueueSequence}`;
  }

  renderEvents(events) {
    this.robotEventsLog.innerHTML = "";
    if (!events.length) {
      this.robotEventsLog.textContent = "No events yet.";
      return;
    }
    for (const event of events.slice().reverse().slice(0, 80)) {
      const row = document.createElement("div");
      row.className = `event-row ${String(event.level || "info").toLowerCase()}`;
      const stamp = event.stamp ? new Date(Number(event.stamp) * 1000).toLocaleTimeString([], { hour12: false }) : "--:--:--";
      row.textContent = `${stamp} ${event.level || "info"} ${event.message || ""}`;
      this.robotEventsLog.append(row);
    }
  }

  renderOperatorMap() {
    const payload = this.activeOperatorMapPayload();
    if (!payload || !payload.map) {
      this.operatorMapSvg.setAttribute("viewBox", "0 0 100 100");
      this.operatorMapImage.removeAttribute("href");
      this.operatorGraphLayer.innerHTML = "";
      this.operatorRouteLayer.innerHTML = "";
      this.operatorLookaheadLayer.innerHTML = "";
      this.operatorLandmarkLayer.innerHTML = "";
      this.operatorEditorLayer.innerHTML = "";
      this.operatorRobotLayer.innerHTML = "";
      return;
    }
    const map = payload.map;
    this.operatorMapSvg.setAttribute("viewBox", `0 0 ${Number(map.viewWidth || 100)} ${Number(map.viewHeight || 100)}`);
    this.operatorMapImage.setAttribute("x", String(Number(map.viewPadding || 0)));
    this.operatorMapImage.setAttribute("y", String(Number(map.viewPadding || 0)));
    this.operatorMapImage.setAttribute("width", String(Number(map.width || 0)));
    this.operatorMapImage.setAttribute("height", String(Number(map.height || 0)));
    this.operatorMapImage.setAttribute("href", String(map.imageDataUrl || ""));
    this.applyMapTransform();
    this.drawGraph();
    this.drawRoute();
    this.drawLookahead();
    this.drawLandmarks();
    this.drawFleetEditorOverlay();
    this.drawRobot();
    this.syncMapControls();
  }

  activeOperatorMapPayload() {
    if (this.isFleetManager() && this.fleetMapEditorActive && this.fleetMapDraft) {
      return this.fleetMapDraft;
    }
    return this.operatorMapPayload;
  }

  drawGraph() {
    const payload = this.activeOperatorMapPayload();
    const landmarks = this.landmarkIndex();
    this.operatorGraphLayer.innerHTML = "";
    for (const edge of payload.edges || []) {
      const element = document.createElementNS("http://www.w3.org/2000/svg", edge.geometry === "bezier" ? "path" : "line");
      const edgeKey = this.edgeKey(edge.from, edge.to);
      element.setAttribute("class", [
        "graph-edge",
        this.fleetMapEditorActive ? "editable" : "",
        edgeKey === this.fleetSelectedEdgeKey ? "selected" : "",
      ].filter(Boolean).join(" "));
      element.dataset.edgeKey = edgeKey;
      element.addEventListener("pointerdown", (event) => {
        if (!this.fleetMapEditorActive || event.button !== 0) {
          return;
        }
        event.preventDefault();
        event.stopPropagation();
        this.selectFleetEditorEdge(edgeKey);
      });
      if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
        const points = edge.control_points.map((point) => this.worldToPixel(point));
        element.setAttribute("d", `M ${points[0].x} ${points[0].y} C ${points[1].x} ${points[1].y}, ${points[2].x} ${points[2].y}, ${points[3].x} ${points[3].y}`);
      } else {
        const start = landmarks.get(edge.from);
        const goal = landmarks.get(edge.to);
        if (!start || !goal) {
          continue;
        }
        const startPx = this.worldToPixel(start);
        const goalPx = this.worldToPixel(goal);
        element.setAttribute("x1", String(startPx.x));
        element.setAttribute("y1", String(startPx.y));
        element.setAttribute("x2", String(goalPx.x));
        element.setAttribute("y2", String(goalPx.y));
      }
      this.operatorGraphLayer.append(element);
      const arrow = this.directionArrow(edge, landmarks);
      if (arrow) {
        this.operatorGraphLayer.append(arrow);
      }
    }
  }

  drawRoute() {
    this.operatorRouteLayer.innerHTML = "";
    if (this.isFleetManager()) {
      const robots = this.fleetRenderRobots();
      for (const robot of robots) {
        if (!Array.isArray(robot.trajectory) || robot.trajectory.length < 2) {
          continue;
        }
        this.drawFleetRoute(robot);
      }
      return;
    }
    const route = (this.currentStatus && this.currentStatus.route) || this.currentRoute;
    if (!route || !Array.isArray(route.trajectory) || route.trajectory.length < 2) {
      return;
    }
    const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
    polyline.setAttribute("class", "planned-route");
    polyline.setAttribute("points", route.trajectory.map((point) => {
      const px = this.worldToPixel(point);
      return `${px.x},${px.y}`;
    }).join(" "));
    this.operatorRouteLayer.append(polyline);
  }

  drawFleetRoute(robot) {
    const trajectory = robot.trajectory || [];
    const active = robot.name === this.selectedFleetRobotName;
    this.appendRoutePolyline(trajectory, active ? "fleet-route-plan active" : "fleet-route-plan");
    if (!active) {
      return;
    }
    const clock = Math.max(0, Number(robot.routeClock || 0));
    const finalTime = Math.max(...trajectory.map((point, index) => Number(point.t ?? index)));
    const done = this.sliceTrajectoryByTime(trajectory, 0, clock);
    const remaining = this.sliceTrajectoryByTime(trajectory, clock, finalTime);
    if (done.length > 1) {
      this.appendRoutePolyline(done, "fleet-route-done");
    }
    if (remaining.length > 1) {
      this.appendRoutePolyline(remaining, "fleet-route-active");
    }
  }

  appendRoutePolyline(points, className) {
    if (!Array.isArray(points) || points.length < 2) {
      return;
    }
    const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
    polyline.setAttribute("class", className);
    polyline.setAttribute("points", points.map((point) => {
      const px = this.worldToPixel(point);
      return `${px.x},${px.y}`;
    }).join(" "));
    this.operatorRouteLayer.append(polyline);
  }

  sliceTrajectoryByTime(points, startTime, endTime) {
    if (!Array.isArray(points) || points.length < 2 || endTime <= startTime) {
      return [];
    }
    const result = [];
    const startPose = this.interpolateTrajectory(points, startTime);
    if (startPose) {
      result.push(startPose);
    }
    for (const point of points) {
      const t = Number(point.t ?? 0);
      if (t > startTime && t < endTime) {
        result.push(point);
      }
    }
    const endPose = this.interpolateTrajectory(points, endTime);
    if (endPose) {
      result.push(endPose);
    }
    return result;
  }

  interpolateTrajectory(points, targetTime) {
    if (!points.length) {
      return null;
    }
    const firstTime = Number(points[0].t ?? 0);
    if (targetTime <= firstTime) {
      return points[0];
    }
    const last = points[points.length - 1];
    const lastTime = Number(last.t ?? points.length - 1);
    if (targetTime >= lastTime) {
      return last;
    }
    for (let index = 0; index < points.length - 1; index += 1) {
      const start = points[index];
      const goal = points[index + 1];
      const t0 = Number(start.t ?? index);
      const t1 = Number(goal.t ?? index + 1);
      if (targetTime < t0 || targetTime > t1) {
        continue;
      }
      const ratio = (targetTime - t0) / Math.max(0.000001, t1 - t0);
      return {
        ...start,
        x: Number(start.x || 0) + ((Number(goal.x || 0) - Number(start.x || 0)) * ratio),
        y: Number(start.y || 0) + ((Number(goal.y || 0) - Number(start.y || 0)) * ratio),
        yaw: this.interpolateAngle(Number(start.yaw || 0), Number(goal.yaw || 0), ratio),
        t: targetTime,
      };
    }
    return last;
  }

  interpolateAngle(start, goal, ratio) {
    const delta = ((goal - start + Math.PI) % (Math.PI * 2)) - Math.PI;
    return start + (delta * ratio);
  }

  drawLookahead() {
    if (!this.operatorLookaheadLayer) {
      return;
    }
    this.operatorLookaheadLayer.innerHTML = "";
    if (!this.isFleetManager() || !this.fleetManualLookahead || !Array.isArray(this.fleetManualLookahead.poses)) {
      return;
    }
    const poses = this.fleetManualLookahead.poses;
    if (poses.length < 2) {
      return;
    }
    poses.slice(1).forEach((pose, index) => {
      if (index % 2 !== 0 && index !== poses.length - 2) {
        return;
      }
      const footprint = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
      footprint.setAttribute("class", this.fleetManualLookahead.blocked ? "lookahead-footprint blocked" : "lookahead-footprint");
      footprint.setAttribute("points", this.robotFootprintPoints(pose));
      this.operatorLookaheadLayer.append(footprint);
    });
    const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
    polyline.setAttribute("class", this.fleetManualLookahead.blocked ? "lookahead-route blocked" : "lookahead-route");
    polyline.setAttribute("points", poses.map((point) => {
      const px = this.worldToPixel(point);
      return `${px.x},${px.y}`;
    }).join(" "));
    this.operatorLookaheadLayer.append(polyline);
    const last = this.worldToPixel(poses[poses.length - 1]);
    const marker = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    marker.setAttribute("class", this.fleetManualLookahead.blocked ? "lookahead-marker blocked" : "lookahead-marker");
    marker.setAttribute("cx", String(last.x));
    marker.setAttribute("cy", String(last.y));
    marker.setAttribute("r", "6");
    this.operatorLookaheadLayer.append(marker);
  }

  drawLandmarks() {
    const statusRobot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    const fleetRobot = this.isFleetManager() ? this.selectedFleetRobot() : null;
    const target = fleetRobot ? (fleetRobot.targetLm || "") : (statusRobot.targetLm || (this.currentRoute && this.currentRoute.goalLm) || "");
    const nearest = fleetRobot ? (fleetRobot.currentLm || "") : statusRobot.nearestLm;
    this.operatorLandmarkLayer.innerHTML = "";
    for (const landmark of this.activeOperatorMapPayload().lms || []) {
      const px = this.worldToPixel(landmark);
      const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
      group.setAttribute("class", [
        "landmark",
        landmark.name === nearest ? "nearest" : "",
        landmark.name === target ? "target" : "",
        this.fleetMapEditorActive && landmark.name === this.fleetSelectedLmName ? "selected" : "",
        this.navigateMode ? "armed" : "",
      ].filter(Boolean).join(" "));
      group.dataset.lmName = landmark.name;
      group.addEventListener("pointerdown", (event) => {
        if (this.fleetMapEditorActive) {
          return;
        }
        if (event.button !== 0 || !this.navigateMode) {
          return;
        }
        event.preventDefault();
        event.stopPropagation();
        this.mapClickConsumed = true;
        this.handleLandmarkTarget(landmark.name);
        window.setTimeout(() => {
          this.mapClickConsumed = false;
        }, 150);
      });
      group.addEventListener("click", (event) => {
        if (this.fleetMapEditorActive) {
          event.preventDefault();
          event.stopPropagation();
          return;
        }
        event.preventDefault();
        event.stopPropagation();
        if (this.navigateMode && !this.mapClickConsumed) {
          this.handleLandmarkTarget(landmark.name);
        }
        window.setTimeout(() => {
          this.mapClickConsumed = false;
        }, 0);
      });
      const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      circle.setAttribute("cx", String(px.x));
      circle.setAttribute("cy", String(px.y));
      circle.setAttribute("r", "5.5");
      group.append(circle);
      const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
      label.setAttribute("x", String(px.x));
      label.setAttribute("y", String(px.y + 18));
      label.textContent = landmark.name;
      group.append(label);
      this.operatorLandmarkLayer.append(group);
    }
  }

  drawRobot() {
    this.operatorRobotLayer.innerHTML = "";
    if (this.isFleetManager()) {
      const robots = this.fleetRenderRobots();
      let focused = false;
      for (const robot of robots) {
        const pose = robot && robot.pose ? robot.pose : null;
        if (!pose) {
          continue;
        }
        const center = this.worldToPixel(pose);
        if (!focused && this.mapView.follow && robot.name === this.selectedFleetRobotName) {
          this.focusMapOn(center);
          focused = true;
        }
        const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
        group.setAttribute("class", robot.name === this.selectedFleetRobotName ? "fleet-robot active" : "fleet-robot");
        group.dataset.robotName = robot.name || "";
        const selectRobot = (event) => {
          if (this.fleetMapEditorActive) {
            return;
          }
          event.preventDefault();
          event.stopPropagation();
          this.selectedFleetRobotName = robot.name || "";
          if (this.navigateMode && this.pendingFleetAction) {
            this.pendingFleetRobotName = this.selectedFleetRobotName;
          }
          window.localStorage.setItem("operator:selectedFleetRobotName", this.selectedFleetRobotName);
          this.renderSelectedRobot();
        };
        group.addEventListener("pointerdown", (event) => {
          if (event.button !== 0) {
            return;
          }
          selectRobot(event);
        });
        group.addEventListener("click", selectRobot);
        const footprint = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
        footprint.setAttribute("class", "robot-footprint");
        footprint.setAttribute("points", this.robotFootprintPoints(pose));
        group.append(footprint);
        const centerDot = document.createElementNS("http://www.w3.org/2000/svg", "circle");
        centerDot.setAttribute("class", "robot-center-dot");
        centerDot.setAttribute("cx", String(center.x));
        centerDot.setAttribute("cy", String(center.y));
        centerDot.setAttribute("r", "4");
        group.append(centerDot);
        const heading = document.createElementNS("http://www.w3.org/2000/svg", "line");
        heading.setAttribute("class", "robot-heading");
        heading.setAttribute("x1", String(center.x));
        heading.setAttribute("y1", String(center.y));
        heading.setAttribute("x2", String(center.x + Math.cos(Number(pose.yaw || 0)) * 18));
        heading.setAttribute("y2", String(center.y - Math.sin(Number(pose.yaw || 0)) * 18));
        group.append(heading);
        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("x", String(center.x));
        label.setAttribute("y", String(center.y + 25));
        label.textContent = robot.name || "";
        group.append(label);
        this.operatorRobotLayer.append(group);
      }
      return;
    }
    const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : null;
    const pose = robot && robot.pose ? robot.pose : null;
    if (!pose) {
      return;
    }
    const center = this.worldToPixel(pose);
    if (this.mapView.follow) {
      this.focusMapOn(center);
    }
    const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const body = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    body.setAttribute("class", "robot-body");
    body.setAttribute("cx", String(center.x));
    body.setAttribute("cy", String(center.y));
    body.setAttribute("r", "12");
    group.append(body);
    const heading = document.createElementNS("http://www.w3.org/2000/svg", "line");
    heading.setAttribute("class", "robot-heading");
    heading.setAttribute("x1", String(center.x));
    heading.setAttribute("y1", String(center.y));
    heading.setAttribute("x2", String(center.x + Math.cos(Number(pose.yaw || 0)) * 20));
    heading.setAttribute("y2", String(center.y - Math.sin(Number(pose.yaw || 0)) * 20));
    group.append(heading);
    this.operatorRobotLayer.append(group);
  }

  robotFootprintPoints(pose) {
    const model = this.fleetParams?.robot_model || this.fleetModelEditor?.getModel() || {};
    const footprint = Array.isArray(model.footprint) && model.footprint.length >= 3
      ? model.footprint
      : [
        { x: 0.220000, y: 0.000000 },
        { x: 0.203253, y: 0.084190 },
        { x: 0.155563, y: 0.155563 },
        { x: 0.084190, y: 0.203253 },
        { x: 0.000000, y: 0.220000 },
        { x: -0.084190, y: 0.203253 },
        { x: -0.155563, y: 0.155563 },
        { x: -0.203253, y: 0.084190 },
        { x: -0.220000, y: 0.000000 },
        { x: -0.203253, y: -0.084190 },
        { x: -0.155563, y: -0.155563 },
        { x: -0.084190, y: -0.203253 },
        { x: 0.000000, y: -0.220000 },
        { x: 0.084190, y: -0.203253 },
        { x: 0.155563, y: -0.155563 },
        { x: 0.203253, y: -0.084190 },
      ];
    const yaw = Number(pose.yaw || 0);
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    return footprint.map((point) => {
      const world = {
        x: Number(pose.x || 0) + (Number(point.x || 0) * cos) - (Number(point.y || 0) * sin),
        y: Number(pose.y || 0) + (Number(point.x || 0) * sin) + (Number(point.y || 0) * cos),
      };
      const pixel = this.worldToPixel(world);
      return `${pixel.x},${pixel.y}`;
    }).join(" ");
  }

  landmarkIndex() {
    return new Map((this.activeOperatorMapPayload()?.lms || []).map((lm) => [lm.name, lm]));
  }

  worldToPixel(point) {
    const map = this.activeOperatorMapPayload()?.map || {};
    const origin = Array.isArray(map.origin) ? map.origin : [0, 0, 0];
    const resolution = Number(map.resolution || 1);
    const padding = Number(map.viewPadding || 0);
    const height = Number(map.height || 0);
    return {
      x: padding + ((Number(point.x || 0) - Number(origin[0] || 0)) / resolution),
      y: padding + (height - 1) - ((Number(point.y || 0) - Number(origin[1] || 0)) / resolution),
    };
  }

  pixelToWorld(point) {
    const map = this.activeOperatorMapPayload()?.map || {};
    const origin = Array.isArray(map.origin) ? map.origin : [0, 0, 0];
    const resolution = Number(map.resolution || 1);
    const padding = Number(map.viewPadding || 0);
    const height = Number(map.height || 0);
    return {
      x: ((point.x - padding) * resolution) + Number(origin[0] || 0),
      y: ((height - 1) - (point.y - padding)) * resolution + Number(origin[1] || 0),
    };
  }

  directionArrow(edge, landmarks) {
    let point = null;
    let tangent = null;
    if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      const points = edge.control_points.map((item) => this.worldToPixel(item));
      point = this.bezierPoint(points, 0.5);
      tangent = this.bezierTangent(points, 0.5);
    } else {
      const start = landmarks.get(edge.from);
      const goal = landmarks.get(edge.to);
      if (!start || !goal) {
        return null;
      }
      const s = this.worldToPixel(start);
      const g = this.worldToPixel(goal);
      point = { x: (s.x + g.x) / 2, y: (s.y + g.y) / 2 };
      tangent = { x: g.x - s.x, y: g.y - s.y };
    }
    const length = Math.hypot(tangent.x, tangent.y);
    if (length <= 0.001) {
      return null;
    }
    const ux = tangent.x / length;
    const uy = tangent.y / length;
    const px = -uy;
    const py = ux;
    const tip = { x: point.x + ux * 8, y: point.y + uy * 8 };
    const base = { x: point.x - ux * 8, y: point.y - uy * 8 };
    const left = { x: base.x + px * 5, y: base.y + py * 5 };
    const right = { x: base.x - px * 5, y: base.y - py * 5 };
    const polygon = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
    polygon.setAttribute("class", "graph-direction");
    polygon.setAttribute("points", `${tip.x},${tip.y} ${left.x},${left.y} ${right.x},${right.y}`);
    return polygon;
  }

  bezierPoint(points, t) {
    const [p0, p1, p2, p3] = points;
    const u = 1 - t;
    return {
      x: (u ** 3 * p0.x) + (3 * u * u * t * p1.x) + (3 * u * t * t * p2.x) + (t ** 3 * p3.x),
      y: (u ** 3 * p0.y) + (3 * u * u * t * p1.y) + (3 * u * t * t * p2.y) + (t ** 3 * p3.y),
    };
  }

  bezierTangent(points, t) {
    const [p0, p1, p2, p3] = points;
    const u = 1 - t;
    return {
      x: (3 * u * u * (p1.x - p0.x)) + (6 * u * t * (p2.x - p1.x)) + (3 * t * t * (p3.x - p2.x)),
      y: (3 * u * u * (p1.y - p0.y)) + (6 * u * t * (p2.y - p1.y)) + (3 * t * t * (p3.y - p2.y)),
    };
  }

  handleMapPointerDown(event) {
    if (this.isFleetManager() && this.fleetMapEditorActive) {
      this.handleFleetEditorPointerDown(event);
      return;
    }
    if (event.button !== 0 || this.navigateMode || event.target.closest(".landmark")) {
      return;
    }
    event.preventDefault();
    const point = this.screenToSvg(event.clientX, event.clientY);
    if (!point) {
      return;
    }
    this.mapDrag = { pointerId: event.pointerId, last: point };
    this.operatorMapSvg.classList.add("dragging");
    this.operatorMapSvg.setPointerCapture(event.pointerId);
  }

  handleMapPointerMove(event) {
    if (this.isFleetManager() && this.fleetMapEditorActive) {
      this.handleFleetEditorPointerMove(event);
      return;
    }
    this.applyMapDragMove(event);
  }

  applyMapDragMove(event) {
    if (!this.mapDrag || this.mapDrag.pointerId !== event.pointerId) {
      return;
    }
    event.preventDefault();
    const point = this.screenToSvg(event.clientX, event.clientY);
    if (!point) {
      return;
    }
    const dx = point.x - this.mapDrag.last.x;
    const dy = point.y - this.mapDrag.last.y;
    if (Math.abs(dx) > 0.001 || Math.abs(dy) > 0.001) {
      this.mapView.follow = false;
      this.mapView.tx += dx;
      this.mapView.ty += dy;
      this.mapDrag.last = point;
      this.mapClickConsumed = true;
      this.applyMapTransform();
      this.syncMapControls();
    }
  }

  handleMapPointerUp(event) {
    if (this.isFleetManager() && this.fleetMapEditorActive) {
      this.handleFleetEditorPointerUp(event);
      return;
    }
    if (this.mapDrag && this.mapDrag.pointerId === event.pointerId) {
      if (this.operatorMapSvg.hasPointerCapture(event.pointerId)) {
        this.operatorMapSvg.releasePointerCapture(event.pointerId);
      }
      this.mapDrag = null;
      this.operatorMapSvg.classList.remove("dragging");
      window.setTimeout(() => {
        this.mapClickConsumed = false;
      }, 0);
    }
  }

  handleMapWheel(event) {
    event.preventDefault();
    const anchor = this.screenToSvg(event.clientX, event.clientY);
    this.zoomMap(event.deltaY < 0 ? 1.12 : 0.88, anchor);
  }

  handleMapClick(event) {
    if (this.isFleetManager() && this.fleetMapEditorActive) {
      return;
    }
    if (this.mapClickConsumed) {
      this.mapClickConsumed = false;
      return;
    }
    if (!this.navigateMode || event.target.closest(".landmark")) {
      return;
    }
    const point = this.screenToSvg(event.clientX, event.clientY);
    if (!point) {
      return;
    }
    const world = this.pixelToWorld({
      x: (point.x - this.mapView.tx) / this.mapView.scale,
      y: (point.y - this.mapView.ty) / this.mapView.scale,
    });
    if (this.isRos2Robot() && !this.isFleetManager()) {
      this.startPoseNavigation(world);
      return;
    }
    const nearest = this.nearestLandmark(world);
    if (!nearest || nearest.distance > 1.2) {
      this.robotMessageText.textContent = "Navigate armed: click closer to a landmark.";
      return;
    }
    this.handleLandmarkTarget(nearest.landmark.name);
  }

  handleMapContextMenu(event) {
    if (!this.isFleetManager() || !this.fleetMapEditorActive) {
      return;
    }
    event.preventDefault();
    const lmName = event.target.closest(".landmark")?.dataset?.lmName || "";
    if (lmName && window.confirm(`Delete ${lmName}?`)) {
      this.deleteFleetEditorLm(lmName);
      return;
    }
    const edgeKey = event.target.closest(".graph-edge")?.dataset?.edgeKey || "";
    if (edgeKey && window.confirm(`Delete edge ${edgeKey}?`)) {
      this.deleteFleetEditorEdge(edgeKey);
    }
  }

  ensureFleetMapDraft() {
    if (!this.fleetMapDraft && this.operatorMapPayload) {
      this.fleetMapDraft = this.cloneJson(this.operatorMapPayload);
      this.fleetMapDirty = false;
    }
    return this.fleetMapDraft;
  }

  reloadFleetMapDraft() {
    if (this.fleetMapDirty && !window.confirm("Discard unsaved fleet map changes?")) {
      return;
    }
    this.fleetMapDraft = this.cloneJson(this.operatorMapPayload);
    this.fleetMapDirty = false;
    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.syncFleetEditorFields();
    this.renderOperatorMap();
    this.robotMessageText.textContent = "Fleet map draft reloaded.";
  }

  cloneJson(value) {
    return JSON.parse(JSON.stringify(value || {}));
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

    if (bezierHandle) {
      const handleEdgeKey = bezierHandle.dataset.edgeKey || this.fleetSelectedEdgeKey;
      this.selectFleetEditorEdge(handleEdgeKey);
      this.fleetEditorBezierDrag = {
        pointerId: event.pointerId,
        edgeKey: handleEdgeKey,
        index: Number(bezierHandle.dataset.bezierIndex || 1),
      };
      this.operatorMapSvg.setPointerCapture(event.pointerId);
      return;
    }

    if (lmName) {
      this.selectFleetEditorLm(lmName);
      if (this.fleetMapTool === "edge") {
        this.fleetEditorEdgeDrag = { pointerId: event.pointerId, currentLm: lmName, lastCreated: "" };
      } else {
        this.fleetEditorLmDrag = { pointerId: event.pointerId, name: lmName, start: world, moved: false };
      }
      this.operatorMapSvg.setPointerCapture(event.pointerId);
      return;
    }

    if (edgeKey) {
      this.selectFleetEditorEdge(edgeKey);
      this.operatorMapSvg.setPointerCapture(event.pointerId);
      return;
    }

    if (this.fleetMapTool === "lm" && world) {
      const added = this.addFleetEditorLm(world);
      this.selectFleetEditorLm(added.name);
      this.renderOperatorMap();
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
    if (this.fleetEditorLmDrag && this.fleetEditorLmDrag.pointerId === event.pointerId) {
      this.fleetEditorLmDrag = null;
      this.fleetEditorGuideWorld = null;
      this.drawFleetEditorOverlay();
    }
    if (this.fleetEditorEdgeDrag && this.fleetEditorEdgeDrag.pointerId === event.pointerId) {
      this.fleetEditorEdgeDrag = null;
      this.fleetEditorPreview = null;
      this.fleetEditorGuideWorld = null;
      this.drawFleetEditorOverlay();
    }
    if (this.fleetEditorBezierDrag && this.fleetEditorBezierDrag.pointerId === event.pointerId) {
      this.fleetEditorBezierDrag = null;
      this.fleetEditorGuideWorld = null;
      this.drawFleetEditorOverlay();
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
      properties: { direction: 2, movestyle: 0 },
      length: 0,
    };
    edge.length = this.edgeLength(edge);
    draft.edges.push(edge);
    this.selectFleetEditorEdge(this.edgeKey(from, to));
  }

  deleteFleetEditorLm(name) {
    const draft = this.ensureFleetMapDraft();
    draft.lms = (draft.lms || []).filter((lm) => lm.name !== name);
    draft.edges = (draft.edges || []).filter((edge) => edge.from !== name && edge.to !== name);
    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.fleetMapDirty = true;
    this.syncFleetEditorFields();
    this.renderOperatorMap();
  }

  deleteFleetEditorEdge(edgeKey) {
    const draft = this.ensureFleetMapDraft();
    const [from, to] = edgeKey.split("->");
    draft.edges = (draft.edges || []).filter((edge) => !(edge.from === from && edge.to === to));
    this.fleetSelectedEdgeKey = "";
    this.fleetMapDirty = true;
    this.syncFleetEditorFields();
    this.renderOperatorMap();
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
    this.renderOperatorMap();
  }

  selectFleetEditorEdge(edgeKey) {
    this.fleetSelectedEdgeKey = edgeKey;
    this.fleetSelectedLmName = "";
    this.syncFleetEditorFields();
    this.renderOperatorMap();
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
      this.fleetEditorEdgeMotionSelect.value = String(Number((edge.properties || {}).direction ?? 2));
    }
    this.fleetEditorFieldSyncing = false;
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
    this.fleetMapDirty = true;
    this.syncFleetEditorFields();
    this.renderOperatorMap();
  }

  applyFleetEditorEdgeFields() {
    if (this.fleetEditorFieldSyncing || !this.fleetSelectedEdgeKey) {
      return;
    }
    const edge = this.edgeFromKey(this.fleetSelectedEdgeKey);
    if (!edge) {
      return;
    }
    edge.properties = {
      ...(edge.properties || {}),
      direction: Number(this.fleetEditorEdgeMotionSelect.value || 2),
      movestyle: Number((edge.properties || {}).movestyle || 0),
    };
    const reverseKey = this.edgeKey(edge.to, edge.from);
    const hasReverse = Boolean(this.edgeFromKey(reverseKey));
    if (this.fleetEditorEdgeTrafficSelect.value === "bidirectional" && !hasReverse) {
      this.addFleetEditorEdge(edge.to, edge.from);
      const reverse = this.edgeFromKey(reverseKey);
      if (reverse) {
        reverse.properties = { ...(edge.properties || {}) };
      }
      this.fleetSelectedEdgeKey = this.edgeKey(edge.from, edge.to);
    }
    if (this.fleetEditorEdgeTrafficSelect.value === "one_way" && hasReverse) {
      this.deleteFleetEditorEdge(reverseKey);
      this.fleetSelectedEdgeKey = this.edgeKey(edge.from, edge.to);
    }
    this.fleetMapDirty = true;
    this.syncFleetEditorFields();
    this.renderOperatorMap();
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
      return;
    }
    let mapName = this.robotMapState.operatorActiveMapName || draft.mapName || "";
    if (saveAs) {
      mapName = window.prompt("Save local fleet map as", `${draft.mapName || "fleet_map"}_copy`) || "";
      mapName = mapName.trim();
      if (!mapName) {
        return;
      }
    } else if (!options.skipConfirm && !window.confirm("Save local fleet map changes? Use Push Map to apply them to Fleet Manager.")) {
      return;
    }
    try {
      const mapPayload = this.cloneJson(draft);
      if (saveAs) {
        mapPayload.mapName = mapName.replace(/\.smap$/i, "");
      }
      await this.postJson("/api/fleet-manager/maps/local/save", {
        mapName,
        map: mapPayload,
        sourceMapName: this.robotMapState.sourceRobotMapName || draft.mapName || mapName,
        activate: true,
      });
      await this.refreshRobotMapState({ quiet: true });
      this.fleetMapDraft = this.cloneJson(this.operatorMapPayload);
      this.fleetMapDirty = false;
      this.robotMessageText.textContent = `Local fleet map saved: ${this.robotMapState.operatorActiveMapName || mapName}. Push Map will apply it.`;
      this.renderSelectedRobot();
      if (!options.skipPrompt) {
        await this.offerMapSyncDecisionAfterLocalSave({
          message: "Local Fleet Manager map was saved and differs from the active map.",
        });
      }
    } catch (error) {
      this.robotMessageText.textContent = `Save local fleet map failed: ${error.message || error}`;
    }
  }

  async handleLandmarkTarget(lmName) {
    if (this.isFleetManager() && this.pendingFleetAction === "queue") {
      await this.queueFleetGoal(lmName);
      return;
    }
    await this.startNavigation(lmName);
  }

  nearestLandmark(world) {
    let best = null;
    let bestDistance = Number.POSITIVE_INFINITY;
    for (const landmark of this.activeOperatorMapPayload()?.lms || []) {
      const distance = Math.hypot(Number(landmark.x) - world.x, Number(landmark.y) - world.y);
      if (distance < bestDistance) {
        best = landmark;
        bestDistance = distance;
      }
    }
    return best ? { landmark: best, distance: bestDistance } : null;
  }

  screenToSvg(clientX, clientY) {
    const ctm = this.operatorMapSvg.getScreenCTM();
    if (!ctm) {
      return null;
    }
    return new DOMPoint(clientX, clientY).matrixTransform(ctm.inverse());
  }

  zoomMap(factor, anchor = null) {
    const previous = this.mapView.scale;
    const next = Math.max(1, Math.min(9, previous * factor));
    if (Math.abs(next - previous) < 0.001) {
      return;
    }
    const map = this.operatorMapPayload?.map || {};
    const pivot = anchor || {
      x: Number(map.viewWidth || 100) / 2,
      y: Number(map.viewHeight || 100) / 2,
    };
    this.mapView.follow = false;
    this.mapView.tx = pivot.x - ((next / previous) * (pivot.x - this.mapView.tx));
    this.mapView.ty = pivot.y - ((next / previous) * (pivot.y - this.mapView.ty));
    this.mapView.scale = next;
    this.applyMapTransform();
    this.syncMapControls();
  }

  resetMapView(keepFollow = false) {
    this.mapView.scale = 1;
    this.mapView.tx = 0;
    this.mapView.ty = 0;
    this.mapView.follow = keepFollow ? this.mapView.follow : false;
    this.applyMapTransform();
    this.syncMapControls();
  }

  focusMapOn(pixel) {
    const map = this.operatorMapPayload?.map || {};
    const center = {
      x: Number(map.viewWidth || 100) / 2,
      y: Number(map.viewHeight || 100) / 2,
    };
    this.mapView.tx = center.x - (this.mapView.scale * pixel.x);
    this.mapView.ty = center.y - (this.mapView.scale * pixel.y);
    this.applyMapTransform();
  }

  applyMapTransform() {
    this.operatorViewport.setAttribute("transform", `matrix(${this.mapView.scale} 0 0 ${this.mapView.scale} ${this.mapView.tx} ${this.mapView.ty})`);
  }

  syncMapControls() {
    this.operatorFollowRobotButton.classList.toggle("primary", this.mapView.follow);
    this.operatorFollowRobotButton.textContent = this.mapView.follow ? "Following Robot" : "Follow Robot";
  }

  toggleNavigateMode() {
    if (!this.operatorMapPayload || !this.operatorMapPayload.map) {
      this.robotMessageText.textContent = `Pull or load the robot map before ${this.navigateButtonIdleText()}.`;
      return;
    }
    if (this.isFleetManager()) {
      if (this.navigateMode && this.pendingFleetAction === "navigate") {
        this.navigateMode = false;
        this.pendingFleetAction = "";
        this.pendingFleetRobotName = "";
      } else {
        const robot = this.selectedFleetRobot();
        if (!robot) {
          this.robotMessageText.textContent = "Add or select a fleet robot first.";
          return;
        }
        this.navigateMode = true;
        this.pendingFleetAction = "navigate";
        this.pendingFleetRobotName = robot.name;
      }
    } else {
      this.navigateMode = !this.navigateMode;
    }
    this.syncModeButtons();
    this.drawLandmarks();
    const target = this.pendingFleetRobotName || "";
    const targetHint = this.isRos2Robot() && !this.isFleetManager()
      ? "click a map pose or select an LM."
      : "select an LM on the map.";
    this.robotMessageText.textContent = this.navigateMode
      ? (target ? `Navigate armed for ${target}: ${targetHint}` : `Navigate armed: ${targetHint}`)
      : "Navigate canceled.";
  }

  toggleFleetQueueMode() {
    if (!this.isFleetManager()) {
      return;
    }
    if (!this.operatorMapPayload || !this.operatorMapPayload.map) {
      this.robotMessageText.textContent = "Load a fleet map before Queue Goal.";
      return;
    }
    if (this.navigateMode && this.pendingFleetAction === "queue") {
      this.navigateMode = false;
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
    } else {
      const robot = this.selectedFleetRobot();
      if (!robot) {
        this.robotMessageText.textContent = "Add or select a fleet robot first.";
        return;
      }
      this.navigateMode = true;
      this.pendingFleetAction = "queue";
      this.pendingFleetRobotName = robot.name;
    }
    this.syncModeButtons();
    this.drawLandmarks();
    const draftCount = this.fleetQueue.length;
    this.robotMessageText.textContent = this.navigateMode
      ? `Queue armed for ${this.pendingFleetRobotName}: select an LM on the map.`
      : (draftCount ? `Queue selection finished. Draft LM goals: ${draftCount}. Press Dispatch.` : "Queue canceled.");
  }

  syncModeButtons() {
    const isFleet = this.isFleetManager();
    const navigateArmed = this.navigateMode && (!isFleet || this.pendingFleetAction === "navigate");
    const queueArmed = this.navigateMode && isFleet && this.pendingFleetAction === "queue";
    const idleText = this.navigateButtonIdleText();
    this.navigateRobotButton.classList.toggle("primary", !navigateArmed);
    this.navigateRobotButton.classList.toggle("danger", navigateArmed);
    this.navigateRobotButton.textContent = navigateArmed
      ? (this.pendingFleetRobotName ? `Select LM: ${this.pendingFleetRobotName}` : "Cancel Navigate")
      : idleText;
    if (this.fleetQueueGoalButton) {
      this.fleetQueueGoalButton.classList.toggle("primary", queueArmed);
      this.fleetQueueGoalButton.classList.toggle("danger", queueArmed);
      this.fleetQueueGoalButton.textContent = queueArmed
        ? `Queue LM: ${this.pendingFleetRobotName || "robot"}`
        : "Queue Goal";
    }
  }

  navigateButtonIdleText() {
    return this.isRos2Robot() && !this.isFleetManager() ? "Navigate To Pose" : "Navigate To LM";
  }

  async startNavigation(goalLm) {
    if (!this.selectedRobot()) {
      return;
    }
    if (this.isFleetManager()) {
      await this.startFleetNavigation(goalLm);
      return;
    }
    this.navigateMode = false;
    this.syncModeButtons();
    this.releaseManualControl();
    const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    const payload = { goalLm };
    if (robot.pose) {
      payload.startPose = {
        x: Number(robot.pose.x || 0),
        y: Number(robot.pose.y || 0),
        yaw: Number(robot.pose.yaw || 0),
      };
    }
    if (robot.nearestLm) {
      payload.startLm = robot.nearestLm;
    }
    try {
      const result = await this.postJson(this.robotApiPath("/api/robot/route/execute"), payload);
      if (result && result.route) {
        this.currentRoute = result.route;
      }
      this.robotMessageText.textContent = `Route execution started to ${goalLm}.`;
      await this.fetchSelectedRobotStatus(true);
    } catch (error) {
      this.robotMessageText.textContent = `Navigate failed: ${error.message || error}`;
    }
  }

  async startPoseNavigation(world) {
    if (!this.selectedRobot() || this.isFleetManager()) {
      return;
    }
    this.navigateMode = false;
    this.syncModeButtons();
    this.releaseManualControl();
    const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    const yaw = Number(robot.pose?.yaw || 0);
    const goalPose = {
      x: Number(world.x || 0),
      y: Number(world.y || 0),
      yaw: Number.isFinite(yaw) ? yaw : 0,
    };
    try {
      const result = await this.postJson(this.robotApiPath("/api/robot/route/execute"), { goalPose });
      if (result && result.route) {
        this.currentRoute = result.route;
      }
      this.robotMessageText.textContent = `Pose navigation started to x ${goalPose.x.toFixed(3)}, y ${goalPose.y.toFixed(3)}.`;
      await this.fetchSelectedRobotStatus(true);
    } catch (error) {
      this.robotMessageText.textContent = `Navigate failed: ${error.message || error}`;
    }
  }

  async startFleetNavigation(goalLm) {
    const robot = this.targetFleetRobot();
    if (!robot) {
      this.robotMessageText.textContent = "Add or select a fleet robot first.";
      return;
    }
    this.navigateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.syncModeButtons();
    await this.releaseFleetManualControl();
    try {
      const result = await this.postJson("/api/fleet-manager/setOrder", {
        id: this.nextFleetOrderId(robot.name),
        vehicle: robot.name,
        targetLm: goalLm,
        priority: 10,
        speed: this.fleetRouteSpeed(),
        replaceActive: true,
      });
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      this.selectedFleetRobotName = robot.name;
      window.localStorage.setItem("operator:selectedFleetRobotName", this.selectedFleetRobotName);
      this.robotMessageText.textContent = `Order sent: ${robot.name} -> ${goalLm}.`;
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Order failed: ${error.message || error}`;
    }
  }

  fleetPlanRequest(robot, goalLm) {
    const startLm = this.startLmForFleetRobot(robot);
    const request = {
      name: robot.name,
      startLm,
      goalLm,
    };
    if (robot.pose) {
      request.startPose = {
        x: Number(robot.pose.x || 0),
        y: Number(robot.pose.y || 0),
        yaw: Number(robot.pose.yaw || 0),
      };
    }
    return request;
  }

  startLmForFleetRobot(robot) {
    if (robot.currentLm) {
      return robot.currentLm;
    }
    if (robot.pose) {
      const nearest = this.nearestLandmark(robot.pose);
      if (nearest && nearest.landmark) {
        return nearest.landmark.name;
      }
    }
    const first = this.operatorMapPayload?.lms?.[0];
    return first ? first.name : "";
  }

  fleetRouteSpeed() {
    return Math.max(0.02, Number(this.fleetRouteSpeedInput?.value || 0.4) || 0.4);
  }

  fleetManualParams() {
    return {
      linearSpeed: Math.max(0.02, Number(this.fleetManualLinearInput?.value || 0.25) || 0.25),
      angularSpeed: Math.max(0.05, Number(this.fleetManualAngularInput?.value || 0.9) || 0.9),
      predictionTime: Math.max(0.1, Number(this.fleetManualLookaheadInput?.value || 1.0) || 1.0),
      predictionStep: Math.max(0.03, Number(this.fleetManualStepInput?.value || 0.1) || 0.1),
    };
  }

  async ensureFleetParamsLoaded(force = false) {
    if (!this.isFleetManager() || (this.fleetParamsLoaded && !force)) {
      return;
    }
    const payload = await this.getJson("/api/fleet-manager/params");
    this.fleetParams = payload.params || {};
    this.fleetParamsLoaded = true;
    this.applyFleetParams(this.fleetParams);
  }

  async ensureRobotParamsLoaded(force = false) {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      return;
    }
    if (!force && this.robotParamsLoaded && this.robotParamsRobotId === robot.id) {
      return;
    }
    try {
      const payload = await this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/params`);
      this.robotParams = payload.params || {};
      this.robotParamsRobotId = robot.id;
      this.robotParamsLoaded = true;
      this.applyRobotParams(this.robotParams);
    } catch (error) {
      this.robotParams = {};
      this.robotParamsRobotId = robot.id;
      this.robotParamsLoaded = false;
      if (this.fleetModelEditor) {
        this.fleetModelEditor.setModel(this.fleetModelEditor.defaultModel());
      }
      this.syncRobotParamsJson(true);
      if (this.robotMessageText) {
        this.robotMessageText.textContent = `Robot params unavailable: ${error.message || error}`;
      }
    }
  }

  paramsJson(params) {
    return JSON.stringify(params || {}, null, 2);
  }

  syncFleetParamsJson(force = false) {
    if (!this.fleetParamsJsonInput) {
      return;
    }
    if (!force && document.activeElement === this.fleetParamsJsonInput) {
      return;
    }
    this.fleetParamsJsonInput.value = this.paramsJson(this.fleetParams);
  }

  syncRobotParamsJson(force = false) {
    if (!this.robotParamsJsonInput) {
      return;
    }
    if (!force && document.activeElement === this.robotParamsJsonInput) {
      return;
    }
    this.robotParamsJsonInput.value = this.paramsJson(this.robotParams);
  }

  parseParamsJson(input, label, fallback = {}) {
    if (!input || !input.value.trim()) {
      return JSON.parse(JSON.stringify(fallback || {}));
    }
    try {
      const parsed = JSON.parse(input.value);
      if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
        throw new Error(`${label} must be a JSON object.`);
      }
      return parsed;
    } catch (error) {
      throw new Error(`${label} JSON is invalid: ${error.message || error}`);
    }
  }

  formatParamsJson(input, fallback = {}) {
    try {
      const parsed = this.parseParamsJson(input, "Params", fallback);
      input.value = this.paramsJson(parsed);
      this.robotMessageText.textContent = "Params JSON formatted.";
    } catch (error) {
      this.robotMessageText.textContent = error.message || String(error);
    }
  }

  applyFleetParams(params) {
    const navigation = params.navigation || {};
    const manual = params.manual || {};
    const fleet = params.fleet || {};
    if (this.fleetRouteSpeedInput && navigation.route_speed !== undefined) {
      this.fleetRouteSpeedInput.value = String(navigation.route_speed);
    }
    if (this.fleetRobotClearanceInput && fleet.robot_clearance_m !== undefined) {
      this.fleetRobotClearanceInput.value = String(fleet.robot_clearance_m);
    }
    if (this.fleetManualLinearInput && manual.linear_speed !== undefined) {
      this.fleetManualLinearInput.value = String(manual.linear_speed);
    }
    if (this.fleetManualAngularInput && manual.angular_speed !== undefined) {
      this.fleetManualAngularInput.value = String(manual.angular_speed);
    }
    if (this.fleetManualLookaheadInput && manual.prediction_time !== undefined) {
      this.fleetManualLookaheadInput.value = String(manual.prediction_time);
    }
    if (this.fleetManualStepInput && manual.prediction_step !== undefined) {
      this.fleetManualStepInput.value = String(manual.prediction_step);
    }
    this.syncFleetParamsJson();
  }

  applyRobotParams(params) {
    if (this.fleetModelEditor) {
      if (params.robot_model) {
        this.fleetModelEditor.setModel(params.robot_model);
      } else {
        this.fleetModelEditor.setModel(this.fleetModelEditor.defaultModel());
      }
    }
    this.syncRobotParamsJson();
  }

  collectFleetParams() {
    const params = this.parseParamsJson(this.fleetParamsJsonInput, "Fleet params", this.fleetParams || {});
    params.navigation = {
      ...(params.navigation || {}),
      route_speed: this.fleetRouteSpeed(),
    };
    params.fleet = {
      ...(params.fleet || {}),
      robot_clearance_m: Math.max(0.0, Number(this.fleetRobotClearanceInput?.value || 0.35) || 0.35),
    };
    const manual = this.fleetManualParams();
    params.manual = {
      ...(params.manual || {}),
      linear_speed: manual.linearSpeed,
      angular_speed: manual.angularSpeed,
      prediction_time: manual.predictionTime,
      prediction_step: manual.predictionStep,
    };
    return params;
  }

  collectRobotParams() {
    const params = JSON.parse(JSON.stringify(this.robotParams || {}));
    if (this.fleetModelEditor) {
      params.robot_model = {
        ...(params.robot_model || {}),
        ...this.fleetModelEditor.getModel(),
      };
    }
    return params;
  }

  async saveFleetParams() {
    try {
      const params = this.collectFleetParams();
      const result = await this.postJson("/api/fleet-manager/params", { params });
      this.fleetParams = result.params || params;
      this.fleetParamsLoaded = true;
      this.applyFleetParams(this.fleetParams);
      this.syncFleetParamsJson(true);
      this.robotMessageText.textContent = "Fleet params saved.";
    } catch (error) {
      this.robotMessageText.textContent = `Save params failed: ${error.message || error}`;
    }
  }

  async saveFleetJsonParams() {
    try {
      const params = this.parseParamsJson(this.fleetParamsJsonInput, "Fleet params", this.fleetParams || {});
      const result = await this.postJson("/api/fleet-manager/params", { params });
      this.fleetParams = result.params || params;
      this.fleetParamsLoaded = true;
      this.applyFleetParams(this.fleetParams);
      this.syncFleetParamsJson(true);
      this.robotMessageText.textContent = "Fleet params JSON saved.";
    } catch (error) {
      this.robotMessageText.textContent = `Save params failed: ${error.message || error}`;
    }
  }

  async saveRobotParams() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      window.alert("Select a robot before saving robot params.");
      return;
    }
    try {
      const params = this.parseParamsJson(this.robotParamsJsonInput, "Robot params", this.robotParams || {});
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/params`, { params });
      this.robotParams = result.params || result.saved?.params || params;
      this.robotParamsRobotId = robot.id;
      this.robotParamsLoaded = true;
      this.applyRobotParams(this.robotParams);
      this.syncRobotParamsJson(true);
      this.robotMessageText.textContent = "Robot params saved.";
    } catch (error) {
      this.robotMessageText.textContent = `Save robot params failed: ${error.message || error}`;
    }
  }

  async saveRobotModelParams() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      window.alert("Select a robot before saving Robot Model.");
      return;
    }
    try {
      const params = this.collectRobotParams();
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/params`, { params });
      this.robotParams = result.params || result.saved?.params || params;
      this.robotParamsRobotId = robot.id;
      this.robotParamsLoaded = true;
      this.syncRobotParamsJson(true);
      this.robotMessageText.textContent = "Robot model saved.";
    } catch (error) {
      this.robotMessageText.textContent = `Save robot model failed: ${error.message || error}`;
    }
  }

  async handleFleetModeChange() {
    try {
      const nextMode = this.fleetModeSelect.value;
      this.selectedFleetRobotName = "";
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.fleetQueue = [];
      this.fleetManualRobotName = "";
      this.fleetManualAnimation = null;
      window.localStorage.removeItem("operator:selectedFleetRobotName");
      const result = await this.postJson("/api/fleet-manager/mode", { mode: this.fleetModeSelect.value });
      this.currentStatus = {
        ...(this.currentStatus || {}),
        mode: result.mode || nextMode,
        robots: [],
        orders: [],
      };
      await this.refreshRobots({ quiet: true });
      await this.fetchSelectedRobotStatus(true);
      this.renderSelectedRobot();
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async handleFleetAddRobot() {
    const requestedName = String(this.fleetRobotNameInput.value || "").trim();
    const spawnLm = String(this.fleetSpawnLmSelect.value || "").trim();
    const mode = String(this.fleetModeSelect?.value || "simulation");
    const robotIp = String(this.fleetRobotApiInput?.value || "").trim();
    if (mode !== "robots" && !requestedName) {
      window.alert("Robot name is required for simulation robots.");
      return;
    }
    if (mode !== "robots" && !spawnLm) {
      window.alert("Start LM is required for simulation robots.");
      return;
    }
    if (mode === "robots" && !robotIp) {
      window.alert("Robot IP is required in Robots mode.");
      return;
    }
    try {
      const payload = mode === "robots"
        ? { mode: "remote", host: robotIp }
        : { name: requestedName, spawnLm, mode: "simulated" };
      const result = await this.postJson("/api/fleet-manager/robots", payload);
      const addedName = String(result.robot?.name || requestedName || "").trim();
      this.selectedFleetRobotName = addedName;
      if (addedName) {
        window.localStorage.setItem("operator:selectedFleetRobotName", addedName);
      }
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      this.fleetNameEdited = false;
      if (mode === "robots") {
        this.fleetRobotApiInput.value = "";
      } else {
        this.fleetRobotNameInput.value = this.nextFleetRobotName(Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []);
      }
      await this.refreshRobots({ quiet: true });
      this.renderSelectedRobot();
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async handleFleetRemoveRobot(robotName = "") {
    const robot = robotName
      ? (Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []).find((item) => item.name === robotName)
      : this.selectedFleetRobot();
    if (!robot) {
      return;
    }
    const confirmed = window.confirm(`Remove ${robot.name} from Fleet Manager?`);
    if (!confirmed) {
      return;
    }
    try {
      const result = await this.postJson("/api/fleet-manager/robots/remove", { name: robot.name });
      this.selectedFleetRobotName = "";
      window.localStorage.removeItem("operator:selectedFleetRobotName");
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      this.fleetNameEdited = false;
      await this.refreshRobots({ quiet: true });
      this.renderSelectedRobot();
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async stopFleetRobot(all) {
    const robot = this.selectedFleetRobot();
    const payload = all || !robot ? {} : { name: robot.name };
    try {
      const result = await this.postJson("/api/fleet-manager/robots/stop", payload);
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
      this.currentRoute = null;
      this.fleetManualLookahead = null;
      this.fleetManualRobotName = "";
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.renderSelectedRobot();
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async cancelRoute() {
    this.navigateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.syncModeButtons();
    if (this.isFleetManager()) {
      await this.stopFleetRobot(false);
      return;
    }
    try {
      await this.postJson(this.robotApiPath("/api/robot/route/cancel"), {});
      this.currentRoute = null;
      await this.fetchSelectedRobotStatus(true);
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async stopRobot() {
    this.navigateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.releaseManualControl();
    this.syncModeButtons();
    if (this.isFleetManager()) {
      await this.stopFleetRobot(true);
      return;
    }
    try {
      await this.postJson(this.robotApiPath("/api/robot/stop"), {});
      this.currentRoute = null;
      await this.fetchSelectedRobotStatus(true);
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  setManualKey(key, active) {
    if (!["w", "a", "s", "d"].includes(key)) {
      return;
    }
    if (!this.selectedRobot()) {
      return;
    }
    if (active) {
      this.manualKeys.add(key);
      if (this.isFleetManager()) {
        this.navigateMode = false;
        this.pendingFleetAction = "";
        this.pendingFleetRobotName = "";
        this.syncModeButtons();
      }
    } else {
      this.manualKeys.delete(key);
      if (!this.manualKeys.size) {
        if (this.isFleetManager()) {
          this.releaseFleetManualControl().catch(() => {});
        } else {
          this.postJson(this.robotApiPath("/api/robot/teleop/stop"), {}).catch(() => {});
        }
      }
    }
    this.syncManualButtons();
  }

  syncManualButtons() {
    document.querySelectorAll("[data-manual-key]").forEach((button) => {
      button.classList.toggle("active", this.manualKeys.has(button.dataset.manualKey));
    });
  }

  manualTwist() {
    const manual = this.isFleetManager()
      ? this.fleetManualParams()
      : { linearSpeed: 0.25, angularSpeed: 0.9 };
    const linearSpeed = manual.linearSpeed;
    const angularSpeed = manual.angularSpeed;
    const forward = this.manualKeys.has("w") ? 1 : 0;
    const backward = this.manualKeys.has("s") ? 1 : 0;
    const left = this.manualKeys.has("a") ? 1 : 0;
    const right = this.manualKeys.has("d") ? 1 : 0;
    return {
      linear: (forward - backward) * linearSpeed,
      angular: (left - right) * angularSpeed,
      params: manual,
    };
  }

  async sendTeleopIfNeeded() {
    if (!this.manualKeys.size || this.teleopPending || !this.selectedRobot()) {
      return;
    }
    const twist = this.manualTwist();
    if (Math.abs(twist.linear) < 0.0001 && Math.abs(twist.angular) < 0.0001) {
      return;
    }
    this.teleopPending = true;
    try {
      if (this.isFleetManager()) {
        await this.sendFleetManualStep(twist);
        return;
      }
      await this.postJson(this.robotApiPath("/api/robot/teleop"), {
        linear: twist.linear,
        angular: twist.angular,
        timeoutMs: 350,
      });
    } finally {
      this.teleopPending = false;
    }
  }

  releaseManualControl() {
    this.manualKeys.clear();
    this.syncManualButtons();
    if (this.selectedRobot() && !this.isFleetManager()) {
      this.postJson(this.robotApiPath("/api/robot/teleop/stop"), {}).catch(() => {});
    }
    if (this.isFleetManager()) {
      this.releaseFleetManualControl().catch(() => {});
    }
  }

  async sendFleetManualStep(twist) {
    const robot = this.selectedFleetRobot();
    if (!robot) {
      this.robotMessageText.textContent = "Select a fleet robot for manual control.";
      return;
    }
    if (this.isFleetRobotsMode()) {
      await this.sendFleetRemoteTeleop(robot, twist);
      return;
    }
    if (this.fleetManualRobotName !== robot.name) {
      await this.postJson("/api/fleet-manager/robots/stop", { name: robot.name });
      this.fleetManualRobotName = robot.name;
      this.fleetManualLastAt = performance.now();
      this.currentStatus = await this.getJson("/api/fleet-manager/state");
    }
    const pose = this.animatedFleetManualPose(robot) || robot.pose || this.poseForLm(robot.currentLm);
    if (!pose) {
      this.robotMessageText.textContent = `${robot.name}: no pose for manual control.`;
      return;
    }
    const now = performance.now();
    const dt = Math.min(0.16, Math.max(0.02, (now - (this.fleetManualLastAt || now)) / 1000));
    this.fleetManualLastAt = now;

    const prediction = this.predictManualTrajectory(
      pose,
      twist.linear,
      twist.angular,
      twist.params.predictionTime,
      twist.params.predictionStep
    );
    const nextPose = this.integratePose(pose, twist.linear, twist.angular, dt);
    const currentLm = this.currentLmForPose(nextPose, 0.25);
    const result = await this.postJson("/api/fleet-manager/manual-step", {
      name: robot.name,
      poses: prediction,
      blockedPose: pose,
      nextPose,
      blockedCurrentLm: this.currentLmForPose(pose, 0.25),
      currentLm,
    });
    this.fleetManualLookahead = {
      poses: prediction,
      blocked: Boolean(result.blocked),
      reason: result.reason || "",
    };
    this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
    this.fleetStatusReceivedAt = performance.now();
    this.fleetStatusObjectRef = this.currentStatus;
    if (result.blocked) {
      this.fleetManualAnimation = null;
      this.robotMessageText.textContent = `${robot.name} manual blocked: ${result.reason || "collision"}.`;
      this.renderFleetRuntimeTick();
      return;
    }
    this.setFleetManualAnimation(robot.name, pose, twist);
    this.robotMessageText.textContent = `${robot.name} manual control active.`;
    this.renderFleetRuntimeTick();
  }

  async sendFleetRemoteTeleop(robot, twist) {
    if (!robot.baseUrl) {
      this.robotMessageText.textContent = `${robot.name}: Robot IP/API URL is missing.`;
      return;
    }
    if (this.fleetManualRobotName !== robot.name) {
      this.fleetManualRobotName = robot.name;
      this.fleetManualLastAt = performance.now();
      this.fleetManualLookahead = null;
      this.fleetManualAnimation = null;
    }
    const result = await this.postJson("/api/fleet-manager/manual-step", {
      name: robot.name,
      linear: twist.linear,
      angular: twist.angular,
      timeoutMs: 350,
    });
    this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
    this.fleetStatusReceivedAt = performance.now();
    this.fleetStatusObjectRef = this.currentStatus;
    this.robotMessageText.textContent = `${robot.name} remote manual control active.`;
    this.renderFleetRuntimeTick();
  }

  async releaseFleetManualControl() {
    if (!this.fleetManualRobotName) {
      this.fleetManualLookahead = null;
      this.renderOperatorMap();
      return;
    }
    const robot = this.selectedFleetRobot();
    if (robot && robot.name === this.fleetManualRobotName) {
      if (this.isFleetRobotsMode()) {
        const result = await this.postJson("/api/fleet-manager/manual-stop", { name: robot.name });
        this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
        this.fleetStatusReceivedAt = performance.now();
        this.fleetStatusObjectRef = this.currentStatus;
        this.fleetManualRobotName = "";
        this.fleetManualLastAt = 0;
        this.fleetManualLookahead = null;
        this.fleetManualAnimation = null;
        this.renderSelectedRobot();
        return;
      }
      const pose = robot.pose || null;
      const payload = {
        name: robot.name,
        status: "IDLE",
        targetLm: "",
        currentLm: pose ? this.currentLmForPose(pose, 0.25) : (robot.currentLm || ""),
      };
      if (pose) {
        payload.pose = pose;
      }
      const result = await this.postJson("/api/fleet-manager/robots/update", payload);
      this.currentStatus = result.state || await this.getJson("/api/fleet-manager/state");
    }
    this.fleetManualRobotName = "";
    this.fleetManualLastAt = 0;
    this.fleetManualLookahead = null;
    this.fleetManualAnimation = null;
    this.renderSelectedRobot();
  }

  predictManualTrajectory(pose, linear, angular, horizon, step) {
    const poses = [{ ...pose }];
    let current = { ...pose };
    for (let elapsed = 0; elapsed < horizon; elapsed += step) {
      current = this.integratePose(current, linear, angular, step);
      poses.push(current);
    }
    return poses;
  }

  integratePose(pose, linear, angular, dt) {
    const yaw = this.normalizeAngle(Number(pose.yaw || 0) + (angular * dt));
    const midYaw = this.normalizeAngle(Number(pose.yaw || 0) + ((angular * dt) / 2));
    return {
      x: Number(pose.x || 0) + (linear * Math.cos(midYaw) * dt),
      y: Number(pose.y || 0) + (linear * Math.sin(midYaw) * dt),
      yaw,
    };
  }

  normalizeAngle(angle) {
    let value = Number(angle || 0);
    while (value > Math.PI) {
      value -= Math.PI * 2;
    }
    while (value < -Math.PI) {
      value += Math.PI * 2;
    }
    return value;
  }

  poseForLm(lmName) {
    const landmark = (this.operatorMapPayload?.lms || []).find((lm) => lm.name === lmName);
    return landmark ? { x: Number(landmark.x || 0), y: Number(landmark.y || 0), yaw: 0 } : null;
  }

  currentLmForPose(pose, tolerance = 0.25) {
    const nearest = this.nearestLandmark(pose);
    return nearest && nearest.distance <= tolerance ? nearest.landmark.name : "";
  }

  renderSidebar() {
    this.sidebarDrawer.classList.toggle("open", this.sidebarOpen);
    this.sidebarBackdrop.classList.toggle("open", this.sidebarOpen);
  }

  openSidebar() {
    this.sidebarOpen = true;
    this.renderSidebar();
  }

  closeSidebar() {
    this.sidebarOpen = false;
    this.renderSidebar();
  }

  async handleEditMapButton() {
    if (this.isFleetManager()) {
      if (this.fleetActiveTab === "map" && this.fleetMapDirty) {
        const shouldSave = window.confirm("Save fleet map changes before closing the editor?");
        if (shouldSave) {
          await this.saveFleetMap(false, { skipConfirm: true });
        } else {
          this.fleetMapDraft = this.cloneJson(this.operatorMapPayload);
          this.fleetMapDirty = false;
          this.fleetSelectedLmName = "";
          this.fleetSelectedEdgeKey = "";
          this.syncFleetEditorFields();
        }
      }
      await this.navigateFleetPage(this.fleetActiveTab === "map" ? "fleet" : "map");
      this.renderMapSyncStatus();
      return;
    }
    this.openMapEditor();
  }

  openAddRobotDialog() {
    this.lastProbe = null;
    this.robotNameInput.value = "";
    this.robotHostInput.value = "";
    if (this.robotDomainInput) {
      this.robotDomainInput.value = "0";
    }
    this.robotPortInput.value = "8790";
    this.showProbeResult("neutral", "Enter the robot IP and ROS domain, then check the connection.");
    this.addRobotDialog.showModal();
  }

  async handleProbe() {
    const payload = this.dialogPayload();
    this.showProbeResult("neutral", `Checking ${payload.host} ...`);
    try {
      const result = await this.postJson("/api/robots/probe", payload);
      this.lastProbe = result.probe;
      const identity = result.probe.identity || {};
      const status = result.probe.status || {};
      const online = result.probe.online ? "online" : "waiting for /robot_status";
      this.showProbeResult("success", `ROS2 bridge ready for ${identity.robotId || "robot"} on map ${identity.mapId || "-"}. ${online}. State: ${status.state || "-"}`);
      if (!this.robotNameInput.value.trim()) {
        this.robotNameInput.value = identity.robotId || "";
      }
    } catch (error) {
      this.lastProbe = null;
      this.showProbeResult("error", error.message || String(error));
    }
  }

  async handleSaveRobot() {
    const payload = this.dialogPayload();
    try {
      const result = await this.postJson("/api/robots", payload);
      this.addRobotDialog.close();
      this.selectedRobotId = result.robot.id;
      window.localStorage.setItem("operator:selectedRobotId", this.selectedRobotId);
      this.closeSidebar();
      await this.refreshRobots({ quiet: true });
      this.showProbeResult("success", "Robot saved. Use Pull Map when you want to copy its active map into the operator cache.");
    } catch (error) {
      this.showProbeResult("error", error.message || String(error));
    }
  }

  async handleRemoveRobot(robot) {
    if (!robot) {
      return;
    }
    const confirmed = window.confirm(`Remove ${robot.name || robot.id} from the operator app?`);
    if (!confirmed) {
      return;
    }
    try {
      await this.deleteJson(`/api/robots/${encodeURIComponent(robot.id)}`);
      if (this.selectedRobotId === robot.id) {
        this.selectedRobotId = "";
        window.localStorage.removeItem("operator:selectedRobotId");
      }
      await this.refreshRobots({ quiet: true });
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  openMapEditor() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    if (this.isFleetManager(robot)) {
      this.navigateFleetPage("map");
      return;
    }
    const robotName = robot.name || robot.identity?.robotId || robot.id;
    const url = `/map-editor.html?robot_id=${encodeURIComponent(robot.id)}&robot_name=${encodeURIComponent(robotName)}`;
    window.location.assign(url);
  }

  async offerMapSyncDecisionAfterLocalSave(context = {}) {
    await this.refreshRobotMapState({ quiet: true });
    this.renderSelectedRobot();
    if (!this.robotMapState.hasLocalChanges) {
      return "none";
    }
    const decision = await this.promptMapSyncDecision(context);
    if (decision === "push") {
      await this.handlePushMap({ skipConfirm: true });
    } else if (decision === "pull") {
      await this.handlePullMap({ skipConfirm: true });
    } else {
      this.robotMessageText.textContent = "Local map saved. Push Map is available when you are ready.";
    }
    return decision;
  }

  promptMapSyncDecision(context = {}) {
    if (!this.mapSyncDecisionDialog || typeof this.mapSyncDecisionDialog.showModal !== "function") {
      const shouldPush = window.confirm("Local map differs from the robot map. Push local changes now?");
      return Promise.resolve(shouldPush ? "push" : "cancel");
    }
    const robot = this.selectedRobot();
    const target = this.isFleetManager(robot) ? "Fleet Manager" : "robot";
    const localName = this.robotMapState.operatorActiveMapName || "-";
    const remoteName = this.robotMapState.robotActiveMapName || this.robotMapState.sourceRobotMapName || "-";
    this.mapSyncDecisionTitle.textContent = "Inconsistent Map Data";
    this.mapSyncDecisionText.textContent = context.message || `Operator local map differs from the active ${target} map.`;
    this.mapSyncDecisionDetail.textContent = `Local: ${localName}. ${target}: ${remoteName}. Choose Push to overwrite the ${target} map, Pull to replace the local draft, or Cancel to keep driving with the current ${target} map.`;
    return new Promise((resolve) => {
      this.mapSyncDecisionResolve = resolve;
      this.mapSyncDecisionDialog.showModal();
    });
  }

  resolveMapSyncDecision(decision) {
    if (!this.mapSyncDecisionResolve) {
      return;
    }
    const resolve = this.mapSyncDecisionResolve;
    this.mapSyncDecisionResolve = null;
    if (this.mapSyncDecisionDialog.open) {
      this.mapSyncDecisionDialog.close();
    }
    resolve(decision);
  }

  async runMapTransfer(kind, callback) {
    const title = kind === "push" ? "Push Map" : "Pull Map";
    this.openMapTransfer(title);
    try {
      await this.setMapTransferProgress(5, "Preparing map transfer...", 100);
      const result = await callback((percent, status, delayMs = 0) => this.setMapTransferProgress(percent, status, delayMs));
      await this.setMapTransferProgress(100, `${title} completed.`, 450);
      this.finishMapTransfer(false);
      return result;
    } catch (error) {
      await this.setMapTransferProgress(100, error.message || String(error), 0);
      this.finishMapTransfer(true);
      throw error;
    }
  }

  openMapTransfer(title) {
    if (this.mapTransferCloseTimer) {
      window.clearTimeout(this.mapTransferCloseTimer);
      this.mapTransferCloseTimer = null;
    }
    this.mapTransferTitle.textContent = title;
    this.mapTransferDialog.querySelector(".dialog-card").classList.add("busy");
    this.mapTransferDialog.querySelector(".dialog-card").classList.remove("error");
    this.mapTransferCloseButton.disabled = true;
    this.setMapTransferProgress(0, "Preparing...", 0);
    if (!this.mapTransferDialog.open && typeof this.mapTransferDialog.showModal === "function") {
      this.mapTransferDialog.showModal();
    }
  }

  async setMapTransferProgress(percent, status, delayMs = 0) {
    const value = Math.max(0, Math.min(100, Math.round(Number(percent || 0))));
    this.mapTransferPercent.textContent = `${value}%`;
    this.mapTransferBar.style.width = `${value}%`;
    this.mapTransferStatus.textContent = status;
    if (delayMs > 0) {
      await new Promise((resolve) => window.setTimeout(resolve, delayMs));
    }
  }

  finishMapTransfer(error) {
    const card = this.mapTransferDialog.querySelector(".dialog-card");
    card.classList.remove("busy");
    card.classList.toggle("error", Boolean(error));
    this.mapTransferCloseButton.disabled = false;
    if (!error) {
      this.mapTransferCloseTimer = window.setTimeout(() => {
        if (this.mapTransferDialog.open) {
          this.mapTransferDialog.close();
        }
      }, 700);
    }
  }

  async handlePullMap(options = {}) {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    const target = this.isFleetManager(robot) ? "Fleet Manager" : "robot";
    if (!options.skipConfirm) {
      const confirmed = window.confirm(`Pull active ${target} map into the operator cache? Local draft changes may be replaced.`);
      if (!confirmed) {
        return;
      }
    }
    try {
      const result = await this.runMapTransfer("pull", async (progress) => {
        await progress(18, `Requesting active map from ${target}...`, 120);
        const payload = this.isFleetManager(robot)
          ? await this.postJson("/api/fleet-manager/maps/pull-sync", {})
          : await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/pull-sync`, {});
        await progress(72, "Saving local operator copy...", 120);
        await this.refreshRobotMapState({ quiet: true });
        await progress(90, "Refreshing map view...", 80);
        await this.fetchSelectedRobotStatus(true);
        return payload;
      });
      this.clearSelectedPendingPush();
      this.renderSelectedRobot();
      this.robotMessageText.textContent = result.message || "Pull map completed.";
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async handlePushMap(options = {}) {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    const target = this.isFleetManager(robot) ? "Fleet Manager" : "robot";
    if (!options.skipConfirm) {
      const confirmed = window.confirm(`Push local operator map to ${target}? This overwrites the active map used by ${target}.`);
      if (!confirmed) {
        return;
      }
    }
    try {
      const result = await this.runMapTransfer("push", async (progress) => {
        await progress(16, "Preparing local map package...", 120);
        const payload = this.isFleetManager(robot)
          ? await this.postJson("/api/fleet-manager/maps/push-sync", {})
          : await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/push-sync`, {});
        await progress(74, `Writing map to ${target}...`, 120);
        await this.refreshRobotMapState({ quiet: true });
        await progress(90, "Refreshing operator state...", 80);
        await this.refreshRobots({ quiet: true, lightweight: true });
        await this.fetchSelectedRobotStatus(true);
        return payload;
      });
      this.clearSelectedPendingPush();
      this.renderSelectedRobot();
      this.robotMessageText.textContent = result.message || "Push map completed.";
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  clearSelectedPendingPush() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    const pendingRobotId = window.sessionStorage.getItem("operator:pendingPushRobotId") || "";
    if (pendingRobotId === robot.id) {
      window.sessionStorage.removeItem("operator:pendingPushRobotId");
    }
  }

  async maybePromptPendingPush() {
    const pendingRobotId = window.sessionStorage.getItem("operator:pendingPushRobotId") || "";
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot) || !pendingRobotId || pendingRobotId !== robot.id) {
      return;
    }
    window.sessionStorage.removeItem("operator:pendingPushRobotId");
    if (!this.robotMapState.hasLocalChanges) {
      return;
    }
    const decision = await this.promptMapSyncDecision({
      message: "Local map draft was saved and differs from the robot map.",
    });
    if (decision === "push") {
      await this.handlePushMap({ skipConfirm: true });
    } else if (decision === "pull") {
      await this.handlePullMap({ skipConfirm: true });
    } else {
      this.robotMessageText.textContent = "Map push skipped. Use Push Map when you are ready.";
    }
  }

  async handleLoadMap() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    try {
      const robotMaps = this.isFleetManager(robot)
        ? await this.getJson("/api/fleet-manager/maps/list")
        : await this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/list`);
      const maps = Array.isArray(robotMaps.maps) ? robotMaps.maps : [];
      if (!maps.length) {
        window.alert(this.isFleetManager(robot) ? "Fleet Manager has no maps." : "Robot has no editable maps.");
        return;
      }
      this.pendingRobotMaps = maps;
      this.loadMapSelect.innerHTML = "";
      for (const item of maps) {
        const option = document.createElement("option");
        const name = item.name || item.folder || "";
        option.value = name;
        option.textContent = item.active ? `${name} (active)` : `${name}`;
        option.selected = Boolean(item.active) || option.value === this.robotMapState.robotActiveMapName;
        this.loadMapSelect.appendChild(option);
      }
      this.loadMapHint.className = "probe-result neutral";
      this.loadMapHint.textContent = this.isFleetManager(robot)
        ? "Choose one of the maps available in Fleet Manager."
        : "Choose one of the maps available on the robot.";
      this.loadMapDialog.showModal();
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async confirmLoadMap() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    const mapName = String(this.loadMapSelect.value || "").trim();
    if (!mapName) {
      this.loadMapHint.className = "probe-result error";
      this.loadMapHint.textContent = "Select a map first.";
      return;
    }
    try {
      let result = null;
      if (this.isFleetManager(robot)) {
        result = await this.postJson("/api/fleet-manager/maps/load", { mapName });
      } else {
        result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/load`, { mapName });
      }
      this.loadMapDialog.close();
      await this.refreshRobotMapState({ quiet: true });
      await this.refreshRobots({ quiet: true });
      await this.fetchSelectedRobotStatus(true);
      window.alert(`${this.isFleetManager(robot) ? "Fleet Manager" : "Robot"} active map changed to ${result.mapName || mapName}.`);
    } catch (error) {
      this.loadMapHint.className = "probe-result error";
      this.loadMapHint.textContent = error.message || String(error);
    }
  }

  dialogPayload() {
    return {
      type: "ros2",
      name: this.robotNameInput.value.trim(),
      host: this.robotHostInput.value.trim(),
      domainId: Number(this.robotDomainInput?.value || 0),
      port: Number(this.robotPortInput.value || 8790),
    };
  }

  showProbeResult(kind, text) {
    this.probeResult.className = `probe-result ${kind}`;
    this.probeResult.textContent = text;
  }

  async getJson(url) {
    const response = await fetch(url, { cache: "no-store" });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || payload.ok === false) {
      throw new Error(payload.error || `Request failed: ${response.status}`);
    }
    return payload;
  }

  async postJson(url, payload) {
    const response = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok || data.ok === false) {
      throw new Error(data.error || `Request failed: ${response.status}`);
    }
    return data;
  }

  async deleteJson(url) {
    const response = await fetch(url, { method: "DELETE" });
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
  const app = new OperatorApp();
  app.init().catch((error) => {
    window.alert(error.message || String(error));
  });
});
