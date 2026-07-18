import { preferences } from "../state/preferences.js";
import { FLEET_RASTER_TOOLS } from "./constants.js";


export const withSceneNavigation = (Base) => class OperatorAppSceneNavigation extends Base {
  async handleLandmarkTarget(lmName) {
    if (this.isFleetManager() && this.pendingFleetAction === "spawn") {
      await this.spawnFleetRobotAtLm(lmName);
      return;
    }
    if (this.isFleetManager() && this.pendingFleetAction === "queue") {
      await this.queueFleetGoal(lmName);
      return;
    }
    await this.startNavigation(lmName);
  }

  fleetTargetActionLabel() {
    if (this.pendingFleetAction === "queue") {
      return "Queue";
    }
    if (this.pendingFleetAction === "spawn") {
      return "Place robot";
    }
    return "Navigate";
  }

  nearestLandmark(world) {
    let best = null;
    let bestDistance = Number.POSITIVE_INFINITY;
    for (const landmark of this.navigationLandmarks()) {
      const distance = Math.hypot(Number(landmark.x) - world.x, Number(landmark.y) - world.y);
      if (distance < bestDistance) {
        best = landmark;
        bestDistance = distance;
      }
    }
    return best ? { landmark: best, distance: bestDistance } : null;
  }

  navigationLandmarks() {
    const fromMap = this.activeOperatorMapPayload()?.lms;
    if (Array.isArray(fromMap) && fromMap.length) {
      return fromMap;
    }
    const sceneManagerId = String(this.scene3dPayload?.managerId || "");
    const selectedManagerId = String(this.selectedRobot()?.id || "");
    if (sceneManagerId && selectedManagerId && sceneManagerId !== selectedManagerId) {
      return [];
    }
    const fromScene = this.scene3dPayload?.lms;
    return Array.isArray(fromScene) ? fromScene : [];
  }

  hasNavigationMapPayload() {
    if (this.isFleetManager()) {
      return Boolean(this.operatorMapPayload?.map) || this.navigationLandmarks().length > 0;
    }
    return Boolean(this.operatorMapPayload?.map);
  }

  screenToSvg(clientX, clientY) {
    const ctm = this.operatorMapSvg.getScreenCTM();
    if (!ctm) {
      return null;
    }
    return new DOMPoint(clientX, clientY).matrixTransform(ctm.inverse());
  }

  screenToMapPixel(clientX, clientY) {
    const point = this.screenToSvg(clientX, clientY);
    if (!point) {
      return null;
    }
    return {
      x: (point.x - this.mapView.tx) / this.mapView.scale,
      y: (point.y - this.mapView.ty) / this.mapView.scale,
    };
  }

  zoomMap(factor, anchor = null) {
    if (this.scene3d && !this.babylonMapFailed && !this.slamActive) {
      this.mapView.follow = false;
      this.scene3d.zoomBy(factor);
      this.syncMapControls();
      return;
    }
    const previous = this.mapView.scale;
    const next = Math.max(1, Math.min(9, previous * factor));
    if (Math.abs(next - previous) < 0.001) {
      return;
    }
    const map = this.activeOperatorMapPayload()?.map || {};
    const pivot = anchor || {
      x: Number(map.viewWidth || 100) / 2,
      y: Number(map.viewHeight || 100) / 2,
    };
    this.mapView.follow = false;
    this.mapView.tx = pivot.x - ((next / previous) * (pivot.x - this.mapView.tx));
    this.mapView.ty = pivot.y - ((next / previous) * (pivot.y - this.mapView.ty));
    this.mapView.scale = next;
    this.applyMapTransform();
    this.scheduleAdaptiveMapLayers();
    this.syncMapControls();
  }

  resetMapView(keepFollow = false) {
    this.mapView.scale = 1;
    this.mapView.tx = 0;
    this.mapView.ty = 0;
    this.mapView.follow = keepFollow ? this.mapView.follow : false;
    if (this.scene3d && !this.babylonMapFailed && !this.slamActive) {
      this.scene3d.resetView();
      this.syncMapControls();
      return;
    }
    this.applyMapTransform();
    this.refreshAdaptiveMapLayers();
    this.syncMapControls();
  }

  focusMapOn(pixel) {
    const map = this.activeOperatorMapPayload()?.map || {};
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
    const hasMap = Boolean(this.activeOperatorMapPayload()?.map);
    const canUse3d = hasMap
      && !this.isRobotModelPage()
      && !this.isParamsPage()
      && !this.fleetMapEditorActive
      && !this.slamActive;
    const show3d = canUse3d && this.mapViewMode === "3d";
    // Keep the working SVG map visible while the Babylon module/runtime is
    // still loading. Hiding it earlier produces a blank workspace on a slow
    // or offline CDN and used to make the fleet appear frozen until F5.
    const useBabylon = hasMap
      && !this.babylonMapFailed
      && !this.slamActive
      && Boolean(this.scene3d?.scene);
    this.operatorMapSvg?.classList.toggle("hidden", useBabylon);
    this.operatorScene3d?.classList.toggle("hidden", !useBabylon);
    this.operatorMap2dButton?.classList.toggle("active", !show3d);
    this.operatorMap3dButton?.classList.toggle("active", show3d);
    this.operatorMap3dButton?.classList.toggle("hidden", !canUse3d);
    this.scene3d?.setViewMode(show3d ? "3d" : "2d");
    this.scene3d?.setLandmarkLabelsVisible(this.lmNamesVisible);
    this.scene3d?.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
    this.operatorLmNamesButton?.classList.toggle("active", this.lmNamesVisible);
    if (this.operatorLmNamesButton) {
      this.operatorLmNamesButton.textContent = `LM names: ${this.lmNamesVisible ? "On" : "Off"}`;
      this.operatorLmNamesButton.setAttribute("aria-pressed", String(this.lmNamesVisible));
    }
    this.operatorEdgeDirectionsButton?.classList.toggle("active", this.edgeDirectionsVisible);
    if (this.operatorEdgeDirectionsButton) {
      this.operatorEdgeDirectionsButton.textContent = `Edge directions: ${this.edgeDirectionsVisible ? "On" : "Off"}`;
      this.operatorEdgeDirectionsButton.setAttribute("aria-pressed", String(this.edgeDirectionsVisible));
    }
    this.operatorFollowRobotButton.classList.toggle("primary", this.mapView.follow);
    this.operatorFollowRobotButton.textContent = this.mapView.follow ? "Following Robot" : "Follow Robot";
  }

  toggleLmNames() {
    this.lmNamesVisible = !this.lmNamesVisible;
    preferences.setBoolean("lmNamesVisible", this.lmNamesVisible);
    this.scene3d?.setLandmarkLabelsVisible(this.lmNamesVisible);
    this.syncMapControls();
    this.drawLandmarks();
  }

  toggleEdgeDirections() {
    this.edgeDirectionsVisible = !this.edgeDirectionsVisible;
    preferences.setBoolean("edgeDirectionsVisible", this.edgeDirectionsVisible);
    this.scene3d?.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
    this.syncMapControls();
    this.drawGraph();
  }

  setMapViewMode(mode) {
    const nextMode = mode === "3d" ? "3d" : "2d";
    if (nextMode === "3d" && this.fleetMapEditorActive) {
      this.robotMessageText.textContent = "Map Editor is available only in the 2D Babylon view.";
      return;
    }
    if (nextMode === "3d" && (!this.activeOperatorMapPayload()?.map || this.slamActive)) {
      this.robotMessageText.textContent = "3D view requires a loaded static map.";
      return;
    }
    this.mapViewMode = nextMode;
    preferences.setString("mapViewMode", this.mapViewMode);
    this.syncMapControls();
    this.renderOperatorMap();
  }

  async ensureScene3d() {
    if (this.scene3d) {
      await this.scene3d.readyPromise;
      if (this.scene3d.initError || !this.scene3d.scene) {
        throw this.scene3d.initError || new Error("Babylon.js did not initialize a scene.");
      }
      return this.scene3d;
    }
    if (!this.scene3dModulePromise) {
      this.scene3dModulePromise = import("../../scene3d.js");
    }
    const module = await this.scene3dModulePromise;
    this.scene3d = new module.OperatorScene3D(this.operatorScene3d);
    this.scene3d.setHandlers({
      onFloorClick: (world) => this.handleScene3dFloorClick(world),
      onLandmarkHover: (lmName) => this.handleScene3dLandmarkHover(lmName),
      onRobotClick: (robotName) => {
        if (this.isFleetManager()) {
          this.selectFleetRobotByName(robotName);
        }
      },
      onPointerDown: (hit) => this.handleBabylonMapPointerDown(hit),
      onPointerMove: (hit) => this.handleBabylonMapPointerMove(hit),
      onPointerUp: (hit) => this.handleBabylonMapPointerUp(hit),
      onContextMenu: (hit) => this.handleBabylonMapContextMenu(hit),
    });
    this.scene3d.setViewMode(this.mapViewMode);
    this.scene3d.setLandmarkLabelsVisible(this.lmNamesVisible);
    this.scene3d.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
    this.scene3d.setTargetArmed(this.scene3dTargetArmed());
    await this.scene3d.readyPromise;
    if (this.scene3d.initError || !this.scene3d.scene) {
      throw this.scene3d.initError || new Error("Babylon.js did not initialize a scene.");
    }
    return this.scene3d;
  }

  operatorBabylonScenePayload() {
    const payload = this.activeOperatorMapPayload();
    const map = payload?.map || {};
    const resolution = Math.max(0.000001, Number(map.resolution || 1));
    const width = Math.max(1, Number(map.width || 0) * resolution || Number(map.viewWidth || 1) * resolution);
    const depth = Math.max(1, Number(map.height || 0) * resolution || Number(map.viewHeight || 1) * resolution);
    const lms = Array.isArray(payload?.lms) ? payload.lms : [];
    const lmIndex = new Map(lms.map((lm) => [String(lm.name || ""), lm]));
    const edges = (Array.isArray(payload?.edges) ? payload.edges : []).map((edge) => {
      if (
        (Array.isArray(edge.world_points) && edge.world_points.length >= 2)
        || (Array.isArray(edge.control_points) && edge.control_points.length === 4)
      ) {
        return edge;
      }
      const from = lmIndex.get(String(edge.from || ""));
      const to = lmIndex.get(String(edge.to || ""));
      return from && to
        ? {
            ...edge,
            world_points: [
              { x: Number(from.x || 0), y: Number(from.y || 0) },
              { x: Number(to.x || 0), y: Number(to.y || 0) },
            ],
          }
        : edge;
    });
    return {
      ok: true,
      managerId: String(this.selectedRobot()?.id || ""),
      mapName: payload?.mapName || this.robotMapState.operatorActiveMapName || "operator-map",
      coordinateFrame: "map_top_left",
      floor: {
        width,
        depth,
        resolution,
        imageDataUrl: String(map.imageDataUrl || ""),
      },
      bounds: { minX: 0, minZ: 0, maxX: width, maxZ: depth },
      // The shared Babylon renderer converts black occupied PGM cells into
      // merged vertical wall instances only when the 3D view is opened.
      walls: [],
      wallHeight: 1.8,
      lms,
      edges,
    };
  }

  operatorBabylonStaticKey() {
    const geometryDragging = Boolean(
      this.fleetEditorLmDrag
      || this.fleetEditorEdgeDrag
      || this.fleetEditorBezierDrag
    );
    if (geometryDragging && this.scene3dStaticKey) {
      return this.scene3dStaticKey;
    }
    const source = this.mapViewMode === "3d" && this.isFleetManager() ? "fleet-3d" : "map";
    return [
      source,
      this.scene3dKey(),
      this.fleetMapEditorActive && this.fleetMapDirty
        ? `draft-${this.babylonMapRevision}`
        : "saved",
      this.activeOperatorMapPayload()?.lms?.length || 0,
      this.activeOperatorMapPayload()?.edges?.length || 0,
    ].join(":");
  }

  operatorBabylonRobots() {
    if (this.fleetMapEditorActive) {
      return [];
    }
    if (this.isFleetManager()) {
      const footprint = this.robotModelFootprint();
      return this.fleetRenderRobots().map((robot) => ({
        ...robot,
        footprint,
      }));
    }
    const selected = this.selectedRobot();
    const statusRobot = this.statusForRobotDisplay(this.currentStatus?.robot || {});
    const rawPose = this.slamActive && this.slamMapFrame?.pose
      ? this.slamMapFrame.pose
      : statusRobot.pose;
    const pose = rawPose ? this.displayPoseForActiveMap(rawPose) : null;
    if (!selected || !pose) {
      return [];
    }
    const route = this.currentStatus?.route || this.currentRoute || {};
    return [{
      name: statusRobot.robotId || selected.name || selected.id,
      pose,
      status: statusRobot.state || "IDLE",
      currentLm: statusRobot.nearestLm || "",
      targetLm: statusRobot.targetLm || "",
      trajectory: Array.isArray(route.trajectory) ? route.trajectory : [],
      routePreview: [],
      routeRevision: route.revision || route.routeRevision || 0,
      routeClock: statusRobot.routeClock || 0,
      reason: statusRobot.message || "",
      footprint: this.robotModelFootprint(),
    }];
  }

  babylonEditorState() {
    const payload = this.activeOperatorMapPayload() || {};
    const dragging = Boolean(
      this.fleetEditorLmDrag
      || this.fleetEditorEdgeDrag
      || this.fleetEditorBezierDrag
      || this.fleetRasterDrag
    );
    return {
      active: Boolean(this.fleetMapEditorActive && this.mapViewMode === "2d"),
      revision: this.babylonMapRevision,
      dragging,
      tool: this.fleetMapTool,
      selectedLmName: this.fleetSelectedLmName,
      selectedEdgeKey: this.fleetSelectedEdgeKey,
      preview: this.fleetEditorPreview,
      lms: Array.isArray(payload.lms) ? payload.lms : [],
      edges: Array.isArray(payload.edges) ? payload.edges : [],
    };
  }

  refreshBabylonEditorState() {
    this.scene3d?.setEditorState(this.babylonEditorState());
  }

  renderOperatorBabylonMap(options = {}) {
    if (!this.activeOperatorMapPayload()?.map || this.babylonMapFailed || this.slamActive) {
      return;
    }
    if (this.scene3dLoadPending) {
      this.scene3dRenderQueued = true;
      return;
    }
    const requestedKey = this.operatorBabylonStaticKey();
    this.scene3dLoadPending = true;
    this.ensureScene3d()
      .then(async (scene) => {
        scene.setViewMode(this.mapViewMode);
        scene.setLandmarkLabelsVisible(this.lmNamesVisible);
        scene.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
        scene.setTargetArmed(this.scene3dTargetArmed());
        if (options.force || this.scene3dStaticKey !== requestedKey) {
          let payload = this.operatorBabylonScenePayload();
          if (this.mapViewMode === "3d" && this.isFleetManager()) {
            payload = await this.getJson(this.fleetApiPath("/scene3d"));
          }
          if (requestedKey !== this.operatorBabylonStaticKey()) {
            this.scene3dRenderQueued = true;
            return;
          }
          payload = {
            ...payload,
            managerId: String(this.selectedRobot()?.id || ""),
          };
          this.scene3dPayload = payload;
          scene.setScene(payload, { preserveView: Boolean(this.scene3dStaticKey) });
          this.scene3dStaticKey = requestedKey;
          if (this.operatorScene3d) {
            this.operatorScene3d.dataset.managerId = String(payload.managerId || "");
            this.operatorScene3d.dataset.mapName = String(payload.mapName || "");
          }
        }
        this.syncMapControls();
        scene.setFloorCanvas(
          this.fleetMapEditorActive && this.fleetRasterGrid
            ? this.fleetRasterGrid.canvas
            : null,
        );
        scene.setEditorState(this.babylonEditorState());
        const robots = this.operatorBabylonRobots();
        const selectedName = this.isFleetManager()
          ? (this.selectedFleetRobot(robots)?.name || "")
          : String(robots[0]?.name || "");
        const waitBlockerName = this.isFleetManager()
          ? this.fleetRobotWaitBlockerName(this.selectedFleetRobot(robots))
          : "";
        if (options.motionOnly && scene.updateRobotPoses(robots)) {
          // Pose buffers were updated without rebuilding robot meshes.
        } else {
          scene.updateRobots(robots, selectedName, waitBlockerName);
        }
        this.drawScanOverlay();
        if (this.mapView.follow) {
          const followed = robots.find((robot) => robot.name === selectedName) || robots[0];
          scene.focusOn(followed?.pose);
        }
      })
      .catch((error) => {
        this.babylonMapFailed = true;
        this.robotMessageText.textContent = `Babylon map failed, SVG fallback enabled: ${error.message || error}`;
        this.syncMapControls();
        this.renderOperatorMap();
      })
      .finally(() => {
        this.scene3dLoadPending = false;
        const rerender = this.scene3dRenderQueued;
        this.scene3dRenderQueued = false;
        if (rerender && typeof window.requestAnimationFrame === "function") {
          window.requestAnimationFrame(() => this.renderOperatorBabylonMap());
        }
      });
  }

  markBabylonMapGeometryDirty() {
    this.babylonMapRevision += 1;
  }

  handleBabylonMapPointerDown(hit) {
    if (Number(hit?.button || 0) !== 0 || !hit?.world) {
      return false;
    }
    if (this.relocateMode && !this.isFleetManager()) {
      this.babylonRelocationDrag = {
        pointerId: hit.pointerId,
        start: { ...hit.world },
        end: { ...hit.world },
      };
      return true;
    }
    if (!this.fleetMapEditorActive || this.mapViewMode !== "2d") {
      return false;
    }
    this.ensureFleetMapDraft();
    if (FLEET_RASTER_TOOLS.has(this.fleetMapTool)) {
      return this.beginFleetRasterPointer(hit);
    }
    if (hit.bezierIndex && hit.edgeKey) {
      this.selectFleetEditorEdge(hit.edgeKey);
      this.fleetEditorBezierDrag = {
        pointerId: hit.pointerId,
        edgeKey: hit.edgeKey,
        index: Number(hit.bezierIndex),
      };
      return true;
    }
    if (hit.lmName) {
      if (this.fleetMapTool === "corridor") {
        this.handleFleetCorridorLm(hit.lmName);
        return true;
      }
      this.selectFleetEditorLm(hit.lmName);
      if (this.fleetMapTool === "edge") {
        this.fleetEditorEdgeDrag = { pointerId: hit.pointerId, currentLm: hit.lmName, lastCreated: "" };
      } else {
        this.fleetEditorLmDrag = { pointerId: hit.pointerId, name: hit.lmName, start: hit.world, moved: false };
      }
      return true;
    }
    if (hit.edgeKey) {
      this.selectFleetEditorEdge(hit.edgeKey);
      return true;
    }
    if (this.fleetMapTool === "lm") {
      const added = this.addFleetEditorLm(hit.world);
      this.selectFleetEditorLm(added.name);
      this.renderOperatorBabylonMap({ force: true });
      return true;
    }
    this.fleetSelectedLmName = "";
    this.fleetSelectedEdgeKey = "";
    this.syncFleetEditorFields();
    this.refreshBabylonEditorState();
    return false;
  }

  handleBabylonMapPointerMove(hit) {
    const world = hit?.world;
    if (this.babylonRelocationDrag?.pointerId === hit?.pointerId && world) {
      this.babylonRelocationDrag.end = { ...world };
      return;
    }
    if (!this.fleetMapEditorActive || !world) {
      return;
    }
    if (this.fleetRasterDrag?.pointerId === hit.pointerId) {
      this.moveFleetRasterPointer(hit);
      return;
    }
    if (this.fleetEditorBezierDrag?.pointerId === hit.pointerId) {
      const snapped = this.snapMapPoint(world);
      this.moveFleetEditorBezierHandle(
        this.fleetEditorBezierDrag.edgeKey,
        this.fleetEditorBezierDrag.index,
        snapped,
      );
      this.fleetMapDirty = true;
      this.syncFleetEditorFields();
      this.refreshBabylonEditorState();
      return;
    }
    if (this.fleetEditorLmDrag?.pointerId === hit.pointerId) {
      const snapped = this.snapMapPoint(world);
      this.moveFleetEditorLm(this.fleetEditorLmDrag.name, snapped);
      this.fleetEditorLmDrag.moved = true;
      this.fleetMapDirty = true;
      this.syncFleetEditorFields();
      this.refreshBabylonEditorState();
      return;
    }
    if (this.fleetEditorEdgeDrag?.pointerId === hit.pointerId) {
      const nearest = this.nearestLandmark(world);
      if (
        nearest
        && nearest.distance <= 0.35
        && nearest.landmark.name !== this.fleetEditorEdgeDrag.currentLm
        && nearest.landmark.name !== this.fleetEditorEdgeDrag.lastCreated
      ) {
        const previous = this.fleetEditorEdgeDrag.currentLm;
        this.addFleetEditorEdge(previous, nearest.landmark.name);
        this.fleetEditorEdgeDrag.lastCreated = previous;
        this.fleetEditorEdgeDrag.currentLm = nearest.landmark.name;
        this.fleetMapDirty = true;
      }
      this.fleetEditorPreview = {
        fromName: this.fleetEditorEdgeDrag.currentLm,
        world: { ...world },
      };
      this.refreshBabylonEditorState();
    }
  }

  handleBabylonMapPointerUp(hit) {
    if (this.babylonRelocationDrag?.pointerId === hit?.pointerId) {
      const drag = this.babylonRelocationDrag;
      this.babylonRelocationDrag = null;
      const dx = drag.end.x - drag.start.x;
      const dy = drag.end.y - drag.start.y;
      const currentYaw = Number(this.currentStatus?.robot?.pose?.yaw || 0);
      const yaw = Math.hypot(dx, dy) > 0.03 ? Math.atan2(dy, dx) : currentYaw;
      this.startRelocation({ ...drag.start, yaw });
      return;
    }
    if (this.fleetRasterDrag?.pointerId === hit?.pointerId) {
      this.endFleetRasterPointer(hit);
      return;
    }
    const geometryChanged = Boolean(
      this.fleetEditorLmDrag
      || this.fleetEditorEdgeDrag
      || this.fleetEditorBezierDrag
    );
    this.fleetEditorLmDrag = null;
    this.fleetEditorEdgeDrag = null;
    this.fleetEditorBezierDrag = null;
    this.fleetEditorPreview = null;
    if (geometryChanged) {
      this.renderOperatorBabylonMap({ force: true });
    } else {
      this.refreshBabylonEditorState();
    }
  }

  handleBabylonMapContextMenu(hit) {
    if (!this.fleetMapEditorActive || this.mapViewMode !== "2d") {
      return;
    }
    if (hit?.lmName && window.confirm(`Delete ${hit.lmName}?`)) {
      this.deleteFleetEditorLm(hit.lmName);
      return;
    }
    if (hit?.edgeKey && window.confirm(`Delete edge ${hit.edgeKey}?`)) {
      this.deleteFleetEditorEdge(hit.edgeKey);
    }
  }

  scene3dTargetArmed() {
    if (!this.navigateMode) {
      return false;
    }
    if (!this.isFleetManager()) {
      return true;
    }
    return ["navigate", "queue", "spawn"].includes(this.pendingFleetAction);
  }

  scene3dKey() {
    const robot = this.selectedRobot();
    const mapName = String(this.currentStatus?.mapName || this.robotMapState.robotActiveMapName || this.robotMapState.operatorActiveMapName || "");
    const signature = String(this.operatorMapSignature || "");
    return `${robot?.id || ""}:${mapName}:${signature}`;
  }

  invalidateOperatorScene3d() {
    this.scene3dStaticKey = "";
    this.scene3dPayload = null;
    this.scene3dRenderQueued = true;
    if (this.activeOperatorMapPayload()?.map) {
      this.renderOperatorBabylonMap({ force: true });
    }
  }

  refreshOperatorScene3d(options = {}) {
    if (this.babylonMapFailed || this.slamActive) {
      return;
    }
    if (!this.updateOperatorScene3dRobots(this.scene3d, options)) {
      this.renderOperatorBabylonMap(options);
    }
  }

  updateOperatorScene3dRobots(scene = this.scene3d, options = {}) {
    if (!scene || this.babylonMapFailed || this.slamActive) {
      return false;
    }
    if (this.scene3dStaticKey !== this.operatorBabylonStaticKey()) {
      return false;
    }
    scene.setViewMode(this.mapViewMode);
    scene.setLandmarkLabelsVisible(this.lmNamesVisible);
    scene.setEdgeDirectionsVisible(this.edgeDirectionsVisible);
    const robots = this.operatorBabylonRobots();
    const selectedRobot = this.isFleetManager() ? this.selectedFleetRobot(robots) : robots[0];
    if (
      options.motionOnly
      && typeof scene.updateRobotPoses === "function"
      && scene.updateRobotPoses(robots)
    ) {
      this.drawScanOverlay();
      if (this.mapView.follow && selectedRobot?.pose) {
        scene.focusOn(selectedRobot.pose);
      }
      return true;
    }
    scene.setTargetArmed(this.scene3dTargetArmed());
    const selectedName = selectedRobot?.name || "";
    const waitBlockerName = this.isFleetManager() ? this.fleetRobotWaitBlockerName(selectedRobot) : "";
    scene.updateRobots(robots, selectedName, waitBlockerName);
    scene.setEditorState(this.babylonEditorState());
    this.drawScanOverlay();
    if (this.mapView.follow && selectedRobot?.pose) {
      scene.focusOn(selectedRobot.pose);
    }
    return true;
  }

  toggleNavigateMode() {
    if (!this.isFleetManager() && this.slamActive) {
      this.robotMessageText.textContent = "Navigation is disabled while 2D SLAM is active.";
      return;
    }
    if (!this.hasNavigationMapPayload()) {
      this.robotMessageText.textContent = `Pull or load the robot map before ${this.navigateButtonIdleText()}.`;
      return;
    }
    this.relocateMode = false;
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
      : this.fleetNavigateUsesPose()
        ? "click a map pose or select an LM; Fleet Manager will snap it to the traffic graph."
        : "select an LM on the map.";
    this.robotMessageText.textContent = this.navigateMode
      ? (target ? `Navigate armed for ${target}: ${targetHint}` : `Navigate armed: ${targetHint}`)
      : "Navigate canceled.";
  }

  toggleRelocateMode() {
    if (this.isFleetManager()) {
      return;
    }
    if (this.slamActive) {
      this.robotMessageText.textContent = "Relocate is disabled while 2D SLAM is active.";
      return;
    }
    if (!this.operatorMapPayload || !this.operatorMapPayload.map) {
      this.robotMessageText.textContent = "Pull or load the robot map before Relocate.";
      return;
    }
    this.navigateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.relocateMode = !this.relocateMode;
    this.relocationDrag = null;
    this.clearRelocationPreview();
    this.syncModeButtons();
    this.drawLandmarks();
    this.robotMessageText.textContent = this.relocateMode
      ? "Relocate armed: hold on the map, drag heading, release."
      : "Relocate canceled.";
  }

  toggleFleetQueueMode() {
    if (!this.isFleetManager()) {
      return;
    }
    if (!this.hasNavigationMapPayload()) {
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
    const spawnArmed = this.navigateMode && isFleet && this.pendingFleetAction === "spawn";
    const relocateArmed = this.relocateMode && !isFleet;
    this.operatorScene3d?.classList.toggle("target-armed", navigateArmed || queueArmed || spawnArmed);
    this.scene3d?.setTargetArmed(this.scene3dTargetArmed());
    const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    const routeState = String(robot.state || "").toUpperCase();
    const routeActive = Boolean(robot.targetLm || robot.routeId || routeState === "EXECUTING_ROUTE" || routeState === "PAUSED");
    const paused = this.robotNavigationPaused(robot);
    const mappingActive = !isFleet && this.slamActive;
    const control = this.robotControlPayload(robot);
    const operatorOwnsControl = control.ownerId === "operator-app";
    const idleText = this.navigateButtonIdleText();
    this.navigateRobotButton.classList.toggle("primary", !navigateArmed);
    this.navigateRobotButton.classList.toggle("danger", navigateArmed);
    this.navigateRobotButton.disabled = relocateArmed || mappingActive;
    this.navigateRobotButton.textContent = navigateArmed
      ? (this.pendingFleetRobotName
          ? `${this.fleetNavigateUsesPose() ? "Select Pose" : "Select LM"}: ${this.pendingFleetRobotName}`
          : "Cancel Navigate")
      : idleText;
    if (this.relocateRobotButton) {
      this.relocateRobotButton.classList.toggle("primary", !relocateArmed);
      this.relocateRobotButton.classList.toggle("danger", relocateArmed);
      this.relocateRobotButton.disabled = mappingActive;
      this.relocateRobotButton.textContent = relocateArmed ? "Cancel Relocate" : "Relocate";
    }
    if (this.pauseRouteButton) {
      this.pauseRouteButton.disabled = mappingActive || !routeActive || paused;
    }
    if (this.resumeRouteButton) {
      this.resumeRouteButton.disabled = mappingActive || !paused;
    }
    if (this.takeControlButton) {
      this.takeControlButton.disabled = mappingActive || operatorOwnsControl;
      this.takeControlButton.textContent = control.ownerId && !operatorOwnsControl
        ? `Take Control from ${control.ownerName || control.ownerId}`
        : "Take Control";
    }
    if (this.releaseControlButton) {
      this.releaseControlButton.disabled = mappingActive || !operatorOwnsControl;
    }
    if (!relocateArmed) {
      this.relocationDrag = null;
      this.clearRelocationPreview();
    }
    if (this.fleetQueueGoalButton) {
      this.fleetQueueGoalButton.classList.toggle("primary", queueArmed);
      this.fleetQueueGoalButton.classList.toggle("danger", queueArmed);
      this.fleetQueueGoalButton.textContent = queueArmed
        ? `Queue LM: ${this.pendingFleetRobotName || "robot"}`
        : "Queue Goal";
    }
    if (this.fleetPlaceRobotButton) {
      this.fleetPlaceRobotButton.classList.toggle("primary", spawnArmed);
      this.fleetPlaceRobotButton.classList.toggle("danger", spawnArmed);
      this.fleetPlaceRobotButton.textContent = spawnArmed ? "Cancel Place" : "Place";
    }
  }

  navigateButtonIdleText() {
    return (this.isRos2Robot() && !this.isFleetManager()) || this.fleetNavigateUsesPose()
      ? "Navigate To Pose"
      : "Navigate To LM";
  }

  fleetNavigateUsesPose() {
    if (!this.isFleetManager()) {
      return false;
    }
    return Boolean(this.targetFleetRobot());
  }

  async startNavigation(goalLm) {
    if (!this.selectedRobot()) {
      return;
    }
    if (!this.isFleetManager() && this.slamActive) {
      this.robotMessageText.textContent = "Navigation is disabled while 2D SLAM is active.";
      return;
    }
    if (this.isFleetManager()) {
      await this.startFleetNavigation(goalLm);
      return;
    }
    if (!await this.ensureRobotControlForNavigation()) {
      return;
    }
    this.navigateMode = false;
    this.relocateMode = false;
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
    if (this.slamActive) {
      this.robotMessageText.textContent = "Navigation is disabled while 2D SLAM is active.";
      return;
    }
    if (!await this.ensureRobotControlForNavigation()) {
      return;
    }
    this.navigateMode = false;
    this.relocateMode = false;
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

  async ensureRobotControlForNavigation() {
    const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    if (this.robotControlPayload(robot).ownerId === "operator-app") {
      return true;
    }
    return this.acquireRobotControl(true, false);
  }

  async acquireRobotControl(force = true, announce = true) {
    if (!this.selectedRobot() || this.isFleetManager()) {
      return false;
    }
    try {
      const result = await this.postJson(this.robotApiPath("/api/robot/control/acquire"), {
        force,
        stopNavigation: force,
      });
      this.currentStatus = result.status || await this.getJson(this.robotApiPath("/api/robot/status"));
      this.renderSelectedRobot();
      if (announce) {
        this.robotMessageText.textContent = result.navigationStopped
          ? "Control acquired. Previous autonomous route stopped safely."
          : "Control acquired.";
      }
      return true;
    } catch (error) {
      this.robotMessageText.textContent = `Take control failed: ${error.message || error}`;
      return false;
    }
  }

  async releaseRobotControl(force = false) {
    if (!this.selectedRobot() || this.isFleetManager()) {
      return;
    }
    try {
      const result = await this.postJson(this.robotApiPath("/api/robot/control/release"), { force });
      this.currentStatus = result.status || await this.getJson(this.robotApiPath("/api/robot/status"));
      this.renderSelectedRobot();
      this.robotMessageText.textContent = "Control released.";
    } catch (error) {
      this.robotMessageText.textContent = `Release control failed: ${error.message || error}`;
    }
  }

  async startFleetPoseNavigation(world) {
    if (!this.fleetNavigateUsesPose()) {
      return;
    }
    const nearest = this.nearestLandmark(world);
    if (!nearest || !nearest.landmark) {
      this.robotMessageText.textContent = "Navigate failed: the fleet map has no graph landmark for this pose.";
      return;
    }
    await this.startFleetNavigation(nearest.landmark.name, {
      requestedPose: {
        x: Number(world.x || 0),
        y: Number(world.y || 0),
      },
      snapDistance: Number(nearest.distance || 0),
    });
  }

  async startRelocation(world) {
    if (!this.selectedRobot() || this.isFleetManager()) {
      return;
    }
    this.relocateMode = false;
    this.relocationDrag = null;
    this.clearRelocationPreview();
    this.syncModeButtons();
    this.drawLandmarks();
    const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    const requestedYaw = Number(world?.yaw);
    const currentYaw = Number(robot.pose?.yaw || 0);
    const yaw = Number.isFinite(requestedYaw) ? requestedYaw : currentYaw;
    const pose = {
      x: Number(world.x || 0),
      y: Number(world.y || 0),
      yaw: Number.isFinite(yaw) ? yaw : 0,
    };
    try {
      const result = await this.postJson(this.robotApiPath("/api/robot/relocate"), { pose });
      this.currentStatus = result.status || await this.getJson(this.robotApiPath("/api/robot/status"));
      this.robotMessageText.textContent = `Relocation pose sent: x ${pose.x.toFixed(3)}, y ${pose.y.toFixed(3)}, yaw ${pose.yaw.toFixed(3)}.`;
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Relocate failed: ${error.message || error}`;
    }
  }

  async pauseRobotRoute() {
    if (!this.selectedRobot() || this.isFleetManager()) {
      return;
    }
    try {
      const result = await this.postJson(this.robotApiPath("/api/robot/route/pause"), {});
      this.currentStatus = result.status || await this.getJson(this.robotApiPath("/api/robot/status"));
      this.robotMessageText.textContent = "Route paused.";
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Pause route failed: ${error.message || error}`;
    }
  }

  async resumeRobotRoute() {
    if (!this.selectedRobot() || this.isFleetManager()) {
      return;
    }
    try {
      const result = await this.postJson(this.robotApiPath("/api/robot/route/resume"), {});
      this.currentStatus = result.status || await this.getJson(this.robotApiPath("/api/robot/status"));
      this.robotMessageText.textContent = "Route resumed.";
      this.renderSelectedRobot();
    } catch (error) {
      this.robotMessageText.textContent = `Resume route failed: ${error.message || error}`;
    }
  }

  async startFleetNavigation(goalLm, options = {}) {
    const robot = this.targetFleetRobot();
    if (!robot) {
      this.robotMessageText.textContent = "Add or select a fleet robot first.";
      return;
    }
    this.navigateMode = false;
    this.relocateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.syncModeButtons();
    await this.releaseFleetManualControl();
    try {
      this.robotMessageText.textContent = `Planning ${robot.name} -> ${goalLm}...`;
      await new Promise((resolve) => window.requestAnimationFrame(resolve));
      const result = await this.postJson(this.fleetApiPath("/setOrder"), {
        id: this.nextFleetOrderId(robot.name),
        vehicle: robot.name,
        targetLm: goalLm,
        priority: 10,
        ...this.fleetMotionParams(),
        replaceActive: true,
      });
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.lastFleetPlanDebug = result.debug || result.state?.debug || null;
      this.selectedFleetRobotName = robot.name;
      this.fleetSelectionCleared = false;
      window.localStorage.setItem("operator:selectedFleetRobotName", this.selectedFleetRobotName);
      this.renderFleetStateImmediately();
      const requestedPose = options.requestedPose;
      this.robotMessageText.textContent = requestedPose
        ? `Order sent: ${robot.name} -> pose x ${requestedPose.x.toFixed(3)}, y ${requestedPose.y.toFixed(3)}; graph target ${goalLm} (${Number(options.snapDistance || 0).toFixed(2)} m snap).`
        : `Order sent: ${robot.name} -> ${goalLm}.`;
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
};
