(function () {
  const data = window.ROBOT_WEB_DATA;
  if (!data) {
    throw new Error("ROBOT_WEB_DATA is missing.");
  }

  class GeometryService {
    constructor(mapData) {
      this.mapData = mapData;
    }

    worldToPixel(point) {
      return {
        x: this.mapData.viewPadding + ((point.x - this.mapData.origin[0]) / this.mapData.resolution),
        y: this.mapData.viewPadding + (this.mapData.height - 1) - ((point.y - this.mapData.origin[1]) / this.mapData.resolution),
      };
    }

    pixelToWorld(point) {
      return {
        x: ((point.x - this.mapData.viewPadding) * this.mapData.resolution) + this.mapData.origin[0],
        y: (((this.mapData.height - 1) - (point.y - this.mapData.viewPadding)) * this.mapData.resolution) + this.mapData.origin[1],
      };
    }

    eventToWorld(event, viewport) {
      const ctm = viewport.getScreenCTM();
      if (!ctm) {
        return null;
      }
      const point = new DOMPoint(event.clientX, event.clientY).matrixTransform(ctm.inverse());
      return this.pixelToWorld(point);
    }

    distance(first, second) {
      return Math.hypot(second.x - first.x, second.y - first.y);
    }
  }

  class GraphModel {
    constructor(landmarks, edges) {
      this.landmarks = landmarks || [];
      this.edges = edges || [];
      this.nodeByName = new Map(this.landmarks.map((item) => [item.name, item]));
    }

    nearestLandmark(point, geometry) {
      let best = null;
      let bestDistance = Number.POSITIVE_INFINITY;
      for (const landmark of this.landmarks) {
        const distance = geometry.distance(point, landmark);
        if (distance < bestDistance) {
          bestDistance = distance;
          best = landmark;
        }
      }
      return { landmark: best, distance: bestDistance };
    }
  }

  class Renderer {
    constructor(dom, geometry, graphModel, onLandmarkClick) {
      this.dom = dom;
      this.geometry = geometry;
      this.graphModel = graphModel;
      this.onLandmarkClick = onLandmarkClick;
    }

    initMap() {
      const mapData = this.geometry.mapData;
      this.dom.mapSvg.setAttribute("viewBox", `0 0 ${mapData.viewWidth} ${mapData.viewHeight}`);
      this.dom.mapImage.setAttribute("x", String(mapData.viewPadding));
      this.dom.mapImage.setAttribute("y", String(mapData.viewPadding));
      this.dom.mapImage.setAttribute("width", String(mapData.width));
      this.dom.mapImage.setAttribute("height", String(mapData.height));
      this.dom.mapImage.setAttribute("href", mapData.imageDataUrl);
    }

    drawGraph() {
      this.dom.graphLayer.innerHTML = "";
      for (const edge of this.graphModel.edges) {
        const element = document.createElementNS("http://www.w3.org/2000/svg", edge.geometry === "bezier" ? "path" : "line");
        element.setAttribute("class", "graph-edge");
        if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
          const points = edge.control_points.map((point) => this.geometry.worldToPixel(point));
          element.setAttribute(
            "d",
            [
              `M ${points[0].x} ${points[0].y}`,
              `C ${points[1].x} ${points[1].y}, ${points[2].x} ${points[2].y}, ${points[3].x} ${points[3].y}`,
            ].join(" ")
          );
        } else {
          const start = this.geometry.worldToPixel(this.graphModel.nodeByName.get(edge.from));
          const goal = this.geometry.worldToPixel(this.graphModel.nodeByName.get(edge.to));
          element.setAttribute("x1", String(start.x));
          element.setAttribute("y1", String(start.y));
          element.setAttribute("x2", String(goal.x));
          element.setAttribute("y2", String(goal.y));
        }
        this.dom.graphLayer.appendChild(element);
      }
    }

    drawLandmarks(nearestName, targetName, navigateMode = false) {
      this.dom.landmarkLayer.innerHTML = "";
      for (const landmark of this.graphModel.landmarks) {
        const point = this.geometry.worldToPixel(landmark);
        const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
        let className = "landmark default";
        if (landmark.name === nearestName) {
          className = "landmark nearest";
        }
        if (landmark.name === targetName) {
          className = "landmark target";
        }
        if (navigateMode) {
          className = `${className} armed`;
        }
        group.setAttribute("class", className);
        group.addEventListener("click", () => this.onLandmarkClick(landmark.name));

        const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
        circle.setAttribute("cx", String(point.x));
        circle.setAttribute("cy", String(point.y));
        circle.setAttribute("r", "6");
        group.appendChild(circle);

        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("x", String(point.x + 9));
        label.setAttribute("y", String(point.y - 9));
        label.textContent = landmark.name;
        group.appendChild(label);

        this.dom.landmarkLayer.appendChild(group);
      }
    }

    drawRoute(route) {
      this.dom.routeLayer.innerHTML = "";
      if (!route || !Array.isArray(route.trajectory) || route.trajectory.length < 2) {
        return;
      }
      const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
      polyline.setAttribute("class", "planned-route");
      polyline.setAttribute(
        "points",
        route.trajectory
          .map((point) => {
            const px = this.geometry.worldToPixel(point);
            return `${px.x},${px.y}`;
          })
          .join(" ")
      );
      this.dom.routeLayer.appendChild(polyline);
    }

    drawRobot(pose) {
      this.dom.robotLayer.innerHTML = "";
      if (!pose) {
        return null;
      }
      const center = this.geometry.worldToPixel(pose);
      const radius = 12;
      const group = document.createElementNS("http://www.w3.org/2000/svg", "g");

      const body = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      body.setAttribute("class", "robot-body");
      body.setAttribute("cx", String(center.x));
      body.setAttribute("cy", String(center.y));
      body.setAttribute("r", String(radius));
      group.appendChild(body);

      const heading = document.createElementNS("http://www.w3.org/2000/svg", "line");
      heading.setAttribute("class", "robot-heading");
      heading.setAttribute("x1", String(center.x));
      heading.setAttribute("y1", String(center.y));
      heading.setAttribute("x2", String(center.x + (Math.cos(pose.yaw) * radius * 1.65)));
      heading.setAttribute("y2", String(center.y - (Math.sin(pose.yaw) * radius * 1.65)));
      group.appendChild(heading);

      this.dom.robotLayer.appendChild(group);
      return center;
    }
  }

  class ViewportController {
    constructor(dom, mapData) {
      this.dom = dom;
      this.mapData = mapData;
      this.scale = 1;
      this.tx = 0;
      this.ty = 0;
      this.minScale = 1;
      this.maxScale = 8;
      this.followRobot = true;
      this.dragState = null;
      this.consumeClick = false;
    }

    enable() {
      this.apply();
      this.syncFollowButton();

      this.dom.zoomInButton.addEventListener("click", () => this.zoom(1.15));
      this.dom.zoomOutButton.addEventListener("click", () => this.zoom(0.87));
      this.dom.resetViewButton.addEventListener("click", () => this.resetView());
      this.dom.followRobotButton.addEventListener("click", () => this.toggleFollow());

      this.dom.mapSvg.addEventListener("pointerdown", (event) => this.handlePointerDown(event));
      this.dom.mapSvg.addEventListener("pointermove", (event) => this.handlePointerMove(event));
      this.dom.mapSvg.addEventListener("pointerup", (event) => this.handlePointerUp(event));
      this.dom.mapSvg.addEventListener("pointercancel", (event) => this.handlePointerUp(event));
      this.dom.mapSvg.addEventListener("wheel", (event) => this.handleWheel(event), { passive: false });
      this.dom.mapSvg.addEventListener("click", (event) => {
        if (this.consumeClick) {
          event.preventDefault();
          event.stopPropagation();
          this.consumeClick = false;
        }
      }, true);
    }

    resetView() {
      this.followRobot = false;
      this.scale = 1;
      this.tx = 0;
      this.ty = 0;
      this.apply();
      this.syncFollowButton();
    }

    toggleFollow(force = null) {
      this.followRobot = force === null ? !this.followRobot : Boolean(force);
      this.syncFollowButton();
    }

    syncFollowButton() {
      this.dom.followRobotButton.classList.toggle("primary", this.followRobot);
      this.dom.followRobotButton.setAttribute("aria-pressed", this.followRobot ? "true" : "false");
      this.dom.followRobotButton.textContent = this.followRobot ? "Following Robot" : "Follow Robot";
    }

    focusOn(point) {
      if (!point || !this.followRobot) {
        return;
      }
      const center = this.viewCenter();
      this.tx = center.x - (this.scale * point.x);
      this.ty = center.y - (this.scale * point.y);
      this.apply();
    }

    zoom(factor, anchor = null) {
      const oldScale = this.scale;
      const nextScale = Math.max(this.minScale, Math.min(this.maxScale, oldScale * factor));
      if (Math.abs(nextScale - oldScale) < 0.0001) {
        return;
      }
      const pivot = anchor || this.viewCenter();
      this.tx = pivot.x - ((nextScale / oldScale) * (pivot.x - this.tx));
      this.ty = pivot.y - ((nextScale / oldScale) * (pivot.y - this.ty));
      this.scale = nextScale;
      this.apply();
    }

    pan(dx, dy) {
      this.tx += dx;
      this.ty += dy;
      this.apply();
    }

    apply() {
      this.dom.viewport.setAttribute("transform", `matrix(${this.scale} 0 0 ${this.scale} ${this.tx} ${this.ty})`);
    }

    handlePointerDown(event) {
      if (event.button !== 0) {
        return;
      }
      const point = this.screenToSvg(event.clientX, event.clientY);
      if (!point) {
        return;
      }
      this.dragState = {
        pointerId: event.pointerId,
        last: point,
      };
      this.dom.mapSvg.setPointerCapture(event.pointerId);
    }

    handlePointerMove(event) {
      if (!this.dragState || event.pointerId !== this.dragState.pointerId) {
        return;
      }
      const point = this.screenToSvg(event.clientX, event.clientY);
      if (!point) {
        return;
      }
      const dx = point.x - this.dragState.last.x;
      const dy = point.y - this.dragState.last.y;
      if (Math.abs(dx) < 0.001 && Math.abs(dy) < 0.001) {
        return;
      }
      if (this.followRobot) {
        this.toggleFollow(false);
      }
      this.pan(dx, dy);
      this.dragState.last = point;
      this.consumeClick = true;
    }

    handlePointerUp(event) {
      if (!this.dragState || event.pointerId !== this.dragState.pointerId) {
        return;
      }
      if (this.dom.mapSvg.hasPointerCapture(event.pointerId)) {
        this.dom.mapSvg.releasePointerCapture(event.pointerId);
      }
      this.dragState = null;
      window.setTimeout(() => {
        this.consumeClick = false;
      }, 0);
    }

    handleWheel(event) {
      event.preventDefault();
      const anchor = this.followRobot
        ? this.viewCenter()
        : this.screenToSvg(event.clientX, event.clientY);
      this.zoom(event.deltaY < 0 ? 1.12 : 0.88, anchor);
    }

    screenToSvg(clientX, clientY) {
      const ctm = this.dom.mapSvg.getScreenCTM();
      if (!ctm) {
        return null;
      }
      return new DOMPoint(clientX, clientY).matrixTransform(ctm.inverse());
    }

    viewCenter() {
      return {
        x: this.mapData.viewWidth / 2,
        y: this.mapData.viewHeight / 2,
      };
    }
  }

  class RobotApp {
    constructor(siteData) {
      this.data = siteData;
      this.dom = this.getDom();
      this.geometry = new GeometryService(siteData.map);
      this.graphModel = new GraphModel(siteData.lms, siteData.edges);
      this.renderer = new Renderer(this.dom, this.geometry, this.graphModel, (name) => this.handleLandmarkSelect(name));
      this.viewport = new ViewportController(this.dom, siteData.map);
      this.currentStatus = null;
      this.currentRoute = null;
      this.manualKeys = new Set();
      this.teleopTimer = null;
      this.pollTimer = null;
      this.lastEventSignature = "";
      this.params = siteData.params || {};
      this.navigateMode = false;
      this.navigatePointerDown = null;
      this.suppressNextNavigateClick = false;
      this.statusRequestPending = false;
    }

    getDom() {
      return {
        pageTitle: document.getElementById("pageTitle"),
        statusText: document.getElementById("statusText"),
        robotIdBadge: document.getElementById("robotIdBadge"),
        connectionBadge: document.getElementById("connectionBadge"),
        navigateButton: document.getElementById("navigateButton"),
        summaryConnectionText: document.getElementById("summaryConnectionText"),
        summaryRobotText: document.getElementById("summaryRobotText"),
        poseSummaryText: document.getElementById("poseSummaryText"),
        mapSvg: document.getElementById("mapSvg"),
        viewport: document.getElementById("viewport"),
        mapImage: document.getElementById("mapImage"),
        graphLayer: document.getElementById("graphLayer"),
        routeLayer: document.getElementById("routeLayer"),
        landmarkLayer: document.getElementById("landmarkLayer"),
        robotLayer: document.getElementById("robotLayer"),
        zoomInButton: document.getElementById("zoomInButton"),
        zoomOutButton: document.getElementById("zoomOutButton"),
        resetViewButton: document.getElementById("resetViewButton"),
        followRobotButton: document.getElementById("followRobotButton"),
        mapStateText: document.getElementById("mapStateText"),
        mapNearestText: document.getElementById("mapNearestText"),
        mapTargetText: document.getElementById("mapTargetText"),
        manualSpeedText: document.getElementById("manualSpeedText"),
        goalSelect: document.getElementById("goalSelect"),
        planRouteButton: document.getElementById("planRouteButton"),
        executeRouteButton: document.getElementById("executeRouteButton"),
        cancelRouteButton: document.getElementById("cancelRouteButton"),
        stopRobotButton: document.getElementById("stopRobotButton"),
        mapStopRobotButton: document.getElementById("mapStopRobotButton"),
        refreshStatusButton: document.getElementById("refreshStatusButton"),
        saveParamsButton: document.getElementById("saveParamsButton"),
        tabButtons: Array.from(document.querySelectorAll(".tab-button")),
        tabPages: Array.from(document.querySelectorAll(".tab-page")),
        modeText: document.getElementById("modeText"),
        localizationText: document.getElementById("localizationText"),
        nearestText: document.getElementById("nearestText"),
        targetText: document.getElementById("targetText"),
        edgeText: document.getElementById("edgeText"),
        progressText: document.getElementById("progressText"),
        poseText: document.getElementById("poseText"),
        velocityText: document.getElementById("velocityText"),
        routeIdText: document.getElementById("routeIdText"),
        routeLengthText: document.getElementById("routeLengthText"),
        routeNodesText: document.getElementById("routeNodesText"),
        routeList: document.getElementById("routeList"),
        eventLog: document.getElementById("eventLog"),
        manualButtons: Array.from(document.querySelectorAll("[data-manual-key]")),
        manualLinearSpeedInput: document.getElementById("manualLinearSpeedInput"),
        manualAngularSpeedInput: document.getElementById("manualAngularSpeedInput"),
        nearestToleranceInput: document.getElementById("nearestToleranceInput"),
        onRouteToleranceInput: document.getElementById("onRouteToleranceInput"),
        sampleDistanceInput: document.getElementById("sampleDistanceInput"),
        routeSpeedInput: document.getElementById("routeSpeedInput"),
        lookaheadInput: document.getElementById("lookaheadInput"),
        stopDistanceInput: document.getElementById("stopDistanceInput"),
        localizationTimeoutInput: document.getElementById("localizationTimeoutInput"),
      };
    }

    async init() {
      this.dom.pageTitle.textContent = this.data.title || "Warehouse Robot Control";
      this.dom.robotIdBadge.textContent = this.data.robotId || "robot";
      if (this.dom.summaryRobotText) {
        this.dom.summaryRobotText.textContent = this.data.robotId || "robot";
      }
      this.renderer.initMap();
      this.renderer.drawGraph();
      this.viewport.enable();
      this.populateGoalSelect();
      this.applyParams(this.params);
      this.updateManualSpeedText();
      this.attachEvents();
      this.syncModeButtons();
      await this.refreshParams();
      await this.fetchStatus();
      this.pollTimer = window.setInterval(() => this.fetchStatus(true), 75);
    }

    populateGoalSelect() {
      this.dom.goalSelect.innerHTML = "";
      for (const landmark of this.graphModel.landmarks) {
        const option = document.createElement("option");
        option.value = landmark.name;
        option.textContent = landmark.name;
        if (landmark.name === this.data.defaultGoal) {
          option.selected = true;
        }
        this.dom.goalSelect.appendChild(option);
      }
    }

    attachEvents() {
      for (const button of this.dom.tabButtons) {
        button.addEventListener("click", () => this.setActiveTab(button.dataset.tab));
      }
      this.dom.navigateButton.addEventListener("click", () => this.toggleNavigateMode());
      this.dom.planRouteButton.addEventListener("click", () => this.planRoute());
      this.dom.executeRouteButton.addEventListener("click", () => this.executeRoute());
      this.dom.cancelRouteButton.addEventListener("click", () => this.cancelRoute());
      this.dom.stopRobotButton.addEventListener("click", () => this.stopRobot());
      this.dom.mapStopRobotButton.addEventListener("click", () => this.stopRobot());
      this.dom.refreshStatusButton.addEventListener("click", () => this.fetchStatus());
      this.dom.saveParamsButton.addEventListener("click", () => this.saveParams());
      this.dom.mapSvg.addEventListener("pointerdown", (event) => this.handleNavigatePointerDown(event), true);
      this.dom.mapSvg.addEventListener("pointerup", (event) => this.handleNavigatePointerUp(event), true);
      this.dom.mapSvg.addEventListener("click", (event) => this.handleMapNavigateClick(event));

      this.dom.manualLinearSpeedInput.addEventListener("input", () => this.updateManualSpeedText());
      this.dom.manualAngularSpeedInput.addEventListener("input", () => this.updateManualSpeedText());

      window.addEventListener("keydown", (event) => this.handleManualKey(event, true));
      window.addEventListener("keyup", (event) => this.handleManualKey(event, false));
      window.addEventListener("blur", () => this.releaseManualControl());

      for (const button of this.dom.manualButtons) {
        const key = button.dataset.manualKey;
        button.addEventListener("pointerdown", (event) => {
          event.preventDefault();
          this.setManualKey(key, true);
          button.setPointerCapture(event.pointerId);
        });
        const release = (event) => {
          this.setManualKey(key, false);
          if (button.hasPointerCapture(event.pointerId)) {
            button.releasePointerCapture(event.pointerId);
          }
        };
        button.addEventListener("pointerup", release);
        button.addEventListener("pointercancel", release);
        button.addEventListener("pointerleave", () => this.setManualKey(key, false));
      }
    }

    setActiveTab(tabName) {
      for (const button of this.dom.tabButtons) {
        button.classList.toggle("active", button.dataset.tab === tabName);
      }
      for (const page of this.dom.tabPages) {
        page.classList.toggle("active", page.id === `tab${tabName[0].toUpperCase()}${tabName.slice(1)}`);
      }
    }

    handleLandmarkSelect(name) {
      if (this.navigateMode) {
        this.suppressNextNavigateClick = true;
        window.setTimeout(() => {
          this.suppressNextNavigateClick = false;
        }, 250);
        this.startNavigation(name);
        return;
      }
      this.dom.goalSelect.value = name;
      this.setActiveTab("route");
      this.setStatus(`Goal selected: ${name}.`);
      this.render();
    }

    handleManualKey(event, pressed) {
      if (event.target && ["INPUT", "TEXTAREA", "SELECT"].includes(event.target.tagName)) {
        return;
      }
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) {
        return;
      }
      event.preventDefault();
      this.setManualKey(key, pressed);
    }

    setManualKey(key, pressed) {
      if (pressed) {
        this.manualKeys.add(key);
        this.startTeleopLoop();
      } else {
        this.manualKeys.delete(key);
        if (this.manualKeys.size === 0) {
          this.releaseManualControl();
        }
      }
      this.syncManualButtons();
    }

    syncManualButtons() {
      for (const button of this.dom.manualButtons) {
        button.classList.toggle("active", this.manualKeys.has(button.dataset.manualKey));
      }
    }

    startTeleopLoop() {
      if (this.teleopTimer !== null) {
        return;
      }
      this.clearNavigateMode(false);
      this.setActiveTab("teleop");
      this.sendTeleop();
      this.teleopTimer = window.setInterval(() => this.sendTeleop(), 100);
    }

    releaseManualControl() {
      if (this.teleopTimer !== null) {
        window.clearInterval(this.teleopTimer);
        this.teleopTimer = null;
      }
      if (this.manualKeys.size) {
        this.manualKeys.clear();
      }
      this.syncManualButtons();
      this.postJson("/api/robot/teleop/stop", {}).catch(() => {});
    }

    manualTwist() {
      const linearSpeed = Math.max(0.02, Number(this.dom.manualLinearSpeedInput.value) || 0.25);
      const angularSpeed = Math.max(0.05, Number(this.dom.manualAngularSpeedInput.value) || 0.9);
      const linearSign = (this.manualKeys.has("w") ? 1 : 0) - (this.manualKeys.has("s") ? 1 : 0);
      const angularSign = (this.manualKeys.has("a") ? 1 : 0) - (this.manualKeys.has("d") ? 1 : 0);
      return {
        linear: linearSign * linearSpeed,
        angular: angularSign * angularSpeed,
      };
    }

    updateManualSpeedText() {
      const linearSpeed = Math.max(0.02, Number(this.dom.manualLinearSpeedInput.value) || 0.25);
      const angularSpeed = Math.max(0.05, Number(this.dom.manualAngularSpeedInput.value) || 0.9);
      this.dom.manualSpeedText.textContent = `${linearSpeed.toFixed(2)} m/s | ${angularSpeed.toFixed(2)} rad/s`;
    }

    async sendTeleop() {
      const twist = this.manualTwist();
      if (Math.abs(twist.linear) < 0.0001 && Math.abs(twist.angular) < 0.0001) {
        return;
      }
      try {
        await this.postJson("/api/robot/teleop", {
          linear: twist.linear,
          angular: twist.angular,
          timeoutMs: 300,
        });
      } catch (error) {
        this.setStatus(`Teleop failed: ${error.message || error}`);
      }
    }

    applyParams(params) {
      this.params = params || {};
      const manual = this.params.manual || {};
      const navigation = this.params.navigation || {};
      const localization = this.params.localization || {};
      const planner = this.params.planner || {};
      this.setInputValue(this.dom.manualLinearSpeedInput, manual.linear_speed);
      this.setInputValue(this.dom.manualAngularSpeedInput, manual.angular_speed);
      this.setInputValue(this.dom.nearestToleranceInput, planner.nearest_lm_tolerance);
      this.setInputValue(this.dom.onRouteToleranceInput, planner.on_route_tolerance);
      this.setInputValue(this.dom.sampleDistanceInput, planner.trajectory_sample_distance);
      this.setInputValue(this.dom.routeSpeedInput, navigation.route_speed);
      this.setInputValue(this.dom.lookaheadInput, navigation.footprint_lookahead);
      this.setInputValue(this.dom.stopDistanceInput, navigation.stop_distance);
      this.setInputValue(this.dom.localizationTimeoutInput, localization.localization_timeout);
      this.updateManualSpeedText();
    }

    collectParams() {
      const params = JSON.parse(JSON.stringify(this.params || {}));
      params.navigation = params.navigation || {};
      params.localization = params.localization || {};
      params.manual = params.manual || {};
      params.planner = params.planner || {};
      params.planner.nearest_lm_tolerance = Number(this.dom.nearestToleranceInput.value);
      params.planner.on_route_tolerance = Number(this.dom.onRouteToleranceInput.value);
      params.planner.trajectory_sample_distance = Number(this.dom.sampleDistanceInput.value);
      params.navigation.route_speed = Number(this.dom.routeSpeedInput.value);
      params.navigation.footprint_lookahead = Number(this.dom.lookaheadInput.value);
      params.navigation.stop_distance = Number(this.dom.stopDistanceInput.value);
      params.localization.localization_timeout = Number(this.dom.localizationTimeoutInput.value);
      params.manual.linear_speed = Number(this.dom.manualLinearSpeedInput.value);
      params.manual.angular_speed = Number(this.dom.manualAngularSpeedInput.value);
      return params;
    }

    async refreshParams() {
      try {
        const params = await this.getJson("/api/params");
        this.applyParams(params);
      } catch (_) {
        // Keep embedded defaults when params API is unavailable.
      }
    }

    async saveParams() {
      try {
        const result = await this.postJson("/api/params", this.collectParams());
        if (result && result.params) {
          this.applyParams(result.params);
        }
        this.setStatus("Params saved.");
      } catch (error) {
        this.setStatus(`Save failed: ${error.message || error}`);
      }
    }

    async planRoute() {
      const goalLm = this.dom.goalSelect.value;
      if (!goalLm) {
        this.setStatus("Select goal LM first.");
        return;
      }
      try {
        const result = await this.postJson("/api/robot/route/plan", { goalLm });
        if (result && result.route) {
          this.currentRoute = result.route;
          this.setActiveTab("route");
          this.setStatus(`Planned route to ${goalLm}.`);
          this.render();
        }
      } catch (error) {
        this.setStatus(`Route plan failed: ${error.message || error}`);
      }
    }

    async executeRoute() {
      this.clearNavigateMode(false);
      const goalLm = this.dom.goalSelect.value;
      if (!goalLm && !this.currentRoute) {
        this.setStatus("Select goal LM first.");
        return;
      }
      try {
        const payload = this.currentRoute ? { route: this.currentRoute } : { goalLm };
        const result = await this.postJson("/api/robot/route/execute", payload);
        if (result && result.route) {
          this.currentRoute = result.route;
        }
        this.setActiveTab("route");
        this.setStatus(`Route execution started${goalLm ? ` to ${goalLm}` : ""}.`);
        await this.fetchStatus();
      } catch (error) {
        this.setStatus(`Execute failed: ${error.message || error}`);
      }
    }

    async cancelRoute() {
      this.clearNavigateMode(false);
      try {
        await this.postJson("/api/robot/route/cancel", {});
        this.currentRoute = null;
        this.setStatus("Route canceled.");
        await this.fetchStatus();
      } catch (error) {
        this.setStatus(`Cancel failed: ${error.message || error}`);
      }
    }

    async stopRobot() {
      this.clearNavigateMode(false);
      this.releaseManualControl();
      try {
        await this.postJson("/api/robot/stop", {});
        this.currentRoute = null;
        this.setStatus("Robot stopped.");
        await this.fetchStatus();
      } catch (error) {
        this.setStatus(`Stop failed: ${error.message || error}`);
      }
    }

    async fetchStatus(silent = false) {
      if (this.statusRequestPending) {
        return;
      }
      this.statusRequestPending = true;
      try {
        const result = await this.getJson("/api/robot/status");
        this.currentStatus = result;
        if (result && result.route) {
          this.currentRoute = result.route;
        }
        this.render();
      } catch (error) {
        if (!silent) {
          this.setStatus(`Status fetch failed: ${error.message || error}`);
        }
        this.dom.connectionBadge.textContent = "offline";
        this.dom.connectionBadge.className = "badge error";
        if (this.dom.summaryConnectionText) {
          this.dom.summaryConnectionText.textContent = "offline";
        }
      } finally {
        this.statusRequestPending = false;
      }
    }

    render() {
      const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : null;
      const route = this.currentStatus && this.currentStatus.route ? this.currentStatus.route : this.currentRoute;
      const pose = robot && robot.pose ? robot.pose : null;

      this.dom.connectionBadge.textContent = robot && robot.connected ? "online" : "offline";
      this.dom.connectionBadge.className = robot && robot.connected ? "badge" : "badge error";
      if (this.dom.summaryConnectionText) {
        this.dom.summaryConnectionText.textContent = robot && robot.connected ? "online" : "offline";
      }
      this.dom.modeText.textContent = robot ? robot.state : "-";
      this.dom.localizationText.textContent = robot
        ? `${robot.localizationOk ? "ok" : "waiting"} (${Number(robot.localizationAgeSec || 0).toFixed(2)} s)`
        : "-";
      this.dom.nearestText.textContent = robot && robot.nearestLm ? robot.nearestLm : "-";
      this.dom.targetText.textContent = robot && robot.targetLm ? robot.targetLm : (route && route.goalLm ? route.goalLm : "-");
      this.dom.edgeText.textContent = robot && robot.currentEdgeId ? robot.currentEdgeId : "-";
      this.dom.progressText.textContent = robot ? `${Math.round((Number(robot.routeProgress || 0) * 100))}%` : "0%";

      this.dom.mapStateText.textContent = robot && robot.state ? robot.state : "-";
      this.dom.mapNearestText.textContent = robot && robot.nearestLm ? robot.nearestLm : "-";
      this.dom.mapTargetText.textContent = robot && robot.targetLm
        ? robot.targetLm
        : (this.dom.goalSelect.value || "-");

      if (pose) {
        this.dom.poseText.textContent = `x: ${Number(pose.x).toFixed(3)}, y: ${Number(pose.y).toFixed(3)}, yaw: ${Number(pose.yaw).toFixed(3)}`;
        if (this.dom.poseSummaryText) {
          this.dom.poseSummaryText.textContent = this.dom.poseText.textContent;
        }
      } else {
        this.dom.poseText.textContent = "x: -, y: -, yaw: -";
        if (this.dom.poseSummaryText) {
          this.dom.poseSummaryText.textContent = this.dom.poseText.textContent;
        }
      }

      if (robot && robot.velocity) {
        this.dom.velocityText.textContent =
          `v: ${Number(robot.velocity.linear || 0).toFixed(3)}, w: ${Number(robot.velocity.angular || 0).toFixed(3)}`;
      } else {
        this.dom.velocityText.textContent = "v: -, w: -";
      }

      this.dom.routeIdText.textContent = route && route.routeId ? route.routeId : "-";
      this.dom.routeLengthText.textContent = route ? `${Number(route.length || 0).toFixed(2)} m` : "0.00 m";
      this.dom.routeNodesText.textContent = route && Array.isArray(route.nodes) && route.nodes.length
        ? route.nodes.join(" -> ")
        : "No route planned.";
      if (this.dom.routeList) {
        this.dom.routeList.innerHTML = "";
        if (route && Array.isArray(route.nodes) && route.nodes.length) {
          for (const name of route.nodes) {
            const item = document.createElement("li");
            item.textContent = name;
            this.dom.routeList.appendChild(item);
          }
        } else {
          const item = document.createElement("li");
          item.textContent = "No route";
          this.dom.routeList.appendChild(item);
        }
      }

      this.renderer.drawRoute(route);
      this.renderer.drawLandmarks(robot ? robot.nearestLm : "", this.dom.goalSelect.value, this.navigateMode);
      const robotPixel = this.renderer.drawRobot(pose);
      if (robotPixel) {
        this.viewport.focusOn(robotPixel);
      }

      this.renderEvents(this.currentStatus && Array.isArray(this.currentStatus.events) ? this.currentStatus.events : []);

      if (robot && robot.message) {
        this.setStatus(robot.message);
      }
    }

    renderEvents(events) {
      const signature = events.map((item) => `${item.stamp}:${item.message}`).join("|");
      if (signature === this.lastEventSignature) {
        return;
      }
      this.lastEventSignature = signature;
      if (!events.length) {
        this.dom.eventLog.textContent = "No events yet.";
        return;
      }
      this.dom.eventLog.innerHTML = "";
      for (const event of events.slice().reverse()) {
        const row = document.createElement("div");
        const stamp = event.stamp ? new Date(event.stamp * 1000).toLocaleTimeString([], { hour12: false }) : "--:--:--";
        row.textContent = `${stamp} ${event.level || "info"} ${event.message || ""}`;
        this.dom.eventLog.appendChild(row);
      }
    }

    setStatus(text) {
      this.dom.statusText.textContent = text;
    }

    toggleNavigateMode() {
      this.navigateMode = !this.navigateMode;
      this.syncModeButtons();
      this.render();
      this.setStatus(this.navigateMode ? "Navigate armed: select an LM." : "Navigate canceled.");
    }

    clearNavigateMode(render = true) {
      if (!this.navigateMode) {
        return;
      }
      this.navigateMode = false;
      this.syncModeButtons();
      if (render) {
        this.render();
      }
    }

    syncModeButtons() {
      this.dom.navigateButton.classList.toggle("active", this.navigateMode);
      this.dom.navigateButton.textContent = this.navigateMode ? "Select LM" : "Navigate To LM";
    }

    handleNavigatePointerDown(event) {
      if (!this.navigateMode) {
        this.navigatePointerDown = null;
        return;
      }
      this.navigatePointerDown = {
        x: event.clientX,
        y: event.clientY,
      };
    }

    handleNavigatePointerUp(event) {
      if (!this.navigateMode) {
        return;
      }
      const down = this.navigatePointerDown;
      this.navigatePointerDown = null;
      if (down) {
        const moved = Math.hypot(event.clientX - down.x, event.clientY - down.y);
        if (moved > 10) {
          return;
        }
      }
      const selected = this.selectNavigateTargetFromEvent(event);
      if (selected) {
        this.suppressNextNavigateClick = true;
        window.setTimeout(() => {
          this.suppressNextNavigateClick = false;
        }, 250);
        event.preventDefault();
        event.stopPropagation();
      }
    }

    handleMapNavigateClick(event) {
      if (this.suppressNextNavigateClick) {
        this.suppressNextNavigateClick = false;
        return;
      }
      if (!this.navigateMode) {
        return;
      }
      this.selectNavigateTargetFromEvent(event);
    }

    selectNavigateTargetFromEvent(event) {
      const world = this.geometry.eventToWorld(event, this.dom.viewport);
      if (!world) {
        return false;
      }
      const nearest = this.graphModel.nearestLandmark(world, this.geometry);
      if (!nearest.landmark) {
        return false;
      }
      if (nearest.distance > 1.20) {
        this.setStatus("Navigate armed: click closer to an LM.");
        return false;
      }
      this.startNavigation(nearest.landmark.name);
      return true;
    }

    async startNavigation(goalLm) {
      this.clearNavigateMode(false);
      this.releaseManualControl();
      this.dom.goalSelect.value = goalLm;
      try {
        const result = await this.postJson("/api/robot/route/execute", { goalLm });
        if (result && result.route) {
          this.currentRoute = result.route;
        }
        this.setActiveTab("status");
        this.setStatus(`Route execution started to ${goalLm}.`);
        await this.fetchStatus();
      } catch (error) {
        this.setStatus(`Navigate failed: ${error.message || error}`);
      }
    }

    setInputValue(input, value) {
      if (value === undefined || value === null || value === "") {
        return;
      }
      input.value = String(value);
    }

    async getJson(url) {
      const response = await fetch(url, { cache: "no-store" });
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      return response.json();
    }

    async postJson(url, payload) {
      const response = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      return response.json();
    }
  }

  new RobotApp(data).init();
}());
