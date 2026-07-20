export const withMapView = (Base) => class OperatorAppMapView extends Base {
  renderOperatorMap() {
    const payload = this.activeOperatorMapPayload();
    if (!payload || !payload.map) {
      this.operatorMapSvg.setAttribute("viewBox", "0 0 100 100");
      this.operatorMapImage.removeAttribute("href");
      if (this.operatorObstacleLayer) {
        this.operatorObstacleLayer.innerHTML = "";
      }
      this.operatorGraphLayer.innerHTML = "";
      this.operatorRouteLayer.innerHTML = "";
      this.operatorLookaheadLayer.innerHTML = "";
      this.operatorLandmarkLayer.innerHTML = "";
      this.operatorEditorLayer.innerHTML = "";
      this.clearScanOverlay();
      this.operatorRobotLayer.innerHTML = "";
      this.clearRelocationPreview();
      this.syncMapControls();
      return;
    }
    if (
      this.fleetMapEditorActive
      && this.fleetRasterDraftRef !== this.ensureFleetMapDraft()
      && !this.fleetRasterLoadPromise
    ) {
      this.ensureFleetRasterGrid();
    }
    if (!this.babylonMapFailed) {
      this.syncMapControls();
      this.renderOperatorBabylonMap();
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
    this.drawMapObstacles(payload);
    this.drawGraph();
    this.drawRoute();
    this.drawLookahead();
    this.drawLandmarks();
    this.drawFleetEditorOverlay();
    this.drawScanOverlay();
    this.drawRobot();
    this.syncMapControls();
  }

  activeOperatorMapPayload() {
    if (this.isFleetManager() && this.fleetMapEditorActive && this.fleetMapDraft) {
      return this.fleetMapDraft;
    }
    if (!this.isFleetManager() && this.slamActive) {
      return this.slamMapPayload || null;
    }
    return this.operatorMapPayload;
  }

  drawMapObstacles(payload = this.activeOperatorMapPayload()) {
    if (!this.operatorObstacleLayer) {
      return;
    }
    this.operatorObstacleLayer.innerHTML = "";
    if (!payload || !payload.map) {
      return;
    }
    const explicit = Array.isArray(payload.obstacles) ? payload.obstacles : [];
    const obstacles = explicit.length ? explicit : this.syntheticRackObstacles(payload);
    for (const obstacle of obstacles) {
      const rect = this.obstacleRectToPixels(obstacle);
      if (!rect) {
        continue;
      }
      const element = document.createElementNS("http://www.w3.org/2000/svg", "rect");
      element.setAttribute("class", obstacle.kind === "rack" ? "map-obstacle rack" : "map-obstacle");
      element.setAttribute("x", String(rect.x));
      element.setAttribute("y", String(rect.y));
      element.setAttribute("width", String(rect.width));
      element.setAttribute("height", String(rect.height));
      element.setAttribute("rx", String(rect.radius));
      element.setAttribute("ry", String(rect.radius));
      this.operatorObstacleLayer.append(element);
    }
  }

  obstacleRectToPixels(obstacle) {
    const map = this.activeOperatorMapPayload()?.map || {};
    const resolution = Number(map.resolution || 0);
    if (!resolution) {
      return null;
    }
    const width = Number(obstacle.width || obstacle.w || 0);
    const height = Number(obstacle.height || obstacle.h || 0);
    if (width <= 0 || height <= 0) {
      return null;
    }
    const center = this.worldToPixel({
      x: Number(obstacle.x || 0),
      y: Number(obstacle.y || 0),
    });
    return {
      x: center.x - (width / resolution / 2),
      y: center.y - (height / resolution / 2),
      width: width / resolution,
      height: height / resolution,
      radius: Math.max(1.5, Math.min(width, height) / resolution * 0.08),
    };
  }

  syntheticRackObstacles(payload) {
    if (!this.shouldDrawSyntheticRacks(payload)) {
      return [];
    }
    const landmarks = Array.isArray(payload.lms) ? payload.lms : [];
    const xs = this.uniqueSortedNumbers(landmarks.map((lm) => lm.x));
    const ys = this.uniqueSortedNumbers(landmarks.map((lm) => lm.y));
    if (xs.length < 4 || ys.length < 4) {
      return [];
    }
    const stepX = this.medianStep(xs);
    const stepY = this.medianStep(ys);
    if (stepX <= 0 || stepY <= 0) {
      return [];
    }
    const rackWidth = stepX * 0.58;
    const rackHeight = stepY * 0.58;
    const obstacles = [];
    for (let row = 0; row < ys.length - 1; row += 1) {
      for (let col = 0; col < xs.length - 1; col += 1) {
        if (this.syntheticRackAisleCell(row, col)) {
          continue;
        }
        obstacles.push({
          kind: "rack",
          x: (xs[col] + xs[col + 1]) / 2,
          y: (ys[row] + ys[row + 1]) / 2,
          width: rackWidth,
          height: rackHeight,
        });
      }
    }
    return obstacles.slice(0, 1600);
  }

  shouldDrawSyntheticRacks(payload) {
    const mapName = String(payload?.mapName || payload?.map?.mapName || "").toLowerCase();
    const landmarks = Array.isArray(payload?.lms) ? payload.lms : [];
    if (landmarks.length < 250) {
      return false;
    }
    if (mapName.includes("benchmark") || mapName.includes("kiva")) {
      return true;
    }
    const benchmarkCount = landmarks.filter((lm) => lm?.properties?.benchmark).length;
    return benchmarkCount > landmarks.length * 0.75;
  }

  syntheticRackAisleCell(row, col) {
    return row % 6 === 5 || col % 6 === 5;
  }

  uniqueSortedNumbers(values) {
    return Array.from(new Set(
      values
        .map((value) => Number(value))
        .filter((value) => Number.isFinite(value))
        .map((value) => Math.round(value * 1000) / 1000)
    )).sort((a, b) => a - b);
  }

  medianStep(values) {
    const steps = [];
    for (let index = 0; index < values.length - 1; index += 1) {
      const step = values[index + 1] - values[index];
      if (step > 0.001) {
        steps.push(step);
      }
    }
    if (!steps.length) {
      return 0;
    }
    steps.sort((a, b) => a - b);
    return steps[Math.floor(steps.length / 2)];
  }

  drawGraph() {
    const payload = this.activeOperatorMapPayload();
    const landmarks = this.landmarkIndex();
    const edges = Array.isArray(payload?.edges) ? payload.edges : [];
    const directedKeys = new Set(
      edges.map((edge) => this.edgeKey(edge.from, edge.to)),
    );
    this.operatorGraphLayer.innerHTML = "";
    const profile = this.mapVisualProfile(payload);
    const strokeWidth = profile.unit(profile.massive ? 0.55 : (profile.dense ? 0.75 : 1.05));
    if (!this.fleetMapEditorActive && profile.dense) {
      this.drawGraphBulk(payload, landmarks, strokeWidth);
      if (this.edgeDirectionsVisible) {
        this.drawGraphDirectionBulk(edges, landmarks, directedKeys);
      }
      return;
    }
    for (const edge of edges) {
      const element = document.createElementNS("http://www.w3.org/2000/svg", edge.geometry === "bezier" ? "path" : "line");
      const edgeKey = this.edgeKey(edge.from, edge.to);
      element.setAttribute("class", [
        "graph-edge",
        this.fleetMapEditorActive ? "editable" : "",
        this.fleetMapEditorActive && edge.properties?.controlled_region ? "controlled" : "",
        edgeKey === this.fleetSelectedEdgeKey ? "selected" : "",
      ].filter(Boolean).join(" "));
      element.style.strokeWidth = String(this.fleetMapEditorActive ? profile.unit(1.8) : strokeWidth);
      element.dataset.edgeKey = edgeKey;
      element.addEventListener("pointerdown", (event) => {
        if (
          !this.fleetMapEditorActive
          || this.fleetMapTool !== "select"
          || event.button !== 0
        ) {
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
      const hasReverse = directedKeys.has(this.edgeKey(edge.to, edge.from));
      const arrow = this.edgeDirectionsVisible
        ? this.directionArrow(edge, landmarks, hasReverse ? 0.56 : 0.5)
        : null;
      if (arrow) {
        this.operatorGraphLayer.append(arrow);
      }
    }
  }

  drawGraphBulk(payload, landmarks, strokeWidth) {
    const edges = Array.isArray(payload?.edges) ? payload.edges : [];
    const commands = [];
    const seen = new Set();
    for (const edge of edges) {
      const key = [String(edge.from || ""), String(edge.to || "")].sort().join("|");
      const geometryKey = `${key}:${edge.geometry || "line"}`;
      if (seen.has(geometryKey)) {
        continue;
      }
      seen.add(geometryKey);
      if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
        const points = edge.control_points.map((point) => this.worldToPixel(point));
        commands.push(`M ${points[0].x} ${points[0].y} C ${points[1].x} ${points[1].y}, ${points[2].x} ${points[2].y}, ${points[3].x} ${points[3].y}`);
        continue;
      }
      const start = landmarks.get(edge.from);
      const goal = landmarks.get(edge.to);
      if (!start || !goal) {
        continue;
      }
      const startPx = this.worldToPixel(start);
      const goalPx = this.worldToPixel(goal);
      commands.push(`M ${startPx.x} ${startPx.y} L ${goalPx.x} ${goalPx.y}`);
    }
    if (!commands.length) {
      return;
    }
    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("class", "graph-edge graph-edge-bulk");
    path.setAttribute("d", commands.join(" "));
    path.style.strokeWidth = String(strokeWidth);
    this.operatorGraphLayer.append(path);
  }

  drawGraphDirectionBulk(edges, landmarks, directedKeys) {
    const commands = [];
    for (const edge of edges) {
      const hasReverse = directedKeys.has(this.edgeKey(edge.to, edge.from));
      const points = this.directionArrowPoints(edge, landmarks, hasReverse ? 0.56 : 0.5);
      if (!points) {
        continue;
      }
      commands.push(
        `M ${points.tip.x} ${points.tip.y} `
        + `L ${points.left.x} ${points.left.y} `
        + `L ${points.right.x} ${points.right.y} Z`,
      );
    }
    if (!commands.length) {
      return;
    }
    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("class", "graph-direction graph-direction-bulk");
    path.setAttribute("d", commands.join(" "));
    this.operatorGraphLayer.append(path);
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
    this.drawSlamTrail();
    if (this.slamActive) {
      return;
    }
    const route = (this.currentStatus && this.currentStatus.route) || this.currentRoute;
    if (!route || !Array.isArray(route.trajectory) || route.trajectory.length < 2) {
      return;
    }
    const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
    polyline.setAttribute("class", "planned-route");
    polyline.style.strokeWidth = String(this.routeStrokeWidth("planned-route"));
    polyline.setAttribute("points", route.trajectory.map((point) => {
      const px = this.worldToPixel(point);
      return `${px.x},${px.y}`;
    }).join(" "));
    this.operatorRouteLayer.append(polyline);
  }

  drawSlamTrail() {
    const trail = Array.isArray(this.slamMapFrame?.trail) ? this.slamMapFrame.trail : [];
    if (!this.slamActive || trail.length < 2) {
      return;
    }
    this.appendRoutePolyline(trail.map((point) => this.displayPointForActiveMap(point)), "slam-trail");
  }

  drawFleetRoute(robot) {
    const trajectory = robot.trajectory || [];
    const active = robot.name === this.selectedFleetRobotName;
    const preview = Array.isArray(robot.routePreview) ? robot.routePreview : [];
    if (active && preview.length >= 2) {
      this.appendRoutePolyline(preview, "fleet-route-preview active", this.fleetRobotColor(robot.name));
    }
    this.appendRoutePolyline(
      trajectory,
      active ? "fleet-route-plan active" : "fleet-route-plan",
      active ? this.fleetRobotColor(robot.name) : "",
    );
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
      this.appendRoutePolyline(remaining, "fleet-route-active", this.fleetRobotColor(robot.name));
    }
  }

  appendRoutePolyline(points, className, stroke = "") {
    if (!Array.isArray(points) || points.length < 2) {
      return;
    }
    const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
    polyline.setAttribute("class", className);
    polyline.style.strokeWidth = String(this.routeStrokeWidth(className));
    if (stroke) {
      polyline.style.stroke = stroke;
    }
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
    // Trajectory timestamps are monotonic. Binary search avoids scanning the
    // complete rolling trajectory for every robot on every animation frame.
    let low = 0;
    let high = points.length - 1;
    while (low + 1 < high) {
      const middle = Math.floor((low + high) / 2);
      const middleTime = Number(points[middle].t ?? middle);
      if (middleTime <= targetTime) {
        low = middle;
      } else {
        high = middle;
      }
    }
    const start = points[low];
    const goal = points[high];
    const t0 = Number(start.t ?? low);
    const t1 = Number(goal.t ?? high);
    const ratio = (targetTime - t0) / Math.max(0.000001, t1 - t0);
    return {
      ...start,
      x: Number(start.x || 0) + ((Number(goal.x || 0) - Number(start.x || 0)) * ratio),
      y: Number(start.y || 0) + ((Number(goal.y || 0) - Number(start.y || 0)) * ratio),
      yaw: this.interpolateAngle(Number(start.yaw || 0), Number(goal.yaw || 0), ratio),
      t: targetTime,
    };
  }

  interpolateAngle(start, goal, ratio) {
    // JavaScript's % keeps the dividend sign, so the common modulo formula
    // can turn a +90 degree wrap across -PI/PI into a visual -270 degree spin.
    const rawDelta = goal - start;
    const delta = Math.atan2(Math.sin(rawDelta), Math.cos(rawDelta));
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
      footprint.style.strokeWidth = String(this.routeStrokeWidth("lookahead-footprint"));
      this.operatorLookaheadLayer.append(footprint);
    });
    const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
    polyline.setAttribute("class", this.fleetManualLookahead.blocked ? "lookahead-route blocked" : "lookahead-route");
    polyline.style.strokeWidth = String(this.routeStrokeWidth("lookahead-route"));
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
    marker.setAttribute("r", String(this.routeStrokeWidth("lookahead-marker") * 1.6));
    this.operatorLookaheadLayer.append(marker);
  }

  routeStrokeWidth(className = "") {
    const profile = this.mapVisualProfile();
    const routeClass = String(className);
    if (routeClass.includes("preview") && routeClass.includes("active")) {
      return profile.unit(profile.massive ? 3.2 : 4.2);
    }
    if (routeClass.includes("route-active")) {
      return profile.unit(profile.massive ? 2.4 : 3.2);
    }
    if (routeClass.includes("active")) {
      return profile.unit(profile.massive ? 1.8 : 2.2);
    }
    if (routeClass.includes("done") || routeClass.includes("plan")) {
      return profile.unit(profile.massive ? 1.1 : 1.5);
    }
    return profile.unit(profile.massive ? 1.2 : 1.7);
  }

  drawLandmarks() {
    const statusRobot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    const fleetRobot = this.isFleetManager() ? this.selectedFleetRobot() : null;
    const target = fleetRobot ? (fleetRobot.targetLm || "") : (statusRobot.targetLm || (this.currentRoute && this.currentRoute.goalLm) || "");
    const nearest = fleetRobot ? (fleetRobot.currentLm || "") : statusRobot.nearestLm;
    const payload = this.activeOperatorMapPayload();
    this.operatorLandmarkLayer.innerHTML = "";
    if (!payload || !payload.map) {
      return;
    }
    const style = this.landmarkRenderStyle(payload);
    const corridorHoldingLms = new Set(
      (payload.edges || [])
        .filter((edge) => edge.properties?.controlled_region)
        .flatMap((edge) => [edge.from, edge.to]),
    );
    for (const landmark of payload.lms || []) {
      const px = this.worldToPixel(landmark);
      const isNearest = landmark.name === nearest;
      const isTarget = landmark.name === target;
      const isSelected = this.fleetMapEditorActive && landmark.name === this.fleetSelectedLmName;
      const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
      group.setAttribute("class", [
        "landmark",
        style.compact && !this.lmNamesVisible ? "compact" : "",
        isNearest ? "nearest" : "",
        isTarget ? "target" : "",
        isSelected ? "selected" : "",
        this.fleetMapEditorActive && landmark.properties?.controlled_region ? "corridor-internal" : "",
        this.fleetMapEditorActive
          && landmark.properties?.holding_point
          && corridorHoldingLms.has(landmark.name)
          ? "corridor-holding"
          : "",
        (this.navigateMode || this.relocateMode) ? "armed" : "",
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
      const hit = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      hit.setAttribute("class", "landmark-hit");
      hit.setAttribute("cx", String(px.x));
      hit.setAttribute("cy", String(px.y));
      hit.setAttribute("r", String(style.hitRadius));
      group.append(hit);
      const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      circle.setAttribute("class", "landmark-dot");
      circle.setAttribute("cx", String(px.x));
      circle.setAttribute("cy", String(px.y));
      circle.setAttribute("r", String(isNearest || isTarget || isSelected ? style.emphasisRadius : style.radius));
      circle.style.strokeWidth = String(style.strokeWidth);
      group.append(circle);
      if (this.lmNamesVisible) {
        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("class", "landmark-label");
        label.setAttribute("x", String(px.x));
        label.setAttribute("y", String(px.y + style.labelOffset));
        label.setAttribute("font-size", String(style.labelFontSize));
        label.style.strokeWidth = String(style.labelStrokeWidth);
        label.textContent = landmark.name;
        group.append(label);
      }
      this.operatorLandmarkLayer.append(group);
    }
  }

  landmarkRenderStyle(payload) {
    const count = Array.isArray(payload?.lms) ? payload.lms.length : 0;
    const profile = this.mapVisualProfile(payload);
    const zoom = Math.max(1, Number(this.mapView.scale || 1));
    if (count >= 900) {
      return {
        compact: zoom < 2.6,
        radius: profile.unit(1.55),
        emphasisRadius: profile.unit(2.6),
        hitRadius: profile.unit(8),
        labelOffset: profile.unit(8),
        labelFontSize: profile.unit(6.8),
        labelStrokeWidth: profile.unit(1.8),
        strokeWidth: profile.unit(0.75),
      };
    }
    if (count >= 400) {
      return {
        compact: zoom < 2.1,
        radius: profile.unit(1.9),
        emphasisRadius: profile.unit(3.0),
        hitRadius: profile.unit(8.5),
        labelOffset: profile.unit(9),
        labelFontSize: profile.unit(7.4),
        labelStrokeWidth: profile.unit(1.9),
        strokeWidth: profile.unit(0.8),
      };
    }
    if (count >= 160) {
      return {
        compact: zoom < 1.55,
        radius: profile.unit(2.3),
        emphasisRadius: profile.unit(3.7),
        hitRadius: profile.unit(9),
        labelOffset: profile.unit(10),
        labelFontSize: profile.unit(8),
        labelStrokeWidth: profile.unit(2.0),
        strokeWidth: profile.unit(0.9),
      };
    }
    return {
      compact: false,
      radius: profile.unit(3.2),
      emphasisRadius: profile.unit(4.8),
      hitRadius: profile.unit(10),
      labelOffset: profile.unit(12),
      labelFontSize: profile.unit(8.5),
      labelStrokeWidth: profile.unit(2.1),
      strokeWidth: profile.unit(1.0),
    };
  }

  drawRobotUncached() {
    this.operatorRobotLayer.innerHTML = "";
    const robotStyle = this.robotRenderStyle();
    const robot = this.statusForRobotDisplay(this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : null);
    const rawPose = this.slamActive && this.slamMapFrame?.pose
      ? this.slamMapFrame.pose
      : (robot && robot.pose ? robot.pose : null);
    const pose = rawPose ? this.displayPoseForActiveMap(rawPose) : null;
    const canDraw = this.slamActive
      ? Boolean(pose && this.slamMapPayload)
      : Boolean(pose && robot.connected && robot.localizationOk);
    if (!canDraw) {
      return;
    }
    const center = this.worldToPixel(pose);
    if (this.mapView.follow) {
      this.focusMapOn(center);
    }
    const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const footprint = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
    footprint.setAttribute("class", "robot-footprint");
    footprint.setAttribute("points", this.robotFootprintPoints(pose));
    footprint.style.strokeWidth = String(robotStyle.footprintStrokeWidth);
    group.append(footprint);
    const centerDot = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    centerDot.setAttribute("class", "robot-center-dot");
    centerDot.setAttribute("cx", String(center.x));
    centerDot.setAttribute("cy", String(center.y));
    centerDot.setAttribute("r", String(robotStyle.centerRadius));
    centerDot.style.strokeWidth = String(robotStyle.centerStrokeWidth);
    group.append(centerDot);
    const heading = document.createElementNS("http://www.w3.org/2000/svg", "line");
    heading.setAttribute("class", "robot-heading");
    heading.setAttribute("x1", String(center.x));
    heading.setAttribute("y1", String(center.y));
    heading.setAttribute("x2", String(center.x + Math.cos(Number(pose.yaw || 0)) * robotStyle.headingLength));
    heading.setAttribute("y2", String(center.y + Math.sin(Number(pose.yaw || 0)) * robotStyle.headingLength));
    heading.style.strokeWidth = String(robotStyle.headingStrokeWidth);
    group.append(heading);
    this.operatorRobotLayer.append(group);
  }

  drawRobot(motionOnly = false) {
    if (this.fleetMapEditorActive) {
      this.operatorRobotLayer.innerHTML = "";
      this.fleetRobotSvgEntries?.clear();
      this.fleetWaitDependencyLine = null;
      return;
    }
    if (!this.isFleetManager()) {
      this.fleetRobotSvgEntries?.clear();
      this.fleetWaitDependencyLine = null;
      this.drawRobotUncached();
      return;
    }
    if (motionOnly && this.drawFleetRobotMotionLayer(this.robotRenderStyle())) {
      return;
    }
    this.drawFleetRobotLayer(this.robotRenderStyle());
  }

  drawFleetRobotMotionLayer(robotStyle) {
    const robots = this.fleetRenderRobots();
    const drawableRobots = robots.filter((robot) => robot?.name && robot?.pose);
    if (
      !this.fleetRobotSvgEntries
      || drawableRobots.length !== this.fleetRobotSvgEntries.size
      || drawableRobots.some((robot) => !this.fleetRobotSvgEntries.has(String(robot.name)))
    ) {
      return false;
    }

    const selectedRobot = this.selectedFleetRobot(robots);
    const waitBlockerName = this.fleetRobotWaitBlockerName(selectedRobot);
    const waitBlocker = waitBlockerName
      ? robots.find((robot) => robot.name === waitBlockerName)
      : null;
    const dependencyLine = this.fleetWaitDependencyLine;
    if (dependencyLine && selectedRobot?.pose && waitBlocker?.pose) {
      const waitingCenter = this.worldToPixel(selectedRobot.pose);
      const blockerCenter = this.worldToPixel(waitBlocker.pose);
      dependencyLine.setAttribute("x1", String(waitingCenter.x));
      dependencyLine.setAttribute("y1", String(waitingCenter.y));
      dependencyLine.setAttribute("x2", String(blockerCenter.x));
      dependencyLine.setAttribute("y2", String(blockerCenter.y));
    }

    let focused = false;
    for (const robot of drawableRobots) {
      const name = String(robot.name);
      const pose = robot.pose;
      const entry = this.fleetRobotSvgEntries.get(name);
      const center = this.worldToPixel(pose);
      if (!focused && this.mapView.follow && name === this.selectedFleetRobotName) {
        this.focusMapOn(center);
        focused = true;
      }
      const footprintPoints = this.robotFootprintPoints(pose);
      entry.blockerHalo.setAttribute("points", footprintPoints);
      entry.selectionHalo.setAttribute("points", footprintPoints);
      entry.footprint.setAttribute("points", footprintPoints);
      entry.centerDot.setAttribute("cx", String(center.x));
      entry.centerDot.setAttribute("cy", String(center.y));
      entry.heading.setAttribute("x1", String(center.x));
      entry.heading.setAttribute("y1", String(center.y));
      entry.heading.setAttribute(
        "x2",
        String(center.x + Math.cos(Number(pose.yaw || 0)) * robotStyle.headingLength),
      );
      entry.heading.setAttribute(
        "y2",
        String(center.y + Math.sin(Number(pose.yaw || 0)) * robotStyle.headingLength),
      );
      entry.label.setAttribute("x", String(center.x));
      entry.label.setAttribute("y", String(center.y + robotStyle.labelOffset));
      entry.alertLabel.setAttribute("x", String(center.x));
      entry.alertLabel.setAttribute("y", String(center.y - robotStyle.labelOffset));
      entry.waitLabel.setAttribute("x", String(center.x));
      entry.waitLabel.setAttribute(
        "y",
        String(center.y + robotStyle.labelOffset + (robotStyle.labelFontSize * 1.15)),
      );
    }
    return true;
  }

  drawFleetRobotLayer(robotStyle) {
    const layer = this.operatorRobotLayer;
    if (!this.fleetRobotSvgEntries) {
      this.fleetRobotSvgEntries = new Map();
    }
    if (this.fleetWaitDependencyLine?.parentNode !== layer) {
      this.fleetWaitDependencyLine = document.createElementNS(
        "http://www.w3.org/2000/svg",
        "line",
      );
      this.fleetWaitDependencyLine.setAttribute("class", "robot-wait-dependency-link");
      layer.prepend(this.fleetWaitDependencyLine);
    }

    const robots = this.fleetRenderRobots();
    const selectedRobot = this.selectedFleetRobot(robots);
    const waitBlockerName = this.fleetRobotWaitBlockerName(selectedRobot);
    const waitBlocker = waitBlockerName
      ? robots.find((robot) => robot.name === waitBlockerName)
      : null;
    const dependencyLine = this.fleetWaitDependencyLine;
    if (selectedRobot?.pose && waitBlocker?.pose) {
      const waitingCenter = this.worldToPixel(selectedRobot.pose);
      const blockerCenter = this.worldToPixel(waitBlocker.pose);
      dependencyLine.style.display = "";
      dependencyLine.setAttribute("x1", String(waitingCenter.x));
      dependencyLine.setAttribute("y1", String(waitingCenter.y));
      dependencyLine.setAttribute("x2", String(blockerCenter.x));
      dependencyLine.setAttribute("y2", String(blockerCenter.y));
      dependencyLine.style.stroke = this.fleetRobotColor(selectedRobot.name);
    } else {
      dependencyLine.style.display = "none";
    }

    const incoming = new Set();
    let focused = false;
    for (const robot of robots) {
      const name = String(robot?.name || "");
      const pose = robot?.pose;
      if (!name || !pose) {
        continue;
      }
      incoming.add(name);
      const center = this.worldToPixel(pose);
      if (!focused && this.mapView.follow && name === this.selectedFleetRobotName) {
        this.focusMapOn(center);
        focused = true;
      }

      let entry = this.fleetRobotSvgEntries.get(name);
      if (entry?.group?.parentNode !== layer) {
        if (entry) {
          this.fleetRobotSvgEntries.delete(name);
        }
        const element = (tag, className) => {
          const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
          node.setAttribute("class", className);
          return node;
        };
        const group = element("g", "fleet-robot");
        group.dataset.robotName = name;
        const selectRobot = (event) => {
          if (this.fleetMapEditorActive) {
            return;
          }
          event.preventDefault();
          event.stopPropagation();
          this.selectFleetRobotByName(name);
        };
        group.addEventListener("pointerdown", (event) => {
          if (event.button === 0) {
            selectRobot(event);
          }
        });
        group.addEventListener("click", selectRobot);
        entry = {
          group,
          blockerHalo: element("polygon", "robot-wait-blocker-halo"),
          selectionHalo: element("polygon", "robot-selection-halo"),
          footprint: element("polygon", "robot-footprint"),
          centerDot: element("circle", "robot-center-dot"),
          heading: element("line", "robot-heading"),
          label: element("text", "robot-label"),
          alertLabel: element("text", "robot-alert-label warning"),
          waitLabel: element("text", "robot-wait-label"),
        };
        group.append(
          entry.blockerHalo,
          entry.selectionHalo,
          entry.footprint,
          entry.centerDot,
          entry.heading,
          entry.label,
          entry.alertLabel,
          entry.waitLabel,
        );
        layer.append(group);
        this.fleetRobotSvgEntries.set(name, entry);
      }

      const active = name === this.selectedFleetRobotName;
      const waitBlockerActive = Boolean(waitBlockerName && name === waitBlockerName);
      const footprintPoints = this.robotFootprintPoints(pose);
      entry.group.setAttribute("class", [
        "fleet-robot",
        active ? "active" : "",
        waitBlockerActive ? "wait-blocker" : "",
      ].filter(Boolean).join(" "));

      entry.blockerHalo.style.display = waitBlockerActive ? "" : "none";
      entry.blockerHalo.setAttribute("points", footprintPoints);
      entry.blockerHalo.style.strokeWidth = String(robotStyle.footprintStrokeWidth * 4.4);
      entry.selectionHalo.style.display = active ? "" : "none";
      entry.selectionHalo.setAttribute("points", footprintPoints);
      entry.selectionHalo.style.stroke = this.fleetRobotColor(name);
      entry.selectionHalo.style.strokeWidth = String(robotStyle.footprintStrokeWidth * 3.6);

      entry.footprint.setAttribute("points", footprintPoints);
      entry.footprint.style.strokeWidth = String(robotStyle.footprintStrokeWidth);
      entry.footprint.style.stroke = active ? this.fleetRobotColor(name) : "";
      entry.footprint.style.fill = active ? `${this.fleetRobotColor(name)}2e` : "";
      entry.centerDot.setAttribute("cx", String(center.x));
      entry.centerDot.setAttribute("cy", String(center.y));
      entry.centerDot.setAttribute("r", String(robotStyle.centerRadius));
      entry.centerDot.style.strokeWidth = String(robotStyle.centerStrokeWidth);
      entry.heading.setAttribute("x1", String(center.x));
      entry.heading.setAttribute("y1", String(center.y));
      entry.heading.setAttribute(
        "x2",
        String(center.x + Math.cos(Number(pose.yaw || 0)) * robotStyle.headingLength),
      );
      entry.heading.setAttribute(
        "y2",
        String(center.y + Math.sin(Number(pose.yaw || 0)) * robotStyle.headingLength),
      );
      entry.heading.style.strokeWidth = String(robotStyle.headingStrokeWidth);

      entry.label.setAttribute("x", String(center.x));
      entry.label.setAttribute("y", String(center.y + robotStyle.labelOffset));
      entry.label.style.fontSize = String(robotStyle.labelFontSize);
      entry.label.style.strokeWidth = String(robotStyle.labelStrokeWidth);
      if (entry.label.textContent !== name) {
        entry.label.textContent = name;
      }

      const alertText = this.fleetRobotAlertLabel(robot);
      entry.alertLabel.style.display = alertText ? "" : "none";
      entry.alertLabel.setAttribute(
        "class",
        `robot-alert-label ${this.fleetRobotAlertSeverity(robot)}`,
      );
      entry.alertLabel.setAttribute("x", String(center.x));
      entry.alertLabel.setAttribute("y", String(center.y - robotStyle.labelOffset));
      entry.alertLabel.style.fontSize = String(robotStyle.labelFontSize * 0.86);
      entry.alertLabel.style.strokeWidth = String(robotStyle.labelStrokeWidth * 0.9);
      if (entry.alertLabel.textContent !== alertText) {
        entry.alertLabel.textContent = alertText;
      }

      const waitText = active ? this.fleetRobotWaitLabel(robot) : "";
      entry.waitLabel.style.display = waitText ? "" : "none";
      entry.waitLabel.setAttribute("x", String(center.x));
      entry.waitLabel.setAttribute(
        "y",
        String(center.y + robotStyle.labelOffset + (robotStyle.labelFontSize * 1.15)),
      );
      entry.waitLabel.style.fontSize = String(robotStyle.labelFontSize * 0.82);
      entry.waitLabel.style.strokeWidth = String(robotStyle.labelStrokeWidth * 0.82);
      if (entry.waitLabel.textContent !== waitText) {
        entry.waitLabel.textContent = waitText;
      }
    }

    for (const [name, entry] of this.fleetRobotSvgEntries.entries()) {
      if (incoming.has(name)) {
        continue;
      }
      entry.group.remove();
      this.fleetRobotSvgEntries.delete(name);
    }
  }

  robotRenderStyle(payload = this.activeOperatorMapPayload()) {
    const profile = this.mapVisualProfile(payload);
    return {
      centerRadius: profile.unit(profile.massive ? 2.2 : 2.8),
      centerStrokeWidth: profile.unit(0.9),
      headingLength: profile.unit(profile.massive ? 9 : 11),
      headingStrokeWidth: profile.unit(1.2),
      footprintStrokeWidth: profile.unit(1.0),
      labelOffset: profile.unit(profile.massive ? 11 : 13),
      labelFontSize: profile.unit(profile.massive ? 7.2 : 8),
      labelStrokeWidth: profile.unit(2.0),
    };
  }

  robotFootprintPoints(pose) {
    const footprint = this.robotModelFootprint();
    const yaw = Number(pose.yaw || 0);
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    return footprint.map((point) => {
      const world = {
        x: Number(pose.x || 0) + (Number(point.x || 0) * cos) + (Number(point.y || 0) * sin),
        y: Number(pose.y || 0) + (Number(point.x || 0) * sin) - (Number(point.y || 0) * cos),
      };
      const pixel = this.worldToPixel(world);
      return `${pixel.x},${pixel.y}`;
    }).join(" ");
  }

  robotModelFootprint() {
    const selected = this.selectedRobot();
    const robotModel = (
      !this.isFleetManager(selected)
      && this.robotParamsRobotId === selected?.id
      && this.robotParams?.robot_model
    ) || (
      this.isFleetManager(selected)
      && this.fleetParamsManagerId === selected?.id
      && this.fleetParams?.robot_model
    ) || this.fleetModelEditor?.getModel() || {};
    const configured = Array.isArray(robotModel.footprint)
      ? robotModel.footprint
        .map((point) => ({
          x: Number(point?.x),
          y: Number(point?.y),
        }))
        .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
      : [];
    return configured.length >= 3
      ? configured
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
  }

  landmarkIndex() {
    return new Map((this.activeOperatorMapPayload()?.lms || []).map((lm) => [lm.name, lm]));
  }

  displayPoseForActiveMap(pose) {
    const source = pose && typeof pose === "object" ? pose : {};
    const point = this.displayPointForActiveMap(pose);
    if (!this.shouldTransformLiveSlamPoint()) {
      return {
        ...source,
        x: point.x,
        y: point.y,
        yaw: Number(source.yaw || 0),
      };
    }
    return {
      ...source,
      x: point.x,
      y: point.y,
      yaw: this.normalizeAngle(-Number(source.yaw || 0)),
    };
  }

  displayPointForActiveMap(point) {
    const source = point && typeof point === "object" ? point : {};
    if (!this.shouldTransformLiveSlamPoint()) {
      return {
        ...source,
        x: Number(source.x || 0),
        y: Number(source.y || 0),
      };
    }
    const map = this.slamMapPayload?.map || {};
    const resolution = Number(map.resolution || 0);
    const height = Number(map.height || 0);
    const rosOrigin = Array.isArray(map.rosOrigin) ? map.rosOrigin : [0, 0, 0];
    const originX = Number(rosOrigin[0] || 0);
    const originY = Number(rosOrigin[1] || 0);
    return {
      ...source,
      x: Number(source.x || 0) - originX,
      y: (height * resolution) - (Number(source.y || 0) - originY),
    };
  }

  shouldTransformLiveSlamPoint() {
    const map = this.slamMapPayload?.map || null;
    return Boolean(
      this.slamActive
      && map
      && Array.isArray(map.rosOrigin)
      && Number(map.resolution || 0) > 0
      && Number(map.height || 0) > 0
    );
  }

  worldToPixel(point) {
    const map = this.activeOperatorMapPayload()?.map || {};
    const resolution = Number(map.resolution || 1);
    const padding = Number(map.viewPadding || 0);
    return {
      x: padding + (Number(point.x || 0) / resolution),
      y: padding + (Number(point.y || 0) / resolution),
    };
  }

  pixelToWorld(point) {
    const map = this.activeOperatorMapPayload()?.map || {};
    const resolution = Number(map.resolution || 1);
    const padding = Number(map.viewPadding || 0);
    return {
      x: (point.x - padding) * resolution,
      y: (point.y - padding) * resolution,
    };
  }

  mapScreenScale(payload = this.activeOperatorMapPayload()) {
    const map = payload?.map || {};
    const viewWidth = Number(map.viewWidth || 0);
    const viewHeight = Number(map.viewHeight || 0);
    const rect = this.operatorMapSvg?.getBoundingClientRect?.();
    const baseScale = rect && rect.width > 0 && rect.height > 0 && viewWidth > 0 && viewHeight > 0
      ? Math.min(rect.width / viewWidth, rect.height / viewHeight)
      : 1;
    return Math.max(0.001, baseScale * Math.max(0.1, Number(this.mapView.scale || 1)));
  }

  screenPxToMapUnits(px, payload = this.activeOperatorMapPayload()) {
    return Number(px || 0) / this.mapScreenScale(payload);
  }

  mapVisualProfile(payload = this.activeOperatorMapPayload()) {
    const lms = Array.isArray(payload?.lms) ? payload.lms.length : 0;
    const edges = Array.isArray(payload?.edges) ? payload.edges.length : 0;
    const dense = lms >= 350 || edges >= 900;
    const massive = lms >= 900 || edges >= 2500;
    return {
      lms,
      edges,
      dense,
      massive,
      unit: (px) => this.screenPxToMapUnits(px, payload),
    };
  }

  refreshAdaptiveMapLayers() {
    if (this.mapAdaptiveLayerTimer) {
      window.clearTimeout(this.mapAdaptiveLayerTimer);
      this.mapAdaptiveLayerTimer = null;
    }
    if (!this.activeOperatorMapPayload()?.map || this.mapViewMode === "3d") {
      return;
    }
    this.drawGraph();
    this.drawRoute();
    this.drawLookahead();
    this.drawLandmarks();
    this.drawFleetEditorOverlay();
    this.drawScanOverlay();
    this.drawRobot();
  }

  scheduleAdaptiveMapLayers() {
    if (this.mapAdaptiveLayerTimer) {
      window.clearTimeout(this.mapAdaptiveLayerTimer);
    }
    this.mapAdaptiveLayerTimer = window.setTimeout(() => {
      this.mapAdaptiveLayerTimer = null;
      this.refreshAdaptiveMapLayers();
    }, 90);
  }

  directionArrow(edge, landmarks, fraction = 0.5) {
    const points = this.directionArrowPoints(edge, landmarks, fraction);
    if (!points) {
      return null;
    }
    const polygon = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
    polygon.setAttribute("class", "graph-direction");
    polygon.setAttribute(
      "points",
      `${points.tip.x},${points.tip.y} ${points.left.x},${points.left.y} ${points.right.x},${points.right.y}`,
    );
    return polygon;
  }

  directionArrowPoints(edge, landmarks, fraction = 0.5) {
    let point = null;
    let tangent = null;
    if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      const points = edge.control_points.map((item) => this.worldToPixel(item));
      point = this.bezierPoint(points, fraction);
      tangent = this.bezierTangent(points, fraction);
    } else {
      const start = landmarks.get(edge.from);
      const goal = landmarks.get(edge.to);
      if (!start || !goal) {
        return null;
      }
      const s = this.worldToPixel(start);
      const g = this.worldToPixel(goal);
      point = {
        x: s.x + ((g.x - s.x) * fraction),
        y: s.y + ((g.y - s.y) * fraction),
      };
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
    const arrowLength = this.screenPxToMapUnits(5.5);
    const arrowWidth = this.screenPxToMapUnits(3.2);
    const tip = { x: point.x + ux * arrowLength, y: point.y + uy * arrowLength };
    const base = { x: point.x - ux * arrowLength, y: point.y - uy * arrowLength };
    const left = { x: base.x + px * arrowWidth, y: base.y + py * arrowWidth };
    const right = { x: base.x - px * arrowWidth, y: base.y - py * arrowWidth };
    return { tip, left, right };
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
    if (this.relocateMode && !this.isFleetManager()) {
      this.beginRelocationDrag(event);
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
    if (this.updateRelocationDrag(event)) {
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
    if (this.finishRelocationDrag(event)) {
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
    if (this.relocateMode) {
      event.preventDefault();
      return;
    }
    if (!this.navigateMode && !this.relocateMode) {
      if (this.isFleetManager() && !event.target.closest(".landmark, .fleet-robot")) {
        this.clearFleetRobotSelection();
      }
      return;
    }
    if (event.target.closest(".landmark")) {
      return;
    }
    const mapPixel = this.screenToMapPixel(event.clientX, event.clientY);
    if (!mapPixel) {
      return;
    }
    const world = this.pixelToWorld(mapPixel);
    if (this.isRos2Robot() && !this.isFleetManager()) {
      this.startPoseNavigation(world);
      return;
    }
    if (this.fleetNavigateUsesPose()) {
      this.startFleetPoseNavigation(world);
      return;
    }
    const nearest = this.nearestLandmark(world);
    if (!nearest || nearest.distance > 1.2) {
      this.robotMessageText.textContent = `${this.fleetTargetActionLabel()} armed: click closer to a landmark.`;
      return;
    }
    this.handleLandmarkTarget(nearest.landmark.name);
  }

  handleScene3dFloorClick(world) {
    if (this.fleetMapEditorActive) {
      return;
    }
    if (this.relocateMode) {
      this.robotMessageText.textContent = "Relocate is available in 2D map mode so yaw can be dragged precisely.";
      return;
    }
    if (!this.navigateMode) {
      const nearest = this.nearestLandmark(world);
      if (!nearest || nearest.distance > 0.55) {
        this.clearFleetRobotSelection();
      }
      return;
    }
    if (this.isRos2Robot() && !this.isFleetManager()) {
      this.startPoseNavigation(world);
      return;
    }
    if (this.fleetNavigateUsesPose()) {
      this.startFleetPoseNavigation(world);
      return;
    }
    const nearest = this.nearestLandmark(world);
    if (!nearest) {
      this.robotMessageText.textContent = `${this.fleetTargetActionLabel()} armed: no landmark found on this map.`;
      return;
    }
    if (nearest.distance > 2.0) {
      this.robotMessageText.textContent = `${this.fleetTargetActionLabel()} armed: click closer to a landmark. Nearest is ${nearest.landmark.name}.`;
      return;
    }
    this.handleLandmarkTarget(nearest.landmark.name);
  }

  handleScene3dLandmarkHover(lmName) {
    const nextName = String(lmName || "");
    if (nextName === this.scene3dHoverLmName) {
      return;
    }
    this.scene3dHoverLmName = nextName;
    if (!this.navigateMode) {
      return;
    }
    const action = this.fleetTargetActionLabel();
    if (!nextName) {
      this.robotMessageText.textContent = this.pendingFleetRobotName
        ? `${action} armed for ${this.pendingFleetRobotName}: hover an LM and click to select.`
        : `${action} armed: hover an LM and click to select.`;
      return;
    }
    const robotName = this.pendingFleetRobotName ? ` for ${this.pendingFleetRobotName}` : "";
    this.robotMessageText.textContent = `${action}${robotName}: click ${nextName} to select.`;
  }

  beginRelocationDrag(event) {
    if (event.button !== 0 || !this.operatorMapPayload?.map) {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    const start = this.screenToMapPixel(event.clientX, event.clientY);
    if (!start) {
      return;
    }
    const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : {};
    const fallbackYaw = Number(robot.pose?.yaw || 0);
    const yaw = Number.isFinite(fallbackYaw) ? fallbackYaw : 0;
    const previewLength = 34 / Math.max(1, this.mapView.scale);
    this.relocationDrag = {
      pointerId: event.pointerId,
      start,
      end: {
        x: start.x + (Math.cos(yaw) * previewLength),
        y: start.y + (Math.sin(yaw) * previewLength),
      },
      hasDirection: false,
    };
    this.mapClickConsumed = true;
    this.operatorMapSvg.classList.add("relocating");
    this.operatorMapSvg.setPointerCapture(event.pointerId);
    this.drawRelocationPreview();
    this.robotMessageText.textContent = "Relocate armed: hold and drag to set yaw, release to apply.";
  }

  updateRelocationDrag(event) {
    if (!this.relocationDrag || this.relocationDrag.pointerId !== event.pointerId) {
      return false;
    }
    event.preventDefault();
    event.stopPropagation();
    const end = this.screenToMapPixel(event.clientX, event.clientY);
    if (!end) {
      return true;
    }
    const distance = Math.hypot(end.x - this.relocationDrag.start.x, end.y - this.relocationDrag.start.y) * this.mapView.scale;
    this.relocationDrag.end = end;
    this.relocationDrag.hasDirection = distance >= 8;
    this.drawRelocationPreview();
    return true;
  }

  finishRelocationDrag(event) {
    if (!this.relocationDrag || this.relocationDrag.pointerId !== event.pointerId) {
      return false;
    }
    event.preventDefault();
    event.stopPropagation();
    const canceled = event.type === "pointercancel";
    const end = this.screenToMapPixel(event.clientX, event.clientY);
    if (end) {
      this.relocationDrag.end = end;
      const distance = Math.hypot(end.x - this.relocationDrag.start.x, end.y - this.relocationDrag.start.y) * this.mapView.scale;
      this.relocationDrag.hasDirection = distance >= 8;
    }
    if (this.operatorMapSvg.hasPointerCapture(event.pointerId)) {
      this.operatorMapSvg.releasePointerCapture(event.pointerId);
    }
    this.operatorMapSvg.classList.remove("relocating");
    const pose = this.relocationPoseFromDrag(this.relocationDrag);
    this.relocationDrag = null;
    this.mapClickConsumed = true;
    window.setTimeout(() => {
      this.mapClickConsumed = false;
    }, 150);
    if (canceled) {
      this.clearRelocationPreview();
      this.robotMessageText.textContent = "Relocate still armed: hold and drag again.";
      return true;
    }
    if (!pose) {
      this.clearRelocationPreview();
      this.robotMessageText.textContent = "Relocate still armed: drag farther to set yaw.";
      return true;
    }
    this.startRelocation(pose);
    return true;
  }

  relocationPoseFromDrag(drag) {
    if (!drag || !drag.hasDirection) {
      return null;
    }
    const dx = drag.end.x - drag.start.x;
    const dy = drag.end.y - drag.start.y;
    if (Math.hypot(dx, dy) <= 0.001) {
      return null;
    }
    const world = this.pixelToWorld(drag.start);
    return {
      x: Number(world.x || 0),
      y: Number(world.y || 0),
      yaw: Math.atan2(dy, dx),
    };
  }

  clearRelocationPreview() {
    if (this.operatorRelocateLayer) {
      this.operatorRelocateLayer.innerHTML = "";
    }
    this.operatorMapSvg?.classList.remove("relocating");
  }

  drawRelocationPreview() {
    if (!this.operatorRelocateLayer) {
      return;
    }
    this.operatorRelocateLayer.innerHTML = "";
    const drag = this.relocationDrag;
    if (!drag) {
      return;
    }
    const dx = drag.end.x - drag.start.x;
    const dy = drag.end.y - drag.start.y;
    const length = Math.hypot(dx, dy);
    const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
    group.setAttribute("class", drag.hasDirection ? "relocate-preview" : "relocate-preview pending");

    const anchor = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    anchor.setAttribute("class", "relocate-anchor");
    anchor.setAttribute("cx", String(drag.start.x));
    anchor.setAttribute("cy", String(drag.start.y));
    anchor.setAttribute("r", "7");
    group.append(anchor);

    if (length > 0.001) {
      const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
      line.setAttribute("class", "relocate-heading-line");
      line.setAttribute("x1", String(drag.start.x));
      line.setAttribute("y1", String(drag.start.y));
      line.setAttribute("x2", String(drag.end.x));
      line.setAttribute("y2", String(drag.end.y));
      group.append(line);

      const ux = dx / length;
      const uy = dy / length;
      const px = -uy;
      const py = ux;
      const headLength = Math.min(16, Math.max(9, length * 0.34));
      const headWidth = Math.min(10, Math.max(6, length * 0.2));
      const base = {
        x: drag.end.x - (ux * headLength),
        y: drag.end.y - (uy * headLength),
      };
      const left = {
        x: base.x + (px * headWidth),
        y: base.y + (py * headWidth),
      };
      const right = {
        x: base.x - (px * headWidth),
        y: base.y - (py * headWidth),
      };
      const head = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
      head.setAttribute("class", "relocate-heading-head");
      head.setAttribute("points", `${drag.end.x},${drag.end.y} ${left.x},${left.y} ${right.x},${right.y}`);
      group.append(head);
    }

    this.operatorRelocateLayer.append(group);
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
};
