(function () {
  const demoData = window.WAREHOUSE_WEB_DATA;
  if (!demoData) {
    throw new Error("WAREHOUSE_WEB_DATA is missing.");
  }

  class GeometryService {
    constructor(mapData) {
      this.mapData = mapData;
    }

    worldToPixel(point) {
      const px = this.mapData.viewPadding + ((point.x - this.mapData.origin[0]) / this.mapData.resolution);
      const py = this.mapData.viewPadding + (this.mapData.height - 1) - ((point.y - this.mapData.origin[1]) / this.mapData.resolution);
      return { x: px, y: py };
    }

    pixelToWorld(point) {
      const x = ((point.x - this.mapData.viewPadding) * this.mapData.resolution) + this.mapData.origin[0];
      const y = ((this.mapData.height - 1) - (point.y - this.mapData.viewPadding)) * this.mapData.resolution + this.mapData.origin[1];
      return { x, y };
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

    normalizeAngle(angle) {
      return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    interpolateAngle(start, goal, t) {
      return this.normalizeAngle(start + (this.normalizeAngle(goal - start) * t));
    }

    cubicBezier(points, t) {
      const u = 1 - t;
      const tt = t * t;
      const uu = u * u;
      const uuu = uu * u;
      const ttt = tt * t;
      return {
        x: (uuu * points[0].x) + (3 * uu * t * points[1].x) + (3 * u * tt * points[2].x) + (ttt * points[3].x),
        y: (uuu * points[0].y) + (3 * uu * t * points[1].y) + (3 * u * tt * points[2].y) + (ttt * points[3].y),
      };
    }

    cubicBezierDerivative(points, t) {
      const u = 1 - t;
      return {
        x: (3 * u * u * (points[1].x - points[0].x)) + (6 * u * t * (points[2].x - points[1].x)) + (3 * t * t * (points[3].x - points[2].x)),
        y: (3 * u * u * (points[1].y - points[0].y)) + (6 * u * t * (points[2].y - points[1].y)) + (3 * t * t * (points[3].y - points[2].y)),
      };
    }
  }

  class GraphModel {
    constructor(landmarks, edges, routes) {
      this.landmarks = landmarks;
      this.edges = edges;
      this.routes = routes || {};
      this.nodeByName = new Map();
      this.edgeByKey = new Map();

      for (const lm of landmarks) {
        this.nodeByName.set(lm.name, lm);
      }
      for (const edge of edges) {
        this.edgeByKey.set(`${edge.from}|${edge.to}`, edge);
      }
    }

    getEdge(fromName, toName) {
      return this.edgeByKey.get(`${fromName}|${toName}`) || null;
    }

    getRoute(startName, goalName) {
      if (startName === goalName) {
        return { nodes: [startName], length: 0 };
      }
      return this.routes[`${startName}|${goalName}`] || null;
    }

    nearestLandmark(point, geometry) {
      let best = null;
      let bestDistance = Number.POSITIVE_INFINITY;
      for (const lm of this.landmarks) {
        const distance = geometry.distance(point, lm);
        if (distance < bestDistance) {
          best = lm;
          bestDistance = distance;
        }
      }
      return { landmark: best, distance: bestDistance };
    }
  }

  class MissionPlanner {
    constructor(graphModel, geometry) {
      this.graphModel = graphModel;
      this.geometry = geometry;
      this.sampleDistance = 0.05;
      this.lmTolerance = 0.05;
      this.onRouteTolerance = 0.12;
    }

    buildMission(currentPose, targetName, speed) {
      const nearest = this.graphModel.nearestLandmark(currentPose, this.geometry);
      if (!nearest.landmark) {
        return null;
      }

      if (nearest.distance > this.lmTolerance) {
        const currentRouteMission = this.buildMissionFromCurrentRoute(currentPose, targetName, speed, nearest);
        if (currentRouteMission) {
          return currentRouteMission;
        }
      }

      let path = [];
      const nodes = [];
      let connectorLength = 0;

      if (nearest.distance > this.lmTolerance) {
        const connector = this.sampleLine(
          currentPose,
          nearest.landmark,
          `CURRENT_POSE->${nearest.landmark.name}`,
          this.sampleDistance
        );
        path.push(...connector);
        nodes.push("CURRENT_POSE", nearest.landmark.name);
        connectorLength = nearest.distance;
      } else {
        nodes.push(nearest.landmark.name);
      }

      if (nearest.landmark.name !== targetName) {
        const route = this.graphModel.getRoute(nearest.landmark.name, targetName);
        if (!route) {
          return null;
        }
        let lmPath = this.buildLmTrajectory(route, speed);
        if (path.length && lmPath.length) {
          lmPath = lmPath.slice(1);
        }
        path.push(...lmPath);
        if (nodes[nodes.length - 1] === route.nodes[0]) {
          nodes.push(...route.nodes.slice(1));
        } else {
          nodes.push(...route.nodes);
        }
      }

      if (path.length === 0) {
        path = [{
          x: currentPose.x,
          y: currentPose.y,
          yaw: currentPose.yaw,
          edgeId: `${targetName}->${targetName}`,
        }];
      }

      path = this.annotateDistances(path, speed);
      return {
        targetName,
        nearestName: nearest.landmark.name,
        nearestDistance: nearest.distance,
        nodes,
        path,
        length: path.length ? Number(path[path.length - 1].s) : connectorLength,
        startMode: "NEAREST_LM",
      };
    }

    buildMissionFromCurrentRoute(currentPose, targetName, speed, nearest) {
      const candidate = this.findBestCurrentEdgeCandidate(currentPose, targetName);
      if (!candidate) {
        return null;
      }

      let path = [...candidate.remainingPath];
      const nodes = [`CURRENT_EDGE ${candidate.edgeId}`, candidate.edge.to];

      if (candidate.route && candidate.route.nodes.length > 1) {
        let lmPath = this.buildLmTrajectory(candidate.route, speed);
        if (path.length && lmPath.length) {
          lmPath = lmPath.slice(1);
        }
        path.push(...lmPath);
        nodes.push(...candidate.route.nodes.slice(1));
      }

      path = this.annotateDistances(path, speed);
      return {
        targetName,
        nearestName: nearest.landmark.name,
        nearestDistance: nearest.distance,
        nodes,
        path,
        length: path.length ? Number(path[path.length - 1].s) : 0,
        startMode: "CURRENT_ROUTE",
        currentEdgeId: candidate.edgeId,
        routeDistance: candidate.distance,
      };
    }

    findBestCurrentEdgeCandidate(currentPose, targetName) {
      let best = null;

      for (const edge of this.graphModel.edges) {
        const samples = this.sampleGraphEdge(edge);
        if (samples.length < 2) {
          continue;
        }

        const projection = this.projectOntoSamples(currentPose, samples);
        if (!projection || projection.distance > this.onRouteTolerance) {
          continue;
        }

        let route = null;
        if (edge.to !== targetName) {
          route = this.graphModel.getRoute(edge.to, targetName);
          if (!route) {
            continue;
          }
        } else {
          route = { nodes: [edge.to], length: 0 };
        }

        const remainingPath = this.remainingEdgePath(samples, projection);
        const remainingLength = this.pathLength(remainingPath);
        const totalLength = remainingLength + Number(route.length || 0);
        const candidate = {
          edge,
          edgeId: `${edge.from}->${edge.to}`,
          route,
          projection,
          remainingPath,
          distance: projection.distance,
          totalLength,
        };

        if (!best || candidate.totalLength < best.totalLength) {
          best = candidate;
        }
      }

      return best;
    }

    sampleGraphEdge(edge) {
      const edgeId = `${edge.from}->${edge.to}`;
      if (edge.geometry === "bezier" && edge.control_points && edge.control_points.length === 4) {
        return this.annotateDistances(
          this.sampleBezier(edge.control_points, edgeId, this.sampleDistance),
          0
        );
      }
      return this.annotateDistances(
        this.sampleLine(
          this.graphModel.nodeByName.get(edge.from),
          this.graphModel.nodeByName.get(edge.to),
          edgeId,
          this.sampleDistance
        ),
        0
      );
    }

    projectOntoSamples(point, samples) {
      let best = null;
      for (let i = 0; i < samples.length - 1; i += 1) {
        const start = samples[i];
        const end = samples[i + 1];
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const lengthSq = (dx * dx) + (dy * dy);
        if (lengthSq <= 0.000001) {
          continue;
        }
        const t = Math.max(0, Math.min(1, (((point.x - start.x) * dx) + ((point.y - start.y) * dy)) / lengthSq));
        const projected = {
          x: start.x + (dx * t),
          y: start.y + (dy * t),
          yaw: Math.atan2(dy, dx),
          edgeId: start.edgeId,
          s: start.s + (Math.sqrt(lengthSq) * t),
        };
        const distance = this.geometry.distance(point, projected);
        if (!best || distance < best.distance) {
          best = { ...projected, distance, segmentIndex: i };
        }
      }
      return best;
    }

    remainingEdgePath(samples, projection) {
      const remaining = [{
        x: projection.x,
        y: projection.y,
        yaw: projection.yaw,
        edgeId: projection.edgeId,
      }];
      for (let i = projection.segmentIndex + 1; i < samples.length; i += 1) {
        remaining.push({
          x: samples[i].x,
          y: samples[i].y,
          yaw: samples[i].yaw,
          edgeId: samples[i].edgeId,
        });
      }
      return remaining;
    }

    pathLength(path) {
      let length = 0;
      for (let i = 1; i < path.length; i += 1) {
        length += this.geometry.distance(path[i - 1], path[i]);
      }
      return length;
    }

    buildLmTrajectory(route, speed) {
      if (!route || route.nodes.length < 2) {
        return [];
      }

      const trajectory = [];
      for (let i = 0; i < route.nodes.length - 1; i += 1) {
        const fromName = route.nodes[i];
        const toName = route.nodes[i + 1];
        const edge = this.graphModel.getEdge(fromName, toName);
        const edgeId = `${fromName}->${toName}`;
        let samples;

        if (edge && edge.geometry === "bezier" && edge.control_points && edge.control_points.length === 4) {
          samples = this.sampleBezier(edge.control_points, edgeId, this.sampleDistance);
        } else {
          samples = this.sampleLine(
            this.graphModel.nodeByName.get(fromName),
            this.graphModel.nodeByName.get(toName),
            edgeId,
            this.sampleDistance
          );
        }

        if (trajectory.length) {
          samples = samples.slice(1);
        }
        trajectory.push(...samples);
      }

      return this.annotateDistances(trajectory, speed);
    }

    sampleLine(start, goal, edgeId, spacing) {
      const length = Math.max(spacing, this.geometry.distance(start, goal));
      const steps = Math.max(1, Math.ceil(length / spacing));
      const yaw = Math.atan2(goal.y - start.y, goal.x - start.x);
      const samples = [];

      for (let i = 0; i <= steps; i += 1) {
        const t = i / steps;
        samples.push({
          x: start.x + ((goal.x - start.x) * t),
          y: start.y + ((goal.y - start.y) * t),
          yaw,
          edgeId,
        });
      }
      return samples;
    }

    sampleBezier(points, edgeId, spacing) {
      const roughLength = points.reduce((total, point, index) => {
        if (index === 0) {
          return 0;
        }
        return total + this.geometry.distance(points[index - 1], point);
      }, 0);
      const steps = Math.max(12, Math.ceil(roughLength / spacing));
      const samples = [];

      for (let i = 0; i <= steps; i += 1) {
        const t = i / steps;
        const point = this.geometry.cubicBezier(points, t);
        const tangent = this.geometry.cubicBezierDerivative(points, t);
        samples.push({
          x: point.x,
          y: point.y,
          yaw: Math.atan2(tangent.y, tangent.x),
          edgeId,
        });
      }
      return samples;
    }

    annotateDistances(path, speed) {
      let distance = 0;
      for (let i = 0; i < path.length; i += 1) {
        if (i > 0) {
          distance += this.geometry.distance(path[i - 1], path[i]);
        }
        path[i].s = distance;
        path[i].targetSpeed = speed;
      }
      return path;
    }

    poseAtDistance(path, distance) {
      if (path.length === 0) {
        return null;
      }
      if (distance <= 0) {
        return path[0];
      }

      const last = path[path.length - 1];
      if (distance >= last.s) {
        return last;
      }

      let index = 0;
      while (index < path.length - 2 && path[index + 1].s < distance) {
        index += 1;
      }

      const start = path[index];
      const goal = path[index + 1];
      const span = Math.max(0.0001, goal.s - start.s);
      const t = (distance - start.s) / span;
      return {
        x: start.x + ((goal.x - start.x) * t),
        y: start.y + ((goal.y - start.y) * t),
        yaw: this.geometry.interpolateAngle(start.yaw, goal.yaw, t),
        s: distance,
        edgeId: start.edgeId,
      };
    }
  }

  class Renderer {
    constructor(dom, graphModel, geometry, onLmClick) {
      this.dom = dom;
      this.graphModel = graphModel;
      this.geometry = geometry;
      this.onLmClick = onLmClick;
      this.robotModel = {
        footprint: [
          { x: 0.35, y: 0.275 },
          { x: 0.35, y: -0.275 },
          { x: -0.35, y: -0.275 },
          { x: -0.35, y: 0.275 },
        ],
        frames: {
          lidar: { x: 0.28, y: 0.00, color: "#1f6feb", label: "LiDAR" },
          imu: { x: 0.00, y: 0.00, color: "#d95521", label: "IMU" },
          wheel_left: { x: 0.00, y: 0.225, color: "#2f3a4a", label: "WL" },
          wheel_right: { x: 0.00, y: -0.225, color: "#2f3a4a", label: "WR" },
        },
      };
    }

    setRobotModel(model) {
      this.robotModel = {
        footprint: model.footprint.map((point) => ({ x: point.x, y: point.y })),
        frames: Object.fromEntries(
          Object.entries(model.frames).map(([name, frame]) => [
            name,
            { ...frame },
          ])
        ),
      };
    }

    initMap() {
      const map = demoData.map;
      this.dom.mapTitle.textContent = demoData.mapName;
      document.title = `${demoData.mapName} Route Simulator`;
      this.dom.mapSvg.setAttribute("viewBox", `0 0 ${map.viewWidth} ${map.viewHeight}`);
      this.dom.mapImage.setAttribute("x", String(map.viewPadding));
      this.dom.mapImage.setAttribute("y", String(map.viewPadding));
      this.dom.mapImage.setAttribute("width", String(map.width));
      this.dom.mapImage.setAttribute("height", String(map.height));
      this.dom.mapImage.setAttribute("href", map.imageDataUrl);
    }

    createSvgElement(tag, attrs) {
      const element = document.createElementNS("http://www.w3.org/2000/svg", tag);
      for (const [key, value] of Object.entries(attrs)) {
        element.setAttribute(key, String(value));
      }
      return element;
    }

    drawGraph() {
      this.dom.graphLayer.innerHTML = "";
      const seen = new Set();

      for (const edge of this.graphModel.edges) {
        const key = [edge.from, edge.to].sort().join("|");
        if (seen.has(key)) {
          continue;
        }
        seen.add(key);

        if (edge.geometry === "bezier" && edge.control_points && edge.control_points.length === 4) {
          const cp = edge.control_points.map((point) => this.geometry.worldToPixel(point));
          this.dom.graphLayer.appendChild(
            this.createSvgElement("path", {
              d: `M ${cp[0].x} ${cp[0].y} C ${cp[1].x} ${cp[1].y}, ${cp[2].x} ${cp[2].y}, ${cp[3].x} ${cp[3].y}`,
              fill: "none",
              stroke: "var(--edge)",
              "stroke-width": 2,
              "stroke-linecap": "round",
            })
          );
          continue;
        }

        const start = this.geometry.worldToPixel(this.graphModel.nodeByName.get(edge.from));
        const goal = this.geometry.worldToPixel(this.graphModel.nodeByName.get(edge.to));
        this.dom.graphLayer.appendChild(
          this.createSvgElement("line", {
            x1: start.x,
            y1: start.y,
            x2: goal.x,
            y2: goal.y,
            stroke: "var(--edge)",
            "stroke-width": 2,
            "stroke-linecap": "round",
          })
        );
      }
    }

    drawLandmarks(nearestName, targetName, navigateMode) {
      this.dom.pointLayer.innerHTML = "";
      for (const lm of this.graphModel.landmarks) {
        const pos = this.geometry.worldToPixel(lm);
        let fill = "#9b2c2c";
        let radius = 4;

        if (lm.name === nearestName) {
          fill = "var(--nearest)";
          radius = 6;
        }
        if (lm.name === targetName) {
          fill = "var(--goal)";
          radius = 7;
        }

        const group = this.createSvgElement("g", {
          class: navigateMode ? "lm-hit armed" : "lm-hit",
          "data-lm": lm.name,
          opacity: navigateMode ? 1 : 0.88,
        });
        group.appendChild(
          this.createSvgElement("circle", {
            cx: pos.x,
            cy: pos.y,
            r: radius,
            fill,
          })
        );
        const label = this.createSvgElement("text", {
          x: pos.x,
          y: pos.y + radius + 12,
          class: "lm-label",
        });
        label.textContent = lm.name;
        group.appendChild(label);
        group.addEventListener("click", (event) => {
          event.stopPropagation();
          this.onLmClick(lm.name);
        });
        this.dom.pointLayer.appendChild(group);
      }
    }

    drawRoute(path, progressDistance = 0) {
      this.dom.routeLayer.innerHTML = "";
      if (!path || path.length < 2) {
        return;
      }

      const planPoints = this.routePoints(path);
      this.dom.routeLayer.appendChild(
        this.createSvgElement("polyline", {
          points: planPoints,
          fill: "none",
          stroke: "var(--route-plan)",
          "stroke-width": 3,
          "stroke-linecap": "round",
          "stroke-linejoin": "round",
          "stroke-dasharray": "10 8",
          opacity: 0.95,
        })
      );

      const passedPath = this.sliceRoute(path, 0, progressDistance);
      if (passedPath.length > 1) {
        this.dom.routeLayer.appendChild(
          this.createSvgElement("polyline", {
            points: this.routePoints(passedPath),
            fill: "none",
            stroke: "var(--route-done)",
            "stroke-width": 5,
            "stroke-linecap": "round",
            "stroke-linejoin": "round",
            opacity: 0.88,
          })
        );
      }

      const remainingPath = this.sliceRoute(path, progressDistance, path[path.length - 1].s);
      if (remainingPath.length > 1) {
        this.dom.routeLayer.appendChild(
          this.createSvgElement("polyline", {
            points: this.routePoints(remainingPath),
            fill: "none",
            stroke: "var(--route-active)",
            "stroke-width": 5,
            "stroke-linecap": "round",
            "stroke-linejoin": "round",
            opacity: 0.82,
          })
        );
      }
    }

    routePoints(path) {
      return path
        .map((pose) => this.geometry.worldToPixel(pose))
        .map((point) => `${point.x},${point.y}`)
        .join(" ");
    }

    sliceRoute(path, startDistance, endDistance) {
      if (!path || path.length < 2 || endDistance <= startDistance) {
        return [];
      }

      const result = [];
      const startPose = this.interpolatePath(path, startDistance);
      if (startPose) {
        result.push(startPose);
      }

      for (const pose of path) {
        if (pose.s > startDistance && pose.s < endDistance) {
          result.push(pose);
        }
      }

      const endPose = this.interpolatePath(path, endDistance);
      if (endPose) {
        result.push(endPose);
      }
      return result;
    }

    interpolatePath(path, distance) {
      if (!path.length) {
        return null;
      }
      if (distance <= 0) {
        return path[0];
      }

      const last = path[path.length - 1];
      if (distance >= last.s) {
        return last;
      }

      let index = 0;
      while (index < path.length - 2 && path[index + 1].s < distance) {
        index += 1;
      }

      const start = path[index];
      const goal = path[index + 1];
      const span = Math.max(0.0001, goal.s - start.s);
      const t = (distance - start.s) / span;
      return {
        x: start.x + ((goal.x - start.x) * t),
        y: start.y + ((goal.y - start.y) * t),
        yaw: this.geometry.interpolateAngle(start.yaw, goal.yaw, t),
        s: distance,
        edgeId: start.edgeId,
      };
    }

    robotConfig() {
      return {
        footprint: this.robotModel.footprint,
        frames: this.robotModel.frames,
        lookahead: Math.max(0.1, Number(this.dom.lookaheadInput.value) || 0.8),
        collisionMargin: Math.max(0, Number(this.dom.collisionMarginInput.value) || 0),
        stopDistance: Math.max(0.05, Number(this.dom.stopDistanceInput.value) || 0.4),
      };
    }

    footprintCorners(pose) {
      const cfg = this.robotConfig();
      return cfg.footprint.map((point) => this.localToWorld(pose, point));
    }

    localToWorld(pose, point) {
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);
      return {
        x: pose.x + (point.x * cos) - (point.y * sin),
        y: pose.y + (point.x * sin) + (point.y * cos),
      };
    }

    drawFootprint(pose, attrs = {}) {
      const points = this.footprintCorners(pose)
        .map((point) => this.geometry.worldToPixel(point))
        .map((point) => `${point.x},${point.y}`)
        .join(" ");
      return this.createSvgElement("polygon", {
        points,
        fill: attrs.fill || "var(--robot-fill)",
        stroke: attrs.stroke || "var(--robot)",
        "stroke-width": attrs.strokeWidth || 2.5,
        "stroke-linejoin": "round",
        opacity: attrs.opacity || 1,
      });
    }

    drawRobotPose(pose, blocked = false) {
      this.dom.robotLayer.innerHTML = "";
      if (!pose) {
        return;
      }
      this.dom.robotLayer.appendChild(
        this.drawFootprint(pose, {
          fill: blocked ? "rgba(201, 42, 42, 0.25)" : "var(--robot-fill)",
          stroke: blocked ? "var(--blocked)" : "var(--robot)",
        })
      );

      const center = this.geometry.worldToPixel(pose);
      const cfg = this.robotConfig();
      const frontX = Math.max(...cfg.footprint.map((point) => point.x), 0.1);
      const nose = this.geometry.worldToPixel({
        x: pose.x + Math.cos(pose.yaw) * frontX,
        y: pose.y + Math.sin(pose.yaw) * frontX,
      });
      this.dom.robotLayer.appendChild(
        this.createSvgElement("line", {
          x1: center.x,
          y1: center.y,
          x2: nose.x,
          y2: nose.y,
          stroke: blocked ? "var(--blocked)" : "var(--robot)",
          "stroke-width": 3,
          "stroke-linecap": "round",
        })
      );
      this.dom.robotLayer.appendChild(
        this.createSvgElement("circle", {
          cx: center.x,
          cy: center.y,
          r: 3.5,
          fill: blocked ? "var(--blocked)" : "var(--robot)",
        })
      );

      for (const frame of Object.values(cfg.frames)) {
        const pos = this.geometry.worldToPixel(this.localToWorld(pose, frame));
        this.dom.robotLayer.appendChild(
          this.createSvgElement("circle", {
            cx: pos.x,
            cy: pos.y,
            r: 4.5,
            fill: blocked ? "var(--blocked)" : frame.color,
            stroke: "#ffffff",
            "stroke-width": 1.5,
          })
        );
      }
    }

    drawObstacles(obstacles, obstacleAreas = [], previewArea = null) {
      this.dom.obstacleLayer.innerHTML = "";
      for (const area of obstacleAreas) {
        this.dom.obstacleLayer.appendChild(this.drawObstacleArea(area, false));
      }
      if (previewArea) {
        this.dom.obstacleLayer.appendChild(this.drawObstacleArea(previewArea, true));
      }
      for (const obstacle of obstacles) {
        const pos = this.geometry.worldToPixel(obstacle);
        this.dom.obstacleLayer.appendChild(
          this.createSvgElement("circle", {
            cx: pos.x,
            cy: pos.y,
            r: Math.max(4, obstacle.radius / demoData.map.resolution),
            fill: "var(--obstacle)",
            stroke: "#ffffff",
            "stroke-width": 2,
            opacity: 0.88,
          })
        );
      }
    }

    drawObstacleArea(area, preview) {
      const normalized = this.normalizeArea(area);
      const corners = [
        { x: normalized.x1, y: normalized.y1 },
        { x: normalized.x2, y: normalized.y1 },
        { x: normalized.x2, y: normalized.y2 },
        { x: normalized.x1, y: normalized.y2 },
      ].map((point) => this.geometry.worldToPixel(point));
      return this.createSvgElement("polygon", {
        points: corners.map((point) => `${point.x},${point.y}`).join(" "),
        fill: preview ? "rgba(111, 66, 193, 0.18)" : "rgba(111, 66, 193, 0.28)",
        stroke: "var(--obstacle)",
        "stroke-width": preview ? 2 : 2.5,
        "stroke-dasharray": preview ? "8 6" : "none",
        opacity: preview ? 0.78 : 0.92,
      });
    }

    normalizeArea(area) {
      return {
        x1: Math.min(area.x1, area.x2),
        x2: Math.max(area.x1, area.x2),
        y1: Math.min(area.y1, area.y2),
        y2: Math.max(area.y1, area.y2),
      };
    }

    drawLookahead(poses, blocked) {
      this.dom.lookaheadLayer.innerHTML = "";
      if (!poses.length) {
        return;
      }
      const step = Math.max(1, Math.floor(poses.length / 8));
      for (let i = 0; i < poses.length; i += step) {
        this.dom.lookaheadLayer.appendChild(
          this.drawFootprint(poses[i], {
            fill: blocked ? "var(--lookahead)" : "rgba(11, 114, 133, 0.08)",
            stroke: blocked ? "var(--blocked)" : "rgba(11, 114, 133, 0.22)",
            strokeWidth: 1,
            opacity: 0.8,
          })
        );
      }
    }

    clearLookahead() {
      this.dom.lookaheadLayer.innerHTML = "";
    }

    updateRouteList(nodes) {
      this.dom.routeList.innerHTML = "";
      for (const name of nodes || []) {
        const item = document.createElement("li");
        item.textContent = name;
        this.dom.routeList.appendChild(item);
      }
    }
  }

  class MapOccupancyChecker {
    constructor(mapData, geometry) {
      this.mapData = mapData;
      this.geometry = geometry;
      this.ready = false;
      this.imageData = null;
      this.readyPromise = this.load();
    }

    load() {
      return new Promise((resolve) => {
        if (!this.mapData.imageDataUrl) {
          resolve(false);
          return;
        }
        const image = new Image();
        image.onload = () => {
          const canvas = document.createElement("canvas");
          canvas.width = this.mapData.width;
          canvas.height = this.mapData.height;
          const context = canvas.getContext("2d", { willReadFrequently: true });
          if (!context) {
            resolve(false);
            return;
          }
          context.drawImage(image, 0, 0, this.mapData.width, this.mapData.height);
          this.imageData = context.getImageData(0, 0, this.mapData.width, this.mapData.height).data;
          this.ready = true;
          resolve(true);
        };
        image.onerror = () => resolve(false);
        image.src = this.mapData.imageDataUrl;
      });
    }

    worldToImage(point) {
      const pixel = this.geometry.worldToPixel(point);
      return {
        x: Math.round(pixel.x - this.mapData.viewPadding),
        y: Math.round(pixel.y - this.mapData.viewPadding),
      };
    }

    isOccupied(point) {
      if (!this.ready || !this.imageData) {
        return false;
      }
      const pixel = this.worldToImage(point);
      if (
        pixel.x < 0 ||
        pixel.y < 0 ||
        pixel.x >= this.mapData.width ||
        pixel.y >= this.mapData.height
      ) {
        return true;
      }
      const index = ((pixel.y * this.mapData.width) + pixel.x) * 4;
      const red = this.imageData[index];
      const green = this.imageData[index + 1];
      const blue = this.imageData[index + 2];
      const alpha = this.imageData[index + 3];
      if (alpha < 10) {
        return false;
      }
      return ((red + green + blue) / 3) < 82;
    }
  }

  class CollisionService {
    constructor(renderer, occupancyChecker) {
      this.renderer = renderer;
      this.occupancyChecker = occupancyChecker;
    }

    checkPoses(poses, obstacles = [], obstacleAreas = []) {
      let count = 0;
      for (const pose of poses) {
        if (this.mapHitsPose(pose)) {
          count += 1;
        }
        for (const obstacle of obstacles) {
          if (this.obstacleHitsPose(obstacle, pose)) {
            count += 1;
          }
        }
        for (const area of obstacleAreas) {
          if (this.areaHitsPose(area, pose)) {
            count += 1;
          }
        }
        if (count > 0) {
          return { blocked: true, count };
        }
      }
      return { blocked: false, count: 0 };
    }

    mapHitsPose(pose) {
      if (!this.occupancyChecker || !this.occupancyChecker.ready) {
        return false;
      }
      const points = this.poseSamplePoints(pose);
      return points.some((point) => this.occupancyChecker.isOccupied(point));
    }

    obstacleHitsPose(obstacle, pose) {
      const cfg = this.renderer.robotConfig();
      const radius = (obstacle.radius || 0.08) + cfg.collisionMargin;
      const localPoint = this.worldToLocal(pose, obstacle);
      return (
        this.pointInPolygon(localPoint, cfg.footprint) ||
        this.distanceToPolygon(localPoint, cfg.footprint) <= radius
      );
    }

    areaHitsPose(area, pose) {
      const normalized = this.normalizeArea(area);
      return this.poseSamplePoints(pose).some((point) => (
        point.x >= normalized.x1 &&
        point.x <= normalized.x2 &&
        point.y >= normalized.y1 &&
        point.y <= normalized.y2
      ));
    }

    poseSamplePoints(pose) {
      const cfg = this.renderer.robotConfig();
      const margin = Math.max(0, cfg.collisionMargin);
      const footprint = cfg.footprint;
      const minX = Math.min(...footprint.map((point) => point.x)) - margin;
      const maxX = Math.max(...footprint.map((point) => point.x)) + margin;
      const minY = Math.min(...footprint.map((point) => point.y)) - margin;
      const maxY = Math.max(...footprint.map((point) => point.y)) + margin;
      const step = Math.max(0.04, demoData.map.resolution * 2);
      const points = [];

      for (let x = minX; x <= maxX + 0.000001; x += step) {
        for (let y = minY; y <= maxY + 0.000001; y += step) {
          const localPoint = { x, y };
          if (
            this.pointInPolygon(localPoint, footprint) ||
            this.distanceToPolygon(localPoint, footprint) <= margin
          ) {
            points.push(this.renderer.localToWorld(pose, localPoint));
          }
        }
      }

      points.push(this.renderer.localToWorld(pose, { x: 0, y: 0 }));
      for (const point of footprint) {
        points.push(this.renderer.localToWorld(pose, point));
      }
      return points;
    }

    worldToLocal(pose, point) {
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);
      const dx = point.x - pose.x;
      const dy = point.y - pose.y;
      return {
        x: (dx * cos) + (dy * sin),
        y: (-dx * sin) + (dy * cos),
      };
    }

    normalizeArea(area) {
      return {
        x1: Math.min(area.x1, area.x2),
        x2: Math.max(area.x1, area.x2),
        y1: Math.min(area.y1, area.y2),
        y2: Math.max(area.y1, area.y2),
      };
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

    distanceToPolygon(point, polygon) {
      let best = Number.POSITIVE_INFINITY;
      for (let i = 0; i < polygon.length; i += 1) {
        const start = polygon[i];
        const end = polygon[(i + 1) % polygon.length];
        best = Math.min(best, this.distanceToSegment(point, start, end));
      }
      return best;
    }

    distanceToSegment(point, start, end) {
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const lengthSq = (dx * dx) + (dy * dy);
      if (lengthSq <= 0.000001) {
        return Math.hypot(point.x - start.x, point.y - start.y);
      }
      const t = Math.max(0, Math.min(1, (((point.x - start.x) * dx) + ((point.y - start.y) * dy)) / lengthSq));
      const closest = {
        x: start.x + (dx * t),
        y: start.y + (dy * t),
      };
      return Math.hypot(point.x - closest.x, point.y - closest.y);
    }
  }

  class RobotModelEditor {
    constructor(dom, onChange) {
      this.dom = dom;
      this.onChange = onChange;
      this.center = { x: 260, y: 230 };
      this.scale = 330;
      this.view = { zoom: 1, panX: 0, panY: 0 };
      this.drag = null;
      this.panDrag = null;
      this.model = this.defaultModel();
      this.snapTolerance = 0.025;
      this.frameOrder = [
        ["lidar", "LiDAR"],
        ["imu", "IMU"],
        ["wheel_left", "Wheel L"],
        ["wheel_right", "Wheel R"],
      ];
    }

    defaultModel() {
      return {
        footprint: [
          { x: 0.35, y: 0.275 },
          { x: 0.35, y: -0.275 },
          { x: -0.35, y: -0.275 },
          { x: -0.35, y: 0.275 },
        ],
        frames: {
          lidar: { x: 0.28, y: 0.00, color: "#1f6feb", label: "LiDAR" },
          imu: { x: 0.00, y: 0.00, color: "#d95521", label: "IMU" },
          wheel_left: { x: 0.00, y: 0.225, color: "#2f3a4a", label: "WL" },
          wheel_right: { x: 0.00, y: -0.225, color: "#2f3a4a", label: "WR" },
        },
      };
    }

    init() {
      this.attachPointerEvents();
      this.dom.resetModelButton.addEventListener("click", () => this.reset());
      this.dom.robotEditorZoomInButton.addEventListener("click", () => this.zoom(1.18));
      this.dom.robotEditorZoomOutButton.addEventListener("click", () => this.zoom(0.85));
      this.dom.robotEditorResetViewButton.addEventListener("click", () => this.resetView());
      this.render();
      this.emitChange();
    }

    setModel(model) {
      if (!model || !Array.isArray(model.footprint) || !model.frames) {
        return;
      }
      const defaults = this.defaultModel();
      this.model = {
        footprint: model.footprint.map((point) => ({
          x: this.round(Number.isFinite(Number(point.x)) ? Number(point.x) : 0),
          y: this.round(Number.isFinite(Number(point.y)) ? Number(point.y) : 0),
        })),
        frames: { ...defaults.frames },
      };

      for (const [name, frame] of Object.entries(model.frames)) {
        const defaultFrame = defaults.frames[name] || { x: 0, y: 0, label: name, color: "#2f3a4a" };
        this.model.frames[name] = {
          ...defaultFrame,
          ...frame,
          x: this.round(Number.isFinite(Number(frame.x)) ? Number(frame.x) : defaultFrame.x),
          y: this.round(Number.isFinite(Number(frame.y)) ? Number(frame.y) : defaultFrame.y),
        };
      }
      this.constrainAllFrames();
    }

    getModel() {
      return {
        footprint: this.model.footprint.map((point) => ({ ...point })),
        frames: Object.fromEntries(
          Object.entries(this.model.frames).map(([name, frame]) => [
            name,
            { ...frame },
          ])
        ),
      };
    }

    reset() {
      this.model = this.defaultModel();
      this.render();
      this.emitChange();
    }

    emitChange() {
      this.onChange(this.getModel());
    }

    createSvgElement(tag, attrs) {
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
      const svg = this.dom.robotEditorSvg;
      const ctm = svg.getScreenCTM();
      if (!ctm) {
        return null;
      }
      const svgPoint = svg.createSVGPoint();
      svgPoint.x = event.clientX;
      svgPoint.y = event.clientY;
      return svgPoint.matrixTransform(ctm.inverse());
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

    clamp(value, min, max) {
      return Math.min(max, Math.max(min, value));
    }

    round(value) {
      return Math.round(value * 1000) / 1000;
    }

    zoom(multiplier) {
      this.view.zoom = this.clamp(this.view.zoom * multiplier, 0.45, 4);
      this.renderSvg();
    }

    resetView() {
      this.view = { zoom: 1, panX: 0, panY: 0 };
      this.renderSvg();
    }

    attachPointerEvents() {
      const svg = this.dom.robotEditorSvg;
      svg.addEventListener("pointerdown", (event) => {
        if (event.button !== 0) {
          return;
        }
        const target = event.target.closest("[data-drag-kind]");
        if (!target) {
          this.panDrag = {
            x: event.clientX,
            y: event.clientY,
          };
          svg.setPointerCapture(event.pointerId);
          return;
        }
        this.drag = {
          kind: target.dataset.dragKind,
          index: target.dataset.index ? Number(target.dataset.index) : null,
          frame: target.dataset.frame || null,
        };
        svg.setPointerCapture(event.pointerId);
        this.applyDrag(event);
      });

      svg.addEventListener("pointermove", (event) => {
        if (this.drag) {
          this.applyDrag(event);
          return;
        }
        if (this.panDrag) {
          const previous = this.eventToSvg({
            clientX: this.panDrag.x,
            clientY: this.panDrag.y,
          });
          const current = this.eventToSvg(event);
          if (previous && current) {
            this.view.panX += current.x - previous.x;
            this.view.panY += current.y - previous.y;
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
        this.zoom(event.deltaY < 0 ? 1.12 : 0.90);
      }, { passive: false });
    }

    applyDrag(event) {
      const point = this.eventToLocal(event);
      if (!point || !this.drag) {
        return;
      }

      if (this.drag.kind === "footprint") {
        const snapped = this.snapFootprintPoint(point, this.drag.index);
        this.model.footprint[this.drag.index] = {
          x: this.round(snapped.x),
          y: this.round(snapped.y),
        };
        this.constrainAllFrames();
      }
      if (this.drag.kind === "frame") {
        const snapped = this.snapFramePoint(point, this.drag.frame);
        const constrained = this.keepInsideFootprint(snapped);
        this.model.frames[this.drag.frame].x = this.round(constrained.x);
        this.model.frames[this.drag.frame].y = this.round(constrained.y);
      }
      this.render();
      this.emitChange();
    }

    snapFootprintPoint(point, index) {
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
        const vertex = this.model.footprint[i];
        if (Math.abs(snapped.x - vertex.x) <= this.snapTolerance) {
          snapped.x = vertex.x;
        }
        if (Math.abs(snapped.y - vertex.y) <= this.snapTolerance) {
          snapped.y = vertex.y;
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

    snapFramePoint(point, frameName) {
      const snapped = { ...point };
      if (Math.abs(snapped.x) <= this.snapTolerance) {
        snapped.x = 0;
      }
      if (Math.abs(snapped.y) <= this.snapTolerance) {
        snapped.y = 0;
      }

      for (const [name, frame] of Object.entries(this.model.frames)) {
        if (name === frameName) {
          continue;
        }
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
        const constrained = this.keepInsideFootprint(frame);
        frame.x = this.round(constrained.x);
        frame.y = this.round(constrained.y);
      }
    }

    footprintCentroid() {
      const total = this.model.footprint.reduce(
        (acc, point) => ({ x: acc.x + point.x, y: acc.y + point.y }),
        { x: 0, y: 0 }
      );
      return {
        x: total.x / this.model.footprint.length,
        y: total.y / this.model.footprint.length,
      };
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
      return {
        x: start.x + (dx * t),
        y: start.y + (dy * t),
      };
    }

    render() {
      this.renderSvg();
      this.renderFields();
    }

    renderSvg() {
      const svg = this.dom.robotEditorSvg;
      svg.innerHTML = "";
      const bounds = { left: 18, right: 502, top: 18, bottom: 442 };

      svg.appendChild(this.createSvgElement("rect", {
        x: 0,
        y: 0,
        width: 520,
        height: 460,
        class: "model-pan-surface",
      }));

      for (let value = -1.2; value <= 1.2001; value += 0.1) {
        const rounded = Math.round(value * 10) / 10;
        const vertical = this.toSvg({ x: value, y: 0 });
        const horizontal = this.toSvg({ x: 0, y: value });
        const major = Math.abs((rounded * 10) % 2) < 0.0001;
        if (vertical.x >= bounds.left && vertical.x <= bounds.right) {
          svg.appendChild(this.createSvgElement("line", {
            x1: vertical.x,
            y1: bounds.top,
            x2: vertical.x,
            y2: bounds.bottom,
            class: major ? "model-grid-line model-grid-major" : "model-grid-line",
          }));
        }
        if (horizontal.y >= bounds.top && horizontal.y <= bounds.bottom) {
          svg.appendChild(this.createSvgElement("line", {
            x1: bounds.left,
            y1: horizontal.y,
            x2: bounds.right,
            y2: horizontal.y,
            class: major ? "model-grid-line model-grid-major" : "model-grid-line",
          }));
        }
      }

      const origin = this.toSvg({ x: 0, y: 0 });
      svg.appendChild(this.createSvgElement("line", {
        x1: bounds.left,
        y1: origin.y,
        x2: bounds.right,
        y2: origin.y,
        class: "model-axis",
      }));
      svg.appendChild(this.createSvgElement("line", {
        x1: origin.x,
        y1: bounds.top,
        x2: origin.x,
        y2: bounds.bottom,
        class: "model-axis",
      }));

      for (let value = -1.0; value <= 1.0001; value += 0.2) {
        const rounded = Math.round(value * 10) / 10;
        if (Math.abs(rounded) < 0.0001) {
          continue;
        }
        const xPos = this.toSvg({ x: rounded, y: 0 });
        if (xPos.x >= bounds.left + 12 && xPos.x <= bounds.right - 12 && origin.y >= bounds.top && origin.y <= bounds.bottom) {
          const label = this.createSvgElement("text", {
            x: xPos.x,
            y: origin.y + 18,
            class: "model-axis-number",
          });
          label.textContent = rounded.toFixed(1);
          svg.appendChild(label);
        }
        const yPos = this.toSvg({ x: 0, y: rounded });
        if (yPos.y >= bounds.top + 12 && yPos.y <= bounds.bottom - 12 && origin.x >= bounds.left && origin.x <= bounds.right) {
          const label = this.createSvgElement("text", {
            x: origin.x - 22,
            y: yPos.y + 4,
            class: "model-axis-number",
          });
          label.textContent = rounded.toFixed(1);
          svg.appendChild(label);
        }
      }

      const xLabel = this.toSvg({ x: 0.72, y: 0 });
      const yLabel = this.toSvg({ x: 0, y: 0.56 });
      svg.appendChild(this.createSvgElement("text", {
        x: Math.min(bounds.right - 16, Math.max(bounds.left + 16, xLabel.x)),
        y: Math.min(bounds.bottom - 8, Math.max(bounds.top + 18, origin.y - 7)),
        class: "model-meter-label",
      })).textContent = "+x";
      svg.appendChild(this.createSvgElement("text", {
        x: Math.min(bounds.right - 16, Math.max(bounds.left + 16, origin.x + 18)),
        y: Math.min(bounds.bottom - 8, Math.max(bounds.top + 18, yLabel.y)),
        class: "model-meter-label",
      })).textContent = "+y";

      const polygonPoints = this.model.footprint
        .map((point) => this.toSvg(point))
        .map((point) => `${point.x},${point.y}`)
        .join(" ");
      svg.appendChild(this.createSvgElement("polygon", {
        points: polygonPoints,
        class: "model-footprint",
      }));

      this.model.footprint.forEach((point, index) => {
        const pos = this.toSvg(point);
        svg.appendChild(this.createSvgElement("circle", {
          cx: pos.x,
          cy: pos.y,
          r: 7,
          class: "model-handle",
          "data-drag-kind": "footprint",
          "data-index": index,
        }));
        const label = this.createSvgElement("text", {
          x: pos.x,
          y: pos.y - 10,
          class: "model-label",
        });
        label.textContent = `V${index + 1}`;
        svg.appendChild(label);
      });

      svg.appendChild(this.createSvgElement("circle", {
        cx: origin.x,
        cy: origin.y,
        r: 4,
        fill: "#111827",
      }));

      for (const [name] of this.frameOrder) {
        const frame = this.model.frames[name];
        const pos = this.toSvg(frame);
        svg.appendChild(this.createSvgElement("circle", {
          cx: pos.x,
          cy: pos.y,
          r: 8,
          fill: frame.color,
          class: "model-marker",
          "data-drag-kind": "frame",
          "data-frame": name,
        }));
        const label = this.createSvgElement("text", {
          x: pos.x,
          y: pos.y + 22,
          class: "model-label",
        });
        label.textContent = frame.label;
        svg.appendChild(label);
      }
    }

    renderFields() {
      this.dom.footprintFields.innerHTML = "";
      this.model.footprint.forEach((point, index) => {
        this.dom.footprintFields.appendChild(
          this.createPointRow(`V${index + 1}`, point, (axis, value) => {
            this.model.footprint[index][axis] = value;
            const snapped = this.snapFootprintPoint(this.model.footprint[index], index);
            this.model.footprint[index].x = this.round(snapped.x);
            this.model.footprint[index].y = this.round(snapped.y);
            this.constrainAllFrames();
          }, "compact")
        );
      });

      this.dom.tfFields.innerHTML = "";
      for (const [name, label] of this.frameOrder) {
        this.dom.tfFields.appendChild(
          this.createPointRow(label, this.model.frames[name], (axis, value) => {
            this.model.frames[name][axis] = value;
            const constrained = this.keepInsideFootprint(this.snapFramePoint(this.model.frames[name], name));
            this.model.frames[name].x = this.round(constrained.x);
            this.model.frames[name].y = this.round(constrained.y);
          })
        );
      }
    }

    createPointRow(name, point, setter, className = "") {
      const row = document.createElement("div");
      row.className = `field-row ${className}`.trim();

      const title = document.createElement("div");
      title.className = "field-name";
      title.textContent = name;
      row.appendChild(title);

      for (const axis of ["x", "y"]) {
        const label = document.createElement("label");
        label.textContent = axis;
        const input = document.createElement("input");
        input.type = "number";
        input.step = "0.001";
        input.value = point[axis].toFixed(3);
        input.addEventListener("change", () => {
          const value = Number(input.value);
          if (!Number.isFinite(value)) {
            input.value = point[axis].toFixed(3);
            return;
          }
          setter(axis, this.round(this.clamp(value, -1.5, 1.5)));
          this.render();
          this.emitChange();
        });
        label.appendChild(input);
        row.appendChild(label);
      }

      return row;
    }
  }

  class BehaviorSimulator {
    constructor(renderer, missionPlanner, collisionService, callbacks) {
      this.renderer = renderer;
      this.missionPlanner = missionPlanner;
      this.collisionService = collisionService;
      this.callbacks = callbacks;
      this.animationFrame = null;
      this.state = null;
    }

    stop(clearLookahead = true) {
      if (this.animationFrame !== null) {
        cancelAnimationFrame(this.animationFrame);
        this.animationFrame = null;
      }
      this.state = null;
      if (clearLookahead) {
        this.renderer.clearLookahead();
      }
    }

    start(mission, speed, obstacles, obstacleAreas) {
      this.stop();
      if (!mission.path || mission.path.length < 2) {
        this.callbacks.onArrive(mission.path[0] || null);
        return;
      }

      this.state = {
        mission,
        speed,
        obstacles,
        obstacleAreas,
        s: 0,
        index: 0,
        lastTs: null,
      };
      this.animationFrame = requestAnimationFrame((ts) => this.step(ts));
    }

    step(ts) {
      if (!this.state) {
        return;
      }

      const path = this.state.mission.path;
      const finalDistance = path[path.length - 1].s;
      if (this.state.s >= finalDistance) {
        const pose = path[path.length - 1];
        this.renderer.drawRoute(path, finalDistance);
        this.renderer.drawRobotPose(pose);
        this.callbacks.onPose(pose);
        this.callbacks.onArrive(pose);
        this.stop();
        return;
      }

      const collision = this.collisionAhead(path, this.state.index, this.state.obstacles, this.state.obstacleAreas);
      if (collision.blocked) {
        const pose = this.missionPlanner.poseAtDistance(path, this.state.s);
        this.renderer.drawRoute(path, this.state.s);
        this.renderer.drawRobotPose(pose, true);
        this.callbacks.onPose(pose);
        this.callbacks.onBlocked(collision.count);
        this.stop(false);
        return;
      }

      if (this.state.lastTs === null) {
        this.state.lastTs = ts;
      }
      const dt = Math.min(0.08, Math.max(0, (ts - this.state.lastTs) / 1000));
      this.state.s = Math.min(finalDistance, this.state.s + (this.state.speed * dt));

      while (
        this.state.index < path.length - 2 &&
        path[this.state.index + 1].s < this.state.s
      ) {
        this.state.index += 1;
      }

      this.state.lastTs = ts;
      const pose = this.missionPlanner.poseAtDistance(path, this.state.s);
      this.renderer.drawRoute(path, this.state.s);
      this.renderer.drawRobotPose(pose);
      this.callbacks.onPose(pose);
      this.callbacks.onProgress(path[this.state.index].edgeId);
      this.animationFrame = requestAnimationFrame((nextTs) => this.step(nextTs));
    }

    collisionAhead(path, index, obstacles, obstacleAreas) {
      const cfg = this.renderer.robotConfig();
      const startDistance = path[index].s;
      const checkDistance = Math.max(cfg.lookahead, cfg.stopDistance);
      const poses = [];

      for (let i = index; i < path.length; i += 1) {
        const pose = path[i];
        if (pose.s - startDistance > checkDistance) {
          break;
        }
        poses.push(pose);
      }

      const collision = this.collisionService.checkPoses(poses, obstacles, obstacleAreas);
      this.renderer.drawLookahead(poses, collision.blocked);
      return collision;
    }

    obstacleHitsPose(obstacle, pose) {
      const cfg = this.renderer.robotConfig();
      const radius = (obstacle.radius || 0.08) + cfg.collisionMargin;
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);
      const dx = obstacle.x - pose.x;
      const dy = obstacle.y - pose.y;
      const localX = (dx * cos) + (dy * sin);
      const localY = (-dx * sin) + (dy * cos);
      const localPoint = { x: localX, y: localY };
      return (
        this.pointInPolygon(localPoint, cfg.footprint) ||
        this.distanceToPolygon(localPoint, cfg.footprint) <= radius
      );
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

    distanceToPolygon(point, polygon) {
      let best = Number.POSITIVE_INFINITY;
      for (let i = 0; i < polygon.length; i += 1) {
        const start = polygon[i];
        const end = polygon[(i + 1) % polygon.length];
        best = Math.min(best, this.distanceToSegment(point, start, end));
      }
      return best;
    }

    distanceToSegment(point, start, end) {
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const lengthSq = (dx * dx) + (dy * dy);
      if (lengthSq <= 0.000001) {
        return Math.hypot(point.x - start.x, point.y - start.y);
      }
      const t = Math.max(0, Math.min(1, (((point.x - start.x) * dx) + ((point.y - start.y) * dy)) / lengthSq));
      const closest = {
        x: start.x + (dx * t),
        y: start.y + (dy * t),
      };
      return Math.hypot(point.x - closest.x, point.y - closest.y);
    }
  }

  class ViewportController {
    constructor(dom, geometry, state, onObstacleAdd, onObstacleAreaAdd, onObstacleAreaPreview) {
      this.dom = dom;
      this.geometry = geometry;
      this.state = state;
      this.onObstacleAdd = onObstacleAdd;
      this.onObstacleAreaAdd = onObstacleAreaAdd;
      this.onObstacleAreaPreview = onObstacleAreaPreview;
      this.view = { zoom: 1, panX: 0, panY: 0, rotation: 0 };
      this.center = {
        x: demoData.map.viewWidth / 2,
        y: demoData.map.viewHeight / 2,
      };
    }

    applyTransform() {
      this.dom.viewport.setAttribute(
        "transform",
        `translate(${this.view.panX} ${this.view.panY}) translate(${this.center.x} ${this.center.y}) rotate(${this.view.rotation}) scale(${this.view.zoom}) translate(${-this.center.x} ${-this.center.y})`
      );
    }

    zoom(multiplier) {
      this.view.zoom = Math.min(8, Math.max(0.35, this.view.zoom * multiplier));
      this.applyTransform();
    }

    rotate(deltaDegrees) {
      this.view.rotation = (this.view.rotation + deltaDegrees) % 360;
      this.applyTransform();
    }

    reset() {
      this.view = { zoom: 1, panX: 0, panY: 0, rotation: 0 };
      this.applyTransform();
    }

    makeArea(start, end) {
      return {
        x1: start.x,
        y1: start.y,
        x2: end.x,
        y2: end.y,
      };
    }

    enable() {
      let active = false;
      let areaDrag = null;
      let lastX = 0;
      let lastY = 0;
      let downX = 0;
      let downY = 0;

      this.dom.mapSvg.addEventListener("pointerdown", (event) => {
        downX = event.clientX;
        downY = event.clientY;
        if (this.state.obstacleAreaMode) {
          const world = this.geometry.eventToWorld(event, this.dom.viewport);
          if (world) {
            areaDrag = { start: world };
            this.onObstacleAreaPreview(this.makeArea(world, world));
            this.dom.mapSvg.setPointerCapture(event.pointerId);
          }
          return;
        }
        if (this.state.obstacleMode || event.target.closest("[data-lm]")) {
          return;
        }
        active = true;
        lastX = event.clientX;
        lastY = event.clientY;
        this.dom.mapSvg.setPointerCapture(event.pointerId);
      });

      this.dom.mapSvg.addEventListener("pointermove", (event) => {
        if (areaDrag) {
          const world = this.geometry.eventToWorld(event, this.dom.viewport);
          if (world) {
            this.onObstacleAreaPreview(this.makeArea(areaDrag.start, world));
          }
          return;
        }
        if (!active) {
          return;
        }
        this.view.panX += event.clientX - lastX;
        this.view.panY += event.clientY - lastY;
        lastX = event.clientX;
        lastY = event.clientY;
        this.applyTransform();
      });

      const stop = (event) => {
        if (areaDrag && event) {
          const world = this.geometry.eventToWorld(event, this.dom.viewport);
          if (world) {
            const area = this.makeArea(areaDrag.start, world);
            const width = Math.abs(area.x2 - area.x1);
            const height = Math.abs(area.y2 - area.y1);
            if (width >= 0.05 && height >= 0.05) {
              this.onObstacleAreaAdd(area);
            } else {
              this.onObstacleAreaPreview(null);
            }
          } else {
            this.onObstacleAreaPreview(null);
          }
          areaDrag = null;
          if (this.dom.mapSvg.hasPointerCapture(event.pointerId)) {
            this.dom.mapSvg.releasePointerCapture(event.pointerId);
          }
          return;
        }
        if (this.state.obstacleMode && event) {
          const moved = Math.hypot(event.clientX - downX, event.clientY - downY);
          if (moved < 6) {
            const world = this.geometry.eventToWorld(event, this.dom.viewport);
            if (world) {
              this.onObstacleAdd({ x: world.x, y: world.y, radius: 0.08 });
            }
          }
          if (this.dom.mapSvg.hasPointerCapture(event.pointerId)) {
            this.dom.mapSvg.releasePointerCapture(event.pointerId);
          }
          return;
        }

        active = false;
        if (event && this.dom.mapSvg.hasPointerCapture(event.pointerId)) {
          this.dom.mapSvg.releasePointerCapture(event.pointerId);
        }
      };

      this.dom.mapSvg.addEventListener("pointerup", stop);
      this.dom.mapSvg.addEventListener("pointercancel", stop);
      this.dom.mapSvg.addEventListener("wheel", (event) => {
        event.preventDefault();
        this.zoom(event.deltaY < 0 ? 1.1 : 0.9);
      }, { passive: false });
    }
  }

  class RouteSimApp {
    constructor(data) {
      this.data = data;
      this.dom = this.getDom();
      this.geometry = new GeometryService(data.map);
      this.graphModel = new GraphModel(data.lms, data.edges, data.routes || {});
      this.missionPlanner = new MissionPlanner(this.graphModel, this.geometry);
      this.state = {
        mode: "IDLE",
        targetName: "",
        navigateMode: false,
        obstacleMode: false,
        obstacleAreaMode: false,
      };
      this.obstacles = [];
      this.obstacleAreas = [];
      this.obstacleAreaPreview = null;
      this.currentMission = null;
      this.currentPose = this.initialRobotPose();
      this.manualKeys = new Set();
      this.manualAnimationFrame = null;
      this.manualLastTs = null;
      this.renderer = new Renderer(
        this.dom,
        this.graphModel,
        this.geometry,
        (lmName) => this.handleLandmarkClick(lmName)
      );
      this.robotModelEditor = new RobotModelEditor(
        this.dom,
        (model) => this.handleRobotModelChange(model)
      );
      this.occupancyChecker = new MapOccupancyChecker(data.map, this.geometry);
      this.collisionService = new CollisionService(this.renderer, this.occupancyChecker);
      this.simulator = new BehaviorSimulator(this.renderer, this.missionPlanner, this.collisionService, {
        onPose: (pose) => this.setRobotPose(pose),
        onProgress: (edgeId) => this.setStatus(`Driving ${edgeId}`),
        onArrive: (pose) => this.handleArrived(pose),
        onBlocked: (count) => this.handleBlocked(count),
      });
      this.viewport = new ViewportController(
        this.dom,
        this.geometry,
        this.state,
        (obstacle) => this.addObstacle(obstacle),
        (area) => this.addObstacleArea(area),
        (area) => this.previewObstacleArea(area)
      );
    }

    getDom() {
      return {
        mapTitle: document.getElementById("mapTitle"),
        statusText: document.getElementById("statusText"),
        mapSvg: document.getElementById("mapSvg"),
        viewport: document.getElementById("viewport"),
        mapImage: document.getElementById("mapImage"),
        graphLayer: document.getElementById("graphLayer"),
        routeLayer: document.getElementById("routeLayer"),
        lookaheadLayer: document.getElementById("lookaheadLayer"),
        obstacleLayer: document.getElementById("obstacleLayer"),
        pointLayer: document.getElementById("pointLayer"),
        robotLayer: document.getElementById("robotLayer"),
        tabButtons: Array.from(document.querySelectorAll(".tab-button")),
        tabPages: Array.from(document.querySelectorAll(".tab-page")),
        navigateButton: document.getElementById("navigateButton"),
        obstacleModeButton: document.getElementById("obstacleModeButton"),
        obstacleAreaModeButton: document.getElementById("obstacleAreaModeButton"),
        stopButton: document.getElementById("stopButton"),
        resetRobotButton: document.getElementById("resetRobotButton"),
        clearObstaclesButton: document.getElementById("clearObstaclesButton"),
        saveRobotParamsButton: document.getElementById("saveRobotParamsButton"),
        saveParamsBottomButton: document.getElementById("saveParamsBottomButton"),
        zoomInButton: document.getElementById("zoomInButton"),
        zoomOutButton: document.getElementById("zoomOutButton"),
        rotateLeftButton: document.getElementById("rotateLeftButton"),
        rotateRightButton: document.getElementById("rotateRightButton"),
        resetViewButton: document.getElementById("resetViewButton"),
        robotEditorZoomInButton: document.getElementById("robotEditorZoomInButton"),
        robotEditorZoomOutButton: document.getElementById("robotEditorZoomOutButton"),
        robotEditorResetViewButton: document.getElementById("robotEditorResetViewButton"),
        manualButtons: Array.from(document.querySelectorAll("[data-manual-key]")),
        modeText: document.getElementById("modeText"),
        targetText: document.getElementById("targetText"),
        nearestText: document.getElementById("nearestText"),
        routeLength: document.getElementById("routeLength"),
        poseText: document.getElementById("poseText"),
        routeList: document.getElementById("routeList"),
        robotEditorSvg: document.getElementById("robotEditorSvg"),
        footprintFields: document.getElementById("footprintFields"),
        tfFields: document.getElementById("tfFields"),
        resetModelButton: document.getElementById("resetModelButton"),
        speedInput: document.getElementById("speedInput"),
        lookaheadInput: document.getElementById("lookaheadInput"),
        collisionMarginInput: document.getElementById("collisionMarginInput"),
        nearestToleranceInput: document.getElementById("nearestToleranceInput"),
        onRouteToleranceInput: document.getElementById("onRouteToleranceInput"),
        sampleDistanceInput: document.getElementById("sampleDistanceInput"),
        stopDistanceInput: document.getElementById("stopDistanceInput"),
        precisionStartInput: document.getElementById("precisionStartInput"),
        poseSourceInput: document.getElementById("poseSourceInput"),
        localizationTimeoutInput: document.getElementById("localizationTimeoutInput"),
        lateralErrorInput: document.getElementById("lateralErrorInput"),
        yawErrorInput: document.getElementById("yawErrorInput"),
        manualLinearSpeedInput: document.getElementById("manualLinearSpeedInput"),
        manualAngularSpeedInput: document.getElementById("manualAngularSpeedInput"),
        manualLookaheadInput: document.getElementById("manualLookaheadInput"),
        manualStepInput: document.getElementById("manualStepInput"),
        saveConfirmOverlay: document.getElementById("saveConfirmOverlay"),
        saveConfirmYesButton: document.getElementById("saveConfirmYesButton"),
        saveConfirmNoButton: document.getElementById("saveConfirmNoButton"),
      };
    }

    async init() {
      this.renderer.initMap();
      this.renderer.drawGraph();
      await this.occupancyChecker.readyPromise;
      this.viewport.enable();
      this.applyParams(await this.loadRuntimeParams());
      this.robotModelEditor.init();
      this.attachEvents();
      this.updatePlannerParams();
      this.renderAll();
      this.setStatus("Ready.");
    }

    attachEvents() {
      for (const button of this.dom.tabButtons) {
        button.addEventListener("click", () => this.setActiveTab(button.dataset.tab));
      }
      this.dom.navigateButton.addEventListener("click", () => this.toggleNavigateMode());
      this.dom.obstacleModeButton.addEventListener("click", () => this.toggleObstacleMode());
      this.dom.obstacleAreaModeButton.addEventListener("click", () => this.toggleObstacleAreaMode());
      this.dom.stopButton.addEventListener("click", () => this.stopRobot());
      this.dom.resetRobotButton.addEventListener("click", () => this.resetRobot());
      this.dom.clearObstaclesButton.addEventListener("click", () => this.clearObstacles());
      this.dom.saveRobotParamsButton.addEventListener("click", () => this.confirmAndSaveParams());
      this.dom.saveParamsBottomButton.addEventListener("click", () => this.confirmAndSaveParams());
      this.dom.zoomInButton.addEventListener("click", () => this.viewport.zoom(1.2));
      this.dom.zoomOutButton.addEventListener("click", () => this.viewport.zoom(0.85));
      this.dom.rotateLeftButton.addEventListener("click", () => this.viewport.rotate(-10));
      this.dom.rotateRightButton.addEventListener("click", () => this.viewport.rotate(10));
      this.dom.resetViewButton.addEventListener("click", () => this.viewport.reset());
      this.dom.nearestToleranceInput.addEventListener("change", () => this.updatePlannerParams());
      this.dom.onRouteToleranceInput.addEventListener("change", () => this.updatePlannerParams());
      this.dom.sampleDistanceInput.addEventListener("change", () => this.updatePlannerParams());
      this.dom.lookaheadInput.addEventListener("change", () => this.renderer.drawRobotPose(this.currentPose));
      this.dom.collisionMarginInput.addEventListener("change", () => this.renderer.drawRobotPose(this.currentPose));
      this.attachManualEvents();
    }

    attachManualEvents() {
      window.addEventListener("keydown", (event) => this.handleManualKey(event, true));
      window.addEventListener("keyup", (event) => this.handleManualKey(event, false));
      window.addEventListener("blur", () => this.stopManualControl());

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

    handleManualKey(event, pressed) {
      if (this.isTypingTarget(event.target)) {
        return;
      }
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) {
        return;
      }
      event.preventDefault();
      this.setManualKey(key, pressed);
    }

    isTypingTarget(target) {
      return target && ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName);
    }

    setManualKey(key, pressed) {
      if (pressed) {
        this.manualKeys.add(key);
        this.startManualControl();
      } else {
        this.manualKeys.delete(key);
      }
      this.syncManualButtons();
    }

    syncManualButtons() {
      for (const button of this.dom.manualButtons) {
        button.classList.toggle("active", this.manualKeys.has(button.dataset.manualKey));
      }
    }

    startManualControl() {
      if (this.manualAnimationFrame !== null) {
        return;
      }
      this.simulator.stop(false);
      this.currentMission = null;
      this.state.targetName = "";
      this.state.navigateMode = false;
      this.state.obstacleMode = false;
      this.state.obstacleAreaMode = false;
      this.obstacleAreaPreview = null;
      this.syncModeButtons();
      this.drawObstacleState();
      this.dom.targetText.textContent = "-";
      this.dom.routeLength.textContent = "0.00 m";
      this.renderer.updateRouteList([]);
      this.renderer.drawRoute([]);
      this.manualLastTs = null;
      this.manualAnimationFrame = requestAnimationFrame((ts) => this.stepManualControl(ts));
    }

    stopManualControl(clearLookahead = true) {
      if (this.manualAnimationFrame !== null) {
        cancelAnimationFrame(this.manualAnimationFrame);
        this.manualAnimationFrame = null;
      }
      this.manualLastTs = null;
      this.manualKeys.clear();
      this.syncManualButtons();
      if (clearLookahead) {
        this.renderer.clearLookahead();
      }
    }

    manualParams() {
      return {
        linearSpeed: Math.max(0.02, Number(this.dom.manualLinearSpeedInput.value) || 0.25),
        angularSpeed: Math.max(0.05, Number(this.dom.manualAngularSpeedInput.value) || 0.9),
        predictionTime: Math.max(0.1, Number(this.dom.manualLookaheadInput.value) || 1),
        predictionStep: Math.max(0.03, Number(this.dom.manualStepInput.value) || 0.1),
      };
    }

    manualTwist() {
      const params = this.manualParams();
      const linearSign = (this.manualKeys.has("w") ? 1 : 0) - (this.manualKeys.has("s") ? 1 : 0);
      const angularSign = (this.manualKeys.has("a") ? 1 : 0) - (this.manualKeys.has("d") ? 1 : 0);
      return {
        linear: linearSign * params.linearSpeed,
        angular: angularSign * params.angularSpeed,
        params,
      };
    }

    stepManualControl(ts) {
      if (this.manualKeys.size === 0) {
        this.stopManualControl();
        if (this.state.mode === "MANUAL" || this.state.mode === "MANUAL_BLOCKED") {
          this.setMode("IDLE");
          this.setStatus("Manual control released.");
          this.renderer.drawRobotPose(this.currentPose);
        }
        return;
      }

      if (this.manualLastTs === null) {
        this.manualLastTs = ts;
      }
      const dt = Math.min(0.08, Math.max(0, (ts - this.manualLastTs) / 1000));
      this.manualLastTs = ts;

      const twist = this.manualTwist();
      const moving = Math.abs(twist.linear) > 0.0001 || Math.abs(twist.angular) > 0.0001;
      const prediction = this.predictManualTrajectory(
        this.currentPose,
        twist.linear,
        twist.angular,
        twist.params.predictionTime,
        twist.params.predictionStep
      );
      const collision = moving
        ? this.collisionService.checkPoses(prediction, this.obstacles, this.obstacleAreas)
        : { blocked: false, count: 0 };

      this.renderer.drawLookahead(prediction, collision.blocked);
      if (collision.blocked) {
        this.setMode("MANUAL_BLOCKED");
        this.setStatus(`Manual blocked: predicted footprint hits map/obstacle (${collision.count}).`);
        this.renderer.drawRobotPose(this.currentPose, true);
      } else {
        const pose = moving ? this.integratePose(this.currentPose, twist.linear, twist.angular, dt) : this.currentPose;
        this.setRobotPose(pose);
        this.setMode("MANUAL");
        this.setStatus("Manual control active.");
        this.renderer.drawRobotPose(this.currentPose);
      }

      this.manualAnimationFrame = requestAnimationFrame((nextTs) => this.stepManualControl(nextTs));
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
      const yaw = this.geometry.normalizeAngle(pose.yaw + (angular * dt));
      const midYaw = this.geometry.normalizeAngle(pose.yaw + ((angular * dt) / 2));
      return {
        x: pose.x + (linear * Math.cos(midYaw) * dt),
        y: pose.y + (linear * Math.sin(midYaw) * dt),
        yaw,
      };
    }

    setActiveTab(tabName) {
      for (const button of this.dom.tabButtons) {
        button.classList.toggle("active", button.dataset.tab === tabName);
      }
      for (const page of this.dom.tabPages) {
        page.classList.toggle("active", page.id === `tab${tabName[0].toUpperCase()}${tabName.slice(1)}`);
      }
    }

    updatePlannerParams() {
      this.missionPlanner.lmTolerance = Math.max(0, Number(this.dom.nearestToleranceInput.value) || 0);
      this.missionPlanner.sampleDistance = Math.max(0.01, Number(this.dom.sampleDistanceInput.value) || 0.05);
      this.missionPlanner.onRouteTolerance = Math.max(0, Number(this.dom.onRouteToleranceInput.value) || 0);
    }

    async loadRuntimeParams() {
      try {
        const response = await fetch("/api/params", { cache: "no-store" });
        if (response.ok) {
          return await response.json();
        }
      } catch (_) {
        // Static file mode has no API; use params embedded during build.
      }
      const stored = localStorage.getItem("warehouse_robot_params");
      if (stored) {
        try {
          return JSON.parse(stored);
        } catch (_) {
          localStorage.removeItem("warehouse_robot_params");
        }
      }
      return this.data.params || {};
    }

    applyParams(params) {
      const navigation = params.navigation || {};
      const planner = params.planner || {};
      const localization = params.localization || {};
      const manual = params.manual || {};

      this.setInputValue(this.dom.speedInput, navigation.route_speed);
      this.setInputValue(this.dom.lookaheadInput, navigation.footprint_lookahead);
      this.setInputValue(this.dom.collisionMarginInput, navigation.collision_margin);
      this.setInputValue(this.dom.stopDistanceInput, navigation.stop_distance);
      this.setInputValue(this.dom.nearestToleranceInput, planner.nearest_lm_tolerance);
      this.setInputValue(this.dom.onRouteToleranceInput, planner.on_route_tolerance);
      this.setInputValue(this.dom.sampleDistanceInput, planner.trajectory_sample_distance);
      this.setInputValue(this.dom.precisionStartInput, planner.precision_start_distance);
      this.setInputValue(this.dom.poseSourceInput, localization.pose_source);
      this.setInputValue(this.dom.localizationTimeoutInput, localization.localization_timeout);
      this.setInputValue(this.dom.lateralErrorInput, localization.allowed_lateral_error);
      this.setInputValue(this.dom.yawErrorInput, localization.allowed_yaw_error_deg);
      this.setInputValue(this.dom.manualLinearSpeedInput, manual.linear_speed);
      this.setInputValue(this.dom.manualAngularSpeedInput, manual.angular_speed);
      this.setInputValue(this.dom.manualLookaheadInput, manual.prediction_time);
      this.setInputValue(this.dom.manualStepInput, manual.prediction_step);

      if (params.robot_model) {
        this.robotModelEditor.setModel(params.robot_model);
      }
    }

    setInputValue(input, value) {
      if (input && value !== undefined && value !== null) {
        input.value = String(value);
      }
    }

    collectParams() {
      return {
        robot_model: this.robotModelEditor.getModel(),
        navigation: {
          route_speed: Number(this.dom.speedInput.value),
          footprint_lookahead: Number(this.dom.lookaheadInput.value),
          collision_margin: Number(this.dom.collisionMarginInput.value),
          stop_distance: Number(this.dom.stopDistanceInput.value),
        },
        planner: {
          nearest_lm_tolerance: Number(this.dom.nearestToleranceInput.value),
          on_route_tolerance: Number(this.dom.onRouteToleranceInput.value),
          trajectory_sample_distance: Number(this.dom.sampleDistanceInput.value),
          precision_start_distance: Number(this.dom.precisionStartInput.value),
        },
        localization: {
          pose_source: this.dom.poseSourceInput.value,
          localization_timeout: Number(this.dom.localizationTimeoutInput.value),
          allowed_lateral_error: Number(this.dom.lateralErrorInput.value),
          allowed_yaw_error_deg: Number(this.dom.yawErrorInput.value),
        },
        manual: {
          linear_speed: Number(this.dom.manualLinearSpeedInput.value),
          angular_speed: Number(this.dom.manualAngularSpeedInput.value),
          prediction_time: Number(this.dom.manualLookaheadInput.value),
          prediction_step: Number(this.dom.manualStepInput.value),
        },
      };
    }

    async confirmAndSaveParams() {
      const confirmed = await this.askSaveConfirmation();
      if (!confirmed) {
        this.setStatus("Save canceled.");
        return;
      }
      await this.saveParams();
    }

    askSaveConfirmation() {
      return new Promise((resolve) => {
        const overlay = this.dom.saveConfirmOverlay;
        const yesButton = this.dom.saveConfirmYesButton;
        const noButton = this.dom.saveConfirmNoButton;
        overlay.hidden = false;
        yesButton.focus();

        const cleanup = (value) => {
          overlay.hidden = true;
          yesButton.removeEventListener("click", yes);
          noButton.removeEventListener("click", no);
          overlay.removeEventListener("click", overlayClick);
          window.removeEventListener("keydown", keydown);
          resolve(value);
        };
        const yes = () => cleanup(true);
        const no = () => cleanup(false);
        const overlayClick = (event) => {
          if (event.target === overlay) {
            cleanup(false);
          }
        };
        const keydown = (event) => {
          if (event.key === "Escape") {
            cleanup(false);
          }
          if (event.key === "Enter") {
            cleanup(true);
          }
        };

        yesButton.addEventListener("click", yes);
        noButton.addEventListener("click", no);
        overlay.addEventListener("click", overlayClick);
        window.addEventListener("keydown", keydown);
      });
    }

    async saveParams() {
      const params = this.collectParams();
      try {
        const response = await fetch("/api/params", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(params),
        });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        this.data.params = params;
        this.setStatus("Saved params.yaml.");
      } catch (_) {
        localStorage.setItem("warehouse_robot_params", JSON.stringify(params));
        this.setStatus("Saved in browser only. Run serve_web.py to write params.yaml.");
      }
    }

    handleRobotModelChange(model) {
      this.renderer.setRobotModel(model);
      if (this.currentPose) {
        this.renderer.drawRobotPose(this.currentPose);
      }
    }

    initialRobotPose() {
      const start = this.graphModel.nodeByName.get(demoData.defaultStart) || this.graphModel.landmarks[0];
      const goal = this.graphModel.nodeByName.get(demoData.defaultGoal);
      const yaw = goal ? Math.atan2(goal.y - start.y, goal.x - start.x) : 0;
      return { x: start.x, y: start.y, yaw };
    }

    selectedSpeed() {
      return Math.max(0.02, Number(this.dom.speedInput.value) || 0.35);
    }

    setRobotPose(pose) {
      if (!pose) {
        return;
      }
      this.currentPose = { x: pose.x, y: pose.y, yaw: pose.yaw };
      this.updateTelemetry();
    }

    setMode(mode) {
      this.state.mode = mode;
      this.dom.modeText.textContent = mode;
    }

    setStatus(text) {
      this.dom.statusText.textContent = text;
    }

    toggleNavigateMode() {
      this.state.navigateMode = !this.state.navigateMode;
      if (this.state.navigateMode) {
        this.state.obstacleMode = false;
        this.state.obstacleAreaMode = false;
        this.obstacleAreaPreview = null;
      }
      this.syncModeButtons();
      this.drawObstacleState();
      this.renderLandmarks();
      this.setStatus(this.state.navigateMode ? "Navigate armed: select an LM." : "Navigate canceled.");
    }

    toggleObstacleMode() {
      this.state.obstacleMode = !this.state.obstacleMode;
      if (this.state.obstacleMode) {
        this.state.navigateMode = false;
        this.state.obstacleAreaMode = false;
        this.obstacleAreaPreview = null;
      }
      this.syncModeButtons();
      this.drawObstacleState();
      this.renderLandmarks();
      this.setStatus(this.state.obstacleMode ? "Obstacle placement active." : "Obstacle placement off.");
    }

    toggleObstacleAreaMode() {
      this.state.obstacleAreaMode = !this.state.obstacleAreaMode;
      if (this.state.obstacleAreaMode) {
        this.state.navigateMode = false;
        this.state.obstacleMode = false;
      } else {
        this.obstacleAreaPreview = null;
      }
      this.syncModeButtons();
      this.drawObstacleState();
      this.renderLandmarks();
      this.setStatus(this.state.obstacleAreaMode ? "Obstacle area drawing active." : "Obstacle area drawing off.");
    }

    syncModeButtons() {
      this.dom.navigateButton.classList.toggle("active", this.state.navigateMode);
      this.dom.navigateButton.textContent = this.state.navigateMode ? "Select Target LM" : "Navigate To LM";
      this.dom.obstacleModeButton.classList.toggle("active", this.state.obstacleMode);
      this.dom.obstacleAreaModeButton.classList.toggle("active", this.state.obstacleAreaMode);
    }

    handleLandmarkClick(lmName) {
      if (this.state.obstacleMode || this.state.obstacleAreaMode) {
        return;
      }
      if (!this.state.navigateMode) {
        this.setStatus("Press Navigate To LM first, then select an LM.");
        return;
      }
      this.state.navigateMode = false;
      this.syncModeButtons();
      this.startNavigation(lmName);
    }

    startNavigation(targetName) {
      this.stopManualControl(false);
      this.updatePlannerParams();
      const mission = this.missionPlanner.buildMission(this.currentPose, targetName, this.selectedSpeed());
      if (!mission) {
        this.setMode("ERROR_ROUTE");
        this.setStatus(`No route to ${targetName}.`);
        return;
      }

      this.currentMission = mission;
      this.state.targetName = targetName;
      this.setMode("FOLLOW_ROUTE");
      this.dom.targetText.textContent = targetName;
      this.dom.routeLength.textContent = `${mission.length.toFixed(2)} m`;
      this.renderer.updateRouteList(mission.nodes);
      this.renderer.drawRoute(mission.path);
      this.renderLandmarks();
      if (mission.startMode === "CURRENT_ROUTE") {
        this.setStatus(`On current route ${mission.currentEdgeId}: ${mission.nodes.join(" -> ")}`);
      } else {
        this.setStatus(`Strict route: ${mission.nodes.join(" -> ")}`);
      }
      this.simulator.start(mission, this.selectedSpeed(), this.obstacles, this.obstacleAreas);
    }

    stopRobot() {
      this.simulator.stop();
      this.stopManualControl(false);
      this.currentMission = null;
      this.setMode("IDLE");
      this.setStatus("Stopped.");
      this.dom.routeLength.textContent = "0.00 m";
      this.renderer.updateRouteList([]);
      this.renderer.drawRoute([]);
      this.renderer.clearLookahead();
      this.renderer.drawRobotPose(this.currentPose);
    }

    resetRobot() {
      this.simulator.stop();
      this.stopManualControl(false);
      this.currentPose = this.initialRobotPose();
      this.currentMission = null;
      this.state.targetName = "";
      this.setMode("IDLE");
      this.dom.targetText.textContent = "-";
      this.dom.routeLength.textContent = "0.00 m";
      this.renderer.updateRouteList([]);
      this.renderer.drawRoute([]);
      this.renderer.clearLookahead();
      this.renderAll();
      this.setStatus("Robot reset.");
    }

    addObstacle(obstacle) {
      this.obstacles.push(obstacle);
      this.drawObstacleState();
      this.setStatus("Obstacle added.");
    }

    addObstacleArea(area) {
      this.obstacleAreaPreview = null;
      this.obstacleAreas.push(area);
      this.drawObstacleState();
      this.setStatus("Obstacle area added.");
    }

    previewObstacleArea(area) {
      this.obstacleAreaPreview = area;
      this.drawObstacleState();
    }

    drawObstacleState() {
      this.renderer.drawObstacles(this.obstacles, this.obstacleAreas, this.obstacleAreaPreview);
    }

    clearObstacles() {
      this.obstacles = [];
      this.obstacleAreas = [];
      this.obstacleAreaPreview = null;
      this.drawObstacleState();
      this.renderer.clearLookahead();
      this.setStatus("Obstacles cleared.");
    }

    handleArrived(pose) {
      if (pose) {
        this.setRobotPose(pose);
      }
      this.setMode("ARRIVED");
      this.setStatus(`Arrived at ${this.state.targetName || "target"}.`);
      this.renderer.clearLookahead();
    }

    handleBlocked(count) {
      this.setMode("ERROR_BLOCKED");
      this.setStatus(`Blocked: predicted footprint hits map/obstacle (${count}).`);
    }

    renderAll() {
      this.renderLandmarks();
      this.drawObstacleState();
      this.renderer.drawRobotPose(this.currentPose);
      this.updateTelemetry();
    }

    renderLandmarks() {
      const nearest = this.graphModel.nearestLandmark(this.currentPose, this.geometry);
      this.renderer.drawLandmarks(
        nearest.landmark ? nearest.landmark.name : "",
        this.state.targetName,
        this.state.navigateMode
      );
    }

    updateTelemetry() {
      const nearest = this.graphModel.nearestLandmark(this.currentPose, this.geometry);
      this.dom.nearestText.textContent = nearest.landmark
        ? `${nearest.landmark.name} (${nearest.distance.toFixed(2)} m)`
        : "-";
      this.dom.poseText.textContent =
        `x: ${this.currentPose.x.toFixed(3)}, y: ${this.currentPose.y.toFixed(3)}, yaw: ${this.currentPose.yaw.toFixed(3)}`;
      this.renderLandmarks();
    }
  }

  new RouteSimApp(demoData).init();
}());
