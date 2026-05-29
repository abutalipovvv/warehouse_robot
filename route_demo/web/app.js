(function () {
  const demoData = window.ROUTE_DEMO_DATA;
  if (!demoData) {
    throw new Error("ROUTE_DEMO_DATA is missing.");
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
      const dx = second.x - first.x;
      const dy = second.y - first.y;
      return Math.hypot(dx, dy);
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

  class RoutePlanner {
    constructor(landmarks, edges, geometry) {
      this.landmarks = landmarks;
      this.edges = edges;
      this.geometry = geometry;
      this.nodeByName = new Map();
      this.adjacency = new Map();
      this.edgeByKey = new Map();
      this.buildData();
    }

    buildData() {
      for (const lm of this.landmarks) {
        this.nodeByName.set(lm.name, lm);
        this.ensureAdjacency(lm.name);
      }

      for (const edge of this.edges) {
        this.edgeByKey.set(`${edge.from}|${edge.to}`, edge);
        this.ensureAdjacency(edge.from).push({
          to: edge.to,
          length: edge.length,
          edge,
        });
      }
    }

    ensureAdjacency(name) {
      if (!this.adjacency.has(name)) {
        this.adjacency.set(name, []);
      }
      return this.adjacency.get(name);
    }

    heuristic(nameA, nameB) {
      return this.geometry.distance(this.nodeByName.get(nameA), this.nodeByName.get(nameB));
    }

    findRoute(startName, goalName) {
      const open = [{ name: startName, f: 0 }];
      const cameFrom = new Map();
      const gScore = new Map([[startName, 0]]);

      while (open.length > 0) {
        open.sort((a, b) => a.f - b.f);
        const current = open.shift().name;

        if (current === goalName) {
          const route = [current];
          let cursor = current;
          while (cameFrom.has(cursor)) {
            cursor = cameFrom.get(cursor);
            route.push(cursor);
          }
          route.reverse();
          return {
            nodes: route,
            length: gScore.get(goalName),
          };
        }

        for (const edge of this.adjacency.get(current) || []) {
          const tentative = gScore.get(current) + edge.length;
          const known = gScore.has(edge.to) ? gScore.get(edge.to) : Number.POSITIVE_INFINITY;
          if (tentative >= known) {
            continue;
          }

          cameFrom.set(edge.to, current);
          gScore.set(edge.to, tentative);
          open.push({
            name: edge.to,
            f: tentative + this.heuristic(edge.to, goalName),
          });
        }
      }

      return null;
    }

    getEdge(fromName, toName) {
      return this.edgeByKey.get(`${fromName}|${toName}`) || null;
    }
  }

  class TrajectoryPlanner {
    constructor(routePlanner, geometry) {
      this.routePlanner = routePlanner;
      this.geometry = geometry;
    }

    sampleLine(start, goal, edgeId, spacing) {
      const length = Math.max(spacing, this.geometry.distance(start, goal));
      const steps = Math.max(2, Math.ceil(length / spacing));
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

    buildTrajectory(route, speedMetersPerSec) {
      if (!route || route.nodes.length < 2) {
        return [];
      }

      const spacing = 0.05;
      const trajectory = [];
      for (let i = 0; i < route.nodes.length - 1; i += 1) {
        const fromName = route.nodes[i];
        const toName = route.nodes[i + 1];
        const edge = this.routePlanner.getEdge(fromName, toName);
        const edgeId = `${fromName}->${toName}`;
        let samples;

        if (edge && edge.geometry === "bezier" && edge.control_points && edge.control_points.length === 4) {
          samples = this.sampleBezier(edge.control_points, edgeId, spacing);
        } else {
          samples = this.sampleLine(
            this.routePlanner.nodeByName.get(fromName),
            this.routePlanner.nodeByName.get(toName),
            edgeId,
            spacing
          );
        }

        if (trajectory.length > 0) {
          samples = samples.slice(1);
        }
        trajectory.push(...samples);
      }

      let distance = 0;
      for (let i = 0; i < trajectory.length; i += 1) {
        if (i > 0) {
          distance += this.geometry.distance(trajectory[i - 1], trajectory[i]);
        }
        trajectory[i].s = distance;
        trajectory[i].targetSpeed = speedMetersPerSec;
      }
      return trajectory;
    }

    poseAtDistance(trajectory, distance) {
      if (distance <= 0) {
        return trajectory[0];
      }
      const last = trajectory[trajectory.length - 1];
      if (distance >= last.s) {
        return last;
      }

      let index = 0;
      while (index < trajectory.length - 2 && trajectory[index + 1].s < distance) {
        index += 1;
      }

      const start = trajectory[index];
      const goal = trajectory[index + 1];
      const span = Math.max(0.0001, goal.s - start.s);
      const t = (distance - start.s) / span;
      return {
        x: start.x + ((goal.x - start.x) * t),
        y: start.y + ((goal.y - start.y) * t),
        yaw: start.yaw + ((goal.yaw - start.yaw) * t),
        s: distance,
        edgeId: start.edgeId,
      };
    }
  }

  class MapRenderer {
    constructor(dom, routePlanner, geometry) {
      this.dom = dom;
      this.routePlanner = routePlanner;
      this.geometry = geometry;
    }

    initMapFrame() {
      const map = demoData.map;
      this.dom.mapTitle.textContent = demoData.mapName;
      document.title = `${demoData.mapName} Route Demo`;
      this.dom.mapSvg.setAttribute("viewBox", `0 0 ${map.viewWidth} ${map.viewHeight}`);
      this.dom.mapImage.setAttribute("x", String(map.viewPadding));
      this.dom.mapImage.setAttribute("y", String(map.viewPadding));
      this.dom.mapImage.setAttribute("width", String(map.width));
      this.dom.mapImage.setAttribute("height", String(map.height));
      this.dom.mapImage.setAttribute("href", map.imageDataUrl);
    }

    populateSelectors() {
      for (const lm of this.routePlanner.landmarks) {
        const startOption = document.createElement("option");
        startOption.value = lm.name;
        startOption.textContent = lm.name;
        this.dom.startSelect.appendChild(startOption);

        const goalOption = document.createElement("option");
        goalOption.value = lm.name;
        goalOption.textContent = lm.name;
        this.dom.goalSelect.appendChild(goalOption);
      }

      this.dom.startSelect.value = demoData.defaultStart;
      this.dom.goalSelect.value = demoData.defaultGoal;
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

      for (const edge of this.routePlanner.edges) {
        const key = [edge.from, edge.to].sort().join("|");
        if (seen.has(key)) {
          continue;
        }
        seen.add(key);

        const start = this.geometry.worldToPixel(this.routePlanner.nodeByName.get(edge.from));
        const goal = this.geometry.worldToPixel(this.routePlanner.nodeByName.get(edge.to));
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
        } else {
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
    }

    drawPoints(startName, goalName) {
      this.dom.pointLayer.innerHTML = "";

      for (const lm of this.routePlanner.landmarks) {
        const pos = this.geometry.worldToPixel(lm);
        let fill = "var(--point)";
        let radius = 4;

        if (lm.name === startName) {
          fill = "var(--start)";
          radius = 7;
        } else if (lm.name === goalName) {
          fill = "var(--goal)";
          radius = 7;
        }

        this.dom.pointLayer.appendChild(
          this.createSvgElement("circle", {
            cx: pos.x,
            cy: pos.y,
            r: radius,
            fill,
            opacity: 0.95,
          })
        );

        const label = this.createSvgElement("text", {
          x: pos.x,
          y: pos.y + radius + 12,
          class: "lm-label",
        });
        label.textContent = lm.name;
        this.dom.pointLayer.appendChild(label);
      }
    }

    updateRouteInfo(route) {
      this.dom.routeLength.textContent = route ? `${route.length.toFixed(2)} m` : "No route";
      this.dom.routeList.innerHTML = "";
      if (!route) {
        return;
      }

      for (const name of route.nodes) {
        const item = document.createElement("li");
        item.textContent = name;
        this.dom.routeList.appendChild(item);
      }
    }

    drawRoute(trajectory) {
      this.dom.pathLayer.innerHTML = "";
      if (!trajectory || trajectory.length < 2) {
        return;
      }

      const points = trajectory.map((pose) => {
        const pos = this.geometry.worldToPixel(pose);
        return `${pos.x},${pos.y}`;
      });

      this.dom.pathLayer.appendChild(
        this.createSvgElement("polyline", {
          points: points.join(" "),
          fill: "none",
          stroke: "var(--route)",
          "stroke-width": 6,
          "stroke-linecap": "round",
          "stroke-linejoin": "round",
          opacity: 0.92,
        })
      );
    }

    robotConfig() {
      return {
        width: Math.max(0.1, Number(this.dom.robotWidthInput.value) || 0.55),
        length: Math.max(0.1, Number(this.dom.robotLengthInput.value) || 0.70),
        lookahead: Math.max(0.1, Number(this.dom.lookaheadInput.value) || 0.8),
      };
    }

    footprintCorners(pose) {
      const cfg = this.robotConfig();
      const halfLength = cfg.length / 2;
      const halfWidth = cfg.width / 2;
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);
      return [
        { x: halfLength, y: halfWidth },
        { x: halfLength, y: -halfWidth },
        { x: -halfLength, y: -halfWidth },
        { x: -halfLength, y: halfWidth },
      ].map((corner) => ({
        x: pose.x + (corner.x * cos) - (corner.y * sin),
        y: pose.y + (corner.x * sin) + (corner.y * cos),
      }));
    }

    drawFootprint(pose, attrs = {}) {
      const points = this.footprintCorners(pose)
        .map((point) => this.geometry.worldToPixel(point))
        .map((point) => `${point.x},${point.y}`)
        .join(" ");
      return this.createSvgElement("polygon", {
        points,
        fill: attrs.fill || "var(--footprint)",
        stroke: attrs.stroke || "var(--robot)",
        "stroke-width": attrs.strokeWidth || 2,
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
          fill: blocked ? "rgba(201, 42, 42, 0.25)" : "var(--footprint)",
          stroke: blocked ? "var(--blocked)" : "var(--robot)",
          strokeWidth: 2.5,
        })
      );

      const nose = this.geometry.worldToPixel({
        x: pose.x + Math.cos(pose.yaw) * (this.robotConfig().length / 2),
        y: pose.y + Math.sin(pose.yaw) * (this.robotConfig().length / 2),
      });
      const center = this.geometry.worldToPixel(pose);
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
    }

    drawRobotAt(pathNodes) {
      if (!pathNodes || pathNodes.length === 0) {
        this.drawRobotPose(null);
        return;
      }
      const current = this.routePlanner.nodeByName.get(pathNodes[0]);
      const next = pathNodes.length > 1 ? this.routePlanner.nodeByName.get(pathNodes[1]) : null;
      const yaw = next ? Math.atan2(next.y - current.y, next.x - current.x) : 0;
      this.drawRobotPose({ x: current.x, y: current.y, yaw });
    }

    drawObstacles(obstacles) {
      this.dom.obstacleLayer.innerHTML = "";
      obstacles.forEach((obstacle, index) => {
        const pos = this.geometry.worldToPixel(obstacle);
        this.dom.obstacleLayer.appendChild(
          this.createSvgElement("circle", {
            cx: pos.x,
            cy: pos.y,
            r: Math.max(4, obstacle.radius / demoData.map.resolution),
            fill: "var(--obstacle)",
            stroke: "white",
            "stroke-width": 2,
            opacity: 0.88,
            "data-index": index,
          })
        );
      });
    }

    drawLookahead(poses, blocked) {
      this.dom.lookaheadLayer.innerHTML = "";
      for (let i = 0; i < poses.length; i += Math.max(1, Math.floor(poses.length / 8))) {
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

    setStatus(text) {
      this.dom.statusText.textContent = text;
    }
  }

  class BehaviorSimulator {
    constructor(renderer, trajectoryPlanner) {
      this.renderer = renderer;
      this.trajectoryPlanner = trajectoryPlanner;
      this.animationFrame = null;
      this.simulation = null;
      this.obstacles = [];
    }

    setObstacles(obstacles) {
      this.obstacles = obstacles;
      this.renderer.drawObstacles(this.obstacles);
    }

    clearObstacles() {
      this.setObstacles([]);
      this.renderer.clearLookahead();
    }

    stop() {
      if (this.animationFrame !== null) {
        cancelAnimationFrame(this.animationFrame);
        this.animationFrame = null;
      }
      this.simulation = null;
      this.renderer.clearLookahead();
    }

    obstacleHitsPose(obstacle, pose) {
      const cfg = this.renderer.robotConfig();
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);
      const dx = obstacle.x - pose.x;
      const dy = obstacle.y - pose.y;
      const localX = (dx * cos) + (dy * sin);
      const localY = (-dx * sin) + (dy * cos);
      const radius = obstacle.radius || 0.08;
      return (
        Math.abs(localX) <= (cfg.length / 2) + radius &&
        Math.abs(localY) <= (cfg.width / 2) + radius
      );
    }

    collisionAhead(trajectory, index) {
      const cfg = this.renderer.robotConfig();
      const startDistance = trajectory[index].s;
      const poses = [];
      for (let i = index; i < trajectory.length; i += 1) {
        const pose = trajectory[i];
        if (pose.s - startDistance > cfg.lookahead) {
          break;
        }
        poses.push(pose);
        for (const obstacle of this.obstacles) {
          if (this.obstacleHitsPose(obstacle, pose)) {
            this.renderer.drawLookahead(poses, true);
            return { blocked: true, poses };
          }
        }
      }
      this.renderer.drawLookahead(poses, false);
      return { blocked: false, poses };
    }

    animateRoute(route, trajectory, speedMetersPerSec) {
      if (!route || route.nodes.length < 2 || trajectory.length < 2) {
        this.renderer.setStatus("Route is empty.");
        return;
      }

      this.stop();
      this.simulation = {
        trajectory,
        index: 0,
        s: 0,
        lastTs: null,
      };
      this.renderer.setStatus(`Driving from ${route.nodes[0]} to ${route.nodes[route.nodes.length - 1]}`);

      const step = (ts) => {
        if (!this.simulation) {
          return;
        }

        if (this.simulation.s >= trajectory[trajectory.length - 1].s) {
          this.renderer.setStatus(`Arrived at ${route.nodes[route.nodes.length - 1]}`);
          this.renderer.drawRobotPose(trajectory[trajectory.length - 1]);
          this.animationFrame = null;
          this.simulation = null;
          this.renderer.clearLookahead();
          return;
        }

        const check = this.collisionAhead(trajectory, this.simulation.index);
        if (check.blocked) {
          this.renderer.setStatus("WAIT_BLOCKED: obstacle intersects swept footprint.");
          this.renderer.drawRobotPose(this.trajectoryPlanner.poseAtDistance(trajectory, this.simulation.s), true);
          this.simulation.lastTs = ts;
          this.animationFrame = requestAnimationFrame(step);
          return;
        }

        if (this.simulation.lastTs === null) {
          this.simulation.lastTs = ts;
        }

        const dt = Math.min(0.08, Math.max(0, (ts - this.simulation.lastTs) / 1000));
        this.simulation.s = Math.min(
          trajectory[trajectory.length - 1].s,
          this.simulation.s + (speedMetersPerSec * dt)
        );
        while (
          this.simulation.index < trajectory.length - 2 &&
          trajectory[this.simulation.index + 1].s < this.simulation.s
        ) {
          this.simulation.index += 1;
        }

        this.simulation.lastTs = ts;
        const pose = this.trajectoryPlanner.poseAtDistance(trajectory, this.simulation.s);
        this.renderer.drawRobotPose(pose);
        this.renderer.setStatus(`Driving ${trajectory[this.simulation.index].edgeId}`);
        this.animationFrame = requestAnimationFrame(step);
      };

      this.animationFrame = requestAnimationFrame(step);
    }
  }

  class ViewportController {
    constructor(dom, geometry, isObstacleMode, onObstacleAdd) {
      this.dom = dom;
      this.geometry = geometry;
      this.isObstacleMode = isObstacleMode;
      this.onObstacleAdd = onObstacleAdd;
      this.scaleState = { zoom: 1, panX: 0, panY: 0 };
    }

    applyTransform() {
      this.dom.viewport.setAttribute(
        "transform",
        `translate(${this.scaleState.panX} ${this.scaleState.panY}) scale(${this.scaleState.zoom})`
      );
    }

    zoom(multiplier) {
      this.scaleState.zoom = Math.min(6, Math.max(0.5, this.scaleState.zoom * multiplier));
      this.applyTransform();
    }

    resetView() {
      this.scaleState.zoom = 1;
      this.scaleState.panX = 0;
      this.scaleState.panY = 0;
      this.applyTransform();
    }

    enableDrag() {
      let active = false;
      let lastX = 0;
      let lastY = 0;
      let downX = 0;
      let downY = 0;

      this.dom.mapSvg.addEventListener("pointerdown", (event) => {
        downX = event.clientX;
        downY = event.clientY;
        if (this.isObstacleMode()) {
          return;
        }
        active = true;
        lastX = event.clientX;
        lastY = event.clientY;
        this.dom.mapSvg.setPointerCapture(event.pointerId);
      });

      this.dom.mapSvg.addEventListener("pointermove", (event) => {
        if (!active) {
          return;
        }
        this.scaleState.panX += event.clientX - lastX;
        this.scaleState.panY += event.clientY - lastY;
        lastX = event.clientX;
        lastY = event.clientY;
        this.applyTransform();
      });

      const stop = (event) => {
        if (this.isObstacleMode() && event) {
          const moved = Math.hypot(event.clientX - downX, event.clientY - downY);
          if (moved < 6) {
            const world = this.geometry.eventToWorld(event, this.dom.viewport);
            if (world) {
              this.onObstacleAdd({ x: world.x, y: world.y, radius: 0.08 });
            }
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
      this.dom.mapSvg.addEventListener(
        "wheel",
        (event) => {
          event.preventDefault();
          this.zoom(event.deltaY < 0 ? 1.1 : 0.9);
        },
        { passive: false }
      );
    }
  }

  class RouteDemoApp {
    constructor(data) {
      this.data = data;
      this.dom = this.getDom();
      this.geometry = new GeometryService(data.map);
      this.routePlanner = new RoutePlanner(data.lms, data.edges, this.geometry);
      this.trajectoryPlanner = new TrajectoryPlanner(this.routePlanner, this.geometry);
      this.renderer = new MapRenderer(this.dom, this.routePlanner, this.geometry);
      this.behaviorSimulator = new BehaviorSimulator(this.renderer, this.trajectoryPlanner);
      this.currentRoute = null;
      this.currentTrajectory = [];
      this.currentPath = [];
      this.obstacleMode = false;
      this.obstacles = [];
      this.viewportController = new ViewportController(
        this.dom,
        this.geometry,
        () => this.obstacleMode,
        (obstacle) => this.addObstacle(obstacle)
      );
    }

    getDom() {
      return {
        mapTitle: document.getElementById("mapTitle"),
        mapSvg: document.getElementById("mapSvg"),
        viewport: document.getElementById("viewport"),
        mapImage: document.getElementById("mapImage"),
        graphLayer: document.getElementById("graphLayer"),
        pathLayer: document.getElementById("pathLayer"),
        lookaheadLayer: document.getElementById("lookaheadLayer"),
        obstacleLayer: document.getElementById("obstacleLayer"),
        pointLayer: document.getElementById("pointLayer"),
        robotLayer: document.getElementById("robotLayer"),
        routeLength: document.getElementById("routeLength"),
        routeList: document.getElementById("routeList"),
        statusText: document.getElementById("statusText"),
        startSelect: document.getElementById("startSelect"),
        goalSelect: document.getElementById("goalSelect"),
        planButton: document.getElementById("planButton"),
        driveButton: document.getElementById("driveButton"),
        stopButton: document.getElementById("stopButton"),
        obstacleModeButton: document.getElementById("obstacleModeButton"),
        clearObstaclesButton: document.getElementById("clearObstaclesButton"),
        robotWidthInput: document.getElementById("robotWidthInput"),
        robotLengthInput: document.getElementById("robotLengthInput"),
        lookaheadInput: document.getElementById("lookaheadInput"),
        speedInput: document.getElementById("speedInput"),
        zoomInButton: document.getElementById("zoomInButton"),
        zoomOutButton: document.getElementById("zoomOutButton"),
        resetViewButton: document.getElementById("resetViewButton"),
      };
    }

    init() {
      this.renderer.initMapFrame();
      this.renderer.populateSelectors();
      this.renderer.drawGraph();
      this.viewportController.enableDrag();
      this.attachEvents();
      this.planRoute();
    }

    attachEvents() {
      this.dom.planButton.addEventListener("click", () => this.planRoute());
      this.dom.driveButton.addEventListener("click", () => this.driveCurrentRoute());
      this.dom.stopButton.addEventListener("click", () => this.stopDriving());
      this.dom.obstacleModeButton.addEventListener("click", () => this.toggleObstacleMode());
      this.dom.clearObstaclesButton.addEventListener("click", () => this.clearObstacles());
      this.dom.robotWidthInput.addEventListener("change", () => this.redrawRobotAtStart());
      this.dom.robotLengthInput.addEventListener("change", () => this.redrawRobotAtStart());
      this.dom.lookaheadInput.addEventListener("change", () => this.renderer.clearLookahead());
      this.dom.speedInput.addEventListener("change", () => this.rebuildTrajectory());
      this.dom.startSelect.addEventListener("change", () => this.planRoute());
      this.dom.goalSelect.addEventListener("change", () => this.planRoute());
      this.dom.zoomInButton.addEventListener("click", () => this.viewportController.zoom(1.2));
      this.dom.zoomOutButton.addEventListener("click", () => this.viewportController.zoom(0.85));
      this.dom.resetViewButton.addEventListener("click", () => this.viewportController.resetView());
    }

    selectedSpeed() {
      return Math.max(0.02, Number(this.dom.speedInput.value) || 0.35);
    }

    planRoute() {
      this.behaviorSimulator.stop();
      const startName = this.dom.startSelect.value;
      const goalName = this.dom.goalSelect.value;
      this.currentRoute = this.routePlanner.findRoute(startName, goalName);
      this.currentPath = this.currentRoute ? this.currentRoute.nodes.slice() : [];
      this.currentTrajectory = this.trajectoryPlanner.buildTrajectory(this.currentRoute, this.selectedSpeed());
      this.renderer.drawPoints(startName, goalName);
      this.renderer.updateRouteInfo(this.currentRoute);
      this.renderer.drawRoute(this.currentTrajectory);
      this.renderer.drawRobotAt(this.currentPath);

      if (this.currentRoute) {
        this.renderer.setStatus(`Route planned: ${startName} -> ${goalName}`);
      } else {
        this.renderer.setStatus(`No route found from ${startName} to ${goalName}`);
      }
    }

    driveCurrentRoute() {
      if (!this.currentRoute) {
        this.renderer.setStatus("Plan a route first.");
        return;
      }
      this.currentTrajectory = this.trajectoryPlanner.buildTrajectory(this.currentRoute, this.selectedSpeed());
      this.renderer.drawRoute(this.currentTrajectory);
      this.behaviorSimulator.animateRoute(this.currentRoute, this.currentTrajectory, this.selectedSpeed());
    }

    stopDriving() {
      this.behaviorSimulator.stop();
      this.renderer.setStatus("Stopped.");
      if (this.currentTrajectory.length > 0) {
        this.renderer.drawRobotPose(this.currentTrajectory[0]);
      } else {
        this.redrawRobotAtStart();
      }
    }

    rebuildTrajectory() {
      if (!this.currentRoute) {
        return;
      }
      this.currentTrajectory = this.trajectoryPlanner.buildTrajectory(this.currentRoute, this.selectedSpeed());
      this.renderer.drawRoute(this.currentTrajectory);
    }

    redrawRobotAtStart() {
      this.renderer.drawRobotAt(this.currentPath.length ? this.currentPath : [this.dom.startSelect.value]);
    }

    toggleObstacleMode() {
      this.obstacleMode = !this.obstacleMode;
      this.dom.obstacleModeButton.textContent = `Add Obstacles: ${this.obstacleMode ? "On" : "Off"}`;
      this.renderer.setStatus(
        this.obstacleMode
          ? "Click the map to add lidar obstacle points."
          : "Obstacle edit mode off."
      );
    }

    addObstacle(obstacle) {
      this.obstacles.push(obstacle);
      this.behaviorSimulator.setObstacles(this.obstacles);
      this.renderer.setStatus("Obstacle point added. Drive will stop if footprint hits it.");
    }

    clearObstacles() {
      this.obstacles = [];
      this.behaviorSimulator.clearObstacles();
      this.renderer.setStatus("Obstacles cleared.");
    }
  }

  new RouteDemoApp(demoData).init();
}());
