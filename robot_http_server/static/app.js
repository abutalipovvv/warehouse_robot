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
        const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
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
        group.appendChild(element);
        const arrow = this.createDirectionArrow(edge);
        if (arrow) {
          group.appendChild(arrow);
        }
        this.dom.graphLayer.appendChild(group);
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
        group.addEventListener("click", (event) => {
          event.stopPropagation();
          this.onLandmarkClick(landmark.name);
        });

        const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
        circle.setAttribute("cx", String(point.x));
        circle.setAttribute("cy", String(point.y));
        circle.setAttribute("r", "5.5");
        group.appendChild(circle);

        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("x", String(point.x));
        label.setAttribute("y", String(point.y + 18));
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

    createDirectionArrow(edge) {
      let point = null;
      let tangent = null;

      if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
        const points = edge.control_points.map((item) => this.geometry.worldToPixel(item));
        point = this.cubicBezierPoint(points, 0.5);
        tangent = this.cubicBezierTangent(points, 0.5);
      } else {
        const startNode = this.graphModel.nodeByName.get(edge.from);
        const endNode = this.graphModel.nodeByName.get(edge.to);
        if (!startNode || !endNode) {
          return null;
        }
        const start = this.geometry.worldToPixel(startNode);
        const goal = this.geometry.worldToPixel(endNode);
        point = {
          x: (start.x + goal.x) / 2,
          y: (start.y + goal.y) / 2,
        };
        tangent = {
          x: goal.x - start.x,
          y: goal.y - start.y,
        };
      }

      if (!point || !tangent) {
        return null;
      }
      const length = Math.hypot(tangent.x, tangent.y);
      if (length <= 0.0001) {
        return null;
      }
      const ux = tangent.x / length;
      const uy = tangent.y / length;
      const px = -uy;
      const py = ux;
      const tip = {
        x: point.x + (ux * 8),
        y: point.y + (uy * 8),
      };
      const base = {
        x: point.x - (ux * 8),
        y: point.y - (uy * 8),
      };
      const left = {
        x: base.x + (px * 5),
        y: base.y + (py * 5),
      };
      const right = {
        x: base.x - (px * 5),
        y: base.y - (py * 5),
      };

      const polygon = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
      polygon.setAttribute(
        "points",
        `${tip.x},${tip.y} ${left.x},${left.y} ${right.x},${right.y}`
      );
      polygon.setAttribute("class", "graph-direction");
      return polygon;
    }

    cubicBezierPoint(points, t) {
      const [p0, p1, p2, p3] = points;
      const u = 1 - t;
      return {
        x: (u ** 3 * p0.x) + (3 * u * u * t * p1.x) + (3 * u * t * t * p2.x) + (t ** 3 * p3.x),
        y: (u ** 3 * p0.y) + (3 * u * u * t * p1.y) + (3 * u * t * t * p2.y) + (t ** 3 * p3.y),
      };
    }

    cubicBezierTangent(points, t) {
      const [p0, p1, p2, p3] = points;
      const u = 1 - t;
      return {
        x: (3 * u * u * (p1.x - p0.x)) + (6 * u * t * (p2.x - p1.x)) + (3 * t * t * (p3.x - p2.x)),
        y: (3 * u * u * (p1.y - p0.y)) + (6 * u * t * (p2.y - p1.y)) + (3 * t * t * (p3.y - p2.y)),
      };
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
      if (this.dom.mapPanel && this.dom.mapPanel.classList.contains("navigate-armed")) {
        return;
      }
      if (event.target && event.target.closest(".landmark")) {
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
      this.render();
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
      if (typeof this.onChange === "function") {
        this.onChange(this.getModel());
      }
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
          this.panDrag = { x: event.clientX, y: event.clientY };
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
        if (!event.ctrlKey && !event.metaKey) {
          return;
        }
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
      this.robotModelEditor = null;
    }

    getDom() {
      return {
        pageTitle: document.getElementById("pageTitle"),
        statusText: document.getElementById("statusText"),
        mapPanel: document.getElementById("mapPanel"),
        robotIdBadge: document.getElementById("robotIdBadge"),
        connectionBadge: document.getElementById("connectionBadge"),
        navigateButton: document.getElementById("navigateButton"),
        summaryConnectionText: document.getElementById("summaryConnectionText"),
        summaryRobotText: document.getElementById("summaryRobotText"),
        poseSummaryText: document.getElementById("poseSummaryText"),
        mapIdText: document.getElementById("mapIdText"),
        inspectorMessageText: document.getElementById("inspectorMessageText"),
        inspectorHintText: document.getElementById("inspectorHintText"),
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
        cancelRouteButton: document.getElementById("cancelRouteButton"),
        stopRobotButton: document.getElementById("stopRobotButton"),
        refreshStatusButton: document.getElementById("refreshStatusButton"),
        saveParamsButton: document.getElementById("saveParamsButton"),
        saveRobotParamsButton: document.getElementById("saveRobotParamsButton"),
        tabButtons: Array.from(document.querySelectorAll(".tab-button")),
        tabPages: Array.from(document.querySelectorAll(".tab-page")),
        modeText: document.getElementById("modeText"),
        localizationText: document.getElementById("localizationText"),
        edgeText: document.getElementById("edgeText"),
        progressText: document.getElementById("progressText"),
        velocitySummaryText: document.getElementById("velocitySummaryText"),
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
        robotEditorSvg: document.getElementById("robotEditorSvg"),
        robotEditorZoomInButton: document.getElementById("robotEditorZoomInButton"),
        robotEditorZoomOutButton: document.getElementById("robotEditorZoomOutButton"),
        robotEditorResetViewButton: document.getElementById("robotEditorResetViewButton"),
        resetModelButton: document.getElementById("resetModelButton"),
        footprintFields: document.getElementById("footprintFields"),
        tfFields: document.getElementById("tfFields"),
      };
    }

    async init() {
      this.dom.pageTitle.textContent = this.data.title || "Warehouse Robot Control";
      this.dom.robotIdBadge.textContent = this.data.robotId || "robot";
      if (this.dom.mapIdText) {
        this.dom.mapIdText.textContent = this.data.mapName || "-";
      }
      if (this.dom.summaryRobotText) {
        this.dom.summaryRobotText.textContent = this.data.robotId || "robot";
      }
      this.renderer.initMap();
      this.renderer.drawGraph();
      this.viewport.enable();
      this.populateGoalSelect();
      this.robotModelEditor = new RobotModelEditor(this.dom, () => {});
      this.robotModelEditor.init();
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
      this.dom.cancelRouteButton.addEventListener("click", () => this.cancelRoute());
      this.dom.stopRobotButton.addEventListener("click", () => this.stopRobot());
      this.dom.refreshStatusButton.addEventListener("click", () => this.fetchStatus());
      this.dom.saveParamsButton.addEventListener("click", () => this.saveParams());
      this.dom.saveRobotParamsButton.addEventListener("click", () => this.saveParams("Robot model saved."));
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
      if (this.suppressNextNavigateClick && !this.navigateMode) {
        return;
      }
      if (this.navigateMode) {
        this.suppressNextNavigateClick = true;
        window.setTimeout(() => {
          this.suppressNextNavigateClick = false;
        }, 250);
        this.startNavigation(name);
        return;
      }
      this.dom.goalSelect.value = name;
      this.setStatus(`Goal selected: ${name}. Press Navigate To Pose to send the robot.`);
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
      this.render();
      this.setActiveTab("robot");
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
      if (this.robotModelEditor && this.params.robot_model) {
        this.robotModelEditor.setModel(this.params.robot_model);
      }
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
      if (this.robotModelEditor) {
        params.robot_model = this.robotModelEditor.getModel();
      }
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

    async saveParams(successMessage = "Params saved.") {
      try {
        const result = await this.postJson("/api/params", this.collectParams());
        if (result && result.params) {
          this.applyParams(result.params);
        }
        this.setStatus(successMessage);
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
          this.setActiveTab("robot");
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
        this.setActiveTab("robot");
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
        this.dom.connectionBadge.className = "status-pill offline";
        if (this.dom.summaryConnectionText) {
          this.dom.summaryConnectionText.textContent = "offline";
          this.dom.summaryConnectionText.className = "mini-badge offline";
        }
      } finally {
        this.statusRequestPending = false;
      }
    }

    render() {
      const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : null;
      const route = this.currentStatus && this.currentStatus.route ? this.currentStatus.route : this.currentRoute;
      const pose = robot && robot.pose ? robot.pose : null;
      const connected = Boolean(robot && robot.connected);
      const connectionClass = connected ? "online" : "offline";
      const connectionText = connected ? "online" : "offline";

      this.dom.connectionBadge.textContent = connectionText;
      this.dom.connectionBadge.className = `status-pill ${connectionClass}`;
      if (this.dom.summaryConnectionText) {
        this.dom.summaryConnectionText.textContent = connectionText;
        this.dom.summaryConnectionText.className = `mini-badge ${connectionClass}`;
      }
      if (robot && robot.robotId) {
        this.dom.robotIdBadge.textContent = robot.robotId;
      }
      if (this.dom.summaryRobotText) {
        this.dom.summaryRobotText.textContent = robot && robot.robotId ? robot.robotId : (this.data.robotId || "robot");
      }
      if (this.dom.mapIdText) {
        this.dom.mapIdText.textContent = robot && robot.mapId ? robot.mapId : (this.data.mapName || "-");
      }
      this.dom.modeText.textContent = robot ? robot.state : "-";
      this.dom.localizationText.textContent = robot
        ? `${robot.localizationOk ? "ok" : "waiting"} (${Number(robot.localizationAgeSec || 0).toFixed(2)} s)`
        : "-";
      this.dom.edgeText.textContent = robot && robot.currentEdgeId ? robot.currentEdgeId : "-";
      this.dom.progressText.textContent = robot ? `${Math.round((Number(robot.routeProgress || 0) * 100))}%` : "0%";

      this.dom.mapStateText.textContent = robot && robot.state ? robot.state : "-";
      this.dom.mapNearestText.textContent = robot && robot.nearestLm ? robot.nearestLm : "-";
      this.dom.mapTargetText.textContent = robot && robot.targetLm
        ? robot.targetLm
        : (this.dom.goalSelect.value || "-");

      const poseText = pose
        ? `x: ${Number(pose.x).toFixed(3)}, y: ${Number(pose.y).toFixed(3)}, yaw: ${Number(pose.yaw).toFixed(3)}`
        : "x: -, y: -, yaw: -";
      if (this.dom.poseSummaryText) {
        this.dom.poseSummaryText.textContent = poseText;
      }

      const velocityText = robot && robot.velocity
        ? `v: ${Number(robot.velocity.linear || 0).toFixed(3)}, w: ${Number(robot.velocity.angular || 0).toFixed(3)}`
        : "v: -, w: -";
      if (this.dom.velocitySummaryText) {
        this.dom.velocitySummaryText.textContent = velocityText;
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

      const inspector = this.describeInspector(robot, route);
      if (this.dom.inspectorMessageText) {
        this.dom.inspectorMessageText.textContent = inspector.message;
      }
      if (this.dom.inspectorHintText) {
        this.dom.inspectorHintText.textContent = inspector.hint;
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
        row.className = `event-log-row ${String(event.level || "info").toLowerCase()}`;
        const stamp = event.stamp ? new Date(event.stamp * 1000).toLocaleTimeString([], { hour12: false }) : "--:--:--";
        row.textContent = `${stamp} ${event.level || "info"} ${event.message || ""}`;
        this.dom.eventLog.appendChild(row);
      }
    }

    describeInspector(robot, route) {
      if (!robot) {
        return {
          message: "No robot status yet.",
          hint: "Check that robot_http_api is running and ROS 2 topics /amcl_pose, /odom, and /tf are available.",
        };
      }

      const raw = String(robot.message || "").trim();
      if (!robot.connected) {
        return {
          message: raw || "Robot bridge is offline.",
          hint: "The browser cannot reach robot_http_api. Check serve_robot.py and the selected HTTP port.",
        };
      }

      if (!robot.localizationOk || robot.state === "LOCALIZING") {
        return {
          message: raw || "Localization is not ready yet.",
          hint: "AMCL or TF pose is stale. Check /scan, /amcl_pose, initial pose, and map alignment.",
        };
      }

      if (robot.state === "ERROR") {
        if (raw.toLowerCase().includes("timeout")) {
          return {
            message: raw,
            hint: "The pose stream became stale. Check AMCL updates, TF freshness, and whether the robot is leaving the known map area.",
          };
        }
        return {
          message: raw || "Robot entered ERROR state.",
          hint: "Open the Events tab and inspect the latest warning or error rows for the exact failing operation.",
        };
      }

      if (robot.state === "EXECUTING_ROUTE") {
        const goal = robot.targetLm || (route && route.goalLm) || "target";
        return {
          message: raw || `Executing route to ${goal}.`,
          hint: "The controller is trying to stay on the graph trajectory. Manual W/A/S/D will cancel the active route.",
        };
      }

      if (robot.state === "MANUAL") {
        return {
          message: raw || "Manual control is active.",
          hint: "Use W/A/S/D or the map pad. Release all keys to stop sending /cmd_vel.",
        };
      }

      if (route && Array.isArray(route.nodes) && route.nodes.length > 1) {
        return {
          message: raw || `Route ready: ${route.nodes[0]} -> ${route.goalLm || route.nodes[route.nodes.length - 1]}.`,
          hint: "Press Navigate To Pose and click a landmark, or keep this planned route for the next execution.",
        };
      }

      return {
        message: raw || "Robot is ready.",
        hint: "Press Navigate To Pose and click a landmark on the map to start motion along the graph route.",
      };
    }

    setStatus(text) {
      this.dom.statusText.textContent = text;
    }

    toggleNavigateMode() {
      this.navigateMode = !this.navigateMode;
      this.setActiveTab("robot");
      this.syncModeButtons();
      this.render();
      this.setStatus(this.navigateMode ? "Navigate armed: select a landmark." : "Navigate canceled.");
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
      this.dom.navigateButton.textContent = this.navigateMode ? "Select Landmark" : "Navigate To Pose";
      if (this.dom.mapPanel) {
        this.dom.mapPanel.classList.toggle("navigate-armed", this.navigateMode);
      }
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
        this.setStatus("Navigate armed: click closer to a landmark.");
        return false;
      }
      this.startNavigation(nearest.landmark.name);
      return true;
    }

    async startNavigation(goalLm) {
      this.clearNavigateMode(false);
      this.render();
      this.releaseManualControl();
      this.dom.goalSelect.value = goalLm;
      const robot = this.currentStatus && this.currentStatus.robot ? this.currentStatus.robot : null;
      const payload = { goalLm };
      if (robot && robot.pose) {
        payload.startPose = {
          x: Number(robot.pose.x || 0),
          y: Number(robot.pose.y || 0),
          yaw: Number(robot.pose.yaw || 0),
        };
      }
      if (robot && robot.nearestLm) {
        payload.startLm = robot.nearestLm;
      }
      try {
        const result = await this.postJson("/api/robot/route/execute", payload);
        if (result && result.route) {
          this.currentRoute = result.route;
        }
        this.setActiveTab("robot");
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
        throw new Error(await this.extractErrorMessage(response));
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
        throw new Error(await this.extractErrorMessage(response));
      }
      return response.json();
    }

    async extractErrorMessage(response) {
      const fallback = `HTTP ${response.status}`;
      let text = "";
      try {
        text = await response.text();
      } catch (_) {
        return fallback;
      }
      if (!text) {
        return fallback;
      }
      try {
        const data = JSON.parse(text);
        if (data && typeof data === "object") {
          return String(data.error || data.message || fallback);
        }
      } catch (_) {
        // HTML/plain text error path from SimpleHTTPRequestHandler.
      }
      const titleMatch = text.match(/<p[^>]*>Message:\s*([^<]+)<\/p>/i);
      if (titleMatch && titleMatch[1]) {
        return `${fallback}: ${titleMatch[1].trim()}`;
      }
      const plain = text.replace(/<[^>]+>/g, " ").replace(/\s+/g, " ").trim();
      if (plain) {
        return plain.length > 180 ? `${fallback}: ${plain.slice(0, 177)}...` : `${fallback}: ${plain}`;
      }
      return fallback;
    }
  }

  new RobotApp(data).init();
}());
