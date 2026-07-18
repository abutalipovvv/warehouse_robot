export class FleetRobotModelEditor {
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
