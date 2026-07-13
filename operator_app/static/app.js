const ROBOT_PARAM_SCHEMA = [
  {
    group: "Nav2",
    section: "AMCL",
    path: "nav2.amcl.update_min_d",
    label: "AMCL update distance",
    description: "Minimum linear movement before AMCL updates localization.",
    type: "number",
    default: 0.01,
    min: 0,
    step: 0.001,
    unit: "m",
  },
  {
    group: "Nav2",
    section: "AMCL",
    path: "nav2.amcl.update_min_a",
    label: "AMCL update angle",
    description: "Minimum angular movement before AMCL updates localization.",
    type: "number",
    default: 0.01,
    min: 0,
    step: 0.001,
    unit: "rad",
  },
  {
    group: "Nav2",
    section: "AMCL",
    path: "nav2.amcl.transform_tolerance",
    label: "AMCL transform tolerance",
    description: "Allowed TF timing tolerance for AMCL map to odom transform.",
    type: "number",
    default: 0.2,
    min: 0,
    step: 0.01,
    unit: "s",
  },
  {
    group: "Nav2",
    section: "AMCL",
    path: "nav2.amcl.min_particles",
    label: "AMCL min particles",
    description: "Lower particle count used by AMCL localization.",
    type: "integer",
    default: 1000,
    min: 100,
    step: 50,
  },
  {
    group: "Nav2",
    section: "AMCL",
    path: "nav2.amcl.max_particles",
    label: "AMCL max particles",
    description: "Upper particle count used by AMCL localization.",
    type: "integer",
    default: 2500,
    min: 100,
    step: 50,
  },
  {
    group: "Nav2",
    section: "Controller",
    path: "nav2.controller_server.controller_frequency",
    label: "Controller frequency",
    description: "How often Nav2 computes velocity commands.",
    type: "number",
    default: 20,
    min: 1,
    step: 1,
    unit: "Hz",
  },
  {
    group: "Nav2",
    section: "Controller",
    path: "nav2.controller_server.xy_goal_tolerance",
    label: "Goal position tolerance",
    description: "Accepted XY distance to a Nav2 goal.",
    type: "number",
    default: 0.05,
    min: 0,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Nav2",
    section: "Controller",
    path: "nav2.controller_server.yaw_goal_tolerance",
    label: "Goal yaw tolerance",
    description: "Accepted heading error at a Nav2 goal.",
    type: "number",
    default: 0.05,
    min: 0,
    step: 0.01,
    unit: "rad",
  },
  {
    group: "Nav2",
    section: "Controller",
    path: "nav2.controller_server.follow_path.vx_max",
    label: "MPPI max forward speed",
    description: "Maximum forward velocity sampled by the Nav2 MPPI controller.",
    type: "number",
    default: 0.5,
    min: 0,
    step: 0.05,
    unit: "m/s",
  },
  {
    group: "Nav2",
    section: "Controller",
    path: "nav2.controller_server.follow_path.vx_min",
    label: "MPPI max reverse speed",
    description: "Minimum X velocity sampled by MPPI. Negative values allow reverse motion.",
    type: "number",
    default: -0.35,
    step: 0.05,
    unit: "m/s",
  },
  {
    group: "Nav2",
    section: "Controller",
    path: "nav2.controller_server.follow_path.wz_max",
    label: "MPPI max angular speed",
    description: "Maximum angular velocity sampled by the Nav2 MPPI controller.",
    type: "number",
    default: 1.9,
    min: 0,
    step: 0.05,
    unit: "rad/s",
  },
  {
    group: "Nav2",
    section: "Costmaps",
    path: "nav2.local_costmap.robot_radius",
    label: "Local robot radius",
    description: "Robot radius used by the local Nav2 costmap.",
    type: "number",
    default: 0.22,
    min: 0.01,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Nav2",
    section: "Costmaps",
    path: "nav2.local_costmap.inflation_radius",
    label: "Local inflation radius",
    description: "Obstacle inflation radius in the local costmap.",
    type: "number",
    default: 0.7,
    min: 0,
    step: 0.05,
    unit: "m",
  },
  {
    group: "Nav2",
    section: "Costmaps",
    path: "nav2.local_costmap.cost_scaling_factor",
    label: "Local cost scaling",
    description: "How quickly inflated obstacle costs decay in the local costmap.",
    type: "number",
    default: 3,
    min: 0,
    step: 0.1,
  },
  {
    group: "Nav2",
    section: "Costmaps",
    path: "nav2.global_costmap.robot_radius",
    label: "Global robot radius",
    description: "Robot radius used by the global Nav2 costmap.",
    type: "number",
    default: 0.22,
    min: 0.01,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Nav2",
    section: "Costmaps",
    path: "nav2.global_costmap.inflation_radius",
    label: "Global inflation radius",
    description: "Obstacle inflation radius in the global costmap.",
    type: "number",
    default: 0.7,
    min: 0,
    step: 0.05,
    unit: "m",
  },
  {
    group: "Nav2",
    section: "Velocity smoother",
    path: "nav2.velocity_smoother.max_velocity_x",
    label: "Smoothed max linear speed",
    description: "Maximum X velocity allowed by the Nav2 velocity smoother.",
    type: "number",
    default: 0.5,
    min: 0,
    step: 0.05,
    unit: "m/s",
  },
  {
    group: "Nav2",
    section: "Velocity smoother",
    path: "nav2.velocity_smoother.max_velocity_theta",
    label: "Smoothed max angular speed",
    description: "Maximum angular velocity allowed by the Nav2 velocity smoother.",
    type: "number",
    default: 2,
    min: 0,
    step: 0.05,
    unit: "rad/s",
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.route_speed",
    label: "Route speed",
    description: "Linear speed used by the LM route executor.",
    type: "number",
    default: 0.35,
    min: 0.02,
    step: 0.05,
    unit: "m/s",
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.footprint_lookahead",
    label: "Footprint lookahead",
    description: "Distance checked ahead of the robot footprint for route collisions.",
    type: "number",
    default: 0.8,
    min: 0,
    step: 0.05,
    unit: "m",
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.collision_margin",
    label: "Collision margin",
    description: "Extra clearance around the robot footprint.",
    type: "number",
    default: 0.04,
    min: 0,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.stop_distance",
    label: "Stop distance",
    description: "Distance from the target at which route execution can stop.",
    type: "number",
    default: 0.4,
    min: 0,
    step: 0.05,
    unit: "m",
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.angular_gain",
    label: "Angular gain",
    description: "Heading correction gain used during LM route following.",
    type: "number",
    default: 2.2,
    min: 0,
    step: 0.1,
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.max_angular_speed",
    label: "Max angular speed",
    description: "Maximum angular speed commanded by the route executor.",
    type: "number",
    default: 0.9,
    min: 0,
    step: 0.05,
    unit: "rad/s",
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.rotate_in_place_angle_deg",
    label: "Rotate-in-place angle",
    description: "Heading error that triggers in-place rotation behavior.",
    type: "number",
    default: 32,
    min: 0,
    max: 180,
    step: 1,
    unit: "deg",
  },
  {
    group: "Route Planner",
    section: "Execution",
    path: "navigation.curve_speed_limit",
    label: "Curve speed limit",
    description: "Speed limit while following curved graph edges.",
    type: "number",
    default: 0.25,
    min: 0,
    step: 0.01,
    unit: "m/s",
  },
  {
    group: "Route Planner",
    section: "Planner",
    path: "planner.nearest_lm_tolerance",
    label: "Nearest LM tolerance",
    description: "Distance threshold for accepting that the robot is already at an LM.",
    type: "number",
    default: 0.05,
    min: 0,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Route Planner",
    section: "Planner",
    path: "planner.trajectory_sample_distance",
    label: "Trajectory sample distance",
    description: "Spacing between generated trajectory samples.",
    type: "number",
    default: 0.05,
    min: 0.01,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Route Planner",
    section: "Planner",
    path: "planner.on_route_tolerance",
    label: "On-route tolerance",
    description: "Allowed lateral distance when reconnecting current pose to a route.",
    type: "number",
    default: 0.12,
    min: 0,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Robot",
    section: "Model",
    path: "robot_model.source",
    label: "Model source",
    description: "Where the robot footprint is derived from.",
    type: "select",
    default: "nav2",
    options: [
      ["nav2", "Nav2"],
      ["manual", "Manual"],
    ],
  },
  {
    group: "Robot",
    section: "Model",
    path: "robot_model.radius",
    label: "Robot radius",
    description: "Fallback circular radius used for planning and footprint generation.",
    type: "number",
    default: 0.22,
    min: 0.01,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Robot",
    section: "Model",
    path: "robot_model.footprint_segments",
    label: "Footprint segments",
    description: "Number of points used when generating a circular footprint.",
    type: "integer",
    default: 16,
    min: 8,
    step: 1,
  },
  {
    group: "Robot",
    section: "Localization",
    path: "localization.localization_timeout",
    label: "Localization timeout",
    description: "Maximum age of localization before route execution treats it as stale.",
    type: "number",
    default: 0.5,
    min: 0,
    step: 0.05,
    unit: "s",
  },
  {
    group: "Robot",
    section: "Localization",
    path: "localization.allowed_lateral_error",
    label: "Allowed lateral error",
    description: "Maximum route lateral error before status reports degraded tracking.",
    type: "number",
    default: 0.02,
    min: 0,
    step: 0.01,
    unit: "m",
  },
  {
    group: "Robot",
    section: "Localization",
    path: "localization.allowed_yaw_error_deg",
    label: "Allowed yaw error",
    description: "Maximum route yaw error before status reports degraded tracking.",
    type: "number",
    default: 1,
    min: 0,
    step: 0.5,
    unit: "deg",
  },
  {
    group: "Robot",
    section: "Localization",
    path: "localization.accept_stale_pose_when_stationary",
    label: "Accept stale stationary pose",
    description: "Keep using the last pose if the robot is stationary and AMCL is temporarily stale.",
    type: "boolean",
    default: true,
  },
  {
    group: "Robot",
    section: "Manual Control",
    path: "manual.linear_speed",
    label: "Manual linear speed",
    description: "Linear speed used by manual teleop buttons.",
    type: "number",
    default: 0.25,
    min: 0,
    step: 0.05,
    unit: "m/s",
  },
  {
    group: "Robot",
    section: "Manual Control",
    path: "manual.angular_speed",
    label: "Manual angular speed",
    description: "Angular speed used by manual teleop buttons.",
    type: "number",
    default: 0.9,
    min: 0,
    step: 0.05,
    unit: "rad/s",
  },
  {
    group: "Robot",
    section: "Manual Control",
    path: "manual.prediction_time",
    label: "Manual prediction time",
    description: "Lookahead time used to draw the manual-control projected pose.",
    type: "number",
    default: 1,
    min: 0.1,
    step: 0.1,
    unit: "s",
  },
];

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
    this.fleetManagerSimId = "__fleet_manager_sim__";
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
    this.robotPingRefreshPending = false;
    this.navigateMode = false;
    this.relocateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.fleetQueue = [];
    this.fleetQueueSequence = 0;
    this.fleetBenchmarkRunId = 0;
    this.lastFleetPlanDebug = null;
    this.fleetSimMapsLoadedAt = 0;
    this.fleetSimMapsLoading = false;
    this.selectedFleetOrderId = "";
    this.fleetParams = null;
    this.fleetParamsLoaded = false;
    this.robotParams = null;
    this.robotParamsRobotId = "";
    this.robotParamsLoaded = false;
    this.fleetNameEdited = false;
    this.fleetTickPending = false;
    this.mapViewMode = window.localStorage.getItem("operator:mapViewMode") || "2d";
    this.scene3dModulePromise = null;
    this.scene3d = null;
    this.scene3dStaticKey = "";
    this.scene3dPayload = null;
    this.scene3dHoverLmName = "";
    this.scene3dLoadPending = false;
    this.scene3dRenderQueued = false;
    this.fleetStatusSocket = null;
    this.fleetStatusManagerId = "";
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
    this.robotStatusReceivedAt = 0;
    this.robotStatusFreshTimeoutMs = 2200;
    this.robotStatusRobotId = "";
    this.scanSocket = null;
    this.scanRobotId = "";
    this.scanEnabled = false;
    this.latestScanFrame = null;
    this.slamSocket = null;
    this.slamRobotId = "";
    this.slamActive = false;
    this.slamState = null;
    this.slamMapPayload = null;
    this.slamMapFrame = null;
    this.slamDefaults = null;
    this.teleopSocket = null;
    this.teleopRobotId = "";
    this.fleetStatusReceivedAt = 0;
    this.fleetStatusObjectRef = null;
    this.fleetAnimationFrame = null;
    this.fleetAnimationLastAt = 0;
    this.fleetRouteRenderLastAt = 0;
    this.fleetVisualClocks = new Map();
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
    this.relocationDrag = null;
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
    this.homePage = document.getElementById("homePage");
    this.homeRobotGrid = document.getElementById("homeRobotGrid");
    this.homeRobotCountText = document.getElementById("homeRobotCountText");
    this.homeRefreshButton = document.getElementById("homeRefreshButton");
    this.homeAddRobotButton = document.getElementById("homeAddRobotButton");
    this.sidebarDrawer = document.getElementById("sidebarDrawer");
    this.sidebarBackdrop = document.getElementById("sidebarBackdrop");
    this.globalHomeButton = document.getElementById("globalHomeButton");
    this.homeButton = document.getElementById("homeButton");
    this.paramsNavButton = document.getElementById("paramsNavButton");
    this.mapEditorNavButton = document.getElementById("mapEditorNavButton");
    this.robotModelNavButton = document.getElementById("robotModelNavButton");
    this.openSidebarButton = document.getElementById("openSidebarButton");
    this.closeSidebarButton = document.getElementById("closeSidebarButton");
    this.emptyState = document.getElementById("emptyState");
    this.robotView = document.getElementById("robotView");
    this.robotWorkspaceTitle = document.getElementById("robotWorkspaceTitle");
    this.robotActiveMapText = document.getElementById("robotActiveMapText");
    this.operatorActiveMapText = document.getElementById("operatorActiveMapText");
    this.robotStateText = document.getElementById("robotStateText");
    this.robotControlText = document.getElementById("robotControlText");
    this.nearestLmText = document.getElementById("nearestLmText");
    this.driveActionGroup = document.getElementById("driveActionGroup");
    this.localizationActionGroup = document.getElementById("localizationActionGroup");
    this.navigationActionGroup = document.getElementById("navigationActionGroup");
    this.visualActionGroup = document.getElementById("visualActionGroup");
    this.mapActionGroup = document.getElementById("mapActionGroup");
    this.slamActionGroup = document.getElementById("slamActionGroup");
    this.navigateRobotButton = document.getElementById("navigateRobotButton");
    this.takeControlButton = document.getElementById("takeControlButton");
    this.releaseControlButton = document.getElementById("releaseControlButton");
    this.relocateRobotButton = document.getElementById("relocateRobotButton");
    this.pauseRouteButton = document.getElementById("pauseRouteButton");
    this.resumeRouteButton = document.getElementById("resumeRouteButton");
    this.cancelRouteButton = document.getElementById("cancelRouteButton");
    this.stopRobotButton = document.getElementById("stopRobotButton");
    this.scanToggleButton = document.getElementById("scanToggleButton");
    this.controlPullMapButton = document.getElementById("controlPullMapButton");
    this.controlPushMapButton = document.getElementById("controlPushMapButton");
    this.controlLoadMapButton = document.getElementById("controlLoadMapButton");
    this.startSlamButton = document.getElementById("startSlamButton");
    this.doneSlamButton = document.getElementById("doneSlamButton");
    this.cancelSlamButton = document.getElementById("cancelSlamButton");
    this.mapSyncStatus = document.getElementById("mapSyncStatus");
    this.operatorConsole = document.getElementById("operatorConsole");
    this.fleetControlPanel = document.getElementById("fleetControlPanel");
    this.robotParamsPanel = document.getElementById("robotParamsPanel");
    this.robotParamsSummary = document.getElementById("robotParamsSummary");
    this.robotParamsTable = document.getElementById("robotParamsTable");
    this.robotParamsJsonInput = document.getElementById("robotParamsJsonInput");
    this.robotReloadParamsButton = document.getElementById("robotReloadParamsButton");
    this.robotFormatParamsButton = document.getElementById("robotFormatParamsButton");
    this.robotDefaultsParamsButton = document.getElementById("robotDefaultsParamsButton");
    this.robotSaveParamsButton = document.getElementById("robotSaveParamsButton");
    this.robotModelPanel = document.getElementById("robotModelPanel");
    this.fleetRobotNameLabel = document.getElementById("fleetRobotNameLabel");
    this.fleetRobotNameInput = document.getElementById("fleetRobotNameInput");
    this.fleetSpawnLmLabel = document.getElementById("fleetSpawnLmLabel");
    this.fleetSpawnLmLabelText = document.getElementById("fleetSpawnLmLabelText");
    this.fleetSpawnLmSelect = document.getElementById("fleetSpawnLmSelect");
    this.fleetRobotApiLabel = document.getElementById("fleetRobotApiLabel");
    this.fleetRobotApiInput = document.getElementById("fleetRobotApiInput");
    this.fleetAddRobotButton = document.getElementById("fleetAddRobotButton");
    this.fleetPlaceRobotButton = document.getElementById("fleetPlaceRobotButton");
    this.fleetSimTools = document.getElementById("fleetSimTools");
    this.fleetSimMapSelect = document.getElementById("fleetSimMapSelect");
    this.fleetSimLoadMapButton = document.getElementById("fleetSimLoadMapButton");
    this.fleetBenchmarkButtons = Array.from(document.querySelectorAll("[data-fleet-benchmark-count]"));
    this.fleetBenchmarkPlanButton = document.getElementById("fleetBenchmarkPlanButton");
    this.fleetBenchmarkHorizonInput = document.getElementById("fleetBenchmarkHorizonInput");
    this.fleetBenchmarkIntervalInput = document.getElementById("fleetBenchmarkIntervalInput");
    this.fleetBenchmarkClearButton = document.getElementById("fleetBenchmarkClearButton");
    this.fleetBenchmarkRefreshMapsButton = document.getElementById("fleetBenchmarkRefreshMapsButton");
    this.fleetBenchmarkOpenLoadButton = document.getElementById("fleetBenchmarkOpenLoadButton");
    this.fleetBenchmarkStatus = document.getElementById("fleetBenchmarkStatus");
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
    this.fleetRouteAccelerationInput = document.getElementById("fleetRouteAccelerationInput");
    this.fleetRotateInput = document.getElementById("fleetRotateInput");
    this.fleetTurnSpeedInput = document.getElementById("fleetTurnSpeedInput");
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
    this.operatorScene3d = document.getElementById("operatorScene3d");
    this.operatorMap2dButton = document.getElementById("operatorMap2dButton");
    this.operatorMap3dButton = document.getElementById("operatorMap3dButton");
    this.operatorViewport = document.getElementById("operatorViewport");
    this.operatorMapImage = document.getElementById("operatorMapImage");
    this.operatorObstacleLayer = document.getElementById("operatorObstacleLayer");
    this.operatorGraphLayer = document.getElementById("operatorGraphLayer");
    this.operatorRouteLayer = document.getElementById("operatorRouteLayer");
    this.operatorLookaheadLayer = document.getElementById("operatorLookaheadLayer");
    this.operatorLandmarkLayer = document.getElementById("operatorLandmarkLayer");
    this.operatorEditorLayer = document.getElementById("operatorEditorLayer");
    this.operatorScanLayer = document.getElementById("operatorScanLayer");
    this.operatorRobotLayer = document.getElementById("operatorRobotLayer");
    this.operatorRelocateLayer = document.getElementById("operatorRelocateLayer");
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
    this.slamDialog = document.getElementById("slamDialog");
    this.closeSlamDialogButton = document.getElementById("closeSlamDialogButton");
    this.cancelSlamDialogButton = document.getElementById("cancelSlamDialogButton");
    this.confirmStartSlamButton = document.getElementById("confirmStartSlamButton");
    this.slamParamsInput = document.getElementById("slamParamsInput");
    this.slamDialogStatus = document.getElementById("slamDialogStatus");
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
      this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    }, 12000);
    window.setInterval(() => {
      this.refreshRobotPings().catch(() => {});
    }, 2000);
    window.setInterval(() => {
      this.fetchSelectedRobotStatus(true).catch(() => {});
    }, 800);
    window.setInterval(() => {
      this.expireRobotStatusIfStale();
    }, 500);
    window.setInterval(() => {
      this.tickFleetIfSelected().catch(() => {});
    }, 80);
    window.setInterval(() => {
      this.sendTeleopIfNeeded().catch(() => {});
    }, 120);
  }

  bindEvents() {
    this.globalHomeButton?.addEventListener("click", async () => this.navigateGlobalHomePage());
    this.homeButton?.addEventListener("click", async () => this.navigateHomePage());
    this.paramsNavButton?.addEventListener("click", async () => this.navigateParamsPage());
    this.mapEditorNavButton?.addEventListener("click", async () => this.navigateMapEditorPage());
    this.robotModelNavButton?.addEventListener("click", async () => this.navigateRobotModelPage());
    this.openSidebarButton?.addEventListener("click", () => this.openSidebar());
    this.closeSidebarButton?.addEventListener("click", () => this.closeSidebar());
    this.sidebarBackdrop?.addEventListener("click", () => this.closeSidebar());
    this.refreshButton?.addEventListener("click", () => this.refreshRobots());
    this.addRobotButton?.addEventListener("click", () => this.openAddRobotDialog());
    this.homeRefreshButton?.addEventListener("click", () => this.refreshRobots());
    this.homeAddRobotButton?.addEventListener("click", () => this.openAddRobotDialog());
    this.navigateRobotButton.addEventListener("click", () => this.toggleNavigateMode());
    this.takeControlButton?.addEventListener("click", () => this.acquireRobotControl(true, true));
    this.releaseControlButton?.addEventListener("click", () => this.releaseRobotControl());
    this.relocateRobotButton?.addEventListener("click", () => this.toggleRelocateMode());
    this.pauseRouteButton?.addEventListener("click", () => this.pauseRobotRoute());
    this.resumeRouteButton?.addEventListener("click", () => this.resumeRobotRoute());
    this.cancelRouteButton.addEventListener("click", () => this.cancelRoute());
    this.stopRobotButton.addEventListener("click", () => this.stopRobot());
    this.scanToggleButton?.addEventListener("click", () => {
      this.toggleScanStream().catch((error) => {
        if (this.robotMessageText) {
          this.robotMessageText.textContent = `Scan failed: ${error.message || error}`;
        }
        this.closeScanStream();
      });
    });
    this.controlPullMapButton.addEventListener("click", () => this.handlePullMap());
    this.controlPushMapButton.addEventListener("click", () => this.handlePushMap());
    this.controlLoadMapButton.addEventListener("click", () => this.handleLoadMap());
    this.operatorMap2dButton?.addEventListener("click", () => this.setMapViewMode("2d"));
    this.operatorMap3dButton?.addEventListener("click", () => this.setMapViewMode("3d"));
    this.startSlamButton?.addEventListener("click", () => this.openSlamDialog());
    this.doneSlamButton?.addEventListener("click", () => this.finishSlam());
    this.cancelSlamButton?.addEventListener("click", () => this.cancelSlam());
    this.fleetRobotNameInput.addEventListener("input", () => {
      this.fleetNameEdited = true;
    });
    this.fleetAddRobotButton.addEventListener("click", () => this.handleFleetAddRobot());
    this.fleetPlaceRobotButton?.addEventListener("click", () => this.toggleFleetSpawnMode());
    this.fleetSimLoadMapButton?.addEventListener("click", () => this.handleFleetSimLoadMap());
    this.fleetSimMapSelect?.addEventListener("change", () => this.syncFleetSimMapLoadButton());
    this.fleetBenchmarkButtons.forEach((button) => {
      button.addEventListener("click", () => this.runFleetBenchmark(Number(button.dataset.fleetBenchmarkCount || 20)));
    });
    this.fleetBenchmarkPlanButton?.addEventListener("click", () => this.planFleetBenchmarkRobots());
    this.fleetBenchmarkClearButton?.addEventListener("click", () => this.runFleetBenchmark(0));
    this.fleetBenchmarkRefreshMapsButton?.addEventListener("click", () => this.refreshFleetSimMapSelect({ force: true }));
    this.fleetBenchmarkOpenLoadButton?.addEventListener("click", () => this.handleLoadMap());
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
    this.robotDefaultsParamsButton.addEventListener("click", () => this.resetRobotParamsToDefaults());
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
    this.closeSlamDialogButton?.addEventListener("click", () => this.slamDialog.close());
    this.cancelSlamDialogButton?.addEventListener("click", () => this.slamDialog.close());
    this.confirmStartSlamButton?.addEventListener("click", () => this.startSlamFromDialog());

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
    window.addEventListener("beforeunload", () => {
      this.closeScanStream();
      this.closeSlamStream();
      this.closeTeleopSocket(true);
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
        if (this.isParamsPage() && !this.isFleetManager()) {
          this.renderRobotParamsTable();
          this.syncRobotParamsJson(true);
        }
      }
    );
    this.fleetModelEditor.init();
  }

  pageForPath(pathname) {
    const path = String(pathname || "/").replace(/\/+$/, "") || "/";
    if (path === "/" || path === "/home") {
      return "robots";
    }
    if (path === "/robot" || path === "/robot-home") {
      return "fleet";
    }
    if (path === "/params") {
      return "params";
    }
    if (path === "/robot_model" || path === "/robot-model") {
      return "model";
    }
    if (path === "/map_editor" || path === "/map-editor") {
      return "map";
    }
    return "robots";
  }

  pathForFleetPage(tabName) {
    const tab = ["robots", "fleet", "params", "model", "map"].includes(tabName) ? tabName : "robots";
    return {
      robots: "/home",
      fleet: "/robot",
      params: "/params",
      model: "/robot_model",
      map: "/map_editor",
    }[tab];
  }

  async navigateGlobalHomePage(options = {}) {
    const path = this.pathForFleetPage("robots");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "robots" }, "", path);
    }
    this.setFleetTab("robots");
    this.closeRobotStatusStream();
    this.closeFleetStatusStream();
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
    this.stopFleetAnimationLoop();
    this.renderSelectedRobot();
  }

  async navigateHomePage(options = {}) {
    const path = this.pathForFleetPage("fleet");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "fleet" }, "", path);
    }
    this.setFleetTab("fleet");
    if (!this.selectedRobot() && this.robots.length) {
      const robot = this.robots.find((item) => !item.system) || this.robots[0];
      this.selectedRobotId = robot.id;
      window.localStorage.setItem("operator:selectedRobotId", robot.id);
    }
    await this.refreshRobotMapState({ quiet: true });
    await this.fetchSelectedRobotStatus(true);
    await this.refreshSelectedSlamState({ quiet: true });
    this.renderSelectedRobot();
  }

  async navigateParamsPage(options = {}) {
    const path = this.pathForFleetPage("params");
    const method = options.replace ? "replaceState" : "pushState";
    if (window.location.pathname !== path) {
      window.history[method]({ fleetPage: "params" }, "", path);
    }
    this.setFleetTab("params");
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
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
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
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
    if (tab === "robots") {
      this.renderSelectedRobot();
      return;
    }
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
    const selected = this.selectedRobot();
    if (selected && this.isFleetManager(selected)) {
      return false;
    }
    const fleet = this.robots.find((robot) => robot.id === this.fleetManagerSimId)
      || this.robots.find((robot) => this.isFleetManager(robot));
    if (!fleet) {
      return false;
    }
    this.selectedRobotId = fleet.id;
    window.localStorage.setItem("operator:selectedRobotId", fleet.id);
    this.currentStatus = null;
    this.currentRoute = null;
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
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
      this.closeScanStream();
      this.closeSlamStream();
      this.closeTeleopSocket(true);
      this.syncFleetStatusStream();
      return false;
    }
    this.selectedRobotId = robot.id;
    window.localStorage.setItem("operator:selectedRobotId", robot.id);
    this.currentStatus = null;
    this.currentRoute = null;
    this.closeScanStream();
    this.closeSlamStream();
    this.closeTeleopSocket(true);
    this.syncFleetStatusStream();
    return true;
  }

  setFleetTab(tabName) {
    const tab = ["robots", "fleet", "params", "model", "map"].includes(tabName) ? tabName : "robots";
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
    const isRobotsHome = this.fleetActiveTab === "robots";
    const isRobotModel = this.fleetActiveTab === "model";
    const isRobotParams = !isFleet && this.fleetActiveTab === "params";
    const pageKey = isRobotsHome ? "robots-home" : (isRobotModel ? "robot-model" : (isFleet ? (this.fleetActiveTab || "fleet") : (isRobotParams ? "robot-params" : "robot")));
    document.body.dataset.fleetPage = pageKey;
    if (this.globalHomeButton) {
      this.globalHomeButton.classList.toggle("primary", isRobotsHome);
    }
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
    if (isFleet && !isRobotModel && !isRobotsHome) {
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
    return Boolean(robot && (
      robot.id === this.fleetManagerId
      || robot.id === this.fleetManagerSimId
      || robot.type === "fleet_manager"
    ));
  }

  isFleetManagerSim(robot = this.selectedRobot()) {
    if (!this.isFleetManager(robot)) {
      return false;
    }
    const identityMode = String(robot?.identity?.mode || "").toLowerCase();
    const statusState = String(robot?.status?.state || "").toLowerCase();
    return robot.id === this.fleetManagerSimId
      || identityMode === "simulation"
      || statusState === "simulation";
  }

  fleetApiBase(robot = this.selectedRobot()) {
    return this.isFleetManagerSim(robot) ? "/api/fleet-manager-sim" : "/api/fleet-manager";
  }

  fleetApiPath(path = "", robot = this.selectedRobot()) {
    const suffix = String(path || "");
    return `${this.fleetApiBase(robot)}${suffix.startsWith("/") ? suffix : `/${suffix}`}`;
  }

  fleetWsPath(robot = this.selectedRobot()) {
    return this.isFleetManagerSim(robot) ? "/ws/fleet-manager-sim" : "/ws/fleet-manager";
  }

  isRos2Robot(robot = this.selectedRobot()) {
    const type = String(robot?.type || robot?.mode || "").toLowerCase();
    return Boolean(robot && ["grpc", "aivison_grpc", "real_grpc"].includes(type));
  }

  robotBaseName(robot) {
    const identity = robot?.identity || robot?.lastIdentity || {};
    return String(robot?.name || identity.robotId || robot?.id || "-");
  }

  robotEndpointLabel(robot) {
    if (!robot || this.isFleetManager(robot)) {
      return "";
    }
    if (this.isRos2Robot(robot) && String(robot.type || "").toLowerCase().includes("grpc")) {
      return `${robot.host || "-"}:${robot.port || 50051}`;
    }
    return robot.host ? `${robot.host}:${robot.port || ""}`.replace(/:$/, "") : "";
  }

  robotDisplayName(robot) {
    const baseName = this.robotBaseName(robot);
    if (!robot || this.isFleetManager(robot)) {
      return baseName;
    }
    const duplicateCount = this.robots.filter((item) => {
      return !this.isFleetManager(item) && this.robotBaseName(item) === baseName;
    }).length;
    const endpoint = this.robotEndpointLabel(robot);
    return duplicateCount > 1 && endpoint ? `${baseName} (${endpoint})` : baseName;
  }

  robotPingLabel(robot) {
    if (!robot || this.isFleetManager(robot)) {
      return "-";
    }
    const pingMs = Number(robot.pingMs);
    if (Number.isFinite(pingMs) && pingMs >= 0) {
      return `${Math.max(1, Math.round(pingMs))} ms`;
    }
    if (robot.probed && !robot.pingOk) {
      const pingError = String(robot.pingError || "").toLowerCase();
      if (pingError.includes("operation not permitted") || pingError.includes("not installed")) {
        return "unavailable";
      }
      return "timeout";
    }
    return "-";
  }

  fleetRuntimeMode(status = this.currentStatus, robot = this.selectedRobot()) {
    const statusManagerId = String(status?.managerId || "");
    if (!statusManagerId || !robot || statusManagerId === robot.id) {
      const mode = String(status?.mode || "").toLowerCase();
      if (mode) {
        return mode;
      }
    }
    return this.isFleetManagerSim(robot) ? "simulation" : "robots";
  }

  isFleetRobotsMode() {
    return this.isFleetManager() && this.fleetRuntimeMode() === "robots";
  }

  isFleetRemoteRobot(robot) {
    const mode = String(robot?.mode || robot?.type || "").toLowerCase();
    return ["remote", "robot", "real", "grpc", "aivison_grpc", "real_grpc"].includes(mode);
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
    if (this.isGlobalHomePage()) {
      this.closeRobotStatusStream();
      this.closeFleetStatusStream();
      this.stopFleetAnimationLoop();
      return;
    }
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
    const selectedFleetManager = this.selectedRobot();
    const managerId = selectedFleetManager?.id || "";
    if (this.fleetStatusSocket && this.fleetStatusManagerId && this.fleetStatusManagerId !== managerId) {
      this.closeFleetStatusStream();
    }
    if (this.fleetStatusStreamOpen() || this.fleetStatusStreamConnecting()) {
      return;
    }
    if (this.fleetStatusReconnectTimer) {
      window.clearTimeout(this.fleetStatusReconnectTimer);
      this.fleetStatusReconnectTimer = null;
    }

    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${scheme}://${window.location.host}${this.fleetWsPath(selectedFleetManager)}?intervalMs=${this.fleetStreamIntervalMs}`);
    this.fleetStatusSocket = socket;
    this.fleetStatusManagerId = managerId;
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
    this.fleetStatusManagerId = "";
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

  markRobotStatusReceived() {
    this.robotStatusReceivedAt = performance.now();
  }

  robotStatusIsStale() {
    if (this.isFleetManager() || !this.robotStatusReceivedAt) {
      return false;
    }
    return performance.now() - this.robotStatusReceivedAt > this.robotStatusFreshTimeoutMs;
  }

  robotStatusStaleMessage() {
    if (!this.robotStatusReceivedAt) {
      return "Waiting for robot status.";
    }
    const ageSec = Math.max(0, (performance.now() - this.robotStatusReceivedAt) / 1000);
    return `Robot status is stale (${ageSec.toFixed(1)}s). Waiting for update.`;
  }

  statusForRobotDisplay(robot) {
    const source = robot && typeof robot === "object" ? robot : {};
    if (!this.robotStatusIsStale()) {
      return source;
    }
    return {
      ...source,
      connected: false,
      localizationOk: false,
      state: "DISCONNECTED",
      message: this.robotStatusStaleMessage(),
      pose: null,
    };
  }

  expireRobotStatusIfStale() {
    if (this.isGlobalHomePage() || this.isFleetManager()) {
      return;
    }
    if (!this.currentStatus?.robot || !this.robotStatusIsStale()) {
      return;
    }
    const robot = this.currentStatus.robot;
    if (robot.connected === false && robot.localizationOk === false && !robot.pose) {
      return;
    }
    this.currentStatus = {
      ...this.currentStatus,
      robot: this.statusForRobotDisplay(robot),
    };
    this.renderRobotRuntimeTick();
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
      this.robotStatusReceivedAt = 0;
      return;
    }
    const socket = this.robotStatusSocket;
    this.robotStatusSocket = null;
    this.robotStatusRobotId = "";
    this.robotStatusStreamAttemptedAt = 0;
    this.robotStatusReceivedAt = 0;
    try {
      socket.close(1000, "operator target changed");
    } catch (_) {
      // Some browsers throw if the socket is already closing.
    }
  }

  scanStreamOpen() {
    return typeof WebSocket !== "undefined"
      && this.scanSocket
      && this.scanSocket.readyState === WebSocket.OPEN;
  }

  scanStreamConnecting() {
    return typeof WebSocket !== "undefined"
      && this.scanSocket
      && this.scanSocket.readyState === WebSocket.CONNECTING;
  }

  async toggleScanStream() {
    if (this.scanStreamOpen() || this.scanStreamConnecting() || this.scanEnabled) {
      this.closeScanStream();
      return;
    }
    await this.openScanStream();
  }

  async openScanStream() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot) || typeof WebSocket === "undefined") {
      this.closeScanStream();
      return;
    }
    if (this.robotParamsRobotId !== robot.id || !this.robotParamsLoaded) {
      await this.ensureRobotParamsLoaded();
    }
    if (this.scanRobotId && this.scanRobotId !== robot.id) {
      this.closeScanStream();
    }
    if (this.scanStreamOpen() || this.scanStreamConnecting()) {
      return;
    }
    const socket = new WebSocket(this.scanWsUrl(robot));
    this.scanSocket = socket;
    this.scanRobotId = robot.id;
    this.scanEnabled = true;
    this.latestScanFrame = null;
    this.syncScanUi("connecting");
    socket.addEventListener("open", () => {
      if (this.scanSocket !== socket) {
        return;
      }
      this.syncScanUi("waiting");
    });
    socket.addEventListener("message", (event) => {
      if (this.scanSocket !== socket) {
        return;
      }
      this.handleScanStreamMessage(event);
    });
    socket.addEventListener("error", () => {
      if (this.scanSocket === socket) {
        this.syncScanUi("error");
      }
    });
    socket.addEventListener("close", () => {
      if (this.scanSocket !== socket) {
        return;
      }
      this.scanSocket = null;
      this.scanRobotId = "";
      this.scanEnabled = false;
      this.syncScanUi("off");
    });
  }

  scanWsUrl(robot) {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    return `${protocol}//${window.location.host}/ws/robot/scan?robotId=${encodeURIComponent(robot.id)}&hz=1`;
  }

  closeScanStream() {
    this.scanEnabled = false;
    this.latestScanFrame = null;
    const socket = this.scanSocket;
    this.scanSocket = null;
    this.scanRobotId = "";
    if (socket) {
      try {
        socket.close(1000, "scan disabled");
      } catch (_) {
        // Some browsers throw if the socket is already closing.
      }
    }
    this.clearScanOverlay();
    this.syncScanUi("off");
  }

  handleScanStreamMessage(event) {
    let payload = null;
    try {
      payload = JSON.parse(event.data);
    } catch (_) {
      return;
    }
    if (!payload || typeof payload !== "object") {
      return;
    }
    if (!payload.ok) {
      this.syncScanUi(payload.error || "waiting");
      return;
    }
    this.latestScanFrame = payload;
    this.drawScanOverlay(payload);
    const count = Array.isArray(payload.ranges) ? payload.ranges.filter((item) => Number.isFinite(Number(item))).length : 0;
    this.syncScanUi(`${count} pts`);
  }

  syncScanUi(statusText = "") {
    const robot = this.selectedRobot();
    this.scanToggleButton?.classList.toggle("primary", this.scanEnabled);
    this.scanToggleButton?.classList.toggle("hidden", !robot || this.isFleetManager(robot) || this.isRobotModelPage() || this.isParamsPage());
    if (this.scanToggleButton) {
      this.scanToggleButton.textContent = this.scanEnabled ? "Scan Off" : "Scan";
      this.scanToggleButton.title = statusText ? `Laser scan: ${statusText}` : "Show LaserScan on the map";
    }
  }

  slamStreamOpen() {
    return typeof WebSocket !== "undefined"
      && this.slamSocket
      && this.slamSocket.readyState === WebSocket.OPEN;
  }

  slamStreamConnecting() {
    return typeof WebSocket !== "undefined"
      && this.slamSocket
      && this.slamSocket.readyState === WebSocket.CONNECTING;
  }

  async openSlamDialog() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      return;
    }
    try {
      this.slamDialogStatus.className = "probe-result neutral";
      this.slamDialogStatus.textContent = "Loading default SLAM parameters...";
      if (typeof this.slamDialog.showModal === "function" && !this.slamDialog.open) {
        this.slamDialog.showModal();
      }
      const defaults = await this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/slam/defaults`);
      const params = defaults.params && typeof defaults.params === "object" ? defaults.params : {};
      this.slamDefaults = params;
      this.slamParamsInput.value = JSON.stringify(params, null, 2);
      this.slamDialogStatus.className = "probe-result success";
      this.slamDialogStatus.textContent = defaults.paramsPath
        ? `Loaded from ${defaults.paramsPath}.`
        : "Default parameters loaded from robot.";
    } catch (error) {
      this.slamDialogStatus.className = "probe-result error";
      this.slamDialogStatus.textContent = error.message || String(error);
    }
  }

  async startSlamFromDialog() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      return;
    }
    let params = {};
    try {
      params = JSON.parse(this.slamParamsInput.value || "{}");
      if (!params || typeof params !== "object" || Array.isArray(params)) {
        throw new Error("SLAM parameters must be a JSON object.");
      }
    } catch (error) {
      this.slamDialogStatus.className = "probe-result error";
      this.slamDialogStatus.textContent = error.message || String(error);
      return;
    }
    try {
      this.confirmStartSlamButton.disabled = true;
      this.slamDialogStatus.className = "probe-result neutral";
      this.slamDialogStatus.textContent = "Starting slam_toolbox...";
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/slam/start`, {
        params,
        useSimTime: true,
      });
      this.slamState = result.state || null;
      this.slamActive = Boolean(result.state?.active ?? true);
      this.beginRobotMapTransition("2D SLAM started. Waiting for live map...");
      this.slamDialog.close();
      this.openSlamStream();
      this.syncSlamUi("mapping");
      this.robotMessageText.textContent = "2D SLAM started. WASD teleop remains available.";
    } catch (error) {
      this.slamDialogStatus.className = "probe-result error";
      this.slamDialogStatus.textContent = error.message || String(error);
    } finally {
      this.confirmStartSlamButton.disabled = false;
    }
  }

  openSlamStream() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot) || typeof WebSocket === "undefined") {
      this.closeSlamStream();
      return;
    }
    if (this.slamRobotId && this.slamRobotId !== robot.id) {
      this.closeSlamStream();
    }
    if (this.slamStreamOpen() || this.slamStreamConnecting()) {
      return;
    }
    const socket = new WebSocket(this.slamWsUrl(robot));
    this.slamSocket = socket;
    this.slamRobotId = robot.id;
    this.slamActive = true;
    this.syncSlamUi("connecting");
    socket.addEventListener("open", () => {
      if (this.slamSocket !== socket) {
        return;
      }
      this.syncSlamUi("waiting for /map");
    });
    socket.addEventListener("message", (event) => {
      if (this.slamSocket !== socket) {
        return;
      }
      this.handleSlamStreamMessage(event);
    });
    socket.addEventListener("error", () => {
      if (this.slamSocket === socket) {
        this.syncSlamUi("error");
      }
    });
    socket.addEventListener("close", () => {
      if (this.slamSocket !== socket) {
        return;
      }
      this.slamSocket = null;
      this.slamRobotId = "";
      this.syncSlamUi(this.slamActive ? "disconnected" : "off");
    });
  }

  async refreshSelectedSlamState(options = {}) {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      this.slamActive = false;
      this.slamState = null;
      this.syncSlamUi("off");
      return;
    }
    const robotId = robot.id;
    try {
      const result = await this.getJson(`/api/robots/${encodeURIComponent(robotId)}/slam/state`);
      if (this.selectedRobotId !== robotId) {
        return;
      }
      const state = result.state || null;
      const active = Boolean(state?.active);
      this.slamState = state;
      this.slamActive = active;
      if (active) {
        if (!this.slamStreamOpen() && !this.slamStreamConnecting()) {
          this.openSlamStream();
        }
        this.syncSlamUi(state?.state || "mapping");
      } else {
        if (this.slamRobotId === robotId) {
          this.closeSlamStream();
        } else {
          this.slamMapPayload = null;
          this.slamMapFrame = null;
          this.syncSlamUi("off");
        }
      }
    } catch (error) {
      if (!options.quiet && this.robotMessageText) {
        this.robotMessageText.textContent = `SLAM state failed: ${error.message || error}`;
      }
      if (this.selectedRobotId === robotId) {
        this.syncSlamUi(this.slamActive ? "state unavailable" : "off");
      }
    }
  }

  slamWsUrl(robot) {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    return `${protocol}//${window.location.host}/ws/robot/slam?robotId=${encodeURIComponent(robot.id)}&hz=1&includeCells=1`;
  }

  closeSlamStream(options = {}) {
    const clearLive = options.clearLive !== false;
    const socket = this.slamSocket;
    this.slamSocket = null;
    this.slamRobotId = "";
    if (socket) {
      try {
        socket.close(1000, "slam stream closed");
      } catch (_) {
        // Some browsers throw if the socket is already closing.
      }
    }
    if (clearLive) {
      this.slamActive = false;
      this.slamState = null;
      this.slamMapPayload = null;
      this.slamMapFrame = null;
      this.renderOperatorMap();
    }
    this.syncSlamUi("off");
  }

  handleSlamStreamMessage(event) {
    let payload = null;
    try {
      payload = JSON.parse(event.data);
    } catch (_) {
      return;
    }
    if (!payload || typeof payload !== "object") {
      return;
    }
    if (!payload.ok) {
      this.syncSlamUi(payload.error || "waiting");
      return;
    }
    this.slamState = payload.state || this.slamState;
    this.slamActive = Boolean(payload.state?.active ?? this.slamActive);
    this.slamMapFrame = payload;
    const nextPayload = this.slamFrameToMapPayload(payload);
    if (nextPayload) {
      const previousSignature = String(this.slamMapPayload?.signature || "");
      this.slamMapPayload = nextPayload;
      if (nextPayload.signature !== previousSignature && !previousSignature) {
        this.resetMapView(true);
      }
    }
    const progress = Number(payload.state?.progress || 0);
    const size = Number(payload.width || 0) && Number(payload.height || 0)
      ? `${payload.width}x${payload.height}`
      : "waiting";
    this.syncSlamUi(progress ? `${size} ${Math.round(progress)}%` : size);
    this.renderOperatorMap();
  }

  slamFrameToMapPayload(frame) {
    const width = Number(frame.width || 0);
    const height = Number(frame.height || 0);
    const resolution = Number(frame.resolution || 0);
    if (width <= 0 || height <= 0 || resolution <= 0 || !frame.cellsBase64) {
      return null;
    }
    const imageDataUrl = this.slamCellsToImageDataUrl(frame);
    if (!imageDataUrl) {
      return null;
    }
    const padding = 36;
    const originX = Number(frame.originX || frame.origin_x || 0);
    const originY = Number(frame.originY || frame.origin_y || 0);
    const originYaw = Number(frame.originYaw || frame.origin_yaw || 0);
    const stamp = Number(frame.stampSec || frame.stamp_sec || Date.now() / 1000);
    return {
      mapName: "SLAM live",
      signature: `slam:${width}:${height}:${resolution}:${originX}:${originY}:${stamp}`,
      map: {
        mapName: "SLAM live",
        imageDataUrl,
        width,
        height,
        resolution,
        origin: [0, 0, 0],
        rosOrigin: [originX, originY, originYaw],
        coordinateFrame: "map_top_left",
        viewPadding: padding,
        viewWidth: width + (padding * 2),
        viewHeight: height + (padding * 2),
      },
      lms: [],
      edges: [],
    };
  }

  slamCellsToImageDataUrl(frame) {
    const width = Number(frame.width || 0);
    const height = Number(frame.height || 0);
    let binary = "";
    try {
      binary = window.atob(String(frame.cellsBase64 || ""));
    } catch (_) {
      return "";
    }
    if (!width || !height || binary.length < width * height) {
      return "";
    }
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const context = canvas.getContext("2d");
    if (!context) {
      return "";
    }
    const image = context.createImageData(width, height);
    for (let y = 0; y < height; y += 1) {
      for (let x = 0; x < width; x += 1) {
        const src = ((height - 1 - y) * width) + x;
        const encoded = binary.charCodeAt(src);
        const value = encoded - 1;
        let shade = 205;
        if (value >= 0) {
          shade = 255 - Math.round(Math.min(100, value) * 2.35);
        }
        const dst = ((y * width) + x) * 4;
        image.data[dst] = shade;
        image.data[dst + 1] = shade;
        image.data[dst + 2] = shade;
        image.data[dst + 3] = 255;
      }
    }
    context.putImageData(image, 0, 0);
    return canvas.toDataURL("image/png");
  }

  async finishSlam() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      return;
    }
    const defaultName = `slam_${new Date().toISOString().slice(0, 16).replace(/[-:T]/g, "")}`;
    const mapName = String(window.prompt("New map name", defaultName) || "").trim();
    if (!mapName) {
      return;
    }
    try {
      const result = await this.runMapTransfer("slam", async (progress) => {
        await progress(3, "Preparing SLAM save...", 100);
        let staged = 8;
        const timer = window.setInterval(() => {
          staged = Math.min(92, staged + (staged < 60 ? 7 : 3));
          this.setMapTransferProgress(staged, "Saving SLAM map and creating smap files...", 0);
        }, 380);
        try {
          const response = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/slam/finish`, {
            mapName,
            activate: true,
          });
          await progress(95, "Pulling saved map into Operator App...", 150);
          return response;
        } finally {
          window.clearInterval(timer);
        }
      });
      this.slamActive = false;
      this.slamState = result.state || null;
      this.beginRobotMapTransition(`Loading saved SLAM map ${result.mapName || mapName}...`);
      this.closeSlamStream();
      await this.refreshRobotMapState({ quiet: true });
      await this.refreshRobots({ quiet: true });
      await this.fetchSelectedRobotStatus(true);
      this.renderSelectedRobot();
      this.robotMessageText.textContent = `SLAM map saved: ${result.mapName || mapName}.`;
    } catch (error) {
      this.robotMessageText.textContent = `Finish SLAM failed: ${error.message || error}`;
    }
  }

  async cancelSlam() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      return;
    }
    if (!window.confirm("Cancel current SLAM session? The live map will not be saved.")) {
      return;
    }
    try {
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/slam/cancel`, {
        reason: "SLAM canceled by operator.",
      });
      this.slamActive = false;
      this.slamState = result.state || null;
      this.beginRobotMapTransition("Restoring navigation map after SLAM cancel...");
      this.closeSlamStream();
      await this.refreshRobotMapState({ quiet: true });
      await this.fetchSelectedRobotStatus(true);
      this.syncSlamUi("canceled");
      this.robotMessageText.textContent = "SLAM canceled.";
    } catch (error) {
      this.robotMessageText.textContent = `Cancel SLAM failed: ${error.message || error}`;
    }
  }

  syncSlamUi(statusText = "") {
    const robot = this.selectedRobot();
    const visible = Boolean(robot) && !this.isFleetManager(robot) && !this.isRobotModelPage() && !this.isParamsPage();
    this.slamActionGroup?.classList.toggle("hidden", !visible);
    this.startSlamButton?.classList.toggle("primary", !this.slamActive);
    if (this.startSlamButton) {
      this.startSlamButton.disabled = this.slamActive;
      this.startSlamButton.textContent = this.slamActive ? "Running" : "Start 2D";
      this.startSlamButton.title = statusText ? `SLAM: ${statusText}` : "Start 2D SLAM";
    }
    if (this.doneSlamButton) {
      this.doneSlamButton.classList.toggle("hidden", !this.slamActive);
      this.doneSlamButton.disabled = !this.slamActive;
      this.doneSlamButton.title = "Save the live SLAM map as a new smap.";
    }
    if (this.cancelSlamButton) {
      this.cancelSlamButton.classList.toggle("hidden", !this.slamActive);
      this.cancelSlamButton.disabled = !this.slamActive;
      this.cancelSlamButton.title = "Cancel SLAM without saving.";
    }
  }

  clearScanOverlay() {
    if (this.operatorScanLayer) {
      this.operatorScanLayer.innerHTML = "";
    }
  }

  beginRobotMapTransition(message = "Map is changing...") {
    this.navigateMode = false;
    this.relocateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    this.currentRoute = null;
    if (!this.isFleetManager() && this.currentStatus?.robot) {
      this.currentStatus = {
        ...this.currentStatus,
        robot: {
          ...this.currentStatus.robot,
          connected: false,
          localizationOk: false,
          state: "LOCALIZING",
          message,
          pose: null,
        },
      };
      this.robotStatusReceivedAt = 0;
    }
    this.latestScanFrame = null;
    this.fleetManualLookahead = null;
    this.operatorMapPayload = null;
    this.operatorMapSignature = "";
    this.slamMapPayload = null;
    this.slamMapFrame = null;
    this.clearScanOverlay();
    this.clearRelocationPreview();
    this.resetMapView(true);
    this.syncModeButtons();
    const mapActionsDisabled = !this.isFleetManager() && this.slamActive;
    if (this.controlLoadMapButton) {
      this.controlLoadMapButton.disabled = mapActionsDisabled;
    }
    if (this.controlPullMapButton) {
      this.controlPullMapButton.disabled = mapActionsDisabled;
    }
    if (this.controlPushMapButton) {
      this.controlPushMapButton.disabled = mapActionsDisabled;
    }
    this.renderOperatorMap();
    if (this.robotMessageText && message) {
      this.robotMessageText.textContent = message;
    }
  }

  drawScanOverlay(frame = this.latestScanFrame) {
    if (!this.operatorScanLayer) {
      return;
    }
    this.operatorScanLayer.innerHTML = "";
    if (!this.scanEnabled || this.isFleetManager() || !frame || !frame.ok) {
      return;
    }
    const payload = this.activeOperatorMapPayload();
    if (!payload || !payload.map) {
      return;
    }
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
    const ranges = Array.isArray(frame.ranges) ? frame.ranges : [];
    if (!ranges.length) {
      return;
    }
    const rangeMin = Math.max(0, Number(frame.rangeMin || 0));
    const rangeMax = Number(frame.rangeMax || 0) > 0 ? Number(frame.rangeMax) : Infinity;
    const angleMin = Number(frame.angleMin || 0);
    const angleIncrement = Number(frame.angleIncrement || 0);
    const sensorPose = this.scanSensorPose(pose, frame);
    const yaw = sensorPose.yaw;
    const originX = sensorPose.x;
    const originY = sensorPose.y;
    const segments = [];
    const stride = Math.max(1, Math.ceil(ranges.length / 1600));
    for (let index = 0; index < ranges.length; index += 1) {
      if (index % stride !== 0) {
        continue;
      }
      const range = Number(ranges[index]);
      if (!Number.isFinite(range) || range <= rangeMin || range > rangeMax) {
        continue;
      }
      const angle = yaw - (angleMin + (index * angleIncrement));
      const point = this.worldToPixel({
        x: originX + (Math.cos(angle) * range),
        y: originY + (Math.sin(angle) * range),
      });
      segments.push(`M ${point.x.toFixed(2)} ${point.y.toFixed(2)} h 0.01`);
    }
    if (!segments.length) {
      return;
    }
    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("class", "scan-point-cloud");
    path.setAttribute("d", segments.join(" "));
    this.operatorScanLayer.append(path);
  }

  scanSensorPose(pose, frame = {}) {
    const baseX = Number(pose.x || 0);
    const baseY = Number(pose.y || 0);
    const yaw = Number(pose.yaw || 0);
    const sensor = this.scanSensorFrame(frame);
    if (!sensor) {
      return { x: baseX, y: baseY, yaw };
    }
    const offsetX = Number(sensor.x || 0);
    const offsetY = Number(sensor.y || 0);
    const offsetYaw = Number(sensor.yaw || sensor.theta || 0);
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    return {
      x: baseX + (offsetX * cos) + (offsetY * sin),
      y: baseY + (offsetX * sin) - (offsetY * cos),
      yaw: yaw - offsetYaw,
    };
  }

  scanSensorFrame(frame = {}) {
    const model = this.currentRobotModel();
    const frames = model && model.frames && typeof model.frames === "object" ? model.frames : null;
    if (!frames) {
      return null;
    }
    const frameId = String(frame.frameId || "").replace(/^\//, "").toLowerCase();
    if (frameId && ["base_link", "base_footprint", "base"].includes(frameId)) {
      return null;
    }
    const normalized = (value) => String(value || "").replace(/^\//, "").toLowerCase();
    for (const [key, value] of Object.entries(frames)) {
      if (frameId && normalized(key) === frameId && value && typeof value === "object") {
        return value;
      }
    }
    if (frameId && /laser|lidar|scan/.test(frameId)) {
      for (const key of ["lidar", "laser", "scan", "base_scan"]) {
        const value = frames[key];
        if (value && typeof value === "object") {
          return value;
        }
      }
    }
    return null;
  }

  currentRobotModel() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot)) {
      return null;
    }
    if (this.robotParamsRobotId !== robot.id || !this.robotParams || typeof this.robotParams !== "object") {
      return null;
    }
    const model = this.robotParams.robot_model;
    return model && typeof model === "object" ? model : null;
  }

  teleopSocketOpenFor(robot) {
    return typeof WebSocket !== "undefined"
      && this.teleopSocket
      && this.teleopSocket.readyState === WebSocket.OPEN
      && this.teleopRobotId === robot?.id;
  }

  teleopSocketConnectingFor(robot) {
    return typeof WebSocket !== "undefined"
      && this.teleopSocket
      && this.teleopSocket.readyState === WebSocket.CONNECTING
      && this.teleopRobotId === robot?.id;
  }

  ensureTeleopSocket() {
    const robot = this.selectedRobot();
    if (!robot || this.isFleetManager(robot) || typeof WebSocket === "undefined") {
      return null;
    }
    if (this.teleopSocketOpenFor(robot) || this.teleopSocketConnectingFor(robot)) {
      return this.teleopSocket;
    }
    this.closeTeleopSocket(false);
    const socket = new WebSocket(this.teleopWsUrl(robot));
    this.teleopSocket = socket;
    this.teleopRobotId = robot.id;
    socket.addEventListener("message", (event) => {
      if (this.teleopSocket !== socket) {
        return;
      }
      this.handleTeleopStreamMessage(event);
    });
    socket.addEventListener("close", () => {
      if (this.teleopSocket !== socket) {
        return;
      }
      this.teleopSocket = null;
      this.teleopRobotId = "";
    });
    socket.addEventListener("error", () => {
      if (this.robotMessageText) {
        this.robotMessageText.textContent = "Manual control stream error.";
      }
    });
    return socket;
  }

  teleopWsUrl(robot) {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    return `${protocol}//${window.location.host}/ws/robot/teleop?robotId=${encodeURIComponent(robot.id)}`;
  }

  closeTeleopSocket(sendStop = false) {
    const socket = this.teleopSocket;
    this.teleopSocket = null;
    this.teleopRobotId = "";
    if (!socket) {
      return;
    }
    try {
      if (sendStop && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ type: "stop", linear: 0, angular: 0, timeoutMs: 80 }));
      }
      socket.close(1000, "manual control closed");
    } catch (_) {
      // Some browsers throw if the socket is already closing.
    }
  }

  sendRobotTeleop(twist, timeoutMs = 350) {
    const socket = this.ensureTeleopSocket();
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      return false;
    }
    socket.send(JSON.stringify({
      type: "teleop",
      linear: Number(twist.linear || 0),
      angular: Number(twist.angular || 0),
      timeoutMs,
    }));
    return true;
  }

  handleTeleopStreamMessage(event) {
    let payload = null;
    try {
      payload = JSON.parse(event.data);
    } catch (_) {
      return;
    }
    if (!payload || typeof payload !== "object") {
      return;
    }
    if (!payload.ok) {
      if (this.robotMessageText) {
        this.robotMessageText.textContent = payload.error || "Manual control failed.";
      }
      return;
    }
    const response = payload.response || {};
    if (response.status && typeof response.status === "object") {
      this.markRobotStatusReceived();
      this.currentStatus = response.status;
      this.renderSelectedRobot();
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
    this.markRobotStatusReceived();
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
    const managerId = String(state.managerId || "");
    const selectedManagerId = String(this.selectedRobot()?.id || "");
    if (managerId && selectedManagerId && managerId !== selectedManagerId) {
      return;
    }
    this.fleetStatusReceivedAt = performance.now();
    if (payload.type === "state") {
      this.currentStatus = state;
      this.renderFleetRuntimeTick();
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
      const frameIntervalMs = this.mapViewMode === "3d" ? 16 : 33;
      if (now - this.fleetAnimationLastAt >= frameIntervalMs) {
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
    if (this.mapViewMode === "3d") {
      this.refreshOperatorScene3d();
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
    this.pruneFleetVisualClocks(robots);
    if (this.currentStatus && this.currentStatus !== this.fleetStatusObjectRef) {
      this.fleetStatusObjectRef = this.currentStatus;
      if (!this.fleetStatusReceivedAt || performance.now() - this.fleetStatusReceivedAt > this.fleetStreamIntervalMs * 2) {
        this.fleetStatusReceivedAt = performance.now();
      }
    }
    return robots.map((robot) => this.fleetRenderRobot(robot));
  }

  fleetVisualRouteKey(robot, trajectory = null) {
    const points = Array.isArray(trajectory) ? trajectory : (Array.isArray(robot?.trajectory) ? robot.trajectory : []);
    const nodes = Array.isArray(robot?.planNodes) ? robot.planNodes : [];
    const samplePoint = (point) => {
      if (!point) {
        return "";
      }
      const x = Number(point.x || 0).toFixed(3);
      const y = Number(point.y || 0).toFixed(3);
      const t = Number(point.t || 0).toFixed(3);
      return `${x},${y},${t}`;
    };
    const sampleNode = (index) => {
      if (!nodes.length) {
        return "";
      }
      const normalized = Math.max(0, Math.min(nodes.length - 1, index));
      return String(nodes[normalized] || "");
    };
    const middlePoint = points.length ? points[Math.floor((points.length - 1) / 2)] : null;
    const middleNodeIndex = nodes.length ? Math.floor((nodes.length - 1) / 2) : 0;
    return [
      String(robot?.name || ""),
      String(robot?.routeRevision || 0),
      String(robot?.activeOrderId || ""),
      String(robot?.targetLm || robot?.targetName || ""),
      String(robot?.routeFinalLm || robot?.routeChunkGoalLm || ""),
      `${nodes.length}:${sampleNode(0)}:${sampleNode(middleNodeIndex)}:${sampleNode(nodes.length - 1)}`,
      `${points.length}:${samplePoint(points[0])}:${samplePoint(middlePoint)}:${samplePoint(points[points.length - 1])}`,
    ].join("|");
  }

  clearFleetVisualClock(robot) {
    if (!this.fleetVisualClocks?.size || !robot?.name) {
      return;
    }
    const prefix = `${String(robot.name)}|`;
    for (const key of this.fleetVisualClocks.keys()) {
      if (key.startsWith(prefix)) {
        this.fleetVisualClocks.delete(key);
      }
    }
  }

  pruneFleetVisualClocks(robots) {
    if (!this.fleetVisualClocks?.size) {
      return;
    }
    const activeKeys = new Set();
    for (const robot of robots) {
      const status = String(robot?.status || "");
      const trajectory = Array.isArray(robot?.trajectory) ? robot.trajectory : [];
      if (status === "MOVING" && trajectory.length >= 2) {
        activeKeys.add(this.fleetVisualRouteKey(robot, trajectory));
      }
    }
    for (const key of this.fleetVisualClocks.keys()) {
      if (!activeKeys.has(key)) {
        this.fleetVisualClocks.delete(key);
      }
    }
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
    const status = String(robot?.status || "");
    if (status !== "MOVING" || trajectory.length < 2 || !this.fleetStatusReceivedAt) {
      this.clearFleetVisualClock(robot);
      return baseClock;
    }
    const last = trajectory[trajectory.length - 1];
    const finalTime = Math.max(0, Number(last.t ?? trajectory.length - 1));
    const key = this.fleetVisualRouteKey(robot, trajectory);
    const prior = this.fleetVisualClocks.get(key) || null;
    const now = performance.now();
    const priorServerClock = prior ? Math.max(0, Number(prior.serverClock || 0)) : 0;
    const routeClockReset = Boolean(prior && baseClock < priorServerClock - 0.25);
    let visualClock = baseClock;
    if (prior && !routeClockReset) {
      // Render with a small confirmation lag. Never extrapolate beyond the
      // server clock: MAPF/collision checking may have stopped the robot while
      // a websocket tick was delayed by planning work.
      const priorClock = Math.min(baseClock, Math.max(0, Number(prior.clock || 0)));
      const frameDelta = status === "MOVING"
        ? Math.min(0.08, Math.max(0, (now - Number(prior.updatedAt || now)) / 1000))
        : 0;
      visualClock = Math.min(baseClock, priorClock + frameDelta);
    }
    visualClock = Math.min(finalTime, Math.max(0, visualClock));
    this.fleetVisualClocks.set(key, {
      clock: visualClock,
      serverClock: baseClock,
      updatedAt: now,
    });
    return visualClock;
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
    const shouldProbe = options.probe ?? !options.quiet;
    const result = await this.getJson(shouldProbe ? "/api/robots" : "/api/robots?probe=0");
    const nextRobots = Array.isArray(result.robots) ? result.robots : [];
    this.robots = options.quiet ? this.mergeQuietRobotPayloads(nextRobots) : nextRobots;
    if (this.selectedRobotId && !this.selectedRobot()) {
      this.selectedRobotId = "";
      window.localStorage.removeItem("operator:selectedRobotId");
      this.closeScanStream();
      this.closeSlamStream();
      this.closeTeleopSocket(true);
    }
    if (!this.selectedRobotId && this.robots.length && !this.isGlobalHomePage()) {
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
    if (this.isGlobalHomePage()) {
      this.robotMapState = this.emptyMapState();
      this.operatorMapPayload = null;
      this.operatorMapSignature = "";
      this.render();
      if (!options.quiet) {
        this.showProbeResult("neutral", "Robot list refreshed.");
      }
      return;
    }
    await this.refreshRobotMapState({ quiet: true });
    await this.fetchSelectedRobotStatus(true);
    await this.refreshSelectedSlamState({ quiet: true });
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
        probed: previous.probed,
        pingOk: previous.pingOk,
        pingMs: previous.pingMs,
        pingError: previous.pingError,
        pingTransport: previous.pingTransport,
      };
    });
  }

  async refreshRobotPings() {
    if (!this.isGlobalHomePage() || this.robotPingRefreshPending) {
      return;
    }
    this.robotPingRefreshPending = true;
    try {
      const result = await this.getJson("/api/robots/ping");
      const updates = new Map();
      for (const item of Array.isArray(result.robots) ? result.robots : []) {
        if (item && item.id) {
          updates.set(String(item.id), item);
        }
      }
      if (!updates.size) {
        return;
      }
      this.robots = this.robots.map((robot) => {
        const update = updates.get(robot.id);
        return update ? { ...robot, ...update } : robot;
      });
      this.applyRobotPingUpdates(updates);
    } finally {
      this.robotPingRefreshPending = false;
    }
  }

  applyRobotPingUpdates(updates) {
    for (const card of document.querySelectorAll(".robot-card[data-robot-id]")) {
      const update = updates.get(card.dataset.robotId || "");
      if (!update) {
        continue;
      }
      const robot = this.robots.find((item) => item.id === update.id) || update;
      const chip = card.querySelector("[data-role='robot-chip']");
      if (chip) {
        chip.className = robot.online ? "robot-chip online" : "robot-chip offline";
        chip.textContent = robot.online ? "online" : "offline";
      }
      const ping = card.querySelector("[data-role='robot-ping']");
      if (ping) {
        ping.textContent = `Ping: ${this.robotPingLabel(robot)}`;
      }
    }
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
        const robotActive = await this.getJson(this.fleetApiPath("/maps/active"));
        let localActive = await this.getJson(this.fleetApiPath("/maps/local/active"));
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

  applyLoadedMapResult(result, requestedMapName = "", robot = this.selectedRobot()) {
    const loadedName = String(result?.mapName || requestedMapName || "").trim();
    const local = result?.local && typeof result.local === "object" ? result.local : null;
    const localName = String(local?.activeMapName || local?.mapName || this.robotMapState.operatorActiveMapName || "").trim();
    const nextSignature = String(local?.signature || this.operatorMapSignature || "").trim();
    if (local?.map && typeof local.map === "object") {
      if (nextSignature && nextSignature !== this.operatorMapSignature) {
        this.resetMapView(true);
      }
      this.operatorMapPayload = local.map;
      this.operatorMapSignature = nextSignature;
    }
    const robotSignature = String(result?.signature || local?.robotSignature || this.robotMapState.robotSignature || "").trim();
    this.robotMapState = {
      ...this.robotMapState,
      robotActiveMapName: loadedName || this.robotMapState.robotActiveMapName,
      operatorActiveMapName: localName,
      robotSignature,
      operatorSignature: nextSignature || this.robotMapState.operatorSignature,
      sourceRobotMapName: String(local?.robotMapName || local?.sourceMapName || loadedName || this.robotMapState.sourceRobotMapName || "").trim(),
      hasLocalChanges: local
        ? Boolean(local.hasLocalChanges || (localName && loadedName && localName !== loadedName))
        : this.robotMapState.hasLocalChanges,
    };
    if (robot && loadedName) {
      robot.mapId = loadedName;
      if (robot.identity && typeof robot.identity === "object") {
        robot.identity.mapId = loadedName;
      }
    }
    if (this.isFleetManager(robot) && this.currentStatus && loadedName) {
      this.currentStatus = {
        ...this.currentStatus,
        mapName: loadedName,
        robots: [],
        route: null,
      };
      this.selectedFleetRobotName = "";
      this.fleetVisualClocks.clear();
    }
  }

  refreshAfterMapLoadInBackground() {
    this.refreshRobotMapState({ quiet: true })
      .then(() => this.renderSelectedRobot())
      .catch(() => {});
    this.refreshRobots({ quiet: true }).catch(() => {});
    this.fetchSelectedRobotStatus(true).catch(() => {});
  }

  async fetchSelectedRobotStatus(silent = false) {
    if (silent && this.isGlobalHomePage()) {
      return;
    }
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
        ? await this.getJson(this.fleetApiPath("/state"))
        : await this.getJson(this.robotApiPath("/api/robot/status"));
      if (this.isFleetManager(robot)) {
        this.fleetStatusReceivedAt = performance.now();
      } else {
        this.markRobotStatusReceived();
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
    if (this.isGlobalHomePage()) {
      this.closeFleetStatusStream();
      this.stopFleetAnimationLoop();
      return;
    }
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
      const result = await this.postJson(this.fleetApiPath("/tick"), {});
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
      const incomingRoutePreview = Array.isArray(robot.routePreview) ? robot.routePreview : [];
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
        routePreview: incomingRoutePreview.length
          ? incomingRoutePreview
          : (hasTarget && Array.isArray(prior.routePreview) ? prior.routePreview : []),
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
    if (this.homeRobotCountText) {
      this.homeRobotCountText.textContent = this.homeRobotCountLabel();
    }
    this.renderRobotList();
    this.renderSelectedRobot();
  }

  homeRobotCountLabel() {
    const robotCount = this.robots.filter((robot) => !this.isFleetManager(robot) && !robot.system).length;
    const systemCount = this.robots.length - robotCount;
    const robotLabel = `${robotCount} ${robotCount === 1 ? "robot" : "robots"}`;
    if (!systemCount) {
      return robotLabel;
    }
    return `${robotLabel}, ${systemCount} system`;
  }

  renderRobotList() {
    this.renderRobotCards(this.robotsList, { openWorkspace: false });
    this.renderHomeRobotGrid();
  }

  renderHomeRobotGrid() {
    if (this.homeRobotCountText) {
      this.homeRobotCountText.textContent = this.homeRobotCountLabel();
    }
    this.renderRobotCards(this.homeRobotGrid, { openWorkspace: true, home: true });
  }

  syncRobotCardSelection() {
    for (const card of document.querySelectorAll(".robot-card[data-robot-id]")) {
      card.classList.toggle("active", card.dataset.robotId === this.selectedRobotId);
    }
  }

  renderRobotCards(container, options = {}) {
    if (!container) {
      return;
    }
    container.innerHTML = "";
    if (!this.robots.length) {
      const empty = document.createElement("div");
      empty.className = "probe-result neutral";
      empty.textContent = "No robots added yet. Use Add Robot + to connect by IP.";
      container.append(empty);
      return;
    }

    for (const robot of this.robots) {
      const isFleet = this.isFleetManager(robot);
      const isRos2 = this.isRos2Robot(robot);
      const button = document.createElement("div");
      button.className = options.home ? "robot-card home-robot-card" : "robot-card";
      button.tabIndex = 0;
      button.setAttribute("role", "button");
      button.dataset.robotId = robot.id;
      if (robot.id === this.selectedRobotId) {
        button.classList.add("active");
      }
      const selectRobot = async ({ enterWorkspace = false } = {}) => {
        if (this.selectedRobotId !== robot.id) {
          this.closeScanStream();
          this.closeSlamStream();
          this.closeTeleopSocket(true);
          this.manualKeys.clear();
          this.syncManualButtons();
        }
        this.selectedRobotId = robot.id;
        window.localStorage.setItem("operator:selectedRobotId", robot.id);
        this.currentStatus = null;
        this.currentRoute = null;
        this.syncFleetStatusStream();
        this.closeSidebar();
        if (options.home && !enterWorkspace) {
          this.syncRobotCardSelection();
          return;
        }
        if (enterWorkspace && options.openWorkspace) {
          await this.navigateHomePage();
          return;
        }
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
        await this.refreshSelectedSlamState({ quiet: true });
        if (this.isRobotModelPage() && !this.isFleetManager(robot)) {
          await this.ensureRobotParamsLoaded(true);
        }
        if (this.isParamsPage()) {
          await this.ensureCurrentParamsLoaded(true);
        }
        this.render();
      };
      button.addEventListener("click", () => selectRobot({ enterWorkspace: false }));
      button.addEventListener("dblclick", (event) => {
        event.preventDefault();
        selectRobot({ enterWorkspace: true }).catch((error) => {
          this.showProbeResult("error", error.message || String(error));
        });
      });
      button.addEventListener("keydown", async (event) => {
        if (event.key !== "Enter" && event.key !== " ") {
          return;
        }
        event.preventDefault();
        await selectRobot({ enterWorkspace: options.openWorkspace && robot.id === this.selectedRobotId });
      });

      const identity = robot.identity || robot.lastIdentity || {};
      const status = robot.status || {};
      const chipClass = robot.online ? "robot-chip online" : "robot-chip offline";
      const systemRobot = isFleet || robot.system;
      const chipText = systemRobot ? "system" : (robot.online ? "online" : "offline");
      const connectionLabel = isFleet
        ? "local fleet controller"
        : (isRos2
          ? (String(robot.type || "").toLowerCase().includes("grpc")
            ? `gRPC ${this.escapeHtml(robot.host || "-")}:${this.escapeHtml(String(robot.port || 50051))}`
            : `ROS2 ${this.escapeHtml(robot.host || "DDS")} domain ${this.escapeHtml(String(robot.domainId ?? 0))}`)
          : `${this.escapeHtml(robot.host)}:${this.escapeHtml(String(robot.port))}`);
      button.innerHTML = `
        <div class="robot-card-header">
          <div>
            <strong>${this.escapeHtml(this.robotDisplayName(robot))}</strong>
            <p>${connectionLabel}</p>
          </div>
          <div class="robot-card-actions">
            <span class="${chipClass}" data-role="robot-chip">${chipText}</span>
            ${systemRobot ? "" : '<button class="robot-card-remove" type="button" aria-label="Remove robot">Delete</button>'}
          </div>
        </div>
        <div class="robot-card-meta">
          <div>Robot ID: ${this.escapeHtml(identity.robotId || "-")}</div>
          <div>Map: ${this.escapeHtml(identity.mapId || "-")}</div>
          <div>State: ${this.escapeHtml(status.state || status.stateText || "-")}</div>
          ${isFleet ? "" : `<div data-role="robot-ping">Ping: ${this.escapeHtml(this.robotPingLabel(robot))}</div>`}
          ${isFleet ? `<div>Fleet robots: ${this.escapeHtml(String(status.robots || 0))}</div>` : ""}
        </div>
      `;
      const removeButton = button.querySelector(".robot-card-remove");
      if (removeButton) {
        removeButton.addEventListener("click", async (event) => {
          event.stopPropagation();
          await this.handleRemoveRobot(robot);
        });
        removeButton.addEventListener("dblclick", (event) => {
          event.stopPropagation();
        });
      }
      container.append(button);
    }
  }

  renderSelectedRobot() {
    const robot = this.selectedRobot();
    this.renderSidebar();
    this.renderHomeRobotGrid();
    if (this.isGlobalHomePage()) {
      this.homePage?.classList.remove("hidden");
      this.emptyState.classList.add("hidden");
      this.robotView.classList.add("hidden");
      this.fleetControlPanel.classList.add("hidden");
      this.robotParamsPanel.classList.add("hidden");
      this.robotModelPanel.classList.add("hidden");
      return;
    }
    this.homePage?.classList.add("hidden");
    if (!robot) {
      this.emptyState.classList.remove("hidden");
      this.robotView.classList.add("hidden");
      this.fleetControlPanel.classList.add("hidden");
      this.robotParamsPanel.classList.add("hidden");
      this.robotModelPanel.classList.add("hidden");
      this.robotActiveMapText.textContent = "-";
      this.operatorActiveMapText.textContent = "-";
      this.robotStateText.textContent = "-";
      if (this.robotControlText) {
        this.robotControlText.textContent = "-";
      }
      this.nearestLmText.textContent = "-";
      this.mapSyncStatus.className = "probe-result neutral";
      this.mapSyncStatus.textContent = "Select a robot to see map sync state.";
      return;
    }

    this.emptyState.classList.add("hidden");
    this.robotView.classList.remove("hidden");
    if (this.robotWorkspaceTitle) {
      this.robotWorkspaceTitle.textContent = this.robotDisplayName(robot);
    }
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

  robotControlPayload(robot) {
    const control = robot && typeof robot.control === "object" ? robot.control : {};
    return {
      state: String(robot?.controlState || control.state || "").toUpperCase(),
      ownerId: String(robot?.controlOwner || control.ownerId || ""),
      ownerName: String(robot?.controlOwnerName || control.ownerName || ""),
    };
  }

  robotControlLabel(robot) {
    const control = this.robotControlPayload(robot);
    if (!control.ownerId) {
      return "free";
    }
    return control.ownerName || control.ownerId;
  }

  robotNavigationPaused(robot) {
    return Boolean(robot?.navigationPaused) || String(robot?.state || "").toUpperCase() === "PAUSED";
  }

  robotLocalizationLabel(robot) {
    return robot?.localizationOk
      ? `ok (${Number(robot.localizationAgeSec || 0).toFixed(2)} s)`
      : "waiting";
  }

  robotLifecycleReason(robot) {
    const parts = [];
    const control = this.robotControlPayload(robot);
    if (control.ownerId) {
      parts.push(`control: ${control.ownerName || control.ownerId}`);
    } else {
      parts.push("control: free");
    }
    if (this.robotNavigationPaused(robot)) {
      parts.push("navigation paused");
    }
    if (robot?.message) {
      parts.push(robot.message);
    }
    return parts.join(" | ") || "-";
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
    const robotReport = payload.robot_report || payload.robotReport;
    if (robotReport && typeof robotReport === "object") {
      for (const key of keys) {
        if (robotReport[key] !== undefined && robotReport[key] !== null && robotReport[key] !== "") {
          return robotReport[key];
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

  isGlobalHomePage() {
    return this.fleetActiveTab === "robots";
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
    const showRobotLifecycle = !isFleet && !isRobotModel && !isParams;
    this.driveActionGroup?.classList.toggle("hidden", !showRobotLifecycle);
    this.localizationActionGroup?.classList.toggle("hidden", !showRobotLifecycle);
    this.navigationActionGroup?.classList.toggle("hidden", isRobotModel || isParams);
    this.visualActionGroup?.classList.toggle("hidden", isRobotModel || isParams);
    this.mapActionGroup?.classList.toggle("hidden", isRobotModel || isParams);
    this.slamActionGroup?.classList.toggle("hidden", !showRobotLifecycle);
    for (const button of [
      this.takeControlButton,
      this.releaseControlButton,
      this.relocateRobotButton,
      this.pauseRouteButton,
      this.resumeRouteButton,
    ]) {
      button?.classList.toggle("hidden", !showRobotLifecycle);
    }
    this.syncScanUi();
    this.syncSlamUi();
    const mapActionsDisabled = !isFleet && this.slamActive;
    this.controlPullMapButton.classList.toggle("hidden", false);
    this.controlPushMapButton.classList.toggle("hidden", false);
    this.controlLoadMapButton.disabled = mapActionsDisabled;
    this.controlPullMapButton.disabled = mapActionsDisabled;
    this.controlPushMapButton.disabled = mapActionsDisabled;
    this.cancelRouteButton.textContent = isFleet ? "Stop Active" : "Cancel Route";
    this.stopRobotButton.textContent = isFleet ? "Stop Fleet" : "Stop";
    this.syncModeButtons();
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
    const robot = this.statusForRobotDisplay(status.robot || {});
    const route = status.route || this.currentRoute || null;
    const pose = robot.pose || null;
    const connected = Boolean(robot.connected);
    const state = String(robot.state || (selected && selected.online ? "ONLINE" : "OFFLINE") || "-");

    this.robotStateText.textContent = state;
    if (this.robotControlText) {
      this.robotControlText.textContent = this.robotControlLabel(robot);
    }
    this.nearestLmText.textContent = robot.nearestLm || "-";
    this.renderInspectorDetails({
      robot: robot.robotId || selected?.name || selected?.id || "-",
      mode: this.slamActive ? "mapping" : "robot",
      connection: this.robotStatusStreamOpen() ? "robot websocket" : (connected ? "online" : "offline"),
      map: robot.mapId || selected?.identity?.mapId || this.robotMapState.robotActiveMapName || "-",
      currentLm: robot.nearestLm || "-",
      localization: this.robotLocalizationLabel(robot),
      targetLm: robot.targetLm || "-",
      currentEdge: robot.currentEdgeId || "-",
      progress: this.formatProgress(robot.routeProgress, "0%"),
      battery: this.formatBattery(robot),
      confidence: this.formatConfidence(robot),
      pose: this.formatPose(pose),
      velocity: this.formatVelocity(robot.velocity),
      api: this.isRos2Robot(selected)
        ? (String(selected?.type || "").toLowerCase().includes("grpc")
          ? (selected?.baseUrl || `grpc://${selected?.host || "-"}:${selected?.port || 50051}`)
          : `ROS2 ${selected?.host || "DDS"} domain ${selected?.domainId ?? 0}`)
        : (selected?.baseUrl || "-"),
      reason: this.robotLifecycleReason(robot),
    });
    this.robotMessageText.textContent = this.slamActive
      ? (this.slamMapPayload ? "2D SLAM running." : "Waiting for live SLAM map.")
      : (robot.message || (this.operatorMapPayload ? "Robot status ready." : "Pull the active robot map to display Map & Control."));
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
    const robot = this.statusForRobotDisplay(status.robot || {});
    const route = status.route || this.currentRoute || null;
    const pose = robot.pose || null;
    const connected = Boolean(robot.connected);
    const state = String(robot.state || (selected && selected.online ? "ONLINE" : "OFFLINE") || "-");

    this.robotStateText.textContent = state;
    if (this.robotControlText) {
      this.robotControlText.textContent = this.robotControlLabel(robot);
    }
    this.nearestLmText.textContent = robot.nearestLm || "-";
    this.renderInspectorDetails({
      robot: robot.robotId || selected?.name || selected?.id || "-",
      mode: this.slamActive ? "mapping" : "robot",
      connection: this.robotStatusStreamOpen() ? "robot websocket" : (connected ? "online" : "offline"),
      map: robot.mapId || selected?.identity?.mapId || this.robotMapState.robotActiveMapName || "-",
      currentLm: robot.nearestLm || "-",
      localization: this.robotLocalizationLabel(robot),
      targetLm: robot.targetLm || "-",
      currentEdge: robot.currentEdgeId || "-",
      progress: this.formatProgress(robot.routeProgress, "0%"),
      battery: this.formatBattery(robot),
      confidence: this.formatConfidence(robot),
      pose: this.formatPose(pose),
      velocity: this.formatVelocity(robot.velocity),
      api: this.isRos2Robot(selected)
        ? (String(selected?.type || "").toLowerCase().includes("grpc")
          ? (selected?.baseUrl || `grpc://${selected?.host || "-"}:${selected?.port || 50051}`)
          : `ROS2 ${selected?.host || "DDS"} domain ${selected?.domainId ?? 0}`)
        : (selected?.baseUrl || "-"),
      reason: this.robotLifecycleReason(robot),
    });
    this.robotMessageText.textContent = this.slamActive
      ? (this.slamMapPayload ? "2D SLAM running." : "Waiting for live SLAM map.")
      : (robot.message || (this.operatorMapPayload ? "Robot status ready." : "Pull the active robot map to display Map & Control."));
    this.routeNodesText.textContent = route && Array.isArray(route.nodes) && route.nodes.length
      ? route.nodes.join(" -> ")
      : "No route planned.";
    this.syncModeButtons();
    this.syncManualButtons();
    this.drawRoute();
    this.drawScanOverlay();
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

    this.robotStateText.textContent = selectedFleetRobot ? String(selectedFleetRobot.status || "-") : mode.toUpperCase();
    if (this.robotControlText) {
      this.robotControlText.textContent = selectedFleetRobot && this.isFleetRemoteRobot(selectedFleetRobot)
        ? this.robotControlLabel(remoteStatus)
        : mode;
    }
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
    if (this.robotControlText) {
      this.robotControlText.textContent = selectedFleetRobot && this.isFleetRemoteRobot(selectedFleetRobot)
        ? this.robotControlLabel(remoteStatus)
        : mode;
    }
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
    this.drawLandmarks();
    this.drawRobot();
    this.syncMapControls();
    this.syncModeButtons();
    this.syncManualButtons();
    this.syncDynamicBenchmarkControls();
    if (this.isFleetManagerSim() && status.dynamicBenchmark?.active && this.fleetBenchmarkStatus) {
      this.fleetBenchmarkStatus.className = "probe-result success compact";
      this.fleetBenchmarkStatus.textContent = this.fleetBenchmarkSummary(
        { benchmark: status.dynamicBenchmark },
        robots.length,
      );
    }
    this.ensureFleetAnimationLoop();
  }

  renderFleetStateImmediately() {
    if (!this.isFleetManager()) {
      return;
    }
    this.fleetStatusReceivedAt = performance.now();
    this.fleetStatusObjectRef = this.currentStatus;
    this.renderSelectedRobot();
    this.renderFleetRuntimeTick();
    this.refreshOperatorScene3d();
    this.syncFleetStatusStream();
    this.syncDynamicBenchmarkControls();
  }

  syncDynamicBenchmarkControls() {
    if (!this.fleetBenchmarkPlanButton) {
      return;
    }
    const dynamic = this.currentStatus?.dynamicBenchmark || {};
    this.fleetBenchmarkPlanButton.textContent = dynamic.active
      ? "Stop Dynamic Orders"
      : "Start Dynamic Orders";
  }

  selectedFleetRobot(robots = null) {
    const items = robots || (Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []);
    if (!items.length) {
      return null;
    }
    const selected = items.find((robot) => robot.name === this.selectedFleetRobotName);
    return selected || items[0];
  }

  selectFleetRobotByName(robotName) {
    const name = String(robotName || "").trim();
    if (!name) {
      return;
    }
    this.selectedFleetRobotName = name;
    if (this.navigateMode && this.pendingFleetAction) {
      this.pendingFleetRobotName = name;
    }
    window.localStorage.setItem("operator:selectedFleetRobotName", name);
    this.renderSelectedRobot();
  }

  renderFleetControls(robots) {
    const lms = this.navigationLandmarks();
    const previousSpawn = this.fleetSpawnLmSelect.value;
    const selectedFleetRobot = this.selectedFleetRobot(robots);

    this.syncFleetRemoteFields();
    this.fillFleetSpawnSelect(lms.map((lm) => lm.name), previousSpawn);
    this.syncFleetSimTools();
    if (selectedFleetRobot) {
      this.selectedFleetRobotName = selectedFleetRobot.name;
      window.localStorage.setItem("operator:selectedFleetRobotName", this.selectedFleetRobotName);
    }
    const isRemoteMode = this.isFleetRobotsMode();
    if (!isRemoteMode && (!this.fleetNameEdited || this.robotNameExists(this.fleetRobotNameInput.value, robots))) {
      this.fleetRobotNameInput.value = this.nextFleetRobotName(robots);
      this.fleetNameEdited = false;
    }
    this.renderFleetRobotList(robots);
  }

  syncFleetRemoteFields() {
    const isRemoteMode = this.isFleetRobotsMode();
    if (this.fleetRobotNameLabel) {
      this.fleetRobotNameLabel.classList.toggle("hidden", false);
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
    if (this.fleetRobotNameInput) {
      this.fleetRobotNameInput.placeholder = isRemoteMode ? "robot-3" : "";
    }
    if (this.fleetSpawnLmLabelText) {
      this.fleetSpawnLmLabelText.textContent = "Start LM";
    }
  }

  syncFleetSimTools() {
    const visible = this.isFleetManagerSim();
    this.fleetSimTools?.classList.toggle("hidden", !visible);
    if (this.fleetPlaceRobotButton) {
      this.fleetPlaceRobotButton.classList.toggle("hidden", !visible);
    }
    if (visible) {
      this.syncFleetSimMapLoadButton();
      this.refreshFleetSimMapSelect({ quiet: true }).catch(() => {});
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
    const isRemoteMode = this.isFleetRobotsMode();
    const current = String(selectedValue || "");
    this.fleetSpawnLmSelect.innerHTML = "";
    if (isRemoteMode) {
      return;
    }
    if (!values.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "No LM loaded";
      this.fleetSpawnLmSelect.append(option);
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

  async refreshFleetSimMapSelect(options = {}) {
    if (!this.isFleetManagerSim() || !this.fleetSimMapSelect || this.fleetSimMapsLoading) {
      return;
    }
    const force = Boolean(options.force);
    const quiet = Boolean(options.quiet);
    const now = Date.now();
    if (!force && this.fleetSimMapsLoadedAt && now - this.fleetSimMapsLoadedAt < 30000 && this.fleetSimMapSelect.options.length) {
      return;
    }
    this.fleetSimMapsLoading = true;
    try {
      const payload = await this.getJson(this.fleetApiPath("/maps/list"));
      const maps = Array.isArray(payload.maps) ? payload.maps : [];
      const active = String(payload.active || this.currentStatus?.mapName || "");
      const previous = String(this.fleetSimMapSelect.value || "");
      this.fleetSimMapSelect.innerHTML = "";
      for (const item of maps) {
        const name = String(item.name || item.folder || "").replace(/\.smap$/, "");
        if (!name) {
          continue;
        }
        const option = document.createElement("option");
        option.value = name;
        option.textContent = item.active ? `${name} (active)` : name;
        option.selected = name === previous || (!previous && (item.active || name === active));
        this.fleetSimMapSelect.append(option);
      }
      this.fleetSimMapsLoadedAt = Date.now();
      this.syncFleetSimMapLoadButton();
      if (this.fleetBenchmarkStatus && !quiet) {
        this.fleetBenchmarkStatus.className = "probe-result success compact";
        this.fleetBenchmarkStatus.textContent = `Maps refreshed: ${this.fleetSimMapSelect.options.length}.`;
      }
    } catch (error) {
      if (this.fleetBenchmarkStatus && !quiet) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = error.message || String(error);
      }
    } finally {
      this.fleetSimMapsLoading = false;
    }
  }

  syncFleetSimMapLoadButton() {
    if (!this.fleetSimLoadMapButton || !this.fleetSimMapSelect) {
      return;
    }
    const selected = String(this.fleetSimMapSelect.value || "").replace(/\.smap$/, "");
    const active = String(this.currentStatus?.mapName || this.robotMapState.robotActiveMapName || "").replace(/\.smap$/, "");
    const alreadyActive = Boolean(selected && active && selected === active);
    this.fleetSimLoadMapButton.disabled = !selected || alreadyActive;
    this.fleetSimLoadMapButton.textContent = alreadyActive ? "Active" : "Use";
  }

  async handleFleetSimLoadMap() {
    if (!this.isFleetManagerSim() || !this.fleetSimMapSelect) {
      return;
    }
    const robot = this.selectedRobot();
    const mapName = String(this.fleetSimMapSelect.value || "").trim();
    if (!mapName) {
      return;
    }
    const activeMap = String(this.currentStatus?.mapName || this.robotMapState.robotActiveMapName || "").replace(/\.smap$/, "");
    if (activeMap && mapName.replace(/\.smap$/, "") === activeMap) {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result neutral compact";
        this.fleetBenchmarkStatus.textContent = `${mapName} is already active.`;
      }
      return;
    }
    try {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result neutral compact";
        this.fleetBenchmarkStatus.textContent = `Loading ${mapName}...`;
      }
      const result = await this.runMapTransfer(`Load ${mapName}`, async (progress) => {
        await progress(12, `Preparing ${mapName}...`, 80);
        this.beginRobotMapTransition(`Loading sim map ${mapName}...`);
        const loaded = await this.postJson(this.fleetApiPath("/maps/load"), { mapName });
        await progress(60, "Refreshing map state...", 80);
        return loaded;
      });
      this.invalidateOperatorScene3d();
      this.applyLoadedMapResult(result, mapName, robot);
      this.currentStatus = await this.getJson(this.fleetApiPath("/state")).catch(() => this.currentStatus);
      this.renderFleetStateImmediately();
      this.refreshAfterMapLoadInBackground();
      this.refreshFleetSimMapSelect({ force: true, quiet: true }).catch(() => {});
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result success compact";
        this.fleetBenchmarkStatus.textContent = `Loaded ${result.mapName || mapName}.`;
      }
    } catch (error) {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = error.message || String(error);
      }
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
    const debug = this.lastFleetPlanDebug || {};
    const benchmark = debug.benchmark || {};
    const details = [
      `${selected.name}: ${selected.status || "IDLE"}`,
      reason,
      nodes,
      benchmark.planned !== undefined && benchmark.count !== undefined
        ? `benchmark ${benchmark.planned}/${benchmark.count} in ${Number(benchmark.elapsedMs || 0).toFixed(0)} ms`
        : "",
      debug.plannerBackend || benchmark.plannerBackend ? `backend ${debug.plannerBackend || benchmark.plannerBackend}` : "",
      debug.deadlock ? `deadlock: ${debug.deadlockReason || "robots holding position"}` : "",
      debug.rejectedPlanCount ? `rejected ${debug.rejectedPlanCount}` : "",
      debug.reason || benchmark.reason || "",
      debug.continuousUnresolved ? `unresolved ${debug.continuousUnresolved}` : "",
    ].filter(Boolean);
    this.fleetPlanDebug.textContent = details.join(" | ");
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
      const result = await this.postJson(this.fleetApiPath("/orders/clear"), { includeActive: false });
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      const canceled = Number(result.canceled || 0);
      this.robotMessageText.textContent = (draftCount || canceled)
        ? `Queue cleared: draft=${draftCount}, backend=${canceled}.`
        : "Queue is empty.";
      this.renderFleetStateImmediately();
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
      const result = await this.runMapTransfer("Dispatch Orders", async (progress) => {
        await progress(15, "Dispatching queued orders...", 80);
        const dispatched = await this.postJson(this.fleetApiPath("/orders/dispatch"), {});
        await progress(78, "Refreshing fleet state...", 80);
        return dispatched;
      });
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      const dispatched = Number(result.dispatched || 0);
      this.robotMessageText.textContent = dispatched ? `Orders dispatched: ${dispatched}.` : "No dispatchable orders right now.";
      this.renderFleetStateImmediately();
    } catch (error) {
      this.robotMessageText.textContent = `Dispatch failed: ${error.message || error}`;
    }
  }

  async dispatchDraftFleetQueue(options = {}) {
    const progress = options.progress;
    if (!progress) {
      return this.runMapTransfer("Dispatch Queue", async (report) => this.dispatchDraftFleetQueue({ progress: report }));
    }
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
      await progress(8, "Preparing draft queue...", 50);
      const totalGroups = Math.max(1, groups.size);
      let groupIndex = 0;
      for (const [robotName, targets] of groups.entries()) {
        if (!targets.length) {
          continue;
        }
        groupIndex += 1;
        await progress(12 + Math.round((groupIndex / totalGroups) * 55), `Planning ${robotName} queue...`, 30);
        const result = await this.postJson(this.fleetApiPath("/setOrder"), {
          id: this.nextFleetOrderId(robotName),
          vehicle: robotName,
          priority: 10,
          targets,
          ...this.fleetMotionParams(),
        });
        sent += targets.length;
        sentRobots.add(robotName);
        lastState = result.state || lastState;
      }
      await progress(82, "Refreshing queue state...", 80);
      this.fleetQueue = [];
      this.navigateMode = false;
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.currentStatus = lastState || await this.getJson(this.fleetApiPath("/state"));
      this.robotMessageText.textContent = `Dispatched draft queue: ${sent} LM goal(s).`;
      this.renderFleetStateImmediately();
    } catch (error) {
      if (sentRobots.size) {
        this.fleetQueue = this.fleetQueue.filter((entry) => !sentRobots.has(entry.robotName));
      }
      if (lastState) {
        this.currentStatus = lastState;
      }
      this.robotMessageText.textContent = `Dispatch failed: ${error.message || error}`;
      this.renderFleetStateImmediately();
    }
  }

  async cancelFleetOrder(orderId) {
    if (!orderId || orderId === "-") {
      return;
    }
    try {
      const result = await this.postJson(this.fleetApiPath("/orders/cancel"), { id: orderId });
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.robotMessageText.textContent = `Order canceled: ${orderId}.`;
      this.renderFleetStateImmediately();
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
      const result = await this.postJson(this.fleetApiPath("/orders/pause"), { id: orderId });
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.robotMessageText.textContent = `Order paused: ${orderId}.`;
      this.renderFleetStateImmediately();
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
      const result = await this.postJson(this.fleetApiPath("/orders/resume"), { id: orderId });
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.robotMessageText.textContent = `Order resumed: ${orderId}.`;
      this.renderFleetStateImmediately();
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
      if (this.operatorObstacleLayer) {
        this.operatorObstacleLayer.innerHTML = "";
      }
      this.operatorGraphLayer.innerHTML = "";
      this.operatorRouteLayer.innerHTML = "";
      this.operatorLookaheadLayer.innerHTML = "";
      this.operatorLandmarkLayer.innerHTML = "";
      this.operatorEditorLayer.innerHTML = "";
      if (this.operatorScanLayer) {
        this.operatorScanLayer.innerHTML = "";
      }
      this.operatorRobotLayer.innerHTML = "";
      this.clearRelocationPreview();
      this.syncMapControls();
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
    this.operatorGraphLayer.innerHTML = "";
    const profile = this.mapVisualProfile(payload);
    const strokeWidth = profile.unit(profile.massive ? 0.55 : (profile.dense ? 0.75 : 1.05));
    if (!this.fleetMapEditorActive && profile.dense) {
      this.drawGraphBulk(payload, landmarks, strokeWidth);
      return;
    }
    for (const edge of payload.edges || []) {
      const element = document.createElementNS("http://www.w3.org/2000/svg", edge.geometry === "bezier" ? "path" : "line");
      const edgeKey = this.edgeKey(edge.from, edge.to);
      element.setAttribute("class", [
        "graph-edge",
        this.fleetMapEditorActive ? "editable" : "",
        edgeKey === this.fleetSelectedEdgeKey ? "selected" : "",
      ].filter(Boolean).join(" "));
      element.style.strokeWidth = String(this.fleetMapEditorActive ? profile.unit(1.8) : strokeWidth);
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
      const arrow = profile.edges > 900 ? null : this.directionArrow(edge, landmarks);
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
      this.appendRoutePolyline(preview, "fleet-route-preview");
    }
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
    polyline.style.strokeWidth = String(this.routeStrokeWidth(className));
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
    if (String(className).includes("active")) {
      return profile.unit(profile.massive ? 1.8 : 2.2);
    }
    if (String(className).includes("done") || String(className).includes("plan")) {
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
    for (const landmark of payload.lms || []) {
      const px = this.worldToPixel(landmark);
      const isNearest = landmark.name === nearest;
      const isTarget = landmark.name === target;
      const isSelected = this.fleetMapEditorActive && landmark.name === this.fleetSelectedLmName;
      const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
      group.setAttribute("class", [
        "landmark",
        style.compact ? "compact" : "",
        isNearest ? "nearest" : "",
        isTarget ? "target" : "",
        isSelected ? "selected" : "",
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
      const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
      label.setAttribute("class", "landmark-label");
      label.setAttribute("x", String(px.x));
      label.setAttribute("y", String(px.y + style.labelOffset));
      label.setAttribute("font-size", String(style.labelFontSize));
      label.style.strokeWidth = String(style.labelStrokeWidth);
      label.textContent = landmark.name;
      group.append(label);
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

  drawRobot() {
    this.operatorRobotLayer.innerHTML = "";
    const robotStyle = this.robotRenderStyle();
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
        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("class", "robot-label");
        label.setAttribute("x", String(center.x));
        label.setAttribute("y", String(center.y + robotStyle.labelOffset));
        label.style.fontSize = String(robotStyle.labelFontSize);
        label.style.strokeWidth = String(robotStyle.labelStrokeWidth);
        label.textContent = robot.name || "";
        group.append(label);
        this.operatorRobotLayer.append(group);
      }
      return;
    }
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
    const model = (!this.isFleetManager() ? this.robotParams?.robot_model : null)
      || this.fleetParams?.robot_model
      || this.fleetModelEditor?.getModel()
      || {};
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
        x: Number(pose.x || 0) + (Number(point.x || 0) * cos) + (Number(point.y || 0) * sin),
        y: Number(pose.y || 0) + (Number(point.x || 0) * sin) - (Number(point.y || 0) * cos),
      };
      const pixel = this.worldToPixel(world);
      return `${pixel.x},${pixel.y}`;
    }).join(" ");
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
    const arrowLength = this.screenPxToMapUnits(5.5);
    const arrowWidth = this.screenPxToMapUnits(3.2);
    const tip = { x: point.x + ux * arrowLength, y: point.y + uy * arrowLength };
    const base = { x: point.x - ux * arrowLength, y: point.y - uy * arrowLength };
    const left = { x: base.x + px * arrowWidth, y: base.y + py * arrowWidth };
    const right = { x: base.x - px * arrowWidth, y: base.y - py * arrowWidth };
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
    if ((!this.navigateMode && !this.relocateMode) || event.target.closest(".landmark")) {
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
      await this.postJson(this.fleetApiPath("/maps/local/save"), {
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
    this.refreshAdaptiveMapLayers();
    this.syncMapControls();
  }

  resetMapView(keepFollow = false) {
    this.mapView.scale = 1;
    this.mapView.tx = 0;
    this.mapView.ty = 0;
    this.mapView.follow = keepFollow ? this.mapView.follow : false;
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
    const canUse3d = this.isFleetManager() && !this.isRobotModelPage() && !this.isParamsPage();
    const show3d = canUse3d && this.mapViewMode === "3d";
    this.operatorMapSvg?.classList.toggle("hidden", show3d);
    this.operatorScene3d?.classList.toggle("hidden", !show3d);
    this.operatorMap2dButton?.classList.toggle("active", !show3d);
    this.operatorMap3dButton?.classList.toggle("active", show3d);
    this.operatorMap3dButton?.classList.toggle("hidden", !canUse3d);
    if (show3d) {
      this.refreshOperatorScene3d();
    }
    this.operatorFollowRobotButton.classList.toggle("primary", this.mapView.follow);
    this.operatorFollowRobotButton.textContent = this.mapView.follow ? "Following Robot" : "Follow Robot";
  }

  setMapViewMode(mode) {
    const nextMode = mode === "3d" ? "3d" : "2d";
    if (nextMode === "3d" && !this.isFleetManager()) {
      this.robotMessageText.textContent = "3D view is available in Fleet Manager and Fleet Manager Sim.";
      return;
    }
    this.mapViewMode = nextMode;
    window.localStorage.setItem("operator:mapViewMode", this.mapViewMode);
    this.syncMapControls();
    this.renderOperatorMap();
  }

  async ensureScene3d() {
    if (this.scene3d) {
      return this.scene3d;
    }
    if (!this.scene3dModulePromise) {
      this.scene3dModulePromise = import("./scene3d.js");
    }
    const module = await this.scene3dModulePromise;
    this.scene3d = new module.OperatorScene3D(this.operatorScene3d);
    this.scene3d.setHandlers({
      onFloorClick: (world) => this.handleScene3dFloorClick(world),
      onLandmarkHover: (lmName) => this.handleScene3dLandmarkHover(lmName),
      onRobotClick: (robotName) => this.selectFleetRobotByName(robotName),
    });
    this.scene3d.setTargetArmed(this.scene3dTargetArmed());
    return this.scene3d;
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

  normalizeScene3dMapName(mapName) {
    return String(mapName || "").trim().replace(/\.smap$/i, "");
  }

  invalidateOperatorScene3d() {
    this.scene3dStaticKey = "";
    this.scene3dPayload = null;
    this.scene3dRenderQueued = true;
    if (this.mapViewMode === "3d") {
      this.renderOperatorScene3d();
    }
  }

  refreshOperatorScene3d() {
    if (this.mapViewMode !== "3d") {
      return;
    }
    if (!this.updateOperatorScene3dRobots()) {
      this.renderOperatorScene3d();
    }
  }

  updateOperatorScene3dRobots(scene = this.scene3d) {
    if (!this.isFleetManager() || this.mapViewMode !== "3d" || !scene) {
      return false;
    }
    if (this.scene3dStaticKey !== this.scene3dKey()) {
      return false;
    }
    scene.setTargetArmed(this.scene3dTargetArmed());
    const robots = this.fleetRenderRobots();
    const selectedName = this.selectedFleetRobot(robots)?.name || this.selectedFleetRobotName || "";
    scene.updateRobots(robots, selectedName);
    return true;
  }

  renderOperatorScene3d() {
    if (!this.isFleetManager() || this.mapViewMode !== "3d" || !this.operatorScene3d) {
      return;
    }
    if (this.scene3dLoadPending) {
      this.scene3dRenderQueued = true;
      return;
    }
    this.ensureScene3d()
      .then(async (scene) => {
        scene.setTargetArmed(this.scene3dTargetArmed());
        const key = this.scene3dKey();
        if (this.scene3dStaticKey !== key) {
          this.scene3dLoadPending = true;
          this.scene3dRenderQueued = false;
          this.scene3dPayload = null;
          try {
            const payload = await this.getJson(this.fleetApiPath("/scene3d"));
            if (this.mapViewMode !== "3d" || !this.isFleetManager()) {
              return;
            }
            const payloadMapName = this.normalizeScene3dMapName(payload?.mapName);
            const currentMapName = this.normalizeScene3dMapName(this.currentStatus?.mapName || this.robotMapState.robotActiveMapName || this.robotMapState.operatorActiveMapName);
            if (payloadMapName && currentMapName && payloadMapName !== currentMapName) {
              this.scene3dRenderQueued = true;
            } else {
              this.scene3dPayload = payload;
              scene.setScene(payload);
              this.scene3dStaticKey = this.scene3dKey();
            }
          } finally {
            this.scene3dLoadPending = false;
          }
        }
        const updated = this.updateOperatorScene3dRobots(scene);
        const needsAnotherPass = this.scene3dRenderQueued || !updated || this.scene3dStaticKey !== this.scene3dKey();
        this.scene3dRenderQueued = false;
        if (needsAnotherPass && this.mapViewMode === "3d" && typeof window.requestAnimationFrame === "function") {
          window.requestAnimationFrame(() => this.renderOperatorScene3d());
        }
        this.ensureFleetAnimationLoop();
      })
      .catch((error) => {
        this.scene3dLoadPending = false;
        this.scene3dRenderQueued = false;
        this.robotMessageText.textContent = `3D view failed: ${error.message || error}`;
      });
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
    return this.isFleetRemoteRobot(this.targetFleetRobot());
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

  fleetMotionParams() {
    return {
      speed: this.fleetRouteSpeed(),
      acceleration: Math.max(0.0, Number(this.fleetRouteAccelerationInput?.value || 0.0) || 0.0),
      rotate: Boolean(this.fleetRotateInput?.checked),
      turnSpeed: Math.max(0.05, Number(this.fleetTurnSpeedInput?.value || 0.9) || 0.9),
      stretchMotionToReservationTicks: true,
    };
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
    const payload = await this.getJson(this.fleetApiPath("/params"));
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

  cloneJson(value) {
    return JSON.parse(JSON.stringify(value || {}));
  }

  getParamPath(source, path) {
    const parts = String(path || "").split(".").filter(Boolean);
    let current = source;
    for (const part of parts) {
      if (!current || typeof current !== "object" || !(part in current)) {
        return undefined;
      }
      current = current[part];
    }
    return current;
  }

  setParamPath(target, path, value) {
    const parts = String(path || "").split(".").filter(Boolean);
    if (!parts.length) {
      return;
    }
    let current = target;
    for (let index = 0; index < parts.length - 1; index += 1) {
      const part = parts[index];
      if (!current[part] || typeof current[part] !== "object" || Array.isArray(current[part])) {
        current[part] = {};
      }
      current = current[part];
    }
    current[parts[parts.length - 1]] = value;
  }

  normalizeRobotParamValue(field, rawValue) {
    if (field.type === "boolean") {
      return Boolean(rawValue);
    }
    if (field.type === "integer") {
      const parsed = Number.parseInt(String(rawValue), 10);
      if (!Number.isFinite(parsed)) {
        return Number(field.default || 0);
      }
      return parsed;
    }
    if (field.type === "number") {
      const parsed = Number.parseFloat(String(rawValue));
      if (!Number.isFinite(parsed)) {
        return Number(field.default || 0);
      }
      return parsed;
    }
    return String(rawValue ?? "");
  }

  robotParamEquals(a, b) {
    if (typeof a === "number" || typeof b === "number") {
      return Math.abs(Number(a || 0) - Number(b || 0)) < 0.000001;
    }
    return JSON.stringify(a) === JSON.stringify(b);
  }

  defaultRobotParams() {
    const params = {};
    for (const field of ROBOT_PARAM_SCHEMA) {
      this.setParamPath(params, field.path, field.default);
    }
    if (this.fleetModelEditor) {
      params.robot_model = {
        ...(params.robot_model || {}),
        ...this.fleetModelEditor.defaultModel(),
        source: this.getParamPath(params, "robot_model.source") || "nav2",
        radius: this.getParamPath(params, "robot_model.radius") ?? 0.22,
        footprint_segments: this.getParamPath(params, "robot_model.footprint_segments") ?? 16,
      };
    }
    return params;
  }

  renderRobotParamsTable() {
    if (!this.robotParamsTable) {
      return;
    }
    const params = this.robotParams || {};
    this.robotParamsTable.innerHTML = "";
    let currentGroup = "";
    let changedCount = 0;

    for (const field of ROBOT_PARAM_SCHEMA) {
      if (field.group !== currentGroup) {
        currentGroup = field.group;
        const group = document.createElement("div");
        group.className = "robot-param-group";
        group.textContent = currentGroup;
        this.robotParamsTable.append(group);
      }

      const value = this.getParamPath(params, field.path);
      const displayValue = value === undefined ? field.default : value;
      const dirty = !this.robotParamEquals(displayValue, field.default);
      if (dirty) {
        changedCount += 1;
      }

      const row = document.createElement("div");
      row.className = `robot-param-row${dirty ? " dirty" : ""}`;
      row.dataset.paramPath = field.path;

      const nameCell = document.createElement("div");
      nameCell.className = "robot-param-name";
      const label = document.createElement("div");
      label.className = "robot-param-label";
      label.textContent = field.label;
      const path = document.createElement("div");
      path.className = "robot-param-path";
      path.textContent = `${field.section} / ${field.path}`;
      nameCell.append(label, path);

      const description = document.createElement("div");
      description.className = "robot-param-description";
      description.textContent = field.description;

      const valueCell = document.createElement("div");
      valueCell.className = "robot-param-value";
      const input = this.createRobotParamInput(field, displayValue);
      const defaultText = document.createElement("div");
      defaultText.className = "robot-param-default";
      defaultText.textContent = `default: ${this.robotParamDisplay(field.default)}${field.unit ? ` ${field.unit}` : ""}`;
      const resetButton = document.createElement("button");
      resetButton.type = "button";
      resetButton.className = "robot-param-reset";
      resetButton.textContent = "Default";
      resetButton.addEventListener("click", () => {
        this.setParamPath(this.robotParams, field.path, field.default);
        this.renderRobotParamsTable();
        this.syncRobotParamsJson(true);
        this.robotMessageText.textContent = `${field.label} reset to default.`;
      });
      valueCell.append(input, defaultText, resetButton);
      row.append(nameCell, description, valueCell);
      this.robotParamsTable.append(row);
    }
    this.updateRobotParamsSummary(changedCount);
  }

  createRobotParamInput(field, value) {
    let input;
    if (field.type === "select") {
      input = document.createElement("select");
      for (const [optionValue, optionLabel] of field.options || []) {
        const option = document.createElement("option");
        option.value = String(optionValue);
        option.textContent = String(optionLabel);
        input.append(option);
      }
      input.value = String(value ?? field.default ?? "");
    } else if (field.type === "boolean") {
      input = document.createElement("input");
      input.type = "checkbox";
      input.checked = Boolean(value);
    } else {
      input = document.createElement("input");
      input.type = field.type === "integer" || field.type === "number" ? "number" : "text";
      input.value = String(value ?? field.default ?? "");
      if (field.min !== undefined) {
        input.min = String(field.min);
      }
      if (field.max !== undefined) {
        input.max = String(field.max);
      }
      if (field.step !== undefined) {
        input.step = String(field.step);
      }
    }
    input.dataset.paramPath = field.path;
    input.dataset.paramType = field.type;
    input.addEventListener("input", () => this.handleRobotParamInput(field, input));
    input.addEventListener("change", () => this.handleRobotParamInput(field, input));
    return input;
  }

  handleRobotParamInput(field, input) {
    this.robotParams = this.robotParams || {};
    const rawValue = field.type === "boolean" ? input.checked : input.value;
    const value = this.normalizeRobotParamValue(field, rawValue);
    this.setParamPath(this.robotParams, field.path, value);
    const row = input.closest(".robot-param-row");
    const dirty = !this.robotParamEquals(value, field.default);
    if (row) {
      row.classList.toggle("dirty", dirty);
    }
    this.syncRobotParamsJson(true);
    this.updateRobotParamsSummary();
  }

  updateRobotParamsSummary(changedCount = null) {
    if (!this.robotParamsSummary) {
      return;
    }
    const count = changedCount === null
      ? ROBOT_PARAM_SCHEMA.filter((field) => {
        const value = this.getParamPath(this.robotParams || {}, field.path);
        return !this.robotParamEquals(value === undefined ? field.default : value, field.default);
      }).length
      : changedCount;
    const total = ROBOT_PARAM_SCHEMA.length;
    this.robotParamsSummary.textContent = count
      ? `${count} of ${total} parameters differ from default. Save writes params.yaml on the robot and applies changes immediately.`
      : `${total} robot parameters are at default values. Save writes params.yaml on the robot and applies changes immediately.`;
  }

  robotParamDisplay(value) {
    if (typeof value === "boolean") {
      return value ? "true" : "false";
    }
    if (typeof value === "number") {
      return Number.isInteger(value) ? String(value) : String(Math.round(value * 1000000) / 1000000);
    }
    return String(value ?? "");
  }

  collectRobotParamsFromTable() {
    const params = this.cloneJson(this.robotParams || {});
    if (!this.robotParamsTable) {
      return params;
    }
    for (const field of ROBOT_PARAM_SCHEMA) {
      const input = this.robotParamsTable.querySelector(`.robot-param-value [data-param-path="${CSS.escape(field.path)}"]`);
      if (!input) {
        continue;
      }
      const rawValue = field.type === "boolean" ? input.checked : input.value;
      this.setParamPath(params, field.path, this.normalizeRobotParamValue(field, rawValue));
    }
    return params;
  }

  resetRobotParamsToDefaults() {
    this.robotParams = this.defaultRobotParams();
    if (this.fleetModelEditor && this.robotParams.robot_model) {
      this.fleetModelEditor.setModel(this.robotParams.robot_model);
    }
    this.renderRobotParamsTable();
    this.syncRobotParamsJson(true);
    this.robotMessageText.textContent = "Robot params reset to defaults. Press Save Robot Params to apply.";
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
    if (this.fleetRouteAccelerationInput && navigation.route_acceleration !== undefined) {
      this.fleetRouteAccelerationInput.value = String(navigation.route_acceleration);
    }
    if (this.fleetRotateInput && navigation.simulate_rotation !== undefined) {
      this.fleetRotateInput.checked = Boolean(navigation.simulate_rotation);
    }
    if (this.fleetTurnSpeedInput && navigation.turn_speed !== undefined) {
      this.fleetTurnSpeedInput.value = String(navigation.turn_speed);
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
    this.renderRobotParamsTable();
    this.syncRobotParamsJson();
  }

  collectFleetParams() {
    const params = this.parseParamsJson(this.fleetParamsJsonInput, "Fleet params", this.fleetParams || {});
    const motion = this.fleetMotionParams();
    params.navigation = {
      ...(params.navigation || {}),
      route_speed: motion.speed,
      route_acceleration: motion.acceleration,
      simulate_rotation: motion.rotate,
      turn_speed: motion.turnSpeed,
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
    const params = this.collectRobotParamsFromTable();
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
      const result = await this.postJson(this.fleetApiPath("/params"), { params });
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
      const result = await this.postJson(this.fleetApiPath("/params"), { params });
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
      const params = this.collectRobotParams();
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/params`, { params });
      this.robotParams = result.params || result.saved?.params || params;
      this.robotParamsRobotId = robot.id;
      this.robotParamsLoaded = true;
      this.applyRobotParams(this.robotParams);
      this.syncRobotParamsJson(true);
      this.robotMessageText.textContent = result.warning
        ? `Robot params saved with warning: ${result.warning}`
        : "Robot params saved and applied.";
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
      this.applyRobotParams(this.robotParams);
      this.robotMessageText.textContent = result.warning
        ? `Robot model saved with warning: ${result.warning}`
        : "Robot model saved and applied.";
    } catch (error) {
      this.robotMessageText.textContent = `Save robot model failed: ${error.message || error}`;
    }
  }

  toggleFleetSpawnMode() {
    if (!this.isFleetManagerSim()) {
      return;
    }
    if (!this.hasNavigationMapPayload()) {
      this.robotMessageText.textContent = "Load a sim map before placing robots.";
      return;
    }
    if (this.navigateMode && this.pendingFleetAction === "spawn") {
      this.navigateMode = false;
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.robotMessageText.textContent = "Place robot canceled.";
    } else {
      this.navigateMode = true;
      this.pendingFleetAction = "spawn";
      this.pendingFleetRobotName = "";
      this.robotMessageText.textContent = "Place robot armed: click an LM on the map.";
    }
    this.relocateMode = false;
    this.syncModeButtons();
    this.drawLandmarks();
  }

  async spawnFleetRobotAtLm(lmName) {
    const name = String(lmName || "").trim();
    if (!name) {
      return;
    }
    this.navigateMode = false;
    this.pendingFleetAction = "";
    this.pendingFleetRobotName = "";
    if (this.fleetSpawnLmSelect) {
      const option = Array.from(this.fleetSpawnLmSelect.options).find((item) => item.value === name);
      if (!option) {
        const added = document.createElement("option");
        added.value = name;
        added.textContent = name;
        this.fleetSpawnLmSelect.append(added);
      }
      this.fleetSpawnLmSelect.value = name;
    }
    this.syncModeButtons();
    this.drawLandmarks();
    await this.handleFleetAddRobot(name);
  }

  setFleetBenchmarkBusy(busy) {
    for (const button of this.fleetBenchmarkButtons || []) {
      button.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkClearButton) {
      this.fleetBenchmarkClearButton.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkPlanButton) {
      this.fleetBenchmarkPlanButton.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkHorizonInput) {
      this.fleetBenchmarkHorizonInput.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkIntervalInput) {
      this.fleetBenchmarkIntervalInput.disabled = Boolean(busy);
    }
    if (this.fleetSimLoadMapButton) {
      this.fleetSimLoadMapButton.disabled = Boolean(busy) || this.fleetSimLoadMapButton.textContent === "Active";
    }
    if (this.fleetBenchmarkRefreshMapsButton) {
      this.fleetBenchmarkRefreshMapsButton.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkOpenLoadButton) {
      this.fleetBenchmarkOpenLoadButton.disabled = Boolean(busy);
    }
  }

  fleetBenchmarkSummary(result, robotCount) {
    const benchmark = result?.benchmark || result?.state?.benchmark || {};
    const debug = result?.debug || {};
    const planned = Number(benchmark.planned ?? (Array.isArray(result?.plans) ? result.plans.length : 0));
    const elapsed = Number(benchmark.elapsedMs || 0).toFixed(0);
    const backend = benchmark.plannerBackend || debug.plannerBackend || "-";
    const reason = benchmark.reason || debug.reason || "";
    const conflicts = Number(debug.continuousConflicts || debug.batchContinuousConflicts || 0);
    const waits = Number(debug.continuousWaits || debug.batchContinuousWaits || 0);
    const unresolved = Number(debug.continuousUnresolved || 0);
    const deadlock = Boolean(debug.deadlock || unresolved);
    const plannedWaiting = Number(benchmark.plannedWaitingRobots || 0);
    const plannedWaitSec = Number(benchmark.plannedWaitSec || 0);
    const priorityRepairs = Number(benchmark.resolvedPriorityConflicts || 0);
    const averageSteps = Number(benchmark.averageRouteSteps || 0);
    const scenario = String(benchmark.scenario || "");
    if (scenario === "continuous_random_orders") {
      const active = Boolean(benchmark.active);
      const generated = Number(benchmark.ordersGenerated || 0);
      const completed = Number(benchmark.ordersCompleted || 0);
      const queued = Number(benchmark.ordersQueued || 0);
      const executing = Number(benchmark.ordersExecuting || 0);
      const waitingRobots = Number(benchmark.waitingRobots || 0);
      const cycles = Number(benchmark.waitCyclesResolved || 0);
      const safetyRollbacks = Number(benchmark.runtimeSafetyRollbacks || 0);
      const averageDistance = Number(benchmark.averageOrderDistanceM || 0);
      const horizon = Number(benchmark.horizonSec || 0);
      return [
        active ? "dynamic orders active" : "dynamic orders stopped",
        `${robotCount} robots`,
        `horizon ${horizon.toFixed(1)} s`,
        `orders ${generated} generated / ${completed} completed`,
        averageDistance ? `avg goal ${averageDistance.toFixed(1)} m` : "",
        `executing ${executing} / queued ${queued}`,
        waitingRobots ? `waiting ${waitingRobots}` : "",
        cycles ? `deadlocks resolved ${cycles}` : "",
        safetyRollbacks ? `safety rollbacks ${safetyRollbacks}` : "",
      ].filter(Boolean).join(" | ");
    }
    const details = [
      `${planned}/${robotCount} planned`,
      `${elapsed} ms`,
      `backend ${backend}`,
      scenario === "traffic_stress" ? "traffic stress" : scenario === "balanced_fallback" ? "safe fallback" : "",
      averageSteps ? `avg route ${averageSteps.toFixed(1)} edges` : "",
      plannedWaiting ? `waiting ${plannedWaiting} robots / ${plannedWaitSec.toFixed(0)} s` : "",
      priorityRepairs ? `priority cycles resolved ${priorityRepairs}` : "",
      conflicts ? `conflicts ${conflicts}` : "",
      waits ? `waits ${waits}` : "",
      deadlock ? "deadlock: robots holding position" : "",
      unresolved ? `unresolved ${unresolved}` : "",
      reason,
    ].filter(Boolean);
    return details.join(" | ");
  }

  async clearFleetSimulation(options = {}) {
    if (!this.isFleetManagerSim()) {
      return null;
    }
    const progress = options.progress;
    if (!progress) {
      return this.runMapTransfer("Clear Sim", async (report) => this.clearFleetSimulation({ progress: report }));
    }
    await progress(8, "Stopping simulated robots...", 60);
    const result = await this.postJson(this.fleetApiPath("/benchmark"), {
      count: 0,
      reset: true,
      seed: 42,
    });
    await progress(55, "Removing robots and queued orders...", 80);
    this.currentStatus = result.state || result.fleetState || await this.getJson(this.fleetApiPath("/state"));
    this.selectedFleetRobotName = "";
    this.pendingFleetRobotName = "";
    this.pendingFleetAction = "";
    this.navigateMode = false;
    this.fleetQueue = [];
    this.fleetManualRobotName = "";
    this.fleetManualAnimation = null;
    this.fleetManualLookahead = null;
    this.fleetVisualClocks.clear();
    this.lastFleetPlanDebug = result.benchmark || null;
    this.syncDynamicBenchmarkControls();
    this.invalidateOperatorScene3d();
    window.localStorage.removeItem("operator:selectedFleetRobotName");
    await progress(86, "Refreshing empty simulation...", 70);
    this.renderFleetStateImmediately();
    this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    if (this.fleetBenchmarkStatus) {
      this.fleetBenchmarkStatus.className = "probe-result success compact";
      this.fleetBenchmarkStatus.textContent = "Simulation cleared.";
    }
    return result;
  }

  async runFleetBenchmark(count) {
    if (!this.isFleetManagerSim()) {
      return;
    }
    const runId = ++this.fleetBenchmarkRunId;
    const robotCount = Math.max(0, Number(count || 0));
    this.setFleetBenchmarkBusy(true);
    if (this.fleetBenchmarkStatus) {
      this.fleetBenchmarkStatus.className = "probe-result neutral compact";
      this.fleetBenchmarkStatus.textContent = robotCount
        ? `Adding robots up to ${robotCount}...`
        : "Clearing simulation...";
    }
    try {
      if (robotCount <= 0) {
        await this.clearFleetSimulation();
        return;
      }
      const result = await this.runMapTransfer(`Add ${robotCount} Robots`, async (progress) => {
        await progress(12, `Checking current robot count...`, 50);
        if (runId !== this.fleetBenchmarkRunId) {
          throw new Error("Robot add superseded by a newer run.");
        }
        const added = await this.postJson(this.fleetApiPath("/benchmark"), {
          action: "add",
          count: robotCount,
          reset: false,
          seed: 42,
        });
        const benchmark = added.benchmark || added.state?.benchmark || {};
        await progress(78, `Robots ${benchmark.robots ?? robotCount}/${robotCount}; added ${benchmark.added ?? 0}.`, 100);
        return added;
      });
      this.currentStatus = result.state || result.fleetState || await this.getJson(this.fleetApiPath("/state"));
      this.lastFleetPlanDebug = result.benchmark || result.state?.benchmark || null;
      const robots = Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : [];
      if (robots.length && !robots.some((robot) => robot.name === this.selectedFleetRobotName)) {
        this.selectedFleetRobotName = robots[0].name || "";
        window.localStorage.setItem("operator:selectedFleetRobotName", this.selectedFleetRobotName);
      } else if (!robots.length) {
        this.selectedFleetRobotName = "";
        window.localStorage.removeItem("operator:selectedFleetRobotName");
      }
      const benchmark = result.benchmark || this.currentStatus?.benchmark || {};
      if (this.fleetBenchmarkStatus) {
        const total = Number(benchmark.robots ?? robots.length);
        const added = Number(benchmark.added ?? 0);
        this.fleetBenchmarkStatus.className = total >= robotCount ? "probe-result success compact" : "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = `Robots ${total}/${robotCount}; added ${added}. Start dynamic orders when ready.`;
      }
      this.renderFleetStateImmediately();
      this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    } catch (error) {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = error.message || String(error);
      }
    } finally {
      if (runId === this.fleetBenchmarkRunId) {
        this.setFleetBenchmarkBusy(false);
      }
    }
  }

  async planFleetBenchmarkRobots() {
    if (!this.isFleetManagerSim()) {
      return;
    }
    const runId = ++this.fleetBenchmarkRunId;
    if (!Array.isArray(this.currentStatus?.robots) || !this.currentStatus.robots.length) {
      this.currentStatus = await this.getJson(this.fleetApiPath("/state")).catch(() => this.currentStatus);
    }
    const robots = Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : [];
    const robotCount = robots.length;
    if (!robotCount) {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = "Add robots before planning.";
      }
      return;
    }
    const dynamicActive = Boolean(this.currentStatus?.dynamicBenchmark?.active);
    this.setFleetBenchmarkBusy(true);
    if (this.fleetBenchmarkStatus) {
      this.fleetBenchmarkStatus.className = "probe-result neutral compact";
      this.fleetBenchmarkStatus.textContent = dynamicActive
        ? "Stopping new dynamic orders..."
        : `Starting continuous orders for ${robotCount} robots...`;
    }
    try {
      await new Promise((resolve) => window.requestAnimationFrame(resolve));
      if (runId !== this.fleetBenchmarkRunId) {
        throw new Error("Plan superseded by a newer run.");
      }
      const result = await this.postJsonRaw(this.fleetApiPath("/benchmark"), {
        action: dynamicActive ? "stop" : "plan",
        count: robotCount,
        reset: false,
        seed: 42,
        horizonSec: Math.max(1, Number(this.fleetBenchmarkHorizonInput?.value || 10)),
        orderIntervalSec: Math.max(0.25, Number(this.fleetBenchmarkIntervalInput?.value || 3)),
        queueDepth: 2,
        ...this.fleetMotionParams(),
        fast: true,
      });
      this.currentStatus = result.state || result.fleetState || await this.getJson(this.fleetApiPath("/state"));
      this.lastFleetPlanDebug = {
        ...(result.debug || {}),
        benchmark: result.benchmark || {},
      };
      const benchmark = result.benchmark || this.currentStatus?.benchmark || {};
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result success compact";
        this.fleetBenchmarkStatus.textContent = this.fleetBenchmarkSummary(result, robotCount);
      }
      this.syncDynamicBenchmarkControls();
      this.renderFleetStateImmediately();
      this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    } catch (error) {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = error.message || String(error);
      }
    } finally {
      if (runId === this.fleetBenchmarkRunId) {
        this.setFleetBenchmarkBusy(false);
      }
    }
  }

  async handleFleetAddRobot(spawnLmOverride = "") {
    const requestedName = String(this.fleetRobotNameInput.value || "").trim();
    const spawnLm = String(spawnLmOverride || this.fleetSpawnLmSelect.value || "").trim();
    const mode = this.isFleetRobotsMode() ? "robots" : "simulation";
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
      window.alert("Robot IP is required for Fleet Manager.");
      return;
    }
    try {
      const payload = mode === "robots"
        ? { mode: "remote", name: requestedName, host: robotIp }
        : { name: requestedName, spawnLm, mode: "simulated" };
      const result = await this.postJson(this.fleetApiPath("/robots"), payload);
      const addedName = String(result.robot?.name || requestedName || "").trim();
      this.selectedFleetRobotName = addedName;
      if (addedName) {
        window.localStorage.setItem("operator:selectedFleetRobotName", addedName);
      }
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.fleetNameEdited = false;
      if (mode === "robots") {
        this.fleetRobotApiInput.value = "";
      } else {
        this.fleetRobotNameInput.value = this.nextFleetRobotName(Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []);
      }
      this.renderFleetStateImmediately();
      this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    } catch (error) {
      await this.refreshRobotMapState({ quiet: true }).catch(() => {});
      this.renderSelectedRobot();
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
      const result = await this.postJson(this.fleetApiPath("/robots/remove"), { name: robot.name });
      this.selectedFleetRobotName = "";
      window.localStorage.removeItem("operator:selectedFleetRobotName");
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.fleetNameEdited = false;
      this.renderFleetStateImmediately();
      this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async stopFleetRobot(all) {
    const robot = this.selectedFleetRobot();
    const payload = all || !robot ? {} : { name: robot.name };
    try {
      const result = await this.postJson(this.fleetApiPath("/robots/stop"), payload);
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.currentRoute = null;
      this.fleetManualLookahead = null;
      this.fleetManualRobotName = "";
      this.pendingFleetAction = "";
      this.pendingFleetRobotName = "";
      this.renderFleetStateImmediately();
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async cancelRoute() {
    this.navigateMode = false;
    this.relocateMode = false;
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
    this.relocateMode = false;
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
          this.sendRobotTeleop({ linear: 0, angular: 0 }, 80);
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
    if (!this.manualKeys.size || !this.selectedRobot()) {
      return;
    }
    const twist = this.manualTwist();
    if (Math.abs(twist.linear) < 0.0001 && Math.abs(twist.angular) < 0.0001) {
      return;
    }
    if (this.isFleetManager()) {
      if (this.teleopPending) {
        return;
      }
      this.teleopPending = true;
      try {
        await this.sendFleetManualStep(twist);
      } finally {
        this.teleopPending = false;
      }
      return;
    }
    this.sendRobotTeleop(twist, 350);
  }

  releaseManualControl() {
    this.manualKeys.clear();
    this.syncManualButtons();
    if (this.selectedRobot() && !this.isFleetManager()) {
      this.closeTeleopSocket(true);
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
      await this.postJson(this.fleetApiPath("/robots/stop"), { name: robot.name });
      this.fleetManualRobotName = robot.name;
      this.fleetManualLastAt = performance.now();
      this.currentStatus = await this.getJson(this.fleetApiPath("/state"));
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
    const result = await this.postJson(this.fleetApiPath("/manual-step"), {
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
    this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
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
    const result = await this.postJson(this.fleetApiPath("/manual-step"), {
      name: robot.name,
      linear: twist.linear,
      angular: twist.angular,
      timeoutMs: 350,
    });
    this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
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
        const result = await this.postJson(this.fleetApiPath("/manual-stop"), { name: robot.name });
        this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
        this.fleetStatusReceivedAt = performance.now();
        this.fleetStatusObjectRef = this.currentStatus;
        this.fleetManualRobotName = "";
        this.fleetManualLastAt = 0;
        this.fleetManualLookahead = null;
        this.fleetManualAnimation = null;
        this.renderFleetStateImmediately();
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
      const result = await this.postJson(this.fleetApiPath("/robots/update"), payload);
      this.currentStatus = result.state || await this.getJson(this.fleetApiPath("/state"));
    }
    this.fleetManualRobotName = "";
    this.fleetManualLastAt = 0;
    this.fleetManualLookahead = null;
    this.fleetManualAnimation = null;
    this.renderFleetStateImmediately();
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
    this.robotPortInput.value = "50051";
    this.showProbeResult("neutral", "Enter the robot IP and check the gRPC connection.");
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
      const online = result.probe.online ? "online" : "waiting for robot status";
      this.showProbeResult("success", `gRPC robot API ready for ${identity.robotId || "robot"} on map ${identity.mapId || "-"}. ${online}. State: ${status.state || "-"}`);
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
      this.closeScanStream();
      this.closeSlamStream();
      this.closeTeleopSocket(true);
      this.selectedRobotId = result.robot.id;
      window.localStorage.setItem("operator:selectedRobotId", this.selectedRobotId);
      this.closeSidebar();
      await this.refreshRobots({ quiet: true, probe: true });
      const warnings = Array.isArray(result.cache?.warnings) ? result.cache.warnings : [];
      const cachedMaps = Array.isArray(result.cache?.cachedMaps) ? result.cache.cachedMaps.length : 0;
      this.showProbeResult(
        warnings.length ? "neutral" : "success",
        warnings.length
          ? `Robot saved. Workspace created with ${cachedMaps} cached map(s); ${warnings.length} cache warning(s).`
          : `Robot saved. Workspace created with ${cachedMaps} cached map(s), params, and robot model.`,
      );
    } catch (error) {
      this.showProbeResult("error", error.message || String(error));
    }
  }

  async handleRemoveRobot(robot) {
    if (!robot) {
      return;
    }
    const confirmed = window.confirm(`Remove ${this.robotDisplayName(robot)} from the operator app?`);
    if (!confirmed) {
      return;
    }
    try {
      await this.deleteJson(`/api/robots/${encodeURIComponent(robot.id)}`);
      if (this.selectedRobotId === robot.id) {
        this.selectedRobotId = "";
        window.localStorage.removeItem("operator:selectedRobotId");
        this.closeScanStream();
        this.closeSlamStream();
        this.closeTeleopSocket(true);
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
    const robotName = this.robotDisplayName(robot);
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
    const title = kind === "push"
      ? "Push Map"
      : (kind === "slam" ? "Finish SLAM" : (kind === "pull" ? "Pull Map" : String(kind || "Operation")));
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
    if (!this.isFleetManager(robot) && this.slamActive) {
      this.robotMessageText.textContent = "Pull Map is disabled while 2D SLAM is active.";
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
        this.beginRobotMapTransition(`Pulling active ${target} map...`);
        const payload = this.isFleetManager(robot)
          ? await this.postJson(this.fleetApiPath("/maps/pull-sync"), {})
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
    if (!this.isFleetManager(robot) && this.slamActive) {
      this.robotMessageText.textContent = "Push Map is disabled while 2D SLAM is active.";
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
        this.beginRobotMapTransition(`Pushing local map to ${target}...`);
        const payload = this.isFleetManager(robot)
          ? await this.postJson(this.fleetApiPath("/maps/push-sync"), {})
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
    if (!this.isFleetManager(robot) && this.slamActive) {
      this.robotMessageText.textContent = "Load Map is disabled while 2D SLAM is active.";
      return;
    }
    try {
      const robotMaps = this.isFleetManager(robot)
        ? await this.getJson(this.fleetApiPath("/maps/list"))
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
    const selectedMap = this.pendingRobotMaps.find((item) => String(item.name || item.folder || "") === mapName);
    const activeName = String(this.robotMapState.robotActiveMapName || this.currentStatus?.mapName || "").replace(/\.smap$/, "");
    if ((selectedMap && selectedMap.active) || (activeName && mapName.replace(/\.smap$/, "") === activeName)) {
      this.loadMapHint.className = "probe-result neutral";
      this.loadMapHint.textContent = `${mapName} is already active.`;
      return;
    }
    try {
      const result = await this.runMapTransfer(`Load ${mapName}`, async (progress) => {
        await progress(10, `Preparing ${mapName}...`, 80);
        this.beginRobotMapTransition(`Loading map ${mapName}...`);
        let loaded = null;
        if (this.isFleetManager(robot)) {
          loaded = await this.postJson(this.fleetApiPath("/maps/load"), { mapName });
        } else {
          loaded = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/load`, { mapName });
        }
        await progress(68, "Refreshing operator map state...", 100);
        return loaded;
      });
      this.loadMapDialog.close();
      if (this.isFleetManager(robot)) {
        this.invalidateOperatorScene3d();
      }
      this.applyLoadedMapResult(result, mapName, robot);
      if (this.isFleetManager(robot)) {
        this.currentStatus = await this.getJson(this.fleetApiPath("/state")).catch(() => this.currentStatus);
        this.renderFleetStateImmediately();
      } else {
        this.renderSelectedRobot();
      }
      this.refreshAfterMapLoadInBackground();
      this.robotMessageText.textContent = `${this.isFleetManager(robot) ? "Fleet Manager" : "Robot"} active map changed to ${result.mapName || mapName}.`;
    } catch (error) {
      await this.refreshRobotMapState({ quiet: true }).catch(() => {});
      this.renderSelectedRobot();
      this.loadMapHint.className = "probe-result error";
      this.loadMapHint.textContent = error.message || String(error);
    }
  }

  dialogPayload() {
    return {
      type: "grpc",
      name: this.robotNameInput.value.trim(),
      host: this.robotHostInput.value.trim(),
      domainId: Number(this.robotDomainInput?.value || 0),
      port: Number(this.robotPortInput.value || 50051),
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

  async postJsonRaw(url, payload) {
    const response = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
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
