import * as THREE from "./vendor/three.module.min.js";

const COLORS = {
  floor: 0xf7f8fb,
  wall: 0x151922,
  edge: 0x2f6fed,
  edgeActive: 0x22c55e,
  lm: 0xf59e0b,
  lmHover: 0xffca4f,
  robot: 0x2563eb,
  robotActive: 0x16a34a,
  route: 0x64748b,
  trpOrange: 0xff6c0a,
  trpBlue: 0x0000cc,
  wheel: 0x1f2937,
};

export class OperatorScene3D {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf8fbff);
    this.scene.fog = new THREE.Fog(0xf8fbff, 35, 95);

    this.camera = new THREE.PerspectiveCamera(48, 1, 0.05, 400);
    this.orthoCamera = new THREE.OrthographicCamera(-10, 10, 10, -10, 0.05, 400);
    this.activeCamera = this.camera;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    this.renderer.setPixelRatio(Math.min(1.35, window.devicePixelRatio || 1));
    this.renderer.shadowMap.enabled = false;
    this.container.append(this.renderer.domElement);

    this.staticGroup = new THREE.Group();
    this.robotGroup = new THREE.Group();
    this.routeGroup = new THREE.Group();
    this.scene.add(this.staticGroup, this.routeGroup, this.robotGroup);

    this.target = new THREE.Vector3(0, 0, 0);
    this.distance = 24;
    this.yaw = -Math.PI / 4;
    this.pitch = 0.85;
    this.bounds = { width: 20, depth: 20 };
    this.drag = null;
    this.handlers = {};
    this.raycaster = new THREE.Raycaster();
    this.floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    this.floorMesh = null;
    this.wallMaterial = null;
    this.lms = [];
    this.landmarkObjects = new Map();
    this.landmarkMesh = null;
    this.hoverMarker = null;
    this.hoverLabel = null;
    this.hoverLmName = "";
    this.targetArmed = false;
    this.robotObjects = new Map();
    this.robotRouteObjects = new Map();
    this.robotRouteKeys = new Map();
    this.maxStaticLmLabels = 220;
    this.maxInactiveRoutePoints = 48;
    this.maxActiveRoutePoints = 220;
    this.needsRender = true;
    this.disposed = false;

    this.addLights();
    this.bindControls();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.container);
    this.resize();
    this.animate();
  }

  addLights() {
    const hemi = new THREE.HemisphereLight(0xffffff, 0xdbeafe, 1.8);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xffffff, 2.5);
    sun.position.set(-8, 18, -12);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.near = 1;
    sun.shadow.camera.far = 80;
    sun.shadow.camera.left = -35;
    sun.shadow.camera.right = 35;
    sun.shadow.camera.top = 35;
    sun.shadow.camera.bottom = -35;
    this.scene.add(sun);
  }

  bindControls() {
    const canvas = this.renderer.domElement;
    canvas.addEventListener("pointerdown", (event) => {
      this.drag = {
        pointerId: event.pointerId,
        button: event.button,
        x: event.clientX,
        y: event.clientY,
        yaw: this.yaw,
        pitch: this.pitch,
        target: this.target.clone(),
        pan: event.button === 1 || event.button === 2 || event.shiftKey,
      };
      canvas.setPointerCapture(event.pointerId);
      event.preventDefault();
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!this.drag) {
        this.updateLandmarkHover(event);
        return;
      }
      const dx = event.clientX - this.drag.x;
      const dy = event.clientY - this.drag.y;
      if (this.drag.pan) {
        this.pan(dx, dy);
      } else {
        this.yaw = this.drag.yaw - dx * 0.006;
        this.pitch = Math.max(0.18, Math.min(1.555, this.drag.pitch + dy * 0.004));
      }
      this.requestRender();
    });
    canvas.addEventListener("pointerup", (event) => this.endDrag(event));
    canvas.addEventListener("pointercancel", (event) => this.endDrag(event));
    canvas.addEventListener("pointerleave", () => this.setLandmarkHover(""));
    canvas.addEventListener("contextmenu", (event) => event.preventDefault());
    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      const factor = Math.exp(event.deltaY * 0.0012);
      this.distance = Math.max(3.5, Math.min(110, this.distance * factor));
      this.requestRender();
    }, { passive: false });
  }

  endDrag(event) {
    if (!this.drag) {
      return;
    }
    const drag = this.drag;
    try {
      this.renderer.domElement.releasePointerCapture(event.pointerId);
    } catch (_) {
      // Pointer capture can already be gone after browser gestures.
    }
    this.drag = null;
    const moved = Math.hypot(event.clientX - drag.x, event.clientY - drag.y);
    if (!drag.pan && drag.button === 0 && moved <= 5) {
      if (!this.isTargetArmed() && this.pickRobot(event)) {
        return;
      }
      this.pickFloor(event);
    }
  }

  pan(dx, dy) {
    const scale = this.distance / Math.max(260, this.container.clientHeight || 260);
    const topDown = this.isTopDown();
    const right = topDown
      ? new THREE.Vector3(1, 0, 0)
      : new THREE.Vector3(Math.cos(this.yaw), 0, -Math.sin(this.yaw));
    const forward = topDown
      ? new THREE.Vector3(0, 0, 1)
      : new THREE.Vector3(Math.sin(this.yaw), 0, Math.cos(this.yaw));
    this.target.copy(this.drag.target)
      .addScaledVector(right, -dx * scale)
      .addScaledVector(forward, -dy * scale);
  }

  resize() {
    const width = Math.max(1, this.container.clientWidth || 1);
    const height = Math.max(1, this.container.clientHeight || 1);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.updateOrthoFrustum();
    this.renderer.setSize(width, height, false);
    this.requestRender();
  }

  setHandlers(handlers = {}) {
    this.handlers = { ...handlers };
  }

  setTargetArmed(armed) {
    this.targetArmed = Boolean(armed);
    this.renderer.domElement.classList.toggle("target-armed", this.targetArmed);
    if (!this.targetArmed) {
      this.setLandmarkHover("");
    }
  }

  isTargetArmed() {
    return this.targetArmed
      || this.container.classList.contains("target-armed")
      || this.renderer.domElement.classList.contains("target-armed");
  }

  isTopDown() {
    return this.pitch >= 1.49;
  }

  updateOrthoFrustum() {
    const width = Math.max(1, this.container.clientWidth || 1);
    const height = Math.max(1, this.container.clientHeight || 1);
    const aspect = width / height;
    const viewHeight = Math.max(4, this.distance * 0.95);
    const viewWidth = viewHeight * aspect;
    this.orthoCamera.left = -viewWidth / 2;
    this.orthoCamera.right = viewWidth / 2;
    this.orthoCamera.top = viewHeight / 2;
    this.orthoCamera.bottom = -viewHeight / 2;
    this.orthoCamera.updateProjectionMatrix();
  }

  floorPointForEvent(event) {
    this.updateCamera();
    const ndc = this.pointerNdc(event);
    if (!ndc) {
      return null;
    }
    const point = new THREE.Vector3();
    this.raycaster.setFromCamera(ndc, this.activeCamera);
    if (!this.raycaster.ray.intersectPlane(this.floorPlane, point)) {
      return null;
    }
    if (point.x < -0.5 || point.z < -0.5 || point.x > this.bounds.width + 0.5 || point.z > this.bounds.depth + 0.5) {
      return null;
    }
    return point;
  }

  pointerNdc(event) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    if (!rect.width || !rect.height) {
      return null;
    }
    return new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -(((event.clientY - rect.top) / rect.height) * 2 - 1)
    );
  }

  pickRobot(event) {
    if (typeof this.handlers.onRobotClick !== "function") {
      return false;
    }
    this.updateCamera();
    const ndc = this.pointerNdc(event);
    if (!ndc) {
      return false;
    }
    this.raycaster.setFromCamera(ndc, this.activeCamera);
    const hits = this.raycaster.intersectObjects(this.robotGroup.children, true);
    for (const hit of hits) {
      let object = hit.object;
      while (object) {
        const robotName = String(object.userData?.robotName || "");
        if (robotName) {
          this.handlers.onRobotClick(robotName);
          return true;
        }
        object = object.parent;
      }
    }
    return false;
  }

  pickFloor(event) {
    if (typeof this.handlers.onFloorClick !== "function") {
      return;
    }
    const point = this.floorPointForEvent(event);
    if (!point) {
      return;
    }
    this.handlers.onFloorClick({ x: point.x, y: point.z });
  }

  updateLandmarkHover(event) {
    if (!this.isTargetArmed()) {
      this.setLandmarkHover("");
      return;
    }
    const point = this.floorPointForEvent(event);
    if (!point) {
      this.setLandmarkHover("");
      return;
    }
    const nearest = this.nearestLandmark(point.x, point.z);
    this.setLandmarkHover(nearest && nearest.distance <= 0.75 ? nearest.landmark.name : "");
  }

  nearestLandmark(x, z) {
    let best = null;
    let bestDistance = Number.POSITIVE_INFINITY;
    for (const lm of this.lms) {
      const distance = Math.hypot(Number(lm.x || 0) - x, Number(lm.y || 0) - z);
      if (distance < bestDistance) {
        best = lm;
        bestDistance = distance;
      }
    }
    return best ? { landmark: best, distance: bestDistance } : null;
  }

  setLandmarkHover(lmName) {
    const nextName = String(lmName || "");
    if (nextName === this.hoverLmName) {
      return;
    }
    this.hoverLmName = nextName;
    const current = this.landmarkObjects.get(this.hoverLmName);
    if (current) {
      this.showHoverLandmark(current.landmark);
    } else {
      this.hideHoverLandmark();
    }
    if (typeof this.handlers.onLandmarkHover === "function") {
      this.handlers.onLandmarkHover(this.hoverLmName);
    }
    this.requestRender();
  }

  showHoverLandmark(landmark) {
    const x = Number(landmark.x || 0);
    const z = Number(landmark.y || 0);
    if (!this.hoverMarker) {
      const geometry = new THREE.CylinderGeometry(0.18, 0.18, 0.06, 20);
      const material = new THREE.MeshBasicMaterial({ color: COLORS.lmHover });
      this.hoverMarker = new THREE.Mesh(geometry, material);
      this.hoverMarker.renderOrder = 12;
      this.staticGroup.add(this.hoverMarker);
    }
    this.hoverMarker.position.set(x, 0.075, z);
    this.hoverMarker.visible = true;

    if (this.hoverLabel) {
      this.staticGroup.remove(this.hoverLabel);
      this.disposeObject(this.hoverLabel);
      this.hoverLabel = null;
    }
    this.hoverLabel = this.floorLabel(String(landmark.name || ""), new THREE.Vector3(x, 0, z));
    this.hoverLabel.scale.set(1.18, 1.18, 1.18);
    this.staticGroup.add(this.hoverLabel);
  }

  hideHoverLandmark() {
    if (this.hoverMarker) {
      this.hoverMarker.visible = false;
    }
    if (this.hoverLabel) {
      this.staticGroup.remove(this.hoverLabel);
      this.disposeObject(this.hoverLabel);
      this.hoverLabel = null;
    }
  }

  setScene(payload) {
    this.disposeGroup(this.staticGroup);
    this.staticGroup.clear();
    for (const name of Array.from(this.robotRouteObjects.keys())) {
      this.removeRobotRoute(name);
    }
    this.landmarkMesh = null;
    this.hoverMarker = null;
    this.hoverLabel = null;
    const floor = payload?.floor || {};
    this.bounds = {
      width: Math.max(1, Number(floor.width || 1)),
      depth: Math.max(1, Number(floor.depth || 1)),
    };
    this.lms = Array.isArray(payload?.lms) ? payload.lms : [];
    this.landmarkObjects.clear();
    this.hoverLmName = "";
    this.target.set(this.bounds.width / 2, 0, this.bounds.depth / 2);
    this.distance = Math.max(8, Math.max(this.bounds.width, this.bounds.depth) * 1.05);
    this.updateOrthoFrustum();

    this.addFloor(floor);
    this.addWalls(Array.isArray(payload?.walls) ? payload.walls : []);
    this.addEdges(Array.isArray(payload?.edges) ? payload.edges : []);
    this.addLandmarks(this.lms);
    this.requestRender();
  }

  addFloor(floor) {
    const geometry = new THREE.PlaneGeometry(this.bounds.width, this.bounds.depth);
    const material = new THREE.MeshStandardMaterial({ color: COLORS.floor, roughness: 0.9, metalness: 0.0 });
    if (floor.imageDataUrl) {
      const texture = new THREE.TextureLoader().load(String(floor.imageDataUrl), () => this.requestRender());
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.anisotropy = 4;
      material.map = texture;
      material.needsUpdate = true;
    }
    const mesh = new THREE.Mesh(geometry, material);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.set(this.bounds.width / 2, -0.01, this.bounds.depth / 2);
    mesh.receiveShadow = true;
    this.floorMesh = mesh;
    this.staticGroup.add(mesh);

    const grid = new THREE.GridHelper(Math.max(this.bounds.width, this.bounds.depth), 24, 0xcbd5e1, 0xe2e8f0);
    grid.position.set(this.bounds.width / 2, 0.002, this.bounds.depth / 2);
    this.staticGroup.add(grid);
  }

  addWalls(walls) {
    if (!walls.length) {
      return;
    }
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const material = new THREE.MeshStandardMaterial({ color: COLORS.wall, roughness: 0.85 });
    material.transparent = true;
    this.wallMaterial = material;
    const mesh = new THREE.InstancedMesh(geometry, material, walls.length);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    const matrix = new THREE.Matrix4();
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    walls.forEach((wall, index) => {
      const height = Math.max(0.05, Number(wall.height || 1.8));
      position.set(Number(wall.x || 0), height / 2, Number(wall.z || 0));
      scale.set(Math.max(0.01, Number(wall.width || 0.01)), height, Math.max(0.01, Number(wall.depth || 0.01)));
      matrix.compose(position, quaternion, scale);
      mesh.setMatrixAt(index, matrix);
    });
    mesh.instanceMatrix.needsUpdate = true;
    this.staticGroup.add(mesh);
  }

  addEdges(edges) {
    const material = new THREE.LineBasicMaterial({ color: COLORS.edge, transparent: true, opacity: 0.78 });
    const activeMaterial = new THREE.LineBasicMaterial({ color: COLORS.edgeActive, transparent: true, opacity: 0.95 });
    const positions = [];
    const activePositions = [];
    for (const edge of edges) {
      const points = this.edgePoints(edge);
      if (points.length < 2) {
        continue;
      }
      const target = Number(edge.motionDirectionCode) === 1 ? activePositions : positions;
      for (let index = 0; index < points.length - 1; index += 1) {
        target.push(
          points[index].x, points[index].y, points[index].z,
          points[index + 1].x, points[index + 1].y, points[index + 1].z
        );
      }
    }
    this.addLineSegments(positions, material);
    this.addLineSegments(activePositions, activeMaterial);
  }

  addLineSegments(positions, material) {
    if (!positions.length) {
      return;
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    this.staticGroup.add(new THREE.LineSegments(geometry, material));
  }

  edgePoints(edge) {
    if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      const [p0, p1, p2, p3] = edge.control_points.map((point) => new THREE.Vector3(Number(point.x || 0), 0.055, Number(point.y || 0)));
      const points = [];
      for (let i = 0; i <= 18; i += 1) {
        const t = i / 18;
        const a = (1 - t) ** 3;
        const b = 3 * ((1 - t) ** 2) * t;
        const c = 3 * (1 - t) * (t ** 2);
        const d = t ** 3;
        points.push(new THREE.Vector3(
          (a * p0.x) + (b * p1.x) + (c * p2.x) + (d * p3.x),
          0.055,
          (a * p0.z) + (b * p1.z) + (c * p2.z) + (d * p3.z)
        ));
      }
      return points;
    }
    const worldPoints = Array.isArray(edge.world_points) ? edge.world_points : [];
    if (worldPoints.length >= 2) {
      return worldPoints.map((point) => new THREE.Vector3(Number(point.x || 0), 0.055, Number(point.y || 0)));
    }
    return [];
  }

  addLandmarks(lms) {
    const geometry = new THREE.CylinderGeometry(0.09, 0.09, 0.035, 16);
    const material = new THREE.MeshBasicMaterial({ color: COLORS.lm });
    const mesh = new THREE.InstancedMesh(geometry, material, lms.length);
    mesh.renderOrder = 8;
    const matrix = new THREE.Matrix4();
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3(1, 1, 1);
    const showStaticLabels = lms.length <= this.maxStaticLmLabels;
    lms.forEach((lm, index) => {
      position.set(Number(lm.x || 0), 0.04, Number(lm.y || 0));
      matrix.compose(position, quaternion, scale);
      mesh.setMatrixAt(index, matrix);
      this.landmarkObjects.set(String(lm.name || ""), { landmark: lm, index });
      if (showStaticLabels && String(lm.name || "").length <= 8) {
        const label = this.floorLabel(String(lm.name || ""), position);
        this.staticGroup.add(label);
      }
    });
    mesh.instanceMatrix.needsUpdate = true;
    this.landmarkMesh = mesh;
    if (lms.length) {
      this.staticGroup.add(mesh);
    }
  }

  floorLabel(text, position) {
    const canvas = document.createElement("canvas");
    canvas.width = 256;
    canvas.height = 96;
    const context = canvas.getContext("2d");
    context.font = "700 34px system-ui, sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillStyle = "rgba(255,255,255,0.78)";
    context.fillRect(14, 20, 228, 56);
    context.strokeStyle = "rgba(15,23,42,0.20)";
    context.strokeRect(14, 20, 228, 56);
    context.fillStyle = "#0f172a";
    context.fillText(text, 128, 50);

    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = 4;
    const material = new THREE.MeshBasicMaterial({
      map: texture,
      transparent: true,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -1,
      polygonOffsetUnits: -1,
    });
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.82, 0.3), material);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.set(position.x, 0.016, position.z + 0.17);
    mesh.renderOrder = 3;
    return mesh;
  }

  labelSprite(text, position, yOffset) {
    const canvas = document.createElement("canvas");
    canvas.width = 256;
    canvas.height = 96;
    const context = canvas.getContext("2d");
    context.font = "600 34px system-ui, sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillStyle = "rgba(255,255,255,0.88)";
    context.fillRect(12, 18, 232, 60);
    context.strokeStyle = "rgba(15,23,42,0.22)";
    context.strokeRect(12, 18, 232, 60);
    context.fillStyle = "#0f172a";
    context.fillText(text, 128, 50);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthWrite: false }));
    sprite.position.set(position.x, position.y + yOffset, position.z);
    sprite.scale.set(0.85, 0.32, 1);
    return sprite;
  }

  updateRobots(robots, selectedName = "") {
    const incoming = new Set();
    const robotList = Array.isArray(robots) ? robots : [];
    const showLabels = robotList.length <= 50;
    for (const robot of robots || []) {
      const pose = this.robotPose(robot);
      if (!Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) {
        continue;
      }
      const name = String(robot.name || "");
      if (!name) {
        continue;
      }
      incoming.add(name);
      const active = String(robot.name || "") === String(selectedName || "");
      let entry = this.robotObjects.get(name);
      if (!entry) {
        const group = this.robotMesh({ ...robot, pose }, active);
        this.robotGroup.add(group);
        entry = { group, active: null };
        this.robotObjects.set(name, entry);
      }
      this.updateRobotObject(entry, { ...robot, pose }, active, showLabels);
      this.updateRobotRoute(robot, active);
    }
    for (const [name, entry] of Array.from(this.robotObjects.entries())) {
      if (incoming.has(name)) {
        continue;
      }
      this.robotGroup.remove(entry.group);
      this.disposeGroup(entry.group);
      this.robotObjects.delete(name);
      this.removeRobotRoute(name);
    }
    this.requestRender();
  }

  updateRobotObject(entry, robot, active, showLabel) {
    const pose = robot.pose || {};
    entry.group.position.set(Number(pose.x || 0), 0, Number(pose.y || 0));
    entry.group.rotation.y = -Number(pose.yaw || 0);
    const label = entry.group.userData.label;
    if (label) {
      label.visible = showLabel || active;
    }
    if (entry.active === active) {
      return;
    }
    entry.active = active;
    const bodyMaterial = entry.group.userData.bodyMaterial;
    if (bodyMaterial) {
      bodyMaterial.color.setHex(active ? COLORS.robotActive : COLORS.trpOrange);
    }
    const ringMaterial = entry.group.userData.ringMaterial;
    if (ringMaterial) {
      ringMaterial.color.setHex(active ? COLORS.edgeActive : COLORS.robot);
      ringMaterial.opacity = active ? 0.95 : 0.45;
    }
  }

  robotPose(robot) {
    const pose = robot?.pose || {};
    if (Number.isFinite(Number(pose.x)) && Number.isFinite(Number(pose.y))) {
      return pose;
    }
    const lmName = String(robot?.currentLm || robot?.nearestLm || robot?.targetLm || "");
    if (!lmName) {
      return {};
    }
    const lm = this.lms.find((item) => String(item.name || "") === lmName);
    if (!lm) {
      return {};
    }
    return {
      x: Number(lm.x || 0),
      y: Number(lm.y || 0),
      yaw: Number(pose.yaw || 0),
    };
  }

  robotMesh(robot, active) {
    const pose = robot.pose || {};
    const group = new THREE.Group();
    group.position.set(Number(pose.x || 0), 0, Number(pose.y || 0));
    group.rotation.y = -Number(pose.yaw || 0);
    group.renderOrder = 20;
    group.userData.robotName = String(robot.name || "");

    this.addTrp1Model(group, active);
    const label = this.labelSprite(String(robot.name || ""), new THREE.Vector3(0, 0.46, 0), 0);
    label.scale.set(0.7, 0.26, 1);
    group.userData.label = label;
    group.add(label);
    return group;
  }

  addTrp1Model(group, active) {
    const orange = new THREE.MeshStandardMaterial({
      color: active ? COLORS.robotActive : COLORS.trpOrange,
      roughness: 0.48,
      metalness: 0.02,
    });
    const blue = new THREE.MeshStandardMaterial({ color: COLORS.trpBlue, roughness: 0.38 });
    const wheel = new THREE.MeshStandardMaterial({ color: COLORS.wheel, roughness: 0.72 });
    const white = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.4 });

    const addBox = (size, position, material) => {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.z, size.y), material);
      mesh.position.set(position.x, position.z, position.y);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.renderOrder = 22;
      group.add(mesh);
      return mesh;
    };
    const addCylinder = (radius, length, position, material, rotation = null) => {
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 24), material);
      mesh.position.set(position.x, position.z, position.y);
      if (rotation) {
        mesh.rotation.set(rotation.x || 0, rotation.y || 0, rotation.z || 0);
      }
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.renderOrder = 22;
      group.add(mesh);
      return mesh;
    };

    group.userData.bodyMaterial = orange;

    // Dimensions mirror sim_robot/ws/src/trp1_description/urdf/trp1.xacro visual primitives.
    addBox({ x: 0.58, y: 0.22, z: 0.3 }, { x: 0, y: 0, z: 0.225 }, orange);
    addBox({ x: 0.38, y: 0.42, z: 0.3 }, { x: 0, y: 0, z: 0.225 }, orange);
    for (const [x, y] of [[0.19, 0.11], [-0.19, -0.11], [0.19, -0.11], [-0.19, 0.11]]) {
      addCylinder(0.1, 0.3, { x, y, z: 0.225 }, orange);
    }
    addCylinder(0.05, 0.31, { x: 0.2, y: 0, z: 0.225 }, blue);
    addCylinder(0.0605, 0.05, { x: 0, y: 0.18, z: 0.0605 }, wheel, { x: Math.PI / 2 });
    addCylinder(0.0605, 0.05, { x: 0, y: -0.18, z: 0.0605 }, wheel, { x: Math.PI / 2 });
    addBox({ x: 0.16, y: 0.055, z: 0.03 }, { x: 0.31, y: 0, z: 0.36 }, white);

    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.38, active ? 0.014 : 0.009, 8, 64),
      new THREE.MeshBasicMaterial({ color: active ? COLORS.edgeActive : COLORS.robot, transparent: true, opacity: active ? 0.95 : 0.45 })
    );
    group.userData.ringMaterial = ring.material;
    ring.rotation.x = Math.PI / 2;
    ring.position.y = 0.024;
    ring.renderOrder = 18;
    group.add(ring);
  }

  addRobotRoute(robot, active) {
    this.updateRobotRoute(robot, active);
  }

  updateRobotRoute(robot, active) {
    const name = String(robot?.name || "");
    if (!name) {
      return;
    }
    const trajectory = Array.isArray(robot?.trajectory) ? robot.trajectory : [];
    if (trajectory.length < 2) {
      this.removeRobotRoute(name);
      return;
    }
    const routeKey = this.robotRouteKey(robot, active);
    if (this.robotRouteKeys.get(name) === routeKey) {
      return;
    }
    this.removeRobotRoute(name);
    const maxPoints = active ? this.maxActiveRoutePoints : this.maxInactiveRoutePoints;
    const points = this.sampleTrajectory(trajectory, maxPoints)
      .map((point) => new THREE.Vector3(Number(point.x || 0), 0.095, Number(point.y || 0)));
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({
      color: active ? COLORS.edgeActive : COLORS.route,
      transparent: true,
      opacity: active ? 0.95 : 0.45,
    });
    const line = new THREE.Line(geometry, material);
    this.routeGroup.add(line);
    this.robotRouteObjects.set(name, line);
    this.robotRouteKeys.set(name, routeKey);
  }

  robotRouteKey(robot, active) {
    const trajectory = Array.isArray(robot?.trajectory) ? robot.trajectory : [];
    const first = trajectory[0] || {};
    const last = trajectory[trajectory.length - 1] || {};
    return [
      active ? "active" : "idle",
      robot?.routeRevision || "",
      trajectory.length,
      first.t ?? "",
      last.t ?? "",
      Number(last.x || 0).toFixed(3),
      Number(last.y || 0).toFixed(3),
    ].join(":");
  }

  sampleTrajectory(trajectory, maxPoints) {
    if (trajectory.length <= maxPoints) {
      return trajectory;
    }
    const result = [];
    const lastIndex = trajectory.length - 1;
    for (let index = 0; index < maxPoints; index += 1) {
      result.push(trajectory[Math.round((index / (maxPoints - 1)) * lastIndex)]);
    }
    return result;
  }

  removeRobotRoute(name) {
    const route = this.robotRouteObjects.get(name);
    if (!route) {
      return;
    }
    this.routeGroup.remove(route);
    this.disposeObject(route);
    this.robotRouteObjects.delete(name);
    this.robotRouteKeys.delete(name);
  }

  updateCamera() {
    const horizontal = Math.cos(this.pitch) * this.distance;
    const height = Math.sin(this.pitch) * this.distance;
    this.camera.position.set(
      this.target.x + Math.sin(this.yaw) * horizontal,
      this.target.y + height,
      this.target.z + Math.cos(this.yaw) * horizontal
    );
    this.camera.lookAt(this.target);
    this.updateOrthoFrustum();
    this.orthoCamera.position.set(this.target.x, this.target.y + Math.max(6, this.distance), this.target.z);
    this.orthoCamera.up.set(0, 0, -1);
    this.orthoCamera.lookAt(this.target);
    const topDown = this.isTopDown();
    this.activeCamera = topDown ? this.orthoCamera : this.camera;
    if (this.wallMaterial) {
      this.wallMaterial.opacity = topDown ? 0.42 : 1.0;
      this.wallMaterial.needsUpdate = true;
    }
  }

  requestRender() {
    this.needsRender = true;
  }

  animate() {
    if (this.disposed) {
      return;
    }
    window.requestAnimationFrame(() => this.animate());
    if (!this.needsRender && !this.drag) {
      return;
    }
    this.needsRender = false;
    this.updateCamera();
    this.renderer.render(this.scene, this.activeCamera);
  }

  disposeGroup(group) {
    group.traverse((object) => {
      this.disposeObjectResources(object);
    });
  }

  disposeObject(object) {
    object.traverse((item) => this.disposeObjectResources(item));
  }

  disposeObjectResources(object) {
    if (object.geometry) {
      object.geometry.dispose();
    }
    if (object.material) {
      const materials = Array.isArray(object.material) ? object.material : [object.material];
      for (const material of materials) {
        if (material.map) {
          material.map.dispose();
        }
        material.dispose();
      }
    }
  }

  dispose() {
    this.disposed = true;
    this.resizeObserver?.disconnect();
    this.disposeGroup(this.scene);
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
