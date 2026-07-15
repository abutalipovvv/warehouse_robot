import * as THREE from "./vendor/three.module.min.js";

const COLORS = {
  floor: 0xf7f8fb,
  wall: 0x151922,
  edge: 0x2f6fed,
  edgeActive: 0x22c55e,
  lm: 0xf59e0b,
  lmHover: 0xffca4f,
  robot: 0x2563eb,
  route: 0x64748b,
  ecomBody: 0xffffff,
  ecomDeck: 0xb8bec8,
  lidar: 0x374151,
  wheel: 0x1f2937,
};

const ROBOT_PALETTE = [
  0x2563eb,
  0x22c55e,
  0xf59e0b,
  0xef4444,
  0x8b5cf6,
  0x06b6d4,
  0xec4899,
  0x84cc16,
];

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
    this.renderPixelRatio = Math.min(1.35, window.devicePixelRatio || 1);
    this.renderer.setPixelRatio(this.renderPixelRatio);
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
    this.lastAnimationRenderAt = 0;
    this.lastRobotMotionAt = 0;
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

  labelSprite(text, position, yOffset, alertSeverity = "") {
    const canvas = document.createElement("canvas");
    canvas.width = 256;
    canvas.height = 96;
    const context = canvas.getContext("2d");
    const lines = String(text || "").split("\n").slice(0, 2);
    const multiline = lines.length > 1;
    context.font = `${multiline ? 700 : 600} ${multiline ? 25 : 34}px system-ui, sans-serif`;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillStyle = "rgba(255,255,255,0.88)";
    context.fillRect(8, multiline ? 7 : 18, 240, multiline ? 82 : 60);
    context.strokeStyle = "rgba(15,23,42,0.22)";
    context.strokeRect(8, multiline ? 7 : 18, 240, multiline ? 82 : 60);
    context.fillStyle = "#0f172a";
    if (multiline) {
      context.fillText(lines[0], 128, 32);
      context.fillStyle = alertSeverity === "error" ? "#c51f2d" : "#b54708";
      context.fillText(lines[1], 128, 65);
    } else {
      context.fillText(lines[0] || "", 128, 50);
    }
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
      depthWrite: false,
    }));
    sprite.position.set(position.x, position.y + yOffset, position.z);
    sprite.scale.set(multiline ? 1.18 : 0.85, multiline ? 0.44 : 0.32, 1);
    sprite.userData.labelText = String(text || "");
    sprite.userData.alertSeverity = String(alertSeverity || "");
    sprite.renderOrder = 20;
    return sprite;
  }

  updateRobots(robots, selectedName = "", waitBlockerName = "") {
    const incoming = new Set();
    const robotList = Array.isArray(robots) ? robots : [];
    this.updateRenderQuality(robotList.length);
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
      const waitBlocker = Boolean(waitBlockerName && name === String(waitBlockerName));
      let entry = this.robotObjects.get(name);
      if (!entry) {
        const group = this.robotMesh({ ...robot, pose }, active);
        this.robotGroup.add(group);
        entry = {
          group,
          active: null,
          waitBlocker: null,
          targetPose: {
            x: Number(pose.x || 0),
            y: Number(pose.y || 0),
            yaw: Number(pose.yaw || 0),
          },
        };
        this.robotObjects.set(name, entry);
      }
      this.updateRobotObject(entry, { ...robot, pose }, active, waitBlocker, showLabels);
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

  updateRobotPoses(robots) {
    const robotList = Array.isArray(robots) ? robots : [];
    if (robotList.length !== this.robotObjects.size) {
      return false;
    }
    for (const robot of robotList) {
      const name = String(robot?.name || "");
      const entry = this.robotObjects.get(name);
      const pose = this.robotPose(robot);
      if (
        !entry
        || !Number.isFinite(Number(pose.x))
        || !Number.isFinite(Number(pose.y))
      ) {
        return false;
      }
      entry.targetPose = {
        x: Number(pose.x || 0),
        y: Number(pose.y || 0),
        yaw: Number(pose.yaw || 0),
      };
    }
    return true;
  }

  updateRenderQuality(robotCount) {
    // At benchmark density fill-rate matters more than supersampling. Keep
    // antialiasing, but avoid rendering almost twice as many physical pixels
    // on HiDPI screens when forty or more full robot models are visible.
    const cap = Number(robotCount || 0) >= 40 ? 1.0 : 1.35;
    const desired = Math.min(cap, window.devicePixelRatio || 1);
    if (Math.abs(desired - this.renderPixelRatio) < 0.01) {
      return;
    }
    this.renderPixelRatio = desired;
    this.renderer.setPixelRatio(desired);
    this.resize();
  }

  updateRobotObject(entry, robot, active, waitBlocker, showLabel) {
    const pose = robot.pose || {};
    entry.targetPose = {
      x: Number(pose.x || 0),
      y: Number(pose.y || 0),
      yaw: Number(pose.yaw || 0),
    };
    const alertText = this.robotAlertLabel(robot);
    const alertSeverity = this.robotAlertSeverity(robot);
    const labelText = alertText
      ? `${String(robot.name || "")}\n${alertText}`
      : String(robot.name || "");
    let label = entry.group.userData.label;
    if (label && (
      label.userData.labelText !== labelText
      || label.userData.alertSeverity !== alertSeverity
    )) {
      entry.group.remove(label);
      this.disposeGroup(label);
      label = this.labelSprite(
        labelText,
        new THREE.Vector3(0, 0.46, 0),
        0,
        alertSeverity,
      );
      if (!alertText) {
        label.scale.set(0.7, 0.26, 1);
      }
      entry.group.userData.label = label;
      entry.group.add(label);
    }
    if (label) {
      label.visible = showLabel || active || Boolean(alertText);
    }
    if (entry.active === active && entry.waitBlocker === waitBlocker) {
      return;
    }
    entry.active = active;
    entry.waitBlocker = waitBlocker;
    const bodyMaterial = entry.group.userData.bodyMaterial;
    if (bodyMaterial) {
      bodyMaterial.color.setHex(COLORS.ecomBody);
    }
    const underglowMaterial = entry.group.userData.underglowMaterial;
    if (underglowMaterial) {
      underglowMaterial.color.setHex(
        waitBlocker ? 0xff7a00 : (entry.group.userData.selectionColor || COLORS.robot),
      );
      underglowMaterial.opacity = active ? 0.72 : (waitBlocker ? 0.5 : 0.12);
    }
    const underglowMesh = entry.group.userData.underglowMesh;
    if (underglowMesh && !active) {
      underglowMesh.scale.set(1, 1, 1);
    }
    const selectionHaloMesh = entry.group.userData.selectionHaloMesh;
    const selectionRingMesh = entry.group.userData.selectionRingMesh;
    const haloColor = waitBlocker
      ? 0xff7a00
      : (entry.group.userData.selectionColor || COLORS.robot);
    entry.group.userData.selectionHaloMaterial?.color.setHex(haloColor);
    entry.group.userData.selectionRingMaterial?.color.setHex(haloColor);
    if (selectionHaloMesh) {
      selectionHaloMesh.visible = active || waitBlocker;
      selectionHaloMesh.scale.set(1, 1, 1);
    }
    if (selectionRingMesh) {
      selectionRingMesh.visible = active || waitBlocker;
      selectionRingMesh.scale.set(1, 1, 1);
    }
  }

  robotAlertLabel(robot) {
    const status = String(robot?.status || "").trim().toUpperCase();
    const remoteError = String(robot?.remoteError || "").trim();
    const reason = String(remoteError || robot?.reason || robot?.routeNote || "").trim();
    if (remoteError) {
      return `error: ${remoteError.slice(0, 28)}`;
    }
    if (["OFFLINE", "ERROR"].includes(status)) {
      return reason ? `error: ${reason.slice(0, 28)}` : status.toLowerCase();
    }
    if (status === "RETREATING") {
      return "deadlock: retreating";
    }
    if (status === "BLOCKED" || status === "MANUAL_BLOCKED") {
      return /deadlock/i.test(reason)
        ? "deadlock: route blocked"
        : (reason ? `blocked: ${reason.slice(0, 27)}` : "route blocked");
    }
    if (reason === "route replan queued") {
      return "replanning route";
    }
    if (/deadlock/i.test(reason)) {
      return ["WAITING", "MOVING", "RETREATING"].includes(status)
        ? "deadlock: resolving"
        : "replanning route";
    }
    if (status !== "WAITING") {
      return "";
    }
    const dependency = robot?.waitDependency;
    const blocker = dependency && typeof dependency === "object"
      ? String(dependency.robot || "").trim()
      : "";
    if (blocker) {
      return `waiting for ${blocker}`;
    }
    if (reason.startsWith("traffic admission wait")) {
      return "waiting for traffic zone";
    }
    if (reason.startsWith("planned traffic wait")) {
      return "planned traffic wait";
    }
    return reason ? `waiting: ${reason.slice(0, 28)}` : "waiting for clearance";
  }

  robotAlertSeverity(robot) {
    const status = String(robot?.status || "").trim().toUpperCase();
    const reason = String(robot?.remoteError || robot?.reason || "");
    return Boolean(String(robot?.remoteError || "").trim())
      || ["OFFLINE", "ERROR", "BLOCKED", "MANUAL_BLOCKED", "RETREATING"].includes(status)
      || /deadlock/i.test(reason)
      ? "error"
      : "warning";
  }

  updateRobotMotion(timestamp = 0) {
    const now = Number(timestamp || 0);
    const dt = this.lastRobotMotionAt > 0 && now > this.lastRobotMotionAt
      ? Math.min(0.05, Math.max(0.001, (now - this.lastRobotMotionAt) / 1000))
      : (1 / 60);
    this.lastRobotMotionAt = now;
    // A short critically-damped visual follow removes websocket/manual-step
    // quantisation while remaining behind confirmed poses. At 60 Hz this
    // applies about 41% of the remaining correction per rendered frame.
    const alpha = 1 - Math.exp(-32 * dt);
    let animating = false;
    for (const entry of this.robotObjects.values()) {
      const target = entry.targetPose;
      if (!target) {
        continue;
      }
      const group = entry.group;
      const dx = Number(target.x || 0) - group.position.x;
      const dz = Number(target.y || 0) - group.position.z;
      const targetRotation = -Number(target.yaw || 0);
      const rotationDelta = Math.atan2(
        Math.sin(targetRotation - group.rotation.y),
        Math.cos(targetRotation - group.rotation.y),
      );
      const distanceSq = (dx * dx) + (dz * dz);
      if (distanceSq <= 0.00000001 && Math.abs(rotationDelta) <= 0.0001) {
        group.position.set(Number(target.x || 0), 0, Number(target.y || 0));
        group.rotation.y = targetRotation;
        continue;
      }
      animating = true;
      // Spawn/reset is a discontinuity, not vehicle motion. Do not animate a
      // robot across the warehouse after an explicit relocation.
      if (distanceSq > 4.0) {
        group.position.set(Number(target.x || 0), 0, Number(target.y || 0));
        group.rotation.y = targetRotation;
        continue;
      }
      group.position.x += dx * alpha;
      group.position.z += dz * alpha;
      group.rotation.y += rotationDelta * alpha;
    }
    return animating;
  }

  robotColor(robotName) {
    const name = String(robotName || "robot");
    let hash = 2166136261;
    for (let index = 0; index < name.length; index += 1) {
      hash ^= name.charCodeAt(index);
      hash = Math.imul(hash, 16777619);
    }
    return ROBOT_PALETTE[(hash >>> 0) % ROBOT_PALETTE.length];
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

    this.addEcomModel(group, robot, active);
    const label = this.labelSprite(String(robot.name || ""), new THREE.Vector3(0, 0.46, 0), 0);
    label.scale.set(0.7, 0.26, 1);
    group.userData.label = label;
    group.add(label);
    return group;
  }

  addEcomModel(group, robot, active) {
    const selectionColor = this.robotColor(robot?.name);
    const body = new THREE.MeshStandardMaterial({
      color: COLORS.ecomBody,
      roughness: 0.48,
      metalness: 0.12,
    });
    const deck = new THREE.MeshStandardMaterial({ color: COLORS.ecomDeck, roughness: 0.52 });
    const wheel = new THREE.MeshStandardMaterial({ color: COLORS.wheel, roughness: 0.72 });
    const lidar = new THREE.MeshStandardMaterial({ color: COLORS.lidar, roughness: 0.34 });
    const underglow = new THREE.MeshBasicMaterial({
      color: selectionColor,
      transparent: true,
      opacity: active ? 0.72 : 0.12,
      depthWrite: false,
      blending: THREE.NormalBlending,
    });
    const selectionHalo = new THREE.MeshBasicMaterial({
      color: selectionColor,
      transparent: true,
      opacity: 0.38,
      depthWrite: false,
      side: THREE.DoubleSide,
      blending: THREE.NormalBlending,
    });
    const selectionRing = new THREE.MeshBasicMaterial({
      color: selectionColor,
      transparent: true,
      opacity: 0.82,
      depthWrite: false,
      side: THREE.DoubleSide,
      blending: THREE.NormalBlending,
    });

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
    const addExtrudedPolygon = (outline, height, baseZ, material) => {
      const vertices = [];
      const indices = [];
      const count = outline.length;
      for (const point of outline) {
        vertices.push(point.x, baseZ, point.y);
      }
      for (const point of outline) {
        vertices.push(point.x, baseZ + height, point.y);
      }
      for (let index = 1; index < count - 1; index += 1) {
        indices.push(0, index, index + 1);
        indices.push(count, count + index + 1, count + index);
      }
      for (let index = 0; index < count; index += 1) {
        const next = (index + 1) % count;
        indices.push(index, count + next, next);
        indices.push(index, count + index, count + next);
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
      geometry.setIndex(indices);
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.renderOrder = 22;
      group.add(mesh);
      return mesh;
    };

    group.userData.bodyMaterial = body;
    group.userData.underglowMaterial = underglow;
    group.userData.selectionColor = selectionColor;
    group.userData.selectionHaloMaterial = selectionHalo;
    group.userData.selectionRingMaterial = selectionRing;

    // Lightweight browser representation of ecom_stage.urdf.xacro.  RViz
    // uses the supplied STL meshes; the browser keeps primitive geometry so
    // a 50-robot view does not duplicate the 21 MB chassis mesh 50 times.
    const bodyOutline = [
      { x: -0.5230, y: -0.1840 },
      { x: -0.5000, y: -0.2350 },
      { x: -0.4520, y: -0.3163 },
      { x: -0.4339, y: -0.3280 },
      { x: -0.0504, y: -0.3532 },
      { x: 0.3507, y: -0.3282 },
      { x: 0.4337, y: -0.2110 },
      { x: 0.4770, y: -0.1000 },
      { x: 0.4770, y: 0.1000 },
      { x: 0.4337, y: 0.2047 },
      { x: 0.3507, y: 0.3219 },
      { x: -0.0479, y: 0.3468 },
      { x: -0.4339, y: 0.3217 },
      { x: -0.5000, y: 0.2300 },
      { x: -0.5230, y: 0.1840 },
    ];
    const deckOutline = [
      { x: -0.420, y: -0.220 },
      { x: -0.360, y: -0.265 },
      { x: 0.275, y: -0.265 },
      { x: 0.355, y: -0.205 },
      { x: 0.380, y: -0.090 },
      { x: 0.380, y: 0.090 },
      { x: 0.355, y: 0.205 },
      { x: 0.275, y: 0.265 },
      { x: -0.360, y: 0.265 },
      { x: -0.420, y: 0.220 },
    ];
    const glowOutline = bodyOutline.map((point) => ({
      x: point.x * 1.07,
      y: point.y * 1.10,
    }));
    const underglowMesh = addExtrudedPolygon(glowOutline, 0.008, 0.006, underglow);
    underglowMesh.renderOrder = 24;
    group.userData.underglowMesh = underglowMesh;
    const selectionHaloMesh = new THREE.Mesh(new THREE.CircleGeometry(0.64, 56), selectionHalo);
    selectionHaloMesh.rotation.x = -Math.PI / 2;
    selectionHaloMesh.position.y = 0.012;
    selectionHaloMesh.renderOrder = 23;
    selectionHaloMesh.visible = active;
    group.userData.selectionHaloMesh = selectionHaloMesh;
    group.add(selectionHaloMesh);
    const selectionRingMesh = new THREE.Mesh(new THREE.RingGeometry(0.61, 0.68, 56), selectionRing);
    selectionRingMesh.rotation.x = -Math.PI / 2;
    selectionRingMesh.position.y = 0.014;
    selectionRingMesh.renderOrder = 24;
    selectionRingMesh.visible = active;
    group.userData.selectionRingMesh = selectionRingMesh;
    group.add(selectionRingMesh);
    addExtrudedPolygon(bodyOutline, 0.170, 0.0, body);
    addExtrudedPolygon(deckOutline, 0.045, 0.160, deck);

    // Keep the drive wheels below and inside the shell. From normal camera
    // angles the body hides them; they are only visible from underneath.
    addCylinder(0.09, 0.057, { x: -0.043, y: 0.300, z: 0.060 }, wheel, { x: Math.PI / 2 });
    addCylinder(0.09, 0.057, { x: -0.043, y: -0.300, z: 0.060 }, wheel, { x: Math.PI / 2 });

    addCylinder(0.0337, 0.042, { x: 0.32487, y: 0.24906, z: 0.218 }, lidar);
    addCylinder(0.0337, 0.042, { x: -0.41524, y: -0.25105, z: 0.218 }, lidar);
    addBox({ x: 0.055, y: 0.26, z: 0.018 }, { x: 0.438, y: 0, z: 0.166 }, lidar);

  }

  addRobotRoute(robot, active) {
    this.updateRobotRoute(robot, active);
  }

  updateRobotRoute(robot, active) {
    const name = String(robot?.name || "");
    if (!name) {
      return;
    }
    const routePreview = Array.isArray(robot?.routePreview) ? robot.routePreview : [];
    const trajectory = active && routePreview.length >= 2
      ? routePreview
      : (Array.isArray(robot?.trajectory) ? robot.trajectory : []);
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
    let routeObject;
    if (active) {
      const material = new THREE.MeshBasicMaterial({
        color: this.robotColor(name),
        transparent: true,
        opacity: 0.9,
        depthWrite: false,
        side: THREE.DoubleSide,
      });
      routeObject = new THREE.Mesh(this.routeRibbonGeometry(points, 0.105), material);
      routeObject.renderOrder = 16;
    } else {
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const material = new THREE.LineBasicMaterial({
        color: COLORS.route,
        transparent: true,
        opacity: 0.38,
      });
      routeObject = new THREE.Line(geometry, material);
    }
    this.routeGroup.add(routeObject);
    this.robotRouteObjects.set(name, routeObject);
    this.robotRouteKeys.set(name, routeKey);
  }

  routeRibbonGeometry(points, width) {
    const vertices = [];
    const indices = [];
    const halfWidth = width * 0.5;
    const direction = (from, to) => {
      const result = new THREE.Vector3(to.x - from.x, 0, to.z - from.z);
      return result.lengthSq() > 0.00000001 ? result.normalize() : null;
    };
    let fallbackDirection = new THREE.Vector3(1, 0, 0);

    for (let index = 0; index < points.length; index += 1) {
      const previous = index > 0 ? direction(points[index - 1], points[index]) : null;
      const next = index < points.length - 1 ? direction(points[index], points[index + 1]) : null;
      const incoming = previous || next || fallbackDirection;
      const outgoing = next || previous || fallbackDirection;
      fallbackDirection = outgoing;
      const tangent = incoming.clone().add(outgoing);
      if (tangent.lengthSq() < 0.00000001) {
        tangent.copy(outgoing);
      }
      tangent.normalize();
      const normal = new THREE.Vector3(-tangent.z, 0, tangent.x);
      const incomingNormal = new THREE.Vector3(-incoming.z, 0, incoming.x);
      const denominator = Math.max(0.35, Math.abs(normal.dot(incomingNormal)));
      const offset = Math.min(halfWidth / denominator, halfWidth * 2.4);
      vertices.push(
        points[index].x + normal.x * offset,
        points[index].y,
        points[index].z + normal.z * offset,
        points[index].x - normal.x * offset,
        points[index].y,
        points[index].z - normal.z * offset,
      );
      if (index > 0) {
        const previousLeft = (index - 1) * 2;
        const previousRight = previousLeft + 1;
        const left = index * 2;
        const right = left + 1;
        indices.push(previousLeft, previousRight, left, previousRight, right, left);
      }
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    return geometry;
  }

  robotRouteKey(robot, active) {
    const routePreview = Array.isArray(robot?.routePreview) ? robot.routePreview : [];
    const trajectory = active && routePreview.length >= 2
      ? routePreview
      : (Array.isArray(robot?.trajectory) ? robot.trajectory : []);
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

  updateSelectionAnimation(timestamp) {
    let animating = false;
    const pulse = 0.5 + (0.5 * Math.sin(Number(timestamp || 0) * 0.005));
    for (const entry of this.robotObjects.values()) {
      const mesh = entry.group.userData.underglowMesh;
      const material = entry.group.userData.underglowMaterial;
      if ((!entry.active && !entry.waitBlocker) || !mesh || !material) {
        continue;
      }
      animating = true;
      const scale = 1.02 + (pulse * (entry.waitBlocker ? 0.14 : 0.1));
      mesh.scale.set(scale, 1, scale);
      material.opacity = (entry.waitBlocker ? 0.42 : 0.58) + (pulse * 0.24);
      const selectionHaloMesh = entry.group.userData.selectionHaloMesh;
      const selectionHaloMaterial = entry.group.userData.selectionHaloMaterial;
      if (selectionHaloMesh && selectionHaloMaterial) {
        const haloScale = 0.98 + (pulse * 0.08);
        selectionHaloMesh.scale.set(haloScale, haloScale, 1);
        selectionHaloMaterial.opacity = 0.3 + (pulse * 0.18);
      }
      const selectionRingMesh = entry.group.userData.selectionRingMesh;
      const selectionRingMaterial = entry.group.userData.selectionRingMaterial;
      if (selectionRingMesh && selectionRingMaterial) {
        const ringScale = 0.96 + (pulse * 0.12);
        selectionRingMesh.scale.set(ringScale, ringScale, 1);
        selectionRingMaterial.opacity = 0.62 + (pulse * 0.28);
      }
    }
    return animating;
  }

  animate(timestamp = 0) {
    if (this.disposed) {
      return;
    }
    window.requestAnimationFrame((nextTimestamp) => this.animate(nextTimestamp));
    const robotAnimating = this.updateRobotMotion(timestamp);
    const selectionAnimating = this.updateSelectionAnimation(timestamp);
    if (!this.needsRender && !this.drag && !selectionAnimating && !robotAnimating) {
      return;
    }
    if (
      selectionAnimating
      && !robotAnimating
      && !this.needsRender
      && !this.drag
      && timestamp - this.lastAnimationRenderAt < 66
    ) {
      return;
    }
    this.needsRender = false;
    this.lastAnimationRenderAt = timestamp;
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
