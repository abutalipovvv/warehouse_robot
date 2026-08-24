import { occupancyWallRectanglesFromImageData } from "./occupancy-walls.js";

export { occupancyWallRectanglesFromImageData } from "./occupancy-walls.js";

const BABYLON_RUNTIME_URL = new URL("./vendor/babylon-9.16.2.js", import.meta.url).href;
const OCCUPANCY_WALL_WORKER_URL = new URL("./occupancy-wall-worker.js", import.meta.url);

const loadBabylon = () => {
  if (globalThis.BABYLON) {
    return Promise.resolve(globalThis.BABYLON);
  }
  return new Promise((resolve, reject) => {
    const existing = document.querySelector("script[data-babylon-runtime]");
    const script = existing || document.createElement("script");
    let settled = false;
    const finish = (callback) => {
      if (settled) {
        return;
      }
      settled = true;
      window.clearTimeout(timeout);
      script.removeEventListener("load", onLoad);
      script.removeEventListener("error", onError);
      callback();
    };
    const onLoad = () => finish(() => globalThis.BABYLON
      ? resolve(globalThis.BABYLON)
      : reject(new Error("Babylon.js loaded without exposing its runtime.")));
    const onError = () => finish(() => reject(new Error("Could not load the Babylon.js runtime.")));
    const timeout = window.setTimeout(() => {
      finish(() => reject(new Error("Babylon.js loading timed out; SVG fallback remains active.")));
    }, 8000);
    script.addEventListener("load", onLoad, { once: true });
    script.addEventListener("error", onError, { once: true });
    if (!existing) {
      script.src = BABYLON_RUNTIME_URL;
      script.dataset.babylonRuntime = "9.16.2";
      document.head.append(script);
    }
  });
};

// The same Babylon runtime renders the orthographic 2D map and the 3D twin.
// HTML/CSS remains responsible only for controls and inspector panels.
const B = globalThis.BABYLON || await loadBabylon();

// PGM rows, LM coordinates and graph coordinates all use map_top_left.
// Babylon ground UV v=0 is placed at map z=0, so the source must not be
// vertically flipped during upload.
const MAP_TEXTURE_INVERT_Y = false;
const DYNAMIC_FLOOR_TEXTURE_INVERT_Y = true;

const COLORS = {
  floor: 0xf8fbff,
  wall: 0x718493,
  edge: 0x8bb4ff,
  edgeDirection: 0x405b83,
  edgeActive: 0x2368ff,
  lm: 0x2368ff,
  lmHover: 0x5298ff,
  robot: 0x3d70a3,
  route: 0x5f7890,
  ecomBody: 0xe8ecef,
  ecomDeck: 0xaeb7bf,
  lidar: 0x303840,
  wheel: 0x252b31,
  frontPanel: 0x161b20,
  editor: 0x2d78b7,
  scan: 0x18a999,
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

const toColor3 = (hex) => B.Color3.FromHexString(
  `#${Number(hex || 0).toString(16).padStart(6, "0")}`,
);

const setMaterialColor = (material, hex) => {
  const color = toColor3(hex);
  if (material.albedoColor) {
    material.albedoColor.copyFrom(color);
  }
  if (material.diffuseColor) {
    material.diffuseColor.copyFrom(color);
  }
  if (material.emissiveColor && material.disableLighting) {
    material.emissiveColor.copyFrom(color);
  }
};

export class OperatorScene3D {
  constructor(container) {
    this.container = container;
    this.canvas = document.createElement("canvas");
    this.canvas.className = "operator-babylon-canvas";
    this.canvas.setAttribute("aria-label", "Babylon.js warehouse digital twin");
    this.canvas.setAttribute("tabindex", "0");
    this.engineBadge = document.createElement("div");
    this.engineBadge.className = "operator-scene-engine-badge";
    this.engineBadge.textContent = "BABYLON · STARTING";
    this.container.append(this.canvas, this.engineBadge);

    this.engine = null;
    this.scene = null;
    this.viewMode = "3d";
    this.saved3dView = null;
    this.camera = null;
    this.orthoCamera = null;
    this.activeCamera = null;
    this.staticRoot = null;
    this.robotRoot = null;
    this.routeRoot = null;
    this.target = new B.Vector3(0, 0, 0);
    this.distance = 24;
    this.yaw = -Math.PI / 4;
    this.pitch = 0.85;
    this.bounds = { width: 20, depth: 20 };
    this.drag = null;
    this.handlers = {};
    this.floorMesh = null;
    this.floorCanvasSource = null;
    this.floorDynamicTexture = null;
    this.wallMaterial = null;
    this.wallMesh = null;
    delete this.container.dataset.occupancyWalls;
    delete this.container.dataset.occupancyWallStride;
    this.currentFloor = null;
    this.currentWallHeight = 1.8;
    this.serverWallsAvailable = false;
    this.occupancyWallBuildGeneration = 0;
    this.occupancyWallBuildPending = false;
    this.occupancyWallBuildComplete = false;
    this.occupancyWallWorkerTask = null;
    this.graphMesh = null;
    this.edgeDirectionMesh = null;
    this.edgeDirectionsVisible = true;
    this.graphFaceMap = [];
    this.lms = [];
    this.landmarkObjects = new Map();
    this.landmarkMesh = null;
    this.landmarkLabelMesh = null;
    this.landmarkLabelSignature = "";
    this.landmarkLabelRefreshTimer = 0;
    this.landmarkLabelsVisible = true;
    this.editorRoot = null;
    this.editorState = {};
    this.editorStateSignature = "";
    this.externalPointer = null;
    this.hoverMarker = null;
    this.hoverLabel = null;
    this.hoverLmName = "";
    this.targetArmed = false;
    this.robotObjects = new Map();
    this.robotRouteObjects = new Map();
    this.robotRouteKeys = new Map();
    this.scanMesh = null;
    this.scanMaterial = null;
    this.scanPointCount = 0;
    this.latestScanPointCloud = [];
    this.pendingScanPointCloud = null;
    this.latestRobots = [];
    this.latestSelectedRobotName = "";
    this.latestWaitBlockerName = "";
    this.maxInactiveRoutePoints = 48;
    // A selected route is an operator diagnostic, not a coarse traffic hint.
    // Keep its graph turns intact; aggressive point thinning made long routes
    // appear truncated or cut diagonally across several LMs.
    this.maxActiveRoutePoints = 2048;
    this.renderPixelRatio = Math.min(1.35, window.devicePixelRatio || 1);
    this.appliedPixelRatio = 0;
    this.lastAnimationRenderAt = 0;
    this.lastRobotMotionAt = 0;
    this.lastCameraModeTopDown = null;
    this.cameraInteracting = false;
    this.cameraInteractionRestoreTimer = 0;
    this.compactRobotMode = false;
    this.needsRender = true;
    this.disposed = false;
    this.animationFrame = 0;
    this.pendingScenePayload = null;
    this.pendingSceneOptions = {};
    this.pendingRobots = null;
    this.appliedMapName = "";
    this.initError = null;

    this.bindControls();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.container);
    this.readyPromise = this.initialize().catch((error) => {
      this.initError = error;
      this.engineBadge.classList.add("error");
      this.engineBadge.textContent = "3D ENGINE ERROR";
      console.error("Babylon.js scene initialization failed", error);
    });
  }

  async initialize() {
    const engineKind = await this.createEngine();
    if (this.disposed) {
      this.engine?.dispose();
      return;
    }

    this.scene = new B.Scene(this.engine);
    // Fleet/map coordinates were authored in the same right-handed frame
    // previously used by Three.js. Keeping that frame also preserves camera
    // orbit direction and positive map yaw.
    this.scene.useRightHandedSystem = true;
    this.scene.clearColor = new B.Color4(0.933, 0.957, 1, 1);
    this.scene.fogMode = B.Scene.FOGMODE_LINEAR;
    this.scene.fogStart = 35;
    this.scene.fogEnd = 95;
    this.scene.fogColor = new B.Color3(0.933, 0.957, 1);
    this.scene.skipPointerMovePicking = true;
    this.scene.imageProcessingConfiguration.toneMappingEnabled = false;
    this.scene.imageProcessingConfiguration.contrast = 1;
    this.scene.imageProcessingConfiguration.exposure = 1;
    const colorCurves = new B.ColorCurves();
    colorCurves.globalSaturation = 0;
    colorCurves.globalExposure = 0;
    this.scene.imageProcessingConfiguration.colorCurves = colorCurves;
    this.scene.imageProcessingConfiguration.colorCurvesEnabled = true;

    this.camera = new B.FreeCamera("warehouse-camera", new B.Vector3(12, 18, 12), this.scene);
    this.camera.minZ = 0.05;
    this.camera.maxZ = 400;
    this.camera.fov = B.Tools.ToRadians(48);
    this.orthoCamera = new B.FreeCamera("warehouse-map-camera", new B.Vector3(10, 30, 10), this.scene);
    this.orthoCamera.mode = B.Camera.ORTHOGRAPHIC_CAMERA;
    this.orthoCamera.minZ = 0.05;
    this.orthoCamera.maxZ = 400;
    this.orthoCamera.upVector = new B.Vector3(0, 0, -1);
    this.activeCamera = this.camera;
    this.scene.activeCamera = this.activeCamera;

    this.staticRoot = new B.TransformNode("static-root", this.scene);
    this.robotRoot = new B.TransformNode("robot-root", this.scene);
    this.routeRoot = new B.TransformNode("route-root", this.scene);
    this.addLights();

    this.engineBadge.textContent = `BABYLON · ${engineKind}`;
    this.container.dataset.renderEngine = engineKind.toLowerCase().replace(/\s+/g, "-");
    this.resize();

    if (this.pendingScenePayload) {
      this.applyScene(this.pendingScenePayload, this.pendingSceneOptions);
    }
    if (this.pendingRobots) {
      const pending = this.pendingRobots;
      this.pendingRobots = null;
      this.updateRobots(pending.robots, pending.selectedName, pending.waitBlockerName);
    }
    if (this.pendingScanPointCloud) {
      const pending = this.pendingScanPointCloud;
      this.pendingScanPointCloud = null;
      this.setScanPointCloud(pending.points, pending.options);
    }
    this.animate();
  }

  async createEngine() {
    if (navigator.gpu && B.WebGPUEngine) {
      try {
        const supportCheck = B.WebGPUEngine.IsSupportedAsync;
        const supported = await Promise.resolve(
          typeof supportCheck === "function"
            ? supportCheck.call(B.WebGPUEngine)
            : supportCheck,
        );
        if (supported !== false) {
          const engine = new B.WebGPUEngine(this.canvas, {
            antialias: true,
            adaptToDeviceRatio: false,
            powerPreference: "high-performance",
          });
          await engine.initAsync();
          this.engine = engine;
          this.applyHardwareScaling();
          return "WEBGPU";
        }
      } catch (error) {
        console.warn("WebGPU initialization failed; falling back to WebGL 2", error);
      }
    }

    this.engine = new B.Engine(
      this.canvas,
      true,
      {
        antialias: true,
        preserveDrawingBuffer: false,
        stencil: true,
        powerPreference: "high-performance",
        disableWebGL2Support: false,
      },
      false,
    );
    this.applyHardwareScaling();
    return this.engine.webGLVersion >= 2 ? "WEBGL 2" : "WEBGL";
  }

  addLights() {
    const ambient = new B.HemisphericLight("warehouse-ambient", new B.Vector3(0, 1, 0), this.scene);
    ambient.intensity = 0.92;
    ambient.diffuse = new B.Color3(1, 1, 1);
    ambient.groundColor = new B.Color3(0.62, 0.68, 0.72);

    const sun = new B.DirectionalLight("warehouse-sun", new B.Vector3(0.35, -1, 0.42), this.scene);
    sun.position = new B.Vector3(-8, 18, -12);
    sun.intensity = 0.82;
  }

  bindControls() {
    const canvas = this.canvas;
    canvas.addEventListener("pointerdown", (event) => {
      const externalHit = this.pointerHit(event);
      if (
        typeof this.handlers.onPointerDown === "function"
        && this.handlers.onPointerDown(externalHit) === true
      ) {
        this.externalPointer = {
          pointerId: event.pointerId,
          x: event.clientX,
          y: event.clientY,
        };
        canvas.setPointerCapture(event.pointerId);
        event.preventDefault();
        return;
      }
      this.beginCameraInteraction();
      this.drag = {
        pointerId: event.pointerId,
        button: event.button,
        x: event.clientX,
        y: event.clientY,
        yaw: this.yaw,
        pitch: this.pitch,
        target: this.target.clone(),
        pan: this.viewMode === "2d" || event.button === 1 || event.button === 2 || event.shiftKey,
      };
      canvas.setPointerCapture(event.pointerId);
      event.preventDefault();
    });
    canvas.addEventListener("pointermove", (event) => {
      if (this.externalPointer?.pointerId === event.pointerId) {
        if (typeof this.handlers.onPointerMove === "function") {
          this.handlers.onPointerMove(this.pointerHit(event));
        }
        event.preventDefault();
        return;
      }
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
    canvas.addEventListener("pointerup", (event) => this.endPointer(event));
    canvas.addEventListener("pointercancel", (event) => this.endPointer(event));
    canvas.addEventListener("pointerleave", () => this.setLandmarkHover(""));
    canvas.addEventListener("contextmenu", (event) => {
      event.preventDefault();
      if (typeof this.handlers.onContextMenu === "function") {
        this.handlers.onContextMenu(this.pointerHit(event));
      }
    });
    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      const factor = Math.exp(event.deltaY * 0.0012);
      this.distance = Math.max(3.5, Math.min(110, this.distance * factor));
      this.scheduleLandmarkLabelRefresh();
      this.requestRender();
    }, { passive: false });
  }

  endPointer(event) {
    if (this.externalPointer?.pointerId === event.pointerId) {
      if (typeof this.handlers.onPointerUp === "function") {
        this.handlers.onPointerUp(this.pointerHit(event));
      }
      try {
        this.canvas.releasePointerCapture(event.pointerId);
      } catch (_) {
        // Capture may already be released by the browser.
      }
      this.externalPointer = null;
      return;
    }
    this.endDrag(event);
  }

  endDrag(event) {
    if (!this.drag) {
      return;
    }
    const drag = this.drag;
    try {
      this.canvas.releasePointerCapture(event.pointerId);
    } catch (_) {
      // Pointer capture can already be gone after a browser gesture.
    }
    this.drag = null;
    this.finishCameraInteraction();
    const moved = Math.hypot(event.clientX - drag.x, event.clientY - drag.y);
    if (drag.button === 0 && moved <= 5) {
      if (!this.isTargetArmed() && this.pickRobot(event)) {
        return;
      }
      this.pickFloor(event);
    }
  }

  pan(dx, dy) {
    const scale = this.distance / Math.max(260, this.container.clientHeight || 260);
    const topDown = this.isTopDown();
    if (topDown) {
      this.target.x = this.drag.target.x - (dx * scale);
      this.target.z = this.drag.target.z - (dy * scale);
      return;
    }
    const cosine = Math.cos(this.yaw);
    const sine = Math.sin(this.yaw);
    this.target.x = this.drag.target.x - (dx * scale * cosine) - (dy * scale * sine);
    this.target.z = this.drag.target.z + (dx * scale * sine) - (dy * scale * cosine);
  }

  beginCameraInteraction() {
    window.clearTimeout(this.cameraInteractionRestoreTimer);
    this.cameraInteractionRestoreTimer = 0;
    if (this.cameraInteracting) {
      return;
    }
    this.cameraInteracting = true;
    this.container.classList.add("camera-interacting");
    this.applyHardwareScaling();
    this.setDetailLabelsEnabled(false);
  }

  finishCameraInteraction() {
    window.clearTimeout(this.cameraInteractionRestoreTimer);
    this.cameraInteractionRestoreTimer = window.setTimeout(() => {
      this.cameraInteractionRestoreTimer = 0;
      this.cameraInteracting = false;
      this.container.classList.remove("camera-interacting");
      this.applyHardwareScaling();
      this.refreshLandmarkLabels();
      this.setDetailLabelsEnabled(true);
      this.requestRender();
    }, 140);
  }

  setDetailLabelsEnabled(enabled) {
    for (const mesh of this.staticRoot?.getChildMeshes(false) || []) {
      if (mesh.metadata?.cameraDetailLabel) {
        mesh.setEnabled(enabled);
      }
    }
    for (const entry of this.robotObjects.values()) {
      entry.group.metadata?.label?.setEnabled(enabled);
    }
  }

  applyHardwareScaling() {
    if (!this.engine) {
      return;
    }
    const desired = this.cameraInteracting
      ? Math.min(this.renderPixelRatio, 0.82)
      : this.renderPixelRatio;
    if (Math.abs(desired - this.appliedPixelRatio) < 0.01) {
      return;
    }
    this.appliedPixelRatio = desired;
    this.engine.setHardwareScalingLevel(1 / desired);
  }

  resize() {
    if (!this.engine) {
      return;
    }
    this.updateOrthoFrustum();
    this.engine.resize();
    this.requestRender();
  }

  setHandlers(handlers = {}) {
    this.handlers = { ...handlers };
  }

  setViewMode(mode) {
    const nextMode = mode === "2d" ? "2d" : "3d";
    if (nextMode === this.viewMode) {
      this.wallMesh?.setEnabled(nextMode === "3d");
      if (this.scanMaterial) {
        this.scanMaterial.pointSize = nextMode === "2d" ? 3.8 : 5.2;
      }
      if (nextMode === "3d") {
        this.ensureOccupancyWalls();
      }
      return;
    }
    if (nextMode === "2d") {
      this.saved3dView = {
        yaw: this.yaw,
        pitch: this.pitch,
        distance: this.distance,
        target: this.target.clone(),
      };
      this.pitch = Math.PI / 2;
    } else if (this.saved3dView) {
      this.yaw = this.saved3dView.yaw;
      this.pitch = this.saved3dView.pitch;
      this.distance = this.saved3dView.distance;
      this.target.copyFrom(this.saved3dView.target);
    } else {
      this.pitch = 0.85;
    }
    this.viewMode = nextMode;
    this.wallMesh?.setEnabled(nextMode === "3d");
    if (this.scanMaterial) {
      this.scanMaterial.pointSize = nextMode === "2d" ? 3.8 : 5.2;
    }
    if (nextMode === "3d") {
      this.ensureOccupancyWalls();
    }
    if (this.robotObjects.size || this.latestRobots.length) {
      this.clearRobotObjects();
      this.updateRobots(
        this.latestRobots,
        this.latestSelectedRobotName,
        this.latestWaitBlockerName,
      );
    }
    this.lastCameraModeTopDown = null;
    this.updateCamera();
    this.refreshLandmarkLabels(true);
    this.requestRender();
  }

  zoomBy(factor) {
    const value = Number(factor || 1);
    if (!Number.isFinite(value) || value <= 0) {
      return;
    }
    this.distance = Math.max(2.5, Math.min(140, this.distance / value));
    this.updateOrthoFrustum();
    this.scheduleLandmarkLabelRefresh();
    this.requestRender();
  }

  resetView() {
    this.target.set(this.bounds.width / 2, 0, this.bounds.depth / 2);
    this.distance = Math.max(8, Math.max(this.bounds.width, this.bounds.depth) * 1.05);
    if (this.viewMode === "2d") {
      this.pitch = Math.PI / 2;
    } else {
      this.yaw = -Math.PI / 4;
      this.pitch = 0.85;
      this.saved3dView = null;
    }
    this.updateCamera();
    this.refreshLandmarkLabels(true);
    this.requestRender();
  }

  focusOn(world) {
    if (!world) {
      return;
    }
    const x = Number(world.x);
    const z = Number(world.y ?? world.z);
    if (!Number.isFinite(x) || !Number.isFinite(z)) {
      return;
    }
    this.target.x = x;
    this.target.z = z;
    this.scheduleLandmarkLabelRefresh();
    this.requestRender();
  }

  setTargetArmed(armed) {
    this.targetArmed = Boolean(armed);
    this.canvas.classList.toggle("target-armed", this.targetArmed);
    if (!this.targetArmed) {
      this.setLandmarkHover("");
    }
  }

  setLandmarkLabelsVisible(visible) {
    const nextVisible = Boolean(visible);
    if (nextVisible === this.landmarkLabelsVisible) {
      return;
    }
    this.landmarkLabelsVisible = nextVisible;
    this.landmarkLabelSignature = "";
    this.refreshLandmarkLabels(true);
    this.requestRender();
  }

  setEdgeDirectionsVisible(visible) {
    const nextVisible = Boolean(visible);
    if (nextVisible === this.edgeDirectionsVisible) {
      return;
    }
    this.edgeDirectionsVisible = nextVisible;
    this.edgeDirectionMesh?.setEnabled(nextVisible);
    this.requestRender();
  }

  setScanPointCloud(points = [], options = {}) {
    const cloud = (Array.isArray(points) ? points : [])
      .map((point) => ({
        x: Number(point?.x),
        y: Number(point?.height ?? 0.24),
        z: Number(point?.y ?? point?.z),
      }))
      .filter((point) => (
        Number.isFinite(point.x)
        && Number.isFinite(point.y)
        && Number.isFinite(point.z)
      ));
    this.latestScanPointCloud = cloud;
    if (!this.scene) {
      this.pendingScanPointCloud = { points: cloud, options };
      return;
    }
    if (!cloud.length) {
      this.clearScanPointCloud();
      return;
    }
    if (!this.scanMesh) {
      this.scanMesh = new B.Mesh("grpc-scan-point-cloud", this.scene);
      this.scanMesh.isPickable = false;
      this.scanMesh.alwaysSelectAsActiveMesh = true;
      this.scanMesh.renderingGroupId = 1;
      this.scanMaterial = this.unlitMaterial("grpc-scan-point-cloud-material", COLORS.scan, 0.92);
      this.scanMaterial.pointsCloud = true;
      this.scanMaterial.pointSize = this.viewMode === "2d" ? 3.8 : 5.2;
      this.scanMaterial.disableDepthWrite = true;
      this.scanMesh.material = this.scanMaterial;
    }
    const positions = new Float32Array(cloud.length * 3);
    const pointCountChanged = this.scanPointCount !== cloud.length;
    const indices = pointCountChanged ? new Array(cloud.length) : null;
    for (let index = 0; index < cloud.length; index += 1) {
      const point = cloud[index];
      const offset = index * 3;
      positions[offset] = point.x;
      positions[offset + 1] = point.y;
      positions[offset + 2] = point.z;
      if (indices) {
        indices[index] = index;
      }
    }
    if (!pointCountChanged) {
      this.scanMesh.updateVerticesData(B.VertexBuffer.PositionKind, positions, true, false);
    } else {
      this.scanMesh.setVerticesData(B.VertexBuffer.PositionKind, positions, true, 3);
      this.scanMesh.setIndices(indices, null, true);
      this.scanPointCount = cloud.length;
    }
    this.scanMesh.refreshBoundingInfo();
    this.scanMesh.metadata = {
      source: "grpc",
      frameId: String(options.frameId || ""),
      stampSec: Number(options.stampSec || 0),
      pointCount: cloud.length,
    };
    this.scanMesh.setEnabled(true);
    this.container.dataset.scanSource = "grpc";
    this.container.dataset.scanPoints = String(cloud.length);
    this.requestRender();
  }

  clearScanPointCloud() {
    this.latestScanPointCloud = [];
    this.pendingScanPointCloud = null;
    this.scanMesh?.setEnabled(false);
    delete this.container.dataset.scanSource;
    delete this.container.dataset.scanPoints;
    this.requestRender();
  }

  isTargetArmed() {
    return this.targetArmed
      || this.container.classList.contains("target-armed")
      || this.canvas.classList.contains("target-armed");
  }

  isTopDown() {
    return this.pitch >= 1.49;
  }

  updateOrthoFrustum() {
    if (!this.orthoCamera) {
      return;
    }
    const width = Math.max(1, this.container.clientWidth || 1);
    const height = Math.max(1, this.container.clientHeight || 1);
    const aspect = width / height;
    const viewHeight = Math.max(4, this.distance * 0.95);
    const viewWidth = viewHeight * aspect;
    this.orthoCamera.orthoLeft = -viewWidth / 2;
    this.orthoCamera.orthoRight = viewWidth / 2;
    this.orthoCamera.orthoTop = viewHeight / 2;
    this.orthoCamera.orthoBottom = -viewHeight / 2;
  }

  scenePick(event, predicate, camera = this.activeCamera) {
    if (!this.scene || !camera) {
      return null;
    }
    const rect = this.canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) {
      return null;
    }
    return this.scene.pick(
      event.clientX - rect.left,
      event.clientY - rect.top,
      predicate,
      false,
      camera,
    );
  }

  pointerHit(event) {
    const worldPoint = this.floorPointForEvent(event);
    const hit = {
      pointerId: Number(event.pointerId ?? 0),
      button: Number(event.button ?? 0),
      clientX: Number(event.clientX || 0),
      clientY: Number(event.clientY || 0),
      world: worldPoint ? { x: worldPoint.x, y: worldPoint.z } : null,
      lmName: "",
      edgeKey: "",
      bezierIndex: 0,
    };
    if (!this.editorState?.active || !this.scene) {
      return hit;
    }
    this.updateCamera();
    const pick = this.scenePick(
      event,
      (mesh) => Boolean(
        mesh === this.landmarkMesh
        || mesh === this.graphMesh
        || mesh.metadata?.editorBezierIndex
      ),
    );
    const pickedMesh = pick?.pickedMesh;
    if (!pick?.hit || !pickedMesh) {
      return hit;
    }
    if (pickedMesh.metadata?.editorBezierIndex) {
      hit.edgeKey = String(pickedMesh.metadata.edgeKey || "");
      hit.bezierIndex = Number(pickedMesh.metadata.editorBezierIndex || 0);
      return hit;
    }
    if (pickedMesh === this.landmarkMesh) {
      const index = Number(pick.thinInstanceIndex);
      hit.lmName = Number.isInteger(index) && index >= 0
        ? String(this.lms[index]?.name || "")
        : "";
      return hit;
    }
    if (pickedMesh === this.graphMesh) {
      hit.edgeKey = String(this.graphFaceMap[Number(pick.faceId)] || "");
    }
    return hit;
  }

  floorPointForEvent(event) {
    if (!this.floorMesh) {
      return null;
    }
    this.updateCamera();
    const pick = this.scenePick(event, (mesh) => mesh === this.floorMesh);
    const point = pick?.hit ? pick.pickedPoint : null;
    if (!point) {
      return null;
    }
    if (point.x < -0.5 || point.z < -0.5 || point.x > this.bounds.width + 0.5 || point.z > this.bounds.depth + 0.5) {
      return null;
    }
    return point;
  }

  pickRobot(event) {
    if (typeof this.handlers.onRobotClick !== "function") {
      return false;
    }
    this.updateCamera();
    const pick = this.scenePick(event, (mesh) => Boolean(mesh.metadata?.robotName));
    const robotName = String(pick?.pickedMesh?.metadata?.robotName || "");
    if (!pick?.hit || !robotName) {
      return false;
    }
    this.handlers.onRobotClick(robotName);
    return true;
  }

  pickFloor(event) {
    if (typeof this.handlers.onFloorClick !== "function") {
      return;
    }
    const point = this.floorPointForEvent(event);
    if (point) {
      this.handlers.onFloorClick({ x: point.x, y: point.z });
    }
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
    if (current && this.scene) {
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
      const marker = B.MeshBuilder.CreateCylinder("lm-hover", {
        diameter: 0.36,
        height: 0.06,
        tessellation: 24,
      }, this.scene);
      marker.material = this.unlitMaterial("lm-hover-material", COLORS.lmHover, 1);
      marker.isPickable = false;
      marker.parent = this.staticRoot;
      this.hoverMarker = marker;
    }
    this.hoverMarker.position.set(x, 0.075, z);
    this.hoverMarker.isVisible = true;

    this.hoverLabel?.dispose(false, true);
    this.hoverLabel = this.floorLabel(String(landmark.name || ""), new B.Vector3(x, 0, z), 1.18);
  }

  hideHoverLandmark() {
    if (this.hoverMarker) {
      this.hoverMarker.isVisible = false;
    }
    if (this.hoverLabel) {
      this.hoverLabel.dispose(false, true);
      this.hoverLabel = null;
    }
  }

  setScene(payload, options = {}) {
    this.pendingScenePayload = payload || {};
    this.pendingSceneOptions = { ...options };
    const floor = payload?.floor || {};
    this.bounds = {
      width: Math.max(1, Number(floor.width || 1)),
      depth: Math.max(1, Number(floor.depth || 1)),
    };
    this.lms = Array.isArray(payload?.lms) ? payload.lms : [];
    if (this.scene) {
      this.applyScene(this.pendingScenePayload, this.pendingSceneOptions);
    }
  }

  setFloorCanvas(canvas) {
    this.floorCanvasSource = canvas || null;
    this.applyFloorCanvas();
  }

  updateFloorCanvas() {
    if (!this.floorDynamicTexture) {
      this.applyFloorCanvas();
      return;
    }
    this.floorDynamicTexture.update(DYNAMIC_FLOOR_TEXTURE_INVERT_Y);
    this.requestRender();
  }

  applyFloorCanvas() {
    const material = this.floorMesh?.material;
    if (!material || !this.floorCanvasSource || !this.scene) {
      return;
    }
    this.alignFloorUvsForDynamicTexture();
    if (
      this.floorDynamicTexture
      && this.floorDynamicTexture._canvas === this.floorCanvasSource
    ) {
      this.floorDynamicTexture.update(DYNAMIC_FLOOR_TEXTURE_INVERT_Y);
      this.requestRender();
      return;
    }
    material.albedoTexture?.dispose();
    const texture = new B.DynamicTexture(
      "warehouse-floor-editor-texture",
      this.floorCanvasSource,
      this.scene,
      false,
      B.Texture.NEAREST_SAMPLINGMODE,
    );
    texture.hasAlpha = false;
    texture.anisotropicFilteringLevel = 1;
    texture.level = 1.18;
    texture.update(DYNAMIC_FLOOR_TEXTURE_INVERT_Y);
    material.albedoTexture = texture;
    material.markAsDirty(B.Material.TextureDirtyFlag);
    this.floorDynamicTexture = texture;
    this.requestRender();
  }

  alignFloorUvsForDynamicTexture() {
    if (!this.floorMesh || this.floorMesh.metadata?.dynamicFloorUvsAligned) {
      return;
    }
    const uvs = this.floorMesh.getVerticesData(B.VertexBuffer.UVKind);
    if (!uvs?.length) {
      return;
    }
    const aligned = uvs.slice();
    for (let index = 1; index < aligned.length; index += 2) {
      aligned[index] = 1 - aligned[index];
    }
    // Babylon's DynamicTexture upload flips canvas rows. Flip the floor UVs
    // once so editor previews retain the same map_top_left orientation as the
    // source PGM and the ordinary (non-editable) floor texture.
    this.floorMesh.setVerticesData(B.VertexBuffer.UVKind, aligned, true);
    this.floorMesh.metadata = {
      ...(this.floorMesh.metadata || {}),
      dynamicFloorUvsAligned: true,
    };
  }

  applyScene(payload, options = {}) {
    const previousBounds = { ...this.bounds };
    const previousView = {
      target: this.target.clone(),
      distance: this.distance,
      yaw: this.yaw,
      pitch: this.pitch,
    };
    const nextMapName = String(payload?.mapName || "");
    this.cancelOccupancyWallWorker("Occupancy map changed.");
    this.staticRoot?.dispose(false, true);
    this.staticRoot = new B.TransformNode("static-root", this.scene);
    this.editorRoot?.dispose(false, true);
    this.editorRoot = new B.TransformNode("editor-root", this.scene);
    this.editorStateSignature = "";
    for (const name of Array.from(this.robotRouteObjects.keys())) {
      this.removeRobotRoute(name);
    }
    this.floorMesh = null;
    this.floorDynamicTexture = null;
    this.wallMaterial = null;
    this.wallMesh = null;
    this.currentFloor = null;
    this.serverWallsAvailable = false;
    delete this.container.dataset.occupancyWalls;
    delete this.container.dataset.occupancyWallStride;
    this.occupancyWallBuildGeneration += 1;
    this.occupancyWallBuildPending = false;
    this.occupancyWallBuildComplete = false;
    this.graphMesh = null;
    this.edgeDirectionMesh = null;
    this.graphFaceMap = [];
    this.landmarkMesh = null;
    this.landmarkLabelMesh = null;
    this.landmarkLabelSignature = "";
    this.hoverMarker = null;
    this.hoverLabel = null;
    this.landmarkObjects.clear();
    this.hoverLmName = "";
    this.lastCameraModeTopDown = null;

    const floor = payload?.floor || {};
    const walls = Array.isArray(payload?.walls) ? payload.walls : [];
    const wallAsset = payload?.wallAsset || {};
    this.bounds = {
      width: Math.max(1, Number(floor.width || 1)),
      depth: Math.max(1, Number(floor.depth || 1)),
    };
    this.currentFloor = floor;
    this.currentWallHeight = Math.max(0.05, Number(payload?.wallHeight || 1.8));
    this.serverWallsAvailable = walls.length > 0 || Boolean(wallAsset.url);
    this.lms = Array.isArray(payload?.lms) ? payload.lms : [];
    const preserveView = Boolean(
      options.preserveView
      && this.appliedMapName
      && this.appliedMapName === nextMapName
      && Math.abs(previousBounds.width - this.bounds.width) < 0.0001
      && Math.abs(previousBounds.depth - this.bounds.depth) < 0.0001
    );
    if (preserveView) {
      this.target.copyFrom(previousView.target);
      this.distance = previousView.distance;
      this.yaw = previousView.yaw;
      this.pitch = this.viewMode === "2d" ? Math.PI / 2 : previousView.pitch;
    } else {
      this.target.set(this.bounds.width / 2, 0, this.bounds.depth / 2);
      this.distance = Math.max(8, Math.max(this.bounds.width, this.bounds.depth) * 1.05);
    }
    this.appliedMapName = nextMapName;
    this.updateOrthoFrustum();

    this.addFloor(floor);
    this.applyFloorCanvas();
    this.addWalls(walls);
    this.occupancyWallBuildComplete = walls.length > 0;
    if (wallAsset.url && !walls.length) {
      this.loadPrebuiltWalls(wallAsset, this.occupancyWallBuildGeneration);
    } else if (this.viewMode === "3d" && !walls.length) {
      this.ensureOccupancyWalls();
    }
    this.addEdges(Array.isArray(payload?.edges) ? payload.edges : []);
    this.addLandmarks(this.lms);
    this.setEditorState(this.editorState);
    this.optimizeStaticScene();
    this.requestRender();
  }

  optimizeStaticScene() {
    for (const mesh of this.staticRoot?.getChildMeshes(false) || []) {
      mesh.freezeWorldMatrix();
      if (mesh !== this.floorMesh) {
        mesh.doNotSyncBoundingInfo = true;
      }
      if (
        mesh.material
        && mesh.material !== this.wallMaterial
        && mesh !== this.floorMesh
        && typeof mesh.material.freeze === "function"
      ) {
        mesh.material.freeze();
      }
    }
  }

  pbrMaterial(name, hex, metallic, roughness) {
    const material = new B.PBRMaterial(name, this.scene);
    material.albedoColor = toColor3(hex);
    material.metallic = metallic;
    material.roughness = roughness;
    material.environmentIntensity = 0.7;
    return material;
  }

  unlitMaterial(name, hex, alpha = 1) {
    const material = new B.StandardMaterial(name, this.scene);
    const color = toColor3(hex);
    material.diffuseColor = color;
    material.emissiveColor = color;
    material.specularColor = B.Color3.Black();
    material.disableLighting = true;
    material.alpha = alpha;
    material.backFaceCulling = false;
    return material;
  }

  addFloor(floor) {
    const mesh = B.MeshBuilder.CreateGround("warehouse-floor", {
      width: this.bounds.width,
      height: this.bounds.depth,
      subdivisions: 1,
    }, this.scene);
    mesh.position.set(this.bounds.width / 2, -0.01, this.bounds.depth / 2);
    mesh.parent = this.staticRoot;
    mesh.isPickable = true;

    const material = this.pbrMaterial("warehouse-floor-material", COLORS.floor, 0, 0.92);
    material.unlit = true;
    if (floor.imageDataUrl) {
      const texture = new B.Texture(
        String(floor.imageDataUrl),
        this.scene,
        false,
        MAP_TEXTURE_INVERT_Y,
        B.Texture.NEAREST_SAMPLINGMODE,
        () => this.requestRender(),
      );
      texture.anisotropicFilteringLevel = 1;
      // Occupancy textures commonly encode free floor as light gray. Boost
      // that channel to white while preserving dark map features.
      texture.level = 1.3;
      material.albedoTexture = texture;
    }
    mesh.material = material;
    this.floorMesh = mesh;
  }

  addWalls(walls) {
    if (!walls.length) {
      return;
    }
    const matrices = new Float32Array(walls.length * 16);
    walls.forEach((wall, index) => {
      const height = Math.max(0.05, Number(wall.height || 1.8));
      const offset = index * 16;
      // The wall boxes never rotate. Write scale + translation matrices
      // directly and avoid thousands of short-lived Vector/Quaternion/Matrix
      // objects and the resulting garbage-collector pause.
      matrices[offset] = Math.max(0.01, Number(wall.width || 0.01));
      matrices[offset + 5] = height;
      matrices[offset + 10] = Math.max(0.01, Number(wall.depth || 0.01));
      matrices[offset + 12] = Number(wall.x || 0);
      matrices[offset + 13] = height / 2;
      matrices[offset + 14] = Number(wall.z || 0);
      matrices[offset + 15] = 1;
    });
    this.addWallMatrices(
      matrices,
      walls.length,
      Number(walls[0]?.stride || 1),
    );
  }

  addWallMatrices(matrices, count, stride = 1) {
    if (!count || matrices.length !== count * 16) {
      return;
    }
    const mesh = B.MeshBuilder.CreateBox("warehouse-walls", { size: 1 }, this.scene);
    // One opaque StandardMaterial is considerably cheaper than evaluating a
    // transparent PBR shader for every wall face on every animated frame.
    const material = new B.StandardMaterial("warehouse-wall-material", this.scene);
    material.diffuseColor = toColor3(COLORS.wall);
    material.specularColor = B.Color3.Black();
    material.alpha = 1;
    material.backFaceCulling = true;
    mesh.material = material;
    mesh.parent = this.staticRoot;
    mesh.isPickable = false;
    mesh.thinInstanceSetBuffer("matrix", matrices, 16, true);
    mesh.thinInstanceRefreshBoundingInfo(true);
    mesh.freezeWorldMatrix();
    mesh.doNotSyncBoundingInfo = true;
    material.freeze();
    this.wallMaterial = material;
    this.wallMesh = mesh;
    this.container.dataset.occupancyWalls = String(count);
    this.container.dataset.occupancyWallStride = String(stride || 1);
    mesh.setEnabled(this.viewMode === "3d");
  }

  async loadPrebuiltWalls(asset, generation) {
    const url = String(asset?.url || "");
    const count = Math.max(0, Number(asset?.count || 0));
    if (!url || !count) {
      this.occupancyWallBuildComplete = true;
      return;
    }
    this.occupancyWallBuildPending = true;
    this.container.dataset.occupancyWalls = "loading";
    try {
      const response = await fetch(url, { cache: "force-cache" });
      if (!response.ok) {
        throw new Error(`wall asset request failed: HTTP ${response.status}`);
      }
      const buffer = await response.arrayBuffer();
      if (buffer.byteLength !== count * 16 * 4) {
        throw new Error(
          `wall asset size mismatch: expected ${count * 16 * 4}, got ${buffer.byteLength}`,
        );
      }
      if (
        this.disposed
        || generation !== this.occupancyWallBuildGeneration
      ) {
        return;
      }
      this.addWallMatrices(
        new Float32Array(buffer),
        count,
        Number(asset?.stride || 1),
      );
      this.occupancyWallBuildComplete = true;
      this.requestRender();
    } catch (error) {
      if (generation === this.occupancyWallBuildGeneration) {
        this.serverWallsAvailable = false;
        this.occupancyWallBuildComplete = false;
        console.warn("Could not load prebuilt 3D wall asset.", error);
        if (this.viewMode === "3d") {
          window.setTimeout(() => this.ensureOccupancyWalls(), 0);
        }
      }
    } finally {
      if (generation === this.occupancyWallBuildGeneration) {
        this.occupancyWallBuildPending = false;
      }
    }
  }

  async ensureOccupancyWalls() {
    if (
      this.disposed
      || !this.scene
      || this.viewMode !== "3d"
      || this.wallMesh
      || this.serverWallsAvailable
      || this.occupancyWallBuildPending
      || this.occupancyWallBuildComplete
    ) {
      return;
    }
    const floor = this.currentFloor;
    if (!floor?.imageDataUrl && !this.floorCanvasSource) {
      this.occupancyWallBuildComplete = true;
      return;
    }

    const generation = this.occupancyWallBuildGeneration;
    const staticRoot = this.staticRoot;
    this.occupancyWallBuildPending = true;
    this.container.dataset.occupancyWalls = "loading";
    try {
      const imageData = await this.occupancyWallImageData(floor);
      if (
        this.disposed
        || generation !== this.occupancyWallBuildGeneration
        || staticRoot !== this.staticRoot
      ) {
        return;
      }
      await new Promise((resolve) => window.setTimeout(resolve, 0));
      const walls = await this.buildOccupancyWalls(
        imageData,
        floor.resolution,
        this.currentWallHeight,
      );
      if (
        this.disposed
        || generation !== this.occupancyWallBuildGeneration
        || staticRoot !== this.staticRoot
      ) {
        return;
      }
      this.addWalls(walls);
      this.occupancyWallBuildComplete = true;
      this.container.dataset.occupancyWalls = String(walls.length);
      this.requestRender();
    } catch (error) {
      if (generation === this.occupancyWallBuildGeneration) {
        this.occupancyWallBuildComplete = true;
        this.container.dataset.occupancyWalls = "error";
        console.warn("Could not build 3D occupancy walls from the map texture.", error);
      }
    } finally {
      if (generation === this.occupancyWallBuildGeneration) {
        this.occupancyWallBuildPending = false;
      }
    }
  }

  async occupancyWallImageData(floor) {
    const canvas = this.floorCanvasSource;
    if (canvas?.width && canvas?.height) {
      const context = canvas.getContext("2d", { willReadFrequently: true });
      if (!context) {
        throw new Error("Could not read the edited occupancy canvas.");
      }
      return context.getImageData(0, 0, canvas.width, canvas.height);
    }
    const source = String(floor?.imageDataUrl || "");
    if (!source) {
      throw new Error("The occupancy map has no image.");
    }
    const image = await new Promise((resolve, reject) => {
      const element = new Image();
      element.addEventListener("load", () => resolve(element), { once: true });
      element.addEventListener("error", () => reject(new Error("Could not decode the occupancy map.")), {
        once: true,
      });
      element.src = source;
    });
    const imageCanvas = document.createElement("canvas");
    imageCanvas.width = Math.max(1, Number(image.naturalWidth || image.width || 1));
    imageCanvas.height = Math.max(1, Number(image.naturalHeight || image.height || 1));
    const context = imageCanvas.getContext("2d", { willReadFrequently: true });
    if (!context) {
      throw new Error("Could not create an occupancy-map canvas.");
    }
    context.drawImage(image, 0, 0);
    return context.getImageData(0, 0, imageCanvas.width, imageCanvas.height);
  }

  cancelOccupancyWallWorker(reason = "Occupancy wall build cancelled.") {
    const task = this.occupancyWallWorkerTask;
    if (!task) {
      return;
    }
    this.occupancyWallWorkerTask = null;
    window.clearTimeout(task.timeout);
    task.worker.terminate();
    task.reject(new Error(reason));
  }

  async buildOccupancyWalls(imageData, resolution, wallHeight) {
    if (typeof globalThis.Worker !== "function") {
      await new Promise((resolve) => window.setTimeout(resolve, 0));
      return occupancyWallRectanglesFromImageData(imageData, resolution, wallHeight);
    }

    this.cancelOccupancyWallWorker();
    const worker = new Worker(OCCUPANCY_WALL_WORKER_URL, {
      type: "module",
      name: "occupancy-wall-builder",
    });
    return new Promise((resolve, reject) => {
      const finish = (callback) => {
        const task = this.occupancyWallWorkerTask;
        if (!task || task.worker !== worker) {
          return;
        }
        this.occupancyWallWorkerTask = null;
        window.clearTimeout(task.timeout);
        worker.terminate();
        callback();
      };
      const task = {
        worker,
        reject,
        timeout: window.setTimeout(() => {
          finish(() => reject(new Error("Occupancy wall generation timed out.")));
        }, 20000),
      };
      this.occupancyWallWorkerTask = task;
      worker.addEventListener("message", (event) => {
        const result = event.data || {};
        if (!result.ok) {
          finish(() => reject(new Error(result.error || "Occupancy wall generation failed.")));
          return;
        }
        finish(() => resolve(Array.isArray(result.walls) ? result.walls : []));
      }, { once: true });
      worker.addEventListener("error", (event) => {
        finish(() => reject(new Error(event.message || "Occupancy wall worker failed.")));
      }, { once: true });

      const pixels = imageData.data;
      const pixelBuffer = (
        pixels.byteOffset === 0 && pixels.byteLength === pixels.buffer.byteLength
          ? pixels.buffer
          : pixels.slice().buffer
      );
      try {
        worker.postMessage({
          width: imageData.width,
          height: imageData.height,
          pixels: pixelBuffer,
          resolution,
          wallHeight,
        }, [pixelBuffer]);
      } catch (error) {
        finish(() => reject(error));
      }
    });
  }

  addEdges(edges) {
    const lines = [];
    const edgeKeys = [];
    const sampleCount = edges.length >= 1200 ? 8 : 18;
    for (const edge of edges) {
      const points = this.edgePoints(edge, sampleCount);
      if (points.length < 2) {
        continue;
      }
      lines.push(points);
      edgeKeys.push(`${String(edge.from || "")}->${String(edge.to || "")}`);
    }
    const width = Math.max(0.025, Math.max(this.bounds.width, this.bounds.depth) / 900);
    this.graphMesh = this.createRibbonBatch(
      "graph-edges",
      lines,
      width,
      this.unlitMaterial("graph-edge-material", COLORS.edge, 0.52),
      this.staticRoot,
      edgeKeys,
    );
    if (this.graphMesh) {
      this.graphMesh.metadata = { editorKind: "edges" };
      this.graphMesh.isPickable = Boolean(this.editorState?.active);
    }
    this.addEdgeDirections(edges);
  }

  addEdgeDirections(edges) {
    const positions = [];
    const indices = [];
    const normals = [];
    const directedKeys = new Set(
      edges.map((edge) => `${String(edge.from || "")}->${String(edge.to || "")}`),
    );
    const maxDimension = Math.max(this.bounds.width, this.bounds.depth);
    const arrowLength = Math.max(0.12, Math.min(0.30, maxDimension / 135));
    const arrowHalfWidth = arrowLength * 0.42;
    const sampleCount = edges.length >= 1200 ? 8 : 18;

    for (const edge of edges) {
      const points = this.edgePoints(edge, sampleCount);
      if (points.length < 2) {
        continue;
      }
      const hasReverse = directedKeys.has(
        `${String(edge.to || "")}->${String(edge.from || "")}`,
      );
      const marker = this.pointAndTangentOnPolyline(points, hasReverse ? 0.56 : 0.5);
      if (!marker) {
        continue;
      }
      const tangent = marker.tangent;
      const normalX = -tangent.z;
      const normalZ = tangent.x;
      const tipX = marker.point.x + (tangent.x * arrowLength * 0.58);
      const tipZ = marker.point.z + (tangent.z * arrowLength * 0.58);
      const backX = marker.point.x - (tangent.x * arrowLength * 0.42);
      const backZ = marker.point.z - (tangent.z * arrowLength * 0.42);
      const base = positions.length / 3;
      positions.push(
        tipX, 0.082, tipZ,
        backX + (normalX * arrowHalfWidth), 0.082, backZ + (normalZ * arrowHalfWidth),
        backX - (normalX * arrowHalfWidth), 0.082, backZ - (normalZ * arrowHalfWidth),
      );
      indices.push(base, base + 1, base + 2);
    }
    if (!indices.length) {
      return;
    }
    B.VertexData.ComputeNormals(positions, indices, normals);
    const mesh = new B.Mesh("graph-edge-directions", this.scene);
    const vertexData = new B.VertexData();
    vertexData.positions = positions;
    vertexData.indices = indices;
    vertexData.normals = normals;
    vertexData.applyToMesh(mesh, false);
    mesh.material = this.unlitMaterial("graph-edge-direction-material", COLORS.edgeDirection, 0.9);
    mesh.parent = this.staticRoot;
    mesh.isPickable = false;
    mesh.metadata = { edgeDirections: true };
    mesh.setEnabled(this.edgeDirectionsVisible);
    this.edgeDirectionMesh = mesh;
  }

  pointAndTangentOnPolyline(points, fraction) {
    const lengths = [];
    let total = 0;
    for (let index = 0; index < points.length - 1; index += 1) {
      const length = B.Vector3.Distance(points[index], points[index + 1]);
      lengths.push(length);
      total += length;
    }
    if (total <= 0.000001) {
      return null;
    }
    const target = Math.max(0, Math.min(1, Number(fraction || 0))) * total;
    let walked = 0;
    for (let index = 0; index < lengths.length; index += 1) {
      const length = lengths[index];
      if (length <= 0.000001) {
        continue;
      }
      if (walked + length >= target || index === lengths.length - 1) {
        const ratio = Math.max(0, Math.min(1, (target - walked) / length));
        const start = points[index];
        const goal = points[index + 1];
        const tangent = goal.subtract(start);
        tangent.y = 0;
        tangent.normalize();
        return {
          point: B.Vector3.Lerp(start, goal, ratio),
          tangent,
        };
      }
      walked += length;
    }
    return null;
  }

  addLineSystem(name, lines, color, alpha, parent) {
    if (!lines.length) {
      return null;
    }
    const mesh = B.MeshBuilder.CreateLineSystem(name, { lines, updatable: false }, this.scene);
    mesh.color = toColor3(color);
    mesh.alpha = alpha;
    mesh.isPickable = false;
    mesh.parent = parent;
    return mesh;
  }

  createRibbonBatch(name, lines, width, material, parent, edgeKeys = []) {
    const positions = [];
    const indices = [];
    const faceMap = [];
    const halfWidth = Math.max(0.004, Number(width || 0.04) / 2);
    lines.forEach((points, lineIndex) => {
      const edgeKey = String(edgeKeys[lineIndex] || "");
      for (let index = 0; index < points.length - 1; index += 1) {
        const start = points[index];
        const goal = points[index + 1];
        const dx = goal.x - start.x;
        const dz = goal.z - start.z;
        const length = Math.hypot(dx, dz);
        if (length <= 0.000001) {
          continue;
        }
        const nx = (-dz / length) * halfWidth;
        const nz = (dx / length) * halfWidth;
        const base = positions.length / 3;
        positions.push(
          start.x + nx, start.y, start.z + nz,
          start.x - nx, start.y, start.z - nz,
          goal.x + nx, goal.y, goal.z + nz,
          goal.x - nx, goal.y, goal.z - nz,
        );
        indices.push(base, base + 1, base + 2, base + 1, base + 3, base + 2);
        faceMap.push(edgeKey, edgeKey);
      }
    });
    if (!indices.length) {
      return null;
    }
    const normals = [];
    B.VertexData.ComputeNormals(positions, indices, normals);
    const mesh = new B.Mesh(name, this.scene);
    const vertexData = new B.VertexData();
    vertexData.positions = positions;
    vertexData.indices = indices;
    vertexData.normals = normals;
    vertexData.applyToMesh(mesh);
    mesh.material = material;
    mesh.parent = parent;
    mesh.isPickable = false;
    if (edgeKeys.length) {
      this.graphFaceMap = faceMap;
    }
    return mesh;
  }

  edgePoints(edge, sampleCount = 18) {
    if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      const [p0, p1, p2, p3] = edge.control_points.map(
        (point) => new B.Vector3(Number(point.x || 0), 0.055, Number(point.y || 0)),
      );
      const points = [];
      for (let index = 0; index <= sampleCount; index += 1) {
        const t = index / sampleCount;
        const a = (1 - t) ** 3;
        const b = 3 * ((1 - t) ** 2) * t;
        const c = 3 * (1 - t) * (t ** 2);
        const d = t ** 3;
        points.push(new B.Vector3(
          (a * p0.x) + (b * p1.x) + (c * p2.x) + (d * p3.x),
          0.055,
          (a * p0.z) + (b * p1.z) + (c * p2.z) + (d * p3.z),
        ));
      }
      return points;
    }
    const worldPoints = Array.isArray(edge.world_points) ? edge.world_points : [];
    const points = worldPoints.map(
      (point) => new B.Vector3(Number(point.x || 0), 0.055, Number(point.y || 0)),
    );
    if (points.length >= 2) {
      return points;
    }
    const from = this.landmarkObjects.get(String(edge.from || ""))?.landmark
      || this.lms.find((lm) => String(lm.name || "") === String(edge.from || ""));
    const to = this.landmarkObjects.get(String(edge.to || ""))?.landmark
      || this.lms.find((lm) => String(lm.name || "") === String(edge.to || ""));
    return from && to
      ? [
          new B.Vector3(Number(from.x || 0), 0.055, Number(from.y || 0)),
          new B.Vector3(Number(to.x || 0), 0.055, Number(to.y || 0)),
        ]
      : [];
  }

  addLandmarks(lms) {
    if (!lms.length) {
      return;
    }
    const mesh = B.MeshBuilder.CreateCylinder("landmarks", {
      diameter: Math.max(0.14, Math.max(this.bounds.width, this.bounds.depth) / 420),
      height: 0.035,
      tessellation: 16,
    }, this.scene);
    mesh.material = this.unlitMaterial("landmark-material", COLORS.lm, 1);
    mesh.parent = this.staticRoot;
    mesh.isPickable = Boolean(this.editorState?.active);
    mesh.thinInstanceEnablePicking = true;
    mesh.metadata = { editorKind: "landmarks" };
    const matrices = new Float32Array(lms.length * 16);
    lms.forEach((lm, index) => {
      const position = new B.Vector3(Number(lm.x || 0), 0.04, Number(lm.y || 0));
      B.Matrix.Translation(position.x, position.y, position.z).copyToArray(matrices, index * 16);
      this.landmarkObjects.set(String(lm.name || ""), { landmark: lm, index });
    });
    mesh.thinInstanceSetBuffer("matrix", matrices, 16, true);
    mesh.thinInstanceRefreshBoundingInfo(true);
    this.landmarkMesh = mesh;
    this.refreshLandmarkLabels(true);
  }

  setEditorState(state = {}) {
    const nextState = { ...state };
    const previewState = nextState.preview;
    const areaPreviewState = nextState.areaPreview;
    const trafficZoneSignature = (nextState.trafficZones || []).map((zone) => {
      const bounds = zone?.bounds || {};
      return [
        zone?.id || "",
        Number(bounds.minX).toFixed(3),
        Number(bounds.minY).toFixed(3),
        Number(bounds.maxX).toFixed(3),
        Number(bounds.maxY).toFixed(3),
      ].join(",");
    }).join(";");
    const signature = [
      this.viewMode,
      Number(nextState.revision || 0),
      nextState.active ? 1 : 0,
      nextState.dragging ? 1 : 0,
      String(nextState.tool || ""),
      String(nextState.selectedLmName || ""),
      String(nextState.selectedEdgeKey || ""),
      String(previewState?.fromName || ""),
      Number(previewState?.world?.x || 0).toFixed(3),
      Number(previewState?.world?.y || 0).toFixed(3),
      String(areaPreviewState?.kind || ""),
      Number(areaPreviewState?.start?.x || 0).toFixed(3),
      Number(areaPreviewState?.start?.y || 0).toFixed(3),
      Number(areaPreviewState?.current?.x || 0).toFixed(3),
      Number(areaPreviewState?.current?.y || 0).toFixed(3),
      trafficZoneSignature,
    ].join(":");
    this.editorState = nextState;
    if (!this.scene) {
      return;
    }
    if (signature === this.editorStateSignature) {
      return;
    }
    this.editorStateSignature = signature;
    this.editorRoot?.dispose(false, true);
    this.editorRoot = new B.TransformNode("editor-root", this.scene);
    const active = Boolean(this.editorState.active && this.viewMode === "2d");
    if (this.canvas) {
      this.canvas.style.cursor = active && this.editorState.tool !== "select"
        ? "crosshair"
        : "";
    }
    if (this.graphMesh) {
      this.graphMesh.isPickable = active;
    }
    if (this.landmarkMesh) {
      this.landmarkMesh.isPickable = active;
    }
    if (!active) {
      this.requestRender();
      return;
    }

    const lms = Array.isArray(this.editorState.lms) ? this.editorState.lms : this.lms;
    const edges = Array.isArray(this.editorState.edges) ? this.editorState.edges : [];
    const lmIndex = new Map(lms.map((lm) => [String(lm.name || ""), lm]));
    const selectedLmName = String(this.editorState.selectedLmName || "");
    const selectedEdgeKey = String(this.editorState.selectedEdgeKey || "");
    const overlayLines = [];
    const overlayKeys = [];
    const corridorLines = [];
    const corridorKeys = [];
    const corridorPairs = new Set();
    for (const edge of edges) {
      const regionId = String(edge?.properties?.controlled_region || "");
      if (!regionId) {
        continue;
      }
      const edgeNames = [String(edge.from || ""), String(edge.to || "")].sort();
      const pairKey = `${regionId}:${edgeNames[0]}<=>${edgeNames[1]}`;
      if (corridorPairs.has(pairKey)) {
        continue;
      }
      corridorPairs.add(pairKey);
      const points = this.editorEdgePoints(edge, lmIndex);
      if (points.length >= 2) {
        corridorLines.push(points);
        corridorKeys.push(pairKey);
      }
    }
    for (const zone of this.editorState.trafficZones || []) {
      if (
        String(zone?.kind || "") !== "controlled_corridor"
        || String(zone?.shape || "rectangle") !== "rectangle"
      ) {
        continue;
      }
      const bounds = zone?.bounds || {};
      const minX = Number(bounds.minX);
      const minY = Number(bounds.minY);
      const maxX = Number(bounds.maxX);
      const maxY = Number(bounds.maxY);
      if (![minX, minY, maxX, maxY].every(Number.isFinite)) {
        continue;
      }
      const width = Math.max(0.001, Math.abs(maxX - minX));
      const depth = Math.max(0.001, Math.abs(maxY - minY));
      const area = B.MeshBuilder.CreateGround(
        `editor-saved-corridor-${String(zone.id || "")}`,
        { width, height: depth },
        this.scene,
      );
      area.position.set((minX + maxX) / 2, 0.1, (minY + maxY) / 2);
      area.material = this.unlitMaterial(
        `editor-saved-corridor-material-${String(zone.id || "")}`,
        0xed7d12,
        0.18,
      );
      area.parent = this.editorRoot;
      area.isPickable = false;
      overlayLines.push([
        new B.Vector3(minX, 0.108, minY),
        new B.Vector3(maxX, 0.108, minY),
        new B.Vector3(maxX, 0.108, maxY),
        new B.Vector3(minX, 0.108, maxY),
        new B.Vector3(minX, 0.108, minY),
      ]);
      overlayKeys.push(`traffic-zone:${String(zone.id || "")}`);
    }

    if (selectedLmName) {
      const selectedLm = lmIndex.get(selectedLmName);
      if (selectedLm) {
        const marker = B.MeshBuilder.CreateCylinder("editor-selected-lm", {
          diameter: Math.max(0.22, Math.max(this.bounds.width, this.bounds.depth) / 360),
          height: 0.045,
          tessellation: 24,
        }, this.scene);
        marker.position.set(Number(selectedLm.x || 0), 0.09, Number(selectedLm.y || 0));
        marker.material = this.unlitMaterial("editor-selected-lm-material", COLORS.lmHover, 0.92);
        marker.parent = this.editorRoot;
        marker.isPickable = false;
        if (!this.editorState.dragging) {
          this.floorLabel(String(selectedLm.name || ""), marker.position, 1.15).parent = this.editorRoot;
        }
      }
      for (const edge of edges) {
        if (edge.from !== selectedLmName && edge.to !== selectedLmName) {
          continue;
        }
        const points = this.editorEdgePoints(edge, lmIndex);
        if (points.length >= 2) {
          overlayLines.push(points);
          overlayKeys.push(`${String(edge.from || "")}->${String(edge.to || "")}`);
        }
      }
    }

    const selectedEdge = selectedEdgeKey
      ? edges.find((edge) => `${String(edge.from || "")}->${String(edge.to || "")}` === selectedEdgeKey)
      : null;
    if (selectedEdge) {
      const points = this.editorEdgePoints(selectedEdge, lmIndex);
      if (points.length >= 2) {
        overlayLines.push(points);
        overlayKeys.push(selectedEdgeKey);
      }
      if (Array.isArray(selectedEdge.control_points) && selectedEdge.control_points.length === 4) {
        [1, 2].forEach((index) => {
          const point = selectedEdge.control_points[index];
          const handle = B.MeshBuilder.CreateCylinder(`editor-bezier-${index}`, {
            diameter: Math.max(0.22, Math.max(this.bounds.width, this.bounds.depth) / 330),
            height: 0.065,
            tessellation: 18,
          }, this.scene);
          handle.position.set(Number(point.x || 0), 0.13, Number(point.y || 0));
          handle.material = this.unlitMaterial(`editor-bezier-material-${index}`, COLORS.lmHover, 1);
          handle.parent = this.editorRoot;
          handle.isPickable = true;
          handle.metadata = { editorBezierIndex: index, edgeKey: selectedEdgeKey };
        });
      }
    }

    const preview = this.editorState.preview;
    if (preview?.fromName && preview?.world) {
      const from = lmIndex.get(String(preview.fromName));
      if (from) {
        overlayLines.push([
          new B.Vector3(Number(from.x || 0), 0.105, Number(from.y || 0)),
          new B.Vector3(Number(preview.world.x || 0), 0.105, Number(preview.world.y || 0)),
        ]);
        overlayKeys.push("preview");
      }
    }
    const areaPreview = this.editorState.areaPreview;
    if (areaPreview?.start && areaPreview?.current) {
      const minX = Math.min(
        Number(areaPreview.start.x || 0),
        Number(areaPreview.current.x || 0),
      );
      const maxX = Math.max(
        Number(areaPreview.start.x || 0),
        Number(areaPreview.current.x || 0),
      );
      const minY = Math.min(
        Number(areaPreview.start.y || 0),
        Number(areaPreview.current.y || 0),
      );
      const maxY = Math.max(
        Number(areaPreview.start.y || 0),
        Number(areaPreview.current.y || 0),
      );
      const width = Math.max(0.001, maxX - minX);
      const depth = Math.max(0.001, maxY - minY);
      const area = B.MeshBuilder.CreateGround("editor-area-preview", {
        width,
        height: depth,
      }, this.scene);
      area.position.set((minX + maxX) / 2, 0.102, (minY + maxY) / 2);
      const corridor = areaPreview.kind === "corridor";
      area.material = this.unlitMaterial(
        `editor-area-preview-material-${corridor ? "corridor" : "raster"}`,
        corridor ? 0xed7d12 : COLORS.editor,
        corridor ? 0.2 : 0.14,
      );
      area.parent = this.editorRoot;
      area.isPickable = false;
      overlayLines.push([
        new B.Vector3(minX, 0.11, minY),
        new B.Vector3(maxX, 0.11, minY),
        new B.Vector3(maxX, 0.11, maxY),
        new B.Vector3(minX, 0.11, maxY),
        new B.Vector3(minX, 0.11, minY),
      ]);
      overlayKeys.push("area-preview");
    }
    if (corridorLines.length) {
      this.createRibbonBatch(
        "editor-corridor-overlay",
        corridorLines,
        Math.max(0.075, Math.max(this.bounds.width, this.bounds.depth) / 560),
        this.unlitMaterial("editor-corridor-overlay-material", 0xd97706, 0.82),
        this.editorRoot,
        corridorKeys,
      );
    }
    if (overlayLines.length) {
      this.createRibbonBatch(
        "editor-edge-overlay",
        overlayLines,
        Math.max(0.055, Math.max(this.bounds.width, this.bounds.depth) / 650),
        this.unlitMaterial("editor-edge-overlay-material", COLORS.editor, 0.92),
        this.editorRoot,
        [],
      );
    }
    this.requestRender();
  }

  editorEdgePoints(edge, lmIndex) {
    if (edge.geometry === "bezier" && Array.isArray(edge.control_points) && edge.control_points.length === 4) {
      return this.edgePoints(edge, 18).map((point) => new B.Vector3(point.x, 0.105, point.z));
    }
    const from = lmIndex.get(String(edge.from || ""));
    const to = lmIndex.get(String(edge.to || ""));
    return from && to
      ? [
          new B.Vector3(Number(from.x || 0), 0.105, Number(from.y || 0)),
          new B.Vector3(Number(to.x || 0), 0.105, Number(to.y || 0)),
        ]
      : this.edgePoints(edge, 18).map((point) => new B.Vector3(point.x, 0.105, point.z));
  }

  dynamicLabel(text, name, alertSeverity = "") {
    const lines = String(text || "").split("\n").slice(0, 2);
    const multiline = lines.length > 1;
    const texture = new B.DynamicTexture(name, { width: 512, height: 192 }, this.scene, false);
    texture.hasAlpha = true;
    const context = texture.getContext();
    context.clearRect(0, 0, 512, 192);
    context.fillStyle = "rgba(255,255,255,0.94)";
    context.fillRect(16, multiline ? 14 : 36, 480, multiline ? 164 : 120);
    context.strokeStyle = "rgba(35,104,255,0.22)";
    context.lineWidth = 2;
    context.strokeRect(16, multiline ? 14 : 36, 480, multiline ? 164 : 120);
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.font = `${multiline ? 700 : 600} ${multiline ? 50 : 68}px system-ui, sans-serif`;
    context.fillStyle = "#143158";
    if (multiline) {
      context.fillText(lines[0], 256, 64);
      context.fillStyle = alertSeverity === "error" ? "#c51f2d" : "#b54708";
      context.fillText(lines[1], 256, 130);
    } else {
      context.fillText(lines[0] || "", 256, 100);
    }
    // Canvas uses a top-left origin while the GPU texture uses bottom-left.
    // DynamicTexture performs the required upload flip when invertY is true.
    texture.update(true);

    const material = new B.StandardMaterial(`${name}-material`, this.scene);
    material.diffuseTexture = texture;
    material.opacityTexture = texture;
    material.emissiveColor = B.Color3.White();
    material.disableLighting = true;
    material.backFaceCulling = false;
    material.useAlphaFromDiffuseTexture = true;
    material.specularColor = B.Color3.Black();
    return material;
  }

  scheduleLandmarkLabelRefresh(delay = 120) {
    window.clearTimeout(this.landmarkLabelRefreshTimer);
    this.landmarkLabelRefreshTimer = window.setTimeout(() => {
      this.landmarkLabelRefreshTimer = 0;
      this.refreshLandmarkLabels();
      this.requestRender();
    }, delay);
  }

  landmarkLabelViewport() {
    const canvasWidth = Math.max(1, this.container.clientWidth || 1);
    const canvasHeight = Math.max(1, this.container.clientHeight || 1);
    if (this.viewMode !== "2d") {
      return {
        left: 0,
        right: this.bounds.width,
        top: 0,
        bottom: this.bounds.depth,
        width: this.bounds.width,
        height: this.bounds.depth,
        canvasWidth,
        canvasHeight,
      };
    }
    const viewHeight = Math.max(4, this.distance * 0.95);
    const viewWidth = viewHeight * (canvasWidth / canvasHeight);
    return {
      left: this.target.x - (viewWidth / 2),
      right: this.target.x + (viewWidth / 2),
      top: this.target.z - (viewHeight / 2),
      bottom: this.target.z + (viewHeight / 2),
      width: viewWidth,
      height: viewHeight,
      canvasWidth,
      canvasHeight,
    };
  }

  landmarkLabelCandidates(viewport) {
    if (!this.landmarkLabelsVisible) {
      return [];
    }
    const visible = this.lms.filter((lm) => {
      const x = Number(lm.x || 0);
      const z = Number(lm.y || 0);
      return x >= viewport.left
        && x <= viewport.right
        && z >= viewport.top
        && z <= viewport.bottom;
    });
    const selectedName = String(this.editorState?.selectedLmName || "");
    return [...visible].sort((first, second) => {
      if (String(first.name || "") === selectedName) {
        return -1;
      }
      if (String(second.name || "") === selectedName) {
        return 1;
      }
      return String(first.name || "").localeCompare(String(second.name || ""));
    });
  }

  refreshLandmarkLabels(force = false) {
    if (!this.scene || !this.staticRoot || !this.lms.length) {
      return;
    }
    const viewport = this.landmarkLabelViewport();
    const candidates = this.landmarkLabelCandidates(viewport);
    const worldPerPixel = viewport.height / viewport.canvasHeight;
    const labelWidth = Math.max(0.42, Math.min(2.4, worldPerPixel * 44));
    const labelHeight = Math.max(0.16, Math.min(0.72, worldPerPixel * 14));
    const signature = [
      this.viewMode,
      this.landmarkLabelsVisible ? "labels-on" : "labels-off",
      labelWidth.toFixed(3),
      labelHeight.toFixed(3),
      candidates.map((lm) => String(lm.name || "")).join("|"),
    ].join(":");
    if (!force && signature === this.landmarkLabelSignature) {
      return;
    }
    this.landmarkLabelSignature = signature;
    this.landmarkLabelMesh?.dispose(false, true);
    this.landmarkLabelMesh = null;
    if (!candidates.length) {
      return;
    }

    // One batched mesh keeps every visible LM name available without
    // thousands of individual Babylon planes. The largest bundled map has
    // fewer than 1,200 LMs, which fits inside a 4096 px-wide atlas.
    const atlasColumns = Math.min(32, candidates.length);
    const atlasRows = Math.ceil(candidates.length / atlasColumns);
    const cellWidth = 128;
    const cellHeight = 40;
    const textureWidth = atlasColumns * cellWidth;
    const textureHeight = atlasRows * cellHeight;
    const texture = new B.DynamicTexture(
      "landmark-label-atlas",
      { width: textureWidth, height: textureHeight },
      this.scene,
      false,
    );
    texture.hasAlpha = true;
    const context = texture.getContext();
    context.clearRect(0, 0, textureWidth, textureHeight);

    const positions = [];
    const indices = [];
    const uvs = [];
    const normals = [];
    candidates.forEach((lm, index) => {
      const column = index % atlasColumns;
      const row = Math.floor(index / atlasColumns);
      const cellX = column * cellWidth;
      const cellY = row * cellHeight;

      const text = String(lm.name || "");
      let fontSize = 18;
      context.font = `700 ${fontSize}px "Segoe UI", system-ui, sans-serif`;
      while (fontSize > 10 && context.measureText(text).width > cellWidth - 14) {
        fontSize -= 1;
        context.font = `700 ${fontSize}px "Segoe UI", system-ui, sans-serif`;
      }
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.lineJoin = "round";
      context.strokeStyle = "rgba(255,255,255,0.96)";
      context.lineWidth = 3;
      context.strokeText(text, cellX + (cellWidth / 2), cellY + (cellHeight / 2));
      context.fillStyle = "#143158";
      context.fillText(text, cellX + (cellWidth / 2), cellY + (cellHeight / 2));

      const x = Number(lm.x || 0);
      const z = Number(lm.y || 0) + (labelHeight * 0.76);
      const halfWidth = labelWidth / 2;
      const halfHeight = labelHeight / 2;
      const base = positions.length / 3;
      positions.push(
        x - halfWidth, 0.018, z + halfHeight,
        x + halfWidth, 0.018, z + halfHeight,
        x + halfWidth, 0.018, z - halfHeight,
        x - halfWidth, 0.018, z - halfHeight,
      );
      indices.push(base, base + 1, base + 2, base, base + 2, base + 3);

      const uMin = cellX / textureWidth;
      const uMax = (cellX + cellWidth) / textureWidth;
      const vMin = (textureHeight - cellY - cellHeight) / textureHeight;
      const vMax = (textureHeight - cellY) / textureHeight;
      uvs.push(uMin, vMin, uMax, vMin, uMax, vMax, uMin, vMax);
    });
    texture.update(true);
    B.VertexData.ComputeNormals(positions, indices, normals);

    const mesh = new B.Mesh("landmark-labels", this.scene);
    const vertexData = new B.VertexData();
    vertexData.positions = positions;
    vertexData.indices = indices;
    vertexData.uvs = uvs;
    vertexData.normals = normals;
    vertexData.applyToMesh(mesh);

    const material = new B.StandardMaterial("landmark-label-material", this.scene);
    material.diffuseTexture = texture;
    material.opacityTexture = texture;
    material.emissiveColor = B.Color3.White();
    material.disableLighting = true;
    material.backFaceCulling = false;
    material.useAlphaFromDiffuseTexture = true;
    material.specularColor = B.Color3.Black();
    mesh.material = material;
    mesh.parent = this.staticRoot;
    mesh.isPickable = false;
    mesh.metadata = { cameraDetailLabel: true };
    mesh.setEnabled(!this.cameraInteracting);
    mesh.freezeWorldMatrix();
    material.freeze();
    this.landmarkLabelMesh = mesh;
  }

  floorLabel(text, position, scale = 1) {
    const name = `lm-label-${String(text || "label")}`;
    const mesh = B.MeshBuilder.CreatePlane(name, { width: 0.82 * scale, height: 0.3 * scale }, this.scene);
    mesh.material = this.dynamicLabel(text, `${name}-texture`);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.set(position.x, 0.016, position.z + (0.17 * scale));
    mesh.parent = this.staticRoot;
    mesh.isPickable = false;
    mesh.metadata = { cameraDetailLabel: true };
    mesh.setEnabled(!this.cameraInteracting);
    return mesh;
  }

  labelSprite(text, yOffset, alertSeverity = "") {
    const multiline = String(text || "").includes("\n");
    const name = `robot-label-${Math.random().toString(36).slice(2)}`;
    const mesh = B.MeshBuilder.CreatePlane(name, {
      width: multiline ? 1.18 : 0.85,
      height: multiline ? 0.44 : 0.32,
    }, this.scene);
    mesh.material = this.dynamicLabel(text, `${name}-texture`, alertSeverity);
    mesh.position.set(0, 0.46 + yOffset, 0);
    mesh.billboardMode = B.Mesh.BILLBOARDMODE_ALL;
    mesh.isPickable = false;
    mesh.metadata = {
      labelText: String(text || ""),
      alertSeverity: String(alertSeverity || ""),
      isRobotLabel: true,
    };
    return mesh;
  }

  updateRobots(robots, selectedName = "", waitBlockerName = "") {
    const robotList = Array.isArray(robots) ? robots : [];
    this.latestRobots = robotList;
    this.latestSelectedRobotName = String(selectedName || "");
    this.latestWaitBlockerName = String(waitBlockerName || "");
    if (!this.scene) {
      this.pendingRobots = { robots: robotList, selectedName, waitBlockerName };
      return;
    }
    this.updateRenderQuality(robotList.length);
    const compactMode = robotList.length >= 40;
    if (compactMode !== this.compactRobotMode && this.robotObjects.size) {
      this.clearRobotObjects();
    }
    this.compactRobotMode = compactMode;
    const incoming = new Set();
    const showLabels = robotList.length <= 50;
    for (const robot of robotList) {
      const pose = this.robotPose(robot);
      if (!Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) {
        continue;
      }
      const name = String(robot.name || "");
      if (!name) {
        continue;
      }
      incoming.add(name);
      const active = name === String(selectedName || "");
      const waitBlocker = Boolean(waitBlockerName && name === String(waitBlockerName));
      const footprintKey = this.robotFootprintKey(robot);
      let entry = this.robotObjects.get(name);
      if (entry && entry.footprintKey !== footprintKey) {
        entry.group.dispose(false, true);
        this.robotObjects.delete(name);
        this.removeRobotRoute(name);
        entry = null;
      }
      if (!entry) {
        const group = this.robotMesh({ ...robot, pose }, active);
        entry = {
          group,
          footprintKey,
          active: null,
          waitBlocker: null,
          poseInterpolated: Boolean(robot?.poseInterpolated),
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
      entry.group.dispose(false, true);
      this.robotObjects.delete(name);
      this.removeRobotRoute(name);
    }
    this.requestRender();
  }

  clearRobotObjects() {
    for (const [name, entry] of this.robotObjects) {
      entry.group.dispose(false, true);
      this.removeRobotRoute(name);
    }
    this.robotObjects.clear();
  }

  updateRobotPoses(robots) {
    const robotList = Array.isArray(robots) ? robots : [];
    if (!this.scene || robotList.length !== this.robotObjects.size) {
      return false;
    }
    for (const robot of robotList) {
      const name = String(robot?.name || "");
      const entry = this.robotObjects.get(name);
      const pose = this.robotPose(robot);
      if (!entry || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) {
        return false;
      }
      entry.targetPose = {
        x: Number(pose.x || 0),
        y: Number(pose.y || 0),
        yaw: Number(pose.yaw || 0),
      };
      entry.poseInterpolated = Boolean(robot?.poseInterpolated);
    }
    return true;
  }

  updateRenderQuality(robotCount) {
    const cap = Number(robotCount || 0) >= 40 ? 1.0 : 1.35;
    const desired = Math.min(cap, window.devicePixelRatio || 1);
    if (!this.engine || Math.abs(desired - this.renderPixelRatio) < 0.01) {
      return;
    }
    this.renderPixelRatio = desired;
    this.applyHardwareScaling();
    this.resize();
  }

  updateRobotObject(entry, robot, active, waitBlocker, showLabel) {
    const pose = robot.pose || {};
    entry.targetPose = {
      x: Number(pose.x || 0),
      y: Number(pose.y || 0),
      yaw: Number(pose.yaw || 0),
    };
    entry.poseInterpolated = Boolean(robot?.poseInterpolated);
    const alertText = this.robotAlertLabel(robot);
    const alertSeverity = this.robotAlertSeverity(robot);
    const labelText = alertText
      ? `${String(robot.name || "")}\n${alertText}`
      : String(robot.name || "");
    const needsLabel = showLabel || active || Boolean(alertText);
    let label = entry.group.metadata.label;
    if (needsLabel && (!label || (
      label.metadata?.labelText !== labelText
      || label.metadata?.alertSeverity !== alertSeverity
    ))) {
      label?.dispose(false, true);
      label = this.labelSprite(labelText, 0, alertSeverity);
      label.parent = entry.group;
      entry.group.metadata.label = label;
    }
    if (label) {
      label.isVisible = needsLabel;
      label.setEnabled(!this.cameraInteracting);
    }
    if (entry.active === active && entry.waitBlocker === waitBlocker) {
      return;
    }
    entry.active = active;
    entry.waitBlocker = waitBlocker;
    const metadata = entry.group.metadata;
    setMaterialColor(metadata.bodyMaterial, COLORS.ecomBody);
    const selectionColor = waitBlocker ? 0xff7a00 : (metadata.selectionColor || COLORS.robot);
    if (metadata.footprintOutline) {
      metadata.footprintOutline.color = toColor3(selectionColor);
      metadata.footprintOutline.alpha = active || waitBlocker ? 1 : 0.82;
    }
    if (metadata.footprintHeading) {
      metadata.footprintHeading.color = toColor3(selectionColor);
    }
    setMaterialColor(metadata.underglowMaterial, selectionColor);
    metadata.underglowMaterial.alpha = active ? 0.72 : (waitBlocker ? 0.5 : 0.12);
    if (!active) {
      metadata.underglowMesh.scaling.set(1, 1, 1);
    }
    setMaterialColor(metadata.selectionHaloMaterial, selectionColor);
    setMaterialColor(metadata.selectionRingMaterial, selectionColor);
    metadata.selectionHaloMesh.isVisible = active || waitBlocker;
    metadata.selectionRingMesh.isVisible = active || waitBlocker;
    metadata.selectionHaloMesh.scaling.set(1, 1, 1);
    metadata.selectionRingMesh.scaling.set(1, 1, 1);
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
    if (reason === "rolling continuation pending") {
      return "planning next route segment";
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
    // Real gRPC status snapshots do not own the simulation's display-clock
    // interpolation, so retain a soft follow for those lower-rate poses.
    // Fleet Manager Sim entries are tagged poseInterpolated and bypass it.
    const alpha = 1 - Math.exp(-14 * dt);
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
      if (entry.poseInterpolated) {
        // The application already supplies a 60 Hz display-clock pose for
        // simulated robots. Babylon must still render whenever that direct
        // target changed; otherwise its on-demand loop sees no animation and
        // presents only occasional selection/UI frames.
        if (distanceSq > 0.00000001 || Math.abs(rotationDelta) > 0.0001) {
          animating = true;
        }
        group.position.set(Number(target.x || 0), 0, Number(target.y || 0));
        group.rotation.y = targetRotation;
        continue;
      }
      if (distanceSq <= 0.00000001 && Math.abs(rotationDelta) <= 0.0001) {
        group.position.set(Number(target.x || 0), 0, Number(target.y || 0));
        group.rotation.y = targetRotation;
        continue;
      }
      animating = true;
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
    return lm ? {
      x: Number(lm.x || 0),
      y: Number(lm.y || 0),
      yaw: Number(pose.yaw || 0),
    } : {};
  }

  robotFootprint(robot) {
    const raw = Array.isArray(robot?.footprint)
      ? robot.footprint
      : (Array.isArray(robot?.robotModel?.footprint) ? robot.robotModel.footprint : []);
    const footprint = raw
      .map((point) => ({
        x: Number(point?.x),
        y: Number(point?.y),
      }))
      .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
    return footprint.length >= 3
      ? footprint
      : [
        { x: -0.523, y: -0.3532 },
        { x: 0.477, y: -0.3532 },
        { x: 0.477, y: 0.3468 },
        { x: -0.523, y: 0.3468 },
      ];
  }

  robotFootprintKey(robot) {
    return this.robotFootprint(robot)
      .map((point) => `${point.x.toFixed(4)},${point.y.toFixed(4)}`)
      .join(";");
  }

  robotMesh(robot, active) {
    const pose = robot.pose || {};
    const group = new B.TransformNode(`robot-${String(robot.name || "robot")}`, this.scene);
    group.position.set(Number(pose.x || 0), 0, Number(pose.y || 0));
    group.rotation.y = -Number(pose.yaw || 0);
    group.parent = this.robotRoot;
    group.metadata = { robotName: String(robot.name || "") };

    if (this.viewMode === "2d") {
      this.addFootprintModel(group, robot, active);
    } else {
      this.addEcomModel(group, robot, active);
    }
    return group;
  }

  markRobotMesh(mesh, group) {
    mesh.parent = group;
    mesh.isPickable = true;
    mesh.metadata = { robotName: String(group.metadata?.robotName || "") };
    return mesh;
  }

  flatFootprintMesh(name, footprint, height, material, group, pickable = false) {
    const positions = [];
    const indices = [];
    for (const point of footprint) {
      // Robot-local +Y points to screen-left in the legacy SVG convention.
      // Babylon's local Z is therefore the negated robot-model Y coordinate.
      positions.push(Number(point.x), height, -Number(point.y));
    }
    for (let index = 1; index < footprint.length - 1; index += 1) {
      indices.push(0, index, index + 1);
    }
    const normals = [];
    B.VertexData.ComputeNormals(positions, indices, normals);
    const mesh = new B.Mesh(name, this.scene);
    const vertexData = new B.VertexData();
    vertexData.positions = positions;
    vertexData.indices = indices;
    vertexData.normals = normals;
    vertexData.applyToMesh(mesh);
    mesh.material = material;
    mesh.parent = group;
    mesh.isPickable = pickable;
    mesh.metadata = { robotName: String(group.metadata?.robotName || "") };
    return mesh;
  }

  addFootprintModel(group, robot, active) {
    const footprint = this.robotFootprint(robot);
    const selectionColor = this.robotColor(robot?.name);
    const body = this.unlitMaterial(`${group.name}-footprint-body`, COLORS.ecomBody, 0.86);
    const underglow = this.unlitMaterial(
      `${group.name}-footprint-underglow`,
      selectionColor,
      active ? 0.62 : 0.10,
    );
    const selectionHalo = this.unlitMaterial(`${group.name}-footprint-halo`, selectionColor, 0.34);
    const selectionRing = this.unlitMaterial(`${group.name}-footprint-ring`, selectionColor, 0.88);
    const underglowMesh = this.flatFootprintMesh(
      `${group.name}-footprint-underglow`,
      footprint,
      0.014,
      underglow,
      group,
      false,
    );
    underglowMesh.scaling.set(1.10, 1, 1.10);
    const footprintMesh = this.flatFootprintMesh(
      `${group.name}-footprint`,
      footprint,
      0.028,
      body,
      group,
      true,
    );

    const outlinePoints = footprint.map(
      (point) => new B.Vector3(Number(point.x), 0.034, -Number(point.y)),
    );
    outlinePoints.push(outlinePoints[0].clone());
    const outline = B.MeshBuilder.CreateLines(`${group.name}-footprint-outline`, {
      points: outlinePoints,
      updatable: false,
    }, this.scene);
    outline.color = toColor3(selectionColor);
    outline.alpha = active ? 1 : 0.82;
    outline.parent = group;
    outline.isPickable = false;

    const frontX = Math.max(...footprint.map((point) => Number(point.x)));
    const minY = Math.min(...footprint.map((point) => Number(point.y)));
    const maxY = Math.max(...footprint.map((point) => Number(point.y)));
    const arrowHalfWidth = Math.max(0.035, Math.min(0.10, (maxY - minY) * 0.18));
    const arrowTip = Math.max(0.08, frontX * 0.78);
    const arrowBase = arrowTip - Math.max(0.06, Math.abs(frontX) * 0.20);
    const heading = B.MeshBuilder.CreateLineSystem(`${group.name}-footprint-heading`, {
      lines: [
        [
          new B.Vector3(Math.min(0, arrowBase), 0.038, 0),
          new B.Vector3(arrowTip, 0.038, 0),
        ],
        [
          new B.Vector3(arrowBase, 0.038, arrowHalfWidth),
          new B.Vector3(arrowTip, 0.038, 0),
          new B.Vector3(arrowBase, 0.038, -arrowHalfWidth),
        ],
      ],
    }, this.scene);
    heading.color = toColor3(selectionColor);
    heading.alpha = 0.92;
    heading.parent = group;
    heading.isPickable = false;

    const radius = Math.max(
      0.22,
      ...footprint.map((point) => Math.hypot(Number(point.x), Number(point.y))),
    );
    const selectionHaloMesh = B.MeshBuilder.CreateDisc(`${group.name}-selection-halo`, {
      radius: radius + 0.08,
      tessellation: 48,
      sideOrientation: B.Mesh.DOUBLESIDE,
    }, this.scene);
    selectionHaloMesh.rotation.x = -Math.PI / 2;
    selectionHaloMesh.position.y = 0.010;
    selectionHaloMesh.material = selectionHalo;
    selectionHaloMesh.isVisible = active;
    selectionHaloMesh.isPickable = false;
    selectionHaloMesh.parent = group;

    const selectionRingMesh = this.createRing(
      `${group.name}-selection-ring`,
      radius + 0.055,
      radius + 0.09,
      48,
      selectionRing,
    );
    selectionRingMesh.position.y = 0.040;
    selectionRingMesh.isVisible = active;
    selectionRingMesh.parent = group;

    group.metadata.bodyMaterial = body;
    group.metadata.footprintMesh = footprintMesh;
    group.metadata.footprintOutline = outline;
    group.metadata.footprintHeading = heading;
    group.metadata.underglowMaterial = underglow;
    group.metadata.underglowMesh = underglowMesh;
    group.metadata.selectionColor = selectionColor;
    group.metadata.selectionHaloMaterial = selectionHalo;
    group.metadata.selectionHaloMesh = selectionHaloMesh;
    group.metadata.selectionRingMaterial = selectionRing;
    group.metadata.selectionRingMesh = selectionRingMesh;
  }

  addEcomModel(group, robot, active) {
    const selectionColor = this.robotColor(robot?.name);
    const body = this.pbrMaterial(`${group.name}-body`, COLORS.ecomBody, 0.12, 0.48);
    const deck = this.pbrMaterial(`${group.name}-deck`, COLORS.ecomDeck, 0.28, 0.52);
    const wheel = this.compactRobotMode
      ? null
      : this.pbrMaterial(`${group.name}-wheel`, COLORS.wheel, 0.04, 0.72);
    const lidar = this.pbrMaterial(`${group.name}-lidar`, COLORS.lidar, 0.32, 0.34);
    const frontPanel = this.pbrMaterial(`${group.name}-front-panel`, COLORS.frontPanel, 0.08, 0.62);
    const underglow = this.unlitMaterial(`${group.name}-underglow`, selectionColor, active ? 0.72 : 0.12);
    const selectionHalo = this.unlitMaterial(`${group.name}-halo`, selectionColor, 0.38);
    const selectionRing = this.unlitMaterial(`${group.name}-ring`, selectionColor, 0.82);

    const addBox = (name, size, position, material) => {
      const mesh = B.MeshBuilder.CreateBox(`${group.name}-${name}`, {
        width: size.x,
        height: size.z,
        depth: size.y,
      }, this.scene);
      mesh.position.set(position.x, position.z, position.y);
      mesh.material = material;
      return this.markRobotMesh(mesh, group);
    };
    const addCylinder = (name, radius, length, position, material, rotation = null) => {
      const mesh = B.MeshBuilder.CreateCylinder(`${group.name}-${name}`, {
        diameter: radius * 2,
        height: length,
        tessellation: 24,
      }, this.scene);
      mesh.position.set(position.x, position.z, position.y);
      if (rotation) {
        mesh.rotation.set(rotation.x || 0, rotation.y || 0, rotation.z || 0);
      }
      mesh.material = material;
      return this.markRobotMesh(mesh, group);
    };
    const addExtrudedPolygon = (name, outline, height, baseZ, material) => {
      const positions = [];
      const indices = [];
      const count = outline.length;
      for (const point of outline) {
        positions.push(point.x, baseZ, point.y);
      }
      for (const point of outline) {
        positions.push(point.x, baseZ + height, point.y);
      }
      for (let index = 1; index < count - 1; index += 1) {
        indices.push(0, index + 1, index);
        indices.push(count, count + index, count + index + 1);
      }
      for (let index = 0; index < count; index += 1) {
        const next = (index + 1) % count;
        indices.push(index, next, count + next);
        indices.push(index, count + next, count + index);
      }
      const normals = [];
      B.VertexData.ComputeNormals(positions, indices, normals);
      const mesh = new B.Mesh(`${group.name}-${name}`, this.scene);
      const vertexData = new B.VertexData();
      vertexData.positions = positions;
      vertexData.indices = indices;
      vertexData.normals = normals;
      vertexData.applyToMesh(mesh);
      mesh.material = material;
      return this.markRobotMesh(mesh, group);
    };

    group.metadata.bodyMaterial = body;
    group.metadata.underglowMaterial = underglow;
    group.metadata.selectionColor = selectionColor;
    group.metadata.selectionHaloMaterial = selectionHalo;
    group.metadata.selectionRingMaterial = selectionRing;

    // A light procedural representation of ecom_stage.urdf.xacro keeps a
    // dense fleet responsive while retaining the real robot footprint.
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
    const glowOutline = bodyOutline.map((point) => ({ x: point.x * 1.07, y: point.y * 1.10 }));
    const underglowMesh = addExtrudedPolygon("underglow", glowOutline, 0.008, 0.006, underglow);
    underglowMesh.isPickable = false;
    group.metadata.underglowMesh = underglowMesh;

    const selectionHaloMesh = B.MeshBuilder.CreateDisc(`${group.name}-selection-halo`, {
      radius: 0.64,
      tessellation: 56,
      sideOrientation: B.Mesh.DOUBLESIDE,
    }, this.scene);
    selectionHaloMesh.rotation.x = -Math.PI / 2;
    selectionHaloMesh.position.y = 0.012;
    selectionHaloMesh.material = selectionHalo;
    selectionHaloMesh.isVisible = active;
    selectionHaloMesh.isPickable = false;
    selectionHaloMesh.parent = group;
    group.metadata.selectionHaloMesh = selectionHaloMesh;

    const selectionRingMesh = this.createRing(`${group.name}-selection-ring`, 0.61, 0.68, 56, selectionRing);
    selectionRingMesh.position.y = 0.014;
    selectionRingMesh.isVisible = active;
    selectionRingMesh.parent = group;
    group.metadata.selectionRingMesh = selectionRingMesh;

    addExtrudedPolygon("body", bodyOutline, 0.170, 0.0, body);
    addExtrudedPolygon("deck", deckOutline, 0.045, 0.160, deck);
    addCylinder("lidar-front", 0.0337, 0.042, { x: 0.32487, y: 0.24906, z: 0.218 }, lidar);
    addCylinder("lidar-rear", 0.0337, 0.042, { x: -0.41524, y: -0.25105, z: 0.218 }, lidar);
    addBox("front-panel", { x: 0.034, y: 0.275, z: 0.056 }, { x: 0.464, y: 0, z: 0.126 }, frontPanel);
    if (!this.compactRobotMode) {
      addCylinder("wheel-left", 0.09, 0.057, { x: -0.043, y: 0.300, z: 0.060 }, wheel, { x: Math.PI / 2 });
      addCylinder("wheel-right", 0.09, 0.057, { x: -0.043, y: -0.300, z: 0.060 }, wheel, { x: Math.PI / 2 });
      addBox("front-led", { x: 0.055, y: 0.26, z: 0.018 }, { x: 0.438, y: 0, z: 0.166 }, underglow);
    }
  }

  createRing(name, innerRadius, outerRadius, segments, material) {
    const positions = [];
    const indices = [];
    for (let index = 0; index <= segments; index += 1) {
      const angle = (index / segments) * Math.PI * 2;
      const cosine = Math.cos(angle);
      const sine = Math.sin(angle);
      positions.push(innerRadius * cosine, 0, innerRadius * sine);
      positions.push(outerRadius * cosine, 0, outerRadius * sine);
      if (index < segments) {
        const start = index * 2;
        indices.push(start, start + 3, start + 1, start, start + 2, start + 3);
      }
    }
    const normals = [];
    B.VertexData.ComputeNormals(positions, indices, normals);
    const mesh = new B.Mesh(name, this.scene);
    const vertexData = new B.VertexData();
    vertexData.positions = positions;
    vertexData.indices = indices;
    vertexData.normals = normals;
    vertexData.applyToMesh(mesh);
    mesh.material = material;
    mesh.isPickable = false;
    return mesh;
  }

  updateRobotRoute(robot, active) {
    const name = String(robot?.name || "");
    if (!name) {
      return;
    }
    // The fleet overview must not turn into a second line grid. Only the
    // selected robot owns a visible route; graph edges already describe the
    // rest of the traffic topology.
    if (!active) {
      this.removeRobotRoute(name);
      return;
    }
    const trajectory = this.futureRobotTrajectory(robot, active);
    if (trajectory.length < 2) {
      this.removeRobotRoute(name);
      return;
    }
    const routeKey = this.robotRouteKey(robot, active, trajectory);
    if (this.robotRouteKeys.get(name) === routeKey) {
      return;
    }
    this.removeRobotRoute(name);
    const maxPoints = active
      ? this.maxActiveRoutePoints
      : Math.min(256, this.maxActiveRoutePoints);
    const routeHeight = this.viewMode === "2d" ? 0.048 : 0.095;
    const points = this.sampleTrajectory(trajectory, maxPoints)
      .map((point) => new B.Vector3(Number(point.x || 0), routeHeight, Number(point.y || 0)));
    const material = this.unlitMaterial(
      `route-${name}-material`,
      this.robotColor(name),
      this.viewMode === "2d" ? 0.94 : 0.82,
    );
    const routeObject = this.routeRibbonGeometry(
      points,
      this.viewMode === "2d" ? 0.13 : 0.105,
      material,
      `route-${name}`,
    );
    routeObject.isPickable = false;
    routeObject.parent = this.routeRoot;
    this.robotRouteObjects.set(name, routeObject);
    this.robotRouteKeys.set(name, routeKey);
  }

  futureRobotTrajectory(robot, active) {
    const trajectory = Array.isArray(robot?.trajectory) ? robot.trajectory : [];
    const preview = active && Array.isArray(robot?.routePreview) ? robot.routePreview : [];
    const result = [];
    const append = (point) => {
      const next = {
        ...point,
        x: Number(point?.x),
        y: Number(point?.y),
      };
      if (!Number.isFinite(next.x) || !Number.isFinite(next.y)) {
        return;
      }
      const previous = result[result.length - 1];
      if (
        previous
        && Math.hypot(next.x - previous.x, next.y - previous.y) < 0.001
      ) {
        return;
      }
      result.push(next);
    };
    if (preview.some((point) => String(point?.phase || "") === "forecast")) {
      for (const point of trajectory) {
        append(point);
      }
      for (const point of preview) {
        if (String(point?.phase || "") === "forecast") {
          append(point);
        }
      }
      return result;
    }

    const source = preview.length >= 2 ? preview : trajectory;
    for (const point of source) {
      append(point);
    }
    return result;
  }

  routeRibbonGeometry(points, width, material, name) {
    const positions = [];
    const indices = [];
    const halfWidth = width * 0.5;
    const direction = (from, to) => {
      const result = new B.Vector3(to.x - from.x, 0, to.z - from.z);
      return result.lengthSquared() > 0.00000001 ? result.normalize() : null;
    };
    let fallbackDirection = new B.Vector3(1, 0, 0);
    for (let index = 0; index < points.length; index += 1) {
      const previous = index > 0 ? direction(points[index - 1], points[index]) : null;
      const next = index < points.length - 1 ? direction(points[index], points[index + 1]) : null;
      const incoming = previous || next || fallbackDirection;
      const outgoing = next || previous || fallbackDirection;
      fallbackDirection = outgoing;
      const tangent = incoming.add(outgoing);
      if (tangent.lengthSquared() < 0.00000001) {
        tangent.copyFrom(outgoing);
      }
      tangent.normalize();
      const normal = new B.Vector3(-tangent.z, 0, tangent.x);
      const incomingNormal = new B.Vector3(-incoming.z, 0, incoming.x);
      const denominator = Math.max(0.35, Math.abs(B.Vector3.Dot(normal, incomingNormal)));
      const offset = Math.min(halfWidth / denominator, halfWidth * 2.4);
      positions.push(
        points[index].x + (normal.x * offset), points[index].y, points[index].z + (normal.z * offset),
        points[index].x - (normal.x * offset), points[index].y, points[index].z - (normal.z * offset),
      );
      if (index > 0) {
        const previousLeft = (index - 1) * 2;
        const previousRight = previousLeft + 1;
        const left = index * 2;
        const right = left + 1;
        indices.push(previousLeft, previousRight, left, previousRight, right, left);
      }
    }
    const normals = [];
    B.VertexData.ComputeNormals(positions, indices, normals);
    const mesh = new B.Mesh(name, this.scene);
    const vertexData = new B.VertexData();
    vertexData.positions = positions;
    vertexData.indices = indices;
    vertexData.normals = normals;
    vertexData.applyToMesh(mesh);
    mesh.material = material;
    return mesh;
  }

  robotRouteKey(robot, active, trajectory = null) {
    const route = Array.isArray(trajectory)
      ? trajectory
      : this.futureRobotTrajectory(robot, active);
    const first = route[0] || {};
    const last = route[route.length - 1] || {};
    return [
      active ? "active" : "idle",
      this.viewMode,
      robot?.routeRevision || "",
      robot?.routeChunkIndex || "",
      Array.isArray(robot?.routePreview) ? robot.routePreview.length : 0,
      route.length,
      first.t ?? "",
      Number(first.x || 0).toFixed(3),
      Number(first.y || 0).toFixed(3),
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
    route.dispose(false, true);
    this.robotRouteObjects.delete(name);
    this.robotRouteKeys.delete(name);
  }

  updateCamera() {
    if (!this.camera || !this.orthoCamera || !this.scene) {
      return;
    }
    const horizontal = Math.cos(this.pitch) * this.distance;
    const height = Math.sin(this.pitch) * this.distance;
    this.camera.position.set(
      this.target.x + (Math.sin(this.yaw) * horizontal),
      this.target.y + height,
      this.target.z + (Math.cos(this.yaw) * horizontal),
    );
    this.camera.setTarget(this.target);
    this.updateOrthoFrustum();
    this.orthoCamera.position.set(this.target.x, this.target.y + Math.max(6, this.distance), this.target.z);
    this.orthoCamera.setTarget(this.target);
    const topDown = this.isTopDown();
    const nextCamera = topDown ? this.orthoCamera : this.camera;
    if (this.activeCamera !== nextCamera) {
      this.activeCamera = nextCamera;
      this.scene.activeCamera = nextCamera;
    }
    this.lastCameraModeTopDown = topDown;
  }

  requestRender() {
    this.needsRender = true;
  }

  updateSelectionAnimation(timestamp) {
    let animating = false;
    const pulse = 0.5 + (0.5 * Math.sin(Number(timestamp || 0) * 0.005));
    for (const entry of this.robotObjects.values()) {
      const metadata = entry.group.metadata;
      if ((!entry.active && !entry.waitBlocker) || !metadata?.underglowMesh) {
        continue;
      }
      animating = true;
      const scale = 1.02 + (pulse * (entry.waitBlocker ? 0.14 : 0.1));
      metadata.underglowMesh.scaling.set(scale, 1, scale);
      metadata.underglowMaterial.alpha = (entry.waitBlocker ? 0.42 : 0.58) + (pulse * 0.24);
      const haloScale = 0.98 + (pulse * 0.08);
      metadata.selectionHaloMesh.scaling.set(haloScale, haloScale, 1);
      metadata.selectionHaloMaterial.alpha = 0.3 + (pulse * 0.18);
      const ringScale = 0.96 + (pulse * 0.12);
      metadata.selectionRingMesh.scaling.set(ringScale, 1, ringScale);
      metadata.selectionRingMaterial.alpha = 0.62 + (pulse * 0.28);
    }
    return animating;
  }

  animate(timestamp = 0) {
    if (this.disposed || !this.scene) {
      return;
    }
    this.animationFrame = window.requestAnimationFrame((nextTimestamp) => this.animate(nextTimestamp));
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
    this.scene.render();
  }

  dispose() {
    this.disposed = true;
    this.occupancyWallBuildGeneration += 1;
    this.cancelOccupancyWallWorker("3D scene disposed.");
    window.cancelAnimationFrame(this.animationFrame);
    window.clearTimeout(this.cameraInteractionRestoreTimer);
    window.clearTimeout(this.landmarkLabelRefreshTimer);
    this.resizeObserver?.disconnect();
    this.scene?.dispose();
    this.engine?.dispose();
    this.canvas.remove();
    this.engineBadge.remove();
  }
}
