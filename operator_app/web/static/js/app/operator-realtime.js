export const withRealtime = (Base) => class OperatorAppRealtime extends Base {
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
      this.latestScanFrame = null;
      this.clearScanOverlay();
      this.syncScanUi("off");
    });
  }

  scanWsUrl(robot) {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    return `${protocol}//${window.location.host}/ws/robot/scan?robotId=${encodeURIComponent(robot.id)}&hz=5`;
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
      this.latestScanFrame = null;
      this.clearScanOverlay();
      this.syncScanUi(payload.error || "waiting");
      return;
    }
    this.latestScanFrame = payload;
    this.drawScanOverlay(payload);
    const count = this.scanPointCloud(payload)?.points.length || 0;
    this.syncScanUi(`${count} pts`);
  }

  syncScanUi(statusText = "") {
    const robot = this.selectedRobot();
    this.scanToggleButton?.classList.toggle("primary", this.scanEnabled);
    this.scanToggleButton?.classList.toggle("hidden", !robot || this.isFleetManager(robot) || this.isRobotModelPage() || this.isParamsPage());
    if (this.scanToggleButton) {
      this.scanToggleButton.textContent = this.scanEnabled ? "Scan Off" : "Scan";
      this.scanToggleButton.title = statusText
        ? `Live gRPC point cloud: ${statusText}`
        : "Show the live gRPC point cloud on the map";
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
    this.scene3d?.clearScanPointCloud();
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
    const cloud = this.scanPointCloud(frame);
    if (!cloud) {
      this.scene3d?.clearScanPointCloud();
      return;
    }
    if (this.scene3d?.scene && !this.babylonMapFailed && !this.slamActive) {
      this.scene3d.setScanPointCloud(cloud.points, {
        frameId: frame.frameId,
        stampSec: frame.stampSec,
      });
      return;
    }
    this.scene3d?.clearScanPointCloud();
    const segments = cloud.points.map((world) => {
      const point = this.worldToPixel(world);
      return `M ${point.x.toFixed(2)} ${point.y.toFixed(2)} h 0.01`;
    });
    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("class", "scan-point-cloud");
    path.setAttribute("d", segments.join(" "));
    this.operatorScanLayer.append(path);
  }

  scanPointCloud(frame = this.latestScanFrame) {
    if (!this.scanEnabled || this.isFleetManager() || !frame || !frame.ok) {
      return null;
    }
    const payload = this.activeOperatorMapPayload();
    if (!payload || !payload.map) {
      return null;
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
      return null;
    }
    const ranges = Array.isArray(frame.ranges) ? frame.ranges : [];
    if (!ranges.length) {
      return null;
    }
    const rangeMin = Math.max(0, Number(frame.rangeMin || 0));
    const rangeMax = Number(frame.rangeMax || 0) > 0 ? Number(frame.rangeMax) : Infinity;
    const angleMin = Number(frame.angleMin || 0);
    const angleIncrement = Number(frame.angleIncrement || 0);
    const sensorPose = this.scanSensorPose(pose, frame);
    const yaw = sensorPose.yaw;
    const originX = sensorPose.x;
    const originY = sensorPose.y;
    const height = Math.max(0.05, Number(sensorPose.z || 0.24));
    const points = [];
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
      points.push({
        x: originX + (Math.cos(angle) * range),
        y: originY + (Math.sin(angle) * range),
        height,
      });
    }
    if (!points.length) {
      return null;
    }
    return {
      points,
      origin: { x: originX, y: originY, height },
    };
  }

  scanSensorPose(pose, frame = {}) {
    const baseX = Number(pose.x || 0);
    const baseY = Number(pose.y || 0);
    const yaw = Number(pose.yaw || 0);
    const sensor = this.scanSensorFrame(frame);
    if (!sensor) {
      return { x: baseX, y: baseY, z: 0.24, yaw };
    }
    const offsetX = Number(sensor.x || 0);
    const offsetY = Number(sensor.y || 0);
    const offsetYaw = Number(sensor.yaw || sensor.theta || 0);
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    return {
      x: baseX + (offsetX * cos) + (offsetY * sin),
      y: baseY + (offsetX * sin) - (offsetY * cos),
      z: Number(sensor.z || 0.24),
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
      this.currentStatus = this.mergeFleetTickState(state);
      this.fleetRuntimeUiLastAt = performance.now();
      this.renderFleetRuntimeTick();
      this.ensureFleetAnimationLoop();
      return;
    }
    this.currentStatus = this.mergeFleetTickState(state);
    this.ensureFleetAnimationLoop();
    // Robot poses are rendered from currentStatus on requestAnimationFrame.
    // Rebuilding the fleet list, queue, inspector and map on every 20 Hz
    // status packet only steals frame time, so keep those heavier panels at
    // a human-readable 5 Hz without delaying robot motion.
    const now = performance.now();
    if (now - this.fleetRuntimeUiLastAt >= 200) {
      this.fleetRuntimeUiLastAt = now;
      this.renderFleetRuntimeTick();
    }
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
      // requestAnimationFrame is already the display clock. A second
      // `>= 1000 / 60` gate drops every other frame on many 60 Hz displays
      // because the callback arrives fractionally before 16.667 ms.
      this.fleetAnimationLastAt = now;
      this.drawFleetAnimationFrame(now);
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
    // Keep transforms on the display clock, but do not repeat labels,
    // selection state and route topology work sixty times per second. Those
    // are control-plane values and 8 Hz is already faster than a human can
    // perceive their changes. This is especially important with 50 robots:
    // DOM and Babylon.js scene-graph updates run on one browser main thread.
    if (!this.babylonMapFailed && !this.slamActive) {
      const motionOnly = now - this.fleetVisualControlLastAt < 125;
      if (!motionOnly) {
        this.fleetVisualControlLastAt = now;
      }
      if (!this.updateOperatorScene3dRobots(this.scene3d, { motionOnly })) {
        this.renderOperatorBabylonMap({ motionOnly });
      }
      return;
    }
    const robotCount = Array.isArray(this.currentStatus?.robots)
      ? this.currentStatus.robots.length
      : 0;
    const minimum2dFrameMs = robotCount >= 80 ? 1000 / 30 : 0;
    if (minimum2dFrameMs && now - this.fleet2dMotionLastAt < minimum2dFrameMs) {
      return;
    }
    this.fleet2dMotionLastAt = now;
    const motionOnly = now - this.fleetVisualControlLastAt < 125;
    if (!motionOnly) {
      this.fleetVisualControlLastAt = now;
    }
    this.drawRobot(motionOnly);
    if (now - this.fleetRouteRenderLastAt >= 180) {
      this.fleetRouteRenderLastAt = now;
      this.drawRoute();
      this.drawLookahead();
      this.syncMapControls();
    }
  }

  fleetNeedsAnimation() {
    if (!this.isFleetManager() || this.fleetMapEditorActive) {
      return false;
    }
    // A disconnected or stopped server can leave the last snapshot in
    // MOVING state. Do not keep a 60 FPS browser loop alive forever for stale
    // robots; the next websocket/HTTP state packet restarts it immediately.
    if (
      !this.fleetStatusReceivedAt
      || performance.now() - this.fleetStatusReceivedAt > this.fleetStatusFreshTimeoutMs
    ) {
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

  fleetVisualRouteKey(robot) {
    // Keep the visual clock across an atomic rolling append or a route
    // revision for the same order. A revision is control-plane metadata, not
    // a reason to stop the rendered robot for one frame.
    return [
      String(robot?.name || ""),
      String(robot?.activeOrderId || ""),
      String(robot?.routeFinalLm || robot?.routeChunkGoalLm || ""),
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
        activeKeys.add(this.fleetVisualRouteKey(robot));
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
    const key = this.fleetVisualRouteKey(robot);
    const prior = this.fleetVisualClocks.get(key) || null;
    const now = performance.now();
    const priorServerClock = prior ? Math.max(0, Number(prior.serverClock || 0)) : 0;
    const routeClockReset = Boolean(prior && baseClock < priorServerClock - 0.25);
    let visualClock = baseClock;
    let serverUpdatedAt = now;
    if (prior && !routeClockReset) {
      const priorClock = Math.max(0, Number(prior.clock || 0));
      const timeScale = Math.max(1, Number(this.currentStatus?.simulationTimeScale || 1));
      // Collision checks run ahead of committed motion. Continue at nominal
      // speed between confirmed clocks instead of freezing and then driving
      // 15% faster to compensate for a delayed packet. The lead is bounded
      // and never leaves the already committed graph trajectory.
      serverUpdatedAt = Math.abs(baseClock - priorServerClock) > 0.0001
        ? now
        : Number(prior.serverUpdatedAt || prior.updatedAt || now);
      const packetAgeSec = Math.max(
        0,
        (now - serverUpdatedAt) / 1000,
      );
      const maximumLeadClock = Math.min(
        1.2,
        this.fleetNavigationPredictionMaxSec * timeScale,
      );
      const visualLeadSec = Math.min(
        maximumLeadClock,
        packetAgeSec * timeScale,
      );
      const targetClock = Math.min(finalTime, baseClock + visualLeadSec);
      const frameDelta = status === "MOVING"
        ? Math.min(
          0.12 * timeScale,
          Math.max(0, (now - Number(prior.updatedAt || now)) / 1000)
            * timeScale,
        )
        : 0;
      visualClock = targetClock >= priorClock
        ? Math.min(targetClock, priorClock + frameDelta)
        : priorClock;
    }
    visualClock = Math.min(finalTime, Math.max(0, visualClock));
    this.fleetVisualClocks.set(key, {
      clock: visualClock,
      serverClock: baseClock,
      serverUpdatedAt,
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
    // Manual rendering is driven locally while the safety ACK is in flight.
    // A bounded 0.75 s window absorbs a busy fleet tick without allowing the
    // visual pose to run indefinitely ahead of backend collision validation.
    const elapsed = Math.min(0.75, Math.max(0, (performance.now() - animation.startedAt) / 1000));
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
};
