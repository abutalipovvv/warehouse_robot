class OperatorApp {
  constructor() {
    this.robots = [];
    this.selectedRobotId = window.localStorage.getItem("operator:selectedRobotId") || "";
    this.lastProbe = null;
    this.sidebarOpen = !this.selectedRobotId;
    this.robotMapState = {
      robotActiveMapName: "",
      operatorActiveMapName: "",
      robotSignature: "",
      operatorSignature: "",
      sourceRobotMapName: "",
      hasLocalChanges: false,
    };

    this.robotsList = document.getElementById("robotsList");
    this.robotCountText = document.getElementById("robotCountText");
    this.sidebarDrawer = document.getElementById("sidebarDrawer");
    this.sidebarBackdrop = document.getElementById("sidebarBackdrop");
    this.openSidebarButton = document.getElementById("openSidebarButton");
    this.closeSidebarButton = document.getElementById("closeSidebarButton");
    this.emptyState = document.getElementById("emptyState");
    this.robotView = document.getElementById("robotView");
    this.robotFrame = document.getElementById("robotFrame");
    this.robotActiveMapText = document.getElementById("robotActiveMapText");
    this.operatorActiveMapText = document.getElementById("operatorActiveMapText");
    this.editMapButton = document.getElementById("editMapButton");
    this.controlPullMapButton = document.getElementById("controlPullMapButton");
    this.controlPushMapButton = document.getElementById("controlPushMapButton");
    this.controlLoadMapButton = document.getElementById("controlLoadMapButton");
    this.mapSyncStatus = document.getElementById("mapSyncStatus");
    this.refreshButton = document.getElementById("refreshButton");
    this.addRobotButton = document.getElementById("addRobotButton");

    this.addRobotDialog = document.getElementById("addRobotDialog");
    this.closeDialogButton = document.getElementById("closeDialogButton");
    this.robotNameInput = document.getElementById("robotNameInput");
    this.robotHostInput = document.getElementById("robotHostInput");
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

    this.frameUrl = "";
    this.frameOnline = false;
    this.pendingRobotMaps = [];
  }

  async init() {
    this.bindEvents();
    await this.refreshRobots();
    window.setInterval(() => {
      this.refreshRobots({ quiet: true }).catch(() => {});
    }, 5000);
  }

  bindEvents() {
    this.openSidebarButton.addEventListener("click", () => this.openSidebar());
    this.closeSidebarButton.addEventListener("click", () => this.closeSidebar());
    this.sidebarBackdrop.addEventListener("click", () => this.closeSidebar());
    this.refreshButton.addEventListener("click", () => this.refreshRobots());
    this.addRobotButton.addEventListener("click", () => this.openAddRobotDialog());
    this.editMapButton.addEventListener("click", () => this.openMapEditor());
    this.controlPullMapButton.addEventListener("click", () => this.handlePullMap());
    this.controlPushMapButton.addEventListener("click", () => this.handlePushMap());
    this.controlLoadMapButton.addEventListener("click", () => this.handleLoadMap());
    this.closeDialogButton.addEventListener("click", () => this.addRobotDialog.close());
    this.probeRobotButton.addEventListener("click", () => this.handleProbe());
    this.saveRobotButton.addEventListener("click", async (event) => {
      event.preventDefault();
      await this.handleSaveRobot();
    });
    this.closeLoadMapDialogButton.addEventListener("click", () => this.loadMapDialog.close());
    this.cancelLoadMapButton.addEventListener("click", () => this.loadMapDialog.close());
    this.confirmLoadMapButton.addEventListener("click", () => this.confirmLoadMap());
  }

  selectedRobot() {
    return this.robots.find((robot) => robot.id === this.selectedRobotId) || null;
  }

  async refreshRobots(options = {}) {
    const result = await this.getJson("/api/robots");
    this.robots = Array.isArray(result.robots) ? result.robots : [];
    if (this.selectedRobotId && !this.selectedRobot()) {
      this.selectedRobotId = "";
      window.localStorage.removeItem("operator:selectedRobotId");
    }
    if (!this.selectedRobotId && this.robots.length) {
      this.selectedRobotId = this.robots[0].id;
      window.localStorage.setItem("operator:selectedRobotId", this.selectedRobotId);
    }
    await this.refreshRobotMapState({ quiet: true });
    this.render();
    await this.maybePromptPendingPush();
    if (!options.quiet) {
      this.showProbeResult("neutral", "Robot list refreshed.");
    }
  }

  async refreshRobotMapState(options = {}) {
    const robot = this.selectedRobot();
    if (!robot) {
      this.robotMapState = {
        robotActiveMapName: "",
        operatorActiveMapName: "",
        robotSignature: "",
        operatorSignature: "",
        sourceRobotMapName: "",
        hasLocalChanges: false,
      };
      return;
    }
    try {
      const [robotActive, localActive] = await Promise.all([
        this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/active`),
        this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/local/active`),
      ]);
      this.robotMapState = {
        robotActiveMapName: String(robotActive.mapName || "").trim(),
        operatorActiveMapName: String(localActive.activeMapName || "").trim(),
        robotSignature: String(robotActive.signature || "").trim(),
        operatorSignature: String(localActive.signature || "").trim(),
        sourceRobotMapName: String(localActive.robotMapName || localActive.sourceMapName || "").trim(),
        hasLocalChanges: Boolean(
          localActive.activeMapName &&
          (
            Boolean(localActive.hasLocalChanges) ||
            (String(localActive.signature || "").trim() &&
             String(robotActive.signature || "").trim() &&
             String(localActive.signature || "").trim() !== String(robotActive.signature || "").trim()) ||
            (String(localActive.activeMapName || "").trim() &&
             String(robotActive.mapName || "").trim() &&
             String(localActive.activeMapName || "").trim() !== String(robotActive.mapName || "").trim())
          )
        ),
      };
    } catch (error) {
      this.robotMapState = {
        robotActiveMapName: "",
        operatorActiveMapName: "",
        robotSignature: "",
        operatorSignature: "",
        sourceRobotMapName: "",
        hasLocalChanges: false,
      };
      if (!options.quiet) {
        window.alert(error.message || String(error));
      }
    }
  }

  render() {
    this.robotCountText.textContent = `${this.robots.length} saved`;
    this.renderRobotList();
    this.renderSelectedRobot();
  }

  renderRobotList() {
    this.robotsList.innerHTML = "";
    if (!this.robots.length) {
      const empty = document.createElement("div");
      empty.className = "probe-result neutral";
      empty.textContent = "No robots added yet. Use Add Robot + to connect by IP.";
      this.robotsList.append(empty);
      return;
    }

    for (const robot of this.robots) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "robot-card";
      if (robot.id === this.selectedRobotId) {
        button.classList.add("active");
      }
      button.addEventListener("click", () => {
        this.selectedRobotId = robot.id;
        window.localStorage.setItem("operator:selectedRobotId", robot.id);
        this.closeSidebar();
        this.refreshRobotMapState({ quiet: true }).then(() => this.render()).catch(() => this.render());
      });

      const identity = robot.identity || robot.lastIdentity || {};
      const status = robot.status || {};
      const chipClass = robot.online ? "robot-chip online" : "robot-chip offline";
      const chipText = robot.online ? "online" : "offline";
      button.innerHTML = `
        <div class="robot-card-header">
          <div>
            <strong>${this.escapeHtml(robot.name || identity.robotId || robot.id)}</strong>
            <p>${this.escapeHtml(robot.host)}:${this.escapeHtml(String(robot.port))}</p>
          </div>
          <div class="robot-card-actions">
            <span class="${chipClass}">${chipText}</span>
            <button class="robot-card-remove" type="button" aria-label="Remove robot">Delete</button>
          </div>
        </div>
        <div class="robot-card-meta">
          <div>Robot ID: ${this.escapeHtml(identity.robotId || "-")}</div>
          <div>Map: ${this.escapeHtml(identity.mapId || "-")}</div>
          <div>State: ${this.escapeHtml(status.state || status.stateText || "-")}</div>
        </div>
      `;
      const removeButton = button.querySelector(".robot-card-remove");
      if (removeButton) {
        removeButton.addEventListener("click", async (event) => {
          event.stopPropagation();
          await this.handleRemoveRobot(robot);
        });
      }
      this.robotsList.append(button);
    }
  }

  renderSelectedRobot() {
    const robot = this.selectedRobot();
    this.renderSidebar();
    if (!robot) {
      this.emptyState.classList.remove("hidden");
      this.robotView.classList.add("hidden");
      this.robotFrame.removeAttribute("src");
      this.frameUrl = "";
      this.frameOnline = false;
      this.robotActiveMapText.textContent = "-";
      this.operatorActiveMapText.textContent = "-";
      this.mapSyncStatus.className = "probe-result neutral";
      this.mapSyncStatus.textContent = "Select a robot to see map sync state.";
      return;
    }

    this.emptyState.classList.add("hidden");
    this.robotView.classList.remove("hidden");
    this.robotActiveMapText.textContent = this.robotMapState.robotActiveMapName || "-";
    this.operatorActiveMapText.textContent = this.robotMapState.operatorActiveMapName || "-";
    this.renderMapSyncStatus();

    if (this.frameUrl !== robot.baseUrl || (robot.online && !this.frameOnline)) {
      this.setRobotFrameUrl(robot.baseUrl);
    }
    this.frameOnline = Boolean(robot.online);
  }

  renderMapSyncStatus() {
    const hasLocal = Boolean(this.robotMapState.operatorActiveMapName);
    const hasChanges = Boolean(this.robotMapState.hasLocalChanges);
    if (!hasLocal) {
      this.mapSyncStatus.className = "probe-result neutral";
      this.mapSyncStatus.textContent = "Operator has no local active map yet. Use Pull Map first.";
      this.controlPushMapButton.classList.remove("primary");
      return;
    }
    if (hasChanges) {
      const source = this.robotMapState.sourceRobotMapName || this.robotMapState.robotActiveMapName || "-";
      this.mapSyncStatus.className = "probe-result warning";
      this.mapSyncStatus.textContent = `Local map differs from robot map ${source}. Use Push Map to apply local changes to the robot.`;
      this.controlPushMapButton.classList.add("primary");
      return;
    }
    this.mapSyncStatus.className = "probe-result success";
    this.mapSyncStatus.textContent = "Operator local map matches the current robot map.";
    this.controlPushMapButton.classList.remove("primary");
  }

  setRobotFrameUrl(baseUrl, { forceReload = false } = {}) {
    const url = forceReload ? `${baseUrl}${baseUrl.includes("?") ? "&" : "?"}_ts=${Date.now()}` : baseUrl;
    this.robotFrame.src = url;
    this.frameUrl = baseUrl;
  }

  reloadRobotFrame() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    this.setRobotFrameUrl(robot.baseUrl, { forceReload: true });
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

  openAddRobotDialog() {
    this.lastProbe = null;
    this.robotNameInput.value = "";
    this.robotHostInput.value = "";
    this.robotPortInput.value = "8790";
    this.showProbeResult("neutral", "Enter 127.0.0.1 for the same PC or the robot LAN IP, then check the connection.");
    this.addRobotDialog.showModal();
  }

  async handleProbe() {
    const payload = this.dialogPayload();
    this.showProbeResult("neutral", `Checking ${payload.host}:${payload.port} ...`);
    try {
      const result = await this.postJson("/api/robots/probe", payload);
      this.lastProbe = result.probe;
      const identity = result.probe.identity || {};
      const status = result.probe.status || {};
      this.showProbeResult(
        "success",
        `Found ${identity.robotId || "robot"} on map ${identity.mapId || "-"}. Current state: ${status.state || "-"}`
      );
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
      this.selectedRobotId = result.robot.id;
      window.localStorage.setItem("operator:selectedRobotId", this.selectedRobotId);
      this.closeSidebar();
      await this.refreshRobots({ quiet: true });
      await this.refreshRobotMapState({ quiet: true });
      this.render();
      if (result.pulled && result.pulled.local && result.pulled.local.mapName) {
        this.showProbeResult("success", `Robot saved and active map ${result.pulled.local.mapName} pulled into operator cache.`);
      }
    } catch (error) {
      this.showProbeResult("error", error.message || String(error));
    }
  }

  async handleRemoveRobot(robot) {
    if (!robot) {
      return;
    }
    const confirmed = window.confirm(`Remove ${robot.name || robot.id} from the operator app?`);
    if (!confirmed) {
      return;
    }
    try {
      await this.deleteJson(`/api/robots/${encodeURIComponent(robot.id)}`);
      if (this.selectedRobotId === robot.id) {
        this.selectedRobotId = "";
        window.localStorage.removeItem("operator:selectedRobotId");
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
    const robotName = robot.name || robot.identity?.robotId || robot.id;
    const url = `/map-editor.html?robot_id=${encodeURIComponent(robot.id)}&robot_name=${encodeURIComponent(robotName)}`;
    window.location.assign(url);
  }

  async handlePullMap() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    try {
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/pull-sync`, {});
      await this.refreshRobotMapState({ quiet: true });
      this.renderSelectedRobot();
      window.alert(result.message || "Pull map completed.");
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async handlePushMap() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    try {
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/push-sync`, {});
      await this.refreshRobotMapState({ quiet: true });
      await this.refreshRobots({ quiet: true });
      this.reloadRobotFrame();
      window.alert(result.message || "Push map completed.");
    } catch (error) {
      window.alert(error.message || String(error));
    }
  }

  async maybePromptPendingPush() {
    const pendingRobotId = window.sessionStorage.getItem("operator:pendingPushRobotId") || "";
    const robot = this.selectedRobot();
    if (!robot || !pendingRobotId || pendingRobotId !== robot.id) {
      return;
    }
    window.sessionStorage.removeItem("operator:pendingPushRobotId");
    if (!this.robotMapState.hasLocalChanges) {
      return;
    }
    const shouldPush = window.confirm("Local map differs from the robot map. Push local changes to the robot now?");
    if (!shouldPush) {
      return;
    }
    await this.handlePushMap();
  }

  async handleLoadMap() {
    const robot = this.selectedRobot();
    if (!robot) {
      return;
    }
    try {
      const robotMaps = await this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/list`);
      const maps = Array.isArray(robotMaps.maps) ? robotMaps.maps : [];
      if (!maps.length) {
        window.alert("Robot has no editable maps.");
        return;
      }
      this.pendingRobotMaps = maps;
      this.loadMapSelect.innerHTML = "";
      for (const item of maps) {
        const option = document.createElement("option");
        option.value = item.name || item.folder || "";
        option.textContent = item.active ? `${item.name} (active)` : `${item.name}`;
        option.selected = Boolean(item.active) || option.value === this.robotMapState.robotActiveMapName;
        this.loadMapSelect.appendChild(option);
      }
      this.loadMapHint.className = "probe-result neutral";
      this.loadMapHint.textContent = "Choose one of the maps available on the robot.";
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
    try {
      const result = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/load`, { mapName });
      this.loadMapDialog.close();
      await this.refreshRobotMapState({ quiet: true });
      await this.refreshRobots({ quiet: true });
      this.reloadRobotFrame();
      window.alert(`Robot active map changed to ${result.mapName || mapName}.`);
    } catch (error) {
      this.loadMapHint.className = "probe-result error";
      this.loadMapHint.textContent = error.message || String(error);
    }
  }

  dialogPayload() {
    return {
      name: this.robotNameInput.value.trim(),
      host: this.robotHostInput.value.trim(),
      port: Number(this.robotPortInput.value || 8790),
    };
  }

  showProbeResult(kind, text) {
    this.probeResult.className = `probe-result ${kind}`;
    this.probeResult.textContent = text;
  }

  async getJson(url) {
    const response = await fetch(url);
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
