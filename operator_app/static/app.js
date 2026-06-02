class OperatorApp {
  constructor() {
    this.robots = [];
    this.selectedRobotId = window.localStorage.getItem("operator:selectedRobotId") || "";
    this.lastProbe = null;
    this.sidebarOpen = !this.selectedRobotId;

    this.robotsList = document.getElementById("robotsList");
    this.robotCountText = document.getElementById("robotCountText");
    this.sidebarDrawer = document.getElementById("sidebarDrawer");
    this.sidebarBackdrop = document.getElementById("sidebarBackdrop");
    this.openSidebarButton = document.getElementById("openSidebarButton");
    this.closeSidebarButton = document.getElementById("closeSidebarButton");
    this.emptyState = document.getElementById("emptyState");
    this.robotView = document.getElementById("robotView");
    this.robotFrame = document.getElementById("robotFrame");
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

    this.frameUrl = "";
    this.frameOnline = false;
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
    this.closeDialogButton.addEventListener("click", () => this.addRobotDialog.close());
    this.probeRobotButton.addEventListener("click", () => this.handleProbe());
    this.saveRobotButton.addEventListener("click", async (event) => {
      event.preventDefault();
      await this.handleSaveRobot();
    });
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
    this.render();
    if (!options.quiet) {
      this.showProbeResult("neutral", "Robot list refreshed.");
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
        this.render();
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
      return;
    }

    this.emptyState.classList.add("hidden");
    this.robotView.classList.remove("hidden");

    if (this.frameUrl !== robot.baseUrl || (robot.online && !this.frameOnline)) {
      this.robotFrame.src = robot.baseUrl;
      this.frameUrl = robot.baseUrl;
    }
    this.frameOnline = Boolean(robot.online);
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
    this.showProbeResult("neutral", "Enter the robot IP and check the connection.");
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
