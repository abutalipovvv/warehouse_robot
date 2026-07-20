import { FLEET_ROBOT_PALETTE } from "./constants.js";


export const withFleetUi = (Base) => class OperatorAppFleetUi extends Base {
  async refreshRobots(options = {}) {
    const shouldProbe = options.probe ?? !options.quiet;
    const result = await this.getJson(shouldProbe ? "/api/robots" : "/api/robots?probe=0");
    const nextRobots = Array.isArray(result.robots) ? result.robots : [];
    this.robots = options.quiet ? this.mergeQuietRobotPayloads(nextRobots) : nextRobots;
    if (this.selectedRobotId && !this.selectedRobot()) {
      this.setSelectedRobotId("");
      this.closeScanStream();
      this.closeSlamStream();
      this.closeTeleopSocket(true);
    }
    if (!this.selectedRobotId && this.robots.length && !this.isGlobalHomePage()) {
      this.setSelectedRobotId(this.robots[0].id);
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
    const context = this.selectionContext(robot);
    if (this.isFleetManager(robot)) {
      try {
        const base = this.fleetApiBase(robot);
        const [robotActive, localActive] = await Promise.all([
          this.getJson(`${base}/maps/active`),
          this.getJson(`${base}/maps/local/active`),
        ]);
        if (!this.selectionIsCurrent(context)) {
          return;
        }
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
        if (!this.selectionIsCurrent(context)) {
          return;
        }
        this.robotMapState = this.emptyMapState();
        this.operatorMapPayload = null;
        this.operatorMapSignature = "";
        if (!options.quiet) {
          window.alert(error.message || String(error));
        }
      } finally {
        this.finishMapContextLoad(context);
      }
      return;
    }
    try {
      const [robotActiveResult, localActiveResult] = await Promise.allSettled([
        this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/active`),
        this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/local/active`),
      ]);
      if (localActiveResult.status !== "fulfilled") {
        throw localActiveResult.reason;
      }
      const localActive = localActiveResult.value;
      const robotActive = robotActiveResult.status === "fulfilled"
        ? robotActiveResult.value
        : {};
      if (!this.selectionIsCurrent(context)) {
        return;
      }
      const nextSignature = String(localActive.signature || "").trim();
      if (nextSignature && nextSignature !== this.operatorMapSignature) {
        this.resetMapView(true);
      }
      this.operatorMapPayload = localActive.map && typeof localActive.map === "object" ? localActive.map : null;
      this.operatorMapSignature = nextSignature;
      this.robotMapState = {
        robotActiveMapName: String(robotActive.mapName || localActive.robotMapName || "").trim(),
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
      if (!this.selectionIsCurrent(context)) {
        return;
      }
      this.robotMapState = this.emptyMapState();
      this.operatorMapPayload = null;
      this.operatorMapSignature = "";
      if (!options.quiet) {
        window.alert(error.message || String(error));
      }
    } finally {
      this.finishMapContextLoad(context);
    }
  }

  finishMapContextLoad(context) {
    if (
      this.selectionIsCurrent(context)
      && this.mapContentLoadingRobotId === context.robotId
    ) {
      this.mapContentLoadingRobotId = "";
    }
  }

  applyLoadedMapResult(result, requestedMapName = "", robot = this.selectedRobot()) {
    if (!robot || robot.id !== this.selectedRobotId) {
      return;
    }
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
    if (!robot || this.statusRequestPending === robot.id) {
      return;
    }
    const context = this.selectionContext(robot);
    this.statusRequestPending = robot.id;
    try {
      if (this.isFleetManager(robot)) {
        await this.ensureFleetParamsLoaded(false, robot);
        if (!this.selectionIsCurrent(context)) {
          return;
        }
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
        ? await this.getJson(this.fleetApiPath("/state", robot))
        : await this.getJson(`/robots/${encodeURIComponent(robot.id)}/api/robot/status`);
      if (!this.selectionIsCurrent(context)) {
        return;
      }
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
      if (!this.selectionIsCurrent(context)) {
        return;
      }
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
      if (this.statusRequestPending === robot.id) {
        this.statusRequestPending = false;
      }
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
    if (now - this.fleetHttpFallbackLastAt < 80) {
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
      const incomingUpdatedAt = Number(robot.updatedAt);
      const priorUpdatedAt = Number(prior.updatedAt);
      if (
        Number.isFinite(incomingUpdatedAt)
        && Number.isFinite(priorUpdatedAt)
        && incomingUpdatedAt + 0.000001 < priorUpdatedAt
      ) {
        return prior;
      }
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

  mergeFleetRobotUpdate(robotUpdate) {
    if (!robotUpdate || typeof robotUpdate !== "object" || !robotUpdate.name) {
      return this.currentStatus || {};
    }
    const previous = this.currentStatus || {};
    const robots = Array.isArray(previous.robots) ? previous.robots : [];
    let replaced = false;
    const nextRobots = robots.map((robot) => {
      if (robot.name !== robotUpdate.name) {
        return robot;
      }
      replaced = true;
      return { ...robot, ...robotUpdate };
    });
    if (!replaced) {
      nextRobots.push(robotUpdate);
    }
    return { ...previous, robots: nextRobots };
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
      const selected = card.dataset.robotId === this.selectedRobotId;
      card.classList.toggle("active", selected);
      card.setAttribute("aria-selected", String(selected));
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
      button.title = options.openWorkspace
        ? "Single click selects. Double click opens the workspace."
        : "Select robot";
      if (robot.id === this.selectedRobotId) {
        button.classList.add("active");
      }
      button.setAttribute("aria-selected", String(robot.id === this.selectedRobotId));
      if (robot.id === this.workspaceLoadingRobotId) {
        button.classList.add("loading");
        button.setAttribute("aria-busy", "true");
      }
      const selectRobot = async ({ enterWorkspace = false } = {}) => {
        const workspaceLoadingStartedAt = enterWorkspace ? performance.now() : 0;
        if (enterWorkspace) {
          const now = performance.now();
          if (
            this.workspaceTransitionRobotId === robot.id
            && now < this.workspaceTransitionUntil
          ) {
            return;
          }
          this.workspaceTransitionRobotId = robot.id;
          this.workspaceTransitionUntil = now + 750;
        }
        const selectionChanged = this.selectedRobotId !== robot.id;
        if (selectionChanged) {
          this.closeScanStream();
          this.closeSlamStream();
          this.closeTeleopSocket(true);
          this.manualKeys.clear();
          this.syncManualButtons();
        }
        this.setSelectedRobotId(robot.id);
        const context = this.selectionContext(robot);
        this.currentStatus = null;
        this.currentRoute = null;
        this.syncFleetStatusStream();
        this.closeSidebar();
        if (options.home && !enterWorkspace) {
          this.syncRobotCardSelection();
          return;
        }
        if (enterWorkspace) {
          this.workspaceLoadingRobotId = robot.id;
        }
        this.renderSelectedRobot();
        if (enterWorkspace && options.openWorkspace) {
          try {
            await this.navigateHomePage();
          } finally {
            const loadingRemainingMs = Math.max(
              0,
              this.workspaceLoadingMinimumMs - (performance.now() - workspaceLoadingStartedAt),
            );
            if (loadingRemainingMs > 0) {
              await new Promise((resolve) => window.setTimeout(resolve, loadingRemainingMs));
            }
            if (this.selectionIsCurrent(context)) {
              this.workspaceLoadingRobotId = "";
              this.syncFleetStatusStream();
              this.render();
            }
          }
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
      button.addEventListener("click", () => {
        selectRobot({ enterWorkspace: false }).catch((error) => {
          this.showProbeResult("error", error.message || String(error));
        });
      });
      if (options.openWorkspace) {
        button.addEventListener("dblclick", (event) => {
          event.preventDefault();
          selectRobot({ enterWorkspace: true }).catch((error) => {
            this.showProbeResult("error", error.message || String(error));
          });
        });
      }
      button.addEventListener("keydown", async (event) => {
        if (event.key !== "Enter" && event.key !== " ") {
          return;
        }
        event.preventDefault();
        await selectRobot({
          enterWorkspace: event.key === "Enter" && Boolean(options.openWorkspace),
        });
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
    const loadingWorkspace = Boolean(this.workspaceLoadingRobotId);
    if (this.appBooting || loadingWorkspace) {
      this.homePage?.classList.add("hidden");
      this.emptyState.classList.add("hidden");
      this.robotView.classList.add("hidden");
      this.workspaceLoadingState?.classList.remove("hidden");
      if (this.workspaceLoadingTitle) {
        this.workspaceLoadingTitle.textContent = loadingWorkspace
          ? `Loading ${this.robotDisplayName(robot)}`
          : "Loading Operator App";
      }
      if (this.workspaceLoadingText) {
        this.workspaceLoadingText.textContent = loadingWorkspace
          ? "Opening the correct map workspace and starting the live status stream..."
          : "Reading systems, robots, and active map contexts...";
      }
      return;
    }
    this.workspaceLoadingState?.classList.add("hidden");
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
    const mapLoading = this.mapContentLoadingRobotId === robot.id;
    this.operatorMapLoading?.classList.toggle("hidden", !mapLoading);
    if (mapLoading && this.operatorMapLoadingTitle && this.operatorMapLoadingText) {
      this.operatorMapLoadingTitle.textContent = `Loading ${this.robotDisplayName(robot)} map`;
      this.operatorMapLoadingText.textContent = "The live status stream is already starting; PGM and graph are loading in the background.";
    }
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
    const remoteLabel = isFleet ? "Fleet Manager" : "Robot";
    const remoteMap = this.robotMapState.robotActiveMapName || "-";
    const localMap = this.robotMapState.operatorActiveMapName || "-";
    if (!hasLocal) {
      this.setMapSyncStatus("neutral", {
        state: "Local map missing",
        local: localMap,
        remoteLabel,
        remote: remoteMap,
        detail: "Use Pull to create this workspace's local operator copy.",
      });
      this.controlPushMapButton.classList.remove("primary");
      return;
    }
    if (hasChanges) {
      const source = this.robotMapState.sourceRobotMapName || this.robotMapState.robotActiveMapName || "-";
      this.setMapSyncStatus("warning", {
        state: "Local changes",
        local: localMap,
        remoteLabel,
        remote: source,
        detail: "Use Push to apply only this workspace's local changes.",
      });
      this.controlPushMapButton.classList.add("primary");
      return;
    }
    this.setMapSyncStatus("success", {
      state: "Maps synchronized",
      local: localMap,
      remoteLabel,
      remote: remoteMap,
      detail: "Local and remote signatures match.",
    });
    this.controlPushMapButton.classList.remove("primary");
  }

  setMapSyncStatus(kind, details) {
    if (!this.mapSyncStatus) {
      return;
    }
    this.mapSyncStatus.className = `probe-result ${kind} map-sync-status`;
    this.mapSyncStatus.replaceChildren();
    const fields = [
      ["State", details.state],
      ["Operator", details.local],
      [details.remoteLabel || "Remote", details.remote],
    ];
    for (const [label, value] of fields) {
      const item = document.createElement("span");
      item.className = "map-sync-field";
      const key = document.createElement("small");
      key.textContent = label;
      const text = document.createElement("strong");
      text.textContent = value || "-";
      item.append(key, text);
      this.mapSyncStatus.append(item);
    }
    const detail = document.createElement("span");
    detail.className = "map-sync-detail";
    detail.textContent = details.detail || "";
    this.mapSyncStatus.append(detail);
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
    if (!this.babylonMapFailed && !this.slamActive) {
      if (!this.updateOperatorScene3dRobots(this.scene3d, { motionOnly: true })) {
        this.renderOperatorBabylonMap({ motionOnly: true });
      }
    } else {
      this.drawRoute();
      this.drawScanOverlay();
      this.drawRobot();
    }
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

    this.robotStateText.textContent = selectedFleetRobot
      ? this.fleetRobotStateLabel(selectedFleetRobot)
      : mode.toUpperCase();
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

    this.robotStateText.textContent = selectedFleetRobot
      ? this.fleetRobotStateLabel(selectedFleetRobot)
      : mode.toUpperCase();
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
    if (!this.babylonMapFailed && !this.slamActive) {
      if (!this.updateOperatorScene3dRobots(this.scene3d)) {
        this.renderOperatorBabylonMap();
      }
    } else {
      this.drawRoute();
      this.drawLookahead();
      this.drawLandmarks();
      this.drawRobot();
    }
    this.syncMapControls();
    this.syncModeButtons();
    this.syncManualButtons();
    this.syncDynamicBenchmarkControls();
    if (
      this.isFleetManagerSim()
      && status.dynamicBenchmark?.scenario
      && (status.dynamicBenchmark?.active || Number(status.dynamicBenchmark?.ordersGenerated || 0) > 0)
      && this.fleetBenchmarkStatus
    ) {
      this.renderFleetBenchmarkSummary(
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
    const packageMode = String(dynamic.generationMode || "continuous") === "package_waves";
    const continuousActive = Boolean(dynamic.active) && !packageMode;
    const packageActive = Boolean(dynamic.active) && packageMode;
    this.fleetBenchmarkPlanButton.textContent = continuousActive
      ? "Stop Dynamic Orders"
      : "Start Dynamic Orders";
    this.fleetBenchmarkPlanButton.disabled = Boolean(this.fleetBenchmarkBusy || packageActive);
    if (this.fleetBenchmarkPackageButton) {
      this.fleetBenchmarkPackageButton.textContent = packageActive
        ? "Stop Package Orders"
        : "Generate Package Orders";
      this.fleetBenchmarkPackageButton.disabled = Boolean(this.fleetBenchmarkBusy || continuousActive);
    }
    if (this.fleetSimulationTimeScaleSelect) {
      const maximum = Math.max(
        1,
        Number(this.currentStatus?.simulationTimeScaleMax || 4),
      );
      for (const option of this.fleetSimulationTimeScaleSelect.options) {
        option.disabled = Number(option.value || 1) > maximum;
      }
      const scale = Math.min(
        maximum,
        Math.max(
          1,
          Number(this.currentStatus?.simulationTimeScale || dynamic.timeScale || 1),
        ),
      );
      this.fleetSimulationTimeScaleSelect.value = String(scale);
    }
  }

  selectedFleetRobot(robots = null) {
    const items = robots || (Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : []);
    if (!items.length || this.fleetSelectionCleared) {
      return null;
    }
    const selected = items.find((robot) => robot.name === this.selectedFleetRobotName);
    return selected || items[0];
  }

  fleetRobotStateLabel(robot) {
    const motion = String(robot?.status || "IDLE").trim().toUpperCase();
    const order = String(robot?.assignedOrderStatus || "").trim().toUpperCase();
    return order && order !== motion
      ? `${motion} · ${order}`
      : (motion || order || "IDLE");
  }

  fleetRobotWaitBlockerName(robot) {
    if (!robot || typeof robot !== "object") {
      return "";
    }
    const dependency = robot.waitDependency;
    if (dependency && typeof dependency === "object" && dependency.robot) {
      return String(dependency.robot).trim();
    }
    const reason = String(robot.reason || "").trim();
    for (const prefix of ["yield to ", "occupied by ", "keep clearance from "]) {
      if (reason.startsWith(prefix)) {
        return reason.slice(prefix.length).trim();
      }
    }
    return "";
  }

  fleetRobotWaitLabel(robot) {
    if (!robot || String(robot.status || "") !== "WAITING") {
      return "";
    }
    const blocker = this.fleetRobotWaitBlockerName(robot);
    if (blocker) {
      return `waiting for ${blocker}`;
    }
    const reason = String(robot.reason || robot.routeNote || "").trim();
    if (reason.startsWith("traffic admission wait at ")) {
      return "waiting for traffic zone";
    }
    if (reason.startsWith("planned traffic wait")) {
      return "planned traffic wait";
    }
    if (reason === "rolling continuation pending") {
      return "planning next route segment";
    }
    if (/obstacle|blocked edge/i.test(reason)) {
      return "blocked by obstacle";
    }
    return reason ? `waiting: ${reason.slice(0, 34)}` : "waiting for route clearance";
  }

  fleetRobotAlertLabel(robot) {
    if (!robot || typeof robot !== "object") {
      return "";
    }
    const status = String(robot.status || "").trim().toUpperCase();
    const remoteError = String(robot.remoteError || "").trim();
    const reason = String(remoteError || robot.reason || robot.routeNote || "").trim();
    const blocker = this.fleetRobotWaitBlockerName(robot);
    if (remoteError) {
      return `error: ${remoteError.slice(0, 34)}`;
    }
    if (["OFFLINE", "ERROR"].includes(status)) {
      return reason ? `error: ${reason.slice(0, 34)}` : status.toLowerCase();
    }
    if (status === "RETREATING") {
      return "deadlock: retreating";
    }
    if (status === "BLOCKED" || status === "MANUAL_BLOCKED") {
      return /deadlock/i.test(reason)
        ? "deadlock: route blocked"
        : (reason ? `blocked: ${reason.slice(0, 32)}` : "route blocked");
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
    if (blocker) {
      return `waiting for ${blocker}`;
    }
    if (reason.startsWith("planned traffic wait")) {
      return "planned traffic wait";
    }
    if (reason.startsWith("traffic admission wait")) {
      return "waiting for traffic zone";
    }
    if (/obstacle|blocked edge/i.test(reason)) {
      return "blocked by obstacle";
    }
    return reason ? `waiting: ${reason.slice(0, 34)}` : "waiting for clearance";
  }

  fleetRobotAlertSeverity(robot) {
    const status = String(robot?.status || "").trim().toUpperCase();
    const reason = String(robot?.remoteError || robot?.reason || "");
    return Boolean(String(robot?.remoteError || "").trim())
      || ["OFFLINE", "ERROR", "BLOCKED", "MANUAL_BLOCKED", "RETREATING"].includes(status)
      || /deadlock/i.test(reason)
      ? "error"
      : "warning";
  }

  fleetRobotColor(robotName) {
    const name = String(robotName || "robot");
    let hash = 2166136261;
    for (let index = 0; index < name.length; index += 1) {
      hash ^= name.charCodeAt(index);
      hash = Math.imul(hash, 16777619);
    }
    return FLEET_ROBOT_PALETTE[(hash >>> 0) % FLEET_ROBOT_PALETTE.length];
  }

  clearFleetRobotSelection() {
    if (!this.isFleetManager()) {
      return;
    }
    this.selectedFleetRobotName = "";
    this.fleetSelectionCleared = true;
    this.mapView.follow = false;
    window.localStorage.removeItem("operator:selectedFleetRobotName");
    this.renderSelectedRobot();
  }

  selectFleetRobotByName(robotName) {
    const name = String(robotName || "").trim();
    if (!name) {
      return;
    }
    this.selectedFleetRobotName = name;
    this.fleetSelectionCleared = false;
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

  renderFleetRobotList(robots) {
    if (!this.fleetRobotList) {
      return;
    }
    if (!robots.length) {
      let empty = this.fleetRobotList.querySelector(":scope > .fleet-list-empty");
      if (!empty || this.fleetRobotList.children.length !== 1) {
        this.fleetRobotList.replaceChildren();
        empty = document.createElement("div");
        empty.className = "probe-result neutral compact fleet-list-empty";
        this.fleetRobotList.append(empty);
      }
      empty.textContent = this.fleetRuntimeMode() === "robots"
        ? "No robots yet. Add a robot IP; LM is read from robot status."
        : "No robots yet. Add a simulation robot from a start LM.";
      return;
    }

    this.fleetRobotList.querySelector(":scope > .fleet-list-empty")?.remove();
    const existing = new Map(
      Array.from(this.fleetRobotList.querySelectorAll(":scope > .fleet-list-item[data-robot-name]"))
        .map((row) => [row.dataset.robotName, row]),
    );
    const queuedGoals = this.fleetQueuedGoalsByRobot();
    const activeNames = new Set();
    robots.forEach((robot, index) => {
      const name = String(robot.name || "");
      activeNames.add(name);
      let row = existing.get(name);
      if (!row) {
        row = this.createFleetRobotListRow(name);
      }
      this.updateFleetRobotListRow(row, robot, queuedGoals.get(name) || "");
      const currentAtIndex = this.fleetRobotList.children[index] || null;
      if (currentAtIndex !== row) {
        this.fleetRobotList.insertBefore(row, currentAtIndex);
      }
    });
    for (const [name, row] of existing.entries()) {
      if (!activeNames.has(name)) {
        row.remove();
      }
    }
  }

  createFleetRobotListRow(robotName) {
    const row = document.createElement("div");
    row.className = "fleet-list-item";
    row.dataset.robotName = robotName;

    const button = document.createElement("button");
    button.type = "button";
    button.className = "fleet-list-main";
    button.addEventListener("click", (event) => {
      event.preventDefault();
      event.stopPropagation();
      this.selectFleetRobotByName(row.dataset.robotName);
    });

    const color = document.createElement("span");
    color.className = "fleet-list-color";
    color.dataset.role = "color";
    button.append(color);

    const info = document.createElement("span");
    info.className = "fleet-list-name";
    const title = document.createElement("strong");
    title.dataset.role = "name";
    const subtitle = document.createElement("span");
    subtitle.dataset.role = "meta";
    info.append(title, subtitle);
    button.append(info);

    const state = document.createElement("span");
    state.className = "fleet-list-state";
    state.dataset.role = "state";
    button.append(state);

    const removeButton = document.createElement("button");
    removeButton.type = "button";
    removeButton.className = "fleet-list-remove";
    removeButton.textContent = "-";
    removeButton.addEventListener("pointerdown", (event) => {
      event.stopPropagation();
    });
    removeButton.addEventListener("click", (event) => {
      event.stopPropagation();
      this.handleFleetRemoveRobot(row.dataset.robotName);
    });

    row.append(button, removeButton);
    row._fleetFields = {
      color,
      title,
      subtitle,
      state,
      removeButton,
    };
    return row;
  }

  updateFleetRobotListRow(row, robot, queuedGoal = "") {
    const name = String(robot.name || "");
    row.dataset.robotName = name;
    row.classList.toggle("active", name === this.selectedFleetRobotName);
    const fields = row._fleetFields || {};
    const color = fields.color || row.querySelector('[data-role="color"]');
    if (color) {
      const nextColor = this.fleetRobotColor(name);
      if (color.style.background !== nextColor) {
        color.style.background = nextColor;
      }
    }
    const title = fields.title || row.querySelector('[data-role="name"]');
    if (title) {
      const nextTitle = name || "-";
      if (title.textContent !== nextTitle) {
        title.textContent = nextTitle;
      }
    }
    const subtitle = fields.subtitle || row.querySelector('[data-role="meta"]');
    if (subtitle) {
      const robotMode = String(robot.mode || robot.type || "simulated");
      const remoteStatus = this.remoteStatusForFleetRobot(robot);
      const mapLabel = this.fleetRobotMapLabel(robot, remoteStatus);
      const meta = [
        `${robot.currentLm || "-"} -> ${robot.targetLm || "-"}`,
        robotMode !== "simulated" ? robotMode : "",
        robotMode !== "simulated" ? (robot.online === false ? "offline" : "online") : "",
        mapLabel && mapLabel !== "-" ? `map ${mapLabel}` : "",
      ].filter(Boolean);
      let nextSubtitle = meta.join(" | ");
      if (queuedGoal) {
        nextSubtitle = `${nextSubtitle} | queued ${queuedGoal}`;
      }
      if (subtitle.textContent !== nextSubtitle) {
        subtitle.textContent = nextSubtitle;
      }
    }
    const state = fields.state || row.querySelector('[data-role="state"]');
    if (state) {
      const nextState = this.fleetRobotStateLabel(robot);
      if (state.textContent !== nextState) {
        state.textContent = nextState;
      }
    }
    const removeButton = fields.removeButton || row.querySelector(".fleet-list-remove");
    if (removeButton) {
      const nextTitle = `Remove ${name}`;
      if (removeButton.title !== nextTitle) {
        removeButton.title = nextTitle;
      }
    }
  }

  fleetQueuedGoalsByRobot() {
    const result = new Map();
    for (const group of this.fleetDraftGroups()) {
      result.set(group.robotName, `${group.goals.length} draft`);
    }
    for (const item of this.fleetOrders()) {
      const status = String(item.status || "").toUpperCase();
      if (this.isOrderTerminal(status)) {
        continue;
      }
      const robotName = String(item.assignedRobot || item.vehicle || "");
      if (!robotName || result.has(robotName)) {
        continue;
      }
      const totalSteps = Number(
        item.totalSteps
        || (Array.isArray(item.targets) ? item.targets.length : 1)
        || 1,
      );
      const currentStep = Math.min(totalSteps, Number(item.currentStep || 0) + 1);
      result.set(
        robotName,
        `${currentStep}/${totalSteps} ${item.targetLm || "-"} ${status.toLowerCase()}`,
      );
    }
    return result;
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
    const draftGroups = this.fleetDraftGroups();
    const orders = this.fleetOrders();
    if (orders.length) {
      this.selectedFleetOrder();
    }
    const renderKey = JSON.stringify({
      manager: String(this.selectedRobot()?.id || ""),
      selected: String(this.selectedFleetOrderId || ""),
      drafts: draftGroups.map((group) => [
        group.robotName,
        group.goals.map((goal) => [
          Number(goal.seq || 0),
          String(goal.targetLm || goal.goalLm || ""),
        ]),
      ]),
      orders: orders.slice(0, 80).map((item) => [
        String(item.id || item.orderId || ""),
        String(item.status || ""),
        String(item.assignedRobot || item.vehicle || ""),
        String(item.targetLm || ""),
        Number(item.currentStep || 0),
        Number(item.totalSteps || 0),
        Array.isArray(item.targets) ? item.targets.map(String) : [],
        Array.isArray(item.routeNodes) ? item.routeNodes.map(String) : [],
        String(item.error || ""),
      ]),
    });
    if (renderKey === this.fleetQueueRenderKey) {
      return;
    }
    this.fleetQueueRenderKey = renderKey;
    this.fleetQueueList.innerHTML = "";
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
    const visibleEvents = events.slice().reverse().slice(0, 80);
    const managerId = String(this.selectedRobot()?.id || "");
    const renderKey = JSON.stringify([
      managerId,
      visibleEvents.map((event) => [
        event.stamp || 0,
        event.level || "info",
        event.message || "",
      ]),
    ]);
    if (renderKey === this.fleetEventsRenderKey) {
      return;
    }
    this.fleetEventsRenderKey = renderKey;
    this.robotEventsLog.innerHTML = "";
    if (!events.length) {
      this.robotEventsLog.textContent = "No events yet.";
      return;
    }
    for (const event of visibleEvents) {
      const row = document.createElement("div");
      row.className = `event-row ${String(event.level || "info").toLowerCase()}`;
      const stamp = event.stamp ? new Date(Number(event.stamp) * 1000).toLocaleTimeString([], { hour12: false }) : "--:--:--";
      row.textContent = `${stamp} ${event.level || "info"} ${event.message || ""}`;
      this.robotEventsLog.append(row);
    }
  }
};
