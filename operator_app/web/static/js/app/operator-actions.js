import { httpClient } from "../api/http-client.js";
import { cloneJson, escapeHtml } from "../shared/json.js";
import { ROBOT_PARAM_SCHEMA } from "./constants.js";


export const withActions = (Base) => class OperatorAppActions extends Base {
  fleetMotionParams() {
    return {
      speed: this.fleetRouteSpeed(),
      acceleration: Math.max(0.0, Number(this.fleetRouteAccelerationInput?.value || 0.0) || 0.0),
      rotate: Boolean(this.fleetRotateInput?.checked) && !this.isFleetRobotsMode(),
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

  async ensureFleetParamsLoaded(force = false, robot = this.selectedRobot()) {
    if (!this.isFleetManager(robot)) {
      return;
    }
    if (this.fleetParamsLoaded && this.fleetParamsManagerId === robot.id && !force) {
      return;
    }
    const context = this.selectionContext(robot);
    const payload = await this.getJson(this.fleetApiPath("/params", robot));
    if (!this.selectionIsCurrent(context)) {
      return;
    }
    this.fleetParams = payload.params || {};
    this.fleetParamsLoaded = true;
    this.fleetParamsManagerId = robot.id;
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
      if (this.selectedRobotId !== robot.id) {
        return;
      }
      this.robotParams = payload.params || {};
      this.robotParamsRobotId = robot.id;
      this.robotParamsLoaded = true;
      this.applyRobotParams(this.robotParams);
    } catch (error) {
      if (this.selectedRobotId !== robot.id) {
        return;
      }
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
    return cloneJson(value || {});
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
      const robotControlledHeading = this.isFleetRobotsMode();
      this.fleetRotateInput.checked = !robotControlledHeading && Boolean(navigation.simulate_rotation);
      this.fleetRotateInput.disabled = robotControlledHeading;
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
    const robot = this.selectedRobot();
    if (!this.isFleetManager(robot)) {
      return;
    }
    try {
      const params = this.collectFleetParams();
      const result = await this.postJson(this.fleetApiPath("/params", robot), { params });
      if (this.selectedRobotId !== robot.id) {
        return;
      }
      this.fleetParams = result.params || params;
      this.fleetParamsLoaded = true;
      this.fleetParamsManagerId = robot.id;
      this.applyFleetParams(this.fleetParams);
      this.syncFleetParamsJson(true);
      this.robotMessageText.textContent = "Fleet params saved.";
    } catch (error) {
      this.robotMessageText.textContent = `Save params failed: ${error.message || error}`;
    }
  }

  async saveFleetJsonParams() {
    const robot = this.selectedRobot();
    if (!this.isFleetManager(robot)) {
      return;
    }
    try {
      const params = this.parseParamsJson(this.fleetParamsJsonInput, "Fleet params", this.fleetParams || {});
      const result = await this.postJson(this.fleetApiPath("/params", robot), { params });
      if (this.selectedRobotId !== robot.id) {
        return;
      }
      this.fleetParams = result.params || params;
      this.fleetParamsLoaded = true;
      this.fleetParamsManagerId = robot.id;
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
    this.fleetBenchmarkBusy = Boolean(busy);
    for (const button of this.fleetBenchmarkButtons || []) {
      button.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkClearButton) {
      this.fleetBenchmarkClearButton.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkPlanButton) {
      this.fleetBenchmarkPlanButton.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkPackageButton) {
      this.fleetBenchmarkPackageButton.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkHorizonInput) {
      this.fleetBenchmarkHorizonInput.disabled = Boolean(busy);
    }
    if (this.fleetBenchmarkIntervalInput) {
      this.fleetBenchmarkIntervalInput.disabled = Boolean(busy);
    }
    if (this.fleetSimulationTimeScaleSelect) {
      this.fleetSimulationTimeScaleSelect.disabled = Boolean(busy);
    }
    this.syncDynamicBenchmarkControls();
  }

  fleetBenchmarkMetricModel(result, robotCount) {
    const benchmark = result?.benchmark || result?.state?.benchmark || {};
    const scenario = String(benchmark.scenario || "");
    if (!["continuous_random_orders", "package_order_waves"].includes(scenario)) {
      return null;
    }
    const active = Boolean(benchmark.active);
    const packageMode = scenario === "package_order_waves";
    const generated = Number(benchmark.ordersGenerated || 0);
    const completed = Number(benchmark.ordersCompleted || 0);
    const queued = Number(benchmark.ordersQueued || 0);
    const executing = Number(benchmark.ordersExecuting || 0);
    const robotsWithOrders = Number(benchmark.robotsWithOrders || 0);
    const robotsWithoutOrders = Number(benchmark.robotsWithoutOrders || 0);
    const waitingRobots = Number(benchmark.waitingRobots || 0);
    const cycles = Number(benchmark.waitCyclesResolved || 0);
    const safetyRollbacks = Number(benchmark.runtimeSafetyRollbacks || 0);
    const averageDistance = Number(benchmark.averageOrderDistanceM || 0);
    const horizon = Number(benchmark.horizonSec || 0);
    const timeScale = Math.max(
      1,
      Number(benchmark.timeScale || this.currentStatus?.simulationTimeScale || 1),
    );
    const throughput = Number(benchmark.throughputOrdersPerMin || 0);
    const elapsedSimSec = Number(benchmark.elapsedSimSec || 0);
    const averageOrderSec = Number(benchmark.averageOrderDurationSec || 0);
    const waveIndex = Number(benchmark.waveIndex || 0);
    const wavesCompleted = Number(benchmark.wavesCompleted || 0);
    const metrics = [
      { label: "Robots", value: String(robotCount) },
      { label: "Time", value: `${timeScale}×` },
      { label: "Horizon", value: `${horizon.toFixed(1)} s` },
      { label: "Generated", value: String(generated) },
      { label: "Completed", value: String(completed), tone: "success" },
      { label: "Throughput", value: `${throughput.toFixed(2)}/min`, tone: "accent" },
      { label: "Elapsed", value: `${(elapsedSimSec / 60).toFixed(1)} sim min` },
      { label: "Avg order", value: `${averageOrderSec.toFixed(1)} s` },
      { label: "Avg goal", value: `${averageDistance.toFixed(1)} m` },
      {
        label: "Order coverage",
        value: `${robotsWithOrders}/${robotCount}`,
        tone: robotsWithoutOrders ? "warning" : "success",
      },
      { label: "Executing", value: String(executing) },
      { label: "Queued", value: String(queued) },
      { label: "Waiting", value: String(waitingRobots), tone: waitingRobots ? "warning" : "" },
      { label: "Deadlocks resolved", value: String(cycles), tone: cycles ? "success" : "" },
    ];
    if (packageMode) {
      metrics.splice(9, 0,
        { label: "Wave", value: String(waveIndex) },
        { label: "Waves done", value: String(wavesCompleted) },
      );
    }
    if (safetyRollbacks) {
      metrics.push({ label: "Safety rollbacks", value: String(safetyRollbacks), tone: "warning" });
    }
    return {
      title: packageMode ? "Package waves" : "Dynamic orders",
      state: active ? "Active" : "Stopped",
      active,
      metrics,
    };
  }

  renderFleetBenchmarkSummary(result, robotCount) {
    if (!this.fleetBenchmarkStatus) {
      return;
    }
    const model = this.fleetBenchmarkMetricModel(result, robotCount);
    if (!model) {
      this.fleetBenchmarkStatus.className = "probe-result success compact";
      this.fleetBenchmarkStatus.textContent = this.fleetBenchmarkSummary(result, robotCount);
      return;
    }
    this.fleetBenchmarkStatus.className = [
      "probe-result",
      model.active ? "success" : "neutral",
      "compact",
      "fleet-benchmark-dashboard",
    ].join(" ");
    this.fleetBenchmarkStatus.replaceChildren();
    const header = document.createElement("div");
    header.className = "fleet-benchmark-dashboard-head";
    const title = document.createElement("strong");
    title.textContent = model.title;
    const state = document.createElement("span");
    state.className = `fleet-benchmark-state ${model.active ? "active" : "stopped"}`;
    state.textContent = model.state;
    header.append(title, state);

    const grid = document.createElement("div");
    grid.className = "fleet-benchmark-metrics";
    for (const metric of model.metrics) {
      const item = document.createElement("div");
      item.className = [
        "fleet-benchmark-metric",
        metric.tone ? `tone-${metric.tone}` : "",
      ].filter(Boolean).join(" ");
      const label = document.createElement("span");
      label.textContent = metric.label;
      const value = document.createElement("strong");
      value.textContent = metric.value;
      item.append(label, value);
      grid.append(item);
    }
    this.fleetBenchmarkStatus.append(header, grid);
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
    if (["continuous_random_orders", "package_order_waves"].includes(scenario)) {
      const active = Boolean(benchmark.active);
      const packageMode = scenario === "package_order_waves";
      const generated = Number(benchmark.ordersGenerated || 0);
      const completed = Number(benchmark.ordersCompleted || 0);
      const queued = Number(benchmark.ordersQueued || 0);
      const executing = Number(benchmark.ordersExecuting || 0);
      const robotsWithOrders = Number(benchmark.robotsWithOrders || 0);
      const waitingRobots = Number(benchmark.waitingRobots || 0);
      const cycles = Number(benchmark.waitCyclesResolved || 0);
      const safetyRollbacks = Number(benchmark.runtimeSafetyRollbacks || 0);
      const averageDistance = Number(benchmark.averageOrderDistanceM || 0);
      const horizon = Number(benchmark.horizonSec || 0);
      const timeScale = Math.max(1, Number(benchmark.timeScale || this.currentStatus?.simulationTimeScale || 1));
      const throughput = Number(benchmark.throughputOrdersPerMin || 0);
      const elapsedSimSec = Number(benchmark.elapsedSimSec || 0);
      const averageOrderSec = Number(benchmark.averageOrderDurationSec || 0);
      const waveIndex = Number(benchmark.waveIndex || 0);
      const wavesCompleted = Number(benchmark.wavesCompleted || 0);
      return [
        active
          ? (packageMode ? "package waves active" : "dynamic orders active")
          : (packageMode ? "package waves stopped" : "dynamic orders stopped"),
        `${robotCount} robots`,
        `${timeScale}x time`,
        `horizon ${horizon.toFixed(1)} s`,
        `orders ${generated} generated / ${completed} completed`,
        `throughput ${throughput.toFixed(2)} orders/min`,
        elapsedSimSec ? `elapsed ${(elapsedSimSec / 60).toFixed(1)} sim min` : "",
        averageOrderSec ? `avg order ${averageOrderSec.toFixed(1)} s` : "",
        packageMode && waveIndex ? `wave ${waveIndex} / ${wavesCompleted} completed` : "",
        averageDistance ? `avg goal ${averageDistance.toFixed(1)} m` : "",
        `orders assigned ${robotsWithOrders}/${robotCount}`,
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
    this.fleetSelectionCleared = false;
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
        this.fleetSelectionCleared = false;
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
        this.renderFleetBenchmarkSummary(result, robotCount);
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

  async planFleetPackageOrders() {
    if (!this.isFleetManagerSim()) {
      return;
    }
    if (!Array.isArray(this.currentStatus?.robots) || !this.currentStatus.robots.length) {
      this.currentStatus = await this.getJson(this.fleetApiPath("/state")).catch(() => this.currentStatus);
    }
    const robots = Array.isArray(this.currentStatus?.robots) ? this.currentStatus.robots : [];
    const robotCount = robots.length;
    if (!robotCount) {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = "Add robots before generating package orders.";
      }
      return;
    }
    const dynamic = this.currentStatus?.dynamicBenchmark || {};
    const packageActive = Boolean(dynamic.active)
      && String(dynamic.generationMode || "") === "package_waves";
    if (dynamic.active && !packageActive) {
      return;
    }
    this.setFleetBenchmarkBusy(true);
    if (this.fleetBenchmarkStatus) {
      this.fleetBenchmarkStatus.className = "probe-result neutral compact";
      this.fleetBenchmarkStatus.textContent = packageActive
        ? "Stopping new package waves; active orders will finish..."
        : `Generating a ${robotCount}-order package wave to the map perimeter...`;
    }
    try {
      await new Promise((resolve) => window.requestAnimationFrame(resolve));
      const result = await this.postJsonRaw(this.fleetApiPath("/benchmark"), {
        action: packageActive ? "stop" : "package_waves",
        count: robotCount,
        reset: false,
        seed: 42,
        horizonSec: Math.max(1, Number(this.fleetBenchmarkHorizonInput?.value || 10)),
        queueDepth: 1,
        ...this.fleetMotionParams(),
        fast: true,
      });
      this.currentStatus = result.state || result.fleetState || result;
      if (!Array.isArray(this.currentStatus?.robots)) {
        this.currentStatus = await this.getJson(this.fleetApiPath("/state"));
      }
      this.lastFleetPlanDebug = {
        ...(result.debug || {}),
        benchmark: result.benchmark || this.currentStatus?.dynamicBenchmark || {},
      };
      if (this.fleetBenchmarkStatus) {
        this.renderFleetBenchmarkSummary(result, robotCount);
      }
      this.renderFleetStateImmediately();
      this.refreshRobots({ quiet: true, lightweight: true, probe: false }).catch(() => {});
    } catch (error) {
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = error.message || String(error);
      }
    } finally {
      this.setFleetBenchmarkBusy(false);
    }
  }

  async setFleetSimulationTimeScale() {
    if (!this.isFleetManagerSim() || !this.fleetSimulationTimeScaleSelect) {
      return;
    }
    const previous = Math.max(1, Number(this.currentStatus?.simulationTimeScale || 1));
    const requested = Math.max(1, Number(this.fleetSimulationTimeScaleSelect.value || 1));
    this.fleetSimulationTimeScaleSelect.disabled = true;
    try {
      const result = await this.postJson(this.fleetApiPath("/benchmark"), {
        action: "time_scale",
        timeScale: requested,
        reset: false,
      });
      const state = result.state || result.fleetState || result;
      this.currentStatus = Array.isArray(state?.robots)
        ? state
        : await this.getJson(this.fleetApiPath("/state"));
      const applied = Math.max(1, Number(this.currentStatus?.simulationTimeScale || requested));
      this.fleetVisualClocks.clear();
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result success compact";
        this.fleetBenchmarkStatus.textContent = `Simulation time: ${applied}x. CPU load increases with the multiplier.`;
      }
      this.renderFleetStateImmediately();
    } catch (error) {
      this.fleetSimulationTimeScaleSelect.value = String(previous);
      if (this.fleetBenchmarkStatus) {
        this.fleetBenchmarkStatus.className = "probe-result error compact";
        this.fleetBenchmarkStatus.textContent = error.message || String(error);
      }
    } finally {
      this.fleetSimulationTimeScaleSelect.disabled = false;
      this.syncDynamicBenchmarkControls();
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
      this.fleetSelectionCleared = false;
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
      const controlledRobot = this.isFleetManager()
        ? this.selectedFleetRobot()
        : (this.currentStatus?.robot || {});
      const requiredOwner = this.isFleetManager() ? "fleet-manager" : "operator-app";
      const requiresLease = !this.isFleetManagerSim();
      if (requiresLease && this.robotControlPayload(controlledRobot).ownerId !== requiredOwner) {
        this.robotMessageText.textContent = this.isFleetManager()
          ? "Use Seize Control before sending Fleet Manager commands."
          : "Use Seize Control before driving the robot.";
        return;
      }
      const wasIdle = this.manualKeys.size === 0;
      this.manualKeys.add(key);
      if (this.isFleetManager()) {
        this.navigateMode = false;
        this.pendingFleetAction = "";
        this.pendingFleetRobotName = "";
        this.syncModeButtons();
      }
      if (wasIdle && this.isFleetManagerSim()) {
        this.startFleetSimManualCommandLoop();
      }
    } else {
      this.manualKeys.delete(key);
      if (!this.manualKeys.size) {
        this.stopFleetSimManualCommandLoop();
        if (this.isFleetManager()) {
          this.releaseFleetManualControl().catch(() => {});
        } else {
          this.sendRobotTeleop({ linear: 0, angular: 0 }, 80);
        }
      }
    }
    if (this.isFleetManagerSim() && this.manualKeys.size) {
      const robot = this.selectedFleetRobot();
      const pose = robot
        ? (
          this.animatedFleetManualPose(robot)
          || robot.pose
          || this.poseForLm(robot.currentLm)
        )
        : null;
      if (robot && pose) {
        // Keyboard direction changes affect the visual controller immediately;
        // the HTTP loop remains an asynchronous safety/authority channel.
        this.setFleetManualAnimation(robot.name, pose, this.manualTwist());
      }
    }
    this.syncManualButtons();
  }

  startFleetSimManualCommandLoop() {
    if (
      !this.isFleetManagerSim()
      || this.fleetSimManualFrame
      || typeof window.requestAnimationFrame !== "function"
    ) {
      return;
    }
    this.fleetSimManualLastAt = 0;
    const publish = (now) => {
      this.fleetSimManualFrame = null;
      if (!this.isFleetManagerSim() || !this.manualKeys.size || !this.selectedRobot()) {
        this.fleetSimManualLastAt = 0;
        return;
      }
      // Fifteen milliseconds admits every frame on a 60 Hz display while
      // capping 120/144 Hz monitors close to the simulator's 60 Hz target.
      if (!this.fleetSimManualLastAt || now - this.fleetSimManualLastAt >= 15) {
        this.fleetSimManualLastAt = now;
        this.sendTeleopIfNeeded().catch(() => {});
      }
      this.fleetSimManualFrame = window.requestAnimationFrame(publish);
    };
    this.fleetSimManualFrame = window.requestAnimationFrame(publish);
  }

  stopFleetSimManualCommandLoop() {
    if (this.fleetSimManualFrame && typeof window.cancelAnimationFrame === "function") {
      window.cancelAnimationFrame(this.fleetSimManualFrame);
    }
    this.fleetSimManualFrame = null;
    this.fleetSimManualLastAt = 0;
    this.fleetSimManualGeneration += 1;
  }

  async waitForFleetSimManualIdle(timeoutMs = 600) {
    const deadline = performance.now() + Math.max(0, Number(timeoutMs || 0));
    while (this.teleopPending && performance.now() < deadline) {
      await new Promise((resolve) => window.setTimeout(resolve, 4));
    }
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
    // Fleet Manager Sim uses the map_top_left frame where positive yaw turns
    // clockwise on screen. ROS cmd_vel keeps the standard positive-left
    // convention, including remote robots supervised by Fleet Manager.
    const simulatedMapTurn = this.isFleetManager() && !this.isFleetRobotsMode();
    return {
      linear: (forward - backward) * linearSpeed,
      angular: (simulatedMapTurn ? right - left : left - right) * angularSpeed,
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
    this.stopFleetSimManualCommandLoop();
    this.syncManualButtons();
    if (this.selectedRobot() && !this.isFleetManager()) {
      this.closeTeleopSocket(true);
    }
    if (this.isFleetManager()) {
      this.releaseFleetManualControl().catch(() => {});
    }
  }

  async sendFleetManualStep(twist) {
    const generation = this.fleetSimManualGeneration;
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
      const stopped = await this.postJson(this.fleetApiPath("/robots/stop"), {
        name: robot.name,
        includeState: false,
      });
      if (generation !== this.fleetSimManualGeneration || !this.manualKeys.size) {
        return;
      }
      this.fleetManualRobotName = robot.name;
      this.fleetManualLastAt = performance.now();
      this.currentStatus = stopped.robot
        ? this.mergeFleetRobotUpdate(stopped.robot)
        : this.currentStatus;
    }
    const pose = this.animatedFleetManualPose(robot) || robot.pose || this.poseForLm(robot.currentLm);
    if (!pose) {
      this.robotMessageText.textContent = `${robot.name}: no pose for manual control.`;
      return;
    }
    const now = performance.now();
    const dt = Math.min(0.16, Math.max(1 / 120, (now - (this.fleetManualLastAt || now)) / 1000));
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
    // Start visual motion before awaiting HTTP. Previously every command
    // froze the model until the global fleet lock became available.
    this.setFleetManualAnimation(robot.name, pose, twist);
    const result = await this.postJson(this.fleetApiPath("/manual-step"), {
      name: robot.name,
      poses: prediction,
      blockedPose: pose,
      nextPose,
      blockedCurrentLm: this.currentLmForPose(pose, 0.25),
      currentLm,
    });
    if (generation !== this.fleetSimManualGeneration || !this.manualKeys.size) {
      return;
    }
    // The old animation kept moving while the HTTP request was in flight.
    // Restarting from `pose` here moved the model backwards on every ACK.
    // Continue from the pose currently visible on screen instead.
    const visualPoseAtAck = this.animatedFleetManualPose(robot) || nextPose;
    this.fleetManualLookahead = {
      poses: prediction,
      blocked: Boolean(result.blocked),
      reason: result.reason || "",
    };
    this.currentStatus = result.state
      ? this.mergeFleetTickState(result.state)
      : this.mergeFleetRobotUpdate(result.robot);
    this.fleetStatusReceivedAt = performance.now();
    this.fleetStatusObjectRef = this.currentStatus;
    if (result.blocked) {
      this.fleetManualAnimation = null;
      this.robotMessageText.textContent = `${robot.name} manual blocked: ${result.reason || "collision"}.`;
      this.renderFleetRuntimeTick();
      return;
    }
    this.setFleetManualAnimation(robot.name, visualPoseAtAck, twist);
    this.robotMessageText.textContent = `${robot.name} manual control active.`;
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
    this.currentStatus = result.state
      ? this.mergeFleetTickState(result.state)
      : this.mergeFleetRobotUpdate(result.robot);
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
    if (this.isFleetManagerSim()) {
      await this.waitForFleetSimManualIdle();
    }
    const robot = this.selectedFleetRobot();
    if (robot && robot.name === this.fleetManualRobotName) {
      if (this.isFleetRobotsMode()) {
        const result = await this.postJson(this.fleetApiPath("/manual-stop"), { name: robot.name });
        const state = result.state || await this.getJson(this.fleetApiPath("/state"));
        this.currentStatus = this.mergeFleetTickState(state);
        this.fleetStatusReceivedAt = performance.now();
        this.fleetStatusObjectRef = this.currentStatus;
        this.fleetManualRobotName = "";
        this.fleetManualLastAt = 0;
        this.fleetManualLookahead = null;
        this.fleetManualAnimation = null;
        this.renderFleetStateImmediately();
        return;
      }
      const pose = this.animatedFleetManualPose(robot) || robot.pose || null;
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
      const state = result.state || await this.getJson(this.fleetApiPath("/state"));
      this.currentStatus = this.mergeFleetTickState(state);
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
      this.setSelectedRobotId(result.robot.id);
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
        this.setSelectedRobotId("");
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
    this.mapSyncDecisionDetail.textContent = `Local: ${localName}. ${target} active: ${remoteName}. Choose Push to upload and verify the local map in storage, Pull to replace the local draft, or Cancel to leave both unchanged. Load activates a pushed map separately.`;
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
          ? await this.postJson(this.fleetApiPath("/maps/pull-sync", robot), {})
          : await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/pull-sync`, {});
        if (this.selectedRobotId !== robot.id) {
          return payload;
        }
        await progress(72, "Saving local operator copy...", 120);
        await this.refreshRobotMapState({ quiet: true });
        await progress(90, "Refreshing map view...", 80);
        await this.fetchSelectedRobotStatus(true);
        return payload;
      });
      if (this.selectedRobotId !== robot.id) {
        return;
      }
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
      return null;
    }
    if (!this.isFleetManager(robot) && this.slamActive) {
      this.robotMessageText.textContent = "Push Map is disabled while 2D SLAM is active.";
      return null;
    }
    const target = this.isFleetManager(robot) ? "Fleet Manager" : "robot";
    if (!options.skipConfirm) {
      const confirmed = window.confirm(
        `Upload and verify the local operator map in ${target} storage? Push does not activate it; use Load separately.`,
      );
      if (!confirmed) {
        return null;
      }
    }
    try {
      const result = await this.runMapTransfer("push", async (progress) => {
        await progress(16, "Preparing local map package...", 120);
        this.beginRobotMapTransition(`Pushing local map to ${target}...`);
        const payload = this.isFleetManager(robot)
          ? await this.postJson(this.fleetApiPath("/maps/push-sync", robot), {})
          : await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/push-sync`, {});
        if (this.selectedRobotId !== robot.id) {
          return payload;
        }
        await progress(74, `Writing map to ${target}...`, 120);
        await this.refreshRobotMapState({ quiet: true });
        await progress(90, "Refreshing operator state...", 80);
        await this.refreshRobots({ quiet: true, lightweight: true });
        await this.fetchSelectedRobotStatus(true);
        return payload;
      });
      if (this.selectedRobotId !== robot.id) {
        return null;
      }
      this.clearSelectedPendingPush();
      this.renderSelectedRobot();
      this.robotMessageText.textContent = result.message || "Push map completed.";
      return result;
    } catch (error) {
      window.alert(error.message || String(error));
      return null;
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
        ? await this.getJson(this.fleetApiPath("/maps/list", robot))
        : await this.getJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/list`);
      if (this.selectedRobotId !== robot.id) {
        return;
      }
      const maps = Array.isArray(robotMaps.maps) ? robotMaps.maps : [];
      if (!maps.length) {
        window.alert(this.isFleetManager(robot) ? "Fleet Manager has no maps." : "Robot has no editable maps.");
        return;
      }
      this.pendingRobotMaps = maps;
      this.pendingRobotMapsRobotId = robot.id;
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
    if (this.pendingRobotMapsRobotId !== robot.id) {
      this.loadMapDialog.close();
      this.pendingRobotMaps = [];
      this.pendingRobotMapsRobotId = "";
      this.robotMessageText.textContent = "Map library context changed. Open Load again for the selected system.";
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
    const selectedIsActive = (selectedMap && selectedMap.active)
      || (activeName && mapName.replace(/\.smap$/, "") === activeName);
    if (selectedIsActive && !this.robotMapState.activationRequired) {
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
          loaded = await this.postJson(this.fleetApiPath("/maps/load", robot), { mapName });
        } else {
          loaded = await this.postJson(`/api/robots/${encodeURIComponent(robot.id)}/maps/load`, { mapName });
        }
        if (this.selectedRobotId !== robot.id) {
          return loaded;
        }
        await progress(68, "Refreshing operator map state...", 100);
        return loaded;
      });
      if (this.selectedRobotId !== robot.id) {
        return;
      }
      this.loadMapDialog.close();
      this.pendingRobotMaps = [];
      this.pendingRobotMapsRobotId = "";
      if (this.isFleetManager(robot)) {
        this.invalidateOperatorScene3d();
      }
      this.applyLoadedMapResult(result, mapName, robot);
      if (this.isFleetManager(robot)) {
        this.currentStatus = await this.getJson(this.fleetApiPath("/state", robot)).catch(() => this.currentStatus);
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

  async getJson(url, options = {}) {
    return httpClient.get(url, options);
  }

  async postJson(url, payload) {
    return httpClient.post(url, payload);
  }

  async postJsonRaw(url, payload) {
    return httpClient.post(url, payload, { rejectApplicationError: false });
  }

  async deleteJson(url) {
    return httpClient.delete(url);
  }

  escapeHtml(value) {
    return escapeHtml(value);
  }
};
