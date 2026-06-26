(function () {
  class GeometryService {
    constructor(mapData) {
      this.mapData = mapData;
    }

    worldToPixel(point) {
      const px = this.mapData.viewPadding + ((point.x - this.mapData.origin[0]) / this.mapData.resolution);
      const py = this.mapData.viewPadding + (this.mapData.height - 1) - ((point.y - this.mapData.origin[1]) / this.mapData.resolution);
      return { x: px, y: py };
    }
  }

  class OperatorPanelApp {
    constructor() {
      this.dom = this.getDom();
      this.config = null;
      this.geometry = null;
      this.stateTimer = null;
      this.lastStateHash = "";
    }

    getDom() {
      return {
        mapTitle: document.getElementById("mapTitle"),
        mapSvg: document.getElementById("mapSvg"),
        mapImage: document.getElementById("mapImage"),
        graphLayer: document.getElementById("graphLayer"),
        routeLayer: document.getElementById("routeLayer"),
        pointLayer: document.getElementById("pointLayer"),
        robotLayer: document.getElementById("robotLayer"),
        goalSelect: document.getElementById("goalSelect"),
        sendGoalButton: document.getElementById("sendGoalButton"),
        stopButton: document.getElementById("stopButton"),
        modeText: document.getElementById("modeText"),
        nearestLmText: document.getElementById("nearestLmText"),
        nearestLmDistanceText: document.getElementById("nearestLmDistanceText"),
        robotPoseText: document.getElementById("robotPoseText"),
        goalText: document.getElementById("goalText"),
        statusText: document.getElementById("statusText"),
        routeList: document.getElementById("routeList"),
      };
    }

    async init() {
      this.config = await this.fetchJson("/api/config");
      this.geometry = new GeometryService(this.config.map);
      this.initMap();
      this.populateGoalSelector();
      this.drawGraph();
      this.drawLandmarks("", "");
      this.attachEvents();
      await this.refreshState();
      this.stateTimer = window.setInterval(() => {
        this.refreshState().catch((error) => {
          this.dom.statusText.textContent = `State update failed: ${error.message}`;
        });
      }, 250);
    }

    async fetchJson(url, options) {
      const response = await fetch(url, options);
      if (!response.ok) {
        throw new Error(`${response.status} ${response.statusText}`);
      }
      return response.json();
    }

    initMap() {
      const map = this.config.map;
      this.dom.mapTitle.textContent = this.config.mapName;
      document.title = `${this.config.mapName} Operator Panel`;
      this.dom.mapSvg.setAttribute("viewBox", `0 0 ${map.viewWidth} ${map.viewHeight}`);
      this.dom.mapImage.setAttribute("x", String(map.viewPadding));
      this.dom.mapImage.setAttribute("y", String(map.viewPadding));
      this.dom.mapImage.setAttribute("width", String(map.width));
      this.dom.mapImage.setAttribute("height", String(map.height));
      this.dom.mapImage.setAttribute("href", map.imageDataUrl);
    }

    populateGoalSelector() {
      for (const lm of this.config.lms) {
        const option = document.createElement("option");
        option.value = lm.name;
        option.textContent = lm.name;
        this.dom.goalSelect.appendChild(option);
      }
    }

    attachEvents() {
      this.dom.sendGoalButton.addEventListener("click", () => this.sendGoal());
      this.dom.stopButton.addEventListener("click", () => this.stopRobot());
    }

    createSvgElement(tag, attrs) {
      const element = document.createElementNS("http://www.w3.org/2000/svg", tag);
      for (const [key, value] of Object.entries(attrs)) {
        element.setAttribute(key, String(value));
      }
      return element;
    }

    drawGraph() {
      this.dom.graphLayer.innerHTML = "";
      const seen = new Set();

      for (const edge of this.config.edges) {
        const key = [edge.from, edge.to].sort().join("|");
        if (seen.has(key)) {
          continue;
        }
        seen.add(key);

        if (edge.geometry === "bezier" && edge.control_points && edge.control_points.length === 4) {
          const cp = edge.control_points.map((point) => this.geometry.worldToPixel(point));
          this.dom.graphLayer.appendChild(
            this.createSvgElement("path", {
              d: `M ${cp[0].x} ${cp[0].y} C ${cp[1].x} ${cp[1].y}, ${cp[2].x} ${cp[2].y}, ${cp[3].x} ${cp[3].y}`,
              fill: "none",
              stroke: "var(--edge)",
              "stroke-width": 2,
              "stroke-linecap": "round",
            })
          );
          continue;
        }

        const start = this.geometry.worldToPixel(this.findLm(edge.from));
        const goal = this.geometry.worldToPixel(this.findLm(edge.to));
        this.dom.graphLayer.appendChild(
          this.createSvgElement("line", {
            x1: start.x,
            y1: start.y,
            x2: goal.x,
            y2: goal.y,
            stroke: "var(--edge)",
            "stroke-width": 2,
            "stroke-linecap": "round",
          })
        );
      }
    }

    drawLandmarks(nearestLm, activeGoalLm) {
      this.dom.pointLayer.innerHTML = "";
      for (const lm of this.config.lms) {
        const pos = this.geometry.worldToPixel(lm);
        let fill = "var(--point)";
        let radius = 4;

        if (lm.name === nearestLm) {
          fill = "var(--nearest)";
          radius = 6;
        }
        if (lm.name === activeGoalLm) {
          fill = "var(--goal)";
          radius = 7;
        }

        this.dom.pointLayer.appendChild(
          this.createSvgElement("circle", {
            cx: pos.x,
            cy: pos.y,
            r: radius,
            fill,
            opacity: 0.95,
          })
        );

        const label = this.createSvgElement("text", {
          x: pos.x,
          y: pos.y + radius + 12,
          class: "lm-label",
        });
        label.textContent = lm.name;
        this.dom.pointLayer.appendChild(label);
      }
    }

    drawRoute(routePath) {
      this.dom.routeLayer.innerHTML = "";
      if (!routePath || routePath.length < 2) {
        return;
      }

      const points = routePath
        .map((pose) => this.geometry.worldToPixel(pose))
        .map((point) => `${point.x},${point.y}`)
        .join(" ");

      this.dom.routeLayer.appendChild(
        this.createSvgElement("polyline", {
          points,
          fill: "none",
          stroke: "var(--route)",
          "stroke-width": 6,
          "stroke-linecap": "round",
          "stroke-linejoin": "round",
          opacity: 0.92,
        })
      );
    }

    drawRobotPose(pose) {
      this.dom.robotLayer.innerHTML = "";
      if (!pose) {
        return;
      }

      const length = 0.70;
      const width = 0.55;
      const halfLength = length / 2;
      const halfWidth = width / 2;
      const cos = Math.cos(pose.yaw);
      const sin = Math.sin(pose.yaw);

      const corners = [
        { x: halfLength, y: halfWidth },
        { x: halfLength, y: -halfWidth },
        { x: -halfLength, y: -halfWidth },
        { x: -halfLength, y: halfWidth },
      ].map((corner) => ({
        x: pose.x + (corner.x * cos) - (corner.y * sin),
        y: pose.y + (corner.x * sin) + (corner.y * cos),
      }));

      const footprint = corners
        .map((point) => this.geometry.worldToPixel(point))
        .map((point) => `${point.x},${point.y}`)
        .join(" ");

      this.dom.robotLayer.appendChild(
        this.createSvgElement("polygon", {
          points: footprint,
          fill: "var(--robot-fill)",
          stroke: "var(--robot)",
          "stroke-width": 2.5,
          "stroke-linejoin": "round",
        })
      );

      const center = this.geometry.worldToPixel(pose);
      const nose = this.geometry.worldToPixel({
        x: pose.x + Math.cos(pose.yaw) * halfLength,
        y: pose.y + Math.sin(pose.yaw) * halfLength,
      });

      this.dom.robotLayer.appendChild(
        this.createSvgElement("line", {
          x1: center.x,
          y1: center.y,
          x2: nose.x,
          y2: nose.y,
          stroke: "var(--robot)",
          "stroke-width": 3,
          "stroke-linecap": "round",
        })
      );
      this.dom.robotLayer.appendChild(
        this.createSvgElement("circle", {
          cx: center.x,
          cy: center.y,
          r: 3.5,
          fill: "var(--robot)",
        })
      );
    }

    async sendGoal() {
      const goalLm = this.dom.goalSelect.value;
      await this.fetchJson("/api/goal-lm", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ goalLm }),
      });
      this.dom.statusText.textContent = `Goal sent: ${goalLm}`;
    }

    async stopRobot() {
      await this.fetchJson("/api/stop", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({}),
      });
      this.dom.statusText.textContent = "Stop requested.";
    }

    async refreshState() {
      const state = await this.fetchJson("/api/state");
      const hash = JSON.stringify(state);
      if (hash === this.lastStateHash) {
        return;
      }
      this.lastStateHash = hash;

      this.dom.modeText.textContent = state.mode;
      this.dom.nearestLmText.textContent = state.nearestLm || "-";
      this.dom.nearestLmDistanceText.textContent = state.nearestLmDistance === null
        ? "-"
        : `${state.nearestLmDistance.toFixed(3)} m ${state.isOnLandmark ? "(AT_LM)" : "(OFF_LM)"}`;
      this.dom.goalText.textContent = state.activeGoalLm || "-";
      this.dom.statusText.textContent = state.statusText;

      if (state.robotPose) {
        this.dom.robotPoseText.textContent =
          `x: ${state.robotPose.x.toFixed(3)}, y: ${state.robotPose.y.toFixed(3)}, yaw: ${state.robotPose.yaw.toFixed(3)}`;
      } else {
        this.dom.robotPoseText.textContent = "x: -, y: -, yaw: -";
      }

      this.dom.routeList.innerHTML = "";
      for (const name of state.routeNodes || []) {
        const item = document.createElement("li");
        item.textContent = name;
        this.dom.routeList.appendChild(item);
      }

      this.drawLandmarks(state.nearestLm || "", state.activeGoalLm || "");
      this.drawRoute(state.routePath || []);
      this.drawRobotPose(state.robotPose);
    }

    findLm(name) {
      return this.config.lms.find((lm) => lm.name === name);
    }
  }

  new OperatorPanelApp().init().catch((error) => {
    const target = document.getElementById("statusText");
    if (target) {
      target.textContent = `Init failed: ${error.message}`;
    }
  });
}());
