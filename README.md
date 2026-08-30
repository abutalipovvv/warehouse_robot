# warehouse_robot

ROS 2 + web stack for warehouse robots, simulation, and operator workflow.

Main runtime split:

- `robot/ros2_libs` - pinned third-party ROS 2 source overlay, built rarely.
- `robot/robot_driver` - hardware-independent robot runtime: route execution,
  localization bringup, maps, status, gRPC API, and motion arbitration.
- `robot/simulation` - Stage implementation of the robot hardware contract.
- `robot/robot_driver/src/robot_planner` - route planning and execution ROS 2 node.
- `robot/robot_driver/src/robot_status` - robot status ROS 2 node.
- `robot/robot_driver/src/robot_map_manager` - robot map state/load/sync services.
- `robot/robot_driver/src/robot_grpc_api` - local ROS 2 robot gRPC API.
- `robot/robot_driver/src/robot_motion_gateway` - the only node allowed to publish
  the final driver `cmd_vel`; it arbitrates route, teleop and Nav2 commands.
- `robot/robot_driver/src/warehouse_nav2_bringup` - localization-first Nav2 launch
  without demo maps, worlds or robot models.
- `operator_app/core/grpc` - Operator App client code using the canonical
  Fleet Manager robot gRPC contract directly.
- `fleet_manager/runtime/grpc/api` - Fleet Manager robot gRPC client/runtime contract.
- `operator_app/core` - Operator App state, Fleet Manager bridge, gRPC client and domain logic.
- `operator_app/web` - offline browser UI and HTTP/WebSocket transport.
- `operator_app/server.py` - Operator App entry point; settings live in `operator_app/config/config.yaml`.
- `fleet_manager` - Fleet Manager map/MAPF/runtime code.

Map/planning ownership:

- `fleet_manager/core/mapping` owns Fleet Manager map loading/edit exchange for `fleet_manager/map_data`.
- `fleet_manager/core/mapf` owns Fleet Manager MAPF and space-time planning.
- `fleet_manager/core/math` contains reusable mathematical
  primitives and graph-search algorithms.
- `fleet_manager/core/mapping/formats/smap_bundle.py` and `smap_raster.py` separate SMAP
  parsing, graph reconstruction, raster math, and durable output.
- `robot/robot_driver/src/robot_planner/robot_planner/route_core` owns robot-side map/route loading.
- Operator App stores its local editable map cache independently and only synchronizes by push/pull.
- `robot_grpc_api` is only the network API contract/runtime; it does not own maps or MAPF.

See [`docs/architecture.md`](docs/architecture.md) for dependency boundaries
and a suggested code-reading order. Test commands are in
[`docs/testing.md`](docs/testing.md).
Русскоязычный маршрут по коду находится в
[`docs/code-reading-guide.ru.md`](docs/code-reading-guide.ru.md).

Runtime transport rule:

- Browser/site <-> Operator App: HTTP/WebSocket.
- Operator App/Fleet Manager <-> robots: gRPC over TCP.
- Inside each robot: local ROS 2/Nav2/topics/services, not exposed to the server.

## Fleet traffic planning

Fleet Manager uses a hierarchy instead of running fleet-wide CBS for every
order:

1. Congestion-aware A* assigns a stable spatial route. Its cost includes the
   remaining committed routes of other robots and a larger penalty for
   opposing traffic, so equal shortest paths are distributed across the map.
2. Dynamic traffic zones admit a capacity-limited group of compatible robots.
   Other robots hold at the last graph LM outside the overloaded zone. Zone
   phases and waiting age prevent alternating-direction thrashing and
   starvation.
3. Rolling SIPP reserves exact graph resources over the configured temporal
   horizon and schedules ordinary waits.
4. Local CBS resolves only the small coupled component that SIPP could not
   separate. Persistent stalls may retreat one robot and request a global
   congestion-aware detour to the same goal.
5. Explicit controlled corridors are an optional local admission layer for
   genuinely narrow passages. An operator marks their edges and legal holding
   LMs with the Corridor tool in Traffic Editor. They do not replace dynamic
   traffic zones, SIPP or CBS, and automatic whole-map corridor inference is
   disabled by default.

Traffic tuning is under `fleet` in `fleet_manager/config/params.yaml`. The principal
settings are `congestion_*`, `traffic_zone_*`, `rolling_horizon_sec`,
`controlled_corridors_enabled`, `controlled_corridor_auto_detect`, and
`local_cbs_max_robots`. Runtime state exposes zone and controlled-corridor
demand, occupancy, queue and phase in `trafficFlow`.

Install gRPC runtime on the operator/server and on each robot:

```bash
sudo apt install python3-grpcio
```

## Build Robot Packages

The reviewed ROS 2 Jazzy dependencies are vendored under
`robot/ros2_libs/src`. A fresh clone therefore contains everything required by
the source overlays; no additional Git checkout is required.

Verify that every ROS dependency is either in the workspace or supplied by
`ros-jazzy-desktop`:

```bash
./robot/tools/check_ros_jazzy_source_dependencies.sh
```

Install the remaining non-ROS Ubuntu build libraries explicitly. This replaces
the opaque `rosdep install` step and does not install binary Nav2 packages:

```bash
./robot/tools/install_ubuntu_build_dependencies.sh
```

Build the pinned libraries once, then build the common runtime and simulator as
separate overlays. Rebuild only the layer whose sources changed:

```bash
./robot/tools/build_ros2_libs.sh
./robot/tools/build_robot_driver.sh
./robot/tools/build_simulation.sh

# Or build all three in dependency order:
./robot/tools/build_all.sh
```

The build does not require `rosdep install`.

For Ubuntu 24.04 x86-64 with ROS 2 Jazzy, the stable library and simulation
overlays are also stored as compressed prebuilt artifacts. A fresh compatible
machine can restore them without compiling those two layers:

```bash
./robot/tools/restore_prebuilt.sh
./robot/tools/build_robot_driver.sh
source robot/setup.bash
```

Prebuilt overlays are ABI-specific. ARM64 or another Ubuntu/ROS release needs
its own bundle; see `robot/prebuilt/README.md`.

## Run Simulator

```bash
cd ~/warehouse_robot
source robot/setup.bash
ros2 launch stage_ros2 simulation.launch.py
```

If needed, start Nav2/AMCL in a separate terminal:

```bash
cd ~/warehouse_robot
source robot/setup.bash
ros2 launch warehouse_nav2_bringup nav2_launch.py profile:=localization
```

Use `profile:=navigation` only when the Nav2 `NavigateToPose` stack is needed.
Its velocity output is routed to `motion/nav2_cmd_vel`, never directly to the
driver.

## Run Robot Stack

This starts the motion gateway, status, route execution, map manager, and
native gRPC robot API nodes.

```bash
cd ~/warehouse_robot
source robot/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch robot_launch launch.py
```

Useful launch overrides:

```bash
ros2 launch robot_launch launch.py map_dir:=/path/to/map.smap params:=/path/to/params.yaml
```

For a physical robot, provision a persistent identity from
`robot/config/robot.env.example` and validate it with
`robot/tools/validate_robot_identity.py`. Fleet communication remains
gRPC; the robot's DDS domain is isolated from other robots.

## Motion ownership

Producers publish only to their dedicated gateway inputs:

- graph route controller: `motion/route_cmd_vel` in mode `ROUTE`;
- manual control: `motion/teleop_cmd_vel` in mode `TELEOP`;
- Nav2: `motion/nav2_cmd_vel` in mode `NAV2`.

`IDLE`, `SLAM`, and `ESTOP` always produce zero velocity. A stale active input
also produces zero through the gateway watchdog. Limits and watchdog timeouts
are reloaded from `motion_gateway` in
`robot/robot_driver/src/params/params.yaml`.

## Run Operator App

Run this on the operator computer. Add robots by IP and gRPC port, default `50051`.

```bash
cd ~/warehouse_robot
python3 serve_operator.py --open
```

The browser runtime, including Babylon.js, is stored locally under
`operator_app/web/static/vendor`; Operator App does not need internet access to
start or render maps. Runtime workspaces and map caches live under
`var/operator_app`.
