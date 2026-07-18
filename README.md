# warehouse_robot

ROS 2 + web stack for a warehouse robot simulator/operator workflow.

Main runtime split:

- `sim_robot/ws/src/robot_planner` - route planning and execution ROS 2 node.
- `sim_robot/ws/src/robot_status` - robot status ROS 2 node.
- `sim_robot/ws/src/robot_map_manager` - robot map state/load/sync ROS 2 services.
- `sim_robot/ws/src/robot_grpc_api` - local ROS 2 robot gRPC API backed by robot topics/services.
- `operator_app/core/grpc` - local Operator App copy of the robot gRPC client contract.
- `fleet_manager/robot_grpc_api` - local Fleet Manager copy of the robot gRPC client contract.
- `operator_app/core` - Operator App state, Fleet Manager bridge, gRPC client and domain logic.
- `operator_app/web` - offline browser UI and HTTP/WebSocket transport.
- `operator_app/server.py` - Operator App entry point; settings live in `operator_app/config/config.yaml`.
- `fleet_manager` - Fleet Manager map/MAPF/runtime code.

Map/planning ownership:

- `fleet_manager/route_core` owns Fleet Manager map loading/edit exchange for `fleet_manager/map_data`.
- `fleet_manager/mapf` owns Fleet Manager MAPF and space-time planning.
- `sim_robot/ws/src/robot_planner/robot_planner/route_core` owns robot-side map/route loading.
- Operator App stores its local editable map cache independently and only synchronizes by push/pull.
- `robot_grpc_api` is only the network API contract/runtime; it does not own maps or MAPF.

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

Traffic tuning is under `fleet` in `fleet_manager/config/params.yaml`. The principal
settings are `congestion_*`, `traffic_zone_*`, `rolling_horizon_sec`, and
`local_cbs_max_robots`. Runtime state exposes zone demand, occupancy, queue and
phase in `trafficFlow`.

Install gRPC runtime on the operator/server and on each robot:

```bash
sudo apt install python3-grpcio
```

## Build Robot Packages

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_msgs robot_planner robot_status robot_map_manager robot_grpc_api robot_launch
source install/local_setup.bash
```

## Run Simulator

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch stage_ros2 stage.launch.py enforce_prefixes:=false one_tf_tree:=false
```

If needed, start Nav2/AMCL in a separate terminal:

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch nav2 nav2_launch.py
```

## Run Robot Stack

This starts status, route execution, map manager, and native gRPC robot API nodes.

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch robot_launch launch.py
```

Useful launch overrides:

```bash
ros2 launch robot_launch launch.py map_dir:=/path/to/map.smap params:=/path/to/params.yaml
```

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
