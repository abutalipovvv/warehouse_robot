# warehouse_robot

ROS 2 + web stack for a warehouse robot simulator/operator workflow.

Main runtime split:

- `sim_robot/ws/src/robot_planner` - route planning and execution ROS 2 node.
- `sim_robot/ws/src/robot_status` - robot status ROS 2 node.
- `sim_robot/ws/src/robot_map_manager` - robot map state/load/sync ROS 2 services.
- `sim_robot/ws/src/robot_api` - native gRPC robot API backed by local ROS 2 topics/services.
- `robot_grpc_api` - local gRPC transport library used by robots, Operator App, and Fleet Manager.
- `operator_app` - desktop/web operator application. Browser talks to it over HTTP/WebSocket; robot communication uses gRPC.
- `fleet_manager/web_simulator` - web/no-ROS fleet simulator runtime.
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

Install gRPC runtime on the operator/server and on each robot:

```bash
python3 -m pip install grpcio
```

## Build Robot Packages

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_msgs robot_planner robot_status robot_map_manager robot_api robot_launch
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
