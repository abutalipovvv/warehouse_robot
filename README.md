# warehouse_robot

ROS 2 + web stack for a warehouse robot simulator/operator workflow.

Main runtime split:

- `robot/ws/src/robot_planner` - route planning and execution ROS 2 node.
- `robot/ws/src/robot_status` - robot status ROS 2 node.
- `robot/ws/src/robot_map_manager` - robot map state/load ROS 2 services.
- `robot/ws/src/robot_http_api` - robot-side HTTP API that bridges operator requests to ROS 2.
- `operator_app` - desktop/web operator application. Connects to a robot by IP, pulls/pushes maps, sends teleop and route commands.
- `fleet_manager` - fleet logic and shared map data.

## Build Robot Packages

```bash
cd ~/warehouse_robot/robot
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_msgs robot_planner robot_status robot_map_manager robot_http_api robot_launch
source install/local_setup.bash
```

## Run Simulator

```bash
cd ~/warehouse_robot/robot
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch stage_ros2 stage.launch.py enforce_prefixes:=false one_tf_tree:=false
```

If needed, start Nav2/AMCL in a separate terminal:

```bash
cd ~/warehouse_robot/robot
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch nav2 nav2_launch.py
```

## Run Robot Stack

This starts status, route execution, map manager, and the robot HTTP API on `0.0.0.0:8790`.

```bash
cd ~/warehouse_robot/robot
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch robot_launch launch.py
```

Useful launch overrides:

```bash
ros2 launch robot_launch launch.py http_host:=0.0.0.0 http_port:=8790
ros2 launch robot_launch launch.py map_dir:=/path/to/map.smap params:=/path/to/params.yaml
```

## Run Operator App

Run this on the operator computer. Then add a robot by IP, for example `192.168.11.104:8790`.

```bash
cd ~/warehouse_robot
python3 serve_operator.py --open
```

## Legacy Single Robot Entry

`serve_robot.py` is kept as a compatibility wrapper around `robot_http_api`. Prefer `ros2 launch robot_launch launch.py` because it starts all robot ROS nodes and the HTTP API together.

```bash
cd ~/warehouse_robot
source /opt/ros/jazzy/setup.bash
source robot/install/local_setup.bash
python3 serve_robot.py --map-dir maps_out/22.05.26_smap.smap
```
