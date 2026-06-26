# robot_http_api

Robot-side HTTP API for the operator application.

This package runs one process that contains:

- HTTP server on `0.0.0.0:8790` by default.
- ROS 2 client bridge for `/robot_status`, `/cmd_vel`, route services, and map manager services.

It does not serve the operator UI. The UI is `operator_app`; it connects to this API by robot IP.

## Build

```bash
cd ~/warehouse_robot/robot
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_msgs robot_planner robot_status robot_map_manager robot_http_api robot_launch
source install/local_setup.bash
```

## Run With Full Robot Stack

Preferred:

```bash
ros2 launch robot_launch launch.py http_host:=0.0.0.0 http_port:=8790
```

## Run API Only

Only use this when the other robot nodes are already running:

```bash
ros2 run robot_http_api robot_http_api \
  --map-dir /home/kaisar/warehouse_robot/fleet_manager/map_data/maps_out/22.05.26_smap.smap \
  --params /home/kaisar/warehouse_robot/fleet_manager/params.yaml \
  --host 0.0.0.0 \
  --port 8790
```

## Main Endpoints

- `GET /health`
- `GET /api/robot/identity`
- `GET /api/robot/status`
- `POST /api/robot/teleop`
- `POST /api/robot/teleop/stop`
- `POST /api/robot/route/plan`
- `POST /api/robot/route/execute`
- `POST /api/robot/route/cancel`
- `POST /api/robot/stop`
- `GET /api/maps/list`
- `GET /api/maps/active`
- `GET /api/maps/pull?name=<map>`
- `POST /api/maps/push`
- `POST /api/maps/load`
- `GET /api/params`
- `POST /api/params`
