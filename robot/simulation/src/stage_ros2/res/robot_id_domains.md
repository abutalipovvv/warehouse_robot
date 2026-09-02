# One Stage world, isolated robot IDs

`fleet_stage.launch.py` reads `config/fleet.yaml` and generates one shared
Stage world from the SMAP PGM. Robot stacks are never started by this launch;
each complete Nav2 + `robot_driver` stack runs in a separate Docker container.
The Stage model name is `robot_id`, `ros_domain_id` is that robot's
`ROS_DOMAIN_ID`, and ROS namespaces stay empty.

```bash
source robot/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch stage_ros2 fleet_stage.launch.py enable_gui:=true
```

The default fleet file is:

```yaml
world: 22.05.26_smap
smap: ../../../../../fleet_manager/map_data/maps_out/22.05.26_smap.smap

robots:
  - robot_id: robot11
    ros_domain_id: 11
    ip: 127.0.0.11
    grpc_port: 50051
    lm: LM91
    yaw: 180.0
    color: LightSteelBlue
```

`lm` must exist in the SMAP's `LMs.yaml`. Stage spawn and AMCL initial pose are
both derived from that LM. The ROS map YAML and PGM in the same SMAP are passed
unchanged to Nav2 and used to calculate the Stage floorplan size, center,
resolution and origin.

Add or remove entries to change the robot count. A custom file inside the
workspace can be passed without editing the package; headless mode disables
only FLTK:

```bash
ros2 launch stage_ros2 fleet_stage.launch.py \
  fleet_config:=/absolute/path/to/my_fleet.yaml enable_gui:=false
```

The sample launches these interfaces from one shared physical world:

| Stage model / `ROBOT_ID` | `ROS_DOMAIN_ID` | ROS interfaces |
| --- | ---: | --- |
| `robot11` | 11 | `/cmd_vel`, `/scan`, `/odom`, `/imu`, `/tf`, `/clock` |
| `robot12` | 12 | `/cmd_vel`, `/scan`, `/odom`, `/imu`, `/tf`, `/clock` |
| `robot13` | 13 | `/cmd_vel`, `/scan`, `/odom`, `/imu`, `/tf`, `/clock` |
| `robot14` | 14 | `/cmd_vel`, `/scan`, `/odom`, `/imu`, `/tf`, `/clock` |

Build the robot image once, then start the containers independently:

```bash
docker compose -f robot/simulation/docker/compose.yaml build
docker compose -f robot/simulation/docker/compose.yaml up -d robot11
docker compose -f robot/simulation/docker/compose.yaml up -d robot12
```

Each Compose service runs `robot_launch/container.launch.py`; its identity,
map and LM-derived initial pose are explicit environment values in
`compose.yaml`. `docker compose up -d` starts all services, but they remain
independent containers rather than native child processes.

Local simulation containers use host networking for reliable CycloneDDS discovery.
Distinct loopback addresses from `127.0.0.0/8` let every robot keep gRPC port
`50051`. Fleet Manager connects by IP; ROS separation is handled by DDS
domains, not by IP or namespace.

The optional `robot_domain_map` launch argument can override only the domain
numbers while reusing the LM-derived poses. It must contain exactly the same IDs:

```bash
ros2 launch stage_ros2 fleet_stage.launch.py \
  robot_domain_map:='robot11=71,robot12=72,robot13=73,robot14=74'
```

Robot IDs and domains must be unique. Invalid, partial or duplicate mappings
are rejected before Stage starts. The generated temporary world is removed on
launch shutdown.

Each robot domain exposes `/reset_odom` and `/reset_positions` for that robot
only. The Stage supervisor exposes whole-world resets as
`/stage/reset_odom` and `/stage/reset_positions` in its own launch domain.
