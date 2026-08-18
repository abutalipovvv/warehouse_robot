# Robot planner

`robot_planner` is the robot-local navigation layer. It does not perform MAPF
coordination; it consumes either a goal LM or an already coordinated LM route.

## Package layout

```text
robot_planner/
├── control/       PID feedback and controller parameters
├── execution/     route state machine and velocity command policy
├── math/          NumPy trajectory geometry and projections
├── planning/      current-pose/LM route construction
├── route_core/    map loading, graph models, directed A*, persistence
├── route_node.py  ROS services, topics and node wiring
└── runtime.py     thread-safe route/executor state
```

Code imports its responsibilities directly from `control`, `execution`,
`math`, `planning`, and `route_core`; duplicate root-level compatibility
modules are intentionally not kept.

## Data flow

1. `RobotStatusNode` transforms `map -> base_link` into the map's top-left
   coordinate frame. This AMCL pose is the global anchor used for A* and
   selecting the nearest LM.
2. `RobotTrajectoryPlanner` selects the active edge or nearest LM and runs A*
   over directed graph edges.
3. Straight and cubic Bezier edges are sampled into one immutable
   `TrajectoryArray` (NumPy structure of arrays).
4. `RouteExecutor` anchors the global pose to `/odom`, then uses continuous
   odometry in its fast local control loop. It projects the robot only onto
   the active graph edge and enforces LM boundaries, traffic time gates, and
   motion-direction changes. The anchor is kept between consecutive route
   chunks and reset when the map or `/initialpose` changes.
5. `PidController` combines lateral and heading error with curvature
   feed-forward and publishes differential-drive `(linear.x, angular.z)`.

## MAPF contract

The coordinator sends an `lm_route` version 2 command containing the goal,
ordered LM nodes, route identity/revision, optional traffic time windows, and
replacement mode. The robot validates that every ordered edge exists in the
local directed graph and that any explicit motion direction agrees with the
map. Edge geometry, Bezier sampling, velocity profiling, PID feedback and
arrival detection are deliberately robot-local responsibilities.

An explicit MAPF route never creates a straight fallback from an arbitrary
pose to its first LM. The pose must already be at the first LM or project onto
a graph edge that reaches it; otherwise execution is rejected so the robot
cannot cut across the map outside reserved edges.

The supported replacement mode is currently `immediate`. Unsupported protocol
versions, stale revisions, disconnected nodes and timing entries outside the
route are rejected instead of being silently repaired.

## Direction semantics

- Graph topology is always `from -> to`; A* never traverses an absent reverse
  edge.
- `forward` means positive chassis velocity along that edge.
- `backward` means negative chassis velocity; the desired body yaw is the edge
  tangent plus pi.
- `not_specified` currently uses forward motion and does not invent a reverse
  graph connection.

Sharp heading changes and forward/backward changes must be performed at the LM.
Geometrically continuous edges advance by along-track progress, preventing an
executor deadlock when a small lateral error remains at the shared LM. In
`strict_edge_tracking` mode the next edge is not activated until progress is
within 2 mm of that boundary, so a following Bezier cannot produce an early
turn on the current edge.

## Strict curve tracking

The lateral Stanley term uses the effective capped velocity, rather than the
uncapped route request. Curves are detected over `curve_preview_distance`; the
preview changes linear speed only and never the current edge heading. The
velocity policy keeps `curve_angular_reserve` of `max_angular_speed` available
for PID correction after curvature feed-forward. This avoids both cutting a
tight Bezier and saturating angular control for the whole turn.

Bezier curves are sampled at approximately uniform arc-length spacing. The
default robot-local spacing is 10 mm, while projection and lookahead continue
to interpolate continuously between samples.

## Speed and precise arrival

`SpeedProfiler` applies acceleration limiting, curvature/error caps, a braking
envelope and a dedicated final-approach speed below
`planner.precision_start_distance`. `stop_distance` starts braking but is not
the arrival tolerance.

Arrival requires all of the following for several consecutive control cycles:

- goal position and along-track error within
  `localization.goal_position_tolerance` (5 mm by default);
- final yaw within `localization.allowed_yaw_error_deg`;
- measured linear and angular velocity below their goal tolerances.

The 5 mm value is a controller acceptance threshold. Absolute 5 mm accuracy on
physical hardware also requires localization or a final docking sensor capable
of measuring position at that accuracy.

## Runtime contract

Inputs are `/robot_status`, `/odom`, `/initialpose`, and route services. Output
is `/cmd_vel`. A route is allowed to finish only when both position and final
yaw are stable within configured tolerances. `/robot_executor_state` and the
gRPC status `tracking` object expose current/max/mean cross-track error, heading
and goal errors, commanded velocity, and arrival stability counters. INFO logs
report lifecycle events and edge changes; detailed PID terms stay at DEBUG
level.
