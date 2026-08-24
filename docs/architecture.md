# Architecture

Подробная модель владения live state, immutable planning snapshot, revisioned
commit и bounded planning scheduler описана в
[`architecture/state_ownership_and_planning.md`](architecture/state_ownership_and_planning.md).
Фактические границы пакетов и разрешённые направления импортов описаны в
[`architecture/package_boundaries.md`](architecture/package_boundaries.md).

The project is split into four dependency layers. Dependencies point
downwards; the transport-neutral core must not import runtime adapters.

```text
Operator App / ROS entry points
              |
              v
Runtime adapters (gRPC, simulation, owned loops)
              |
              v
Fleet policy (tasks, traffic, MAPF, motion)
              |
              v
Math, search, map models, endpoint and gateway contracts
```

## Reading order

For the planning algorithms, start here:

1. `fleet_manager/core/math` — vectors, poses, polygons and
   half-open intervals.
2. `fleet_manager/core/search` — the generic search problem
   and deterministic A*.
3. `fleet_manager/core/mapf/graph/traffic_graph_models.py` — traffic graph;
   `traffic_graph_builder.py` and
   `traffic_graph_properties.py` contain the model, construction math, and
   legacy-map parsing.
4. `fleet_manager/core/mapf/common/reservations.py` — capacity-aware calendars.
5. `fleet_manager/core/mapf/sipp/sipp.py` — the single-robot planner.
6. `fleet_manager/core/mapf/sipp/sipp_problem.py` — SIPP state transitions.
7. `fleet_manager/core/mapf/rolling/rolling_sipp.py` — multi-robot planner;
   `prioritized_planning.py` and `rolling_reservations.py` contain the policy
   and reservation writing.
8. `fleet_manager/core/mapf/cbs/cbs_high_level.py` — local CBS composition over
   `cbs_models.py`, `cbs_conflicts.py`, and `cbs_low_level.py`. Request
   normalization lives in `cbs_setup.py`; the
   high-level constraint tree lives in `cbs_tree.py`.
9. `fleet_manager/core/mapf/fleet/fleet_planner.py` — fleet planner composition;
   request preparation, backend selection, results, and trajectory generation
   live in the adjacent `fleet_planner_*` modules.

For application execution, start here:

1. `operator_app/server.py` — process entry point.
2. `operator_app/core/state.py` — application-level orchestration.
   Registry, robot control, robot maps, Fleet Manager API/maps, and runtime
   ownership live in the adjacent `state_*` capability modules.
3. `operator_app/core/fleet_manager.py` — web-facing Fleet Manager facade.
   Context, maps, manual commands, snapshots, and benchmarks are separate
   `fleet_*` services.
4. `operator_app/web/handler.py` — HTTP routing and JSON responses;
   `socket_handlers.py` owns complete WebSocket sessions and `websocket.py`
   contains the protocol codec.
5. `fleet_manager/runtime/loop.py` — independently owned real/sim loops.
6. `fleet_manager/manager/manager.py` — transport-neutral
   composition root; adjacent modules own state, snapshots, commands, robot
   lifecycle, remote control and route metadata.
7. `fleet_manager/runtime/grpc` and `fleet_manager/runtime/simulation` —
   concrete execution adapters.

## Mathematical conventions

- Angles are radians.
- `Vector2` and `Pose2D` are immutable.
- Time intervals are half-open: `[start, end)`.
- Touching time windows do not conflict.
- Search costs and heuristics are finite and non-negative.
- A* tie-breaking follows neighbour iteration order.
- SIPP state identity is landmark + safe interval + heading; earlier arrival
  dominates later arrival inside the same identity.

## Planning hierarchy

The Fleet Manager intentionally combines algorithms instead of asking one
global solver to do every job:

1. Congestion-aware spatial routing selects a stable route.
2. Traffic zones and controlled corridors decide admission.
3. Rolling SIPP schedules reservations and ordinary waiting.
4. Local CBS handles only a small coupled conflict that prioritization could
   not resolve.
5. Runtime recovery can retreat or detour a stalled robot.

Each layer should expose a small input/output model. It should not reach into
UI state or a concrete gRPC client.

The larger orchestration classes are composition modules:

- `manager/movement/motion.py` composes step, kinematics, safety, retreat
  and replanning;
- `manager/coordination/routing/routing.py` composes spatial detours, traffic-zone admission,
  controlled-corridor admission/prefetch/passage and rolling-route helpers;
- `manager/coordination/coordinator.py` composes wait detection, priority, leases,
  evacuation, escape installation and wait-cycle recovery;
- `manager/tasks/dispatch.py` composes order admission, planning jobs, request
  construction, continuations, recovery and result commit;
- `operator_app/core/state.py` and `operator_app/core/fleet_manager.py` expose
  stable application APIs while delegating capabilities to focused services.

Existing hook names remain on these facade classes, while each implementation
file represents one reason to change.

Long recovery flows are pipelines rather than single methods:

- rolling collapse collects a stopped cohort, builds a resource-dependency
  graph, selects a free sink, and only then runs bounded vacancy Dijkstra;
- stationary clearance restores one causal episode, proves a releasing graph
  cut, selects a safe pocket, and commits an internal maintenance order;
- deadlock evacuation separates swept-body geometry, activation policy,
  graph-escape planning, stale corridor-state release, and atomic commit;
- rolling result trimming separates the horizon boundary, corridor atomicity,
  resource-safe holding point, physical trajectory boundary, and array commit.

The independently deployed robot controller follows the same pattern.
`robot_planner.executor` uses `RouteControlParameters`, `RouteProgress`, and
`RouteSteeringState` to separate parameter normalization, path projection,
reservation gating, arrival checks, steering geometry, and velocity limits.
The normalized parameters are cached by the params mapping identity; hot reload
replaces that mapping and therefore invalidates the cache without polling or
locks in the 10 Hz control path.

The same rule applies to ROS runtime code. The canonical server runtime under
`fleet_manager/runtime/grpc/api` is split into lifecycle, control, maps, SLAM,
parameters and ROS helpers. The independently deployable
`robot/robot_driver/src/robot_grpc_api` package has the same local component
boundaries and intentionally does not import `fleet_manager`.

## Runtime ownership

Real and simulation managers each have a managed, non-daemon `RuntimeLoop`.
They continue polling, advancing routes and recovering traffic without a
browser or WebSocket client. HTTP and WebSocket reads return snapshots; they
do not advance time.

Application shutdown follows the reverse ownership order:

1. signal all loops;
2. join loop threads;
3. close planning workers;
4. close managers and transport clients.

## Data and generated files

- Canonical editable maps: `fleet_manager/map_data/maps_out`.
- Runtime workspaces: `var/operator_app` (ignored by Git).
- Benchmark samples: `var/fleet_sim_benchmarks` and `var/rds_benchmarks`
  (ignored by Git).
- Third-party ROS overlay: `robot/ros2_libs/{build,install,log}` (ignored).
- Common robot overlay: `robot/robot_driver/{build,install,log}` (ignored).
- Simulation overlay: `robot/simulation/{build,install,log}` (ignored).
- Independently deployed robot packages share durable map/parameter writes and
  PGM parsing through `robot_planner.route_core.atomic_storage` and
  `robot_planner.route_core.pgm`.
- The canonical protobuf and server-side gRPC client live under
  `fleet_manager/runtime/grpc/api`. Operator modules import those generated
  contracts directly. The independently deployable ROS robot package retains deliberate
  source copies guarded by parity tests.

Do not add runtime cache copies under `operator_app/operator_data`.

## Migration discipline

Core modules import their real package paths directly. Compatibility and
namespace-router packages are not used. Network payloads remain stable while
internals are split.
For an algorithm replacement:

1. keep the stable public entry method in its owning module;
2. move models and pure state-space logic behind it;
3. run deterministic differential scenarios against the committed version;
4. run safety and performance tests;
5. remove the old implementation only after equivalence is demonstrated.

Exact route equality is useful for deterministic regression, but the required
production invariants are collision freedom, reservation validity, liveness,
bounded planning time and stable external payloads.

SMAP conversion follows the same rule: parser and writer changes are checked
against byte-identical legacy artifacts as well as round-trip map loading.
