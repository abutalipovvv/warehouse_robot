# Architecture

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

1. `fleet_manager/math` — vectors, poses, polygons and half-open intervals.
2. `fleet_manager/search` — the generic search problem and deterministic A*.
3. `fleet_manager/core/mapf/traffic_graph.py` — stable graph facade;
   `traffic_graph_models.py`, `traffic_graph_builder.py`, and
   `traffic_graph_properties.py` contain the model, construction math, and
   legacy-map parsing.
4. `fleet_manager/core/mapf/reservations.py` — capacity-aware calendars.
5. `fleet_manager/core/mapf/sipp.py` — the public single-robot planner facade.
6. `fleet_manager/core/mapf/sipp_problem.py` — SIPP state transitions.
7. `fleet_manager/core/mapf/rolling_sipp.py` — multi-robot facade;
   `prioritized_planning.py` and `rolling_reservations.py` contain the policy
   and reservation writing.
8. `fleet_manager/core/mapf/lm_cbs.py` — compatibility facade over
   `cbs_models.py`, `cbs_conflicts.py`, `cbs_low_level.py`, and
   `cbs_high_level.py`. Request normalization lives in `cbs_setup.py`; the
   high-level constraint tree lives in `cbs_tree.py`.
9. `fleet_manager/core/mapf/fleet_planner.py` — stable public planner facade;
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
6. `fleet_manager/core/manager.py` — transport-neutral composition root;
   `manager_state.py`, `manager_snapshots.py`, `manager_commands.py`,
   `manager_robots.py`, `manager_remote.py`, and `manager_routes.py` own the
   individual capabilities.
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

The large compatibility surfaces are composition facades:

- `core/motion.py` composes step, kinematics, safety, retreat and replanning;
- `core/traffic/routing.py` composes spatial detours, traffic-zone admission,
  controlled-corridor admission/prefetch/passage and rolling-route helpers;
- `core/traffic/coordinator.py` composes wait detection, priority, leases,
  evacuation, escape installation and wait-cycle recovery;
- `core/tasks/dispatch.py` composes order admission, planning jobs, request
  construction, continuations, recovery and result commit;
- `operator_app/core/state.py` and `operator_app/core/fleet_manager.py` expose
  stable application APIs while delegating capabilities to focused services.

Existing hook names remain on these facade classes, while each implementation
file represents one reason to change.

The same rule applies to ROS runtime code. The canonical server runtime under
`fleet_manager/runtime/grpc/api` is split into lifecycle, control, maps, SLAM,
parameters and ROS helpers. The independently deployable
`sim_robot/ws/src/robot_grpc_api` package has the same local component
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
- Robot ROS build products: `sim_robot/ws/{build,install,log}` (ignored).
- Independently deployed robot packages share durable map/parameter writes and
  PGM parsing through `robot_planner.route_core.atomic_storage` and
  `robot_planner.route_core.pgm`.
- The canonical protobuf and server-side gRPC client live under
  `fleet_manager/runtime/grpc/api`. Operator modules are thin compatibility
  facades. The independently deployable ROS robot package retains deliberate
  source copies guarded by parity tests.

Do not add runtime cache copies under `operator_app/operator_data`.

## Compatibility during migration

Public imports and network payloads remain stable while internals are split.
For an algorithm replacement:

1. keep the old public facade;
2. move models and pure state-space logic behind it;
3. run deterministic differential scenarios against the committed version;
4. run safety and performance tests;
5. remove the old implementation only after equivalence is demonstrated.

Exact route equality is useful for deterministic regression, but the required
production invariants are collision freedom, reservation validity, liveness,
bounded planning time and stable external payloads.

SMAP conversion follows the same rule: parser and writer changes are checked
against byte-identical legacy artifacts as well as round-trip map loading.
