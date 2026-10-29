# fleetgraph-m3 — FleetGraph Module 3 (robot execution, perception, social navigation)

UDE NES bachelor project *FleetGraph*, WS 2026/27. This repo holds Module 3 — the software
on each robot — plus the **proposed shared contracts** for all three modules.

Status (week 3, 2026-10-29): week-3/4 milestone scope — task state machine, mock task
receiver, Gazebo goal navigation with Nav2, telemetry publisher — implemented and tested
(see *Verification*). Next: local scene graph + SceneObservation publisher (weeks 5–6).

## Contracts (read first if you are in Module 1 or 2)

`docs/interfaces.md` — proposed message formats, topics, QoS and the reasoning behind them
(VDA 5050, Open-RMF, ROS conventions, scene-graph practice). The `.msg` files in
`src/fleetgraph_interfaces/` are the source of truth; `docs/contracts.json` has the enum
lookup tables for JSON consumers (regenerate with `python3 tools/export_contracts.py`).
`mock_data/scenarios/*.yaml` are example task scripts.

## Quick start (headless host, Docker)

```bash
docker compose up -d --build            # image + container "fleetgraph-m3" (course base + Nav2)
git config core.hooksPath tools/hooks   # once per clone: pre-commit runs the fast tier
./t --build -k unit                     # colcon build + unit tests (seconds)
./t --fast                              # commit tier: unit + selftest + contract scenario (~25 s)
./t -k weekly                           # regression scenarios incl. Gazebo + Nav2 (~6 min)
./t -k demo                             # week-4 demo: full mock task script (~17 min)
```

Test tiers: `--fast` on every code commit (pre-commit hook; it also checks that generated
files are current and that the container mounts *this* checkout), `-k weekly` after Nav2 /
sim changes, `-k demo` before each course gate. Docs-only commits skip the hook.

Run things by hand (`tools/in_ros.sh` sources ROS + workspace in the container):

```bash
R="docker compose exec fleetgraph /home/ubuntu/tools/in_ros.sh"
$R ros2 launch fleetgraph_sim sim.launch.py          # Gazebo (headless) + robot + bridge
$R ros2 launch fleetgraph_sim nav.launch.py          # Nav2 + AMCL on the generated map
$R ros2 run fleetgraph_robot agent --ros-args -p use_sim_time:=true
$R ros2 run fleetgraph_robot mock_orchestrator --ros-args -p use_sim_time:=true \
     -p scenario_file:=/home/ubuntu/mock_data/scenarios/week4_demo.yaml
$R ros2 topic echo /fleet/task_update
```

With a display: `sim.launch.py headless:=false`, plus the course's `compose.wslg.yaml` /
`compose.vcxsrv.yaml` overrides for X forwarding.

## Layout

| Path | What |
|---|---|
| `src/fleetgraph_interfaces/` | shared contract messages (proposal 0.1.0) |
| `src/fleetgraph_robot/task_machine.py` | robot task state machine — pure Python, unit-tested |
| `src/fleetgraph_robot/agent_node.py` | ROS node: contract topics ↔ state machine ↔ Nav2, RobotState heartbeat, battery model |
| `src/fleetgraph_robot/mock_orchestrator.py` | mock Module 1: replays task scripts, M1-side lifecycle, resend-until-ack, JSONL event log + metrics |
| `src/fleetgraph_sim/` | world/map/locations (generated), robot URDF, gz bridge, Nav2 params, launch |
| `tools/gen_world.py` | single spec → Gazebo world + Nav2 map + location registry |
| `tests/` | pytest + YAML scenario harness (`./t`) |

## Robot behaviour

- One active task; a task is a list of Nav2 legs: GO_TO [target], DELIVERY [pickup, dropoff]
  with a 2 s dwell at each, PATROL waypoints × loops.
- Rejects before moving, with code + reason: unknown location (201), missing capability (202),
  low battery (203), unavailable/maintenance (204), malformed task (101), busy (200).
- Idempotent: repeated (task_id, rev) is acked, older rev dropped, newer rev restarts.
- A failed leg is retried once (counted as a replanning event); then FAILED (300) with
  `reassignment_requested`.
- Progress 0..1 is monotonic, computed from Nav2 `distance_remaining` per leg.
- RobotState at 2 Hz (pose from AMCL TF, never ground truth), RobotConnection ONLINE/OFFLINE.
- Maintenance: `ros2 param set /fleetgraph_agent maintenance true`.

## Simulation notes

- World "lab_floor" (10 × 6 m): hall with dock/storage, two labs behind 1 m doors, and an
  **unmapped crate** on the dock→lab_1 straight line (lidar-only obstacle). Edit
  `tools/gen_world.py`, never the generated files.
- Robot = the tutorial robot with: DiffDrive odom→base_footprint TF on /tf, **sim-only**
  kinematic wheel separation 0.22 (measured friction-patch effect, see URDF comment),
  ground-truth odometry `/ground_truth/odom` (tests only), camera throttled to 5 Hz, and the
  lidar on a 6 cm mast — flush on the body (tutorial) its scan plane cut the robot's own
  wheels (self-returns at 0.12 m), which made Nav2's collision monitor hold the robot.
- Nav2: Jazzy defaults with Regulated Pure Pursuit instead of MPPI (CPU), footprint/inflation
  for the 0.9 m door gaps. Every change is marked `FleetGraph:` in `config/nav2_params.yaml`.

## Verification (2026-10-29, headless, software rendering)

Evidence of every scenario run lands in `tests/artifacts/<name>.json` (gitignored).
Each row below passed on the final code except where noted: a last combined
`./t -k "not demo"` re-run was stopped by host memory pressure after selftest and
contract scenario passed (go_to had passed separately after the lidar fix). Gazebo + Nav2
runs are memory-heavy (not yet measured); stop other containers first on a loaded host.

| Suite | Result | Evidence |
|---|---|---|
| `./t -k unit` | 48 tests pass | task machine (accept/reject, legs, progress, retry, cancel, idempotency), assertions PASS+FAIL paths, msg ↔ Python mirrors |
| `selftest_expected_failure` | fails as required | harness detects failures |
| `weekly_contract_rejections` (no sim) | pass | codes 201/202/101/101 before any Nav2 goal, lifecycle RECEIVED→PARSED→QUEUED→ASSIGNED→REJECTED, `meta.seq` gap-free, 2 Hz heartbeat |
| `weekly_nav_go_to` (Gazebo+Nav2) | pass, 6.4 min wall | go_to lab_1 around the unmapped crate; ground truth within 0.35 m of goal |
| `demo_week4` (Gazebo+Nav2) | pass, 16.7 min wall | see below |

`demo_week4` task timeline (sim seconds; mock M1 summary: 6 tasks, avg wait 47.9 s,
avg completion 57.2 s):

| Task | Type | Outcome |
|---|---|---|
| task-0001 | go_to lab_1 | EXECUTING 19.3 → COMPLETED 38.5 |
| task-0002 | delivery storage→lab_2 (high priority, jumps the queue) | 38.5 → COMPLETED 81.9 |
| task-0003 | go_to "cafeteria" | REJECTED 201 UNKNOWN_LOCATION at 81.9 |
| task-0004 | go_to needing `arm` | REJECTED 202 MISSING_CAPABILITY at 82.0 |
| task-0005 | patrol hall, dock | 82.1 → COMPLETED 114.0 |
| task-0006 | go_to lab_1, cancelled 6 s in | 114.1 → CANCELLED 400 at 120.2 |

RobotState: 954 messages in 480 sim s = 2.0 Hz.

Harness lessons (both now encoded in `tests/recorder.py`): a best-effort depth-5 recorder
lost bursts of contract events, and a volatile subscriber missed events published before
discovery completed — record contract topics reliable, and transient_local where the
contract publishes transient_local. The same applies to M1/M2 subscribers.

## Next (weeks 5–7, course calendar)

- Local scene graph + `SceneObservation` publisher (obstacles/people from lidar + annotated
  sim detections), duplicate handling, confidence values.
- People / social-group models in the world, social costmap layer, wait/replan behaviour.
- Three robots (namespaced Nav2 + gz bridges), still one fleet topic per contract message.
- Agree / freeze the contracts with M1 and M2; add rosbridge to the image for the dashboard.
