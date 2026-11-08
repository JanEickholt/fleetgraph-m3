# fleetgraph-m3: FleetGraph Module 3

Module 3 of the UDE NES bachelor project *FleetGraph*: the software on each robot (execution,
perception, social navigation), plus the **proposed shared contracts** for all three modules.

## Contracts (read first if you are in Module 1 or 2)

`docs/interfaces.md` describes the proposed message formats, topics, QoS and the reasoning
behind them (VDA 5050, Open-RMF, ROS conventions, scene-graph practice). The `.msg` files in
`src/fleetgraph_interfaces/` are the source of truth. `docs/contracts.json` has the enum
lookup tables for JSON consumers (regenerate with `python3 tools/export_contracts.py`).
`mock_data/scenarios/*.yaml` are example task scripts.

## Quick start (headless host, Docker)

```bash
docker compose up -d --build            # image + container "fleetgraph-m3" (course base + Nav2)
git config core.hooksPath tools/hooks   # once per clone: pre-commit runs the fast tier
./t --build -k unit                     # colcon build + unit tests
./t --fast                              # unit + selftest + contract scenario
./t -k weekly                           # regression scenarios incl. Gazebo + Nav2
./t -k demo                             # full mock task script
```

`--fast` runs in the pre-commit hook; run `-k weekly` after Nav2 or simulation changes.

Run things by hand (`tools/in_ros.sh` sources ROS + workspace in the container):

```bash
R="docker compose exec fleetgraph /home/ubuntu/tools/in_ros.sh"
$R ros2 launch fleetgraph_sim sim.launch.py robots:=3   # Gazebo (headless) + robot_1..3 + bridge
$R ros2 launch fleetgraph_sim nav.launch.py robots:=3   # Nav2 + AMCL per robot, same count
$R ros2 run fleetgraph_robot agent --ros-args -r __ns:=/robot_1 -p robot_id:=robot_1 \
     -p use_sim_time:=true                               # one agent per robot_N
$R ros2 run fleetgraph_robot mock_orchestrator --ros-args -p use_sim_time:=true \
     -p scenario_file:=/home/ubuntu/mock_data/scenarios/fleet_three_robots.yaml
$R ros2 topic echo /fleet/task_update
```

`robots` is 1..3 (default 1). A task script entry may name its `robot_id` (default: the
script's, else robot_1). Camera sensors: `sim.launch.py camera:=true`. With a display:
`sim.launch.py headless:=false`, plus the course's `compose.wslg.yaml` or
`compose.vcxsrv.yaml` overrides for X forwarding.

## Layout

| Path | What |
|---|---|
| `src/fleetgraph_interfaces/` | shared contract messages (proposal 0.1.0) |
| `src/fleetgraph_robot/task_machine.py` | robot task state machine, pure Python |
| `src/fleetgraph_robot/battery.py` | simulated battery (distance + idle drain), pure Python |
| `src/fleetgraph_robot/agent_node.py` | ROS node: contract topics, state machine, Nav2, RobotState heartbeat |
| `src/fleetgraph_robot/mock_orchestrator.py` | mock Module 1: replays task scripts, one queue per robot, resend-until-ack, JSONL event log + metrics |
| `src/fleetgraph_sim/` | world/map/locations (generated), robot URDF, gz bridge, Nav2 params, launch |
| `tools/gen_world.py` | single spec to Gazebo world + Nav2 map + location registry |
| `tests/` | pytest + YAML scenario harness (`./t`); results in `tests/artifacts/` (gitignored) |

## Robot behaviour

- One active task. A task is a list of Nav2 legs: GO_TO [target], DELIVERY [pickup, dropoff]
  with a 2 s dwell at each, PATROL waypoints × loops.
- Rejects before moving, with code + reason: unknown location (201), missing capability (202),
  low battery (203), unavailable/maintenance (204), malformed task (101), busy (200).
- Idempotent: repeated (task_id, rev) is acked, older rev dropped, newer rev restarts.
- A failed leg is retried once (counted as a replanning event), then FAILED (300) with
  `reassignment_requested`.
- Progress 0..1 is monotonic, computed from Nav2 `distance_remaining` per leg.
- RobotState at 2 Hz (pose from AMCL TF, never ground truth), RobotConnection ONLINE/OFFLINE.
- Maintenance: `ros2 param set /robot_1/fleetgraph_agent maintenance true`.

## Simulation notes

- World "lab_floor" (10 × 6 m): hall with docks/storage, two labs behind 1 m doors, and an
  **unmapped crate** on the dock to lab_1 straight line (lidar-only obstacle). Edit
  `tools/gen_world.py`, never the generated files.
- Robots: robot_N starts at dock, dock_2, dock_3 and lives in namespace `/robot_N` (scan,
  odom, cmd_vel, joint_states, Nav2, agent; TF on `/robot_N/tf` and `/robot_N/tf_static`
  with unprefixed frames). All share the map frame (= Gazebo world); `/clock` and the
  `/fleet/*` contract topics are global. Robots do not see each other: their bodies sit
  below the scan plane.
- Robot is the tutorial robot with: DiffDrive odom→base_footprint TF, **sim-only**
  kinematic wheel separation 0.22 (see URDF comment), ground-truth odometry
  `/ground_truth/robot_N/odom` (tests only), camera off by default (5 Hz when on), and the
  lidar on a 6 cm mast so its scan plane clears the robot's own wheels.
- Nav2: Jazzy defaults with Regulated Pure Pursuit instead of MPPI (CPU), footprint/inflation
  for the 0.9 m door gaps. Every change is marked `FleetGraph:` in `config/nav2_params.yaml`.
- Load, measured 2026-11-08 on jan-arch (Ryzen 7 9800X3D, 16 threads, 30 GB): 3 robots
  (`weekly_fleet_three_robots`) run at real-time factor 1.0 (the world's cap) with 1.22 GiB
  container memory and about 4 cores busy; 1 robot (`weekly_nav_go_to`): 1.0, 736 MiB.
  RTF is the evidence key `real_time_factor` in `tests/artifacts/<name>.json`, memory the
  peak of `docker stats --no-stream fleetgraph-m3` sampled during the run.
