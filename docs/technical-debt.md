# Technical debt

- **Belongs here:** decisions we took knowingly whose drawbacks are still unpaid (shortcuts, mocks,
  simplified models), each entry carrying the trigger that forces it to be paid.
- **Does not belong:** a description of current behaviour (`README.md`, code comments), a bug report
  (fix it, or schedule it in the current phase), or a wish. An entry without a trigger is a wishlist
  item, not debt, and gets dropped.
- **Maintained by deletion:** paid off means the entry is gone, not ticked; when a decision changes,
  its entry is rewritten or deleted in the same commit.

## Simulation fidelity

### Robot visibility — robots sit below each other's scan plane

- **Cost:** the lidar scans at 0.19 m and the tallest part of a robot is its wheel tops at 0.16 m,
  so robots never appear in each other's scans or costmaps. Nav2 cannot avoid another robot; two
  robots on crossing paths collide.
- **Why deferred:** a fix needs a taller collision body (URDF, Nav2 footprint, a new self-return
  check like the one of 2026-10-29) or robot poses fed into the costmaps; neither fit weeks 5-6.
- **Trigger:** the weeks 11-12 reassignment and recovery scenes that put two robots in one
  corridor, or the first robot-robot contact in any scenario.
- **Where:** `src/fleetgraph_sim/urdf/robot.urdf.xacro` (`lidar_joint` and the comment above it);
  obstacle layers in `src/fleetgraph_sim/config/nav2_params.yaml`.

### Detections — simulated from ground truth, camera off

- **Cost:** the detector model sees an entity when it is in range and field of view and no map cell
  or nearer entity blocks the line of sight to its centre; it then drops some and adds Gaussian
  noise. It never invents a detection, never confuses a class and knows no partial occlusion, so
  scene-graph and social-navigation results are optimistic. The camera sensor is off by default
  (`camera:=false`) because software rendering makes images costly.
- **Why deferred:** Module 3's work starts after detection (fusion, sharing, social navigation); a
  trained detector on rendered images costs CPU the headless container lacks and adds its own
  failures to every test.
- **Trigger:** a course gate or the final report requires detection from images, or the dashboard
  needs camera frames.
- **Where:** `src/fleetgraph_sim_tools/fleetgraph_sim_tools/detector_model.py`; `camera` argument
  in `src/fleetgraph_sim/launch/sim.launch.py`.

### Physics step — 1 ms steps make three-robot runs CPU-bound

- **Cost:** the world steps physics every 1 ms at a real-time-factor target of 1.0. Three robots
  with lidars on software rendering fall below real time once other sims share the host, which
  stretches weekly-tier runs.
- **Why deferred:** the kinematic wheel separation (0.22 m) was measured on 2026-10-26 at the 1 ms
  step; a coarser step changes wheel contact and needs that measurement repeated. The course
  reference repository casabot runs 4 ms steps.
- **Trigger:** a weekly scenario timing out because the sim fell behind (`real_time_factor` in the
  runner evidence), or a fourth robot.
- **Where:** `tools/gen_world.py:109` (`max_step_size`, generates `worlds/lab_floor.sdf`); casabot
  `worlds/generate_flat.py` (4 ms).

### Nav2 timeouts — upstream defaults under multi-robot load

- **Cost:** `bt_navigator` gives action servers 20 ms (`default_server_timeout`) and lifecycle bonds
  time out after 4 s. Three Nav2 stacks on one loaded host can miss either: a behaviour-tree node
  gives up on a slow server, or a lifecycle manager declares a node dead and takes that robot's
  stack down. casabot raises them to 200 ms and 10 s for this reason.
- **Why deferred:** the weekly fleet scenarios pass with the defaults so far; a change needs a
  weekly-tier run to confirm.
- **Trigger:** a server-timeout or broken-bond line in any run log, or the week-16 final demo
  rehearsal, whichever comes first.
- **Where:** `src/fleetgraph_sim/config/nav2_params.yaml:67`; `src/fleetgraph_sim/launch/nav.launch.py`
  (casabot sets `bond_timeout` with `SetParameter` in `launch/navigation.launch.py`).

### Test world — only the generated lab floor

- **Cost:** maps, docks, locations and scenes exist only for `lab_floor`. The course staff offered
  casabot (github.com/AdrianMelendez/casabot, MIT) as a common starting point for tests; its `flat`
  world (12 x 9 m, furnished, a corridor with five doors) fits the "group occupies part of a
  corridor" scene, but our stack cannot run in it.
- **Why deferred:** a second world needs its own occupancy map (casabot ships none and maps by
  exploration), docks, locations, and every scene placed again.
- **Trigger:** joint integration with the other groups, or the staff's evaluation, runs in a
  casabot world.
- **Where:** `tools/gen_world.py` (one spec generates world, map and locations); casabot
  `worlds/generate_flat.py`.

## Social navigation

### Social zones — no motion model

- **Cost:** a person is a disc of body plus `min_distance`, a group the convex hull of its members'
  discs plus `group_margin`, redrawn from each scene-graph update. The scene graph estimates each
  entity's velocity, but `social_zones` ignores it: the zone trails a walking person, and nothing
  keeps the robot out of the space the person walks into.
- **Why deferred:** the required scenes (a person blocking the route, a group in a corridor) work
  with zones that follow detections; motion-aware zones need velocity estimates good enough to
  trust.
- **Trigger:** a scene where the robot passes closer than `min_distance` to a walking person, or the
  week-14 social metrics failing on moving people.
- **Where:** `src/fleetgraph_robot/fleetgraph_robot/social.py` (`SocialParams`, `social_zones`);
  `velocity_smoothing` in `scene_graph.py`.

### Keepout mask — zones not grown by the robot radius

- **Cost:** `rasterize` marks the cells whose centres lie inside a zone. Nav2 applies costmap
  filters after the inflation layer, so keepout cells get no inflation: the planner lets the robot
  centre reach the mask edge, and its 0.25 m body enters the zone.
- **Why deferred:** the margin belongs where the mask is published for the KeepoutFilter, which is
  week-7 work; `path_conflicts` already takes `robot_radius`.
- **Trigger:** wiring the mask into Nav2's KeepoutFilter (week 7).
- **Where:** `src/fleetgraph_robot/fleetgraph_robot/social.py` (`rasterize`, `path_conflicts`).

## Scene graph

### Fusion thresholds — chosen, not calibrated

- **Cost:** association gates, time-to-live, merge radius and the confidence model in `GraphParams`
  and `FusionParams` were set by reasoning about the detector model's noise. Two people closer than
  the gate can merge into one entity; a long time-to-live keeps ghosts of people who left.
- **Why deferred:** calibration needs entity-level matching against ground truth over many runs.
- **Trigger:** a scene where two nearby people merge into one entity or a person who left stays in
  the graph, or the week 13-14 metrics showing either.
- **Where:** `src/fleetgraph_robot/fleetgraph_robot/scene_graph.py` (`GraphParams`),
  `central_graph.py` (`FusionParams`).

## Mocks standing in for other modules

### Module 1 and the central scene graph — mocks in this repository

- **Cost:** `mock_orchestrator` (text parsing, allocation, queues) and `mock_central_node` (fusion,
  versions, snapshots, resync) implement our reading of contracts 0.1.0. Timing, retries and
  ordering of the real Module 1 are guesses, and every place it reads the contracts differently
  shows up only in a joint run.
- **Why deferred:** the other groups build their modules in parallel; the mocks let Module 3 test
  end to end alone. Interfaces froze in week 3, and later changes require a documented
  compatibility decision.
- **Trigger:** the first joint run with the real Module 1 or Module 2, or any contract change after
  0.1.0.
- **Where:** `src/fleetgraph_robot/fleetgraph_robot/mock_orchestrator.py`, `mock_central_node.py`;
  `docs/contracts.json`.

### Operator text — keyword grammar, not language understanding

- **Cost:** one task per message, with verbs and locations from a fixed vocabulary. Anything outside
  it, or two tasks in one message, fails with ERROR_PARSE instead of a guess.
- **Why deferred:** parsing is Module 1's job; the mock only needs the demo phrasing, and a language
  model would make tests nondeterministic.
- **Trigger:** the real Module 1 takes over parsing (then delete the parser), or a demo script needs
  phrasing the grammar rejects.
- **Where:** `src/fleetgraph_robot/fleetgraph_robot/task_text.py`.

### Allocation — straight-line distance

- **Cost:** the allocator ranks robots by straight-line distance to the task's first location; a
  robot behind a wall can beat one with a shorter path.
- **Why deferred:** path length needs one planner request per candidate robot (Nav2
  `ComputePathToPose` in each namespace); allocation is mock Module 1 code the real module may
  replace.
- **Trigger:** a scene where the straight-line pick takes the clearly longer route, or allocation
  policies compared in the week 13-14 metrics.
- **Where:** `src/fleetgraph_robot/fleetgraph_robot/allocation.py`.

### Fleet membership — fixed at launch

- **Cost:** `robots:=1..3` spawns the fleet at startup (`MAX_ROBOTS = 3`, one dock per robot in the
  location registry). A robot cannot join a running simulation, while Module 2 must offer
  "add robot" and "remove robot".
- **Why deferred:** joining at runtime needs a Gazebo spawn, a namespaced Nav2 bringup and an agent
  started from a running system; the course minimum is three robots.
- **Trigger:** Module 2 integrates its add-robot action, or a scenario needs a fourth robot.
- **Where:** `src/fleetgraph_sim/launch/sim.launch.py:26`, `nav.launch.py:22`;
  `src/fleetgraph_sim/config/locations.yaml` (docks, generated by `tools/gen_world.py`).

## Interfaces and operations

### Module 2 link — unauthenticated websocket on every host interface

- **Cost:** `compose.yaml` publishes port 9090 on all interfaces, and rosbridge runs without TLS or
  authentication. The globs limit what a client reaches (`/fleet/*` topics and services, publishing
  only to `/mock_m1/task_text` and `/mock_m2/*`), not who connects: anyone on the network reads
  fleet state and sends operator text.
- **Why deferred:** the link serves development and demos on a trusted network; rosbridge
  authentication needs a token issuer and client support on the Module 2 side.
- **Trigger:** the dashboard connects across a network we do not control (university Wi-Fi, the
  internet), or the demo machine joins one.
- **Where:** `compose.yaml` (`ports`); `src/fleetgraph_robot/launch/m2_bridge.launch.py` (`GLOBS`).

### TF — one tree per robot, identical frame names

- **Cost:** every robot publishes `map → odom → base_footprint` on `/robot_N/tf` with the same frame
  names. One TF listener sees one robot; a fleet view in RViz or the dashboard needs per-robot
  remapping, and transforms between robots do not exist. Messages carry map-frame poses instead
  (robot state, observation sensor pose).
- **Why deferred:** it is the nav2_bringup multi-robot layout; prefixed frames mean rewriting every
  Nav2 frame parameter, the Gazebo bridge frame ids and the URDF.
- **Trigger:** Module 2 wants all robots in one TF view, or a node needs another robot's frames.
- **Where:** `src/fleetgraph_sim/launch/sim.launch.py` (bridge of `/robot_N/tf`), `nav.launch.py`.

## Tooling and testing

### Simulation tiers — manual, not on commit

- **Cost:** the pre-commit hook runs the generator checks and `./t --fast` (no simulation). A change
  that breaks launch files, Nav2 parameters or the sim integration passes the hook and shows up at
  the next manual `./t -k weekly`.
- **Why deferred:** the weekly tier needs a running sim and minutes per scenario, too long to block
  every commit.
- **Trigger:** a sim regression reaching `main` unnoticed, or a CI runner that can run Gazebo.
- **Where:** `tools/hooks/pre-commit`; `t`.

### Parallel simulations — slots configured outside the repository

- **Cost:** `compose.yaml` fixes the project and container name (one build volume for every
  worktree), so a second sim on the same host needs an untracked `.env` (project name, ROS domain,
  a compose override with its own container name and no published ports). Without it,
  `docker compose up -d` in another checkout re-creates the one container on that checkout's
  sources, and the first checkout's tests run foreign code until the pre-commit mount check stops a
  commit.
- **Why deferred:** the fixed name is deliberate (no rebuild per worktree); only parallel agent
  lanes need a second sim.
- **Trigger:** a second developer or a CI job runs sims next to a dev container on one host.
- **Where:** `compose.yaml` (`name`, `container_name`, `ports`); `tools/hooks/pre-commit` (mount
  check).
