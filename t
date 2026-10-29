#!/usr/bin/env bash
# FleetGraph M3 test harness entry. Runs pytest inside the fleetgraph-m3
# container (tests/ and src/ are bind-mounted, see compose.yaml).
#   ./t                  everything, incl. the ~17 min demo
#   ./t --fast           commit tier: unit + selftest + no-sim contract scenario (~25 s),
#                        no process sweep (safe next to a manually running sim)
#   ./t -k unit          no-sim unit tests only (seconds)
#   ./t -k weekly        weekly regression scenarios (Gazebo + Nav2, ~6 min)
#   ./t -k demo          week-4 demo scenario (long: full task script)
#   ./t --build ...      colcon build first (fails if the build fails)
set -euo pipefail
cd "$(dirname "$0")"

FAST_TIER='unit or selftest or contract_rejections'

exec_ros() { docker compose exec -T fleetgraph bash -c "$1"; }

run_pytest() {
  docker compose exec -T fleetgraph bash -c '
    source /opt/ros/jazzy/setup.bash
    source /home/ubuntu/ws/install/setup.bash 2>/dev/null || true
    cd /home/ubuntu
    PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider tests "$@"' _ "$@"
}

if [ "${1:-}" = "--build" ]; then
  shift
  exec_ros 'set -o pipefail; source /opt/ros/jazzy/setup.bash && cd ~/ws &&
            colcon build --symlink-install 2>&1 | tail -5'
fi

if [ "${1:-}" = "--fast" ]; then
  shift
  # No sweep: these tests start no Gazebo, and the runner gives every
  # scenario its own random ROS_DOMAIN_ID and kills its process groups.
  run_pytest -q -k "$FAST_TIER" "$@"
  exit $?
fi

# Idempotency + teardown: kill leftover sim / Nav2 / robot processes. TERM
# first so `ros2 launch` reaps its children, then KILL the orphan classes
# (a SIGKILLed launch leaks parameter_bridge, which relays EVERY gz sim on
# the host into the ROS domain, and nav2 / agent nodes that keep acting).
# Then VERIFY zero: one leftover scrambles every later measurement.
cleanup_ros() {
  exec_ros '
    # every pattern has a [x] bracket so it never matches this shell itself
    pats="[g]z sim|[r]os2 launch|[r]os2 run|[p]arameter_bridge|/opt/ros/jazzy/lib/[n]av2_|[c]omponent_container|[f]leetgraph_robot/lib|/opt/ros/jazzy/lib/[r]obot_state_publisher"
    IFS="|"; for p in $pats; do pkill -f "$p" 2>/dev/null; done
    sleep 2
    for p in $pats; do pkill -9 -f "$p" 2>/dev/null; done
    unset IFS
    for i in $(seq 1 30); do
      n=$(pgrep -fc "[g]z sim|[p]arameter_bridge|[c]omponent_container|/opt/ros/jazzy/lib/[n]av2_|[f]leetgraph_robot/lib" || true)
      [ "$n" = "0" ] && break
      sleep 2
    done
    echo "leftover ROS processes: $n" >&2
    [ "$n" = "0" ]'
}

cleanup_ros || { echo "ABORT: leftover ROS processes will not die" >&2; exit 9; }

set +e
run_pytest "$@"
rc=$?
set -e

cleanup_ros || echo "WARNING: leftover ROS processes after run" >&2
exit $rc
