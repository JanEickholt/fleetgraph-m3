#!/usr/bin/env bash
# Run a command inside the container with ROS + the workspace sourced
# (non-interactive `docker compose exec` shells do not source them).
#   docker compose exec fleetgraph /home/ubuntu/tools/in_ros.sh ros2 topic list
#   docker compose exec -d -e IN_ROS_LOG=/tmp/sim.log fleetgraph \
#       /home/ubuntu/tools/in_ros.sh ros2 launch fleetgraph_sim sim.launch.py
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash 2>/dev/null || true
if [ -n "${IN_ROS_LOG:-}" ]; then
  exec "$@" > "$IN_ROS_LOG" 2>&1
fi
exec "$@"
