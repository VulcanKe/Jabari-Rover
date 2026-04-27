#!/usr/bin/env bash
#
# jabari_pi_start.sh - Start discovery server and Pi core nodes
#
set -eo pipefail

WS=${WS:-$HOME/ros2_ws}

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/src/network_env_pi.sh"

echo "[jabari_pi_start] Starting FastDDS discovery server..."
fastdds discovery --server-id 0 &
DISCOVERY_PID=$!

cleanup() {
  echo "[jabari_pi_start] Cleaning up..."
  kill $DISCOVERY_PID 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "[jabari_pi_start] Launching core nodes..."
ros2 launch jabari_rover_bringup jabari_pi_core.launch.py "$@"
