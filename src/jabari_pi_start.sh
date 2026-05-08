#!/usr/bin/env bash
set -eo pipefail

WS=${WS:-$HOME/ros2_ws/src}

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/network_env_pi.sh"

echo ""

echo "$WS"

echo "[jabari_pi_start] Using existing FastDDS discovery server at $ROS_DISCOVERY_SERVER"

echo "[jabari_pi_start] Launching core nodes..."
ros2 launch jabari_rover_bringup jabari_pi_core.launch.py "$@"
