#!/usr/bin/env bash
set -eo pipefail

# Point to your root workspace directory (fixed from /src)
WS=${WS:-$HOME/ros2_ws/src}

# Source ROS 2 Humble base and local workspace
source /opt/ros/humble/setup.bash

if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
else
    echo "[ERROR] Workspace install setup not found at $WS/install/setup.bash"
    exit 1
fi

# Load network environment configurations
if [ -f "$WS/network_env_pi.sh" ]; then
    source "$WS/network_env_pi.sh"
elif [ -f "$(dirname "$0")/network_env_pi.sh" ]; then
    source "$(dirname "$0")/network_env_pi.sh"
else
    echo "[ERROR] network_env_pi.sh configuration file not found!"
    exit 1
fi

echo ""
echo "[jabari_pi_start] Workspace Path: $WS"
echo "[jabari_pi_start] Starting local FastDDS Discovery Server at $ROS_DISCOVERY_SERVER..."

# Spin up the background discovery server process on the assigned port
fastdds discovery -i 0 -p "${DISCOVERY_PORT:-11811}" > /dev/null 2>&1 &
SERVER_PID=$!

# Ensure the discovery server closes safely if this script is closed/killed
trap "kill $SERVER_PID 2>/dev/null" EXIT

# Give the server a moment to bind to the network socket
sleep 1.5

echo "[jabari_pi_start] Launching core nodes..."
ros2 launch jabari_rover_bringup jabari_pi_core.launch.py "$@"
