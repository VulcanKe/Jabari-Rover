#!/usr/bin/env bash
#
# jabari_client_start.sh - Start Manual Control Node on client
#
set -euo pipefail

WS=${WS:-$HOME/ros2_ws}

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/network_env_client.sh"

PARAMS_FILE=${1:-}
PARAM_ARGS=()
if [[ -n "$PARAMS_FILE" && -f "$PARAMS_FILE" ]]; then
  PARAM_ARGS=(--ros-args --params-file "$PARAMS_FILE")
fi

echo "[jabari_client_start] Launching manual_control_node"
ros2 run manual_control manual_control_node "${PARAM_ARGS[@]}"
echo "[jabari_client_start] Manual control node started successfully"