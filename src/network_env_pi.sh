#!/usr/bin/env bash
#
# network_env_pi.sh - ROS 2 network environment for Raspberry Pi core stack
#

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
DISCOVERY_PORT=${DISCOVERY_PORT:-11811}

# Robust IP detection: ignores loopback (127.0.0.1) and isolates the main active IP
if [ -z "$PI_IP" ]; then
    PI_IP=$(hostname -I | awk '{for(i=1;i<=NF;i++) if($i !~ /^127\./) {print $i; break}}')
fi

# Fallback if no network interface is active
if [ -z "$PI_IP" ]; then
    echo "[WARNING] No active network IP detected. Defaulting to localhost."
    PI_IP="127.0.0.1"
fi

export ROS_IP="$PI_IP"
export ROS_HOSTNAME="$PI_IP"
export ROS_DISCOVERY_SERVER="${PI_IP}:${DISCOVERY_PORT}"

echo "[network_env_pi] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[network_env_pi] Detected Pi IP=$ROS_IP"
echo "[network_env_pi] ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"

