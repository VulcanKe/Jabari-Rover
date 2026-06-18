#!/usr/bin/env bash
#
# network_env_pi.sh - ROS 2 network environment for Raspberry Pi core stack
#tes

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
DISCOVERY_PORT=${DISCOVERY_PORT:-11811}

# Detect Pi IP if not provided
PI_IP=${PI_IP:-$(hostname -I | awk '{print $1}')}

export ROS_IP="$PI_IP"
export ROS_HOSTNAME="$PI_IP"
export ROS_DISCOVERY_SERVER="${PI_IP}:${DISCOVERY_PORT}"

echo "[network_env_pi] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[network_env_pi] ROS_IP=$ROS_IP"
echo "[network_env_pi] ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
