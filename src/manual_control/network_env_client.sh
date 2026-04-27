#!/usr/bin/env bash
#
# network_env_client.sh - ROS 2 network environment for remote control client
#

PI_IP=${PI_IP:-192.168.1.50}   # <-- EDIT Pi IP

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
DISCOVERY_PORT=${DISCOVERY_PORT:-11811}

CLIENT_IP=$(hostname -I | awk '{print $1}')

export ROS_IP="$CLIENT_IP"
export ROS_HOSTNAME="$CLIENT_IP"
export ROS_DISCOVERY_SERVER="${PI_IP}:${DISCOVERY_PORT}"

echo "[network_env_client] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[network_env_client] ROS_IP=$ROS_IP"
echo "[network_env_client] ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
echo "[network_env_client] PI_IP=$PI_IP"
