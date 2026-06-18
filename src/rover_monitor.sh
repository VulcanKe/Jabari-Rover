#!/usr/bin/env bash
#
# rover_monitor.sh - Run introspection tools from a remote laptop targeting the Pi server
#

# --- CONFIGURATION ---
# Replace this with your Raspberry Pi's actual static IP address
ROBOT_PI_IP="127.0.0.1" 
DISCOVERY_PORT=11811
# ---------------------

COMMAND_TO_RUN="${*:-ros2 topic list --no-daemon}"

source /opt/ros/humble/setup.bash

# Create an on-the-fly Super Client XML schema config
TMP_DIR=$(mktemp -d)
XML_PATH="$TMP_DIR/super_client.xml"

cat << EOF > "$XML_PATH"
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://eprosima.com">
    <participant profile_name="super_client_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                    <discoveryServerList>
                        <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                            <locator>
                                <udpv4>
                                    <address>$ROBOT_PI_IP</address>
                                    <port>$DISCOVERY_PORT</port>
                                </udpv4>
                            </locator>
                        </RemoteServer>
                    </discoveryServerList>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="$XML_PATH"
export ROS_DISCOVERY_SERVER="$ROBOT_PI_IP:$DISCOVERY_PORT"

echo "[MONITOR] Connected to $ROS_DISCOVERY_SERVER. Running: $COMMAND_TO_RUN"
echo "------------------------------------------------------------------------"

eval "$COMMAND_TO_RUN"

# Wipe temporary configuration profile
rm -rf "$TMP_DIR"
