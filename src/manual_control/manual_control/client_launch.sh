#!/bin/bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 run manual_control manual_control_node.py &
ros2 run joy joy_node

