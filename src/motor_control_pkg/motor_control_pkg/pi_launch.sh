#!/bin/bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Start motor_controller controller
python3 motor_controller_node.py & 
python3 dual_motor_driver_node.py

