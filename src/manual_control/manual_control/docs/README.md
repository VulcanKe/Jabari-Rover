# JABARI Manual Control Node

A comprehensive ROS2 Humble package for manual control of the JABARI Mars Rover. This node handles manual velocity commands from various input sources with built-in safety features, monitoring, and communication gateways.

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [API Reference](#api-reference)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## 🔍 Overview

The Manual Control Node serves as a critical component in the JABARI rover's control system, providing a safe and reliable interface for manual rover operation. It processes manual control inputs from web interfaces, joysticks, or direct ROS2 commands and publishes validated velocity commands to the rover's motion control system.

### Key Responsibilities
- **Command Processing**: Receive and validate manual velocity commands
- **Safety Management**: Implement velocity limits, emergency stops, and command timeouts
- **Communication Gateway**: Bridge between user interfaces and rover control systems
- **Status Monitoring**: Provide real-time status information and diagnostics
- **Multi-Source Input**: Handle commands from various input sources simultaneously

## ✨ Features

### 🛡️ Safety Features
- **Velocity Limiting**: Configurable maximum linear and angular velocity limits
- **Emergency Stop**: Immediate stop functionality with system-wide emergency stop support
- **Command Timeout**: Automatic rover stop if no commands received within timeout period
- **Deadband Filtering**: Filter out noise from analog input devices
- **Parameter Validation**: Runtime validation of all configuration parameters

### 🚀 Performance Features
- **Multi-threaded Execution**: Efficient command processing using ROS2 MultiThreadedExecutor
- **Thread-Safe Operations**: Safe concurrent access to command data
- **Configurable QoS**: Reliable message delivery with customizable Quality of Service
- **Real-time Status**: Live monitoring and diagnostics publishing

### 🔧 Integration Features
- **Communication Gateway**: Seamless integration with other rover nodes
- **Flexible Input Sources**: Support for web interfaces, joysticks, and direct topic commands
- **Launch File Support**: Easy deployment with parameterized launch configurations
- **Dynamic Reconfiguration**: Runtime parameter updates without node restart

## 📋 Requirements

### System Requirements
- **OS**: Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- **ROS2**: Humble Hawksbill distribution
- **Python**: 3.8 or higher
- **Hardware**: Raspberry Pi 4 (2GB+ RAM) or equivalent computing platform

### ROS2 Dependencies
```bash
# Core ROS2 packages
rclpy
std_msgs
geometry_msgs

# Build tools
ament_cmake
ament_cmake_python
```

### Python Dependencies
```bash
# Standard library modules (included with Python)
threading
json
time
typing

# No additional pip packages required for basic functionality
```

## 🚀 Installation

### Step 1: Create ROS2 Workspace
```bash
# Create workspace directory
mkdir -p ~/jabari_ws/src
cd ~/jabari_ws/src

# Clone or create the package
git clone <repository-url> jabari_manual_control
# or manually create the package structure
```

### Step 2: Package Structure Setup
Create the following directory structure:
```
jabari_manual_control/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   └── manual_control_params.yaml
├── launch/
│   └── manual_control.launch.py
├── jabari_manual_control/
│   ├── __init__.py
│   └── manual_control_node.py
├── scripts/
│   ├── manual_control_node.py
│   └── test_manual_control.py
├── resource/
│   └── jabari_manual_control
├── test/
│   └── test_manual_control.py
├── docs/
│   └── README.md
└── README.md
```

### Step 3: Copy Package Files
Copy the provided files to their respective locations:

1. **Main Node**: Copy `manual_control_node.py` to both `jabari_manual_control/` and `scripts/`
2. **Configuration**: Copy `package.xml`, `CMakeLists.txt`, `setup.py`
3. **Launch Files**: Copy launch files to `launch/`
4. **Config Files**: Copy YAML configuration to `config/`
5. **Tests**: Copy test files to `test/` and `scripts/`

### Step 4: Build the Package
```bash
# Navigate to workspace root
cd ~/jabari_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select jabari_manual_control

# Source the workspace
source install/setup.bash
```

### Step 5: Verify Installation
```bash
# Check if package is available
ros2 pkg list | grep jabari_manual_control

# Verify executable is available
ros2 run jabari_manual_control manual_control_node --help
```

## 🎮 Usage

### Basic Usage

#### 1. Launch the Node
```bash
# Basic launch
ros2 launch jabari_manual_control manual_control.launch.py

# Launch with custom parameters
ros2 launch jabari_manual_control manual_control.launch.py max_linear_velocity:=0.5 enable_debug:=true

# Launch for simulation
ros2 launch jabari_manual_control manual_control.launch.py use_sim_time:=