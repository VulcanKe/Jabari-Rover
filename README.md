# Jabari Mars Rover 🚀

## Overview

The **Jabari Mars Rover** repository contains the full software stack for a ROS 2‑based autonomous rover designed for planetary exploration simulations.  It includes low‑level drivers (motors, IMU, LiDAR), perception pipelines (YOLOv8 image processing), manual tele‑operation, and a complete bring‑up infrastructure for Raspberry Pi hardware.

> **Note**: This project targets **ROS 2 Humble Hawksbill** and Ubuntu 22.04 (or the equivalent Windows ROS 2 installation).

---

## Key Features

- **Hardware Drivers**
  - Dual motor driver & motor controller (`motor_control_pkg`)
  - IMU driver for MPU‑6050 (`imu_mpu6050`)
  - RPLidar A1/A2 driver (`rplidar_ros`)
- **Perception**
  - Real‑time YOLOv8 object detection (`yolo_image_processor`)
  - Camera streaming & publishing (`camera_publisher_pkg`)
- **Control**
  - Manual joystick/keyboard control node (`manual_control`)
  - Pan‑tilt servo control (`jabari_pan_tilt_control_pkg`)
- **Navigation & SLAM**
  - Nav2 stack integration (`robot_bringup`)
  - SLAM Toolbox (`slam_toolbox`)
  - Robot localization (`robot_localization`)
- **Bring‑up & Launch**
  - Central Pi launch (`jabari_pi_start.sh` & `jabari_pi_core.launch.py`)
  - Distributed client launch (`jabari_client_start.sh`)
  - Network environment scripts for ROS 2 DDS (`network_env_pi.sh`, `network_env_client.sh`)

---

## Repository Structure

```
Jabari-Rover/
├─ src/                         # ROS 2 workspace root
│   ├─ camera_publisher_pkg/    # Camera driver & web streamer
│   ├─ gps_pkg/                 # GPS utilities (if used)
│   ├─ imu_mpu6050/             # MPU‑6050 driver
│   ├─ jabari_dht11_control_pkg/ # DHT11 temperature/humidity (optional)
│   ├─ jabari_pan_tilt_control_pkg/ # Servo & pan‑tilt control
│   ├─ jabari_rover_bringup/    # Core Pi launch files
│   ├─ manual_control/          # Tele‑op node and scripts
│   ├─ motor_control_pkg/        # Motor driver nodes
│   ├─ robot_bringup/           # Full robot bring‑up (Nav2, SLAM, etc.)
│   ├─ yolo_image_processor/    # YOLOv8 inference node
│   └─ … (additional packages)
├─ jabari_pi_start.sh           # Starts discovery server + core nodes on the Pi
├─ jabari_client_start.sh       # Starts manual‑control node on a client PC
├─ network_env_pi.sh            # Sets ROS_DOMAIN_ID, DDS discovery, IP env vars
├─ network_env_client.sh         # Mirrors the same for the client side
└─ README.md                    # **This file**
```

---

## Prerequisites

- **Operating System**: Ubuntu 22.04 (or Windows with ROS 2 Humble installed)
- **ROS 2**: `ros-humble-desktop` (install via apt)
- **Python 3.10+**
- **Colcon**: `sudo apt install python3-colcon-common-extensions`
- **Hardware** (for full functionality):
  - Raspberry Pi 4 (64‑bit) running Ubuntu 22.04
  - Dual motor driver board
  - MPU‑6050 IMU (I2C)
  - RPLidar A1/A2 (UART)
  - USB webcam (for YOLO image processing)
  - Optional: DHT11 sensor, GPS module

---

## Dependencies

All ROS 2 dependencies can be installed automatically via `rosdep` (see the Installation section), but they are listed here explicitly for reference.

### Core ROS 2 Packages
```bash
sudo apt install \
  ros-humble-rclpy \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-launch \
  ros-humble-launch-ros
```

### Navigation Stack (Nav2)
```bash
sudo apt install \
  ros-humble-nav2-msgs \
  ros-humble-nav2-bringup \
  ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager
```

### SLAM & Localization
```bash
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization
```

### Perception & Vision
```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv
```

### Hardware Drivers
```bash
sudo apt install \
  ros-humble-rplidar-ros \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-rviz2
```

### Python Libraries (pip)
```bash
pip install \
  ultralytics \   # YOLOv8
  smbus2 \        # MPU-6050 IMU (I2C)
  RPi.GPIO \      # GPIO control (Raspberry Pi only)
  pynmea2 \       # GPS NMEA parsing
  pyserial        # Serial communication (GPS, LiDAR)
```

> [!NOTE]
> `RPi.GPIO` is only required on Raspberry Pi hardware. It is conditionally declared in `motor_control_pkg` and will be skipped automatically on non-Pi builds.

---

## Installation & Build

```bash
# 1️⃣ Clone the repository
git clone https://github.com/your‑org/Jabari-Rover.git
cd Jabari-Rover

# 2️⃣ Initialize and update sub‑modules (if any)
# git submodule update --init --recursive   # ← uncomment if sub‑modules are used

# 3️⃣ Install ROS dependencies automatically
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4️⃣ Build the workspace
colcon build --symlink-install

# 5️⃣ Source the workspace (add to .bashrc for convenience)
source install/setup.bash
```

---

## Quick Start

### 1️⃣ Start the Pi Core Stack (on the rover)
```bash
# On the Raspberry Pi
./jabari_pi_start.sh
```
This script launches the DDS discovery server and the core launch file `jabari_pi_core.launch.py`, which brings up:
- Servo & pan‑tilt nodes
- Motor controller & driver
- YOLO image processor
- Camera publisher & web streamer
- Manual‑control node (ready for remote commands)

### 2️⃣ Connect a client PC for tele‑operation
```bash
# On your laptop/desktop
./jabari_client_start.sh
```
The client script sources the ROS network environment, then runs the `manual_control_node`.

### 3️⃣ Full Robot Bring‑up (navigation, SLAM, etc.)
```bash
# On the Pi (or a separate compute node)
ros2 launch robot_bringup rpi_drivers_launch.py
# Then launch Nav2 (example)
ros2 launch robot_bringup nav2.launch.py
```
Refer to the `robot_bringup/launch/` directory for the available launch files.

---

## Development & Usage Tips

- **Parameters**: Many nodes expose configurable parameters via YAML files in each package’s `config/` folder. Override them on launch with `--ros-args -p <param_name>:=<value>`.
- **Debugging**: Use `ros2 topic list`, `ros2 topic echo <topic>` and `rqt_graph` to inspect the node graph.
- **Testing**: Unit tests are located under each package’s `test/` folder. Run them with `colcon test`.
- **Static Transforms**: The launch files publish static transforms for the base link, laser frame, and IMU frame.

---

## Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/awesome‑feature`).
3. Ensure code follows the existing style (PEP‑8, black, flake8).
4. Add/Update unit tests.
5. Run `colcon test` and ensure all tests pass.
6. Open a Pull Request with a clear description of the changes.

---

## License

This project is licensed under the **MIT License** – see the [LICENSE](./LICENSE) file for details.

---

*Happy rover building!* 🚀
