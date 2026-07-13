# ROS2 QoS Architecture & Implementation Guide

This document serves as a presentation-ready guide outlining how **ROS2 Quality of Service (QoS)** is configured and utilized across the Jabari-Rover workspace. It defines the core QoS concepts used in the system and documents their exact implementation in the codebase.

---

## 📊 Summary of QoS Profiles in Jabari-Rover

The table below summarizes the custom QoS configurations defined in the codebase and their respective target use cases:

| Package | QoS Profile | Reliability | Durability | History & Depth | Target Topics / Use Case |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`camera_publisher_pkg`** | Camera stream QoS | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` (1) | `/camera/image_raw/compressed`<br>`/camera/image/compressed` |
| | Detection feed QoS | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` (5) | `/image_annotated`<br>`/yolo/detections/compressed` |
| | Telemetry QoS | `RELIABLE` | `VOLATILE` | `KEEP_LAST` (10) | `/yolo/detections_log`, `/gps/fix`, `/cmd_vel` |
| **`manual_control`** | Servo command QoS | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` (1) | `/vector3_cmd` (Pan/Tilt servo commands) |
| | System control QoS | `RELIABLE` | `VOLATILE` | `KEEP_LAST` (10) | `/cmd_vel`, `/joy`, `/emergency_stop` |
| **`robot_localization`** | Sensor data QoS | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` (Custom / 1) | High-rate IMU, GPS, Odometry inputs |
| | State estimation QoS | `RELIABLE` | `VOLATILE` | `KEEP_LAST` (10) | Filtered Odometry/Acceleration outputs |
| **`rplidar_ros`** | Laser Scan subscriber | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` (Default) | `/scan` (LIDAR points) |
| | Laser Scan publisher | `RELIABLE` | `VOLATILE` | `KEEP_LAST` (10) | `/scan` (LIDAR points) |
| **`yolo_image_processor`** | YOLO processing QoS | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` (10 / Config) | `/camera/image_raw/compressed`, `/image_annotated` |

---

## 🔍 Core QoS Definitions & Design Rationale

### 1. Reliability Policy
Reliability defines how network packet loss is handled:
*   **`BEST_EFFORT`** (`ReliabilityPolicy.BEST_EFFORT` / `rclcpp::SensorDataQoS`)
    *   *Definition*: The publisher attempts to send messages but does not expect acknowledgements. If packets are lost over the network, they are not retransmitted.
    *   *Design Rationale*: Crucial for high-frequency or high-bandwidth streaming (like video frames and LIDAR point clouds). Dropped frames are ignored because a new frame will arrive immediately; waiting for retransmissions would cause lag and head-of-line blocking.
*   **`RELIABLE`** (`ReliabilityPolicy.RELIABLE`)
    *   *Definition*: The middleware guarantees delivery of every packet. Lost packets are retransmitted until successfully received.
    *   *Design Rationale*: Required for state-changing commands and safety messages (e.g., speed commands, emergency stops, GPS coordinates) where loss of information could lead to safety hazards or system errors.

### 2. Durability Policy
Durability determines if messages are preserved for late-joining subscribers:
*   **`TRANSIENT_LOCAL`** (`DurabilityPolicy.TRANSIENT_LOCAL`)
    *   *Definition*: The publisher retains the last sent messages. When a new subscription connects, the middleware automatically sends the retained message history to it immediately.
    *   *Design Rationale*: Ideal for infrequently updated state commands (e.g., servo angles, system settings). This ensures that if the driver node crashes and restarts, it immediately resumes with its last configured target position instead of remaining in an undefined state.
*   **`VOLATILE`** (`DurabilityPolicy.VOLATILE`)
    *   *Definition*: Messages are sent and immediately discarded. Late-joining subscribers only receive messages published *after* they connect.
    *   *Design Rationale*: Default behavior for continuous feeds where historical data is obsolete upon arrival.

### 3. History Policy & Depth
History controls queue sizes inside the ROS2 middleware:
*   **`KEEP_LAST`** (`HistoryPolicy.KEEP_LAST` with a numeric `depth`)
    *   *Definition*: Retains up to the specified `depth` limit of messages. Older messages are discarded when the queue fills.
    *   *Design Rationale*:
        *   **`depth = 1`**: Used for real-time video and state feedback. If processing falls behind, old data is dropped to ensure the processor only works on the latest state.
        *   **`depth = 10`**: Standard balance allowing small temporary processing lags to catch up without dropping commands or sensor telemetry.

---

## 🛠️ Detailed Implementation in Jabari-Rover

### 📹 `camera_publisher_pkg`
Manages camera frames and telemetry visualization.

*   **[camera_publisher.py](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/camera_publisher_pkg/camera_publisher_pkg/camera_publisher.py#L16-L28)**:
    ```python
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )
    self.publisher_ = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', qos_profile)
    ```
    *Rationale*: Video frames are sent over a Best-Effort policy with a queue depth of 1 to minimize latency.
*   **[web_streamer.py](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/camera_publisher_pkg/camera_publisher_pkg/web_streamer.py#L97-L115)**:
    *   Implements `camera_qos` (Best Effort, depth 1) for the camera stream.
    *   Implements `detection_qos` (Best Effort, depth 5) for object detection feedback.
    *   Implements `reliable_qos` (Reliable, depth 10) for critical topics such as control commands, motor speeds, and GPS coordinates to ensure they are accurately updated on the dashboard.

---

### 🎮 `manual_control`
Translates user input commands (Joystick/Keyboard) to vehicle movements and gimbal positions.

*   **[manual_control_node.py](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/manual_control/manual_control/manual_control_node.py#L161-L177)**:
    ```python
    reliable_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    servo_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
    )
    ```
    *Rationale*: Uses `TRANSIENT_LOCAL` durability for pan/tilt servo angles. If the downstream servo driver node restarts, it immediately retrieves the last command angle on `/vector3_cmd` and prevents physical drift or sudden jerks on boot.

---

### 🧭 `robot_localization`
Performs state estimation, fusing multiple sensor inputs (IMU, GPS, Odometry) via EKF/UKF.

*   **[navsat_transform.cpp](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/robot_localization/src/navsat_transform.cpp#L173-L190)** & **[ros_filter.cpp](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/robot_localization/src/ros_filter.cpp#L1179-L1185)**:
    *   Subscribes to sensor inputs using `rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size))` to ensure high-rate sensors do not accumulate lag or overload the network.
    *   Publishes state estimates (`odometry/filtered`, `accel/filtered`) using `rclcpp::QoS(10)` with reliable delivery, ensuring localization algorithms receive steady odometry updates.

---

### 📡 `rplidar_ros`
Handles the LIDAR scanner driver and client nodes.

*   **[rplidar_client.cpp](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/rplidar_ros/src/rplidar_client.cpp#L35)**:
    *   Subscribes to `/scan` with `rclcpp::SensorDataQoS()`.
*   **[rplidar_node.cpp](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/rplidar_ros/src/rplidar_node.cpp#L440)**:
    *   Publishes LaserScans using a standard `rclcpp::QoS(rclcpp::KeepLast(10))` reliability structure.

---

### 🧠 `yolo_image_processor`
Handles real-time object detection on the vehicle's camera feed.

*   **[object_detection.py](file:///c:/Users/HomePC/Desktop/Vulcan!/Jabari-Rover/src/yolo_image_processor/yolo_image_processor/object_detection.py#L51-L59)**:
    *   Creates a `BEST_EFFORT` QoS profile with `KEEP_LAST` and a configurable parameter depth (`self.qos_depth`, default 10).
    *   This is used to subscribe to `/camera/image_raw/compressed` and publish processed `/image_annotated` frames, matching the camera feed reliability to prevent queue backlogs during inference.
