#!/usr/bin/env python3
"""
Mapless Mission Controller with Balloon Sequence (Pink → Yellow → Black → White → Blue)
Enhanced with move-and-scan behavior and startup yaw acknowledgment
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
import time
import math
import numpy as np


class MaplessMissionNode(Node):
    def __init__(self):  # ✅ FIXED constructor
        super().__init__('mapless_mission_node')

        # --- Parameters ---
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('w_max', 0.4)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('retreat_speed', -0.2)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('obstacle_threshold', 0.25)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('scan_speed', 0.15)
        self.declare_parameter('approach_distance', 1.0)
        self.declare_parameter('startup_yaw_duration', 4.0)

        # --- Balloon sequence ---
        self.balloon_sequence = ["pink", "yellow", "black", "white", "blue"]
        self.sequence_index = 0

        # --- Startup yaw acknowledgment ---
        self.startup_phase = True
        self.startup_start_time = time.time()
        self.initial_yaw = None
        self.target_yaw_cycles = 2

        # --- States ---
        self.target_detected = None
        self.visited_balloons = set()
        self.min_distance_ahead = None
        self.lidar_ranges = None
        self.in_stop_phase = False
        self.stop_start_time = None

        # --- Exploration ---
        self.exploration_state = "scanning"
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.investigation_start_time = None
        self.investigation_duration = 3.0
        self.last_rotation_time = time.time()
        self.rotation_interval = 5.0

        # --- IMU ---
        self.current_yaw = None
        self.target_yaw = None

        # --- Publishers and Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(String, '/object_info', self.detection_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # --- Timer ---
        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("✅ Mapless Mission Node (Move and Scan) started...")
        self.get_logger().info("🤖 Performing startup acknowledgment yaw motion...")

    def detection_cb(self, msg: String):
        if self.startup_phase:
            return
        data = msg.data.lower()
        current_target = self.get_current_target()
        if current_target and current_target in data:
            if data not in self.visited_balloons:
                self.target_detected = data
                self.target_yaw = self.current_yaw
                self.exploration_state = "approaching_balloon"
                self.get_logger().info(f"🎯 Target balloon detected: {data}")
            else:
                self.get_logger().info(f"⏭ Already visited: {data}")
        elif data and data != "none":
            self.get_logger().info(f"👀 Found {data}, but looking for {current_target}")
            if self.exploration_state == "investigating":
                self.investigation_start_time = time.time()

    def lidar_cb(self, msg: LaserScan):
        if not self.get_parameter('use_lidar').value:
            return
        self.lidar_ranges = msg.ranges
        center_index = len(msg.ranges) // 2
        d = msg.ranges[center_index]
        if math.isinf(d):
            d = 10.0
        self.min_distance_ahead = d
        if not self.startup_phase:
            self.detect_obstacles_for_investigation(msg)

    def detect_obstacles_for_investigation(self, msg: LaserScan):
        approach_distance = self.get_parameter('approach_distance').get_parameter_value().double_value
        if self.exploration_state != "scanning" or self.target_detected:
            return
        ranges = np.array(msg.ranges)
        center_idx = len(ranges) // 2
        sector_width = int(len(ranges) * 0.17)
        front_sector = ranges[center_idx - sector_width:center_idx + sector_width]
        valid_distances = front_sector[~np.isinf(front_sector)]
        if len(valid_distances) > 0:
            min_dist = np.min(valid_distances)
            if 0.1 < min_dist < approach_distance:
                self.obstacle_detected = True
                self.exploration_state = "approaching_obstacle"
                self.get_logger().info(f"🔍 Obstacle detected at {min_dist:.2f}m, investigating...")

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.initial_yaw is None and self.current_yaw is not None:
            self.initial_yaw = self.current_yaw

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def get_current_target(self):
        if self.sequence_index < len(self.balloon_sequence):
            return self.balloon_sequence[self.sequence_index]
        return None

    def perform_startup_yaw(self):
        if self.current_yaw is None or self.initial_yaw is None:
            return Twist()
        startup_duration = self.get_parameter('startup_yaw_duration').get_parameter_value().double_value
        w_max = self.get_parameter('w_max').get_parameter_value().double_value
        elapsed_time = time.time() - self.startup_start_time
        cmd = Twist()
        cycle_period = startup_duration / self.target_yaw_cycles
        yaw_amplitude = math.pi / 4
        if elapsed_time < startup_duration:
            phase = (elapsed_time % cycle_period) / cycle_period * 2 * math.pi
            target_yaw_offset = yaw_amplitude * math.sin(phase)
            target_yaw = self.initial_yaw + target_yaw_offset
            yaw_error = self.angle_diff(target_yaw, self.current_yaw)
            cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
            if int(elapsed_time * 2) % 2 == 0:
                self.get_logger().info(f"🤖 Startup yaw motion: {(elapsed_time / startup_duration) * 100:.0f}% complete")
        else:
            yaw_error = self.angle_diff(self.initial_yaw, self.current_yaw)
            if abs(yaw_error) > 0.1:
                cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
                self.get_logger().info("🎯 Returning to initial orientation...")
            else:
                self.startup_phase = False
                self.get_logger().info("✅ Startup acknowledgment complete! Starting mission...")
                self.get_logger().info(f"🎯 Looking for first balloon: {self.get_current_target()}")
                cmd = Twist()
        return cmd

    def control_loop(self):
        if self.startup_phase:
            cmd = self.perform_startup_yaw()
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()

        # --- Parameters ---
        v_max = self.get_parameter('v_max').get_parameter_value().double_value
        w_max = self.get_parameter('w_max').get_parameter_value().double_value
        safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        retreat_speed = self.get_parameter('retreat_speed').get_parameter_value().double_value
        stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        yaw_gain = self.get_parameter('yaw_gain').get_parameter_value().double_value
        scan_speed = self.get_parameter('scan_speed').get_parameter_value().double_value
        use_lidar = self.get_parameter('use_lidar').value

        current_target = self.get_current_target()
        if current_target is None:
            self.get_logger().info("🎉 Mission complete! All balloons visited.")
            self.cmd_pub.publish(Twist())
            return

        # --- Stop phase ---
        if self.in_stop_phase:
            now = time.time()
            if now - self.stop_start_time < stop_duration:
                self.get_logger().info("⏸ Holding at balloon...")
            elif now - self.stop_start_time < stop_duration + 1.0:
                self.get_logger().info("↩ Retreating...")
                cmd.linear.x = retreat_speed
            else:
                self.in_stop_phase = False
                self.target_detected = None
                self.target_yaw = None
                self.sequence_index += 1
                self.exploration_state = "scanning"
                next_target = self.get_current_target()
                if next_target:
                    self.get_logger().info(f"🔄 Searching for next balloon: {next_target}")
                else:
                    self.get_logger().info("🎉 All balloons collected!")
            self.cmd_pub.publish(cmd)
            return

        # --- Target balloon approach ---
        if self.target_detected and self.exploration_state == "approaching_balloon":
            distance = self.min_distance_ahead if (use_lidar and self.min_distance_ahead is not None) else 1e6
            self.get_logger().info(f"📏 Distance to {current_target} balloon: {distance:.2f} m")
            if use_lidar and distance <= safe_distance:
                self.get_logger().info(f"🎈 Reached {current_target} balloon at {distance:.2f}m")
                self.visited_balloons.add(self.target_detected)
                self.in_stop_phase = True
                self.stop_start_time = time.time()
            else:
                yaw_error = 0.0
                if self.current_yaw is not None and self.target_yaw is not None:
                    yaw_error = self.angle_diff(self.target_yaw, self.current_yaw)
                speed_factor = min(1.0, (distance - safe_distance + 0.2) / 0.5) if use_lidar else 1.0
                speed_factor = max(0.1, speed_factor)
                cmd.linear.x = v_max * speed_factor
                cmd.angular.z = max(min(yaw_gain * yaw_error, w_max), -w_max)
                self.get_logger().info(f"➡ Approaching {current_target} (dist={distance:.2f}m, speed={speed_factor:.2f})")

        # --- Exploration ---
        else:
            current_time = time.time()
            if self.exploration_state == "scanning":
                if use_lidar and self.min_distance_ahead is not None and self.min_distance_ahead <= safe_distance:
                    cmd.linear.x = 0.0
                    cmd.angular.z = w_max
                    self.get_logger().warn(f"⚠ Obstacle at {self.min_distance_ahead:.2f}m - turning")
                else:
                    cmd.linear.x = scan_speed
                    if current_time - self.last_rotation_time > self.rotation_interval:
                        self.last_rotation_time = current_time
                        self.rotation_interval = 3.0 + 4.0 * (time.time() % 1)
                    phase = (current_time - self.last_rotation_time) / self.rotation_interval
                    cmd.angular.z = 0.3 * math.sin(2 * math.pi * phase)
                    self.get_logger().info(f"🔍 Scanning for {current_target} balloon...")

            elif self.exploration_state == "approaching_obstacle":
                if use_lidar and self.min_distance_ahead is not None and self.min_distance_ahead <= safe_distance:
                    self.exploration_state = "investigating"
                    self.investigation_start_time = time.time()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info("🔎 Reached obstacle - start investigation...")
                else:
                    speed_factor = min(1.0, (self.min_distance_ahead - safe_distance) / 0.5)
                    speed_factor = max(0.1, speed_factor)
                    cmd.linear.x = scan_speed * speed_factor
                    cmd.angular.z = 0.0
                    self.get_logger().info(f"➡ Approaching obstacle: {self.min_distance_ahead:.2f}m")

            elif self.exploration_state == "investigating":
                if time.time() - self.investigation_start_time < self.investigation_duration:
                    cmd.angular.z = 0.4
                    self.get_logger().info("🔎 Investigating...")
                else:
                    self.exploration_state = "scanning"
                    self.obstacle_detected = False
                    self.last_rotation_time = time.time()
                    self.get_logger().info("📍 Investigation complete.")

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MaplessMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # ✅ FIXED
    main()

