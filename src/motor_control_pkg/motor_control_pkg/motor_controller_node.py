#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10
        )
        self.pub_left = self.create_publisher(Float32, '/motor/left_speed', 10)
        self.pub_right = self.create_publisher(Float32, '/motor/right_speed', 10)

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        if linear_vel == 0.0 and angular_vel != 0.0:
            # Pivot mode: full voltage spin
            left_speed = -1.0 if angular_vel > 0 else 1.0
            right_speed = 1.0 if angular_vel > 0 else -1.0
            self.get_logger().info("Pivot Mode Activated")
        else:
            # Normal mode (linear or curve motion)
            left_speed = right_speed = linear_vel
            left_speed = max(min(left_speed, 1.0), -1.0)
            right_speed = max(min(right_speed, 1.0), -1.0)

        # Publish to motor drivers
        self.pub_left.publish(Float32(data=left_speed))
        self.pub_right.publish(Float32(data=right_speed))

        self.get_logger().info(
            f"Cmd → linear.x: {linear_vel:.2f}, angular.z: {angular_vel:.2f} → "
            f"L: {left_speed:.2f}, R: {right_speed:.2f}"
        )


def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

