#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Default speed settings
        self.speed = 0.5    # Linear speed
        self.turn = 0.5     # Angular speed
        self.pivot_mode = False  # Turning mode: False = normal, True = full pivot

        # Display instructions
        self.print_instructions()

        self.run_control_loop()

    def print_instructions(self):
        self.get_logger().info(
            'Keyboard Controls:\n'
            '  W/S: Move forward/backward\n'
            '  A/D: Turn left/right\n'
            '  U/J: Increase/decrease linear speed\n'
            '  I/K: Increase/decrease angular speed\n'
            '  M: Toggle turning mode (Pivot / Normal)\n'
            '  Q: Quit\n'
            f'Current speeds - Linear: {self.speed:.1f} m/s, Angular: {self.turn:.1f} rad/s\n'
            f'Turning Mode: {"Pivot (Full Voltage)" if self.pivot_mode else "Normal"}'
        )

    def run_control_loop(self):
        while True:
            key = self.get_key()
            twist = Twist()

            if key == 'w':
                twist.linear.x = self.speed
            elif key == 's':
                twist.linear.x = -self.speed
            elif key == 'a':
                twist.angular.z = 1.0 if self.pivot_mode else self.turn
            elif key == 'd':
                twist.angular.z = -1.0 if self.pivot_mode else -self.turn
            elif key == 'u':
                self.speed = min(1.0, self.speed + 0.1)
                self.get_logger().info(f'Linear speed: {self.speed:.1f} m/s')
            elif key == 'j':
                self.speed = max(0.0, self.speed - 0.1)
                self.get_logger().info(f'Linear speed: {self.speed:.1f} m/s')
            elif key == 'i':
                self.turn = min(1.0, self.turn + 0.1)
                self.get_logger().info(f'Angular speed: {self.turn:.1f} rad/s')
            elif key == 'k':
                self.turn = max(0.0, self.turn - 0.1)
                self.get_logger().info(f'Angular speed: {self.turn:.1f} rad/s')
            elif key == 'm':
                self.pivot_mode = not self.pivot_mode
                self.get_logger().info(f'Turning Mode: {"Pivot (Full Voltage)" if self.pivot_mode else "Normal"}')
            elif key == 'q':
                self.get_logger().info('Shutting down...')
                break
            else:
                # Optional: stop motors if unexpected key pressed
                pass

            self.pub.publish(twist)

    def get_key(self):
        """Get a single key press without requiring Enter"""
        tty.setraw(sys.stdin.fileno())
        try:
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                termios.tcgetattr(sys.stdin)
            )
        return key


def main():
    rclpy.init()
    try:
        keyboard_control = KeyboardControl()
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

