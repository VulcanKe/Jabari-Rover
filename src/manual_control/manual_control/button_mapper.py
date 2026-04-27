#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class ButtonMapper(Node):
    def __init__(self):
        super().__init__('button_mapper')
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.last_buttons = []
        self.last_axes = []
        
    def joy_callback(self, msg):
        # Check for button changes
        if self.last_buttons:
            for i, (current, last) in enumerate(zip(msg.buttons, self.last_buttons)):
                if current != last and current == 1:
                    self.get_logger().info(f'Button {i} pressed')
        
        # Check for significant axis changes
        if self.last_axes:
            for i, (current, last) in enumerate(zip(msg.axes, self.last_axes)):
                if abs(current - last) > 0.1:
                    self.get_logger().info(f'Axis {i}: {current:.2f}')
        
        self.last_buttons = msg.buttons[:]
        self.last_axes = msg.axes[:]

def main():
    rclpy.init()
    mapper = ButtonMapper()
    rclpy.spin(mapper)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
