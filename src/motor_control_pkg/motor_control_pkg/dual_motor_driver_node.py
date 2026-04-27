#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

class DualMotorDriver(Node):
    def __init__(self):
        super().__init__('dual_motor_driver')

        # Motor GPIO configuration
        self.en_a = 12  # Left motor enable (PWM)
        self.in1 = 17
        self.in2 = 27

        self.en_b = 13  # Right motor enable (PWM)
        self.in3 = 19
        self.in4 = 26

        GPIO.setup([self.en_a, self.in1, self.in2, self.en_b, self.in3, self.in4], GPIO.OUT)

        # Initialize PWM at 1kHz
        self.pwm_a = GPIO.PWM(self.en_a, 1000)
        self.pwm_b = GPIO.PWM(self.en_b, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        # Subscriptions
        self.create_subscription(Float32, '/motor/left_speed', self.left_callback, 10)
        self.create_subscription(Float32, '/motor/right_speed', self.right_callback, 10)

        self.get_logger().info("Dual Motor Driver Node Started")

    def left_callback(self, msg):
        speed = msg.data
        if speed == 0.0:
            self.pwm_a.ChangeDutyCycle(0)
            GPIO.output(self.in1, False)
            GPIO.output(self.in2, False)
        else:
            GPIO.output(self.in1, speed > 0)
            GPIO.output(self.in2, speed < 0)
            self.pwm_a.ChangeDutyCycle(100)  # Always full voltage

    def right_callback(self, msg):
        speed = msg.data
        if speed == 0.0:
            self.pwm_b.ChangeDutyCycle(0)
            GPIO.output(self.in3, False)
            GPIO.output(self.in4, False)
        else:
            GPIO.output(self.in3, speed > 0)
            GPIO.output(self.in4, speed < 0)
            self.pwm_b.ChangeDutyCycle(100)  # Always full voltage

    def destroy_node(self):
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

