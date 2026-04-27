#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

class RearMotorDriver(Node):
    def __init__(self):
        super().__init__('rear_motor_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_speed_topic', '/motor/left_speed'),
                ('right_speed_topic', '/motor/right_speed'),
                ('en_a', 2), ('in1', 3), ('in2', 4),
                ('en_b', 2), ('in3', 3), ('in4', 4)  # example: en_b reused
            ]
        )

        self.en_a = self.get_parameter('en_a').value
        self.in1 = self.get_parameter('in1').value
        self.in2 = self.get_parameter('in2').value
        self.en_b = self.get_parameter('en_b').value
        self.in3 = self.get_parameter('in3').value
        self.in4 = self.get_parameter('in4').value

        self._validate_pins()

        GPIO.setup([self.en_a, self.in1, self.in2, self.en_b, self.in3, self.in4], GPIO.OUT)

        self.pwm_a = GPIO.PWM(self.en_a, 1000)
        self.pwm_a.start(0)

        if self.en_b != self.en_a:
            self.pwm_b = GPIO.PWM(self.en_b, 1000)
            self.pwm_b.start(0)
        else:
            self.pwm_b = self.pwm_a  # Reuse

        left_topic = self.get_parameter('left_speed_topic').value
        right_topic = self.get_parameter('right_speed_topic').value

        self.left_sub = self.create_subscription(Float32, left_topic, self.left_callback, 10)
        self.right_sub = self.create_subscription(Float32, right_topic, self.right_callback, 10)

    def _validate_pins(self):
        pins = [self.en_a, self.in1, self.in2, self.en_b, self.in3, self.in4]
        for pin in pins:
            if pin < 0:
                raise ValueError(f"Invalid GPIO pin value: {pin}")

    def left_callback(self, msg):
        speed = max(min(msg.data, 1.0), -1.0)
        self.pwm_a.ChangeDutyCycle(abs(speed) * 100)
        GPIO.output(self.in1, speed > 0)
        GPIO.output(self.in2, speed < 0)

    def right_callback(self, msg):
        speed = max(min(msg.data, 1.0), -1.0)
        self.pwm_b.ChangeDutyCycle(abs(speed) * 100)
        GPIO.output(self.in3, speed > 0)
        GPIO.output(self.in4, speed < 0)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    driver = None
    try:
        driver = RearMotorDriver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if driver:
            driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

