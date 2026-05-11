import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

class Vector3ToPWMNode(Node):
    def __init__(self):
        super().__init__('vector3_to_pwm_node')

        # Create I2C and PCA9685 instance
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # typical for servos

        # Subscribe to /vector3_cmd
        self.subscription = self.create_subscription(
            Vector3,
            '/vector3_cmd',
            self.vector3_callback,
            10
        )

        self.get_logger().info("Vector3 to PWM node started.")

    def vector3_callback(self, msg: Vector3):
        # Map each component (in degrees or unit range) to PWM
        pwm0 = self.angle_to_pwm(msg.x)
        pwm1 = self.angle_to_pwm(msg.y)
        # pwm2 = self.angle_to_pwm(msg.z)

        # Send to channels 2, 3
        self.pca.channels[2].duty_cycle = pwm0
        self.pca.channels[3].duty_cycle = pwm1
        

        self.get_logger().info(f"PWM Set - CH2: {pwm0}, CH3: {pwm1}")

    def angle_to_pwm(self, angle):
        """
        Maps an angle (0-180 degrees) to 16-bit PWM value for PCA9685.
        Adjust these based on your servo's range.
        """
        angle = max(0, min(180, angle))  # clamp
        pulse_us = 500 + (angle / 180.0) * 2000  # from 500us to 2500us
        duty_cycle = int((pulse_us * 65535) / (1000000 / self.pca.frequency))
        return duty_cycle


def main(args=None):
    rclpy.init(args=args)
    node = Vector3ToPWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pca.deinit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

