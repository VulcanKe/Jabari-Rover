#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2

class MPU6050(Node):
    def __init__(self):
        super().__init__('imu_mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.bus = smbus2.SMBus(1)   # I2C bus
        self.address = 0x68          # MPU6050 default I2C address

        # Wake up MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)

        self.timer = self.create_timer(0.05, self.publish_imu)  # 20 Hz

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def publish_imu(self):
        imu_msg = Imu()

        # Read gyro
        gyro_x = self.read_word_2c(0x43) / 131.0
        gyro_y = self.read_word_2c(0x45) / 131.0
        gyro_z = self.read_word_2c(0x47) / 131.0

        # Read accel
        accel_x = self.read_word_2c(0x3B) / 16384.0
        accel_y = self.read_word_2c(0x3D) / 16384.0
        accel_z = self.read_word_2c(0x3F) / 16384.0

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

