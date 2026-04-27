#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.serial_port = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

        self.publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(0.5, self.read_gps)

    def read_gps(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='replace')

            if line.startswith('$GPRMC'):
                msg_nmea = pynmea2.parse(line)

                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.latitude = msg_nmea.latitude
                gps_msg.longitude = msg_nmea.longitude
                gps_msg.altitude = 0.0  # RMC does not report altitude

                self.publisher.publish(gps_msg)
                self.get_logger().info(f"GPS: {gps_msg.latitude}, {gps_msg.longitude}")

        except pynmea2.ParseError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error reading GPS: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

