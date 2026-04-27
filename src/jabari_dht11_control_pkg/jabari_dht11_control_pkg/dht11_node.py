import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Adafruit_DHT

class DHT11Publisher(Node):
    def __init__(self):
        super().__init__('dht11_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.sensor = Adafruit_DHT.DHT11
        self.pin = 18  # BCM GPIO 4 (adjust if using a different pin)
        self.timer = self.create_timer(5.0, self.read_and_publish)  # Every 5 seconds
        self.get_logger().info("DHT11Publisher started. Reading from GPIO 4...")

    def read_and_publish(self):
        humidity, temperature = Adafruit_DHT.read_retry(self.sensor, self.pin)

        if humidity is not None and temperature is not None:
            msg = Float32()
            msg.data = float(temperature)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Temp: {temperature:.1f}°C, Humidity: {humidity:.1f}%")
        else:
            self.get_logger().warn("Failed to read from DHT11 sensor. No data published.")

def main(args=None):
    rclpy.init(args=args)
    node = DHT11Publisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

