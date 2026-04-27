import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    # For development/testing on non-RPi machines
    import types
    GPIO = types.SimpleNamespace(
        BCM=None,
        OUT=None,
        HIGH=True,
        LOW=False,
        setmode=lambda mode: print("[MOCK GPIO] setmode called"),
        setup=lambda pin, mode: print(f"[MOCK GPIO] setup called on pin {pin}"),
        output=lambda pin, state: print(f"[MOCK GPIO] Pin {pin} set to {'HIGH' if state else 'LOW'}"),
        cleanup=lambda: print("[MOCK GPIO] Cleanup called"),
    )

FAN_RELAY_PIN = 17  # GPIO 17 BCM

class FanController(Node):
    def __init__(self):
        super().__init__('fan_controller')
        self.threshold = 30.0  # degrees Celsius
        self.fan_pin = FAN_RELAY_PIN

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.fan_pin, GPIO.OUT)
        GPIO.output(self.fan_pin, GPIO.HIGH)  # Start OFF (relay inactive)

        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )

        self.get_logger().info(f"FanController started. Relay on GPIO {self.fan_pin}, threshold = {self.threshold}°C")

    def temperature_callback(self, msg):
        temp = msg.data
        if temp >= self.threshold:
            GPIO.output(self.fan_pin, GPIO.LOW)  # LOW = relay active (fan ON)
            self.get_logger().info(f"Temp: {temp:.1f}°C → Fan ON")
        else:
            GPIO.output(self.fan_pin, GPIO.HIGH)  # HIGH = relay inactive (fan OFF)
            self.get_logger().info(f"Temp: {temp:.1f}°C → Fan OFF")

    def destroy_node(self):
        GPIO.output(self.fan_pin, GPIO.HIGH)
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FanController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


