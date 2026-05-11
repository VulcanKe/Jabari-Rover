import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import sys
import termios
import tty
import select

class KeyboardServoController(Node):
    def __init__(self):
        super().__init__('keyboard_servo_controller')
        self.publisher = self.create_publisher(Vector3, '/vector3_cmd', 10)
        self.get_logger().info("Use arrow keys to pan/tilt. Press 'q' to quit.")

        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.increment = 5.0

        self.run()

    def publish_vector(self):
        # Clamp FIRST, then assign to message
        self.pan_angle = max(0.0, min(180.0, self.pan_angle))
        self.tilt_angle = max(0.0, min(180.0, self.tilt_angle))

        msg = Vector3()
        msg.x = float(self.pan_angle)
        msg.y = float(self.tilt_angle)
        msg.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f"Sent Vector3: x={msg.x}, y={msg.y}")

    def get_key(self):
        # Non-blocking key read
        dr, dw, de = select.select([sys.stdin], [], [], 0.1)
        if dr:
            return sys.stdin.read(1)
        return None

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = self.get_key()

                if key is None:
                    continue

                if key == '\x1b':  # consume arrow key escape sequences without acting
                    self.get_key()  # '['
                    self.get_key()  # A/B/C/D — ignored
                # WASD as plain keypresses
                elif key == 'w':
                    self.tilt_angle -= self.increment
                    self.publish_vector()
                elif key == 's':
                    self.tilt_angle += self.increment
                    self.publish_vector()
                elif key == 'd':
                    self.pan_angle += self.increment
                    self.publish_vector()
                elif key == 'a':
                    self.pan_angle -= self.increment
                    self.publish_vector()
                elif key == 'q':
                    self.get_logger().info("Exiting...")
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = KeyboardServoController()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

