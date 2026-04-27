 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # ✅ BEST_EFFORT QoS for real-time speed
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher: Compressed images
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed',
            qos_profile
        )

        # Open default camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Camera not detected.")
        else:
            self.get_logger().info("Camera detected and running.")

        # Timer for frame publishing (~30 FPS)
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame.")
            return

        # Encode with JPEG (quality = 75 for speed/quality balance)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
