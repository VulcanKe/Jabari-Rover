#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from pathlib import Path
import os
import time

class YoloImageProcessor(Node):
    def __init__(self):
        super().__init__('yolo_image_processor')

        home_dir = str(Path.home())
        model_path = f'{home_dir}/ros2_ws/src/my_py_pkg/models/new_model.pt'

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file '{model_path}' does not exist")
            raise FileNotFoundError(model_path)

        self.declare_parameter('image_scale', 0.5)
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('save_images', False)
        self.declare_parameter('save_dir', '/tmp/yolo_images')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('log_interval', 1.0)

        self.image_scale = self.get_parameter('image_scale').get_parameter_value().double_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.save_images = self.get_parameter('save_images').get_parameter_value().bool_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value

        self.enable_display = False
        self.get_logger().info("Display disabled via hardcoded parameter.")

        os.makedirs(self.save_dir, exist_ok=True)

        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded model: {model_path}")

        qos = QoSProfile(
            depth=self.qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, qos)
        self.publisher = self.create_publisher(String, '/object_info', 10)
        self.image_pub = self.create_publisher(Image, '/image_annotated', qos)

        self.bridge = CvBridge()

        self.target_classes = [
            'red balloon', 'blue balloon', 'yellow balloon', 'pink balloon',
            'green balloon',
            'traffic cone', 'tennis ball', 'hammer'
        ]

        self.class_colors = {
            'red balloon': (0, 0, 255),
            'blue balloon': (255, 0, 0),
            'yellow balloon': (0, 255, 255),
            'pink balloon': (255, 105, 180),
            'green balloon': (0, 255, 0),
            'traffic cone': (0, 140, 255),
            'tennis ball': (0, 255, 0),
            'hammer': (128, 128, 128)
        }

        self.last_log_time = self.get_clock().now()
        self.frame_counter = 0
        self.last_annotated = None
        self.last_output = None

        self.create_timer(10.0, self.heartbeat)
        self.get_logger().info('YOLO Image Processor Node Started')

    def image_callback(self, msg):
        start_time = self.get_clock().now().nanoseconds

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_h, original_w = cv_image.shape[:2]
            if self.image_scale != 1.0:
                cv_image = cv2.resize(cv_image, (int(original_w * self.image_scale), int(original_h * self.image_scale)))
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        self.frame_counter += 1
        DETECTION_INTERVAL = 3

        if self.frame_counter % DETECTION_INTERVAL == 0:
            detections = []
            results = self.model(cv_image, conf=self.confidence_threshold, iou=self.iou_threshold)

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    if x2 <= x1 or y2 <= y1:
                        continue
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    confidence = float(box.conf[0])
                    if class_name in self.target_classes and confidence > self.confidence_threshold:
                        detections.append({
                            'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                            'class_name': class_name, 'confidence': confidence
                        })

            output = String()
            for det in detections:
                x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']
                class_name, confidence = det['class_name'], det['confidence']
                box_color = self.class_colors.get(class_name, (0, 255, 0))
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), box_color, 2)
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(cv_image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                output.data += f"Object: {class_name}, Confidence: {confidence:.2f}; "

            self.last_output = output
            self.last_annotated = cv_image.copy()

            if self.save_images:
                timestamp = int(time.time())
                cv2.imwrite(os.path.join(self.save_dir, f"yolo_frame_{timestamp}.jpg"), cv_image)

            current_time = self.get_clock().now()
            if (current_time - self.last_log_time).nanoseconds / 1e9 >= self.log_interval:
                self.get_logger().info(output.data or "No objects detected")
                self.last_log_time = current_time

        if self.last_output:
            self.publisher.publish(self.last_output)

        if self.last_annotated is not None:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(self.last_annotated, encoding='bgr8')
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")

        if self.enable_display:
            cv2.imshow("YOLO Detections", self.last_annotated)
            cv2.waitKey(1)

        elapsed_time = (self.get_clock().now().nanoseconds - start_time) / 1e6
        if (self.get_clock().now() - self.last_log_time).nanoseconds / 1e9 >= self.log_interval:
            self.get_logger().info(f"Inference time: {elapsed_time:.2f} ms")

    def heartbeat(self):
        self.get_logger().info("YOLO node is running... waiting for camera input.")

    def destroy_node(self):
        if self.enable_display:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Node crashed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
















