#!/usr/bin/env python3
"""
Comprehensive ROS2 Web Streamer with Enhanced Monitoring
Combines all features from both implementations (cleaned/refactored,
with full original dashboard HTML/CSS/JS restored).
"""
import json
import logging
import socket
import threading
import time
from collections import deque
from datetime import datetime
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, request, render_template_string
from flask_socketio import SocketIO, emit
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import CompressedImage, Image, NavSatFix
from std_msgs.msg import Float32, String
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class LogEntry:
    """Structured log entry for web display"""
    def __init__(self, timestamp: float, level: str, source: str, message: str, data: Optional[Dict] = None):
        self.timestamp = timestamp
        self.level = level
        self.source = source
        self.message = message
        self.data = data or {}

    def to_dict(self) -> Dict:
        return {
            'timestamp': self.timestamp,
            'time_str': datetime.fromtimestamp(self.timestamp).strftime('%H:%M:%S.%f')[:-3],
            'level': self.level,
            'source': self.source,
            'message': self.message,
            'data': self.data
        }


class ComprehensiveWebStreamer(Node):
    def __init__(self, socketio: SocketIO):
        super().__init__('comprehensive_web_streamer')
        self.bridge = CvBridge()
        self.socketio = socketio

        # Track start time for uptime monitoring
        self._start_time = time.time()

        # Log and status tracking
        self.log_buffer = deque(maxlen=5000)
        self.system_status = {
            'camera_frames': 0,
            'detection_frames': 0,
            'camera_status': 'waiting',
            'detection_status': 'waiting',
            'start_time': time.time(),
            'motor_left': 0.0,
            'motor_right': 0.0
        }

        # Frame storage and locks
        self.camera_frame = None
        self.detection_frame = None
        self.camera_lock = threading.Lock()
        self.detection_lock = threading.Lock()

        self.camera_frame_count = 0
        self.detection_frame_count = 0

        # Animal detection tracking
        self.recent_animals: Dict[str, float] = {}
        self.last_object_detection: Optional[str] = None
        self.valid_animals = ['horse', 'rooster', 'duck', 'cow']

        # Motor status
        self.motor_status = {'left_speed': 0.0, 'right_speed': 0.0}

        # GPS tracking
        self.gps_data = {
            'latitude': None,
            'longitude': None,
            'altitude': 0.0,
            'last_update': None,
            'status': 'waiting'
        }
        self.gps_history = deque(maxlen=100)  # Store last 100 GPS points

        # QoS profiles
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Camera subscription attempt (try common topics)
        try:
            self.camera_subscription = self.create_subscription(
                CompressedImage,
                '/camera/image_raw/compressed',
                self.camera_callback,
                camera_qos)
            self.log('INFO', 'system', 'Subscribed to /camera/image_raw/compressed')
        except Exception as e:
            try:
                self.camera_subscription = self.create_subscription(
                    CompressedImage,
                    '/camera/image/compressed',
                    self.camera_callback,
                    camera_qos)
                self.log('INFO', 'system', 'Subscribed to /camera/image/compressed')
            except Exception as e2:
                self.log('WARNING', 'system', f'Failed to subscribe to camera topics: {e} / {e2}')

        # Detection subscription attempt (try common topics)
        try:
            self.detection_subscription = self.create_subscription(
                CompressedImage,
                # '/yolo/annotated/compressed', 
                '/image_annotated',
                self.detection_callback,
                detection_qos)
            self.log('INFO', 'system', 'Subscribed to /yolo/annotated/compressed')
        except Exception as e:
            try:
                self.detection_subscription = self.create_subscription(
                    CompressedImage,
                    '/yolo/detections/compressed',
                    self.detection_callback,
                    detection_qos)
                self.log('INFO', 'system', 'Subscribed to /yolo/detections/compressed')
            except Exception as e2:
                self.log('WARNING', 'system', f'Failed to subscribe to detection topics: {e} / {e2}')

        # Object detection and other reliable subscriptions
        try:
            self.object_info_sub = self.create_subscription(
                String, '/yolo/detections_log', self.object_detection_log_callback, reliable_qos)
            self.object_detection_sub = self.create_subscription(
                Image, '/object_detection/detections', self.object_detection_callback, reliable_qos)
        except Exception as e:
            self.log('WARNING', 'system', f'Object detection subscriptions may be unavailable: {e}')

        # Control and status subscriptions
        try:
            self.status_sub = self.create_subscription(
                String, '/manual_control_status', self.status_callback, reliable_qos)
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, reliable_qos)
            self.vector3_sub = self.create_subscription(
                Vector3, '/vector3_cmd', self.vector3_callback, reliable_qos)
            self.left_motor_sub = self.create_subscription(
                Float32, '/motor/left_speed', self.left_motor_callback, reliable_qos)
            self.right_motor_sub = self.create_subscription(
                Float32, '/motor/right_speed', self.right_motor_callback, reliable_qos)
        except Exception as e:
            self.log('WARNING', 'system', f'Control/status subscriptions may be unavailable: {e}')

        # GPS subscription
        try:
            self.gps_sub = self.create_subscription(
                NavSatFix, '/gps/fix', self.gps_callback, reliable_qos)
            self.log('INFO', 'system', 'Subscribed to /gps/fix')
        except Exception as e:
            self.log('WARNING', 'system', f'GPS subscription may be unavailable: {e}')

        self.get_logger().info('Comprehensive web streamer with all features started.')
        self.setup_log_capture()

        # Start periodic status emission in background thread
        threading.Thread(target=self._periodic_status_update, daemon=True).start()

    def log(self, level: str, source: str, message: str, data: Optional[Dict] = None):
        """Enhanced logging method that appends to buffer and emits to web clients"""
        entry = LogEntry(time.time(), level, source, message, data)
        self.log_buffer.append(entry)

        # Emit log to web clients asynchronously
        try:
            self.socketio.start_background_task(self.socketio.emit, 'new_log', entry.to_dict())
        except Exception:
            # Avoid recursive logging if socketio is not ready
            pass

        # Also log to ROS2 logger
        ros_logger = self.get_logger()
        if level.upper() == 'ERROR':
            ros_logger.error(f'[{source}] {message}')
        elif level.upper() == 'WARNING':
            ros_logger.warning(f'[{source}] {message}')
        elif level.upper() == 'DEBUG':
            ros_logger.debug(f'[{source}] {message}')
        else:
            ros_logger.info(f'[{source}] {message}')

    def _periodic_status_update(self):
        """Send periodic status updates to the dashboard"""
        while rclpy.ok():
            try:
                status = {
                    'camera_status': self.system_status.get('camera_status', 'waiting'),
                    'detection_status': self.system_status.get('detection_status', 'waiting'),
                    'camera_frames': self.camera_frame_count,
                    'detection_frames': self.detection_frame_count,
                    'uptime': time.time() - self._start_time,
                    'motor_left': self.motor_status.get('left_speed', 0.0),
                    'motor_right': self.motor_status.get('right_speed', 0.0)
                }
                # Emit asynchronously
                self.socketio.start_background_task(self.socketio.emit, 'status_update', status)
                time.sleep(5)
            except Exception as e:
                # Use log() to capture issues
                self.log('WARNING', 'status_updater', f'Status update error: {e}')
                time.sleep(5)

    def setup_log_capture(self):
        """Setup a custom log handler to capture Python/ROS logs and forward to web UI"""
        class WebLogHandler(logging.Handler):
            def __init__(self, monitor):
                super().__init__()
                self.monitor = monitor

            def emit(self, record):
                try:
                    level = record.levelname
                    source = getattr(record, 'name', 'unknown')
                    message = record.getMessage()

                    log_entry = LogEntry(
                        timestamp=time.time(),
                        level=level,
                        source=source,
                        message=message
                    )
                    self.monitor.add_log_entry(log_entry)
                except Exception:
                    # Silently ignore to avoid recursion
                    pass

        handler = WebLogHandler(self)
        logging.getLogger().addHandler(handler)

    def add_log_entry(self, log_entry: LogEntry):
        """Add log entry to buffer and emit to web clients"""
        self.log_buffer.append(log_entry)
        try:
            self.socketio.start_background_task(self.socketio.emit, 'new_log', log_entry.to_dict())
        except Exception:
            pass

    # ---------- ROS Callbacks ----------
    def status_callback(self, msg: String):
        """Handle manual control status updates (expects JSON string)"""
        try:
            status_data = json.loads(msg.data)
            self.system_status.update(status_data)
            self.log('INFO', 'manual_control', 'Status update received', status_data)
            self.socketio.start_background_task(self.socketio.emit, 'status_update', status_data)
        except Exception as e:
            self.log('ERROR', 'manual_control', f'Error processing status: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """Handle movement commands"""
        try:
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                self.log('INFO', 'movement',
                         f'Movement: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}',
                         {'linear_x': msg.linear.x, 'angular_z': msg.angular.z})
        except Exception as e:
            self.log('DEBUG', 'movement', f'cmd_vel processing error: {e}')

    def vector3_callback(self, msg: Vector3):
        """Handle servo commands"""
        try:
            self.log('INFO', 'servos',
                     f'Servo: pan={msg.x:.1f}°, tilt={msg.y:.1f}°, third={msg.z:.1f}°',
                     {'pan': msg.x, 'tilt': msg.y, 'third': msg.z})
        except Exception as e:
            self.log('DEBUG', 'servos', f'vector3 processing error: {e}')

    def left_motor_callback(self, msg: Float32):
        """Handle left motor speed updates"""
        try:
            self.motor_status['left_speed'] = float(msg.data)
            self.system_status['motor_left'] = float(msg.data)
            if abs(msg.data) > 0.1:
                self.log('DEBUG', 'motor_controller', f'Left motor: {msg.data:.2f}', {'left_speed': msg.data})
        except Exception as e:
            self.log('DEBUG', 'motor_controller', f'Left motor callback error: {e}')

    def right_motor_callback(self, msg: Float32):
        """Handle right motor speed updates"""
        try:
            self.motor_status['right_speed'] = float(msg.data)
            self.system_status['motor_right'] = float(msg.data)
            if abs(msg.data) > 0.1:
                self.log('DEBUG', 'motor_controller', f'Right motor: {msg.data:.2f}', {'right_speed': msg.data})
        except Exception as e:
            self.log('DEBUG', 'motor_controller', f'Right motor callback error: {e}')

    def gps_callback(self, msg: NavSatFix):
        """Handle GPS location updates"""
        try:
            if msg.latitude != 0.0 and msg.longitude != 0.0:
                self.gps_data = {
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude,
                    'last_update': time.time(),
                    'status': 'active'
                }
                
                # Add to history
                self.gps_history.append({
                    'lat': msg.latitude,
                    'lon': msg.longitude,
                    'timestamp': time.time()
                })
                
                # Log every 10 GPS updates
                if len(self.gps_history) % 10 == 0:
                    self.log('INFO', 'gps', 
                             f'Location: {msg.latitude:.6f}, {msg.longitude:.6f}',
                             {'latitude': msg.latitude, 'longitude': msg.longitude, 'altitude': msg.altitude})
                
                # Emit GPS update to web clients
                self.socketio.start_background_task(self.socketio.emit, 'gps_update', {
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude,
                    'timestamp': time.time()
                })
        except Exception as e:
            self.log('ERROR', 'gps', f'Error processing GPS data: {e}')

    def object_detection_log_callback(self, msg: String):
        """Handle generic object detection results"""
        try:
            if msg.data and msg.data != self.last_object_detection:
                detection_count = msg.data.count('Object:')
                if detection_count > 0:
                    self.log('INFO', 'object_detection',
                             f'Detected {detection_count} objects',
                             {'detection_count': detection_count, 'full_detection': msg.data[:200]})
                self.last_object_detection = msg.data
        except Exception as e:
            self.log('DEBUG', 'object_detection', f'object_detection_log error: {e}')

    def object_detection_callback(self, msg: Image):
        """Handle animal detection with stability filtering"""
        try:
            # If message format is a string inside Image, handle accordingly; otherwise this callback can be adapted
            # Here we assume msg.data is bytes; convert to a brief text content if possible
            if not hasattr(msg, 'data') or not msg.data:
                return

            text_sample = None
            try:
                # Attempt to decode ascii subset for logs embedded in Image (best-effort)
                text_sample = msg.data[:512].decode('utf-8', errors='ignore').lower()
            except Exception:
                text_sample = None

            if text_sample is None:
                return

            if text_sample == self.last_object_detection:
                return

            self.last_object_detection = text_sample
            detected_animals = [a for a in self.valid_animals if a in text_sample]

            if not detected_animals:
                return

            now = time.time()
            for animal in detected_animals:
                self.recent_animals[animal] = now

            # Clean up old detections (older than 3 seconds)
            self.recent_animals = {a: t for a, t in self.recent_animals.items() if now - t < 3}

            stable_animals = list(self.recent_animals.keys())
            if not stable_animals:
                return

            detection_summary = ", ".join(stable_animals).title()
            self.log('INFO', 'animal_detection', f'Stable detection: {detection_summary}',
                     {'animals': stable_animals})

            self.system_status['last_detection'] = {
                'animals': stable_animals,
                'timestamp': time.time(),
                'message': detection_summary,
            }

            # Emit animal update
            try:
                self.socketio.start_background_task(self.socketio.emit, 'animal_update', {
                    'animals': stable_animals,
                    'message': detection_summary
                })
            except Exception:
                pass

        except Exception as e:
            self.log('ERROR', 'animal_detection', f'Error in object_detection_callback: {e}')

    def camera_callback(self, msg: CompressedImage):
        """Callback for camera compressed images"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.log('WARNING', 'camera', 'Failed to decode compressed image')
                return

            with self.camera_lock:
                self.camera_frame = cv_image
                self.camera_frame_count += 1
                self.system_status['camera_status'] = 'active'

            # Log every 30 frames
            if self.camera_frame_count % 30 == 0:
                height, width = cv_image.shape[:2]
                self.log('DEBUG', 'camera',
                         f'Frame #{self.camera_frame_count} - {width}x{height}',
                         {'frame_count': self.camera_frame_count, 'width': width, 'height': height})

        except Exception as e:
            self.log('ERROR', 'camera', f'Error processing camera image: {e}')

    def detection_callback(self, msg: CompressedImage):
        """Callback for detection compressed images"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.log('WARNING', 'detection', 'Failed to decode detection image')
                return

            with self.detection_lock:
                self.detection_frame = cv_image
                self.detection_frame_count += 1
                self.system_status['detection_status'] = 'active'

            # Log every 10 frames
            if self.detection_frame_count % 10 == 0:
                height, width = cv_image.shape[:2]
                self.log('DEBUG', 'yolo_processor',
                         f'Detection frame #{self.detection_frame_count} - {width}x{height}')

        except Exception as e:
            self.log('ERROR', 'detection', f'Error processing detection image: {e}')

    # ---------- Frame generators ----------
    def generate_camera_frames(self):
        """Generate frames for camera streaming (multipart JPEG)"""
        last_frame_count = 0
        while True:
            frame_to_send = None
            current_frame_count = 0

            with self.camera_lock:
                if self.camera_frame is not None and self.camera_frame_count > last_frame_count:
                    frame_to_send = self.camera_frame.copy()
                    current_frame_count = self.camera_frame_count

            if frame_to_send is not None:
                # Resize for web streaming
                height, width = frame_to_send.shape[:2]
                if width > 800:
                    new_width = 800
                    new_height = int(height * (800 / width))
                    frame_to_send = cv2.resize(frame_to_send, (new_width, new_height))

                ret, buffer = cv2.imencode('.jpg', frame_to_send, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    last_frame_count = current_frame_count
            else:
                placeholder = self.create_placeholder('CAMERA', 'Waiting for camera feed...')
                ret, buffer = cv2.imencode('.jpg', placeholder)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

            time.sleep(0.05)

    def generate_detection_frames(self):
        """Generate frames for detection streaming (multipart JPEG)"""
        last_frame_count = 0
        last_valid_frame = None

        while True:
            frame_to_send = None
            current_frame_count = 0

            with self.detection_lock:
                if self.detection_frame is not None:
                    if self.detection_frame_count > last_frame_count:
                        frame_to_send = self.detection_frame.copy()
                        current_frame_count = self.detection_frame_count
                        last_frame_count = current_frame_count
                        last_valid_frame = frame_to_send.copy()
                    elif last_valid_frame is not None:
                        frame_to_send = last_valid_frame.copy()

            if frame_to_send is not None:
                height, width = frame_to_send.shape[:2]
                if width > 800:
                    new_width = 800
                    new_height = int(height * (800 / width))
                    frame_to_send = cv2.resize(frame_to_send, (new_width, new_height))

                ret, buffer = cv2.imencode('.jpg', frame_to_send, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                placeholder = self.create_placeholder('AI DETECTION', 'Waiting for AI detection...')
                ret, buffer = cv2.imencode('.jpg', placeholder)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

            time.sleep(0.1)

    def generate_combined_frames(self):
        """Generate frames combining both camera and detection feeds"""
        last_camera_count = 0
        last_detection_count = 0
        last_valid_detection = None

        while True:
            camera_frame = None
            detection_frame = None

            with self.camera_lock:
                if self.camera_frame is not None and self.camera_frame_count > last_camera_count:
                    camera_frame = self.camera_frame.copy()
                    last_camera_count = self.camera_frame_count
                elif self.camera_frame is not None:
                    camera_frame = self.camera_frame.copy()

            with self.detection_lock:
                if self.detection_frame is not None:
                    if self.detection_frame_count > last_detection_count:
                        detection_frame = self.detection_frame.copy()
                        last_detection_count = self.detection_frame_count
                        last_valid_detection = detection_frame.copy()
                    elif last_valid_detection is not None:
                        detection_frame = last_valid_detection.copy()

            if camera_frame is not None:
                combined_frame = self.create_combined_frame(camera_frame, detection_frame)
            else:
                combined_frame = self.create_combined_placeholder()

            ret, buffer = cv2.imencode('.jpg', combined_frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ret:
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

            time.sleep(0.1)

    # ---------- Frame utilities ----------
    def create_combined_frame(self, camera_frame, detection_frame=None):
        """Create a combined frame with camera and detection side by side"""
        camera_resized = cv2.resize(camera_frame, (640, 480))

        if detection_frame is not None:
            detection_resized = cv2.resize(detection_frame, (640, 480))
        else:
            detection_resized = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(detection_resized, 'Waiting for AI...', (200, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        combined = np.hstack((camera_resized, detection_resized))

        # Add labels and separator
        cv2.putText(combined, 'CAMERA FEED', (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(combined, 'AI DETECTION', (660, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
        cv2.line(combined, (640, 0), (640, 480), (100, 100, 100), 2)

        return combined

    def create_combined_placeholder(self):
        """Create placeholder for combined feed"""
        frame = np.zeros((480, 1280, 3), dtype=np.uint8)
        cv2.putText(frame, 'Waiting for camera...', (480, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.line(frame, (640, 0), (640, 480), (100, 100, 100), 2)
        return frame

    def create_placeholder(self, title, message):
        """Create generic placeholder frame"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, title, (200, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(frame, message, (150, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        return frame

    def get_recent_logs(self, count: int = 100) -> List[Dict]:
        """Get recent log entries as dicts"""
        return [entry.to_dict() for entry in list(self.log_buffer)[-count:]]

    # ---------- Flask route methods ----------
    def camera_feed(self):
        """Camera streaming route"""
        response = Response(self.generate_camera_frames(),
                           mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response

    def detection_feed(self):
        """Detection streaming route"""
        response = Response(self.generate_detection_frames(),
                           mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response

    def combined_feed(self):
        """Combined streaming route"""
        response = Response(self.generate_combined_frames(),
                           mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response

    def status(self):
        """Enhanced status endpoint (returns JSON)"""
        with self.camera_lock:
            has_camera = self.camera_frame is not None
            camera_count = self.camera_frame_count

        with self.detection_lock:
            has_detection = self.detection_frame is not None
            detection_count = self.detection_frame_count

        uptime = int(time.time() - self._start_time)

        response_data = {
            'camera_status': 'active' if has_camera else 'waiting',
            'detection_status': 'active' if has_detection else 'waiting',
            'camera_frames': camera_count,
            'detection_frames': detection_count,
            'uptime': uptime,
            'log_count': len(self.log_buffer),
            'motor_left': self.motor_status.get('left_speed', 0.0),
            'motor_right': self.motor_status.get('right_speed', 0.0),
            'last_detection': self.system_status.get('last_detection'),
            'gps': self.gps_data,
            'message': f'Camera: {"✓" if has_camera else "⏳"} | Detection: {"✓" if has_detection else "⏳"}',
            'timestamp': time.time()
        }

        response = jsonify(response_data)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response

    def health(self):
        """Health check endpoint"""
        uptime = time.time() - self._start_time

        health_data = {
            'status': 'healthy',
            'timestamp': time.time(),
            'uptime': uptime,
            'camera_active': self.camera_frame is not None,
            'detection_active': self.detection_frame is not None,
            'ros_node_active': rclpy.ok(),
            'log_buffer_size': len(self.log_buffer),
            'system_status': {
                'camera_frames': self.camera_frame_count,
                'detection_frames': self.detection_frame_count
            }
        }

        if not rclpy.ok():
            health_data['status'] = 'unhealthy'
            status_code = 503
        elif not self.camera_frame:
            health_data['status'] = 'degraded'
            status_code = 200
        else:
            status_code = 200

        response = jsonify(health_data)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        return response, status_code

    def get_logs(self):
        """API endpoint to get recent logs with optional filtering"""
        try:
            count = request.args.get('count', 100, type=int)
            level_filter = request.args.get('level', '')
            source_filter = request.args.get('source', '')

            count = min(count, 5000)
            logs = self.get_recent_logs(count)

            if level_filter:
                logs = [log for log in logs if log['level'].upper() == level_filter.upper()]
            if source_filter:
                logs = [log for log in logs if source_filter.lower() in log['source'].lower()]

            response = jsonify({
                'logs': logs,
                'total': len(logs),
                'filtered': bool(level_filter or source_filter)
            })

            response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
            response.headers['Access-Control-Allow-Origin'] = '*'
            return response

        except Exception as e:
            self.log('ERROR', 'api', f'Error in get_logs endpoint: {e}')
            return jsonify({'error': str(e), 'logs': []}), 500

    def get_status(self):
        """API endpoint to get system status"""
        response = jsonify(self.system_status)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response

    def get_gps_history(self):
        """API endpoint to get GPS location history"""
        response = jsonify({
            'current': self.gps_data,
            'history': list(self.gps_history),
            'count': len(self.gps_history)
        })
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response

    def index(self):
        """Comprehensive dashboard HTML served inline (full original content restored)"""
        return render_template_string('''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Jabari Rover - Comprehensive Control Center</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        :root {
            --primary-bg: #0a0f18;
            --secondary-bg: #1a1f2e;
            --card-bg: #252b3a;
            --accent-blue: #00d4ff;
            --accent-orange: #ff6b35;
            --accent-green: #00ff88;
            --accent-purple: #a855f7;
            --text-primary: #ffffff;
            --text-secondary: #b8bcc8;
            --border-color: #3d4758;
        }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Inter', sans-serif;
            background: linear-gradient(135deg, var(--primary-bg) 0%, var(--secondary-bg) 100%);
            color: var(--text-primary);
            min-height: 100vh;
        }
        .header {
            background: rgba(37, 43, 58, 0.95);
            backdrop-filter: blur(20px);
            border-bottom: 2px solid var(--border-color);
            padding: 1.2rem 0;
            position: sticky;
            top: 0;
            z-index: 100;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5);
        }
        .header-content {
            max-width: 1800px;
            margin: 0 auto;
            padding: 0 2rem;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }
        h1 { 
            font-size: 1.8rem;
            font-weight: 700;
            background: linear-gradient(135deg, var(--accent-blue), var(--accent-green));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }
        .connection-badge {
            padding: 0.4rem 1rem;
            border-radius: 20px;
            font-size: 0.85rem;
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: 0.5rem;
            background: rgba(0, 255, 136, 0.1);
            border: 1px solid var(--accent-green);
            color: var(--accent-green);
        }
        .connection-badge.disconnected {
            background: rgba(255, 107, 53, 0.1);
            border-color: var(--accent-orange);
            color: var(--accent-orange);
        }
        .container { max-width: 1800px; margin: 0 auto; padding: 2rem; }
        
        /* Status Cards */
        .status-section {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 1.5rem;
            margin-bottom: 2rem;
        }
        .status-card {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 16px;
            padding: 1.5rem;
            transition: all 0.3s ease;
            position: relative;
            overflow: hidden;
        }
        .status-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 3px;
            background: linear-gradient(90deg, var(--accent-blue), var(--accent-green));
        }
        .status-card:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 24px rgba(0, 212, 255, 0.2);
        }
        .status-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            margin-bottom: 1rem;
        }
        .status-icon {
            width: 14px;
            height: 14px;
            border-radius: 50%;
            animation: pulse 2s infinite;
        }
        .status-active { background: var(--accent-green); box-shadow: 0 0 10px var(--accent-green); }
        .status-waiting { background: var(--accent-orange); box-shadow: 0 0 10px var(--accent-orange); }
        @keyframes pulse { 
            0%, 100% { opacity: 1; transform: scale(1); } 
            50% { opacity: 0.6; transform: scale(0.9); } 
        }
        .status-label {
            font-size: 0.75rem;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: var(--text-secondary);
            font-weight: 600;
        }
        .status-value {
            font-size: 1.8rem;
            font-weight: 700;
            color: var(--text-primary);
            margin-top: 0.5rem;
        }
        .status-subtitle {
            font-size: 0.85rem;
            color: var(--text-secondary);
            margin-top: 0.3rem;
        }
        
        /* Animal Detection Card */
        .animal-detection-card {
            background: linear-gradient(135deg, rgba(168, 85, 247, 0.1), rgba(0, 212, 255, 0.1));
            border: 2px solid var(--accent-purple);
            border-radius: 16px;
            padding: 2rem;
            margin-bottom: 2rem;
            text-align: center;
        }
        .animal-display {
            font-size: 2rem;
            font-weight: 700;
            color: var(--accent-green);
            margin-top: 1rem;
            min-height: 3rem;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .animal-icon {
            font-size: 3rem;
            margin-right: 1rem;
        }
        
        /* Camera Streams */
        .camera-section {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 2rem;
            margin-bottom: 2rem;
        }
        .stream-card {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 16px;
            overflow: hidden;
            transition: all 0.3s ease;
        }
        .stream-card:hover {
            box-shadow: 0 8px 32px rgba(0, 212, 255, 0.3);
        }
        .stream-header { 
            padding: 1rem 1.5rem; 
            border-bottom: 1px solid var(--border-color);
            background: rgba(0, 0, 0, 0.3);
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }
        .stream-header h3 {
            font-size: 1rem;
            font-weight: 600;
        }
        .stream-content { 
            position: relative; 
            background: #000; 
            min-height: 360px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .stream-image { 
            width: 100%; 
            height: auto; 
            display: block;
        }
        .stream-loading {
            position: absolute;
            color: var(--text-secondary);
            font-size: 0.9rem;
        }
        
        /* GPS Map Section */
        .map-section {
            margin-bottom: 2rem;
        }
        .map-card {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 16px;
            overflow: hidden;
        }
        .map-header {
            padding: 1rem 1.5rem;
            border-bottom: 1px solid var(--border-color);
            background: rgba(0, 0, 0, 0.3);
            display: flex;
            align-items: center;
            justify-content: space-between;
        }
        .map-header h3 {
            font-size: 1rem;
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }
        .gps-coords {
            font-size: 0.85rem;
            color: var(--accent-green);
            font-family: 'Courier New', monospace;
        }
        #map {
            height: 400px;
            width: 100%;
            background: #1a1f2e;
        }
        .leaflet-container {
            background: #1a1f2e;
        }
        
        /* Logs Section */
        .logs-section { 
            display: grid; 
            grid-template-columns: 2fr 1fr; 
            gap: 2rem;
            margin-bottom: 2rem;
        }
        .log-panel {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 16px;
            padding: 1.5rem;
        }
        .log-panel h3 {
            margin-bottom: 1rem;
            font-size: 1.2rem;
            color: var(--accent-blue);
        }
        .log-controls {
            display: flex;
            gap: 0.5rem;
            margin-bottom: 1rem;
            flex-wrap: wrap;
        }
        .log-container {
            background: rgba(0, 0, 0, 0.4);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 12px;
            padding: 1rem;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 0.75rem;
            max-height: 500px;
            line-height: 1.6;
        }
        .log-entry { 
            padding: 0.4rem 0; 
            border-bottom: 1px solid rgba(255, 255, 255, 0.05);
            display: flex;
            gap: 0.5rem;
            align-items: flex-start;
        }
        .log-entry:last-child {
            border-bottom: none;
        }
        .log-time {
            color: var(--text-secondary);
            font-size: 0.7rem;
            min-width: 80px;
        }
        .log-level { 
            font-weight: bold; 
            padding: 0.15rem 0.4rem; 
            border-radius: 4px; 
            font-size: 0.65rem;
            min-width: 55px;
            text-align: center;
        }
        .log-level-INFO { background: #2196F3; color: white; }
        .log-level-DEBUG { background: #9E9E9E; color: white; }
        .log-level-WARNING { background: #FF9800; color: white; }
        .log-level-ERROR { background: #F44336; color: white; }
        .log-source {
            color: var(--accent-green);
            min-width: 100px;
            font-size: 0.7rem;
        }
        .log-message {
            color: var(--text-primary);
            flex: 1;
        }
        
        /* Statistics */
        .stats-grid {
            display: grid;
            gap: 1rem;
        }
        .stat-item {
            background: rgba(0, 0, 0, 0.3);
            padding: 1rem;
            border-radius: 8px;
            border-left: 3px solid var(--accent-blue);
        }
        .stat-label {
            font-size: 0.75rem;
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .stat-value {
            font-size: 1.5rem;
            font-weight: 700;
            color: var(--text-primary);
            margin-top: 0.3rem;
        }
        
        /* Buttons */
        .button-group {
            display: flex;
            gap: 0.8rem;
            flex-wrap: wrap;
            margin-bottom: 2rem;
        }
        button {
            background: linear-gradient(135deg, var(--accent-blue), #4dd0e1);
            color: white;
            border: none;
            padding: 0.7rem 1.3rem;
            border-radius: 10px;
            cursor: pointer;
            font-weight: 600;
            font-size: 0.85rem;
            transition: all 0.3s ease;
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0, 212, 255, 0.4);
        }
        button:active {
            transform: translateY(0);
        }
        button.secondary {
            background: linear-gradient(135deg, #6366f1, #8b5cf6);
        }
        button.danger {
            background: linear-gradient(135deg, #f43f5e, #dc2626);
        }
        
        /* Responsive Design */
        @media (max-width: 1400px) {
            .camera-section { grid-template-columns: 1fr; }
            .logs-section { grid-template-columns: 1fr; }
        }
        @media (max-width: 768px) {
            .status-section { grid-template-columns: 1fr; }
            .header-content { flex-direction: column; gap: 1rem; }
            h1 { font-size: 1.4rem; }
        }
        
        /* Loading Animation */
        @keyframes spin {
            to { transform: rotate(360deg); }
        }
        .spinner {
            width: 20px;
            height: 20px;
            border: 3px solid rgba(255, 255, 255, 0.3);
            border-top-color: var(--accent-blue);
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }
    </style>
</head>
<body>
    <header class="header">
        <div class="header-content">
            <h1><span>🚀</span> Jabari Rover - Comprehensive Control Center</h1>
            <div class="connection-badge" id="connectionBadge">
                <div class="spinner"></div>
                <span>Connecting...</span>
            </div>
        </div>
    </header>
    
    <div class="container">
        <!-- Status Cards -->
        <div class="status-section">
            <div class="status-card">
                <div class="status-header">
                    <span class="status-label">Camera Status</span>
                    <div class="status-icon status-waiting" id="cameraStatusIcon"></div>
                </div>
                <div class="status-value" id="cameraFrames">0</div>
                <div class="status-subtitle">frames received</div>
            </div>
            
            <div class="status-card">
                <div class="status-header">
                    <span class="status-label">AI Detection</span>
                    <div class="status-icon status-waiting" id="detectionStatusIcon"></div>
                </div>
                <div class="status-value" id="detectionFrames">0</div>
                <div class="status-subtitle">detections processed</div>
            </div>
            
            <div class="status-card">
                <div class="status-header">
                    <span class="status-label">Motor Left</span>
                </div>
                <div class="status-value" id="motorLeft">0.0</div>
                <div class="status-subtitle">speed</div>
            </div>
            
            <div class="status-card">
                <div class="status-header">
                    <span class="status-label">Motor Right</span>
                </div>
                <div class="status-value" id="motorRight">0.0</div>
                <div class="status-subtitle">speed</div>
            </div>
            
            <div class="status-card">
                <div class="status-header">
                    <span class="status-label">System Uptime</span>
                </div>
                <div class="status-value" id="uptime">0s</div>
                <div class="status-subtitle">running time</div>
            </div>
            
            <div class="status-card">
                <div class="status-header">
                    <span class="status-label">Log Entries</span>
                </div>
                <div class="status-value" id="logCount">0</div>
                <div class="status-subtitle">total logs</div>
            </div>
            
            <div class="status-card">
                <div class="status-header">
                    <span class="status-label">GPS Status</span>
                    <div class="status-icon status-waiting" id="gpsStatusIcon"></div>
                </div>
                <div class="status-value" id="gpsStatus">Waiting</div>
                <div class="status-subtitle" id="gpsCoords">--</div>
            </div>
        </div>
        
        <!-- Animal Detection Display -->
        <div class="animal-detection-card">
            <h3 style="color: var(--accent-purple); font-size: 1.3rem;">🐾 Animal Detection System</h3>
            <div class="animal-display" id="animalDetection">
                <span style="color: var(--text-secondary);">Waiting for detection...</span>
            </div>
            <div style="font-size: 0.85rem; color: var(--text-secondary); margin-top: 0.5rem;">
                Monitoring: Horse, Rooster, Duck, Cow
            </div>
        </div>
        
        <!-- GPS Live Map -->
        <div class="map-section">
            <div class="map-card">
                <div class="map-header">
                    <h3><span>📍</span> Live GPS Location</h3>
                    <div class="gps-coords" id="mapCoords">Waiting for GPS...</div>
                </div>
                <div id="map"></div>
            </div>
        </div>
        
        <!-- Camera Streams -->
        <div class="camera-section">
            <div class="stream-card">
                <div class="stream-header">
                    <h3>📹 Live Camera Feed</h3>
                </div>
                <div class="stream-content">
                    <img src="/camera_feed" alt="Camera" id="cameraStream" class="stream-image" 
                         onerror="this.style.display='none'">
                    <div class="stream-loading" id="cameraLoading">Loading camera...</div>
                </div>
            </div>
            
            <div class="stream-card">
                <div class="stream-header">
                    <h3>🤖 AI Detection Feed</h3>
                </div>
                <div class="stream-content">
                    <img src="/detection_feed" alt="Detection" id="detectionStream" class="stream-image"
                         onerror="this.style.display='none'">
                    <div class="stream-loading" id="detectionLoading">Loading detection...</div>
                </div>
            </div>
        </div>
        
        <!-- Control Buttons -->
        <div class="button-group">
            <button onclick="refreshStreams()">🔄 Refresh Streams</button>
            <button onclick="checkStatus()" class="secondary">📊 Check Status</button>
            <button onclick="centerMapOnRover()" class="secondary">📍 Center Map</button>
            <button onclick="clearLogs()" class="danger">🗑️ Clear Logs</button>
            <button onclick="openCombined()" class="secondary">🎭 Combined View</button>
        </div>
        
        <!-- Logs and Statistics -->
        <div class="logs-section">
            <div class="log-panel">
                <h3>🔍 System Logs (Real-time)</h3>
                <div class="log-controls">
                    <button onclick="filterLogs('ALL')" style="padding: 0.4rem 0.8rem; font-size: 0.75rem;">All</button>
                    <button onclick="filterLogs('INFO')" class="secondary" style="padding: 0.4rem 0.8rem; font-size: 0.75rem;">Info</button>
                    <button onclick="filterLogs('ERROR')" class="danger" style="padding: 0.4rem 0.8rem; font-size: 0.75rem;">Errors</button>
                </div>
                <div class="log-container" id="logContainer">
                    <div class="log-entry">
                        <span class="log-time">--:--:--</span>
                        <span class="log-level log-level-INFO">INFO</span>
                        <span class="log-source">system</span>
                        <span class="log-message">Initializing dashboard...</span>
                    </div>
                </div>
            </div>
            
            <div class="log-panel">
                <h3>📈 Live Statistics</h3>
                <div class="stats-grid">
                    <div class="stat-item">
                        <div class="stat-label">Total Logs</div>
                        <div class="stat-value" id="statLogCount">0</div>
                    </div>
                    <div class="stat-item" style="border-left-color: var(--accent-orange);">
                        <div class="stat-label">Errors</div>
                        <div class="stat-value" id="errorCount">0</div>
                    </div>
                    <div class="stat-item" style="border-left-color: var(--accent-green);">
                        <div class="stat-label">Info Messages</div>
                        <div class="stat-value" id="infoCount">0</div>
                    </div>
                    <div class="stat-item" style="border-left-color: var(--accent-purple);">
                        <div class="stat-label">FPS (Camera)</div>
                        <div class="stat-value" id="cameraFPS">0</div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        const socket = io();
        let logCount = 0;
        let errorCount = 0;
        let infoCount = 0;
        let currentFilter = 'ALL';
        let lastCameraFrame = 0;
        let lastDetectionFrame = 0;
        
        // GPS and Map variables
        let map = null;
        let roverMarker = null;
        let pathPolyline = null;
        let pathCoordinates = [];
        let lastGpsUpdate = null;
        
        // Initialize map on page load
        document.addEventListener('DOMContentLoaded', function() {
            initializeMap();
        });
        
        function initializeMap() {
            // Initialize Leaflet map centered on a default location
            map = L.map('map').setView([0, 0], 2);
            
            // Add OpenStreetMap tiles
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors',
                maxZoom: 19,
            }).addTo(map);
            
            // Custom rover icon
            const roverIcon = L.divIcon({
                className: 'rover-marker',
                html: '<div style="background: linear-gradient(135deg, #00d4ff, #00ff88); width: 20px; height: 20px; border-radius: 50%; border: 3px solid white; box-shadow: 0 0 10px rgba(0,212,255,0.6);"></div>',
                iconSize: [20, 20],
                iconAnchor: [10, 10]
            });
            
            // Initialize marker (hidden until first GPS data)
            roverMarker = L.marker([0, 0], {icon: roverIcon}).addTo(map);
            roverMarker.bindPopup('<b>Jabari Rover</b><br>Waiting for GPS...');
            
            // Initialize path polyline
            pathPolyline = L.polyline([], {
                color: '#00d4ff',
                weight: 3,
                opacity: 0.7,
                smoothFactor: 1
            }).addTo(map);
            
            console.log('🗺️ Map initialized');
        }
        
        function updateMapLocation(latitude, longitude, altitude) {
            if (!map || !roverMarker) return;
            
            const newLatLng = [latitude, longitude];
            
            // Update marker position
            roverMarker.setLatLng(newLatLng);
            roverMarker.setPopupContent(
                `<b>🚀 Jabari Rover</b><br>` +
                `Lat: ${latitude.toFixed(6)}<br>` +
                `Lon: ${longitude.toFixed(6)}<br>` +
                `Alt: ${altitude.toFixed(1)}m`
            );
            
            // Add to path
            pathCoordinates.push(newLatLng);
            if (pathCoordinates.length > 100) {
                pathCoordinates.shift(); // Keep only last 100 points
            }
            pathPolyline.setLatLngs(pathCoordinates);
            
            // Center map on rover (only on first update or if user clicks a button)
            if (!lastGpsUpdate) {
                map.setView(newLatLng, 16);
            }
            
            lastGpsUpdate = Date.now();
            
            // Update coordinates display
            document.getElementById('mapCoords').textContent = 
                `${latitude.toFixed(6)}, ${longitude.toFixed(6)}`;
        }
        
        function centerMapOnRover() {
            if (roverMarker && lastGpsUpdate) {
                map.setView(roverMarker.getLatLng(), 16);
            }
        }
        
        // Connection handling
        socket.on('connect', function() {
            const badge = document.getElementById('connectionBadge');
            badge.innerHTML = '<div class="status-icon status-active"></div><span>Connected</span>';
            badge.classList.remove('disconnected');
            console.log('✓ Connected to server');
        });
        
        socket.on('disconnect', function() {
            const badge = document.getElementById('connectionBadge');
            badge.innerHTML = '<div class="status-icon status-waiting"></div><span>Disconnected</span>';
            badge.classList.add('disconnected');
            console.log('✗ Disconnected from server');
        });
        
        // Animal detection updates
        socket.on('animal_update', function(data) {
            const display = document.getElementById('animalDetection');
            if (data.animals && data.animals.length > 0) {
                const animalEmojis = {
                    'horse': '🐴',
                    'rooster': '🐓',
                    'duck': '🦆',
                    'cow': '🐄'
                };
                const emoji = animalEmojis[data.animals[0]] || '🐾';
                display.innerHTML = `<span class="animal-icon">${emoji}</span><span>${data.message}</span>`;
            } else {
                display.innerHTML = '<span style="color: var(--text-secondary);">No detection</span>';
            }
        });
        
        // GPS location updates
        socket.on('gps_update', function(data) {
            if (data.latitude && data.longitude) {
                updateMapLocation(data.latitude, data.longitude, data.altitude || 0);
                
                // Update GPS status card
                const gpsIcon = document.getElementById('gpsStatusIcon');
                gpsIcon.className = 'status-icon status-active';
                document.getElementById('gpsStatus').textContent = 'Active';
                document.getElementById('gpsCoords').textContent = 
                    `${data.latitude.toFixed(4)}, ${data.longitude.toFixed(4)}`;
            }
        });
        
        // Status updates
        socket.on('status_update', function(status) {
            if (status.camera_status) {
                const icon = document.getElementById('cameraStatusIcon');
                icon.className = `status-icon ${status.camera_status === 'active' ? 'status-active' : 'status-waiting'}`;
            }
            if (status.detection_status) {
                const icon = document.getElementById('detectionStatusIcon');
                icon.className = `status-icon ${status.detection_status === 'active' ? 'status-active' : 'status-waiting'}`;
            }
            if (status.camera_frames !== undefined) {
                document.getElementById('cameraFrames').textContent = status.camera_frames;
                // Calculate FPS
                if (lastCameraFrame > 0) {
                    const fps = Math.round((status.camera_frames - lastCameraFrame) / 5);
                    document.getElementById('cameraFPS').textContent = fps;
                }
                lastCameraFrame = status.camera_frames;
            }
            if (status.detection_frames !== undefined) {
                document.getElementById('detectionFrames').textContent = status.detection_frames;
            }
            if (status.motor_left !== undefined) {
                document.getElementById('motorLeft').textContent = status.motor_left.toFixed(2);
            }
            if (status.motor_right !== undefined) {
                document.getElementById('motorRight').textContent = status.motor_right.toFixed(2);
            }
            if (status.uptime !== undefined) {
                const mins = Math.floor(status.uptime / 60);
                const secs = Math.floor(status.uptime % 60);
                document.getElementById('uptime').textContent = `${mins}m ${secs}s`;
            }
        });
        
        // Log updates
        socket.on('new_log', function(log) {
            if (currentFilter !== 'ALL' && log.level !== currentFilter) {
                return;
            }
            
            const container = document.getElementById('logContainer');
            const entry = document.createElement('div');
            entry.className = 'log-entry';
            entry.innerHTML = `
                <span class="log-time">${log.time_str}</span>
                <span class="log-level log-level-${log.level}">${log.level}</span>
                <span class="log-source">${log.source}</span>
                <span class="log-message">${log.message}</span>
            `;
            container.appendChild(entry);
            container.scrollTop = container.scrollHeight;
            
            // Update stats
            logCount++;
            if (log.level === 'ERROR') errorCount++;
            if (log.level === 'INFO') infoCount++;
            
            document.getElementById('logCount').textContent = logCount;
            document.getElementById('statLogCount').textContent = logCount;
            document.getElementById('errorCount').textContent = errorCount;
            document.getElementById('infoCount').textContent = infoCount;
            
            // Keep only last 500 entries in DOM
            while (container.children.length > 500) {
                container.removeChild(container.firstChild);
            }
        });
        
        // Image loading handlers
        document.getElementById('cameraStream').onload = function() {
            document.getElementById('cameraLoading').style.display = 'none';
            this.style.display = 'block';
        };
        
        document.getElementById('detectionStream').onload = function() {
            document.getElementById('detectionLoading').style.display = 'none';
            this.style.display = 'block';
        };
        
        // Control functions
        function refreshStreams() {
            const ts = new Date().getTime();
            document.getElementById('cameraStream').src = '/camera_feed?' + ts;
            document.getElementById('detectionStream').src = '/detection_feed?' + ts;
            console.log('🔄 Streams refreshed');
        }
        
        function checkStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    console.log('📊 Status:', data);
                    alert(`System Status:\n\nCamera: ${data.camera_status}\nDetection: ${data.detection_status}\nUptime: ${data.uptime}s\nLogs: ${data.log_count}`);
                })
                .catch(err => console.error('Error:', err));
        }
        
        function clearLogs() {
            if (confirm('Clear all logs?')) {
                socket.emit('clear_logs');
                document.getElementById('logContainer').innerHTML = '';
                logCount = 0;
                errorCount = 0;
                infoCount = 0;
                document.getElementById('logCount').textContent = '0';
                document.getElementById('statLogCount').textContent = '0';
                document.getElementById('errorCount').textContent = '0';
                document.getElementById('infoCount').textContent = '0';
                console.log('🗑️ Logs cleared');
            }
        }
        
        function filterLogs(level) {
            currentFilter = level;
            fetch(`/api/logs?level=${level === 'ALL' ? '' : level}&count=500`)
                .then(r => r.json())
                .then(data => {
                    const container = document.getElementById('logContainer');
                    container.innerHTML = '';
                    data.logs.forEach(log => {
                        const entry = document.createElement('div');
                        entry.className = 'log-entry';
                        entry.innerHTML = `
                            <span class="log-time">${log.time_str}</span>
                            <span class="log-level log-level-${log.level}">${log.level}</span>
                            <span class="log-source">${log.source}</span>
                            <span class="log-message">${log.message}</span>
                        `;
                        container.appendChild(entry);
                    });
                    container.scrollTop = container.scrollHeight;
                    console.log(`🔍 Filtered logs: ${level}`);
                })
                .catch(err => console.error('Error:', err));
        }
        
        function openCombined() {
            window.open('/combined_feed', 'Combined View', 'width=1280,height=480');
        }
        
        // Periodic status check
        setInterval(function() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    // Update quietly without alerts
                })
                .catch(err => {});
        }, 10000);
        
        // Initialize
        console.log('🚀 Dashboard initialized');
        setTimeout(refreshStreams, 1000);
    </script>
</body>
</html>
''')

def main(args=None):
    rclpy.init(args=args)

    # Initialize Flask+SocketIO
    app = Flask(__name__)
    app.config['SECRET_KEY'] = 'jabari_camera_streamer_secret'
    socketio = SocketIO(app, cors_allowed_origins="*")

    # Create node with socketio support
    node = ComprehensiveWebStreamer(socketio)
    node.app = app

    # Setup routes
    app.add_url_rule('/camera_feed', 'camera_feed', node.camera_feed)
    app.add_url_rule('/detection_feed', 'detection_feed', node.detection_feed)
    app.add_url_rule('/combined_feed', 'combined_feed', node.combined_feed)
    app.add_url_rule('/', 'index', node.index)
    app.add_url_rule('/status', 'status', node.status)
    app.add_url_rule('/health', 'health', node.health)
    app.add_url_rule('/api/logs', 'get_logs', node.get_logs)
    app.add_url_rule('/api/status', 'get_status', node.get_status)
    app.add_url_rule('/api/gps', 'get_gps_history', node.get_gps_history)

    # SocketIO event handlers
    @socketio.on('connect')
    def handle_connect():
        try:
            node.get_logger().info(f"Web client connected: {request.sid}")
            recent_logs = node.get_recent_logs(50)
            for log_entry in recent_logs:
                emit('new_log', log_entry)
        except Exception:
            pass

    @socketio.on('disconnect')
    def handle_disconnect():
        try:
            node.get_logger().info(f"Web client disconnected: {request.sid}")
        except Exception:
            pass

    @socketio.on('clear_logs')
    def handle_clear_logs():
        node.log_buffer.clear()
        socketio.emit('logs_cleared', broadcast=True)

    # Start Flask-SocketIO in a background thread
    flask_thread = threading.Thread(
        target=lambda: socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False),
        daemon=True
    )
    flask_thread.start()

    # Get local IP (best effort)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception:
        local_ip = "localhost"

    print("\n" + "="*60)
    print("✅ ENHANCED WEB STREAMER STARTED!")
    print("="*60)
    print(f"📱 Dashboard:       http://{local_ip}:5000/")
    print(f"🎥 Camera Feed:     http://{local_ip}:5000/camera_feed")
    print(f"🤖 Detection Feed:  http://{local_ip}:5000/detection_feed")
    print(f"🎭 Combined View:   http://{local_ip}:5000/combined_feed")
    print(f"❤️  Health Check:    http://{local_ip}:5000/health")
    print(f"📊 Status API:      http://{local_ip}:5000/status")
    print(f"📍 GPS API:         http://{local_ip}:5000/api/gps")
    print("="*60)
    print("\n🚀 Features:")
    print("   ✓ Real-time video streaming (compressed)")
    print("   ✓ WebSocket live log monitoring")
    print("   ✓ System status dashboard")
    print("   ✓ Motor & servo tracking")
    print("   ✓ Object detection monitoring")
    print("   ✓ Live GPS location mapping")
    print("   ✓ Health check endpoint")
    print("="*60 + "\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
