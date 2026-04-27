#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Import both Image and CompressedImage
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
# Import more QoS policies
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import cv2
import threading
from flask import Flask, Response, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
import time
import logging
import json
from datetime import datetime
from collections import deque
from typing import Dict, List, Optional
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Vector3


class LogEntry:
    """Structured log entry for web display"""
    def __init__(self, timestamp: float, level: str, source: str, message: str, data: Optional[Dict] = None):
        self.timestamp = timestamp
        self.level = level
        self.source = source
        self.message = message
        self.data = data or {}
        
    def to_dict(self):
        return {
            'timestamp': self.timestamp,
            'time_str': datetime.fromtimestamp(self.timestamp).strftime('%H:%M:%S.%f')[:-3],
            'level': self.level,
            'source': self.source,
            'message': self.message,
            'data': self.data
        }


class DualWebStreamer(Node):
    def __init__(self, socketio):
        super().__init__('dual_web_streamer')
        self.bridge = CvBridge()
        self.socketio = socketio
        
        # Track start time for uptime monitoring
        self._start_time = time.time()
        
        self.log_buffer = deque(maxlen=1000)
        self.system_status = {}
        
        self.camera_frame = None
        self.detection_frame = None
        self.camera_lock = threading.Lock()
        self.detection_lock = threading.Lock()
        
        self.camera_frame_count = 0
        self.detection_frame_count = 0
        
        # QoS profile to match the camera publisher
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for detection
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribe to the correct COMPRESSED topic with the correct message type
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_callback,
            camera_qos)
            
        self.detection_subscription = self.create_subscription(
            CompressedImage,
            '/yolo/annotated/compressed',
            self.detection_callback,
            detection_qos)
        
        # Additional topic subscriptions for comprehensive monitoring
        self.status_sub = self.create_subscription(
            String, '/manual_control_status', self.status_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.vector3_sub = self.create_subscription(
            Vector3, '/vector3_cmd', self.vector3_callback, 10)
        self.left_motor_sub = self.create_subscription(
            Float32, '/motor/left_speed', self.left_motor_callback, 10)
        self.right_motor_sub = self.create_subscription(
            Float32, '/motor/right_speed', self.right_motor_callback, 10)
        self.object_info_sub = self.create_subscription(
            String, '/yolo/detections_log', self.object_detection_callback, 10)
        
        self.motor_status = {'left_speed': 0.0, 'right_speed': 0.0}
        self.last_object_detection = None
        
        self.get_logger().info('Enhanced dual web streamer with comprehensive monitoring started.')
        self.app = None
        self.setup_log_capture()
        
        # Start periodic status emission to the web dashboard
        threading.Thread(target=self._periodic_status_update, daemon=True).start()

    def _periodic_status_update(self):
        """Send periodic status updates to the dashboard."""
        while rclpy.ok():
            try:
                status = {
                    'camera_status': 'active' if self.camera_frame is not None else 'waiting',
                    'detection_status': 'active' if self.detection_frame is not None else 'waiting',
                    'camera_frames': self.camera_frame_count,
                    'detection_frames': self.detection_frame_count,
                    'uptime': time.time() - self._start_time
                }
                self.socketio.start_background_task(self.socketio.emit, 'status_update', status)
                time.sleep(5)
            except Exception as e:
                self.get_logger().warn(f"Status update thread error: {e}")
                time.sleep(5)

    def setup_log_capture(self):
        """Setup custom log handler to capture ROS2 logs"""
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
                    pass
        
        handler = WebLogHandler(self)
        logging.getLogger().addHandler(handler)
    
    def status_callback(self, msg):
        """Handle manual control status updates"""
        try:
            status_data = json.loads(msg.data)
            self.system_status.update(status_data)
            
            log_entry = LogEntry(
                timestamp=time.time(),
                level='INFO',
                source='manual_control',
                message='Status update received',
                data=status_data
            )
            self.add_log_entry(log_entry)
            
            self.socketio.start_background_task(self.socketio.emit, 'status_update', status_data)
            
        except Exception as e:
            self.get_logger().error(f"Error processing status: {e}")
    
    def cmd_vel_callback(self, msg):
        """Handle movement commands"""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            log_entry = LogEntry(
                timestamp=time.time(),
                level='INFO',
                source='movement',
                message=f'Movement: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}',
                data={
                    'linear_x': msg.linear.x,
                    'angular_z': msg.angular.z
                }
            )
            self.add_log_entry(log_entry)
    
    def vector3_callback(self, msg):
        """Handle servo commands"""
        log_entry = LogEntry(
            timestamp=time.time(),
            level='INFO',
            source='servos',
            message=f'Servo: pan={msg.x:.1f}°, tilt={msg.y:.1f}°, third={msg.z:.1f}°',
            data={
                'pan': msg.x,
                'tilt': msg.y,
                'third': msg.z
            }
        )
        self.add_log_entry(log_entry)
    
    def left_motor_callback(self, msg):
        """Handle left motor speed updates"""
        self.motor_status['left_speed'] = msg.data
        log_entry = LogEntry(
            timestamp=time.time(),
            level='INFO',
            source='motor_controller',
            message=f'Left motor speed: {msg.data:.2f}',
            data={'left_speed': msg.data}
        )
        self.add_log_entry(log_entry)
        
        self.system_status['motor_left'] = msg.data
        self.socketio.start_background_task(self.socketio.emit, 'status_update', {'motor_left': msg.data})
    
    def right_motor_callback(self, msg):
        """Handle right motor speed updates"""
        self.motor_status['right_speed'] = msg.data
        log_entry = LogEntry(
            timestamp=time.time(),
            level='INFO',
            source='motor_controller',
            message=f'Right motor speed: {msg.data:.2f}',
            data={'right_speed': msg.data}
        )
        self.add_log_entry(log_entry)
        
        self.system_status['motor_right'] = msg.data
        self.socketio.start_background_task(self.socketio.emit, 'status_update', {'motor_right': msg.data})
    
    def object_detection_callback(self, msg):
        """Handle object detection results"""
        if msg.data and msg.data != self.last_object_detection:
            self.last_object_detection = msg.data
            
            detection_count = msg.data.count('Object:')
            
            log_entry = LogEntry(
                timestamp=time.time(),
                level='INFO',
                source='object_detection',
                message=f'Detected {detection_count} objects: {msg.data[:100]}...' if len(msg.data) > 100 else msg.data,
                data={
                    'detection_count': detection_count,
                    'full_detection': msg.data
                }
            )
            self.add_log_entry(log_entry)
            
            self.system_status['last_detection'] = {
                'count': detection_count,
                'timestamp': time.time(),
                'data': msg.data
            }
            self.socketio.start_background_task(self.socketio.emit, 'status_update', {'object_detection': self.system_status['last_detection']})
    
    def add_log_entry(self, log_entry: LogEntry):
        """Add log entry to buffer and emit to web clients"""
        self.log_buffer.append(log_entry)
        self.socketio.start_background_task(self.socketio.emit, 'new_log', log_entry.to_dict())
    
    def get_recent_logs(self, count: int = 100) -> List[Dict]:
        """Get recent log entries"""
        return [entry.to_dict() for entry in list(self.log_buffer)[-count:]]

    def camera_callback(self, msg):
        """Callback for camera images"""
        try:
            # Decode the compressed image manually
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warn('Failed to decode compressed image.')
                return

            with self.camera_lock:
                self.camera_frame = cv_image
                self.camera_frame_count += 1
                
            if self.camera_frame_count % 30 == 0:
                height, width, _ = cv_image.shape
                log_entry = LogEntry(
                    timestamp=time.time(),
                    level='DEBUG',
                    source='camera',
                    message=f'Camera frame #{self.camera_frame_count} - {width}x{height}',
                    data={
                        'frame_count': self.camera_frame_count,
                        'width': width,
                        'height': height,
                        'format': msg.format
                    }
                )
                self.add_log_entry(log_entry)
                
                self.system_status['camera'] = {
                    'frame_count': self.camera_frame_count,
                    'resolution': f"{width}x{height}",
                    'format': msg.format,
                    'last_frame_time': time.time()
                }
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')
            
    def detection_callback(self, msg: CompressedImage):
        """Callback for detection images"""
        try:
            # Manually decode the compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn('Failed to decode compressed detection image.')
                return

            with self.detection_lock:
                self.detection_frame = cv_image
                self.detection_frame_count += 1
            
            height, width, _ = cv_image.shape
        
            log_entry = LogEntry(
                timestamp=time.time(),
                level='DEBUG',
                source='yolo_processor',
                message=f'Annotated compressed image received - {width}x{height}',
                data={
                    'width': width,
                    'height': height,
                    'format': msg.format
                }
            )
            self.add_log_entry(log_entry)
        
        except Exception as e:
            self.get_logger().error(f'Error processing detection image: {str(e)}')

    def generate_camera_frames(self):
        """Generate frames for camera streaming"""
        last_frame_count = 0
        
        while True:
            frame_to_send = None
            current_frame_count = 0
            
            with self.camera_lock:
                if self.camera_frame is not None and self.camera_frame_count > last_frame_count:
                    frame_to_send = self.camera_frame.copy()
                    current_frame_count = self.camera_frame_count
            
            if frame_to_send is not None:
                height, width = frame_to_send.shape[:2]
                if width > 800:
                    new_width = 800
                    new_height = int(height * (800 / width))
                    frame_to_send = cv2.resize(frame_to_send, (new_width, new_height))
                
                ret, buffer = cv2.imencode('.jpg', frame_to_send, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    last_frame_count = current_frame_count
            else:
                placeholder = self.create_camera_placeholder()
                ret, buffer = cv2.imencode('.jpg', placeholder, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            time.sleep(0.05)
            
    def generate_detection_frames(self):
        """Generate frames for detection streaming"""
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
                new_width = 400
                new_height = int(height * (400 / width))
                frame_to_send = cv2.resize(frame_to_send, (new_width, new_height))
                
                ret, buffer = cv2.imencode('.jpg', frame_to_send, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                placeholder = self.create_detection_placeholder()
                ret, buffer = cv2.imencode('.jpg', placeholder, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            time.sleep(0.2)
    
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
            
            ret, buffer = cv2.imencode('.jpg', combined_frame, 
                                     [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ret:
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            time.sleep(0.1)
    
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
    
    def create_camera_placeholder(self):
        """Create placeholder for camera feed"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, 'Waiting for camera...', (180, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return frame
        
    def create_detection_placeholder(self):
        """Create placeholder for detection feed"""
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.putText(frame, 'Waiting for', (90, 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, 'detections...', (70, 140), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return frame
    
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
        """Enhanced status endpoint with proper headers"""
        with self.camera_lock:
            has_camera = self.camera_frame is not None
            camera_count = self.camera_frame_count
            
        with self.detection_lock:
            has_detection = self.detection_frame is not None
            detection_count = self.detection_frame_count
        
        response_data = {
            'camera_status': 'active' if has_camera else 'waiting',
            'detection_status': 'active' if has_detection else 'waiting',
            'camera_frames': camera_count,
            'detection_frames': detection_count,
            'message': f'Camera: {"✓" if has_camera else "⏳"} | Detection: {"✓" if has_detection else "⏳"}',
            'timestamp': time.time(),
            'system_status': self.system_status
        }
        
        response = jsonify(response_data)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        
        return response
    
    def health(self):
        """Health check endpoint for monitoring"""
        health_data = {
            'status': 'healthy',
            'timestamp': time.time(),
            'uptime': time.time() - self._start_time,
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
        """Enhanced API endpoint to get recent logs"""
        try:
            count = request.args.get('count', 100, type=int)
            level_filter = request.args.get('level', '')
            source_filter = request.args.get('source', '')
            
            count = min(count, 1000)
            
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
            response.headers['Pragma'] = 'no-cache'
            response.headers['Expires'] = '0'
            response.headers['Access-Control-Allow-Origin'] = '*'
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in get_logs endpoint: {e}")
            return jsonify({'error': str(e), 'logs': []}), 500
    
    def get_status(self):
        """API endpoint to get system status"""
        response = jsonify(self.system_status)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response

    def index(self):
        """Unified main page with camera streams and log dashboard"""
        return render_template_string('''
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>Jabari ROS2 Camera System & Log Dashboard</title>
            <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
            <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet">
            <style>
                :root {
                    --primary-bg: #0f1419;
                    --secondary-bg: #1a1f2e;
                    --card-bg: #252b3a;
                    --accent-blue: #00d4ff;
                    --accent-orange: #ff6b35;
                    --accent-green: #00ff88;
                    --text-primary: #ffffff;
                    --text-secondary: #b8bcc8;
                    --text-muted: #8b92a5;
                    --border-color: #3d4758;
                    --shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
                    --gradient: linear-gradient(135deg, var(--primary-bg) 0%, var(--secondary-bg) 100%);
                }

                * {
                    margin: 0;
                    padding: 0;
                    box-sizing: border-box;
                }

                body { 
                    font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
                    background: var(--gradient);
                    color: var(--text-primary);
                    min-height: 100vh;
                    line-height: 1.6;
                }

                .header {
                    background: rgba(37, 43, 58, 0.8);
                    backdrop-filter: blur(20px);
                    border-bottom: 1px solid var(--border-color);
                    padding: 1rem 0;
                    position: sticky;
                    top: 0;
                    z-index: 100;
                }

                .header-content {
                    max-width: 1600px;
                    margin: 0 auto;
                    padding: 0 2rem;
                    display: flex;
                    align-items: center;
                    justify-content: space-between;
                }

                .logo {
                    display: flex;
                    align-items: center;
                    gap: 12px;
                }

                .logo-icon {
                    width: 40px;
                    height: 40px;
                    background: linear-gradient(135deg, var(--accent-blue), var(--accent-green));
                    border-radius: 12px;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    font-size: 1.5rem;
                }

                h1 { 
                    font-size: 1.5rem;
                    font-weight: 600;
                    background: linear-gradient(135deg, var(--accent-blue), var(--accent-green));
                    -webkit-background-clip: text;
                    -webkit-text-fill-color: transparent;
                    background-clip: text;
                }

                .container {
                    max-width: 1600px;
                    margin: 0 auto;
                    padding: 2rem;
                }

                .status-card {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    padding: 1.5rem;
                    box-shadow: var(--shadow);
                    margin-bottom: 2rem;
                }

                .status-grid {
                    display: grid;
                    grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
                    gap: 1rem;
                }

                .status-item {
                    display: flex;
                    align-items: center;
                    gap: 12px;
                    padding: 1rem;
                    background: rgba(255, 255, 255, 0.03);
                    border-radius: 12px;
                    border: 1px solid rgba(255, 255, 255, 0.05);
                }

                .status-icon {
                    width: 12px;
                    height: 12px;
                    border-radius: 50%;
                    animation: pulse 2s infinite;
                }

                .status-active { background: var(--accent-green); }
                .status-waiting { background: var(--accent-orange); }
                .status-error { 
                    background: #f44336;
                    box-shadow: 0 0 10px rgba(244, 67, 54, 0.5);
                }

                @keyframes pulse {
                    0%, 100% { opacity: 1; }
                    50% { opacity: 0.5; }
                }

                .status-text {
                    font-size: 0.9rem;
                    color: var(--text-secondary);
                }

                .status-value {
                    font-weight: 600;
                    color: var(--text-primary);
                }

                .camera-streams-section {
                    display: grid;
                    grid-template-columns: 1fr 1fr;
                    gap: 1.5rem;
                    margin-bottom: 2rem;
                }

                .controls-section {
                    margin-bottom: 2rem;
                }

                .logs-section {
                    display: grid;
                    grid-template-columns: 2fr 1fr;
                    gap: 2rem;
                }

                .stream-card {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    overflow: hidden;
                    box-shadow: var(--shadow);
                }

                .stream-header {
                    padding: 1rem 1.5rem;
                    border-bottom: 1px solid var(--border-color);
                    display: flex;
                    align-items: center;
                    gap: 12px;
                }

                .stream-icon {
                    width: 32px;
                    height: 32px;
                    border-radius: 8px;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    font-size: 1.2rem;
                }

                .camera-icon {
                    background: linear-gradient(135deg, var(--accent-blue), #4dd0e1);
                }

                .detection-icon {
                    background: linear-gradient(135deg, var(--accent-orange), #ff8a65);
                }

                .stream-title {
                    font-size: 1rem;
                    font-weight: 600;
                    color: var(--text-primary);
                }

                .stream-subtitle {
                    font-size: 0.8rem;
                    color: var(--text-muted);
                }

                .stream-content {
                    position: relative;
                    background: #000;
                    min-height: 280px;
                }

                .stream-image {
                    width: 100%;
                    height: auto;
                    display: block;
                    transition: opacity 0.3s ease;
                }

                .stream-overlay {
                    position: absolute;
                    top: 1rem;
                    left: 1rem;
                    background: rgba(0, 0, 0, 0.7);
                    backdrop-filter: blur(10px);
                    color: white;
                    padding: 0.3rem 0.8rem;
                    border-radius: 6px;
                    font-size: 0.75rem;
                    font-weight: 500;
                }

                .controls-card {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    padding: 1rem;
                    box-shadow: var(--shadow);
                }

                .controls-grid {
                    display: grid;
                    grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
                    gap: 0.8rem;
                }

                .log-panel {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    padding: 1.5rem;
                    box-shadow: var(--shadow);
                    display: flex;
                    flex-direction: column;
                    min-height: 500px;
                }

                .log-stats-panel {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    padding: 1rem;
                    box-shadow: var(--shadow);
                    display: flex;
                    flex-direction: column;
                    gap: 1.5rem;
                }

                .log-controls {
                    display: flex;
                    gap: 0.5rem;
                    margin-bottom: 1rem;
                    flex-wrap: wrap;
                    align-items: center;
                }

                .control-group {
                    display: flex;
                    flex-direction: column;
                    gap: 0.2rem;
                }

                .control-group label {
                    font-size: 0.8rem;
                    color: var(--text-muted);
                }

                select, input, button {
                    padding: 0.4rem 0.8rem;
                    border: 1px solid var(--border-color);
                    border-radius: 6px;
                    background: rgba(0, 0, 0, 0.2);
                    color: white;
                    font-size: 0.8rem;
                    outline: none;
                }

                button {
                    cursor: pointer;
                    transition: all 0.3s;
                    background: linear-gradient(135deg, var(--accent-blue), #4dd0e1);
                    border: none;
                    font-weight: 500;
                }

                button:hover {
                    transform: translateY(-1px);
                    box-shadow: 0 4px 12px rgba(0, 212, 255, 0.3);
                }

                .log-container {
                    flex: 1;
                    background: rgba(0, 0, 0, 0.3);
                    border-radius: 8px;
                    padding: 1rem;
                    overflow-y: auto;
                    font-family: 'Courier New', monospace;
                    font-size: 0.75rem;
                    line-height: 1.4;
                    border: 1px solid var(--border-color);
                    max-height: 350px;
                }

                .log-entry {
                    padding: 0.2rem 0;
                    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
                    animation: fadeIn 0.3s ease-in;
                }

                .log-entry:last-child {
                    border-bottom: none;
                }

                .log-timestamp {
                    color: #9e9e9e;
                    font-size: 0.7rem;
                }

                .log-level {
                    font-weight: bold;
                    padding: 0.1rem 0.3rem;
                    border-radius: 3px;
                    font-size: 0.65rem;
                    margin: 0 0.3rem;
                }

                .log-level-INFO { background: #2196F3; color: white; }
                .log-level-DEBUG { background: #9E9E9E; color: white; }
                .log-level-WARNING { background: #FF9800; color: white; }
                .log-level-ERROR { background: #F44336; color: white; }

                .log-source {
                    color: #81C784;
                    font-weight: 500;
                }

                .log-message {
                    color: #ffffff;
                    margin-left: 0.3rem;
                }

                .connection-status {
                    display: inline-block;
                    padding: 0.3rem 0.8rem;
                    border-radius: 15px;
                    font-size: 0.8rem;
                    font-weight: bold;
                }

                .status-connected {
                    background: #4CAF50;
                    color: white;
                }

                .status-disconnected {
                    background: #f44336;
                    color: white;
                }

                .stats-grid {
                    display: grid;
                    grid-template-columns: 1fr 1fr;
                    gap: 0.8rem;
                }

                .stat-card {
                    background: rgba(0, 0, 0, 0.3);
                    padding: 0.8rem;
                    border-radius: 8px;
                    text-align: center;
                    border: 1px solid var(--border-color);
                }

                .stat-value {
                    font-size: 1.2rem;
                    font-weight: bold;
                    color: var(--accent-green);
                }

                .stat-label {
                    font-size: 0.7rem;
                    color: var(--text-muted);
                }

                .system-status {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    padding: 1rem;
                    box-shadow: var(--shadow);
                }

                .status-items {
                    max-height: 200px;
                    overflow-y: auto;
                }

                .system-status-item {
                    background: rgba(0, 0, 0, 0.2);
                    padding: 0.8rem;
                    border-radius: 8px;
                    margin-bottom: 0.5rem;
                    border-left: 4px solid var(--accent-blue);
                }

                .system-status-item h4 {
                    color: var(--accent-green);
                    margin-bottom: 0.3rem;
                    font-size: 0.9rem;
                }

                .system-status-item p {
                    margin: 0.1rem 0;
                    font-size: 0.8rem;
                    color: var(--text-secondary);
                }

                @keyframes fadeIn {
                    from { opacity: 0; transform: translateY(5px); }
                    to { opacity: 1; transform: translateY(0); }
                }

                .btn {
                    background: linear-gradient(135deg, var(--accent-blue), #4dd0e1);
                    color: white;
                    border: none;
                    padding: 0.6rem 1rem;
                    border-radius: 8px;
                    font-size: 0.8rem;
                    font-weight: 500;
                    cursor: pointer;
                    transition: all 0.3s ease;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    gap: 6px;
                }

                .btn:hover {
                    transform: translateY(-1px);
                    box-shadow: 0 4px 12px rgba(0, 212, 255, 0.3);
                }

                .btn-secondary {
                    background: linear-gradient(135deg, var(--accent-orange), #ff8a65);
                }

                .btn-secondary:hover {
                    box-shadow: 0 4px 12px rgba(255, 107, 53, 0.3);
                }

                .section-title {
                    font-size: 1.1rem;
                    font-weight: 600;
                    margin-bottom: 1rem;
                    color: var(--text-primary);
                    display: flex;
                    align-items: center;
                    gap: 8px;
                }

                @media (max-width: 1200px) {
                    .camera-streams-section {
                        grid-template-columns: 1fr;
                    }
                    
                    .logs-section {
                        grid-template-columns: 1fr;
                        gap: 1rem;
                    }
                    
                    .container {
                        padding: 1rem;
                    }
                    
                    .log-container {
                        max-height: 250px;
                    }
                }

                @media (max-width: 768px) {
                    .status-grid {
                        grid-template-columns: 1fr;
                    }
                    
                    .controls-grid {
                        grid-template-columns: 1fr;
                    }
                    
                    .header-content {
                        flex-direction: column;
                        gap: 1rem;
                    }

                    .camera-streams-section {
                        grid-template-columns: 1fr;
                    }
                    
                    .logs-section {
                        grid-template-columns: 1fr;
                    }
                }
            </style>
        </head>
        <body>
            <header class="header">
                <div class="header-content">
                    <div class="logo">
                        <div class="logo-icon">🚀</div>
                        <h1>Jabari Rover Control Center</h1>
                    </div>
                    <div class="connection-status" id="connectionStatus">
                        <span id="connectionText">Initializing...</span>
                    </div>
                </div>
            </header>

            <div class="container">
                <!-- System Status Overview -->
                <div class="status-card">
                    <div class="section-title">📊 System Status</div>
                    <div class="status-grid" id="statusGrid">
                        <div class="status-item">
                            <div class="status-icon status-waiting" id="cameraStatusIcon"></div>
                            <div>
                                <div class="status-text">Camera Feed</div>
                                <div class="status-value" id="cameraStatus">Connecting...</div>
                            </div>
                        </div>
                        <div class="status-item">
                            <div class="status-icon status-waiting" id="detectionStatusIcon"></div>
                            <div>
                                <div class="status-text">AI Detection</div>
                                <div class="status-value" id="detectionStatus">Connecting...</div>
                            </div>
                        </div>
                        <div class="status-item">
                            <div class="status-icon status-waiting" id="rosStatusIcon"></div>
                            <div>
                                <div class="status-text">ROS2 Logs</div>
                                <div class="status-value" id="rosStatus">Connecting...</div>
                            </div>
                        </div>
                        <div class="status-item">
                            <div class="status-icon status-waiting"></div>
                            <div>
                                <div class="status-text">Total Frames</div>
                                <div class="status-value" id="totalFrames">0</div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Camera Streams Section -->
                <div class="section-title">🎥 Live Camera Feeds</div>
                <div class="camera-streams-section">
                    <div class="stream-card">
                        <div class="stream-header">
                            <div class="stream-icon camera-icon">📹</div>
                            <div>
                                <div class="stream-title">Live Camera Feed</div>
                                <div class="stream-subtitle">Real-time video stream</div>
                            </div>
                        </div>
                        <div class="stream-content">
                            <img src="/camera_feed" alt="Camera Feed" id="cameraStream" class="stream-image">
                            <div class="stream-overlay">🔴 LIVE</div>
                        </div>
                    </div>

                    <div class="stream-card">
                        <div class="stream-header">
                            <div class="stream-icon detection-icon">🤖</div>
                            <div>
                                <div class="stream-title">AI Object Detection</div>
                                <div class="stream-subtitle">YOLO detection overlay</div>
                            </div>
                        </div>
                        <div class="stream-content">
                            <img src="/detection_feed" alt="Detection Feed" id="detectionStream" class="stream-image">
                            <div class="stream-overlay">🧠 AI</div>
                        </div>
                    </div>
                </div>

                <!-- Stream Controls Section -->
                <div class="controls-section">
                    <div class="controls-card">
                        <div class="controls-grid">
                            <button class="btn" onclick="refreshStreams()">🔄 Refresh</button>
                            <button class="btn btn-secondary" onclick="checkStatus()">📊 Status</button>
                            <button class="btn" onclick="toggleFullscreen('cameraStream')">🔍 Camera Full</button>
                            <button class="btn btn-secondary" onclick="toggleFullscreen('detectionStream')">🎯 Detection Full</button>
                        </div>
                    </div>
                </div>

                <!-- Logs and Statistics Section -->
                <div class="logs-section">
                    <div class="log-panel">
                        <div class="section-title">🔍 System Logs</div>
                        <div class="log-controls">
                            <div class="control-group">
                                <label>Level</label>
                                <select id="levelFilter">
                                    <option value="">All</option>
                                    <option value="DEBUG">Debug</option>
                                    <option value="INFO">Info</option>
                                    <option value="WARNING">Warning</option>
                                    <option value="ERROR">Error</option>
                                </select>
                            </div>
                            <div class="control-group">
                                <label>Source</label>
                                <input type="text" id="sourceFilter" placeholder="Filter...">
                            </div>
                            <button onclick="clearLogs()">Clear</button>
                            <button onclick="exportLogs()" class="btn-secondary">Export</button>
                        </div>
                        <div class="log-container" id="logContainer">
                            <div class="log-entry">
                                <span class="log-timestamp">Initializing log monitor...</span>
                            </div>
                        </div>
                    </div>

                    <div class="log-stats-panel">
                        <div>
                            <div class="section-title">📈 Log Statistics</div>
                            <div class="stats-grid">
                                <div class="stat-card">
                                    <div class="stat-value" id="logCount">0</div>
                                    <div class="stat-label">Total Logs</div>
                                </div>
                                <div class="stat-card">
                                    <div class="stat-value" id="errorCount">0</div>
                                    <div class="stat-label">Errors</div>
                                </div>
                            </div>
                        </div>
                        
                        <div class="system-status">
                            <div class="section-title">🔧 System Components</div>
                            <div class="status-items" id="statusItems">
                                <div class="system-status-item">
                                    <h4>Connection</h4>
                                    <p>Status: <span id="nodeStatus">Initializing...</span></p>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>

            <script>
                const socket = io();
                let logCount = 0;
                let errorCount = 0;
                let statusData = {};

                // Enhanced connection handlers
                socket.on('connect', function() {
                    updateConnectionStatus('connected');
                    document.getElementById('nodeStatus').textContent = 'Connected';
                    document.getElementById('rosStatusIcon').className = 'status-icon status-active';
                    document.getElementById('rosStatus').textContent = 'Connected';
                    console.log('WebSocket connected:', new Date().toISOString());
                });

                socket.on('disconnect', function() {
                    updateConnectionStatus('disconnected');
                    document.getElementById('nodeStatus').textContent = 'Disconnected';
                    document.getElementById('rosStatusIcon').className = 'status-icon status-error';
                    document.getElementById('rosStatus').textContent = 'Disconnected';
                    console.log('WebSocket disconnected:', new Date().toISOString());
                });

                socket.on('new_log', function(log) {
                    addLogEntry(log);
                    logCount++;
                    if (log.level === 'ERROR') {
                        errorCount++;
                    }
                    updateStats();
                });

                socket.on('status_update', function(status) {
                    statusData = { ...statusData, ...status };
                    updateSystemStatus();
                });

                socket.on('logs_cleared', function() {
                    document.getElementById('logContainer').innerHTML = '';
                    logCount = 0;
                    errorCount = 0;
                    updateStats();
                });

                function addLogEntry(log) {
                    const container = document.getElementById('logContainer');
                    const entry = document.createElement('div');
                    entry.className = 'log-entry';
                    
                    entry.innerHTML = `
                        <span class="log-timestamp">[${log.time_str}]</span>
                        <span class="log-level log-level-${log.level}">${log.level}</span>
                        <span class="log-source">${log.source}</span>
                        <span class="log-message">${log.message}</span>
                    `;
                    
                    container.appendChild(entry);
                    container.scrollTop = container.scrollHeight;
                    
                    while (container.children.length > 300) {
                        container.removeChild(container.firstChild);
                    }
                }

                function updateStats() {
                    document.getElementById('logCount').textContent = logCount;
                    document.getElementById('errorCount').textContent = errorCount;
                }

                function updateSystemStatus() {
                    const statusContainer = document.getElementById('statusItems');
                    
                    Object.keys(statusData).forEach(key => {
                        let statusItem = document.getElementById(`status-${key}`);
                        if (!statusItem) {
                            statusItem = document.createElement('div');
                            statusItem.className = 'system-status-item';
                            statusItem.id = `status-${key}`;
                            statusContainer.appendChild(statusItem);
                        }
                        
                        const value = statusData[key];
                        let content = `<h4>${key}</h4>`;
                        
                        if (typeof value === 'object') {
                            Object.keys(value).forEach(subKey => {
                                content += `<p>${subKey}: ${value[subKey]}</p>`;
                            });
                        } else {
                            content += `<p>Value: ${value}</p>`;
                        }
                        
                        statusItem.innerHTML = content;
                    });
                }

                function refreshStreams() {
                    const timestamp = new Date().getTime();
                    document.getElementById('cameraStream').src = '/camera_feed?' + timestamp;
                    document.getElementById('detectionStream').src = '/detection_feed?' + timestamp;
                }

                // Enhanced checkStatus with proper error handling
                function checkStatus() {
                    fetch('/status')
                        .then(response => {
                            if (!response.ok) {
                                throw new Error(`HTTP error! status: ${response.status}`);
                            }
                            
                            const contentType = response.headers.get('content-type');
                            if (!contentType || !contentType.includes('application/json')) {
                                throw new Error('Response is not JSON');
                            }
                            
                            return response.json();
                        })
                        .then(data => {
                            console.log('Status data received:', {
                                timestamp: new Date().toISOString(),
                                camera: data.camera_status,
                                detection: data.detection_status,
                                frames: data.camera_frames + data.detection_frames
                            });
                            
                            updateStatus(data);
                        })
                        .catch(error => {
                            console.error('Status check failed:', {
                                error: error.message,
                                timestamp: new Date().toISOString()
                            });
                            updateErrorStatus();
                        });
                }

                function updateStatus(data) {
                    try {
                        if (!data || typeof data !== 'object') {
                            throw new Error('Invalid status data');
                        }
                        
                        const cameraIcon = document.getElementById('cameraStatusIcon');
                        const detectionIcon = document.getElementById('detectionStatusIcon');
                        
                        if (cameraIcon && detectionIcon) {
                            cameraIcon.className = `status-icon ${data.camera_status === 'active' ? 'status-active' : 'status-waiting'}`;
                            detectionIcon.className = `status-icon ${data.detection_status === 'active' ? 'status-active' : 'status-waiting'}`;
                        }
                        
                        const cameraStatus = document.getElementById('cameraStatus');
                        const detectionStatus = document.getElementById('detectionStatus');
                        const totalFrames = document.getElementById('totalFrames');
                        
                        if (cameraStatus) {
                            cameraStatus.textContent = data.camera_status === 'active' ? 'Active' : 'Waiting';
                        }
                        if (detectionStatus) {
                            detectionStatus.textContent = data.detection_status === 'active' ? 'Active' : 'Waiting';
                        }
                        if (totalFrames) {
                            const total = (data.camera_frames || 0) + (data.detection_frames || 0);
                            totalFrames.textContent = total.toLocaleString();
                        }
                        
                        updateConnectionStatus('connected');
                        
                    } catch (error) {
                        console.error('Error updating status UI:', error);
                    }
                }

                function updateConnectionStatus(status) {
                    const statusElement = document.getElementById('connectionStatus');
                    const textElement = document.getElementById('connectionText');
                    
                    if (statusElement && textElement) {
                        if (status === 'connected') {
                            statusElement.className = 'connection-status status-connected';
                            textElement.textContent = 'Connected';
                        } else {
                            statusElement.className = 'connection-status status-disconnected';
                            textElement.textContent = 'Disconnected';
                        }
                    }
                }

                function updateErrorStatus() {
                    const elements = {
                        cameraStatusIcon: document.getElementById('cameraStatusIcon'),
                        detectionStatusIcon: document.getElementById('detectionStatusIcon'),
                        cameraStatus: document.getElementById('cameraStatus'),
                        detectionStatus: document.getElementById('detectionStatus')
                    };
                    
                    if (elements.cameraStatusIcon) {
                        elements.cameraStatusIcon.className = 'status-icon status-error';
                    }
                    if (elements.detectionStatusIcon) {
                        elements.detectionStatusIcon.className = 'status-icon status-error';
                    }
                    
                    if (elements.cameraStatus) {
                        elements.cameraStatus.textContent = 'Error';
                    }
                    if (elements.detectionStatus) {
                        elements.detectionStatus.textContent = 'Error';
                    }
                    
                    updateConnectionStatus('disconnected');
                }

                function toggleFullscreen(elementId) {
                    const element = document.getElementById(elementId);
                    if (element.requestFullscreen) {
                        element.requestFullscreen();
                    } else if (element.webkitRequestFullscreen) {
                        element.webkitRequestFullscreen();
                    } else if (element.mozRequestFullScreen) {
                        element.mozRequestFullScreen();
                    }
                }

                function clearLogs() {
                    socket.emit('clear_logs');
                }

                function exportLogs() {
                    fetch('/api/logs')
                        .then(response => response.json())
                        .then(data => {
                            const logs = data.logs || [];
                            const logText = logs.map(log => 
                                `[${log.time_str}] ${log.level} ${log.source}: ${log.message}`
                            ).join('\\n');
                            
                            const blob = new Blob([logText], { type: 'text/plain' });
                            const url = window.URL.createObjectURL(blob);
                            const a = document.createElement('a');
                            a.href = url;
                            a.download = `jabari-logs-${new Date().toISOString().split('T')[0]}.txt`;
                            a.click();
                            window.URL.revokeObjectURL(url);
                        });
                }

                // Tab visibility detection
                document.addEventListener('visibilitychange', function() {
                    if (document.hidden) {
                        console.log('Tab hidden - pausing status checks');
                    } else {
                        console.log('Tab visible - resuming status checks');
                        checkStatus();
                    }
                });

                window.addEventListener('load', function() {
                    console.log('Page loaded - starting status monitoring');
                    checkStatus();
                });

                // Periodic status check
                setInterval(checkStatus, 5000);
            </script>
        </body>
        </html>
        ''')


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize Flask-SocketIO
    app = Flask(__name__)
    app.config['SECRET_KEY'] = 'jabari_camera_streamer_secret'
    socketio = SocketIO(app, cors_allowed_origins="*")
    
    # Create node with socketio support
    node = DualWebStreamer(socketio)
    
    # Set the Flask app to the node
    node.app = app
    
    # Setup routes on the main Flask app
    app.add_url_rule('/camera_feed', 'camera_feed', node.camera_feed)
    app.add_url_rule('/detection_feed', 'detection_feed', node.detection_feed)
    app.add_url_rule('/combined_feed', 'combined_feed', node.combined_feed)
    app.add_url_rule('/', 'index', node.index)
    app.add_url_rule('/status', 'status', node.status)
    app.add_url_rule('/health', 'health', node.health)
    app.add_url_rule('/api/logs', 'get_logs', node.get_logs)
    app.add_url_rule('/api/status', 'get_status', node.get_status)
    
    # Add SocketIO event handlers
    @socketio.on('connect')
    def handle_connect():
        node.get_logger().info(f"Web client connected: {request.sid}")
        recent_logs = node.get_recent_logs(50)
        for log_entry in recent_logs:
            emit('new_log', log_entry)
        if node.system_status:
            emit('status_update', node.system_status)

    @socketio.on('disconnect')
    def handle_disconnect():
        node.get_logger().info(f"Web client disconnected: {request.sid}")

    @socketio.on('clear_logs')
    def handle_clear_logs():
        node.log_buffer.clear()
        socketio.start_background_task(socketio.emit, 'logs_cleared')
    
    # Start Flask-SocketIO in a separate thread
    flask_thread = threading.Thread(
        target=lambda: socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    )
    flask_thread.daemon = True
    flask_thread.start()
    
    # Get local IP address
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except:
        local_ip = "localhost"
    
    print("\n" + "="*60)
    print("✅ ULTIMATE WEB STREAMER STARTED!")
    print("="*60)
    print(f"📱 Dashboard:       http://{local_ip}:5000/")
    print(f"🎥 Camera Feed:     http://{local_ip}:5000/camera_feed")
    print(f"🤖 Detection Feed:  http://{local_ip}:5000/detection_feed")
    print(f"🎭 Combined View:   http://{local_ip}:5000/combined_feed")
    print(f"❤️  Health Check:    http://{local_ip}:5000/health")
    print(f"📊 Status API:      http://{local_ip}:5000/status")
    print(f"📝 Logs API:        http://{local_ip}:5000/api/logs")
    print("="*60)
    print("\n🚀 Features Enabled:")
    print("   ✓ Real-time video streaming (compressed images)")
    print("   ✓ WebSocket-based live log monitoring")
    print("   ✓ System status dashboard")
    print("   ✓ Motor speed tracking")
    print("   ✓ Object detection monitoring")
    print("   ✓ Movement command logging")
    print("   ✓ Servo position tracking")
    print("   ✓ Health check endpoint")
    print("   ✓ Enhanced error handling")
    print("   ✓ CORS support")
    print("   ✓ Cache control headers")
    print("\n⚙️  Subscribed Topics:")
    print("   • /camera/image_raw/compressed")
    print("   • /yolo/annotated/compressed")
    print("   • /manual_control_status")
    print("   • /cmd_vel")
    print("   • /vector3_cmd")
    print("   • /motor/left_speed")
    print("   • /motor/right_speed")
    print("   • /yolo/detections_log")
    print("\n💡 Quick Commands:")
    print("   Test Health:  curl http://localhost:5000/health")
    print("   Test Status:  curl http://localhost:5000/status")
    print("   View Logs:    curl http://localhost:5000/api/logs?count=10")
    print("="*60 + "\n")
    
    try:
        node.get_logger().info("ROS2 Web Streamer is spinning...")
        rclpy.logging.get_logger('dual_web_streamer').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" + "="*60)
        print("🛑 Shutting down web streamer...")
        print("="*60)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
