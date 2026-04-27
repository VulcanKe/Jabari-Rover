#!/usr/bin/env python3
"""
Merged Dual Web Streamer
Combines Document 1 (feature-rich SocketIO + logs) with Document 2 (headers, CORS, safer status).
Save as: dual_web_streamer_merged.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
import threading
from flask import Flask, Response, render_template_string, request, jsonify, abort, redirect
from flask_socketio import SocketIO, emit
from flask_login import (
    LoginManager, UserMixin,
    login_user, login_required,
    logout_user, current_user
)
import time
import logging
import json
from datetime import datetime
from collections import deque
from typing import Dict, List, Optional
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Vector3

# -----------------------------------------------------------------------------
# Structured log entry for web display (from Document 1)
class LogEntry:
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

# -----------------------------------------------------------------------------
class DualWebStreamer(Node):
    def __init__(self, socketio):
        super().__init__('dual_web_streamer')
        self.bridge = CvBridge()
        self.socketio = socketio

        # log and status
        self.log_buffer = deque(maxlen=1000)
        self.system_status = {}

        # frames + locks
        self.camera_frame = None
        self.detection_frame = None
        self.camera_lock = threading.Lock()
        self.detection_lock = threading.Lock()

        self.camera_frame_count = 0
        self.detection_frame_count = 0

        # Choose QoS settings appropriate for compressed camera publishers
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

        # Subscriptions to compressed topics (Document 1 style decoding)
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_callback,
            camera_qos
        )
        self.detection_subscription = self.create_subscription(
            CompressedImage,
            '/yolo/annotated/compressed',
            self.detection_callback,
            detection_qos
        )

        # Optional telemetry subscriptions (from Document 1)
        self.status_sub = self.create_subscription(String, '/manual_control_status', self.status_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.vector3_sub = self.create_subscription(Vector3, '/vector3_cmd', self.vector3_callback, 10)
        self.left_motor_sub = self.create_subscription(Float32, '/motor/left_speed', self.left_motor_callback, 10)
        self.right_motor_sub = self.create_subscription(Float32, '/motor/right_speed', self.right_motor_callback, 10)
        self.object_info_sub = self.create_subscription(String, '/yolo/detections_log', self.object_detection_callback, 10)

        self.motor_status = {'left_speed': 0.0, 'right_speed': 0.0}
        self.last_object_detection = None

        # logging capture
        self.setup_log_capture()

        # periodic status emission
        threading.Thread(target=self._periodic_status_update, daemon=True).start()
        self.get_logger().info('Dual web streamer initialized.')

    # -------------------------------------------------------------------------
    def _periodic_status_update(self):
        """Send periodic status updates to connected web clients (SocketIO)."""
        while rclpy.ok():
            try:
                status = {
                    'camera_status': 'active' if self.camera_frame is not None else 'waiting',
                    'detection_status': 'active' if self.detection_frame is not None else 'waiting',
                    'camera_frames': self.camera_frame_count,
                    'detection_frames': self.detection_frame_count,
                    'timestamp': time.time()
                }
                # update internal snapshot
                self.system_status.update(status)
                # emit in background
                self.socketio.start_background_task(self.socketio.emit, 'status_update', status)
                time.sleep(5)
            except Exception as e:
                self.get_logger().warn(f"Status update thread error: {e}")
                time.sleep(5)

    # -------------------------------------------------------------------------
    def setup_log_capture(self):
        """Setup custom log handler to capture Python/RCL logs into the buffer."""
        class WebLogHandler(logging.Handler):
            def __init__(self, monitor):
                super().__init__()
                self.monitor = monitor
            def emit(self, record):
                try:
                    level = record.levelname
                    source = getattr(record, 'name', 'unknown')
                    message = record.getMessage()
                    log_entry = LogEntry(timestamp=time.time(), level=level, source=source, message=message)
                    self.monitor.add_log_entry(log_entry)
                except Exception:
                    pass

        handler = WebLogHandler(self)
        logging.getLogger().addHandler(handler)

    # -------------------------------------------------------------------------
    def add_log_entry(self, log_entry: LogEntry):
        """Add log entry to buffer and notify web clients via SocketIO."""
        self.log_buffer.append(log_entry)
        self.socketio.start_background_task(self.socketio.emit, 'new_log', log_entry.to_dict())

    def get_recent_logs(self, count: int = 100) -> List[Dict]:
        return [entry.to_dict() for entry in list(self.log_buffer)[-count:]]

    # -------------------------------------------------------------------------
    # ROS callbacks (camera/detection decode using numpy+cv2 for CompressedImage)
    def camera_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().warn('Failed to decode compressed camera image.')
                return
            with self.camera_lock:
                self.camera_frame = cv_image
                self.camera_frame_count += 1

            if self.camera_frame_count % 30 == 0:
                h, w = cv_image.shape[:2]
                self.add_log_entry(LogEntry(time.time(), 'DEBUG', 'camera', f'Camera frame #{self.camera_frame_count} - {w}x{h}', data={'frame_count': self.camera_frame_count}))
                self.system_status['camera'] = {'frame_count': self.camera_frame_count, 'resolution': f"{w}x{h}", 'last_frame_time': time.time()}
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def detection_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().warn('Failed to decode compressed detection image.')
                return
            with self.detection_lock:
                self.detection_frame = cv_image
                self.detection_frame_count += 1

            h, w = cv_image.shape[:2]
            self.add_log_entry(LogEntry(time.time(), 'DEBUG', 'yolo_processor', f'Annotated image received - {w}x{h}', data={'width': w, 'height': h}))
            self.system_status['last_detection'] = {'count': self.detection_frame_count, 'timestamp': time.time()}
        except Exception as e:
            self.get_logger().error(f'Error processing detection image: {e}')

    # -------------------------------------------------------------------------
    # Movement + telemetry callbacks (keep light touch — from Document 1)
    def status_callback(self, msg: String):
        try:
            status_data = json.loads(msg.data)
            self.system_status.update(status_data)
            self.add_log_entry(LogEntry(time.time(), 'INFO', 'manual_control', 'Status update received', data=status_data))
            self.socketio.start_background_task(self.socketio.emit, 'status_update', status_data)
        except Exception as e:
            self.get_logger().error(f"Error processing status: {e}")

    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.add_log_entry(LogEntry(time.time(), 'INFO', 'movement', f'linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}', data={'linear_x': msg.linear.x, 'angular_z': msg.angular.z}))

    def vector3_callback(self, msg: Vector3):
        self.add_log_entry(LogEntry(time.time(), 'INFO', 'servos', f'pan={msg.x:.1f}, tilt={msg.y:.1f}', data={'pan': msg.x, 'tilt': msg.y}))

    def left_motor_callback(self, msg: Float32):
        self.motor_status['left_speed'] = msg.data
        self.add_log_entry(LogEntry(time.time(), 'INFO', 'motor_controller', f'Left motor speed: {msg.data:.2f}', data={'left_speed': msg.data}))
        self.system_status['motor_left'] = msg.data
        self.socketio.start_background_task(self.socketio.emit, 'status_update', {'motor_left': msg.data})

    def right_motor_callback(self, msg: Float32):
        self.motor_status['right_speed'] = msg.data
        self.add_log_entry(LogEntry(time.time(), 'INFO', 'motor_controller', f'Right motor speed: {msg.data:.2f}', data={'right_speed': msg.data}))
        self.system_status['motor_right'] = msg.data
        self.socketio.start_background_task(self.socketio.emit, 'status_update', {'motor_right': msg.data})

    def object_detection_callback(self, msg: String):
        if msg.data and msg.data != self.last_object_detection:
            self.last_object_detection = msg.data
            det_count = msg.data.count('Object:')
            self.add_log_entry(LogEntry(time.time(), 'INFO', 'object_detection', f'Detected {det_count} objects', data={'full_detection': msg.data}))
            self.system_status['last_detection'] = {'count': det_count, 'timestamp': time.time()}
            self.socketio.start_background_task(self.socketio.emit, 'status_update', {'object_detection': self.system_status['last_detection']})

    # -------------------------------------------------------------------------
    # Frame generators (from both documents; keep good defaults)
    def generate_camera_frames(self):
        last_frame_count = 0
        while True:
            frame_to_send = None
            current_frame_count = 0
            with self.camera_lock:
                if self.camera_frame is not None and self.camera_frame_count > last_frame_count:
                    frame_to_send = self.camera_frame.copy()
                    current_frame_count = self.camera_frame_count
            if frame_to_send is not None:
                h, w = frame_to_send.shape[:2]
                if w > 800:
                    new_w = 800
                    new_h = int(h * (800 / w))
                    frame_to_send = cv2.resize(frame_to_send, (new_w, new_h))
                ret, buffer = cv2.imencode('.jpg', frame_to_send, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                    last_frame_count = current_frame_count
            else:
                placeholder = self.create_camera_placeholder()
                ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.05)

    def generate_detection_frames(self):
        last_frame_count = 0
        last_valid_frame = None
        while True:
            frame_to_send = None
            with self.detection_lock:
                if self.detection_frame is not None:
                    if self.detection_frame_count > last_frame_count:
                        frame_to_send = self.detection_frame.copy()
                        last_frame_count = self.detection_frame_count
                        last_valid_frame = frame_to_send.copy()
                    elif last_valid_frame is not None:
                        frame_to_send = last_valid_frame.copy()
            if frame_to_send is not None:
                h, w = frame_to_send.shape[:2]
                new_w = 400
                new_h = int(h * (400 / w))
                frame_to_send = cv2.resize(frame_to_send, (new_w, new_h))
                ret, buffer = cv2.imencode('.jpg', frame_to_send, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            else:
                placeholder = self.create_detection_placeholder()
                ret, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.2)

    def generate_combined_frames(self):
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
                combined = self.create_combined_frame(camera_frame, detection_frame)
            else:
                combined = self.create_combined_placeholder()
            ret, buffer = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.1)

    # -------------------------------------------------------------------------
    # helpers: create placeholders & combined image
    def create_combined_frame(self, camera_frame, detection_frame=None):
        camera_resized = cv2.resize(camera_frame, (640, 480))
        if detection_frame is not None:
            detection_resized = cv2.resize(detection_frame, (640, 480))
        else:
            detection_resized = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(detection_resized, 'Waiting for AI...', (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        combined = np.hstack((camera_resized, detection_resized))
        cv2.putText(combined, 'CAMERA FEED', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv2.putText(combined, 'AI DETECTION', (660, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,165,0), 2)
        cv2.line(combined, (640,0), (640,480), (100,100,100), 2)
        return combined

    def create_combined_placeholder(self):
        frame = np.zeros((480, 1280, 3), dtype=np.uint8)
        cv2.putText(frame, 'Waiting for camera...', (480, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.line(frame, (640,0), (640,480), (100,100,100), 2)
        return frame

    def create_camera_placeholder(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, 'Waiting for camera...', (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        return frame

    def create_detection_placeholder(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.putText(frame, 'Waiting for', (90, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(frame, 'detections...', (70, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        return frame

    # -------------------------------------------------------------------------
    # Flask endpoints (status with headers & CORS from Document 2)
    def camera_feed(self):
        response = Response(self.generate_camera_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response

    def detection_feed(self):
        response = Response(self.generate_detection_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response

    def combined_feed(self):
        response = Response(self.generate_combined_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response

    def status(self):
        """Status endpoint — returns JSON with headers and CORS (Document 2 style)"""
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
            'message': f'Camera: {"✓" if has_camera else "⏳"} | Detection: {"✓" if has_detection else "⏳"}'
        }
        response = jsonify(response_data)
        # Prevent caching and allow CORS for simple dashboard fetches
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response

    def health(self):
        """Minimal health endpoint for external checks"""
        return ('OK', 200, {'Content-Type': 'text/plain', 'Cache-Control': 'no-cache'})

    def index(self):
        """Render a compact dashboard (Document 1 UI but with improved JS error handling)"""
        return render_template_string("""
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Jabari Dual Streamer</title>
  <style>body{font-family:Arial,Helvetica,sans-serif;background:#0f1419;color:#fff;margin:0;padding:0}</style>
</head>
<body>
  <header style="padding:1rem;background:#111;display:flex;justify-content:space-between;align-items:center">
    <div style="display:flex;gap:.5rem;align-items:center"><div>🚀</div><h3>Jabari Stream</h3></div>
    <div id="conn" style="font-weight:600">Initializing...</div>
  </header>

  <main style="padding:1rem;">
    <section style="display:grid;grid-template-columns:1fr 1fr;gap:1rem">
      <div style="background:#222;padding:.5rem;border-radius:8px">
        <h4>Live Camera</h4>
        <img id="cameraStream" src="/camera_feed" style="width:100%;background:#000" />
      </div>
      <div style="background:#222;padding:.5rem;border-radius:8px">
        <h4>AI Detection</h4>
        <img id="detectionStream" src="/detection_feed" style="width:100%;background:#000" />
      </div>
    </section>

    <section style="margin-top:1rem;background:#222;padding:.75rem;border-radius:8px">
      <button onclick="refreshStreams()">Refresh Streams</button>
      <button onclick="checkStatus()">Check Status</button>
      <span id="statusText" style="margin-left:1rem">—</span>
    </section>

    <section style="margin-top:1rem">
      <h4>Logs</h4>
      <div id="logContainer" style="height:220px;overflow:auto;background:#111;padding:.5rem;border-radius:6px;font-family:monospace;font-size:.85rem">Waiting for logs...</div>
    </section>
  </main>

<script>
  // Socket.IO connection
  const socket = io();

  socket.on('connect', () => {
    document.getElementById('conn').textContent = 'Connected';
    console.log('Socket connected');
  });

  socket.on('disconnect', () => {
    document.getElementById('conn').textContent = 'Disconnected';
    console.warn('Socket disconnected');
  });

  socket.on('new_log', (log) => {
    addLogEntry(log);
    console.debug('New log:', log);
  });

  socket.on('status_update', (s) => {
    console.debug('Status update:', s);
    document.getElementById('statusText').textContent = `Camera: ${s.camera_status} | Detection: ${s.detection_status}`;
  });

  function addLogEntry(log) {
    const c = document.getElementById('logContainer');
    if (!c) return;
    const div = document.createElement('div');
    div.textContent = `[${log.time_str}] ${log.level} ${log.source}: ${log.message}`;
    c.appendChild(div);
    c.scrollTop = c.scrollHeight;
    // limit to last ~500 lines
    while (c.children.length > 500) c.removeChild(c.firstChild);
  }

  // Robust fetch with status check
  async function checkStatus() {
    try {
      const res = await fetch('/status');
      if (!res.ok) {
        throw new Error('HTTP error! status: ' + res.status);
      }
      const data = await res.json();
      console.log('Status data received:', data);
      document.getElementById('statusText').textContent = `Camera: ${data.camera_status} | Detection: ${data.detection_status} (${data.camera_frames}+${data.detection_frames})`;
    } catch (err) {
      console.error('Error checking status:', err);
      document.getElementById('statusText').textContent = 'Status fetch error';
    }
  }

  function refreshStreams(){
    const t = Date.now();
    document.getElementById('cameraStream').src = '/camera_feed?_=' + t;
    document.getElementById('detectionStream').src = '/detection_feed?_=' + t;
    console.log('Streams refreshed');
  }

  // Tab visibility: pause frequent refreshes when hidden (example use)
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      console.log('Page hidden - deferring heavy UI updates');
      // e.g., you could stop periodic status checks or reduce update frequency
    } else {
      console.log('Page visible - resuming UI updates');
      checkStatus();
    }
  });

  // Periodically check status (conservative)
  setInterval(checkStatus, 5000);

  // Commented: optional auto-refresh every 10s (disabled to reduce load)
  // setInterval(() => {
  //   const ts = Date.now();
  //   document.getElementById('detectionStream').src = '/detection_feed?ts=' + ts;
  //   document.getElementById('cameraStream').src = '/camera_feed?ts=' + ts;
  // }, 10000);
</script>
</body>
</html>
        """)

    # -------------------------------------------------------------------------
    # API endpoints for logs and system status
    def get_logs(self):
        # Validate query content-type or params optionally (defensive)
        count = request.args.get('count', 100, type=int)
        level_filter = request.args.get('level', '')
        source_filter = request.args.get('source', '')
        logs = self.get_recent_logs(count)
        if level_filter:
            logs = [log for log in logs if log['level'].upper() == level_filter.upper()]
        if source_filter:
            logs = [log for log in logs if source_filter.lower() in log['source'].lower()]
        response = jsonify(logs)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response

    def get_status(self):
        # Return last known system_status snapshot
        response = jsonify(self.system_status)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response

# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    app = Flask(__name__)
    app.config['SECRET_KEY'] = 'jabari_camera_streamer_secret'

    # ---------------------------------------------------------------------
    # AUTHENTICATION SETUP (Phase 1)
    # ---------------------------------------------------------------------
    login_manager = LoginManager()
    login_manager.init_app(app)
    login_manager.login_view = 'login'

    USERS = {'admin': {'password': 'jabari123'}}  # change this password!

    class User(UserMixin):
        def __init__(self, username):
            self.id = username

    @login_manager.user_loader
    def load_user(username):
        if username in USERS:
            return User(username)
        return None

    socketio = SocketIO(app, cors_allowed_origins="*")

    node = DualWebStreamer(socketio)
    node.app = app

    # ---------------------------------------------------------------------
    # ROUTES (with login protection)
    # ---------------------------------------------------------------------
    app.add_url_rule('/', 'index', login_required(node.index))
    app.add_url_rule('/camera_feed', 'camera_feed', login_required(node.camera_feed))
    app.add_url_rule('/detection_feed', 'detection_feed', login_required(node.detection_feed))
    app.add_url_rule('/combined_feed', 'combined_feed', login_required(node.combined_feed))
    app.add_url_rule('/status', 'status', login_required(node.status))
    app.add_url_rule('/health', 'health', node.health)
    app.add_url_rule('/api/logs', 'get_logs', login_required(node.get_logs))
    app.add_url_rule('/api/status', 'get_status', node.get_status)

    # ---------------------------------------------------------------------
    # AUTH ROUTES
    # ---------------------------------------------------------------------
    @app.route('/login', methods=['GET', 'POST'])
    def login():
        if request.method == 'POST':
            username = request.form.get('username')
            password = request.form.get('password')
            user = USERS.get(username)
            if user and user['password'] == password:
                login_user(User(username))
                node.get_logger().info(f"User '{username}' logged in.")
                return redirect('/')
            else:
                return render_template_string("""
                    <h3>Login failed</h3>
                    <a href="/login">Try again</a>
                """)
        return render_template_string("""
            <html>
            <head><title>Login</title></head>
            <body style="font-family:Arial;background:#0f1419;color:#fff;display:flex;align-items:center;justify-content:center;height:100vh;">
                <form method="POST" style="background:#111;padding:2rem;border-radius:10px;">
                    <h3>🔒 Rover Dashboard Login</h3>
                    <p><input type="text" name="username" placeholder="Username" required></p>
                    <p><input type="password" name="password" placeholder="Password" required></p>
                    <p><button type="submit">Login</button></p>
                </form>
            </body>
            </html>
        """)

    @app.route('/logout')
    @login_required
    def logout():
        uname = current_user.id
        logout_user()
        node.get_logger().info(f"User '{uname}' logged out.")
        return render_template_string("""
            <h3>Logged out</h3>
            <a href="/login">Login again</a>
        """)

    # ---------------------------------------------------------------------
    # SOCKET.IO HANDLERS
    # ---------------------------------------------------------------------
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

    # Run Flask + SocketIO
    flask_thread = threading.Thread(
    target=lambda: socketio.run(app, host='0.0.0.0', port=5000, debug=False),
    daemon=True
    )

    flask_thread.start()

    # Local IP for display
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception:
        local_ip = "localhost"

    print(f"\n✅ Merged Web Streamer Started! Dashboard: http://{local_ip}:5000/\n")

    try:
        node.get_logger().info("ROS2 Web Streamer spinning...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down web streamer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
