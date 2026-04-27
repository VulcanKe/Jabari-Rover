#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
import threading
from flask import Flask, Response
import time
import logging

class DualWebStreamer(Node):
    def __init__(self):
        super().__init__('dual_web_streamer')
        self.bridge = CvBridge()
        
        # Separate frames for each stream
        self.camera_frame = None
        self.detection_frame = None
        self.camera_lock = threading.Lock()
        self.detection_lock = threading.Lock()
        
        self.camera_frame_count = 0
        self.detection_frame_count = 0
        
        # Match QoS profile with publishers
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscribe to both camera and detection topics
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            qos)
            
        self.detection_subscription = self.create_subscription(
            Image,
            '/image_annotated',
            self.detection_callback,
            qos)
        
        self.get_logger().info('Dual web streamer started. Waiting for camera and detection data...')
        
        # Flask app setup
        self.app = Flask(__name__)
        self.app.logger.setLevel(logging.WARNING)
        self.setup_routes()
        
    def setup_routes(self):
        """Set up Flask routes"""
        self.app.add_url_rule('/camera_feed', 'camera_feed', self.camera_feed)
        self.app.add_url_rule('/detection_feed', 'detection_feed', self.detection_feed)
        self.app.add_url_rule('/combined_feed', 'combined_feed', self.combined_feed)
        self.app.add_url_rule('/', 'index', self.index)
        self.app.add_url_rule('/status', 'status', self.status)
        
    def camera_callback(self, msg):
        """Callback for camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.camera_lock:
                self.camera_frame = cv_image
                self.camera_frame_count += 1
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')
            
    def detection_callback(self, msg):
        """Callback for detection images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.detection_lock:
                self.detection_frame = cv_image
                self.detection_frame_count += 1
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
                # Resize to larger size for main feed (800px width)
                height, width = frame_to_send.shape[:2]
                if width > 800:
                    new_width = 800
                    new_height = int(height * (800 / width))
                    frame_to_send = cv2.resize(frame_to_send, (new_width, new_height))
                
                # Higher quality for camera feed
                ret, buffer = cv2.imencode('.jpg', frame_to_send, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    last_frame_count = current_frame_count
            else:
                # Send a placeholder frame
                placeholder = self.create_camera_placeholder()
                ret, buffer = cv2.imencode('.jpg', placeholder, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            time.sleep(0.05)  # 20 FPS for smooth camera feed
            
    def generate_detection_frames(self):
        """Generate frames for detection streaming - keeps last frame alive"""
        last_frame_count = 0
        last_valid_frame = None
        
        while True:
            frame_to_send = None
            current_frame_count = 0
            
            with self.detection_lock:
                if self.detection_frame is not None:
                    if self.detection_frame_count > last_frame_count:
                        # New frame available - use it
                        frame_to_send = self.detection_frame.copy()
                        current_frame_count = self.detection_frame_count
                        last_frame_count = current_frame_count
                        # Store this as the last valid frame
                        last_valid_frame = frame_to_send.copy()
                    elif last_valid_frame is not None:
                        # No new frame, but we have a previous valid frame - keep showing it
                        frame_to_send = last_valid_frame.copy()
            
            if frame_to_send is not None:
                # Resize to smaller size for detection feed (400px width)
                height, width = frame_to_send.shape[:2]
                new_width = 400
                new_height = int(height * (400 / width))
                frame_to_send = cv2.resize(frame_to_send, (new_width, new_height))
                
                # Standard quality for detection feed
                ret, buffer = cv2.imencode('.jpg', frame_to_send, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                # Only send placeholder if we've never received a detection frame
                placeholder = self.create_detection_placeholder()
                ret, buffer = cv2.imencode('.jpg', placeholder, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            time.sleep(0.2)  # 5 FPS for detection feed (slower is fine)
    
    def generate_combined_frames(self):
        """Generate frames combining both camera and detection feeds"""
        last_camera_count = 0
        last_detection_count = 0
        last_valid_detection = None
        
        while True:
            camera_frame = None
            detection_frame = None
            
            # Get camera frame
            with self.camera_lock:
                if self.camera_frame is not None and self.camera_frame_count > last_camera_count:
                    camera_frame = self.camera_frame.copy()
                    last_camera_count = self.camera_frame_count
                elif self.camera_frame is not None:
                    camera_frame = self.camera_frame.copy()
            
            # Get detection frame
            with self.detection_lock:
                if self.detection_frame is not None:
                    if self.detection_frame_count > last_detection_count:
                        detection_frame = self.detection_frame.copy()
                        last_detection_count = self.detection_frame_count
                        last_valid_detection = detection_frame.copy()
                    elif last_valid_detection is not None:
                        detection_frame = last_valid_detection.copy()
            
            # Create combined frame
            if camera_frame is not None:
                combined_frame = self.create_combined_frame(camera_frame, detection_frame)
            else:
                combined_frame = self.create_combined_placeholder()
            
            # Encode and yield frame
            ret, buffer = cv2.imencode('.jpg', combined_frame, 
                                     [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ret:
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            time.sleep(0.1)  # 10 FPS for combined feed
    
    def create_combined_frame(self, camera_frame, detection_frame=None):
        """Create a combined frame with camera and detection side by side"""
        # Resize camera frame to standard size
        camera_resized = cv2.resize(camera_frame, (640, 480))
        
        if detection_frame is not None:
            # Resize detection frame to same size
            detection_resized = cv2.resize(detection_frame, (640, 480))
        else:
            # Create placeholder for detection
            detection_resized = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(detection_resized, 'Waiting for AI...', (200, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Create combined frame (side by side)
        combined = np.hstack((camera_resized, detection_resized))
        
        # Add labels
        cv2.putText(combined, 'CAMERA FEED', (20, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(combined, 'AI DETECTION', (660, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
        
        # Add separator line
        cv2.line(combined, (640, 0), (640, 480), (100, 100, 100), 2)
        
        return combined
    
    def create_combined_placeholder(self):
        """Create placeholder for combined feed when no camera is available"""
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
        """Status endpoint"""
        with self.camera_lock:
            has_camera = self.camera_frame is not None
            camera_count = self.camera_frame_count
            
        with self.detection_lock:
            has_detection = self.detection_frame is not None
            detection_count = self.detection_frame_count
        
        return {
            'camera_status': 'active' if has_camera else 'waiting',
            'detection_status': 'active' if has_detection else 'waiting',
            'camera_frames': camera_count,
            'detection_frames': detection_count,
            'message': f'Camera: {"✓" if has_camera else "⏳"} | Detection: {"✓" if has_detection else "⏳"}'
        }
    
    def index(self):
        """Main page with dual video streams"""
        return '''
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>Jabari ROS2 Camera System</title>
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
                    max-width: 1400px;
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
                    max-width: 1400px;
                    margin: 0 auto;
                    padding: 2rem;
                }

                .status-card {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    padding: 1.5rem;
                    margin-bottom: 2rem;
                    box-shadow: var(--shadow);
                }

                .status-grid {
                    display: grid;
                    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                    gap: 1rem;
                    margin-bottom: 1rem;
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

                .streams-container {
                    display: grid;
                    grid-template-columns: 2fr 1fr;
                    gap: 2rem;
                    margin-bottom: 2rem;
                }

                .combined-view {
                    margin-bottom: 2rem;
                }

                .stream-card {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    overflow: hidden;
                    box-shadow: var(--shadow);
                    transition: transform 0.3s ease, box-shadow 0.3s ease;
                }

                .stream-card:hover {
                    transform: translateY(-4px);
                    box-shadow: 0 20px 40px rgba(0, 0, 0, 0.4);
                }

                .stream-header {
                    padding: 1.5rem;
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
                    font-size: 1.1rem;
                    font-weight: 600;
                    color: var(--text-primary);
                }

                .stream-subtitle {
                    font-size: 0.85rem;
                    color: var(--text-muted);
                }

                .stream-content {
                    position: relative;
                    background: #000;
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
                    padding: 0.5rem 1rem;
                    border-radius: 8px;
                    font-size: 0.8rem;
                    font-weight: 500;
                }

                .controls-card {
                    background: var(--card-bg);
                    border: 1px solid var(--border-color);
                    border-radius: 16px;
                    padding: 1.5rem;
                    box-shadow: var(--shadow);
                }

                .controls-grid {
                    display: grid;
                    grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
                    gap: 1rem;
                }

                .btn {
                    background: linear-gradient(135deg, var(--accent-blue), #4dd0e1);
                    color: white;
                    border: none;
                    padding: 0.8rem 1.5rem;
                    border-radius: 12px;
                    font-size: 0.9rem;
                    font-weight: 500;
                    cursor: pointer;
                    transition: all 0.3s ease;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    gap: 8px;
                }

                .btn:hover {
                    transform: translateY(-2px);
                    box-shadow: 0 8px 25px rgba(0, 212, 255, 0.3);
                }

                .btn-secondary {
                    background: linear-gradient(135deg, var(--accent-orange), #ff8a65);
                }

                .btn-secondary:hover {
                    box-shadow: 0 8px 25px rgba(255, 107, 53, 0.3);
                }

                .footer {
                    text-align: center;
                    padding: 2rem;
                    color: var(--text-muted);
                    font-size: 0.9rem;
                    border-top: 1px solid var(--border-color);
                    margin-top: 2rem;
                }

                @media (max-width: 1024px) {
                    .streams-container {
                        grid-template-columns: 1fr;
                    }
                    
                    .header-content {
                        padding: 0 1rem;
                    }
                    
                    .container {
                        padding: 1rem;
                    }
                }

                @media (max-width: 768px) {
                    .status-grid {
                        grid-template-columns: 1fr;
                    }
                    
                    .controls-grid {
                        grid-template-columns: 1fr;
                    }
                    
                    h1 {
                        font-size: 1.2rem;
                    }
                }

                .loading {
                    opacity: 0.6;
                    animation: loading 1.5s infinite;
                }

                @keyframes loading {
                    0%, 100% { opacity: 0.6; }
                    50% { opacity: 0.3; }
                }
            </style>
        </head>
        <body>
            <header class="header">
                <div class="header-content">
                    <div class="logo">
                        <div class="logo-icon">🎥</div>
                        <h1>Jabari Camera System</h1>
                    </div>
                    <div class="status-text" id="headerStatus">Connecting...</div>
                </div>
            </header>

            <div class="container">
                <div class="status-card">
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
                                <div class="status-text">Detection System</div>
                                <div class="status-value" id="detectionStatus">Connecting...</div>
                            </div>
                        </div>
                        <div class="status-item">
                            <div class="status-icon status-waiting"></div>
                            <div>
                                <div class="status-text">Camera Frames</div>
                                <div class="status-value" id="cameraFrames">0</div>
                            </div>
                        </div>
                        <div class="status-item">
                            <div class="status-icon status-waiting"></div>
                            <div>
                                <div class="status-text">Detection Frames</div>
                                <div class="status-value" id="detectionFrames">0</div>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="streams-container">
                    <div class="stream-card">
                        <div class="stream-header">
                            <div class="stream-icon camera-icon">📹</div>
                            <div>
                                <div class="stream-title">Live Camera Feed</div>
                                <div class="stream-subtitle">Real-time video stream • 20 FPS</div>
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
                                <div class="stream-title">AI Detection</div>
                                <div class="stream-subtitle">Object detection overlay • 5 FPS</div>
                            </div>
                        </div>
                        <div class="stream-content">
                            <img src="/detection_feed" alt="Detection Feed" id="detectionStream" class="stream-image">
                            <div class="stream-overlay">🧠 AI</div>
                        </div>
                    </div>
                </div>
                
                <div class="stream-card combined-view">
                    <div class="stream-header">
                        <div class="stream-icon" style="background: linear-gradient(135deg, var(--accent-blue), var(--accent-orange));">🎭</div>
                        <div>
                            <div class="stream-title">Combined View</div>
                            <div class="stream-subtitle">Side-by-side camera and AI detection • 10 FPS</div>
                        </div>
                    </div>
                    <div class="stream-content">
                        <img src="/combined_feed" alt="Combined Feed" id="combinedStream" class="stream-image">
                        <div class="stream-overlay">📺 DUAL</div>
                    </div>
                </div>
                
                <div class="controls-card">
                    <div class="controls-grid">
                        <button class="btn" onclick="refreshStreams()">
                            🔄 Refresh All Streams
                        </button>
                        <button class="btn btn-secondary" onclick="checkStatus()">
                            📊 Check Status
                        </button>
                        <button class="btn" onclick="toggleFullscreen('cameraStream')">
                            🔍 Camera Fullscreen
                        </button>
                        <button class="btn btn-secondary" onclick="toggleFullscreen('detectionStream')">
                            🎯 Detection Fullscreen
                        </button>
                        <button class="btn" onclick="toggleFullscreen('combinedStream')">
                            🎭 Combined Fullscreen
                        </button>
                        <button class="btn btn-secondary" onclick="toggleView()">
                            📺 Toggle View Mode
                        </button>
                    </div>
                </div>
            </div>

            <footer class="footer">
                <p>Jabari ROS2 Camera System • Real-time streaming with AI detection</p>
            </footer>
            
            <script>
                let lastCameraFrames = 0;
                let lastDetectionFrames = 0;
                let showCombinedOnly = false;

                function refreshStreams() {
                    const timestamp = new Date().getTime();
                    const cameraImg = document.getElementById('cameraStream');
                    const detectionImg = document.getElementById('detectionStream');
                    const combinedImg = document.getElementById('combinedStream');
                    
                    cameraImg.classList.add('loading');
                    detectionImg.classList.add('loading');
                    combinedImg.classList.add('loading');
                    
                    cameraImg.src = '/camera_feed?' + timestamp;
                    detectionImg.src = '/detection_feed?' + timestamp;
                    combinedImg.src = '/combined_feed?' + timestamp;
                    
                    setTimeout(() => {
                        cameraImg.classList.remove('loading');
                        detectionImg.classList.remove('loading');
                        combinedImg.classList.remove('loading');
                    }, 1000);
                }

                function toggleView() {
                    const streamsContainer = document.querySelector('.streams-container');
                    const combinedView = document.querySelector('.combined-view');
                    
                    showCombinedOnly = !showCombinedOnly;
                    
                    if (showCombinedOnly) {
                        streamsContainer.style.display = 'none';
                        combinedView.style.display = 'block';
                    } else {
                        streamsContainer.style.display = 'grid';
                        combinedView.style.display = 'block';
                    }
                }
                
                function checkStatus() {
                    fetch('/status')
                        .then(response => response.json())
                        .then(data => {
                            updateStatus(data);
                        })
                        .catch(error => {
                            console.error('Error checking status:', error);
                            updateErrorStatus();
                        });
                }

                function updateStatus(data) {
                    // Update status indicators
                    const cameraIcon = document.getElementById('cameraStatusIcon');
                    const detectionIcon = document.getElementById('detectionStatusIcon');
                    
                    cameraIcon.className = `status-icon ${data.camera_status === 'active' ? 'status-active' : 'status-waiting'}`;
                    detectionIcon.className = `status-icon ${data.detection_status === 'active' ? 'status-active' : 'status-waiting'}`;
                    
                    // Update status text
                    document.getElementById('cameraStatus').textContent = data.camera_status === 'active' ? 'Active' : 'Waiting';
                    document.getElementById('detectionStatus').textContent = data.detection_status === 'active' ? 'Active' : 'Waiting';
                    document.getElementById('cameraFrames').textContent = data.camera_frames.toLocaleString();
                    document.getElementById('detectionFrames').textContent = data.detection_frames.toLocaleString();
                    
                    // Update header status
                    const headerStatus = document.getElementById('headerStatus');
                    if (data.camera_status === 'active' && data.detection_status === 'active') {
                        headerStatus.textContent = '🟢 All Systems Active';
                        headerStatus.style.color = 'var(--accent-green)';
                    } else if (data.camera_status === 'active') {
                        headerStatus.textContent = '🟡 Camera Active';
                        headerStatus.style.color = 'var(--accent-orange)';
                    } else {
                        headerStatus.textContent = '🔴 Connecting...';
                        headerStatus.style.color = 'var(--accent-orange)';
                    }
                }

                function updateErrorStatus() {
                    document.getElementById('headerStatus').textContent = '⚠️ Connection Error';
                    document.getElementById('headerStatus').style.color = 'var(--accent-orange)';
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

                // Error handling for images
                document.getElementById('cameraStream').onerror = function() {
                    this.style.opacity = '0.5';
                };

                document.getElementById('detectionStream').onerror = function() {
                    this.style.opacity = '0.5';
                };

                document.getElementById('combinedStream').onerror = function() {
                    this.style.opacity = '0.5';
                };

                document.getElementById('cameraStream').onload = function() {
                    this.style.opacity = '1';
                };

                document.getElementById('detectionStream').onload = function() {
                    this.style.opacity = '1';
                };

                document.getElementById('combinedStream').onload = function() {
                    this.style.opacity = '1';
                };
                
                // Check status periodically
                setInterval(checkStatus, 3000);
                
                // Check status on page load
                window.onload = function() {
                    checkStatus();
                };
                
                // Auto-refresh detection stream less frequently
                setInterval(function() {
                    const timestamp = new Date().getTime();
                    document.getElementById('detectionStream').src = '/detection_feed?' + timestamp;
                    document.getElementById('combinedStream').src = '/combined_feed?' + timestamp;
                }, 10000); // Refresh every 10 seconds
            </script>
        </body>
        </html>
        '''

def main(args=None):
    rclpy.init(args=args)
    node = DualWebStreamer()
    
    # Run Flask in a separate thread
    flask_thread = threading.Thread(
        target=lambda: node.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    )
    flask_thread.daemon = True
    flask_thread.start()
    
    print("Dual web streamer started!")
    print("Camera feed: http://your_pi_ip:5000/camera_feed")
    print("Detection feed: http://your_pi_ip:5000/detection_feed")
    print("Main page: http://your_pi_ip:5000")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down dual web streamer...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
