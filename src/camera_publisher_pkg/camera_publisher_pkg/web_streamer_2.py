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
        <html>
        <head>
            <title>ROS2 Dual Camera Stream</title>
            <style>
                body { 
                    font-family: Arial, sans-serif; 
                    background-color: #f0f0f0;
                    margin: 0;
                    padding: 20px;
                }
                .container {
                    max-width: 1200px;
                    margin: 0 auto;
                    background: white;
                    padding: 20px;
                    border-radius: 10px;
                    box-shadow: 0 2px 10px rgba(0,0,0,0.1);
                }
                h1 { 
                    color: #333; 
                    text-align: center;
                    margin-bottom: 10px;
                }
                .status {
                    text-align: center;
                    margin: 10px 0 20px 0;
                    padding: 10px;
                    background-color: #e8f5e8;
                    border-radius: 5px;
                    font-weight: bold;
                }
                .streams-container {
                    display: flex;
                    gap: 20px;
                    justify-content: center;
                    align-items: flex-start;
                    flex-wrap: wrap;
                }
                .stream-section {
                    text-align: center;
                    margin: 10px 0;
                }
                .stream-section h3 {
                    margin: 0 0 10px 0;
                    color: #555;
                }
                .camera-stream {
                    flex: 2;
                    min-width: 400px;
                }
                .detection-stream {
                    flex: 1;
                    min-width: 300px;
                }
                .camera-stream img {
                    max-width: 100%;
                    height: auto;
                    border: 3px solid #2196F3;
                    border-radius: 8px;
                    box-shadow: 0 4px 8px rgba(0,0,0,0.2);
                }
                .detection-stream img {
                    max-width: 100%;
                    height: auto;
                    border: 3px solid #FF9800;
                    border-radius: 8px;
                    box-shadow: 0 4px 8px rgba(0,0,0,0.2);
                }
                .controls {
                    text-align: center;
                    margin: 30px 0;
                }
                button {
                    background-color: #4CAF50;
                    color: white;
                    padding: 12px 24px;
                    border: none;
                    border-radius: 5px;
                    cursor: pointer;
                    margin: 0 10px;
                    font-size: 16px;
                }
                button:hover {
                    background-color: #45a049;
                }
                .legend {
                    display: flex;
                    justify-content: center;
                    gap: 30px;
                    margin: 20px 0;
                    font-size: 14px;
                }
                .legend-item {
                    display: flex;
                    align-items: center;
                    gap: 8px;
                }
                .legend-color {
                    width: 20px;
                    height: 20px;
                    border-radius: 3px;
                }
                .camera-color { background-color: #2196F3; }
                .detection-color { background-color: #FF9800; }
                
                @media (max-width: 768px) {
                    .streams-container {
                        flex-direction: column;
                        align-items: center;
                    }
                    .camera-stream, .detection-stream {
                        width: 100%;
                        min-width: unset;
                    }
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>ROS2 Dual Camera Stream</h1>
                <div class="status" id="status">Loading...</div>
                
                <div class="legend">
                    <div class="legend-item">
                        <div class="legend-color camera-color"></div>
                        <span>Raw Camera Feed (Smooth)</span>
                    </div>
                    <div class="legend-item">
                        <div class="legend-color detection-color"></div>
                        <span>Object Detection (AI Analysis)</span>
                    </div>
                </div>
                
                <div class="streams-container">
                    <div class="stream-section camera-stream">
                        <h3>🎥 Live Camera Feed</h3>
                        <img src="/camera_feed" alt="Camera Feed" id="cameraStream">
                    </div>
                    
                    <div class="stream-section detection-stream">
                        <h3>🤖 Object Detection</h3>
                        <img src="/detection_feed" alt="Detection Feed" id="detectionStream">
                    </div>
                </div>
                
                <div class="controls">
                    <button onclick="refreshStreams()">🔄 Refresh Streams</button>
                    <button onclick="checkStatus()">📊 Check Status</button>
                </div>
            </div>
            
            <script>
                function refreshStreams() {
                    const timestamp = new Date().getTime();
                    document.getElementById('cameraStream').src = '/camera_feed?' + timestamp;
                    document.getElementById('detectionStream').src = '/detection_feed?' + timestamp;
                }
                
                function checkStatus() {
                    fetch('/status')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('status').innerHTML = 
                                `${data.message}<br>Camera Frames: ${data.camera_frames} | Detection Frames: ${data.detection_frames}`;
                        })
                        .catch(error => {
                            document.getElementById('status').textContent = 'Error checking status';
                        });
                }
                
                // Check status periodically
                setInterval(checkStatus, 3000);
                
                // Check status on page load
                window.onload = function() {
                    checkStatus();
                };
                
                // Auto-refresh detection stream less frequently to account for slower updates
                setInterval(function() {
                    const timestamp = new Date().getTime();
                    document.getElementById('detectionStream').src = '/detection_feed?' + timestamp;
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
