#!/bin/bash

# ROS2 Camera System Launcher
# This script runs camera_publisher.py and web_streamer.py simultaneously

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}[SYSTEM]${NC} $1"
}

# Function to cleanup processes on exit
cleanup() {
    print_header "Shutting down camera system..."
    
    # Kill background processes
    if [ ! -z "$CAMERA_PID" ]; then
        print_status "Stopping camera publisher (PID: $CAMERA_PID)"
        kill $CAMERA_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$STREAMER_PID" ]; then
        print_status "Stopping web streamer (PID: $STREAMER_PID)"
        kill $STREAMER_PID 2>/dev/null || true
    fi
    
    # Wait a moment for graceful shutdown
    sleep 1
    
    # Force kill if still running
    if [ ! -z "$CAMERA_PID" ] && kill -0 $CAMERA_PID 2>/dev/null; then
        print_warning "Force killing camera publisher"
        kill -9 $CAMERA_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$STREAMER_PID" ] && kill -0 $STREAMER_PID 2>/dev/null; then
        print_warning "Force killing web streamer"
        kill -9 $STREAMER_PID 2>/dev/null || true
    fi
    
    print_header "Camera system shutdown complete"
    exit 0
}

# Set trap to cleanup on script exit
trap cleanup SIGINT SIGTERM EXIT

# Configuration
CAMERA_SCRIPT="camera_publisher.py"
STREAMER_SCRIPT="web_streamer.py"
LOG_DIR="./logs"

# Create logs directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Get current timestamp for log files
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')

print_header "Starting ROS2 Camera System"
echo "========================================"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 environment not sourced!"
    print_status "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

print_status "ROS2 Distribution: $ROS_DISTRO"

# Check if scripts exist
if [ ! -f "$CAMERA_SCRIPT" ]; then
    print_error "Camera publisher script not found: $CAMERA_SCRIPT"
    exit 1
fi

if [ ! -f "$STREAMER_SCRIPT" ]; then
    print_error "Web streamer script not found: $STREAMER_SCRIPT"
    exit 1
fi

print_status "Both script files found"

# Make scripts executable
chmod +x "$CAMERA_SCRIPT" "$STREAMER_SCRIPT"

# Parse command line arguments for camera configuration
CAMERA_TYPE="webcam"
CAMERA_SOURCE="auto"
FRAME_RATE="30.0"

while [[ $# -gt 0 ]]; do
    case $1 in
        --camera-type)
            CAMERA_TYPE="$2"
            shift 2
            ;;
        --camera-source)
            CAMERA_SOURCE="$2"
            shift 2
            ;;
        --frame-rate)
            FRAME_RATE="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --camera-type TYPE     Camera type (webcam, droidcam, picam) [default: webcam]"
            echo "  --camera-source SRC    Camera source (auto, /dev/videoX, IP) [default: auto]"
            echo "  --frame-rate RATE      Frame rate in fps [default: 30.0]"
            echo "  --help                 Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Use default webcam"
            echo "  $0 --camera-type droidcam --camera-source http://192.168.1.100:4747/video"
            echo "  $0 --camera-source /dev/video1 --frame-rate 15.0"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            print_status "Use --help for usage information"
            exit 1
            ;;
    esac
done

print_status "Camera configuration:"
print_status "  Type: $CAMERA_TYPE"
print_status "  Source: $CAMERA_SOURCE"
print_status "  Frame rate: $FRAME_RATE fps"

echo ""
print_header "Starting camera publisher..."

# Start camera publisher with parameters
python3 "$CAMERA_SCRIPT" \
    --ros-args \
    -p camera_type:="$CAMERA_TYPE" \
    -p camera_source:="$CAMERA_SOURCE" \
    -p frame_rate:=$FRAME_RATE \
    > "$LOG_DIR/camera_publisher_$TIMESTAMP.log" 2>&1 &

CAMERA_PID=$!
print_status "Camera publisher started (PID: $CAMERA_PID)"

# Wait a moment for camera to initialize
sleep 3

# Check if camera publisher is still running
if ! kill -0 $CAMERA_PID 2>/dev/null; then
    print_error "Camera publisher failed to start!"
    print_status "Check log: $LOG_DIR/camera_publisher_$TIMESTAMP.log"
    exit 1
fi

print_header "Starting web streamer..."

# Start web streamer
python3 "$STREAMER_SCRIPT" \
    > "$LOG_DIR/web_streamer_$TIMESTAMP.log" 2>&1 &

STREAMER_PID=$!
print_status "Web streamer started (PID: $STREAMER_PID)"

# Wait a moment for web server to initialize
sleep 2

# Check if web streamer is still running
if ! kill -0 $STREAMER_PID 2>/dev/null; then
    print_error "Web streamer failed to start!"
    print_status "Check log: $LOG_DIR/web_streamer_$TIMESTAMP.log"
    exit 1
fi

echo ""
print_header "🎉 Camera system is running!"
echo "========================================"
print_status "Camera Publisher: Running (PID: $CAMERA_PID)"
print_status "Web Streamer: Running (PID: $STREAMER_PID)"
echo ""
print_status "Access the camera streams at:"
print_status "  🌐 Main page: http://$(hostname -I | awk '{print $1}'):5000"
print_status "  📹 Camera feed: http://$(hostname -I | awk '{print $1}'):5000/camera_feed"
print_status "  🤖 Detection feed: http://$(hostname -I | awk '{print $1}'):5000/detection_feed"
print_status "  📊 Status: http://$(hostname -I | awk '{print $1}'):5000/status"
echo ""
print_status "Log files:"
print_status "  📝 Camera: $LOG_DIR/camera_publisher_$TIMESTAMP.log"
print_status "  📝 Streamer: $LOG_DIR/web_streamer_$TIMESTAMP.log"
echo ""
print_warning "Press Ctrl+C to stop the system"

# Monitor both processes
while true; do
    sleep 5
    
    # Check if camera publisher is still running
    if ! kill -0 $CAMERA_PID 2>/dev/null; then
        print_error "Camera publisher stopped unexpectedly!"
        print_status "Check log: $LOG_DIR/camera_publisher_$TIMESTAMP.log"
        break
    fi
    
    # Check if web streamer is still running
    if ! kill -0 $STREAMER_PID 2>/dev/null; then
        print_error "Web streamer stopped unexpectedly!"
        print_status "Check log: $LOG_DIR/web_streamer_$TIMESTAMP.log"
        break
    fi
done

# If we get here, one of the processes died
cleanup
