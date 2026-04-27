#!/usr/bin/env python3
from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'   # Change if your lidar is on another port
lidar = RPLidar(PORT_NAME)

try:
    print("Starting RPLidar test... (press Ctrl+C to stop)")
    for scan in lidar.iter_scans():
        # Each scan is a list of (quality, angle, distance) tuples
        distances = [d for (_, _, d) in scan if d > 0]
        if distances:
            closest = min(distances)
            print(f"Closest object: {closest:.1f} mm ({closest/1000:.2f} m)")
        else:
            print("No objects detected")
except KeyboardInterrupt:
    print("Stopping...")
finally:
    lidar.stop()
    lidar.disconnect()

