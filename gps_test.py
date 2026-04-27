import serial
import time
import pynmea2
import math

# Configure the serial port
port = "/dev/ttyAMA0"   # For UART0 on Raspberry Pi
baud = 9600

# Open the serial connection once
ser = serial.Serial(port, baud, timeout=1)

TARGET_LAT = -1.095044934
TARGET_LNG = 37.012349834

try:
    while True:
        data = ser.readline().decode('ascii', errors='replace')

        if data.startswith("$GPRMC"):
            # try:
            #     msg = pynmea2.parse(data)
            #     lat = msg.latitude
            #     lng = msg.longitude
            #     gps_data = f"Latitude: {lat}, Longitude: {lng}"
            #     print(gps_data)

            try:
                msg = pynmea2.parse(data)
                lat = msg.latitude
                lng = msg.longitude
                
                # Calculate the raw degree deltas
                d_lat = lat - TARGET_LAT
                d_lng = lng - TARGET_LNG
                
                # Simple Euclidean distance in degrees (rough estimation)
                # To get meters, a rough multiplier is 111,139 meters per degree
                distance_deg = math.sqrt(d_lat**2 + d_lng**2)
                distance_m = distance_deg * 111139

                print(f"LAT: {lat:.8f} | LNG: {lng:.8f}")
                print(f"Δ Lat: {d_lat:.8f} | Δ Lng: {d_lng:.8f}")
                print(f"Rough Distance from Target: {distance_m:.2f} meters")
                print("-" * 40)

            except pynmea2.ParseError:
                pass  # Ignore malformed data

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()

