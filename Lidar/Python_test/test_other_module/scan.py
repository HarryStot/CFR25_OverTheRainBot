#!/usr/bin/env python3

import time
from rplidar import RPLidar
import numpy as np

# Configuration
PORT_NAME = '/dev/ttyUSB0'  # Change this to match your setup
DANGER_THRESHOLD = 1000  # Distance in mm to consider an obstacle dangerous
DANGER_ANGLE = 180  # Check for obstacles within ±60° of front


def detect_obstacles():
    lidar = RPLidar(PORT_NAME)
    # lidar.motor_speed = 1000

    try:
        print("Starting LiDAR...")
        lidar.start_motor()
        time.sleep(1)  # Let the motor stabilize

        print("Starting scan...")
        for i, scan in enumerate(lidar.iter_scans()):
            # Process data and check for obstacles
            front_obstacles = []

            for _, angle, distance in scan:
                # Check if the point is in the forward-facing sector
                # (assuming 0° is forward, adjust if your LiDAR is oriented differently)
                if (0 <= angle <= DANGER_ANGLE) or (360 - DANGER_ANGLE <= angle <= 360):
                    if distance < DANGER_THRESHOLD:
                        front_obstacles.append((angle, distance))

            # Report obstacles
            if front_obstacles:
                print(f"WARNING: {len(front_obstacles)} obstacles detected in front!")
                closest = min(front_obstacles, key=lambda x: x[1])
                print(f"Closest obstacle: angle={closest[0]}°, distance={closest[1]}mm")
                # Here you would send commands to your robot to avoid the obstacle
            else:
                print("Path clear")

            # Limit output rate
            if i % 10 == 0:
                print(f"Scan #{i}: {len(scan)} points processed")

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()


if __name__ == '__main__':
    detect_obstacles()