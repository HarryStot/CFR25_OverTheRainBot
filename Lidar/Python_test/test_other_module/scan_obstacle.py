#!/usr/bin/env python3

import time
from rplidar import RPLidar
import numpy as np

# Configuration
PORT_NAME = '/dev/ttyUSB0'
DANGER_THRESHOLD = 1000
DANGER_ANGLE = 180

class Lidar:
    def __init__(self):
        self.lidar = RPLidar(PORT_NAME)
        self.lidar.start_motor()
        time.sleep(1)

    def detect_obstacles(self):
        try:
            print("Starting LiDAR...")
            print("Starting scan...")
            for i, scan in enumerate(self.lidar.iter_scans()):
                front_obstacles = []

                for _, angle, distance in scan:
                    if (0 <= angle <= DANGER_ANGLE) or (360 - DANGER_ANGLE <= angle <= 360):
                        if distance < DANGER_THRESHOLD:
                            front_obstacles.append((angle, distance))

                if front_obstacles:
                    print(f"WARNING: {len(front_obstacles)} obstacles detected in front!")
                    closest = min(front_obstacles, key=lambda x: x[1])
                    print(f"Closest obstacle: angle={closest[0]}°, distance={closest[1]}mm")
                else:
                    print("Path clear")

                if i % 10 == 0:
                    print(f"Scan #{i}: {len(scan)} points processed")

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()