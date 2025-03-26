#!/usr/bin/env python3

import time
import logging
from rplidar import RPLidar
import threading
import numpy as np
from math import *

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class LidarInterface:
    def __init__(self, port='/dev/ttyUSB0', target_x=150.0, target_y=100.0, alert_distance=50.0):
        try:
            self.lidar = RPLidar(port)
            self.running = False
            self.scan_data = []
            self.obstacles = []
            self.lock = threading.Lock()
            self.scan_thread = None
            self.target_x = target_x  # Target X coordinate
            self.target_y = target_y  # Target Y coordinate
            self.alert_distance = alert_distance  # Obstacle alert distance threshold
            self.offset_angle = 90  # Angle offset for Lidar
            self.debug_cv = False # Add a debug_cv parameter
            self.closest_obstacle = None # Initialize closest_obstacle
            logger.info(f"LidarInterface initialized with port {port}")
        except Exception as e:
            logger.error(f"Failed to initialize LidarInterface: {e}")
            raise

    def start_scanning(self):
        try:
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan_loop)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            self.lidar.start_motor()
            logger.info("Lidar scanning started")
        except Exception as e:
            logger.error(f"Error starting scanning: {e}")
            self.running = False
            raise

    def _scan_loop(self):
        """Continuous scanning loop with obstacle detection and visualization"""
        while self.running:
            try:
                logger.debug("Starting standard scan...")
                scan_generator = self.lidar.iter_scans()
                scan_data = list(scan_generator) #convert to a list

                with self.lock:
                    self.scan_data = scan_data
                    if self.scan_data:
                        self.process_scan_data()
                    else:
                        logger.warning("No scan data received")

            except Exception as e:
                logger.error(f"Error in scan loop: {e}")
                time.sleep(1)

            time.sleep(0.1)

    def process_scan_data(self):
        """Process scan data to identify obstacles and their proximity to the trajectory"""
        try:
            obstacles = []
            closest_obstacle = None
            min_distance = float('inf')
            current_x, current_y, current_z = 0, 0, 0 #place holder, should be updated.

            if self.scan_data:
                # Get current position and orientation.  Assume it is the first scan.
                _, current_z, _ = self.scan_data[0][0] #angle

                # Calculate trajectory parameters (a, b, c)
                a = self.target_y - current_y
                b = current_x - self.target_x
                c = -(b * current_y + a * current_x)
                trajectory_distance = sqrt((self.target_x - current_x) ** 2 + (self.target_y - current_y) ** 2)

                for scan in self.scan_data:
                    for _, angle, distance in scan:
                        if 0 < distance < 5000:  # Filter for valid distances (in mm)
                            # Convert polar to Cartesian coordinates (in cm)
                            radians = (current_z - angle - self.offset_angle) * pi / 180.0
                            x = distance * 0.1 * cos(radians) + current_x
                            y = distance * 0.1 * sin(radians) + current_y
                            obstacles.append((x, y))

                            # Calculate distance from the point to the trajectory line
                            if a != 0 or b != 0:
                                dist_to_trajectory = abs(a * x + b * y + c) / sqrt(a ** 2 + b ** 2)
                            else:
                                dist_to_trajectory = float('inf')

                            # Calculate distance from the obstacle to the robot and target
                            dist_to_robot = sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
                            dist_to_target = sqrt((x - self.target_x) ** 2 + (y - self.target_y) ** 2)

                            # Check if the obstacle is within the alert zone, between robot and target, and near trajectory
                            if (
                                dist_to_robot < (trajectory_distance + self.alert_distance)
                                and dist_to_target < trajectory_distance
                                and dist_to_trajectory < self.alert_distance
                            ):
                                if dist_to_trajectory < min_distance:
                                    min_distance = dist_to_trajectory
                                    closest_obstacle = (x, y)

                with self.lock:
                    self.obstacles = obstacles  # Update the obstacles list
                    self.closest_obstacle = closest_obstacle #store closest obstacle
                    logger.debug(f"Detected {len(obstacles)} obstacles")

        except Exception as e:
            logger.error(f"Error processing scan data: {e}")

    def get_obstacles(self):
        """Returns the current list of obstacles."""
        with self.lock:
            return list(self.obstacles)

    def get_closest_obstacle(self):
        """Returns the closest obstacle to the trajectory"""
        with self.lock:
            return self.closest_obstacle

    def stop_scanning(self):
        try:
            self.running = False
            if self.scan_thread:
                self.scan_thread.join(timeout=2)
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            logger.info("Lidar scanning stopped")
        except Exception as e:
            logger.error(f"Error stopping lidar: {e}")

    def __del__(self):
        self.stop_scanning()



def main():
    """Main function to demonstrate LidarInterface usage"""
    try:
        # Initialize the LidarInterface with target coordinates and alert distance
        lidar_interface = LidarInterface(target_x=150.0, target_y=100.0, alert_distance=20.0)

        # Start scanning
        lidar_interface.start_scanning()

        # Allow some time for the lidar to collect data
        time.sleep(5)

        # Get obstacle data and closest obstacle
        obstacles = lidar_interface.get_obstacles()
        closest_obstacle = lidar_interface.get_closest_obstacle()

        if obstacles:
            print(f"Detected {len(obstacles)} obstacles:")
            for x, y in obstacles:
                print(f"  x: {x:.2f}, y: {y:.2f}")
        else:
            print("No obstacles detected.")

        if closest_obstacle:
            print(f"Closest obstacle to trajectory: x: {closest_obstacle[0]:.2f}, y: {closest_obstacle[1]:.2f}")
        else:
            print("No obstacles close to the trajectory.")

        # Stop scanning
        lidar_interface.stop_scanning()

    except Exception as e:
        logger.error(f"Main function error: {e}")


if __name__ == "__main__":
    main()
