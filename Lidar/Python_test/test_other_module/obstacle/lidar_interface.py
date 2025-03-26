#!/usr/bin/env python3

import time
import logging
from rplidar import RPLidar
import threading
import numpy as np

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class LidarInterface:
    def __init__(self, port='/dev/ttyUSB0'):
        try:
            self.lidar = RPLidar(port)
            self.running = False
            self.scan_data = []
            self.obstacles = []
            self.lock = threading.Lock()
            self.scan_thread = None
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
        """Continuous scanning loop with improved error handling"""
        while self.running:
            try:
                logger.debug("Starting standard scan...")
                scan_generator = self.lidar.iter_scans()

                with self.lock:
                    # Convert generator to list to prevent exhaustion
                    self.scan_data = list(scan_generator)
                    if self.scan_data:
                        self.process_scan_data()
                    else:
                        logger.warning("No scan data received")

            except Exception as e:
                logger.error(f"Error in scan loop: {e}")
                # Add a small delay to prevent tight error loop
                time.sleep(1)

            time.sleep(0.1)

    def process_scan_data(self):
        """Process scan data to identify obstacles"""
        try:
            obstacles = []
            for scan in self.scan_data:
                for _, angle, distance in scan:
                    # Added more robust coordinate conversion
                    if 0 < distance < 1000:  # Filter out invalid distances
                        try:
                            x = distance * np.cos(np.radians(angle))
                            y = distance * np.sin(np.radians(angle))
                            obstacles.append((x, y))
                        except Exception as coord_error:
                            logger.warning(f"Coordinate conversion error: {coord_error}")

            with self.lock:
                self.obstacles = obstacles
                logger.debug(f"Detected {len(obstacles)} obstacles")
        except Exception as e:
            logger.error(f"Error processing scan data: {e}")

    def get_obstacles(self):
        """
        Returns the current list of obstacles.  This is thread-safe.
        """
        with self.lock:
            return list(self.obstacles)  # Return a copy to prevent external modification

    def get_raw_scan_data(self):
        """
        Returns the current raw scan data.  This is thread-safe.
        """
        with self.lock:
            return list(self.scan_data)

    def stop_scanning(self):
        try:
            self.running = False
            if self.scan_thread:
                self.scan_thread.join(timeout=2)  # Wait for thread to stop

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
        # Initialize the LidarInterface
        lidar_interface = LidarInterface()

        # Start scanning
        lidar_interface.start_scanning()

        # Allow some time for the lidar to collect data
        time.sleep(5)

        # # Get and print the obstacle data
        # obstacles = lidar_interface.get_obstacles()
        # if obstacles:
        #     print(f"Detected {len(obstacles)} obstacles:")
        #     for x, y in obstacles:
        #         print(f"  x: {x:.2f}, y: {y:.2f}")
        # else:
        #     print("No obstacles detected.")

        # Get and print the raw scan data
        scan_data = lidar_interface.get_raw_scan_data()
        print(f"Received {len(scan_data)} scans")
        if scan_data:
            print(f"Received {len(scan_data)} scans")
        else:
            print("No scan data received")


        # Stop scanning
        lidar_interface.stop_scanning()

    except Exception as e:
        logger.error(f"Main function error: {e}")
    finally:
        if 'lidar_interface' in locals():
            lidar_interface.stop_scanning() # Ensure stop_scanning is called.

if __name__ == "__main__":
    main()
