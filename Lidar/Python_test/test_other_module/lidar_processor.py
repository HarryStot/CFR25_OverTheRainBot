from rplidar import RPLidar
import numpy as np
import threading
import time
from sklearn.cluster import DBSCAN
from collections import defaultdict


class LidarProcessor:
    def __init__(self, port='/dev/ttyUSB2'):
        self.lidar = RPLidar(port)
        self._set_motor_speed(500)  # Start the motor

        self.scan_data = []
        self.obstacles_raw = []  # List of (x,y) coordinates detected by lidar
        self.obstacles = []  # List of (x,y,r) obstacles detected by lidar
        self.robot_height = 0.35
        self.lock = threading.Lock()
        self.running = False
        self.scan_thread = None

    def _set_motor_speed(self, speed):
        # RPLidar uses _set_pwm instead of set_motor_pwm
        self.lidar._set_pwm(speed)

    def start_scanning(self):
        """Start lidar scanning in a separate thread"""
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_loop)
        self.scan_thread.daemon = True
        self.scan_thread.start()

    def _scan_loop(self):
        """Continuous scanning loop"""
        try:
            # In rplidar, we use iter_scans instead of start_scan
            scan_iterator = self.lidar.iter_scans()

            while self.running:
                try:
                    # Get next scan frame
                    scan = next(scan_iterator)
                    with self.lock:
                        self.scan_data = scan
                        self.process_scan_data()
                    time.sleep(0.1)
                except StopIteration:
                    print("Scan iteration ended")
                    break
                except Exception as e:
                    print(f"Error during scanning: {e}")
                    break
        except Exception as e:
            print(f"Error setting up scan iterator: {e}")
        finally:
            if self.running:
                print("Scan loop ended")

    def process_scan_data(self):
        """Process scan data to identify obstacles_raw"""
        obstacles_raw = []
        for measurement in self.scan_data:
            # RPLidar format: (quality, angle, distance)
            quality = measurement[0]
            angle = measurement[1]
            distance = measurement[2] / 1000.0  # Convert to meters

            # Filter out low-quality measurements if needed
            if quality > 10:  # Adjust threshold as needed
                # Convert to Cartesian coordinates (robot as origin)
                x = distance * np.cos(np.radians(angle))
                y = distance * np.sin(np.radians(angle))
                obstacles_raw.append((x, y))

        # Update obstacles_raw list
        self.obstacles_raw = obstacles_raw

    def process_obstacles(self):
        """Process raw obstacles to filter obstacles and determine their centers and radius"""
        if not self.obstacles_raw or len(self.obstacles_raw) < 3:
            return []

        # Convert to numpy array for clustering
        points = np.array(self.obstacles_raw)

        # Use DBSCAN to cluster points belonging to the same obstacle
        clustering = DBSCAN(eps=0.1, min_samples=3).fit(points)

        # Get labels for each point (-1 means noise)
        labels = clustering.labels_

        # Group points by cluster
        clusters = defaultdict(list)
        for i, label in enumerate(labels):
            if label != -1:  # Ignore noise
                clusters[label].append(points[i])

        obstacles = []

        # Process each cluster to find center and radius
        for cluster_id, cluster_points in clusters.items():
            cluster_points = np.array(cluster_points)

            # Calculate center as mean of points
            center_x = np.mean(cluster_points[:, 0])
            center_y = np.mean(cluster_points[:, 1])

            # Calculate radius as maximum distance from center to any point
            distances = np.sqrt(np.sum(np.square(cluster_points - [center_x, center_y]), axis=1))
            radius = np.max(distances)

            # Add a small buffer to the radius for safety
            radius += 0.05

            # Simple outlier rejection
            if radius < 0.5:  # Maximum expected obstacle radius
                obstacles.append((center_x, center_y, radius))

        self.obstacles = obstacles
        return obstacles

    def get_obstacles_raw(self):
        """Return the current list of detected obstacles"""
        with self.lock:
            return self.obstacles_raw.copy()

    def stop(self):
        """Stop the lidar"""
        self.running = False
        if self.scan_thread:
            self.scan_thread.join(timeout=1.0)
        self._set_motor_speed(0)  # Stop motor
        self.lidar.stop()
        self.lidar.disconnect()