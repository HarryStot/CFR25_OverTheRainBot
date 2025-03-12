from pyrplidar import PyRPlidar
import numpy as np
import threading
import time
from sklearn.cluster import DBSCAN
from collections import defaultdict


class LidarProcessor:
    def __init__(self, port='/dev/ttyUSB2'):
        self.lidar = PyRPlidar()
        self.lidar.connect(port=port)
        self.lidar.set_motor_pwm(500)

        self.scan_data = []
        self.obstacles_raw = []  # List of (x,y) coordinates detected by lidar
        self.obstacles = []  # List of (x,y,r) obstacles detected by lidar
        self.robot_height = 0.35  # FIXME Height of robot in meters
        self.lock = threading.Lock()
        self.running = False
        self.scan_thread = None

    def start_scanning(self):
        """Start lidar scanning in a separate thread"""
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_loop)
        self.scan_thread.daemon = True
        self.scan_thread.start()

    def _scan_loop(self):
        """Continuous scanning loop"""
        self.lidar.start_scan()

        while self.running:
            try:
                print("Starting standard scan...")
                scan = self.lidar.start_scan()
            except Exception as e:
                print(f"Error starting standard scan: {e}")
                try:
                    print("Trying force scan...")
                    scan = self.lidar.force_scan()
                except Exception as e:
                    print(f"Error starting force scan: {e}")
                    self.lidar.stop()
                    self.lidar.set_motor_pwm(0)
                    self.lidar.disconnect()
                    return
            # scan = self.lidar.get_measurements()  # FIXME get_measurements() is not a function in PyRPLidar
            with self.lock:
                self.scan_data = scan
                self.process_scan_data()

            time.sleep(0.1)  # TODO Adjust as needed

    def process_scan_data(self):
        """Process scan data to identify obstacles_raw"""
        obstacles_raw = []
        for measurement in self.scan_data:
            # Extract distance and angle
            distance = measurement.distance / 1000.0  # Convert to meters
            angle = measurement.angle

            # Convert to Cartesian coordinates (robot as origin)
            x = distance * np.cos(np.radians(angle))
            y = distance * np.sin(np.radians(angle))

            # Add to obstacles_raw list if it's at robot height
            # You may need to filter based on your specific setup
            obstacles_raw.append((x, y))

        # Update obstacles_raw list
        self.obstacles_raw = obstacles_raw

    def process_obstacles(self):
        """Process raw obstacles to filter obstacles and determine their centers and radius
        :return: List of tuples (center_x, center_y, radius) for each detected obstacle
        """
        if not self.obstacles_raw or len(self.obstacles_raw) < 3:
            return []

        # Convert to numpy array for clustering
        points = np.array(self.obstacles_raw)

        # Use DBSCAN to cluster points belonging to the same obstacle
        # eps is the maximum distance between samples (adjust based on your scale)
        # min_samples is the number of samples in a neighborhood for a point to be a core point
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

            # Simple outlier rejection - if the radius is too large relative to the number of points,
            # it might be a noise cluster or multiple merged obstacles
            if radius < 0.5:  # Maximum expected obstacle radius
                obstacles.append((center_x, center_y, radius))

        self.obstacles = obstacles

    def get_obstacles_raw(self):
        """Return the current list of detected obstacles"""
        with self.lock:
            return self.obstacles_raw.copy()

    def stop(self):
        """Stop the lidar"""
        self.running = False
        if self.scan_thread:
            self.scan_thread.join(timeout=1.0)
        self.lidar.set_motor_pwm(0)
        self.lidar.stop()
        self.lidar.disconnect()
