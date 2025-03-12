from pyrplidar import PyRPlidar
import numpy as np
import threading
import time


class LidarProcessor:
    def __init__(self, port='/dev/ttyUSB2'):
        self.lidar = PyRPlidar()
        self.lidar.connect(port=port)
        self.lidar.set_motor_pwm(500)

        self.scan_data = []
        self.obstacles_raw = []  # List of (x,y) coordinates detected by lidar
        self.obstacles = [] # List of (x,y,r) obstacles detected by lidar
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
            scan = self.lidar.get_measurements() # FIXME get_measurements() is not a function in PyRPLidar
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

    def treat_obstacles(self):
        """
        Treat the obstacles detected by the lidar to identify obstacles
        :return: list of (x,y,r) obstacles detected by the lidar (r is the radius of the obstacle)
        """
        # TODO Implement obstacle avoidance algorithm
        pass