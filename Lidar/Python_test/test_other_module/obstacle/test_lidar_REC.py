import threading
from numpy import cos, sin, sqrt, pi
import numpy as np
import cv2
from rplidar import RPLidar
import time
import logging
from position_manager import position_manager
from sklearn.cluster import DBSCAN

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Global variables initialization
posX, posY, posZ = 100, 100, 0  # Robot position
targX, targY = 200, 150  # Target position
lastV = "0"  # Last velocity command
cote = 1  # Side parameter
runningMatch = True  # Match running status
offsetAngle = 0  # Angle offset for Lidar (degrees)
alertDist = 50  # Alert distance threshold (cm)
debug = False  # Debug mode flag


class LidarThread(threading.Thread):
    def __init__(self, serial_port='/dev/ttyUSB0', stop_event=None, end_event=None, debugCV=False):
        super().__init__()
        self.serial_port = serial_port
        self.stop_event = stop_event or threading.Event()
        self.end_event = end_event or threading.Event()
        self.lidar = None
        self.debugCV = debugCV
        # Buffer with timestamps for point aging
        self.accumulated_points = []  # Will store (x, y, timestamp) tuples
        self.last_clear_time = time.time()
        # Reduced max points and refresh interval for better handling of moving obstacles
        self.max_points = 360  # One full revolution's worth of points
        self.refresh_interval = 1.0  # Faster refresh for moving obstacles
        # Point age limit (seconds)
        self.point_age_limit = 2.0

    def process_lidar_data(self, scan):
        """
        Process lidar data to find clusters of points representing obstacles
        including handling of moving obstacles
        """
        current_time = time.time()
        global posX, posY, posZ

        # Check if we need to refresh the buffer (position change or timeout)
        position_data = (posX, posY, posZ)
        if not hasattr(self, 'last_position') or self.last_position != position_data or \
                (current_time - self.last_clear_time) > self.refresh_interval:
            self.accumulated_points = []
            self.last_clear_time = current_time
            self.last_position = position_data
            logger.info(f"Point buffer refreshed. Position: {position_data}")

        # Add new points to buffer with timestamp
        for _, angle, distance in scan:
            if distance > 0:
                radians = (posZ - angle - offsetAngle) * pi / 180.0  # Convert to radians
                x = distance * 0.1 * cos(radians) + posX
                y = distance * 0.1 * sin(radians) + posY
                if 0 <= x <= 300 and 0 <= y <= 200:
                    self.accumulated_points.append([x, y, current_time])

        # Remove old points based on age
        self.accumulated_points = [p for p in self.accumulated_points
                                   if (current_time - p[2]) <= self.point_age_limit]

        # Limit buffer size
        if len(self.accumulated_points) > self.max_points:
            # Keep the most recent points
            self.accumulated_points = sorted(self.accumulated_points, key=lambda p: p[2], reverse=True)
            self.accumulated_points = self.accumulated_points[:self.max_points]

        # Create array of only x,y coordinates for clustering
        if len(self.accumulated_points) < 20:
            logger.debug(f"Insufficient points: {len(self.accumulated_points)}")
            return []

        # logger.info(f"Number of accumulated points: {len(self.accumulated_points)}")

        # Extract just the x,y coordinates for DBSCAN
        points_array = np.array([[p[0], p[1]] for p in self.accumulated_points])

        # Adjust clustering parameters based on point density
        eps_value = 15 if len(points_array) < 100 else 10
        min_samples_value = 3 if len(points_array) < 100 else 5

        # Apply different weights to more recent points (future enhancement)
        clustering = DBSCAN(eps=eps_value, min_samples=min_samples_value).fit(points_array)

        obstacles = []
        labels = clustering.labels_
        unique_labels = set(labels)

        n_clusters = len(unique_labels) - (1 if -1 in unique_labels else 0)
        n_noise = list(labels).count(-1)
        # logger.info(f"Clusters found: {n_clusters}, noise points: {n_noise}")

        for label in unique_labels:
            if label == -1:
                continue

            cluster_points = points_array[labels == label]

            min_x = np.min(cluster_points[:, 0])
            max_x = np.max(cluster_points[:, 0])
            min_y = np.min(cluster_points[:, 1])
            max_y = np.max(cluster_points[:, 1])

            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            width = max_x - min_x
            height = max_y - min_y

            obstacles.append((center_x, center_y, width, height))

            # logger.info(f"Obstacle detected: center=({center_x:.1f}, {center_y:.1f}), "
            #             f"size=({width:.1f}, {height:.1f})")

        return obstacles

    def run(self):
        global posX, posY, posZ, targX, targY, lastV, cote, runningMatch

        try:
            minX, minY = 0, 0
            prev_posX, prev_posY, prev_posZ = -1, -1, -1
            prev_targX, prev_targY = -1, -1

            try:
                logger.info(f"Connecting to LIDAR on {self.serial_port}")
                self.lidar = RPLidar(self.serial_port)
                logger.info("LidarThread started successfully")
            except Exception as e:
                logger.error(f"Failed to initialize LIDAR: {e}")
                raise

            affCoef = 3  # Visualization scaling factor
            affSize = 2  # Size of objects in visualization

            if self.debugCV:
                imgBase = np.zeros((200 * affCoef, 300 * affCoef, 3), np.uint8)
                cv2.imshow('frame', imgBase)
                cv2.moveWindow('frame', 400, 150)
                logger.info("OpenCV visualization window opened")

            for scan in self.lidar.iter_scans():
                if self.end_event.is_set():
                    break

                # Get current position data from position manager
                posX, posY, posZ = position_manager.get_position()
                targX, targY = position_manager.get_target()
                lastV = position_manager.get_velocity()

                obstacles = self.process_lidar_data(scan)

                # Check if the position has changed
                position_changed = (posX != prev_posX or posY != prev_posY or posZ != prev_posZ or
                                    targX != prev_targX or targY != prev_targY)

                # Update previous position data
                prev_posX, prev_posY, prev_posZ = posX, posY, posZ
                prev_targX, prev_targY = targX, targY

                # Reset visualization image
                if self.debugCV and (position_changed or not runningMatch):
                    imgBase = np.zeros((200 * affCoef, 300 * affCoef, 3), np.uint8)

                if not self.end_event.is_set():
                    if self.debugCV:
                        img = cv2.circle(imgBase.copy(),
                                         (int(posX * affCoef), int((200 - posY) * affCoef)),
                                         affSize * affCoef, (0, 255, 0), -1)

                    # Calculate trajectory parameters
                    a, b = targY - posY, posX - targX
                    c = -(b * posY + a * posX)
                    distT = sqrt((targX - posX) ** 2 + (targY - posY) ** 2)

                    minDistance, minAngle, mindistP = 300, 0, 300
                    for (_, angle, distance) in scan:
                        if distance > 0:
                            radians = (posZ - angle - offsetAngle) * pi / 180.0
                            x = distance * 0.1 * cos(radians) + posX
                            y = distance * 0.1 * sin(radians) + posY
                            if 0 < int(x) < 300 and 0 < int(y) < 200:
                                if self.debugCV:
                                    img[int((200 - y) * affCoef), int(x * affCoef)] = [255, 255, 0]
                                    if runningMatch:
                                        imgBase[int((200 - y) * affCoef), int(x * affCoef)] = [0, 0, 255]

                                if distance > 0 and (a * a + b * b) != 0:
                                    distN = abs(a * x + b * y + c) / sqrt(a * a + b * b)
                                    distR = sqrt((x - posX) ** 2 + (y - posY) ** 2)
                                    distA = sqrt((x - targX) ** 2 + (y - targY) ** 2)

                                    if distR < (distT + alertDist) and distA < distT and distN < alertDist:
                                        mindist = min(distN, distA, distR)
                                        if mindist < minDistance:
                                            minDistance = mindist
                                            minAngle = angle - posZ - offsetAngle
                                            minX, minY = x, y

                    if self.debugCV:
                        color = (0, 0, 255) if minDistance < alertDist else (0, 255, 0)
                        cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                 (int(minX * affCoef), int((200 - minY) * affCoef)), color, 2)
                        img = cv2.circle(img, (int(minX * affCoef), int((200 - minY) * affCoef)),
                                         alertDist * affCoef, (0, 0, 255), 1)

                        # Draw line to target
                        radians = posZ * pi / 180.0
                        cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                 (int(targX * affCoef), int((200 - targY) * affCoef)), (0, 255, 255), 2)
                        cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                 (int((posX + affSize * cos(radians)) * affCoef),
                                  int((200 - (posY + affSize * sin(radians))) * affCoef)), (255, 0, 0), 2)

                        # Draw all obstacles
                        if obstacles:
                            for (center_x, center_y, width, height) in obstacles:
                                cv2.rectangle(img,
                                              (int((center_x - width / 2) * affCoef),
                                               int((200 - (center_y + height / 2)) * affCoef)),
                                              (int((center_x + width / 2) * affCoef),
                                               int((200 - (center_y - height / 2)) * affCoef)),
                                              (255, 0, 0), 1)

                        cv2.imshow('frame', img)
                        cv2.waitKey(1)

                    if minDistance < alertDist and runningMatch and not debug:
                        if hasattr(self, 'serial_port') and hasattr(self.serial_port, 'write'):
                            self.serial_port.write("0\r\n".encode())
                        self.stop_event.set()
                    else:
                        if self.stop_event.is_set():
                            logger.info(f'Resuming with last velocity: {lastV}')
                            if hasattr(self, 'serial_port') and hasattr(self.serial_port, 'write'):
                                self.serial_port.write(lastV.encode())
                                self.serial_port.write("\r\n".encode())
                            self.stop_event.clear()

                else:
                    break

        except KeyboardInterrupt:
            logger.info('Stopping due to keyboard interrupt.')
        except Exception as e:
            logger.error(f"Error in LidarThread: {e}")
        finally:
            self.stop_lidar()
            if self.debugCV:
                cv2.destroyAllWindows()

    def stop_lidar(self):
        """Stop the lidar and clean up resources"""
        if self.lidar:
            try:
                logger.info("Stopping LIDAR")
                self.lidar.stop()
                self.lidar.disconnect()
            except Exception as e:
                logger.error(f"Error stopping LIDAR: {e}")
        else:
            logger.warning("LIDAR not initialized, nothing to stop")


def main():
    """Main function to run the LidarThread"""
    stop_event = threading.Event()
    end_event = threading.Event()

    try:
        # Start the lidar thread
        lidar_thread = LidarThread('/dev/ttyUSB0', stop_event, end_event)
        lidar_thread.start()
        logger.info("LidarThread started")

        # Run for a few seconds for demonstration
        time.sleep(1000)

        # Signal to stop and end
        logger.info("Signaling LidarThread to stop and end")
        stop_event.set()
        end_event.set()
        time.sleep(1)

        # Wait for thread to finish
        lidar_thread.join(timeout=5)
        logger.info("LidarThread ended")

    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        end_event.set()
        stop_event.set()


if __name__ == '__main__':
    main()
