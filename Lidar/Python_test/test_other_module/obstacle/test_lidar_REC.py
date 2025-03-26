import threading
from numpy import cos, sin, sqrt, pi
import numpy as np
import cv2
from rplidar import RPLidar
import time
import logging
from position_manager import position_manager

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
        threading.Thread.__init__(self)
        self.serial_port = serial_port
        self.stop_event = stop_event or threading.Event()
        self.end_event = end_event or threading.Event()
        self.lidar = None
        self.debugCV = debugCV

    def run(self):
        global posX, posY, posZ, targX, targY, lastV, cote, runningMatch

        try:
            minX, minY = 0, 0

            # Add better error handling for LIDAR initialization
            try:
                logger.info(f"Connecting to LIDAR on {self.serial_port}")
                self.lidar = RPLidar(self.serial_port)
                logger.info("LidarThread started successfully")
            except Exception as e:
                logger.error(f"Failed to initialize LIDAR: {e}")
                # Re-raise to be caught by outer exception handler
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

                if self.debugCV and not runningMatch:
                    # Create a black image (300 x 200)
                    imgBase = np.zeros((200 * affCoef, 300 * affCoef, 3), np.uint8)

                if not self.end_event.is_set():
                    if self.debugCV:
                        img = cv2.circle(imgBase.copy(),
                                         (int(posX * affCoef), int((200 - posY) * affCoef)),
                                         affSize * affCoef, (0, 255, 0), -1)

                    # Calculate trajectory parameters
                    a = (targY - posY)
                    b = (posX - targX)
                    c = -(b * posY + a * posX)
                    # Distance between robot and target
                    distT = sqrt((targX - posX) ** 2 + (targY - posY) ** 2)

                    minDistance = 300
                    minAngle = 0
                    mindistP = 300
                    for (_, angle, distance) in scan:
                        if distance > 0:  # Ignore initially ungathered data points
                            radians = (posZ - angle - offsetAngle) * pi / 180.0
                            x = distance * 0.1 * cos(radians) + posX
                            y = distance * 0.1 * sin(radians) + posY
                            if 0 < int(x) < 300 and 0 < int(y) < 200:
                                if self.debugCV:
                                    img[int((200 - y) * affCoef), int(x * affCoef)] = [255, 255, 0]
                                    if runningMatch:
                                        imgBase[int((200 - y) * affCoef), int(x * affCoef)] = [0, 0, 255]

                                if distance > 0 and (a * a + b * b) != 0:
                                    # Normal distance to trajectory
                                    distN = abs(a * x + b * y + c) / sqrt(a * a + b * b)
                                    # Distance from robot position
                                    distR = sqrt((x - posX) ** 2 + (y - posY) ** 2)
                                    # Distance to target
                                    distA = sqrt((x - targX) ** 2 + (y - targY) ** 2)

                                    # Check if in alert zone, between robot and target, and close to trajectory
                                    if distR < (distT + alertDist) and distA < distT and distN < alertDist:
                                        mindist = min(distN, distA, distR)
                                        if mindist < minDistance:
                                            minDistance = mindist
                                            minAngle = angle - posZ - offsetAngle
                                            minX = x
                                            minY = y

                    if self.debugCV:
                        if minDistance < alertDist:
                            cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                     (int(minX * affCoef), int((200 - minY) * affCoef)), (0, 0, 255), 2)
                        else:
                            cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                     (int(minX * affCoef), int((200 - minY) * affCoef)), (0, 255, 0), 2)
                        img = cv2.circle(img, (int(minX * affCoef), int((200 - minY) * affCoef)),
                                         alertDist * affCoef, (0, 0, 255), 1)

                    if minDistance < alertDist and runningMatch and not debug:
                        logger.info('Obstacle detected - Emergency stop')
                        if hasattr(self, 'serial_port') and hasattr(self.serial_port, 'write'):
                            self.serial_port.write("0\r\n".encode())  # Stop signal
                        self.stop_event.set()
                    else:
                        if self.stop_event.is_set():
                            logger.info(f'Resuming with last velocity: {lastV}')
                            if hasattr(self, 'serial_port') and hasattr(self.serial_port, 'write'):
                                self.serial_port.write(lastV.encode())
                                self.serial_port.write("\r\n".encode())
                            self.stop_event.clear()

                    radians = posZ * pi / 180.0

                    # Draw trajectory
                    if self.debugCV:
                        cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                 (int(targX * affCoef), int((200 - targY) * affCoef)),
                                 (0, 255, 255), 2)

                    # Draw orientation
                    if self.debugCV:
                        cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                 (int((posX + affSize * cos(radians)) * affCoef),
                                  int((200 - (posY + affSize * sin(radians))) * affCoef)),
                                 (255, 0, 0), 2)
                    if self.debugCV:
                        cv2.imshow('frame', img)
                        cv2.waitKey(1)
                else:
                    break

        except KeyboardInterrupt:
            logger.info('Stopping due to keyboard interrupt.')
        except Exception as e:
            logger.error(f"Error in LidarThread: {e}")
        finally:
            self.stop_lidar()

    def stop_lidar(self):
        """Safely stop the lidar"""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                logger.info("Lidar stopped and disconnected")
            except Exception as e:
                logger.error(f"Error stopping lidar: {e}")


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

        # Signal to stop
        logger.info("Signaling LidarThread to stop")
        stop_event.set()
        time.sleep(1)

        # Signal to end
        logger.info("Signaling LidarThread to end")
        end_event.set()

        # Wait for thread to finish
        lidar_thread.join(timeout=5)
        logger.info("LidarThread ended")

    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        # Ensure events are set in case of exception
        end_event.set()
        stop_event.set()


if __name__ == '__main__':
    main()
