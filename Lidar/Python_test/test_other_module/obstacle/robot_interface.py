# robot_interface.py
import time
import threading
from position_manager import position_manager
import logging
import serial
import re

logger = logging.getLogger(__name__)


class RobotInterface(threading.Thread):
    """Update robot position and target from serial data"""

    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200, stop_event=None):
        super().__init__()
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.stop_event = stop_event or threading.Event()
        self.daemon = True
        self.ser = None

    def run(self):
        try:
            logger.info(f"Connecting to port {self.serial_port} at {self.baud_rate} baud")
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()

            while not self.stop_event.is_set():
                logger.debug("Waiting for data...")
                if self.ser.in_waiting > 0:
                    ligne = self.ser.readline().decode('utf-8').strip()
                    logger.debug(f"Receive: {ligne}")
                    if ligne.startswith("POS,"):
                        # Nouveau format: "POS,X:32.3,Y:84.0,Z:41.0"
                        if ":" in ligne:
                            # Extraire les coordonnées du nouveau format
                            x = y = orientation = None
                            target_x = target_y = None

                            # Utiliser des expressions régulières pour extraire les valeurs
                            x_match = re.search(r'X:([-+]?\d*\.?\d+)', ligne)
                            y_match = re.search(r'Y:([-+]?\d*\.?\d+)', ligne)
                            z_match = re.search(r'Z:([-+]?\d*\.?\d+)', ligne)

                            if x_match:
                                x = float(x_match.group(1))
                            if y_match:
                                y = float(y_match.group(1))
                            if z_match:
                                orientation = float(z_match.group(1))

                            if x is not None and y is not None:
                                position_manager.set_position(x, y, orientation)
                                if target_x is not None and target_y is not None:
                                    position_manager.set_target(target_x, target_y)
                                logger.info(
                                    f"New robot pos: ({x}, {y}, {orientation}), target: ({target_x}, {target_y})")
                        else:
                            # Format ancien: "POS,x,y,orientation,target_x,target_y"
                            data = ligne[4:].split(',')
                            if len(data) >= 3:
                                x, y, orientation = map(float, data[:3])
                                target_x, target_y = map(float, data[3:5]) if len(data) >= 5 else (None, None)
                                position_manager.set_position(x, y, orientation)
                                if target_x is not None and target_y is not None:
                                    position_manager.set_target(target_x, target_y)
                                logger.info(
                                    f"New robot pos: ({x}, {y}, {orientation}), target: ({target_x}, {target_y})")

                time.sleep(0.01)

        except Exception as e:
            logger.error(f"Error in RobotInterface: {e}")

        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                logger.info("Serial port closed")
