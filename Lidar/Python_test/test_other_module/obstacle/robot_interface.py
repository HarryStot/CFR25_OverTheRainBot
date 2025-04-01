# robot_interface.py
import time
import threading
from position_manager import position_manager
import logging
import serial

logger = logging.getLogger(__name__)


class RobotInterface(threading.Thread):
    """Update robot position and target from serial data"""
    def __init__(self, serial_port='/dev/pts/3', baud_rate=115200, stop_event=None):
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
                if self.ser.in_waiting > 0:
                    ligne = self.ser.readline().decode('utf-8').strip()
                    logger.debug(f"Receive: {ligne}")
                    if ligne.startswith("POS,"):
                        data = ligne[4:].split(',')
                        if len(data) >= 5:
                            x, y, orientation, target_x, target_y = map(float, data[:5])
                            position_manager.set_position(x, y, orientation)
                            position_manager.set_target(target_x, target_y)
                            logger.info(f"New robot pos: ({x}, {y}, {orientation}), target: ({target_x}, {target_y})")

                time.sleep(0.01)

        except Exception as e:
            logger.error(f"Error in RobotInterface: {e}")

        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                logger.info("Serial port closed")