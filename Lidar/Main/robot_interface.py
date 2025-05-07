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
        self.connected = False
        self.position_received = threading.Event()
        self.buffer = ""

    def run(self):
        retry_count = 0
        max_retries = 5

        while not self.stop_event.is_set() and retry_count < max_retries:
            try:
                logger.info(f"Connecting to {self.serial_port} at {self.baud_rate} bauds")
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                time.sleep(2)  # Waiting for the serial port to open

                if self.ser.is_open:
                    logger.info(f"{self.serial_port} connected!")
                    self.ser.reset_input_buffer()
                    logger.info(
                        "Asking for initial position...")  # TODO: Don't ask for position at startup (add team and set command)
                    self.ser.write("P\n".encode())
                    self.connected = True
                    retry_count = 0

                    logger.info("Starting to read data from Arduino...")
                    while not self.stop_event.is_set():
                        # TODO: Remove ?
                        # if time.time() % 1 < 0.01:  # Environ une fois par seconde
                        #     self.ser.write(b"P\n")

                        # Read all available data
                        if self.ser.in_waiting > 0:
                            raw_data = self.ser.read(self.ser.in_waiting)
                            try:
                                data = raw_data.decode()
                                self.buffer += data

                                # Process complete lines
                                while '\n' in self.buffer:
                                    line, self.buffer = self.buffer.split('\n', 1)
                                    line = line.strip()
                                    if line:  # Ignore empty lines
                                        logger.debug(f"Reçu: {line}")
                                        self.process_line(line)
                            except UnicodeDecodeError:
                                logger.warning("Invalid data received, emptying buffer")
                                self.buffer = ""

                        time.sleep(0.001) # Waiting 1 ms

            except serial.SerialException as e:
                logger.error(f"Serial error: {e}")
                retry_count += 1
                logger.info(f"New tentative ({retry_count}/{max_retries})...")
                time.sleep(2)

            except Exception as e:
                logger.error(f"Error in RobotInterface: {e}")
                break

            finally:
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                        logger.info("Serial port closed")
                    except Exception as e:
                        logger.error(f"Error closing the serial port: {e}")

                self.connected = False

        if retry_count >= max_retries:
            logger.error(f"Échec de connexion après {max_retries} tentatives")

    def process_line(self, line):
        """Process a line received from the Arduino"""
        try:
            if "POS" in line:
                self.position_received.set()

                x_match = re.search(r'X:?\s*([-+]?[0-9]*\.?[0-9]+)', line)
                y_match = re.search(r'Y:?\s*([-+]?[0-9]*\.?[0-9]+)', line)
                z_match = re.search(r'Z:?\s*([-+]?[0-9]*\.?[0-9]+)', line)

                if all([x_match, y_match, z_match]):
                    x = float(x_match.group(1))
                    y = float(y_match.group(1))
                    z = float(z_match.group(1))
                    position_manager.set_position(x, y, z)
                    logger.info(f"Position updated: X={x}, Y={y}, Z={z}")
                else:
                    missing = []
                    if not x_match: missing.append("X")
                    if not y_match: missing.append("Y")
                    if not z_match: missing.append("Z")
                    logger.warning(f"Incomplete position data: {missing} missing in: {line}")

            elif line.startswith("Target"): # FIXME: Do not exist in Arduino code
                # Extract target information if provided
                target_x_match = re.search(r'X:(\d+\.?\d*)', line)
                target_y_match = re.search(r'Y:(\d+\.?\d*)', line)

                if target_x_match and target_y_match:
                    target_x = float(target_x_match.group(1))
                    target_y = float(target_y_match.group(1))
                    position_manager.set_target(target_x, target_y)
                    logger.info(f"Updated target: X={target_x}, Y={target_y}")

            elif line.startswith("Velocity"): # FIXME: Do not exist in Arduino code
                # Extract velocity information
                vel_match = re.search(r'to: (\d+)', line)
                if vel_match:
                    velocity = int(vel_match.group(1))
                    position_manager.set_velocity(velocity)
                    logger.info(f"Updated velocity: {velocity}")

            elif "unknown command" in line.lower():
                logger.warning(f"Arduino reported unknown command: {line}")

            elif "error" in line.lower():
                logger.error(f"Arduino reported error: {line}")

        except Exception as e:
            logger.error(f"Error when processing line: {e}, line: {line}")

    def send_command(self, command):
        """Send a movement command to the Arduino"""
        if self.connected:
            try:
                if self.ser.is_open:
                    logger.info(f"Send command: {command}")
                    self.ser.write(f"{command}\n".encode())

                    # Wait for Arduino to respond
                    timeout_counter = 0
                    while self.ser.in_waiting == 0 and timeout_counter < 50:
                        time.sleep(0.01)
                        timeout_counter += 1

                    if self.ser.in_waiting > 0:
                        answer = self.ser.readline()
                        logger.info(f"Response: {answer}")
                        self.ser.reset_input_buffer()
                else:
                    logger.warning("The serial port is not open, cannot send command")
            except serial.SerialException as e:
                logger.error(f"Error sending command: {e}")
        else:
            logger.warning("Not connected to the Arduino, cannot send command")
