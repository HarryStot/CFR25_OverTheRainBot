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

        # Add a flag to track connection status
        self.connected = False

        # Add an event to signal when position data is received
        self.position_received = threading.Event()

    def run(self):
        retry_count = 0
        max_retries = 5

        while not self.stop_event.is_set() and retry_count < max_retries:
            try:
                logger.info(f"Connecting to port {self.serial_port} at {self.baud_rate} baud")
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                time.sleep(2)  # Give the Arduino time to reset after opening serial

                # Clear any startup messages
                self.ser.reset_input_buffer()

                # Send a position request to start getting data
                logger.info("Requesting initial position from Arduino")
                self.ser.write(b"P\n")

                self.connected = True
                retry_count = 0  # Reset retry count on successful connection

                logger.info("Starting to read from serial port...") # TODO: Adjust log level ?
                while not self.stop_event.is_set():
                    logger.debug("Checking for data...") # TODO: Remove or adjust log level
                    if self.ser.in_waiting > 0:
                        try:
                            line = self.ser.readline().decode('utf-8').strip()
                            if line:  # Ignore empty lines
                                logger.debug(f"Received: {line}")
                                self.process_line(line)
                        except UnicodeDecodeError:
                            logger.warning("Received invalid data, skipping")
                    else:
                        logger.debug("No data available")

                    # Periodically request position updates
                    time.sleep(0.1)

                    # Send position request every few seconds if no data received
                    if not self.position_received.is_set():
                        self.ser.write(b"P\n")
                        self.position_received.wait(timeout=0.5)
                        self.position_received.clear()

            except serial.SerialException as e:
                logger.error(f"Serial error: {e}")
                retry_count += 1
                logger.info(f"Retrying connection ({retry_count}/{max_retries})...")
                time.sleep(2)  # Wait before retry

            except Exception as e:
                logger.error(f"Error in RobotInterface: {e}")
                break

            finally:
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                        logger.info("Serial port closed")
                    except Exception as e:
                        logger.error(f"Error closing serial port: {e}")

                self.connected = False

        if retry_count >= max_retries:
            logger.error(f"Failed to connect after {max_retries} attempts")

    def process_line(self, line):
        """Process a line received from the Arduino"""
        if line.startswith("POS,"):
            self.position_received.set()

            # Use regular expressions to extract position values
            x_match = re.search(r'X:([-+]?\d*\.?\d+)', line)
            y_match = re.search(r'Y:([-+]?\d*\.?\d+)', line)
            z_match = re.search(r'Z:([-+]?\d*\.?\d+)', line)

            x = y = z = None

            if x_match:
                x = float(x_match.group(1))
            if y_match:
                y = float(y_match.group(1))
            if z_match:
                z = float(z_match.group(1))

            if x is not None and y is not None and z is not None:
                position_manager.set_position(x, y, z)
                logger.info(f"Updated position: X={x}, Y={y}, Z={z}")
            else:
                logger.warning(f"Incomplete position data in: {line}")

        elif line.startswith("Target"):
            # Extract target information if provided
            target_x_match = re.search(r'X:(\d+\.?\d*)', line)
            target_y_match = re.search(r'Y:(\d+\.?\d*)', line)

            if target_x_match and target_y_match:
                target_x = float(target_x_match.group(1))
                target_y = float(target_y_match.group(1))
                position_manager.set_target(target_x, target_y)
                logger.info(f"Updated target: X={target_x}, Y={target_y}")

        elif line.startswith("Velocity"):
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