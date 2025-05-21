# robot_interface.py
import time
import threading

from position_manager import position_manager
import logging
import serial
import re

logger = logging.getLogger(__name__)


class RobotInterface(threading.Thread):
    """
    Manages communication with an Arduino-based robot through a serial interface.

    This class handles establishing a serial connection with the robot, sending
    commands, receiving data, and interpreting messages from the robot. It extends
    `threading.Thread` to operate on a separate thread, continuously monitoring the
    connection and processing data from the robot.

    :ivar serial_port: Path to the serial port used to communicate with the robot.
    :type serial_port: str
    :ivar baud_rate: Baud rate used for serial communication.
    :type baud_rate: int
    :ivar stop_event: Event object used to signal the thread to stop execution.
    :type stop_event: threading.Event
    :ivar daemon: Indicates whether the thread should run as a daemon thread.
    :type daemon: bool
    :ivar ser: Object to represent the serial connection with the robot.
    :type ser: serial.Serial or None
    :ivar connected: Connection state with the robot, True if connected.
    :type connected: bool
    :ivar position_received: Event object triggered when a position update is received.
    :type position_received: threading.Event
    :ivar buffer: Temporary storage for data received from the robot for line-based processing.
    :type buffer: str
    :ivar interface_type: Type of interface - either "movement" or "action".
    :type interface_type: str
    :ivar ultrasonic_data: Dictionary storing the latest readings from ultrasonic sensors.
    :type ultrasonic_data: dict
    :ivar us_data_received: Event object triggered when ultrasonic sensor data is received.
    :type us_data_received: threading.Event
    """

    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200, stop_event=None, interface_type="movement"):
        super().__init__()
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.stop_event = stop_event or threading.Event()
        self.daemon = True
        self.ser = None
        self.connected = False
        self.position_received = threading.Event()
        self.us_data_received = threading.Event()
        self.buffer = ""
        self.interface_type = interface_type  # "movement" or "action"
        self.ultrasonic_data = {}  # Store ultrasonic sensor data {sensor_id: distance}
        # Nouveaux événements pour la navigation et l'orientation
        self.navigation_stopped = threading.Event()
        self.orientation_stopped = threading.Event()

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

                    # Initialize based on interface type
                    if self.interface_type == "movement":
                        logger.info("Movement interface initialized.")
                    elif self.interface_type == "action":
                        logger.info("Action interface initialized.")

                    self.connected = True
                    retry_count = 0

                    logger.info("Starting to read data from Arduino...")
                    while not self.stop_event.is_set():
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
                                        logger.debug(f"Received: {line}")
                                        self.process_line(line)
                            except UnicodeDecodeError:
                                logger.warning("Invalid data received, emptying buffer")
                                self.buffer = ""

                        time.sleep(0.001)  # Waiting 1 ms

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
                logger.warning("Robot disconnected")

        if retry_count >= max_retries:
            logger.error(f"Connection failed after {max_retries} attempts")

    def process_line(self, line):
        """Process a line received from the Arduino"""
        try:
            # Détection de la fin de navigation
            if line.strip() == "STOP_NAVIGATION":
                self.navigation_stopped.set()
                logger.info("STOP_NAVIGATION receive from Arduino")
                return
            # Détection de la fin d'orientation
            if line.strip() == "STOP_ORIENTATION":
                self.orientation_stopped.set()
                logger.info("STOP_ORIENTATION receive from Arduino")
                return
            # Process position data (movement interface)
            if "POS" in line:
                self.position_received.set()

                x_match = re.search(r'X:?\s*([-+]?[0-9]*\.?[0-9]+)', line)
                y_match = re.search(r'Y:?\s*([-+]?[0-9]*\.?[0-9]+)', line)
                z_match = re.search(r'Z:?\s*([-+]?[0-9]*\.?[0-9]+)', line)

                if all([x_match, y_match, z_match]):
                    x = float(x_match.group(1)) * 100
                    y = float(y_match.group(1)) * 100
                    z = float(z_match.group(1)) * 100
                    position_manager.set_position(x, y, z)
                    logger.debug(f"Position updated: X={x}, Y={y}, Z={z}")
                else:
                    missing = []
                    if not x_match: missing.append("X")
                    if not y_match: missing.append("Y")
                    if not z_match: missing.append("Z")
                    logger.warning(f"Incomplete position data: {missing} missing in: {line}")

            # Process ultrasonic sensor data (action interface)
            elif "US" in line:
                self.us_data_received.set()
                # Parse format US<id>:<dis>;<id>:<dis>
                us_data = re.findall(r'(\d+):(\d+\.?\d*)', line)

                if us_data:
                    # Clear existing data before updating
                    self.ultrasonic_data.clear()

                    for sensor_id, distance in us_data:
                        self.ultrasonic_data[int(sensor_id)] = float(distance)

                    logger.debug(f"Ultrasonic data updated: {self.ultrasonic_data}")
                else:
                    logger.warning(f"Invalid ultrasonic data format: {line}")

            elif "unknown command" in line.lower():
                logger.warning(f"Arduino reported unknown command: {line}")

            elif "error" in line.lower():
                logger.error(f"Arduino reported error: {line}")

        except Exception as e:
            logger.error(f"Error when processing line: {e}, line: {line}")

    def send_command(self, command):
        """
        Send a command to the Arduino without waiting for a response.
        The interface will continuously listen for data in its main loop.
        """
        if self.connected:
            try:
                if self.ser.is_open:
                    logger.info(f"Send command: {command}")
                    self.ser.write(f"{command}\n".encode())
                else:
                    logger.warning("The serial port is not open, cannot send command")
            except serial.SerialException as e:
                logger.error(f"Error sending command: {e}")
        else:
            logger.warning("Not connected to the Arduino, cannot send command")

    def get_ultrasonic_data(self):
        """
        Get the latest ultrasonic sensor readings

        :return: Dictionary of sensor readings {sensor_id: distance}
        :rtype: dict
        """
        return self.ultrasonic_data.copy()  # Return a copy to avoid thread issues

    def request_sensor_data(self):
        """Request ultrasonic sensor data from Arduino (action interface)"""
        if self.interface_type == "action" and self.connected:
            self.send_command("R")  # Assuming 'R' is the command to request sensor data
        else:
            logger.warning("Cannot request sensor data: interface type is not 'action' or not connected")
