#!/usr/bin/env python3

import time
import threading
import logging
import os
import sys
import serial
import signal

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Global flag for graceful shutdown
running = True


def signal_handler(sig, frame):
    global running
    logger.info("Received shutdown signal, stopping...")
    running = False


def detect_arduino_port():
    """
    Try to automatically detect the Arduino serial port.
    Returns the first available serial port or defaults to '/dev/ttyACM0'.
    """
    import serial.tools.list_ports

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        logger.warning("No serial ports found. Defaulting to '/dev/ttyACM0'")
        return '/dev/ttyACM0'

    # Look for Arduino-like port names
    arduino_ports = [
        p.device for p in ports
        if 'Arduino' in p.description or 'ACM' in p.device or 'USB' in p.device
    ]

    if arduino_ports:
        logger.info(f"Detected Arduino port: {arduino_ports[0]}")
        return arduino_ports[0]

    # If no Arduino port found, use the first available
    logger.info(f"No Arduino-specific port found. Using: {ports[0].device}")
    return ports[0].device


def test_serial_connection(port, baud_rate=115200, timeout=2):
    """Test if we can establish a serial connection to the specified port."""
    try:
        ser = serial.Serial(port, baud_rate, timeout=timeout)
        if ser.is_open:
            logger.info(f"Successfully connected to {port}")
            ser.close()
            return True
        return False
    except Exception as e:
        logger.error(f"Failed to connect to {port}: {e}")
        return False


def main():
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Try to detect Arduino port
    arduino_port = detect_arduino_port()

    # Test connection to the Arduino
    if not test_serial_connection(arduino_port):
        logger.error("Failed to connect to Arduino. Please check connections and try again.")
        return

    # Import your modules only after confirming serial connection
    try:
        from position_manager import position_manager
        from robot_interface import RobotInterface
        from test_lidar_REC import LidarThread
    except ImportError as e:
        logger.error(f"Failed to import required modules: {e}")
        return

    stop_event = threading.Event()
    end_event = threading.Event()
    interface_stop_event = threading.Event()

    lidar_thread = None
    robot_interface = None

    try:
        # Start the robot interface
        logger.info(f"Starting RobotInterface on port {arduino_port}")
        robot_interface = RobotInterface(serial_port=arduino_port, baud_rate=115200,
                                         stop_event=interface_stop_event)
        robot_interface.start()

        # Wait for the interface to connect
        time.sleep(2)

        # Test basic movement commands
        logger.info("Testing basic movement commands...")
        with serial.Serial(arduino_port, 115200, timeout=1) as ser:
            # Set velocity
            ser.write(b"V50\n")
            time.sleep(0.5)

            # Move to position (100, 100, 90)
            ser.write(b"GX100,Y100,Z90\n")
            time.sleep(2)

            # Request position
            ser.write(b"P\n")
            time.sleep(0.5)

            # Read response
            while ser.in_waiting:
                response = ser.readline().decode('utf-8').strip()
                logger.info(f"Arduino response: {response}")

        # Get current position from position manager
        pos = position_manager.get_position()
        target = position_manager.get_target()
        logger.info(f"Position manager reports: position={pos}, target={target}")

        # Check if LIDAR is required and available
        try_lidar = input("Do you want to test LIDAR functionality? (y/n): ").lower() == 'y'
        if try_lidar:
            lidar_port = '/dev/ttyUSB0'  # Default LIDAR port
            alt_lidar_port = input(f"Enter LIDAR port (default: {lidar_port}): ")
            if alt_lidar_port:
                lidar_port = alt_lidar_port

            # Try to import RPLidar
            try:
                from rplidar import RPLidar
                logger.info("Starting LidarThread...")
                lidar_thread = LidarThread(lidar_port, stop_event, end_event, debugCV=True)
                lidar_thread.start()
            except ImportError:
                logger.error("RPLidar package not installed. Run 'pip install rplidar' to install.")
            except Exception as e:
                logger.error(f"Failed to start LIDAR: {e}")

        # Keep the test running until interrupted
        logger.info("System is running. Press Ctrl+C to stop.")
        while running:
            time.sleep(1)
            # Get position updates
            pos = position_manager.get_position()
            target = position_manager.get_target()
            logger.info(f"Current position: {pos}, target: {target}")

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    except Exception as e:
        logger.error(f"Error in test: {e}")
    finally:
        # Signal to stop all threads
        logger.info("Stopping all threads...")
        stop_event.set()
        interface_stop_event.set()
        end_event.set()

        # Wait for threads to finish
        if robot_interface and robot_interface.is_alive():
            robot_interface.join(timeout=5)

        if lidar_thread and lidar_thread.is_alive():
            lidar_thread.join(timeout=5)

        logger.info("Test complete")


if __name__ == '__main__':
    main()