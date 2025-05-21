#!/usr/bin/env python3

import logging
import signal
import threading
import time

from robot_interface import RobotInterface

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

running = True

def signal_handler(sig, frame):
    global running
    logger.info("Received shutdown signal, stopping...")
    running = False


def main():
    signal.signal(signal.SIGINT, signal_handler)

    stop_event = threading.Event()
    robot_interface = None

    try:
        # Robot interface initialization
        logger.info("Starting robot interface...")
        robot_interface = RobotInterface(serial_port='/dev/ttyACM0',
                                         baud_rate=115200,
                                         stop_event=stop_event)
        robot_interface.start()
        logger.info("Robot interface started")

        # Wait for the robot to be ready
        time.sleep(3)
        robot_interface.send_command("S")
        robot_interface.send_command("INITX1.775Y0.05Z1.57")
        # robot_interface.send_command("V150")
        time.sleep(1)

        logger.info("Sending command to move 1 meter...")
        robot_interface.send_command("GX1.9Y0.80Z1.57")
        # robot_interface.send_command("R2")

        # Modified loop to use the global running variable
        while not robot_interface.navigation_stopped.is_set() and running:
            time.sleep(0.1)

        # Check if we exited due to signal
        if not running:
            logger.info("Operation cancelled by user")
            return

        while not robot_interface.orientation_stopped.is_set() and running:
            time.sleep(0.1)

        # Check again if we exited due to signal
        if not running:
            logger.info("Operation cancelled by user")
            return

        robot_interface.navigation_stopped.clear()
        robot_interface.orientation_stopped.clear()

        robot_interface.send_command("GX1.9Y0.80Z0")

        while not robot_interface.navigation_stopped.is_set() and running:
            time.sleep(0.1)

        if not running:
            logger.info("Operation cancelled by user")
            return

        while not robot_interface.orientation_stopped.is_set() and running:
            time.sleep(0.1)

        if not running:
            logger.info("Operation cancelled by user")
            return

        robot_interface.navigation_stopped.clear()
        robot_interface.orientation_stopped.clear()

        robot_interface.send_command("GX1.9Y0.80Z-1.57")

        while not robot_interface.navigation_stopped.is_set() and running:
            time.sleep(0.1)

        if not running:
            logger.info("Operation cancelled by user")
            return

        while not robot_interface.orientation_stopped.is_set() and running:
            time.sleep(0.1)

        if not running:
            logger.info("Operation cancelled by user")
            return

        robot_interface.navigation_stopped.clear()
        robot_interface.orientation_stopped.clear()

        # Stopping the robot
        logger.info("Stopping the robot...")
        robot_interface.send_command("S")

    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        logger.info("Stopping the robot...")
        robot_interface.send_command("S")

        logger.info("Stopping all threads...")
        stop_event.set()

        if robot_interface and robot_interface.is_alive():
            logger.info("Waiting for robot interface to finish...")
            robot_interface.join(timeout=5)

        logger.info("Bye!")


if __name__ == '__main__':
    main()