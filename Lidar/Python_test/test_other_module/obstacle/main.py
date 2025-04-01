#!/usr/bin/env python3

import time
import threading
from test_lidar_REC import LidarThread
import logging
import signal
from position_manager import position_manager
from robot_interface import RobotInterface

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


def main():
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    stop_event = threading.Event()
    end_event = threading.Event()
    interface_stop_event = threading.Event()

    lidar_thread = None
    robot_interface = None

    try:
        # Start the robot interface
        robot_interface = RobotInterface(serial_port='/dev/pts/3', baud_rate=115200,
                                         stop_event=interface_stop_event)
        robot_interface.start()
        logger.info("Starting RobotInterface...")

        # Wait for the interface to connect
        time.sleep(1)

        # Start the lidar thread
        lidar_thread = LidarThread('/dev/ttyUSB0', stop_event, end_event, debugCV=True)
        lidar_thread.start()
        logger.info("LidarThread started")

        # Show initial position and target
        pos = position_manager.get_position()
        target = position_manager.get_target()
        logger.info(f"Initial position: {pos}, target: {target}")

        # Keep main thread running until CTRL+C
        while running:
            time.sleep(0.1)

    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        # Signal to stop and end
        logger.info("Stopping all the threads...")
        stop_event.set()
        interface_stop_event.set()
        end_event.set()
        time.sleep(1)

        # Wait for threads to finish
        if robot_interface and robot_interface.is_alive():
            logger.info("Waiting the end ofRobotInterface...")
            robot_interface.join(timeout=5)

        if lidar_thread and lidar_thread.is_alive():
            logger.info("Waiting the end of LidarThread...")
            lidar_thread.join(timeout=5)

        logger.info("Goodbye!")


if __name__ == '__main__':
    main()
