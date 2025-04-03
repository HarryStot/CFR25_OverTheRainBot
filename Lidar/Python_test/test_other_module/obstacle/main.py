#!/usr/bin/env python3

import time
import threading

from lidar_interface import LidarThread
import logging
import signal
from position_manager import position_manager
from robot_interface import RobotInterface
from robot_brain import RobotBrain, Location, Task, RobotState

debug = True
# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

if debug:
    logger.setLevel(logging.DEBUG)

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
    brain_stop_event = threading.Event()

    lidar_thread = None
    robot_interface = None
    robot_brain = None

    try:
        # Logger setup
        logger.info("Starting main program...")
        logger.debug("Debug mode is enabled")

        # Start the robot interface
        robot_interface = RobotInterface(serial_port='/dev/ttyACM0', baud_rate=115200,
                                         stop_event=interface_stop_event)
        robot_interface.start()
        logger.info("Starting RobotInterface...")

        # Wait for the interface to connect
        time.sleep(1)

        # Start the lidar thread
        lidar_thread = LidarThread('/dev/ttyUSB0', stop_event, end_event, debugCV=True)
        lidar_thread.start()
        logger.info("LidarThread started")

        # Start the robot brain
        robot_brain = RobotBrain(
            movement_port='/dev/ttyACM0',  # Same as robot interface
            action_port=None,  # For action commands
            stop_event=brain_stop_event
        )

        # Set up a demo mission with locations and tasks
        location1 = Location("Starting Point", 10, 10, 90, [
            Task("Task 1", "Ser", ["12:0", "13:0"], 5)
        ])

        location2 = Location("Checkpoint 1", 150, 150, 45, [])

        location3 = Location("Destination", 200, 100, 0, [])

        # Add locations to the mission
        robot_brain.add_location(location1)
        robot_brain.add_location(location2)
        robot_brain.add_location(location3)

        # Start the brain thread
        robot_brain.start()
        logger.info("RobotBrain started")

        # Show initial position and target
        pos = position_manager.get_position()
        target = position_manager.get_target()
        logger.info(f"Initial position: {pos}, target: {target}")

        # Connect the LidarThread obstacle detection to the brain
        def update_brain_obstacles():
            # This function runs periodically to update the brain about obstacles
            while running:
                # If stop_event is set by the LidarThread, an obstacle was detected
                if stop_event.is_set():
                    continue
                    # robot_brain.set_obstacle_detected(True) # TODO: UNCOMMENT!!! it's just for testing
                else:
                    robot_brain.set_obstacle_detected(False)
                time.sleep(0.1)

        # Start obstacle update thread
        obstacle_thread = threading.Thread(target=update_brain_obstacles, daemon=True)
        obstacle_thread.start()

        # Keep main thread running until CTRL+C
        while running:
            # Print the current state for debugging
            if robot_brain:
                logger.info(f"Current state: {robot_brain.current_state}")
                if robot_brain.current_state == RobotState.NAVIGATING:
                    if robot_brain.current_location_index >= 0:
                        loc = robot_brain.locations[robot_brain.current_location_index]
                        pos = position_manager.get_position()
                        distance = ((pos[0] - loc.x) ** 2 + (pos[1] - loc.y) ** 2) ** 0.5
                        logger.info(f"Navigating to {loc.name}, distance: {distance:.2f}")
                elif robot_brain.current_state == RobotState.EXECUTING_TASK:
                    if (robot_brain.current_location_index >= 0 and
                            robot_brain.current_task_index >= 0):
                        loc = robot_brain.locations[robot_brain.current_location_index]
                        task = loc.tasks[robot_brain.current_task_index]
                        elapsed = time.time() - robot_brain.task_start_time
                        logger.info(f"Executing task: {task.name} at {loc.name}, elapsed: {elapsed:.1f}s")

            time.sleep(1)

    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        # Signal to stop and end
        logger.info("Stopping all the threads...")
        stop_event.set()
        interface_stop_event.set()
        brain_stop_event.set()
        end_event.set()
        time.sleep(1)

        # Wait for threads to finish
        if robot_interface and robot_interface.is_alive():
            logger.info("Waiting for RobotInterface to end...")
            robot_interface.join(timeout=5)

        if lidar_thread and lidar_thread.is_alive():
            logger.info("Waiting for LidarThread to end...")
            lidar_thread.join(timeout=5)

        if robot_brain and robot_brain.is_alive():
            logger.info("Waiting for RobotBrain to end...")
            robot_brain.join(timeout=5)

        # Join the obstacle update thread if needed
        if obstacle_thread.is_alive():
            logger.info("Waiting for obstacle_thread to end...")
            obstacle_thread.join(timeout=2)

        logger.info("Goodbye!")


if __name__ == '__main__':
    main()
