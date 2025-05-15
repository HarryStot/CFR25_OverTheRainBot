#!/usr/bin/env python3

import logging
import signal
import threading
import time

from lidar_interface import LidarThread
from position_manager import position_manager, Team
from robot_brain import RobotBrain, RobotState
from robot_interface import RobotInterface

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
    """
    Handles system signals to trigger a graceful shutdown of the application.

    This function is typically used to intercept signals like SIGINT or SIGTERM
    to perform cleanup actions and stop the application safely by setting
    a global variable to signal that the application should terminate.
    It also logs the receipt of the signal for debugging or monitoring purposes.

    :param sig: Signal number intercepted by the signal handler.
        This value corresponds to system-specific signals like SIGINT or
        SIGTERM and is provided automatically by the operating system.
    :type sig: int
    :param frame: Execution stack frame at the time the signal was
        received. This is primarily used for debugging purposes and is
        automatically provided by the Python runtime.
    :type frame: Optional[types.FrameType]
    """
    global running
    logger.info("Received shutdown signal, stopping...")
    running = False


def main():
    """
    Main function of the robot control system. It initializes and manages multiple
    components such as robot interfaces, lidar thread, the robot brain, and other
    necessary background threads. The function includes signal handling, thread
    management, and communication between components to ensure seamless operation.

    The main responsibilities include:

    - Initializing and starting the robot interfaces to control movement and actions.
    - Initializing the lidar thread to handle obstacle detection and perception.
    - Setting up and running the robot brain to manage high-level state transitions and tasks.
    - Periodically updating internal states, such as obstacle detection and navigation information.
    - Managing safe shutdown of all components and resources in the case of interruptions or errors.

    Sections:

    - Lidar thread setup: The LidarThread is initialized to obtain obstacle data for navigation.
    - Robot interfaces setup: Movement and action interfaces are initialized to ensure communication
      with the robot's hardware for low-level control.
    - RobotBrain setup: The high-level control logic of the robot is managed by the RobotBrain.
    - Obstacle detection update: A separate thread is spawned to monitor and update obstacle
      information.
    - Main loop: The main loop ensures that tasks, navigation, and state management proceed
      while monitoring the system's health.
    - Graceful shutdown: Ensures that all threads and hardware interfaces are stopped
      safely when the program exits.

    :return: None
    """
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    stop_event = threading.Event()
    end_event = threading.Event()
    movement_interface_stop_event = threading.Event()
    action_interface_stop_event = threading.Event()
    brain_stop_event = threading.Event()

    lidar_thread = None
    movement_robot_interface = None
    action_robot_interface = None
    robot_brain = None

    try:
        # Logger setup
        logger.info("Starting main program...")
        logger.debug("Debug mode is enabled")

        # Start the robot's interfaces
        movement_robot_interface = RobotInterface(serial_port='/dev/ttyACM0', baud_rate=115200,
                                                  stop_event=movement_interface_stop_event)
        movement_robot_interface.start()
        action_robot_interface = RobotInterface(serial_port='/dev/ttyACM1', baud_rate=115200,
                                                stop_event=action_interface_stop_event)
        action_robot_interface.start()
        logger.info("Starting RobotInterfaces...")

        # Wait longer for interfaces to connect
        wait_time = 5  # seconds
        logger.info(f"Waiting {wait_time} seconds for interfaces to connect...")
        time.sleep(wait_time)

        # Start the lidar thread
        lidar_thread = LidarThread('/dev/ttyUSB0', stop_event, end_event, debugCV=True)
        lidar_thread.start()
        logger.info("LidarThread started")

        # Start the robot brain - begins with team selection state
        robot_brain = RobotBrain(
            movement_interface=movement_robot_interface,
            action_interface=action_robot_interface,
            stop_event=brain_stop_event
        )

        # Set the initial state to team selection
        robot_brain.set_state(RobotState.TEAM_SELECTION)

        robot_brain.send_lcd_message("ROBOT STARTING", "SELECT TEAM")

        # Start the brain thread
        robot_brain.start()
        logger.info("RobotBrain started in team selection state")

        # Connect the LidarThread obstacle detection to the brain
        def update_brain_obstacles():
            # This function runs periodically to update the brain about obstacles
            while running:
                if lidar_thread:
                    # Get obstacles from lidar thread
                    obstacles = lidar_thread.get_obstacles()
                    # Update the brain with obstacle data
                    robot_brain.update_obstacles(obstacles)

                    # Update the obstacle_detected flag based on distance to obstacles
                    obstacle_detected = False
                    if obstacles:
                        robot_x, robot_y = position_manager.get_position()[:2]
                        for obstacle in obstacles:
                            # Calculate obstacle center
                            obstacle_center_x = obstacle[0] + obstacle[2] / 2
                            obstacle_center_y = obstacle[1] + obstacle[3] / 2

                            # Calculate distance to obstacle center
                            distance_to_obstacle = ((robot_x - obstacle_center_x) ** 2 +
                                                    (robot_y - obstacle_center_y) ** 2) ** 0.5

                            # Check if distance is within tolerance (including obstacle size)
                            threshold = robot_brain.position_tolerance + max(obstacle[2], obstacle[3]) / 2
                            if distance_to_obstacle < threshold:
                                obstacle_detected = True
                                break  # No need to check other obstacles if one is close
                        robot_brain.set_obstacle_detected(obstacle_detected)
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

                # Update position manager with team selection if it was changed in robot_brain
                if robot_brain.current_state != RobotState.TEAM_SELECTION and robot_brain.is_blue_team is not None:
                    team = Team.BLUE if robot_brain.is_blue_team else Team.YELLOW
                    # Only update if needed to avoid constant resets
                    if team != position_manager.get_team():
                        position_manager.set_team(team)
                        logger.info(f"PositionManager updated with team: {team.name}")

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
        if robot_brain:
            # Use a shorter error message to fit on LCD display
            error_msg = str(e)
            if len(error_msg) > 16:
                error_msg = error_msg[:13] + "..."
            robot_brain.send_lcd_message("ERROR IN MAIN", error_msg)

    finally:
        # Signal to stop and end
        logger.info("Stopping all the threads...")
        if robot_brain:
            robot_brain.send_movement_command("S")
        stop_event.set()
        movement_interface_stop_event.set()
        action_interface_stop_event.set()
        brain_stop_event.set()
        end_event.set()

        time.sleep(1)

        # Wait for threads to finish
        if movement_robot_interface and movement_robot_interface.is_alive():
            logger.info("Waiting for RobotInterface to end...")
            movement_robot_interface.join(timeout=5)

        if action_robot_interface and action_robot_interface.is_alive():
            logger.info("Waiting for ActionRobotInterface to end...")
            action_robot_interface.join(timeout=5)

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
