#!/usr/bin/env python3

import threading
import time
import logging
import serial
from enum import Enum
from position_manager import position_manager

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """States for the robot state machine"""
    IDLE = 0
    NAVIGATING = 1
    EXECUTING_TASK = 2
    ERROR = 3


class Task:
    """Represents a task to be performed at a location"""

    def __init__(self, name, command, params=None, completion_time=5):
        self.name = name
        self.command = command
        self.params = params or {}
        self.completion_time = completion_time  # seconds to complete the task

    def __str__(self):
        return f"Task: {self.name} - Command: {self.command} - Params: {self.params}"


class Location:
    """Represents a location with optional tasks"""

    def __init__(self, name, x, y, orientation=None, tasks=None):
        self.name = name
        self.x = x
        self.y = y
        self.orientation = orientation
        self.tasks = tasks or []

    def __str__(self):
        return f"Location: {self.name} at ({self.x}, {self.y})"


class RobotBrain(threading.Thread):
    """Main controller for the robot behavior"""

    def __init__(self,
                 movement_port='/dev/ttyACM0',
                 action_port='/dev/pts/4',
                 baud_rate=115200,
                 stop_event=None):
        super().__init__()
        self.movement_port = movement_port
        self.action_port = action_port
        self.baud_rate = baud_rate
        self.stop_event = stop_event or threading.Event()
        self.daemon = True

        # Initialize serial ports
        self.movement_ser = None
        self.action_ser = None

        # State machine variables
        self.current_state = RobotState.NAVIGATING
        self.previous_state = None
        self.state_changed = threading.Event()

        # Mission variables
        self.locations = []
        self.current_location_index = -1
        self.current_task_index = -1

        # Navigation parameters
        self.position_tolerance = 5.0  # How close the robot needs to be to consider it at the target
        self.orientation_tolerance = 2.0  # Degrees
        self.obstacle_detected = False
        self.navigation_timeout = 60  # seconds
        self.navigation_start_time = 0

        # Task execution variables
        self.task_start_time = 0
        self.task_timeout = 300  # seconds

        # Lock for thread safety
        self.lock = threading.RLock()

    def add_location(self, location):
        """Add a location to the mission"""
        with self.lock:
            self.locations.append(location)
            logger.info(f"Added location: {location}")

    def clear_locations(self):
        """Clear all locations"""
        with self.lock:
            self.locations = []
            self.current_location_index = -1
            self.current_task_index = -1
            logger.info("Cleared all locations")

    def set_state(self, new_state):
        """Change the robot state"""
        with self.lock:
            if new_state != self.current_state:
                self.previous_state = self.current_state
                self.current_state = new_state
                self.state_changed.set()
                logger.info(f"State changed: {self.previous_state} -> {self.current_state}")

    def handle_idle_state(self):
        """Handle the IDLE state"""
        if self.locations and self.current_location_index < len(self.locations) - 1:
            # Move to the next location
            self.current_location_index += 1
            self.current_task_index = -1
            logger.info(f"Moving to next location: {self.locations[self.current_location_index]}")
            self.set_state(RobotState.NAVIGATING)
            self.navigation_start_time = time.time()
        else:
            # No more locations, wait for new ones
            time.sleep(0.5)

    def handle_navigating_state(self):
        """Handle the NAVIGATING state"""
        current_location = self.locations[self.current_location_index]

        # Check if navigation timed out
        if time.time() - self.navigation_start_time > self.navigation_timeout:
            logger.warning(f"Navigation to {current_location.name} timed out")
            self.set_state(RobotState.ERROR)
            return

        # Check if obstacle detected
        if self.obstacle_detected:
            logger.warning("Obstacle detected during navigation")
            self.set_state(RobotState.ERROR)
            return

        # Get current position
        current_x, current_y, current_z = position_manager.get_position()

        # Calculate distance to target
        distance = ((current_x - current_location.x) ** 2 +
                    (current_y - current_location.y) ** 2) ** 0.5

        # Check if we're at the target location
        if distance <= self.position_tolerance:
            # Check orientation if specified
            if current_location.orientation is not None:
                orientation_diff = abs(current_z - current_location.orientation) % 360
                orientation_diff = min(orientation_diff, 360 - orientation_diff)

                if orientation_diff > self.orientation_tolerance:
                    # Need to adjust orientation
                    self.send_movement_command(
                        f"GX{current_location.x},Y{current_location.y},Z{current_location.orientation}")
                    logger.info(f"Adjusting orientation to {current_location.orientation} degrees")
                    return

            logger.info(f"Reached location: {current_location.name}")

            # If there are tasks to perform, move to EXECUTING_TASK state
            if current_location.tasks:
                self.current_task_index = 0
                self.set_state(RobotState.EXECUTING_TASK)
                self.task_start_time = time.time()
            else:
                # No tasks, go back to IDLE
                self.set_state(RobotState.IDLE)
        else:
            # Send movement command to the target
            if current_location.orientation is not None:
                self.send_movement_command(
                    f"GX{current_location.x},Y{current_location.y},Z{current_location.orientation}")
            else:
                self.send_movement_command(f"GX{current_location.x},Y{current_location.y}")

            logger.info(f"Navigating to {current_location.name}, distance: {distance:.2f}")

    def handle_executing_task_state(self):
        """Handle the EXECUTING_TASK state"""
        current_location = self.locations[self.current_location_index]
        current_task = current_location.tasks[self.current_task_index]

        # Check if task execution timed out
        if time.time() - self.task_start_time > self.task_timeout:
            logger.warning(f"Task {current_task.name} timed out")
            self.set_state(RobotState.ERROR)
            return

        # Check if the task has been running long enough
        task_elapsed_time = time.time() - self.task_start_time
        if task_elapsed_time >= current_task.completion_time:
            logger.info(f"Completed task: {current_task.name}")

            # Move to the next task or back to IDLE
            self.current_task_index += 1
            if self.current_task_index < len(current_location.tasks):
                # Start the next task
                current_task = current_location.tasks[self.current_task_index]
                logger.info(f"Starting task: {current_task.name}")
                self.send_action_command(current_task.command, current_task.params)
                self.task_start_time = time.time()
            else:
                # All tasks complete, go back to IDLE
                logger.info(f"All tasks completed at location: {current_location.name}")
                self.set_state(RobotState.IDLE)
        else:
            # Task still in progress
            time.sleep(0.1)

    def handle_error_state(self):
        """Handle the ERROR state"""
        # In a real implementation, you might want more sophisticated error recovery
        # For now, we'll just stop, wait, and then go back to IDLE
        logger.error("Handling error state")

        # Stop any movement
        self.send_movement_command("STOP")

        # Wait a bit
        time.sleep(2)

        # Go back to IDLE
        self.set_state(RobotState.IDLE)

    def send_movement_command(self, command):
        """Send a command to the movement serial port"""
        if self.movement_ser and self.movement_ser.is_open:
            try:
                full_command = f"{command}\r\n"
                self.movement_ser.write(full_command.encode())
                logger.info(f"Sent movement command: {command}")
            except Exception as e:
                logger.error(f"Error sending movement command: {e}")
                self.set_state(RobotState.ERROR)

    def send_action_command(self, command, params=None):
        """Send a command to the action serial port"""
        if self.action_ser and self.action_ser.is_open:
            try:
                # Build the command string based on the command and params
                params_str = ""
                if params:
                    params_str = "," + ",".join([f"{k}{v}" for k, v in params.items()])

                full_command = f"{command}{params_str}\r\n"
                self.action_ser.write(full_command.encode())
                logger.info(f"Sent action command: {command}{params_str}")
            except Exception as e:
                logger.error(f"Error sending action command: {e}")
                self.set_state(RobotState.ERROR)

    def set_obstacle_detected(self, detected):
        """Set the obstacle detection flag"""
        with self.lock:
            if detected != self.obstacle_detected:
                self.obstacle_detected = detected
                logger.info(f"Obstacle detection changed: {detected}")

    def run(self):
        """Main brain thread function"""
        try:
            # Connect to serial ports
            logger.info(f"Connecting to movement port {self.movement_port} at {self.baud_rate} baud")
            self.movement_ser = serial.Serial(self.movement_port, self.baud_rate, timeout=1)
            if not self.movement_ser.is_open:
                logger.error(f"Failed to open movement port {self.movement_port}")

            logger.info(f"Connecting to action port {self.action_port} at {self.baud_rate} baud")
            self.action_ser = serial.Serial(self.action_port, self.baud_rate, timeout=1)
            if not self.action_ser.is_open:
                logger.error(f"Failed to open action port {self.action_port}")

            time.sleep(2)  # Wait for connections to stabilize

            while not self.stop_event.is_set():
                # State machine
                if self.current_state == RobotState.IDLE:
                    self.handle_idle_state()
                elif self.current_state == RobotState.NAVIGATING:
                    self.handle_navigating_state()
                elif self.current_state == RobotState.EXECUTING_TASK:
                    self.handle_executing_task_state()
                elif self.current_state == RobotState.ERROR:
                    self.handle_error_state()

                time.sleep(0.05)  # Small sleep to prevent CPU hogging

        except Exception as e:
            logger.error(f"Error in RobotBrain: {e}")

        finally:
            # Clean up resources
            if self.movement_ser and self.movement_ser.is_open:
                self.movement_ser.close()
                logger.info("Movement serial port closed")

            if self.action_ser and self.action_ser.is_open:
                self.action_ser.close()
                logger.info("Action serial port closed")
