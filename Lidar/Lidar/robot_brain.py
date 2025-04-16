#!/usr/bin/env python3

import threading
import time
import logging
import serial
from enum import Enum
from position_manager import position_manager
import RPi.GPIO as GPIO
import numpy as np
from avoidance_system import PotentialFieldNavigation

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """States for the robot state machine"""
    START = 0
    IDLE = 1
    NAVIGATING = 2
    EXECUTING_TASK = 3
    ERROR = 4
    # TODO: Add more states as needed like PAUSED, RESUMING, etc.


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
                 movement_port='/dev/pts/3',
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
        self.avoidance_enabled = False

        # Mission variables
        self.locations = []
        self.current_location_index = -1
        self.current_task_index = -1

        # Navigation parameters
        self.position_tolerance = 2.0  # How close the robot needs to be to consider it at the target
        self.orientation_tolerance = 1.0  # Degrees
        self.obstacle_detected = False
        self.navigation_timeout = 60  # seconds
        self.navigation_start_time = 0

        # Task execution variables
        self.task_start_time = 0
        self.task_timeout = 20  # seconds

        # Lock for thread safety
        self.lock = threading.RLock()

        # GPIO configuration
        self.gpio_pin = 17  # GPIO pin for pull switch
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize potential field navigation
        self.potential_nav = PotentialFieldNavigation()

        # List of detected obstacles (to be updated from LidarThread)
        self.obstacles = []

        # Last calculated control command (for debugging)
        self.last_v = 0.0
        self.last_omega = 0.0

    def update_obstacles(self, obstacle_list):
        """Update the list of obstacles

        Arguments:
            obstacle_list: List of obstacles in format [(center_x, center_y, width, height), ...]
        """
        with self.lock:
            # Convert from rectangle format to point format for potential field navigation
            # Each obstacle is represented as just its center point
            self.obstacles = [(obstacle[0], obstacle[1]) for obstacle in obstacle_list]

            if obstacle_list:
                logger.debug(f"Updated {len(obstacle_list)} obstacles")
                if len(obstacle_list) < 5:  # Only log details for a few obstacles
                    for obs in obstacle_list:
                        logger.debug(f"Obstacle at ({obs[0]:.1f}, {obs[1]:.1f}), size: {obs[2]:.1f}x{obs[3]:.1f}")

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

    def handle_start_state(self):
        """Handle the START state"""
        # Initialize the robot and move to IDLE state
        logger.info("Initializing robot...")

        logger.info("Ready...")
        while GPIO.input(self.gpio_pin) == GPIO.HIGH:
            time.sleep(0.1)

        logger.info("Steady...")
        while GPIO.input(self.gpio_pin) == GPIO.LOW:
            time.sleep(0.1)

        logger.info("Go!")
        self.set_state(RobotState.IDLE)

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
        """Handle the NAVIGATING state with potential field navigation"""
        current_location = self.locations[self.current_location_index]

        # Check if navigation timed out
        # if time.time() - self.navigation_start_time > self.navigation_timeout:
        #     logger.warning(f"Navigation to {current_location.name} timed out")
        #     self.set_state(RobotState.ERROR)
        #     return

        # Check if obstacle detected
        if self.obstacle_detected:
            logger.warning("Obstacle detected during navigation")
            # We'll continue with potential field navigation instead of going to ERROR state
            # because the potential field should handle obstacle avoidance

        # Get current position
        current_x, current_y, current_z = position_manager.get_position()
        current_pos = (current_x, current_y)
        target_pos = (current_location.x, current_location.y)

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
                        f"GX{current_location.x:.2f}Y{current_location.y:.2f}Z{current_location.orientation:.2f}")
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
        elif self.avoidance_enabled:

            # Navigation using potential field
            # Convert orientation to radians for potential field calculation
            robot_heading_rad = np.radians(current_z)

            # Compute control commands using potential field
            v, omega = self.potential_nav.compute_control(
                robot_pos=current_pos,
                robot_heading=robot_heading_rad,
                target_pos=target_pos,
                obstacles=self.obstacles
            )

            # Save last computed control values for debugging
            self.last_v = v
            self.last_omega = omega

            # Calculate next waypoint based on potential field
            # Use a shorter look-ahead distance when obstacles are nearby
            if self.obstacles:
                look_ahead_distance = min(distance * 0.5, 5.0)  # Half of distance to target, max 5 units
            else:
                look_ahead_distance = min(distance * 0.7, 10.0)  # Longer look-ahead when no obstacles

            # Convert omega (angular velocity) to a heading change
            # Small delta_t for prediction
            delta_t = 0.1
            new_heading_rad = robot_heading_rad + omega * delta_t

            # Calculate waypoint coordinates
            waypoint_x = current_x + v * np.cos(new_heading_rad) * delta_t * look_ahead_distance
            waypoint_y = current_y + v * np.sin(new_heading_rad) * delta_t * look_ahead_distance

            # Convert heading back to degrees
            new_heading_deg = np.degrees(new_heading_rad) % 360

            # Send movement command to robot
            self.send_movement_command(f"GX{waypoint_x:.2f}Y{waypoint_y:.2f}Z{new_heading_deg:.2f}")

            # Log navigation status
            if self.obstacles:
                logger.info(
                    f"Navigating to {current_location.name} with {len(self.obstacles)} obstacles, distance: {distance:.2f}")
            else:
                logger.info(f"Navigating to {current_location.name}, distance: {distance:.2f}")

        elif not self.avoidance_enabled:
            # Simple direct navigation without potential field
            self.send_movement_command(f"GX{current_location.x:.2f}Y{current_location.y:.2f}Z{current_z:.2f}")
            logger.info(f"Directly navigating to {current_location.name}, distance: {distance:.2f}")

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
        # For now, we'll just stop, wait, and then go back to NAVIGATING
        logger.error("Handling error state")

        # Stop any movement
        self.send_movement_command("S")  # Stop command

        # Wait a bit
        time.sleep(2)

        # Go back to NAVIGATING
        self.set_state(RobotState.NAVIGATING)

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
                    params_str = ";".join([f"{k}{v}" for k, v in params.items()])

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

            if self.action_port:
                logger.info(f"Connecting to action port {self.action_port} at {self.baud_rate} baud")
                self.action_ser = serial.Serial(self.action_port, self.baud_rate, timeout=1)
            else:
                logger.info("No action port specified, skipping")

            time.sleep(2)  # Wait for connections to stabilize

            while not self.stop_event.is_set():
                # State machine
                if self.current_state == RobotState.START:
                    self.handle_start_state()
                elif self.current_state == RobotState.IDLE:
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

            GPIO.cleanup()