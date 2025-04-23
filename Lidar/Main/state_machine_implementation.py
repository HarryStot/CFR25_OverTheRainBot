"""
New state machine implementation for the robot controller.
This defines the states and handlers for team selection, pull switch waiting,
time-based navigation, and returning to the end zone.
"""

import time
import logging
import numpy as np
from enum import Enum
import RPi.GPIO as GPIO

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """States for the robot state machine"""
    TEAM_SELECTION = 0     # Initial state to select team (blue/yellow)
    WAITING_FOR_START = 1  # Waiting for pull switch
    NAVIGATING = 2         # Moving to a target location
    EXECUTING_TASK = 3     # Performing a task at the current location
    RETURNING_TO_END = 4   # Going to the end zone when time is almost up
    COMPLETED = 5          # Mission completed
    ERROR = 6              # Error state


# State handler methods to be added to RobotBrain class

def handle_team_selection_state(self):
    """Handle the TEAM_SELECTION state - determine team based on switch position"""
    # Check team selection switch
    if GPIO.input(self.team_select_pin) == GPIO.HIGH:
        self.is_blue_team = True
        logger.info("Blue team selected")
    else:
        self.is_blue_team = False
        logger.info("Yellow team selected")

    # Load mission locations based on team
    self.load_team_missions()
    
    # Move to waiting for start
    self.set_state(RobotState.WAITING_FOR_START)


def handle_waiting_for_start_state(self):
    """Handle the WAITING_FOR_START state - wait for pull switch activation"""
    # Check if pull switch has been activated
    if GPIO.input(self.pull_switch_pin) == GPIO.LOW:
        logger.info("Pull switch activated! Starting mission...")
        self.mission_start_time = time.time()
        
        # Start the mission with the first location
        self.current_location_index = 0
        self.current_task_index = -1
        
        # Move to navigating state to go to first location
        self.set_state(RobotState.NAVIGATING)
        self.navigation_start_time = time.time()
    else:
        # Still waiting for pull switch activation
        time.sleep(0.1)


def load_team_missions(self):
    """Load mission locations based on selected team"""
    self.clear_locations()
    
    if self.is_blue_team:
        # Blue team mission waypoints
        self.add_location(Location("BlueStart", 30, 30, 0))
        self.add_location(Location("BlueActionPoint1", 50, 70, 90, [
            Task("GrabBlueItem", "GRAB", {"S": 1}, 3)
        ]))
        self.add_location(Location("BlueActionPoint2", 90, 80, 180, [
            Task("DropBlueItem", "DROP", {"S": 1}, 2)
        ]))
        # Add more blue team locations as needed
        self.add_location(Location("BlueEndZone", 180, 30, 270))
    else:
        # Yellow team mission waypoints
        self.add_location(Location("YellowStart", 270, 30, 180))
        self.add_location(Location("YellowActionPoint1", 250, 70, 90, [
            Task("GrabYellowItem", "GRAB", {"S": 2}, 3)
        ]))
        self.add_location(Location("YellowActionPoint2", 210, 80, 0, [
            Task("DropYellowItem", "DROP", {"S": 2}, 2)
        ]))
        # Add more yellow team locations as needed
        self.add_location(Location("YellowEndZone", 120, 30, 270))
        
    logger.info(f"Loaded {len(self.locations)} locations for {'blue' if self.is_blue_team else 'yellow'} team")


def handle_navigating_state(self):
    """Handle the NAVIGATING state with potential field navigation"""
    # Check if we need to return to end zone based on time
    elapsed_time = time.time() - self.mission_start_time
    remaining_time = self.mission_duration - elapsed_time
    
    if remaining_time <= self.end_zone_time:
        logger.warning(f"Only {remaining_time:.1f} seconds remaining! Heading to end zone.")
        self.set_state(RobotState.RETURNING_TO_END)
        return
    
    # Continue with the original navigation logic
    current_location = self.locations[self.current_location_index]

    # Check if navigation timed out
    if time.time() - self.navigation_start_time > self.navigation_timeout:
        logger.warning(f"Navigation to {current_location.name} timed out")
        self.set_state(RobotState.ERROR)
        return

    # Check if obstacle detected
    if self.obstacle_detected:
        logger.warning("Obstacle detected during navigation")
        # Continue with potential field navigation for obstacle avoidance

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
        self.send_movement_command("S")  # Stop the robot
        
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
            logger.info(f"Navigating to {current_location.name} with {len(self.obstacles)} obstacles, distance: {distance:.2f}")
        else:
            logger.info(f"Navigating to {current_location.name}, distance: {distance:.2f}")

    elif not self.avoidance_enabled:
        # Simple direct navigation without potential field
        self.send_movement_command(f"GX{current_location.x:.2f}Y{current_location.y:.2f}Z{current_z:.2f}")
        logger.info(f"Directly navigating to {current_location.name}, distance: {distance:.2f}")


def handle_executing_task_state(self):
    """Handle the EXECUTING_TASK state"""
    # Check if we need to return to end zone based on time
    elapsed_time = time.time() - self.mission_start_time
    remaining_time = self.mission_duration - elapsed_time
    
    if remaining_time <= self.end_zone_time:
        logger.warning(f"Only {remaining_time:.1f} seconds remaining! Abandoning task and heading to end zone.")
        self.send_movement_command("S")  # Stop the robot
        self.set_state(RobotState.RETURNING_TO_END)
        return
    
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


def handle_returning_to_end_state(self):
    """Handle the RETURNING_TO_END state - go to end zone before time runs out"""
    # Get the end zone location (last location in the list)
    end_location = self.locations[-1]
    
    # Get current position
    current_x, current_y, current_z = position_manager.get_position()
    distance = ((current_x - end_location.x) ** 2 +
               (current_y - end_location.y) ** 2) ** 0.5
    
    # Check if we've reached the end zone
    if distance <= self.position_tolerance:
        logger.info("Reached end zone! Mission completed.")
        self.send_movement_command("S")  # Stop the robot
        self.set_state(RobotState.COMPLETED)
        return
        
    # Continue navigating to end zone
    self.send_movement_command(f"GX{end_location.x:.2f}Y{end_location.y:.2f}Z{end_location.orientation:.2f}")
    logger.info(f"Returning to end zone, distance: {distance:.2f}")
    
    # Check if we're about to run out of time
    elapsed_time = time.time() - self.mission_start_time
    remaining_time = self.mission_duration - elapsed_time
    
    if remaining_time < 2:
        # Almost out of time, emergency stop
        logger.warning("Mission time almost up! Performing emergency stop.")
        self.send_movement_command("S")
        self.set_state(RobotState.COMPLETED)


def handle_completed_state(self):
    """Handle the COMPLETED state - mission is done"""
    # Just stop the robot and wait
    self.send_movement_command("S")
    time.sleep(0.5)
    # Idle until shutdown
    pass


# Updated run method to handle all states in the state machine
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
            if self.current_state == RobotState.TEAM_SELECTION:
                self.handle_team_selection_state()
            elif self.current_state == RobotState.WAITING_FOR_START:
                self.handle_waiting_for_start_state()
            elif self.current_state == RobotState.NAVIGATING:
                self.handle_navigating_state()
            elif self.current_state == RobotState.EXECUTING_TASK:
                self.handle_executing_task_state()
            elif self.current_state == RobotState.RETURNING_TO_END:
                self.handle_returning_to_end_state()
            elif self.current_state == RobotState.COMPLETED:
                self.handle_completed_state()
            elif self.current_state == RobotState.ERROR:
                self.handle_error_state()

            time.sleep(0.05)  # Small sleep to prevent CPU hogging

    except Exception as e:
        logger.error(f"Error in RobotBrain: {e}")
        import traceback
        logger.error(traceback.format_exc())

    finally:
        # Clean up resources
        if self.movement_ser and self.movement_ser.is_open:
            self.movement_ser.close()
            logger.info("Movement serial port closed")

        if self.action_ser and self.action_ser.is_open:
            self.action_ser.close()
            logger.info("Action serial port closed")

        GPIO.cleanup()
