#!/usr/bin/env python3
import json
import logging
import os
import threading
import time
from enum import Enum

import RPi.GPIO as GPIO
import numpy as np

from avoidance_system import PotentialFieldNavigation
from position_manager import position_manager

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """
    Represents the different states a robot can be in during its operation.

    This enumeration class defines the sequence of states that a robot transitions
    through during its lifecycle, from team selection to completing tasks, handling
    errors, and mission completion.

    :cvar TEAM_SELECTION: Initial state to select team (blue/yellow).
    :type TEAM_SELECTION: int
    :cvar WAITING_FOR_START: State where the robot is waiting for the pull switch
        to start the operation.
    :type WAITING_FOR_START: int
    :cvar NAVIGATING: State where the robot is moving to a target location.
    :type NAVIGATING: int
    :cvar EXECUTING_TASK: State where the robot is performing a task at the current
        location.
    :type EXECUTING_TASK: int
    :cvar RETURNING_TO_END: State where the robot is heading to the end zone as
        the time available is almost up.
    :type RETURNING_TO_END: int
    :cvar COMPLETED: State indicating that the robot has successfully completed its
        mission.
    :type COMPLETED: int
    :cvar ERROR: State representing an error condition.
    :type ERROR: int
    """
    TEAM_SELECTION = 0  # Initial state to select team (blue/yellow)
    WAITING_FOR_START = 1  # Waiting for pull switch
    NAVIGATING = 2  # Moving to a target location
    EXECUTING_TASK = 3  # Performing a task at the current location
    RETURNING_TO_END = 4  # Going to the end zone when time is almost up
    COMPLETED = 5  # Mission completed
    ERROR = 6  # Error state
    IDLE = 7  # Idle state, waiting for new locations


class Task:
    """
    Represents a work-related task with a specific command and parameters.

    This class is designed to encapsulate the details of a single task
    in a system. Each task has a name, a command to execute, optional
    parameters, and an expected completion time in seconds. The task
    can also be easily represented as a string for display purposes.

    :ivar name: The name of the task.
    :type name: str
    :ivar command: The command associated with the task.
    :type command: str
    :ivar params: Optional parameters for the task.
    :type params: dict
    :ivar completion_time: Expected time in seconds to complete the task.
    :type completion_time: int
    """

    def __init__(self, name, command, params=None, completion_time=5):
        self.name = name
        self.command = command
        self.params = params or {}
        self.completion_time = completion_time  # seconds to complete the task

    def __str__(self):
        return f"Task: {self.name} - Command: {self.command} - Params: {self.params}"


class Location:
    """
    Represents a location with coordinates, orientation, and associated tasks.

    The Location class serves as a way to store and manage information about a
    specific location, including its name, coordinates, optional orientation,
    and a list of associated tasks. This class also provides a string
    representation for easier identification of the location.

    :ivar name: The name of the location.
    :type name: str
    :ivar x: The x-coordinate of the location.
    :type x: float
    :ivar y: The y-coordinate of the location.
    :type y: float
    :ivar orientation: Optional orientation of the location.
    :type orientation: Optional[float]
    :ivar tasks: List of tasks associated with the location.
    :type tasks: list
    """

    def __init__(self, name, x, y, orientation=None, tasks=None):
        self.name = name
        self.x = x
        self.y = y
        self.orientation = orientation
        self.tasks = tasks or []

    def __str__(self):
        return f"Location: {self.name} at ({self.x}, {self.y})"


class RobotBrain(threading.Thread):
    """
    Handles the robot's decision-making, state management, navigation, and task
    execution processes in a multi-threaded environment.

    The `RobotBrain` class inherits from `threading.Thread` and serves as the core
    controller for the robot's operations. It maintains the robot's state machine
    and provides mechanisms for managing team selection, navigation to mission
    locations, obstacle detection and handling, as well as executing specific tasks
    at designated waypoints. Designed with thread safety using locks, it ensures
    proper synchronization between its various components and external interfaces.

    :ivar movement_interface: Interface responsible for managing robot movement.
    :type movement_interface: Optional[RobotInterface]
    :ivar action_interface: Interface for executing specific actions or tasks.
    :type action_interface: Optional[RobotInterface]
    :ivar stop_event: Synchronization event to handle termination of the thread.
    :type stop_event: threading.Event
    :ivar current_state: The robot's current state in the state machine.
    :type current_state: RobotState
    :ivar previous_state: The robot's previous state before transitioning.
    :type previous_state: RobotState | None
    :ivar state_changed: Event signaling changes in the robot's state.
    :type state_changed: threading.Event
    :ivar avoidance_enabled: Whether obstacle avoidance is active.
    :type avoidance_enabled: bool
    :ivar is_blue_team: Indicates if the robot is in the blue team (True) or yellow
        team (False).
    :type is_blue_team: bool | None
    :ivar mission_start_time: Timestamp of when the mission started.
    :type mission_start_time: float
    :ivar mission_duration: Total allowed mission time in seconds.
    :type mission_duration: int
    :ivar end_zone_time: Time in seconds before mission end when the robot should
        start returning.
    :type end_zone_time: int
    :ivar locations: List of predetermined mission locations.
    :type locations: list[Location]
    :ivar current_location_index: Index of the robot's current location in the
        mission.
    :type current_location_index: int
    :ivar current_task_index: Index of the current task being executed at the
        current location.
    :type current_task_index: int
    :ivar position_tolerance: Position tolerance in centimeters for reaching a
        location.
    :type position_tolerance: float
    :ivar orientation_tolerance: Orientation tolerance in degrees for alignment.
    :type orientation_tolerance: float
    :ivar obstacle_detected: Whether the robot has detected any obstacles.
    :type obstacle_detected: bool
    :ivar navigation_timeout: Timeout in seconds for navigation tasks.
    :type navigation_timeout: int
    :ivar navigation_start_time: Timestamp when navigation to the current location
        started.
    :type navigation_start_time: float
    :ivar task_start_time: Timestamp when the current task execution started.
    :type task_start_time: float
    :ivar task_timeout: Timeout in seconds for completing a task.
    :type task_timeout: int
    :ivar lock: Reentrant lock for thread-safe operations.
    :type lock: threading.RLock
    :ivar team_select_pin: GPIO pin number configured for team selection.
    :type team_select_pin: int
    :ivar validation_pin: GPIO pin number configured for the validation button.
    :type validation_pin: int
    :ivar pull_switch_pin: GPIO pin number configured for the mission start switch.
    :type pull_switch_pin: int
    :ivar potential_nav: Instance for handling potential field navigation.
    :type potential_nav: PotentialFieldNavigation
    :ivar obstacles: List of currently detected obstacles represented as tuples of
        their coordinates.
    :type obstacles: list[tuple[float, float]]
    :ivar last_v: Last calculated linear velocity for debugging.
    :type last_v: float
    :ivar last_omega: Last calculated angular velocity for debugging.
    :type last_omega: float
    """

    def __init__(self,
                 movement_interface=None,
                 action_interface=None,
                 stop_event=None):
        super().__init__()
        self.movement_interface = movement_interface  # RobotInterface instance for movement
        self.action_interface = action_interface  # RobotInterface instance for actions
        self.stop_event = stop_event or threading.Event()
        self.daemon = True

        # State machine variables
        self.current_state = RobotState.TEAM_SELECTION
        self.previous_state = None
        self.state_changed = threading.Event()
        self.avoidance_enabled = False

        # Last position sent to the robot
        self.last_sent_position = None  # Last position sent to the robot
        self.last_sent_orientation = None  # Last orientation sent to the robot

        # Team and timing variables
        self.is_blue_team = None  # True for blue team, False for yellow team
        self.mission_start_time = 0  # When the pull switch was activated
        self.mission_duration = 85  # 85 seconds total mission time
        self.end_zone_time = 15  # How many seconds before end to start returning

        # Mission variables
        self.locations = []
        self.current_location_index = -1
        self.current_task_index = -1

        # Navigation parameters
        self.position_tolerance = 4  # [cm]
        self.orientation_tolerance = 1.5  # [degrees]
        self.obstacle_detected = False
        self.navigation_timeout = 60  # [seconds]
        self.navigation_start_time = 0

        # Task execution variables
        self.task_start_time = 0
        self.task_timeout = 20  # [seconds]

        # Lock for thread safety
        self.lock = threading.RLock()

        # GPIO configuration
        GPIO.setmode(GPIO.BCM)
        self.team_select_pin = 27  # GPIO pin for team selection switch
        self.validation_pin = 21  # GPIO pin for validation button
        self.pull_switch_pin = 17  # GPIO pin for pull switch to start
        GPIO.setup(self.team_select_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.validation_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pull_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize potential field navigation
        self.potential_nav = PotentialFieldNavigation()

        # List of detected obstacles (to be updated from LidarThread)
        self.obstacles = []

        # Last calculated control command (for debugging)
        self.last_v = 0.0
        self.last_omega = 0.0

        # Last LCD message sent
        self.last_lcd_line1 = ""
        self.last_lcd_line2 = ""

    def __del__(self):
        """
        Destructor for the RobotBrain class. Cleans up GPIO settings and stops
        the thread if it is still running.

        :return: None
        """
        GPIO.cleanup()

    def update_obstacles(self, obstacle_list):
        """
        Updates the internal list of obstacles used for potential field navigation. Converts the
        obstacle information from rectangle format to a simplified point representation using only
        the center point of each obstacle. Logs the update and detailed obstacle information if the
        number of obstacles is fewer than five.

        :param obstacle_list: A list of obstacles, where each obstacle is represented as a tuple of
            four float values (x, y, width, height). The x and y coordinates represent the center
            of the obstacle, while width and height define its dimensions.
        :type obstacle_list: list[tuple[float, float, float, float]]
        :return: None
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
        """
        Adds a new location to the list of locations in a thread-safe manner.

        Utilizes a lock to ensure that multiple threads can safely update the shared
        locations list without generating concurrency issues. Logs the addition
        of the new location for operational awareness.

        :param location: New location to be added to the list.
        :type location: Any
        :return: None
        """
        with self.lock:
            self.locations.append(location)
            logger.info(f"Added location: {location}")

    def clear_locations(self):
        """
        Clears all stored locations and resets related indices.

        This method safely clears the list of locations by acquiring a lock to
        ensure thread-safety. It also resets the indices for the current location
        and task to initial default values. An informational log message is
        generated upon successful completion of this operation.

        :raises None: This method does not raise any exceptions.
        :return: None
        """
        with self.lock:
            self.locations = []
            self.current_location_index = -1
            self.current_task_index = -1
            logger.info("Cleared all locations")

    def set_state(self, new_state):
        """
        Sets a new state for the object in a thread-safe manner. If the new state
        differs from the current state, it updates the previous state with the
        current state's value, assigns the new value to the current state, and
        triggers the `state_changed` event. Logs information about the state change.

        :param new_state: The new state value to set.
        :return: None
        """
        with self.lock:
            if new_state != self.current_state:
                self.previous_state = self.current_state
                self.current_state = new_state
                self.state_changed.set()
                logger.info(f"State changed: {self.previous_state} -> {self.current_state}")

    def handle_team_selection_state(self):
        """
        Handles the team selection state by monitoring the team selection and
        validation buttons. This function determines and sets the team (either
        blue or yellow), updates the relevant display information, and transitions
        to the next state when the team selection is confirmed.

        In the team selection state, this function checks the state of the team
        selection GPIO pin to determine whether the blue team or the yellow team
        is selected. It updates the LCD accordingly to show the selected team
        and then waits for the validation button to confirm the selection. Once
        the button is pressed, it updates the position manager with the selected
        team, shows a confirmation message on the LCD, and transitions the robot
        to the next state where it awaits the start signal.

        :raises RuntimeError: If there is an issue with the position manager module.

        """
        # Check team selection switch
        if GPIO.input(self.team_select_pin) == GPIO.LOW:
            self.is_blue_team = True
            team_name = "BLUE TEAM"
            logger.info("Blue team selected")
        else:
            self.is_blue_team = False
            team_name = "YELLOW TEAM"
            logger.info("Yellow team selected")

        # Update LCD with team selection
        self.send_lcd_message("SELECT TEAM:", team_name)

        # Check if validation button is pressed to confirm team selection
        if GPIO.input(self.validation_pin) == GPIO.LOW:
            logger.info("Validation button pressed, confirming team selection")

            # Update position manager with team selection
            from position_manager import position_manager, Team
            team = Team.BLUE if self.is_blue_team else Team.YELLOW
            position_manager.set_team(team)

            # Update LCD to show team is confirmed
            self.send_lcd_message(f"{team_name}", "CONFIRMED!")
            time.sleep(1)  # Show confirmation briefly

            # Load mission locations based on team
            self.load_team_missions()

            # Move to waiting for start
            logger.info("Moving to WAITING_FOR_START state")
            self.set_state(RobotState.WAITING_FOR_START)
        else:
            # Still in team selection, allow team to be changed
            time.sleep(0.1)

    def handle_waiting_for_start_state(self):
        """
        Handles the "waiting for start" state of the robot, including interactions
        with a physical pull switch and updates to the robot's display. This method
        initializes the robot's mission behavior, starting from the ready state
        through to mission activation.

        Key transitions handled within this method:
        - Waits for the pull switch to be pressed to move to the "steady" state.
        - Waits for the pull switch to be released to activate the "go" state.
        - Initializes required state values for starting the mission.

        :param self: The instance of the robot's control class managing its states and behavior.
        :raises: This method does not raise any specific exceptions.

        :rtype: None
        :return: This method does not return any value.
        """
        logger.info("Initializing robot...")

        # Display "Ready" message and wait for switch to be pressed
        logger.info("Ready...")
        self.send_lcd_message(f"Team {'Blue' if self.is_blue_team else 'Yellow'}", "Ready...")

        # Wait for pull switch to be pressed (LOW)
        while GPIO.input(self.pull_switch_pin) == GPIO.HIGH:
            time.sleep(0.1)
            # Check if we should exit (e.g. if stop_event is set)
            if self.stop_event.is_set():
                return

        # Display "Steady" message and wait for switch to be released (HIGH)
        logger.info("Steady...")
        self.send_lcd_message(f"Team {'Blue' if self.is_blue_team else 'Yellow'}", "Steady...")

        # Wait for pull switch to be released (HIGH)
        while GPIO.input(self.pull_switch_pin) == GPIO.LOW:
            time.sleep(0.1)
            # Check if we should exit
            if self.stop_event.is_set():
                return

        # Display "Go" message and start the mission
        logger.info("GO!!!")
        self.send_lcd_message(f"Team {'Blue' if self.is_blue_team else 'Yellow'}", "GO!!!")

        # Record mission start time
        self.mission_start_time = time.time()

        # Start the mission with the first location
        self.current_location_index = 0
        self.current_task_index = -1

        # Move to navigating state to go to first location
        self.set_state(RobotState.NAVIGATING)
        self.navigation_start_time = time.time()

    def load_team_missions(self):
        """
        Loads mission waypoints from JSON files with support for variables, task templates,
        movement templates, and action sequences.
        """
        self.clear_locations()

        # Determine which file to load based on team
        mission_file = "blue_missions.json" if self.is_blue_team else "yellow_missions.json"
        file_path = os.path.join(os.path.dirname(__file__), mission_file)

        try:
            with open(file_path, 'r') as f:
                config = json.load(f)

            # Extract configuration components
            variables = config.get("variables", {})
            task_templates = config.get("task_templates", {})
            movement_templates = config.get("movement_templates", {})
            action_sequences = config.get("action_sequences", {})
            locations_data = config.get("locations", [])

            # Process each location
            for location_data in locations_data:
                # Basic location properties
                name = location_data["name"]
                x = self._resolve_variables(location_data["x"], variables)
                y = self._resolve_variables(location_data["y"], variables)
                orientation = self._resolve_variables(location_data.get("orientation"), variables)

                tasks = []

                # Process action sequence if specified
                if "action_sequence" in location_data and location_data["action_sequence"] in action_sequences:
                    sequence_name = location_data["action_sequence"]
                    sequence = action_sequences[sequence_name]
                    params_override = location_data.get("params_override", {})

                    # Convert action sequence to tasks
                    tasks = self._process_action_sequence(
                        sequence,
                        task_templates,
                        movement_templates,
                        variables,
                        params_override
                    )
                # Process individual tasks if specified
                elif "tasks" in location_data and location_data["tasks"]:
                    tasks = self._process_tasks(location_data["tasks"], task_templates, variables)

                # Create Location object with processed data
                self.add_location(Location(
                    name=name,
                    x=x,
                    y=y,
                    orientation=orientation,
                    tasks=tasks
                ))

            logger.info(f"Loaded {len(self.locations)} locations for {'blue' if self.is_blue_team else 'yellow'} team")
        except Exception as e:
            logger.error(f"Error loading mission file {mission_file}: {str(e)}. Using default values.")
            # Add default/fallback locations if file has issues
            self.add_location(Location("Test", 100, 0, 0))
            self.add_location(Location("Test 2", 200, 0, 0))

    def _process_action_sequence(self, sequence, task_templates, movement_templates, variables, params_override):
        """
        Converts an action sequence into a list of Task objects.

        :param sequence: Action sequence definition from JSON
        :param task_templates: Dictionary of task templates
        :param movement_templates: Dictionary of movement templates
        :param variables: Dictionary of configuration variables
        :param params_override: Optional parameter overrides specified in location
        :return: List of Task objects
        """
        tasks = []
        for i, step in enumerate(sequence.get("steps", [])):
            step_type = step.get("type")
            template_name = step.get("template")

            if step_type == "task" and template_name in task_templates:
                # Process task step
                template = task_templates[template_name]

                # Apply parameter overrides if present
                task_params = template.get("params", {}).copy()
                if template_name in params_override:
                    if "params" in params_override[template_name]:
                        task_params.update(params_override[template_name]["params"])

                # Resolve variables in parameters
                task_params = self._resolve_variables(task_params, variables)

                # Create task with resolved parameters
                tasks.append(Task(
                    name=f"{template_name}_{i}",
                    command=template["command"],
                    params=task_params,
                    completion_time=self._resolve_variables(template.get("completion_time", 5), variables)
                ))

            elif step_type == "movement" and template_name in movement_templates:
                # Process movement step
                movement_steps = movement_templates[template_name]

                # Apply parameter overrides if present
                if template_name in params_override:
                    overrides = params_override[template_name]
                    for j, override in enumerate(overrides):
                        if j < len(movement_steps):
                            if "params" in override and "params" in movement_steps[j]:
                                movement_steps[j]["params"].update(override["params"])

                # Convert movement steps to tasks
                for j, movement in enumerate(movement_steps):
                    mv_params = movement.get("params", {}).copy()

                    # Resolve variables in movement parameters
                    mv_params = self._resolve_variables(mv_params, variables)

                    tasks.append(Task(
                        name=f"{template_name}_{i}_{j}",
                        command=movement["command"],
                        params=mv_params,
                        completion_time=self._resolve_variables(movement.get("completion_time", 2), variables)
                    ))

        return tasks

    def _process_tasks(self, tasks_data, task_templates, variables):
        """
        Process individual task definitions into Task objects.

        :param tasks_data: List of task definitions from JSON
        :param task_templates: Dictionary of task templates
        :param variables: Dictionary of configuration variables
        :return: List of Task objects
        """
        tasks = []
        for task_data in tasks_data:
            if "template" in task_data and task_data["template"] in task_templates:
                # Task uses a template
                template = task_templates[task_data["template"]]
                task_name = task_data["name"]
                task_command = template["command"]

                # Handle params with both template and overrides
                task_params = template.get("params", {}).copy()
                if "params_override" in task_data:
                    task_params.update(task_data["params_override"])

                # Resolve variables in params - use the enhanced resolver
                task_params = self._resolve_variables(task_params, variables)

                completion_time = task_data.get("completion_time",
                                                self._resolve_variables(template.get("completion_time", 5),
                                                                        variables))
            else:
                # Regular task definition without template
                task_name = task_data["name"]
                task_command = task_data["command"]
                task_params = task_data.get("params", {})
                completion_time = task_data.get("completion_time", 5)

                # Resolve variables in params
                task_params = self._resolve_variables(task_params, variables)

            tasks.append(Task(
                name=task_name,
                command=task_command,
                params=task_params,
                completion_time=completion_time
            ))

        return tasks

    def _resolve_variables(self, value, variables):
        """
        Recursively resolve all variables in a value, supporting various data structures
        and formats including variables in keys and compound strings.

        :param value: The value to process (can be dict, list, string, or primitive)
        :param variables: Dictionary of available variables
        :return: The value with all variables resolved
        """
        # Base case: None
        if value is None:
            return None

        # Handle dictionaries (including variable keys)
        if isinstance(value, dict):
            result = {}
            for k, v in value.items():
                # Resolve key if it's a variable
                resolved_key = k
                if isinstance(k, str) and k.startswith('$'):
                    var_name = k[1:]
                    if var_name in variables:
                        resolved_key = variables[var_name]

                # Recursively resolve the value
                resolved_value = self._resolve_variables(v, variables)
                result[resolved_key] = resolved_value
            return result

        # Handle lists
        if isinstance(value, list):
            return [self._resolve_variables(item, variables) for item in value]

        # Handle strings with embedded variables
        if isinstance(value, str):
            # Simple variable replacement (entire string is a variable)
            if value.startswith('$') and not ':' in value:
                var_name = value[1:]
                if var_name in variables:
                    return variables[var_name]
                return value

            # Complex string with embedded variables
            if '$' in value:
                result = value
                # Find all variable references
                import re
                var_refs = re.findall(r'\$([a-zA-Z0-9_]+)', value)

                # Replace each variable reference
                for var_name in var_refs:
                    if var_name in variables:
                        # Replace variable with its value, converting to string if needed
                        var_value = str(variables[var_name])
                        result = result.replace(f'${var_name}', var_value)

                return result

        # Return primitive values as is
        return value

    def handle_idle_state(self):
        """
        Handles the robot's behavior in the idle state. In the idle state, the robot
        either progresses to the next location or remains idle based on whether there
        are more locations to visit.

        If there are more locations left in the sequence, the robot transitions to
        the navigating state, updating its current location index and resetting the
        current task index. If there are no further locations, the robot will
        wait and periodically check for new destinations.

        :return: None
        """
        if self.locations and self.current_location_index < len(self.locations) - 1:
            # Move to the next location
            self.current_location_index += 1
            self.current_task_index = -1
            logger.info(f"Moving to next location: {self.locations[self.current_location_index]}")
            time.sleep(0.5)  # Allow some time for the robot to prepare
            self.set_state(RobotState.NAVIGATING)
            self.navigation_start_time = time.time()
        else:
            # No more locations, wait for new ones
            time.sleep(0.5)

    def handle_navigating_state(self):
        """
        Handles navigation logic for the robot while it is in the navigating state.

        This method determines the appropriate navigation actions based on the robot's
        current location, mission time constraints, and obstacle detection status. It
        also calculates the distance to the target, evaluates the required orientation
        corrections, and sends movement commands accordingly. The method makes use of
        potential field navigation for obstacle avoidance if enabled, and supports
        direct navigation otherwise. Additionally, state transitions occur based on
        defined criteria such as reaching a target, running out of time, or encountering
        obstacles.

        :raises ValueError: If any unexpected state or condition is encountered
                            during navigation.
        """
        current_location = self.locations[self.current_location_index]

        # Check if we need to return to end zone based on time
        elapsed_time = time.time() - self.mission_start_time
        remaining_time = self.mission_duration - elapsed_time

        if remaining_time <= self.end_zone_time:
            logger.warning(f"Only {remaining_time:.1f} seconds remaining! Heading to end zone.")
            # self.set_state(RobotState.RETURNING_TO_END)
            return

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

                # if orientation_diff > self.orientation_tolerance:
                #     # Need to adjust orientation
                #               #
                #     self.send_movement_command(
                #         f"GX{current_location.x/100:.2f}Y{current_location.y/100:.2f}Z{current_location.orientation/100:.2f}")
                #     logger.info(f"Adjusting orientation to {current_location.orientation} degrees")
                #     return

            logger.info(f"Reached location: {current_location.name}")
            self.send_movement_command("S")  # Stop the robot
            # If there are tasks to perform, move to EXECUTING_TASK state
            if current_location.tasks:
                self.current_task_index = 0
                self.set_state(RobotState.EXECUTING_TASK)
                self.task_start_time = time.time()
            else:
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

            # Display the current position
            self.send_lcd_message(f"Team {'Blue' if self.is_blue_team else 'Yellow'}",
                                  "X: {:.2f} Y: {:.2f}".format(current_x, current_y))

            # Log navigation status
            if self.obstacles:
                logger.info(
                    f"Navigating to {current_location.name} with {len(self.obstacles)} obstacles, distance: {distance:.2f}")
            else:
                logger.info(f"Navigating to {current_location.name}, distance: {distance:.2f}")

        elif not self.avoidance_enabled:
            # Simple direct navigation without potential field

            # Check if we already sent the last position
            if self.last_sent_position == (current_location.x, current_location.y):
                logger.debug("Already sent last position, skipping movement command")
                self.send_lcd_message(f"Team {'Blue' if self.is_blue_team else 'Yellow'}",
                                      "X: {:.2f} Y: {:.2f}".format(current_x, current_y))
                return
            else:
                # Update last sent position
                self.last_sent_position = (current_location.x, current_location.y)

            self.send_movement_command(
                f"GX{current_location.x / 100:.2f}Y{current_location.y / 100:.2f}Z{current_z / 100:.2f}")

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
                self.set_state(RobotState.IDLE)  # FIXME: Change to NAVIGATING?
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
        self.send_movement_command(
            f"GX{end_location.x / 100:.2f}Y{end_location.y / 100:.2f}Z{end_location.orientation / 100:.2f}")
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
        """
        Sends a movement command through the movement interface if it is connected.

        This method handles the task of sending movement commands to the robot's
        movement interface. If the movement interface is not connected, it logs an
        error and sets the robot state to an error state. Any exceptions raised during
        the command transmission are logged, and the robot state is updated to error.

        Examples for command strings:
            - "GX10.0Y20.0Z90.0" (go to coordinates command)
            - "S" (stop command)
            - "P" (get position command)
            - "INITX10.0Y20.0Z90.0" (initialize position command)
            - "V50" (set velocity command)

        :param command: Movement command to be sent to the movement interface.
        :type command: str
        :return: None
        """
        if self.movement_interface and self.movement_interface.connected:
            try:
                self.movement_interface.send_command(command)
                logger.info(f"Sent movement command: {command}")
            except Exception as e:
                logger.error(f"Error sending movement command: {e}")
                # self.set_state(RobotState.ERROR)
        else:
            logger.error("Movement interface not connected, cannot send command")
            # self.set_state(RobotState.ERROR)

    def send_action_command(self, command, params=None):
        """
        Sends an action command to the robot through the connected action interface. Constructs
        the full command string by concatenating the command with the formatted parameters.
        Logs the command being sent or any encountered errors. If the action interface is not
        connected, sets the robot state as an error.

        Examples for command and params:
            - command: "LCD"
              params: {"L1": "Hello", "L2": "World"}
              full_command: "LCDL1Hello;L2World"
            - command: "SRV"
              params: {"id": "angle:speed", "id2": "angle:speed"}
              full_command: "SRVid:angle:speed"
            - command: "STEP"
              params: {"step": "1"} FIXME: Change to actual command
              full_command: "STEPstep1"
            - command: "US" (ultrasonic sensor)
              params: {"ultrasonic_sensor": "1"} FIXME: Change to actual command
              full_command: "USultrasonic_sensor1"

        :param command: The base command string to send to the robot.
        :type command: str
        :param params: Additional parameters for the command as key-value pairs. Defaults to None.
        :type params: dict, optional
        :return: None
        """
        if self.action_interface and self.action_interface.connected:
            try:
                # Build the command string based on the command and params
                params_str = ""
                if params:
                    params_str = ";".join([f"{k}{v}" for k, v in params.items()])

                full_command = f"{command}{params_str}"
                self.action_interface.send_command(full_command)
                logger.info(f"Sent action command: {command}{params_str}")
            except Exception as e:
                logger.error(f"Error sending action command: {e}")
                # self.set_state(RobotState.ERROR)
        else:
            logger.error("Action interface not connected, cannot send command")
            # self.set_state(RobotState.ERROR)

    def set_obstacle_detected(self, detected):
        """
        Updates the obstacle detection status with thread safety. It ensures that the
        obstacle detection state is only updated if the new value differs from the
        previous state. The method also logs the updated state whenever a change
        occurs.

        :param detected: New obstacle detection status.
        :type detected: bool
        :return: None
        """
        with self.lock:
            if detected != self.obstacle_detected:
                self.obstacle_detected = detected
                logger.info(f"Obstacle detection changed: {detected}")

    def send_lcd_message(self, line1="", line2=""):
        """
        Sends a two-line message to the LCD display. The provided text for each
        line will immediately be sent to the LCD using the appropriate action
        command. Debug logging is performed for the message sent.

        :param line1: The text for the first line of the LCD display.
            Defaults to an empty string if not provided.
            Type: str
        :param line2: The text for the second line of the LCD display.
            Defaults to an empty string if not provided.
            Type: str
        :return: None
        """
        params = {
            "L1:": line1,
            "L2:": line2
        }
        if line1 == self.last_lcd_line1 and line2 == self.last_lcd_line2:
            # No change in message, do not send again
            return
        self.last_lcd_line1 = line1
        self.last_lcd_line2 = line2
        # Use the action command method to send LCD commands
        self.send_action_command("LCD", params)
        logger.debug(f"Sent to LCD - Line 1: '{line1}', Line 2: '{line2}'")

    def run(self):
        """Main brain thread function"""
        # logger.info("Running main loop...")
        try:
            # Check if interfaces are connected
            if not (self.movement_interface and self.movement_interface.connected):
                logger.error("Movement interface not connected, cannot run brain")
                return

            if not (self.action_interface and self.action_interface.connected):
                logger.warning("Action interface not connected, some functions will be limited")

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
                elif self.current_state == RobotState.IDLE:
                    self.handle_idle_state()
                elif self.current_state == RobotState.RETURNING_TO_END:
                    self.handle_returning_to_end_state()
                elif self.current_state == RobotState.COMPLETED:
                    self.handle_completed_state()
                elif self.current_state == RobotState.ERROR:
                    self.handle_error_state()

                time.sleep(0.005)  # Small sleep to prevent CPU hogging

        except Exception as e:
            logger.error(f"Error in RobotBrain: {e}")

        finally:
            # Just clean up GPIO, interfaces are handled in main.py
            GPIO.cleanup()
