#!/usr/bin/env python3

import os
import json
import logging
import argparse
import re
from pprint import pprint
from typing import Dict, List, Any, Union, Optional

# Configure logging for testing
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


# Import the classes needed for testing
class Task:
    def __init__(self, name, command, params=None, completion_time=5):
        self.name = name
        self.command = command
        self.params = params or {}
        self.completion_time = completion_time

    def __str__(self):
        return f"Task: {self.name} - Command: {self.command} - Params: {self.params}"


class Location:
    def __init__(self, name, x, y, orientation=None, tasks=None):
        self.name = name
        self.x = x
        self.y = y
        self.orientation = orientation
        self.tasks = tasks or []

    def __str__(self):
        return f"Location: {self.name} at ({self.x}, {self.y}), orientation: {self.orientation}, tasks: {len(self.tasks)}"


class MockActionInterface:
    """Mock action interface for testing."""

    def __init__(self):
        self.commands_sent = []
        self.connected = True

    def send_command(self, command):
        """Simulate sending a command to the robot."""
        self.commands_sent.append(command)
        logger.debug(f"Mock interface sending command: {command}")
        return True

    def get_command_history(self):
        """Get history of commands sent."""
        return self.commands_sent


class MissionTester:
    """Test class for mission JSON parsing and task generation."""

    def __init__(self, json_file=None, team="blue"):
        self.json_file = json_file
        self.team = team.lower()
        self.locations = []
        self.action_interface = MockActionInterface()

    def clear_locations(self):
        """Clear all locations."""
        self.locations = []
        logger.info("Cleared all locations")

    def add_location(self, location):
        """Add a location to the list."""
        self.locations.append(location)
        logger.debug(f"Added location: {location}")

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

    def load_missions_from_file(self, json_file=None):
        """
        Load mission data from a JSON file.

        :param json_file: Path to the JSON file. If None, uses the instance's json_file.
        :return: True if successful, False otherwise.
        """
        if json_file:
            self.json_file = json_file

        if not self.json_file:
            logger.error("No JSON file specified")
            return False

        try:
            with open(self.json_file, 'r') as f:
                config = json.load(f)

            self.clear_locations()

            # Extract variables, templates, and locations
            variables = config.get("variables", {})
            task_templates = config.get("task_templates", {})
            movement_templates = config.get("movement_templates", {})
            action_sequences = config.get("action_sequences", {})
            locations_data = config.get("locations", [])

            # Process each location
            for location_data in locations_data:
                # Replace variable references in location properties
                name = location_data["name"]
                x = self._resolve_variables(location_data["x"], variables)
                y = self._resolve_variables(location_data["y"], variables)
                orientation = self._resolve_variables(location_data.get("orientation"), variables)

                tasks = []
                # Create Task objects if tasks exist in the location data
                if "action_sequence" in location_data and location_data["action_sequence"] in action_sequences:
                    sequence_name = location_data["action_sequence"]
                    sequence = action_sequences[sequence_name]
                    params_override = location_data.get("params_override", {})

                    tasks = self._process_action_sequence(
                        sequence,
                        task_templates,
                        movement_templates,
                        variables,
                        params_override
                    )
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

            logger.info(f"Loaded {len(self.locations)} locations from {self.json_file}")
            return True
        except Exception as e:
            logger.error(f"Error loading mission file {self.json_file}: {str(e)}")
            return False

    def send_action_command(self, command, params=None):
        """
        Sends an action command to the robot through the connected action interface.

        :param command: The base command string to send to the robot.
        :param params: Additional parameters for the command as key-value pairs. Defaults to None.
        :return: None
        """
        if command == "F" or command == "R":
            # Handle special commands
            logger.info(f"Sending special command: {command + str(params['distance'] / 100)}")
            return command + str(params['distance'] / 100)


        if self.action_interface and self.action_interface.connected:
            try:
                # Build the command string based on the command and params
                params_str = ""
                if params:
                    params_str = ";".join([f"{k}{v}" for k, v in params.items()])

                full_command = f"{command}{params_str}"
                self.action_interface.send_command(full_command)
                logger.info(f"Sent action command: {full_command}")
                return full_command
            except Exception as e:
                logger.error(f"Error sending action command: {e}")
                return None
        else:
            logger.error("Action interface not connected, cannot send command")
            return None

    def simulate_mission_execution(self):
        """
        Simulate the execution of all mission tasks to test the command generation.
        """
        logger.info("=== MISSION SIMULATION STARTED ===")

        for loc_idx, location in enumerate(self.locations):
            logger.info(f"\nLocation {loc_idx + 1}/{len(self.locations)}: {location.name}")
            logger.info(f"  Coordinates: ({location.x}, {location.y}), Orientation: {location.orientation}")
            logger.info(f"  Tasks: {len(location.tasks)}")

            # Simulate navigation to location
            logger.info(f"  Navigating to location {location.name}...")

            # Simulate executing each task
            for task_idx, task in enumerate(location.tasks):
                logger.info(f"  Executing task {task_idx + 1}/{len(location.tasks)}: {task.name}")
                logger.info(f"    Command: {task.command}")
                logger.info(f"    Parameters: {task.params}")
                logger.info(f"    Expected completion time: {task.completion_time}s")

                # Simulate sending command to robot using your action command format
                if task.command and task.params:
                    cmd = self.send_action_command(task.command, task.params)
                    logger.info(f"    Generated Command: {cmd}")

                    # For SRV commands, explain what would happen
                    if task.command == "SRV":
                        self.explain_srv_command(cmd)

            if not location.tasks:
                logger.info("  No tasks to execute at this location")

        logger.info("\n=== MISSION SIMULATION COMPLETED ===")

    def explain_srv_command(self, cmd):
        """Explain what a SRV command would do."""
        if not cmd or not cmd.startswith("SRV"):
            return

        # Parse the command
        parts = cmd.split(';')
        cmd_name = parts[0][:3]  # SRV

        # For each parameter (servo setting)
        for param in parts:
            if param.startswith("SRV"):
                param = param[3:]  # Remove SRV prefix

            # Try to parse servo_id:angle:velocity
            if ':' in param:
                components = param.split(':')
                if len(components) >= 3:
                    servo_id = components[0]
                    angle = components[1]
                    velocity = components[2]
                    logger.info(f"    → Servo {servo_id} would move to angle {angle}° at velocity {velocity}")


def main():
    """Main function to run the tester."""
    parser = argparse.ArgumentParser(description='Test robot mission configuration parser')
    parser.add_argument('--file', type=str, help='Path to mission JSON file')
    parser.add_argument('--team', type=str, default='blue', choices=['blue', 'yellow'],
                        help='Team color (blue or yellow)')
    args = parser.parse_args()

    # Default to blue_missions.json if no file specified
    if not args.file:
        team_prefix = args.team.lower()
        args.file = f"missions/{team_prefix}_missions.json"

    tester = MissionTester(args.file, args.team)
    if tester.load_missions_from_file():
        # Print loaded locations and tasks
        print("\n=== LOADED MISSION CONFIGURATION ===")
        for i, location in enumerate(tester.locations):
            print(f"Location {i + 1}: {location}")
            for j, task in enumerate(location.tasks):
                print(f"  Task {j + 1}: {task}")

        # Simulate mission execution
        tester.simulate_mission_execution()
    else:
        print("Failed to load mission configuration")


if __name__ == "__main__":
    main()