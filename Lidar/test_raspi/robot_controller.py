import time
import numpy as np

from robot_state import RobotState, StateMachine
from arduino_controller import ArduinoMotionController, ArduinoServoController
from lidar_processor import LidarProcessor
from potential_field_navigation import PotentialFieldNavigation

class RobotController:
    def __init__(self):
        # Initialize components
        self.state_machine = StateMachine()
        self.navigation = PotentialFieldNavigation()
        self.motion_arduino = ArduinoMotionController(port='/dev/ttyUSB0') # TODO Change port as needed
        self.servo_arduino = ArduinoServoController(port='/dev/ttyUSB1')
        self.lidar = LidarProcessor(port='/dev/ttyUSB2')

        # Robot state
        self.current_position = np.array([0.0, 0.0])  # x, y in meters
        self.current_heading = 0.0  # radians
        self.waypoints = []
        self.current_waypoint_index = 0
        self.goal_reached_threshold = 0.05  # meters
        self.obstacle_threshold = 0.2  # meters

    def start(self):
        """Start the robot operation"""
        self.lidar.start_scanning()
        self.state_machine.start_mission()

        # Main control loop
        try:
            while self.state_machine.current_state != RobotState.FINISHED:
                self.update_position()
                self.state_machine.update(self)
                time.sleep(0.05)  # 20Hz update rate
        except KeyboardInterrupt:
            print("Stopping robot...")
        finally:
            self.stop()

    def stop(self):
        """Stop all robot operations"""
        self.lidar.stop()
        self.motion_arduino.send_position_command(0, 0, 0)  # Stop motion

    def update_position(self):
        """Update current position from odometry feedback"""
        feedback = self.motion_arduino.read_feedback()
        if feedback:
            try:
                # Parse the feedback - depends on your Arduino's format
                # Assuming format like "POS:X0.123Y0.456Z0.789"
                if feedback.startswith("POS:"):
                    pos_data = feedback[4:]
                    x_idx = pos_data.find('X')
                    y_idx = pos_data.find('Y')
                    z_idx = pos_data.find('Z')

                    if x_idx >= 0 and y_idx >= 0:
                        x = float(pos_data[x_idx + 1:y_idx])
                        y = float(pos_data[y_idx + 1:z_idx])
                        self.current_position = np.array([x, y])

                    if z_idx >= 0:
                        # Z might represent heading
                        z = float(pos_data[z_idx + 1:])
                        self.current_heading = z
            except Exception as e:
                print(f"Error parsing position feedback: {e}")

    def plan_path(self):
        """Plan a path to the next goal"""
        # Simple implementation - in real robot you might use A* or similar
        if not self.waypoints:
            # TODO Demo waypoints - replace with competition waypoints
            self.waypoints = [
                np.array([0.5, 0.5]),
                np.array([1.0, 1.0]),
                np.array([1.5, 0.5]),
                np.array([2.0, 1.0])
            ]
            self.current_waypoint_index = 0
        elif self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1

    def get_current_waypoint(self):
        """Get the current waypoint"""
        if self.waypoints and self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

    def obstacle_detected(self):
        """Check if obstacles are detected in the path"""
        obstacles = self.lidar.get_obstacles_raw()
        for obs in obstacles:
            dist = np.linalg.norm(np.array(obs) - self.current_position)
            if dist < self.obstacle_threshold:
                return True
        return False

    def reached_waypoint(self):
        """Check if current waypoint is reached"""
        waypoint = self.get_current_waypoint()
        if waypoint is not None:
            dist = np.linalg.norm(waypoint - self.current_position)
            return dist < self.goal_reached_threshold
        return False

    def reached_final_goal(self):
        """Check if final goal is reached"""
        return self.current_waypoint_index == len(self.waypoints) - 1 and self.reached_waypoint()

    def manipulation_complete(self):
        """Check if manipulation task is complete"""
        # TODO Implement this based on specific manipulation tasks
        # For now, just return True after a delay
        time.sleep(1)
        return True

    def update_movement(self):
        """Update movement commands based on potential field"""
        waypoint = self.get_current_waypoint()
        if waypoint is None:
            return

        obstacles = self.lidar.get_obstacles_raw()
        force_vector = self.navigation.calculate_resultant_force(
            self.current_position, waypoint, obstacles
        )

        # Convert force vector to motion command
        speed = np.linalg.norm(force_vector)

        if speed > 0:
            target_heading = np.arctan2(force_vector[1], force_vector[0])

            # Send command to motion Arduino
            # Convert to whatever format your Arduino expects
            x = self.current_position[0] + force_vector[0] * 0.1  # Small step
            y = self.current_position[1] + force_vector[1] * 0.1
            z = target_heading

            self.motion_arduino.send_position_command(x, y, z)