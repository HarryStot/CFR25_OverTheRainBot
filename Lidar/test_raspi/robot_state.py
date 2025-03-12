import enum
import time


class RobotState(enum.Enum):
    IDLE = 0
    PLANNING = 1
    MOVING = 2
    AVOIDING = 3
    MANIPULATING = 4  # For future gripper/magnet operations
    FINISHED = 5


class StateMachine:
    def __init__(self):
        self.current_state = RobotState.IDLE
        self.start_time = None
        self.mission_timeout = 100  # 100 seconds for competition

    def transition_to(self, new_state):
        print(f"Transitioning from {self.current_state.name} to {new_state.name}")
        self.current_state = new_state

    def start_mission(self):
        self.start_time = time.time()
        self.transition_to(RobotState.PLANNING)

    def check_timeout(self):
        if self.start_time is None:
            return False
        return (time.time() - self.start_time) >= self.mission_timeout

    def update(self, robot_controller):
        # Check for timeout
        if self.check_timeout():
            self.transition_to(RobotState.FINISHED)
            return

        # State logic
        if self.current_state == RobotState.IDLE:
            # Wait for start command
            pass

        elif self.current_state == RobotState.PLANNING:
            # Plan path to next goal
            robot_controller.plan_path()
            self.transition_to(RobotState.MOVING)

        elif self.current_state == RobotState.MOVING:
            # Check for obstacles
            if robot_controller.obstacle_detected():
                self.transition_to(RobotState.AVOIDING)
            # Check if reached waypoint
            elif robot_controller.reached_waypoint():
                if robot_controller.reached_final_goal():
                    self.transition_to(RobotState.MANIPULATING)
                else:
                    self.transition_to(RobotState.PLANNING)

        elif self.current_state == RobotState.AVOIDING:
            # When avoidance maneuver is complete
            if not robot_controller.obstacle_detected():
                self.transition_to(RobotState.MOVING)

        elif self.current_state == RobotState.MANIPULATING:
            # When manipulation is complete
            if robot_controller.manipulation_complete():
                self.transition_to(RobotState.PLANNING)

        elif self.current_state == RobotState.FINISHED:
            # Stop all operations
            robot_controller.stop()