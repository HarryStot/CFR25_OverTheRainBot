#!/usr/bin/env python3
import threading
import logging
import enum

logger = logging.getLogger(__name__)


class Team(enum.Enum):
    """Team colors enum"""
    YELLOW = 0
    BLUE = 1


class PositionManager:
    """
    Manages the position, target, and velocity of a robot, along with team-specific configurations.

    The `PositionManager` class handles assigning and managing the robot's position,
    target position, and velocity in the context of team configurations (e.g., YELLOW or
    BLUE). Team-specific settings such as initial position and target position are also
    managed. Thread safety is ensured through the use of a reentrant lock.

    :ivar _lock: Reentrant lock ensuring thread safety for property updates.
    :type _lock: threading.RLock
    :ivar _team: Current team of the robot. The default team is YELLOW.
    :type _team: Team
    :ivar _team_configs: Stores team-specific configuration data.
    :type _team_configs: dict
    :ivar _pos_x: Current x-coordinate of the robot's position.
    :type _pos_x: int
    :ivar _pos_y: Current y-coordinate of the robot's position.
    :type _pos_y: int
    :ivar _pos_z: Current orientation (z) of the robot.
    :type _pos_z: int
    :ivar _targ_x: Current x-coordinate of the robot's target position.
    :type _targ_x: int
    :ivar _targ_y: Current y-coordinate of the robot's target position.
    :type _targ_y: int
    :ivar _velocity: Current velocity of the robot.
    :type _velocity: int
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._team = Team.YELLOW  # The default team is yellow

        # Team-specific positions
        self._team_configs = {
            Team.YELLOW: {
                'initial_pos': (10, 10, 90),  # x, y, orientation TODO: Change
                'target_pos': (150, 180),  # x, y TODO: Remove?
            },
            Team.BLUE: {
                'initial_pos': (290, 10, 90),  # x, y, orientation TODO: Change
                'target_pos': (150, 180),  # x, y TODO: Remove?
            }
        }

        # Robot position
        self._pos_x, self._pos_y, self._pos_z = self._team_configs[self._team]['initial_pos']

        # Target position
        self._targ_x, self._targ_y = self._team_configs[self._team]['target_pos']

        # Movement state
        self._velocity = 0

    # Team-related methods
    def set_team(self, team):
        """Set the team color and update positions accordingly"""
        if not isinstance(team, Team):
            raise ValueError(f"Team must be a Team enum value, got {type(team)}")

        with self._lock:
            self._team = team
            logger.info(f"Team set to: {team.name}")

            # Update initial positions based on team
            self._pos_x, self._pos_y, self._pos_z = self._team_configs[team]['initial_pos']
            self._targ_x, self._targ_y = self._team_configs[team]['target_pos'] # TODO: Remove?

            logger.info(f"Initial position updated to: ({self._pos_x}, {self._pos_y}, {self._pos_z})")
            logger.info(f"Target position updated to: ({self._targ_x}, {self._targ_y})") # TODO: Remove?

    def get_team(self):
        """Get current team"""
        with self._lock:
            return self._team

    # Getters
    def get_position(self):
        """
        Retrieves the current position coordinates (x, y, z) of the object in a thread-safe manner.
        The position values are protected by a lock to ensure safe concurrent access.

        :raises None: This method does not explicitly raise any exceptions.
        :returns: A tuple containing the x, y, and z coordinates of the object.
        """
        with self._lock:
            return self._pos_x, self._pos_y, self._pos_z

    def get_target(self):
        """Get current target position (x, y)"""
        with self._lock:
            return self._targ_x, self._targ_y

    def get_velocity(self):
        """Get current velocity"""
        with self._lock:
            return str(self._velocity)

    # Setters
    def set_position(self, x, y, z=None):
        """Set robot position"""
        with self._lock:
            self._pos_x = x
            self._pos_y = y
            if z is not None:
                self._pos_z = z
            logger.debug(f"Position updated: ({self._pos_x}, {self._pos_y}, {self._pos_z})")

    def set_target(self, x, y):
        """Set target position"""
        with self._lock:
            self._targ_x = x
            self._targ_y = y
            logger.debug(f"Target updated: ({self._targ_x}, {self._targ_y})")

    def set_velocity(self, velocity):
        """Set current velocity"""
        with self._lock:
            self._velocity = velocity
            logger.debug(f"Velocity updated: {self._velocity}")

    def get_team_config(self, team=None):
        """Get configuration for a specific team or current team if none specified"""
        with self._lock:
            team_to_use = team if team is not None else self._team
            return self._team_configs[team_to_use]


# Create a singleton instance
position_manager = PositionManager()
