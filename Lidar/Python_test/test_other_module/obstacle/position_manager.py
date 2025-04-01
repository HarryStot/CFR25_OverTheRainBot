#!/usr/bin/env python3

import threading
import logging

logger = logging.getLogger(__name__)


class PositionManager:
    """Manages robot position data shared between modules"""

    def __init__(self):
        self._lock = threading.RLock()
        # Robot position
        self._pos_x = 150
        self._pos_y = 100
        self._pos_z = 0  # Orientation in degrees
        # Target position
        self._targ_x = 200
        self._targ_y = 100
        # Movement state
        self._velocity = 0

    # Getters
    def get_position(self):
        """Get current robot position (x, y, orientation)"""
        with self._lock:
            return (self._pos_x, self._pos_y, self._pos_z)

    def get_target(self):
        """Get current target position (x, y)"""
        with self._lock:
            return (self._targ_x, self._targ_y)

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
            logger.debug(f"Robot position updated: ({x}, {y}, {self._pos_z})")

    def set_target(self, x, y):
        """Set target position"""
        with self._lock:
            self._targ_x = x
            self._targ_y = y
            logger.debug(f"Target updated: ({x}, {y})")

    def set_velocity(self, velocity):
        """Set current velocity"""
        with self._lock:
            self._velocity = velocity
            logger.debug(f"Velocity updated: {velocity}")


# Create a singleton instance
position_manager = PositionManager()
