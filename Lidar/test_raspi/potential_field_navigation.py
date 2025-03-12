import numpy as np
import math


class PotentialFieldNavigation:
    def __init__(self):
        self.attractive_constant = 1.0
        self.repulsive_constant = 100.0
        self.influence_radius = 0.5  # meters
        self.min_distance = 0.1  # meters

    def attractive_force(self, current_pos, goal_pos):
        dx = goal_pos[0] - current_pos[0]
        dy = goal_pos[1] - current_pos[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Unit vector towards goal
        if distance > 0:
            fx = self.attractive_constant * dx / distance
            fy = self.attractive_constant * dy / distance
        else:
            fx, fy = 0, 0

        return np.array([fx, fy])

    def repulsive_force(self, current_pos, obstacle_pos):
        dx = current_pos[0] - obstacle_pos[0]
        dy = current_pos[1] - obstacle_pos[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < self.min_distance:
            distance = self.min_distance  # Prevent division by zero

        if distance <= self.influence_radius:
            # Repulsive force inversely proportional to distance
            magnitude = self.repulsive_constant * (1 / distance - 1 / self.influence_radius) / (distance ** 2)

            # Unit vector away from obstacle
            if distance > 0:
                fx = magnitude * dx / distance
                fy = magnitude * dy / distance
            else:
                fx, fy = 0, 0
        else:
            fx, fy = 0, 0

        return np.array([fx, fy])

    def calculate_resultant_force(self, current_pos, goal_pos, obstacles):
        # Calculate attractive force towards goal
        f_att = self.attractive_force(current_pos, goal_pos)

        # Sum up all repulsive forces from obstacles
        f_rep = np.array([0.0, 0.0])
        for obstacle in obstacles:
            f_rep += self.repulsive_force(current_pos, obstacle)

        # Resultant force
        resultant = f_att + f_rep

        # Convert to motion command
        distance = np.linalg.norm(resultant)
        if distance > 0:
            # Normalize and scale
            direction = resultant / distance
            # Return motion vector (can be converted to speed and heading)
            return direction * min(distance, 1.0)  # Cap maximum speed
        else:
            return np.array([0.0, 0.0])