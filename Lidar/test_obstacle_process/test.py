import numpy as np
import random
import math
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from collections import defaultdict
import time
from matplotlib.animation import FuncAnimation


def generate_obstacles_raw(robot_pos=(0.5, 0.5), lidar_range=3.0, lidar_angle_min=0, lidar_angle_max=360):
    """Generate random obstacles_raw for testing with lidar limitations

    Args:
        robot_pos: (x, y) position of the robot/lidar
        lidar_range: Maximum range of the lidar in meters
        lidar_angle_min: Minimum angle in degrees (0 = right, 90 = up)
        lidar_angle_max: Maximum angle in degrees

    Returns:
        List of (x, y) points representing raw LiDAR readings of obstacle edges
    """
    obstacles_raw = []

    # Define table boundaries (3m x 2m)
    table_width = 3.0
    table_height = 2.0

    # Generate 2-4 random box obstacles
    num_obstacles = random.randint(2, 4)
    obstacle_corners = []

    for _ in range(num_obstacles):
        # Random box center position within table boundaries (with margins)
        margin = 0.2
        center_x = random.uniform(margin, table_width - margin)
        center_y = random.uniform(margin, table_height - margin)

        # Random box dimensions
        box_width = random.uniform(0.1, 0.3)
        box_height = random.uniform(0.1, 0.3)

        # Calculate box vertices
        half_w = box_width / 2
        half_h = box_height / 2

        # Box corners
        corners = [
            (center_x - half_w, center_y - half_h),  # bottom-left
            (center_x + half_w, center_y - half_h),  # bottom-right
            (center_x + half_w, center_y + half_h),  # top-right
            (center_x - half_w, center_y + half_h)  # top-left
        ]
        obstacle_corners.append(corners)

        # Simulate LiDAR scan points along each edge
        for i in range(4):
            start = corners[i]
            end = corners[(i + 1) % 4]

            # Number of points to generate along this edge
            edge_length = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
            num_points = max(3, int(edge_length * 20))  # Approx. 20 points per meter

            # Generate points along the edge
            for j in range(num_points):
                ratio = j / (num_points - 1)
                x = start[0] + ratio * (end[0] - start[0])
                y = start[1] + ratio * (end[1] - start[1])

                # Check if point is visible from robot position
                if is_point_visible(robot_pos, (x, y), obstacle_corners[:-1], lidar_range,
                                    lidar_angle_min, lidar_angle_max):
                    # Add some noise to simulate real LiDAR readings
                    noise_x = random.gauss(0, 0.005)  # 5mm standard deviation
                    noise_y = random.gauss(0, 0.005)

                    obstacles_raw.append((x + noise_x, y + noise_y))

    # Add boundaries of the table as obstacles
    table_corners = [
        (0, 0), (table_width, 0),
        (table_width, table_height), (0, table_height)
    ]

    boundary_points = []
    for i in range(4):
        start = table_corners[i]
        end = table_corners[(i + 1) % 4]

        # Number of points to generate along this edge
        edge_length = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
        num_points = max(3, int(edge_length * 10))  # Fewer points for table boundaries

        # Generate points along the edge
        for j in range(num_points):
            ratio = j / (num_points - 1)
            x = start[0] + ratio * (end[0] - start[0])
            y = start[1] + ratio * (end[1] - start[1])

            # Check if point is visible from robot position
            if is_point_visible(robot_pos, (x, y), obstacle_corners, lidar_range,
                                lidar_angle_min, lidar_angle_max):
                # Add some noise to simulate real LiDAR readings
                noise_x = random.gauss(0, 0.003)  # 3mm standard deviation
                noise_y = random.gauss(0, 0.003)

                boundary_points.append((x + noise_x, y + noise_y))

    # Add table boundaries to the obstacles
    obstacles_raw.extend(boundary_points)

    # Add some random noise points (false positives)
    num_noise = random.randint(5, 15)
    for _ in range(num_noise):
        x = random.uniform(0, table_width)
        y = random.uniform(0, table_height)
        # Check if noise point is in lidar range and field of view
        if is_point_visible(robot_pos, (x, y), obstacle_corners, lidar_range,
                            lidar_angle_min, lidar_angle_max):
            obstacles_raw.append((x, y))

    return obstacles_raw, obstacle_corners


def is_point_visible(robot_pos, point, obstacles_list, max_range, angle_min, angle_max):
    """Check if a point is visible from the robot position given lidar limitations

    Args:
        robot_pos: (x, y) position of the robot/lidar
        point: (x, y) position of the point to check
        obstacles_list: List of obstacle corners for occlusion checking
        max_range: Maximum lidar range
        angle_min: Minimum angle in degrees
        angle_max: Maximum angle in degrees

    Returns:
        Boolean indicating if the point is visible
    """
    # Check if point is within range
    dx = point[0] - robot_pos[0]
    dy = point[1] - robot_pos[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)

    if distance > max_range:
        return False

    # Check if point is within angle range
    angle = math.degrees(math.atan2(dy, dx)) % 360
    if not (angle_min <= angle <= angle_max or angle_min > angle_max and (angle <= angle_max or angle >= angle_min)):
        return False

    # Check for occlusions
    for corners in obstacles_list:
        for i in range(4):
            start = corners[i]
            end = corners[(i + 1) % 4]

            # Check if line from robot to point intersects with this edge
            if line_segments_intersect(robot_pos, point, start, end):
                return False

    return True


def line_segments_intersect(p1, p2, p3, p4):
    """Check if line segments (p1,p2) and (p3,p4) intersect"""

    # Excluding endpoints to avoid false positives for adjacent edges

    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # collinear
        return 1 if val > 0 else 2  # clockwise or counterclockwise

    def on_segment(p, q, r):
        return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

    # If the endpoints are the same, they're not considered "intersecting"
    if p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4:
        return False

    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases
    if o1 == 0 and on_segment(p1, p3, p2):
        return True
    if o2 == 0 and on_segment(p1, p4, p2):
        return True
    if o3 == 0 and on_segment(p3, p1, p4):
        return True
    if o4 == 0 and on_segment(p3, p2, p4):
        return True

    return False


def process_obstacles(obstacles_raw):
    """Process raw obstacles to filter obstacles and determine their centers and radii

    Args:
        obstacles_raw: List of (x, y) points from LiDAR

    Returns:
        List of tuples (center_x, center_y, radius) for each detected obstacle
    """
    if not obstacles_raw or len(obstacles_raw) < 3:
        return []

    # Convert to numpy array for clustering
    points = np.array(obstacles_raw)

    # Use DBSCAN to cluster points belonging to the same obstacle
    # eps is the maximum distance between samples (adjust based on your scale)
    # min_samples is the number of samples in a neighborhood for a point to be a core point
    clustering = DBSCAN(eps=0.1, min_samples=3).fit(points)

    # Get labels for each point (-1 means noise)
    labels = clustering.labels_

    # Group points by cluster
    clusters = defaultdict(list)
    for i, label in enumerate(labels):
        if label != -1:  # Ignore noise
            clusters[label].append(points[i])

    obstacles = []

    # Process each cluster to find center and radius
    for cluster_id, cluster_points in clusters.items():
        cluster_points = np.array(cluster_points)

        # Calculate center as mean of points
        center_x = np.mean(cluster_points[:, 0])
        center_y = np.mean(cluster_points[:, 1])

        # Calculate radius as maximum distance from center to any point
        distances = np.sqrt(np.sum(np.square(cluster_points - [center_x, center_y]), axis=1))
        radius = np.max(distances)

        # Add a small buffer to the radius for safety
        radius += 0.05

        # Simple outlier rejection - if the radius is too large relative to the number of points,
        # it might be a noise cluster or multiple merged obstacles
        if radius < 0.5:  # Maximum expected obstacle radius
            obstacles.append((center_x, center_y, radius))

    return obstacles


class PotentialFieldNavigation:
    def __init__(self):
        self.attractive_constant = .1
        self.repulsive_constant = 10.0
        self.influence_radius = 0.15  # meters
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

    def repulsive_force(self, current_pos, obstacle_pos, obstacle_radius):
        dx = current_pos[0] - obstacle_pos[0]
        dy = current_pos[1] - obstacle_pos[1]
        distance = math.sqrt(dx ** 2 + dy ** 2) - obstacle_radius  # Account for obstacle size

        if distance < self.min_distance:
            distance = self.min_distance  # Prevent division by zero

        if distance <= self.influence_radius:
            # Repulsive force inversely proportional to distance
            magnitude = self.repulsive_constant * (1 / distance - 1 / self.influence_radius) / (distance ** 2)

            # Unit vector away from obstacle
            if distance > 0:
                fx = magnitude * dx / (distance + obstacle_radius)
                fy = magnitude * dy / (distance + obstacle_radius)
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
        for center_x, center_y, radius in obstacles:
            obstacle_pos = (center_x, center_y)
            f_rep += self.repulsive_force(current_pos, obstacle_pos, radius)

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


def simulate_robot_path(start_pos, goal_pos, obstacles, navigation, step_size=0.05, max_steps=500):
    """Simulate the robot's path using potential field navigation

    Args:
        start_pos: (x, y) starting position
        goal_pos: (x, y) goal position
        obstacles: List of (center_x, center_y, radius) for obstacles
        navigation: PotentialFieldNavigation instance
        step_size: Distance to move in each step
        max_steps: Maximum number of steps to simulate

    Returns:
        List of (x, y) positions representing the robot's path
    """
    path = [start_pos]
    current_pos = np.array(start_pos)
    goal_reached = False
    stuck_count = 0
    last_progress = 0

    for _ in range(max_steps):
        # Calculate the force vector
        force = navigation.calculate_resultant_force(current_pos, goal_pos, obstacles)

        # Check if we're close to the goal
        distance_to_goal = np.linalg.norm(current_pos - goal_pos)
        if distance_to_goal < 0.1:
            path.append(goal_pos)
            goal_reached = True
            break

        # Check if we're stuck
        magnitude = np.linalg.norm(force)
        if magnitude < 0.01:
            stuck_count += 1
            if stuck_count > 10:
                # We're stuck, add a random perturbation
                force += np.array([random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)])
                stuck_count = 0
        else:
            stuck_count = 0

        # Move in the direction of the force
        normalized_force = force / np.linalg.norm(force) if np.linalg.norm(force) > 0 else force
        next_pos = current_pos + normalized_force * step_size

        # Check if we're making progress
        new_distance = np.linalg.norm(next_pos - goal_pos)
        if abs(last_progress - new_distance) < 0.001:
            stuck_count += 1
        else:
            last_progress = new_distance

        # Check for collisions with obstacles
        collision = False
        for center_x, center_y, radius in obstacles:
            obstacle_pos = np.array([center_x, center_y])
            distance = np.linalg.norm(next_pos - obstacle_pos)
            if distance <= radius:
                collision = True
                break

        # Check for out of bounds
        if next_pos[0] < 0 or next_pos[0] > 3 or next_pos[1] < 0 or next_pos[1] > 2:
            collision = True

        if not collision:
            current_pos = next_pos
            path.append(tuple(current_pos))

    return path, goal_reached


def plot_obstacles(obstacles_raw, processed_obstacles, robot_pos=None, goal_pos=None, field_vectors=None, path=None,
                   lidar_fov=None):
    """Plot raw obstacles, processed obstacles, and optional robot/goal positions

    Args:
        obstacles_raw: List of (x, y) points from LiDAR
        processed_obstacles: List of (center_x, center_y, radius) for detected obstacles
        robot_pos: (x, y) tuple of robot position or None
        goal_pos: (x, y) tuple of goal position or None
        field_vectors: List of ((x, y), (dx, dy)) tuples for potential field vectors or None
        path: List of (x, y) positions representing the robot's path or None
        lidar_fov: Tuple of (robot_pos, lidar_range, angle_min, angle_max) or None
    """
    plt.figure(figsize=(12, 8))

    # Plot table boundary (3m x 2m)
    plt.plot([0, 3, 3, 0, 0], [0, 0, 2, 2, 0], 'k-', linewidth=2)

    # Plot lidar field of view if provided
    if lidar_fov:
        robot_pos, lidar_range, angle_min, angle_max = lidar_fov
        # Convert angles to radians
        angle_min_rad = math.radians(angle_min)
        angle_max_rad = math.radians(angle_max)

        # Create a wedge to represent the lidar FOV
        if angle_max - angle_min == 360:
            # Full circle
            circle = plt.Circle(robot_pos, lidar_range, fill=False, color='lightgreen', alpha=0.3)
            plt.gca().add_patch(circle)
        else:
            # Draw arc for the lidar FOV
            angles = np.linspace(angle_min_rad, angle_max_rad, 100)
            x = robot_pos[0] + lidar_range * np.cos(angles)
            y = robot_pos[1] + lidar_range * np.sin(angles)
            plt.fill([robot_pos[0], *x, robot_pos[0]], [robot_pos[1], *y, robot_pos[1]],
                     color='lightgreen', alpha=0.3)

    # Plot raw obstacle points
    raw_x = [p[0] for p in obstacles_raw]
    raw_y = [p[1] for p in obstacles_raw]
    plt.scatter(raw_x, raw_y, c='lightgray', s=10, label='Raw LiDAR Points')

    # Plot processed obstacles as circles
    for center_x, center_y, radius in processed_obstacles:
        circle = plt.Circle((center_x, center_y), radius, fill=False, color='blue', linewidth=2)
        plt.gca().add_patch(circle)
        plt.scatter(center_x, center_y, c='blue', s=30)

    # Plot potential field vectors if provided
    if field_vectors:
        for pos, vec in field_vectors:
            plt.arrow(pos[0], pos[1], vec[0] * 0.1, vec[1] * 0.1,
                      head_width=0.05, head_length=0.05, fc='black', ec='black', alpha=0.3)

    # Plot path if provided
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        plt.plot(path_x, path_y, 'g-', linewidth=2, label='Robot Path')

    # Plot robot position if provided
    if robot_pos:
        plt.scatter(robot_pos[0], robot_pos[1], c='green', s=100, label='Robot')

    # Plot goal position if provided
    if goal_pos:
        plt.scatter(goal_pos[0], goal_pos[1], c='red', s=100, label='Goal')

    plt.title('Obstacle Detection and Path Planning')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    return plt.gcf()  # Return the figure for potential further use


def animate_robot_movement(obstacles_raw, processed_obstacles, path, robot_pos, goal_pos, lidar_range=3.0, angle_min=0,
                           angle_max=360):
    """Create an animation of the robot moving along the path"""
    fig, ax = plt.subplots(figsize=(12, 8))

    def init():
        ax.clear()
        # Plot table boundary
        ax.plot([0, 3, 3, 0, 0], [0, 0, 2, 2, 0], 'k-', linewidth=2)

        # Plot obstacles
        for center_x, center_y, radius in processed_obstacles:
            circle = plt.Circle((center_x, center_y), radius, fill=False, color='blue', linewidth=2)
            ax.add_patch(circle)

        # Plot goal
        ax.scatter(goal_pos[0], goal_pos[1], c='red', s=100, label='Goal')

        # Plot raw points
        raw_x = [p[0] for p in obstacles_raw]
        raw_y = [p[1] for p in obstacles_raw]
        ax.scatter(raw_x, raw_y, c='lightgray', s=5, alpha=0.5)

        # Set up plot
        ax.set_xlim(0, 3)
        ax.set_ylim(0, 2)
        ax.grid(True)
        ax.set_title('Robot Navigation Simulation')
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')

        return []

    robot_marker = None
    lidar_fov = None
    path_line = None

    def animate(i):
        nonlocal robot_marker, lidar_fov, path_line

        # Remove previous robot and lidar FOV
        if robot_marker:
            robot_marker.remove()
        if lidar_fov:
            lidar_fov.remove()

        # Current position on path
        current_pos = path[min(i, len(path) - 1)]

        # Plot robot
        robot_marker = ax.scatter(current_pos[0], current_pos[1], c='green', s=100)

        # Plot lidar FOV
        if angle_max - angle_min == 360:
            # Full circle
            lidar_fov = plt.Circle(current_pos, lidar_range, fill=False, color='lightgreen', alpha=0.3)
            ax.add_patch(lidar_fov)
        else:
            # Draw arc for the lidar FOV
            angle_min_rad = math.radians(angle_min)
            angle_max_rad = math.radians(angle_max)
            angles = np.linspace(angle_min_rad, angle_max_rad, 100)
            x = current_pos[0] + lidar_range * np.cos(angles)
            y = current_pos[1] + lidar_range * np.sin(angles)
            lidar_fov = ax.fill([current_pos[0], *x, current_pos[0]], [current_pos[1], *y, current_pos[1]],
                                color='lightgreen', alpha=0.3)[0]

        # Plot path so far
        if path_line:
            path_line.remove()
        path_so_far = path[:min(i + 1, len(path))]
        path_x = [p[0] for p in path_so_far]
        path_y = [p[1] for p in path_so_far]
        path_line, = ax.plot(path_x, path_y, 'g-', linewidth=2)

        return [robot_marker, lidar_fov, path_line]

    # Create animation
    frames = min(len(path), 100)  # Limit frames for smoother animation
    step = max(1, len(path) // frames)

    ani = FuncAnimation(fig, animate, frames=range(0, len(path), step),
                        init_func=init, blit=True, interval=100)

    return ani


def main():
    """Main function to demonstrate obstacle detection and path planning with lidar limitations"""
    # Initialize navigation
    navigation = PotentialFieldNavigation()

    # Define robot and goal positions
    robot_pos = (0.5, 0.5)
    goal_pos = (2.5, 1.5)

    # Define lidar parameters
    lidar_range = 8.0  # 8 meter range
    lidar_angle_min = 0  # degrees
    lidar_angle_max = 360  # degrees (full circle)

    # Generate synthetic obstacles with lidar limitations
    print("Generating synthetic obstacle data with lidar limitations...")
    obstacles_raw, obstacle_corners = generate_obstacles_raw(
        robot_pos=robot_pos,
        lidar_range=lidar_range,
        lidar_angle_min=lidar_angle_min,
        lidar_angle_max=lidar_angle_max
    )
    print(f"Generated {len(obstacles_raw)} raw obstacle points")

    # Process obstacles
    print("Processing obstacles...")
    start_time = time.time()
    processed_obstacles = process_obstacles(obstacles_raw)
    processing_time = time.time() - start_time
    print(f"Detected {len(processed_obstacles)} obstacles in {processing_time:.3f} seconds")

    # Calculate potential field vectors for visualization
    field_vectors = []
    grid_size = 10
    for x in np.linspace(0.2, 2.8, grid_size):
        for y in np.linspace(0.2, 1.8, grid_size):
            pos = (x, y)
            force = navigation.calculate_resultant_force(pos, goal_pos, processed_obstacles)
            field_vectors.append((pos, force))

    # Simulate robot path
    print("Simulating robot path...")
    path, goal_reached = simulate_robot_path(robot_pos, goal_pos, processed_obstacles, navigation)

    if goal_reached:
        print(f"Goal reached in {len(path)} steps!")
    else:
        print(f"Goal not reached after {len(path)} steps.")

    # Plot results
    print("Plotting results...")
    lidar_fov = (robot_pos, lidar_range, lidar_angle_min, lidar_angle_max)
    fig = plot_obstacles(obstacles_raw, processed_obstacles, robot_pos, goal_pos,
                         field_vectors, path, lidar_fov)
    plt.savefig('obstacle_detection_with_path.png')
    plt.show()

    # Create animation
    print("Creating animation...")
    ani = animate_robot_movement(obstacles_raw, processed_obstacles, path, robot_pos, goal_pos,
                                 lidar_range, lidar_angle_min, lidar_angle_max)

    # Save animation as gif or mp4
    # ani.save('robot_navigation.gif', writer='pillow', fps=10)
    # Uncomment to save as MP4 (requires ffmpeg)
    # ani.save('robot_navigation.mp4', writer='ffmpeg', fps=10)

    plt.show()

    print("Done!")


if __name__ == "__main__":
    main()