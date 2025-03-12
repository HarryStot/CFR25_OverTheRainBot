import matplotlib.pyplot as plt
import numpy as np
import random
import math
from sklearn.cluster import DBSCAN
from collections import defaultdict


def generate_obstacles_raw():
    """Generate random obstacles_raw for testing
    :returns: List of (x, y) points representing raw LiDAR readings of obstacle edges
    """
    obstacles_raw = []

    # Define table boundaries (3m x 2m)
    table_width = 3.0
    table_height = 2.0

    # Generate 2-4 random box obstacles
    num_obstacles = random.randint(2, 4)

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

                # Add some noise to simulate real LiDAR readings
                noise_x = random.gauss(0, 0.005)  # 5mm standard deviation
                noise_y = random.gauss(0, 0.005)

                obstacles_raw.append((x + noise_x, y + noise_y))

    # Add some random noise points (false positives)
    num_noise = random.randint(5, 15)
    for _ in range(num_noise):
        x = random.uniform(0, table_width)
        y = random.uniform(0, table_height)
        obstacles_raw.append((x, y))

    return obstacles_raw


def process_obstacles(obstacles_raw):
    """Process raw obstacles to filter obstacles and determine their centers and radii
    :arg:
        obstacles_raw: List of (x, y) points from LiDAR
    :returns:
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

        # Simple outlier rejection - if the radius is too large relative to the number of points,
        # it might be a noise cluster or multiple merged obstacles
        if radius < 0.5:  # Maximum expected obstacle radius
            obstacles.append((center_x, center_y, radius))

    return obstacles


def plot_obstacles(obstacles_raw, obstacles):
    """Plot raw and processed obstacles"""
    plt.figure(figsize=(8, 4))
    plt.subplot(1, 2, 1)
    plt.title("Raw Obstacles")
    for obstacle in obstacles_raw:
        x, y = obstacle
        plt.plot(x, y, 'ro')
    plt.axis('equal')
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.title("Processed Obstacles")
    for obstacle in obstacles:
        x, y = obstacle
        plt.plot(x, y, 'bo')
    plt.axis('equal')
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    obstacles_raw = generate_obstacles_raw()
    robot_height = 0.1
    obstacles = process_obstacles(obstacles_raw)
    plot_obstacles(obstacles_raw, obstacles)