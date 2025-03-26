#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import argparse
import time
import sys
import signal

# Import the LidarProcessor
from lidar_processor import LidarProcessor


def parse_args():
    parser = argparse.ArgumentParser(description='Test LIDAR clustering')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='LIDAR device port')
    parser.add_argument('--range', type=float, default=3.0,
                        help='Visualization range in meters')
    return parser.parse_args()


def main():
    # Parse command line arguments
    args = parse_args()

    # Initialize the plot
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-args.range, args.range)
    ax.set_ylim(-args.range, args.range)
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_title('LIDAR Scan with Obstacle Clustering')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')

    # Plot elements for raw data and statistics text
    raw_points, = ax.plot([], [], 'bo', markersize=2, alpha=0.5, label='Raw points')
    stats_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                         verticalalignment='top', bbox=dict(boxstyle='round', alpha=0.2))

    # Initialize LidarProcessor
    lidar_processor = None
    obstacle_circles = []  # Will store circle patches

    try:
        print(f"Connecting to LIDAR on port {args.port}...")
        lidar_processor = LidarProcessor(port=args.port)

        # Start the LIDAR
        print("Starting LIDAR scanning...")
        lidar_processor.start_scanning()

        # Give the LIDAR a moment to start collecting data
        time.sleep(2)

        def init():
            """Initialize the animation"""
            raw_points.set_data([], [])
            stats_text.set_text('')
            return [raw_points, stats_text]

        def update(frame):
            """Update the visualization for each frame"""
            nonlocal obstacle_circles

            # Clear previous obstacle circles
            for circle in obstacle_circles:
                circle.remove()
            obstacle_circles = []

            # Get raw data points
            raw_data = lidar_processor.get_obstacles_raw()

            # Update raw points
            if raw_data:
                x_points = [p[0] for p in raw_data]
                y_points = [p[1] for p in raw_data]
                raw_points.set_data(x_points, y_points)

            # Process and get obstacle clusters
            obstacles = lidar_processor.process_obstacles()

            # Draw obstacle circles
            for x, y, radius in obstacles:
                circle = Circle((x, y), radius, fill=False, color='r', linewidth=2)
                ax.add_patch(circle)
                obstacle_circles.append(circle)

            # Update statistics
            stats_text.set_text(f'Raw points: {len(raw_data)}\n'
                                f'Obstacles: {len(obstacles)}')

            return [raw_points, stats_text] + obstacle_circles

        # Set up animation
        ani = animation.FuncAnimation(fig, update, init_func=init,
                                      interval=100, blit=True)

        # Handle Ctrl+C gracefully
        def on_close(event):
            if lidar_processor:
                print("Stopping LIDAR...")
                lidar_processor.stop()

        fig.canvas.mpl_connect('close_event', on_close)

        # Show the plot
        plt.legend()
        plt.tight_layout()
        plt.show()

    except KeyboardInterrupt:
        print("Test interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if lidar_processor:
            print("Stopping LIDAR...")
            lidar_processor.stop()


if __name__ == "__main__":
    main()