#!/usr/bin/env python3

import time
import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar
import math

# Configuration
PORT_NAME = '/dev/ttyUSB0'  # This might be different on your system


# Common alternatives are: /dev/ttyAMA0, /dev/ttyS0

def run():
    """Main function to run the LiDAR scanning"""
    lidar = RPLidar(PORT_NAME)

    try:
        print('Press Ctrl+C to stop')

        # Display device information
        info = lidar.get_info()
        print(f"LiDAR info: {info}")

        health = lidar.get_health()
        print(f"LiDAR health: {health}")

        # Start scanning
        print("Starting scan...")

        for i, scan in enumerate(lidar.iter_scans()):
            if i > 10:  # Just display 10 scans for demonstration
                break

            print(f"\nScan #{i + 1}")
            print(f"Number of points: {len(scan)}")

            # Print first 5 points for demonstration
            for point_num, (quality, angle, distance) in enumerate(scan[:5]):
                print(f"Point {point_num + 1}: Quality: {quality}, Angle: {angle}°, Distance: {distance} mm")

        # Plot one scan
        plot_scan(lidar)

    except KeyboardInterrupt:
        print('Stopping...')
    finally:
        # Stop the motor and disconnect
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()


def plot_scan(lidar):
    """Plot a single scan from the LiDAR"""
    # Get a single complete scan
    scan = next(lidar.iter_scans())

    # Create polar plot
    plt.figure(figsize=(8, 8))
    ax = plt.subplot(111, projection='polar')

    # Extract angles and distances
    angles = np.array([point[1] for point in scan])
    distances = np.array([point[2] for point in scan])

    # Convert to radians for polar plot
    angles_rad = np.radians(angles)

    # Plot the scan
    ax.scatter(angles_rad, distances, c='blue', s=5)

    # Set plot title and labels
    ax.set_title('LiDAR Scan (Polar Coordinates)')
    ax.set_rticks([0, 1000, 2000, 3000, 4000, 5000])  # Distance in mm
    plt.grid(True)

    # Save the plot
    plt.savefig('lidar_scan.png')
    print("Scan plot saved as 'lidar_scan.png'")


if __name__ == '__main__':
    run()