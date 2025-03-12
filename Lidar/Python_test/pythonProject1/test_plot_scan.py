from pyrplidar import PyRPlidar
import matplotlib.pyplot as plt
import numpy as np
import time
import math


def lidar_live_plot():
    # Connect to LIDAR
    lidar = PyRPlidar()
    try:
        lidar.connect(port="COM3", baudrate=256000, timeout=3)
        print("Connected to LIDAR")
    except Exception as e:
        print(f"Error connecting to LIDAR: {e}")
        return

    # Spin up the motor
    lidar.set_motor_pwm(500)
    print("Waiting for motor to spin up...")
    time.sleep(2)  # Wait for motor to stabilize

    # Initialize plot in interactive mode
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = plt.subplot(111, projection='polar')
    ax.set_title('RPLIDAR Scan')
    ax.set_theta_zero_location('N')  # 0 degrees points North/up
    ax.set_theta_direction(-1)  # Clockwise rotation
    ax.set_rlim(0, 6000)  # Range limit in mm (6 meters)

    # Start scan - try different methods if one fails
    scan_generator = None
    try:
        print("Starting standard scan...")
        scan_generator = lidar.start_scan()
    except Exception as e:
        print(f"Error starting standard scan: {e}")
        try:
            print("Trying force scan...")
            scan_generator = lidar.force_scan()
        except Exception as e:
            print(f"Error starting force scan: {e}")
            lidar.stop()
            lidar.set_motor_pwm(0)
            lidar.disconnect()
            return

    # Create scatter plot (initially empty)
    scatter = ax.scatter([], [], s=5, c='red')

    try:
        while True:
            angles = []
            distances = []

            # Collect a batch of measurements
            count = 0
            for scan in scan_generator():
                angle_rad = math.radians(scan.angle)
                distance = scan.distance

                # Filter out invalid measurements
                if distance > 0:
                    angles.append(angle_rad)
                    distances.append(distance)

                count += 1
                if count >= 100:  # Adjust for performance
                    break

            # Update the visualization
            if angles and distances:
                scatter.set_offsets(np.column_stack([angles, distances]))
                fig.canvas.flush_events()

            plt.pause(0.05)  # Allow GUI to update

    except KeyboardInterrupt:
        print("Scan interrupted by user")
    except Exception as e:
        print(f"Error during scanning: {e}")
    finally:
        # Clean up
        print("Stopping LIDAR...")
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        plt.ioff()
        plt.close()


if __name__ == "__main__":
    lidar_live_plot()