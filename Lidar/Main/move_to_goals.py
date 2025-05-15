#!/usr/bin/env python3

import logging
import signal
import threading
import time
import argparse

from robot_interface import RobotInterface

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

running = True

def signal_handler(sig, frame):
    global running
    logger.info("Received shutdown signal, stopping...")
    running = False

def move_to_goals(robot_interface, goals, velocity=150, wait_time=2):
    """
    Move the robot to a sequence of goal positions.
    
    Args:
        robot_interface: The robot interface to send commands to
        goals: List of (x, y, z) tuples where z is orientation
        velocity: Movement velocity (default: 150)
        wait_time: Time to wait after each goal is reached (default: 2 seconds)
    """
    if not goals:
        logger.warning("No goals provided. Nothing to do.")
        return
    
    logger.info(f"Starting movement to {len(goals)} goals with velocity {velocity}")
    robot_interface.send_command("S")  # Stop any existing movement
    robot_interface.send_command(f"V{velocity}")  # Set velocity
    
    for i, goal in enumerate(goals):
        if not running:
            logger.info("Stopping due to interrupt")
            break
            
        # Extract coordinates, handle both (x,y,z) and (x,y) formats
        if len(goal) == 3:
            x, y, z = goal
        elif len(goal) == 2:
            x, y = goal
            z = 0.0  # Default orientation if not provided
        else:
            logger.error(f"Invalid goal format: {goal}")
            continue
        
        logger.info(f"Moving to goal {i+1}/{len(goals)}: X={x}, Y={y}, Z={z}")
        
        # Send movement command
        robot_interface.send_command(f"GX{x:.2f}Y{y:.2f}Z{z:.2f}")
        
        # Wait for the movement to complete
        logger.info(f"Waiting {wait_time} seconds for the movement to complete...")
        time.sleep(wait_time)
    
    # Stop the robot after reaching all goals
    logger.info("Stopping the robot")
    robot_interface.send_command("S")

def main():
    parser = argparse.ArgumentParser(description='Move robot to a sequence of goal positions')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='Serial port for robot communication')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial communication')
    parser.add_argument('--velocity', type=int, default=150, help='Movement velocity (1-255)')
    parser.add_argument('--wait', type=float, default=2.0, help='Wait time after reaching each goal (seconds)')
    args = parser.parse_args()
    
    # Register signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Example goals: List of (x, y, z) tuples where z is orientation in degrees
    # These can be replaced with actual goal coordinates
    goals = [
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 90.0)
    ]

    stop_event = threading.Event()
    robot_interface = None

    try:
        # Robot interface initialization
        logger.info("Starting robot interface...")
        robot_interface = RobotInterface(serial_port=args.port,
                                         baud_rate=args.baud,
                                         stop_event=stop_event)
        robot_interface.start()
        logger.info("Robot interface started")

        # Wait for the robot to be ready
        time.sleep(3)
        
        # Move to all goals
        move_to_goals(robot_interface, goals, args.velocity, args.wait)

    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        logger.info("Stopping all threads...")
        stop_event.set()

        if robot_interface and robot_interface.is_alive():
            logger.info("Waiting for robot interface to finish...")
            robot_interface.join(timeout=5)

        logger.info("Bye!")


if __name__ == '__main__':
    main()