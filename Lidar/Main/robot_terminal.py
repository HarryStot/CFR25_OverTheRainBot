#!/usr/bin/env python
# robot_terminal.py

import argparse
import logging
import time
import sys
import threading

from robot_interface import RobotInterface

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RobotTerminal:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, interface_type='movement'):
        self.port = port
        self.baud_rate = baud_rate
        self.interface_type = interface_type
        self.robot = None
        self.running = False

    def start(self):
        """Initialize and start the robot interface"""
        logger.info(f"Initializing robot interface on {self.port} at {self.baud_rate} baud")
        self.robot = RobotInterface(
            serial_port=self.port,
            baud_rate=self.baud_rate,
            interface_type=self.interface_type
        )

        # Start the interface thread
        self.robot.start()

        # Wait for connection
        timeout = 5  # seconds
        start_time = time.time()
        while not self.robot.connected and time.time() - start_time < timeout:
            time.sleep(0.1)

        if not self.robot.connected:
            logger.error(f"Failed to connect to robot within {timeout} seconds")
            self.robot.stop_event.set()
            return False

        logger.info(f"Connected to robot ({self.interface_type} interface)")
        return True

    def run_terminal(self):
        """Run an interactive terminal for sending commands"""
        if not self.robot or not self.robot.connected:
            if not self.start():
                return

        self.running = True
        print("\n===== Robot Command Terminal =====")
        print(f"Interface type: {self.interface_type}")
        print("Type 'exit' or 'quit' to end the session")
        print("Type 'help' for available commands")
        print("=====================================")

        while self.running and self.robot.connected:
            try:
                # Get user input for command
                command = input("\nEnter command: ").strip()

                if command.lower() in ('exit', 'quit'):
                    print("Exiting terminal...")
                    self.running = False
                    break

                elif command.lower() == 'help':
                    self.show_help()

                elif command.lower() == 'status':
                    self.show_status()

                elif command:
                    # Send command to robot
                    self.robot.send_command(command)

                    # If requesting sensor data, wait and display it
                    if self.interface_type == 'action' and command == 'R':
                        # Clear the event flag
                        self.robot.us_data_received.clear()

                        # Wait for up to 2 seconds for data
                        if self.robot.us_data_received.wait(2):
                            print("Ultrasonic sensor data:")
                            for sensor_id, distance in self.robot.get_ultrasonic_data().items():
                                print(f"  Sensor {sensor_id}: {distance} cm")
                        else:
                            print("No sensor data received")

            except KeyboardInterrupt:
                print("\nKeyboard interrupt received, exiting...")
                self.running = False
                break

            except Exception as e:
                logger.error(f"Error in terminal: {e}")

        # Cleanup
        self.stop()

    def stop(self):
        """Stop the robot interface"""
        if self.robot:
            logger.info("Shutting down robot interface...")
            self.robot.stop_event.set()
            self.robot.join(timeout=2)

    def show_help(self):
        """Display help information"""
        print("\n----- Available Commands -----")
        print("exit, quit - Exit the terminal")
        print("help       - Show this help message")
        print("status     - Show robot connection status")

        if self.interface_type == 'movement':
            print("\nMovement Interface Commands:")
            print("  (Add specific movement commands here)")
        elif self.interface_type == 'action':
            print("\nAction Interface Commands:")
            print("  R         - Request ultrasonic sensor data")
            print("  (Add other action commands here)")

    def show_status(self):
        """Display robot status"""
        if not self.robot:
            print("Robot interface not initialized")
            return

        print("\n----- Robot Status -----")
        print(f"Connected: {self.robot.connected}")
        print(f"Interface Type: {self.robot.interface_type}")
        print(f"Serial Port: {self.robot.serial_port}")
        print(f"Baud Rate: {self.robot.baud_rate}")

        if self.interface_type == 'action':
            print("Ultrasonic Sensor Data:")
            for sensor_id, distance in self.robot.get_ultrasonic_data().items():
                print(f"  Sensor {sensor_id}: {distance} cm")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Interactive terminal for sending commands to the robot')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                        help='Serial port for Arduino connection')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate for serial connection')
    parser.add_argument('--type', type=str, choices=['movement', 'action'], default='movement',
                        help='Interface type (movement or action)')

    args = parser.parse_args()

    # Create and run the terminal
    terminal = RobotTerminal(
        port=args.port,
        baud_rate=args.baud,
        interface_type=args.type
    )

    try:
        terminal.run_terminal()
    except Exception as e:
        logger.error(f"Error running terminal: {e}")
    finally:
        terminal.stop()

if __name__ == "__main__":
    main()
