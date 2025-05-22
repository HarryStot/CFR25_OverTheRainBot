# switch_launcher.py
import RPi.GPIO as GPIO
import subprocess
import time
import os
import sys
import signal

# --- Configuration ---
# GPIO pin your switch is connected to (using BCM numbering)
SWITCH_PIN = 16  # Using GPIO 16 as in your example

# Path to your main.py script
MAIN_SCRIPT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main.py")

# Path to your virtual environment's Python interpreter
VENV_PYTHON = "/home/overtherainbot/.virtualenvs/Lidar/bin/python"

# How often to check switch state (seconds)
POLLING_INTERVAL = 0.2


# --- Script Logic ---
def setup_gpio():
    """Sets up the GPIO pin for the switch."""
    try:
        GPIO.setmode(GPIO.BCM)
        # Setup the switch pin as input with an internal pull-up resistor
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print(f"Switch pin {SWITCH_PIN} configured as input with pull-up.")

        # Test reading the pin to ensure it's working
        pin_value = GPIO.input(SWITCH_PIN)
        print(f"Current switch state: {'OFF' if pin_value else 'ON'}")
        return True
    except Exception as e:
        print(f"Error setting up GPIO: {e}")
        return False


def main():
    """Main function to monitor switch and control the script."""
    print("Switch launcher started.")
    print(f"Will control: {MAIN_SCRIPT_PATH}")

    process = None
    last_switch_state = GPIO.input(SWITCH_PIN)
    print(f"Initial switch state: {'OFF' if last_switch_state else 'ON'}")

    try:
        while True:
            # Get current switch state (LOW/False = ON, HIGH/True = OFF with pull-up)
            current_switch_state = GPIO.input(SWITCH_PIN)

            # If switch state changed
            if current_switch_state != last_switch_state:
                if current_switch_state == GPIO.LOW:  # Switch turned ON
                    print("Switch turned ON - Starting main.py")
                    try:
                        # Start the process using the virtual environment Python
                        process = subprocess.Popen([VENV_PYTHON, MAIN_SCRIPT_PATH])
                        print(f"Started main.py with PID: {process.pid}")
                    except Exception as e:
                        print(f"Error starting process: {e}")
                        process = None
                else:  # Switch turned OFF
                    print("Switch turned OFF - Stopping main.py gracefully")
                    if process and process.poll() is None:  # If process is running
                        try:
                            # Send SIGTERM to trigger graceful shutdown via signal_handler
                            print(f"Sending SIGTERM to PID: {process.pid} for graceful shutdown")
                            process.terminate()  # This sends SIGTERM

                            # Give it generous time to shut down gracefully
                            # Your main.py needs time to stop all threads properly
                            print("Waiting for graceful shutdown (up to 15 seconds)...")
                            try:
                                exit_code = process.wait(timeout=15)
                                print(f"Process exited gracefully with code: {exit_code}")
                            except subprocess.TimeoutExpired:
                                print("Graceful shutdown timeout - sending SIGKILL...")
                                process.kill()
                                exit_code = process.wait()
                                print(f"Process force-killed with exit code: {exit_code}")

                        except Exception as e:
                            print(f"Error stopping process: {e}")
                    else:
                        print("No process running or process already exited")
                    process = None

                # Update last state
                last_switch_state = current_switch_state

            # Check if the process unexpectedly terminated while switch is ON
            if process and current_switch_state == GPIO.LOW and process.poll() is not None:
                exit_code = process.poll()
                print(f"WARNING: Process exited unexpectedly with code: {exit_code}")
                print("Switch is still ON - restarting process")
                try:
                    process = subprocess.Popen([VENV_PYTHON, MAIN_SCRIPT_PATH])
                    print(f"Restarted main.py with PID: {process.pid}")
                except Exception as e:
                    print(f"Error restarting process: {e}")
                    process = None

            # Sleep to avoid CPU hogging
            time.sleep(POLLING_INTERVAL)

    except KeyboardInterrupt:
        print("Switch launcher stopped by user.")
    except Exception as e:
        print(f"An error occurred in the switch launcher: {e}")
    finally:
        # Clean up before exiting
        if process and process.poll() is None:
            print("Terminating main.py process gracefully before exit")
            try:
                # Send SIGTERM for graceful shutdown
                process.terminate()
                print("Waiting for graceful shutdown...")
                try:
                    process.wait(timeout=15)
                    print("Process shut down gracefully")
                except subprocess.TimeoutExpired:
                    print("Graceful shutdown timeout - force killing")
                    process.kill()
                    process.wait()
            except Exception as e:
                print(f"Error during cleanup: {e}")

        GPIO.cleanup()
        print("GPIO cleanup done. Exiting.")


if __name__ == '__main__':
    if setup_gpio():
        main()
    else:
        print("Failed to set up GPIO. Exiting.")
        sys.exit(1)