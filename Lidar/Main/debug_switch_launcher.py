# debug_switch_launcher.py
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
MAIN_SCRIPT_PATH = "/home/overtherainbot/src/Main/main.py"

# Path to your virtual environment's Python interpreter
VENV_PYTHON = "/home/overtherainbot/.virtualenvs/Lidar/bin/python"

# How often to check switch state (seconds)
POLLING_INTERVAL = 0.2


# --- Script Logic ---
def setup_gpio():
    """Sets up the GPIO pin for the switch."""
    try:
        print("Setting up GPIO...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print(f"Switch pin {SWITCH_PIN} configured as input with pull-up.")

        # Test reading the pin multiple times
        for i in range(5):
            pin_value = GPIO.input(SWITCH_PIN)
            print(f"Pin read {i + 1}: {pin_value} ({'HIGH/OFF' if pin_value else 'LOW/ON'})")
            time.sleep(0.1)

        return True
    except Exception as e:
        print(f"Error setting up GPIO: {e}")
        return False


def test_script_execution():
    """Test if we can execute the main script."""
    print(f"Testing script execution...")
    print(f"Virtual environment Python: {VENV_PYTHON}")
    print(f"Main script path: {MAIN_SCRIPT_PATH}")

    # Check if files exist
    if not os.path.exists(VENV_PYTHON):
        print(f"ERROR: Virtual environment Python not found at {VENV_PYTHON}")
        return False

    if not os.path.exists(MAIN_SCRIPT_PATH):
        print(f"ERROR: Main script not found at {MAIN_SCRIPT_PATH}")
        return False

    print("Both files exist. Testing basic execution...")

    # Test basic Python execution
    try:
        result = subprocess.run([VENV_PYTHON, "--version"],
                                capture_output=True, text=True, timeout=5)
        print(f"Python version: {result.stdout.strip()}")
    except Exception as e:
        print(f"Error testing Python: {e}")
        return False

    return True


def main():
    """Main function to monitor switch and control the script."""
    print("=== DEBUG Switch Launcher Started ===")
    print(f"PID: {os.getpid()}")
    print(f"User: {os.getenv('USER', 'unknown')}")
    print(f"Working directory: {os.getcwd()}")

    if not test_script_execution():
        print("Script execution test failed. Exiting.")
        return

    process = None
    last_switch_state = None
    state_change_count = 0

    try:
        while True:
            # Get current switch state (LOW/False = ON, HIGH/True = OFF with pull-up)
            current_switch_state = GPIO.input(SWITCH_PIN)

            # Print state periodically (every 5 seconds)
            if state_change_count % 25 == 0:  # Every 5 seconds (0.2s * 25)
                print(f"Switch state: {current_switch_state} ({'OFF' if current_switch_state else 'ON'})")
                if process:
                    if process.poll() is None:
                        print(f"Process running with PID: {process.pid}")
                    else:
                        print(f"Process not running (exit code: {process.poll()})")
                else:
                    print("No process started")

            # If switch state changed
            if current_switch_state != last_switch_state and last_switch_state is not None:
                print(f"=== SWITCH STATE CHANGED ===")
                print(f"From: {last_switch_state} ({'OFF' if last_switch_state else 'ON'})")
                print(f"To: {current_switch_state} ({'OFF' if current_switch_state else 'ON'})")

                if current_switch_state == GPIO.LOW:  # Switch turned ON
                    print(">>> Switch turned ON - Starting main.py")
                    if process and process.poll() is None:
                        print("WARNING: Process already running!")
                    else:
                        try:
                            print(f"Executing: {VENV_PYTHON} {MAIN_SCRIPT_PATH}")
                            # Start the process without waiting for it to complete
                            process = subprocess.Popen([VENV_PYTHON, MAIN_SCRIPT_PATH],
                                                       stdout=subprocess.PIPE,
                                                       stderr=subprocess.PIPE)
                            print(f"Started main.py with PID: {process.pid}")
                        except Exception as e:
                            print(f"ERROR starting process: {e}")
                            process = None

                else:  # Switch turned OFF
                    print(">>> Switch turned OFF - Stopping main.py gracefully")
                    if process and process.poll() is None:  # If process is running
                        try:
                            print(f"Sending SIGTERM to PID: {process.pid}")
                            process.terminate()

                            print("Waiting for graceful shutdown (up to 10 seconds)...")
                            try:
                                exit_code = process.wait(timeout=10)
                                print(f"Process exited gracefully with code: {exit_code}")
                            except subprocess.TimeoutExpired:
                                print("Graceful shutdown timeout - sending SIGKILL...")
                                process.kill()
                                exit_code = process.wait()
                                print(f"Process force-killed with exit code: {exit_code}")

                        except Exception as e:
                            print(f"ERROR stopping process: {e}")
                    else:
                        print("No process running or process already exited")
                    process = None

            # Update last state
            last_switch_state = current_switch_state
            state_change_count += 1

            # Check if the process unexpectedly terminated while switch is ON
            if (process and current_switch_state == GPIO.LOW and
                    process.poll() is not None):
                exit_code = process.poll()
                print(f"WARNING: Process exited unexpectedly with code: {exit_code}")

                # Get stdout and stderr if available
                try:
                    stdout, stderr = process.communicate(timeout=1)
                    if stdout:
                        print(f"STDOUT: {stdout.decode()[-200:]}")  # Last 200 chars
                    if stderr:
                        print(f"STDERR: {stderr.decode()[-200:]}")  # Last 200 chars
                except:
                    pass

                print("Switch is still ON - restarting process in 2 seconds...")
                time.sleep(2)
                try:
                    process = subprocess.Popen([VENV_PYTHON, MAIN_SCRIPT_PATH],
                                               stdout=subprocess.PIPE,
                                               stderr=subprocess.PIPE)
                    print(f"Restarted main.py with PID: {process.pid}")
                except Exception as e:
                    print(f"ERROR restarting process: {e}")
                    process = None

            # Sleep to avoid CPU hogging
            time.sleep(POLLING_INTERVAL)

    except KeyboardInterrupt:
        print("\n=== Switch launcher stopped by user ===")
    except Exception as e:
        print(f"ERROR in switch launcher: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up before exiting
        if process and process.poll() is None:
            print("Terminating main.py process gracefully before exit")
            try:
                process.terminate()
                process.wait(timeout=10)
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