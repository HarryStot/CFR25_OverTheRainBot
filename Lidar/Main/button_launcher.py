# button_launcher.py
import RPi.GPIO as GPIO
import subprocess
import time
import os

# --- Configuration ---
# GPIO pin your button is connected to (using BCM numbering)
BUTTON_PIN = 16 # FIXME: Change this

# Path to your main.py script
# Assumes main.py is in the same directory as this script.
# If not, provide the full path e.g., "/home/pi/my_project/main.py"
MAIN_SCRIPT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main.py")

# Debounce time in seconds to prevent multiple triggers from one press
DEBOUNCE_TIME = 0.5


# --- Script Logic ---
def setup_gpio():
    """Sets up the GPIO pin for the button."""
    GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
    # Setup the button pin as input with an internal pull-up resistor.
    # This means the pin will be HIGH by default, and LOW when the button is pressed (if connected to GND).
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print(f"Button pin {BUTTON_PIN} configured as input with pull-up.")


def main():
    """Main function to monitor button and launch the script."""
    print("Button launcher started. Waiting for button press...")
    print(f"Will launch: {MAIN_SCRIPT_PATH}")

    last_press_time = 0

    try:
        while True:
            # Wait for the button to be pressed (falling edge from HIGH to LOW)
            # The timeout is set so the loop can be interrupted by Ctrl+C if needed.
            GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING, timeout=60000)  # 60 second timeout then re-check

            current_time = time.time()
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:  # Check if button is actually pressed
                if (current_time - last_press_time) > DEBOUNCE_TIME:
                    print(f"Button pressed on pin {BUTTON_PIN}!")
                    last_press_time = current_time

                    try:
                        print(f"Starting {MAIN_SCRIPT_PATH}...")
                        # Run main.py and wait for it to complete.
                        # stdout and stderr will be inherited by this script.
                        process = subprocess.run(['python3', MAIN_SCRIPT_PATH], check=True)
                        print(f"{MAIN_SCRIPT_PATH} finished with return code {process.returncode}.")
                    except FileNotFoundError:
                        print(f"Error: {MAIN_SCRIPT_PATH} not found. Please check the path.")
                    except subprocess.CalledProcessError as e:
                        print(f"Error running {MAIN_SCRIPT_PATH}: {e}")
                    except Exception as e:
                        print(f"An unexpected error occurred: {e}")

                    print("Waiting for next button press...")
                else:
                    # Debounced
                    pass

            # Small sleep to prevent hogging CPU if wait_for_edge timeouts frequently
            # or if not using timeout in wait_for_edge.
            # If wait_for_edge has a timeout, this might not be strictly necessary
            # but doesn't harm.
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Button launcher stopped by user.")
    except Exception as e:
        print(f"An error occurred in the button launcher: {e}")
    finally:
        GPIO.cleanup()
        print("GPIO cleanup done. Exiting.")


if __name__ == '__main__':
    setup_gpio()
    main()
