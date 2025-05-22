import RPi.GPIO as GPIO
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

pull_switch_pin = 16  # GPIO pin number for the pull switch
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(pull_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set pin as input with pull-up resistor

while True:
    try:
        # Read the state of the pull switch
        pull_switch_state = GPIO.input(pull_switch_pin)
        if pull_switch_state == GPIO.LOW:  # Switch is pressed
            logger.info("Pull switch is pressed")
        else:  # Switch is not pressed
            logger.info("Pull switch is not pressed")

        time.sleep(0.1)  # Small delay to avoid excessive CPU usage
    except KeyboardInterrupt:
        logger.info("Exiting...")
        break
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        break