#!/usr/bin/env python3
# switch_test.py - Simple script to test switch wiring

import RPi.GPIO as GPIO
import time

SWITCH_PIN = 16


def test_switch():
    print("=== Simple Switch Test ===")
    print("This will continuously read the switch state.")
    print("Toggle your switch to see if it changes.")
    print("Press Ctrl+C to exit")
    print()

    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        last_state = None

        while True:
            current_state = GPIO.input(SWITCH_PIN)

            if current_state != last_state:
                print(f"Switch changed: {current_state} ({'OFF' if current_state else 'ON'})")
                last_state = current_state

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nTest stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        GPIO.cleanup()
        print("GPIO cleanup done")


if __name__ == '__main__':
    test_switch()