# robot_interface.py
import time
import threading
from position_manager import position_manager
import logging

logger = logging.getLogger(__name__)


class RobotInterface(threading.Thread):
    """Updates robot position from external sources"""

    def __init__(self, stop_event=None):
        threading.Thread.__init__(self)
        self.stop_event = stop_event or threading.Event()
        self.daemon = True

    def run(self):
        try:
            while not self.stop_event.is_set():
                # Example: read position from file
                try:
                    with open('robot_position.txt', 'r') as f:
                        data = f.read().strip().split(',')
                        if len(data) >= 5:
                            position_manager.set_position(
                                float(data[0]),
                                float(data[1]),
                                float(data[2])
                            )
                            position_manager.set_target(
                                float(data[3]),
                                float(data[4])
                            )
                except Exception as e:
                    pass  # File might not exist yet

                time.sleep(0.1)  # Check for updates 10 times per second

        except Exception as e:
            logger.error(f"Error in RobotInterface: {e}")