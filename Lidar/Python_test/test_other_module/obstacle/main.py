#!/usr/bin/env python3

import time
import threading
from test_lidar_REC import LidarThread
import logging
import signal

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Global flag for graceful shutdown
running = True


def signal_handler(sig, frame):
    global running
    logger.info("Received shutdown signal, stopping...")
    running = False


if __name__ == '__main__':
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    stop_event = threading.Event()
    end_event = threading.Event()
    lidar_thread = None

    try:
        # Start the lidar thread
        lidar_thread = LidarThread('/dev/ttyUSB0', stop_event, end_event, debugCV=True)
        lidar_thread.start()
        logger.info("LidarThread started")

        # Keep main thread running until CTRL+C
        while running:
            time.sleep(0.1)

    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        # Signal to stop
        logger.info("Signaling LidarThread to stop")
        stop_event.set()
        time.sleep(1)

        # Signal to end
        logger.info("Signaling LidarThread to end")
        end_event.set()

        # Wait for thread to finish
        if lidar_thread and lidar_thread.is_alive():
            logger.info("Waiting for LidarThread to finish...")
            lidar_thread.join(timeout=5)

        logger.info("Program terminated")