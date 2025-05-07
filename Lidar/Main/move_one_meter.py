#!/usr/bin/env python3

import logging
import signal
import threading
import time

from position_manager import position_manager
from robot_brain import RobotBrain, Location, RobotState
from robot_interface import RobotInterface

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

running = True

def signal_handler(sig, frame):
    global running
    logger.info("Signal d'arrêt reçu, arrêt en cours...")
    running = False


def main():
    # Enregistrement du gestionnaire de signal pour un arrêt propre
    signal.signal(signal.SIGINT, signal_handler)

    stop_event = threading.Event()
    robot_interface = None

    try:
        # Initialisation de l'interface robot
        logger.info("Démarrage de l'interface robot...")
        robot_interface = RobotInterface(serial_port='/dev/ttyACM0',
                                         baud_rate=115200,
                                         stop_event=stop_event)
        robot_interface.start()
        logger.info("Interface robot démarrée")

        # Attendre que l'interface soit prête
        time.sleep(5)
        robot_interface.send_command("S")

        # Commande pour avancer d'un mètre (100cm)
        logger.info("Envoi de la commande pour avancer d'un mètre")
        robot_interface.send_command("GX100Y0Z0")

        # Attendre que le mouvement soit terminé
        # Le temps d'attente dépend de la vitesse du robot
        logger.info("Attente de la fin du mouvement...")
        time.sleep(10)  # Ajuster selon la vitesse du robot

        # Arrêter le robot
        logger.info("Arrêt du robot")
        robot_interface.send_command("S")

    except Exception as e:
        logger.error(f"Erreur dans main: {e}")
    finally:
        # Signal d'arrêt
        logger.info("Arrêt de tous les threads...")
        stop_event.set()

        if robot_interface and robot_interface.is_alive():
            logger.info("Attente de l'arrêt de RobotInterface...")
            robot_interface.join(timeout=5)

        logger.info("Terminé!")


if __name__ == '__main__':
    main()