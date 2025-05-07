#!/usr/bin/env python3

import time
import threading
import logging
import signal
import sys
import os

from robot_interface import RobotInterface
from position_manager import position_manager

# Configuration du logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Variable pour arrêt propre
running = True


def signal_handler(sig, frame):
    global running
    logger.info("Signal d'arrêt reçu, arrêt en cours...")
    running = False


# Liste des commandes prédéfinies pour test rapide
COMMANDES_PREDEFINIES = [
    ("Avancer (vitesse 100)", "V100"),
    ("Reculer (vitesse -100)", "V-100"),
    ("Arrêter", "S"),
    ("Aller à la position X=100, Y=0, Z=90", "GX100Y0Z90"),
    ("Régler la vitesse à 150", "V150"),
    ("Demander position actuelle", "P"),
    ("Commander servo 12 à position 45 vitesse 5", "SRV12:45:5")
]


def afficher_position():
    """Affiche la position et la vitesse actuelles du robot"""
    x, y, z = position_manager.get_position()
    target_x, target_y = position_manager.get_target()
    velocity = position_manager.get_velocity()

    logger.info(f"Position: X={x}, Y={y}, Z={z}")
    logger.info(f"Cible: X={target_x}, Y={target_y}")
    logger.info(f"Vitesse: {velocity}")


def main():
    """Fonction principale pour tester les commandes de mouvement"""
    global running
    signal.signal(signal.SIGINT, signal_handler)

    # Demander le port série
    default_port = '/dev/ttyACM0'
    port = input(f"Port série (défaut: {default_port}): ") or default_port

    baud_rate = 115200
    stop_event = threading.Event()

    try:
        # Démarrer l'interface robot
        robot_interface = RobotInterface(serial_port=port, baud_rate=baud_rate,
                                         stop_event=stop_event)
        robot_interface.start()
        logger.info(f"Interface robot démarrée sur {port}")

        # Attendre que l'interface se connecte
        time.sleep(1)

        # Boucle principale
        print("\nEntrez 'exit' ou 'q' pour quitter.")
        print("Entrez 'help' pour l'aide.")

        while running:
            print("\n--- Test de communication série ---")
            print("0. Commande personnalisée")

            for i, (desc, cmd) in enumerate(COMMANDES_PREDEFINIES, 1):
                print(f"{i}. {desc} ({cmd})")

            print("h. Aide")
            print("p. Afficher position actuelle")
            print("q. Quitter")

            choice = input("\nChoix: ").strip().lower()

            if choice in ('q', 'quit', 'exit'):
                break

            elif choice == '0':
                cmd = input("Commande: ").strip()
                if cmd:
                    logger.info(f"Envoi: {cmd}")
                    robot_interface.send_command(cmd)
                    time.sleep(0.5)

            elif choice == 'h' or choice == 'help':
                print("\n--- Aide ---")
                print("V<valeur>: Vitesse (ex: V100)")
                print("R<valeur>: Rotation en degrés (ex: R90)")
                print("S: Stop")
                print("G<X,Y>: Aller à position (ex: GX100,Y200)")
                print("P: Demander position")
                print("SX<val>,Y<val>,Z<val>: Définir position")
                print("SRV<servo_id>:<pos>:<vitesse>: Contrôler servo")

            elif choice == 'p':
                afficher_position()

            elif choice.isdigit() and 1 <= int(choice) <= len(COMMANDES_PREDEFINIES):
                idx = int(choice) - 1
                desc, cmd = COMMANDES_PREDEFINIES[idx]
                logger.info(f"Envoi: {cmd} ({desc})")
                robot_interface.send_command(cmd)
                time.sleep(0.5)

            else:
                print("Option non valide")

            # Afficher position après chaque commande
            afficher_position()

    except Exception as e:
        logger.error(f"Erreur: {e}")

    finally:
        # Arrêter proprement
        logger.info("Arrêt de l'interface...")
        stop_event.set()

        if 'robot_interface' in locals() and robot_interface.is_alive():
            robot_interface.join(timeout=2)

        logger.info("Test terminé!")


if __name__ == "__main__":
    main()