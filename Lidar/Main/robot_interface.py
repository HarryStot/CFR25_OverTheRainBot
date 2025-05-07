# robot_interface.py
import time
import threading

from position_manager import position_manager
import logging
import serial
import re

logger = logging.getLogger(__name__)


class RobotInterface(threading.Thread):
    """Update robot position and target from serial data"""

    def __init__(self, serial_port='COM3', baud_rate=9600, stop_event=None):
        super().__init__()
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.stop_event = stop_event or threading.Event()
        self.daemon = True
        self.ser = None
        self.connected = False
        self.position_received = threading.Event()

    def run(self):
        retry_count = 0
        max_retries = 5

        while not self.stop_event.is_set() and retry_count < max_retries:
            try:
                logger.info(f"Connexion au port {self.serial_port} à {self.baud_rate} bauds")
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                time.sleep(0.1)  # Temps d'attente pour l'ouverture du port série

                if self.ser.isOpen():
                    logger.info(f"{self.serial_port} connecté!")

                    # Nettoyer les tampons
                    self.ser.reset_input_buffer()

                    # Demander la position initiale
                    logger.info("Demande de position initiale à l'Arduino")
                    self.ser.write(b"P\n")

                    self.connected = True
                    retry_count = 0  # Réinitialisation du compteur d'essais

                    logger.info("Début de lecture du port série...")
                    while not self.stop_event.is_set():
                        # Envoyer une demande de position
                        self.ser.write(b"P\n")

                        # Attendre que des données soient disponibles
                        timeout_counter = 0
                        while self.ser.inWaiting() == 0 and timeout_counter < 50:
                            time.sleep(0.01)
                            timeout_counter += 1

                        # Lire les données si disponibles
                        if self.ser.inWaiting() > 0:
                            try:
                                line = self.ser.readline().decode('utf-8').strip()
                                if line:
                                    logger.debug(f"Reçu: {line}")
                                    self.process_line(line)
                                self.ser.flushInput()  # Vider le tampon après lecture
                            except UnicodeDecodeError:
                                logger.warning("Données invalides reçues, ignorées")
                                self.ser.flushInput()

                        time.sleep(0.1)

            except serial.SerialException as e:
                logger.error(f"Erreur série: {e}")
                retry_count += 1
                logger.info(f"Nouvelle tentative ({retry_count}/{max_retries})...")
                time.sleep(2)

            except Exception as e:
                logger.error(f"Erreur dans RobotInterface: {e}")
                break

            finally:
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                        logger.info("Port série fermé")
                    except Exception as e:
                        logger.error(f"Erreur lors de la fermeture du port série: {e}")

                self.connected = False

        if retry_count >= max_retries:
            logger.error(f"Échec de connexion après {max_retries} tentatives")

    def process_line(self, line):
        """Process a line received from the Arduino"""
        if line.startswith("POS,"):
            self.position_received.set()

            # Utiliser des expressions régulières pour extraire les valeurs de position
            x_match = re.search(r'X:([-+]?\d*\.?\d+)', line)
            y_match = re.search(r'Y:([-+]?\d*\.?\d+)', line)
            z_match = re.search(r'Z:([-+]?\d*\.?\d+)', line)

            x = y = z = None

            if x_match:
                x = float(x_match.group(1))
            if y_match:
                y = float(y_match.group(1))
            if z_match:
                z = float(z_match.group(1))

            if x is not None and y is not None and z is not None:
                position_manager.set_position(x, y, z)
                logger.info(f"Position mise à jour: X={x}, Y={y}, Z={z}")
            else:
                logger.warning(f"Données de position incomplètes dans: {line}")

    def process_line(self, line):
        """Process a line received from the Arduino"""
        if line.startswith("POS,"):
            self.position_received.set()

            # Use regular expressions to extract position values
            x_match = re.search(r'X:([-+]?\d*\.?\d+)', line)
            y_match = re.search(r'Y:([-+]?\d*\.?\d+)', line)
            z_match = re.search(r'Z:([-+]?\d*\.?\d+)', line)

            x = y = z = None

            if x_match:
                x = float(x_match.group(1))
            if y_match:
                y = float(y_match.group(1))
            if z_match:
                z = float(z_match.group(1))

            if x is not None and y is not None and z is not None:
                position_manager.set_position(x, y, z)
                logger.info(f"Updated position: X={x}, Y={y}, Z={z}")
            else:
                logger.warning(f"Incomplete position data in: {line}")

        elif line.startswith("Target"):
            # Extract target information if provided
            target_x_match = re.search(r'X:(\d+\.?\d*)', line)
            target_y_match = re.search(r'Y:(\d+\.?\d*)', line)

            if target_x_match and target_y_match:
                target_x = float(target_x_match.group(1))
                target_y = float(target_y_match.group(1))
                position_manager.set_target(target_x, target_y)
                logger.info(f"Updated target: X={target_x}, Y={target_y}")

        elif line.startswith("Velocity"):
            # Extract velocity information
            vel_match = re.search(r'to: (\d+)', line)
            if vel_match:
                velocity = int(vel_match.group(1))
                position_manager.set_velocity(velocity)
                logger.info(f"Updated velocity: {velocity}")

        elif "unknown command" in line.lower():
            logger.warning(f"Arduino reported unknown command: {line}")

        elif "error" in line.lower():
            logger.error(f"Arduino reported error: {line}")

    def send_command(self, command):
        """Send a movement command to the Arduino"""
        if self.connected:
            try:
                if self.ser.is_open:
                    print(f"Envoi de commande: {command}")
                    logger.info(f"Envoi de commande: {command}")
                    self.ser.write(f"{command}\n".encode())

                    # Attendre la réponse
                    timeout_counter = 0
                    while self.ser.inWaiting() == 0 and timeout_counter < 50:
                        time.sleep(0.01)
                        timeout_counter += 1

                    if self.ser.inWaiting() > 0:
                        answer = self.ser.readline()
                        logger.info(f"Réponse: {answer}")
                        self.ser.flushInput()
                else:
                    logger.warning("Le port série n'est pas ouvert, impossible d'envoyer la commande")
            except serial.SerialException as e:
                logger.error(f"Erreur lors de l'envoi de la commande: {e}")
        else:
            logger.warning("Non connecté à l'Arduino, impossible d'envoyer la commande")