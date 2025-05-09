import serial
import time
import math
import threading

ser_arduino1 = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Arduino1 = gestion des moteurs/odométrie
ser_arduino2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Arduino2 = ultrasons/servo-moteurs

# paramètres physiques du robot
L = 0.245 / 2  # écart entre les 2 roues
r = 0.065 / 2  # rayon d'une roue
beta = 45 * 2*math.pi/360 # angle capteur ultrason/sol

# Variables initialisées pour les rendre globales
x, y, theta = 0.0, 0.0, 0.0

# Coordonnées cibles
goals = [(0, 0, 0), (0, 1, 0), (2, 1, 0), (0, 0, 0)]

class Robot:
    def __init__(self):
        self.state = "DEPART"
        self.x_goal = [0, 0, 2, 0]
        self.y_goal = [0, 1, 1, 0]
        self.i_goal = 0
        self.K1 = 2  # Gain
        self.eps = 0.1  # seuil pour atteindre le but

    def send_goal_arduino1(self, x_goal, y_goal, theta_goal):
        ser_arduino1.write(f"{x_goal},{y_goal},{theta_goal}\n".encode())

    def update_state(self, x, y, theta):
        if self.state == "DEPART":
            self.send_speed_arduino1(0, 0)
            self.state = "NAVIGATION"

        elif self.state == "NAVIGATION":
            if self.i_goal < len(goals) - 1:
                self.i_goal += 1
                self.send_goal(*goals[self.i_goal])

        elif self.state == "STOP":
            self.send_speed_arduino1(0, 0)

robot = Robot()

#-----ULTRASONS-----
def thread_serial_arduino2():
    while True:
        try:
            data = ser_arduino2.readline().decode().strip()
            if data.startswith("us"):
                prefix, sensor_id, mesure = data.split(',')
                sensor_id = int(sensor_id)
                mesure = float(mesure)
                # Todo : Rajouter la différence entre capteurs
                x_obstacle = math.cos(theta)*mesure/math.sin(beta) 
                y_obstacle = math.sin(theta)*mesure/math.sin(beta)
                # Todo : rajouter l'obstacle dans la liste d'obstacle
            time.sleep(0.1)
        except Exception as e:
            print(f"Erreur dans le thread série: {e}")
            break
thread_ultrason = threading.Thread(target=thread_serial_arduino2)
thread_ultrason.daemon = True
thread_ultrason.start()
#-----------------

while True:
    try:
        ser_arduino1.write(b"\n")  # Demander les données
        data = ser_arduino1.readline().decode().strip()
        if data:
            x, y, theta = map(float, data.split(','))
            robot.update_state(x, y, theta)
        time.sleep(0.1)

    except KeyboardInterrupt:
        ser_arduino1.close()
        ser_arduino2.close()
        break
