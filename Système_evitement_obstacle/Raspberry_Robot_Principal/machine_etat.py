import serial
import time
import math

ser_arduino1 = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Arduino1 = celle qui gère les moteurs/odométrie

# parametres physiques du robot
L = 0.245 / 2 # écart entre les 2 roues
r = 0.065 / 2 # rayon d'une roue

class Robot:
    def __init__(self):
        self.state = "DEPART"
        self.x_goal = [0, 0, 2, 0]
        self.y_goal = [0, 1, 1, 0]
        self.i_goal = 0
        self.K1 = 300 # PID
        self.eps = 0.1 # small threshold for determining when the robot has reached a goal

    def send_speed_arduino1(self, v, w):
        phiR = (v + w * L) / r
        phiL = (v - w * L) / r
        ser_arduino1.write(f"{int(phiR)},{int(phiL)}\n".encode())

    def update_state(self, x, y, theta):
        if self.state == "DEPART":
            self.send_speed_arduino1(0, 0)
            self.state = "NAVIGATION"

        elif self.state == "NAVIGATION":
            x_goal, y_goal = self.x_goal[self.i_goal], self.y_goal[self.i_goal]
            thetaref = math.atan2(y_goal - y, x_goal - x)
            w = self.K1 * math.atan2(math.sin(thetaref - theta), math.cos(thetaref - theta))
            v = 10 if abs(thetaref - theta) < self.eps else 0

            self.send_speed_arduino1(v, w)

            if abs(x - x_goal) < self.eps and abs(y - y_goal) < self.eps:
                self.i_goal += 1
                if self.i_goal >= len(self.x_goal):
                    self.state = "STOP"

        elif self.state == "STOP":
            self.send_speed_arduino1(0, 0)

robot = Robot() #initialise __init__

while True:
    try:
        #Demande les donnees x,y,theta, puis les met à jour en appelant update_state
        ser_arduino1.write(b"\n")  # Demander les données
        data = ser_arduino1.readline().decode().strip()
        if data:
            x, y, theta = map(float, data.split(',')) #ex : convertit "1.0, 1.2, 3.6" en [1.0, 1.2, 3.6]
            robot.update_state(x, y, theta)
        time.sleep(0.1)

    except KeyboardInterrupt:
        ser_arduino1.close()
        break
