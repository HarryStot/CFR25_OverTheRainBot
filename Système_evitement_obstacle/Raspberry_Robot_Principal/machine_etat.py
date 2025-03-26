import serial
import time
import math

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

class Robot:
    def __init__(self):
        self.state = "DEPART"
        self.x_goal = [0, 0, 2, 0]
        self.y_goal = [0, 1, 1, 0]
        self.i_goal = 0
        self.K1 = 300
        self.eps = 0.1

    def send_speed(self, v, w, L, r):
        phiR = (v + w * L) / r
        phiL = (v - w * L) / r
        ser.write(f"{int(phiR)},{int(phiL)}\n".encode())

    def update_state(self, x, y, theta):
        if self.state == "DEPART":
            self.send_speed(0, 0, 0.245 / 2, 0.065 / 2)
            self.state = "NAVIGATION"

        elif self.state == "NAVIGATION":
            x_goal, y_goal = self.x_goal[self.i_goal], self.y_goal[self.i_goal]
            thetaref = math.atan2(y_goal - y, x_goal - x)
            w = self.K1 * math.atan2(math.sin(thetaref - theta), math.cos(thetaref - theta))
            v = 10 if abs(thetaref - theta) < self.eps else 0

            self.send_speed(v, w, 0.245 / 2, 0.065 / 2)

            if abs(x - x_goal) < self.eps and abs(y - y_goal) < self.eps:
                self.i_goal += 1
                if self.i_goal >= len(self.x_goal):
                    self.state = "STOP"

        elif self.state == "STOP":
            self.send_speed(0, 0, 0.245 / 2, 0.065 / 2)

robot = Robot()

while True:
    try:
        ser.write(b"\n")  # Demander les données
        data = ser.readline().decode().strip()
        if data:
            x, y, theta = map(float, data.split(','))
            robot.update_state(x, y, theta)
        time.sleep(0.1)

    except KeyboardInterrupt:
        ser.close()
        break
