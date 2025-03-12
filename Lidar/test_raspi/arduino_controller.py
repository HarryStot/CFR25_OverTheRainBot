import serial
import threading
import time


class ArduinoMotionController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.lock = threading.Lock()
        time.sleep(2)  # Wait for Arduino to reset

    def send_position_command(self, x, y, z):
        """Send position command to the motion Arduino"""
        command = f"X{x:.3f}Y{y:.3f}Z{z:.3f}\n" # TODO Adjust format as needed (3 decimal places)
        with self.lock:
            self.serial_port.write(command.encode())

    def read_feedback(self):
        """Read feedback from the motion Arduino"""
        if self.serial_port.in_waiting > 0:
            with self.lock:
                response = self.serial_port.readline().decode().strip()
                return response
        return None


class ArduinoServoController:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.lock = threading.Lock()
        time.sleep(2)  # Wait for Arduino to reset

    def send_motor_command(self, motor_commands):
        """Send commands to servo/stepper Arduino
        motor_commands: dict with motor_id as key and pwm_value as value
        """
        command = ""
        for motor_id, pwm_value in motor_commands.items():
            command += f"{motor_id}:{pwm_value};"
        command += "\n"

        with self.lock:
            self.serial_port.write(command.encode())

    def read_feedback(self):
        """Read feedback from the servo Arduino"""
        if self.serial_port.in_waiting > 0:
            with self.lock:
                response = self.serial_port.readline().decode().strip()
                return response
        return None