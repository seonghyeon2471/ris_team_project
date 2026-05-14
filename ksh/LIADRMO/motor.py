import serial
import time
from config import *

class MotorController:

    def __init__(self):

        self.ser = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=1)
        time.sleep(2)

    def send(self, v, w):

        msg = f"{v:.3f},{w:.3f}\n"
        self.ser.write(msg.encode())

    def stop(self):
        self.send(0.0, 0.0)
