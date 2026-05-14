import serial
import math
from config import *

class SimpleLidar:

    def __init__(self):

        self.ser = serial.Serial(
            LIDAR_PORT,
            LIDAR_BAUD,
            timeout=1
        )

    def read_scan(self):

        points = []

        while self.ser.in_waiting > 2000:

            data = self.ser.read(47)

            if len(data) < 47:
                continue

            try:

                for i in range(12):

                    offset = 11 + i * 3

                    raw_dist = data[offset] | (data[offset + 1] << 8)
                    raw_angle = data[offset + 2]

                    dist = raw_dist
                    angle = raw_angle - 128

                    if angle < ANGLE_MIN:
                        continue

                    if angle > ANGLE_MAX:
                        continue

                    if dist < MIN_LIDAR_DIST:
                        continue

                    if dist > MAX_LIDAR_DIST:
                        continue

                    points.append((angle, dist))

            except:
                pass

        return points
