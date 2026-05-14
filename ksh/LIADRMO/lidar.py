import serial
import math

from config import *

class SimpleLidar:

    def __init__(self):

        self.ser = serial.Serial(
            LIDAR_PORT,
            LIDAR_BAUD,
            timeout=0.1
        )

    def read_scan(self):

        points = []

        while True:

            # 헤더 탐색
            b = self.ser.read(1)

            if len(b) == 0:
                break

            if b[0] != 0x54:
                continue

            second = self.ser.read(1)

            if len(second) == 0:
                continue

            # packet length 확인
            if second[0] != 0x2C:
                continue

            rest = self.ser.read(45)

            if len(rest) != 45:
                continue

            data = b + second + rest

            try:

                start_angle = (
                    data[4] |
                    (data[5] << 8)
                ) / 100.0

                end_angle = (
                    data[42] |
                    (data[43] << 8)
                ) / 100.0

                if end_angle < start_angle:
                    end_angle += 360

                angle_step = (
                    end_angle - start_angle
                ) / 11.0

                for i in range(12):

                    offset = 6 + i * 3

                    dist = (
                        data[offset] |
                        (data[offset + 1] << 8)
                    )

                    angle = (
                        start_angle +
                        angle_step * i
                    )

                    if angle > 180:
                        angle -= 360

                    if angle < ANGLE_MIN:
                        continue

                    if angle > ANGLE_MAX:
                        continue

                    if dist < MIN_LIDAR_DIST:
                        continue

                    if dist > MAX_LIDAR_DIST:
                        continue

                    points.append(
                        (angle, dist)
                    )

            except:
                pass

            # 어느 정도 모이면 반환
            if len(points) > 80:
                break

        return points
