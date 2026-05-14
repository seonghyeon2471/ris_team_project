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

        while self.ser.in_waiting >= 47:

            try:

                # 헤더 찾기
                b = self.ser.read(1)

                if b[0] != 0x54:
                    continue

                data = b + self.ser.read(46)

                if len(data) != 47:
                    continue

                # 패킷 파싱
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

                    # -180 ~ 180 변환
                    if angle > 180:
                        angle -= 360

                    # 범위 제한
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

        return points
