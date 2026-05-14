from rplidar import RPLidar
from config import *

class SimpleLidar:

    def __init__(self):

        self.lidar = RPLidar(
            LIDAR_PORT,
            baudrate=LIDAR_BAUD
        )

        self.iterator = self.lidar.iter_scans()

    def read_scan(self):

        scan = next(self.iterator)

        points = []

        for (_, angle, distance) in scan:

            # angle 변환
            if angle > 180:
                angle -= 360

            # 전방만 사용
            if angle < ANGLE_MIN:
                continue

            if angle > ANGLE_MAX:
                continue

            # 거리 제한
            if distance < MIN_LIDAR_DIST:
                continue

            if distance > MAX_LIDAR_DIST:
                continue

            points.append(
                (angle, distance)
            )

        return points

    def stop(self):

        self.lidar.stop()

        self.lidar.disconnect()
