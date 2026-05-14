from rplidar import RPLidar
from config import *

class SimpleLidar:

    def __init__(self):

        self.lidar = RPLidar(
            LIDAR_PORT,
            baudrate=LIDAR_BAUD
        )

        # 모터 시작
        self.lidar.start_motor()

        self.iterator = self.lidar.iter_scans()

    def read_scan(self):

        scan = next(self.iterator)

        points = []

        for (_, angle, distance) in scan:

            if angle > 180:
                angle -= 360

            if angle < ANGLE_MIN:
                continue

            if angle > ANGLE_MAX:
                continue

            if distance < MIN_LIDAR_DIST:
                continue

            if distance > MAX_LIDAR_DIST:
                continue

            points.append((angle, distance))

        return points

    def stop(self):

        self.lidar.stop()

        self.lidar.stop_motor()

        self.lidar.disconnect()
