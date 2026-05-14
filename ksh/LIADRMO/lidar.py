from rplidar import RPLidar
import time

from config import *

class SimpleLidar:

    def __init__(self):

        self.lidar = RPLidar(
            LIDAR_PORT,
            baudrate=LIDAR_BAUD,
            timeout=3
        )

        # 안정화
        self.lidar.stop()

        time.sleep(1)

        self.lidar.reset()

        time.sleep(2)

        self.lidar.start_motor()

        time.sleep(2)

        self.iterator = self.lidar.iter_scans(
            max_buf_meas=5000
        )

    def read_scan(self):

        raw_scan = next(self.iterator)

        points = []

        for (_, angle, distance) in raw_scan:

            # 0~360 -> -180~180
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

        self.lidar.stop_motor()

        self.lidar.disconnect()
