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

        # 내부 상태 초기화
        self.lidar.stop()

        time.sleep(1)

        self.lidar.reset()

        time.sleep(2)

        # 모터 시작
        self.lidar.start_motor()

        time.sleep(2)

        self.iterator = self.lidar.iter_scans(
            max_buf_meas=500
        )

    def read_scan(self):

        scan = next(self.iterator)

        points = []

        for (_, angle, distance) in scan:

            # 0~360 -> -180~180
            if angle > 180:
                angle -= 360

            # 전방만 사용
            if angle < ANGLE_MIN:
                continue

            if angle > ANGLE_MAX:
                continue

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
