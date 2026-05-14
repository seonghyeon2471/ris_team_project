from rplidar import RPLidar
import time
from config import *

class SimpleLidar:

    def __init__(self):

        self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)

        self.lidar.stop()
        time.sleep(1)

        self.lidar.reset()
        time.sleep(2)

        self.lidar.start_motor()
        time.sleep(2)

        self.iterator = self.lidar.iter_scans(max_buf_meas=300)

    def read_scan(self):

        scan = None

        # 최신 데이터 확보
        for _ in range(2):
            scan = next(self.iterator)

        if not scan:
            return []

        points = []

        for _, angle, dist in scan:

            if angle > 180:
                angle -= 360

            if not (ANGLE_MIN <= angle <= ANGLE_MAX):
                continue

            if not (MIN_LIDAR_DIST <= dist <= MAX_LIDAR_DIST):
                continue

            points.append((angle, dist))

            if len(points) > 200:
                break

        return points

    def stop(self):
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except:
            pass
