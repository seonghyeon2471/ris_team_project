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

        self.iterator = self.lidar.iter_scans(max_buf_meas=500)

    def read_scan(self):

        scan = next(self.iterator)

        points = []

        for (_, angle, dist) in scan:

            if angle > 180:
                angle -= 360

            if angle < ANGLE_MIN or angle > ANGLE_MAX:
                continue

            if dist < MIN_LIDAR_DIST or dist > MAX_LIDAR_DIST:
                continue

            points.append((angle, dist))

        return points

    def stop(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
