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

        # 🔥 iterator 1개만 사용 (중요)
        self.iterator = self.lidar.iter_scans(max_buf_meas=300)

    # =========================
    # SAFE SCAN (핵심)
    # =========================
    def read_scan(self):

        scan = None

        # 🔥 최신 데이터 1~2개만 소비 (buffer flush 효과)
        for _ in range(2):
            scan = next(self.iterator)

        if not scan:
            return []

        points = []

        for (_, angle, dist) in scan:

            # 0~360 → -180~180
            if angle > 180:
                angle -= 360

            # angle filter
            if not (ANGLE_MIN <= angle <= ANGLE_MAX):
                continue

            # distance filter
            if not (MIN_LIDAR_DIST <= dist <= MAX_LIDAR_DIST):
                continue

            points.append((angle, dist))

            # 🔥 안전: 너무 많으면 컷 (CPU 보호)
            if len(points) > 200:
                break

        return points

    # =========================
    # STOP
    # =========================
    def stop(self):

        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except:
            pass
