from rplidar import RPLidar
import time
from config import *

class SimpleLidar:

    def __init__(self):

        self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)

        # 🔥 완전 안정화 시퀀스
        self.lidar.stop_motor()
        time.sleep(1)

        self.lidar.disconnect()
        time.sleep(1)

        # 재연결 (중요)
        self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)

        time.sleep(2)
        self.lidar.start_motor()
        time.sleep(2)

    # =========================
    # SAFE SCAN (핵심)
    # =========================
    def read_scan(self):

        try:
            scan = self.lidar.iter_measures()

            points = []

            for _, angle, dist in scan:

                if angle > 180:
                    angle -= 360

                if not (ANGLE_MIN <= angle <= ANGLE_MAX):
                    continue

                if not (MIN_LIDAR_DIST <= dist <= MAX_LIDAR_DIST):
                    continue

                points.append((angle, dist))

                # 🔥 과부하 방지
                if len(points) > 200:
                    break

            return points

        except Exception as e:

            print("LIDAR ERROR RESET:", e)

            # 🔥 강제 복구
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass

            time.sleep(1)

            # 재초기화
            self.__init__()

            return []

    def stop(self):
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except:
            pass
