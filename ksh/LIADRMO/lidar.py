from rplidar import RPLidar, RPLidarException
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

    def _make_iterator(self):
        """이터레이터 재생성 (예외 복구용)."""
        try:
            self.lidar.stop()
        except Exception:
            pass
        time.sleep(0.5)
        self.iterator = self.lidar.iter_scans(max_buf_meas=2000)

    def read_scan(self):
        """스캔 1회 읽기. 패킷 오류 시 자동 복구 후 빈 리스트 반환."""
        try:
            raw_scan = next(self.iterator)
        except RPLidarException as e:
            print(f"[LIDAR] 패킷 오류 복구 중: {e}")
            self._make_iterator()
            return []
        except StopIteration:
            print("[LIDAR] 이터레이터 종료 → 재생성")
            self._make_iterator()
            return []

        points = []

        for (_, angle, distance) in raw_scan:

            # 0~360 → -180~180
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

            points.append((angle, distance))

        return points

    def stop(self):
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception as e:
            print(f"[LIDAR] stop 중 오류 (무시): {e}")
