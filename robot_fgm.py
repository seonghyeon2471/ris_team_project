import serial
import time
import numpy as np
import math
from scipy.interpolate import CubicSpline
from rplidar import RPLidar

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.05)

# =========================================
# RPLIDAR
# =========================================
lidar = RPLidar('/dev/ttyUSB0', baudrate=460800)
lidar.start_motor()
print("✅ RPLIDAR 연결 완료")

# =========================================
# 미션 & 튜닝
# =========================================
GLOBAL_GOAL_DISTANCE = 10.0   # 10m 앞
MAX_SPEED = 0.16
STEERING_GAIN = 3.8
INFLATION_MARGIN = 0.12       # 장애물 inflation 강화 (12cm)
LOOKAHEAD = 1.2               # 스플라인 lookahead

print("🚀 10m 전역 목표 + 스플라인 지역경로 시작")


class HybridSplineNavigator:
    def __init__(self):
        self.prev_steering = 0.0

    def inflate_obstacles(self, angles, ranges):
        """장애물 주변 강한 inflation"""
        proc = ranges.copy()
        for i in range(1, len(ranges)-1):
            if ranges[i] < 2.0:
                d_near = max(min(ranges[i], ranges[i-1]), 0.1)
                delta = math.asin(INFLATION_MARGIN / d_near)
                mask = int(math.ceil(delta / math.radians(1.0)))
                for k in range(-mask, mask + 1):
                    idx = i + k
                    if 0 <= idx < len(proc):
                        proc[idx] = 0.0
        return proc

    def find_farthest_centerline(self, angles, ranges):
        """가장 멀리 갈 수 있는 중앙선 찾기"""
        proc_ranges = self.inflate_obstacles(angles, ranges)
        free_mask = proc_ranges > 0.15

        if np.sum(free_mask) < 30:
            return 0.0

        free_angles = angles[free_mask]
        free_ranges = proc_ranges[free_mask]

        # 가장 먼 거리의 점들로 spline
        if len(free_ranges) > 25:
            # 먼 거리 상위 점들 선택
            far_idx = np.argsort(free_ranges)[-30:]
            x = free_ranges[far_idx] * np.cos(np.radians(free_angles[far_idx]))
            y = free_ranges[far_idx] * np.sin(np.radians(free_angles[far_idx]))

            sort_idx = np.argsort(x)
            x_s = x[sort_idx]
            y_s = y[sort_idx]

            try:
                spline = CubicSpline(x_s, y_s, bc_type='natural')
                target_y = spline(LOOKAHEAD)
                target_angle = math.degrees(math.atan2(target_y, LOOKAHEAD))
                return target_angle
            except:
                pass

        # fallback
        return np.average(free_angles, weights=free_ranges)

    def process(self, scan):
        angles = np.array([m[1] for m in scan])
        ranges = np.array([m[2] for m in scan]) / 1000.0   # mm → m

        # 전방 180도
        mask = (angles > -90) & (angles < 90)
        target_angle = self.find_farthest_centerline(angles[mask], ranges[mask])

        # smoothing
        steering = STEERING_GAIN * target_angle
        steering = 0.78 * steering + 0.22 * self.prev_steering
        self.prev_steering = steering

        steering = np.clip(steering, -0.70, 0.70)

        # 속도
        front_clear = np.max(ranges) if len(ranges) > 0 else 1.0
        v = np.clip(front_clear * 0.45, 0.18, MAX_SPEED)

        return steering, v


navigator = HybridSplineNavigator()

print("🚀 10m 전역 + 스플라인 farthest centerline 시작!")

try:
    for scan in lidar.iter_scans(max_buf_meas=1500, min_len=100):
        steering, v = navigator.process(scan)
        w = steering * 4.0

        cmd = f"{v:.3f},{w:.3f}\n"
        arduino_ser.write(cmd.encode('utf-8'))

        print(f"Steer: {steering:+6.1f}° | v:{v:.3f}")

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
    print("✅ 정리 완료")
