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
print("✅ RPLIDAR + Cubic Spline 버전 시작")

# =========================================
# 파라미터
# =========================================
GLOBAL_GOAL_DISTANCE = 10.0   # 10m 앞
MAX_SPEED = 0.16
STEERING_GAIN = 3.6
INFLATION_MARGIN = 0.10       # 10cm inflation
LOOKAHEAD = 1.4               # 스플라인 lookahead

class SplineKalmanNavigator:
    def __init__(self):
        self.prev_steering = 0.0
        self.kf_steering = self.KalmanFilter(0.08, 1.0)

    class KalmanFilter:
        def __init__(self, Q=0.08, R=1.0):
            self.x = 0.0
            self.P = 1.0
            self.Q = Q
            self.R = R

        def update(self, z):
            self.P += self.Q
            K = self.P / (self.P + self.R)
            self.x += K * (z - self.x)
            self.P = (1 - K) * self.P
            return self.x

    def inflate(self, angles, ranges):
        proc = ranges.copy()
        half = ROBOT_WIDTH / 2 + INFLATION_MARGIN
        for i in range(1, len(ranges)-1):
            if ranges[i] < 2.0:
                d_near = max(min(ranges[i], ranges[i-1]), 0.1)
                delta = math.asin(half / d_near)
                mask = int(math.ceil(delta / math.radians(1.0)))
                for k in range(-mask, mask + 1):
                    if 0 <= i + k < len(proc):
                        proc[i + k] = 0.0
        return proc

    def get_spline_target(self, angles, ranges):
        """연속 중앙선 스플라인"""
        proc = self.inflate(angles, ranges)
        free = proc > 0.15

        if np.sum(free) < 35:
            return 0.0

        free_angles = angles[free]
        free_ranges = proc[free]

        x = free_ranges * np.cos(np.radians(free_angles))
        y = free_ranges * np.sin(np.radians(free_angles))

        if len(x) > 25:
            sort_idx = np.argsort(x)[-32:]   # 앞쪽 점들
            x_s = x[sort_idx]
            y_s = y[sort_idx]

            try:
                spline = CubicSpline(x_s, y_s, bc_type='natural')
                target_y = spline(LOOKAHEAD)
                target_angle = math.degrees(math.atan2(target_y, LOOKAHEAD))
                return target_angle
            except:
                pass

        return np.average(free_angles, weights=free_ranges)

    def process(self, scan):
        angles = np.array([m[1] for m in scan])
        ranges = np.array([m[2] for m in scan]) / 1000.0   # m

        mask = (angles > -90) & (angles < 90)
        target_angle = self.get_spline_target(angles[mask], ranges[mask])

        # Kalman + Smoothing
        steering = self.kf_steering.update(target_angle)
        steering = STEERING_GAIN * steering
        steering = 0.78 * steering + 0.22 * self.prev_steering
        self.prev_steering = steering

        steering = np.clip(steering, -0.68, 0.68)

        v = np.clip(np.max(ranges) * 0.48, 0.18, MAX_SPEED)
        if np.min(ranges) < 0.40:
            v *= 0.6

        return steering, v


navigator = SplineKalmanNavigator()

print("🚀 10m 전역 + Cubic Spline + Kalman Filter 시작!")

try:
    for scan in lidar.iter_scans(max_buf_meas=2000, min_len=120):
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
