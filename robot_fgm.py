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
# 파라미터
# =========================================
MAX_SPEED = 0.16
STEERING_GAIN = 3.5
INFLATION_MARGIN = 0.10   # 10cm inflation
LOOKAHEAD = 1.3           # 1.3m 앞 lookahead

print("🚀 연속 중앙선 스플라인 버전 시작")


class ContinuousSplineNavigator:
    def __init__(self):
        self.prev_steering = 0.0

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

    def find_continuous_centerline(self, angles, ranges):
        """연속된 중앙점들을 찾아 스플라인 생성"""
        proc_ranges = self.inflate(angles, ranges)
        free_mask = proc_ranges > 0.12

        if np.sum(free_mask) < 40:
            return 0.0

        free_angles = angles[free_mask]
        free_ranges = proc_ranges[free_mask]

        # Cartesian 변환
        x = free_ranges * np.cos(np.radians(free_angles))
        y = free_ranges * np.sin(np.radians(free_angles))

        # x축 기준으로 정렬 후 여러 점 샘플링
        sort_idx = np.argsort(x)
        x_sorted = x[sort_idx]
        y_sorted = y[sort_idx]

        # 앞쪽으로 25~30개 점을 골라 spline
        if len(x_sorted) > 25:
            sample_idx = np.linspace(0, len(x_sorted)-1, 28, dtype=int)
            x_s = x_sorted[sample_idx]
            y_s = y_sorted[sample_idx]

            try:
                spline = CubicSpline(x_s, y_s, bc_type='natural')
                # lookahead 지점의 y값
                target_y = spline(LOOKAHEAD)
                target_angle = math.degrees(math.atan2(target_y, LOOKAHEAD))
                return target_angle
            except:
                pass

        # fallback
        return np.average(free_angles, weights=free_ranges**1.6)


    def process(self, scan):
        angles = np.array([m[1] for m in scan])
        ranges = np.array([m[2] for m in scan]) / 1000.0   # mm → m

        mask = (angles > -90) & (angles < 90)
        target_angle = self.find_continuous_centerline(angles[mask], ranges[mask])

        # smoothing
        steering = STEERING_GAIN * target_angle
        steering = 0.78 * steering + 0.22 * self.prev_steering
        self.prev_steering = steering

        steering = np.clip(steering, -0.68, 0.68)

        front_clear = np.max(ranges) if len(ranges) > 0 else 1.0
        v = np.clip(front_clear * 0.45, 0.18, MAX_SPEED)

        return steering, v


navigator = ContinuousSplineNavigator()

print("🚀 연속 중앙선 스플라인 + 10m 목표 방향 시작!")

try:
    for scan in lidar.iter_scans(max_buf_meas=1800, min_len=120):
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
