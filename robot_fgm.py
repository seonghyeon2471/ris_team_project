import serial
import time
import numpy as np
import math
from scipy.interpolate import CubicSpline

# =========================================
# SERIAL & LIDAR
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# 로봇 스펙
# =========================================
ROBOT_WIDTH = 0.20
LIDAR_OFFSET_FRONT = 0.025

MAX_SPEED = 0.16
STEERING_GAIN = 5.5        # ← 조향 감도 증가
INFLATION_MARGIN = 0.05

print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ LIDAR SCAN START")


class SplineNavigator:
    def __init__(self):
        self.prev_steering = 0.0

    def safe_asin(self, x):
        return math.asin(np.clip(x, -0.999, 0.999))

    def inflate_obstacles(self, angles_deg, ranges):
        proc = ranges.copy()
        half_width = ROBOT_WIDTH / 2 + INFLATION_MARGIN
        for i in range(1, len(ranges)-1):
            if ranges[i] < 1.5 or abs(ranges[i] - ranges[i-1]) > 0.15:
                d_near = max(min(ranges[i], ranges[i-1]), 0.08)
                delta = self.safe_asin(half_width / d_near)
                mask = int(math.ceil(delta / math.radians(1.0)))
                for k in range(-mask, mask + 1):
                    if 0 <= i + k < len(proc):
                        proc[i + k] = 0.0
        return proc

    def generate_spline_target(self, angles_deg, ranges):
        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        proc_ranges = self.inflate_obstacles(angles, ranges)
        free_mask = proc_ranges > 0.08

        if np.sum(free_mask) < 20:
            return 0.0

        free_angles = angles[free_mask]
        free_ranges = proc_ranges[free_mask]

        x = free_ranges * np.cos(np.radians(free_angles))
        y = free_ranges * np.sin(np.radians(free_angles))

        # ==================== 조향 강화 ====================
        if len(x) > 15:
            sort_idx = np.argsort(x)[-30:]   # 앞쪽 점들
            x_s = x[sort_idx]
            y_s = y[sort_idx]

            try:
                spline = CubicSpline(x_s, y_s, bc_type='natural')
                target_y = spline(1.0)                    # 1m 앞
                target_angle = math.degrees(math.atan2(target_y, 1.0))

                # 좌우 bias를 더 강하게 반영
                left_bias = np.mean(y_s[x_s < 0.3]) if np.any(x_s < 0.3) else 0
                right_bias = np.mean(y_s[x_s > 0.3]) if np.any(x_s > 0.3) else 0
                bias = (left_bias - right_bias) * 25

                target_angle += bias
                return target_angle

            except:
                pass

        # fallback (좌우 gap 차이)
        left = free_angles < 0
        right = free_angles > 0
        if np.any(left) and np.any(right):
            left_mean = np.mean(free_angles[left])
            right_mean = np.mean(free_angles[right])
            return (left_mean + right_mean) / 2 * 1.8   # 더 민감하게

        return np.mean(free_angles)

    def process(self, angles_deg, ranges):
        target_angle = self.generate_spline_target(angles_deg, ranges)

        steering = STEERING_GAIN * target_angle
        steering = np.clip(steering, -0.70, 0.70)

        front_clear = np.max(ranges) if len(ranges) > 0 else 1.0
        v = np.clip(front_clear * 0.45, 0.18, MAX_SPEED)

        d_min = np.min(ranges) if len(ranges) > 0 else 1.0
        if d_min < 0.40:
            v *= 0.65

        return steering, v


navigator = SplineNavigator()

print("🚀 조향 강화 + 스플라인 버전 시작")

buffer = bytearray()

try:
    while True:
        data = lidar_ser.read(512)
        if data:
            buffer.extend(data)

        points = []
        i = 0
        while i + 5 <= len(buffer):
            try:
                byte1 = buffer[i]
                quality = byte1 >> 2
                angle_q6 = ((buffer[i+1] << 7) | (byte1 >> 1)) & 0x7FFF
                angle_deg = (angle_q6 / 64.0) - 180.0
                dist_mm = ((buffer[i+3] << 8) | buffer[i+2]) * 4

                if quality >= 5 and 60 < dist_mm < 12000:
                    points.append((angle_deg, dist_mm / 1000.0))
            except:
                pass
            i += 5

        if len(points) >= 60:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])

            front_mask = (angles > -90) & (angles < 90)
            if np.any(front_mask):
                steering, v = navigator.process(angles[front_mask], ranges[front_mask])
                w = steering * 4.5

                cmd = f"{v:.3f},{w:.3f}\n"
                arduino_ser.write(cmd.encode('utf-8'))

                print(f"Steer: {steering:+6.1f}° | v:{v:.3f} | d_min:{np.min(ranges[front_mask]):.2f}m")

            buffer = buffer[i:]

        time.sleep(0.008)

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
