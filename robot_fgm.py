import serial
import time
import numpy as np
import math

# =========================================
# SERIAL & LIDAR
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# 로봇 스펙
# =========================================
ROBOT_WIDTH = 0.24
LIDAR_OFFSET_FRONT = 0.025

# =========================================
# 튜닝 파라미터 (조향 부드럽게 수정)
# =========================================
MAX_SPEED = 0.15          
STEERING_GAIN = 2.8       # ← 조향 감도 크게 낮춤 (부드럽게)
STEERING_REVERSE = False  # ← 방향 반전 (False로 변경)

SMOOTH_FACTOR = 0.78      # ← smoothing 더 강하게

# =========================================
# LIDAR START
# =========================================
print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ LIDAR SCAN START")

# =========================================
# Centerline Follower
# =========================================
class CenterlineFollower:
    def __init__(self):
        self.prev_target = 0.0

    def correct_to_center(self, angles_deg, ranges):
        angles_rad = np.radians(angles_deg)
        x = ranges * np.cos(angles_rad) - LIDAR_OFFSET_FRONT
        y = ranges * np.sin(angles_rad)
        dist = np.sqrt(x**2 + y**2 + 1e-8)
        ang = np.degrees(np.arctan2(y, x))
        return ang, dist

    def find_centerline(self, angles_deg, ranges):
        corr_angles, corr_ranges = self.correct_to_center(angles_deg, ranges)
        mask = (corr_angles > -70) & (corr_angles < 70)
        angles = corr_angles[mask]
        ranges = corr_ranges[mask]

        if len(angles) < 20:
            return 0.0, 1.0

        left = angles < 0
        right = angles > 0

        if np.any(left) and np.any(right):
            center_angle = (np.mean(angles[left]) + np.mean(angles[right])) / 2
        else:
            center_angle = np.mean(angles)

        # 더 부드러운 smoothing
        target = SMOOTH_FACTOR * center_angle + (1 - SMOOTH_FACTOR) * self.prev_target
        self.prev_target = target

        return target, np.max(ranges)


follower = CenterlineFollower()

print("🚀 조향 부드럽게 + 방향 반전 적용 버전")

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

        if len(points) >= 50:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])

            front_mask = (angles > -75) & (angles < 75)
            if np.any(front_mask):
                target_angle, forward_clear = follower.find_centerline(angles[front_mask], ranges[front_mask])

                steering = target_angle * STEERING_GAIN
                if STEERING_REVERSE:
                    steering = -steering

                steering = np.clip(steering, -0.55, 0.55)   # 최대 조향각도 제한

                v = np.clip(forward_clear * 0.42, 0.18, MAX_SPEED)

                d_min = np.min(ranges[front_mask]) if len(ranges[front_mask]) > 0 else 1.0
                if d_min < 0.35:
                    v *= 0.7

                w = steering * 3.5                     # angular velocity도 부드럽게

                cmd = f"{v:.3f},{w:.3f}\n"
                arduino_ser.write(cmd.encode('utf-8'))

                print(f"Target: {target_angle:+6.1f}° | Steer: {steering:+6.1f}° | v:{v:.3f} | d_min:{d_min:.2f}m")

            buffer = buffer[i:]

        time.sleep(0.007)

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
