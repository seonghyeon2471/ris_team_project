import serial
import time
import numpy as np
import math

# =========================================
# SERIAL & LIDAR
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.05)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.0)

# =========================================
# 로봇 스펙
# =========================================
ROBOT_WIDTH = 0.22
LIDAR_OFFSET_FRONT = 0.025

MAX_SPEED = 0.14
STEERING_GAIN = 1.8        # ← 크게 낮춤
SMOOTH_FACTOR = 0.88       # ← smoothing 매우 강하게
DEADZONE = 12.0            # ±12도 이내는 직진

print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(1.5)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ 도리도리 방지 + 안정 버전 시작")


class StableFollower:
    def __init__(self):
        self.prev_steering = 0.0

    def process(self, angles_deg, ranges):
        if len(ranges) < 40:
            return 0.0, 0.13

        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        left_clear = np.min(ranges[angles < -20]) if np.any(angles < -20) else 3.0
        right_clear = np.min(ranges[angles > 20]) if np.any(angles > 20) else 3.0

        diff = right_clear - left_clear
        target_angle = diff * 22.0          # 민감도 낮춤

        # 강력 smoothing
        steering = STEERING_GAIN * target_angle
        steering = SMOOTH_FACTOR * steering + (1 - SMOOTH_FACTOR) * self.prev_steering
        self.prev_steering = steering

        # Deadzone (작은 오차는 무시 → 도리도리 방지)
        if abs(steering) < DEADZONE:
            steering = 0.0

        steering = np.clip(steering, -0.55, 0.55)

        # 속도
        v = np.clip(np.max(ranges) * 0.45, 0.18, MAX_SPEED)
        if np.min(ranges) < 0.40:
            v *= 0.65

        return steering, v


follower = StableFollower()

buffer = bytearray()

try:
    while True:
        data = lidar_ser.read(1024)
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

                if quality >= 3 and 50 < dist_mm < 12000:
                    points.append((angle_deg, dist_mm / 1000.0))
            except:
                pass
            i += 5

        if len(points) >= 45:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])

            steering, v = follower.process(angles, ranges)
            w = steering * 3.5

            cmd = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(cmd.encode('utf-8'))

            print(f"Steer: {steering:+6.1f}° | v:{v:.3f}")

            buffer = buffer[i:]
        else:
            if len(buffer) > 4096:
                buffer = buffer[-2048:]

        time.sleep(0.003)

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
