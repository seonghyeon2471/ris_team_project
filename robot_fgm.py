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
ROBOT_WIDTH = 0.20
LIDAR_OFFSET_FRONT = 0.025

MAX_SPEED = 0.15
STEERING_GAIN = 2.4        # 부드럽게
SMOOTH_FACTOR = 0.82
DEADZONE = 9.0

print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(1.5)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ 에러 방지 + 안정 버전 시작")


class StableFollower:
    def __init__(self):
        self.prev_steering = 0.0

    def process(self, angles_deg, ranges):
        if len(angles_deg) < 30 or len(ranges) == 0:
            print("⚠️ 데이터 부족")
            return 0.0, 0.15

        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        if len(ranges) == 0:
            return 0.0, 0.15

        # 좌우 clearance
        left_clear = np.min(ranges[angles < -20]) if np.any(angles < -20) else 3.0
        right_clear = np.min(ranges[angles > 20]) if np.any(angles > 20) else 3.0

        diff = right_clear - left_clear
        target_angle = diff * 25.0

        # smoothing
        steering = STEERING_GAIN * target_angle
        steering = SMOOTH_FACTOR * steering + (1 - SMOOTH_FACTOR) * self.prev_steering
        self.prev_steering = steering

        if abs(steering) < DEADZONE:
            steering = 0.0

        steering = np.clip(steering, -0.55, 0.55)

        # 안전한 max/min 처리
        front_clear = np.max(ranges) if len(ranges) > 0 else 1.0
        d_min = np.min(ranges) if len(ranges) > 0 else 1.0

        v = np.clip(front_clear * 0.45, 0.18, MAX_SPEED)
        if d_min < 0.40:
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

        if len(points) >= 40:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])

            steering, v = follower.process(angles, ranges)
            w = steering * 3.8

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
