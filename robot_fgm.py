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
# 로봇 스펙 (최신)
# =========================================
ROBOT_WIDTH = 0.22        # ← 22cm로 변경
LIDAR_OFFSET_FRONT = 0.025

MAX_SPEED = 0.15
STEERING_GAIN = 3.6
INFLATION_MARGIN = 0.06   # 6cm inflation (너비 증가에 맞춰 조정)

print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(1.5)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ ROBOT_WIDTH = 22cm 적용 버전 시작")


class SafePathFollower:
    def __init__(self):
        self.prev_steering = 0.0

    def inflate_obstacles(self, angles, ranges):
        """장애물 주변 안전 공간 확장"""
        proc = ranges.copy()
        half_width = ROBOT_WIDTH / 2 + INFLATION_MARGIN

        for i in range(1, len(ranges)-1):
            if ranges[i] < 2.0:
                d_near = max(min(ranges[i], ranges[i-1]), 0.08)
                delta = math.asin(half_width / d_near)
                mask = int(math.ceil(delta / math.radians(1.0)))
                for k in range(-mask, mask + 1):
                    idx = i + k
                    if 0 <= idx < len(proc):
                        proc[idx] = 0.0
        return proc

    def process(self, angles_deg, ranges):
        if len(ranges) < 35:
            return 0.0, 0.13

        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        safe_ranges = self.inflate_obstacles(angles, ranges)

        free_mask = safe_ranges > 0.10
        if not np.any(free_mask):
            return 0.0, -0.18   # 후진

        free_angles = angles[free_mask]
        free_ranges = safe_ranges[free_mask]

        # 가중 중앙 계산
        weights = free_ranges ** 1.8
        target_angle = np.average(free_angles, weights=weights)

        # smoothing
        steering = STEERING_GAIN * target_angle
        steering = 0.75 * steering + 0.25 * self.prev_steering
        self.prev_steering = steering

        steering = np.clip(steering, -0.62, 0.62)

        # 속도
        v = np.clip(np.max(free_ranges) * 0.45, 0.18, MAX_SPEED)
        d_min = np.min(ranges) if len(ranges) > 0 else 1.0
        if d_min < 0.40:
            v *= 0.6

        return steering, v


follower = SafePathFollower()

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
            w = steering * 4.0

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
