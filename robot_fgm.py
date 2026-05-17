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
STEERING_GAIN = 3.2        # 부드럽게
SMOOTH_FACTOR = 0.75

print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(1.5)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ 중앙 잘 찾는 버전 시작")


class PathFollower:
    def __init__(self):
        self.prev_steering = 0.0

    def process(self, angles_deg, ranges):
        if len(angles_deg) < 40:
            return 0.0, 0.15

        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        # ==================== 중앙 찾기 강화 ====================
        # 1. 가장 넓은 gap 찾기
        bin_size = 5.0
        bins = np.arange(-90, 91, bin_size)
        max_gap = 0
        best_center = 0.0

        for i in range(len(bins)-1):
            bin_mask = (angles >= bins[i]) & (angles < bins[i+1])
            if np.any(bin_mask):
                min_dist_in_bin = np.min(ranges[bin_mask])
                gap_width = np.sum(bin_mask) * bin_size
                if min_dist_in_bin > 0.25 and gap_width > max_gap:
                    max_gap = gap_width
                    best_center = (bins[i] + bins[i+1]) / 2

        # 2. 좌우 clearance 보조
        left_clear = np.min(ranges[angles < -15]) if np.any(angles < -15) else 3.0
        right_clear = np.min(ranges[angles > 15]) if np.any(angles > 15) else 3.0

        target_angle = best_center * 0.7 + (right_clear - left_clear) * 15

        # smoothing
        steering = STEERING_GAIN * target_angle
        steering = SMOOTH_FACTOR * steering + (1 - SMOOTH_FACTOR) * self.prev_steering
        self.prev_steering = steering

        steering = np.clip(steering, -0.60, 0.60)

        # 속도
        v = np.clip(np.max(ranges) * 0.45, 0.18, MAX_SPEED)
        if np.min(ranges) < 0.35:
            v *= 0.65

        return steering, v


follower = PathFollower()

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
            w = steering * 4.0

            cmd = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(cmd.encode('utf-8'))

            print(f"Steer: {steering:+6.1f}° | v:{v:.3f} | L:{np.min(ranges[angles<-15]):.2f} R:{np.min(ranges[angles>15]):.2f}")

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
