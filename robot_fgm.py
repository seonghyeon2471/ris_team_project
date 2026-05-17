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
ROBOT_WIDTH = 0.16
LIDAR_OFFSET_FRONT = 0.025
LOOKAHEAD_DISTANCE = 0.8   # Pure Pursuit lookahead (m)

SAFETY_FACTOR = 1.22
ALPHA = 2.6
BETA = 0.7

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
# Local Centerline Follower
# =========================================
class CenterlineFollower:
    def __init__(self):
        self.prev_target_angle = 0.0
        self.smooth_factor = 0.65   # 경로 부드럽게 (0~1)

    def correct_to_center(self, angles_deg, ranges):
        angles_rad = np.radians(angles_deg)
        x = ranges * np.cos(angles_rad) - LIDAR_OFFSET_FRONT
        y = ranges * np.sin(angles_rad)
        dist = np.sqrt(x**2 + y**2 + 1e-8)
        ang = np.degrees(np.arctan2(y, x))
        return ang, dist

    def find_centerline_target(self, angles_deg, ranges):
        """자유 공간의 중앙선 방향 찾기"""
        corr_angles, corr_ranges = self.correct_to_center(angles_deg, ranges)

        # 앞쪽만 사용
        mask = (corr_angles > -70) & (corr_angles < 70)
        angles = corr_angles[mask]
        ranges = corr_ranges[mask]

        # 좌우 벽 찾기 (간단한 centerline approximation)
        left_mask = angles < 0
        right_mask = angles > 0

        if np.any(left_mask) and np.any(right_mask):
            left_dist = np.min(ranges[left_mask])
            right_dist = np.min(ranges[right_mask])
            left_angle = np.mean(angles[left_mask][ranges[left_mask] == left_dist])
            right_angle = np.mean(angles[right_mask][ranges[right_mask] == right_dist])
            
            # 중앙 각도
            center_angle = (left_angle + right_angle) / 2
        else:
            # gap 중심
            valid = ranges > 0.08
            if np.any(valid):
                center_angle = np.mean(angles[valid])
            else:
                center_angle = 0.0

        # smoothing
        target = self.smooth_factor * center_angle + (1 - self.smooth_factor) * self.prev_target_angle
        self.prev_target_angle = target

        return target, np.max(ranges) if len(ranges) > 0 else 1.0

    def process(self, angles_deg, ranges):
        target_angle, forward_clear = self.find_centerline_target(angles_deg, ranges)

        # Pure Pursuit + PID 스타일
        error = target_angle
        steering = np.clip(error * 2.8, -0.65, 0.65)   # rad

        # 속도
        v = np.clip(forward_clear * 0.45, 0.22, 0.72)

        # 안전 마진
        d_min = np.min(ranges) if len(ranges) > 0 else 1.0
        if d_min < 0.25:
            v = v * 0.6
            steering = steering * 1.3

        return steering, v


follower = CenterlineFollower()

print("🚀 Local Centerline Following 시작 (중앙선 따라가기)")

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

            front_mask = (angles > -75) & (angles < 75)
            if np.any(front_mask):
                steering, v = follower.process(angles[front_mask], ranges[front_mask])
                w = steering * 3.0   # angular velocity

                cmd = f"{v:.3f},{w:.3f}\n"
                arduino_ser.write(cmd.encode('utf-8'))

                print(f"TargetAngle: {steering/np.pi*180:+6.1f}° | v:{v:.3f} | w:{w:+6.3f}")

            buffer = buffer[i:]

        time.sleep(0.007)

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
