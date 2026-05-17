import serial
import time
import numpy as np
import math

# =========================================
# SERIAL & LIDAR INIT (당신 원본 그대로)
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# IMU (원본 그대로)
# =========================================
USE_IMU = True
try:
    import smbus2
    imu_bus = smbus2.SMBus(1)
    IMU_ADDR = 0x68
    imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0)
    print("✅ IMU ON")
except Exception:
    USE_IMU = False
    print("[WARN] IMU OFF")

# =========================================
# 로봇 물리 스펙
# =========================================
ROBOT_WIDTH = 0.16       # m
LIDAR_OFFSET_FRONT = 0.025
SAFETY_FACTOR = 1.32
ALPHA = 2.15
BETA = 0.85

# =========================================
# LIDAR START (원본 그대로)
# =========================================
print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))   # STOP
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))   # SCAN
lidar_ser.read(7)
print("✅ LIDAR SCAN START")

# =========================================
# Advanced Gap Follower (IFGM + Disparity + 좁은통로 개선)
# =========================================
class AdvancedFollower:
    def __init__(self):
        self.alpha = ALPHA
        self.beta = BETA
        self.max_steering = 0.55
        self.normal_radius = ROBOT_WIDTH/2 + 0.08
        self.tight_radius = ROBOT_WIDTH/2 + 0.035

    def is_narrow_passage(self, ranges):
        valid = ranges > 0.05
        if not np.any(valid):
            return False
        return np.min(ranges[valid]) < 0.28

    def disparity_extender(self, ranges):
        extended = ranges.copy()
        for i in range(1, len(ranges)-1):
            if abs(ranges[i] - ranges[i-1]) > 0.22:
                d_near = max(min(ranges[i], ranges[i-1]), 0.1)
                delta_theta = math.asin((ROBOT_WIDTH/2 * SAFETY_FACTOR) / d_near)
                mask = int(math.ceil(delta_theta / math.radians(0.8)))
                for k in range(-mask, mask + 1):
                    idx = i + k
                    if 0 <= idx < len(extended):
                        extended[idx] = 0.0
        return extended

    def process(self, ranges, angles_deg):
        angles_rad = np.radians(angles_deg)
        proc = self.disparity_extender(ranges)

        is_narrow = self.is_narrow_passage(ranges)
        current_radius = self.tight_radius if is_narrow else self.normal_radius
        min_valid = 0.06 if is_narrow else 0.08

        if is_narrow:
            print("🟡 좁은 통로 모드 ON")

        # Bubble
        if len(proc) > 0:
            closest_idx = np.argmin(proc)
            closest_d = max(proc[closest_idx], 0.1)
            bubble = math.atan2(current_radius * 1.25, closest_d)
            for i, a in enumerate(angles_rad):
                if abs(a - angles_rad[closest_idx]) < bubble:
                    proc[i] = 0.0

        # Gap 찾기
        valid = proc > min_valid
        if not np.any(valid):
            if np.min(ranges) > 0.055:
                valid = ranges > 0.055
            else:
                return 0.0, 0.0   # 완전 막힘

        valid_idx = np.where(valid)[0]
        gap_center_idx = int((valid_idx[0] + valid_idx[-1]) / 2)
        phi_gap = angles_rad[gap_center_idx]

        # IFGM
        d_min = np.min(ranges) if len(ranges) > 0 else 2.0
        phi_final = (self.alpha * d_min * phi_gap) / (self.alpha * d_min + 1.0)

        # 속도 + Safety
        v_max = np.max(ranges) if len(ranges) > 0 else 1.5
        v = np.clip(v_max * 0.40, 0.22, 0.70)
        safety = math.exp(-self.beta * v / max(d_min, 0.12))
        steering = np.clip(phi_final * safety, -self.max_steering, self.max_steering)

        return steering, v


follower = AdvancedFollower()

print("🚀 로봇 자율 주행 시작! (Ctrl+C로 종료)")

# =========================================
# Raw LIDAR 파싱 루프
# =========================================
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

                if quality > 0 and 150 < dist_mm < 12000:
                    points.append((angle_deg, dist_mm))
            except:
                pass
            i += 5

        # 충분한 데이터 모이면 처리
        if len(points) >= 80:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] / 1000.0 for p in points])   # mm → m

            # 앞쪽만 사용 (출발선 회귀 방지)
            front_mask = (angles > -65) & (angles < 65)
            if np.any(front_mask):
                front_angles = angles[front_mask]
                front_ranges = ranges[front_mask]

                steering, v = follower.process(front_ranges, front_angles)
                w = steering * 2.9   # steering → angular velocity

                cmd = f"{v:.3f},{w:.3f}\n"
                arduino_ser.write(cmd.encode('utf-8'))

                print(f"Steer: {np.degrees(steering):+6.1f}° | v:{v:.3f} | d_min:{np.min(front_ranges):.2f}m | Points:{len(points)}")

            # 사용한 데이터 정리
            buffer = buffer[i:]

        time.sleep(0.008)

except KeyboardInterrupt:
    print("\n\n🛑 종료")

finally:
    lidar_ser.write(bytes([0xA5, 0x40]))  # STOP
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
    print("✅ 정리 완료")
