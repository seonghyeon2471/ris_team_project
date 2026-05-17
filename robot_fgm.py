import serial
import time
import numpy as np
import math

# =========================================
# SERIAL & LIDAR
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# IMU
USE_IMU = True
try:
    import smbus2
    imu_bus = smbus2.SMBus(1)
    IMU_ADDR = 0x68
    imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0)
    print("✅ IMU ON")
except:
    USE_IMU = False
    print("[WARN] IMU OFF")

# =========================================
# 로봇 스펙 + 튜닝 파라미터 (좁은 통로 최적)
# =========================================
ROBOT_LENGTH = 0.21
ROBOT_WIDTH = 0.16
LIDAR_OFFSET_FRONT = 0.025

SAFETY_FACTOR = 1.22
ALPHA = 2.85
BETA = 0.68

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
# NarrowPathFollower (정면 10cm 기준)
# =========================================
class NarrowPathFollower:
    def __init__(self):
        self.alpha = ALPHA
        self.beta = BETA
        self.max_steering = 0.63

    def correct_lidar_to_robot_center(self, angles_deg, ranges):
        angles_rad = np.radians(angles_deg)
        x = ranges * np.cos(angles_rad) - LIDAR_OFFSET_FRONT
        y = ranges * np.sin(angles_rad)
        corrected_dist = np.sqrt(x**2 + y**2)
        corrected_angle = np.degrees(np.arctan2(y, x))
        return corrected_angle, corrected_dist

    def process(self, angles_deg, ranges):
        corr_angles, corr_ranges = self.correct_lidar_to_robot_center(angles_deg, ranges)

        # 정면 최소 거리
        front_mask = (corr_angles > -35) & (corr_angles < 35)
        front_clear = np.min(corr_ranges[front_mask]) if np.any(front_mask) else 2.0

        print(f"Front: {front_clear*100:5.1f}cm | Min: {np.min(corr_ranges):.2f}m")

        # ==================== 정면 10cm 이내 ====================
        if front_clear < 0.10:          # ← 여기서 10cm로 변경
            print("⚠️⚠️ 정면 10cm 이내! 강제 회피")
            left_clear = np.min(corr_ranges[(corr_angles > -70) & (corr_angles < -15)])
            right_clear = np.min(corr_ranges[(corr_angles > 15) & (corr_angles < 70)])
            
            if left_clear > right_clear:
                steering = 0.60
            else:
                steering = -0.60
            v = -0.25 if front_clear < 0.08 else 0.15   # 8cm 이내면 후진
            return steering, v

        # ==================== 정상 주행 ====================
        proc = corr_ranges.copy()

        # Disparity Extender
        for i in range(1, len(proc)-1):
            if abs(proc[i] - proc[i-1]) > 0.18:
                d_near = max(min(proc[i], proc[i-1]), 0.08)
                delta = math.asin((ROBOT_WIDTH/2 * SAFETY_FACTOR) / d_near)
                mask = int(math.ceil(delta / math.radians(0.6)))
                for k in range(-mask, mask+1):
                    if 0 <= i+k < len(proc):
                        proc[i+k] = 0.0

        # Bubble
        if len(proc) > 0:
            closest_idx = np.argmin(proc)
            closest_d = max(proc[closest_idx], 0.08)
            bubble = math.atan2(ROBOT_WIDTH/2 + 0.045, closest_d)
            angles_rad = np.radians(corr_angles)
            for i, a in enumerate(angles_rad):
                if abs(a - angles_rad[closest_idx]) < bubble:
                    proc[i] = 0.0

        # Gap 찾기
        valid = proc > 0.055
        if not np.any(valid):
            return 0.0, -0.20

        valid_idx = np.where(valid)[0]
        weights = proc[valid_idx] ** 1.7
        best_idx = valid_idx[np.argmax(weights)]
        phi_gap = np.radians(corr_angles[best_idx])

        d_min = np.min(corr_ranges)
        phi_final = (self.alpha * d_min * phi_gap) / (self.alpha * d_min + 1.0)

        v_max = np.max(corr_ranges)
        v = np.clip(v_max * 0.46, 0.20, 0.70)

        safety = math.exp(-self.beta * v / max(d_min, 0.1))
        steering = np.clip(phi_final * safety, -self.max_steering, self.max_steering)

        return steering, v


follower = NarrowPathFollower()

print("🚀 정면 10cm 감지 + 좁은 통로 버전 시작!")

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

                if quality > 0 and 100 < dist_mm < 12000:
                    points.append((angle_deg, dist_mm))
            except:
                pass
            i += 5

        if len(points) >= 60:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1]/1000.0 for p in points])

            front_mask = (angles > -70) & (angles < 70)
            if np.any(front_mask):
                steering, v = follower.process(angles[front_mask], ranges[front_mask])
                w = steering * 3.2

                cmd = f"{v:.3f},{w:.3f}\n"
                arduino_ser.write(cmd.encode('utf-8'))

            buffer = buffer[i:]

        time.sleep(0.006)

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
