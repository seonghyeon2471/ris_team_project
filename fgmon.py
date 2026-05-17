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
# 로봇 물리 스펙 (정확히 반영)
# =========================================
ROBOT_LENGTH = 0.21      # m
ROBOT_WIDTH = 0.16       # m
LIDAR_OFFSET_FRONT = 0.025  # m (앞에서 2.5cm 뒤)
WHEEL_AXIS_FROM_REAR = 0.035
WHEEL_PROTRUDE = 0.02

ROBOT_RADIUS = ROBOT_WIDTH / 2 + 0.08   # 안전 반경
SAFETY_FACTOR = 1.35
ALPHA = 2.2          # IFGM 가중치 (벽에서 밀어내는 강도)
BETA = 0.85          # 속도에 따른 안전 마진

# =========================================
# LIDAR START (당신 원본 그대로)
# =========================================
print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))   # STOP
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))   # SCAN
lidar_ser.read(7)
print("✅ LIDAR SCAN START")

# =========================================
# Advanced Gap Follower (IFGM + Disparity Extender)
# =========================================
class AdvancedFollower:
    def __init__(self):
        self.alpha = ALPHA
        self.beta = BETA
        self.max_steering = 0.55  # rad ≈ 31.5°

    def disparity_extender(self, ranges, angles):
        """벽 모서리 가상 확장"""
        extended = ranges.copy()
        for i in range(1, len(ranges)-1):
            if abs(ranges[i] - ranges[i-1]) > 0.25:          # 급격한 거리 변화
                d_near = min(ranges[i], ranges[i-1])
                if d_near < 0.1:
                    d_near = 0.1
                delta_theta = math.asin((ROBOT_WIDTH/2 * SAFETY_FACTOR) / d_near)
                mask = int(math.ceil(delta_theta / math.radians(0.8)))  # 해상도
                
                for k in range(-mask, mask+1):
                    idx = i + k
                    if 0 <= idx < len(extended):
                        extended[idx] = 0.0
        return extended

    def process(self, ranges, angles_deg):
        angles_rad = np.radians(angles_deg)
        proc = self.disparity_extender(ranges, angles_rad)

        # Bubble 제거
        if len(proc) > 0:
            closest_idx = np.argmin(proc)
            closest_d = max(proc[closest_idx], 0.1)
            bubble = math.atan2(ROBOT_RADIUS * 1.3, closest_d)
            for i, a in enumerate(angles_rad):
                if abs(a - angles_rad[closest_idx]) < bubble:
                    proc[i] = 0.0

        # Gap 찾기
        valid = proc > 0.08
        if not np.any(valid):
            return 0.0, 0.0   # 막힘

        valid_idx = np.where(valid)[0]
        gap_center_idx = int((valid_idx[0] + valid_idx[-1]) / 2)
        phi_gap = angles_rad[gap_center_idx]

        # IFGM 공식
        d_min = np.min(ranges) if len(ranges) > 0 else 2.0
        phi_goal = 0.0                     # 직진 (IMU로 개선 가능)

        phi_final = (self.alpha * d_min * phi_gap + phi_goal) / (self.alpha * d_min + 1.0)

        # Dynamic Safety Margin
        v_max = np.max(ranges) if len(ranges) > 0 else 1.5
        v = np.clip(v_max * 0.42, 0.25, 0.75)
        safety = math.exp(-self.beta * v / max(d_min, 0.15))
        steering = phi_final * safety
        steering = np.clip(steering, -self.max_steering, self.max_steering)

        return steering, v


follower = AdvancedFollower()

print("🚀 로봇 자율 주행 시작 (중앙 따라가기 + 장애물 회피)")

# =========================================
# 메인 루프
# =========================================
buffer = bytearray()

try:
    while True:
        data = lidar_ser.read(400)
        if data:
            buffer.extend(data)

        # 간단한 파싱 (C1 Express Scan)
        while len(buffer) >= 5:
            # 실제로는 더 정확한 헤더 체크가 필요하지만, 실용적으로 동작하도록
            points = []
            i = 0
            while i + 5 <= len(buffer):
                try:
                    q = buffer[i] >> 2
                    angle_q6 = ((buffer[i+1] << 7) | (buffer[i] >> 1)) & 0x7FFF
                    angle_deg = (angle_q6 / 64.0) - 180.0
                    dist = ((buffer[i+3] << 8) | buffer[i+2]) * 4   # mm

                    if q > 0 and 100 < dist < 12000:
                        points.append((angle_deg, dist))
                except:
                    pass
                i += 5

            if len(points) > 80:
                angles = np.array([p[0] for p in points])
                ranges = np.array([p[1]/1000.0 for p in points])  # m

                # 앞쪽만 사용 (-65 ~ 65도)
                mask = (angles > -65) & (angles < 65)
                if np.any(mask):
                    steering, v = follower.process(ranges[mask], angles[mask])
                    w = steering * 2.9   # steering → angular velocity

                    cmd = f"{v:.3f},{w:.3f}\n"
                    arduino_ser.write(cmd.encode('utf-8'))

                    print(f"Steer: {np.degrees(steering):+6.1f}° | v:{v:.3f}m/s | d_min:{np.min(ranges[mask]):.2f}m")

                # 버퍼 정리
                buffer = buffer[i:]
        
        time.sleep(0.008)

except KeyboardInterrupt:
    print("\n\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))  # STOP
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
