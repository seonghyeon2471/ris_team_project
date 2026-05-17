import serial
import time
import numpy as np
import math

# =========================================
# SERIAL (최대 지연 최소화)
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.03)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.0)   # Non-blocking

# =========================================
# 로봇 스펙
# =========================================
ROBOT_WIDTH = 0.22
LIDAR_OFFSET_FRONT = 0.025

MAX_SPEED = 0.15
STEERING_GAIN = 2.6
SMOOTH_FACTOR = 0.80

print("LIDAR 시작 중 (Ultra Low Latency Mode)...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(1)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ Ultra Low Latency + Clean Reading 시작")


class FastFollower:
    def __init__(self):
        self.prev_steering = 0.0

    def process(self, angles_deg, ranges):
        if len(ranges) < 50:
            return 0.0, 0.14

        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        left_clear = np.min(ranges[angles < -20]) if np.any(angles < -20) else 3.0
        right_clear = np.min(ranges[angles > 20]) if np.any(angles > 20) else 3.0

        diff = right_clear - left_clear
        target = diff * 28.0

        steering = STEERING_GAIN * target
        steering = SMOOTH_FACTOR * steering + (1 - SMOOTH_FACTOR) * self.prev_steering
        self.prev_steering = steering

        if abs(steering) < 10.0:        # deadzone
            steering = 0.0

        steering = np.clip(steering, -0.58, 0.58)

        v = np.clip(np.max(ranges) * 0.48, 0.18, MAX_SPEED)
        if np.min(ranges) < 0.35:
            v *= 0.65

        return steering, v


follower = FastFollower()
buffer = bytearray()

try:
    while True:
        # ==================== 지연 최소화 ====================
        data = lidar_ser.read(2048)      # 큰 버퍼로 한 번에 읽기
        if data:
            buffer.extend(data)

        points = []
        i = 0
        n = len(buffer)
        while i + 5 <= n:
            try:
                byte1 = buffer[i]
                quality = byte1 >> 2
                angle_q6 = ((buffer[i+1] << 7) | (byte1 >> 1)) & 0x7FFF
                angle_deg = (angle_q6 / 64.0) - 180.0
                dist_mm = ((buffer[i+3] << 8) | buffer[i+2]) * 4

                if quality >= 5 and 60 < dist_mm < 10000:   # quality 강화
                    points.append((angle_deg, dist_mm / 1000.0))
            except:
                pass
            i += 5

        if len(points) >= 60:                     # 충분한 데이터만 처리
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])

            steering, v = follower.process(angles, ranges)
            w = steering * 4.0

            cmd = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(cmd.encode('utf-8'))

            print(f"Steer: {steering:+6.1f}° | v:{v:.3f} | Points:{len(points)}")

            buffer = buffer[i:]      # 사용한 데이터 정리
        else:
            # buffer 과부하 방지
            if len(buffer) > 8192:
                buffer = buffer[-4096:]

        time.sleep(0.001)   # 1ms (최소)

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
