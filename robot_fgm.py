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
ROBOT_WIDTH = 0.20
LIDAR_OFFSET_FRONT = 0.025

MAX_SPEED = 0.16
STEERING_GAIN = 8.0        # 매우 민감하게

print("LIDAR 시작 중...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ LIDAR SCAN START")


class Navigator:
    def process(self, angles_deg, ranges):
        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        if len(angles) < 30:
            print("데이터 부족")
            return 0.0, 0.15

        # ==================== 좌우 gap 계산 (강화) ====================
        left_mask = (angles < -15)
        right_mask = (angles > 15)

        left_clear = np.min(ranges[left_mask]) if np.any(left_mask) else 3.0
        right_clear = np.min(ranges[right_mask]) if np.any(right_mask) else 3.0

        diff = right_clear - left_clear          # 오른쪽이 더 트이면 양수 → 우회전

        target_angle = diff * 60.0               # 차이에 강한 가중치

        # 디버깅 출력
        print(f"L:{left_clear:.2f}m | R:{right_clear:.2f}m | Diff:{diff:+.3f} | TargetAngle:{target_angle:+6.1f}°")

        steering = STEERING_GAIN * target_angle
        steering = np.clip(steering, -0.75, 0.75)

        front_clear = np.max(ranges) if len(ranges) > 0 else 1.0
        v = np.clip(front_clear * 0.45, 0.18, MAX_SPEED)

        d_min = np.min(ranges) if len(ranges) > 0 else 1.0
        if d_min < 0.40:
            v *= 0.6

        return steering, v


navigator = Navigator()

print("🚀 좌우 gap 기반 강력 조향 테스트 버전")

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

            front_mask = (angles > -90) & (angles < 90)
            if np.any(front_mask):
                steering, v = navigator.process(angles[front_mask], ranges[front_mask])
                w = steering * 5.0

                cmd = f"{v:.3f},{w:.3f}\n"
                arduino_ser.write(cmd.encode('utf-8'))

            buffer = buffer[i:]

        time.sleep(0.008)

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
