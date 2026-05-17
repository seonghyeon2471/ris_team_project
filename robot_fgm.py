import serial
import time
import numpy as np
import math

# =========================================
# SERIAL 설정 (지연 최소화)
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.05)
lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.0)   # Non-blocking

# =========================================
# 로봇 스펙
# =========================================
ROBOT_WIDTH = 0.20
LIDAR_OFFSET_FRONT = 0.025

MAX_SPEED = 0.16
STEERING_GAIN = 6.5

print("LIDAR 시작 중 (Low Latency Mode)...")
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(1)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("✅ Low Latency Mode 시작")

class LowLatencyNavigator:
    def process(self, angles_deg, ranges):
        if len(angles_deg) < 35:
            return 0.0, 0.15

        mask = (angles_deg > -90) & (angles_deg < 90)
        angles = angles_deg[mask]
        ranges = ranges[mask]

        left_clear = np.min(ranges[angles < -15]) if np.any(angles < -15) else 3.0
        right_clear = np.min(ranges[angles > 15]) if np.any(angles > 15) else 3.0

        diff = right_clear - left_clear
        target_angle = diff * 55.0

        steering = STEERING_GAIN * target_angle
        steering = np.clip(steering, -0.78, 0.78)

        v = np.clip(np.max(ranges) * 0.48, 0.18, MAX_SPEED)
        if np.min(ranges) < 0.40:
            v *= 0.6

        return steering, v


navigator = LowLatencyNavigator()
buffer = bytearray()

try:
    while True:
        # ==================== 지연 최소화 핵심 ====================
        data = lidar_ser.read(2048)      # 큰 크기로 한 번에 읽기
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

                # quality 기준 완화 + 속도 최적화
                if quality >= 3 and 50 < dist_mm < 12000:
                    points.append((angle_deg, dist_mm / 1000.0))
            except:
                pass
            i += 5

        # 데이터 충분하면 처리
        if len(points) >= 40:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])

            steering, v = navigator.process(angles, ranges)
            w = steering * 5.2

            cmd = f"{v:.3f},{w:.3f}\n"
            arduino_ser.write(cmd.encode('utf-8'))

            print(f"Steer: {steering:+6.1f}° | v:{v:.3f} | Points:{len(points)}")

            buffer = buffer[i:]          # 사용한 데이터 정리
        else:
            # buffer가 너무 커지면 정리
            if len(buffer) > 4096:
                buffer = buffer[-2048:]

        # sleep 최소 (지연 핵심)
        time.sleep(0.0005)   # 0.5ms

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar_ser.write(bytes([0xA5, 0x40]))
    lidar_ser.close()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
