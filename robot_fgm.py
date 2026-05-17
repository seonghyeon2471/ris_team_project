import serial
import time
import numpy as np
import math
from rplidar import RPLidar

# =========================================
# SERIAL (Arduino)
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.05)

# =========================================
# 로봇 스펙
# =========================================
ROBOT_WIDTH = 0.22
LIDAR_OFFSET_FRONT = 0.025
MAX_SPEED = 0.16
STEERING_GAIN = 3.5

print("RPLIDAR 라이브러리 버전 시작...")

# =========================================
# RPLIDAR 초기화
# =========================================
lidar = RPLidar('/dev/ttyUSB0', baudrate=460800)
lidar.start_motor()
print("✅ RPLIDAR 연결 완료")


class RPLidarFollower:
    def __init__(self):
        self.prev_steering = 0.0

    def process(self, scan):
        # scan = [(quality, angle_deg, distance_mm), ...]
        angles = np.array([m[1] for m in scan])
        distances = np.array([m[2] for m in scan]) / 10.0   # mm → cm

        # 전방 180도
        mask = (angles > -90) & (angles < 90)
        angles = angles[mask]
        ranges = distances[mask] / 100.0   # cm → m

        if len(ranges) < 50:
            return 0.0, 0.14

        # 좌우 clearance
        left_clear = np.min(ranges[angles < -20]) if np.any(angles < -20) else 3.0
        right_clear = np.min(ranges[angles > 20]) if np.any(angles > 20) else 3.0

        # 중앙 방향 계산
        diff = right_clear - left_clear
        target_angle = diff * 32.0

        # smoothing
        steering = STEERING_GAIN * target_angle
        steering = 0.78 * steering + 0.22 * self.prev_steering
        self.prev_steering = steering

        steering = np.clip(steering, -0.65, 0.65)

        # 속도
        front_clear = np.max(ranges) if len(ranges) > 0 else 1.0
        v = np.clip(front_clear * 0.45, 0.18, MAX_SPEED)

        if np.min(ranges) < 0.40:
            v *= 0.65

        return steering, v


follower = RPLidarFollower()

print("🚀 rplidar 라이브러리 + 중앙 따라가기 시작!")

try:
    for scan in lidar.iter_scans(max_buf_meas=1500, min_len=100):
        steering, v = follower.process(scan)
        w = steering * 4.0

        cmd = f"{v:.3f},{w:.3f}\n"
        arduino_ser.write(cmd.encode('utf-8'))

        print(f"Steer: {steering:+6.1f}° | v:{v:.3f}")

except KeyboardInterrupt:
    print("\n🛑 종료")
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    arduino_ser.write(b"0,0\n")
    arduino_ser.close()
    print("✅ 정리 완료")
