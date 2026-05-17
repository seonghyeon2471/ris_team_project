import serial
import time
import numpy as np
import math
from rplidar import RPLidar

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.05)

# =========================================
# RPLIDAR
# =========================================
lidar = RPLidar('/dev/ttyUSB0', baudrate=460800)
lidar.start_motor()
print("✅ RPLIDAR + Kalman Filter 버전 시작")

# =========================================
# 칼만 필터 클래스
# =========================================
class KalmanFilter:
    def __init__(self, process_var=0.1, measurement_var=0.8):
        self.x = 0.0          # 추정값
        self.P = 1.0          # 오차 공분산
        self.Q = process_var      # 프로세스 노이즈
        self.R = measurement_var  # 측정 노이즈

    def update(self, measurement):
        # 예측
        self.P = self.P + self.Q
        # 칼만 이득
        K = self.P / (self.P + self.R)
        # 업데이트
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * self.P
        return self.x


# =========================================
# 메인 클래스
# =========================================
class KalmanFollower:
    def __init__(self):
        self.kf_steering = KalmanFilter(process_var=0.05, measurement_var=1.2)
        self.kf_speed = KalmanFilter(process_var=0.02, measurement_var=0.5)
        self.prev_steering = 0.0

    def process(self, scan):
        angles = np.array([m[1] for m in scan])
        distances = np.array([m[2] for m in scan]) / 1000.0   # mm → m

        mask = (angles > -90) & (angles < 90)
        angles = angles[mask]
        ranges = distances[mask]

        if len(ranges) < 50:
            return 0.0, 0.14

        # 좌우 clearance
        left_clear = np.min(ranges[angles < -20]) if np.any(angles < -20) else 3.0
        right_clear = np.min(ranges[angles > 20]) if np.any(angles > 20) else 3.0

        diff = right_clear - left_clear
        raw_target = diff * 28.0

        # 칼만 필터 적용
        steering = self.kf_steering.update(raw_target)
        steering = 3.2 * steering                     # gain
        steering = np.clip(steering, -0.60, 0.60)

        # smoothing
        steering = 0.82 * steering + 0.18 * self.prev_steering
        self.prev_steering = steering

        # 속도
        front_clear = np.max(ranges)
        v = self.kf_speed.update(front_clear * 0.45)
        v = np.clip(v, 0.18, MAX_SPEED)

        if np.min(ranges) < 0.40:
            v *= 0.6

        return steering, v


follower = KalmanFollower()

print("🚀 Kalman Filter 적용 버전 시작!")

try:
    for scan in lidar.iter_scans(max_buf_meas=1500, min_len=100):
        steering, v = follower.process(scan)
        w = steering * 3.8

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
