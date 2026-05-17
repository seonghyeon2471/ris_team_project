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
print("✅ RPLIDAR 연결 완료")

# =========================================
# 파라미터 (회전 방지 강화)
# =========================================
MAX_SPEED = 0.14
STEERING_GAIN = 1.8        # 매우 낮춤
SMOOTH_FACTOR = 0.88       # smoothing 강하게
DEADZONE = 15.0            # ±15도 이내는 직진
MAX_TURN = 0.45            # 최대 steering 제한

print("🚀 제자리 회전 방지 + 안정 버전 시작")


class StableFollower:
    def __init__(self):
        self.prev_steering = 0.0
        self.steering_history = [0.0] * 5   # 최근 5개 steering 평균

    def process(self, scan):
        angles = np.array([m[1] for m in scan])
        distances = np.array([m[2] for m in scan]) / 10.0   # cm

        mask = (angles > -90) & (angles < 90)
        angles = angles[mask]
        ranges = distances[mask] / 100.0   # m 단위

        if len(ranges) < 40:
            return 0.0, 0.12

        # 좌우 clearance
        left_clear = np.min(ranges[angles < -20]) if np.any(angles < -20) else 3.0
        right_clear = np.min(ranges[angles > 20]) if np.any(angles > 20) else 3.0

        diff = right_clear - left_clear
        raw_target = diff * 22.0

        # steering 계산
        steering = STEERING_GAIN * raw_target

        # 강력 smoothing
        self.steering_history.append(steering)
        self.steering_history = self.steering_history[-5:]
        steering = np.mean(self.steering_history)

        # Deadzone 강화
        if abs(steering) < DEADZONE:
            steering = 0.0

        steering = np.clip(steering, -MAX_TURN, MAX_TURN)

        # 속도
        v = np.clip(np.max(ranges) * 0.45, 0.18, MAX_SPEED)
        if np.min(ranges) < 0.40:
            v *= 0.6

        return steering, v


follower = StableFollower()

try:
    for scan in lidar.iter_scans(max_buf_meas=1200, min_len=80):
        steering, v = follower.process(scan)
        w = steering * 3.2                     # angular velocity도 낮춤

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
