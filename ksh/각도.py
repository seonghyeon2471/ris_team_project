import serial
import time

# =========================================
# SERIAL PORTS (당신이 준 그대로)
# =========================================
ARDUINO_PORT = "/dev/serial0"      # Arduino (Serial1)
LIDAR_PORT   = "/dev/ttyUSB0"      # RPLIDAR

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar   = serial.Serial(LIDAR_PORT,   460800, timeout=0.05)

# =========================================
# LIDAR 시작 (당신이 이미 쓰던 명령어)
# =========================================
print("LiDAR 시작 중...")
lidar.write(bytes([0xA5, 0x40]))   # Motor on / info
time.sleep(1)
lidar.write(bytes([0xA5, 0x20]))   # Standard scan 시작
time.sleep(2)
lidar.reset_input_buffer()
print("LiDAR + Arduino 연결 완료! (60/90/120도 + 10cm 모드)")

# =========================================
# RPLIDAR 5바이트 패킷 파싱
# =========================================
def get_scan_points():
    chunk = lidar.read(1200)          # 한 번에 충분히 읽기
    points = []                       # (angle_deg, dist_m)
    i = 0
    while i + 4 < len(chunk):
        q = chunk[i]
        a1 = chunk[i+1]
        a2 = chunk[i+2]
        d1 = chunk[i+3]
        d2 = chunk[i+4]

        # RPLIDAR A1/A2 표준 파싱 (검증된 방식)
        angle_raw = (a2 << 8) | a1
        angle = (angle_raw / 64.0) % 360.0          # 0 ~ 360

        distance_mm = (d2 << 8) | d1
        dist = distance_mm / 4.0 / 1000.0           # mm → m

        # 유효한 점만 (너무 가까운 노이즈 제외)
        if 0.08 < dist < 5.0 and (q & 0x01):       # quality 체크
            if angle > 180:
                angle -= 360
            points.append((angle, dist))

        i += 5
    return points

# =========================================
# 메인 루프 - tiered avoidance
# =========================================
try:
    while True:
        points = get_scan_points()

        # 기본 직진
        v = 0.28      # m/s (Arduino PWM_SCALE에 맞춰 조정)
        w = 0.0       # rad/s

        if points:
            # 전방 ±120도 내 가장 가까운 장애물 찾기
            min_dist = float('inf')
            best_theta = 0.0

            for theta, dist in points:
                if abs(theta) <= 120 and dist < min_dist:
                    min_dist = dist
                    best_theta = theta

            if min_dist < float('inf'):
                abs_theta = abs(best_theta)

                # ================== 60 / 90 / 120 tier ==================
                if abs_theta <= 60:           # 정면 → 강하게
                    strength = 1.0
                elif abs_theta <= 90:         # 중간
                    strength = 0.65
                elif abs_theta <= 120:        # 옆 → 약하게
                    strength = 0.35
                else:
                    strength = 0.0

                # steering 방향: 장애물이 왼쪽(θ>0) → 오른쪽으로 회전 (w < 0)
                steering = - (best_theta / 45.0) * strength

                # 거리 10cm 이하일수록 더 강하게
                if min_dist < 0.15:
                    steering *= 1.8

                v = 0.25
                w = steering * 1.8            # angular velocity gain (튜닝 포인트)

                # w 제한
                w = max(-2.5, min(2.5, w))

        # ================== Arduino로 전송 ==================
        cmd = f"{v:.2f},{w:.3f}\n"
        arduino.write(cmd.encode())

        # 디버그 (필요하면 주석 해제)
        # print(f"θ: {best_theta:+6.1f}°  dist: {min_dist:.2f}m  v:{v:.2f} w:{w:.3f}")

        time.sleep(0.03)   # 약 30Hz 제어 주기

except KeyboardInterrupt:
    print("\n\n종료")
    arduino.write(b"0.0,0.0\n")

finally:
    lidar.write(bytes([0xA5, 0x25]))  # scan stop
    arduino.close()
    lidar.close()
