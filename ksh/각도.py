import serial
import time

# =========================================
# SERIAL PORTS
# =========================================
ARDUINO_PORT = "/dev/serial0"
LIDAR_PORT   = "/dev/ttyUSB0"

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar   = serial.Serial(LIDAR_PORT,   460800, timeout=0.05)

# LIDAR 시작
print("LiDAR 시작 중...")
lidar.write(bytes([0xA5, 0x40]))
time.sleep(1)
lidar.write(bytes([0xA5, 0x20]))
time.sleep(2)
lidar.reset_input_buffer()
print("✅ 연결 완료! (v 고정 + 흔들림 방지 smoothing 모드)")

REACTION_DIST = 0.70

# smoothing 변수
smoothed_w = 0.0
SMOOTH_ALPHA = 0.35   # 0.3~0.5 사이에서 테스트 (작을수록 더 부드러움)

def get_scan_points():
    chunk = lidar.read(1200)
    points = []
    i = 0
    while i + 4 < len(chunk):
        q = chunk[i]
        a1 = chunk[i+1]
        a2 = chunk[i+2]
        d1 = chunk[i+3]
        d2 = chunk[i+4]

        angle_raw = (a2 << 8) | a1
        angle = (angle_raw / 64.0) % 360.0
        distance_mm = (d2 << 8) | d1
        dist = distance_mm / 4.0 / 1000.0

        if 0.08 < dist < 5.0 and (q & 0x01):
            if angle > 180:
                angle -= 360
            points.append((angle, dist))
        i += 5
    return points

# =========================================
# 메인 루프 - 흔들림 방지 버전
# =========================================
try:
    while True:
        points = get_scan_points()

        v = 0.22                    # v 완전 고정
        w = 0.0
        min_dist = float('inf')
        best_theta = 0.0

        if points:
            for theta, dist in points:
                if abs(theta) <= 120 and dist < min_dist:
                    min_dist = dist
                    best_theta = theta

            if min_dist < REACTION_DIST:
                abs_theta = abs(best_theta)

                # 작은 각도는 무시 (deadzone)
                if abs_theta < 8:
                    strength = 0.0
                elif abs_theta <= 60:
                    strength = 1.65
                elif abs_theta <= 90:
                    strength = 1.30
                elif abs_theta <= 120:
                    strength = 1.00
                else:
                    strength = 0.0

                steering = - (best_theta / 20.0) * strength

                if min_dist < 0.15:
                    steering *= 2.2

                # ================== smoothing 적용 ==================
                new_w = steering * 8.5                     # gain 약간 낮춤
                smoothed_w = SMOOTH_ALPHA * new_w + (1 - SMOOTH_ALPHA) * smoothed_w
                w = smoothed_w

                w = max(-6.5, min(6.5, w))

        # Arduino 전송
        cmd = f"{v:.2f},{w:.3f}\n"
        arduino.write(cmd.encode())

        # 디버깅
        if min_dist < float('inf'):
            status = "REACTION" if min_dist < REACTION_DIST else "IGNORE"
            print(f"θ: {best_theta:+6.1f}°  dist: {min_dist:.2f}m  [{status}]  v:{v:.2f}  w:{w:+.3f}")

        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n\n종료")
    arduino.write(b"0.0,0.0\n")

finally:
    lidar.write(bytes([0xA5, 0x25]))
    arduino.close()
    lidar.close()
