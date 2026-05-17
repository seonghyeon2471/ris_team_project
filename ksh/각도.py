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
print("✅ LiDAR + Arduino 연결 완료! (강한 회전 모드)")

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
# 메인 루프 - 강한 회전 버전
# =========================================
try:
    while True:
        points = get_scan_points()

        v = 0.22      # 기본 속도 (조금 낮춤)
        w = 0.0

        if points:
            min_dist = float('inf')
            best_theta = 0.0

            for theta, dist in points:
                if abs(theta) <= 120 and dist < min_dist:
                    min_dist = dist
                    best_theta = theta

            if min_dist < float('inf'):
                abs_theta = abs(best_theta)

                # ================== 강한 tiered steering ==================
                if abs_theta <= 60:      # 정면
                    strength = 1.0
                elif abs_theta <= 90:    # 중간
                    strength = 0.85
                elif abs_theta <= 120:   # 옆
                    strength = 0.55
                else:
                    strength = 0.0

                # steering 강도 대폭 증가
                steering = - (best_theta / 35.0) * strength

                # 거리 15cm 이하이면 더 강하게
                if min_dist < 0.15:
                    steering *= 1.8

                v = 0.20 if abs(steering) > 0.8 else 0.22
                w = steering * 3.0          # ← 여기서 크게 올림 (1.8 → 3.0)

                w = max(-3.0, min(3.0, w))   # w 제한

        # Arduino로 전송
        cmd = f"{v:.2f},{w:.3f}\n"
        arduino.write(cmd.encode())

        # 실시간 디버깅 (터미널에 출력됨)
        if points and min_dist < float('inf'):
            print(f"θ: {best_theta:+6.1f}°  dist: {min_dist:.2f}m  v:{v:.2f} w:{w:+.3f}")

        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n\n종료")
    arduino.write(b"0.0,0.0\n")

finally:
    lidar.write(bytes([0xA5, 0x25]))
    arduino.close()
    lidar.close()
