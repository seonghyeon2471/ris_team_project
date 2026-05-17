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
print("✅ 연결 완료! (반응 거리 80cm 제한 + strength >1)")

# ================== 새로 추가 ==================
REACTION_DIST = 0.80   # 80cm 이내일 때만 회전 (필요하면 0.7~1.0으로 조정)

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
# 메인 루프
# =========================================
try:
    while True:
        points = get_scan_points()

        v = 0.20
        w = 0.0
        min_dist = float('inf')
        best_theta = 0.0

        if points:
            for theta, dist in points:
                if abs(theta) <= 120 and dist < min_dist:
                    min_dist = dist
                    best_theta = theta

            # ================== 핵심 수정 ==================
            if min_dist < REACTION_DIST:          # ← 80cm 이내일 때만 반응!
                abs_theta = abs(best_theta)

                if abs_theta <= 60:
                    strength = 1.55      # 정면 초강력
                elif abs_theta <= 90:
                    strength = 1.20
                elif abs_theta <= 120:
                    strength = 0.90
                else:
                    strength = 0.0

                steering = - (best_theta / 20.0) * strength

                # 12cm 이하 초강력
                if min_dist < 0.12:
                    steering *= 2.3

                v = 0.15 if abs(steering) > 1.0 else 0.20
                w = steering * 6.2

                w = max(-5.0, min(5.0, w))

        # Arduino 전송
        cmd = f"{v:.2f},{w:.3f}\n"
        arduino.write(cmd.encode())

        # 디버깅 (매우 유용함)
        if min_dist < float('inf'):
            status = "REACTION" if min_dist < REACTION_DIST else "IGNORE"
            print(f"θ: {best_theta:+6.1f}°  dist: {min_dist:.2f}m  [{status}]  w:{w:+.3f}")

        time.sleep(0.025)

except KeyboardInterrupt:
    print("\n\n종료")
    arduino.write(b"0.0,0.0\n")

finally:
    lidar.write(bytes([0xA5, 0x25]))
    arduino.close()
    lidar.close()
