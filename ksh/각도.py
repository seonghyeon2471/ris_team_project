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
print("✅ 연결 완료! (v 고정 + 지속 초강력 w 모드)")

REACTION_DIST = 0.70   # 70cm 이내에서만 반응

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
# 메인 루프 - v 고정 + w 초강력 지속
# =========================================
try:
    while True:
        points = get_scan_points()

        v = 0.22                    # ← v는 항상 이 속도로 유지 (사용자 요청)
        w = 0.0
        min_dist = float('inf')
        best_theta = 0.0

        if points:
            for theta, dist in points:
                if abs(theta) <= 120 and dist < min_dist:
                    min_dist = dist
                    best_theta = theta

            if min_dist < REACTION_DIST:          # 장애물이 감지되는 동안
                abs_theta = abs(best_theta)

                if abs_theta <= 60:      # 정면
                    strength = 1.80
                elif abs_theta <= 90:
                    strength = 1.40
                elif abs_theta <= 120:
                    strength = 1.05
                else:
                    strength = 0.0

                steering = - (best_theta / 18.0) * strength

                # 가까우면 더 강하게
                if min_dist < 0.15:
                    steering *= 2.4

                v = 0.22                                   # v는 절대 낮추지 않음
                w = steering * 13.0                        # ← w 지속 강하게 (대폭 증가)

                w = max(-7.0, min(7.0, w))

        # Arduino 전송
        cmd = f"{v:.2f},{w:.3f}\n"
        arduino.write(cmd.encode())

        # 디버깅
        if min_dist < float('inf'):
            status = "REACTION" if min_dist < REACTION_DIST else "IGNORE"
            print(f"θ: {best_theta:+6.1f}°  dist: {min_dist:.2f}m  [{status}]  v:{v:.2f}  w:{w:+.3f}")

        time.sleep(0.025)

except KeyboardInterrupt:
    print("\n\n종료")
    arduino.write(b"0.0,0.0\n")

finally:
    lidar.write(bytes([0xA5, 0x25]))
    arduino.close()
    lidar.close()
