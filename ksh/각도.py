import serial
import time

# =========================================
# SERIAL
# =========================================
ARDUINO_PORT = "/dev/serial0"
LIDAR_PORT   = "/dev/ttyUSB0"

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar   = serial.Serial(LIDAR_PORT,   460800, timeout=0.05)

print("LiDAR 시작 중...")
lidar.write(bytes([0xA5, 0x40]))
time.sleep(1)
lidar.write(bytes([0xA5, 0x20]))
time.sleep(2)
lidar.reset_input_buffer()
print("✅ 거리별 tier 회피 (30/20/10cm) - 999 제거 버전")

TIER_30 = 0.30
TIER_20 = 0.20
TIER_10 = 0.10

BASE_V = 0.22
MAX_W  = 8.0

smoothed_w = 0.0
ALPHA = 0.45

def get_scan_points():
    chunk = lidar.read(1600)          # 조금 더 많이 읽기
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
        dist = ((d2 << 8 | d1) / 4.0) / 1000.0

        if 0.08 < dist < 5.0 and (q & 0x01):
            if angle > 180:
                angle -= 360
            points.append((angle, dist))
        i += 5
    return points

try:
    while True:
        points = get_scan_points()

        v = BASE_V
        w = 0.0
        min_dist = float('inf')      # ← 999 대신 inf 사용
        best_theta = 0.0

        for theta, dist in points:
            if abs(theta) <= 120 and dist < min_dist:
                min_dist = dist
                best_theta = theta

        # ================== 거리별 tier ==================
        if min_dist < TIER_30:
            direction = -1 if best_theta > 0 else 1   # 왼쪽→오른쪽, 오른쪽→왼쪽

            if min_dist < TIER_10:
                strength = 1.0
            elif min_dist < TIER_20:
                strength = 0.75
            else:
                strength = 0.45

            target_w = direction * 10.5 * strength

            smoothed_w = ALPHA * target_w + (1 - ALPHA) * smoothed_w
            w = smoothed_w

            if min_dist < TIER_10:
                v = 0.10
            elif min_dist < TIER_20:
                v = 0.15
        else:
            smoothed_w *= 0.65

        w = max(-MAX_W, min(MAX_W, w))

        # ================== 깔끔한 출력 ==================
        if min_dist < TIER_30:
            side = "LEFT" if best_theta > 0 else "RIGHT"
            tier = "10cm↑" if min_dist < TIER_10 else "20cm↑" if min_dist < TIER_20 else "30cm↑"
            print(f"{side} [{tier}] dist:{min_dist:.2f}m  θ:{best_theta:+6.1f}°  v:{v:.2f}  w:{w:+.3f}")
        else:
            print(f"CLEAR (no obstacle in front)  v:{v:.2f}  w:{w:+.3f}")

        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n종료")
    arduino.write(b"0.0,0.0\n")

finally:
    lidar.write(bytes([0xA5, 0x25]))
    arduino.close()
    lidar.close()
