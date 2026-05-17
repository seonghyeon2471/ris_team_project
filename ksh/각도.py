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
print("✅ 거리별 tier 회피 시작 (30cm / 20cm / 10cm)")

# ================== 거리 tier ==================
TIER_30CM = 0.30
TIER_20CM = 0.20
TIER_10CM = 0.10

BASE_V = 0.22
MAX_W  = 7.5

# smoothing
smoothed_w = 0.0
ALPHA = 0.45

def get_scan_points():
    chunk = lidar.read(1400)
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
        dist = ((d2 << 8 | d1) / 4.0) / 1000.0   # m 단위

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

        v = BASE_V
        w = 0.0
        min_dist = 999.0
        best_theta = 0.0

        # 전방 ±120도 내 가장 가까운 장애물 찾기
        for theta, dist in points:
            if abs(theta) <= 120 and dist < min_dist:
                min_dist = dist
                best_theta = theta

        # ================== 거리별 + 좌우 반대 방향 ==================
        if min_dist < TIER_30CM:
            abs_theta = abs(best_theta)
            
            # 좌우 판단 및 반대 방향 회전
            if best_theta > 0:      # 왼쪽 장애물 → 오른쪽으로 회전 (w < 0)
                direction = -1
            else:                   # 오른쪽 장애물 → 왼쪽으로 회전 (w > 0)
                direction = 1

            # 거리 tier별 강도
            if min_dist < TIER_10CM:
                strength = 1.0      # 10cm 이하 = 초강력
            elif min_dist < TIER_20CM:
                strength = 0.75     # 20cm
            else:
                strength = 0.45     # 30cm

            target_w = direction * 9.5 * strength

            # smoothing 적용
            smoothed_w = ALPHA * target_w + (1 - ALPHA) * smoothed_w
            w = smoothed_w

            # 매우 가까우면 속도 낮춤
            if min_dist < TIER_10CM:
                v = 0.10
            elif min_dist < TIER_20CM:
                v = 0.16

        else:
            # 장애물 없으면 w 천천히 0으로
            smoothed_w *= 0.65

        w = max(-MAX_W, min(MAX_W, w))

        # Arduino로 전송
        cmd = f"{v:.2f},{w:.3f}\n"
        arduino.write(cmd.encode())

        # 실시간 출력
        if min_dist < TIER_30CM:
            side = "LEFT" if best_theta > 0 else "RIGHT"
            tier = "10cm↑" if min_dist < TIER_10CM else "20cm↑" if min_dist < TIER_20CM else "30cm↑"
            print(f"{side} [{tier}] dist:{min_dist:.2f}m  θ:{best_theta:+5.1f}°  v:{v:.2f}  w:{w:+.3f}")
        else:
            print(f"CLEAR  dist:{min_dist:.2f}m  v:{v:.2f}  w:{w:+.3f}")

        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n종료")
    arduino.write(b"0.0,0.0\n")

finally:
    lidar.write(bytes([0xA5, 0x25]))
    arduino.close()
    lidar.close()
