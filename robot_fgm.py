import serial
import time
import math
from rplidar import RPLidar

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# =========================================
# RPLIDAR C1
# =========================================
lidar = RPLidar('/dev/ttyUSB0', baudrate=460800, timeout=1)

# =========================================
# ROBOT PARAMETERS (단위: meter)
# =========================================
SAFE_DIST = 0.30             # 30cm
MIN_PATH_WIDTH = 0.45

# PID
KP = 1.2
KI = 0.05
KD = 0.4

integral = 0.0
last_error = 0.0
last_time = time.time()

print("🚀 RPLIDAR + Differential Drive Navigation Start")

lidar.start_motor()
time.sleep(2)

# ==================== 수정된 부분 ====================
lidar.clear_input()          # ← 여기서 오류 났던 부분 수정

def send_vw(v: float, w: float):
    cmd = f"{v:.3f},{w:.3f}\n"
    arduino_ser.write(cmd.encode())
    arduino_ser.flush()

def get_lidar_points():
    try:
        scan = next(lidar.iter_scans(max_buf_meas=1500, min_len=50))
        points = []
        for _, angle, dist in scan:
            if 150 < dist < 12000:          # 0.15m ~ 12m
                rad = math.radians(angle)
                x = (dist / 1000.0) * math.cos(rad)
                y = (dist / 1000.0) * math.sin(rad)
                points.append((angle, dist/1000.0, x, y))
        return points
    except:
        return []

def find_center_error(points):
    left_dist = 999.0
    right_dist = 999.0

    for angle, dist, x, y in points:
        if dist < 0.15: 
            continue

        if 45 < angle < 135:          # 전방 영역
            if angle < 90:            # 오른쪽
                right_dist = min(right_dist, dist)
            else:                     # 왼쪽
                left_dist = min(left_dist, dist)

    if left_dist < SAFE_DIST or right_dist < SAFE_DIST:
        return None, left_dist, right_dist

    error = left_dist - right_dist   # 양수 = 왼쪽이 더 넓음 → 오른쪽으로 correction
    return error, left_dist, right_dist

try:
    while True:
        points = get_lidar_points()
        if len(points) < 60:
            send_vw(0.0, 0.0)
            time.sleep(0.08)
            continue

        error, left_d, right_d = find_center_error(points)

        current_time = time.time()
        dt = max(current_time - last_time, 0.01)
        last_time = current_time

        if error is None:                     # 장애물
            print(f"⚠️  Obstacle! L:{left_d:.2f} R:{right_d:.2f}")
            send_vw(-0.18, 0.0)               # 후진
            time.sleep(0.5)
            send_vw(0.0, 1.5)                 # 강한 회전
            time.sleep(0.7)
            continue

        # PID Control
        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error

        w = KP * error + KI * integral + KD * derivative
        w = max(min(w, 2.5), -2.5)

        v = 0.28 if min(left_d, right_d) > 0.6 else 0.16   # 통로 폭에 따라 속도 조절

        send_vw(v, w)

        # 디버그 (필요시 주석 해제)
        # print(f"v:{v:.2f} w:{w:.2f} err:{error:.3f} L:{left_d:.2f} R:{right_d:.2f}")

        time.sleep(0.045)   # 약 22Hz

except KeyboardInterrupt:
    print("\n🛑 Stopping...")
    send_vw(0.0, 0.0)
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    arduino_ser.close()
except Exception as e:
    print("Error:", e)
    send_vw(0.0, 0.0)
    lidar.stop_motor()
    lidar.disconnect()
