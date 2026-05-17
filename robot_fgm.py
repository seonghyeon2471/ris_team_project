import serial
import time
import math
from rplidar import RPLidar

# =========================================
# SERIAL & LIDAR
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

lidar = RPLidar('/dev/ttyUSB0', baudrate=460800, timeout=1)

# =========================================
# ROBOT PARAMETERS (meter)
# =========================================
SAFE_DIST = 0.30          # 30cm 안전거리
KP = 1.2
KI = 0.05
KD = 0.4

integral = 0.0
last_error = 0.0
last_time = time.time()

print("🚀 RPLIDAR + Differential Drive Navigation Start")

# LIDAR 시작
lidar.start_motor()
time.sleep(2)
lidar.clean_input()        # 버퍼 정리

def send_vw(v: float, w: float):
    """v (m/s), w (rad/s) 전송"""
    cmd = f"{v:.3f},{w:.3f}\n"
    arduino_ser.write(cmd.encode())
    arduino_ser.flush()

def get_lidar_points():
    """LiDAR 스캔 데이터 가져오기"""
    try:
        scan = next(lidar.iter_scans(max_buf_meas=1200, min_len=40))
        points = []
        for _, angle, dist in scan:
            if 150 < dist < 12000:   # 0.15m ~ 12m
                rad = math.radians(angle)
                x = (dist / 1000.0) * math.cos(rad)
                y = (dist / 1000.0) * math.sin(rad)
                points.append((angle, dist/1000.0, x, y))
        return points
    except:
        return []

def find_center_error(points):
    """전방 좌/우 거리 차이 계산"""
    left_dist = 999.0
    right_dist = 999.0

    for angle, dist, _, _ in points:
        if dist < 0.15:
            continue
        if 45 < angle < 135:          # 전방 90° ±45°
            if angle < 90:            # 오른쪽
                right_dist = min(right_dist, dist)
            else:                     # 왼쪽
                left_dist = min(left_dist, dist)

    if left_dist < SAFE_DIST or right_dist < SAFE_DIST:
        return None, left_dist, right_dist

    error = left_dist - right_dist   # 양수 → 왼쪽이 넓음 → 오른쪽으로 correction
    return error, left_dist, right_dist


try:
    while True:
        points = get_lidar_points()

        if len(points) < 60:
            send_vw(0.0, 0.0)
            time.sleep(0.1)
            continue

        error, left_d, right_d = find_center_error(points)

        # PID dt 계산
        current_time = time.time()
        dt = max(current_time - last_time, 0.01)
        last_time = current_time

        if error is None:   # 장애물 감지
            print(f"⚠️  Obstacle! L:{left_d:.2f}m R:{right_d:.2f}m")
            send_vw(-0.18, 0.0)      # 후진
            time.sleep(0.5)
            send_vw(0.0, 1.6)        # 강한 회전
            time.sleep(0.7)
            continue

        # PID 제어
        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error

        w = KP * error + KI * integral + KD * derivative
        w = max(min(w, 2.5), -2.5)   # angular velocity 제한

        # 속도 조절 (통로가 좁으면 천천히)
        v = 0.28 if min(left_d, right_d) > 0.6 else 0.16

        send_vw(v, w)

        # 디버그 출력 (필요하면 주석 해제)
        # print(f"v:{v:.2f} w:{w:.2f} err:{error:.3f} L:{left_d:.2f} R:{right_d:.2f}")

        time.sleep(0.045)   # ≈22Hz

except KeyboardInterrupt:
    print("\n🛑 프로그램 종료")
    send_vw(0.0, 0.0)

except Exception as e:
    print("에러 발생:", e)

finally:
    send_vw(0.0, 0.0)
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    arduino_ser.close()
    print("✅ 모든 장치 정리 완료")
