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
SAFE_DIST = 0.30          # 30cm
KP = 1.2
KI = 0.05
KD = 0.4

integral = 0.0
last_error = 0.0
last_time = time.time()

print("🚀 RPLIDAR C1 + Differential Drive Start")

# LIDAR 초기화
lidar.start_motor()
time.sleep(2)
lidar.clean_input()

def send_vw(v: float, w: float):
    cmd = f"{v:.3f},{w:.3f}\n"
    arduino_ser.write(cmd.encode())
    arduino_ser.flush()

def get_lidar_points():
    """버퍼 오버플로우 방지 최적화"""
    try:
        # C1 고속 대응 설정
        scan = next(lidar.iter_scans(
            max_buf_meas=600,   # 버퍼 크기 크게 줄임
            min_len=80          # 한 번에 많은 데이터 처리
        ))
        points = []
        for _, angle, dist in scan:
            if 150 < dist < 12000:   # 0.15m ~ 12m
                rad = math.radians(angle)
                x = (dist / 1000.0) * math.cos(rad)
                y = (dist / 1000.0) * math.sin(rad)
                points.append((angle, dist/1000.0, x, y))
        return points
    except StopIteration:
        return []
    except Exception:
        lidar.clean_input()   # 에러 발생 시 즉시 버퍼 정리
        return []

def find_center_error(points):
    left_dist = 999.0
    right_dist = 999.0

    for angle, dist, _, _ in points:
        if dist < 0.15:
            continue
        if 45 < angle < 135:          # 전방
            if angle < 90:
                right_dist = min(right_dist, dist)
            else:
                left_dist = min(left_dist, dist)

    if left_dist < SAFE_DIST or right_dist < SAFE_DIST:
        return None, left_dist, right_dist

    error = left_dist - right_dist
    return error, left_dist, right_dist


try:
    while True:
        points = get_lidar_points()

        if len(points) < 50:          # 데이터 부족 시 대기
            send_vw(0.0, 0.0)
            time.sleep(0.05)
            continue

        error, left_d, right_d = find_center_error(points)

        current_time = time.time()
        dt = max(current_time - last_time, 0.01)
        last_time = current_time

        if error is None:   # 장애물
            print(f"⚠️ Obstacle! L:{left_d:.2f}m R:{right_d:.2f}m")
            send_vw(-0.18, 0.0)
            time.sleep(0.45)
            send_vw(0.0, 1.6)      # 회전
            time.sleep(0.65)
            continue

        # PID
        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error

        w = KP * error + KI * integral + KD * derivative
        w = max(min(w, 2.5), -2.5)

        v = 0.28 if min(left_d, right_d) > 0.6 else 0.16

        send_vw(v, w)

        # print(f"v:{v:.2f} w:{w:.2f} err:{error:.3f}")  # 필요시 주석 해제

        time.sleep(0.03)   # 더 빠른 루프 (버퍼 쌓임 방지)

except KeyboardInterrupt:
    print("\n🛑 프로그램 종료")

except Exception as e:
    print("에러:", e)

finally:
    send_vw(0.0, 0.0)
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    arduino_ser.close()
    print("✅ 모든 장치 정리 완료")
