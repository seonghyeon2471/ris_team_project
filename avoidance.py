import asyncio
from rplidarc1.scanner import RPLidar
import serial
import time
import math

# ====================== 설정 ======================
LIDAR_PORT = '/dev/ttyUSB0'
ARDUINO_PORT = '/dev/serial0'

BAUD_LIDAR = 460800
BAUD_ARDUINO = 115200

# GRP 파라미터 (논문 기반)
V_MAX = 0.15
V_MIN = 0.07
W_MAX = 2.0

# GRP weighted steering 파라미터 (논문 α, β)
ALPHA = 40.0   # d_min에 따른 gap 가중치 (논문에서 π/d_min 역할)
BETA = 1.0     # reference bias (논문 β)

GLOBAL_GOAL_DISTANCE = 3.5   # m (10m → 경기장 크기 3.1m에 맞춤)

GAP_THRESHOLD = 200   # mm
SAFETY_DIST = 70     # mm

TIME_LIMIT = 58.0
# ================================================

ser = None

def send_command(v: float, w: float):
    global ser
    cmd = f"{v:.2f},{w:.2f}\n"
    try:
        ser.write(cmd.encode('utf-8'))
        ser.flush()
    except:
        pass

def find_best_gap_and_steering(scan_data):
    """GRP 논문 스타일: gap + reference bias weighted steering"""
    if not scan_data:
        return 0.0, 0.0

    points = [(ang, info["d_mm"]) for ang, info in scan_data.items() 
              if 200 < info.get("d_mm", 0) < 3000 and info.get("q", 0) > 8]

    if not points:
        return 0.0, 0.0

    points.sort(key=lambda x: x[0])

    best_gap_center = 0.0
    max_score = -9999
    i = 0
    n = len(points)

    while i < n:
        start_idx = i
        while i < n and points[i][1] >= GAP_THRESHOLD:
            i += 1
        if i > start_idx:
            gap_start = points[start_idx][0]
            gap_end = points[i-1][0]
            gap_width = gap_end - gap_start
            gap_center = gap_start + gap_width / 2.0

            # GRP reference bias (global goal 방향 = 0도 선호)
            ref_bias = abs(gap_center) * 0.4
            score = gap_width - ref_bias

            if score > max_score:
                max_score = score
                best_gap_center = gap_center

        i += 1

    # 정면 안전거리 체크
    front_min = min((d for ang, d in points if -35 < ang < 35), default=9999)
    if front_min < SAFETY_DIST:
        best_gap_center *= 1.8

    # GRP weighted steering (논문 공식)
    d_min = front_min if front_min < 2000 else 2000
    phi_gap = best_gap_center
    phi_ref = 0.0   # global goal 방향 (현재는 직진)

    weight_gap = ALPHA / d_min
    phi_s = (weight_gap * phi_gap + BETA * phi_ref) / (weight_gap + BETA)

    return phi_s, d_min

async def main():
    global ser
    ser = serial.Serial(ARDUINO_PORT, BAUD_ARDUINO, timeout=0.1)
    print(f"🔌 Arduino 연결: {ARDUINO_PORT}")

    lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_LIDAR)
    print("🚀 LiDAR 초기화 완료 - GRP 기반 주행 시작")

    start_time = time.time()

    try:
        # LiDAR 스캔 백그라운드 시작
        scan_task = asyncio.create_task(lidar.simple_scan(make_return_dict=True))

        while True:
            if time.time() - start_time > TIME_LIMIT:
                print("⏰ 시간 제한 → 종료")
                break

            scan_data = lidar.output_dict.copy() if hasattr(lidar, 'output_dict') else {}

            gap_angle, d_min = find_best_gap_and_steering(scan_data)

            # steering angle → angular velocity
            w = math.radians(gap_angle) * 3.0
            w = max(min(w, W_MAX), -W_MAX)

            v = V_MIN if abs(gap_angle) > 35 else V_MAX

            send_command(v, w)

            await asyncio.sleep(0.06)

    except asyncio.CancelledError:
        print("🛑 중단됨")
    except Exception as e:
        print("❌ 에러:", e)
    finally:
        send_command(0.0, 0.0)
        if ser and ser.is_open:
            ser.close()
        try:
            lidar.reset()
        except:
            pass
        print("🏁 주행 종료")

if __name__ == "__main__":
    asyncio.run(main())
