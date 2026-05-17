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

V_MAX = 0.15
V_MIN = 0.07
W_MAX = 2.0

ALPHA = 40.0
BETA = 1.0
GAP_THRESHOLD = 200
SAFETY_DIST = 70

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
    """None 방어 코드 추가"""
    if scan_data is None or not isinstance(scan_data, dict) or len(scan_data) < 10:
        return 0.0, 9999   # 스캔 데이터가 아직 준비되지 않았으면 직진

    points = []
    for ang, info in scan_data.items():
        if not isinstance(info, dict):
            continue
        d_mm = info.get("d_mm", 0)
        q = info.get("q", 0)
        if 200 < d_mm < 3000 and q > 8:
            points.append((ang, d_mm))

    if not points:
        return 0.0, 9999

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
            ref_bias = abs(gap_center) * 0.4
            score = gap_width - ref_bias
            if score > max_score:
                max_score = score
                best_gap_center = gap_center
        i += 1

    front_min = min((d for ang, d in points if -35 < ang < 35), default=9999)
    if front_min < SAFETY_DIST:
        best_gap_center *= 1.8

    d_min = front_min if front_min < 2000 else 2000
    phi_gap = best_gap_center
    phi_ref = 0.0
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
        asyncio.create_task(lidar.simple_scan(make_return_dict=True))

        # 스캔 데이터가 준비될 때까지 잠시 대기
        await asyncio.sleep(1.0)

        while True:
            if time.time() - start_time > TIME_LIMIT:
                print("\n⏰ 시간 제한 → 종료")
                break

            scan_data = lidar.output_dict.copy() if hasattr(lidar, 'output_dict') and lidar.output_dict is not None else {}

            gap_angle, d_min = find_best_gap_and_steering(scan_data)

            w = math.radians(gap_angle) * 3.0
            w = max(min(w, W_MAX), -W_MAX)

            v = V_MIN if abs(gap_angle) > 35 else V_MAX

            send_command(v, w)

            # 실시간 디버그 (필요하면 주석 처리)
            print(f"Gap: {gap_angle:6.1f}° | d_min: {d_min:4.0f}mm | v:{v:.2f} w:{w:.2f}", end="\r")

            await asyncio.sleep(0.06)

    except asyncio.CancelledError:
        print("\n🛑 중단됨")
    except Exception as e:
        print("\n❌ 에러:", e)
    finally:
        send_command(0.0, 0.0)
        if ser and ser.is_open:
            ser.close()
        try:
            lidar.reset()
        except:
            pass
        print("\n🏁 주행 종료")

if __name__ == "__main__":
    asyncio.run(main())
