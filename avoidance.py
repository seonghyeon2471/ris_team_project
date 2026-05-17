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

V_MAX = 0.38
V_MIN = 0.22
W_MAX = 2.2

GAP_THRESHOLD = 400
SAFETY_DIST = 350

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

def find_best_gap(scan_data):
    if not scan_data:
        return 0.0

    points = []
    for ang, info in scan_data.items():
        if 200 < info.get("d_mm", 0) < 2500 and info.get("q", 0) > 8:
            points.append((ang, info["d_mm"]))

    if not points:
        return 0.0

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
            ref_bias = abs(gap_center) * 0.35
            score = gap_width - ref_bias
            if score > max_score:
                max_score = score
                best_gap_center = gap_center
        i += 1

    front_min = min((d for ang, d in points if -35 < ang < 35), default=9999)
    if front_min < SAFETY_DIST:
        best_gap_center *= 1.6

    return best_gap_center

async def main():
    global ser
    ser = serial.Serial(ARDUINO_PORT, BAUD_ARDUINO, timeout=0.1)
    print(f"🔌 Arduino 연결: {ARDUINO_PORT}")

    lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_LIDAR)
    print("🚀 LiDAR 초기화 완료")

    start_time = time.time()

    try:
        async for scan in lidar.simple_scan(make_return_dict=True):
            if time.time() - start_time > TIME_LIMIT:
                print("⏰ 시간 제한 → 종료")
                break

            gap_angle = find_best_gap(scan)

            w = math.radians(gap_angle) * 2.8
            w = max(min(w, W_MAX), -W_MAX)

            v = V_MIN if abs(gap_angle) > 40 else V_MAX

            send_command(v, w)

            await asyncio.sleep(0.07)

    except asyncio.CancelledError:
        print("🛑 Ctrl+C로 중단됨")
    except Exception as e:
        print("❌ 에러:", e)
    finally:
        send_command(0.0, 0.0)
        if ser and ser.is_open:
            ser.close()
        try:
            await lidar.shutdown()
        except:
            pass
        print("🏁 주행 종료")

if __name__ == "__main__":
    asyncio.run(main())
