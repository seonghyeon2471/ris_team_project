import asyncio
from rplidarc1 import RPLidar          # ←←← 이 부분이 핵심 수정!
import serial
import time
import math

# ====================== 설정 ======================
LIDAR_PORT = '/dev/ttyUSB0'
ARDUINO_PORT = '/dev/serial0'

BAUD_LIDAR = 460800
BAUD_ARDUINO = 115200

# FGM / GRP 파라미터 (현장 튜닝용)
V_MAX = 0.38          # 직진 최대 속도 (m/s)
V_MIN = 0.22          # 회전 시 느린 속도
W_MAX = 2.2           # 최대 회전 각속도 (rad/s)

GAP_THRESHOLD = 400   # mm (gap 판단 기준 거리)
SAFETY_DIST = 350     # mm (정면 안전거리)

TIME_LIMIT = 58.0
# ================================================

ser = None

def send_command(v: float, w: float):
    global ser
    cmd = f"{v:.2f},{w:.2f}\n"
    try:
        ser.write(cmd.encode('utf-8'))
        ser.flush()
    except Exception as e:
        print("Serial 전송 에러:", e)

def find_best_gap(scan):
    """FGM + GRP 스타일 widest gap + reference bias"""
    valid = [p for p in scan if 200 < p["d_mm"] < 2500 and p["q"] > 8]
    if not valid:
        return 0.0

    valid.sort(key=lambda p: p["a_deg"])

    best_gap_center = 0.0
    max_score = -9999
    current_start = None
    current_start_angle = 0.0

    for i in range(len(valid)):
        if current_start is None:
            current_start = i
            current_start_angle = valid[i]["a_deg"]
            continue

        if valid[i]["d_mm"] < GAP_THRESHOLD:
            # gap 계산
            gap_width = valid[i-1]["a_deg"] - current_start_angle
            gap_center = current_start_angle + gap_width / 2.0
            
            # GRP reference bias (0도 방향 선호)
            ref_bias = abs(gap_center) * 0.35
            score = gap_width - ref_bias

            if score > max_score:
                max_score = score
                best_gap_center = gap_center

            current_start = None

    # 마지막 gap 처리
    if current_start is not None:
        gap_width = valid[-1]["a_deg"] - current_start_angle
        gap_center = current_start_angle + gap_width / 2.0
        ref_bias = abs(gap_center) * 0.35
        score = gap_width - ref_bias
        if score > max_score:
            best_gap_center = gap_center

    # 정면에 매우 가까운 장애물 → 강제 회전
    front_min = min((p["d_mm"] for p in valid if -35 < p["a_deg"] < 35), default=9999)
    if front_min < SAFETY_DIST:
        best_gap_center *= 1.6

    return best_gap_center

async def main():
    global ser
    ser = serial.Serial(ARDUINO_PORT, BAUD_ARDUINO, timeout=0.1)
    print(f"🔌 Arduino 연결: {ARDUINO_PORT}")

    lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_LIDAR)   # ← 수정된 클래스
    await lidar.connect()
    health = await lidar.healthcheck()
    print("🩺 LiDAR Health:", health)
    print("🚀 FGM-GRP 기반 장애물 회피 시작!")

    start_time = time.time()

    try:
        async for scan in lidar.simple_scan():
            if time.time() - start_time > TIME_LIMIT:
                print("⏰ 시간 제한 → 종료")
                break

            gap_angle = find_best_gap(scan)

            # steering angle → angular velocity
            w = math.radians(gap_angle) * 2.8
            w = max(min(w, W_MAX), -W_MAX)

            # 속도 결정
            v = V_MIN if abs(gap_angle) > 40 else V_MAX

            send_command(v, w)

            await asyncio.sleep(0.07)

    except Exception as e:
        print("❌ 에러:", e)
    finally:
        send_command(0.0, 0.0)
        if ser:
            ser.close()
        await lidar.shutdown()
        print("🏁 주행 종료")

if __name__ == "__main__":
    asyncio.run(main())
