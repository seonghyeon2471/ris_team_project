import time
import math
import serial
from collections import deque

# ====================== 설정 (정면 충돌 방지 강화) ======================
LIDAR_PORT = '/dev/ttyUSB0'
ARDUINO_PORT = '/dev/serial0'

V_MAX = 0.14
V_MIN = 0.06
W_MAX = 2.2

ALPHA = 50.0                    # gap 가중치 ↑
BETA = 1.2
GAP_THRESHOLD = 400
SAFETY_DIST = 450               # ← 정면 안전거리 크게 증가 (45cm부터 반응)
SAFETY_WALL_DIST = 280
MAX_OBSTACLE_DIST = 2200

TIME_LIMIT = 58.0
INITIAL_STRAIGHT_TIME = 2.0

MEMORY_SCANS = 8
# ================================================

ser_lidar = None
ser_arduino = None
scan_memory = deque(maxlen=MEMORY_SCANS)

def send_command(v: float, w: float):
    global ser_arduino
    cmd = f"{v:.2f},{w:.2f}\n"
    try:
        ser_arduino.write(cmd.encode('utf-8'))
        ser_arduino.flush()
    except:
        pass

def parse_raw_scan(raw_bytes):
    scan_data = {}
    i = 0
    while i + 5 <= len(raw_bytes):
        if (raw_bytes[i] & 0x03) == 0x01:
            quality = raw_bytes[i] >> 2
            angle_q6 = ((raw_bytes[i+1] << 7) | (raw_bytes[i] >> 1)) & 0x7FFF
            distance_q2 = (raw_bytes[i+3] << 8) | raw_bytes[i+2]
            angle = angle_q6 / 64.0
            distance = distance_q2 / 4.0
            if 200 < distance < MAX_OBSTACLE_DIST and quality > 8:
                scan_data[angle] = {"d_mm": distance, "q": quality}
        i += 5
    return scan_data

def merge_memory():
    merged = {}
    for scan in scan_memory:
        for ang, info in scan.items():
            if ang not in merged or info["d_mm"] < merged[ang]["d_mm"]:
                merged[ang] = info
    return merged

def emergency_front_avoid(scan_data):
    """정면이 막혔을 때 가장 강력한 회피"""
    if not scan_data:
        return None

    front = [info["d_mm"] for ang, info in scan_data.items() if -40 < ang < 40]
    front_min = min(front) if front else 9999

    if front_min < SAFETY_DIST:
        left = min((info["d_mm"] for ang, info in scan_data.items() if 30 < ang < 120), default=9999)
        right = min((info["d_mm"] for ang, info in scan_data.items() if -120 < ang < -30), default=9999)

        if left > right:
            print(f"🚨 정면 막힘! ({front_min:.0f}mm) → 강 좌회전")
            return 2.2
        else:
            print(f"🚨 정면 막힘! ({front_min:.0f}mm) → 강 우회전")
            return -2.2
    return None

def find_best_gap_and_steering(scan_data, elapsed_time):
    # 1. 정면 긴급 회피 (최우선)
    emergency = emergency_front_avoid(scan_data)
    if emergency is not None:
        return emergency, 0

    # 2. 벽 회피
    if elapsed_time > INITIAL_STRAIGHT_TIME:
        left_wall = min((info["d_mm"] for ang, info in scan_data.items() if -125 < ang < -45), default=9999)
        right_wall = min((info["d_mm"] for ang, info in scan_data.items() if 45 < ang < 125), default=9999)
        if left_wall < SAFETY_WALL_DIST:
            return -2.0, 0
        if right_wall < SAFETY_WALL_DIST:
            return 2.0, 0

    # 3. 정상 GRP + memory
    merged = merge_memory()
    current_merged = {**merged, **scan_data}

    if not current_merged or len(current_merged) < 30:
        return 0.0, 9999

    points = [(ang, info["d_mm"]) for ang, info in current_merged.items()]
    points.sort(key=lambda x: x[0])

    best_gap_center = 0.0
    max_score = -9999
    i = 0
    n = len(points)

    while i < n:
        start_idx = i
        while i < n and points[i][1] >= GAP_THRESHOLD:
            i += 1
        if i > start_idx + 3:   # 너무 좁은 gap은 무시
            gap_start = points[start_idx][0]
            gap_end = points[i-1][0]
            gap_width = gap_end - gap_start
            gap_center = gap_start + gap_width / 2.0
            ref_bias = abs(gap_center) * 0.55
            score = gap_width - ref_bias
            if score > max_score:
                max_score = score
                best_gap_center = gap_center
        i += 1

    front_min = min((d for ang, d in points if -40 < ang < 40), default=9999)
    if front_min < SAFETY_DIST:
        best_gap_center *= 2.0   # 정면 막히면 steering 강하게

    d_min = front_min if front_min < 2000 else 2000
    phi_gap = best_gap_center
    phi_ref = 0.0
    weight_gap = ALPHA / d_min
    phi_s = (weight_gap * phi_gap + BETA * phi_ref) / (weight_gap + BETA)

    return phi_s, d_min

def main():
    global ser_lidar, ser_arduino

    ser_arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
    print(f"🔌 Arduino 연결: {ARDUINO_PORT}")

    ser_lidar = serial.Serial(LIDAR_PORT, 460800, timeout=0.1)
    print("🚀 RPLIDAR + 기억하면서 주행 (정면 충돌 방지 강화)")

    ser_lidar.write(b'\xA5\x20')
    time.sleep(1.5)

    start_time = time.time()

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > TIME_LIMIT:
                print("\n⏰ 시간 제한 → 종료")
                break

            raw_data = ser_lidar.read(ser_lidar.in_waiting) if ser_lidar.in_waiting > 0 else b''
            current_scan = parse_raw_scan(raw_data)

            if current_scan:
                scan_memory.append(current_scan)

            gap_angle, d_min = find_best_gap_and_steering(current_scan, elapsed)

            w = math.radians(gap_angle) * 2.4
            w = max(min(w, W_MAX), -W_MAX)
            v = V_MIN if abs(gap_angle) > 40 else V_MAX

            send_command(v, w)

            print(f"Gap: {gap_angle:6.1f}° | d_min: {d_min:4.0f}mm | v:{v:.2f} w:{w:.2f}", end="\r")

            time.sleep(0.16)

    except KeyboardInterrupt:
        print("\n🛑 중단됨")
    except Exception as e:
        print("\n❌ 에러:", e)
    finally:
        send_command(0.0, 0.0)
        if ser_arduino and ser_arduino.is_open:
            ser_arduino.close()
        if ser_lidar and ser_lidar.is_open:
            ser_lidar.write(b'\xA5\x25')
            ser_lidar.close()
        print("\n🏁 주행 종료")

if __name__ == "__main__":
    main()
