import time
import math
import serial

# ====================== 설정 ======================
LIDAR_PORT = '/dev/ttyUSB0'
ARDUINO_PORT = '/dev/serial0'

V_MAX = 0.15
V_MIN = 0.07
W_MAX = 2.0

ALPHA = 40.0
BETA = 1.2                    # ← 직진 더 선호하게 조금 올림
GAP_THRESHOLD = 350           # ← 핵심! 작은 gap 무시 (350mm 이상만 gap으로 인정)
SAFETY_DIST = 200             # ← 정면 안전거리 (너무 늦지 않게)
MAX_OBSTACLE_DIST = 2200      # 먼 벽은 인식하되 너무 멀면 제한

TIME_LIMIT = 58.0
# ================================================

ser_lidar = None
ser_arduino = None

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

def find_best_gap_and_steering(scan_data):
    if not scan_data or len(scan_data) < 30:
        return 0.0, 9999

    points = [(ang, info["d_mm"]) for ang, info in scan_data.items()]
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
            ref_bias = abs(gap_center) * 0.45          # ← 직진 선호 강화
            score = gap_width - ref_bias
            if score > max_score:
                max_score = score
                best_gap_center = gap_center
        i += 1

    front_min = min((d for ang, d in points if -35 < ang < 35), default=9999)
    if front_min < SAFETY_DIST:
        best_gap_center *= 1.6

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
    print("🚀 RPLIDAR C1 raw 연결 완료")

    ser_lidar.write(b'\xA5\x20')   # Start Scan
    time.sleep(1.5)

    start_time = time.time()

    try:
        while True:
            if time.time() - start_time > TIME_LIMIT:
                print("\n⏰ 시간 제한 → 종료")
                break

            raw_data = ser_lidar.read(ser_lidar.in_waiting) if ser_lidar.in_waiting > 0 else b''
            scan_data = parse_raw_scan(raw_data)

            gap_angle, d_min = find_best_gap_and_steering(scan_data)

            w = math.radians(gap_angle) * 2.0          # ← 회전 강도 낮춤 (일찍 꺾는 현상 완화)
            w = max(min(w, W_MAX), -W_MAX)
            v = V_MIN if abs(gap_angle) > 40 else V_MAX

            send_command(v, w)

            print(f"Gap: {gap_angle:6.1f}° | d_min: {d_min:4.0f}mm | v:{v:.2f} w:{w:.2f}", end="\r")

            time.sleep(0.18)

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
