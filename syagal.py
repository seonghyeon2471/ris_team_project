import serial
import math
import time
import threading
import numpy as np
import cv2

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)
print("LIDAR START")

# =========================================
# PARAMETERS — LiDAR / 회피
# =========================================
MAX_SPEED      = 0.30
MIN_SPEED      = 0.09
MAX_W          = 0.9
THRESH_30      = 32.0
THRESH_20      = 22.0
THRESH_10      = 12.0
FRONT_CHECK_RANGE = 45

EMA_ALPHA = 0.35
MEDIAN_K  = 2
scan_data = np.full(360, 150.0, dtype=np.float32)

# =========================================
# PARAMETERS — 색상 인식
# =========================================
# HSV 범위: [lower, upper]
COLOR_RANGES = {
    "black":  ([0,   0,   0  ], [180, 60,  60 ]),
    "red":    ([0,   120, 70 ], [10,  255, 255]),
    "yellow": ([20,  100, 100], [35,  255, 255]),
    "blue":   ([100, 120, 70 ], [130, 255, 255]),
}

# 주차 순서: 검정(출발) → 빨강 → 노랑 → 파랑
MISSION_SEQUENCE = ["red", "yellow", "blue"]
MIN_CONTOUR_AREA = 3000   # 주차구역으로 인정할 최소 픽셀 면적
PARK_CONFIRM_COUNT = 15   # 연속 N프레임 인식 시 정차 확정
PARK_DIST_THRESH   = 20.0 # 정차 판단용 전방 거리 임계값 (cm)
PARK_DURATION      = 2.0  # 정차 유지 시간 (초)

# =========================================
# 상태 머신
# =========================================
STATE_NAVIGATE = "NAVIGATE"   # 다음 주차구역으로 이동 중
STATE_PARK     = "PARK"       # 주차구역 진입 · 정차 중
STATE_DONE     = "DONE"       # 미션 완료

state          = STATE_NAVIGATE
mission_index  = 0            # 현재 목표 인덱스 (0=red, 1=yellow, 2=blue)
confirm_count  = 0            # 색상 연속 인식 카운터
detected_color = None         # 카메라가 감지한 현재 색상
color_lock     = threading.Lock()

# =========================================
# UTIL — LiDAR 필터
# =========================================
def apply_ema(angle, new_dist_cm):
    if not isinstance(new_dist_cm, (int, float)) or new_dist_cm <= 0:
        return
    scan_data[angle] = (1.0 - EMA_ALPHA) * scan_data[angle] + EMA_ALPHA * new_dist_cm

def apply_median_filter():
    k = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        indices = [(i + d) % 360 for d in range(-k, k + 1)]
        values  = np.sort(scan_data[indices])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

def get_front_min():
    indices = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE + 1) % 360
    return float(np.min(scan_data[indices]))

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# UTIL — 색상 인식 (카메라 스레드)
# =========================================
def color_detect_loop():
    """별도 스레드에서 카메라 프레임을 계속 읽고 detected_color 갱신"""
    global detected_color
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.03)
            continue

        # 하단 1/3 영역만 사용 (바닥 주차구역 집중)
        h = frame.shape[0]
        roi = frame[int(h * 2 / 3):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        found = None
        max_area = 0
        for color_name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5)))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7)))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                area = cv2.contourArea(max(contours, key=cv2.contourArea))
                if area > MIN_CONTOUR_AREA and area > max_area:
                    max_area = area
                    found    = color_name

        with color_lock:
            detected_color = found

        time.sleep(0.03)  # ~30 fps

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# 주차 동작
# =========================================
def do_park():
    """천천히 전진하며 주차구역 중앙 진입 후 정차"""
    print("  >> 주차 진입 — 천천히 전진")
    send_cmd(0.08, 0.0)
    time.sleep(0.8)
    stop_robot()
    print(f"  >> 정차 완료 ({PARK_DURATION}초 대기)")
    time.sleep(PARK_DURATION)

# =========================================
# 메인 루프
# =========================================
# 카메라 스레드 시작
cam_thread = threading.Thread(target=color_detect_loop, daemon=True)
cam_thread.start()
print("CAMERA THREAD START")
print(f"MISSION START: {MISSION_SEQUENCE}")

try:
    while True:
        # ── LiDAR 패킷 파싱 ──────────────────────
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - s_flag) \
                or (raw[1] & 0x01) != 1 \
                or (raw[0] >> 2) < 3:
            continue

        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0

        if 3 < dist_cm < 150:
            apply_ema(angle, dist_cm)

        if s_flag != 1:
            continue

        apply_median_filter()

        # ── 미션 완료 ─────────────────────────────
        if state == STATE_DONE:
            stop_robot()
            print("✅ 모든 주차구역 완료! 정지.")
            break

        front_min = get_front_min()

        # ── PARK 상태: 색상 확인 후 정차 ─────────
        if state == STATE_PARK:
            target_color = MISSION_SEQUENCE[mission_index]

            with color_lock:
                cur_color = detected_color

            if cur_color == target_color:
                confirm_count += 1
                print(f"  [{target_color}] 감지 중... ({confirm_count}/{PARK_CONFIRM_COUNT})")
            else:
                confirm_count = max(0, confirm_count - 1)

            if confirm_count >= PARK_CONFIRM_COUNT:
                # 정차 실행
                stop_robot()
                do_park()
                print(f"✅ [{target_color}] 주차 완료!")
                confirm_count = 0
                mission_index += 1

                if mission_index >= len(MISSION_SEQUENCE):
                    state = STATE_DONE
                    print("🏁 미션 전체 완료!")
                else:
                    next_target = MISSION_SEQUENCE[mission_index]
                    print(f"➡️  다음 목표: [{next_target}]")
                    state = STATE_NAVIGATE
                continue

            # 주차구역 탐색 중 — 천천히 전진 (장애물 회피 유지)
            if front_min < THRESH_10:
                direction = choose_avoid_direction()
                send_cmd(MIN_SPEED, direction * MAX_W)
            elif front_min < THRESH_20:
                direction = choose_avoid_direction()
                send_cmd(0.10, direction * 0.7)
            else:
                send_cmd(0.12, 0.0)   # 주차 탐색 중엔 느리게
            continue

        # ── NAVIGATE 상태: 장애물 회피하며 이동 ──
        target_color = MISSION_SEQUENCE[mission_index]

        # 목표 색상이 보이기 시작하면 PARK 상태로 전환
        with color_lock:
            cur_color = detected_color

        if cur_color == target_color and front_min < THRESH_30:
            print(f"🔍 [{target_color}] 발견! PARK 모드 전환")
            state         = STATE_PARK
            confirm_count = 0
            continue

        # 일반 장애물 회피 (기존 로직 유지)
        if front_min < THRESH_10:
            direction = choose_avoid_direction()
            v, w = MIN_SPEED, direction * MAX_W
            print(f"🚨 VERY CLOSE! front={front_min:.1f}cm → STRONG TURN")
        elif front_min < THRESH_20:
            direction = choose_avoid_direction()
            v, w = 0.12, direction * 0.8
            print(f"⚠️  CRITICAL front={front_min:.1f}cm → STRONG TURN")
        elif front_min < THRESH_30:
            direction = choose_avoid_direction()
            v, w = 0.15, direction * 0.7
            print(f"⚡ WARNING front={front_min:.1f}cm → MEDIUM TURN")
        else:
            v, w = MAX_SPEED, 0.0

        send_cmd(v, w)

except KeyboardInterrupt:
    print("STOP by user")
finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    print("SHUTDOWN COMPLETE")
