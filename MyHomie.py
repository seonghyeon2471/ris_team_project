import serial
import math
import time
import numpy as np
import cv2

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

lidar_ser = serial.Serial(
    "/dev/ttyUSB0",
    460800,
    timeout=0.1
)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

FRAME_W = 320
FRAME_H = 240

# 색상이 "아래로 사라졌다"고 판단할 Y 기준선
# 화면 하단 20% 이내에 색상 중심이 있으면 → 도착
BOTTOM_THRESHOLD = int(FRAME_H * 0.80)

# 색상이 충분히 크면 추적 시작 (노이즈 제거)
MIN_COLOR_AREA = 800

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
# PARAMETERS
# =========================================
MAX_SPEED   = 0.28
MIN_SPEED   = 0.09
TRACK_SPEED = 0.15      # 색상 추적 중 전진 속도
MAX_W       = 0.9

THRESH_30   = 32.0
THRESH_20   = 22.0
THRESH_10   = 12.0
FRONT_CHECK_RANGE = 45

# 조향 P 게인 (픽셀 오차 → 각속도)
KP_STEER = 0.004

# =========================================
# HSV 색상 범위
# =========================================
COLOR_RANGES = {
    "RED": [
        (np.array([0,   120,  70]),  np.array([10,  255, 255])),
        (np.array([170, 120,  70]),  np.array([180, 255, 255])),
    ],
    "YELLOW": [
        (np.array([20,  120,  70]),  np.array([35,  255, 255])),
    ],
    "BLUE": [
        (np.array([100, 120,  70]),  np.array([130, 255, 255])),
    ],
}

# =========================================
# 상태머신
# =========================================
STATE_SEQUENCE = [
    "FIND_RED",
    "PARK_RED",
    "FIND_YELLOW",
    "PARK_YELLOW",
    "FIND_BLUE",
    "PARK_BLUE",
    "DONE",
]

state_idx = 0

def current_state():
    return STATE_SEQUENCE[state_idx]

def next_state():
    global state_idx
    state_idx += 1
    print(f">>> STATE: {current_state()}")

# =========================================
# FILTER
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K  = 2

scan_data = np.full(360, 150.0, dtype=np.float32)

def apply_ema(angle, new_dist_cm):
    if not isinstance(new_dist_cm, (int, float)) or new_dist_cm <= 0:
        return
    scan_data[angle] = (
        (1.0 - EMA_ALPHA) * scan_data[angle]
        + EMA_ALPHA * new_dist_cm
    )

def apply_median_filter():
    k = MEDIAN_K
    window = 2 * k + 1
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        values = np.sort(scan_data[idx])
        filtered[i] = values[window // 2]
    scan_data[:] = filtered

def get_front_min():
    idx = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE + 1) % 360
    return float(np.min(scan_data[idx]))

def choose_avoid_direction():
    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# CAMERA: 색상 감지
# 반환: (cx, cy, area) 또는 None
# =========================================
def detect_color(frame, color_name):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for (lo, hi) in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, lo, hi)

    # 모폴로지 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_COLOR_AREA:
        return None

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return (cx, cy, area)

# =========================================
# 라이다 기반 회피 조향값 계산
# (send 하지 않고 v, w 만 반환)
# =========================================
def lidar_avoid_cmd():
    front_min = get_front_min()

    if front_min < THRESH_10:
        direction = choose_avoid_direction()
        return MIN_SPEED, direction * MAX_W

    elif front_min < THRESH_20:
        direction = choose_avoid_direction()
        return 0.12, direction * 0.8

    elif front_min < THRESH_30:
        direction = choose_avoid_direction()
        return 0.15, direction * 0.7

    else:
        return MAX_SPEED, 0.0

# =========================================
# 라이다 1프레임 읽기
# 반환: s_flag (스캔 완료 여부)
# =========================================
def read_lidar_frame():
    raw = lidar_ser.read(5)
    if len(raw) != 5:
        return False

    s_flag = raw[0] & 0x01

    if (
        ((raw[0] & 0x02) >> 1) != (1 - s_flag)
        or (raw[1] & 0x01) != 1
        or (raw[0] >> 2) < 3
    ):
        return False

    angle    = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
    dist_cm  = (raw[3] | (raw[4] << 8)) / 40.0

    if 3 < dist_cm < 150:
        apply_ema(angle, dist_cm)

    if s_flag == 1:
        apply_median_filter()
        return True

    return False

# =========================================
# MAIN LOOP
# =========================================
print(f"START → {current_state()}")

# 현재 추적 중인 색상 이름
TARGET_COLOR = {
    "FIND_RED":    "RED",
    "PARK_RED":    "RED",
    "FIND_YELLOW": "YELLOW",
    "PARK_YELLOW": "YELLOW",
    "FIND_BLUE":   "BLUE",
    "PARK_BLUE":   "BLUE",
}

try:
    while True:

        state = current_state()

        # ──────────────────────────────
        # DONE
        # ──────────────────────────────
        if state == "DONE":
            stop_robot()
            print("ALL DONE. 모든 색상 주차 완료.")
            break

        # ──────────────────────────────
        # 라이다 읽기 (매 루프)
        # ──────────────────────────────
        scan_complete = read_lidar_frame()

        # ──────────────────────────────
        # 카메라 읽기 (매 루프)
        # ──────────────────────────────
        ret, frame = cap.read()
        color_name = TARGET_COLOR[state]
        detection  = detect_color(frame, color_name) if ret else None

        # ──────────────────────────────
        # FIND_* 상태
        # 라이다로 주행, 색상 발견 시 PARK_* 전환
        # ──────────────────────────────
        if state.startswith("FIND_"):

            if detection:
                cx, cy, area = detection
                print(
                    f"[{color_name}] 발견! cx={cx} cy={cy} "
                    f"area={area:.0f} → PARK 전환"
                )
                next_state()   # FIND_* → PARK_*
                continue

            # 색상 미발견 → 라이다 회피 주행
            if scan_complete:
                v, w = lidar_avoid_cmd()
                send_cmd(v, w)

        # ──────────────────────────────
        # PARK_* 상태
        # 색상 X축 중앙값으로 P 조향
        # 색상이 하단으로 사라지면 정지 → 다음 FIND_* 전환
        # ──────────────────────────────
        elif state.startswith("PARK_"):

            if detection is None:
                # 색상을 잃었으면 잠깐 전진하며 재탐색
                send_cmd(MIN_SPEED, 0.0)
                print(f"[{color_name}] 추적 중 소실, 재탐색...")
                continue

            cx, cy, area = detection

            # 도착 판정: 색상 중심이 화면 하단 아래로 내려옴
            if cy >= BOTTOM_THRESHOLD:
                stop_robot()
                print(
                    f"[{color_name}] 하단 도달 (cy={cy}) → 주차 완료"
                )
                time.sleep(0.5)
                next_state()   # PARK_* → FIND_* (또는 DONE)
                continue

            # P 조향: 화면 중앙과 색상 중심의 X 오차
            error = cx - (FRAME_W / 2)     # 양수 = 오른쪽
            w     = KP_STEER * error       # 오른쪽이면 오른쪽 조향
            v     = TRACK_SPEED

            # 라이다 장애물이 매우 가까우면 속도 감소
            front_min = get_front_min()
            if front_min < THRESH_10:
                v = MIN_SPEED

            send_cmd(v, w)
            print(
                f"[{color_name}] PARK cx={cx} cy={cy} "
                f"err={error:+.0f} w={w:+.3f}"
            )

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    lidar_ser.close()
    arduino_ser.close()
