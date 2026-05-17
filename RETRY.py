import serial
import math
import time
import numpy as np

# =========================================================
# SERIAL
# =========================================================
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

# =========================================================
# LIDAR START
# =========================================================
lidar_ser.write(bytes([0xA5, 0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)

print("LIDAR START")

# =========================================================
# PARAMETER
# =========================================================

SCAN_LIMIT = 150
BASE_SPEED = 0.22
MIN_SPEED  = 0.12
MAX_W      = 1.2
TURN_GAIN  = 0.022
WHEEL_BASE = 0.17

# =========================================================
# OBSTACLE PARAMETER
# =========================================================

SAFE_DIST  = 30
FRONT_DIST = 55

# ★ 추가: 긴급 정지 거리
EMERGENCY_DIST = 10   # cm — 이 거리 이하면 즉시 탈출 시퀀스 진입

# =========================================================
# FILTER
# =========================================================

EMA_ALPHA = 0.35
MEDIAN_K  = 2

# =========================================================
# STEERING SMOOTHING
# =========================================================

STEERING_ALPHA = 0.22
current_w      = 0.0

# =========================================================
# ESCAPE MEMORY
# =========================================================

escape_dir = 1

# =========================================================
# ★ STATE MACHINE
# =========================================================

STATE_NORMAL  = 0
STATE_REVERSE = 1
STATE_ROTATE  = 2

state             = STATE_NORMAL
maneuver_end_time = 0.0
rotate_dir        = 1

REVERSE_DURATION  = 0.25    # 후진 시간 (초)
ROTATE_DURATION   = 1.00    # 회전 시간 (초)
REVERSE_SPEED     = -0.10   # 후진 속도 (m/s)
ROTATE_W          = 0.9     # 탈출 회전 각속도 (rad/s)

# ★ 추가: 막힘 감지 타이머
stuck_since = None
STUCK_TIME  = 0.4   # 초 — 이 시간 이상 EMERGENCY_DIST 이하 유지 시 탈출

# =========================================================
# DATA
# =========================================================

scan_data = np.full(
    360,
    float(SCAN_LIMIT),
    dtype=np.float32
)

# =========================================================
# EMA FILTER
# =========================================================
def apply_ema(angle, dist):

    scan_data[angle] = (
        (1.0 - EMA_ALPHA)
        * scan_data[angle]
        + EMA_ALPHA * dist
    )

# =========================================================
# MEDIAN FILTER
# =========================================================
def apply_median_filter():

    filtered = np.empty(360, dtype=np.float32)

    for i in range(360):

        idx    = [(i + d) % 360 for d in range(-MEDIAN_K, MEDIAN_K + 1)]
        values = np.sort(scan_data[idx])
        filtered[i] = values[len(values) // 2]

    scan_data[:] = filtered

# =========================================================
# REGION DISTANCE
# =========================================================
def get_region_mean(start_deg, end_deg):

    if start_deg <= end_deg:
        idx = np.arange(start_deg, end_deg + 1)
    else:
        idx = np.concatenate((
            np.arange(start_deg, 360),
            np.arange(0, end_deg + 1)
        ))

    return float(np.mean(scan_data[idx]))


def get_region_min(start_deg, end_deg):

    if start_deg <= end_deg:
        idx = np.arange(start_deg, end_deg + 1)
    else:
        idx = np.concatenate((
            np.arange(start_deg, 360),
            np.arange(0, end_deg + 1)
        ))

    return float(np.min(scan_data[idx]))

# =========================================================
# ★ 추가: 탈출 방향 결정 (좌우 평균 거리 비교)
# =========================================================
def choose_avoid_direction():

    left_avg  = float(np.mean(scan_data[1:90]))
    right_avg = float(np.mean(scan_data[271:360]))

    return 1 if left_avg >= right_avg else -1

# =========================================================
# ★ 추가: 탈출 모터 명령 전송
# =========================================================
def send_escape_cmd(v, w):
    """
    차동구동 변환 없이 v, w 직접 전달
    v > 0: 전진, v < 0: 후진
    w > 0: 좌회전, w < 0: 우회전
    """
    left_wheel  = v - (WHEEL_BASE / 2.0) * w
    right_wheel = v + (WHEEL_BASE / 2.0) * w
    send_motor(left_wheel, right_wheel)

# =========================================================
# SENSOR ANALYSIS
# =========================================================
def analyze_obstacle():

    # 전방
    front_region = scan_data[np.arange(-15, 16) % 360]
    front_mean   = float(np.mean(front_region))
    front_min    = float(np.min(front_region))
    front        = front_mean * 0.7 + front_min * 0.3

    # ★ 수정: 측면 감지 범위 확장 (70° → 90°) — 옆면 벽 감지 개선
    left_mean  = get_region_mean(15, 90)
    left_min   = get_region_min(15, 90)
    left       = left_mean * 0.7 + left_min * 0.3

    right_mean = get_region_mean(270, 345)
    right_min  = get_region_min(270, 345)
    right      = right_mean * 0.7 + right_min * 0.3

    return front, left, right

# =========================================================
# CONTROL
# =========================================================
def compute_control():

    global current_w, escape_dir

    front, left, right = analyze_obstacle()

    # Force 계산
    left_force  = max(0.0, SAFE_DIST - left)
    right_force = max(0.0, SAFE_DIST - right)

    # 기본 steering
    target_w = (left_force - right_force) * TURN_GAIN

    # 정면 장애물 처리
    if front < FRONT_DIST:

        if left > right:
            escape_dir = 1
        else:
            escape_dir = -1

        boost    = (FRONT_DIST - front) / FRONT_DIST
        target_w *= (1.0 + boost * 4.0)
        target_w += (0.35 * escape_dir)

    # steering smoothing
    current_w = (
        current_w * (1.0 - STEERING_ALPHA)
        + target_w * STEERING_ALPHA
    )

    current_w = np.clip(current_w, -MAX_W, MAX_W)

    # 속도 계산
    front_scale = np.clip(front / 80.0, 0.45, 1.0)
    v           = BASE_SPEED * front_scale

    turn_scale  = max(0.70, 1.0 - abs(current_w) * 0.35)
    v          *= turn_scale
    v           = max(v, MIN_SPEED)

    # 차동구동 변환
    left_wheel  = v - (WHEEL_BASE / 2.0) * current_w
    right_wheel = v + (WHEEL_BASE / 2.0) * current_w

    return (
        left_wheel, right_wheel,
        v, current_w,
        front, left, right,
        left_force, right_force,
        escape_dir
    )

# =========================================================
# MOTOR SEND
# =========================================================
def send_motor(left_wheel, right_wheel):

    cmd = f"{left_wheel:.3f},{right_wheel:.3f}\n"
    arduino_ser.write(cmd.encode())


def stop_robot():
    send_motor(0.0, 0.0)

# =========================================================
# MAIN LOOP
# =========================================================
print("START NAVIGATION")

try:

    while True:

        raw = lidar_ser.read(5)

        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01

        if (raw[0] & 0x02) >> 1 != (1 - s_flag):
            continue

        if (raw[1] & 0x01) != 1:
            continue

        if (raw[0] >> 2) < 3:
            continue

        angle = int(
            ((raw[1] >> 1) | (raw[2] << 7)) / 64.0
        ) % 360

        dist = (raw[3] | (raw[4] << 8)) / 40.0

        if 3 < dist < SCAN_LIMIT:
            apply_ema(angle, dist)

        if s_flag != 1:
            continue

        apply_median_filter()

        now = time.time()

        # =====================================================
        # ★ STATE MACHINE
        # =====================================================

        # --- 후진 중 ---
        if state == STATE_REVERSE:

            if now < maneuver_end_time:
                send_escape_cmd(REVERSE_SPEED, 0.0)
            else:
                # 후진 완료 → 회전으로 전환
                state             = STATE_ROTATE
                maneuver_end_time = now + ROTATE_DURATION

            print(f"[REVERSE] remain:{maneuver_end_time - now:.2f}s")
            continue

        # --- 회전 중 ---
        if state == STATE_ROTATE:

            if now < maneuver_end_time:
                send_escape_cmd(0.0, ROTATE_W * rotate_dir)
            else:
                # 회전 완료 → 정상 주행 복귀
                rotate_dir  *= -1   # 다음 탈출 시 반대 방향
                state        = STATE_NORMAL
                current_w    = 0.0  # steering 리셋
                stuck_since  = None

                # 스캔 데이터 전방 초기화 (오래된 장애물 정보 제거)
                for a in range(-45, 46):
                    scan_data[a % 360] = float(SCAN_LIMIT)

            print(f"[ROTATE] remain:{maneuver_end_time - now:.2f}s | dir:{rotate_dir}")
            continue

        # --- 정상 주행 ---

        (
            left_wheel, right_wheel,
            v, w,
            front, left, right,
            left_force, right_force,
            escape_dir
        ) = compute_control()

        # =====================================================
        # ★ 막힘 감지 → 탈출 시퀀스 진입
        # =====================================================

        front_min = float(np.min(scan_data[np.arange(-10, 11) % 360]))

        if front_min < EMERGENCY_DIST:

            if stuck_since is None:
                stuck_since = now

            elif now - stuck_since > STUCK_TIME:
                # STUCK 판정 → 후진 시작
                rotate_dir        = choose_avoid_direction()
                state             = STATE_REVERSE
                maneuver_end_time = now + REVERSE_DURATION
                stuck_since       = None

                send_escape_cmd(REVERSE_SPEED, 0.0)

                print(f"[STUCK] → REVERSE | rotate_dir:{rotate_dir}")
                continue

        else:
            # 정상 거리 복귀 시 타이머 리셋
            stuck_since = None

        # =====================================================
        # 정상 모터 전송
        # =====================================================

        send_motor(left_wheel, right_wheel)

        print(
            f"v:{v:.2f} | "
            f"w:{w:.2f} | "
            f"F:{front:.1f} | "
            f"L:{left:.1f} | "
            f"R:{right:.1f} | "
            f"LF:{left_force:.1f} | "
            f"RF:{right_force:.1f} | "
            f"DIR:{escape_dir} | "
            f"stuck:{round(now - stuck_since, 2) if stuck_since else '-'}"
        )

except KeyboardInterrupt:

    print("STOP")

finally:

    stop_robot()
    lidar_ser.write(bytes([0xA5, 0x25]))
