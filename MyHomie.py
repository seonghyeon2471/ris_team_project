import serial
import math
import time
import numpy as np
import cv2
import threading

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

if not cap.isOpened():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        lidar_ser.close()
        arduino_ser.close()
        exit(1)

cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

time.sleep(1.0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"카메라 열림: {int(actual_w)}x{int(actual_h)}")

FRAME_W = int(actual_w)
FRAME_H = int(actual_h)

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
# LIDAR PARAMETERS
# =========================================
EMA_ALPHA         = 0.35
MEDIAN_K          = 2
FRONT_CHECK_RANGE = 45

THRESH_30 = 32.0
THRESH_20 = 22.0
THRESH_10 = 12.0

_scan_buf    = np.full(360, 150.0, dtype=np.float32)
_scan_shared = np.full(360, 150.0, dtype=np.float32)
scan_lock    = threading.Lock()

def _apply_ema(angle, dist_cm):
    if dist_cm <= 0:
        return
    _scan_buf[angle] = (
        (1.0 - EMA_ALPHA) * _scan_buf[angle]
        + EMA_ALPHA * dist_cm
    )

def _apply_median_filter():
    k        = MEDIAN_K
    filtered = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx         = [(i + d) % 360 for d in range(-k, k + 1)]
        filtered[i] = np.sort(_scan_buf[idx])[k]
    _scan_buf[:] = filtered

def _publish_scan():
    with scan_lock:
        _scan_shared[:] = _scan_buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5:
            continue
        s_flag = raw[0] & 0x01
        if (
            ((raw[0] & 0x02) >> 1) != (1 - s_flag)
            or (raw[1] & 0x01) != 1
            or (raw[0] >> 2) < 3
        ):
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150:
            _apply_ema(angle, dist_cm)
        if s_flag == 1:
            _apply_median_filter()
            _publish_scan()

lidar_thread = threading.Thread(target=lidar_loop, daemon=True)
lidar_thread.start()

def get_scan():
    with scan_lock:
        return _scan_shared.copy()

def get_front_min(scan):
    idx = np.arange(-FRONT_CHECK_RANGE, FRONT_CHECK_RANGE + 1) % 360
    return float(np.min(scan[idx]))

def choose_avoid_direction(scan):
    left_avg  = float(np.mean(scan[1:90]))
    right_avg = float(np.mean(scan[271:360]))
    return 1 if left_avg >= right_avg else -1

# =========================================
# DRIVE PARAMETERS
# =========================================
MAX_SPEED   = 0.24
MIN_SPEED   = 0.10
TRACK_SPEED = 0.22
MAX_W       = 0.8
KP_STEER    = 0.003

MIN_COLOR_AREA = 400
TARGET_AREA    = 6000

# ROI 도착 판정
ROI_Y              = 180
ROI_MIN_PIXEL      = 800
ROI_CONFIRM_FRAMES = 8

# 주차 시퀀스
CREEP_SEC       = 0.5    # ROI 확정 후 0.5초 직진
PARK_SEC        = 1.2    # 직진 후 1.2초 정지
INSIDE_STOP_SEC = 1.0

# 바운더리 탐색
BOUNDARY_PHASE1_SEC = 1.0
BOUNDARY_PHASE2_SEC = 1.8
BOUNDARY_PHASE3_SEC = 0.8
BOUNDARY_W          = 0.55
BOUNDARY_BACK_V     = -0.10

APPROACH_MAX_TIMEOUT = 4.0
APPROACH_DRIVE_SEC   = 1.5
NEAR_DIST_THRESHOLD  = 50.0

# =========================================
# HSV 색상 범위
# =========================================
COLOR_CFG = {
    "red": {
        "hsv1": ([0,   45,  50], [15,  255, 255]),
        "hsv2": ([160, 45,  50], [179, 255, 255]),
        "draw": (0, 0, 255),
    },
    "yellow": {
        "hsv1": ([15,  30,  60], [40,  255, 255]),
        "hsv2": ([10,  0,  190], [45,  45,  255]),
        "draw": (0, 200, 255),
    },
    "blue": {
        "hsv1": ([90,  45,  40], [140, 255, 255]),
        "hsv2": None,
        "draw": (255, 80, 0),
    },
}

MISSION = ["red", "yellow", "blue"]

# =========================================
# 상태 변수
# =========================================
mission_index        = 0
state                = "SEARCH"   # SEARCH / TRACK / APPROACH / CREEP / PARKING
last_seen_x          = FRAME_W // 2
last_dist_cm         = 150.0
park_start           = None
creep_start          = None       # 0.5초 직진 시작 시각
inside_stop_start    = None
approach_start_time  = None
search_start_time    = time.time()
roi_count            = 0

boundary_phase       = 0
boundary_phase_start = None
boundary_direction   = 1

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    v = float(np.clip(v, -0.3, 0.3))
    w = float(np.clip(w, -0.9, 0.9))
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# MASK
# =========================================
def make_mask(frame, hsv, color_name):
    cfg  = COLOR_CFG[color_name]
    lo1  = np.array(cfg["hsv1"][0])
    hi1  = np.array(cfg["hsv1"][1])
    mask = cv2.inRange(hsv, lo1, hi1)
    if cfg["hsv2"] is not None:
        lo2  = np.array(cfg["hsv2"][0])
        hi2  = np.array(cfg["hsv2"][1])
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo2, hi2))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def flush_camera_buffer(n=8):
    for _ in range(n):
        cap.grab()

# =========================================
# 바운더리 탐색
# =========================================
def start_boundary_search(lost_x):
    global boundary_phase, boundary_phase_start, boundary_direction, state
    boundary_direction   = 1 if lost_x > FRAME_W // 2 else -1
    boundary_phase       = 1
    boundary_phase_start = time.time()
    state                = "BOUNDARY"
    print(f"[BOUNDARY] 탐색 시작 | 방향={'우' if boundary_direction > 0 else '좌'}")

def run_boundary_search():
    global boundary_phase, boundary_phase_start, state, search_start_time
    elapsed = time.time() - boundary_phase_start

    if boundary_phase == 1:
        if elapsed < BOUNDARY_PHASE1_SEC:
            return 0.03, BOUNDARY_W * boundary_direction
        boundary_phase       = 2
        boundary_phase_start = time.time()
        print("[BOUNDARY] 2단계: 반대 방향")

    if boundary_phase == 2:
        elapsed = time.time() - boundary_phase_start
        if elapsed < BOUNDARY_PHASE2_SEC:
            return 0.03, BOUNDARY_W * -boundary_direction
        boundary_phase       = 3
        boundary_phase_start = time.time()
        print("[BOUNDARY] 3단계: 후진")

    if boundary_phase == 3:
        elapsed = time.time() - boundary_phase_start
        if elapsed < BOUNDARY_PHASE3_SEC:
            return BOUNDARY_BACK_V, 0.0
        boundary_phase    = 0
        state             = "SEARCH"
        search_start_time = time.time()
        print("[BOUNDARY] 실패 → SEARCH 복귀")
        return 0.0, 0.0

    return 0.0, 0.0

# =========================================
# 라이다 장애물 회피
# =========================================
def lidar_override(cam_v, cam_w, scan):
    front_min = get_front_min(scan)

    if front_min < THRESH_10:
        direction = choose_avoid_direction(scan)
        print(f"[LIDAR] 근접 {front_min:.1f}cm → 회피")
        return MIN_SPEED, direction * MAX_W

    elif front_min < THRESH_20:
        direction = choose_avoid_direction(scan)
        return 0.12, direction * 0.7

    elif front_min < THRESH_30:
        direction = choose_avoid_direction(scan)
        return min(cam_v, 0.15), direction * 0.5

    return cam_v, cam_w

# =========================================
# 주차 완료 후 다음 미션 시작
# =========================================
def start_next_mission():
    global mission_index, roi_count, inside_stop_start
    global boundary_phase, last_dist_cm, state, search_start_time, last_seen_x

    mission_index    += 1
    roi_count         = 0
    inside_stop_start = None
    boundary_phase    = 0
    last_dist_cm      = 150.0

    if mission_index < len(MISSION):
        flush_camera_buffer(n=15)
        state             = "SEARCH"        # BOUNDARY 아닌 SEARCH로 바로 복귀
        search_start_time = time.time()
        last_seen_x       = FRAME_W // 2   # 탐색 방향 초기화
        print(f"[미션] NEXT TARGET: {MISSION[mission_index]} → SEARCH 시작")
    else:
        state = "DONE"
        print("모든 미션 클리어")

# =========================================
# MAIN LOOP
# =========================================
print(f"START → {MISSION[mission_index]}")

try:
    while True:

        # ── 카메라: 버퍼 비우고 최신 프레임 ──────────────────────
        for _ in range(3):
            cap.grab()
        ret, frame = cap.retrieve()

        if not ret:
            print("[WARN] 카메라 프레임 읽기 실패")
            continue

        frame = cv2.flip(frame, 1)
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ── 라이다 스냅샷 ─────────────────────────────────────────
        scan = get_scan()

        # ══════════════════════════════════════════════════════════
        # DONE
        # ══════════════════════════════════════════════════════════
        if state == "DONE" or mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL DONE", (60, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        target = MISSION[mission_index]
        draw   = COLOR_CFG[target]["draw"]

        # ── 마스크 & ROI ──────────────────────────────────────────
        mask       = make_mask(frame, hsv, target)
        roi_mask   = mask[ROI_Y:, :]
        roi_pixels = cv2.countNonZero(roi_mask)
        roi_hit    = (roi_pixels >= ROI_MIN_PIXEL)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        cam_v, cam_w = 0.0, 0.0

        # ══════════════════════════════════════════════════════════
        # CREEP: ROI 확정 후 0.5초 직진
        # ══════════════════════════════════════════════════════════
        if state == "CREEP":
            creep_elapsed = time.time() - creep_start
            if creep_elapsed < CREEP_SEC:
                send_cmd(MIN_SPEED, 0.0)
                cv2.putText(frame, f"CREEP {CREEP_SEC - creep_elapsed:.1f}s",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("frame", frame)
                cv2.imshow("mask",  mask)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue
            else:
                # 0.5초 직진 완료 → PARKING
                stop_robot()
                state      = "PARKING"
                park_start = time.time()
                inside_stop_start = None
                print(f"[{target}] CREEP 완료 → PARKING 시작")
                continue

        # ══════════════════════════════════════════════════════════
        # PARKING: 1.2초 정지
        # ══════════════════════════════════════════════════════════
        if state == "PARKING":
            stop_robot()

            if roi_hit:
                if inside_stop_start is None:
                    inside_stop_start = time.time()
                    print(f"[{target}] 색지 내부 정지 시작")
                inside_elapsed = time.time() - inside_stop_start
            else:
                if inside_stop_start is not None:
                    print(f"[{target}] ROI 유실 → 타이머 리셋")
                inside_stop_start = None
                inside_elapsed    = 0.0

            park_elapsed = time.time() - park_start
            inside_ok    = (
                inside_stop_start is not None
                and (time.time() - inside_stop_start) >= INSIDE_STOP_SEC
            )

            if park_elapsed >= PARK_SEC and inside_ok:
                print(f"[{target}] 주차 완료")
                start_next_mission()

            roi_color = (0, 255, 0) if roi_hit else (100, 100, 100)
            cv2.line(frame, (0, ROI_Y), (FRAME_W, ROI_Y), roi_color, 1)
            cv2.putText(frame, f"PARKING {PARK_SEC - park_elapsed:.1f}s",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"TARGET: {target}",
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
            cv2.imshow("frame", frame)
            cv2.imshow("mask",  mask)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # ══════════════════════════════════════════════════════════
        # APPROACH 타임아웃 안전장치
        # ══════════════════════════════════════════════════════════
        if state == "APPROACH" and approach_start_time is not None:
            if time.time() - approach_start_time > APPROACH_MAX_TIMEOUT:
                print(f"[안전장치] APPROACH 타임아웃 → CREEP")
                state       = "CREEP"
                creep_start = time.time()
                roi_count   = 0
                continue

        # ══════════════════════════════════════════════════════════
        # ROI 도착 판정 (APPROACH 중)
        # ══════════════════════════════════════════════════════════
        if state == "APPROACH":
            if roi_hit:
                roi_count += 1
            else:
                roi_count = 0

            if roi_count >= ROI_CONFIRM_FRAMES:
                state       = "CREEP"        # PARKING 대신 CREEP 먼저
                creep_start = time.time()
                roi_count   = 0
                print(f"[{target}] ROI 확정 → CREEP 0.5초 직진 시작")
                continue

        # ══════════════════════════════════════════════════════════
        # 컨투어 처리
        # ══════════════════════════════════════════════════════════
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_COLOR_AREA:

                if state in ["BOUNDARY", "SEARCH"]:
                    state = "TRACK"
                    print(f"[{target}] 발견 → TRACK (area={int(area)})")

                rect               = cv2.minAreaRect(c)
                (cx, cy_obj), _, _ = rect
                cx, cy_obj         = int(cx), int(cy_obj)
                last_seen_x        = cx
                error_x            = cx - (FRAME_W // 2)

                cv2.drawContours(
                    frame, [np.int32(cv2.boxPoints(rect))], 0, draw, 2
                )
                cv2.circle(frame, (cx, cy_obj), 5, (255, 0, 0), -1)

                if state == "APPROACH" or area > TARGET_AREA:
                    if state != "APPROACH":
                        state               = "APPROACH"
                        approach_start_time = time.time()
                        roi_count           = 0
                        print(f"[{target}] APPROACH 진입 (area={int(area)})")

                    cam_w = -KP_STEER * error_x * 0.5
                    cam_v = TRACK_SPEED

                else:
                    cam_w    = -KP_STEER * error_x
                    rem_area = max(0.0, TARGET_AREA - area)
                    cam_v    = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (rem_area / TARGET_AREA)
                    cam_v    = float(np.clip(cam_v, MIN_SPEED, MAX_SPEED))

                cv2.putText(frame, f"Area:{int(area)}",
                            (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            else:
                if state not in ["BOUNDARY", "PARKING", "CREEP"]:
                    if last_dist_cm <= NEAR_DIST_THRESHOLD:
                        state             = "SEARCH"
                        search_start_time = time.time()
                        print(f"[근거리 유실] → SEARCH")
                    else:
                        start_boundary_search(last_seen_x)

        # ══════════════════════════════════════════════════════════
        # 객체 완전 유실
        # ══════════════════════════════════════════════════════════
        else:
            if state == "APPROACH":
                cam_v, cam_w = TRACK_SPEED, 0.0
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    state       = "CREEP"
                    creep_start = time.time()
                    roi_count   = 0
                    print(f"[{target}] 블라인드 완료 → CREEP")

            elif state == "BOUNDARY":
                cam_v, cam_w = run_boundary_search()

            elif state == "SEARCH":
                # 라이다 기반 전진 + 색상 방향 회전 병행
                cam_v = 0.10
                cam_w = -0.65 if last_seen_x > FRAME_W // 2 else 0.65

        # ══════════════════════════════════════════════════════════
        # 라이다 장애물 회피 합산
        # ══════════════════════════════════════════════════════════
        cam_v, cam_w = lidar_override(cam_v, cam_w, scan)
        send_cmd(cam_v, cam_w)

        # ── 디스플레이 ────────────────────────────────────────────
        roi_color = (0, 255, 0) if roi_hit else (100, 100, 100)
        cv2.line(frame, (0, ROI_Y), (FRAME_W, ROI_Y), roi_color, 1)
        cv2.putText(frame, f"STATE: {state}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"TARGET: {target}",
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw, 2)
        cv2.putText(frame, f"ROI:{roi_pixels}px",
                    (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi_color, 1)
        cv2.imshow("frame", frame)
        cv2.imshow("mask",  mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("STOP")

finally:
    stop_robot()
    cap.release()
    lidar_ser.write(bytes([0xA5, 0x25]))
    lidar_ser.close()
    arduino_ser.close()
    cv2.destroyAllWindows()
    print("SHUTDOWN")
